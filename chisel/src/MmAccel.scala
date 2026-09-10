//> using scala "2.13.14"
//> using dep "org.chipsalliance::chisel:6.6.0"
//> using plugin "org.chipsalliance:::chisel-plugin:6.6.0"
//> using options "-Ymacro-annotations"

// Parameterised systolic-array GEMM accelerator, AXI4-Lite slave.
// Chisel successor to src/mm_accel.v.
//
// WHY THE REGISTER MAP CHANGED FROM THE VERILOG
// The hand-written 2x2 gave each operand lane and each accumulator its own
// fixed word: pushes at 4..4+DIM-1 and 4+DIM..4+2*DIM-1, results at
// 8..8+DIM*DIM-1. That is 2*DIM + DIM*DIM words, which at DIM=16 is 288 words
// against a 64-word MMIO window - it does not fit, and the map is what blocks
// scaling, not the array.
//
// This map is CONSTANT SIZE in DIM (10 words) because the lane and the
// accumulator are addressed by index rather than by address:
//
//   word 0  CTRL       (W)  bit0=START (ignored while BUSY), bit1=SOFT_RST
//   word 1  STATUS     (R)  bit0=BUSY, bit1=DONE, bit2=DMA_BUSY, bit3=DMA_DONE
//   word 2  K_LEN      (RW) reduction depth for the next run, <= maxK
//   word 3  LOAD_K     (RW) k-GROUP index; each group is 4 packed INT8
//   word 4  LOAD_LANE  (RW) which row (A) or column (B) to push into.
//                           Writing it resets LOAD_K to 0.
//   word 5  A_PUSH     (W)  4 packed INT8 -> aRow[LOAD_LANE][4*LOAD_K ..+3],
//                           then LOAD_K auto-increments
//   word 6  B_PUSH     (W)  same for bCol[LOAD_LANE]
//   word 7  RESULT_IDX (RW) accumulator to read, = row*DIM + col
//   word 8  RESULT     (R)  accumulator[RESULT_IDX], then RESULT_IDX
//                           auto-increments - so a whole tile reads back with
//                           one transaction per element instead of two
//   word 9  INFO       (R)  {bPanels[23:16], maxK[15:8], dim[7:0]} - geometry
//                           discovery, so software need not hardcode any of it
//   word 10 DEST_ADDR  (RW) byte address the result DMA writes to. Must be
//                           16-byte aligned; the burst is dim*dim/4 lines of
//                           128 bits, written in row-major accumulator order.
//   word 15 B_PANEL_USE (RW) which B scratchpad panel the ARRAY reads
//   word 16 B_PANEL_LOAD(RW) which B panel a push or operand DMA WRITES.
//                           Split from B_PANEL_USE so the next panel can be
//                           filled while the current one feeds a run.
//   word 11 DEST_STRIDE(RW) byte distance between consecutive RESULT ROWS.
//                           0 = contiguous (stride = dim*4). Set this to the
//                           full matrix row pitch (N*4) to have a tile land
//                           directly in its place inside a larger C, with no
//                           software copy. Ignored at dim=2, where one
//                           128-bit line spans both rows.
//
// CTRL bit2 kicks the result DMA, which drains the accumulators over the
// mem_arbiter line port instead of through this 32-bit register window. That
// is the whole point of it: readback measured 70% of GEMM runtime at dim=16
// because each result cost ~5 cycles of AXI4-Lite protocol for 4 bytes.
//
// Packing 4 INT8 per 32-bit write plus auto-increment cuts the operand traffic
// for DIM=16,K=16 from 512 writes to ~160. It does NOT make the accelerator
// compute-bound - MMIO is still the limit, roughly 17x - and fixing that needs
// the accelerator to fetch its own operands (a master port). This step only
// removes the addressability wall so DIM can scale at all.
//
// Words 0-3 keep their meaning from the Verilog, so CTRL/STATUS/K_LEN polling
// code carries over unchanged; the operand and result access is what moved.

package mmaccel

import chisel3._
import chisel3.util._

/** Output-stationary systolic PE: acc += a*b each enabled cycle, or acc = a*b
  * on clearAcc (the first valid term of a run, so a stale accumulator from the
  * previous run is not added onto). a/b pass to neighbours with exactly one
  * cycle of registered delay - that delay is what makes the skewed edge feed
  * line up as values propagate through the array.
  */
class SystolicPE(width: Int = 8, accWidth: Int = 32) extends Module {
  val io = IO(new Bundle {
    val en       = Input(Bool())
    val clearAcc = Input(Bool())
    val aIn      = Input(SInt(width.W))
    val bIn      = Input(SInt(width.W))
    val aOut     = Output(SInt(width.W))
    val bOut     = Output(SInt(width.W))
    val acc      = Output(SInt(accWidth.W))
  })

  val aReg   = RegInit(0.S(width.W))
  val bReg   = RegInit(0.S(width.W))
  val accReg = RegInit(0.S(accWidth.W))

  when(io.en) {
    aReg := io.aIn
    bReg := io.bIn
    val prod = io.aIn * io.bIn
    accReg := Mux(io.clearAcc, prod, accReg + prod)
  }

  io.aOut := aReg
  io.bOut := bReg
  io.acc  := accReg
}

/** AXI4-Lite slave wrapping a DIM x DIM output-stationary systolic array.
  *
  * RawModule rather than Module so the emitted ports are clk/rst/s_axi_* and
  * the module is named mm_accel - matching the existing RTL and testbench
  * exactly. Chisel's defaults would give clock/reset/io_* and would not drop in.
  */
class MmAccel(val dim: Int = 2, val maxK: Int = 16, val bPanels: Int = 4,
              val fifoDepth: Int = 8,
              val enableReadBursts: Boolean = true,
              val enableWriteBursts: Boolean = true) extends RawModule {
  require(dim >= 1, "dim must be positive")
  require(maxK % 4 == 0, "maxK must be a multiple of 4: operands pack 4 INT8 per write")
  require(isPow2(maxK), "maxK is used as an index width; keep it a power of two")
  require(bPanels >= 1 && isPow2(bPanels), "bPanels must be a power of two")
  require(fifoDepth >= 2, "the result FIFO needs at least two entries to run ahead")

  override def desiredName = "mm_accel"

  val kBits    = log2Ceil(maxK)        // index of a single INT8 within a lane
  val kGrpBits = log2Ceil(maxK / 4)    // index of a 4-byte group
  val laneBits = log2Ceil(dim) max 1
  val accBits  = log2Ceil(dim * dim) max 1
  val panBits  = log2Ceil(bPanels) max 1

  val clk = IO(Input(Clock()))
  val rst = IO(Input(Bool()))

  val s_axi_awaddr  = IO(Input(UInt(32.W)))
  val s_axi_awvalid = IO(Input(Bool()))
  val s_axi_awready = IO(Output(Bool()))
  val s_axi_wdata   = IO(Input(UInt(32.W)))
  val s_axi_wstrb   = IO(Input(UInt(4.W)))
  val s_axi_wvalid  = IO(Input(Bool()))
  val s_axi_wready  = IO(Output(Bool()))
  val s_axi_bresp   = IO(Output(UInt(2.W)))
  val s_axi_bvalid  = IO(Output(Bool()))
  val s_axi_bready  = IO(Input(Bool()))
  val s_axi_araddr  = IO(Input(UInt(32.W)))
  val s_axi_arvalid = IO(Input(Bool()))
  val s_axi_arready = IO(Output(Bool()))
  val s_axi_rdata   = IO(Output(UInt(32.W)))
  val s_axi_rresp   = IO(Output(UInt(2.W)))
  val s_axi_rvalid  = IO(Output(Bool()))
  val s_axi_rready  = IO(Input(Bool()))

  // Result-DMA master port. Deliberately the mem_arbiter LINE protocol, not
  // AXI4 - the SoC already arbitrates two 128-bit cache ports into
  // axi_cache_adapter, so becoming a third requester reuses proven plumbing
  // and gets 16 bytes per transaction instead of AXI4-Lite's 4.
  val mem_req_valid = IO(Output(Bool()))
  val mem_req_write = IO(Output(Bool()))
  val mem_req_addr  = IO(Output(UInt(32.W)))
  val mem_wline     = IO(Output(UInt(128.W)))
  // Burst length in LINES for the current request. Driven to 1 initially so
  // this is a pure interface addition; the burst sequencing lands next.
  val mem_req_lines = IO(Output(UInt(8.W)))
  // Adapter asks for the next line of a write burst.
  val mem_wnext     = IO(Input(Bool()))
  val mem_rline     = IO(Input(UInt(128.W)))
  val mem_ready     = IO(Input(Bool()))

  withClockAndReset(clk, rst) {

    val busy      = RegInit(false.B)
    val done      = RegInit(false.B)
    val kLen      = RegInit(0.U(8.W))
    val loadK     = RegInit(0.U(kGrpBits.W))
    val loadLane  = RegInit(0.U(laneBits.W))
    val resultIdx = RegInit(0.U(accBits.W))
    val t         = RegInit(0.U(8.W))

    // Result-DMA state. Declared here with the rest of the register file so the
    // write-decode block below can reach it; the datapath and burst sequencer
    // live further down with the accumulator selection they depend on.
    // destAddr is what software programs; dmaAddr is the working cursor that
    // walks it, so a re-kick does not need the address rewritten.
    // Result-line geometry. Declared here rather than beside the DMA
    // sequencer because the accumulator mux is indexed by fillGrp and appears
    // earlier in the module than the sequencer does.
    val lineWords = 4
    val nGroups   = (dim * dim) / lineWords
    val grpBits   = log2Ceil(nGroups) max 1

    // FIFO fill pointer and remaining-group count.
    val fillGrp  = RegInit(0.U(grpBits.W))
    val fillLeft = RegInit(0.U(9.W))

    val destAddr = RegInit(0.U(32.W))
    val dmaAddr  = RegInit(0.U(32.W))
    val dmaBusy  = RegInit(false.B)
    val dmaDone  = RegInit(false.B)

    // Byte distance between consecutive RESULT ROWS in memory - OpenGeMM's
    // "programmable strided memory access", and the thing that decides whether
    // this DMA is useful at all.
    //
    // A tile computes C[ti*dim+i][tj*dim+j] of a bigger M x N matrix. Each tile
    // row is contiguous (dim words), but the next row starts N*4 bytes later.
    // Writing dim*dim words contiguously would land the tile in a scratch
    // buffer that software then has to copy into place - which is exactly the
    // CPU-mediated movement the DMA exists to eliminate.
    //
    // 0 means "rows are contiguous", i.e. stride = dim*4, which is the correct
    // behaviour when the destination really is a dim x dim buffer.
    val destStride = RegInit(0.U(32.W))
    val dmaRowBase = RegInit(0.U(32.W))

    // Operand-load (input prefetch) state. Mirrors the result DMA on the same
    // line port, but READING. This is the other half of OpenGeMM's input
    // pre-fetching with output buffering, and the half the measurements said
    // matters most: operand load was 81% of tile time on hardware, moving bytes
    // at 0.31 B/cycle through the register window against 1.90 B/cycle here.
    //
    // A panel is row-major - lane i is row i, at aSrcAddr + i*srcStride.
    // B panel is COLUMN-major: lane j holds column j of B, so B must be stored
    // transposed. That is not a shortcut. bColBuf has always been fed columns,
    // and a row-major B would make each lane a strided gather of kLen separate
    // bytes rather than one contiguous line; transposed-B is the standard GEMM
    // layout for exactly this reason.
    val aSrcAddr  = RegInit(0.U(32.W))
    val bSrcAddr  = RegInit(0.U(32.W))
    val srcStride = RegInit(0.U(32.W))
    val ldBusy    = RegInit(false.B)
    val ldDone    = RegInit(false.B)

    // B-ONLY load. This is what makes double buffering possible.
    //
    // A full load rewrites aRowBuf as well, and the array READS aRowBuf while
    // it computes - so prefetching during a run would be writing a buffer that
    // is in use. In the tiled schedule A does not change within a row-strip
    // anyway (only B advances per tile), so the useful prefetch is B alone:
    // fill panel N+1 while the array computes from panel N.
    //
    // Skipping A also halves the prefetch traffic, since A would otherwise be
    // re-fetched identically for every tile in the strip.
    val ldBOnly   = RegInit(false.B)

    // Operand buffers: registers rather than Mem, indexed [lane][k].
    //
    // Mem would be the obvious choice, but the operand DMA fills these a whole
    // 128-bit LINE at a time - 16 bytes, which at maxK=16 is one entire lane.
    // Writing a byte-addressed Mem would take 16 cycles per lane against a
    // ~9-cycle line fetch, making the buffer write the bottleneck instead of
    // the memory. As registers a line lands in one cycle.
    //
    // Cost is dim*maxK bytes per buffer (128 B per buffer at dim=8, maxK=16),
    // small next to the array itself, and Mem at this size lowers to flops or
    // distributed RAM anyway.
    val aRowBuf = Reg(Vec(dim, Vec(maxK, SInt(8.W))))

    // B SCRATCHPAD: bPanels column-panels held locally instead of one.
    //
    // This is the reuse that OpenGeMM gets from tight memory coupling, in the
    // smallest form that pays. For output tile (ti,tj) the array needs A row-
    // panel ti and B column-panel tj. Software already hoists the A load out of
    // the inner loop - the buffers persist - but B was reloaded for EVERY tile,
    // which is why operand load stayed 81% of tile time even after packing and
    // auto-increment.
    //
    // With several panels resident the inner loop reloads nothing: fill panels
    // once, then walk tiles selecting a panel per run. Arithmetic intensity for
    // a bPanels-wide strip rises from dim/2 to roughly dim*bPanels/(1+bPanels)
    // MACs per byte, which is what moves the design toward compute-bound.
    //
    // Cost is bPanels*dim*maxK bytes (512 B at dim=8, maxK=16, bPanels=4) -
    // cheap next to 64 PEs, and the FPGA build has 88% of its flops free.
    val bColBuf = Reg(Vec(bPanels, Vec(dim, Vec(maxK, SInt(8.W)))))

    // Which panel the ARRAY reads during a run, and which the DMA/push WRITES.
    // Separate registers on purpose: that split is what allows the next panel
    // to be filled while the current one feeds a run - double buffering, once
    // the fill no longer has to be serialised behind compute.
    val bPanelUse  = RegInit(0.U(panBits.W))
    val bPanelLoad = RegInit(0.U(panBits.W))

    // ---- AXI write channel ----
    // The bridge master always presents AWVALID and WVALID together, so both
    // are accepted in one cycle. The ready signals are REGISTERED, so they
    // assert the cycle after doWrite - while the register file is written on
    // doWrite itself. Copied from the Verilog deliberately; the C driver and
    // the bridge both depend on this timing.
    val wIdle :: wResp :: Nil = Enum(2)
    val wState    = RegInit(wIdle)
    val doWrite   = s_axi_awvalid && s_axi_wvalid && (wState === wIdle)
    val waddrWord = s_axi_awaddr(7, 2)

    val awreadyReg = RegInit(false.B)
    val wreadyReg  = RegInit(false.B)
    val bvalidReg  = RegInit(false.B)

    switch(wState) {
      is(wIdle) {
        when(doWrite) {
          awreadyReg := true.B; wreadyReg := true.B; bvalidReg := true.B
          wState := wResp
        }.otherwise {
          awreadyReg := false.B; wreadyReg := false.B
        }
      }
      is(wResp) {
        awreadyReg := false.B; wreadyReg := false.B
        when(s_axi_bready) { bvalidReg := false.B; wState := wIdle }
      }
    }

    s_axi_awready := awreadyReg
    s_axi_wready  := wreadyReg
    s_axi_bvalid  := bvalidReg
    s_axi_bresp   := 0.U

    // ---- AXI read channel ----
    val rIdle :: rResp :: Nil = Enum(2)
    val rState     = RegInit(rIdle)
    val raddrWord  = RegInit(0.U(6.W))
    val arreadyReg = RegInit(false.B)
    val rvalidReg  = RegInit(false.B)

    switch(rState) {
      is(rIdle) {
        when(s_axi_arvalid) {
          arreadyReg := true.B
          raddrWord  := s_axi_araddr(7, 2)
          rvalidReg  := true.B
          rState     := rResp
        }.otherwise {
          arreadyReg := false.B
        }
      }
      is(rResp) {
        arreadyReg := false.B
        when(s_axi_rready) {
          rvalidReg := false.B
          rState    := rIdle
          // Auto-increment RESULT_IDX on a RESULT read, mirroring what LOAD_K
          // already does for operand pushes.
          //
          // This is the single biggest throughput fix in the register map, and
          // it came out of measurement rather than inspection. Benchmarking a
          // real tiled GEMM (Testbenches/tb_mm_accel_bench.v) put readback at
          // 54-60% of total runtime, ahead of operand load at 33-40%, because
          // every result cost TWO AXI transactions - set RESULT_IDX, then read
          // RESULT - while a packed operand push moves 4 bytes in ONE. Per byte
          // moved, readback was 8x less efficient than load.
          //
          // With this, sweeping the accumulators costs one transaction each.
          // accBits is exactly log2(dim*dim) for every power-of-two dim, so the
          // counter wraps cleanly at the end of the array with no compare.
          //
          // Precedence: this sits BEFORE the register-write block, so an
          // explicit write to RESULT_IDX still overrides the auto-increment.
          when(raddrWord === 8.U) { resultIdx := resultIdx + 1.U }
        }
      }
    }

    s_axi_arready := arreadyReg
    s_axi_rvalid  := rvalidReg
    s_axi_rresp   := 0.U

    // ---- register writes (ignored while BUSY so an in-flight run is safe) ----
    val startPulse   = doWrite && (waddrWord === 0.U) && s_axi_wdata(0) && !busy
    val softRstPulse = doWrite && (waddrWord === 0.U) && s_axi_wdata(1)
    // CTRL bit2 kicks the result DMA. Gated on !dmaBusy so a second write
    // during a burst cannot restart it mid-flight and corrupt the cursor.
    // Both engines share one line port, so each start must exclude the other.
    // dmaStart originally checked only !dmaBusy, which let a result DMA be
    // kicked DURING an operand load: dmaDrive would then raise mem_req_write
    // and swing mem_req_addr mid-read, corrupting the in-flight fetch.
    val dmaStart     = doWrite && (waddrWord === 0.U) && s_axi_wdata(2) && !dmaBusy && !ldBusy
    // CTRL bit3 kicks the operand load. Gated on BOTH engines being idle: they
    // share one line port and one mem_ready, so overlapping them would corrupt
    // whichever request happened to be in flight.
    val ldStart      = doWrite && (waddrWord === 0.U) && s_axi_wdata(3) && !ldBusy && !dmaBusy
    // CTRL bit4: load ONLY the B panel, leaving aRowBuf untouched so this may
    // run concurrently with compute. Safe only when B_PANEL_LOAD differs from
    // B_PANEL_USE - software's responsibility, and what double buffering means.
    val ldStartB     = doWrite && (waddrWord === 0.U) && s_axi_wdata(4) && !ldBusy && !dmaBusy

    val pushA = doWrite && !busy && (waddrWord === 5.U)
    val pushB = doWrite && !busy && (waddrWord === 6.U)

    when(doWrite && !busy) {
      when(waddrWord === 2.U)  { kLen := s_axi_wdata(7, 0) }
      when(waddrWord === 10.U) { destAddr   := s_axi_wdata } // DEST_ADDR
      when(waddrWord === 11.U) { destStride := s_axi_wdata } // DEST_STRIDE
      when(waddrWord === 12.U) { aSrcAddr   := s_axi_wdata } // A_SRC_ADDR
      when(waddrWord === 13.U) { bSrcAddr   := s_axi_wdata } // B_SRC_ADDR
      when(waddrWord === 14.U) { srcStride  := s_axi_wdata } // SRC_STRIDE
      when(waddrWord === 15.U) { bPanelUse  := s_axi_wdata(panBits - 1, 0) }
      when(waddrWord === 16.U) { bPanelLoad := s_axi_wdata(panBits - 1, 0) }
      when(waddrWord === 3.U) { loadK := s_axi_wdata(kGrpBits - 1, 0) }
      when(waddrWord === 4.U) {
        loadLane := s_axi_wdata(laneBits - 1, 0)
        loadK    := 0.U          // a new lane always starts at k=0
      }
      when(waddrWord === 7.U) { resultIdx := s_axi_wdata(accBits - 1, 0) }

      // Operand pushes: 4 packed INT8 per write, then auto-increment the group
      // index so a lane is filled by a straight run of stores with no index
      // rewrite between them.
      for (i <- 0 until dim) {
        when(loadLane === i.U) {
          for (b <- 0 until 4) {
            val addr = Cat(loadK, b.U(2.W))
            val byte = s_axi_wdata(8 * b + 7, 8 * b).asSInt
            when(pushA) { aRowBuf(i)(addr) := byte }
            when(pushB) { bColBuf(bPanelLoad)(i)(addr) := byte }
          }
        }
      }
      // When a lane's k-groups are exhausted, advance to the next lane and wrap
      // k back to 0. Same motivation as the RESULT_IDX auto-increment: at
      // kLen=8 a lane cost one LOAD_LANE write plus only two pushes, so a THIRD
      // of operand traffic was index bookkeeping rather than data. A whole
      // matrix now loads as one LOAD_LANE write followed by dim*(kLen/4)
      // back-to-back pushes.
      //
      // laneBits is exactly log2(dim) for power-of-two dim, so the lane counter
      // wraps at the end of the array on its own. Software may still write
      // LOAD_LANE explicitly to seek to a lane: that is waddrWord 4 while a
      // push is 5 or 6, so the two are mutually exclusive and never race.
      when(pushA || pushB) {
        val lastGrp = (kLen >> 2) - 1.U
        when(loadK === lastGrp) {
          loadK    := 0.U
          loadLane := loadLane + 1.U
        }.otherwise {
          loadK := loadK + 1.U
        }
      }
    }

    // ---- run sequencer ----
    // Total length K_LEN + 2*(dim-1): a value fed to row i needs j more hops to
    // reach PE(i,j), so PE(i,j)'s last term (k = K_LEN-1) lands at t = k+i+j.
    val lastT = kLen + (2 * (dim - 1)).U - 1.U
    when(softRstPulse) {
      busy := false.B; done := false.B; t := 0.U
    }.elsewhen(startPulse) {
      busy := true.B; done := false.B; t := 0.U
    }.elsewhen(busy) {
      when(t === lastT) { busy := false.B; done := true.B }
      t := t + 1.U
    }

    // ---- skewed edge feed: row i's k-th value enters at t = k+i ----
    val aEdge = Wire(Vec(dim, SInt(8.W)))
    val bEdge = Wire(Vec(dim, SInt(8.W)))
    for (i <- 0 until dim) {
      val kIdx   = t.zext - i.S                    // goes negative early in a run
      val kValid = (kIdx >= 0.S) && (kIdx < kLen.zext)
      val kSel   = kIdx.asUInt(kBits - 1, 0)
      aEdge(i) := Mux(kValid, aRowBuf(i)(kSel), 0.S)
      // Panel select as an explicit mux over STATIC panel indices.
      //
      // The natural form, bColBuf(bPanelUse)(i)(kSel), is a dynamic index into
      // a Vec of Vec of Vec, which firtool lowers to a packed-array expression
      // that our disallowPackedArrays option rejects. It happened to lower
      // cleanly at bPanels 1, 4 and 8 and failed only at 2, so the whole class
      // of bug was invisible until bPanels was actually swept.
      bEdge(i) := Mux(kValid,
                      VecInit((0 until bPanels).map(p => bColBuf(p)(i)(kSel)))(bPanelUse),
                      0.S)
    }

    // ---- PE array ----
    val pes = Seq.tabulate(dim, dim)((_, _) => Module(new SystolicPE(8, 32)))
    for (i <- 0 until dim; j <- 0 until dim) {
      val pe = pes(i)(j)
      pe.io.en       := busy
      pe.io.clearAcc := t === (i + j).U
      pe.io.aIn      := (if (j == 0) aEdge(i) else pes(i)(j - 1).io.aOut)
      pe.io.bIn      := (if (i == 0) bEdge(j) else pes(i - 1)(j).io.bOut)
    }

    // ---- result DMA: drain the accumulators to memory as 128-bit lines ----
    //
    // Why this exists. Benchmarking a real tiled GEMM put RESULT readback at
    // 70% of runtime at dim=16: every accumulator left through a 32-bit
    // AXI4-Lite register window costing ~5 cycles of protocol per 4 bytes, so
    // 256 results cost ~1800 cycles against 46 cycles of actual compute. No
    // register-map trick fixes that - the window itself has to get wider.
    //
    // The SoC already has a wide path: mem_arbiter carries 128-bit LINES to
    // axi_cache_adapter for the two caches. Rather than build a private AXI4
    // master, this port speaks that same line protocol and becomes a third
    // requester. 16 bytes per transaction instead of 4, over proven plumbing.
    //
    // Draining 4 accumulators per cycle keeps the line format natural and
    // matches the arbiter's width exactly. The selection mirrors the two-stage
    // structure that fixed routing at dim=16 - group muxes local to a row, a
    // register, then a mux across rows - because a flat dim*dim:1 mux over
    // 32-bit accumulators is what made global routing fail in the first place.

    val dmaGrp   = RegInit(0.U(grpBits.W))

    // Group g holds flattened accumulators 4g .. 4g+3. For dim >= 4 an aligned
    // group of four never straddles a row, so the first stage stays row-local.
    // dim = 2 is the degenerate case: the whole array is a single line, so
    // there is no selection to make and no mux to build.
    val lineData = Wire(UInt(128.W))
    if (nGroups == 1) {
      lineData := Cat(pes.flatten.map(_.io.acc.asUInt).reverse)
    } else {
      val perRow    = nGroups / dim                    // groups per row, >= 1
      val rowLines  = VecInit(pes.map { row =>
        val groups = VecInit((0 until perRow).map { g =>
          Cat((0 until lineWords).map(w => row(g * lineWords + w).io.acc.asUInt).reverse)
        })
        if (perRow == 1) groups(0) else groups(fillGrp(log2Ceil(perRow) - 1, 0))
      })
      val rowLineReg = RegNext(rowLines)
      // Indexed by the FILL pointer: the FIFO decouples production from the
      // AXI send pointer, so the mux follows fillGrp rather than dmaGrp.
      // Delayed by exactly the cycle rowLineReg adds, so the row index and the
      // group inside that row come from the SAME fillGrp value.
      val fillGrpD   = RegNext(fillGrp, 0.U(grpBits.W))
      val rowSelDma  = if (perRow == 1) fillGrpD else fillGrpD >> log2Ceil(perRow)
      lineData := rowLineReg(rowSelDma)
    }

    // ---- operand load: fetch A and B panels as 128-bit lines ----
    //
    // A lane is kLen bytes. At maxK=16 that is exactly one line, so linesPerLane
    // is 1 and a whole lane lands in a single cycle - which is why the operand
    // buffers became registers. The general form is kept so a larger maxK only
    // changes a constant.
    val linesPerLane = if (maxK > 16) maxK / 16 else 1
    val lplBits      = log2Ceil(linesPerLane) max 1
    val ldTotal      = 2 * dim * linesPerLane        // both panels
    val ldIdxBits    = log2Ceil(ldTotal) max 1

    val ldIdx  = RegInit(0.U(ldIdxBits.W))
    val ldAddr = RegInit(0.U(32.W))
    val ldReq  = RegInit(false.B)

    // Which panel/lane/chunk this index refers to. Second half of the sequence
    // is the B panel.
    val ldInB   = if (dim * linesPerLane == 0) false.B else ldIdx >= (dim * linesPerLane).U
    val ldLocal = Mux(ldInB, ldIdx - (dim * linesPerLane).U, ldIdx)
    val ldLane  = if (linesPerLane == 1) ldLocal else (ldLocal >> lplBits)
    val ldChunk = if (linesPerLane == 1) 0.U else ldLocal(lplBits - 1, 0)

    // Effective per-lane stride: 0 means panels are packed, one lane after
    // another with no gap, i.e. stride = kLen bytes.
    val effSrcStride = Mux(srcStride === 0.U, kLen, srcStride)

    // ---- burst fast path ----
    //
    // Hardware measured 29.87 cycles to move ONE 128-bit line: almost entirely
    // AXI round-trip latency, paid per transaction. Issuing one request for a
    // whole panel amortises that latency across every line in it.
    //
    // An AXI INCR burst is CONTIGUOUS, so this only applies when a panel's
    // lanes sit back-to-back - stride exactly one lane. A panel sliced out of a
    // wider matrix has gaps between lanes and cannot be one transaction, so it
    // falls back to per-line requests and gains nothing. That makes packed
    // operand staging a real software contract for the fast path, not a
    // preference.
    val linesPerPanel = dim * linesPerLane
    // Bursting is off by default: it regressed measured hardware throughput
    // (29.87 -> 61.18 cycles per line) while never actually engaging.
    // Operand fetch bursts. Safe direction: the adapter receives here, so it
    // can never stall waiting for the requester to present data.
    val burstOK       = if (enableReadBursts) effSrcStride === (16 * linesPerLane).U
                        else false.B
    val panelBase     = Mux(ldInB, bSrcAddr, aSrcAddr)
    val ldBase       = Mux(ldInB, bSrcAddr, aSrcAddr)
    val ldNextAddr   = ldBase + (ldLane * effSrcStride) + (ldChunk << 4.U)

    when(ldStart || ldStartB) {
      ldBusy  := true.B
      ldDone  := false.B
      ldBOnly := ldStartB
      // A B-only load starts partway through the sequence, at the first B line.
      ldIdx   := Mux(ldStartB, (dim * linesPerLane).U, 0.U)
      ldAddr  := Mux(ldStartB, bSrcAddr, aSrcAddr)
      ldReq   := true.B
    }.elsewhen(ldBusy) {
      when(mem_ready) {
        // Capture the returned line into its lane. 16 bytes at a time; the
        // buffers are registers precisely so this costs one cycle.
        for (i <- 0 until dim) {
          when(ldLane === i.U) {
            for (b <- 0 until 16) {
              val kOff = if (linesPerLane == 1) b.U(kBits.W)
                         else (ldChunk << 4.U).asUInt + b.U
              when(kOff < maxK.U) {
                val byte = mem_rline(8 * b + 7, 8 * b).asSInt
                when(!ldInB) { aRowBuf(i)(kOff) := byte }
                  .otherwise { bColBuf(bPanelLoad)(i)(kOff) := byte }
              }
            }
          }
        }
        when(ldIdx === (ldTotal - 1).U) {
          ldBusy := false.B
          ldDone := true.B
          ldReq  := false.B
        }.otherwise {
          ldIdx := ldIdx + 1.U
          // In burst mode the arbiter has already latched a request covering
          // the whole panel, so valid must DROP after the first line or the
          // arbiter would start a second transaction when this one retires.
          // It is re-asserted only at the A->B panel boundary, where a new
          // burst genuinely begins. Per-line mode re-requests every line.
          when(burstOK) {
            ldReq := (ldIdx + 1.U) === linesPerPanel.U
          }.otherwise {
            ldReq := true.B
          }
        }
      }
    }

    // Address for the NEXT index, computed combinationally from ldIdx so the
    // request presented each cycle already matches the current index. Unlike
    // the result DMA there is no registered mux in this path, so no settle
    // cycle is needed.
    ldAddr := ldNextAddr

    // One outstanding line request at a time, exactly like each cache port on
    // mem_arbiter - so this needs no reordering and no tags.
    //
    // dmaSettle exists because lineData is REGISTERED (rowLineReg). For dim >= 8
    // there is more than one group per row, so the group mux is driven by dmaGrp
    // and its output is therefore one cycle behind a change of dmaGrp. Asserting
    // the request in that cycle publishes the PREVIOUS group's data at the new
    // address.
    //
    // This was a real bug and it only appeared against ZERO-LATENCY memory: with
    // any wait states the register had already settled before mem_ready came
    // back, so dim=8 and dim=16 passed at lat>=3 and corrupted line 1 onward at
    // lat=0. Testing the DMA only against slow memory would have shipped it.
    //
    // One settle cycle per line costs nothing whenever memory has any latency at
    // all, which on a port shared with two caches is the normal case.
    // Output FIFO. The mux fills it at one line every other cycle; the burst
    // drains it at AXI rate. Depth only has to cover the difference, so a
    // handful of entries is enough - this is a rate matcher, not a store.
    val lineFifo = Module(new Queue(UInt(128.W), fifoDepth))
    lineFifo.io.enq.valid := false.B
    lineFifo.io.enq.bits  := lineData
    lineFifo.io.deq.ready := false.B

    // Fill pipeline: fillGrp selects, rowLineReg lands one cycle later, so a
    // valid bit has to be delayed by exactly the same cycle.
    // count < depth-1, not enq.ready: a line launched this cycle lands NEXT
    // cycle, so the slot it needs has to be reserved now. Gating on enq.ready
    // alone dropped that in-flight line whenever the queue filled in between,
    // silently skipping a result group.
    val fillFire  = dmaBusy && (fillLeft =/= 0.U) &&
                    (lineFifo.io.count < (fifoDepth - 1).U)
    val fillValid = RegNext(fillFire, false.B)

    when(fillFire) {
      fillGrp  := fillGrp + 1.U
      fillLeft := fillLeft - 1.U
    }
    lineFifo.io.enq.valid := fillValid

    val memReqValid = RegInit(false.B)
    val dmaSettle   = RegInit(false.B)
    // Groups per result row, and whether this group ends one. Both are
    // compile-time constants, so the row test is a bit-compare, not a divide.
    //
    // dim = 2 is excluded: the whole 2x2 tile is a single 128-bit line spanning
    // BOTH rows, so there is no row boundary to stride at. Stride is ignored
    // there and the tile is written contiguously - correct for a 2x2 scratch
    // destination, which is the only sensible target at that size anyway.
    val perRowGrp = if (nGroups >= dim) nGroups / dim else 0
    val strideOK  = perRowGrp >= 1
    val atRowEnd: Bool =
      if (!strideOK || perRowGrp == 1) true.B
      else dmaGrp(log2Ceil(perRowGrp) - 1, 0) === (perRowGrp - 1).U

    // stride 0 = contiguous rows, i.e. exactly one tile row of dim words
    val effStride = Mux(destStride === 0.U, (dim * 4).U, destStride)

    // A burst is a CONTIGUOUS INCR transaction, so how many lines it can cover
    // depends on the destination layout:
    //   contiguous C  -> the whole tile in one burst
    //   strided C     -> one burst per result ROW, then jump by the stride
    // Either way it is far fewer round trips than one transaction per line,
    // which is what the 29.87 cycles/line measured on hardware was paying.
    val destContig  = destStride === 0.U || destStride === (dim * 4).U
    val rowLinesU   = if (strideOK) perRowGrp.U else nGroups.U
    // Result writeback bursts stay OFF: the adapter must be fed a line per
    // BEATS_PER_LINE beats, and the registered accumulator mux cannot sustain
    // that without a deeper prefetch than the current FIFO provides.
    val wBurstLines = if (enableWriteBursts) Mux(destContig, nGroups.U, rowLinesU)
                      else 1.U

    val dmaSent = RegInit(0.U(9.W))     // lines accepted by memory so far

    when(dmaStart) {
      dmaBusy    := true.B
      dmaDone    := false.B
      dmaAddr    := destAddr
      dmaRowBase := destAddr
      fillGrp    := 0.U
      fillLeft   := nGroups.U
      dmaSent    := 0.U
    }.elsewhen(dmaBusy) {
      // mem_ready marks the END of a whole burst, not of a line.
      when(mem_ready) {
        val nextSent = dmaSent + wBurstLines
        when(nextSent >= nGroups.U) {
          dmaBusy := false.B
          dmaDone := true.B
        }.otherwise {
          dmaSent := nextSent
          // Address walk. With bursts off this advances one line at a time,
          // jumping by the stride at each result-row boundary - the behaviour
          // the 5.68x hardware measurement was taken with.
          val atRowEndSent: Bool =
            if (!strideOK || perRowGrp == 1) true.B
            else nextSent(log2Ceil(perRowGrp) - 1, 0) === 0.U
          when(destContig) {
            dmaAddr := dmaAddr + (wBurstLines << 4.U)
          }.elsewhen(atRowEndSent) {
            dmaAddr    := dmaRowBase + effStride
            dmaRowBase := dmaRowBase + effStride
          }.otherwise {
            dmaAddr := dmaAddr + (lineWords * 4).U
          }
        }
      }
    }

    // The FIFO decouples the registered accumulator mux from AXI: it fills at
    // one line per cycle and drains at one line per BEATS_PER_LINE beats, so
    // once primed it cannot underrun mid-burst.
    // Drain the FIFO whenever the DMA is idle. A run that ends with lines still
    // buffered would otherwise hand them to the NEXT run, which is exactly how
    // the strided pass failed straight after the contiguous one: correct data,
    // one run stale.
    // Pop on mem_wnext (mid-burst) AND on mem_ready (end of burst): the final
    // line of a burst is consumed by memory but never gets a wnext, so without
    // this it stayed at the head and the NEXT burst began one line behind -
    // which is exactly how the strided pass wrote row 0 correctly and then
    // shifted every row after it by one line.
    // Also drained while idle so a run never inherits the previous run's tail.
    lineFifo.io.deq.ready := mem_wnext || (dmaBusy && mem_ready) || !dmaBusy

    // Both engines share one line port. Mutual exclusion is enforced at the
    // START gates above - each requires the other engine idle - so at most one
    // of dmaDrive / ldBusy can be true here.
    // Hold the request until the burst retires; the arbiter latches once and
    // stays in SVC_A, and dmaBusy drops on completion so no second transaction
    // is started. Gated on the FIFO having a line so the burst never begins
    // ahead of its data.
    val dmaDrive = dmaBusy && lineFifo.io.deq.valid
    mem_req_valid := dmaDrive || (ldBusy && ldReq)
    mem_req_write := dmaDrive                          // loads are reads
    // A burst addresses the PANEL BASE and covers linesPerPanel lines; the
    // per-line path addresses each line individually with a length of 1.
    mem_req_addr  := Mux(dmaDrive, dmaAddr,
                         Mux(burstOK, panelBase, ldNextAddr))
    mem_wline     := lineFifo.io.deq.bits
    mem_req_lines := Mux(dmaDrive, wBurstLines,
                         Mux(burstOK, linesPerPanel.U, 1.U))


    // ---- read mux ----
    // Two-stage, and it has to be. The obvious version,
    //     val results = VecInit(pes.flatten.map(_.io.acc.asUInt))
    //     rdata := results(resultIdx)
    // is a flat DIM*DIM:1 mux over 32-bit accumulators. At DIM=16 that pulls
    // 256*32 = 8192 wires from PEs spread across the whole die into a single
    // point, and global routing fails: GRT-0119, every congested net an
    // accReg, hotspots smeared across the die rather than clustered because
    // the WIRES are what span it.
    //
    // Do not try to fix that by lowering FP_CORE_UTIL. It was tried at 30%
    // (8.24 mm^2) and 22% (11.22 mm^2) and failed identically - for a
    // convergent structure a bigger die makes every one of those wires
    // longer, so adding area adds routing demand.
    //
    // Instead: one DIM:1 mux per row, over PEs that are already physically
    // adjacent, then a register, then one DIM:1 mux across the rows. The
    // register anchors each row mux beside its own row, so long-haul routing
    // drops from DIM*DIM*32 wires to DIM*32 - 16x at DIM=16.
    //
    // The extra cycle is free: RESULT_IDX and RESULT are separate AXI
    // transactions many cycles apart, so the stage register is always settled
    // before the read that uses it.
    val idxBits   = math.max(1, log2Ceil(dim))
    val resCol    = resultIdx(idxBits - 1, 0)
    val resRow    = if (dim == 1) 0.U else resultIdx(2 * idxBits - 1, idxBits)
    val rowSel    = VecInit(pes.map(r => VecInit(r.map(_.io.acc.asUInt))(resCol)))
    val rowSelReg = RegNext(rowSel)
    val results   = rowSelReg(resRow)
    val rdata     = WireDefault(0.U(32.W))
    switch(raddrWord) {
      is(1.U)  { rdata := Cat(0.U(26.W), ldDone, ldBusy, dmaDone, dmaBusy, done, busy) }
      is(10.U) { rdata := destAddr }
      is(11.U) { rdata := destStride }
      is(12.U) { rdata := aSrcAddr }
      is(13.U) { rdata := bSrcAddr }
      is(14.U) { rdata := srcStride }
      is(15.U) { rdata := bPanelUse }
      is(16.U) { rdata := bPanelLoad }
      is(2.U) { rdata := kLen }
      is(3.U) { rdata := loadK }
      is(4.U) { rdata := loadLane }
      is(7.U) { rdata := resultIdx }
      is(8.U) { rdata := results }
      is(9.U) { rdata := Cat(0.U(8.W), bPanels.U(8.W), maxK.U(8.W), dim.U(8.W)) }
    }
    s_axi_rdata := rdata
  }
}

object EmitMmAccel extends App {
  val dim  = sys.env.getOrElse("MM_DIM", "2").toInt
  val maxK = sys.env.getOrElse("MM_MAXK", "16").toInt
  val out  = sys.env.getOrElse("MM_OUT", "generated")
  // bPanels was reachable only as a Scala default, so every sweep silently
  // built the same 4-panel design and the parameter was never actually varied.
  val bPan = sys.env.getOrElse("MM_BPANELS", "4").toInt

  // _root_ is required: `import chisel3.util._` brings chisel3.util.circt into
  // scope, so a bare `circt.stage` resolves to the wrong package.
  //
  // emitSystemVerilogFile writes straight to disk, so elaboration messages on
  // stdout can never land inside the .sv - which they do if the caller pipes
  // emitSystemVerilog's return value to a file.
  _root_.circt.stage.ChiselStage.emitSystemVerilogFile(
    new MmAccel(dim, maxK, bPan),
    args = Array("--target-dir", out),
    firtoolOpts = Array(
      "-disable-all-randomization",
      "-strip-debug-info",
      // noAlwaysComb emits `always @(*)` instead of `always_comb`. cpu.v pulls
      // this file in with `include and the rest of the SoC is Verilog-2001, so
      // a SystemVerilog-only keyword here fails Vivado's parser on the parent
      // file. This keeps ONE artifact that drops into the SoC, iverilog and
      // OpenLane alike, rather than a .sv for synthesis and a converted .v for
      // the CPU that could silently drift apart.
      "--lowering-options=disallowPackedArrays,disallowLocalVariables,noAlwaysComb"
    )
  )
  println(s"[EmitMmAccel] wrote $out/mm_accel.sv for dim=$dim maxK=$maxK bPanels=$bPan")
}
