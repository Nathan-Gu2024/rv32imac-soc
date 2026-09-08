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
//   word 9  INFO       (R)  {maxK[15:8], dim[7:0]} - geometry discovery, so
//                           software need not hardcode the array size
//   word 10 DEST_ADDR  (RW) byte address the result DMA writes to. Must be
//                           16-byte aligned; the burst is dim*dim/4 lines of
//                           128 bits, written in row-major accumulator order.
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
class MmAccel(val dim: Int = 2, val maxK: Int = 16) extends RawModule {
  require(dim >= 1, "dim must be positive")
  require(maxK % 4 == 0, "maxK must be a multiple of 4: operands pack 4 INT8 per write")
  require(isPow2(maxK), "maxK is used as an index width; keep it a power of two")

  override def desiredName = "mm_accel"

  val kBits    = log2Ceil(maxK)        // index of a single INT8 within a lane
  val kGrpBits = log2Ceil(maxK / 4)    // index of a 4-byte group
  val laneBits = log2Ceil(dim) max 1
  val accBits  = log2Ceil(dim * dim) max 1

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

    val aRowBuf = Seq.fill(dim)(Mem(maxK, SInt(8.W)))
    val bColBuf = Seq.fill(dim)(Mem(maxK, SInt(8.W)))

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
    val dmaStart     = doWrite && (waddrWord === 0.U) && s_axi_wdata(2) && !dmaBusy

    val pushA = doWrite && !busy && (waddrWord === 5.U)
    val pushB = doWrite && !busy && (waddrWord === 6.U)

    when(doWrite && !busy) {
      when(waddrWord === 2.U)  { kLen := s_axi_wdata(7, 0) }
      when(waddrWord === 10.U) { destAddr   := s_axi_wdata } // DEST_ADDR
      when(waddrWord === 11.U) { destStride := s_axi_wdata } // DEST_STRIDE
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
            when(pushA) { aRowBuf(i).write(addr, byte) }
            when(pushB) { bColBuf(i).write(addr, byte) }
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
      aEdge(i) := Mux(kValid, aRowBuf(i).read(kSel), 0.S)
      bEdge(i) := Mux(kValid, bColBuf(i).read(kSel), 0.S)
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
    val lineWords = 4
    val nGroups   = (dim * dim) / lineWords            // 128-bit lines per tile
    val grpBits   = log2Ceil(nGroups) max 1

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
        if (perRow == 1) groups(0) else groups(dmaGrp(log2Ceil(perRow) - 1, 0))
      })
      val rowLineReg = RegNext(rowLines)
      val rowSelDma  = if (perRow == 1) dmaGrp else dmaGrp >> log2Ceil(perRow)
      lineData := rowLineReg(rowSelDma)
    }

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

    when(dmaStart) {
      dmaBusy    := true.B
      dmaDone    := false.B
      dmaGrp     := 0.U
      dmaAddr    := destAddr
      dmaRowBase := destAddr
      dmaSettle  := true.B
    }.elsewhen(dmaBusy) {
      when(dmaSettle) {
        dmaSettle   := false.B          // let rowLineReg catch up to dmaGrp
        memReqValid := true.B
      }.elsewhen(mem_ready) {
        when(dmaGrp === (nGroups - 1).U) {
          dmaBusy     := false.B
          dmaDone     := true.B
          memReqValid := false.B
        }.otherwise {
          dmaGrp := dmaGrp + 1.U
          if (strideOK) {
            when(atRowEnd) {
              dmaAddr    := dmaRowBase + effStride
              dmaRowBase := dmaRowBase + effStride
            }.otherwise {
              dmaAddr := dmaAddr + (lineWords * 4).U
            }
          } else {
            dmaAddr := dmaAddr + (lineWords * 4).U
          }
          dmaSettle   := true.B
          memReqValid := false.B
        }
      }
    }

    mem_req_valid := memReqValid && dmaBusy && !dmaSettle
    mem_req_write := true.B                            // Phase 1 is write-only
    mem_req_addr  := dmaAddr
    mem_wline     := lineData


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
      is(1.U)  { rdata := Cat(0.U(28.W), dmaDone, dmaBusy, done, busy) }
      is(10.U) { rdata := destAddr }
      is(11.U) { rdata := destStride }
      is(2.U) { rdata := kLen }
      is(3.U) { rdata := loadK }
      is(4.U) { rdata := loadLane }
      is(7.U) { rdata := resultIdx }
      is(8.U) { rdata := results }
      is(9.U) { rdata := Cat(0.U(16.W), maxK.U(8.W), dim.U(8.W)) }
    }
    s_axi_rdata := rdata
  }
}

object EmitMmAccel extends App {
  val dim  = sys.env.getOrElse("MM_DIM", "2").toInt
  val maxK = sys.env.getOrElse("MM_MAXK", "16").toInt
  val out  = sys.env.getOrElse("MM_OUT", "generated")

  // _root_ is required: `import chisel3.util._` brings chisel3.util.circt into
  // scope, so a bare `circt.stage` resolves to the wrong package.
  //
  // emitSystemVerilogFile writes straight to disk, so elaboration messages on
  // stdout can never land inside the .sv - which they do if the caller pipes
  // emitSystemVerilog's return value to a file.
  _root_.circt.stage.ChiselStage.emitSystemVerilogFile(
    new MmAccel(dim, maxK),
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
  println(s"[EmitMmAccel] wrote $out/mm_accel.sv for dim=$dim maxK=$maxK")
}
