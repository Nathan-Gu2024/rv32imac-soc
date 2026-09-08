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
//   word 1  STATUS     (R)  bit0=BUSY, bit1=DONE
//   word 2  K_LEN      (RW) reduction depth for the next run, <= maxK
//   word 3  LOAD_K     (RW) k-GROUP index; each group is 4 packed INT8
//   word 4  LOAD_LANE  (RW) which row (A) or column (B) to push into.
//                           Writing it resets LOAD_K to 0.
//   word 5  A_PUSH     (W)  4 packed INT8 -> aRow[LOAD_LANE][4*LOAD_K ..+3],
//                           then LOAD_K auto-increments
//   word 6  B_PUSH     (W)  same for bCol[LOAD_LANE]
//   word 7  RESULT_IDX (RW) accumulator to read, = row*DIM + col
//   word 8  RESULT     (R)  accumulator[RESULT_IDX]
//   word 9  INFO       (R)  {maxK[15:8], dim[7:0]} - geometry discovery, so
//                           software need not hardcode the array size
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

  withClockAndReset(clk, rst) {

    val busy      = RegInit(false.B)
    val done      = RegInit(false.B)
    val kLen      = RegInit(0.U(8.W))
    val loadK     = RegInit(0.U(kGrpBits.W))
    val loadLane  = RegInit(0.U(laneBits.W))
    val resultIdx = RegInit(0.U(accBits.W))
    val t         = RegInit(0.U(8.W))

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
        when(s_axi_rready) { rvalidReg := false.B; rState := rIdle }
      }
    }

    s_axi_arready := arreadyReg
    s_axi_rvalid  := rvalidReg
    s_axi_rresp   := 0.U

    // ---- register writes (ignored while BUSY so an in-flight run is safe) ----
    val startPulse   = doWrite && (waddrWord === 0.U) && s_axi_wdata(0) && !busy
    val softRstPulse = doWrite && (waddrWord === 0.U) && s_axi_wdata(1)

    val pushA = doWrite && !busy && (waddrWord === 5.U)
    val pushB = doWrite && !busy && (waddrWord === 6.U)

    when(doWrite && !busy) {
      when(waddrWord === 2.U) { kLen := s_axi_wdata(7, 0) }
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
      when(pushA || pushB) { loadK := loadK + 1.U }
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
      is(1.U) { rdata := Cat(0.U(30.W), done, busy) }
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
      "--lowering-options=disallowPackedArrays,disallowLocalVariables"
    )
  )
  println(s"[EmitMmAccel] wrote $out/mm_accel.sv for dim=$dim maxK=$maxK")
}
