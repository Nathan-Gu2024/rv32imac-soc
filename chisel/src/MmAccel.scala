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
// This map is CONSTANT SIZE in DIM - 23 words, 0..22, inside the 64-word MMIO
// window - because the lane and the accumulator are addressed by index rather
// than by address. It is the size in DIM that matters, not the count: the count
// grew from 10 to 23 as the DMA, the panel split and the descriptor queue
// landed, and none of that scaled with DIM.
//
// Listed in address order. The authority is the decode at the bottom of this
// file (`switch(raddrWord)` and the `waddrWord` whens); if they disagree, they
// are right and this comment is stale.
//
//   word 0  CTRL        (W) bit0=START (ignored while BUSY), bit1=SOFT_RST,
//                           bit2=result-DMA start, bit3=operand-DMA start
//                           (A and B), bit4=operand-DMA start (B only),
//                           bit5=descriptor-queue start
//   word 1  STATUS      (R) bit0=BUSY,    bit1=DONE,
//                           bit2=DMA_BUSY bit3=DMA_DONE   (result store)
//                           bit4=LD_BUSY  bit5=LD_DONE    (operand fetch)
//                           bit6=Q_BUSY   bit7=Q_DONE     (batch)
//                           bit8=Q_OVERFLOW - sticky, a DESC_PUSH was dropped
//                           because the queue was full. Cleared by the next
//                           queue kick. Check it after assembling a batch: the
//                           alternative is a missing tile with no error.
//   word 2  K_LEN       (RW) reduction depth for the next run, <= maxK
//   word 3  LOAD_K      (RW) k-GROUP index; each group is 4 packed INT8
//   word 4  LOAD_LANE   (RW) which row (A) or column (B) to push into.
//                           Writing it resets LOAD_K to 0.
//   word 5  A_PUSH      (W) 4 packed INT8 -> aRow[LOAD_LANE][4*LOAD_K ..+3],
//                           then LOAD_K auto-increments
//   word 6  B_PUSH      (W) same for bCol[LOAD_LANE]
//   word 7  RESULT_IDX  (RW) accumulator to read, = row*DIM + col
//   word 8  RESULT      (R) accumulator[RESULT_IDX], then RESULT_IDX
//                           auto-increments - so a whole tile reads back with
//                           one transaction per element instead of two
//   word 9  INFO        (R) {bPanels[23:16], maxK[15:8], dim[7:0]} - geometry
//                           discovery, so software need not hardcode any of it
//   word 10 DEST_ADDR   (RW) byte address the result DMA writes to. Must be
//                           16-byte aligned; the burst is dim*dim/4 lines of
//                           128 bits, written in row-major accumulator order.
//   word 11 DEST_STRIDE (RW) byte distance between consecutive RESULT ROWS.
//                           0 = contiguous (stride = dim*4). Set this to the
//                           full matrix row pitch (N*4) to have a tile land
//                           directly in its place inside a larger C, with no
//                           software copy. Ignored at dim=2, where one
//                           128-bit line spans both rows - and it must be 0 in
//                           INT8 mode, where a line spans 16/dim result rows
//                           and there is no row boundary to stride at.
//   word 12 A_SRC_ADDR  (RW) byte address the operand DMA fetches the A panel
//   word 13 B_SRC_ADDR  (RW) byte address for the B panel
//   word 14 SRC_STRIDE  (RW) byte pitch between operand rows at the source
//   word 15 B_PANEL_USE (RW) which B scratchpad panel the ARRAY reads
//   word 16 B_PANEL_LOAD(RW) which B panel a push or operand DMA WRITES.
//                           Split from B_PANEL_USE so the next panel can be
//                           filled while the current one feeds a run.
//   word 17 DESC_A_SRC  (W) staged into the next descriptor (see DESC_PUSH)
//   word 18 DESC_B_SRC  (W) same
//   word 19 DESC_DEST   (W) same
//   word 20 DESC_PUSH   (W) commits a descriptor built from words 17-19 plus:
//                           {bOnly[16], panelLoad[15:12], panelUse[11:8], kLen[7:0]}
//                           bOnly=1 fetches ONLY the B panel, leaving the A
//                           panel resident from the previous tile. In a tiled
//                           GEMM C[ti][tj] = A[ti]*B[tj], a row of tiles holds
//                           ti fixed, so A is identical across all of them and
//                           re-fetching it is pure waste: at dim=8,maxK=64 the
//                           A panel is 32 of the 80 lines a tile moves.
//   word 21 QUEUE_FREE  (R) descriptor slots still free, so software can push
//                           without overflowing the queue
//   word 22 OUT_CTRL    (RW) bits[4:0]=outShift, bit8=INT8 requantize enable.
//                           With INT8 on, a tile stores as dim*dim BYTES
//                           instead of INT32 accumulators - four cache lines
//                           at dim=16 rather than sixteen, which is where most
//                           of the destination-invalidate cost went.
//
// Two interrupt lines leave this module alongside the register window:
// irq_batch (a queued batch finished) and irq_dma (a standalone result store
// finished). Both are sticky levels, not pulses, because intc.v edge-captures
// them - see the note at their declaration.
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
              val enableWriteBursts: Boolean = true,
              // Overlap tile N+1's operand load with tile N's compute. Measured
              // worth 25% on an A-reuse batch, but currently corrupts the first
              // prefetched tile - see tryPrefetch's STATUS note. OFF until that
              // is root-caused; turning it on is how you reproduce the failure.
              val enablePrefetch: Boolean = true,
              val descDepth: Int = 8) extends RawModule {
  require(dim >= 1, "dim must be positive")
  require(maxK % 4 == 0, "maxK must be a multiple of 4: operands pack 4 INT8 per write")
  require(isPow2(maxK), "maxK is used as an index width; keep it a power of two")
  require(bPanels >= 1 && isPow2(bPanels), "bPanels must be a power of two")
  // The operand DMA moves whole 128-bit lines and a lane is stored as 16-byte
  // rows, so a lane shorter than one line has never been representable on that
  // path - the byte guard that used to hide this just dropped the surplus.
  require(maxK >= 16, "maxK must be at least 16: an operand row is one 128-bit line")
  require(fifoDepth >= 2, "the result FIFO needs at least two entries to run ahead")

  override def desiredName = "mm_accel"

  val kBits    = log2Ceil(maxK)        // index of a single INT8 within a lane
  val kGrpBits = log2Ceil(maxK / 4)    // index of a 4-byte group
  val laneBits = log2Ceil(dim) max 1
  val accBits  = log2Ceil(dim * dim) max 1
  val panBits  = log2Ceil(bPanels) max 1
  // 128-bit lines per lane: the unit both the DMA and the operand buffers work
  // in. Hoisted here because the buffer declaration now needs it too.
  val linesPerLane = maxK / 16
  val lplBits      = log2Ceil(linesPerLane) max 1

  // A k index splits into (row, byte) over the 16-byte operand rows.
  private def kRow(k: UInt): UInt  = if (linesPerLane == 1) 0.U else k(kBits - 1, 4)
  private def kByte(k: UInt): UInt = k(3, 0)

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
  // TWO channels, not one. The single port carried no direction tag on its
  // completion, so the load capture had to guess with !dmaBusy and the store
  // credit guessed with nothing - a read line arriving during a store was both
  // dropped by the loader and miscredited to the store. Separate completions
  // make that unrepresentable rather than merely guarded against.
  //
  // The asymmetry is part of the contract and is not an accident: a READ
  // completes one LINE at a time, a WRITE once per TRANSACTION.
  //
  // ---- read channel: operand fetch ----
  val mem_rd_req_valid = IO(Output(Bool()))
  val mem_rd_req_addr  = IO(Output(UInt(32.W)))
  val mem_rd_req_lines = IO(Output(UInt(8.W)))
  val mem_rline        = IO(Input(UInt(128.W)))
  val mem_rd_ready     = IO(Input(Bool()))      // ONE PULSE PER LINE
  // ---- write channel: result store ----
  val mem_wr_req_valid = IO(Output(Bool()))
  val mem_wr_req_addr  = IO(Output(UInt(32.W)))
  val mem_wr_req_lines = IO(Output(UInt(8.W)))
  val mem_wline        = IO(Output(UInt(128.W)))
  val mem_wnext        = IO(Input(Bool()))      // requester: present next line
  val mem_wr_ready     = IO(Input(Bool()))      // ONE PULSE PER TRANSACTION

  // Completion interrupts, driven straight from the sticky DONE registers.
  //
  // These are LEVELS, not pulses, and that is deliberate: src/intc.v captures
  // on the 0->1 EDGE (irq_edge = irq_in & ~irq_in_prev) and latches into its
  // own pending register, which software clears write-1-to-clear. qDone is set
  // once when a batch drains and cleared only by the next CTRL bit5 kick, so
  // it produces exactly one clean edge per batch - which is precisely the
  // shape intc wants. A pulse generator here would add state for nothing, and
  // an accelerator-side enable/clear register would duplicate intc's own
  // ENABLE and PENDING.
  //
  // Only these two are exported. done/ldDone also go high once per TILE while
  // a queued batch runs (the sequencer drives startAny and ldStartAny from
  // qComp/qLoad), so wiring them to interrupts would fire the ISR once per
  // tile for markers that have no meaning outside the sequencer.
  val irq_batch = IO(Output(Bool()))   // a queued batch finished (CTRL bit5)
  val irq_dma   = IO(Output(Bool()))   // a standalone result store finished

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

    // The result FIFO must hold an ENTIRE tile, not merely enough to run ahead.
    // It is what frees the accumulators: once every line has been pushed, the
    // array can start the next run while the FIFO drains to memory at whatever
    // rate the platform accepts writes. At depth 8 against 16 lines per tile
    // the accumulators stayed live for the whole ~178-cycle store, so the array
    // sat idle through it; at nGroups they are free after nGroups cycles.
    val fifoLines = fifoDepth max nGroups

    // ---- INT8 requantized output geometry ----
    //
    // A raw INT32 accumulator is four bytes; a requantized result is one. That
    // turns a dim=8 tile from 16 lines into 4, and result writeback was 61% of
    // the tile's memory time because this platform accepts writes at roughly
    // 1.4 B/cycle against 4.5 for reads. Accumulating wide and scaling down on
    // the way out is also simply what a quantized INT8 GEMM does - Gemmini and
    // OpenGeMM both requantize before anything reaches memory.
    //
    // A 16-byte line now holds 16 results, i.e. 16/dim whole result ROWS, so
    // this needs dim to divide 16 and a tile to fill at least one line. dim=2
    // gives a 4-byte tile and cannot: the line port has no byte strobes, so a
    // partial line is not writable.
    val int8Capable = (dim * dim) >= 16 && (16 % dim == 0)
    val rowsPerLine = if (int8Capable) 16 / dim else 1
    val nGroups8    = if (int8Capable) (dim * dim) / 16 else 1
    val grp8Bits    = log2Ceil(nGroups8) max 1

    // FIFO fill pointer and remaining-group count.
    val fillGrp  = RegInit(0.U(grpBits.W))
    val fillLeft = RegInit(0.U(9.W))

    // ---- descriptor queue ----
    // One descriptor fully describes a tile: where A and B come from, where the
    // result goes, and the geometry. Software stages the first three words and
    // the write to DESC_PUSH commits the entry.
    class Descriptor extends Bundle {
      val aSrc = UInt(32.W)
      val bSrc = UInt(32.W)
      val dest = UInt(32.W)
      val ctl  = UInt(32.W)
    }
    // hasFlush so SOFT_RST can actually empty the queue. Without it there is no
    // way to abandon a batch short of a full SoC reset, which is what made the
    // driver's timeout path worse than having none.
    val descQ  = Module(new Queue(new Descriptor, descDepth, hasFlush = true))
    val descA  = RegInit(0.U(32.W))
    val descB  = RegInit(0.U(32.W))
    val descD  = RegInit(0.U(32.W))
    val qRun   = RegInit(false.B)
    val qBusy  = RegInit(false.B)
    val qDone  = RegInit(false.B)
    // Sticky: a DESC_PUSH arrived with the queue full and was DROPPED.
    //
    // A Chisel Queue silently discards the beat when enq.ready is low, and
    // enq.valid is driven from descPush alone - so without this the failure mode
    // is a missing tile in the output matrix while STATUS reports success.
    // Software is expected to read QUEUE_FREE first and the driver does, but that
    // is a separate AXI transaction with nothing enforcing the ordering, so the
    // hardware needs to be able to say it happened.
    val qOverflow = RegInit(false.B)
    // Set from the current descriptor: fetch only the B panel and leave A
    // resident. See DESC_PUSH in the register map above.
    val qBOnly = RegInit(false.B)

    val destAddr = RegInit(0.U(32.W))
    val dmaAddr  = RegInit(0.U(32.W))
    val dmaBusy  = RegInit(false.B)
    val dmaDone  = RegInit(false.B)

    // See the port declarations above for why these are levels off the sticky
    // DONE bits rather than pulses.
    irq_batch := qDone
    irq_dma   := dmaDone

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

    // Operand buffers: one synchronous-read memory per lane, 16-byte rows.
    //
    // These were flat registers indexed [lane][k], read COMBINATIONALLY at a
    // dynamic k. That is exactly the structure the L1 caches were converted
    // away from: an earlier revision read those arrays combinationally, could
    // not infer block RAM, and cost ~11K LUTs and ~20K flops in distributed RAM
    // plus F7/F8 mux trees. At maxK=16 the operand buffers were small enough
    // that it never showed. At maxK=64 they are 20,480 bits behind a 64:1 mux
    // per lane per panel, and firtool emitted 102K lines of Verilog for one
    // accelerator - 17x the maxK=16 build.
    //
    // A 16-byte row is the natural width: the operand DMA delivers a 128-bit
    // LINE, so a whole row still lands in ONE cycle, which is the property the
    // registers existed to provide. Byte-granular MMIO pushes survive through
    // the write mask, so that path is unchanged.
    //
    // The read is synchronous, so the address is driven one cycle ahead - the
    // same trick both caches use (the icache from the next PC, the dcache from
    // a dedicated EX-stage adder). Within a run k advances by exactly one per
    // cycle, so the next address is always known: no stall, no schedule change,
    // and cycle counts stay identical to the register version.
    val aMem = Seq.fill(dim)(SyncReadMem(linesPerLane, Vec(16, UInt(8.W))))

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
    // The panel index is folded into the ROW ADDRESS rather than selected by a
    // mux after the read. That removes the bPanels-way mux on the array's
    // critical path entirely - and with it the packed-array lowering that made
    // bPanels=2 fail to elaborate while 1, 4 and 8 happened to survive.
    val bMem = Seq.fill(dim)(SyncReadMem(bPanels * linesPerLane, Vec(16, UInt(8.W))))

    // Panel p occupies rows [p*linesPerLane, (p+1)*linesPerLane). The multiply
    // is by a power-of-two constant, so it lowers to a concatenation.
    def bAddr(panel: UInt, row: UInt): UInt =
      if (bPanels == 1) row else (panel * linesPerLane.U) + row

    // ONE write port per buffer, shared by the MMIO push path and the operand
    // DMA. They cannot collide - a push is an AXI transaction and the DMA
    // writes only while ldBusy - but written as two separate .write() sites
    // firtool emitted a 1R2W memory, which no block RAM or SRAM macro can
    // implement, so the whole point of moving off registers would have been
    // lost. Driving shared wires keeps each buffer a plain 1R1W.
    val aRowW = log2Ceil(linesPerLane) max 1
    val bRowW = log2Ceil(bPanels * linesPerLane) max 1
    val aWen  = WireDefault(VecInit(Seq.fill(dim)(false.B)))
    val bWen  = WireDefault(VecInit(Seq.fill(dim)(false.B)))
    val aWrow = WireDefault(VecInit(Seq.fill(dim)(0.U(aRowW.W))))
    val bWrow = WireDefault(VecInit(Seq.fill(dim)(0.U(bRowW.W))))
    val wData = WireDefault(VecInit(Seq.fill(dim)(VecInit(Seq.fill(16)(0.U(8.W))))))
    val wMask = WireDefault(VecInit(Seq.fill(dim)(VecInit(Seq.fill(16)(false.B)))))
    for (i <- 0 until dim) {
      when(aWen(i)) { aMem(i).write(aWrow(i), wData(i), wMask(i)) }
      when(bWen(i)) { bMem(i).write(bWrow(i), wData(i), wMask(i)) }
    }

    // Which panel the ARRAY reads during a run, and which the DMA/push WRITES.
    // Separate registers on purpose: that split is what allows the next panel
    // to be filled while the current one feeds a run - double buffering, once
    // the fill no longer has to be serialised behind compute.
    // OUT_CTRL: {int8[8], shift[4:0]}. Default 0 keeps the INT32 behaviour
    // every existing driver and testbench depends on - requantization is
    // opt-in, because it changes the output TYPE, not merely its encoding.
    val outShift = RegInit(0.U(5.W))
    val int8Req  = RegInit(false.B)
    val int8Mode = if (int8Capable) int8Req else false.B

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
    val dmaStart     = doWrite && (waddrWord === 0.U) && s_axi_wdata(2) && !dmaBusy
    // CTRL bit3 kicks the operand load. Gated on BOTH engines being idle: they
    // share one line port and one mem_ready, so overlapping them would corrupt
    // whichever request happened to be in flight.
    val ldStart      = doWrite && (waddrWord === 0.U) && s_axi_wdata(3) && !ldBusy
    // CTRL bit4: load ONLY the B panel, leaving aRowBuf untouched so this may
    // run concurrently with compute. Safe only when B_PANEL_LOAD differs from
    // B_PANEL_USE - software's responsibility, and what double buffering means.
    val ldStartB     = doWrite && (waddrWord === 0.U) && s_axi_wdata(4) && !ldBusy
    // CTRL bit5 starts the queue. Writing DESC_PUSH (word 20) commits a
    // descriptor built from the three staged words plus this one.
    val qStartPulse  = doWrite && (waddrWord === 0.U) && s_axi_wdata(5) && !qBusy
    val descPush     = doWrite && (waddrWord === 20.U)

    // Latch a dropped push. Cleared when a new batch is kicked, so the bit
    // always describes the batch being assembled rather than accumulating
    // forever.
    when(descPush && !descQ.io.enq.ready) { qOverflow := true.B }
      .elsewhen(qStartPulse)              { qOverflow := false.B }

    descQ.io.enq.valid     := descPush
    descQ.io.enq.bits.aSrc := descA
    descQ.io.enq.bits.bSrc := descB
    descQ.io.enq.bits.dest := descD
    descQ.io.enq.bits.ctl  := s_axi_wdata
    descQ.io.deq.ready     := false.B

    // ---- queue sequencer ----
    // Per descriptor: fetch operands, compute, write results back - with no CPU
    // round trip between phases. Each phase has a one-cycle issue state and a
    // wait state, so the pulses below are naturally single-cycle.
    // qPreW is the overlap state: the current tile's store has drained and the
    // NEXT tile's operands are already loading (or loaded), so the sequencer
    // waits on that prefetch instead of starting a fresh load from idle.
    val qIdle :: qLoad :: qLoadW :: qComp :: qCompW :: qStore :: qStoreW :: qPreW :: qPreIssue :: Nil = Enum(9)
    val qState = RegInit(qIdle)

    // ---- operand prefetch: overlap tile N+1's load with tile N's compute ----
    //
    // Measured opportunity: the memory port is idle for 100% of the array's
    // compute window (tb_mm_accel_queue reports array-busy-with-port-idle = 23%
    // of a batch), because the sequencer ran load -> compute -> store strictly in
    // series with ONE set of config registers.
    //
    // The blocker was that kLen fed both the load sizing (ldLines) and the array
    // schedule (lastT/kValid), so advancing to the next descriptor mid-compute
    // would have resized the running tile. Hence two contexts:
    //
    //   LOAD ctx    aSrcAddr, bSrcAddr, bPanelLoad, qBOnly, ldKLen
    //   STAGED      the same descriptor's compute/store fields, held until its
    //               operands are resident
    //   COMPUTE ctx kLen, bPanelUse, destAddr  - promoted from STAGED on ldDone
    //
    // A prefetch is only started when the panel it would overwrite is not the
    // panel the array is about to read, which is what bPanels > 1 exists for.
    val ldKLen      = RegInit(0.U(8.W))   // load sizing only; array uses kLen
    val stgKLen     = RegInit(0.U(8.W))
    val stgPanelUse = RegInit(0.U(panBits.W))
    val stgDest     = RegInit(0.U(32.W))

    // preBusy: a prefetch has been issued for the tile after the one computing.
    // preDone: that prefetch's ldDone has been seen (sticky, because it usually
    //          lands during compute or the store, long before it is consumed).
    val preBusy = RegInit(false.B)
    val preDone = RegInit(false.B)
    // One-cycle start pulse for the load engine, driven from the FSM below and
    // consumed by ldStartAny further down.
    //
    // It is asserted the cycle AFTER the descriptor is latched, not the same
    // cycle. The load engine's start block reads the load context as REGISTERS
    // (ldInB := bOnly, with bOnly taken from qBOnly), so firing
    // preStart in the same cycle as tryPrefetch's writes starts the load on the
    // PREVIOUS descriptor's address and b_only flag. The qLoad path never had
    // this problem because qIdle latches and only then moves to qLoad, a full
    // cycle apart; prePend reproduces that separation for the prefetch.
    //
    // Concretely, without it: the first prefetch is issued from qLoadW, where the
    // descriptor that just finished is tile 0 - which is NOT b_only. qBOnly is
    // therefore still false, so the prefetch starts an A+B load with
    // ldInB := false and walks the A buffer instead of loading B alone.
    val preStart = WireDefault(false.B)
    val prePend  = RegInit(false.B)

    // Operands for the staged descriptor are now resident: it becomes the tile
    // the array and the store work on.
    def promoteStaged(): Unit = {
      kLen      := stgKLen
      bPanelUse := stgPanelUse
      destAddr  := stgDest
    }

    // Start loading the next descriptor, if there is one and it is safe.
    //
    // TWO conditions, both necessary.
    //
    // 1. The panel being loaded into must not be the panel the array is about to
    //    read. That is what bPanels > 1 exists for. stgPanelUse is the panel just
    //    promoted to bPanelUse, so it is what to compare against.
    //
    // 2. The prefetched descriptor must be B-ONLY. The B scratchpad is banked
    //    into bPanels panels, but there is exactly ONE A buffer - A is not
    //    double-buffered at all. So prefetching a tile that fetches A would
    //    overwrite the A operands the CURRENTLY COMPUTING tile is still reading.
    //    That is not a theoretical hazard: without this term the queue bench
    //    reported 124 wrong results, because every tile in it fetches A.
    //
    //    This costs nothing in the case the feature exists for. In a tiled GEMM
    //    walking a row of C, ti is fixed, so tile 0 fetches A and every tile
    //    after it is b_only - which is exactly the shape the A-reuse work
    //    introduced and what the driver already emits.
    //
    // When either condition fails the prefetch is skipped: that tile loses its
    // overlap and nothing is incorrect.
    // STATUS: ENABLED. A-reuse batch in tb_mm_accel_queue: 469 -> 371 cycles
    // (21%), results correct, suite 10/11 with only the pre-existing
    // tb_mm_accel_c_test failure outstanding.
    //
    // Three separate bugs had to be fixed to get here, and the first version of
    // this feature had all three at once - which is why it appeared to "work and
    // be fast" at 350 cycles while returning a tile of zeros. That 350 was not a
    // speedup: it was the cost of an operand load the sequencer never waited for.
    //
    //   1. preStart fired in the SAME cycle tryPrefetch wrote the load context,
    //      so the load engine sampled the PREVIOUS descriptor's address and
    //      b_only flag. The qLoad path was always immune because qIdle latches
    //      and only then transitions. Fixed with prePend.
    //
    //   2. preDone latched on ldDone as a LEVEL. ldDone is not a pulse - it stays
    //      high from completion until the next load starts - so preBusy rising
    //      inside that window latched the previous load's completion, and qPreW
    //      promoted a tile whose B panel had never been fetched. That produced
    //      the tile of zeros (62 of 64 words, not 6 as first recorded). Fixed by
    //      requiring a rising edge.
    //
    //   3. With qPreW finally waiting properly, a real DEADLOCK surfaced: the
    //      last line of an in-flight read burst was discarded when a store
    //      started on that exact cycle, because the load captures only
    //      `when(mem_ready && !dmaBusy)` and mem_ready is ambiguous between
    //      "read line ready" and "write accepted". Fixed by refusing to enter
    //      qStore while ldBusy.
    //
    // Note what (3) means for sequencing: prefetch does NOT come for free before
    // the read/write split. It overlaps the operand load with COMPUTE only;
    // overlapping it with the result store needs independent read and write paths
    // and an unambiguous mem_ready, which is the T3.2 work.
    def tryPrefetch(): Unit = {
      val nextCtl   = descQ.io.deq.bits.ctl
      val nextPanel = nextCtl(12 + panBits - 1, 12)
      val nextBOnly = nextCtl(16)
      // !preBusy makes the one-prefetch-outstanding rule LOCAL. It already
      // held, but only because of where this is called from - qPreIssue, and
      // qPreW which clears preBusy first. That is a non-local invariant, and the
      // single panel comparison below is only sufficient while it holds: with two
      // prefetches in flight there would be more than one live B panel and
      // comparing against stgPanelUse alone would let a load overwrite a panel
      // the array is still reading.
      when(enablePrefetch.B && !preBusy && descQ.io.deq.valid && nextBOnly &&
           (nextPanel =/= stgPanelUse)) {
        aSrcAddr    := descQ.io.deq.bits.aSrc
        bSrcAddr    := descQ.io.deq.bits.bSrc
        ldKLen      := nextCtl(7, 0)
        bPanelLoad  := nextPanel
        qBOnly      := nextCtl(16)
        stgKLen     := nextCtl(7, 0)
        stgPanelUse := nextCtl(8 + panBits - 1, 8)
        stgDest     := descQ.io.deq.bits.dest
        descQ.io.deq.ready := true.B
        prePend     := true.B   // preStart fires next cycle; see its declaration
        preBusy     := true.B
        preDone     := false.B
      }
    }

    when(qStartPulse) {
      qRun  := true.B
      qBusy := true.B
      qDone := false.B
    }

    switch(qState) {
      is(qIdle) {
        when(qRun && descQ.io.deq.valid) {
          // Adopt into the LOAD context plus staging, then retire the
          // descriptor. The compute context is NOT written here: it is promoted
          // from staging once these operands are actually resident, which is
          // what lets the next descriptor be adopted while this tile computes.
          aSrcAddr    := descQ.io.deq.bits.aSrc
          bSrcAddr    := descQ.io.deq.bits.bSrc
          ldKLen      := descQ.io.deq.bits.ctl(7, 0)
          bPanelLoad  := descQ.io.deq.bits.ctl(12 + panBits - 1, 12)
          qBOnly      := descQ.io.deq.bits.ctl(16)
          stgKLen     := descQ.io.deq.bits.ctl(7, 0)
          stgPanelUse := descQ.io.deq.bits.ctl(8 + panBits - 1, 8)
          stgDest     := descQ.io.deq.bits.dest
          descQ.io.deq.ready := true.B
          qState := qLoad
        // !ldBusy added alongside !dmaBusy. A batch must not report DONE with a
        // fetch still in flight. That held before only because no load could be
        // outstanding without either qLoadW waiting on it or preBusy set - a
        // non-local invariant, and loads now overlap stores.
        }.elsewhen(qRun && !descQ.io.deq.valid && !dmaBusy && !ldBusy) {
          // Queue drained AND the last store has completed. The !dmaBusy term
          // is load-bearing now that qStoreW releases on the drain rather than
          // on completion: without it the batch could report DONE with a write
          // still in flight, and software would read the destination early.
          qRun  := false.B
          qBusy := false.B
          qDone := true.B
        }
      }
      is(qLoad)   { qState := qLoadW }
      is(qLoadW)  {
        when(ldDone) {
          promoteStaged()
          // NOTE: no tryPrefetch() here. Starting the next load on the SAME
          // cycle ldDone is observed corrupted the prefetched tile - only that
          // tile, and only when prefetched from here rather than from qPreW,
          // which is what localised it. The prefetch is issued from qPreIssue
          // one cycle later instead.
          qState := qPreIssue
        }
      }
      // One cycle of separation between "the previous load reported done" and
      // "the next load starts". Costs a single cycle per batch, not per tile,
      // because every later prefetch is issued from qPreW.
      is(qPreIssue) {
        tryPrefetch()
        qState := qComp
      }
      is(qComp)   { qState := qCompW }
      // Only one result DMA may be in flight: entering qStore asserts
      // dmaStartAny, which reloads dmaAddr and fillLeft, so doing that while
      // the previous store is still draining would redirect it mid-transfer.
      // !ldBusy is load-bearing with prefetch enabled. A read burst is requested
      // once and then streams, with ldReq dropped after the first line; the load
      // captures only `when(mem_ready && !dmaBusy)`, so a store starting
      // mid-burst makes the load DISCARD lines that were actually delivered and
      // wait for them forever. Measured: the 8th line of an 8-line burst landed
      // on the same cycle dmaBusy rose, and the load hung on lane 7.
      //
      // Serialising here costs the load-against-store overlap only. The overlap
      // prefetch exists for - load against COMPUTE - is unaffected. Overlapping
      // reads with writes needs independent read/write paths and an unambiguous
      // mem_ready, which is the T3.2 work, not this.
      // !ldBusy is GONE: the store no longer waits for the operand fetch. That
      // term was never about the array - it was there because a read line
      // arriving while dmaBusy was set got dropped by the loader and miscredited
      // to the store. Separate completions make that impossible.
      // !dmaBusy stays: still one store at a time.
      is(qCompW)  { when(done && !dmaBusy) { qState := qStore } }
      is(qStore)  { qState := qStoreW }
      // Release the array as soon as the accumulators are DRAINED, not when
      // the write completes. A store is two separate things: pushing nGroups
      // lines into the result FIFO, which takes nGroups cycles, and getting
      // those lines into memory, which takes ~178 on hardware because the
      // write port accepts roughly a beat every five cycles. Only the first
      // needs the accumulators. Waiting for dmaDone held the entire array idle
      // through a transfer it had already finished contributing to.
      //
      // The next tile's load then overlaps the tail of this store; the port
      // arbitration keeps them from colliding, so the load waits for the port
      // while the rest of its sequencing proceeds.
      is(qStoreW) {
        when(dmaDone || (dmaBusy && fillLeft === 0.U)) {
          // If the next tile's operands are already on their way, go straight to
          // waiting on that rather than back to idle - qIdle would try to
          // dequeue a descriptor that has already been consumed.
          qState := Mux(preBusy, qPreW, qIdle)
        }
      }
      // The overlap payoff lands here: by this point the prefetch has usually
      // finished during compute or the store, so preDone is already set and this
      // state costs a single cycle instead of a whole operand fetch.
      is(qPreW) {
        when(preDone) {
          promoteStaged()
          preBusy := false.B
          preDone := false.B
          tryPrefetch()
          qState := qComp
        }
      }
    }

    // Latch the prefetch's completion. Sticky, because the completion normally
    // arrives while the array is still computing or the store is draining - many
    // cycles before qPreW consumes it.
    //
    // ON THE RISING EDGE, which is the whole point. ldDone is NOT a pulse: the
    // load engine asserts it on completion and clears it only on the next
    // ldStartAny, so it stays high across the entire gap between one load
    // finishing and the next starting. preBusy goes high inside that gap - at
    // qPreIssue, one cycle after qLoadW observed the previous load's ldDone - so
    // a level-sensitive latch here fired on the PREVIOUS load's completion,
    // before the prefetch had fetched a single line. qPreW then found preDone
    // already set, promoted the staged descriptor immediately, and computed the
    // tile against a B panel that had never been loaded: the whole tile came out
    // as its cleared accumulator value. That was the 62-of-64 zeros in the first
    // prefetched tile, and it is why the failure was specific to the first one -
    // only there does preBusy rise while a stale ldDone is still asserted.
    val ldDonePrev = RegNext(ldDone, false.B)
    when(ldDone && !ldDonePrev && preBusy) { preDone := true.B }

    val pushA = doWrite && !busy && (waddrWord === 5.U)
    val pushB = doWrite && !busy && (waddrWord === 6.U)

    // DESCRIPTOR STAGING IS NOT GATED ON !busy, deliberately, and this is a fix
    // rather than an oversight in the other direction.
    //
    // These three were originally inside the `!busy` block below alongside the
    // config registers, but DESC_PUSH (word 20) never was - so a descriptor
    // pushed while the array was computing committed the PREVIOUS descriptor's
    // addresses and both read and wrote the wrong buffers, silently. `busy` is
    // high for the compute phase of every tile (~23% of a tile at dim=8, K=64)
    // and each of these is an independent ~9.5-cycle AXI write, so the window is
    // wide open.
    //
    // Ungating the staging registers is the right half to change: unlike kLen,
    // destAddr and the panel selects, these are never read by the running array.
    // The queue sequencer copies a DEQUEUED descriptor into the config registers,
    // so the staging copies are dead until a push consumes them. Gating
    // DESC_PUSH instead would have silently dropped the push and foreclosed
    // streaming descriptors into a running queue, which is the only way past the
    // depth-8 batch ceiling.
    when(doWrite) {
      when(waddrWord === 17.U) { descA := s_axi_wdata }   // DESC_A_SRC
      when(waddrWord === 18.U) { descB := s_axi_wdata }   // DESC_B_SRC
      when(waddrWord === 19.U) { descD := s_axi_wdata }   // DESC_DEST
    }

    when(doWrite && !busy) {
      // Clamped to maxK. The register map documents "<= maxK" as a caller
      // constraint, but nothing enforced it and the failure was silent: kSel
      // wraps, so K_LEN=100 at maxK=64 reads wrapped operand rows and produces
      // a plausible-looking wrong answer. ldLines right beside this is already
      // clamped, so this was an inconsistency as much as a hole.
      when(waddrWord === 2.U) {
        val kReq = s_axi_wdata(7, 0)
        // Both copies. On the manual path software writes K_LEN and then drives
        // the load itself, so the load engine's ldKLen must follow it; only the
        // descriptor queue ever sets them to different values, and only while a
        // prefetch is in flight.
        // kLen is the ARRAY's copy and !busy is the right guard for it.
        // ldKLen is the LOAD engine's and needs !ldBusy: it feeds ldLines, hence
        // ldPanelEnd (the load's completion condition), linesPerPanel (hence
        // mem_req_lines) and burstOK (hence burst mode). The arbiter latches its
        // own line count at grant, so moving this mid-load makes the two sides
        // disagree about how many lines are coming - the load then either
        // finishes early or waits forever for a line nobody asked for.
        kLen   := Mux(kReq > maxK.U, maxK.U, kReq)
        when(!ldBusy) { ldKLen := Mux(kReq > maxK.U, maxK.U, kReq) }
      }
      // Destination geometry belongs to the store; source geometry to the load.
      // Each is now held still while its own engine is running, which is the
      // invariant the split relies on and which !busy alone did not provide -
      // busy is the ARRAY, and the array is idle for most of a transfer.
      when(waddrWord === 10.U) { when(!dmaBusy) { destAddr   := s_axi_wdata } }
      when(waddrWord === 11.U) { when(!dmaBusy) { destStride := s_axi_wdata } }
      when(waddrWord === 12.U) { when(!ldBusy)  { aSrcAddr   := s_axi_wdata } }
      when(waddrWord === 13.U) { when(!ldBusy)  { bSrcAddr   := s_axi_wdata } }
      when(waddrWord === 14.U) { when(!ldBusy)  { srcStride  := s_axi_wdata } }
      when(waddrWord === 22.U) {                      // OUT_CTRL
        outShift := s_axi_wdata(4, 0)
        int8Req  := s_axi_wdata(8)
      }
      when(waddrWord === 15.U) { bPanelUse  := s_axi_wdata(panBits - 1, 0) }
      // bPanelLoad picks the row the load's captures are written into, so it
      // must not move under an in-flight load. bPanelUse above is the array's,
      // and !busy is correct for that one.
      when(waddrWord === 16.U) {
        when(!ldBusy) { bPanelLoad := s_axi_wdata(panBits - 1, 0) }
      }
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
          // A push carries 4 packed bytes at a 4-byte-aligned k, so they always
          // land in the same 16-byte row: one masked write, not four.
          val row = if (linesPerLane == 1) 0.U else loadK(kGrpBits - 1, 2)
          val grp = loadK(1, 0)
          when(pushA || pushB) {
            wData(i) := VecInit(Seq.tabulate(16)(b =>
                          s_axi_wdata(8 * (b % 4) + 7, 8 * (b % 4))))
            wMask(i) := VecInit(Seq.tabulate(16)(b => (b / 4).U === grp))
            aWrow(i) := row
            bWrow(i) := bAddr(bPanelLoad, row)
            aWen(i)  := pushA
            bWen(i)  := pushB
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
    // The sequencer issues the same phase starts software would, so both paths
    // share one implementation rather than duplicating the run logic.
    val startAny = startPulse || (qState === qComp)
    // SOFT_RST clears the ARRAY here; every other engine is reset at its own
    // declaration site below, and descQ is flushed via its flush port. See the
    // note at softRstPulse for why partial coverage was actively harmful.
    // busy's next value as a named wire. The PE enable replicas below are loaded
    // from THIS, not from busy, so each replica holds the same value as busy in
    // the same cycle rather than one cycle behind it.
    val busyNext = WireDefault(busy)
    when(softRstPulse) {
      busyNext := false.B; done := false.B; t := 0.U
    }.elsewhen(startAny) {
      busyNext := true.B; done := false.B; t := 0.U
    }.elsewhen(busy) {
      when(t === lastT) { busyNext := false.B; done := true.B }
      t := t + 1.U
    }
    busy := busyNext

    // ---- skewed edge feed: row i's k-th value enters at t = k+i ----
    //
    // The operand rows are read SYNCHRONOUSLY, so each lane's address is driven
    // one cycle early. Lane i needs k = t-i in cycle t, and t is a counter, so
    // the address to present now is simply the row holding k = (t+1)-i.
    //
    // On the start pulse t has not been reset yet, so the general expression
    // would use the OLD t. Row 0 is forced instead: at t=0 lane 0 is the only
    // lane with a valid k, and it needs k=0. Every later lane i first becomes
    // valid at t=i, whose address was computed normally at t=i-1.
    val aEdge = Wire(Vec(dim, SInt(8.W)))
    val bEdge = Wire(Vec(dim, SInt(8.W)))
    for (i <- 0 until dim) {
      val kNext  = Mux(startAny, 0.S, t.zext + 1.S - i.S)
      // Past the end of a run kNext can exceed maxK; the row wraps and the data
      // is discarded by kValid, so only the width needs bounding here.
      val rowNext = kRow(Mux(kNext < 0.S, 0.S, kNext).asUInt.pad(kBits))
      val aRd = aMem(i).read(rowNext)
      val bRd = bMem(i).read(bAddr(bPanelUse, rowNext))

      val kIdx   = t.zext - i.S                    // goes negative early in a run
      val kValid = (kIdx >= 0.S) && (kIdx < kLen.zext)
      val kSel   = kIdx.asUInt(kBits - 1, 0)
      aEdge(i) := Mux(kValid, aRd(kByte(kSel)).asSInt, 0.S)
      bEdge(i) := Mux(kValid, bRd(kByte(kSel)).asSInt, 0.S)
    }

    // ---- PE array ----
    val pes = Seq.tabulate(dim, dim)((_, _) => Module(new SystolicPE(8, 32)))

    // ENABLE REPLICATION, one copy per PE. This is a timing fix, and it is the
    // FPGA critical path: `pe.io.en := busy` put one flop in front of every
    // enable pin in the array - aReg(8) + bReg(8) + accReg(32) per PE, so
    // 64 x 48 = 3072 pins - and the router needed 15.582 ns to distribute it
    // (fanout 2995, zero logic in the path, 85% of a 16.667 ns period).
    //
    // Each copy is a REGISTER loaded from busyNext, not a buffer in series with
    // busy, so all copies change on the same edge and every one is bit-identical
    // to busy in every cycle. Inserting a pipeline stage instead would delay the
    // array by a cycle against the skew schedule and silently corrupt results.
    //
    // dontTouch keeps the copies from being folded back into one: they are
    // structurally identical by construction, which is exactly what CSE and
    // equivalent-register removal look for. If a synthesis run still merges
    // them, the FPGA flow needs -keep_equivalent_registers (or DONT_TOUCH on
    // these cells) or the fanout comes straight back.
    val peEn = Seq.tabulate(dim, dim)((_, _) => RegNext(busyNext, false.B))
    peEn.foreach(_.foreach(dontTouch(_)))

    for (i <- 0 until dim; j <- 0 until dim) {
      val pe = pes(i)(j)
      pe.io.en       := peEn(i)(j)
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

    // ---- requantized line: 16 results instead of 4 ----
    //
    // Round-half-up, arithmetic shift, saturate to INT8. The accumulator stays
    // 32-bit - that is the whole point of accumulating wide - and only the
    // value that LEAVES is narrowed.
    //
    // Saturation is not optional. At K=64 an INT8 dot product reaches
    // 64*127*127, which needs 21 bits, so any shift small enough to preserve
    // precision on typical data will still clip on outliers. Wrapping there
    // would turn a large positive result into a large negative one silently.
    def requant(acc: SInt): SInt = {
      val half    = Mux(outShift === 0.U, 0.U(32.W), (1.U(32.W) << (outShift - 1.U))(31, 0))
      val rounded = acc +& half.zext
      val shifted = rounded >> outShift
      Mux(shifted > 127.S, 127.S(8.W),
        Mux(shifted < -128.S, (-128).S(8.W), shifted(7, 0).asSInt))
    }

    val lineData8 = Wire(UInt(128.W))
    if (int8Capable) {
      // A result ROW becomes dim bytes, and rowsPerLine of them fill a line.
      // Requantizing at the PE output means the selection mux downstream runs
      // on bytes rather than 32-bit words - a quarter of the wires that made
      // global routing fail at dim=16, so this path needs no two-stage trick.
      val rowBytes = VecInit(pes.map(row =>
        Cat(row.map(pe => requant(pe.io.acc).asUInt).reverse)))
      val lines8 = VecInit((0 until nGroups8).map { g =>
        Cat((0 until rowsPerLine).map(r => rowBytes(g * rowsPerLine + r)).reverse)
      })
      // Registered to match the INT32 path's latency exactly, so dmaSettle
      // stays correct for both.
      lineData8 := RegNext(
        if (nGroups8 == 1) lines8(0) else lines8(fillGrp(grp8Bits - 1, 0)))
    } else {
      lineData8 := 0.U
    }

    // How many lines a tile occupies, and which line feed the FIFO takes.
    val nGroupsEff = if (int8Capable) Mux(int8Mode, nGroups8.U, nGroups.U)
                     else nGroups.U
    val lineOut    = if (int8Capable) Mux(int8Mode, lineData8, lineData)
                     else lineData

    // ---- operand load: fetch A and B panels as 128-bit lines ----
    //
    // How many 128-bit lines a lane occupies for THIS RUN - a runtime value,
    // not the compile-time linesPerLane.
    //
    // Fetching linesPerLane lines unconditionally is correct but wasteful: a
    // maxK=64 build running K_LEN=16 moved four lines per lane where one holds
    // all the data, and measured 490 cycles per tile against the maxK=16
    // build's 133 on identical work. Raising maxK must not penalise every run
    // that does not use it.
    val ldLines = if (linesPerLane == 1) 1.U else {
      // ldKLen, not kLen: the load engine sizes the tile it is FETCHING, which
      // during a prefetch is not the tile the array is computing.
      val ceilLines = (ldKLen +& 15.U)(8, 4)             // ceil(ldKLen/16)
      val clamped   = Mux(ceilLines > linesPerLane.U, linesPerLane.U, ceilLines)
      Mux(ceilLines === 0.U, 1.U, clamped)((lplBits + 1) - 1, 0)
    }

    val ldReq   = RegInit(false.B)

    // Panel / lane / chunk as three explicit counters rather than one flat
    // index. The flat form divided by a COMPILE-TIME linesPerLane to recover
    // the lane, which cannot express a per-run line count; counting the chunk
    // inside the lane needs no divide at all.
    val ldInB   = RegInit(false.B)
    val ldLane  = RegInit(0.U(laneBits.W))
    val ldChunk = RegInit(0.U(lplBits.W))

    val ldLastChunk = ldChunk === (ldLines - 1.U)
    val ldLastLane  = ldLane === (dim - 1).U
    val ldPanelEnd  = ldLastLane && ldLastChunk

    // Effective per-lane stride: 0 means lanes are packed with no gap. That is
    // a whole number of LINES per lane, which equals kLen only when kLen is a
    // multiple of 16 - the buffers are addressed in 16-byte rows.
    val effSrcStride = Mux(srcStride === 0.U, (ldLines << 4).asUInt, srcStride)

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
    // dim is a power of two, so this multiply is a shift.
    val linesPerPanel = (if (dim == 1) ldLines else (ldLines << log2Ceil(dim)).asUInt)
    // Operand fetch bursts. Safe direction: the adapter receives here, so it
    // can never stall waiting for the requester to present data.
    val burstOK       = if (enableReadBursts) effSrcStride === (ldLines << 4).asUInt
                        else false.B
    val panelBase     = Mux(ldInB, bSrcAddr, aSrcAddr)
    val ldBase       = Mux(ldInB, bSrcAddr, aSrcAddr)
    val ldNextAddr   = ldBase + (ldLane * effSrcStride) + (ldChunk << 4.U)

    // Deferred prefetch start: the cycle after tryPrefetch latched the load
    // context, so ldInB/bPanelLoad/ldKLen below sample the prefetched one.
    when(prePend) { prePend := false.B }
    preStart := prePend

    val ldStartAny = ldStart || ldStartB || (qState === qLoad) || preStart
    when(ldStartAny) {
      ldBusy  := true.B
      ldDone  := false.B
      // A queue-driven load always fetches both panels; only an explicit
      // CTRL bit4 asks for B alone.
      // Two ways to ask for a B-only load: CTRL bit4 directly, or a queued
      // descriptor with bOnly set. The queue used to force a full load
      // unconditionally, so a batch walking one tile ROW re-fetched the same A
      // panel for every tile - half the operand traffic, discarded.
      // A queue-driven load - whether the first of a batch (qLoad) or a prefetch
      // (preStart) - takes bOnly from the descriptor. preStart MUST be included:
      // without it a prefetched b-only tile would re-fetch the A panel, undoing
      // the A-reuse saving and, worse, overwriting the resident A panel that the
      // currently-computing tile is still reading.
      val fromQueue = (qState === qLoad) || preStart
      val bOnly = (ldStartB && !fromQueue) || (fromQueue && qBOnly)
      ldBOnly := bOnly
      ldInB   := bOnly
      ldLane  := 0.U
      ldChunk := 0.U
      ldReq   := true.B
    }.elsewhen(ldBusy) {
      // mem_ready belongs to whoever owns the port. With a store in flight it
      // is the store's completion, and consuming it here would advance the
      // load by a line it never received.
      // mem_rd_ready is a READ-channel signal: no write completion can reach
      // here, so there is nothing left to disambiguate.
      when(mem_rd_ready) {
        // Capture the returned line into its lane. 16 bytes at a time; the
        // buffers are registers precisely so this costs one cycle.
        for (i <- 0 until dim) {
          when(ldLane === i.U) {
            // A returned line IS a row, so this is one full-width write rather
            // than sixteen byte writes - the property the registers existed to
            // provide, kept intact by the row layout.
            val row = if (linesPerLane == 1) 0.U else ldChunk
            wData(i) := VecInit(Seq.tabulate(16)(b => mem_rline(8 * b + 7, 8 * b)))
            wMask(i) := VecInit(Seq.fill(16)(true.B))
            aWrow(i) := row
            bWrow(i) := bAddr(bPanelLoad, row)
            aWen(i)  := !ldInB
            bWen(i)  := ldInB
          }
        }
        when(ldPanelEnd && ldInB) {
          ldBusy := false.B
          ldDone := true.B
          ldReq  := false.B
        }.otherwise {
          when(ldLastChunk) {
            ldChunk := 0.U
            when(ldLastLane) { ldInB := true.B; ldLane := 0.U }
              .otherwise     { ldLane := ldLane + 1.U }
          }.otherwise {
            ldChunk := ldChunk + 1.U
          }
          // In burst mode the arbiter has already latched a request covering
          // the whole panel, so valid must DROP after the first line or the
          // arbiter would start a second transaction when this one retires.
          // It is re-asserted only at the A->B panel boundary, where a new
          // burst genuinely begins. Per-line mode re-requests every line.
          when(burstOK) {
            ldReq := !ldInB && ldPanelEnd
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
    val lineFifo = Module(new Queue(UInt(128.W), fifoLines))
    lineFifo.io.enq.valid := false.B
    lineFifo.io.enq.bits  := lineOut
    lineFifo.io.deq.ready := false.B

    // Fill pipeline: fillGrp selects, rowLineReg lands one cycle later, so a
    // valid bit has to be delayed by exactly the same cycle.
    // count < depth-1, not enq.ready: a line launched this cycle lands NEXT
    // cycle, so the slot it needs has to be reserved now. Gating on enq.ready
    // alone dropped that in-flight line whenever the queue filled in between,
    // silently skipping a result group.
    val fillFire  = dmaBusy && (fillLeft =/= 0.U) &&
                    (lineFifo.io.count < (fifoLines - 1).U)
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
    // In INT8 mode a line spans rowsPerLine result rows, so there is no row
    // boundary to stride at: the tile lands as dim*dim contiguous bytes. That
    // is a documented constraint of the mode, not an oversight - placing a
    // requantized tile inside a wider matrix needs either dim >= 16 (one row
    // per line) or a software copy.
    val destContigEff = destContig || int8Mode
    val wBurstLines = if (enableWriteBursts)
                        Mux(int8Mode, nGroups8.U,
                            Mux(destContig, nGroups.U, rowLinesU))
                      else 1.U

    val dmaSent = RegInit(0.U(9.W))     // lines accepted by memory so far

    val dmaStartAny = dmaStart || (qState === qStore)
    when(dmaStartAny) {
      dmaBusy    := true.B
      dmaDone    := false.B
      dmaAddr    := destAddr
      dmaRowBase := destAddr
      fillGrp    := 0.U
      fillLeft   := nGroupsEff
      dmaSent    := 0.U
    }.elsewhen(dmaBusy) {
      // mem_ready marks the END of a whole burst, not of a line.
      // mem_wr_ready is a WRITE-channel signal. This is the half of the double
      // fault that had no guard at all: a read completion used to land here and
      // credit dmaSent for a line the store never wrote.
      when(mem_wr_ready) {
        val nextSent = dmaSent + wBurstLines
        when(nextSent >= nGroupsEff) {
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
          when(destContigEff) {
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
    lineFifo.io.deq.ready := mem_wnext || (dmaBusy && mem_wr_ready) || !dmaBusy

    // ---- SOFT_RST: clear EVERY engine, not just the array ----
    //
    // Placed here, after all engine logic, so Chisel's last-connect semantics
    // make it authoritative over whatever each engine's own state machine
    // decided this cycle. Splitting it across the engines would reintroduce
    // exactly the bug below the moment someone adds a new one.
    //
    // It used to clear only busy/done/t. Everything else kept running, and the
    // consequence was not a stuck accelerator but a FALSE SUCCESS on the next
    // batch: the driver's timeout writes SOFT_RST
    // (gemm_rv32_5stage.c), the abandoned batch keeps DMA-writing into a dest
    // buffer the caller is now free to reuse, the next gemm_submit cannot start
    // because qStartPulse requires !qBusy, and then a late qDone from the
    // abandoned batch satisfies the NEW batch's semaphore. The caller reads a
    // buffer nothing wrote and is told it succeeded. A recovery path that does
    // that is worse than no recovery path.
    //
    // descQ is flushed through its flush port rather than cleared here, because
    // a Queue's pointers are internal.
    descQ.io.flush.get := softRstPulse

    when(softRstPulse) {
      qRun     := false.B
      qBusy    := false.B
      qDone    := false.B
      qState   := qIdle
      ldBusy   := false.B
      ldDone   := false.B
      dmaBusy  := false.B
      dmaDone  := false.B
      fillLeft := 0.U
      // Drop a stale overflow report too: it describes the batch being
      // abandoned, not the next one.
      qOverflow := false.B
      // And the prefetch bookkeeping, or the next batch would enter qPreW
      // waiting on a load belonging to the batch that was just abandoned.
      preBusy := false.B
      preDone := false.B
      // prePend is the one that actually matters here. It self-clears one cycle
      // later, and that cycle asserts preStart - so a SOFT_RST landing while it
      // is set launches a load for the descriptor that was just abandoned, using
      // register state belonging to a batch that no longer exists.
      prePend := false.B
      // dmaSent is belt-and-braces, NOT a bug fix, and the distinction is worth
      // recording because it was briefly claimed as one. It is already cleared
      // unconditionally by the dmaStartAny block below, so a value surviving this
      // reset is wiped before the next store can read it. It is cleared here only
      // so that the reset leaves no engine showing the abandoned batch's state.
      dmaSent := 0.U
      // The rest are re-initialised at their engine's next start, so they cannot
      // corrupt anything. They are cleared because a waveform taken after a
      // SOFT_RST is far harder to read when half the engine still shows the
      // abandoned batch's state.
      ldReq   := false.B
      ldInB   := false.B
      ldLane  := 0.U
      ldChunk := 0.U
      fillGrp := 0.U
    }

    // Both engines share one line port. Mutual exclusion is enforced at the
    // START gates above - each requires the other engine idle - so at most one
    // of dmaDrive / ldBusy can be true here.
    // Hold the request until the burst retires; the arbiter latches once and
    // stays in SVC_A, and dmaBusy drops on completion so no second transaction
    // is started. Gated on the FIFO having a line so the burst never begins
    // ahead of its data.
    val dmaDrive = dmaBusy && lineFifo.io.deq.valid
    // An operand load may now be in flight at the same time as a result store,
    // because the queue releases the array as soon as the accumulators drain.
    // They cannot share the port, so the DMA holds it for the WHOLE
    // transaction - gated on dmaBusy, not dmaDrive, since a momentarily empty
    // FIFO must not hand the port to the load mid-burst.
    // !dmaBusy is GONE - this is the term that actually cost the overlap. The
    // load may now present its request while a store is in flight; the arbiter
    // and the AXI engine carry them on independent channels.
    val ldDrive  = ldBusy && ldReq
    // Each channel's address, length and data are now selected by the engine
    // that owns them, rather than by dmaBusy steering one shared set. There is
    // deliberately no `Mux(valid, ..., 0.U)` here: an idle channel's fields are
    // simply not sampled, because its valid is low.
    mem_rd_req_valid := ldDrive
    mem_wr_req_valid := dmaDrive
    // A burst addresses the PANEL BASE and covers linesPerPanel lines; the
    // per-line path addresses each line individually with a length of 1.
    mem_wr_req_addr := dmaAddr
    mem_rd_req_addr := Mux(burstOK, panelBase, ldNextAddr)
    mem_wline     := lineFifo.io.deq.bits
    mem_wr_req_lines := wBurstLines
    mem_rd_req_lines := Mux(burstOK, linesPerPanel, 1.U)


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
      is(1.U)  { rdata := Cat(0.U(23.W), qOverflow,
                              qDone, qBusy, ldDone, ldBusy,
                              dmaDone, dmaBusy, done, busy) }
      is(10.U) { rdata := destAddr }
      is(11.U) { rdata := destStride }
      is(12.U) { rdata := aSrcAddr }
      is(13.U) { rdata := bSrcAddr }
      is(14.U) { rdata := srcStride }
      is(22.U) { rdata := Cat(0.U(23.W), int8Mode, 0.U(3.W), outShift) }
      is(15.U) { rdata := bPanelUse }
      is(16.U) { rdata := bPanelLoad }
      // Free slots left, so software can push without overflowing.
      is(21.U) { rdata := Cat(0.U(24.W), (descDepth.U - descQ.io.count)) }
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
