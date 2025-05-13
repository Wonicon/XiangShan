package xiangshan.backend.rob

import chisel3._
import chisel3.util._
import chisel3.experimental.BundleLiterals._
import org.chipsalliance.cde.config.Parameters
import utility._
import xiangshan._
import xiangshan.backend.BackendParams
import xiangshan.backend.fu.matrix.Bundles._

class AmuCtrlBufferIO()(implicit val p: Parameters, params: BackendParams) extends Bundle with HasXSParameter {
  // rob enq
  val enqReqValids = Input(Vec(RenameWidth, Bool()))
  val enqAllocPtrVec = Input(Vec(RenameWidth, new RobPtr))
  val enqNeedAMU = Input(Vec(RenameWidth, Bool()))
  val outCanEnqueue = Output(Bool())

  // rob wb
  val wb = Flipped(params.genWrite2CtrlBundles)

  // Commit
  val deqCommitPtrVec = Input(Vec(CommitWidth, new RobPtr))
  val deqCommitValid = Input(Vec(CommitWidth, Bool()))

  // Redirect (no need to walk)
  val redirect = Input(Valid(new Redirect))

  // To Amu
  val toAMU = Vec(CommitWidth, DecoupledIO(new AmuCtrlIO))
}


class AmuCtrlEntry(implicit p: Parameters) extends XSBundle {
  val valid = Bool()
  val needAMU = Bool()
  val writebacked = Bool()
  val committed = Bool()
  val amuCtrl = new AmuCtrlIO
}


/**
 * AMU Control Buffer
 *
 * Purpose:
 * A dedicated buffer parallel to ROB for storing AMU control signals, avoiding storing them in ROB directly.
 * The buffer size matches ROB size.
 *
 * Operation:
 * 1. Instructions enter this buffer synchronously when entering ROB
 * 2. When buffer is full, it will backpressure ROB enqueue
 * 3. Synchronizes with ROB writeback and flush, no need for walking to rebuild context
 * 4. After ROB commit, starts handshake with AMU and dequeues after handshake fires
 */
class AmuCtrlBufferTrace(implicit val p: Parameters) extends Bundle with HasXSParameter {
  val cycle = UInt(64.W)
  // IO ports
  val io_enqReqValids = Vec(RenameWidth, Bool())
  val io_enqAllocPtrVec = Vec(RenameWidth, UInt(log2Up(RobSize).W))
  val io_enqNeedAMU = Vec(RenameWidth, Bool())
  val io_outCanEnqueue = Bool()
  val io_deqCommitPtrVec = Vec(CommitWidth, UInt(log2Up(RobSize).W))
  val io_deqCommitValid = Vec(CommitWidth, Bool())
  val io_redirect = UInt(log2Up(RobSize).W)
  val io_toAMU_valid = Vec(CommitWidth, Bool())
  val io_toAMU_ready = Vec(CommitWidth, Bool())

  // State and control signals
  val ctrl_deqPtr = UInt(log2Up(RobSize).W)

  // Buffer status
  val buf_numValidEntries = UInt(log2Up(RobSize + 1).W)
  val buf_numNewEntries = UInt(log2Up(RenameWidth + 1).W)
  val buf_commitValidThisLine = Vec(CommitWidth, Bool())
  val buf_hasCommitted = Vec(CommitWidth, Bool())
  val buf_allCommitted = Bool()

  // Entry status
  val entry_valid = Vec(RobSize, Bool())
  val entry_needAMU = Vec(RobSize, Bool())
  val entry_writebacked = Vec(RobSize, Bool())
  val entry_committed = Vec(RobSize, Bool())
}

class AmuCtrlBuffer()(implicit override val p: Parameters, params: BackendParams) extends XSModule
  with HasXSParameter with HasCircularQueuePtrHelper with WithChiselDBTracing {

  val io = IO(new AmuCtrlBufferIO)

  val amuCtrlEntries = RegInit(VecInit.fill(RobSize)((new AmuCtrlEntry).Lit(_.valid -> false.B)))

  // Enqueue
  // DynInst does not carry amuCtrl info,
  // so we only need to mark valid for entries who need amuCtrl.
  for (i <- 0 until RobSize) {
    val indexMatch = io.enqAllocPtrVec.map(_.value === i.U)
    val enqOH = VecInit(io.enqReqValids.zip(indexMatch).map(x => x._1 && x._2))
    val needOH = VecInit(io.enqNeedAMU.zip(indexMatch).map(x => x._1 && x._2))
    amuCtrlEntries(i).valid := enqOH.asUInt.orR
    amuCtrlEntries(i).needAMU := needOH.asUInt.orR
  }

  // Calculate number of valid entries and new entries to be enqueued
  val numValidEntries = PopCount(amuCtrlEntries.map(_.valid))
  val numNewEntries = PopCount(io.enqReqValids.zip(io.enqNeedAMU).map { case (valid, need) => valid && need })

  // Check if there's enough space in the queue
  io.outCanEnqueue := numValidEntries + numNewEntries <= (RobSize - RenameWidth).U

  // Writeback
  val amuCtrlWb = io.wb.filter(_.bits.amuCtrl.nonEmpty).toSeq
  for (i <- 0 until RobSize) {
    val amu_data = amuCtrlWb.map{ wb =>
      val valid_match = wb.valid && wb.bits.robIdx.value === i.U
      Mux(valid_match, wb.bits.amuCtrl.get.asUInt, 0.U)
    }.reduce(_ | _)

    val entry = amuCtrlEntries(i)
    when (entry.valid && amu_data.orR) {
      entry.writebacked := true.B
      entry.amuCtrl := amu_data.asTypeOf(new AmuCtrlIO)
    }
  }

  // Commit
  for (i <- 0 until RobSize) {
    val deqSel = io.deqCommitPtrVec.map(_.value === i.U)
    val deqValid = io.deqCommitValid.zip(deqSel).map(x => x._1 && x._2)
    val commitCond = deqValid.reduce(_ || _)
    val ent = amuCtrlEntries(i)
    when (commitCond) {
      amuCtrlEntries(i).committed := true.B
    }
  }

  // Redirect
  for (i <- 0 until RobSize) {
    val redirectCond = io.redirect.valid
    val robPtr = Wire(new RobPtr(RobSize))
    robPtr := io.deqCommitPtrVec(0) + i.U
    when (robPtr.needFlush(io.redirect)) {
      amuCtrlEntries(i).valid := false.B
    }
  }

  // To AMU
  val deqPtr = RegInit(0.U.asTypeOf(new RobPtr))
  val entriesCommit = (0 until CommitWidth).map(i => amuCtrlEntries(deqPtr.value + i.U))
  val commitCandicates = entriesCommit.map(e => e.valid && e.committed)
  val amuReqValids = entriesCommit.map(e => e.valid && e.needAMU && e.writebacked && e.committed)
  val amuReqValidCount = PopCount(VecInit(amuReqValids).asUInt)
  io.toAMU.zip(amuReqValids).foreach { case (amuCtrl, valid) =>
    amuCtrl.valid := valid
  }
  io.toAMU.zipWithIndex.foreach { case (amuCtrl, i) =>
    amuCtrl.bits := amuCtrlEntries(deqPtr.value + i.U).amuCtrl
  }


  val commitValidThisLine = VecInit(Seq.fill(CommitWidth)(false.B))
  val hasCommitted = RegInit(VecInit(Seq.fill(CommitWidth)(false.B)))
  val allCommitted = Wire(Bool())

  val toAMU_anyFire = io.toAMU.map(_.fire).reduce(_ || _)

  when(allCommitted) {
    hasCommitted := 0.U.asTypeOf(hasCommitted)
  }.elsewhen(toAMU_anyFire) {
    for (i <- 0 until CommitWidth) {
      hasCommitted(i) := commitCandicates(i) || hasCommitted(i)
    }
  }
  allCommitted := toAMU_anyFire && commitCandicates.last

  // Update deqPtr and invalidate entries
  when(toAMU_anyFire) {
    val deqSteps = PopCount(VecInit(commitCandicates).asUInt)
    deqPtr := deqPtr + deqSteps
    for (i <- 0 until CommitWidth) {
      when(commitCandicates(i)) {
        amuCtrlEntries(deqPtr.value + i.U).valid := false.B
        amuCtrlEntries(deqPtr.value + i.U).needAMU := false.B
        amuCtrlEntries(deqPtr.value + i.U).writebacked := false.B
        amuCtrlEntries(deqPtr.value + i.U).committed := false.B
      }
    }
  }

  this.makeChiselDBLog()
}


trait WithChiselDBTracing {
  this: AmuCtrlBuffer =>

  // ChiselDB tracing
  val trace_tbl = ChiselDB.createTable("AmuCtrlBufferTrace", new AmuCtrlBufferTrace)

  def makeChiselDBLog(): Unit = {
    val trace = Wire(new AmuCtrlBufferTrace)
    trace.cycle := GTimer()
    // IO ports
    trace.io_enqReqValids := io.enqReqValids
    trace.io_enqAllocPtrVec := VecInit(io.enqAllocPtrVec.map(_.value))
    trace.io_enqNeedAMU := io.enqNeedAMU
    trace.io_outCanEnqueue := io.outCanEnqueue
    trace.io_deqCommitPtrVec := VecInit(io.deqCommitPtrVec.map(_.value))
    trace.io_deqCommitValid := io.deqCommitValid
    trace.io_redirect := io.redirect.bits.robIdx.value
    trace.io_toAMU_valid := VecInit(io.toAMU.map(_.valid))
    trace.io_toAMU_ready := VecInit(io.toAMU.map(_.ready))
    // State and control signals
    trace.ctrl_deqPtr := deqPtr.value
    // Buffer status
    trace.buf_numValidEntries := numValidEntries
    trace.buf_numNewEntries := numNewEntries
    trace.buf_commitValidThisLine := commitValidThisLine
    trace.buf_hasCommitted := hasCommitted
    trace.buf_allCommitted := allCommitted
    // Entry status
    trace.entry_valid := VecInit(amuCtrlEntries.map(_.valid))
    trace.entry_needAMU := VecInit(amuCtrlEntries.map(_.needAMU))
    trace.entry_writebacked := VecInit(amuCtrlEntries.map(_.writebacked))
    trace.entry_committed := VecInit(amuCtrlEntries.map(_.committed))
    // Write to ChiselDB
    trace_tbl.log(trace, true.B, "amu ctrl buffer trace", clock, reset)
  }
}