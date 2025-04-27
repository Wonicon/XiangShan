package xiangshan.backend.rob

import chisel3._
import chisel3.util._
import chisel3.experimental.BundleLiterals._
import org.chipsalliance.cde.config.Parameters
import utility.HasCircularQueuePtrHelper
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

  // Redirect
  val redirect = Input(Bool())
  val walkPtr = Input(new RobPtr)

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
 * 0. 不希望 amuCtrl 存进 ROB，单独一个跟 ROB 一样大的 buffer。先用 reg vec 简易实现？
 * 1. AMUCtrl 进 ROB，同步进 Buffer
 * 2. Buffer 满了反压 ROB 入队
 * 3. 同步 ROB 冲刷
 * 4. ROB commit 后，开始与 AMU 握手
 * 5. 保证握手顺序？
 */
class AmuCtrlBuffer()(implicit override val p: Parameters, params: BackendParams) extends XSModule
  with HasXSParameter with HasCircularQueuePtrHelper {

  val io = IO(new AmuCtrlBufferIO)

  val s_idle :: s_walk :: Nil = Enum(2)
  val state = RegInit(s_idle)
  val state_next = Wire(chiselTypeOf(state))

  // Add walk-related signals
  val walkPtrVec = Reg(Vec(CommitWidth, new RobPtr))
  val walkPtrTrue = Reg(new RobPtr)
  val lastWalkPtr = Reg(new RobPtr)
  val donotNeedWalk = RegInit(VecInit(Seq.fill(CommitWidth)(false.B)))
  val walkFinished = walkPtrTrue > lastWalkPtr

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
  val amuCtrl_wb = amuCtrlWb
  for (i <- 0 until RobSize) {
    val amuCtrlCanWbSeq = amuCtrl_wb.map(wb => wb.valid && wb.bits.robIdx.value === i.U)
    val amuCtrlRes = amuCtrlCanWbSeq.zip(amuCtrl_wb).map { case (canWb, wb) => Mux(canWb, wb.bits.amuCtrl.get.asUInt, 0.U) }.fold(0.U)(_ | _)
    when (amuCtrlEntries(i).valid && amuCtrlRes.orR) {
      amuCtrlEntries(i).writebacked := true.B
      amuCtrlEntries(i).amuCtrl := amuCtrlRes.asTypeOf(new AmuCtrlIO)
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

  // 假设 amu bus 的 ready 总能接收全部宽度的请求
  // 如果当前 commit window 里所有 amu req 都一次性握手，或者没有 amu req，则可以批量退队
  // (这里都是已经在 ROB 中标记已经 commit 的 entry，没有 AMU 需求可以直接退)
  // TODO 但是要避免有 amu 需求的 entry 还没有 writeback 或 commit，这时候不能批量退队当前 commit 窗口。

  // Add commit and dequeue logic
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

  // Walk
  val currWalkPtr = RegInit(0.U.asTypeOf(new RobPtr))
  when (state === s_idle && io.redirect) {
    currWalkPtr := io.walkPtr
    state := s_walk
  }

  when (state === s_walk) {
    for (i <- 0 until CommitWidth) {
      amuCtrlEntries(currWalkPtr.value + i.U).valid := true.B
      amuCtrlEntries(currWalkPtr.value + i.U).needAMU := true.B
    }
    currWalkPtr := currWalkPtr + CommitWidth.U
  }

  when (currWalkPtr === deqPtr) {
    state := s_idle
  }

  // Add walk control logic
  val shouldWalkVec = Wire(Vec(CommitWidth, Bool()))
  val walkingPtrVec = RegNext(walkPtrVec)

  when(io.redirect) {
    shouldWalkVec := 0.U.asTypeOf(shouldWalkVec)
  }.elsewhen(RegNext(io.redirect)) {
    shouldWalkVec := 0.U.asTypeOf(shouldWalkVec)
  }.elsewhen(state === s_walk) {
    shouldWalkVec := VecInit(walkingPtrVec.map(_ <= lastWalkPtr).zip(donotNeedWalk).map(x => x._1 && !x._2))
  }.otherwise {
    shouldWalkVec := 0.U.asTypeOf(shouldWalkVec)
  }

  // Update walk pointers
  val deqPtrReadBank = deqPtr.lineHeadPtr
  val deqPtrVecForWalk = VecInit((0 until CommitWidth).map(i => deqPtrReadBank + i.U))
  val walkPtrVec_next: Vec[RobPtr] = Mux(io.redirect,
    deqPtrVecForWalk,
    Mux((state === s_walk) && !walkFinished, VecInit(walkPtrVec.map(_ + CommitWidth.U)), walkPtrVec)
  )
  val walkPtrTrue_next: RobPtr = Mux(io.redirect,
    deqPtr,
    Mux((state === s_walk) && !walkFinished, walkPtrVec_next.head, walkPtrTrue)
  )

  walkPtrVec := walkPtrVec_next
  walkPtrTrue := walkPtrTrue_next

  // Update lastWalkPtr
  when(io.redirect) {
    lastWalkPtr := io.walkPtr
  }

  // Update donotNeedWalk
  val walkPtrLowBits = Reg(UInt(log2Up(CommitWidth).W))
  when(io.redirect) {
    walkPtrLowBits := io.walkPtr.value(log2Up(CommitWidth)-1, 0)
  }
  when(io.redirect) {
    donotNeedWalk := Fill(donotNeedWalk.length, true.B).asTypeOf(donotNeedWalk)
  }.elsewhen(RegNext(io.redirect)) {
    donotNeedWalk := (0 until CommitWidth).map(i => (i.U < walkPtrLowBits))
  }.otherwise {
    donotNeedWalk := 0.U.asTypeOf(donotNeedWalk)
  }

  // Update state
  state_next := Mux(
    io.redirect || RegNext(io.redirect), s_walk,
    Mux(
      state === s_walk && walkFinished, s_idle,
      state
    )
  )
  state := state_next

  // Walk amuCtrlEntries
  when(state === s_walk) {
    for (i <- 0 until CommitWidth) {
      when(shouldWalkVec(i)) {
        amuCtrlEntries(walkingPtrVec(i).value).valid := false.B
        amuCtrlEntries(walkingPtrVec(i).value).needAMU := false.B
        amuCtrlEntries(walkingPtrVec(i).value).writebacked := false.B
        amuCtrlEntries(walkingPtrVec(i).value).committed := false.B
      }
    }
  }
}