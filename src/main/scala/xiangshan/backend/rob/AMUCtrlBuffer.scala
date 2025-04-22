package xiangshan.backend.rob

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import utility.HasCircularQueuePtrHelper
import xiangshan.XSModule
import xiangshan.backend.fu.matrix.Bundles._

/**
 * 0. 不希望 amuCtrl 存进 ROB，单独一个跟 ROB 一样大的 buffer。先用 reg vec 简易实现？
 * 1. AMUCtrl 进 ROB，同步进 Buffer
 * 2. Buffer 满了反压 ROB 入队
 * 3. 同步 ROB 冲刷
 * 4. ROB commit 后，开始与 AMU 握手
 * 5. 保证握手顺序？
 *
 * @param robSize
 * @param p
 */
class AmuCtrlBuffer(robSize: Int)(implicit p: Parameters) extends XSModule with HasCircularQueuePtrHelper {
  val RenameWidth = 6
  val CommitWidth = 6

  val io = IO(new Bundle {
    // rob enq
    val enq = Flipped(DecoupledIO(Vec(RenameWidth, new Bundle {
      val robIndex = UInt(log2Ceil(robSize).W)
    })))
    // WB
    val wb = Flipped(DecoupledIO(Vec(RenameWidth, new Bundle {
      val amuCtrl = new AmuCtrlIO
      val robIndex = UInt(log2Ceil(robSize).W)
    })))
    // Redirect
    val walk = Flipped(DecoupledIO(Vec(CommitWidth, new Bundle {
      val robIndex = UInt(log2Ceil(robSize).W)
    })))
    // Commit
    val commit = Flipped(DecoupledIO(Vec(CommitWidth, new Bundle {
      val robIndex = UInt(log2Ceil(robSize).W)
    })))
    // To Amu
    val toAMU = DecoupledIO(Vec(CommitWidth, new AmuCtrlIO))
  })

  // 状态定义
  val s_idle :: s_walk :: Nil = Enum(2)
  val state = RegInit(s_idle)
  val state_next = Wire(chiselTypeOf(state))

  // 指针定义
  val enqPtrVec = RegInit(VecInit.tabulate(RenameWidth)(_.U.asTypeOf(new RobPtr)))
  val deqPtrVec = RegInit(VecInit.tabulate(CommitWidth)(_.U.asTypeOf(new RobPtr)))
  val walkPtrVec = RegInit(VecInit.tabulate(CommitWidth)(_.U.asTypeOf(new RobPtr)))

  // 存储定义
  val amuCtrlValids = RegInit(VecInit(Seq.fill(robSize){ false.B }))
  val committed = RegInit(VecInit(Seq.fill(robSize){ false.B }))
  val amuCtrlEntries = Reg(Vec(robSize, new AmuCtrlIO))

  // 计算实际需要分配的条目数
  val needAllocVec = io.enq.bits.map(req => req.robIndex =/= 0.U)
  val enqCount = PopCount(needAllocVec)

  // 计算分配指针
  val allocatePtrVec = VecInit((0 until RenameWidth).map(i => 
    enqPtrVec(PopCount(needAllocVec.take(i)))))

  // 入队逻辑
  val canEnqueue = !isFull(enqPtrVec(0), deqPtrVec(0))
  io.enq.ready := canEnqueue

  for (i <- 0 until RenameWidth) {
    when(io.enq.fire) {
      amuCtrlValids(io.enq.bits(i).robIndex) := true.B
      committed(io.enq.bits(i).robIndex) := false.B
    }
  }

  // 写回逻辑
  for (i <- 0 until RenameWidth) {
    when(io.wb.fire) {
      amuCtrlEntries(io.wb.bits(i).robIndex) := io.wb.bits(i).amuCtrl
    }
  }

  // 提交逻辑
  for (i <- 0 until CommitWidth) {
    when(io.commit.fire) {
      committed(io.commit.bits(i).robIndex) := true.B
    }
  }

  // Walk 逻辑
  for (i <- 0 until CommitWidth) {
    when(io.walk.fire) {
      amuCtrlValids(io.walk.bits(i).robIndex) := false.B
      committed(io.walk.bits(i).robIndex) := false.B
    }
  }

  // 状态转换
  when(io.walk.valid) {
    state_next := s_walk
  }.otherwise {
    state_next := s_idle
  }
  state := state_next

  // 出队逻辑
  for (i <- 0 until CommitWidth) {
    val canDequeue = amuCtrlValids(deqPtrVec(i).value) && committed(deqPtrVec(i).value)
    io.toAMU.bits(i) := amuCtrlEntries(deqPtrVec(i).value)
  }

  io.toAMU.valid := canEnqueue && state === s_idle

  when(io.toAMU.fire) {
    for (i <- 0 until CommitWidth) {
      amuCtrlValids(deqPtrVec(i).value) := false.B
      committed(deqPtrVec(i).value) := false.B
    }
  }

  // 指针更新
  for (i <- 0 until RenameWidth) {
    when(io.walk.valid) {
      enqPtrVec(i) := io.walk.bits(0).robIndex + i.U
    }.otherwise {
      enqPtrVec(i) := enqPtrVec(i) + enqCount
    }
  }

  val commitCnt = PopCount(io.commit.bits.map(_.robIndex =/= 0.U))
  for (i <- 0 until CommitWidth) {
    deqPtrVec(i) := deqPtrVec(i) + commitCnt
  }

  val walkCnt = PopCount(io.walk.bits.map(_.robIndex =/= 0.U))
  when(state === s_walk) {
    for (i <- 0 until CommitWidth) {
      walkPtrVec(i) := walkPtrVec(i) + walkCnt
    }
  }
}