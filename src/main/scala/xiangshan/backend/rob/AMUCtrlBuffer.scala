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
  val io = IO(new Bundle {
    // rob enq
    val enq = Flipped(DecoupledIO(new Bundle {
      val robIndex = UInt(log2Ceil(robSize).W)
    }))
    // WB
    val wb = Flipped(DecoupledIO(new Bundle {
      val amuCtrl = new AmuCtrlIO
      val robIndex = UInt(log2Ceil(robSize).W)
    }))
    // Redirect
    val walk = Flipped(DecoupledIO(new Bundle {
      val robIndex = UInt(log2Ceil(robSize).W)
    }))
    // Commit
    val commit = Flipped(DecoupledIO(new Bundle {
      val robIndex = UInt(log2Ceil(robSize).W)
    }))
    // To Amu
    val toAMU = DecoupledIO(new AmuCtrlIO)
  })

  val amuCtrlValids = RegInit(VecInit(Seq.fill(robSize){ false.B }))
  val committed = RegInit(VecInit(Seq.fill(robSize){ false.B }))
  val robIndexBuffer = Reg(Vec(robSize, UInt(log2Ceil(robSize).W)))
  val amuCtrlEntries = Reg(Vec(robSize, new AmuCtrlIO))
  val wbV_dest = MuxLookup(io.wb.bits.robIndex, 0.U.asTypeOf(false.B))(robIndexBuffer.zip(amuCtrlValids))
  val wbE_dest = MuxLookup(io.wb.bits.robIndex, 0.U.asTypeOf(io.wb.bits.amuCtrl))(robIndexBuffer.zip(amuCtrlEntries))
  when (io.wb.valid) {
    wbV_dest := true.B
    wbE_dest := io.wb.bits.amuCtrl
  }

  val commit_dest = MuxLookup(io.wb.bits.robIndex, 0.U.asTypeOf(io.wb.bits.amuCtrl))(robIndexBuffer.zip(amuCtrlEntries))
  when (io.wb.valid) {

  }

  val walk_ptr = RegInit(0.U)
  val valid = amuCtrlValids.zipWithIndex.map{ case (v, i) => v && (i.U === walk_ptr) }.reduce(_ || _)
  io.toAMU.valid := valid
  io.toAMU.bits := amuCtrlEntries(walk_ptr)
  when (io.toAMU.fire || !amuCtrlValids(walk_ptr)) {
    walk_ptr := walk_ptr + 1.U
  }
}