/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

package xiangshan.backend.rename.freelist

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import xiangshan.backend.rename._
import utils._
import utility._


class StdFreeList(freeListSize: Int, numLogicRegs: Int, regType: RegType, realNumLogicRegs: Int = 32)(implicit p: Parameters) extends BaseFreeList(freeListSize, realNumLogicRegs) with HasPerfEvents {

  val freeList = RegInit(VecInit(Seq.tabulate(freeListSize)( i => (i + numLogicRegs).U(PhyRegIdxWidth.W) )))
  val lastTailPtr = RegInit(FreeListPtr(true, 0)) // tailPtr in the last cycle (need to add freeReqReg)
  val tailPtr = Wire(new FreeListPtr) // this is the real tailPtr
  val tailPtrOHReg = RegInit(0.U(freeListSize.W))

  //
  // free committed instructions' `old_pdest` reg
  //
  val freeReqReg = io.freeReq
  for (i <- 0 until RabCommitWidth) {
    val offset = if (i == 0) 0.U else PopCount(freeReqReg.take(i))
    val enqPtr = lastTailPtr + offset

    // Why RegNext (from RAT and Rename): for better timing
    // Why we can RegNext: these free registers won't be used in the next cycle,
    // since we set canAllocate only when the current free regs > RenameWidth.
    when (freeReqReg(i)) {
      freeList(enqPtr.value) := io.freePhyReg(i)
    }
    XSDebug(io.freeReq(i), p"req#$i free physical reg: ${io.freePhyReg(i)}\n")
  }

  tailPtr := lastTailPtr + PopCount(freeReqReg)
  lastTailPtr := tailPtr

  //
  // allocate new physical registers for instructions at rename stage
  //
  val freeRegCnt = Wire(UInt()) // number of free registers in free list
  io.canAllocate := GatedValidRegNext(freeRegCnt >= RenameWidth.U) // use RegNext for better timing
  XSDebug(p"freeRegCnt: $freeRegCnt\n")

  val phyRegCandidates = VecInit(headPtrOHVec.map(sel => Mux1H(sel, freeList)))

  for(i <- 0 until RenameWidth) {
    io.allocatePhyReg(i) := phyRegCandidates(PopCount(io.allocateReq.take(i)))
    XSDebug(p"req:${io.allocateReq(i)} canAllocate:${io.canAllocate} pdest:${io.allocatePhyReg(i)}\n")
  }
  val doCommit = io.commit.isCommit
  val archAlloc = io.commit.commitValid zip io.commit.info map { case (valid, info) =>
    valid && (regType match {
      case Reg_F => info.fpWen
      case Reg_V => info.vecWen
      case Reg_V0 => info.v0Wen
      case Reg_Vl => info.vlWen
      case Reg_Mx => info.mxWen
    })
  }
  val numArchAllocate = PopCount(archAlloc)
  val archHeadPtrNew  = archHeadPtr + numArchAllocate
  val archHeadPtrNext = Mux(doCommit, archHeadPtrNew, archHeadPtr)
  archHeadPtr := archHeadPtrNext

  val isWalkAlloc = io.walk && io.doAllocate
  val isNormalAlloc = io.canAllocate && io.doAllocate
  val isAllocate = isWalkAlloc || isNormalAlloc
  val numAllocate = Mux(io.walk, PopCount(io.walkReq), PopCount(io.allocateReq))
  val headPtrAllocate = Mux(lastCycleRedirect, redirectedHeadPtr, headPtr + numAllocate)
  val headPtrOHAllocate = Mux(lastCycleRedirect, redirectedHeadPtrOH, headPtrOHVec(numAllocate))
  val headPtrNext = Mux(isAllocate, headPtrAllocate, headPtr)
  freeRegCnt := Mux(isWalkAlloc && !lastCycleRedirect, distanceBetween(tailPtr, headPtr) - PopCount(io.walkReq),
                Mux(isNormalAlloc,                     distanceBetween(tailPtr, headPtr) - PopCount(io.allocateReq),
                                                       distanceBetween(tailPtr, headPtr)))

  // priority: (1) exception and flushPipe; (2) walking; (3) mis-prediction; (4) normal dequeue
  val realDoAllocate = !io.redirect && isAllocate
  headPtr := Mux(realDoAllocate, headPtrAllocate, headPtr)
  headPtrOH := Mux(realDoAllocate, headPtrOHAllocate, headPtrOH)

  XSDebug(p"head:$headPtr tail:$tailPtr\n")

  XSError(!isFull(tailPtr, archHeadPtr), s"${regType}ArchFreeList should always be full\n")

  val enableFreeListCheck = false
  if (enableFreeListCheck) {
    for (i <- 0 until freeListSize) {
      for (j <- i+1 until freeListSize) {
        XSError(freeList(i) === freeList(j), s"Found same entry in free list! (i=$i j=$j)\n")
      }
    }
  }

  XSPerfAccumulate("utilization", PopCount(io.allocateReq))
  XSPerfAccumulate("allocation_blocked_cycle", !io.canAllocate)
  XSPerfAccumulate("can_alloc_wrong", !io.canAllocate && freeRegCnt >= RenameWidth.U)

  val freeRegCntReg = RegNext(freeRegCnt)
  val perfEvents = Seq(
    ("std_freelist_1_4_valid", freeRegCntReg <  (freeListSize / 4).U                                            ),
    ("std_freelist_2_4_valid", freeRegCntReg >= (freeListSize / 4).U && freeRegCntReg < (freeListSize / 2).U    ),
    ("std_freelist_3_4_valid", freeRegCntReg >= (freeListSize / 2).U && freeRegCntReg < (freeListSize * 3 / 4).U),
    ("std_freelist_4_4_valid", freeRegCntReg >= (freeListSize * 3 / 4).U                                        )
  )

  QueuePerf(size = freeListSize, utilization = freeRegCntReg, full = freeRegCntReg === 0.U)

  generatePerfEvent()
}
