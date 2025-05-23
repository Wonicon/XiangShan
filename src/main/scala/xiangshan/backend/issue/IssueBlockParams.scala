package xiangshan.backend.issue

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import utils.SeqUtils
import xiangshan.backend.BackendParams
import xiangshan.backend.Bundles._
import xiangshan.backend.datapath.DataConfig.DataConfig
import xiangshan.backend.datapath.WbConfig._
import xiangshan.backend.datapath.{WakeUpConfig, WakeUpSource}
import xiangshan.backend.exu.{ExeUnit, ExeUnitParams}
import xiangshan.backend.fu.{FuConfig, FuType}
import xiangshan.SelImm
import xiangshan.backend.issue.EntryBundles.EntryDeqRespBundle

case class IssueBlockParams(
  // top down
  private val exuParams: Seq[ExeUnitParams],
  val numEntries       : Int,
  numEnq               : Int,
  numComp              : Int,
  numDeqOutside        : Int = 0,
  numWakeupFromOthers  : Int = 0,
  XLEN                 : Int = 64,
  VLEN                 : Int = 128,
  // calculate in scheduler
  var idxInSchBlk      : Int = 0,
)(
  implicit
  val schdType: SchedulerType,
) {
  var backendParam: BackendParams = null

  val exuBlockParams: Seq[ExeUnitParams] = exuParams.filterNot(_.fakeUnit)

  val allExuParams = exuParams

  def updateIdx(idx: Int): Unit = {
    this.idxInSchBlk = idx
  }

  def inMemSchd: Boolean = schdType == MemScheduler()

  def inIntSchd: Boolean = schdType == IntScheduler()

  def inVfSchd: Boolean = schdType == VfScheduler()

  def isMfSchd: Boolean = schdType == MfScheduler()

  def isMemAddrIQ: Boolean = inMemSchd && (LduCnt > 0 || StaCnt > 0 || VlduCnt > 0 || VstuCnt > 0 || HyuCnt > 0 || MlsCnt > 0)

  def isLdAddrIQ: Boolean = inMemSchd && LduCnt > 0

  def isStAddrIQ: Boolean = inMemSchd && StaCnt > 0

  def isHyAddrIQ: Boolean = inMemSchd && HyuCnt > 0

  def isVecLduIQ: Boolean = inMemSchd && (VlduCnt + VseglduCnt) > 0

  def isVecStuIQ: Boolean = inMemSchd && (VstuCnt + VsegstuCnt) > 0

  def isMatrixMemIQ: Boolean = inMemSchd && MlsCnt > 0

  def isVecMemIQ: Boolean = isVecLduIQ || isVecStuIQ

  def needFeedBackSqIdx: Boolean = isVecMemIQ || isStAddrIQ

  def needFeedBackLqIdx: Boolean = isVecMemIQ || isLdAddrIQ

  def needFeedBackMlsqIdx: Boolean = isMatrixMemIQ

  def needLoadDependency: Boolean = exuBlockParams.map(_.needLoadDependency).reduce(_ || _)

  def numExu: Int = exuBlockParams.count(!_.fakeUnit)

  def numIntSrc: Int = exuBlockParams.map(_.numIntSrc).max

  def numFpSrc: Int = exuBlockParams.map(_.numFpSrc).max

  def numVecSrc: Int = exuBlockParams.map(_.numVecSrc).max

  def numVfSrc: Int = exuBlockParams.map(_.numVfSrc).max

  def numV0Src: Int = exuBlockParams.map(_.numV0Src).max

  def numVlSrc: Int = exuBlockParams.map(_.numVlSrc).max

  def numMxSrc: Int = exuBlockParams.map(_.numMxSrc).max

  def numRegSrc: Int = exuBlockParams.map(_.numRegSrc).max

  def numSrc: Int = exuBlockParams.map(_.numSrc).max

  def readIntRf: Boolean = numIntSrc > 0

  def readFpRf: Boolean = numFpSrc > 0

  def readVecRf: Boolean = numVecSrc > 0

  def readVfRf: Boolean = numVfSrc > 0

  def readV0Rf: Boolean = numV0Src > 0

  def readVlRf: Boolean = numVlSrc > 0

  def readMxRf: Boolean = numMxSrc > 0

  def writeIntRf: Boolean = exuBlockParams.map(_.writeIntRf).reduce(_ || _)

  def writeFpRf: Boolean = exuBlockParams.map(_.writeFpRf).reduce(_ || _)

  def writeVecRf: Boolean = exuBlockParams.map(_.writeVecRf).reduce(_ || _)

  def writeV0Rf: Boolean = exuBlockParams.map(_.writeV0Rf).reduce(_ || _)

  def writeVlRf: Boolean = exuBlockParams.map(_.writeVlRf).reduce(_ || _)

  def writeMxRf: Boolean = exuBlockParams.map(_.writeMxRf).reduce(_ || _)

  def exceptionOut: Seq[Int] = exuBlockParams.map(_.exceptionOut).reduce(_ ++ _).distinct.sorted

  def hasLoadError: Boolean = exuBlockParams.map(_.hasLoadError).reduce(_ || _)

  def flushPipe: Boolean = exuBlockParams.map(_.flushPipe).reduce(_ || _)

  def replayInst: Boolean = exuBlockParams.map(_.replayInst).reduce(_ || _)

  def trigger: Boolean = exuBlockParams.map(_.trigger).reduce(_ || _)

  def needExceptionGen: Boolean = exceptionOut.nonEmpty || flushPipe || replayInst || trigger

  def needPc: Boolean = JmpCnt + BrhCnt + FenceCnt > 0

  def needSrcFrm: Boolean = exuBlockParams.map(_.needSrcFrm).reduce(_ || _)

  def needSrcVxrm: Boolean = exuBlockParams.map(_.needSrcVxrm).reduce(_ || _)

  def writeVConfig: Boolean = exuBlockParams.map(_.writeVConfig).reduce(_ || _)
  
  def writeVType: Boolean = exuBlockParams.map(_.writeVType).reduce(_ || _)

  def writeMType: Boolean = exuBlockParams.map(_.writeMType).reduce(_ || _)

  def numPcReadPort: Int = (if (needPc) 1 else 0) * numEnq

  def numWriteIntRf: Int = exuBlockParams.count(_.writeIntRf)

  def numWriteFpRf: Int = exuBlockParams.count(_.writeFpRf)

  def numWriteVecRf: Int = exuBlockParams.count(_.writeVecRf)

  def numWriteVfRf: Int = exuBlockParams.count(_.writeVfRf)

  def numWriteMxRf: Int = exuBlockParams.count(_.writeMxRf)

  def numNoDataWB: Int = exuBlockParams.count(_.hasNoDataWB)

  def dataBitsMax: Int = if (numVecSrc > 0) VLEN else XLEN

  def numDeq: Int = numDeqOutside + exuBlockParams.length

  def numSimp: Int = numEntries - numEnq - numComp

  def isAllComp: Boolean = numComp == (numEntries - numEnq)

  def isAllSimp: Boolean = numComp == 0

  def hasCompAndSimp: Boolean = !(isAllComp || isAllSimp)

  def JmpCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.fuType == FuType.jmp)).sum

  def BrhCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.fuType == FuType.brh)).sum

  def I2fCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.fuType == FuType.i2f)).sum

  def CsrCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.fuType == FuType.csr)).sum

  def AluCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.fuType == FuType.alu)).sum

  def MulCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.fuType == FuType.mul)).sum

  def DivCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.fuType == FuType.div)).sum

  def FenceCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.fuType == FuType.fence)).sum

  def BkuCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.fuType == FuType.bku)).sum

  def VsetCnt: Int = exuBlockParams.map(_.fuConfigs.count(x => x.fuType == FuType.vsetiwi || x.fuType == FuType.vsetiwf || x.fuType == FuType.vsetfwf)).sum

  def FmacCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.fuType == FuType.fmac)).sum

  def fDivSqrtCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.fuType == FuType.fDivSqrt)).sum

  def LduCnt: Int = exuBlockParams.count(x => x.hasLoadFu && !x.hasStoreAddrFu)

  def StaCnt: Int = exuBlockParams.count(x => !x.hasLoadFu && x.hasStoreAddrFu)

  def MouCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.fuType == FuType.mou)).sum

  def StdCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.name == "std")).sum

  def HyuCnt: Int = exuBlockParams.count(_.hasHyldaFu) // only count hylda, since it equals to hysta

  def LdExuCnt: Int = LduCnt + HyuCnt + MlsCnt

  def LdWakeupCnt: Int = LduCnt + HyuCnt

  def VipuCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.fuType == FuType.vipu)).sum

  def VfpuCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.fuType == FuType.vfpu)).sum

  def VlduCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.fuType == FuType.vldu)).sum

  def VstuCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.fuType == FuType.vstu)).sum

  def MlsCnt: Int = exuBlockParams.count(_.hasMlsFu)

  def VseglduCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.fuType == FuType.vsegldu)).sum

  def VsegstuCnt: Int = exuBlockParams.map(_.fuConfigs.count(_.fuType == FuType.vsegstu)).sum

  def numRedirect: Int = exuBlockParams.count(_.hasRedirect)

  def numWriteRegCache: Int = exuBlockParams.map(x => if (x.needWriteRegCache) 1 else 0).sum

  def needWriteRegCache: Boolean = numWriteRegCache > 0

  def needReadRegCache: Boolean = exuBlockParams.map(_.needReadRegCache).reduce(_ || _)

  def needOg2Resp: Boolean = exuBlockParams.map(_.needOg2).reduce(_ || _)

  /**
    * Get the regfile type that this issue queue need to read
    */
  def pregReadSet: Set[DataConfig] = exuBlockParams.map(_.pregRdDataCfgSet).fold(Set())(_ union _)

  /**
    * Get the regfile type that this issue queue need to read
    */
  def pregWriteSet: Set[DataConfig] = exuBlockParams.map(_.pregWbDataCfgSet).fold(Set())(_ union _)

  /**
    * Get the max width of psrc
    */
  def rdPregIdxWidth = {
    this.pregReadSet.map(cfg => backendParam.getPregParams(cfg).addrWidth).fold(0)(_ max _)
  }

  /**
    * Get the max width of pdest
    */
  def wbPregIdxWidth = {
    this.pregWriteSet.map(cfg => backendParam.getPregParams(cfg).addrWidth).fold(0)(_ max _)
  }

  def iqWakeUpSourcePairs: Seq[WakeUpConfig] = exuBlockParams.flatMap(_.iqWakeUpSourcePairs)

  /**
    * Get exu source wake up
    */
  def wakeUpInExuSources: Seq[WakeUpSource] = {
    exuBlockParams
      .flatMap(_.iqWakeUpSinkPairs)
      .map(_.source)
      .distinctBy(_.name)
  }

  def wakeUpOutExuSources: Seq[WakeUpSource] = {
    exuBlockParams
      .flatMap(_.iqWakeUpSourcePairs)
      .map(_.source)
      .distinctBy(_.name)
  }

  def wakeUpToExuSinks = exuBlockParams
    .flatMap(_.iqWakeUpSourcePairs)
    .map(_.sink).distinct

  def numWakeupToIQ: Int = wakeUpInExuSources.size

  def numWakeupFromIQ: Int = wakeUpInExuSources.size

  def numAllWakeUp: Int = numWakeupFromWB + numWakeupFromIQ + numWakeupFromOthers

  def numWakeupFromWB = {
    val pregSet = this.pregReadSet
    pregSet.map(cfg => backendParam.getRfWriteSize(cfg)).sum
  }

  def hasIQWakeUp: Boolean = numWakeupFromIQ > 0 && numRegSrc > 0

  def needWakeupFromIntWBPort = backendParam.allExuParams.filter(x => !wakeUpInExuSources.map(_.name).contains(x.name) && this.readIntRf).groupBy(x => x.getIntWBPort.getOrElse(IntWB(port = -1)).port).filter(_._1 != -1)

  def needWakeupFromFpWBPort = if (this.exuBlockParams.map(_.hasStdFu).reduce(_ || _)) {
    // here add fp load WB wakeup to std
    backendParam.allExuParams.filter(x => (!wakeUpInExuSources.map(_.name).contains(x.name) || x.hasLoadExu) && this.readFpRf).groupBy(x => x.getFpWBPort.getOrElse(FpWB(port = -1)).port).filter(_._1 != -1)
  }
  else {
    backendParam.allExuParams.filter(x => !wakeUpInExuSources.map(_.name).contains(x.name) && this.readFpRf).groupBy(x => x.getFpWBPort.getOrElse(FpWB(port = -1)).port).filter(_._1 != -1)
  }

  def needWakeupFromVfWBPort = backendParam.allExuParams.filter(x => !wakeUpInExuSources.map(_.name).contains(x.name) && this.readVecRf).groupBy(x => x.getVfWBPort.getOrElse(VfWB(port = -1)).port).filter(_._1 != -1)

  def needWakeupFromV0WBPort = backendParam.allExuParams.filter(x => !wakeUpInExuSources.map(_.name).contains(x.name) && this.readV0Rf).groupBy(x => x.getV0WBPort.getOrElse(V0WB(port = -1)).port).filter(_._1 != -1)

  def needWakeupFromVlWBPort = backendParam.allExuParams.filter(x => !wakeUpInExuSources.map(_.name).contains(x.name) && this.readVlRf).groupBy(x => x.getVlWBPort.getOrElse(VlWB(port = -1)).port).filter(_._1 != -1)

  def needWakeupFromMxWBPort = backendParam.allExuParams.filter(x => !wakeUpInExuSources.map(_.name).contains(x.name) && this.readMxRf).groupBy(x => x.getMxWBPort.getOrElse(MxWB(port = -1)).port).filter(_._1 != -1)

  def hasWakeupFromMem: Boolean = backendParam.allExuParams.filter(x => wakeUpInExuSources.map(_.name).contains(x.name)).map(_.isMemExeUnit).fold(false)(_ | _)

  def hasWakeupFromVf: Boolean = backendParam.allExuParams.filter(x => wakeUpInExuSources.map(_.name).contains(x.name)).map(_.isVfExeUnit).fold(false)(_ | _)

  def getFuCfgs: Seq[FuConfig] = exuBlockParams.flatMap(_.fuConfigs).distinct

  def deqFuCfgs: Seq[Seq[FuConfig]] = exuBlockParams.map(_.fuConfigs)

  def deqFuInterSect: Seq[FuConfig] = if (numDeq == 2) deqFuCfgs(0).intersect(deqFuCfgs(1)) else Seq()

  def deqFuSame: Boolean = (numDeq == 2) && deqFuInterSect.length == deqFuCfgs(0).length && deqFuCfgs(0).length == deqFuCfgs(1).length

  def deqFuDiff: Boolean = (numDeq == 2) && deqFuInterSect.length == 0

  def deqImmTypes: Seq[UInt] = getFuCfgs.flatMap(_.immType).distinct

  // set load imm to 32-bit for fused_lui_load
  def deqImmTypesMaxLen: Int = if (isLdAddrIQ || isHyAddrIQ) 32 else deqImmTypes.map(SelImm.getImmUnion(_)).maxBy(_.len).len

  def needImm: Boolean = deqImmTypes.nonEmpty

  // cfgs(exuIdx)(set of exu's wb)

  /**
    * Get [[PregWB]] of this IssueBlock
    * @return set of [[PregWB]] of [[ExeUnit]]
    */
  def getWbCfgs: Seq[Set[PregWB]] = {
    exuBlockParams.map(exu => exu.wbPortConfigs.toSet)
  }

  def canAccept(fuType: UInt): Bool = {
    Cat(getFuCfgs.map(_.fuType.U === fuType)).orR
  }

  def bindBackendParam(param: BackendParams): Unit = {
    backendParam = param
  }

  def wakeUpSourceExuIdx: Seq[Int] = {
    wakeUpInExuSources.map(x => backendParam.getExuIdx(x.name))
  }

  def genExuInputDecoupledBundle(implicit p: Parameters): MixedVec[DecoupledIO[ExuInput]] = {
    MixedVec(this.exuBlockParams.map(x => DecoupledIO(x.genExuInputBundle)))
  }

  def genExuInputDecoupledCopySrcBundle(implicit p: Parameters): MixedVec[DecoupledIO[ExuInput]] = {
    MixedVec(this.exuBlockParams.map(x => DecoupledIO(x.genExuInputCopySrcBundle)))
  }

  def genExuOutputDecoupledBundle(implicit p: Parameters): MixedVec[DecoupledIO[ExuOutput]] = {
    MixedVec(this.exuParams.map(x => DecoupledIO(x.genExuOutputBundle)))
  }

  def genExuOutputValidBundle(implicit p: Parameters): MixedVec[ValidIO[ExuOutput]] = {
    MixedVec(this.exuParams.map(x => ValidIO(x.genExuOutputBundle)))
  }

  def genExuBypassValidBundle(implicit p: Parameters): MixedVec[ValidIO[ExuBypassBundle]] = {
    MixedVec(this.exuParams.filterNot(_.fakeUnit).map(x => ValidIO(x.genExuBypassBundle)))
  }

  def genIssueDecoupledBundle(implicit p: Parameters): MixedVec[DecoupledIO[IssueQueueIssueBundle]] = {
    MixedVec(exuBlockParams.filterNot(_.fakeUnit).map(x => DecoupledIO(new IssueQueueIssueBundle(this, x))))
  }

  def genIssueValidBundle(implicit p: Parameters): MixedVec[ValidIO[IssueQueueIssueBundle]] = {
    MixedVec(exuBlockParams.filterNot(_.fakeUnit).map(x => ValidIO(new IssueQueueIssueBundle(this, x))))
  }

  def genWBWakeUpSinkValidBundle(implicit p: Parameters): MixedVec[ValidIO[IssueQueueWBWakeUpBundle]] = {
    val intBundle: Seq[ValidIO[IssueQueueWBWakeUpBundle]] = schdType match {
      case IntScheduler() | MemScheduler() | MfScheduler() => needWakeupFromIntWBPort.map(x => ValidIO(new IssueQueueWBWakeUpBundle(x._2.map(_.exuIdx), backendParam))).toSeq
      case _ => Seq()
    }
    val fpBundle = schdType match {
      case FpScheduler() | MemScheduler() => needWakeupFromFpWBPort.map(x => ValidIO(new IssueQueueWBWakeUpBundle(x._2.map(_.exuIdx), backendParam))).toSeq
      case _ => Seq()
    }
    val vfBundle = schdType match {
      case VfScheduler() | MemScheduler() => needWakeupFromVfWBPort.map(x => ValidIO(new IssueQueueWBWakeUpBundle(x._2.map(_.exuIdx), backendParam))).toSeq
      case _ => Seq()
    }
    val v0Bundle = schdType match {
      case VfScheduler() | MemScheduler() => needWakeupFromV0WBPort.map(x => ValidIO(new IssueQueueWBWakeUpBundle(x._2.map(_.exuIdx), backendParam))).toSeq
      case _ => Seq()
    }
    val vlBundle = schdType match {
      case VfScheduler() | MemScheduler() => needWakeupFromVlWBPort.map(x => ValidIO(new IssueQueueWBWakeUpBundle(x._2.map(_.exuIdx), backendParam))).toSeq
      case _ => Seq()
    }
    val mxBundle = schdType match {
      case MfScheduler() | MemScheduler() => needWakeupFromMxWBPort.map(x => ValidIO(new IssueQueueWBWakeUpBundle(x._2.map(_.exuIdx), backendParam))).toSeq
      case _ => Seq()
    }
    MixedVec(intBundle ++ fpBundle ++ vfBundle ++ v0Bundle ++ vlBundle ++ mxBundle)
  }

  def genIQWakeUpSourceValidBundle(implicit p: Parameters): MixedVec[ValidIO[IssueQueueIQWakeUpBundle]] = {
    MixedVec(exuBlockParams.map(x => ValidIO(new IssueQueueIQWakeUpBundle(x.exuIdx, backendParam, x.copyWakeupOut, x.copyNum))))
  }

  def genIQWakeUpSinkValidBundle(implicit p: Parameters): MixedVec[ValidIO[IssueQueueIQWakeUpBundle]] = {
    MixedVec(this.wakeUpInExuSources.map(x => ValidIO(new IssueQueueIQWakeUpBundle(backendParam.getExuIdx(x.name), backendParam))))
  }

  def genOGRespBundle(implicit p: Parameters) = {
    implicit val issueBlockParams = this
    MixedVec(exuBlockParams.map(_ => new OGRespBundle))
  }

  def genOG2RespBundle(implicit p: Parameters) = {
    implicit val issueBlockParams = this
    MixedVec(exuBlockParams.map(_ => new Valid(new EntryDeqRespBundle)))
  }

  def genWbFuBusyTableWriteBundle(implicit p: Parameters) = {
    implicit val issueBlockParams = this
    MixedVec(exuBlockParams.map(x => new WbFuBusyTableWriteBundle(x)))
  }

  def genWbFuBusyTableReadBundle(implicit p: Parameters) = {
    implicit val issueBlockParams = this
    MixedVec(exuBlockParams.map{ x =>
      new WbFuBusyTableReadBundle(x)
    })
  }

  def genWbConflictBundle()(implicit p: Parameters) = {
    implicit val issueBlockParams = this
    MixedVec(exuBlockParams.map { x =>
      new WbConflictBundle(x)
    })
  }

  def getIQName = {
    "IssueQueue" ++ getFuCfgs.map(_.name).distinct.map(_.capitalize).reduce(_ ++ _)
  }

  def getEntryName = {
    "Entries" ++ getFuCfgs.map(_.name).distinct.map(_.capitalize).reduce(_ ++ _)
  }
}
