package xiangshan.backend.datapath

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import utility.{GatedValidRegNext, SignExt, ZeroExt}
import xiangshan.{JumpOpType, SelImm, XSBundle, XSModule}
import xiangshan.backend.BackendParams
import xiangshan.backend.Bundles.{ExuBypassBundle, ExuInput, ExuOutput, ExuVec, ImmInfo}
import xiangshan.backend.issue.{FpScheduler, ImmExtractor, IntScheduler, MemScheduler, VfScheduler, MfScheduler}
import xiangshan.backend.datapath.DataConfig.RegDataMaxWidth
import xiangshan.backend.decode.ImmUnion
import xiangshan.backend.regcache._
import xiangshan.backend.Bundles._
import xiangshan.backend.fu.FuType

class BypassNetworkIO()(implicit p: Parameters, params: BackendParams) extends XSBundle {
  // params
  private val intSchdParams = params.schdParams(IntScheduler())
  private val fpSchdParams = params.schdParams(FpScheduler())
  private val vfSchdParams = params.schdParams(VfScheduler())
  private val mfSchdParams = params.schdParams(MfScheduler())
  private val memSchdParams = params.schdParams(MemScheduler())

  val fromDataPath = new FromDataPath
  val toExus = new ToExus
  val fromExus = new FromExus

  class FromDataPath extends Bundle {
    val int: MixedVec[MixedVec[DecoupledIO[ExuInput]]] = Flipped(intSchdParams.genExuInputBundle)
    val fp : MixedVec[MixedVec[DecoupledIO[ExuInput]]] = Flipped(fpSchdParams.genExuInputBundle)
    val vf : MixedVec[MixedVec[DecoupledIO[ExuInput]]] = Flipped(vfSchdParams.genExuInputBundle)
    val mf : MixedVec[MixedVec[DecoupledIO[ExuInput]]] = Flipped(mfSchdParams.genExuInputBundle)
    val mem: MixedVec[MixedVec[DecoupledIO[ExuInput]]] = Flipped(memSchdParams.genExuInputBundle)
    val immInfo: Vec[ImmInfo] = Input(Vec(params.allExuParams.size, new ImmInfo))
    val rcData: MixedVec[MixedVec[Vec[UInt]]] = MixedVec(
      Seq(intSchdParams, fpSchdParams, vfSchdParams, mfSchdParams, memSchdParams).map(schd => schd.issueBlockParams.map(iq => 
        MixedVec(iq.exuBlockParams.map(exu => Input(Vec(exu.numRegSrc, UInt(exu.srcDataBitsMax.W)))))
      )).flatten
    )
  }

  class ToExus extends Bundle {
    val int: MixedVec[MixedVec[DecoupledIO[ExuInput]]] = intSchdParams.genExuInputCopySrcBundle
    val fp : MixedVec[MixedVec[DecoupledIO[ExuInput]]] = fpSchdParams.genExuInputCopySrcBundle
    val vf : MixedVec[MixedVec[DecoupledIO[ExuInput]]] = vfSchdParams.genExuInputCopySrcBundle
    val mf : MixedVec[MixedVec[DecoupledIO[ExuInput]]] = mfSchdParams.genExuInputCopySrcBundle
    val mem: MixedVec[MixedVec[DecoupledIO[ExuInput]]] = memSchdParams.genExuInputCopySrcBundle
  }

  class FromExus extends Bundle {
    val int: MixedVec[MixedVec[ValidIO[ExuBypassBundle]]] = Flipped(intSchdParams.genExuBypassValidBundle)
    val fp : MixedVec[MixedVec[ValidIO[ExuBypassBundle]]] = Flipped(fpSchdParams.genExuBypassValidBundle)
    val vf : MixedVec[MixedVec[ValidIO[ExuBypassBundle]]] = Flipped(vfSchdParams.genExuBypassValidBundle)
    val mf : MixedVec[MixedVec[ValidIO[ExuBypassBundle]]] = Flipped(mfSchdParams.genExuBypassValidBundle)
    val mem: MixedVec[MixedVec[ValidIO[ExuBypassBundle]]] = Flipped(memSchdParams.genExuBypassValidBundle)

    def connectExuOutput(
      getSinkVecN: FromExus => MixedVec[MixedVec[ValidIO[ExuBypassBundle]]]
    )(
      sourceVecN: MixedVec[MixedVec[DecoupledIO[ExuOutput]]]
    ): Unit = {
      getSinkVecN(this).zip(sourceVecN).foreach { case (sinkVec, sourcesVec) =>
        sinkVec.zip(sourcesVec).foreach { case (sink, source) =>
          sink.valid := source.valid
          sink.bits.intWen := source.bits.intWen.getOrElse(false.B)
          sink.bits.pdest := source.bits.pdest
          sink.bits.data := source.bits.data(0)
        }
      }
    }
  }

  val toDataPath: Vec[RCWritePort] = Vec(params.getIntExuRCWriteSize + params.getMemExuRCWriteSize, 
    Flipped(new RCWritePort(params.intSchdParams.get.rfDataWidth, RegCacheIdxWidth, params.intSchdParams.get.pregIdxWidth, params.debugEn)))
}

class BypassNetwork()(implicit p: Parameters, params: BackendParams) extends XSModule {
  val io: BypassNetworkIO = IO(new BypassNetworkIO)

  private val fromDPs: Seq[DecoupledIO[ExuInput]] = (io.fromDataPath.int ++ io.fromDataPath.fp ++ io.fromDataPath.vf ++ io.fromDataPath.mf ++ io.fromDataPath.mem).flatten.toSeq
  private val fromExus: Seq[ValidIO[ExuBypassBundle]] = (io.fromExus.int ++ io.fromExus.fp ++ io.fromExus.vf ++ io.fromExus.mf ++ io.fromExus.mem).flatten.toSeq
  private val toExus: Seq[DecoupledIO[ExuInput]] = (io.toExus.int ++ io.toExus.fp ++ io.toExus.vf ++ io.toExus.mf ++ io.toExus.mem).flatten.toSeq
  private val fromDPsRCData: Seq[Vec[UInt]] = io.fromDataPath.rcData.flatten.toSeq
  private val immInfo = io.fromDataPath.immInfo

  println(s"[BypassNetwork] RCData num: ${fromDPsRCData.size}")

  // (exuIdx, srcIdx, bypassExuIdx)
  private val forwardOrBypassValidVec3: MixedVec[Vec[Vec[Bool]]] = MixedVecInit(
    fromDPs.map { (x: DecoupledIO[ExuInput]) =>
      println(s"[BypassNetwork] ${x.bits.params.name} numRegSrc: ${x.bits.params.numRegSrc}")
      VecInit(x.bits.exuSources.map(_.map(_.toExuOH(x.bits.params))).getOrElse(
        // TODO: remove tmp max 1 for fake HYU1
        VecInit(Seq.fill(x.bits.params.numRegSrc max 1)(VecInit(0.U(params.numExu.W).asBools)))
      ))
    }
  )

  private val forwardDataVec: Vec[UInt] = VecInit(
    fromExus.map(x => ZeroExt(x.bits.data, RegDataMaxWidth))
  )

  private val bypassDataVec = VecInit(
    fromExus.map(x => ZeroExt(RegEnable(x.bits.data, x.valid), RegDataMaxWidth))
  )

  private val intExuNum = params.intSchdParams.get.numExu
  private val fpExuNum  = params.fpSchdParams.get.numExu
  private val vfExuNum  = params.vfSchdParams.get.numExu
  private val mfExuNum  = params.mfSchdParams.get.numExu
  private val memExuNum = params.memSchdParams.get.numExu

  println(s"[BypassNetwork] allExuNum: ${toExus.size} intExuNum: ${intExuNum} fpExuNum: ${fpExuNum} vfExuNum: ${vfExuNum} mfExuNum: ${mfExuNum} memExuNum: ${memExuNum}")

  private val fromDPsHasBypass2Source = fromDPs.filter(x => x.bits.params.isIQWakeUpSource && x.bits.params.writeVfRf && (x.bits.params.isVfExeUnit || x.bits.params.hasLoadExu)).map(_.bits.params.exuIdx)
  private val fromDPsHasBypass2Sink   = fromDPs.filter(x => x.bits.params.isIQWakeUpSink && x.bits.params.readVfRf && (x.bits.params.isVfExeUnit || x.bits.params.isMemExeUnit)).map(_.bits.params.exuIdx)

  private val bypass2ValidVec3 = MixedVecInit(
    fromDPsHasBypass2Sink.map(forwardOrBypassValidVec3(_)).map(exu => VecInit(exu.map(exuOH => 
      VecInit(fromDPsHasBypass2Source.map(exuOH(_))).asUInt
    )))
  )
  if(params.debugEn){
    dontTouch(bypass2ValidVec3)
  }
  private val bypass2DateEn = VecInit(
    fromExus.map(x => GatedValidRegNext(x.valid))
  ).asUInt
  private val bypass2DataVec = if (fromDPsHasBypass2Source.length == 0) VecInit(Seq(0.U)) else VecInit(
    fromDPsHasBypass2Source.map(x => RegEnable(bypassDataVec(x), bypass2DateEn(x).asBool))
  )

  println(s"[BypassNetwork] HasBypass2SourceExuNum: ${fromDPsHasBypass2Source.size} HasBypass2SinkExuNum: ${fromDPsHasBypass2Sink.size} bypass2DataVecSize: ${bypass2DataVec.length}")
  println(s"[BypassNetwork] HasBypass2SourceExu: ${fromDPsHasBypass2Source}")
  println(s"[BypassNetwork] HasBypass2SinkExu: ${fromDPsHasBypass2Sink}")

  toExus.zip(fromDPs).foreach { case (sink, source) =>
    connectSamePort(sink.bits, source.bits)
    sink.valid := source.valid
    source.ready := sink.ready
  }

  toExus.zipWithIndex.foreach { case (exuInput, exuIdx) =>
    exuInput.bits.src.zipWithIndex.foreach { case (src, srcIdx) =>
      val imm = ImmExtractor(
        immInfo(exuIdx).imm,
        immInfo(exuIdx).immType,
        exuInput.bits.params.destDataBitsMax,
        exuInput.bits.params.immType.map(_.litValue)
      )
      val immLoadSrc0 = SignExt(ImmUnion.U.toImm32(immInfo(exuIdx).imm(immInfo(exuIdx).imm.getWidth - 1, ImmUnion.I.len)), XLEN)
      val exuParm = exuInput.bits.params
      val isIntScheduler = exuParm.isIntExeUnit
      val isReadVfRf= exuParm.readVfRf
      val dataSource = exuInput.bits.dataSources(srcIdx)
      val isWakeUpSink = params.allIssueParams.filter(_.exuBlockParams.contains(exuParm)).head.exuBlockParams.map(_.isIQWakeUpSink).reduce(_ || _)
      val readForward = if (isWakeUpSink) dataSource.readForward else false.B
      val readBypass = if (isWakeUpSink) dataSource.readBypass else false.B
      val readZero = if (isIntScheduler) dataSource.readZero else false.B
      val readV0 = if (srcIdx < 3 && isReadVfRf) dataSource.readV0 else false.B
      val readRegOH = exuInput.bits.dataSources(srcIdx).readRegOH
      val readRegCache = if (exuParm.needReadRegCache) exuInput.bits.dataSources(srcIdx).readRegCache else false.B
      val readImm = if (exuParm.immType.nonEmpty || exuParm.hasLoadExu) exuInput.bits.dataSources(srcIdx).readImm else false.B
      val bypass2ExuIdx = fromDPsHasBypass2Sink.indexOf(exuIdx)
      println(s"${exuParm.name}: bypass2ExuIdx is ${bypass2ExuIdx}")
      val readBypass2 = if (bypass2ExuIdx >= 0) dataSource.readBypass2 else false.B
      print(s"[BypassNetwork] exuIdx: $exuIdx, srcIdx: $srcIdx, isWakeUpSink: $isWakeUpSink, isIntScheduler: $isIntScheduler, isReadVfRf: $isReadVfRf\n")
      print(s"[BypassNetwork] exuParm.needReadRegCache: ${exuParm.needReadRegCache} exuParm.immType: ${exuParm.immType} bypass2ExuIdx: $bypass2ExuIdx\n")
      src := Mux1H(
        Seq(
          readForward    -> Mux1H(forwardOrBypassValidVec3(exuIdx)(srcIdx), forwardDataVec),
          readBypass     -> Mux1H(forwardOrBypassValidVec3(exuIdx)(srcIdx), bypassDataVec),
          readBypass2    -> (if (bypass2ExuIdx >= 0) Mux1H(bypass2ValidVec3(bypass2ExuIdx)(srcIdx), bypass2DataVec) else 0.U),
          readZero       -> 0.U,
          readV0         -> (if (srcIdx < 3 && isReadVfRf) exuInput.bits.src(3) else 0.U),
          readRegOH      -> fromDPs(exuIdx).bits.src(srcIdx),
          readRegCache   -> fromDPsRCData(exuIdx)(srcIdx),
          readImm        -> (if (exuParm.hasLoadExu && srcIdx == 0) immLoadSrc0 else imm)
        )
      )
    }
    if (exuInput.bits.params.hasBrhFu) {
      val immWidth = exuInput.bits.params.immType.map(x => SelImm.getImmUnion(x).len).max
      val nextPcOffset = exuInput.bits.ftqOffset.get +& Mux(exuInput.bits.preDecode.get.isRVC, 1.U, 2.U)
      val imm = ImmExtractor(
        immInfo(exuIdx).imm,
        immInfo(exuIdx).immType,
        exuInput.bits.params.destDataBitsMax,
        exuInput.bits.params.immType.map(_.litValue)
      )
      val isJALR = FuType.isJump(exuInput.bits.fuType) && JumpOpType.jumpOpisJalr(exuInput.bits.fuOpType)
      val immBJU = imm + Mux(isJALR, 0.U, (exuInput.bits.ftqOffset.getOrElse(0.U) << instOffsetBits).asUInt)
      exuInput.bits.imm := immBJU
      exuInput.bits.nextPcOffset.get := nextPcOffset
    }
    exuInput.bits.copySrc.get.map( copysrc =>
      copysrc.zip(exuInput.bits.src).foreach{ case(copy, src) => copy := src}
    )
  }

  // to reg cache
  private val forwardIntWenVec = VecInit(
    fromExus.filter(_.bits.params.needWriteRegCache).map(x => x.valid && x.bits.intWen)
  )
  private val forwardTagVec = VecInit(
    fromExus.filter(_.bits.params.needWriteRegCache).map(x => x.bits.pdest)
  )

  private val bypassIntWenVec = VecInit(
    forwardIntWenVec.map(x => GatedValidRegNext(x))
  )
  private val bypassTagVec = VecInit(
    forwardTagVec.zip(forwardIntWenVec).map(x => RegEnable(x._1, x._2))
  )
  private val bypassRCDataVec = VecInit(
    fromExus.zip(bypassDataVec).filter(_._1.bits.params.needWriteRegCache).map(_._2)
  )

  println(s"[BypassNetwork] WriteRegCacheExuNum: ${forwardIntWenVec.size}")

  io.toDataPath.zipWithIndex.foreach{ case (x, i) => 
    x.wen := bypassIntWenVec(i)
    x.addr := DontCare
    x.data := bypassRCDataVec(i)
    x.tag.foreach(_ := bypassTagVec(i))
  }
}
