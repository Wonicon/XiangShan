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

package top

import chisel3._
import chisel3.util._
import xiangshan._
import utils._
import utility._
import system._
import org.chipsalliance.cde.config._
import freechips.rocketchip.tile.{BusErrorUnit, BusErrorUnitParams, XLen}
import xiangshan.frontend.icache.ICacheParameters
import freechips.rocketchip.devices.debug._
import freechips.rocketchip.tile.{MaxHartIdBits, XLen}
import system._
import utility._
import utils._
import huancun._
import openLLC.{OpenLLCParam}
import freechips.rocketchip.diplomacy._
import xiangshan._
import xiangshan.backend.dispatch.DispatchParameters
import xiangshan.backend.regfile.{IntPregParams, VfPregParams}
import xiangshan.cache.DCacheParameters
import xiangshan.cache.mmu.{L2TLBParameters, TLBParameters}
import device.{EnableJtag, XSDebugModuleParams}
import huancun._
import coupledL2._
import coupledL2.prefetch._
import xiangshan.frontend.icache.ICacheParameters

class BaseConfig(n: Int) extends Config((site, here, up) => {
  case XLen => 64
  case DebugOptionsKey => DebugOptions()
  case SoCParamsKey => SoCParameters()
  case PMParameKey => PMParameters()
  case XSTileKey => Seq.tabulate(n){ i => XSCoreParameters(HartId = i) }
  case ExportDebug => DebugAttachParams(protocols = Set(JTAG))
  case DebugModuleKey => Some(XSDebugModuleParams(site(XLen)))
  case JtagDTMKey => JtagDTMKey
  case MaxHartIdBits => log2Up(n) max 6
  case EnableJtag => true.B
})

// Synthesizable minimal XiangShan
// * It is still an out-of-order, super-scalaer arch
// * L1 cache included
// * L2 cache NOT included
// * L3 cache included
class MinimalConfig(n: Int = 1) extends Config(
  new BaseConfig(n).alter((site, here, up) => {
    case XSTileKey => up(XSTileKey).map(
      p => p.copy(
        DecodeWidth = 6,
        RenameWidth = 6,
        RobCommitWidth = 8,
        FetchWidth = 4,
        VirtualLoadQueueSize = 24,
        LoadQueueRARSize = 24,
        LoadQueueRAWSize = 12,
        LoadQueueReplaySize = 24,
        LoadUncacheBufferSize = 8,
        LoadQueueNWriteBanks = 4, // NOTE: make sure that LoadQueue{RAR, RAW, Replay}Size is divided by LoadQueueNWriteBanks.
        RollbackGroupSize = 8,
        StoreQueueSize = 20,
        StoreQueueNWriteBanks = 4, // NOTE: make sure that StoreQueueSize is divided by StoreQueueNWriteBanks
        StoreQueueForwardWithMask = true,
        // ============ VLSU ============
        VlMergeBufferSize = 16,
        VsMergeBufferSize = 8,
        UopWritebackWidth = 2,
        // ==============================
        RobSize = 48,
        RabSize = 96,
        FtqSize = 8,
        IBufSize = 24,
        IBufNBank = 6,
        StoreBufferSize = 4,
        StoreBufferThreshold = 3,
        IssueQueueSize = 10,
        IssueQueueCompEntrySize = 4,
        dpParams = DispatchParameters(
          IntDqSize = 12,
          FpDqSize = 12,
          LsDqSize = 12,
          IntDqDeqWidth = 8,
          FpDqDeqWidth = 6,
          VecDqDeqWidth = 6,
          LsDqDeqWidth = 6
        ),
        intPreg = IntPregParams(
          numEntries = 64,
          numRead = None,
          numWrite = None,
        ),
        vfPreg = VfPregParams(
          numEntries = 160,
          numRead = None,
          numWrite = None,
        ),
        icacheParameters = ICacheParameters(
          nSets = 64, // 16KB ICache
          tagECC = Some("parity"),
          dataECC = Some("parity"),
          replacer = Some("setplru"),
        ),
        dcacheParametersOpt = Some(DCacheParameters(
          nSets = 64, // 32KB DCache
          nWays = 8,
          tagECC = Some("secded"),
          dataECC = Some("secded"),
          replacer = Some("setplru"),
          nMissEntries = 4,
          nProbeEntries = 4,
          nReleaseEntries = 8,
          nMaxPrefetchEntry = 2,
          enableTagEcc = true,
          enableDataEcc = true,
          cacheCtrlAddressOpt = Some(AddressSet(0x38022000, 0x7f))
        )),
        // ============ BPU ===============
        EnableLoop = false,
        EnableGHistDiff = false,
        FtbSize = 256,
        FtbWays = 2,
        RasSize = 8,
        RasSpecSize = 16,
        TageTableInfos =
          Seq((512, 4, 6),
            (512, 9, 6),
            (1024, 19, 6)),
        SCNRows = 128,
        SCNTables = 2,
        SCHistLens = Seq(0, 5),
        ITTageTableInfos =
          Seq((256, 4, 7),
            (256, 8, 7),
            (512, 16, 7)),
        // ================================
        itlbParameters = TLBParameters(
          name = "itlb",
          fetchi = true,
          useDmode = false,
          NWays = 4,
        ),
        ldtlbParameters = TLBParameters(
          name = "ldtlb",
          NWays = 4,
          partialStaticPMP = true,
          outsideRecvFlush = true,
          outReplace = false,
          lgMaxSize = 4
        ),
        sttlbParameters = TLBParameters(
          name = "sttlb",
          NWays = 4,
          partialStaticPMP = true,
          outsideRecvFlush = true,
          outReplace = false,
          lgMaxSize = 4
        ),
        hytlbParameters = TLBParameters(
          name = "hytlb",
          NWays = 4,
          partialStaticPMP = true,
          outsideRecvFlush = true,
          outReplace = false,
          lgMaxSize = 4
        ),
        pftlbParameters = TLBParameters(
          name = "pftlb",
          NWays = 4,
          partialStaticPMP = true,
          outsideRecvFlush = true,
          outReplace = false,
          lgMaxSize = 4
        ),
        btlbParameters = TLBParameters(
          name = "btlb",
          NWays = 4,
        ),
        l2tlbParameters = L2TLBParameters(
          l3Size = 4,
          l2Size = 4,
          l1nSets = 4,
          l1nWays = 4,
          l1ReservedBits = 1,
          l0nSets = 4,
          l0nWays = 8,
          l0ReservedBits = 0,
          spSize = 4,
        ),
        L2CacheParamsOpt = Some(L2Param(
          name = "L2",
          ways = 8,
          sets = 128,
          echoField = Seq(huancun.DirtyField()),
          prefetch = Nil,
          clientCaches = Seq(L1Param(
            "dcache",
            isKeywordBitsOpt = p.dcacheParametersOpt.get.isKeywordBitsOpt
          )),
        )),
        L2NBanks = 8,
        prefetcher = None // if L2 pf_recv_node does not exist, disable SMS prefetcher
      )
    )
    case SoCParamsKey =>
      val tiles = site(XSTileKey)
      up(SoCParamsKey).copy(
        L3CacheParamsOpt = Some(up(SoCParamsKey).L3CacheParamsOpt.get.copy(
          sets = 1024,
          inclusive = false,
          clientCaches = tiles.map{ core =>
            val clientDirBytes = tiles.map{ t =>
              t.L2NBanks * t.L2CacheParamsOpt.map(_.toCacheParams.capacity).getOrElse(0)
            }.sum
            val l2params = core.L2CacheParamsOpt.get.toCacheParams
            l2params.copy(sets = 2 * clientDirBytes / core.L2NBanks / l2params.ways / 64)
          },
          simulation = !site(DebugOptionsKey).FPGAPlatform,
          prefetch = None
        )),
        L3NBanks = 1
      )
  })
)

class MinimalMatrixConfig(n: Int) extends Config(
  new MinimalConfig(n).alter((site, here, up) => {
    case XSTileKey => up(XSTileKey).map(_.copy(
      l2tlbParameters = L2TLBParameters(
        l3Size = 4,
        l2Size = 4,
        l1nSets = 4,
        l1nWays = 4,
        l1ReservedBits = 1,
        l0nSets = 4,
        l0nWays = 8,
        l0ReservedBits = 0,
        spSize = 4,
        dfilterSize = 48,
      ),
    ))
  })
)

// Non-synthesizable MinimalConfig, for fast simulation only
class MinimalSimConfig(n: Int = 1) extends Config(
  new MinimalConfig(n).alter((site, here, up) => {
    case XSTileKey => up(XSTileKey).map(_.copy(
      dcacheParametersOpt = None,
      softPTW = true
    ))
    case SoCParamsKey => up(SoCParamsKey).copy(
      L3CacheParamsOpt = None
    )
  })
)

class WithNKBL1D(n: Int, ways: Int = 8) extends Config((site, here, up) => {
  case XSTileKey =>
    val sets = n * 1024 / ways / 64
    up(XSTileKey).map(_.copy(
      dcacheParametersOpt = Some(DCacheParameters(
        nSets = sets,
        nWays = ways,
        tagECC = Some("secded"),
        dataECC = Some("secded"),
        replacer = Some("setplru"),
        nMissEntries = 16,
        nProbeEntries = 8,
        nReleaseEntries = 18,
        nMaxPrefetchEntry = 6,
        enableTagEcc = true,
        enableDataEcc = true,
        cacheCtrlAddressOpt = Some(AddressSet(0x38022000, 0x7f))
      ))
    ))
})

class WithNKBL2
(
  n: Int,
  ways: Int = 8,
  inclusive: Boolean = true,
  banks: Int = 1,
  tp: Boolean = true
) extends Config((site, here, up) => {
  case XSTileKey =>
    require(inclusive, "L2 must be inclusive")
    val upParams = up(XSTileKey)
    val l2sets = n * 1024 / banks / ways / 64
    upParams.map(p => p.copy(
      L2CacheParamsOpt = Some(L2Param(
        name = "L2",
        ways = ways,
        sets = l2sets,
        clientCaches = Seq(L1Param(
          "dcache",
          sets = 2 * p.dcacheParametersOpt.get.nSets / banks,
          ways = p.dcacheParametersOpt.get.nWays + 2,
          aliasBitsOpt = p.dcacheParametersOpt.get.aliasBitsOpt,
          vaddrBitsOpt = Some(p.GPAddrBitsSv48x4 - log2Up(p.dcacheParametersOpt.get.blockBytes)),
          isKeywordBitsOpt = p.dcacheParametersOpt.get.isKeywordBitsOpt
        )),
        reqField = Seq(utility.ReqSourceField()),
        echoField = Seq(huancun.DirtyField()),
        tagECC = Some("secded"),
        dataECC = Some("secded"),
        enableTagECC = true,
        enableDataECC = true,
        dataCheck = Some("oddparity"),
        prefetch = Seq(BOPParameters()) ++
          (if (tp) Seq(TPParameters()) else Nil) ++
          (if (p.prefetcher.nonEmpty) Seq(PrefetchReceiverParams()) else Nil),
        enablePerf = !site(DebugOptionsKey).FPGAPlatform && site(DebugOptionsKey).EnablePerfDebug,
        enableRollingDB = site(DebugOptionsKey).EnableRollingDB,
        enableMonitor = site(DebugOptionsKey).AlwaysBasicDB,
        elaboratedTopDown = !site(DebugOptionsKey).FPGAPlatform
      )),
      L2NBanks = banks
    ))
})

class WithNKBL3(n: Int, ways: Int = 8, inclusive: Boolean = true, banks: Int = 1) extends Config((site, here, up) => {
  case SoCParamsKey =>
    val sets = n * 1024 / banks / ways / 64
    val tiles = site(XSTileKey)
    val clientDirBytes = tiles.map{ t =>
      t.L2NBanks * t.L2CacheParamsOpt.map(_.toCacheParams.capacity).getOrElse(0)
    }.sum
    up(SoCParamsKey).copy(
      L3NBanks = banks,
      L3CacheParamsOpt = Some(HCCacheParameters(
        name = "L3",
        level = 3,
        ways = ways,
        sets = sets,
        inclusive = inclusive,
        clientCaches = tiles.map{ core =>
          val l2params = core.L2CacheParamsOpt.get.toCacheParams
          l2params.copy(sets = 2 * clientDirBytes / core.L2NBanks / l2params.ways / 64, ways = l2params.ways + 2)
        },
        enablePerf = !site(DebugOptionsKey).FPGAPlatform && site(DebugOptionsKey).EnablePerfDebug,
        ctrl = Some(CacheCtrl(
          address = 0x39000000,
          numCores = tiles.size
        )),
        reqField = Seq(utility.ReqSourceField()),
        sramClkDivBy2 = true,
        sramDepthDiv = 4,
        tagECC = Some("secded"),
        dataECC = Some("secded"),
        simulation = !site(DebugOptionsKey).FPGAPlatform,
        prefetch = Some(huancun.prefetch.L3PrefetchReceiverParams()),
        tpmeta = Some(huancun.prefetch.DefaultTPmetaParameters())
      )),
      OpenLLCParamsOpt = Some(OpenLLCParam(
        name = "LLC",
        ways = ways,
        sets = sets,
        banks = banks,
        fullAddressBits = 48,
        clientCaches = tiles.map { core =>
          val l2params = core.L2CacheParamsOpt.get
          l2params.copy(sets = 2 * clientDirBytes / core.L2NBanks / l2params.ways / 64, ways = l2params.ways + 2)
        }
      ))
    )
})

class WithL3DebugConfig extends Config(
  new WithNKBL3(256, inclusive = false) ++ new WithNKBL2(64)
)

class MinimalL3DebugConfig(n: Int = 1) extends Config(
  new WithL3DebugConfig ++ new MinimalConfig(n)
)

class DefaultL3DebugConfig(n: Int = 1) extends Config(
  new WithL3DebugConfig ++ new BaseConfig(n)
)

class WithFuzzer extends Config((site, here, up) => {
  case DebugOptionsKey => up(DebugOptionsKey).copy(
    EnablePerfDebug = false,
  )
  case SoCParamsKey => up(SoCParamsKey).copy(
    L3CacheParamsOpt = Some(up(SoCParamsKey).L3CacheParamsOpt.get.copy(
      enablePerf = false,
    )),
  )
  case XSTileKey => up(XSTileKey).zipWithIndex.map{ case (p, i) =>
    p.copy(
      L2CacheParamsOpt = Some(up(XSTileKey)(i).L2CacheParamsOpt.get.copy(
        enablePerf = false,
      )),
    )
  }
})

class MinimalAliasDebugConfig(n: Int = 1) extends Config(
  new WithNKBL3(512, inclusive = false) ++
    new WithNKBL2(256, inclusive = true) ++
    new WithNKBL1D(128) ++
    new MinimalConfig(n)
)

class MediumConfig(n: Int = 1) extends Config(
  new WithNKBL3(4096, inclusive = false, banks = 4)
    ++ new WithNKBL2(512, inclusive = true)
    ++ new WithNKBL1D(128)
    ++ new BaseConfig(n)
)

class FuzzConfig(dummy: Int = 0) extends Config(
  new WithFuzzer
    ++ new DefaultConfig(1)
)

class DefaultConfig(n: Int = 1) extends Config(
  new WithNKBL3(16 * 1024, inclusive = false, banks = 4, ways = 16)
    ++ new WithNKBL2(2 * 512, inclusive = true, banks = 4)
    ++ new WithNKBL1D(64, ways = 4)
    ++ new BaseConfig(n)
)

class DefaultMatrixConfig(n: Int = 1) extends Config(
  (new WithNKBL3(16 * 1024, inclusive = false, banks = 4, ways = 16)
    ++ new WithNKBL2(2 * 512, inclusive = true, banks = 4)
    ++ new WithNKBL1D(64, ways = 4)
    ++ new BaseConfig(n)).alter((site, here, up) => {
    case XSTileKey => up(XSTileKey).map(_.copy(
      l2tlbParameters = L2TLBParameters(
        dfilterSize = 48,
      ),
    ))
  })
)

class WithCHI extends Config((_, _, _) => {
  case EnableCHI => true
})

class KunminghuV2Config(n: Int = 1) extends Config(
  new WithCHI
    ++ new Config((site, here, up) => {
      case SoCParamsKey => up(SoCParamsKey).copy(L3CacheParamsOpt = None) // There will be no L3
    })
    ++ new WithNKBL2(2 * 512, inclusive = true, banks = 4, tp = false)
    ++ new WithNKBL1D(64, ways = 4)
    ++ new DefaultConfig(n)
)

class KunminghuV2MinimalConfig(n: Int = 1) extends Config(
  new WithCHI
    ++ new Config((site, here, up) => {
      case SoCParamsKey => up(SoCParamsKey).copy(L3CacheParamsOpt = None) // There will be no L3
    })
    ++ new WithNKBL2(128, inclusive = true, banks = 1, tp = false)
    ++ new WithNKBL1D(32, ways = 4)
    ++ new MinimalConfig(n)
)

class XSNoCTopConfig(n: Int = 1) extends Config(
  (new KunminghuV2Config(n)).alter((site, here, up) => {
    case SoCParamsKey => up(SoCParamsKey).copy(UseXSNoCTop = true)
  })
)

class XSNoCTopMinimalConfig(n: Int = 1) extends Config(
  (new KunminghuV2MinimalConfig(n)).alter((site, here, up) => {
    case SoCParamsKey => up(SoCParamsKey).copy(UseXSNoCTop = true)
  })
)

class FpgaDefaultConfig(n: Int = 1) extends Config(
  (new WithNKBL3(3 * 1024, inclusive = false, banks = 1, ways = 6)
    ++ new WithNKBL2(2 * 512, inclusive = true, banks = 4)
    ++ new WithNKBL1D(64, ways = 4)
    ++ new BaseConfig(n)).alter((site, here, up) => {
    case DebugOptionsKey => up(DebugOptionsKey).copy(
      AlwaysBasicDiff = false,
      AlwaysBasicDB = false
    )
    case SoCParamsKey => up(SoCParamsKey).copy(
      L3CacheParamsOpt = Some(up(SoCParamsKey).L3CacheParamsOpt.get.copy(
        sramClkDivBy2 = false,
      )),
    )
  })
)

class FpgaDiffDefaultConfig(n: Int = 1) extends Config(
  (new WithNKBL3(3 * 1024, inclusive = false, banks = 1, ways = 6)
    ++ new WithNKBL2(2 * 512, inclusive = true, banks = 4)
    ++ new WithNKBL1D(64, ways = 8)
    ++ new BaseConfig(n)).alter((site, here, up) => {
    case DebugOptionsKey => up(DebugOptionsKey).copy(
      AlwaysBasicDiff = true,
      AlwaysBasicDB = false
    )
    case SoCParamsKey => up(SoCParamsKey).copy(
      L3CacheParamsOpt = Some(up(SoCParamsKey).L3CacheParamsOpt.get.copy(
        sramClkDivBy2 = false,
      )),
    )
  })
)
