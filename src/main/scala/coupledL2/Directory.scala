/** *************************************************************************************
  * Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
  * Copyright (c) 2020-2021 Peng Cheng Laboratory
  *
  * XiangShan is licensed under Mulan PSL v2.
  * You can use this software according to the terms and conditions of the Mulan PSL v2.
  * You may obtain a copy of Mulan PSL v2 at:
  * http://license.coscl.org.cn/MulanPSL2
  *
  * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
  * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
  * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
  *
  * See the Mulan PSL v2 for more details.
  * *************************************************************************************
  */

package coupledL2

import chisel3._
import chisel3.util._
import freechips.rocketchip.util.SetAssocLRU
import coupledL2.utils._
import utility.{ParallelPriorityMux, RegNextN}
import org.chipsalliance.cde.config.Parameters
import coupledL2.prefetch.PfSource
import freechips.rocketchip.tilelink.TLMessages._
import coupledL2.MetaData._
import freechips.rocketchip.util.DontTouch

class MetaEntry(implicit p: Parameters) extends L2Bundle {
  val dirty = Bool()
  val state = UInt(stateBits.W)
  val clients = UInt(clientBits.W)  // valid-bit of clients
  // TODO: record specific state of clients instead of just 1-bit
  val alias = aliasBitsOpt.map(width => UInt(width.W)) // alias bits of client
  val prefetch = if (hasPrefetchBit) Some(Bool()) else None // whether block is prefetched
  val prefetchSrc = if (hasPrefetchSrc) Some(UInt(PfSource.pfSourceBits.W)) else None // prefetch source
  val accessed = Bool()
  val compressed = Bool()

  def =/=(entry: MetaEntry): Bool = {
    this.asUInt =/= entry.asUInt
  }
}

object
MetaEntry {
  def apply()(implicit p: Parameters) = {
    val init = WireInit(0.U.asTypeOf(new MetaEntry))
    init
  }
  def apply(dirty: Bool, state: UInt, clients: UInt, alias: Option[UInt], prefetch: Bool = false.B,
            pfsrc: UInt = PfSource.NoWhere.id.U, accessed: Bool = false.B, compressed: Bool = false.B
  )(implicit p: Parameters) = {
    val entry = Wire(new MetaEntry)
    entry.dirty := dirty
    entry.state := state
    entry.clients := clients
    entry.alias.foreach(_ := alias.getOrElse(0.U))
    entry.prefetch.foreach(_ := prefetch)
    entry.prefetchSrc.foreach(_ := pfsrc)
    entry.accessed := accessed
    entry.compressed := compressed
    entry
  }
}

class DirRead(implicit p: Parameters) extends L2Bundle {
  val tag = UInt(tagBits.W)
  val set = UInt(setBits.W)
  // dirResult.way must only be in the wayMask
  // TODO: change wayMask to 8-bits
  val wayMask = UInt(cacheParams.ways.W)
  val replacerInfo = new ReplacerInfo()
  // dirRead when refill
  val refill = Bool()
  val mshrId = UInt(mshrBits.W)
}

class DirResult(implicit p: Parameters) extends L2Bundle {
  val hit = Bool()
  val tag = Vec(2, UInt(tagBits.W))
  // val ucTag = Vec(2, UInt(tagBits.W))
  val set = UInt(setBits.W)
  val way = UInt((wayBits + 1).W)  // hit way or victim way
  // val ucWay = UInt(wayBits.W)
  val meta = Vec(2, new MetaEntry())
  // val ucMeta = Vec(2, new MetaEntry())
  val error = Bool()
  val replacerInfo = new ReplacerInfo() // for TopDown usage
}

class ReplacerResult(implicit p: Parameters) extends L2Bundle {
  val tag = Vec(2, UInt(tagBits.W))
  val set = UInt(setBits.W)
  val way = UInt((wayBits + 1).W)
  val meta = Vec(2, new MetaEntry())
  val mshrId = UInt(mshrBits.W)
  val retry = Bool()
  val releaseTask = UInt(2.W)
}

class MetaWrite(implicit p: Parameters) extends L2Bundle {
  val set = UInt(setBits.W)
  val wayOH = UInt(cacheParams.ways.W)
  val wmeta = new MetaEntry
}

class TagWrite(implicit p: Parameters) extends L2Bundle {
  val set = UInt(setBits.W)
  val way = UInt(wayBits.W)
  val wtag = UInt(tagBits.W)
}

class Directory(implicit p: Parameters) extends L2Module {

  val io = IO(new Bundle() {
    val read = Flipped(DecoupledIO(new DirRead))
    val fromMainPipe_s3 = new Bundle() {
      val wdataCompressible = Input(Bool())
    }
    val resp = ValidIO(new DirResult())
    val metaWReq = Flipped(Vec(2, ValidIO(new MetaWrite)))
    val tagWReq = Flipped(Vec(2, ValidIO(new TagWrite)))
    val replResp = ValidIO(new ReplacerResult())
    // used to count occWays for Grant to retry
    val msInfo = Vec(mshrsAll, Flipped(ValidIO(new MSHRInfo)))
  })

  def invalid_way_sel(metaVec: Seq[MetaEntry], repl: UInt) = {
    // group (1,2,3,4,5,6,7,8) to ((1,2,3,4),(5,6,7,8))
    val invalid_vec = metaVec.map(_.state === MetaData.INVALID).grouped(metaVec.size / 2).toSeq
    val uc_invalid_vec = invalid_vec(0).zip(invalid_vec(1)).map(x => x._1 & x._2)
    val uc_has_invalid_way = Cat(uc_invalid_vec).orR
    val ucWay = ParallelPriorityMux(uc_invalid_vec.zipWithIndex.map(x => x._1 -> x._2.U(wayBits.W)))
    // val ucWay = ParallelPriorityMux(uc_invalid_vec.zipWithIndex.map(x => x._1 -> x._2.U(wayBits.W)))
    // if two sub line is both invalid, we tend to refill uncompressible data in it
    // Seq(x & ~y, y & ~x) means:
    // 0, 0 -> (0, 0)
    // 1, 0 -> (1, 0)
    // 0, 1 -> (0, 1)
    // 1, 1 -> (0, 0), which is the case that two sub line is both invalid
    val cc_invalid_vec_tmp: Seq[Seq[Bool]] = invalid_vec(0).zip(invalid_vec(1)).map {
      case (x, y) => Seq(x & ~y, y & ~x)
    }
    val cc_invalid_vec = cc_invalid_vec_tmp.map(_(0)) ++ cc_invalid_vec_tmp.map(_(1))
    val cc_has_invalid_way = Cat(cc_invalid_vec).orR
    val ccWay = Mux(cc_has_invalid_way,
      ParallelPriorityMux(cc_invalid_vec.zipWithIndex.map(x => x._1 -> x._2.U((wayBits + 1).W))),
      ucWay
    )
    (cc_has_invalid_way | uc_has_invalid_way, ccWay, uc_has_invalid_way, ucWay)
  }

  val sets = cacheParams.sets
  val ways = cacheParams.ways

  val tagWen  = io.tagWReq.map(_.valid)
  val metaWen = io.metaWReq.map(_.valid)
  val replacerWen = WireInit(false.B)

  val tagArray  = Seq.fill(2)(Module(new SRAMTemplate(UInt(tagBits.W), sets, ways, singlePort = true)))
  val metaArray = Seq.fill(2)(Module(new SRAMTemplate(new MetaEntry, sets, ways, singlePort = true)))

  val resetFinish = RegInit(false.B)
  val resetIdx = RegInit((sets - 1).U)

  // Replacer
  val repl = ReplacementPolicy.fromString(cacheParams.replacement, ways * 2)
  val random_repl = cacheParams.replacement == "random"
  val replacer_sram_opt = if(random_repl) None else
    Some(Module(new SRAMTemplate(UInt(repl.nBits.W), sets, 1, singlePort = true, shouldReset = true)))

  /* ====== Generate response signals ====== */
  // hit/way calculation in stage 3, Cuz SRAM latency is high under high frequency
  /* stage 1: io.read.fire, access Tag/Meta
     stage 2: get Tag/Meta, latch
     stage 3: calculate hit/way and chosen meta/tag by way
  */
  val reqValid_s2 = RegNext(io.read.fire, false.B)
  val reqValid_s3 = RegNext(reqValid_s2, false.B)
  val req_s2 = RegEnable(io.read.bits, 0.U.asTypeOf(io.read.bits), io.read.fire)
  val req_s3 = RegEnable(req_s2, 0.U.asTypeOf(req_s2), reqValid_s2)

  val refillReqValid_s2 = RegNext(io.read.fire && io.read.bits.refill, false.B)
  val refillReqValid_s3 = RegNext(refillReqValid_s2, false.B)

  // Tag/Meta R
  val tagRead = VecInit(tagArray.map(_.io.r(io.read.fire, io.read.bits.set).resp.data).flatten)
  val metaRead = VecInit(metaArray.map(_.io.r(io.read.fire, io.read.bits.set).resp.data).flatten)
  // Tag/Meta W
  for (i <- 0 until 2) {
    tagArray(i).io.w(
      tagWen(i),
      io.tagWReq(i).bits.wtag,
      io.tagWReq(i).bits.set,
      UIntToOH(io.tagWReq(i).bits.way)
    )
    metaArray(i).io.w(
      metaWen(i),
      io.metaWReq(i).bits.wmeta,
      io.metaWReq(i).bits.set,
      io.metaWReq(i).bits.wayOH
    )
  }

  val metaAll_s3 = RegEnable(metaRead, 0.U.asTypeOf(metaRead), reqValid_s2)
  val tagAll_s3 = RegEnable(tagRead, 0.U.asTypeOf(tagRead), reqValid_s2)

  val tagMatchVec = tagAll_s3.map(_ (tagBits - 1, 0) === req_s3.tag)
  val metaValidVec = metaAll_s3.map(_.state =/= MetaData.INVALID)
  val hitVec = tagMatchVec.zip(metaValidVec).map(x => x._1 && x._2)
  val (hitVec_l, hitVec_h) = hitVec.splitAt(hitVec.length / 2)

  val hitWay_l = OHToUInt(hitVec_l)
  val hitLow = Cat(hitVec_l).orR
  val hitWay = Mux(hitLow, hitWay_l, OHToUInt(hitVec))
  val replaceWay = WireInit(UInt((wayBits + 1).W), 0.U)
  val (ccInv, ccInvalidWay, ucInv, ucInvalidWay) = invalid_way_sel(metaAll_s3, replaceWay)
  val chosenCcWay = Mux(ccInv, ccInvalidWay, replaceWay)
  val chosenUcWay = Mux(ucInv, ucInvalidWay, replaceWay)(wayBits - 1, 0)
  // if chosenWay not in wayMask, then choose a way in wayMask
  // TODO: consider remove this is not used for better timing
  // for retry bug fixing: if the chosenway cause retry last time, choose another way
  // TODO: wayMask need to be 8 bits
  val doubleWayMask = Fill(2, req_s3.wayMask)
  val ccFinalWay = Mux(
    doubleWayMask(chosenCcWay),
    chosenCcWay,
    PriorityEncoder(req_s3.wayMask)
  )(wayBits, 0)
  val ucFinalWay = Mux(
    doubleWayMask(Cat("b0".U, chosenUcWay)) && doubleWayMask(Cat("b1".U, chosenUcWay)),
    chosenUcWay,
    PriorityEncoder(req_s3.wayMask)
  )(wayBits - 1, 0)
  val finalWay = Mux(io.fromMainPipe_s3.wdataCompressible, ccFinalWay, ucFinalWay)

  val hit_s3 = Cat(hitVec).orR
//  val ccMeta_s3 = metaAll_s3(hitWay)
  val way_s3 = Mux(hit_s3, hitWay, finalWay)

  def l(way: UInt) = Cat("b0".U, way(wayBits - 1, 0))
  def r(way: UInt) = Cat("b1".U, way(wayBits - 1, 0))

  val meta_s3 = VecInit(metaAll_s3(l(way_s3)), metaAll_s3(r(way_s3)))
  val tag_s3 = VecInit(tagAll_s3(l(way_s3)), tagAll_s3(r(way_s3)))
  val set_s3 = req_s3.set
  val replacerInfo_s3 = req_s3.replacerInfo

  io.resp.valid      := reqValid_s3
  io.resp.bits.hit   := hit_s3
  io.resp.bits.way   := hitWay
  io.resp.bits.meta  := meta_s3
  io.resp.bits.tag   := tag_s3
  io.resp.bits.set   := set_s3
  io.resp.bits.error := false.B  // depends on ECC
  io.resp.bits.replacerInfo := replacerInfo_s3
  when (RegNext((tag_s3(0) =/= tag_s3(1)) && meta_s3.map(_.state =/= INVALID).reduce(_ && _))) {
    assert(RegNext(meta_s3(0).compressed === true.B && meta_s3(1).compressed === true.B),
      "there can't be two valid uncompressed way in a block")
  }
  when (RegNext((tag_s3(0) === tag_s3(1)) && meta_s3.map(_.state =/= INVALID).reduce(_ && _))) {
    assert(RegNext(meta_s3(0).compressed === false.B && meta_s3(1).compressed === false.B),
      "err2")
  }
  when (RegNext(meta_s3(0).state === INVALID && meta_s3(1).state =/= INVALID)) {
    assert(RegNext(meta_s3(1).compressed === true.B))
  }
  when (RegNext(meta_s3(1).state === INVALID && meta_s3(0).state =/= INVALID)) {
    assert(RegNext(meta_s3(0).compressed === true.B))
  }

  dontTouch(io)
  metaArray.foreach(meta => dontTouch(meta.io))
  tagArray.foreach(tag => dontTouch(tag.io))

  io.read.ready := !io.metaWReq.map(_.valid).reduce(_ | _) &&
    !io.tagWReq.map(_.valid).reduce(_ | _) &&
    !replacerWen

  /* ====== refill retry ====== */
  // if refill chooses a way that has not finished writing its refillData back to DS (in MSHR Release),
  // or the way is using by Alias-Acquire (hit), we cancel the Grant and LET IT RETRY

  // comparing set is done at Stage2 for better timing
  val wayConflictPartI  = RegEnable(VecInit(io.msInfo.zipWithIndex.map{case(s,i) =>
    s.valid && s.bits.set === req_s2.set && req_s2.mshrId =/= i.U}).asUInt, refillReqValid_s2)

  val wayConflictPartII = VecInit(io.msInfo.map(s =>
    (s.bits.blockRefill || s.bits.dirHit) && s.bits.way(wayBits - 1, 0) === finalWay(wayBits - 1, 0)
  )).asUInt
  val refillRetry = (wayConflictPartI & wayConflictPartII).orR
  val wayConflict = wayConflictPartI & wayConflictPartII; dontTouch(wayConflict)

  /* ======!! Replacement logic !!====== */
  /* ====== Read, choose replaceWay ====== */
  val repl_state_s3 = if(random_repl) {
    when(io.tagWReq.map(_.fire).reduce(_ | _)){
      repl.miss
    }
    0.U
  } else {
    val repl_sram_r = replacer_sram_opt.get.io.r(io.read.fire, io.read.bits.set).resp.data(0)
    val repl_state = RegEnable(repl_sram_r, 0.U(repl.nBits.W), reqValid_s2)
    repl_state
  }

  replaceWay := repl.get_replace_way(repl_state_s3)

  io.replResp.valid := refillReqValid_s3
  io.replResp.bits.tag := VecInit(tagAll_s3(l(finalWay)), tagAll_s3(r(finalWay)))
  io.replResp.bits.set := req_s3.set
  io.replResp.bits.way := finalWay
  val replMeta = VecInit(metaAll_s3(l(finalWay)), metaAll_s3(r(finalWay)))
  when (RegNext((io.replResp.bits.tag(0) =/= io.replResp.bits.tag(1)) && replMeta.map(_.state =/= INVALID).reduce(_ && _))) {
    assert(RegNext(replMeta(0).compressed === true.B && replMeta(1).compressed === true.B),
      "there can't be two valid uncompressed way in a block")
  }
  when (RegNext((io.replResp.bits.tag(0) === io.replResp.bits.tag(1)) && replMeta.map(_.state =/= INVALID).reduce(_ && _))) {
    assert(RegNext(replMeta(0).compressed === false.B && replMeta(1).compressed === false.B),
      "err2")
  }
  when (RegNext(replMeta(0).state === INVALID && replMeta(1).state =/= INVALID)) {
    assert(RegNext(replMeta(1).compressed === true.B))
  }
  when (RegNext(replMeta(1).state === INVALID && replMeta(0).state =/= INVALID)) {
    assert(RegNext(replMeta(0).compressed === true.B))
  }
  io.replResp.bits.meta := replMeta
  io.replResp.bits.mshrId := req_s3.mshrId
  io.replResp.bits.retry := refillRetry
  val replSide = finalWay(wayBits)
  val ccReleaseTask = Mux(
    replMeta(replSide).state === INVALID,
    "b00".U,
    Mux(replMeta(replSide).compressed, UIntToOH(replSide, 2), "b01".U)
  )
  val ucReleaseTask = Mux(!replMeta(0).compressed && replMeta(0).state =/= INVALID,
    "b01".U,
    Cat(Seq.tabulate(2)(i => replMeta(i).state =/= INVALID).reverse)
  )
  val releaseTask = Mux(io.fromMainPipe_s3.wdataCompressible, ccReleaseTask, ucReleaseTask)
  io.replResp.bits.releaseTask := releaseTask

  /* ====== Update ====== */
  // update replacer only when A hit or refill, at stage 3
  val updateHit = reqValid_s3 && hit_s3 && req_s3.replacerInfo.channel(0) &&
    (req_s3.replacerInfo.opcode === AcquirePerm || req_s3.replacerInfo.opcode === AcquireBlock)
  val updateRefill = refillReqValid_s3 && !refillRetry
  replacerWen := updateHit || updateRefill

  // !!![TODO]!!! check this @CLS
  // hit-Promotion, miss-Insertion for RRIP, so refill should hit = false.B
  val touch_way_s3 = Mux(refillReqValid_s3, replaceWay, way_s3)
  val rrip_hit_s3 = Mux(refillReqValid_s3, false.B, hit_s3)

  if(cacheParams.replacement == "srrip"){
    val next_state_s3 = repl.get_next_state(repl_state_s3, touch_way_s3, rrip_hit_s3)
    val repl_init = Wire(Vec(ways, UInt(2.W)))
    repl_init.foreach(_ := 2.U(2.W))
    replacer_sram_opt.get.io.w(
      !resetFinish || replacerWen,
      Mux(resetFinish, next_state_s3, repl_init.asUInt),
      Mux(resetFinish, set_s3, resetIdx),
      1.U
    )
  } else if(cacheParams.replacement == "drrip"){
    //Set Dueling
    val PSEL = RegInit(512.U(10.W)) //32-monitor sets, 10-bits psel
    // track monitor sets' hit rate for each policy: srrip-0,128...3968;brrip-64,192...4032
    when(refillReqValid_s3 && (set_s3(6,0)===0.U) && !rrip_hit_s3){  //SDMs_srrip miss
      PSEL := PSEL + 1.U
    } .elsewhen(refillReqValid_s3 && (set_s3(6,0)===64.U) && !rrip_hit_s3){ //SDMs_brrip miss
      PSEL := PSEL - 1.U
    }
    // decide use which policy by policy selection counter, for insertion
    /* if set -> SDMs: use fix policy
       else if PSEL(MSB)==0: use srrip
       else if PSEL(MSB)==1: use brrip */
    val repl_type = WireInit(false.B)
    repl_type := Mux(set_s3(6,0)===0.U, false.B,
      Mux(set_s3(6,0)===64.U, true.B,
        Mux(PSEL(9)===0.U, false.B, true.B)))    // false.B - srrip, true.B - brrip
    val next_state_s3 = repl.get_next_state(repl_state_s3, touch_way_s3, rrip_hit_s3, repl_type)

    val repl_init = Wire(Vec(ways, UInt(2.W)))
    repl_init.foreach(_ := 2.U(2.W))
    replacer_sram_opt.get.io.w(
      !resetFinish || replacerWen,
      Mux(resetFinish, next_state_s3, repl_init.asUInt),
      Mux(resetFinish, set_s3, resetIdx),
      1.U
    )
  } else {
    val next_state_s3 = repl.get_next_state(repl_state_s3, touch_way_s3)
    replacer_sram_opt.get.io.w(
      !resetFinish || replacerWen,
      Mux(resetFinish, next_state_s3, 0.U),
      Mux(resetFinish, set_s3, resetIdx),
      1.U
    )
  }

  /* ====== Reset ====== */
  when(resetIdx === 0.U) {
    resetFinish := true.B
  }
  when(!resetFinish) {
    resetIdx := resetIdx - 1.U
  }

  XSPerfAccumulate(cacheParams, "dirRead_cnt", io.read.fire)
  XSPerfAccumulate(cacheParams, "choose_busy_way", reqValid_s3 && !doubleWayMask(chosenCcWay))
}
