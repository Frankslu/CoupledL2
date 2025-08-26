package coupledL2.wpu

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import utility.{RegNextN, XSPerfAccumulate}
import coupledL2._


class WPUWrapper(wpuParam: WPUParameters, updLatency: Int)(implicit p:Parameters) extends L2Module {
  val wpu = AlgoWPUMap(wpuParam)
  val in = IO(Input(new Bundle {
    val fromMPS1 = ValidIO(new WPURead())
    val fromMPS3 = ValidIO(new WPUUpdate())
  }))
  val out = IO(Output(new Bundle {
    val toMPS1 = ValidIO(new WPUResult())
  }))

  wpu.in.read := in.fromMPS1
  wpu.in.upd := RegNextN(in.fromMPS3, updLatency)
  out.toMPS1 := wpu.out.res

  // Perf
  val algo_name = wpuParam.algoName

  val s1_valid = in.fromMPS1.valid
  val s3_valid = in.fromMPS3.valid
  val s3_upd = in.fromMPS3.bits
  val pred_valid = s3_valid & !s3_upd.isEvict & !s3_upd.isReplace
  val predhit_succ = pred_valid & s3_upd.actualHit & s3_upd.predHit & s3_upd.actualWay === s3_upd.predWay
  val predmiss_succ = pred_valid & !s3_upd.actualHit & !s3_upd.predHit
  val pred_succ = predhit_succ | predmiss_succ
  val pred_unmatch = pred_valid & s3_upd.actualHit & s3_upd.predHit & s3_upd.actualWay =/= s3_upd.predWay
  val pred_miss_but_hit = pred_valid & !s3_upd.predHit & s3_upd.actualHit
  val pred_hit_but_miss = pred_valid & s3_upd.predHit & !s3_upd.actualHit
  val repl = s3_valid & s3_upd.isReplace
  val evict = s3_valid & s3_upd.isEvict
//  assert(RegNextN(s1_valid, 2, Some(false.B)) === (s3_valid & !s3_upd.isEvict & !s3_upd.isReplace), "always update after predict")
  XSPerfAccumulate(s"${algo_name}WPU_pred_times", pred_valid)
  XSPerfAccumulate(s"${algo_name}WPU_pred_succ", pred_succ)
  XSPerfAccumulate(s"${algo_name}WPU_predhit_succ", predhit_succ)
  XSPerfAccumulate(s"${algo_name}WPU_predmiss_succ", predmiss_succ)
  XSPerfAccumulate(s"${algo_name}WPU_unmatch", pred_unmatch)
  XSPerfAccumulate(s"${algo_name}WPU_pmiss_ahit", pred_miss_but_hit)
  XSPerfAccumulate(s"${algo_name}WPU_phit_amiss", pred_hit_but_miss)
  XSPerfAccumulate(s"${algo_name}WPU_repl", repl)
  XSPerfAccumulate(s"${algo_name}WPU_evict", evict)

  DebugAccumulate(s3_valid, "s3_valid")
  DebugAccumulate(pred_valid, "pred_valid")
  DebugAccumulate(pred_succ, "pred_succ")
  DebugAccumulate(predhit_succ, "predhit_succ")
  DebugAccumulate(predmiss_succ, "predmiss_succ")
  DebugAccumulate(pred_unmatch, "pred_unmatch")
  DebugAccumulate(pred_miss_but_hit, "pred_miss_but_hit")
  DebugAccumulate(pred_hit_but_miss, "pred_hit_but_miss")
  DebugAccumulate(repl, "repl")
  DebugAccumulate(evict, "evict")
}
