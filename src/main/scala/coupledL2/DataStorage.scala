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
import coupledL2.compress.DecompressUnit
import coupledL2.utils.SRAMTemplate
import utility.RegNextN
import org.chipsalliance.cde.config.Parameters

class DSRequest(implicit p: Parameters) extends L2Bundle {
  val way = UInt(wayBits.W)
  val set = UInt(setBits.W)
  val wmask = Input(UInt(2.W))
  val rdataCompressible = Bool()
  val wen = Bool()
}

// mask not used
class DSBeat(implicit p: Parameters) extends L2Bundle {
  val data = UInt((beatBytes * 8).W)
}

class DSBlock(implicit p: Parameters) extends L2Bundle {
  val data = UInt((blockBytes * 8).W)
}

class DataStorage(implicit p: Parameters) extends L2Module {
  val io = IO(new Bundle() {
    // there is only 1 read or write request in the same cycle,
    // so only 1 req port is necessary
    val req = Flipped(ValidIO(new DSRequest))
    val rdata = Output(Vec(2, new DSBlock))
    val wdata = Input(new DSBlock)
  })

  val array = Seq.fill(2) {
    Module(new SRAMTemplate(
      gen = new DSBeat,
      set = blocks,
      way = 1,
      singlePort = true
    ))
  }
  val arrayIdx = Cat(io.req.bits.way, io.req.bits.set)

  for (i <- 0 until 2) {
    val wen = io.req.valid && io.req.bits.wen && io.req.bits.wmask(i)
    val ren = io.req.valid && !io.req.bits.wen
    // array(i).io.w.apply(wen, io.wdata, arrayIdx, 1.U)
    val wport_width = io.wdata.data.getWidth / 2
    val wport = io.wdata.data((i + 1) * wport_width - 1, i * wport_width).asTypeOf(new DSBeat)
    array(i).io.w.apply(wen, wport, arrayIdx, 1.U)
    array(i).io.r.apply(ren, arrayIdx)

    // TODO: timing: we should not use reg here, instead set this as multicycle path
    // s3 read, s4 pass and s5 to destination
  }
  val rports_s4 = array.map(_.io.r.resp.data(0).data)
  val decompressors = Seq.fill(2)(Module(new DecompressUnit()))
  decompressors.zip(rports_s4).foreach {
    case (decompressor, rport) =>
      decompressor.io.in.bits := rport
      decompressor.io.in.valid := DontCare
      decompressor.io.out.ready := DontCare
  }
  val rdata_s5 = RegNextN(Cat(rports_s4.reverse), 1)
  val decompressedData_s5 = RegNextN(VecInit(decompressors.map(_.io.out.bits)), 1)

  io.rdata.zip(decompressedData_s5).foreach {
    case (r, dec) =>
      r.data := Mux(RegNextN(io.req.bits.rdataCompressible, 2), dec, rdata_s5)
  }
  // io.rdata.data(1,0) := RegNextN(array(i).io.r.resp.data(0), 1)
}
