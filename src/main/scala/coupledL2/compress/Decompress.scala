package coupledL2.compress

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import utility.RegNextN
import coupledL2._

class DecompressUnit(implicit p: Parameters) extends L2Module with CCParameters {
  val io = IO(new Bundle() {
    val in = Flipped(DecoupledIO(UInt((beatBytes * 8).W)))
    val out = DecoupledIO(UInt((blockBytes * 8).W))
  })

  val entry_num = blockBytes * 8 / ccEntryBits
  val CC_offsetWidth = log2Ceil(blockBytes * 8) + 1

  /** The order has been arranged according to the prefix */
  val patternSeq = Seq(
    ZeroRun,
    FourbitSignExt,
    OneByteSignExt,
    HalfWordSignExt,
    PadHalfZero,
    TwoSignExt,
    RepeatedBytes,
    UnCompressed
  ).sortBy(_.prefix.litValue)

  val s1_valid = RegInit(false.B)
  val s1_ready = WireInit(true.B)
  // val s2_valid = RegInit(false.B)
  // val s2_ready = WireInit(true.B)

  // stage 1
  val prefix =
    Seq.tabulate(entry_num)(i => io.in.bits((i + 1) * singleEntryPrefixBits - 1, i * singleEntryPrefixBits))
  val prefixOH = VecInit(prefix.map(UIntToOH(_, CC_offsetWidth)))
  val compressedData = io.in.bits(beatBytes * 8 - 1, ccPrefixBits)

  val entryWidth = VecInit(
    prefixOH.map(prefixOH =>
      Mux1H(
        patternSeq.zipWithIndex.map { case (pat, i) => prefixOH(i) -> pat.width.U }
      )
    )
  )
  val entryOffset = Seq.fill(entry_num)(Wire(UInt(CC_offsetWidth.W)))
  entryOffset.zip(entryWidth).foldLeft(0.U(CC_offsetWidth.W)) {
    case (sum, (offset, width)) =>
      offset := sum
      sum +& width
  }

  io.in.ready := s1_ready
  s1_ready := s1_valid & io.out.ready | ~s1_valid
  when(s1_ready) {
    s1_valid := io.in.valid
  }

  val s1_prefixOH = RegEnable(prefixOH, io.in.fire)
  val s1_entryOffset = RegEnable(VecInit(entryOffset), io.in.fire)
  val s1_compressedData = RegEnable(compressedData, io.in.fire)

  // stage 2
  val shifter = s1_entryOffset.map(offset => (s1_compressedData >> offset)(ccEntryBits - 1, 0))
  val decompressedData = VecInit(shifter.zip(s1_prefixOH).map {
    case (data, prefixOH) =>
      Mux1H(
        patternSeq.zipWithIndex.map {
          case (pat, i) => (prefixOH(i) -> pat.decompress(data))
        }
      )
  })

  // s2_ready := s2_valid & io.out.ready | ~s2_valid
  // when(s2_ready) {
  //   s2_valid := s1_valid
  // }

  // val s2_fire = s2_ready & s1_valid
  // val s2_decompressedData = RegEnable(decompressedData, s2_fire)

  io.out.bits := Cat(decompressedData.reverse)
  io.out.valid := s1_valid
}

object DecompressUnit {
  val latency = 2
}
