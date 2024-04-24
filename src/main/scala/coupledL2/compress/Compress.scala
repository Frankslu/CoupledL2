package coupledL2.compress

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import coupledL2._

class CompressUnit(implicit p: Parameters) extends L2Module{
  class DataOut extends Bundle {
    val data = UInt((beatBytes * 8).W)
    val compressible = Bool()
  }

  val io = IO(new Bundle() {
    val in = Flipped(DecoupledIO(UInt((blockBytes * 8).W)))
    val out = DecoupledIO(new DataOut)
  })

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

  val entry_num = blockBytes * 8 / ccEntryBits // current value is 8
  val CC_offsetWidth = log2Ceil(blockBytes * 8) + 1

  val s1_valid = WireInit(false.B)
  val s1_ready = WireInit(true.B)
  // val s2_valid = RegInit(false.B)
  // val s2_ready = WireInit(true.B)

  // stage 1
  val ccEntry = VecInit(
    Seq.tabulate(entry_num)(i => io.in.bits((i + 1) * ccEntryBits - 1, i * ccEntryBits))
  )
  val prefixOH =
    VecInit(
      ccEntry.map(entry =>
        MuxCase(patternSeq.last.prefixOH, patternSeq.init.map(pat => pat.detect(entry) -> pat.prefixOH))
      )
    )
  val compressedEntry = VecInit(prefixOH.zip(ccEntry).map {
    case (prefix, data) =>
      Mux1H {
        patternSeq.zipWithIndex.map {
          case (pat, i) => prefix(i) -> pat.compress(data)
        }
      }
  })
  val entryWidth = prefixOH.map(prefixOH =>
    Mux1H(
      patternSeq.zipWithIndex.map { case (pat, i) => prefixOH(i) -> pat.width.U }
    )
  )
  val entryOffset = Wire(Vec(entry_num, UInt(CC_offsetWidth.W)))
  val totalWidth = entryWidth.zip(entryOffset).foldLeft(0.U(CC_offsetWidth.W)) {
    case (sum, (width, offset)) =>
      offset := sum
      sum +& width
  }

  io.in.ready := s1_ready
  s1_valid := io.in.valid
  s1_ready := io.out.ready
  // when(s1_ready) {
  //   s1_valid := io.in.valid
  // }

  // val s1_prefixOH = RegEnable(prefixOH, io.in.fire)
  // val s1_compressedEntry = RegEnable(compressedEntry, io.in.fire)
  // val s1_entryOffset = RegEnable(VecInit(entryOffset), io.in.fire)
  // val s1_totalWidth = RegEnable(totalWidth, io.in.fire)
  val s1_prefixOH = prefixOH
  val s1_compressedEntry = compressedEntry
  val s1_entryOffset = entryOffset
  val s1_totalWidth = totalWidth

  // stage 2
  val compressedData: UInt = s1_compressedEntry
    .zip(s1_entryOffset)
    .map {
      case (entry, offset) =>
        entry(31, 0) << offset
    }
    .reduce(_ | _)
  val prefix = Cat(s1_prefixOH.reverse.map(prefixOH => OHToUInt(prefixOH)))

  val compressible = s1_totalWidth <= (beatBytes * 8 - ccPrefixBits).U

  // s2_ready := s2_valid & io.out.ready | ~s2_valid
  // when(s2_ready) {
  //   s2_valid := s1_valid
  // }

  // val s2_fire = s2_ready & s1_valid
  // val s2_compressedData = RegEnable(compressedData, s2_fire)
  // val s2_prefix = RegEnable(prefix, s2_fire)
  // val s2_compressible = RegEnable(compressible, s2_fire)

  io.out.bits.data := Cat(compressedData, prefix)
  io.out.bits.compressible := compressible
  io.out.valid := s1_valid
}

object CompressUnit {
  val latency = 2
}
