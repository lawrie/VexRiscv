package vexriscv.demo

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.misc._

case class SevenSegment() extends Bundle with IMasterSlave {
  val digitPin = Bool
  val segPins = Bits(7 bits)

  override def asMaster(): Unit = {
    out(digitPin)
    out(segPins)
  }
}

case class SevenSegmentCtrl() extends Component {
  val io = new Bundle {
    val sevenSegment = master(SevenSegment())
    val value = in UInt(8 bits)
  }

  val segROM = Vec(Bits(7 bits), 16)

  segROM(0x0) := "1111110"
  segROM(0x1) := "0110000"
  segROM(0x2) := "1101101"
  segROM(0x3) := "1111001"
  segROM(0x4) := "0110011"
  segROM(0x5) := "1011011"
  segROM(0x6) := "1011111"
  segROM(0x7) := "1110000"
  segROM(0x8) := "1111111"
  segROM(0x9) := "1111011"
  segROM(0xa) := "1110111"
  segROM(0xb) := "0011111"
  segROM(0xc) := "1101110"
  segROM(0xd) := "0111101"
  segROM(0xe) := "1001111"
  segROM(0xf) := "1000111"

  val prescaler = Reg(UInt(24 bits))
  val digPos = Reg(Bool)
  val segOut = Reg(Bits(7 bits))

  io.sevenSegment.digitPin := digPos
  io.sevenSegment.segPins := segOut

  prescaler := prescaler + 1

  when (prescaler === 50000) {
    prescaler := 0;
    digPos := !digPos

    segOut := segROM(digPos ? io.value(3 downto 0) | io.value(7 downto 4))
  }


  def driveFrom(busCtrl : BusSlaveFactory, baseAddress : Int = 0) () = new Area {
    busCtrl.drive(io.value, baseAddress)
  }
}

/*
 * Value    -> 0x00 Write register to set the hex value of the byte to display
 **/
case class Apb3SevenSegmentCtrl() extends Component {
  val io = new Bundle {
    val apb = slave(Apb3(Apb3Config(addressWidth = 8, dataWidth = 32)))
    val sevenSegment = master(SevenSegment())
  }

  val busCtrl = Apb3SlaveFactory(io.apb)
  val sevenSegmentCtrl = SevenSegmentCtrl()
  io.sevenSegment <> sevenSegmentCtrl.io.sevenSegment

  sevenSegmentCtrl.driveFrom(busCtrl)()
}
