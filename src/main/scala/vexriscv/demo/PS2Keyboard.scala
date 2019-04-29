package vexriscv.demo

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.misc._

case class PS2Keyboard() extends Bundle with IMasterSlave {
  val ps2Clk = Bool
  val ps2Data = Bool

  override def asMaster(): Unit = {
    in(ps2Clk, ps2Data)
  }
}

case class PS2KeyboardCtrl() extends Component {
  val io = new Bundle {
    val ps2 = master(PS2Keyboard())
    val data = out Bits(8 bits)
    val valid = out Bool
    val error = out Bool
  }

  val dataOut = Reg(Bits(8 bits))
  io.data := dataOut

  val validOut = Reg(Bool) init False
  io.valid := validOut

  val errorOut = Reg(Bool) init False
  io.error := errorOut

  val ps2ClkIn = Reg(Bool) init True
  val ps2DataIn = Reg(Bool) init True
  ps2DataIn := io.ps2.ps2Data

  val clkFilter = Reg(Bits(8 bits)) init 0xff
  clkFilter := io.ps2.ps2Clk ## clkFilter(7 downto 1)

  val bitCount = Reg(UInt(4 bits)) init 0
  val shiftReg = Reg(Bits(9 bits)) init 0
  val parity = Reg(Bool) init False
  val clkEdge = Reg(Bool) init False

  clkEdge := False

  when (clkFilter === 0xFF) {
    ps2ClkIn := True
  } elsewhen (clkFilter === 0) {
    when (ps2ClkIn) {
      clkEdge := True
    }
    ps2ClkIn := False
  }

  validOut := False
  errorOut := False

  when (clkEdge) {
    when (bitCount === 0) {
      parity := False
      when (!ps2DataIn) {
        bitCount := bitCount + 1
      }
    } otherwise {
      when (bitCount < 10) {
        bitCount := bitCount + 1
        shiftReg := ps2DataIn ## shiftReg(8 downto 1)
        parity := parity ^ ps2DataIn
      } elsewhen (ps2DataIn) {
        bitCount := 0
        when (parity) {
          dataOut := shiftReg(7 downto 0)
          validOut := True
        } otherwise {
          errorOut := True
        }
      } otherwise {
        bitCount := 0
        errorOut := True
      }
    }
  }

  def driveFrom(busCtrl : BusSlaveFactory, baseAddress : Int = 0) () = new Area {
    busCtrl.read(io.data, baseAddress)
  }
}

/*
 * Data -> 0x00 Read register to read the position
 **/
case class Apb3PS2KeyboardCtrl() extends Component {
  val io = new Bundle {
    val apb = slave(Apb3(Apb3Config(addressWidth = 8, dataWidth = 32)))
    val ps2 = master(PS2Keyboard())
  }

  val busCtrl = Apb3SlaveFactory(io.apb)
  val ps2KeyboardCtrl = PS2KeyboardCtrl()
  io.ps2 <> ps2KeyboardCtrl.io.ps2

  ps2KeyboardCtrl.driveFrom(busCtrl)()
}
