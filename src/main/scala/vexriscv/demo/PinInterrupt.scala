package vexriscv.demo

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.misc._

case class PinInterrupt() extends Bundle with IMasterSlave {
  val pin = Bool

  override def asMaster(): Unit = {
    in(pin)
  }
}

case class PinInterruptCtrl() extends Component {
  val io = new Bundle {
    val pinInterrupt = master(PinInterrupt())
    val rising = in Bool
    val falling = in Bool
    val interrupt = out Bool
  }

  io.interrupt := False

  io.interrupt setWhen(io.rising & io.pinInterrupt.pin.rise() ||
                       io.falling & io.pinInterrupt.pin.fall())

  def driveFrom(busCtrl : BusSlaveFactory, baseAddress : Int = 0) () = new Area {
    busCtrl.driveAndRead(io.rising, baseAddress, bitOffset = 0)
    busCtrl.driveAndRead(io.falling, baseAddress, bitOffset = 1)
  }
}

/*
 * Config -> 0x00 Write register to set condition for interrupt, bit 0 = rising, 1 = falling
 **/
case class Apb3PinInterruptCtrl() extends Component {
  val io = new Bundle {
    val apb = slave(Apb3(Apb3Config(addressWidth = 8, dataWidth = 32)))
    val pinInterrupt = master(PinInterrupt())
    val interrupt = out Bool
  }

  val busCtrl = Apb3SlaveFactory(io.apb)
  val pinInterruptCtrl = PinInterruptCtrl()
  io.pinInterrupt <> pinInterruptCtrl.io.pinInterrupt
  io.interrupt <> pinInterruptCtrl.io.interrupt

  pinInterruptCtrl.driveFrom(busCtrl)()
}
