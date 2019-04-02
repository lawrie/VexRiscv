package vexriscv.demo

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.misc._
import spinal.lib.misc.InterruptCtrl

case class PinInterrupt(width: Int) extends Bundle with IMasterSlave {
  val pins = Bits(width bits)

  override def asMaster(): Unit = {
    in(pins)
  }
}

case class PinInterruptCtrl(width: Int) extends Component {
  val io = new Bundle {
    val pinInterrupt = master(PinInterrupt(width))
    val rising = in Bits(width bits)
    val falling = in Bits(width bits)
    val interrupt = out Bits(width bits)
  }

  for(i <- 0 until width) {
    io.interrupt(i) := False

    io.interrupt(i) setWhen(io.rising(i) & io.pinInterrupt.pins(i).rise() ||
                       io.falling(i) & io.pinInterrupt.pins(i).fall())
  }

  def driveFrom(busCtrl : BusSlaveFactory, baseAddress : Int = 0) () = new Area {
    busCtrl.driveAndRead(io.rising, baseAddress)
    busCtrl.driveAndRead(io.falling, baseAddress + 4)
  }
}

/*
 * Rising -> 0x00 Write register to set rising interrupt
 * Falling -> 0x00 Write register to set falling interrupt
 **/
case class Apb3PinInterruptCtrl(width: Int) extends Component {
  val io = new Bundle {
    val apb = slave(Apb3(Apb3Config(addressWidth = 8, dataWidth = 32)))
    val pinInterrupt = master(PinInterrupt(width))
    val interrupt = out Bool
  }

  val busCtrl = Apb3SlaveFactory(io.apb)
  val pinInterruptCtrl = PinInterruptCtrl(width)
  val interruptCtrl = InterruptCtrl(width)
  val interruptCtrlBridge = interruptCtrl.driveFrom(busCtrl,0x10)
  
  for(i <- 0 until width) {
    interruptCtrl.io.inputs(i) := pinInterruptCtrl.io.interrupt(i)
  }

  io.interrupt := interruptCtrl.io.pendings.orR
  io.pinInterrupt <> pinInterruptCtrl.io.pinInterrupt

  pinInterruptCtrl.driveFrom(busCtrl)
}
