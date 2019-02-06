package vexriscv.demo

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.misc._

case class Pwm() extends Bundle with IMasterSlave {
  val pin = Bool

  override def asMaster(): Unit = {
    out(pin)
  }
}

case class PwmCtrl() extends Component {
  val io = new Bundle {
    val pwm = master(Pwm())
    val duty = in UInt(8 bits)
  }

  val counter = Reg(UInt(8 bits))

  counter := counter + 1

  io.pwm.pin := (counter <= io.duty) && !(io.duty === 0)

  def driveFrom(busCtrl : BusSlaveFactory, baseAddress : Int = 0) () = new Area {
    busCtrl.drive(io.duty, baseAddress)
  }
}

/*
 * Duty -> 0x00 Write register to set the duty cycle value
 **/
case class Apb3PwmCtrl() extends Component {
  val io = new Bundle {
    val apb = slave(Apb3(Apb3Config(addressWidth = 8, dataWidth = 32)))
    val pwm = master(Pwm())
  }

  val busCtrl = Apb3SlaveFactory(io.apb)
  val pwmCtrl = PwmCtrl()
  io.pwm <> pwmCtrl.io.pwm

  pwmCtrl.driveFrom(busCtrl)()
}
