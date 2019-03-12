package vexriscv.demo

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.misc._

case class Servo() extends Bundle with IMasterSlave {
  val pin = Bool

  override def asMaster(): Unit = {
    out(pin)
  }
}

case class ServoCtrl() extends Component {
  val io = new Bundle {
    val servo = master(Servo())
    val pulseMicros = in UInt(12 bits)
  }

  val clockMhz = 50
  val counter = Reg(UInt(6 bits))
  val micros = Reg(UInt(15 bits))

  counter := counter + 1

  when (counter === (clockMhz - 1)) {
    micros := micros + 1
    counter := 0
  }

  io.servo.pin := (micros < io.pulseMicros && io.pulseMicros =/= 0)

  when (micros === 19999) {
    micros := 0
  }

  def driveFrom(busCtrl : BusSlaveFactory, baseAddress : Int = 0) () = new Area {
    busCtrl.drive(io.pulseMicros, baseAddress)
  }
}

/*
 * pulseMicros -> 0x00 Write register to set pulse width in micro seconds
 **/
case class Apb3ServoCtrl() extends Component {
  val io = new Bundle {
    val apb = slave(Apb3(Apb3Config(addressWidth = 8, dataWidth = 32)))
    val servo = master(Servo())
  }

  val busCtrl = Apb3SlaveFactory(io.apb)
  val servoCtrl = ServoCtrl()
  io.servo <> servoCtrl.io.servo

  servoCtrl.driveFrom(busCtrl)()
}
