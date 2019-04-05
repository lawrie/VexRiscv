package vexriscv.demo

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.misc._

case class Servo(width: Int) extends Bundle with IMasterSlave {
  val pins = Bits(width bits)

  override def asMaster(): Unit = {
    out(pins)
  }
}

case class ServoCtrl(width: Int) extends Component {
  val io = new Bundle {
    val servo = master(Servo(width))
    val pulseMicros = in Vec(UInt(12 bits), width)
  }

  val clockMhz = 50
  val counter = Reg(UInt(6 bits))
  val micros = Reg(UInt(15 bits))

  counter := counter + 1

  when (counter === (clockMhz - 1)) {
    micros := micros + 1
    counter := 0
  }

  for(i <- 0 until width) {
    io.servo.pins(i) := (micros < io.pulseMicros(i) && io.pulseMicros(i) =/= 0)
  }

  when (micros === 19999) {
    micros := 0
  }

  def driveFrom(busCtrl : BusSlaveFactory, baseAddress : Int = 0) () = new Area {
    for (i <- 0 until width) {
      busCtrl.driveAndRead(io.pulseMicros(i), baseAddress + (i << 2))
    }
  }
}

/*
 * pulseMicros -> 4 * n Read/write register to set pulse width in micro seconds for servo n
 **/
case class Apb3ServoCtrl(width: Int) extends Component {
  val io = new Bundle {
    val apb = slave(Apb3(Apb3Config(addressWidth = 8, dataWidth = 32)))
    val servo = master(Servo(width))
  }

  val busCtrl = Apb3SlaveFactory(io.apb)
  val servoCtrl = ServoCtrl(width)
  io.servo <> servoCtrl.io.servo

  servoCtrl.driveFrom(busCtrl)()
}
