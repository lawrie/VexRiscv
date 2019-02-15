package vexriscv.demo

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.misc._

case class PulseIn() extends Bundle with IMasterSlave {
  val pin = Bool

  override def asMaster(): Unit = {
    in(pin)
  }
}

case class PulseInCtrl() extends Component {
  val io = new Bundle {
    val pulseIn = master(PulseIn())
    val timeout = in UInt(32 bits)
    val value = in Bool
    val req = in Bool
    val pulseLength = out UInt(32 bits)
  }

  val pulseOut = Reg(UInt(32 bits))
  io.pulseLength := pulseOut

  val clockMhz = 100
  val micros = Reg(UInt(32 bits))
  val counter = Reg(UInt(8 bits))
  val req = Reg(Bool)

  when (io.req) {
    req := True
    pulseOut := 0
  }

  when (req) {
    when (io.pulseIn.pin === io.value || (io.timeout > 0 && micros === (io.timeout - 1))) {
      pulseOut := micros | U(31 -> true, (30 downto 0) -> false)
      req := False
    } otherwise {
      counter := counter + 1

      when (counter === (clockMhz - 1)) {
        micros := micros + 1
        counter := 0
      }
    }
  } otherwise {
    counter := 0
    micros := 0
  }

  def driveFrom(busCtrl : BusSlaveFactory, baseAddress : Int = 0) () = new Area {
    val busSetting = False 
    busCtrl.drive(io.value, baseAddress)
    busCtrl.drive(io.timeout, baseAddress + 4)
    busCtrl.read(io.pulseLength,baseAddress + 8)

    busSetting setWhen(busCtrl.isWriting(baseAddress))

    io.req := busSetting
  }
}

/*
 * Value       -> 0x00 Write register to set the value of the pulse (HIGH or LOW)
 * Timeout     -> 0x04 Write register to set the timeout in microseconds
 * PulseLength -> 0x08 Read register to read the pulse length in microseconds
 **/
case class Apb3PulseInCtrl() extends Component {
  val io = new Bundle {
    val apb = slave(Apb3(Apb3Config(addressWidth = 8, dataWidth = 32)))
    val pulseIn = master(PulseIn())
  }

  val busCtrl = Apb3SlaveFactory(io.apb)
  val pulseInCtrl = PulseInCtrl()
  io.pulseIn <> pulseInCtrl.io.pulseIn

  pulseInCtrl.driveFrom(busCtrl)()
}