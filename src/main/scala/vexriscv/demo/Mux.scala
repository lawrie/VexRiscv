package vexriscv.demo

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.misc._

case class Mux() extends Bundle with IMasterSlave {
  val pins = Bits(32 bits)

  override def asMaster(): Unit = {
    out(pins)
  }
}

case class MuxCtrl() extends Component {
  val io = new Bundle {
    val mux = master(Mux())
    val signals = in Bits(32 bits)
  }

  io.mux.pins := io.signals

  def driveFrom(busCtrl : BusSlaveFactory, baseAddress : Int = 0) () = new Area {
    busCtrl.driveAndRead(io.signals, baseAddress)
  }
}

/*
 * Signals -> 0x00 Write register to set mux signals
 **/
case class Apb3MuxCtrl() extends Component {
  val io = new Bundle {
    val apb = slave(Apb3(Apb3Config(addressWidth = 8, dataWidth = 32)))
    val mux = master(Mux())
  }

  val busCtrl = Apb3SlaveFactory(io.apb)
  val muxCtrl = MuxCtrl()
  io.mux <> muxCtrl.io.mux

  muxCtrl.driveFrom(busCtrl)()
}
