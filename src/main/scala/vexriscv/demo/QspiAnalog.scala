package vexriscv.demo

import spinal.core._
import spinal.lib._
import spinal.lib.io.TriStateArray
import spinal.lib.master
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.misc._


case class Qspi() extends Bundle with IMasterSlave {
  val qck = Bool
  val qss = Bool
  val qd = TriStateArray(4)
  override def asMaster(): Unit = {
    in(qck, qss)
    master(qd)
  }
}

case class QspiCtrl() extends Component {
  val io = new Bundle {
    val qspi = master(Qspi())
    val a0 = in Bits(10 bits)
    val a1 = in Bits(10 bits)
    val a2 = in Bits(10 bits)
    val a3 = in Bits(10 bits)
    val a4 = in Bits(10 bits)
    val a5 = in Bits(10 bits)
  }

  // Detect rise on qss and rise and fall on qck
  val qssR = Reg(Bits(3 bits))
  qssR := qssR(1 downto 0) ## io.qspi.qss

  val qdR = Reg(Bits(8 bits))
  qdR := qdR(3 downto 0) ## io.qspi.qd.read

  val qckR = Reg(Bits(3 bits))
  qckR := qckR(1 downto 0) ## io.qspi.qck

  val qckRise = (qckR(2 downto 1) === B"01")
  val qckFall = (qckR(2 downto 1) === B"10")
  val qssRise = (qssR(2 downto 1) === B"01")

  // QSPI RX slave
  val rx = new QspiSlaveRX
  rx.io.qckRise := qckRise
  rx.io.qssRise := qssRise
  rx.io.qd :=  qdR(7 downto 4)
  
  // QSPI TX slave
  val tx = new QspiSlaveTX
  tx.io.qckFall := qckFall
  tx.io.qssRise := qssRise
  io.qspi.qd.write := tx.io.qd
  
  // Swap between reading and writing when qss rises
  val readEnable = Reg(Bits(4 bits))
  io.qspi.qd.writeEnable := ~readEnable
  rx.io.readEnable := (readEnable =/= 0)
  tx.io.writeEnable := (readEnable === 0)

  when (qssRise) {
    readEnable := ~readEnable
  }

  // Read byte and echo it back
  val rxData = Reg(Bits(8 bits))

  when (rx.io.rxReady) {  
    rxData := rx.io.rxData
  }

  tx.io.txData := rxData

  // Set all the analog registers
  when (rx.io.rxReady) {
    when (rx.io.byteNumber === 2) {
      io.a0(7 downto 0) := rx.io.rxData
    }

    when (rx.io.byteNumber === 3) {
      io.a0(9 downto 8) := rx.io.rxData(1 downto 0)
    }

    when (rx.io.byteNumber === 4) {
      io.a1(7 downto 0) := rx.io.rxData
    }

    when (rx.io.byteNumber === 5) {
      io.a1(9 downto 8) := rx.io.rxData(1 downto 0)
    }

    when (rx.io.byteNumber === 6) {
      io.a2(7 downto 0) := rx.io.rxData
    }

    when (rx.io.byteNumber === 7) {
      io.a2(9 downto 8) := rx.io.rxData(1 downto 0)
    }

    when (rx.io.byteNumber === 8) {
      io.a3(7 downto 0) := rx.io.rxData
    }

    when (rx.io.byteNumber === 9) {
      io.a3(9 downto 8) := rx.io.rxData(1 downto 0)
    }

    when (rx.io.byteNumber === 10) {
      io.a4(7 downto 0) := rx.io.rxData
    }

    when (rx.io.byteNumber === 11) {
      io.a4(9 downto 8) := rx.io.rxData(1 downto 0)
    }

    when (rx.io.byteNumber === 12) {
      io.a5(7 downto 0) := rx.io.rxData
    }

    when (rx.io.byteNumber === 13) {
      io.a5(9 downto 8) := rx.io.rxData(1 downto 0)
    }
  }
  
  def driveFrom(busCtrl : BusSlaveFactory, baseAddress : Int = 0) () = new Area {
    busCtrl.read(io.a0, baseAddress)
    busCtrl.read(io.a1, baseAddress + 0x04)
    busCtrl.read(io.a2, baseAddress + 0x08)
    busCtrl.read(io.a3, baseAddress + 0x0c)
    busCtrl.read(io.a4, baseAddress + 0x10)
    busCtrl.read(io.a5, baseAddress + 0x14)
  }

}
  
class QspiSlaveTX extends Component {
  val io = new Bundle {
    val qckFall = in Bool
    val qssRise = in Bool
    val qd = out Bits(4 bits)
    val txReady = out Bool
    val txData = in Bits(8 bits)
    val writeEnable = in Bool
  }

  val outData = Reg(Bits(4 bits))
  val firstNibble = Reg(Bool)

  io.qd := outData

  val txReady = Reg(Bool)  
  io.txReady := txReady

  when (io.writeEnable) {
    when (io.qssRise) { // io.qss deselect
      firstNibble := True
    } 

    when (io.qckFall) {
      when (firstNibble) {
        outData := io.txData(7 downto 4)
        firstNibble := False
      } otherwise {
        outData := io.txData(3 downto 0)
        txReady := True
      }
    }
  }    
}

class QspiSlaveRX extends Component {
  val io = new Bundle {
    val qckRise = in Bool
    val qssRise = in Bool
    val qd = in Bits(4 bits)
    val rxReady = out Bool
    val rxData = out Bits(8 bits)
    val byteNumber = out UInt(4 bits)
    val readEnable = in Bool
  }
  
  val shiftReg = Reg(Bits(8 bits))
  io.rxData := shiftReg

  val lastByte = Reg(Bits(8 bits))
  val byteNumber = Reg(UInt(4 bits))
  io.byteNumber := byteNumber

  val firstNibble = Reg(Bool)

  val rxReady = Reg(Bool) 
  io.rxReady := rxReady

  rxReady := False

  when (io.readEnable) {
    when (io.qssRise) {
      firstNibble := True
      lastByte := shiftReg
    } 

    when (io.qckRise) { 
      when (firstNibble) {
        byteNumber := byteNumber + 1
        shiftReg(7 downto 4) := io.qd
        firstNibble := False
      } otherwise {
        shiftReg(3 downto 0) := io.qd
        rxReady := True
      }
    }    
  }
}

/*
 * A0 -> 0x00 Read register to get the analog value
 * A1 -> 0x04 Read register to get the analog value
 * A2 -> 0x08 Read register to get the analog value
 * A3 -> 0x0c Read register to get the analog value
 * A4 -> 0x10 Read register to get the analog value
 * A5 -> 0x14 Read register to get the analog value
 **/
case class Apb3QspiCtrl() extends Component {
  val io = new Bundle {
    val apb = slave(Apb3(Apb3Config(addressWidth = 8, dataWidth = 32)))
    val qspi = master(Qspi())
  }

  val busCtrl = Apb3SlaveFactory(io.apb)
  val qspiCtrl = QspiCtrl()
  io.qspi <> qspiCtrl.io.qspi

  qspiCtrl.driveFrom(busCtrl)()
}

