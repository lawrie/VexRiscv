package vexriscv.demo

import spinal.core._
import spinal.lib._
import spinal.lib.bus.simple._
import spinal.lib.io.TriState

case class SramLayout(addressWidth: Int, dataWidth : Int){
  def bytePerWord = dataWidth/8
  def capacity = BigInt(1) << addressWidth
}

case class SramInterface(g : SramLayout) extends Bundle with IMasterSlave{
  val addr = UInt(g.addressWidth bits)
  val dat = TriState(Bits(g.dataWidth bits))
  val cs  = Bool
  val we  = Bool
  val oe  = Bool
  val lb  = Bool
  val ub  = Bool
  val leds = Bits(8 bits)

  override def asMaster(): Unit = {
    out(addr,cs,we,oe,lb,ub,leds)
    master(dat)
  }
}

case class MuraxPipelinedMemoryBusSram(pipelinedMemoryBusConfig : PipelinedMemoryBusConfig) extends Component{
  val io = new Bundle{
    val bus = slave(PipelinedMemoryBus(pipelinedMemoryBusConfig))
    val sram = master(SramInterface(SramLayout(18, 16)))
  }

  val we = Reg(Bool)
  io.sram.we := !we

  val oe = Reg(Bool)
  io.sram.oe := !oe

  val lb = Reg(Bool)
  io.sram.lb := !lb

  val ub = Reg(Bool)
  io.sram.ub := !ub

  val state = Reg(UInt(2 bits))
  
  val datOut = Reg(Bits(16 bits))
  io.sram.dat.write := datOut

  val addr = Reg(UInt(18 bits))
  io.sram.addr := addr

  io.sram.cs := !io.bus.cmd.valid

  val cmdReady = Reg(Bool)
  io.bus.cmd.ready := cmdReady

  val rspValid = Reg(Bool)
  io.bus.rsp.valid := rspValid

  val rspData = Reg(Bits(32 bits))
  io.bus.rsp.data := rspData

  io.sram.dat.writeEnable := we

  rspValid := cmdReady && !io.bus.cmd.write

  val leds = Reg(Bits(8 bits))
  io.sram.leds := leds

  leds := B(0, 8 bits)
  leds(7 downto 6) := state.asBits

  cmdReady := False

  when (io.bus.cmd.valid && !cmdReady) {
    when(io.bus.cmd.write) {
      leds(0) := True
      when (state === 0) {
        addr := (io.bus.cmd.address >> 1).resized
        we := True
        datOut := io.bus.cmd.data(15 downto 0)
        lb := io.bus.cmd.mask(0)
        ub := io.bus.cmd.mask(1)
        state := 1
      } elsewhen (state === 1) {
        we := False
        state := 2
      } elsewhen (state === 2) {
        addr := addr + 1      
        we := True
        datOut := io.bus.cmd.data(31 downto 16)
        lb := io.bus.cmd.mask(2)
        ub := io.bus.cmd.mask(3)
        state := 3
      } elsewhen (state === 3) {
        we := False
        state := 0
        cmdReady := True
      }  
    } otherwise { // Read
      when (state === 0) {
        oe := True
        addr := (io.bus.cmd.address >> 1).resized
        state := 1
      } elsewhen (state === 1) {
        rspData(15 downto 0) := io.sram.dat.read 
        state := 2
        addr := addr + 1
      } elsewhen (state === 2) {
        rspData(31 downto 16) := io.sram.dat.read
        oe := False
        cmdReady := True
        state := 0
      }
    }
  } 
}

