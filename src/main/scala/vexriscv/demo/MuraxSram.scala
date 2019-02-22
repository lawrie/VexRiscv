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
  val addr = Bits((g.addressWidth - 1) bits)
  val dat = TriState(Bits(g.dataWidth bits))
  val cs  = Bool
  val we  = Bool
  val oe  = Bool
  val lb  = Bool
  val ub  = Bool

  override def asMaster(): Unit = {
    out(addr,cs,we,oe,lb,ub)
    master(dat)
  }
}

case class MuraxPipelinedMemoryBusSram(pipelinedMemoryBusConfig: PipelinedMemoryBusConfig,
                                       sramLayout : SramLayout) extends Component{
  val io = new Bundle{
    val bus = slave(PipelinedMemoryBus(pipelinedMemoryBusConfig))
    val sram = master(SramInterface(sramLayout))
  }

  val we = Reg(Bool)
  io.sram.we := !we

  val oe = Reg(Bool)
  io.sram.oe := !oe

  val lb = Reg(Bool)
  io.sram.lb := !lb

  val ub = Reg(Bool)
  io.sram.ub := !ub

  val state = Reg(UInt(2 bits)) init(0)
  
  val datOut = Reg(Bits(16 bits))
  io.sram.dat.write := datOut

  val addr = Reg(Bits(18 bits))
  io.sram.addr := addr

  io.sram.cs := !io.bus.cmd.valid

  io.bus.rsp.valid := RegNext(io.bus.cmd.ready && !io.bus.cmd.write) init(False)

  val rspData = Reg(Bits(32 bits))
  io.bus.rsp.data := rspData

  io.sram.dat.writeEnable := we

  io.bus.cmd.ready := False
    
  we := False
  oe := False

  when (io.bus.cmd.valid) {
    when(io.bus.cmd.write) {
      when (state === 0) {
        addr := io.bus.cmd.address(sramLayout.addressWidth - 1  downto 2) ## B"0"
        we := True
        datOut := io.bus.cmd.data(15 downto 0)
        lb := io.bus.cmd.mask(0)
        ub := io.bus.cmd.mask(1)
        state := 1
      } elsewhen (state === 1) {
        state := 2
      } elsewhen (state === 2) {
        addr := io.bus.cmd.address(sramLayout.addressWidth - 1 downto 2) ## B"1"
        we := True
        datOut := io.bus.cmd.data(31 downto 16)
        lb := io.bus.cmd.mask(2)
        ub := io.bus.cmd.mask(3)
        state := 3
      } elsewhen (state === 3) {
        io.bus.cmd.ready := True
        state := 0
      } 
    } otherwise { // Read
      lb := True
      ub := True
      when (state === 0) {
        oe := True
        addr := io.bus.cmd.address(sramLayout.addressWidth - 1 downto 2) ## B"0"
        state := 1
      } elsewhen (state === 1) {
        rspData(15 downto 0) := io.sram.dat.read 
        state := 2
      } elsewhen (state === 2) {
        addr := io.bus.cmd.address(sramLayout.addressWidth - 1 downto 2) ## B"1"
        oe := True
        state := 3
      } elsewhen (state === 3) {
        rspData(31 downto 16) := io.sram.dat.read
        io.bus.cmd.ready := True
        state := 0
      }
    }
  } 
}

