package vexriscv

import java.awt
import java.awt.event.{ActionEvent, ActionListener, MouseEvent, MouseListener}

import spinal.sim._
import spinal.core._
import spinal.core.sim._
import vexriscv.demo.{Murax, MuraxConfig,MuraxArduino,MuraxArduinoConfig}
import javax.swing._

import spinal.lib.com.jtag.sim.JtagTcp
import spinal.lib.com.uart.sim.{UartDecoder, UartEncoder}
import vexriscv.test.{JLedArray, JSwitchArray}

import scala.collection.mutable



object MuraxArduinoSim {
  def main(args: Array[String]): Unit = {
//    def config = MuraxConfig.default.copy(onChipRamSize = 256 kB)
    def config = MuraxConfig.default(withXip = false).copy(onChipRamSize = 4 kB, onChipRamHexFile = "src/main/ressource/hex/muraxDemo.hex")
    val simSlowDown = false
    //SimConfig.allOptimisation.compile(new Murax(config)).doSimUntilVoid{dut =>
    SimConfig.withWave.compile(MuraxArduino(MuraxArduinoConfig.default)).doSimUntilVoid{dut =>      val mainClkPeriod = (1e12/dut.config.coreFrequency.toDouble).toLong
      val jtagClkPeriod = mainClkPeriod*4
      val uartBaudRate = 115200
      val uartBaudPeriod = (1e12/uartBaudRate).toLong

      val clockDomain = ClockDomain(dut.io.mainClk, dut.io.asyncReset)
      clockDomain.forkStimulus(mainClkPeriod)
//      clockDomain.forkSimSpeedPrinter(2)

      val tcpJtag = JtagTcp(
        jtag = dut.io.jtag,
        jtagClkPeriod = jtagClkPeriod
      )

      val uartTx = UartDecoder(
        uartPin = dut.io.uart.txd,
        baudPeriod = uartBaudPeriod
      )

      val uartRx = UartEncoder(
        uartPin = dut.io.uart.rxd,
        baudPeriod = uartBaudPeriod
      )

      if(config.xipConfig != null)dut.io.xip.data(1).read #= 0

      val guiThread = fork{
        val guiToSim = mutable.Queue[Any]()

        var ledsValue = 0l
        var switchValue : () => BigInt = null
        val ledsFrame = new JFrame{
          setLayout(new BoxLayout(getContentPane, BoxLayout.Y_AXIS))

          add(new JLedArray(8){
            override def getValue = ledsValue
          })
          add{
            val switches = new JSwitchArray(8)
            switchValue = switches.getValue
            switches
          }

          add(new JButton("Reset"){
            addActionListener(new ActionListener {
              override def actionPerformed(actionEvent: ActionEvent): Unit = {
                println("ASYNC RESET")
                guiToSim.enqueue("asyncReset")
              }
            })
            setAlignmentX(awt.Component.CENTER_ALIGNMENT)
          })
          setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE)
          pack()
          setVisible(true)

        }

        val sram = Array.fill(1024*1024)(0)
        clockDomain.onSamplings{
          delayed(2000){
            dut.io.sram.dat.read.randomize()
            if(!dut.io.sram.cs.toBoolean){
              val addr = dut.io.sram.addr.toInt
              if (!dut.io.sram.we.toBoolean && dut.io.sram.oe.toBoolean) {
                var content = sram(addr)
                val data = dut.io.sram.dat.write.toInt
                if (!dut.io.sram.lb.toBoolean) content = (content & ~0x00FF) | (data & 0x00FF)
                if (!dut.io.sram.ub.toBoolean) content = (content & ~0xFF00) | (data & 0xFF00)
                sram(addr) = content
              } else {
                if(!dut.io.sram.oe.toBoolean) dut.io.sram.dat.read #= sram(addr)
              }
            }
          }
        }

        clockDomain.onActiveEdges{
          delayed(100){ //100 ns Delay
            dut.io.i2c.scl.read #= dut.io.i2c.scl.write.toBoolean
            dut.io.i2c.sda.read #= dut.io.i2c.sda.write.toBoolean
          }
        }

        //Fast refresh
//        clockDomain.onSampling{
//          dut.io.gpioA.read #= (dut.io.gpioA.write.toLong & dut.io.gpioA.writeEnable.toLong) | (switchValue() << 8)
//        }

        //Slow refresh
        while(true){
          sleep(mainClkPeriod*50000)

          val dummy = if(guiToSim.nonEmpty){
            val request = guiToSim.dequeue()
            if(request == "asyncReset"){
              dut.io.asyncReset #= true
              sleep(mainClkPeriod*32)
              dut.io.asyncReset #= false
            }
          }

          dut.io.gpioA.read #= (dut.io.gpioA.write.toLong & dut.io.gpioA.writeEnable.toLong) | (switchValue() << 8)
          ledsValue = dut.io.gpioA.write.toLong
          ledsFrame.repaint()
          if(simSlowDown) Thread.sleep(400)
        }
      }
    }
  }
}
