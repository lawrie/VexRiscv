
package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice test configuration
object MuraxBlackiceTest{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(

      coreFrequency = 50 MHz,
      ioAddress = 0xF0000000l,
      onChipRamHexFile = "src/main/ressource/hex/muraxArduino.hex",
      onChipRamSize = 10 kB,
      sramAddress = 0x90000000l,
      sramAddressWidth = 19,
      sramDataWidth = 16,
      sramSize = 512 kB,
      includeI2c = true,
      includeQspiAnalog = true,
      includeSevenSegmentA = true,
      includeSevenSegmentB = true,
      includeSpiMaster = true,
      gpioAWidth = 32,
      gpioBWidth = 32,
      pinInterruptWidth = 2,
      pwmWidth = 5,
      maxWs2811Leds = 16,
      gpioAAddress = 0x00000,
      gpioBAddress = 0x08000,
      i2cAddress = 0x70000,
      machineTimerAddress = 0xB0000,
      muxAddress = 0xD0000,
      pinInterruptAddress = 0xE0000,
      pwmAddress = 0x30000,
      qspiAnalogAddress = 0xF0000,
      sevenSegmentAAddress = 0x90000,
      sevenSegmentBAddress = 0x98000,
      spiMasterAddress = 0x60000,
      timerAddress = 0x20000,
      uartAddress = 0x10000,
      ws2811Address = 0xD8000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

