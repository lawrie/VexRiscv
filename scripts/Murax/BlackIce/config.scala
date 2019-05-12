
package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice test configuration
object MuraxBlackiceTest{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(

      onChipRamHexFile = "src/main/ressource/hex/muraxArduino.hex",
      ioAddress = 0xF0000000l,
      onChipRamSize = 10 kB,
      coreFrequency = 50 MHz,
      sramDataWidth = 16,
      sramAddressWidth = 19,
      sramAddress = 0x90000000l,
      sramSize = 512 kB,
      includeQuadrature = true,
      includePs2 = true,
      includeQspiAnalog = true,
      includeSpiMaster = true,
      includeShiftIn = true,
      includeShiftOut = true,
      includeI2c = true,
      includeSevenSegmentA = true,
      includeTone = true,
      includeSevenSegmentB = true,
      pwmWidth = 5,
      servoWidth = 4,
      pinInterruptWidth = 2,
      pulseInWidth = 2,
      gpioAWidth = 32,
      gpioBWidth = 32,
      maxWs2811Leds = 16,
      timerAddress = 0x20000,
      quadratureAddress = 0xF8000,
      uartAddress = 0x10000,
      machineTimerAddress = 0xB0000,
      pwmAddress = 0x30000,
      ps2Address = 0xA80000,
      servoAddress = 0xC0000,
      muxAddress = 0xD0000,
      pinInterruptAddress = 0xE0000,
      qspiAnalogAddress = 0xF0000,
      spiMasterAddress = 0x60000,
      shiftInAddress = 0xA00000,
      shiftOutAddress = 0x50000,
      pulseInAddress = 0x80000,
      gpioAAddress = 0x00000,
      i2cAddress = 0x70000,
      sevenSegmentAAddress = 0x90000,
      toneAddress = 0x40000,
      gpioBAddress = 0x08000,
      ws2811Address = 0xD8000,
      sevenSegmentBAddress = 0x98000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

