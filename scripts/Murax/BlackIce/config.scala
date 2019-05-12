
package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice test configuration
object MuraxBlackiceTest{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(

      coreFrequency = 50 MHz,
      onChipRamSize = 10 kB,
      ioAddress = 0xF0000000l,
      onChipRamHexFile = "src/main/ressource/hex/muraxArduino.hex",
      sramSize = 512 kB,
      sramAddress = 0x90000000l,
      sramDataWidth = 16,
      sramAddressWidth = 19,
      includeSevenSegmentB = true,
      includeSpiMaster = true,
      includeTone = true,
      includeShiftIn = true,
      includeShiftOut = true,
      includeQspiAnalog = true,
      includeQuadrature = true,
      includeSevenSegmentA = true,
      includePs2 = true,
      includeI2c = true,
      pwmWidth = 5,
      servoWidth = 4,
      pinInterruptWidth = 2,
      pulseInWidth = 2,
      gpioAWidth = 32,
      gpioBWidth = 32,
      maxWs2811Leds = 16,
      sevenSegmentBAddress = 0x98000,
      pwmAddress = 0x30000,
      ws2811Address = 0xD8000,
      spiMasterAddress = 0x60000,
      servoAddress = 0xC0000,
      machineTimerAddress = 0xB0000,
      toneAddress = 0x40000,
      pinInterruptAddress = 0xE0000,
      muxAddress = 0xD0000,
      shiftInAddress = 0xA00000,
      timerAddress = 0x20000,
      shiftOutAddress = 0x50000,
      qspiAnalogAddress = 0xF0000,
      uartAddress = 0x10000,
      pulseInAddress = 0x80000,
      quadratureAddress = 0xF8000,
      sevenSegmentAAddress = 0x90000,
      ps2Address = 0xA80000,
      gpioAAddress = 0x00000,
      i2cAddress = 0x70000,
      gpioBAddress = 0x08000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

