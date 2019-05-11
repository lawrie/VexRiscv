
package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice test configuration
object MuraxBlackiceTest{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(

      onChipRamSize = 10 kB,
      onChipRamHexFile = "src/main/ressource/hex/muraxArduino.hex",
      ioAddress = 0xF0000000l,
      coreFrequency = 50 MHz,
      sramSize = 512 kB,
      sramAddress = 0x90000000l,
      sramDataWidth = 16,
      sramAddressWidth = 19,
      includeQuadrature = true,
      includeQspiAnalog = true,
      includePs2Keyboard = true,
      includeShiftOut = true,
      includeTone = true,
      includeI2c = true,
      includeSevenSegmentB = true,
      includeSevenSegmentA = true,
      includeShiftIn = true,
      includeSpi = true,
      pwmWidth = 5,
      pulseInWidth = 2,
      pinInterruptWidth = 2,
      gpioBWidth = 32,
      servoWidth = 4,
      gpioAWidth = 32,
      quadratureAddress = 0xF8000,
      qspiAnalogAddress = 0xF0000,
      ps2KeyboardAddress = 0xA80000,
      shiftOutAddress = 0x50000,
      toneAddress = 0x40000,
      i2cAddress = 0x70000,
      pwmAddress = 0x30000,
      sevenSegmentBAddress = 0x98000,
      timerAddress = 0x20000,
      uartAddress = 0x10000,
      pulseInAddress = 0x80000,
      sevenSegmentAAddress = 0x90000,
      shiftInAddress = 0xA00000,
      machineTimerAddress = 0xB0000,
      spiAddress = 0x60000,
      muxAddress = 0xD0000,
      pinInterruptAddress = 0xE0000,
      gpioBAddress = 0x08000,
      servoAddress = 0xC0000,
      ws2811Address = 0xD8000,
      gpioAAddress = 0x00000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

