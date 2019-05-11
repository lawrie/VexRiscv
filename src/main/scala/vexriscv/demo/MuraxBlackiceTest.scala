
package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice test configuration
object MuraxBlackiceTest{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(

      onChipRamHexFile = "src/main/ressource/hex/muraxArduino.hex",
      coreFrequency = 50 MHz,
      ioAddress = 0xF0000000l,
      onChipRamSize = 10 kB,
      sramDataWidth = 16,
      sramAddressWidth = 19,
      sramAddress = 0x90000000l,
      sramSize = 512 kB,
      includeShiftOut = true,
      includeI2c = true,
      includeSevenSegmentB = true,
      includeQspiAnalog = true,
      includePs2Keyboard = true,
      includeTone = true,
      includeSevenSegmentA = true,
      includeSpi = true,
      includeQuadrature = true,
      includeShiftIn = true,
      gpioAWidth = 32,
      servoWidth = 4,
      gpioBWidth = 32,
      pulseInWidth = 2,
      pwmWidth = 5,
      pinInterruptWidth = 2,
      gpioAAddress = 0x00000,
      servoAddress = 0xC0000,
      gpioBAddress = 0x08000,
      shiftOutAddress = 0x50000,
      pulseInAddress = 0x80000,
      i2cAddress = 0x70000,
      sevenSegmentBAddress = 0x98000,
      qspiAnalogAddress = 0xF0000,
      machineTimerAddress = 0xB0000,
      timerAddress = 0x20000,
      uartAddress = 0x10000,
      ps2KeyboardAddress = 0xA80000,
      toneAddress = 0x40000,
      sevenSegmentAAddress = 0x90000,
      pwmAddress = 0x30000,
      pinInterruptAddress = 0xE0000,
      spiAddress = 0x60000,
      ws2811Address = 0xD8000,
      quadratureAddress = 0xF8000,
      muxAddress = 0xD0000,
      shiftInAddress = 0xA00000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

