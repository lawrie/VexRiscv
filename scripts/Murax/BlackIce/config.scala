
package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice configuration
object MuraxBlackice{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(

      coreFrequency = 50 MHz,
      onChipRamHexFile = "src/main/ressource/hex/muraxArduino.hex",
      ioAddress = 0xF0000000l,
      onChipRamSize = 10 kB,
      sramSize = 512 kB,
      sramAddressWidth = 19,
      sramAddress = 0x90000000l,
      sramDataWidth = 16,
      includeSevenSegmentA = true,
      includeQuadrature = true,
      includeI2c = true,
      includeSevenSegmentB = true,
      includeShiftIn = true,
      includeTone = true,
      includeShiftOut = true,
      includeSpi = true,
      includeQspiAnalog = true,
      includePs2Keyboard = true,
      pinInterruptWidth = 2,
      servoWidth = 4,
      gpioBWidth = 32,
      pulseInWidth = 2,
      pwmWidth = 5,
      gpioAWidth = 32,
      machineTimerAddress = 0xB0000,
      timerAddress = 0x20000,
      sevenSegmentAAddress = 0x90000,
      ws2811Address = 0xD8000,
      muxAddress = 0xD0000,
      quadratureAddress = 0xF8000,
      uartAddress = 0x10000,
      i2cAddress = 0x70000,
      pinInterruptAddress = 0xE0000,
      sevenSegmentBAddress = 0x98000,
      shiftInAddress = 0xA00000,
      toneAddress = 0x40000,
      servoAddress = 0xC0000,
      shiftOutAddress = 0x50000,
      spiAddress = 0x60000,
      gpioBAddress = 0x08000,
      pulseInAddress = 0x80000,
      pwmAddress = 0x30000,
      gpioAAddress = 0x00000,
      qspiAnalogAddress = 0xF0000,
      ps2KeyboardAddress = 0xA80000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

