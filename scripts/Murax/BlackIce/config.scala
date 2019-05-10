
package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice configuration
object MuraxBlackice{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(

      onChipRamHexFile = src/main/ressource/hex/muraxArduino.hex,
      ioAddress = 0xF0000000l,
      onChipRamSize = 10kB,
      coreFrequency = 50MMz,
      sramAddressWidth = 19,
      sramAddress = 0x90000000l,
      sramSize = 512kB,
      sramDataWidth = 16,
      includeTone = true,
      includeI2c = true,
      includeSevenSegmentB = true,
      includeSpi = true,
      includeSevenSegmentA = true,
      includeShiftOut = true,
      includeShiftIn = true,
      includePs2Keyboard = true,
      includeQspiAnalog = true,
      includeQuadrature = true,
      pwmWidth = 5,
      pinInterruptWidth = 2,
      gpioAWidth = 32,
      pulseInWidth = 2,
      servoWidth = 4,
      gpioBWidth = 32,
      toneAddress = 0x40000,
      i2cAddress = 0x70000,
      pwmAddress = 0x30000,
      sevenSegmentBAddress = 0x98000,
      pinInterruptAddress = 0xE0000,
      machineTimerAddress = 0xB0000,
      gpioAAddress = 0x00000,
      spiAddress = 0x60000,
      sevenSegmentAAddress = 0x90000,
      pulseInAddress = 0x80000,
      shiftOutAddress = 0x50000,
      muxAddress = 0xD0000,
      shiftInAddress = 0xA00000,
      ws2811Address = 0xD8000,
      servoAddress = 0xC0000,
      uartAddress = 0x10000,
      ps2KeyboardAddress = 0xA80000,
      timerAddress = 0x20000,
      gpioBAddress = 0x08000,
      qspiAnalogAddress = 0xF0000,
      quadratureAddress = 0xF8000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

