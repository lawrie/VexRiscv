
package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice configuration
object MuraxBlackice{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(

      onChipRamSize = 10 kB,
      input mainClk = CLK,
      onChipRamHexFile = src/main/ressource/hex/muraxArduino.hex,
      ioAddress = 0xF0000000l,
      coreFrequency = 50 MHz,
      sramSize = 512 kB,
      sramDataWidth = 16,
      sramAddress = 0x90000000l,
      sramAddressWidth = 19,
      includeSevenSegmentB = true,
      includeTone = true,
      includePs2Keyboard = true,
      includeSpi = true,
      includeI2c = true,
      includeSevenSegmentA = true,
      includeQspiAnalog = true,
      includeShiftIn = true,
      includeQuadrature = true,
      includeShiftOut = true,
      pwmWidth = 5,
      pulseInWidth = 2,
      gpioAWidth = 32,
      gpioBWidth = 32,
      pinInterruptWidth = 2,
      servoWidth = 4,
      sevenSegmentBAddress = 0x98000,
      toneAddress = 0x40000,
      pwmAddress = 0x30000,
      pulseInAddress = 0x80000,
      gpioAAddress = 0x00000,
      machineTimerAddress = 0xB0000,
      ps2KeyboardAddress = 0xA80000,
      spiAddress = 0x60000,
      i2cAddress = 0x70000,
      ws2811Address = 0xD8000,
      sevenSegmentAAddress = 0x90000,
      timerAddress = 0x20000,
      qspiAnalogAddress = 0xF0000,
      gpioBAddress = 0x08000,
      shiftInAddress = 0xA00000,
      pinInterruptAddress = 0xE0000,
      quadratureAddress = 0xF8000,
      uartAddress = 0x10000,
      muxAddress = 0xD0000,
      shiftOutAddress = 0x50000,
      servoAddress = 0xC0000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

