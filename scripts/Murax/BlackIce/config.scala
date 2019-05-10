
package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice configuration
object MuraxBlackice{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(

      input mainClk = CLK,
      ioAddress = 0xF0000000l,
      onChipRamHexFile = src/main/ressource/hex/muraxArduino.hex,
      onChipRamSize = 10 kB,
      coreFrequency = 50 MHz,
      sramSize = 512 kB,
      sramAddress = 0x90000000l,
      sramDataWidth = 16,
      sramAddressWidth = 19,
      includeSevenSegmentA = true,
      includeSevenSegmentB = true,
      includeShiftIn = true,
      includeSpi = true,
      includeQspiAnalog = true,
      includeI2c = true,
      includePs2Keyboard = true,
      includeTone = true,
      includeQuadrature = true,
      includeShiftOut = true,
      servoWidth = 4,
      pinInterruptWidth = 2,
      pwmWidth = 5,
      gpioBWidth = 32,
      pulseInWidth = 2,
      gpioAWidth = 32,
      sevenSegmentAAddress = 0x90000,
      ws2811Address = 0xD8000,
      servoAddress = 0xC0000,
      sevenSegmentBAddress = 0x98000,
      pinInterruptAddress = 0xE0000,
      shiftInAddress = 0xA00000,
      pwmAddress = 0x30000,
      machineTimerAddress = 0xB0000,
      spiAddress = 0x60000,
      qspiAnalogAddress = 0xF0000,
      i2cAddress = 0x70000,
      uartAddress = 0x10000,
      ps2KeyboardAddress = 0xA80000,
      toneAddress = 0x40000,
      gpioBAddress = 0x08000,
      quadratureAddress = 0xF8000,
      pulseInAddress = 0x80000,
      shiftOutAddress = 0x50000,
      muxAddress = 0xD0000,
      gpioAAddress = 0x00000,
      timerAddress = 0x20000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

