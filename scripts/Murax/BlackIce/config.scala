
package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice configuration
object MuraxBlackice{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(

      coreFrequency = 50 MHz,
      onChipRamSize = 10 kB,
      ioAddress = 0xF0000000l,
      input mainClk = CLK,
      onChipRamHexFile = src/main/ressource/hex/muraxArduino.hex,
      sramAddressWidth = 19,
      sramDataWidth = 16,
      sramAddress = 0x90000000l,
      sramSize = 512 kB,
      includeTone = true,
      includeSevenSegmentA = true,
      includeQuadrature = true,
      includeSpi = true,
      includeQspiAnalog = true,
      includeI2c = true,
      includePs2Keyboard = true,
      includeSevenSegmentB = true,
      includeShiftOut = true,
      includeShiftIn = true,
      pwmWidth = 5,
      pulseInWidth = 2,
      pinInterruptWidth = 2,
      servoWidth = 4,
      gpioAWidth = 32,
      gpioBWidth = 32,
      toneAddress = 0x40000,
      pwmAddress = 0x30000,
      ws2811Address = 0xD8000,
      timerAddress = 0x20000,
      pulseInAddress = 0x80000,
      sevenSegmentAAddress = 0x90000,
      quadratureAddress = 0xF8000,
      spiAddress = 0x60000,
      pinInterruptAddress = 0xE0000,
      qspiAnalogAddress = 0xF0000,
      uartAddress = 0x10000,
      servoAddress = 0xC0000,
      gpioAAddress = 0x00000,
      gpioBAddress = 0x08000,
      i2cAddress = 0x70000,
      ps2KeyboardAddress = 0xA80000,
      sevenSegmentBAddress = 0x98000,
      shiftOutAddress = 0x50000,
      machineTimerAddress = 0xB0000,
      muxAddress = 0xD0000,
      shiftInAddress = 0xA00000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

