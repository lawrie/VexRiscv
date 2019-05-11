
package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice test configuration
object MuraxBlackiceTest{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(

      ioAddress = 0xF0000000l,
      onChipRamHexFile = "src/main/ressource/hex/muraxArduino.hex",
      onChipRamSize = 10 kB,
      coreFrequency = 50 MHz,
      sramSize = 512 kB,
      sramAddress = 0x90000000l,
      sramAddressWidth = 19,
      sramDataWidth = 16,
      includeQuadrature = true,
      includeShiftOut = true,
      includeSpi = true,
      includeSevenSegmentA = true,
      includeSevenSegmentB = true,
      includeQspiAnalog = true,
      includeTone = true,
      includeShiftIn = true,
      includeI2c = true,
      includePs2Keyboard = true,
      pinInterruptWidth = 2,
      gpioBWidth = 32,
      pulseInWidth = 2,
      pwmWidth = 5,
      servoWidth = 4,
      gpioAWidth = 32,
      maxWs2811Leds = 16,
      pinInterruptAddress = 0xE0000,
      gpioBAddress = 0x08000,
      pulseInAddress = 0x80000,
      muxAddress = 0xD0000,
      uartAddress = 0x10000,
      quadratureAddress = 0xF8000,
      pwmAddress = 0x30000,
      servoAddress = 0xC0000,
      shiftOutAddress = 0x50000,
      spiAddress = 0x60000,
      machineTimerAddress = 0xB0000,
      ws2811Address = 0xD8000,
      sevenSegmentAAddress = 0x90000,
      sevenSegmentBAddress = 0x98000,
      qspiAnalogAddress = 0xF0000,
      toneAddress = 0x40000,
      gpioAAddress = 0x00000,
      shiftInAddress = 0xA00000,
      timerAddress = 0x20000,
      i2cAddress = 0x70000,
      ps2KeyboardAddress = 0xA80000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

