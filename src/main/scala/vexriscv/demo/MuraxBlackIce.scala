package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice configuration
object MuraxBlackice{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(
      coreFrequency         = 50 MHz,
      onChipRamSize         = 10 kB, 
      onChipRamHexFile      = "src/main/ressource/hex/muraxArduino.hex",
      ioAddress             = 0xF0000000l,
      sramAddress           = 0x90000000l,
      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true,
      sramSize              = 512 kB,
      sramAddressWidth      = 19,
      sramDataWidth         = 16,
      includeShiftIn        = false,
      includeShiftOut       = false,
      includeQuadrature     = false,
      includeTone           = false,
      includePs2Keyboard    = false,
      includeQspiAnalog     = false,
      includeSevenSegmentA  = false,
      includeSevenSegmentB  = false,
      includeSpi            = false,
      includeI2c            = false,
      pulseInWidth          = 0,
      pwmWidth              = 0,
      servoWidth            = 0,
      pinInterruptWidth     = 0,
      gpioAWidth            = 0,
      gpioBWidth            = 0,
      maxWs2811Leds         = 0,
      gpioAAddress          = 0x00000,
      gpioBAddress          = 0x08000,
      uartAddress           = 0x10000,
      timerAddress          = 0x20000,
      pwmAddress            = 0x30000,
      toneAddress           = 0x40000,
      shiftOutAddress       = 0x50000,
      spiAddress            = 0x60000,
      i2cAddress            = 0x70000,
      pulseInAddress        = 0x80000,
      sevenSegmentAAddress  = 0x90000,
      sevenSegmentBAddress  = 0x98000,
      shiftInAddress        = 0xA0000,
      ps2KeyboardAddress    = 0xA8000,
      machineTimerAddress   = 0xB0000,
      servoAddress          = 0xC0000,
      muxAddress            = 0xD0000,
      ws2811Address         = 0xD8000,
      pinInterruptAddress   = 0xE0000,
      qspiAnalogAddress     = 0xF0000,
      quadratureAddress     = 0xF8000
    )))
  }
}

