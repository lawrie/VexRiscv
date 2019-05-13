
package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice configuration
object MuraxBlackice{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(

      coreFrequency = 50 MHz,
      ioAddress = 0xF0000000l,
      onChipRamSize = 10 kB,
      onChipRamHexFile = "src/main/ressource/hex/muraxArduino.hex",
      sramAddressWidth = 19,
      sramAddress = 0x90000000l,
      sramDataWidth = 16,
      sramSize = 512 kB,
      includeShiftIn = true,
      includeI2c = true,
      includeQuadrature = true,
      includeSpiMaster = true,
      includeSevenSegmentB = true,
      includeQspiAnalog = true,
      includePs2 = true,
      includeShiftOut = true,
      includeTone = true,
      includeSevenSegmentA = true,
      pinInterruptWidth = 2,
      servoWidth = 4,
      gpioBWidth = 32,
      gpioAWidth = 32,
      pwmWidth = 5,
      pulseInWidth = 2,
      shiftInAddress = 0xA00000,
      i2cAddress = 0x70000,
      pinInterruptAddress = 0xE0000,
      servoAddress = 0xC0000,
      quadratureAddress = 0xF8000,
      spiMasterAddress = 0x60000,
      sevenSegmentBAddress = 0x98000,
      machineTimerAddress = 0xB0000,
      ws2811Address = 0xD8000,
      qspiAnalogAddress = 0xF0000,
      ps2Address = 0xA80000,
      gpioBAddress = 0x08000,
      shiftOutAddress = 0x50000,
      toneAddress = 0x40000,
      sevenSegmentAAddress = 0x90000,
      gpioAAddress = 0x00000,
      pwmAddress = 0x30000,
      timerAddress = 0x20000,
      pulseInAddress = 0x80000,
      muxAddress = 0xD0000,
      uartAddress = 0x10000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

