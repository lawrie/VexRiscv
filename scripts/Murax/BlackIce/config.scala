
package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice test configuration
object MuraxBlackiceTest{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(

      coreFrequency = 50 MHz,
      ioAddress = 0xF0000000l,
      onChipRamHexFile = "src/main/ressource/hex/muraxArduino.hex",
      onChipRamSize = 10 kB,
      sramAddress = 0x90000000l,
      sramAddressWidth = 19,
      sramDataWidth = 16,
      sramSize = 512 kB,
      includeI2c = true,
      includePs2 = true,
      includeQspiAnalog = true,
      includeQuadrature = true,
      includeSevenSegmentA = true,
      includeSevenSegmentB = true,
      includeShiftIn = true,
      includeShiftOut = true,
      includeSpiMaster = true,
      includeTone = true,
      gpioAWidth = 32,
      gpioBWidth = 32,
      pinInterruptWidth = 2,
      pulseInWidth = 2,
      pwmWidth = 5,
      servoWidth = 4,
      maxWs2811Leds = 16,
      gpioAAddress = 0x00000,
      gpioBAddress = 0x08000,
      i2cAddress = 0x70000,
      machineTimerAddress = 0xB0000,
      muxAddress = 0xD0000,
      pinInterruptAddress = 0xE0000,
      ps2Address = 0xA80000,
      pulseInAddress = 0x80000,
      pwmAddress = 0x30000,
      qspiAnalogAddress = 0xF0000,
      quadratureAddress = 0xF8000,
      servoAddress = 0xC0000,
      sevenSegmentAAddress = 0x90000,
      sevenSegmentBAddress = 0x98000,
      shiftInAddress = 0xA00000,
      shiftOutAddress = 0x50000,
      spiMasterAddress = 0x60000,
      timerAddress = 0x20000,
      toneAddress = 0x40000,
      uartAddress = 0x10000,
      ws2811Address = 0xD8000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

