
package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice test configuration
object MuraxBlackiceTest{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(

      onChipRamHexFile = "src/main/ressource/hex/muraxArduino.hex",
      coreFrequency = 50 MHz,
      onChipRamSize = 10 kB,
      ioAddress = 0xF0000000l,
      sramAddress = 0x90000000l,
      sramSize = 512 kB,
      sramAddressWidth = 19,
      sramDataWidth = 16,
      includeQuadrature = true,
      includeShiftOut = true,
      includeTone = true,
      includeSpiMaster = true,
      includeSevenSegmentA = true,
      includeShiftIn = true,
      includePs2 = true,
      includeI2c = true,
      includeSevenSegmentB = true,
      includeQspiAnalog = true,
      pulseInWidth = 2,
      pwmWidth = 5,
      gpioAWidth = 32,
      gpioBWidth = 32,
      servoWidth = 4,
      pinInterruptWidth = 2,
      maxWs2811Leds = 16,
      machineTimerAddress = 0xB0000,
      muxAddress = 0xD0000,
      quadratureAddress = 0xF8000,
      shiftOutAddress = 0x50000,
      toneAddress = 0x40000,
      pulseInAddress = 0x80000,
      uartAddress = 0x10000,
      pwmAddress = 0x30000,
      gpioAAddress = 0x00000,
      gpioBAddress = 0x08000,
      spiMasterAddress = 0x60000,
      ws2811Address = 0xD8000,
      sevenSegmentAAddress = 0x90000,
      servoAddress = 0xC0000,
      shiftInAddress = 0xA00000,
      ps2Address = 0xA80000,
      pinInterruptAddress = 0xE0000,
      i2cAddress = 0x70000,
      sevenSegmentBAddress = 0x98000,
      qspiAnalogAddress = 0xF0000,
      timerAddress = 0x20000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

