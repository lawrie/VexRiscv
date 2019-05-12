
package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice test configuration
object MuraxBlackiceTest{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(

      coreFrequency = 50 MHz,
      onChipRamHexFile = "src/main/ressource/hex/muraxArduino.hex",
      onChipRamSize = 10 kB,
      ioAddress = 0xF0000000l,
      sramDataWidth = 16,
      sramSize = 512 kB,
      sramAddressWidth = 19,
      sramAddress = 0x90000000l,
      includePs2 = true,
      includeTone = true,
      includeSevenSegmentB = true,
      includeI2c = true,
      includeQuadrature = true,
      includeQspiAnalog = true,
      includeShiftIn = true,
      includeSevenSegmentA = true,
      includeShiftOut = true,
      includeSpiMaster = true,
      gpioBWidth = 32,
      gpioAWidth = 32,
      pinInterruptWidth = 2,
      servoWidth = 4,
      pwmWidth = 5,
      pulseInWidth = 2,
      maxWs2811Leds = 16,
      timerAddress = 0x20000,
      uartAddress = 0x10000,
      ps2Address = 0xA80000,
      toneAddress = 0x40000,
      machineTimerAddress = 0xB0000,
      sevenSegmentBAddress = 0x98000,
      gpioBAddress = 0x08000,
      gpioAAddress = 0x00000,
      i2cAddress = 0x70000,
      quadratureAddress = 0xF8000,
      qspiAnalogAddress = 0xF0000,
      pinInterruptAddress = 0xE0000,
      servoAddress = 0xC0000,
      pwmAddress = 0x30000,
      shiftInAddress = 0xA00000,
      sevenSegmentAAddress = 0x90000,
      ws2811Address = 0xD8000,
      pulseInAddress = 0x80000,
      muxAddress = 0xD0000,
      shiftOutAddress = 0x50000,
      spiMasterAddress = 0x60000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

