
package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice test configuration
object MuraxBlackiceTest{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(

      onChipRamSize = 10 kB,
      onChipRamHexFile = "src/main/ressource/hex/muraxArduino.hex",
      ioAddress = 0xF0000000l,
      coreFrequency = 50 MHz,
      sramSize = 512 kB,
      sramAddress = 0x90000000l,
      sramAddressWidth = 19,
      sramDataWidth = 16,
      includePs2 = true,
      includeShiftOut = true,
      includeSpiMaster = true,
      includeShiftIn = true,
      includeSevenSegmentB = true,
      includeI2c = true,
      includeQspiAnalog = true,
      includeQuadrature = true,
      includeSevenSegmentA = true,
      includeTone = true,
      pwmWidth = 5,
      pinInterruptWidth = 2,
      servoWidth = 4,
      pulseInWidth = 2,
      gpioAWidth = 32,
      gpioBWidth = 32,
      maxWs2811Leds = 16,
      pwmAddress = 0x30000,
      pinInterruptAddress = 0xE0000,
      timerAddress = 0x20000,
      ps2Address = 0xA80000,
      shiftOutAddress = 0x50000,
      spiMasterAddress = 0x60000,
      shiftInAddress = 0xA00000,
      sevenSegmentBAddress = 0x98000,
      ws2811Address = 0xD8000,
      uartAddress = 0x10000,
      servoAddress = 0xC0000,
      i2cAddress = 0x70000,
      pulseInAddress = 0x80000,
      qspiAnalogAddress = 0xF0000,
      quadratureAddress = 0xF8000,
      muxAddress = 0xD0000,
      gpioAAddress = 0x00000,
      machineTimerAddress = 0xB0000,
      gpioBAddress = 0x08000,
      sevenSegmentAAddress = 0x90000,
      toneAddress = 0x40000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

