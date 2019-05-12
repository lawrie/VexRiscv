
package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice test configuration
object MuraxBlackiceTest{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(

      coreFrequency = 50 MHz,
      ioAddress = 0xF0000000l,
      onChipRamSize = 10 kB,
      onChipRamHexFile = "src/main/ressource/hex/muraxArduino.hex",
      sramAddressWidth = 19,
      sramSize = 512 kB,
      sramDataWidth = 16,
      sramAddress = 0x90000000l,
      includeSevenSegmentB = true,
      includeSpiMaster = true,
      includeSevenSegmentA = true,
      includeShiftOut = true,
      includeQspiAnalog = true,
      includeQuadrature = true,
      includeShiftIn = true,
      includeTone = true,
      includePs2 = true,
      includeI2c = true,
      gpioBWidth = 32,
      pwmWidth = 5,
      gpioAWidth = 32,
      servoWidth = 4,
      pinInterruptWidth = 2,
      pulseInWidth = 2,
      maxWs2811Leds = 16,
      timerAddress = 0x20000,
      muxAddress = 0xD0000,
      sevenSegmentBAddress = 0x98000,
      gpioBAddress = 0x08000,
      spiMasterAddress = 0x60000,
      sevenSegmentAAddress = 0x90000,
      shiftOutAddress = 0x50000,
      uartAddress = 0x10000,
      qspiAnalogAddress = 0xF0000,
      ws2811Address = 0xD8000,
      pwmAddress = 0x30000,
      machineTimerAddress = 0xB0000,
      quadratureAddress = 0xF8000,
      shiftInAddress = 0xA00000,
      toneAddress = 0x40000,
      gpioAAddress = 0x00000,
      servoAddress = 0xC0000,
      ps2Address = 0xA80000,
      pinInterruptAddress = 0xE0000,
      pulseInAddress = 0x80000,
      i2cAddress = 0x70000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

