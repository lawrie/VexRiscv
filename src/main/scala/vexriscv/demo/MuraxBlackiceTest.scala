
package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice test configuration
object MuraxBlackiceTest{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default(false, 0x80000000l).copy(

      coreFrequency = 50 MHz,
      onChipRamSize = 10 kB,
      onChipRamHexFile = "src/main/ressource/hex/muraxArduino.hex",
      ioAddress = 0xF0000000l,
      sramDataWidth = 16,
      sramAddress = 0x90000000l,
      sramAddressWidth = 19,
      sramSize = 512 kB,
      includeShiftOut = true,
      includeShiftIn = true,
      includePs2 = true,
      includeSevenSegmentB = true,
      includeQuadrature = true,
      includeSpiMaster = true,
      includeSevenSegmentA = true,
      includeI2c = true,
      includeTone = true,
      includeQspiAnalog = true,
      pwmWidth = 5,
      pulseInWidth = 2,
      gpioAWidth = 32,
      gpioBWidth = 32,
      servoWidth = 4,
      pinInterruptWidth = 2,
      maxWs2811Leds = 16,
      pwmAddress = 0x30000,
      pulseInAddress = 0x80000,
      shiftOutAddress = 0x50000,
      shiftInAddress = 0xA00000,
      ps2Address = 0xA80000,
      muxAddress = 0xD0000,
      gpioAAddress = 0x00000,
      sevenSegmentBAddress = 0x98000,
      quadratureAddress = 0xF8000,
      spiMasterAddress = 0x60000,
      ws2811Address = 0xD8000,
      uartAddress = 0x10000,
      sevenSegmentAAddress = 0x90000,
      gpioBAddress = 0x08000,
      i2cAddress = 0x70000,
      servoAddress = 0xC0000,
      timerAddress = 0x20000,
      toneAddress = 0x40000,
      qspiAnalogAddress = 0xF0000,
      pinInterruptAddress = 0xE0000,
      machineTimerAddress = 0xB0000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

