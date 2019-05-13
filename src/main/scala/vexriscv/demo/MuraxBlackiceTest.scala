
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
      sramAddress = 0x90000000l,
      sramDataWidth = 16,
      sramSize = 512 kB,
      sramAddressWidth = 19,
      includePs2 = true,
      includeQuadrature = true,
      includeTone = true,
      includeQspiAnalog = true,
      includeShiftOut = true,
      includeI2c = true,
      includeSevenSegmentB = true,
      includeShiftIn = true,
      includeSpiMaster = true,
      includeSevenSegmentA = true,
      pwmWidth = 5,
      servoWidth = 4,
      gpioBWidth = 32,
      gpioAWidth = 32,
      pulseInWidth = 2,
      pinInterruptWidth = 2,
      maxWs2811Leds = 16,
      ps2Address = 0xA80000,
      quadratureAddress = 0xF8000,
      ws2811Address = 0xD8000,
      toneAddress = 0x40000,
      uartAddress = 0x10000,
      pwmAddress = 0x30000,
      timerAddress = 0x20000,
      qspiAnalogAddress = 0xF0000,
      machineTimerAddress = 0xB0000,
      shiftOutAddress = 0x50000,
      i2cAddress = 0x70000,
      servoAddress = 0xC0000,
      gpioBAddress = 0x08000,
      sevenSegmentBAddress = 0x98000,
      gpioAAddress = 0x00000,
      shiftInAddress = 0xA00000,
      pulseInAddress = 0x80000,
      muxAddress = 0xD0000,
      spiMasterAddress = 0x60000,
      pinInterruptAddress = 0xE0000,
      sevenSegmentAAddress = 0x90000,

      pipelineDBus          = true,
      pipelineMainBus       = false,
      pipelineApbBridge     = true
    )))
  }
}

