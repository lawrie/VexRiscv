package vexriscv.demo

import spinal.core._

// MuraxArduino Blackice configuration
object MuraxBlackice{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default.copy(
      onChipRamSize = 10 kB, 
      onChipRamHexFile = "src/main/ressource/hex/muraxArduino.hex",
      includeShiftIn = false,
      includeShiftOut = false,
      includeQuadrature = false,
      includeTone = false,
      includePs2Keyboard = false,
      includeQspiAnalog = false,
      includeSevenSegmentA = false,
      includeSevenSegmentB = false,
      includeSpi = false,
      includeI2c = false,
      pulseInWidth = 0,
      pwmWidth = 0,
      servoWidth = 0,
      pinInterruptWidth = 0,
      gpioAWidth = 0,
      gpioBWidth = 0,
      maxWs2811Leds = 0
    )))
  }
}

