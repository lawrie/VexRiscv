package vexriscv.demo

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.misc.SizeMapping
import spinal.lib.bus.simple.PipelinedMemoryBus
import spinal.lib.com.jtag.Jtag
import spinal.lib.com.spi.ddr.SpiXdrMaster
import spinal.lib.com.uart._
import spinal.lib.io.{InOutWrapper, TriStateArray}
import spinal.lib.misc.{InterruptCtrl, Prescaler, Timer}
import vexriscv.plugin._
import vexriscv.{VexRiscv, VexRiscvConfig, plugin}
import spinal.lib.com.spi.ddr._
import spinal.lib.bus.simple._
import scala.collection.mutable.ArrayBuffer
import spinal.lib.com.spi._
import spinal.lib.com.i2c._

/**
 * Created by PIC32F_USER on 28/07/2017.
 *
 * Murax is a very light SoC which could work without any external component.
 * - ICE40-hx8k + icestorm =>  53 Mhz, 2142 LC
 * - 0.37 DMIPS/Mhz
 * - 8 kB of on-chip ram
 * - JTAG debugger (eclipse/GDB/openocd ready)
 * - Interrupt support
 * - APB bus for peripherals
 * - 32 GPIO pin
 * - one 16 bits prescaler, two 16 bits timers
 * - one UART with tx/rx fifo
 *
 * And MuraxArduino adds lots more peripherals.
 *
 */

case class MuraxArduinoConfig(
                       coreFrequency           : HertzNumber,
                       onChipRamSize           : BigInt,
                       ramAddress              : BigInt,
                       sramAddress             : BigInt,
                       ioAddress               : BigInt,
                       sramSize                : BigInt,
                       sramAddressWidth        : Int,
                       sramDataWidth           : Int,
                       onChipRamHexFile        : String,
                       pipelineDBus            : Boolean,
                       pipelineMainBus         : Boolean,
                       pipelineApbBridge       : Boolean,
                       gpioAWidth              : Int,
                       gpioBWidth              : Int,
                       pinInterruptWidth       : Int,
                       pwmWidth                : Int,
                       servoWidth              : Int,
                       pulseInWidth            : Int,
                       maxWs2811Leds           : Int,
                       includeSpiMaster        : Boolean,
                       includeI2c              : Boolean,
                       includeTone             : Boolean,
                       includeShiftIn          : Boolean,
                       includeShiftOut         : Boolean,
                       includeQuadrature       : Boolean,
                       includePs2              : Boolean,
                       includeQspiAnalog       : Boolean,
                       includeSevenSegmentA    : Boolean,
                       includeSevenSegmentB    : Boolean,
                       gpioAAddress            : Int,
                       gpioBAddress            : Int,
                       uartAddress             : Int,
                       timerAddress            : Int,
                       pwmAddress              : Int,
                       toneAddress             : Int,
                       shiftOutAddress         : Int,
                       spiMasterAddress        : Int,
                       i2cAddress              : Int,
                       pulseInAddress          : Int,
                       sevenSegmentAAddress    : Int,
                       sevenSegmentBAddress    : Int,
                       shiftInAddress          : Int,
                       ps2Address              : Int,
                       machineTimerAddress     : Int,
                       servoAddress            : Int,
                       muxAddress              : Int,
                       ws2811Address           : Int,
                       pinInterruptAddress     : Int,
                       qspiAnalogAddress       : Int,
                       quadratureAddress       : Int,
                       uartCtrlConfig          : UartCtrlMemoryMappedConfig,
                       spiMasterCtrlConfig     : SpiMasterCtrlMemoryMappedConfig,
                       i2cCtrlConfig           : I2cSlaveMemoryMappedGenerics,
                       xipConfig               : SpiXdrMasterCtrl.MemoryMappingParameters,
                       hardwareBreakpointCount : Int,
                       cpuPlugins              : ArrayBuffer[Plugin[VexRiscv]]){
  require(pipelineApbBridge || pipelineMainBus, "At least pipelineMainBus or pipelineApbBridge should be enable to avoid wipe transactions")
  val genXip = xipConfig != null

}

object MuraxArduinoConfig{
  def default : MuraxArduinoConfig = default(false, 0x80000000l)
  def default(withXip : Boolean, ramAddress: BigInt) =  MuraxArduinoConfig(
    coreFrequency         = 50 MHz,
    ramAddress            = ramAddress,
    ioAddress             = 0xF0000000l,
    sramAddress           = 0x90000000l,
    sramSize              = 512 kB,
    sramAddressWidth      = 19,
    sramDataWidth         = 16,
    onChipRamSize         = 8 kB,
    onChipRamHexFile      = null,
    pipelineDBus          = true,
    pipelineMainBus       = false,
    pipelineApbBridge     = true,
    gpioAWidth            = 32,
    gpioBWidth            = 32,
    pinInterruptWidth     = 2,
    pwmWidth              = 5,
    servoWidth            = 4,
    pulseInWidth          = 2,
    maxWs2811Leds         = 8,
    includeSpiMaster      = true,
    includeI2c            = true,
    includeTone           = true,
    includeShiftIn        = true,
    includeShiftOut       = true,
    includeQuadrature     = true,
    includePs2            = true,
    includeQspiAnalog     = true,
    includeSevenSegmentA  = true,
    includeSevenSegmentB  = true,
    gpioAAddress          = 0x00000,
    gpioBAddress          = 0x08000,
    uartAddress           = 0x10000,
    timerAddress          = 0x20000,
    pwmAddress            = 0x30000,
    toneAddress           = 0x40000,
    shiftOutAddress       = 0x50000,
    spiMasterAddress      = 0x60000,
    i2cAddress            = 0x70000,
    pulseInAddress        = 0x80000,
    sevenSegmentAAddress  = 0x90000,
    sevenSegmentBAddress  = 0x98000,
    shiftInAddress        = 0xA0000,
    ps2Address            = 0xA8000,
    machineTimerAddress   = 0xB0000,
    servoAddress          = 0xC0000,
    muxAddress            = 0xD0000,
    ws2811Address         = 0xD8000,
    pinInterruptAddress   = 0xE0000,
    qspiAnalogAddress     = 0xF0000,
    quadratureAddress     = 0xF8000,
    xipConfig = ifGen(withXip) (SpiXdrMasterCtrl.MemoryMappingParameters(
      SpiXdrMasterCtrl.Parameters(8, 12, SpiXdrParameter(2, 2, 1)).addFullDuplex(0,1,false),
      cmdFifoDepth = 32,
      rspFifoDepth = 32,
      xip = SpiXdrMasterCtrl.XipBusParameters(addressWidth = 24, dataWidth = 32)
    )),
    hardwareBreakpointCount = if(withXip) 3 else 0,
    cpuPlugins = ArrayBuffer( //DebugPlugin added by the toplevel
      new IBusSimplePlugin(
        resetVector = if(withXip) 0xF001E000l else ramAddress,
        cmdForkOnSecondStage = true,
        cmdForkPersistence = withXip, //Required by the Xip controller
        prediction = NONE,
        catchAccessFault = false,
        compressedGen = false
      ),
      new DBusSimplePlugin(
        catchAddressMisaligned = false,
        catchAccessFault = false,
        earlyInjection = false
      ),
      new CsrPlugin(CsrPluginConfig.smallest(mtvecInit = if(withXip) 0xE0040020l else (ramAddress | 0x20))),
      new DecoderSimplePlugin(
        catchIllegalInstruction = false
      ),
      new RegFilePlugin(
        regFileReadyKind = plugin.SYNC,
        zeroBoot = false
      ),
      new IntAluPlugin,
      new SrcPlugin(
        separatedAddSub = false,
        executeInsertion = false
      ),
      new LightShifterPlugin,
      new HazardSimplePlugin(
        bypassExecute = false,
        bypassMemory = false,
        bypassWriteBack = false,
        bypassWriteBackBuffer = false,
        pessimisticUseSrc = false,
        pessimisticWriteRegFile = false,
        pessimisticAddressMatch = false
      ),
      new BranchPlugin(
        earlyBranch = false,
        catchAddressMisaligned = false
      ),
      new YamlPlugin("cpu0.yaml")
    ),
    uartCtrlConfig = UartCtrlMemoryMappedConfig(
      uartCtrlConfig = UartCtrlGenerics(
        dataWidthMax      = 8,
        clockDividerWidth = 20,
        preSamplingSize   = 1,
        samplingSize      = 3,
        postSamplingSize  = 1
      ),
      initConfig = UartCtrlInitConfig(
        baudrate = 115200,
        dataLength = 7,  //7 => 8 bits
        parity = UartParityType.NONE,
        stop = UartStopType.ONE
      ),
      busCanWriteClockDividerConfig = false,
      busCanWriteFrameConfig = false,
      txFifoDepth = 16,
      rxFifoDepth = 16
    ),
    spiMasterCtrlConfig = SpiMasterCtrlMemoryMappedConfig(
      SpiMasterCtrlGenerics(
        ssWidth   = 1,
        timerWidth = 32,
        dataWidth = 8)
    ),
    i2cCtrlConfig = I2cSlaveMemoryMappedGenerics(
     ctrlGenerics = I2cSlaveGenerics(
       samplingWindowSize = 3,
       samplingClockDividerWidth = 10 bits,
       timeoutWidth = 20 bits
     ),
     addressFilterCount = 0,
     masterGenerics = I2cMasterMemoryMappedGenerics(
       timerWidth = 12
     )
    )
  )

  def fast = {
    val config = default

    //Replace HazardSimplePlugin to get datapath bypass
    config.cpuPlugins(config.cpuPlugins.indexWhere(_.isInstanceOf[HazardSimplePlugin])) = new HazardSimplePlugin(
      bypassExecute = true,
      bypassMemory = true,
      bypassWriteBack = true,
      bypassWriteBackBuffer = true
    )
//    config.cpuPlugins(config.cpuPlugins.indexWhere(_.isInstanceOf[LightShifterPlugin])) = new FullBarrelShifterPlugin()

    config
  }
}

case class MuraxArduino(config : MuraxArduinoConfig) extends Component{
  import config._

  val io = new Bundle {
    //Clocks / reset
    val asyncReset = in Bool
    val mainClk = in Bool

    //Main components IO
    val jtag = slave(Jtag())

    //Peripherals IO
    val gpioA = ifGen(gpioAWidth > 0) (master(TriStateArray(gpioAWidth bits)))
    val gpioB = ifGen(gpioBWidth > 0) (master(TriStateArray(gpioBWidth bits)))
    val uart = master(Uart())
    val pinInterrupt = ifGen(pinInterruptWidth > 0) (master(PinInterrupt(pinInterruptWidth)))
    val pwm = ifGen(pwmWidth > 0) (master(Pwm(pwmWidth)))
    val ws2811 = ifGen(maxWs2811Leds > 0) (master(Ws2811()))
    val servo = ifGen(servoWidth > 0) (master(Servo(servoWidth)))
    val mux = master(Mux())
    val machineTimer = master(MachineTimer())
    val tone = ifGen(includeTone) (master(Tone()))
    val shiftOut = ifGen(includeShiftOut) (master(ShiftOut()))
    val spiMaster = ifGen(includeSpiMaster) (master(SpiMaster()))
    val i2c = ifGen(includeI2c) (master(I2c()))
    val pulseIn = ifGen(pulseInWidth > 0) (master(PulseIn(pulseInWidth)))
    val sevenSegmentA = ifGen(includeSevenSegmentA) (master(SevenSegment()))
    val sevenSegmentB = ifGen(includeSevenSegmentB) (master(SevenSegment()))
    val shiftIn = ifGen(includeShiftIn) (master(ShiftIn()))
    val qspi = ifGen(includeQspiAnalog) (master(Qspi()))
    val quadrature = ifGen(includeQuadrature) (master(Quadrature()))
    val ps2 = ifGen(includePs2) (master(PS2Keyboard()))
    val sram = master(SramInterface(SramLayout(sramAddressWidth, sramDataWidth)))
    val xip = ifGen(genXip)(master(SpiXdrMaster(xipConfig.ctrl.spi)))
  }

  val resetCtrlClockDomain = ClockDomain(
    clock = io.mainClk,
    config = ClockDomainConfig(
      resetKind = BOOT
    )
  )

  val resetCtrl = new ClockingArea(resetCtrlClockDomain) {
    val mainClkResetUnbuffered  = False

    //Implement an counter to keep the reset axiResetOrder high 64 cycles
    // Also this counter will automatically do a reset when the system boot.
    val systemClkResetCounter = Reg(UInt(6 bits)) init(0)
    when(systemClkResetCounter =/= U(systemClkResetCounter.range -> true)){
      systemClkResetCounter := systemClkResetCounter + 1
      mainClkResetUnbuffered := True
    }
    when(BufferCC(io.asyncReset)){
      systemClkResetCounter := 0
    }

    //Create all reset used later in the design
    val mainClkReset = RegNext(mainClkResetUnbuffered)
    val systemReset  = RegNext(mainClkResetUnbuffered)
  }

  val systemClockDomain = ClockDomain(
    clock = io.mainClk,
    reset = resetCtrl.systemReset,
    frequency = FixedFrequency(coreFrequency)
  )

  val debugClockDomain = ClockDomain(
    clock = io.mainClk,
    reset = resetCtrl.mainClkReset,
    frequency = FixedFrequency(coreFrequency)
  )

  val system = new ClockingArea(systemClockDomain) {
    val pipelinedMemoryBusConfig = PipelinedMemoryBusConfig(
      addressWidth = 32,
      dataWidth = 32
    )

    //Arbiter of the cpu dBus/iBus to drive the mainBus
    //Priority to dBus, !! cmd transactions can change on the fly !!
    val mainBusArbiter = new MuraxMasterArbiter(pipelinedMemoryBusConfig)

    //Instanciate the CPU
    val cpu = new VexRiscv(
      config = VexRiscvConfig(
        plugins = cpuPlugins += new DebugPlugin(debugClockDomain, hardwareBreakpointCount)
      )
    )

    //Checkout plugins used to instanciate the CPU to connect them to the SoC
    val timerInterrupt = False
    val externalInterrupt = False
    for(plugin <- cpu.plugins) plugin match{
      case plugin : IBusSimplePlugin =>
        mainBusArbiter.io.iBus.cmd <> plugin.iBus.cmd
        mainBusArbiter.io.iBus.rsp <> plugin.iBus.rsp
      case plugin : DBusSimplePlugin => {
        if(!pipelineDBus)
          mainBusArbiter.io.dBus <> plugin.dBus
        else {
          mainBusArbiter.io.dBus.cmd << plugin.dBus.cmd.halfPipe()
          mainBusArbiter.io.dBus.rsp <> plugin.dBus.rsp
        }
      }
      case plugin : CsrPlugin        => {
        plugin.externalInterrupt := externalInterrupt
        plugin.timerInterrupt := timerInterrupt
      }
      case plugin : DebugPlugin         => plugin.debugClockDomain{
        resetCtrl.systemReset setWhen(RegNext(plugin.io.resetOut))
        io.jtag <> plugin.io.bus.fromJtag()
      }
      case _ =>
    }

    //****** MainBus slaves ********
    val mainBusMapping = ArrayBuffer[(PipelinedMemoryBus,SizeMapping)]()
    val ram = new MuraxPipelinedMemoryBusRam(
      onChipRamSize = onChipRamSize,
      onChipRamHexFile = onChipRamHexFile,
      pipelinedMemoryBusConfig = pipelinedMemoryBusConfig
    )
    mainBusMapping += ram.io.bus -> (ramAddress, onChipRamSize)

    val sramCtrl = new MuraxPipelinedMemoryBusSram(pipelinedMemoryBusConfig, 
                                                   SramLayout(sramAddressWidth, sramDataWidth))
    sramCtrl.io.sram <> io.sram
    mainBusMapping += sramCtrl.io.bus -> (sramAddress, sramSize)

    val apbBridge = new PipelinedMemoryBusToApbBridge(
      apb3Config = Apb3Config(
        addressWidth = 20,
        dataWidth = 32
      ),
      pipelineBridge = pipelineApbBridge,
      pipelinedMemoryBusConfig = pipelinedMemoryBusConfig
    )
    mainBusMapping += apbBridge.io.pipelinedMemoryBus -> (ioAddress, 1 MB)

    //******** APB peripherals *********
    val apbMapping = ArrayBuffer[(Apb3, SizeMapping)]()

    val uartCtrl = Apb3UartCtrl(uartCtrlConfig)
    uartCtrl.io.uart <> io.uart
    externalInterrupt setWhen(uartCtrl.io.interrupt)
    apbMapping += uartCtrl.io.apb  -> (uartAddress, 4 kB)

    val timer = new MuraxApb3Timer()
    timerInterrupt setWhen(timer.io.interrupt)
    apbMapping += timer.io.apb     -> (timerAddress, 4 kB)

    val muxCtrl = Apb3MuxCtrl()
    muxCtrl.io.mux <> io.mux
    apbMapping += muxCtrl.io.apb   -> (muxAddress, 2 kB)

    val machineTimerCtrl = Apb3MachineTimerCtrl(coreFrequency.toInt / 1000000)
    apbMapping += machineTimerCtrl.io.apb   -> (machineTimerAddress, 4 kB)

    if (gpioAWidth > 0) {
      val gpioACtrl = Apb3Gpio(gpioWidth = gpioAWidth)
      io.gpioA <> gpioACtrl.io.gpio
      apbMapping += gpioACtrl.io.apb -> (gpioAAddress, 4 kB)
    }

    if (gpioBWidth > 0) {
      val gpioBCtrl = Apb3Gpio(gpioWidth = gpioBWidth)
      io.gpioB <> gpioBCtrl.io.gpio
      apbMapping += gpioBCtrl.io.apb -> (gpioBAddress, 4 kB)
    }

    if (pinInterruptWidth > 0) {
      val pinInterruptCtrl = Apb3PinInterruptCtrl(pinInterruptWidth)
      pinInterruptCtrl.io.pinInterrupt <> io.pinInterrupt
      externalInterrupt setWhen(pinInterruptCtrl.io.interrupt)
      apbMapping += pinInterruptCtrl.io.apb  -> (pinInterruptAddress, 4 kB)
    }

    if (pwmWidth > 0) {
      val pwmCtrl = Apb3PwmCtrl(pwmWidth)
      pwmCtrl.io.pwm <> io.pwm
      apbMapping += pwmCtrl.io.apb   -> (pwmAddress, 4 kB)
    }

    if (maxWs2811Leds > 0) {
      val ws2811Ctrl = Apb3Ws2811Ctrl(maxLeds = maxWs2811Leds, clockHz = coreFrequency.toInt)
      ws2811Ctrl.io.ws2811 <> io.ws2811
      apbMapping += ws2811Ctrl.io.apb   -> (ws2811Address, 2 kB)
    }

    if (servoWidth > 0) {
      val servoCtrl = Apb3ServoCtrl(servoWidth)
      servoCtrl.io.servo <> io.servo
      apbMapping += servoCtrl.io.apb   -> (servoAddress, 4 kB)
    }

    if (includeTone) {
      val toneCtrl = Apb3ToneCtrl(coreFrequency.toInt)
      toneCtrl.io.tone <> io.tone
      apbMapping += toneCtrl.io.apb   -> (toneAddress, 4 kB)
    }

    if (includeShiftOut) {
      val shiftOutCtrl = Apb3ShiftOutCtrl()
      shiftOutCtrl.io.shiftOut <> io.shiftOut
      apbMapping += shiftOutCtrl.io.apb   -> (shiftOutAddress, 4 kB)
    }

    if (includeSpiMaster) {
      val spiMasterCtrl = Apb3SpiMasterCtrl(spiMasterCtrlConfig)
      spiMasterCtrl.io.spi <> io.spiMaster
      apbMapping += spiMasterCtrl.io.apb   -> (spiMasterAddress, 4 kB)
    }

    if (includeI2c) {
      val i2cCtrl = Apb3I2cCtrl(i2cCtrlConfig)
      i2cCtrl.io.i2c <> io.i2c
      apbMapping += i2cCtrl.io.apb   -> (i2cAddress, 4 kB)
    }

    if (pulseInWidth > 0) {
      val pulseInCtrl = Apb3PulseInCtrl(pulseInWidth)
      pulseInCtrl.io.pulseIn <> io.pulseIn
      apbMapping += pulseInCtrl.io.apb   -> (pulseInAddress, 4 kB)
    }

    if (includeSevenSegmentA) {
      val sevenSegmentACtrl = Apb3SevenSegmentCtrl()
      sevenSegmentACtrl.io.sevenSegment <> io.sevenSegmentA
      apbMapping += sevenSegmentACtrl.io.apb   -> (sevenSegmentAAddress, 2 kB)
    }

    if (includeSevenSegmentB) {
      val sevenSegmentBCtrl = Apb3SevenSegmentCtrl()
      sevenSegmentBCtrl.io.sevenSegment <> io.sevenSegmentB
      apbMapping += sevenSegmentBCtrl.io.apb   -> (sevenSegmentBAddress, 2 kB)
    }

    if (includeShiftIn) {
      val shiftInCtrl = Apb3ShiftInCtrl()
      shiftInCtrl.io.shiftIn <> io.shiftIn
      apbMapping += shiftInCtrl.io.apb   -> (shiftInAddress, 2 kB)
    }

    if (includeQspiAnalog) {
      val qspiCtrl = Apb3QspiCtrl()
      qspiCtrl.io.qspi <> io.qspi
      apbMapping += qspiCtrl.io.apb   -> (qspiAnalogAddress, 2 kB)
    }

    if (includeQuadrature) {
      val quadratureCtrl = Apb3QuadratureCtrl(8)
      quadratureCtrl.io.quadrature <> io.quadrature
      apbMapping += quadratureCtrl.io.apb   -> (quadratureAddress, 2 kB)
    }

    if (includePs2) {
      val ps2Ctrl = Apb3PS2KeyboardCtrl()
      ps2Ctrl.io.ps2 <> io.ps2
      apbMapping += ps2Ctrl.io.apb   -> (ps2Address, 2 kB)
    }

    val xip = ifGen(genXip)(new Area{
      val ctrl = Apb3SpiXdrMasterCtrl(xipConfig)
      ctrl.io.spi <> io.xip
      externalInterrupt setWhen(ctrl.io.interrupt)
      apbMapping += ctrl.io.apb     -> (0x1F000, 4 kB)

      val accessBus = new PipelinedMemoryBus(PipelinedMemoryBusConfig(24,32))
      mainBusMapping += accessBus -> (0xE0000000l, 16 MB)

      ctrl.io.xip.cmd.valid <> (accessBus.cmd.valid && !accessBus.cmd.write)
      ctrl.io.xip.cmd.ready <> accessBus.cmd.ready
      ctrl.io.xip.cmd.payload <> accessBus.cmd.address

      ctrl.io.xip.rsp.valid <> accessBus.rsp.valid
      ctrl.io.xip.rsp.payload <> accessBus.rsp.data

      val bootloader = Apb3Rom("src/main/c/murax/xipBootloader/crt.bin")
      apbMapping += bootloader.io.apb     -> (0x1E000, 4 kB)
    })

    //******** Memory mappings *********
    val apbDecoder = Apb3Decoder(
      master = apbBridge.io.apb,
      slaves = apbMapping
    )

    val mainBusDecoder = new Area {
      val logic = new MuraxPipelinedMemoryBusDecoder(
        master = mainBusArbiter.io.masterBus.cmdS2mPipe(),
        specification = mainBusMapping,
        pipelineMaster = pipelineMainBus
      )
    }
  }
}

object MuraxArduino{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default))
  }
}

// Runs SREC HEX bootloader that works with f32c/arduino
object MuraxArduinoWithRamInit{
  def main(args: Array[String]) {
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default.copy(onChipRamSize = 10 kB, onChipRamHexFile = "src/main/ressource/hex/muraxArduino.hex")))
  }
}
