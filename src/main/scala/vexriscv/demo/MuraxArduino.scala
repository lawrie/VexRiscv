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
 */


case class MuraxArduinoConfig(
                       coreFrequency           : HertzNumber,
                       onChipRamSize           : BigInt,
                       sramSize                : BigInt,
                       sramAddressWidth        : Int,
                       sramDataWidth           : Int,
                       onChipRamHexFile        : String,
                       pipelineDBus            : Boolean,
                       pipelineMainBus         : Boolean,
                       pipelineApbBridge       : Boolean,
                       gpioWidth               : Int,
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
  def default : MuraxArduinoConfig = default(false)
  def default(withXip : Boolean) =  MuraxArduinoConfig(
    coreFrequency         = 50 MHz,
    sramSize              = 512 kB,
    sramAddressWidth      = 19,
    sramDataWidth         = 16,
    onChipRamSize         = 8 kB,
    onChipRamHexFile      = null,
    pipelineDBus          = true,
    pipelineMainBus       = false,
    pipelineApbBridge     = true,
    gpioWidth = 32,
    xipConfig = ifGen(withXip) (SpiXdrMasterCtrl.MemoryMappingParameters(
      SpiXdrMasterCtrl.Parameters(8, 12, SpiXdrParameter(2, 2, 1)).addFullDuplex(0,1,false),
      cmdFifoDepth = 32,
      rspFifoDepth = 32,
      xip = SpiXdrMasterCtrl.XipBusParameters(addressWidth = 24, dataWidth = 32)
    )),
    hardwareBreakpointCount = if(withXip) 3 else 0,
    cpuPlugins = ArrayBuffer( //DebugPlugin added by the toplevel
      new IBusSimplePlugin(
        resetVector = if(withXip) 0xF001E000l else 0x80000000l,
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
      new CsrPlugin(CsrPluginConfig.smallest(mtvecInit = if(withXip) 0xE0040020l else 0x80000020l)),
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
    val gpioA = master(TriStateArray(gpioWidth bits))
    val uart = master(Uart())
    val pinInterrupt = master(PinInterrupt(2))

    val pwm = master(Pwm(3))
    val servo = master(Servo())
    val mux = master(Mux())
    val machineTimer = master(MachineTimer())
    val tone = master(Tone())
    val shiftOut = master(ShiftOut())
    val spiMaster = master(SpiMaster())
    val i2c = master(I2c())
    val pulseIn = master(PulseIn())
    val sevenSegment = master(SevenSegment())
    val shiftIn = master(ShiftIn())
    val qspi = master(Qspi())
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
    mainBusMapping += ram.io.bus -> (0x80000000l, onChipRamSize)

    val sramCtrl = new MuraxPipelinedMemoryBusSram(pipelinedMemoryBusConfig, 
                                                   SramLayout(sramAddressWidth, sramDataWidth))
    sramCtrl.io.sram <> io.sram
    mainBusMapping += sramCtrl.io.bus -> (0x90000000l, sramSize)

    val apbBridge = new PipelinedMemoryBusToApbBridge(
      apb3Config = Apb3Config(
        addressWidth = 20,
        dataWidth = 32
      ),
      pipelineBridge = pipelineApbBridge,
      pipelinedMemoryBusConfig = pipelinedMemoryBusConfig
    )
    mainBusMapping += apbBridge.io.pipelinedMemoryBus -> (0xF0000000l, 1 MB)



    //******** APB peripherals *********
    val apbMapping = ArrayBuffer[(Apb3, SizeMapping)]()
    val gpioACtrl = Apb3Gpio(gpioWidth = gpioWidth)
    io.gpioA <> gpioACtrl.io.gpio
    apbMapping += gpioACtrl.io.apb -> (0x00000, 4 kB)

    val uartCtrl = Apb3UartCtrl(uartCtrlConfig)
    uartCtrl.io.uart <> io.uart
    externalInterrupt setWhen(uartCtrl.io.interrupt)
    apbMapping += uartCtrl.io.apb  -> (0x10000, 4 kB)

    val pinInterruptCtrl = Apb3PinInterruptCtrl(2)
    pinInterruptCtrl.io.pinInterrupt <> io.pinInterrupt
    externalInterrupt setWhen(pinInterruptCtrl.io.interrupt)
    apbMapping += pinInterruptCtrl.io.apb  -> (0xE0000, 4 kB)

    val timer = new MuraxApb3Timer()
    timerInterrupt setWhen(timer.io.interrupt)
    apbMapping += timer.io.apb     -> (0x20000, 4 kB)

    val pwmCtrl = Apb3PwmCtrl(3)
    pwmCtrl.io.pwm <> io.pwm
    apbMapping += pwmCtrl.io.apb   -> (0x30000, 4 kB)

    val servoCtrl = Apb3ServoCtrl()
    servoCtrl.io.servo <> io.servo
    apbMapping += servoCtrl.io.apb   -> (0xC0000, 4 kB)

    val muxCtrl = Apb3MuxCtrl()
    muxCtrl.io.mux <> io.mux
    apbMapping += muxCtrl.io.apb   -> (0xD0000, 4 kB)

    val machineTimerCtrl = Apb3MachineTimerCtrl()
    apbMapping += machineTimerCtrl.io.apb   -> (0xB0000, 4 kB)

    val toneCtrl = Apb3ToneCtrl()
    toneCtrl.io.tone <> io.tone
    apbMapping += toneCtrl.io.apb   -> (0x40000, 4 kB)

    val shiftOutCtrl = Apb3ShiftOutCtrl()
    shiftOutCtrl.io.shiftOut <> io.shiftOut
    apbMapping += shiftOutCtrl.io.apb   -> (0x50000, 4 kB)

    val spiMasterCtrl = Apb3SpiMasterCtrl(spiMasterCtrlConfig)
    spiMasterCtrl.io.spi <> io.spiMaster
    apbMapping += spiMasterCtrl.io.apb   -> (0x60000, 4 kB)

    val i2cCtrl = Apb3I2cCtrl(i2cCtrlConfig)
    i2cCtrl.io.i2c <> io.i2c
    apbMapping += i2cCtrl.io.apb   -> (0x70000, 4 kB)

    val pulseInCtrl = Apb3PulseInCtrl()
    pulseInCtrl.io.pulseIn <> io.pulseIn
    apbMapping += pulseInCtrl.io.apb   -> (0x80000, 4 kB)

    val sevenSegmentCtrl = Apb3SevenSegmentCtrl()
    sevenSegmentCtrl.io.sevenSegment <> io.sevenSegment
    apbMapping += sevenSegmentCtrl.io.apb   -> (0x90000, 4 kB)

    val shiftInCtrl = Apb3ShiftInCtrl()
    shiftInCtrl.io.shiftIn <> io.shiftIn
    apbMapping += shiftInCtrl.io.apb   -> (0xA0000, 4 kB)

    val qspiCtrl = Apb3QspiCtrl()
    qspiCtrl.io.qspi <> io.qspi
    apbMapping += qspiCtrl.io.apb   -> (0xF0000, 4 kB)

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
    SpinalVerilog(MuraxArduino(MuraxArduinoConfig.default.copy(onChipRamSize = 12 kB, onChipRamHexFile = "src/main/ressource/hex/muraxArduino.hex")))
  }
}
