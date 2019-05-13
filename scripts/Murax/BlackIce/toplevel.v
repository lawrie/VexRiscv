`timescale 1ns / 1ps

`include "config.vh"

module toplevel(
    // System clock
    input   CLK,

`ifdef INCLUDE_UART
    // Built-in uart
    input   UART_RX,
    output  UART_TX,
    input   GRESET,
`endif

`ifdef INCLUDE_JTAG
    // Jtag interface for RISC-V CPU
    input   JTAG_TCK,
    input   JTAG_TMS,
    input   JTAG_TDI,
    output  JTAG_TDO,
`endif

`ifdef INCLUDE_I2C
    // Hardware IC
    inout   SDA,
    inout   SCL,
`endif

`ifdef INCLUDE_GPIO_A
    // GPIO  A pins
    inout   [`GPIO_A_WIDTH-1:0] GPIOA,
`endif

`ifdef INCLUDE_GPIO_B
    // GPIO  B pins
    inout   [`GPIO_B_WIDTH-1:0] GPIOB,
`endif

`ifdef INCLUDE_SRAM
    // External SRAM pins
    inout   [`SRAM_DATA_WIDTH-1:0] DAT,
    output  [`SRAM_ADDRESS_WIDTH-1:0] ADR,
    output  RAMCS,
    output  RAMWE,
    output  RAMOE,
    output  RAMUB,
    output  RAMLB,
`endif

`ifdef INCLUDE_QSPI_ANALOG
    // QSPI between ice40 and STM32 co-processor
    input   QSS,
    input   QCK,
    inout   [3:0] QD,
`endif

    // Shared Leds
    output  DEBUG,
    output  DONE
  );

  // Use PLL to downclock external clock.
  wire io_mainClk;

  toplevel_pll toplevel_pll_inst(.REFERENCECLK(CLK),
                                 .PLLOUTCORE(io_mainClk),
                                 .PLLOUTGLOBAL(),
                                 .LOCK(pll_locked),
                                 .RESET(1'b1));

  // Reset Generator
  reg [7:0] reset_counter = 0;
  wire reset = !(&reset_counter);
  wire pll_locked;

  always @(posedge CLK) begin
    if (!pll_locked)
      reset_counter <= 0;
    else if (reset)
      reset_counter <= reset_counter + 1;
  end

  wire greset_falling;

  sync_reset sr (
    .clk(io_mainClk),
    .reset_in(GRESET),
    .reset_out(greset_falling)
  );

`ifdef INCLUDE_SRAM
  // SRAM
  wire [`SRAM_DATA_WIDTH-1:0] io_sram_dat_read;
  wire [`SRAM_DATA_WIDTH-1:0] io_sram_dat_write;
  wire io_sram_dat_writeEnable;

  SB_IO #(
    .PIN_TYPE(6'b 1010_01),
    .PULLUP(1'b0)
  ) sram_data_pins [`SRAM_DATA_WIDTH-1:0] (
    .PACKAGE_PIN(DAT),
    .OUTPUT_ENABLE(io_sram_dat_writeEnable),
    .D_OUT_0(io_sram_dat_write),
    .D_IN_0(io_sram_dat_read)
  );
`endif

`ifdef INCLUDE_MUX
  // Mux pins
  wire [31:0] io_mux_pins;
`endif

`ifdef INCLUDE_GPIO_A
  // GPIO A peripheral
  wire [`IO_GPIO_A_WIDTH-1:0] io_gpioA_read;
  wire [`IO_GPIO_A_WIDTH-1:0] io_gpioA_write;
  wire [`IO_GPIO_A_WIDTH-1:0] io_gpioA_writeEnable;

  wire [`GPIO_A_WIDTH-1:0] gpioA_read, gpioA_write, gpioA_writeEnable;

  SB_IO #(
    .PIN_TYPE(6'b 1010_01),
    .PULLUP(1'b 0)
  ) ioa [`GPIO_A_WIDTH-1:0] (
    .PACKAGE_PIN(GPIOA),
    .OUTPUT_ENABLE(gpioA_writeEnable),
    .D_OUT_0(gpioA_write),
    .D_IN_0(gpioA_read)
  );
`endif

`ifdef INCLUDE_GPIO_B
  // GPIO B peripheral
  wire [`IO_GPIO_B_WIDTH-1:0] io_gpioB_read;
  wire [`IO_GPIO_B_WIDTH-1:0] io_gpioB_write;
  wire [`IO_GPIO_B_WIDTH-1:0] io_gpioB_writeEnable;

  wire [`GPIO_B_WIDTH-1:0] gpioB_read;
  wire [`GPIO_B_WIDTH-1:0] gpioB_write;
  wire [`GPIO_B_WIDTH-1:0] gpioB_writeEnable;
  
  SB_IO #(
    .PIN_TYPE(6'b 1010_01),
    .PULLUP(1'b 0)
  ) iob [`GPIO_B_WIDTH-1:0] (
    .PACKAGE_PIN(GPIOB),
    .OUTPUT_ENABLE(gpioB_writeEnable),
    .D_OUT_0(gpioB_write),
    .D_IN_0(gpioB_read)
  );
`endif

`ifdef INCLUDE_QSPI_ANALOG
  // QSPI analog peripheral
  wire [3:0] io_qspi_qd_read, io_qspi_qd_write, io_qspi_qd_writeEnable;

  SB_IO #(
    .PIN_TYPE(6'b 1010_01),
    .PULLUP(1'b0)
  ) qd [3:0] (
    .PACKAGE_PIN(QD),
    .OUTPUT_ENABLE(io_qspi_qd_writeEnable),
    .D_OUT_0(io_qspi_qd_write),
    .D_IN_0(io_qspi_qd_read)
  );
`endif

`ifdef INCLUDE_I2C
  // I2C peripheral
  wire io_i2c_sda_read, io_i2c_sda_write;
  wire io_i2c_scl_read, io_i2c_scl_write;
  
  SB_IO #(
    .PIN_TYPE(6'b 1010_01),
    .PULLUP(1'b 1)
  ) sda_io (
    .PACKAGE_PIN(SDA),
    .OUTPUT_ENABLE(!io_i2c_sda_write),
    .D_OUT_0(io_i2c_sda_write),
    .D_IN_0(io_i2c_sda_read)
  );

  SB_IO #(
    .PIN_TYPE(6'b 1010_01),
    .PULLUP(1'b 1)
  ) scl_io (
    .PACKAGE_PIN(SCL),
    .OUTPUT_ENABLE(!io_i2c_scl_write),
    .D_OUT_0(io_i2c_scl_write),
    .D_IN_0(io_i2c_scl_read)
  );
`endif

`ifdef INCLUDE_SHIFT_IN
  // ShiftIn peripheral
  wire io_shiftIn_clockPin;
  wire io_shiftIn_dataPin;
`endif

`ifdef INCLUDE_SHIFT_OUT
  // ShiftOut peripheral
  wire io_shiftOut_clockPin;
  wire io_shiftOut_dataPin;
`endif

`ifdef INCLUDE_SEVEN_SEGMENT_A
  // 7-segment A peripheral
  wire io_sevenSegmentA_digitPin;
  wire [6:0] io_sevenSegmentA_segPins;
`endif

`ifdef INCLUDE_SEVEN_SEGMENT_B
  // 7-segment B peripheral
  wire io_sevenSegmentB_digitPin;
  wire [6:0] io_sevenSegmentB_segPins;
`endif

`ifdef INCLUDE_QUADRATURE
  // Quadrature peripheral
  wire io_quadrature_quadA, io_quadrature_quadB;
`endif

`ifdef INCLUDE_SERVO
  // Servo peripherals
  wire [`SERVO_WIDTH-1:0] io_servo_pins;
`endif

`ifdef INCLUDE_PWM
  // PWM pins
  wire [`PWM_WIDTH-1:0] io_pwm_pins;
`endif

`ifdef INCLUDE_TONE
  // Tone peripheral
  wire io_tone_pin;
`endif

`ifdef INCLUDE_PULSE_IN
  // PulseIn peripheral
  wire [`PULSE_IN_WIDTH-1:0] io_pulseIn_pins;
`endif

`ifdef INCLUDE_SPI_MASTER
  // SPI peripheral
  wire io_spiMaster_sclk, io_spiMaster_mosi, io_spiMaster_miso, io_spiMaster_ss;
`endif

`ifdef INCLUDE_PS2
  // PS/2 Keyboard peripheral
  wire io_ps2_ps2Clk;
  wire io_ps2_ps2Data;
`endif

`ifdef INCLUDE_WS2811
  // WS2812B LED strip
  wire io_ws2811_dout;
`endif

`ifdef INCLUDE_PIN_INTERRUPT
  // Pin interrupts
  wire [`PIN_INTERRUPT_WIDTH-1:0] io_pinInterrupt_pins;
`endif

`include "assignments.vh"

  // MuraxArduino interface
  MuraxArduino murax ( 
    .io_mainClk (io_mainClk),

`ifdef INCLUDE_JTAG
    .io_jtag_tck(JTAG_TCK),
    .io_jtag_tdi(JTAG_TDI),
    .io_jtag_tdo(JTAG_TDO),
    .io_jtag_tms(JTAG_TMS),
`endif

`ifdef INCLUDE_UART
    .io_uart_txd(UART_TX),
    .io_uart_rxd(UART_RX),
`endif

`ifdef INCLUDE_MUX
    .io_mux_pins(io_mux_pins),
`endif

`ifdef INCLUDE_GPIO_A
    .io_gpioA_read(io_gpioA_read),
    .io_gpioA_write(io_gpioA_write),
    .io_gpioA_writeEnable(io_gpioA_writeEnable),
`endif

`ifdef INCLUDE_GPIO_B
    .io_gpioB_read(io_gpioB_read),
    .io_gpioB_write(io_gpioB_write),
    .io_gpioB_writeEnable(io_gpioB_writeEnable),
`endif

`ifdef INCLUDE_PWM
    .io_pwm_pins(io_pwm_pins),
`endif

`ifdef INCLUDE_SERVO
    .io_servo_pins(io_servo_pins),
`endif

`ifdef INCLUDE_TONE
    .io_tone_pin(io_tone_pin),
`endif

`ifdef INCLUDE_SHIFT_OUT
    .io_shiftOut_clockPin(io_shiftOut_clockPin),
    .io_shiftOut_dataPin(io_shiftOut_dataPin),
`endif

`ifdef INCLUDE_SHIFT_IN
    .io_shiftIn_clockPin(io_shiftIn_clockPin),
    .io_shiftIn_dataPin(io_shiftIn_dataPin),
`endif

`ifdef INCLUDE_SPI_MASTER
    .io_spiMaster_sclk(io_spiMaster_sclk),
    .io_spiMaster_mosi(io_spiMaster_mosi),
    .io_spiMaster_miso(io_spiMaster_miso),
    .io_spiMaster_ss(io_spiMaster_ss),
`endif

`ifdef INCLUDE_PULSE_IN
    .io_pulseIn_pins(io_pulseIn_pins),
`endif

`ifdef INCLUDE_SEVEN_SEGMENT_A
    .io_sevenSegmentA_digitPin(io_sevenSegmentA_digitPin),
    .io_sevenSegmentA_segPins(io_sevenSegmentA_segPins),
`endif

`ifdef INCLUDE_SEVEN_SEGMENT_B
    .io_sevenSegmentB_digitPin(io_sevenSegmentB_digitPin),
    .io_sevenSegmentB_segPins(io_sevenSegmentB_segPins),
`endif

`ifdef INCLUDE_I2C
    .io_i2c_sda_read(io_i2c_sda_read),
    .io_i2c_sda_write(io_i2c_sda_write),
    .io_i2c_scl_read(io_i2c_scl_read),
    .io_i2c_scl_write(io_i2c_scl_write),
`endif

`ifdef INCLUDE_PIN_INTERRUPT
    .io_pinInterrupt_pins(io_pinInterrupt_pins),
`endif

`ifdef INCLUDE_PS2
    .io_ps2_ps2Clk(io_ps2_ps2Clk),
    .io_ps2_ps2Data(io_ps2_ps2Data),
`endif

`ifdef INCLUDE_QSPI_ANALOG
    .io_qspi_qss(QSS),
    .io_qspi_qck(QCK),
    .io_qspi_qd_read(io_qspi_qd_read),
    .io_qspi_qd_write(io_qspi_qd_write),
    .io_qspi_qd_writeEnable(io_qspi_qd_writeEnable),
`endif

`ifdef INCLUDE_QUADRATURE
    .io_quadrature_quadA(io_quadrature_quadA),
    .io_quadrature_quadB(io_quadrature_quadB),
`endif

`ifdef INCLUDE_WS2811
    .io_ws2811_dout(io_ws2811_dout),
`endif

`ifdef INCLUDE_SRAM
    .io_sram_addr(ADR),
    .io_sram_dat_read(io_sram_dat_read),
    .io_sram_dat_write(io_sram_dat_write),
    .io_sram_dat_writeEnable(io_sram_dat_writeEnable),
    .io_sram_we(RAMWE),
    .io_sram_oe(RAMOE),
    .io_sram_cs(RAMCS),
    .io_sram_lb(RAMLB),
    .io_sram_ub(RAMUB),
`endif

    .io_asyncReset(reset | greset_falling)
  );

endmodule

module sync_reset(
	input 	clk,
	input 	reset_in, 
	output	reset_out
	);

	wire	reset_in;
	reg	reset_in_p1;
	reg	reset_in_p2;
	wire	reset_out;

        reg old_reset;

	always @(posedge clk or posedge reset_in)
	begin

		if (reset_in) begin
			reset_in_p1 	<= 1'b1;
			reset_in_p2 	<= 1'b1;
		end
		else begin
			old_reset       <= reset_in_p2;
			reset_in_p1 	<= reset_in;
			reset_in_p2 	<= reset_in_p1;
		end
	end

	assign reset_out = (reset_in_p2 == 0) && (old_reset == 1); // greset falling

endmodule
