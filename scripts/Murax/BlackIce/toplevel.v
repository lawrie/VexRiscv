`timescale 1ns / 1ps

module toplevel(
    input   CLK,
    input   GRESET,
    input   BUT1,
    input   BUT2,
    input   UART_RX,
    output  UART_TX,
    output  LED1,
    output  LED2,
    output  LED3,
    output  LED4,
    output  SPI_SCK,
    output  SPI_MOSI,
    output  SPI_SS,
    input   SPI_MISO,
    input   [1:0] INPUT,
    output  [6:0] OUTPUT,
    inout   [21:0] GPIO,
    output  D,
    output  [6:0] SEG,
    inout   SDA,
    inout   SCL,
    inout   [15:0] DAT,
    output  [17:0] ADR,
    output  RAMCS,
    output  RAMWE,
    output  RAMOE,
    output  RAMUB,
    output  RAMLB,
    input   JTAG_TCK,
    input   JTAG_TMS,
    input   JTAG_TDI,
    output  JTAG_TDO,
    input   QSS,
    input   QCK,
    inout   [3:0] QD,
    output  DEBUG,
    output  DONE
  );

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

  wire [15:0] io_sram_dat_read;
  wire [15:0] io_sram_dat_write;
  wire io_sram_dat_writeEnable;

  SB_IO #(
    .PIN_TYPE(6'b 1010_01),
    .PULLUP(1'b0)
  ) sram_data_pins [15:0] (
    .PACKAGE_PIN(DAT),
    .OUTPUT_ENABLE(io_sram_dat_writeEnable),
    .D_OUT_0(io_sram_dat_write),
    .D_IN_0(io_sram_dat_read)
  );

  wire [31:0] io_gpioA_read;
  wire [31:0] io_gpioA_write;
  wire [31:0] io_gpioA_writeEnable;

  assign LED1 = io_gpioA_write[0];
  assign LED2 = io_gpioA_write[1];
  assign LED3 = io_gpioA_write[2];
  assign LED4 = io_gpioA_write[3];

  assign OUTPUT[0] = io_gpioA_write[4];
 
  wire io_mainClk;

  assign io_gpioA_read[7:0] = 0; // Output only
  assign io_gpioA_read[8] = BUT1;
  assign io_gpioA_read[9] = BUT2;

  wire [21:0] io_gpio_read, io_gpio_write, io_gpio_writeEnable;

  assign io_gpioA_read[31:10] = io_gpio_read;
  assign io_gpio_writeEnable = io_gpioA_writeEnable[31:10];
   
  SB_IO #(
    .PIN_TYPE(6'b 1010_01),
    .PULLUP(1'b 0)
  ) ios [21:0] (
    .PACKAGE_PIN(GPIO),
    .OUTPUT_ENABLE(io_gpio_writeEnable),
    .D_OUT_0(io_gpio_write),
    .D_IN_0(io_gpio_read)
  );

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

  wire [31:0] io_mux_pins;
   
  // Use PLL to downclock external clock.
  toplevel_pll toplevel_pll_inst(.REFERENCECLK(CLK),
                                 .PLLOUTCORE(io_mainClk),
                                 .PLLOUTGLOBAL(),
                                 .LOCK(pll_locked),
                                 .RESET(1'b1));

  // -------------------------------
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

  wire io_shiftIn_clockPin;
  wire io_shiftOut_clockPin;
  wire io_shiftOut_dataPin;

  assign OUTPUT[6] = io_mux_pins[0] ? io_shiftIn_clockPin : io_gpioA_write[5];
  assign OUTPUT[5] = io_mux_pins[3] ? io_servo_pins[1] :
                     io_mux_pins[1] ? io_shiftOut_clockPin : io_gpioA_write[6];
  assign OUTPUT[4] = io_mux_pins[3] ? io_servo_pins[0] :
                     io_mux_pins[1] ? io_shiftOut_dataPin : io_gpioA_write[7];

  wire io_sevenSegmentB_digitPin;
  wire [6:0] io_sevenSegmentB_segPins;

  assign io_gpio_write[5:0] = io_gpioA_write[15:10];
  assign io_gpio_write[13:10] = io_gpioA_write[23:20];
  assign io_gpio_write[21:18] = io_gpioA_write[31:28];
  
  assign io_gpio_write[17] = io_mux_pins[2] ? io_sevenSegmentB_digitPin : io_gpioA_write[27];
  assign io_gpio_write[16:14] = io_mux_pins[2] ? 
                                 io_sevenSegmentB_segPins[2:0] : 
                                 io_gpioA_write[26:24];
  assign io_gpio_write[9:6] = io_mux_pins[2] ? io_sevenSegmentB_segPins[6:3] : io_gpioA_write[19:16];

  wire io_quadrature_quadA, io_quadrature_quadB;

  assign io_quadrature_quadA = io_gpio_read[6];
  assign io_quadrature_quadB = io_gpio_read[7];

  wire [3:0] io_servo_pins;
  assign OUTPUT[3] = io_servo_pins[3];

  wire [2:0] io_pwm_pins;
  assign OUTPUT[1] = io_pwm_pins[0];
  assign DEBUG = io_pwm_pins[1];
  assign DONE = io_pwm_pins[2];

  wire io_tone_pin;
  assign OUTPUT[2] = io_mux_pins[3] ? io_servo_pins[2] : io_tone_pin;

  wire io_pulseIn_pin;
  assign io_pulseIn_pin = INPUT[0];

  wire io_shiftIn_dataPin;
  assign io_shiftIn_dataPin = INPUT[1];
  
  MuraxArduino murax ( 
    .io_asyncReset(reset | greset_falling),
    .io_mainClk (io_mainClk),
    .io_jtag_tck(JTAG_TCK),
    .io_jtag_tdi(JTAG_TDI),
    .io_jtag_tdo(JTAG_TDO),
    .io_jtag_tms(JTAG_TMS),
    .io_gpioA_read(io_gpioA_read),
    .io_gpioA_write(io_gpioA_write),
    .io_gpioA_writeEnable(io_gpioA_writeEnable),
    .io_uart_txd(UART_TX),
    .io_uart_rxd(UART_RX),
    .io_pwm_pins(io_pwm_pins),
    .io_servo_pins(io_servo_pins),
    .io_tone_pin(io_tone_pin),
    .io_shiftOut_clockPin(io_shiftOut_clockPin),
    .io_shiftOut_dataPin(io_shiftOut_dataPin),
    .io_shiftIn_clockPin(io_shiftIn_clockPin),
    .io_shiftIn_dataPin(io_shiftIn_dataPin),
    .io_spiMaster_sclk(SPI_SCK),
    .io_spiMaster_mosi(SPI_MOSI),
    .io_spiMaster_miso(SPI_MISO),
    .io_spiMaster_ss(SPI_SS),
    .io_pulseIn_pin(io_pulseIn_pin),
    .io_sevenSegmentA_digitPin(D),
    .io_sevenSegmentA_segPins(SEG),
    .io_sevenSegmentB_digitPin(io_sevenSegmentB_digitPin),
    .io_sevenSegmentB_segPins(io_sevenSegmentB_segPins),
    .io_i2c_sda_read(io_i2c_sda_read),
    .io_i2c_sda_write(io_i2c_sda_write),
    .io_i2c_scl_read(io_i2c_scl_read),
    .io_i2c_scl_write(io_i2c_scl_write),
    .io_sram_addr(ADR),
    .io_sram_dat_read(io_sram_dat_read),
    .io_sram_dat_write(io_sram_dat_write),
    .io_sram_dat_writeEnable(io_sram_dat_writeEnable),
    .io_sram_we(RAMWE),
    .io_sram_oe(RAMOE),
    .io_sram_cs(RAMCS),
    .io_sram_lb(RAMLB),
    .io_sram_ub(RAMUB),
    .io_mux_pins(io_mux_pins),
    .io_pinInterrupt_pins({BUT2, BUT1}),
    .io_qspi_qss(QSS),
    .io_qspi_qck(QCK),
    .io_qspi_qd_read(io_qspi_qd_read),
    .io_qspi_qd_write(io_qspi_qd_write),
    .io_qspi_qd_writeEnable(io_qspi_qd_writeEnable),
    .io_quadrature_quadA(io_quadrature_quadA),
    .io_quadrature_quadB(io_quadrature_quadB)
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
