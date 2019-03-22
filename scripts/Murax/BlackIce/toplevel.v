`timescale 1ns / 1ps

module toplevel(
    input   CLK,
    input   BUT1,
    input   BUT2,
    input   UART_RX,
    output  UART_TX,
    output  LED1,
    output  LED2,
    output  LED3,
    output  LED4,
    output  PWM,
    output  SERVO,
    output  TONE,
    output  SHIFT_OUT_DATA,
    output  SHIFT_OUT_CLK,
    input   SHIFT_IN_DATA,
    output  SHIFT_IN_CLK,
    output  SPI_SCK,
    output  SPI_MOSI,
    output  SPI_SS,
    input   SPI_MISO,
    input   ECHO,
    output  TRIGGER,
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
    output  RAMLB
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

  assign LED1 = io_gpioA_write[0];
  assign LED2 = io_gpioA_write[1];
  assign LED3 = io_gpioA_write[2];
  assign LED4 = io_gpioA_write[3];

  assign TRIGGER = io_gpioA_write[4];
 
  wire [31:0] io_gpioA_read;
  wire [31:0] io_gpioA_write;
  wire [31:0] io_gpioA_writeEnable;

  wire io_mainClk;

  assign io_gpioA_read[8] = BUT1;
  assign io_gpioA_read[9] = BUT2;

  SB_IO #(
    .PIN_TYPE(6'b 1010_01),
    .PULLUP(1'b 0)
  ) ios [21:0] (
    .PACKAGE_PIN(GPIO),
    .OUTPUT_ENABLE(io_gpioA_writeEnable[31:10]),
    .D_OUT_0(io_gpioA_write[31:10]),
    .D_IN_0(io_gpioA_read[31:10])
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

  wire io_shiftIn_clockPin;
  wire io_shiftOut_clockPin;
  wire io_shiftOut_dataPin;

  assign SHIFT_IN_CLK = io_mux_pins[0] ? io_shiftIn_clockPin : io_gpioA_write[5];
  assign SHIFT_OUT_CLK = io_mux_pins[1] ? io_shiftOut_clockPin : io_gpioA_write[6];
  assign SHIFT_OUT_DATA = io_mux_pins[1] ? io_shiftOut_dataPin : io_gpioA_write[7];

  MuraxArduino murax ( 
    .io_asyncReset(reset),
    .io_mainClk (io_mainClk),
    .io_jtag_tck(1'b0),
    .io_jtag_tdi(1'b0),
    .io_jtag_tdo(),
    .io_jtag_tms(1'b0),
    .io_gpioA_read       (io_gpioA_read),
    .io_gpioA_write      (io_gpioA_write),
    .io_gpioA_writeEnable(io_gpioA_writeEnable),
    .io_uart_txd(UART_TX),
    .io_uart_rxd(UART_RX),
    .io_pwm_pin(PWM),
    .io_servo_pin(SERVO),
    .io_tone_pin(TONE),
    .io_shiftOut_clockPin(io_shiftOut_clockPin),
    .io_shiftOut_dataPin(io_shiftOut_dataPin),
    .io_shiftIn_clockPin(io_shiftIn_clockPin),
    .io_shiftIn_dataPin(SHIFT_IN_DATA),
    .io_spiMaster_sclk(SPI_SCK),
    .io_spiMaster_mosi(SPI_MOSI),
    .io_spiMaster_miso(SPI_MISO),
    .io_spiMaster_ss(SPI_SS),
    .io_pulseIn_pin(ECHO),
    .io_sevenSegment_digitPin(D),
    .io_sevenSegment_segPins(SEG),
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
    .io_pinInterrupt_pin(BUT1)
  );

endmodule
