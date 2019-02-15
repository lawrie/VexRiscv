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
    input   SW1,
    input   SW2,
    input   SW3,
    input   SW4,
    inout   [7:0] GPIO,
    output  D,
    output  [6:0] SEG,
    inout   SDA,
    inout   SCL
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
  assign io_gpioA_read[10] = SW1;
  assign io_gpioA_read[11] = SW2;
  assign io_gpioA_read[12] = SW3;
  assign io_gpioA_read[13] = SW4;

  SB_IO #(
    .PIN_TYPE(6'b 1010_01),
    .PULLUP(1'b 0)
  ) ios [7:0] (
    .PACKAGE_PIN(GPIO),
    .OUTPUT_ENABLE(io_gpioA_writeEnable[23:16]),
    .D_OUT_0(io_gpioA_write[23:16]),
    .D_IN_0(io_gpioA_read[23:16])
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
   
  // Use PLL to downclock external clock.
  toplevel_pll toplevel_pll_inst(.REFERENCECLK(CLK),
                                 .PLLOUTCORE(io_mainClk),
                                 .PLLOUTGLOBAL(),
                                 .RESET(1'b1));

  MuraxArduino murax ( 
    .io_asyncReset(1'b0),
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
    .io_tone_pin(TONE),
    .io_shiftOut_clockPin(SHIFT_OUT_CLK),
    .io_shiftOut_dataPin(SHIFT_OUT_DATA),
    .io_shiftIn_clockPin(SHIFT_IN_CLK),
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
    .io_i2c_scl_write(io_i2c_scl_write)
  );

endmodule
