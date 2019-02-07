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
    output  SHIFT_DATA,
    output  SHIFT_CLK
  );

  assign LED1 = io_gpioA_write[0];
  assign LED2 = io_gpioA_write[1];
  assign LED3 = io_gpioA_write[2];
  assign LED4 = io_gpioA_write[7];

  wire [31:0] io_gpioA_read;
  wire [31:0] io_gpioA_write;
  wire [31:0] io_gpioA_writeEnable;
  wire io_mainClk;

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
    .io_shiftOut_clockPin(SHIFT_CLK),
    .io_shiftOut_dataPin(SHIFT_DATA)
  );

endmodule
