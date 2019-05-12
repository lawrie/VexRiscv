  // GPIO and Mux assignments
  assign gpioA_write[4:0] =   io_gpioA_write[4:0];
  assign gpioA_write[5] =     io_mux_pins[`MUX_SHIFT_IN] ? io_shiftIn_clockPin : io_gpioA_write[5];
  assign gpioA_write[6] =     io_mux_pins[`MUX_SERVO] ? io_servo_pins[1] :
                              io_mux_pins[`MUX_SHIFT_OUT] ? io_shiftOut_clockPin : io_gpioA_write[6];
  assign gpioA_write[7] =     io_mux_pins[`MUX_SERVO] ? io_servo_pins[0] :
                              io_mux_pins[`MUX_SHIFT_OUT] ? io_shiftOut_dataPin : io_gpioA_write[7];
  assign gpioA_write[9:8] =   io_gpioA_write[9:8];
  assign gpioA_write[15:10] = io_gpioA_write[15:10];
  assign gpioA_write[19:16] = io_mux_pins[`MUX_SEVEN_SEGMENT_B] ? io_sevenSegmentB_segPins[6:3] : io_gpioA_write[19:16];
  assign gpioA_write[23:20] = io_gpioA_write[23:20];
  assign gpioA_write[26:24] = io_mux_pins[`MUX_SEVEN_SEGMENT_B] ? 
                              io_sevenSegmentB_segPins[2:0] : 
                              io_gpioA_write[26:24];
  assign gpioA_write[27] =    io_mux_pins[`MUX_SEVEN_SEGMENT_B] ? io_sevenSegmentB_digitPin : io_gpioA_write[27];
  assign gpioA_write[31:28] = io_gpioA_write[31:28];

  
  assign gpioB_write[2:0] =   io_mux_pins[`MUX_SEVEN_SEGMENT_A] ? io_sevenSegmentA_segPins[2:0] : io_gpioB_write[2:0];
  assign gpioB_write[3] =     io_mux_pins[`MUX_SEVEN_SEGMENT_A] ? io_sevenSegmentA_digitPin : io_gpioB_write[3];
  assign gpioB_write[7:4] =   io_mux_pins[`MUX_SEVEN_SEGMENT_A] ? io_sevenSegmentA_segPins[6:3] : io_gpioB_write[7:4];
  assign gpioB_write[8] =     io_mux_pins[`MUX_SPI_MASTER] ? io_spiMaster_sclk : io_gpioB_write[8];
  assign gpioB_write[9] =     io_mux_pins[`MUX_SPI_MASTER] ? io_spiMaster_mosi : io_gpioB_write[9];
  assign gpioB_write[10] = io_gpioB_write[10];
  assign gpioB_write[11] = io_mux_pins[`MUX_SPI_MASTER] ? io_spiMaster_ss : io_gpioB_write[11];
  assign gpioB_write[12] =    io_mux_pins[`MUX_PWM_3] ? io_pwm_pins[3] : 
                              io_mux_pins[`MUX_WS2811] ? io_ws2811_dout : io_gpioB_write[12];
  assign gpioB_write[13] =    io_mux_pins[`MUX_PWM_4] ? io_pwm_pins[4] : io_gpioB_write[13];
  assign gpioB_write[14] =    io_mux_pins[`MUX_PWM_0] ? io_pwm_pins[0] : io_gpioB_write[14];
  assign gpioB_write[15] =    io_mux_pins[`MUX_SERVO] ? io_servo_pins[2] : 
                              io_mux_pins[`MUX_TONE] ? io_tone_pin : io_gpioB_write[15];
  assign gpioB_write[16] =    io_mux_pins[`MUX_SERVO] ? io_servo_pins[3] : io_gpioB_write[16];

  assign gpioA_writeEnable =  io_gpioA_writeEnable;
  assign gpioB_writeEnable =  io_gpioB_writeEnable[16:0];

  assign DEBUG =              io_mux_pins[`MUX_PWM_1] ? io_pwm_pins[1] : io_gpioB_write[17];
  assign DONE =               io_mux_pins[`MUX_PWM_2] ? io_pwm_pins[2] : io_gpioB_write[18];

  assign io_gpioA_read = gpioA_read;
  assign io_gpioB_read[16:0] = gpioB_read;

  assign io_gpioB_read[31:17] = 0;
  
  assign io_ps2_ps2Data =      gpioA_read[7];
  assign io_pinInterrupt_pins = gpioA_read[9:8];
  assign io_quadrature_quadA = gpioA_read[16];
  assign io_quadrature_quadB = gpioA_read[17];

  assign io_spiMaster_miso =   gpioB_read[10];
  assign io_pulseIn_pins[0] =  gpioB_read[12];
  assign io_pulseIn_pins[1] =  gpioB_read[13];
  assign io_shiftIn_dataPin =  gpioB_read[13];
  assign io_ps2_ps2Clk =       gpioB_read[15];

