
// GPIO and Mux assignments

assign gpioA_write[0] =   io_gpioA_write[0];
assign gpioA_write[1] =   io_gpioA_write[1];
assign gpioA_write[2] =   io_gpioA_write[2];
assign gpioA_write[3] =   io_gpioA_write[3];
assign gpioA_write[4] =   io_gpioA_write[4];
assign gpioA_write[5] =   io_mux_pins[0] ? io_shiftIn_clockPin
                          : io_gpioA_write[5];
assign gpioA_write[6] =   io_mux_pins[3] ? io_servo_pins[1]
                          : io_mux_pins[1] ? io_shiftOut_clockPin
                          : io_gpioA_write[6];
assign gpioA_write[7] =   io_mux_pins[3] ? io_servo_pins[0]
                          : io_mux_pins[1] ? io_shiftOut_dataPin
                          : io_gpioA_write[7];
assign gpioA_write[8] =   io_gpioA_write[8];
assign gpioA_write[9] =   io_gpioA_write[9];
assign gpioA_write[10] =   io_gpioA_write[10];
assign gpioA_write[11] =   io_gpioA_write[11];
assign gpioA_write[12] =   io_mux_pins[10] ? io_pwm_pins[3]
                          : io_gpioA_write[12];
assign gpioA_write[13] =   io_mux_pins[11] ? io_pwm_pins[4]
                          : io_gpioA_write[13];
assign gpioA_write[14] =   io_gpioA_write[14];
assign gpioA_write[15] =   io_gpioA_write[15];
assign gpioA_write[16] =   io_mux_pins[2] ? io_sevenSegmentB_segPins[3]
                          : io_gpioA_write[16];
assign gpioA_write[17] =   io_mux_pins[2] ? io_sevenSegmentB_segPins[4]
                          : io_gpioA_write[17];
assign gpioA_write[18] =   io_mux_pins[2] ? io_sevenSegmentB_segPins[5]
                          : io_gpioA_write[18];
assign gpioA_write[19] =   io_mux_pins[2] ? io_sevenSegmentB_segPins[6]
                          : io_gpioA_write[19];
assign gpioA_write[20] =   io_gpioA_write[20];
assign gpioA_write[21] =   io_gpioA_write[21];
assign gpioA_write[22] =   io_gpioA_write[22];
assign gpioA_write[23] =   io_gpioA_write[23];
assign gpioA_write[24] =   io_mux_pins[2] ? io_sevenSegmentB_segPins[0]
                          : io_gpioA_write[24];
assign gpioA_write[25] =   io_mux_pins[2] ? io_sevenSegmentB_segPins[1]
                          : io_gpioA_write[25];
assign gpioA_write[26] =   io_mux_pins[2] ? io_sevenSegmentB_segPins[2]
                          : io_gpioA_write[26];
assign gpioA_write[27] =   io_mux_pins[2] ? io_sevenSegmentB_digitPin
                          : io_gpioA_write[27];
assign gpioA_write[28] =   io_gpioA_write[28];
assign gpioA_write[29] =   io_gpioA_write[29];
assign gpioA_write[30] =   io_gpioA_write[30];
assign gpioA_write[31] =   io_gpioA_write[31];

assign gpioB_write[0] =   io_mux_pins[4] ? io_sevenSegmentA_segPins[0]
                          : io_gpioB_write[0];
assign gpioB_write[1] =   io_mux_pins[4] ? io_sevenSegmentA_segPins[1]
                          : io_gpioB_write[1];
assign gpioB_write[2] =   io_mux_pins[4] ? io_sevenSegmentA_segPins[2]
                          : io_gpioB_write[2];
assign gpioB_write[3] =   io_mux_pins[4] ? io_sevenSegmentA_digitPin
                          : io_gpioB_write[3];
assign gpioB_write[4] =   io_mux_pins[4] ? io_sevenSegmentA_segPins[3]
                          : io_gpioB_write[4];
assign gpioB_write[5] =   io_mux_pins[4] ? io_sevenSegmentA_segPins[4]
                          : io_gpioB_write[5];
assign gpioB_write[6] =   io_mux_pins[4] ? io_sevenSegmentA_segPins[5]
                          : io_gpioB_write[6];
assign gpioB_write[7] =   io_mux_pins[4] ? io_sevenSegmentA_segPins[6]
                          : io_gpioB_write[7];
assign gpioB_write[8] =   io_mux_pins[5] ? io_spiMaster_sclk
                          : io_gpioB_write[8];
assign gpioB_write[9] =   io_mux_pins[5] ? io_spiMaster_mosi
                          : io_gpioB_write[9];
assign gpioB_write[10] =   io_gpioB_write[10];
assign gpioB_write[11] =   io_mux_pins[5] ? io_spiMaster_ss
                          : io_gpioB_write[11];
assign gpioB_write[12] =   io_mux_pins[12] ? io_ws2811_dout
                          : io_gpioB_write[12];
assign gpioB_write[13] =   io_gpioB_write[13];
assign gpioB_write[14] =   io_mux_pins[6] ? io_pwm_pins[0]
                          : io_gpioB_write[14];
assign gpioB_write[15] =   io_mux_pins[3] ? io_servo_pins[2]
                          : io_mux_pins[9] ? io_tone_pin
                          : io_gpioB_write[15];
assign gpioB_write[16] =   io_mux_pins[3] ? io_servo_pins[3]
                          : io_gpioB_write[16];

// GPIO read assignments

assign io_ps2_ps2Data = gpioA_read[7];
assign io_pinInterrupt_pins[0] = gpioA_read[8];
assign io_pinInterrupt_pins[1] = gpioA_read[9];
assign io_quadrature_quadA = gpioA_read[16];
assign io_quadrature_quadB = gpioA_read[17];

assign io_spiMaster_miso = gpioB_read[10];
assign io_pulseIn_pins[1] = gpioB_read[12];
assign io_shiftIn_dataPin = gpioB_read[13];
assign io_pulseIn_pins[0] = gpioB_read[13];
assign io_ps2_ps2Clk = gpioB_read[15];

assign DEBUG = io_mux_pins[7] ? io_pwm_pins[1] : io_gpioB_write[17];
assign DONE = io_mux_pins[8] ? io_pwm_pins[2] : io_gpioB_write[18];

assign gpioA_writeEnable =  io_gpioA_writeEnable;
assign gpioB_writeEnable =  io_gpioB_writeEnable[16:0];

assign io_gpioA_read = gpioA_read;
assign io_gpioB_read[16:0] = gpioB_read;
assign io_gpioB_read[31:17] = 0;

