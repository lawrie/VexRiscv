
#ifndef _f32c_variant_h
#define _f32c_variant_h

#include <stdint.h>

/* LEDs */
#define PIN_LED_13           3
#define PIN_LED              3
#define LED_BUILTIN          3

#include "Arduino.h"
#ifdef __cplusplus
#include "UARTClass.h"
#endif

#ifdef __cplusplus
extern UARTClass Serial;
#endif

/*
 * Analog pins - currently dummies to make thing compile
 */
static const uint8_t A0  = 16;
static const uint8_t A1  = 17;
static const uint8_t A2  = 18;
static const uint8_t A3  = 19;
static const uint8_t A4  = 20;
static const uint8_t A5  = 21;


// Other pins

static const int8_t SEVEN_SEGMENT_B_SEG_PINS[] = {24,25,26,16,17,18,19};
static const uint8_t SEVEN_SEGMENT_B_DIGIT_PIN = 27;
static const int8_t PWM_PINS[] = {46,-1,-1,44,13};
static const uint8_t WS2811_DOUT = 44;
static const uint8_t SPI_MASTER_MISO = 42;
static const uint8_t SPI_MASTER_SCLK = 40;
static const uint8_t SPI_MASTER_SS = 43;
static const uint8_t SPI_MASTER_MOSI = 41;
static const int8_t SERVO_PINS[] = {7,6,47,48};
static const uint8_t TONE_PIN = 47;
static const int8_t PIN_INTERRUPT_PINS[] = {8,9};
static const uint8_t SHIFT_IN_DATA_PIN = 45;
static const uint8_t SHIFT_IN_CLOCK_PIN = 5;
static const uint8_t SHIFT_OUT_CLOCK_PIN = 6;
static const uint8_t SHIFT_OUT_DATA_PIN = 7;
static const int8_t PULSE_IN_PINS[] = {45,44};
static const uint8_t QUADRATURE_QUAD_A = 16;
static const uint8_t QUADRATURE_QUAD_B = 17;
static const int8_t SEVEN_SEGMENT_A_SEG_PINS[] = {32,33,34,36,37,38,39};
static const uint8_t SEVEN_SEGMENT_A_DIGIT_PIN = 35;
static const uint8_t PS2_PS2_CLK = 47;
static const uint8_t PS2_PS2_DATA = 7;

#define SPI_SCLK SPI_MASTER_SCLK
#define SPI_MOSI SPI_MASTER_MOSI
#define SPI_MISO SPI_MASTER_MISO
#define SPI_SS SPI_MASTER_SS
#define SPI_START_PIN SPI_SCLK



// Muxes

static const uint8_t SEVEN_SEGMENT_B_MUX = 2;
static const uint8_t WS2811_MUX = 12;
static const uint8_t SPI_MASTER_MUX = 5;
static const uint8_t SERVO_MUX = 3;
static const uint8_t TONE_MUX = 9;
static const uint8_t SHIFT_IN_MUX = 0;
static const uint8_t SHIFT_OUT_MUX = 1;
static const uint8_t SEVEN_SEGMENT_A_MUX = 4;

static const uint8_t PWM_MUX = 6;

#endif /* _f32c_variant_h */

