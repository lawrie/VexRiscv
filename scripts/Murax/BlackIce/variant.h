
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

static const unin8_t SEVEN_SEGMENT_A_DIGIT_PIN 35
static const unin8_t SEVEN_SEGMENT_A_SEG_PINS 36
static const unin8_t WS2811_DOUT 44
static const unin8_t SERVO_PINS 48
static const unin8_t SEVEN_SEGMENT_B_DIGIT_PIN 27
static const unin8_t SEVEN_SEGMENT_B_SEG_PINS 16
static const unin8_t PIN_INTERRUPT_PINS 8
static const unin8_t SHIFT_IN_CLK 5
static const unin8_t SHIFT_IN_DATA 45
static const unin8_t PWM_PINS 13
static const unin8_t SPI_MISO 42
static const unin8_t SPI_MOSI 41
static const unin8_t SPI_SS 43
static const unin8_t SPI_SCLK 40
static const unin8_t PS2_KEYBOARD_PS2_DATA 39
static const unin8_t PS2_KEYBOARD_PS2_CLK 47
static const unin8_t TONE_PIN 47
static const unin8_t SRAM_ADDR 0
static const unin8_t QUADRATURE_QUAD_A 16
static const unin8_t QUADRATURE_QUAD_B 17
static const unin8_t PULSE_IN_PINS 44
static const unin8_t SHIFT_OUT_DATA 7
static const unin8_t SHIFT_OUT_CLK 6

// Muxes

define SEVEN_SEGMENT_A_MUX 4
define WS2811_MUX 12
define SERVO_MUX 3
define SEVEN_SEGMENT_B_MUX 2
define SHIFT_IN_MUX 0
define SPI_MUX 5
define TONE_MUX 9
define SHIFT_OUT_MUX 1

#endif /* _f32c_variant_h */

