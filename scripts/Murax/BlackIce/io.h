
#ifndef _IO_H_
#define	_IO_H_

#ifdef __riscv
#include <riscv/io.h>
#endif


#define RAM_BASE 0x80000000l


#define IO_BASE 0xF0000000l

#define	IO_ADDR(a)	(IO_BASE | (a))

#define IO_MACHINE_TIMER IO_ADDR(0xB0000)
#define IO_MUX IO_ADDR(0xD0000)
#define IO_QUADRATURE IO_ADDR(0xF8000)
#define IO_SHIFT_OUT IO_ADDR(0x50000)
#define IO_TONE IO_ADDR(0x40000)
#define IO_PULSE_IN IO_ADDR(0x80000)
#define IO_UART IO_ADDR(0x10000)
#define IO_PWM IO_ADDR(0x30000)
#define IO_GPIO_A IO_ADDR(0x00000)
#define IO_GPIO_B IO_ADDR(0x08000)
#define IO_SPI_MASTER IO_ADDR(0x60000)
#define IO_WS2811 IO_ADDR(0xD8000)
#define IO_SEVEN_SEGMENT_A IO_ADDR(0x90000)
#define IO_SERVO IO_ADDR(0xC0000)
#define IO_SHIFT_IN IO_ADDR(0xA00000)
#define IO_PS2 IO_ADDR(0xA80000)
#define IO_PIN_INTERRUPT IO_ADDR(0xE0000)
#define IO_I2C IO_ADDR(0x70000)
#define IO_SEVEN_SEGMENT_B IO_ADDR(0x98000)
#define IO_QSPI_ANALOG IO_ADDR(0xF0000)
#define IO_TIMER IO_ADDR(0x20000)

#define	IO_SIO_BYTE	(IO_UART + 0)
#define	IO_SIO_STATUS	(IO_UART + 4)
#define	IO_SIO_BAUD	(IO_UART + 8)


#define	IO_GPIO_A_INPUT	(IO_GPIO_A + 0)
#define	IO_GPIO_A_DATA	(IO_GPIO_A + 4)
#define	IO_GPIO_A_CTL	(IO_GPIO_A + 8)


#define	IO_GPIO_B_INPUT	(IO_GPIO_B + 0)
#define	IO_GPIO_B_DATA	(IO_GPIO_B + 4)
#define	IO_GPIO_B_CTL	(IO_GPIO_B + 8)


#define	IO_PWM_DUTY	(IO_PWM + 0)


#define	IO_PULSE_IN_VALUE	(IO_PULSE_IN + 0)
#define	IO_PULSE_IN_TIMEOUT	(IO_PULSE_IN + 4)
#define	IO_PULSE_IN_LENGTH	(IO_PULSE_IN + 8)


#define	IO_TONE_PERIOD		(IO_TONE + 0)
#define	IO_TONE_DURATION	(IO_TONE + 4)


#define	IO_SHIFT_IN_BYTE_VALUE	(IO_SHIFT_IN + 0)
#define	IO_SHIFT_IN_PRE_SCALE	(IO_SHIFT_IN + 4)
#define	IO_SHIFT_IN_BIT_ORDER	(IO_SHIFT_IN + 8)


#define	IO_SHIFT_OUT_BYTE_VALUE	(IO_SHIFT_OUT + 0)
#define	IO_SHIFT_OUT_BIT_ORDER	(IO_SHIFT_OUT + 4)
#define	IO_SHIFT_OUT_PRE_SCALE	(IO_SHIFT_OUT + 8)


#define IO_ANALOG IO_QSPI_ANALOG


#define IO_PIN_INTERRUPT_RISING  (IO_PIN_INTERRUPT + 0)
#define IO_PIN_INTERRUPT_FALLING  (IO_PIN_INTERRUPT + 4)
#define IO_PIN_INTERRUPT_PENDING  (IO_PIN_INTERRUPT + 0x10)
#define IO_PIN_INTERRUPT_MASKS  (IO_PIN_INTERRUPT + 0x14)


#define IO_TIMER_INTERRUPT_MASKS (IO_TIMER_INTERRUPT + 0x14)


#define IO_TIMER_INTERRUPT (RAM_BASE + 0x2000)
#define IO_PIN_INTERRUPT_ADDR (RAM_BASE + 0x2004)

/* SIO status bitmask - TODO: get rid of this */
#define	SIO_TX_BUSY	0x4

#endif /* !_IO_H_ */

