/*
 * Copyright (c) 2021 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#ifndef _VARIANT_NUCLEO_STM32WB55RG_
#define _VARIANT_NUCLEO_STM32WB55RG_

// The definitions here needs a STM32WB core >=1.6.6
#define ARDUINO_STM32WB_VARIANT_COMPLIANCE 10606

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Master clock frequency */
#define VARIANT_MCK			  F_CPU

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (28u)
#define NUM_DIGITAL_PINS     (16u)
#define NUM_ANALOG_INPUTS    (6u)
#define NUM_ANALOG_OUTPUTS   (0u)

// LEDs
// LEDs
#define PIN_LED_RXL          (23u)
#define PIN_LED_TXL          (24u)
#define PIN_LED              (22u)
#define PIN_LED1             PIN_LED_RXL
#define PIN_LED2             PIN_LED_TXL
#define LED_BUILTIN          (22u)

/*
 * Analog pins
 */
#define PIN_A0               (16ul)
#define PIN_A1               (17ul)
#define PIN_A2               (18ul)
#define PIN_A3               (19ul)
#define PIN_A4               (20ul)
#define PIN_A5               (21ul)

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;
#define ADC_RESOLUTION		12

#define PIN_BUTTON           (25)
#define PIN_BUTTON1          (25)
static const uint8_t BUTTON = PIN_BUTTON;
static const uint8_t BUTTON1 = PIN_BUTTON1;

/*
 * Serial interfaces
 */

#define SERIAL_INTERFACES_COUNT 2

#define PIN_SERIAL1_RX       (0ul)
#define PIN_SERIAL1_TX       (1ul)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MOSI         (11u)
#define PIN_SPI_MISO         (12u)
#define PIN_SPI_SCK          (13u)

static const uint8_t SS	  = 10;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (14u)
#define PIN_WIRE_SCL         (15u)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * I2S Interfaces
 */
#define I2S_INTERFACES_COUNT 1

#define PIN_I2S_SD           (6u)
#define PIN_I2S_WS           (9u)
#define PIN_I2S_SCK          (3u)

/*
 * PDM Interfaces
 */
#define PDM_INTERFACES_COUNT 1

#define PIN_PDM_CK           (6u)
#define PIN_PDM_DIN          (9u)

#define PWM_INSTANCE_COUNT  2

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern Uart Serial;
extern Uart Serial1;
#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_HARDWARE1       Serial
#define SERIAL_PORT_HARDWARE2       Serial1
#define SERIAL_PORT_HARDWARE_OPEN2  Serial1

#endif /* _VARIANT_NUCLEO_STM32WB55RG_ */

