/*
 * Copyright (c) 2024 Thomas Roell.  All rights reserved.
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

#ifndef _VARIANT_SASQUATCH_STM32WB5MMG_
#define _VARIANT_SASQUATCH_STM32WB5MMG_

// The definitions here needs a STM32WB core >=1.6.6
#define ARDUINO_STM32WB_VARIANT_COMPLIANCE 10606

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

#define STM32WB_CONFIG_PIN_VBAT_SWITCH    STM32WB_GPIO_PIN_PE4
#define STM32WB_CONFIG_VBAT_SENSE_CHANNEL STM32WB_ADC_CHANNEL_VBAT
#define STM32WB_CONFIG_VBAT_SENSE_DELAY   10000
#define STM32WB_CONFIG_VBAT_SENSE_PERIOD  STM32WB_ADC_VBAT_PERIOD
#define STM32WB_CONFIG_VBAT_SENSE_SCALE   (2.0 * 3.0)

#define STM32WB_CONFIG_BLE_LSE_SOURCE     BLE_LSE_SOURCE_MOD5MM

#define USBCON

/** Master clock frequency */
#define VARIANT_MCK			  F_CPU

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
#include "USBAPI.h"
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
#define PINS_COUNT           (42u)
#define NUM_DIGITAL_PINS     (16u)
#define NUM_ANALOG_INPUTS    (6u)
#define NUM_ANALOG_OUTPUTS   (0u)

// LEDs
#define PIN_LED_GREEN        (22u)
#define PIN_LED_RED          (23u)
#define PIN_LED_BLUE         (24u)
#define LED_BUILTIN          PIN_LED_GREEN

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
static const uint8_t A5  = PIN_A4;
#define ADC_RESOLUTION		12

#define PIN_BUTTON           (25)
static const uint8_t BUTTON = PIN_BUTTON;

/*
 * Serial interfaces
 */

#define SERIAL_INTERFACES_COUNT 3

#define PIN_SERIAL1_RX       (0ul)
#define PIN_SERIAL1_TX       (1ul)
#define PIN_SERIAL1_RTS      (2ul)
#define PIN_SERIAL1_CTS      (3ul)

#define PIN_SERIAL2_RX       (26ul)
#define PIN_SERIAL2_TX       (27ul)
#define PIN_SERIAL2_RTS      (28ul)
#define PIN_SERIAL2_CTS      (29ul)
#define PIN_SERIAL2_DTR      (30ul)
#define PIN_SERIAL2_DSR      (31ul)
#define PIN_SERIAL2_DCD      (32ul)
#define PIN_SERIAL2_RI       (33ul)

#define PIN_MODEM_BOOT       (34ul)
#define PIN_MODEM_RESET      (36ul)
#define PIN_MODEM_IO         (37ul)
#define PIN_MODEM_ENABLE     (38ul)
  
  
/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 2

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
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA         (14u)
#define PIN_WIRE_SCL         (15u)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * PDM Interfaces
 */
#define PDM_INTERFACES_COUNT 1

#define PWM_INSTANCE_COUNT  2

/*
 *  Blues Notecard Interface
 */

#define PIN_BLUES_RX         (26ul)
#define PIN_BLUES_TX         (27ul)
#define PIN_BLUES_AUX_EN     (28ul)
#define PIN_BLUES_AUX_ACTIVE (29ul)
#define PIN_BLUES_RTX        (30ul)
#define PIN_BLUES_CTX        (31ul)
#define PIN_BLUES_ATTN       (33ul)

#define PIN_BLUES_AUX1       (2ul)
#define PIN_BLUES_AUX2       (3ul)
#define PIN_BLUES_AUX3       (4ul)
#define PIN_BLUES_AUX4       (19ul)

#define SerialBLUES          Serial2
#define WireBLUES            Wire1
  
#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern CDC  Serial;
extern Uart Serial1;
extern Uart Serial2;
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
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE2       Serial2

// Alias SerialUSB to Serial
#define SerialUSB SERIAL_PORT_USBVIRTUAL

#endif /* _VARIANT_SASQUATCH_STM32WB5MMG_ */

