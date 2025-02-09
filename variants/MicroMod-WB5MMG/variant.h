/*
 * Copyright (c) 2020 Thomas Roell.  All rights reserved.
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

#ifndef _VARIANT_MOTHRA_STM32WB5MMG_
#define _VARIANT_MOTHRA_STM32WB5MMG_

// The definitions here needs a STM32WB core >=1.6.6
#define ARDUINO_STM32WB_VARIANT_COMPLIANCE 10606

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

#define STM32WB_CONFIG_VBAT_SENSE_CHANNEL STM32WB_ADC_CHANNEL_13 // PC4
#define STM32WB_CONFIG_VBAT_SENSE_PERIOD  4000                   // nS
#define STM32WB_CONFIG_VBAT_SENSE_SCALE   (3.0)

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
#define PINS_COUNT           (37u)
#define NUM_DIGITAL_PINS     (21u)
#define NUM_ANALOG_INPUTS    (4u)
#define NUM_ANALOG_OUTPUTS   (0u)

#define D0                   (0u)
#define D1                   (1u)

#define G0                   (2u)
#define G1                   (3u)
#define G2                   (4u)
#define G3                   (5u)
#define G4                   (6u)
#define G5                   (7u)
#define G6                   (8u)
#define G7                   (9u)

#define PWM0                 (27u)
#define PWM1                 (28u)

#define I2C_SDA              (14u)
#define I2C_SCL              (15u)
#define I2C_INT              (16u)
  
#define AUD_MLCK             (29u)
#define AUD_OUT              (30u)
#define AUD_IN               (31u)
#define AUD_LRCLK            (32u)
#define AUD_BCLK             (33u)

#define BATT_VIN             (34u)

// LEDs
#define PIN_LED              (35u)
static const uint8_t LED_BUILTIN = PIN_LED;

// Buttons  
#define PIN_BUTTON           (36u)
static const uint8_t BUTTON = PIN_BUTTON;

/*
 * Analog pins
 */
#define PIN_A0               (21u)
#define PIN_A1               (22u)
#define PIN_A2               (23u)
#define PIN_A3               (24u)

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
#define ADC_RESOLUTION	     12

/*
 * Serial interfaces
 */

#define SERIAL_INTERFACES_COUNT 3

#define PIN_SERIAL1_RX       (17u)
#define PIN_SERIAL1_TX       (18u)
#define PIN_SERIAL1_RTS      (19u)
#define PIN_SERIAL1_CTS      (20u)

#define PIN_SERIAL2_RX       (23u)
#define PIN_SERIAL2_TX       (24u)
  
/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_SS           (10u)
#define PIN_SPI_MOSI         (11u)
#define PIN_SPI_MISO         (12u)
#define PIN_SPI_SCK          (13u)

static const uint8_t SS	  = PIN_SPI_SS;
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

#define PIN_WIRE_SDA1        (25u)
#define PIN_WIRE_SCL1        (26u)
  
static const uint8_t SDA1 = PIN_WIRE_SDA1;
static const uint8_t SCL1 = PIN_WIRE_SCL1;

/*
 * I2S Interfaces
 */
#define I2S_INTERFACES_COUNT 1

#define PIN_I2S_SD           (31u)
#define PIN_I2S_WS           (32u)
#define PIN_I2S_SCK          (33u)

/*
 * PDM Interfaces
 */
#define PDM_INTERFACES_COUNT 1

#define PIN_PDM_CK           PIN_I2S_SCK
#define PIN_PDM_DIN          PIN_I2S_WS

/*
 * MikroBus
 */

#define PIN_MIK_RX           (17u)
#define PIN_MIK_TX           (18u)
#define PIN_MIK_SDA          (14u)
#define PIN_MIK_SCL          (15u)
#define PIN_MIK_CS           (10u)
#define PIN_MIK_MOSI         (11u)
#define PIN_MIK_MISO         (12u)
#define PIN_MIK_SCK          (13u)
#define PIN_MIK_INT          (0u)
#define PIN_MIK_RST          (1u)
#define PIN_MIK_ADC          (21u)
#define PIN_MIK_PWM          (27u)
  
#define PWM_INSTANCE_COUNT  2

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

#endif /* _VARIANT_MOTHRA_STM32WB5MMG_ */

