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

#ifndef _VARIANT_DISCO_STM32WB5MMG_
#define _VARIANT_DISCO_STM32WB5MMG_

// The definitions here needs a STM32WB core >=1.6.6
#define ARDUINO_STM32WB_VARIANT_COMPLIANCE 10606

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

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
#define PINS_COUNT           (44u)
#define NUM_DIGITAL_PINS     (16u)
#define NUM_ANALOG_INPUTS    (6u)
#define NUM_ANALOG_OUTPUTS   (0u)

// Buttons  
#define PIN_BUTTON           (40u)
static const uint8_t BUTTON = PIN_BUTTON;

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

/*
 * Serial interfaces
 */

#define SERIAL_INTERFACES_COUNT 3

#define PIN_SERIAL1_RX       (0u)
#define PIN_SERIAL1_TX       (1u)
  
/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 2

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

/*
 * PDM Interfaces
 */
#define PDM_INTERFACES_COUNT 1

#define PWM_INSTANCE_COUNT  4

#define SerialSTMOD          Serial1
#define WireSTMOD            Wire
#define SPISTMOD             SPI1
  
#define PIN_STMOD_RX         (0u)
#define PIN_STMOD_TX         (1u)
#define PIN_STMOD_SDA        (14u)
#define PIN_STMOD_SCL        (15u)
#define PIN_STMOD_CS         (22u)
#define PIN_STMOD_MOSI       (23u)
#define PIN_STMOD_MISO       (24u)
#define PIN_STMOD_SCK        (25u)
#define PIN_STMOD_INT        (26u)
#define PIN_STMOD_RST        (27u)
#define PIN_STMOD_ADC        (28u)
#define PIN_STMOD_PWM        (29u)
#define PIN_STMOD_GPIO1      (30u)
#define PIN_STMOD_GPIO2      (31u)
#define PIN_STMOD_GPIO3      (18u)
#define PIN_STMOD_GPIO4      (19u)


#define SerialMODEM          Serial1
  
#define PIN_MODEM_RX         (0u)
#define PIN_MODEM_TX         (1u)
#define PIN_MODEM_RTS        (25u)
#define PIN_MODEM_CTS        (22u)
#define PIN_MODEM_DTR        (29u)
#define PIN_MODEM_RI         (26u)
#define PIN_MODEM_STATUS     (19u)
#define PIN_MODEM_RESET      (27u)
#define PIN_MODEM_ENABLE     (24u)
  
#define PIN_OLED_CS          (32u)
#define PIN_OLED_DC          (33u)
#define PIN_OLED_RST         (34u)
  
#define PIN_STTS22H_DRDY     (35u)
#define PIN_ISM303DHCX_INT   (36u)
#define PIN_VL53L0X_INT      (37u)

#define PIN_GPIO1_SELECT     (38u) // STMOD / ARD Serial1 select
#define PIN_GPIO2_SELECT     (39u) // RGB LED / MOSI
  
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

#endif /* _VARIANT_DISCO_STM32WB5MMG_ */

