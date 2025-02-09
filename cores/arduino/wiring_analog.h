/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * \brief STM32WB products have only one reference for ADC
 */
typedef enum _eAnalogReference
{
  AR_INTERNAL_2V0 = 0,
  AR_INTERNAL_2V5 = 1,
  AR_EXTERNAL     = 2,
  AR_DEFAULT      = AR_EXTERNAL
} eAnalogReference ;


extern void analogReference(eAnalogReference reference);
extern void analogReadPeriod(float period);
extern void analogReadSamples(int samples);
extern void analogReadResolution(int resolution);
extern uint32_t analogRead(uint32_t pin);
  
/*
 * \brief Writes an analog value (PWM wave) to a pin.
 *
 * \param ulPin
 * \param ulValue
 */
extern void analogWrite( uint32_t ulPin, uint32_t ulValue ) ;

/*  
 * \brief Set the resolution of analogWrite parameters. Default is 8 bits (range from 1 to 12).
 *
 * \param res
 */
extern void analogWriteResolution(int resolution);

/*
 * \brief Set the frequency of analogWrite PWM. A 0 switches back to the default.
 *
 * \param res
 */
extern void analogWriteFrequency( unsigned long frequency );
  
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

extern uint32_t analogRead(uint32_t pin, bool stop);
extern bool analogRead(const uint8_t pins[], uint16_t data[], uint32_t count, bool stop = true);

#endif
