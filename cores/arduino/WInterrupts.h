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

#ifndef _WIRING_INTERRUPTS_
#define _WIRING_INTERRUPTS_

#include <stdint.h>

//      LOW 0
//      HIGH 1
#define CHANGE 2
#define FALLING 3
#define RISING 4

#ifdef __cplusplus
void attachInterrupt(uint32_t pin, void(*callback)(void), uint32_t mode, int priority = 2);
void attachInterrupt(uint32_t pin, Callback callback, uint32_t mode, int priority = 2);

extern "C" {
void detachInterrupt(uint32_t pin);
}

#else

void attachInterrupt(uint32_t pin, void(*callback)(void), uint32_t mode);
void detachInterrupt(uint32_t pin);

#endif

#endif
