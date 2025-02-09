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

#ifdef __cplusplus
extern "C" {
#endif

#define ARDUINO_APPLICATION_UUID(_u15, _u14, _u13, _u12, _u11, _u10, _u9, _u8, _u7, _u6, _u5, _u4, _u3, _u2, _u1, _u0)        \
    const uint8_t __attribute__((used, section(".info.uuid"))) __application_uuid__[16] = {                                   \
        (uint8_t)(_u0),  (uint8_t)(_u1),  (uint8_t)(_u2),  (uint8_t)(_u3),                                                    \
        (uint8_t)(_u4),  (uint8_t)(_u5),  (uint8_t)(_u6),  (uint8_t)(_u7),                                                    \
        (uint8_t)(_u8),  (uint8_t)(_u9),  (uint8_t)(_u10), (uint8_t)(_u11),                                                   \
        (uint8_t)(_u12), (uint8_t)(_u13), (uint8_t)(_u14), (uint8_t)(_u15)                                                    \
    }

#define ARDUINO_APPLICATION_VERSION(_major,_minor,_revision,_build)                                                           \
    const uint32_t __attribute__((used, section(".info.version"))) __application_version__[2] = {                             \
        ((((uint32_t)(_major) & 0xff) << 0) | (((uint32_t)(_minor) & 0xff) << 8) | (((uint32_t)(_revision) & 0xffff) << 16)), \
        (uint32_t)(_build)                                                                                                    \
    }
    
#define ARDUINO_APPLICATION_SEQUENCE(_sequence)                                                                               \
    const uint32_t __attribute__((used, section(".info.sequence"))) __application_sequence__[1] = {                           \
        ((uint32_t)(_sequence))                                                                                               \
    }
  
extern void init(void);

extern uint32_t SystemCoreClock;

#define F_CPU SystemCoreClock

extern uint32_t __FlashBase;
extern uint32_t __FlashLimit;

#ifdef __cplusplus
}
#endif
