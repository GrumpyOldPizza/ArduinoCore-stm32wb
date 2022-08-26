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

#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <inttypes.h>

#include "Stream.h"

#define SERIAL_STOP_BIT_1    0x0000
#define SERIAL_STOP_BIT_2    0x0001
#define SERIAL_STOP_BIT_MASK 0x0001

#define SERIAL_PARITY_NONE   0x0000
#define SERIAL_PARITY_ODD    0x0002
#define SERIAL_PARITY_EVEN   0x0004
#define SERIAL_PARITY_MASK   0x0006

#define SERIAL_DATA_7        0x0000
#define SERIAL_DATA_8        0x0008
#define SERIAL_DATA_MASK     0x0008

#define SERIAL_RTS           0x0020
#define SERIAL_CTS           0x0040
#define SERIAL_RTSCTS        (SERIAL_RTS | SERIAL_CTS)

#define SERIAL_7N1           (SERIAL_DATA_7 | SERIAL_PARITY_NONE | SERIAL_STOP_BIT_1)
#define SERIAL_8N1           (SERIAL_DATA_8 | SERIAL_PARITY_NONE | SERIAL_STOP_BIT_1)
#define SERIAL_7N2           (SERIAL_DATA_7 | SERIAL_PARITY_NONE | SERIAL_STOP_BIT_2)
#define SERIAL_8N2           (SERIAL_DATA_8 | SERIAL_PARITY_NONE | SERIAL_STOP_BIT_2)
#define SERIAL_7O1           (SERIAL_DATA_7 | SERIAL_PARITY_ODD  | SERIAL_STOP_BIT_1)
#define SERIAL_8O1           (SERIAL_DATA_8 | SERIAL_PARITY_ODD  | SERIAL_STOP_BIT_1)
#define SERIAL_7O2           (SERIAL_DATA_7 | SERIAL_PARITY_ODD  | SERIAL_STOP_BIT_2)
#define SERIAL_8O2           (SERIAL_DATA_8 | SERIAL_PARITY_ODD  | SERIAL_STOP_BIT_2)
#define SERIAL_7E1           (SERIAL_DATA_7 | SERIAL_PARITY_EVEN | SERIAL_STOP_BIT_1)
#define SERIAL_8E1           (SERIAL_DATA_8 | SERIAL_PARITY_EVEN | SERIAL_STOP_BIT_1)
#define SERIAL_7E2           (SERIAL_DATA_7 | SERIAL_PARITY_EVEN | SERIAL_STOP_BIT_2)
#define SERIAL_8E2           (SERIAL_DATA_8 | SERIAL_PARITY_EVEN | SERIAL_STOP_BIT_2)

class HardwareSerial : public Stream
{
  public:
    virtual void begin(unsigned long) = 0;
    virtual void begin(unsigned long baudrate, uint32_t config) = 0;
    virtual void end() = 0;
    virtual int available(void) = 0;
    virtual int peek(void) = 0;
    virtual int read(void) = 0;
    virtual int read(uint8_t *buffer, size_t size) = 0;
    virtual int availableForWrite() = 0;;
    virtual size_t write(uint8_t) = 0;
    virtual size_t write(const uint8_t *buffer, size_t size);
    virtual void flush(void) = 0;
    using Print::write; // pull in write(str) and write(buf, size) from Print
    virtual operator bool() = 0;
};

extern void serialEventRun(void);

#endif
