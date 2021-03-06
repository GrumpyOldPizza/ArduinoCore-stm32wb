/*
  Servo.h - Interrupt driven Servo library for Arduino using 16 bit timers- Version 2
  Copyright (c) 2009 Michael Margolis.  All right reserved.
  Copyright (c) 2015 Thomas Roell.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef Servo_h
#define Servo_h

#include <inttypes.h>

#define Servo_VERSION           2     // software version of this library

#define MIN_PULSE_WIDTH       800     // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH      2200     // the longest pulse sent to a servo 
#define DEFAULT_PULSE_WIDTH  1500     // default pulse width when servo is attached
#define REFRESH_INTERVAL    20000     // minumim time to refresh servos in microseconds 

// NOTE: to maintain a strict refresh interval the user needs to not exceed 2200us pulse width
#define MAX_SERVOS              9

class Servo
{
public:
    Servo();
    uint8_t attach(uint32_t pin, int min = MIN_PULSE_WIDTH, int max = MAX_PULSE_WIDTH);
    void detach();
    void write(int angle);             // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds 
    void writeMicroseconds(int width);
    int read();
    int readMicroseconds();
    bool attached();
private:
    uint8_t m_index;
    uint16_t m_pin;
    int16_t m_width;
    int16_t m_min;
    int16_t m_max;
};

#endif
