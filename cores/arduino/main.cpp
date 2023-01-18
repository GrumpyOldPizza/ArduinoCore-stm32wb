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

#define ARDUINO_MAIN
#include "Arduino.h"
#include "wiring_private.h"

#if defined(ARDUINO_MAKEFILE)

void setup(void) {
}

void loop(void) {
}

#endif

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() {
}

#if defined(USBCON)

void initUSB() __attribute__((weak));
void initUSB() {
  USBDevice.begin();
  USBDevice.start();
}

#endif

void (*g_serialEventRun)(void) = NULL;

/*
 * \brief Main entry point of Arduino application
 */
int main( void ) {
  init();
  initVariant();

#if defined(USBCON)
  initUSB();
#endif
  
  setup();

  for (;;)
  {
    loop();
    if (g_serialEventRun) (*g_serialEventRun)();
  }

  return 0;
}
