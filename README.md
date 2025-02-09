# Arduino Core for STM32WB based boards

## What is it ?

ArduinoCore-stm32wb is targeted at ultra low power scenarios, sensor hubs, with BLE connectivity.


## Supported boards

### Tlera Corp
 * [Firefly-STM32WB55CG](https://www.tindie.com/products/TleraCorp/firefly-ble-development-board)
 * [Katydid-STM32WB55CG](https://www.tindie.com/products/tleracorp/katydid-wearable-ble-sensor-board)

### Sparkfun
 * [MicroMod-STM32WB5MMG](https://www.sparkfun.com/products/21438)

### STMicroelectronics
 * [NUCLEO-WB55RG](https://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/stm32-nucleo-expansion-boards/p-nucleo-wb55.html)


## Installing

 The Arduino Core for STM32WB does use Secure Boot / Secure Firmware Update. To do so, a special bootloader is installed the first time a sketch is uploaded. To change the bootloader to a newer version, or to switch security settings between `None` and `Signature & Encryption` the STM32WB board has to be reset by toggle the RESET button while holding down the BOOT button, and then uploading a sketch with the new boot code or settings.

 PLEASE ALWAYS WHEN UGRADING, TOGGLE THE RESET BUTTON WHILE HOLDING DOWN THE BOOT BUTTON BEFORE UPLOADING THE FIRST SKETCH.


### Board Manager

 1. [Download and install the Arduino IDE](https://www.arduino.cc/en/Main/Software) (at least version v1.8.13)
 2. Start the Arduino IDE
 3. Go into Preferences
 4. Add `https://grumpyoldpizza.github.io/ArduinoCore-stm32wb/package_stm32wb_boards_index.json` as an "Additional Board Manager URL"
 5. Open the Boards Manager from the Tools -> Board menu and install "Tlera Corp STM32WB Boards"
 6. Select your STM32WB board from the Tools -> Board menu

#### OS Specific Setup

##### Linux

 1. Go to `~/.arduino15/packages/TleraCorp/hardware/stm32wb/<VERSION>/drivers/linux/`
 2. `sudo cp *.rules /etc/udev/rules.d`
 3. reboot

##### Windows

###### STM32 BOOTLOADER driver setup for Tlera Corp boards

 1. Plugin STM32WB board and toggle the RESET button while holding down the BOOT button
 2. Go to `%AppData%` and navigate from to `AppData\Local\Arduino15\packages\TleraCorp\hardware\stm32wb\<VERSION>\drivers\windows`
 3. Right-click on `STM32Bootloader.bat` and select `Run as administrator`

###### ST-LINK driver setup for NUCLEO boards

 1. Plugin NUCLEO board
 2. Download and install [ST-Link USB Drivers](http://www.st.com/en/embedded-software/stsw-link009.html)

### From git (for core development)

 1. Follow steps from Board Manager section above
 2. `cd <SKETCHBOOK>`, where `<SKETCHBOOK>` is your Arduino Sketch folder:
  * OS X: `~/Documents/Arduino`
  * Linux: `~/Arduino`
  * Windows: `Documents\Arduino`
 3. Create a folder named `hardware`, if it does not exist, and change directories to it
 4. Clone this repo: `git clone https://github.com/grumpyoldpizza/ArduinoCore-stm32wb.git TleraCorp/stm32wb`
 5. Restart the Arduino IDE

## BLE / Wireless stack firmware installation

 STM32WB uses a signed/encrypted firmware image for the BLE stack. Please flash the `FWUpdate` sketch under the `STM32WB` examples before using BLE. The update will take up few seconds/minutes. During that time the LED will stay on, while the serial monitor will say disconnected. When done the LED will blink if the update was succesful, or simply be turned off if an error was encountered. The serial monitor should pop up again and report back the newly updated firmware versions. The `FWInfo` sketch can be used to verify what version is installed. At this time it should report back 1.14.2 for the BLE stack.

## Secure Boot / Secure Firmware Update / BLE OTA

 STM32WB can use a signed/encrypted firmware image for the Arduino application/sketche. With the `Export compiled Binary` entry in the `Sketch` menu 5 files are exported. The *.dfu file is used for secure firmware update via USB/DFU, while the *.ota file is use BLE/OTA. This can be used/tested with either the "ST BLE Toolbox" application for Android/iOS (the file might have to be renamed to fit ST's naming conventions), or with the supplied `stm32wb-ota.py` python3 script (requires `python3` and `bleak`). USB/DFU uses signature/encryption, while BLE/OTA uses signature/encryption/compression.

## Recovering from a faulty sketch for Tlera Corp / Sparkfun Boards

 Sometimes a faulty sketch can render the normal USB DUF based integration into the Arduindo IDE not working. In this case plugin the STM32WB board and toggle the RESET button while holding down the BOOT button and program a known to be working sketch to go back to a working USB DFU setup.

## Credits

This core is based on the [Arduino SAMD Core](https://github.com/arduino/ArduinoCore-samd)

