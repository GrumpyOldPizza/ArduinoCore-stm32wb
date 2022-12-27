# Arduino Core for STM32WB based boards

## What is it ?

ArduinoCore-stm32wb is targeted at ultra low power scenarios, sensor hubs, with BLE connectivity.


## Supported boards

### Tlera Corp
 * [Firefly-STM32WB55CG](https://www.tindie.com/products/TleraCorp/firefly-ble-development-board)
 * [Katydid-STM32WB55CG](https://www.tindie.com/products/tleracorp/katydid-wearable-ble-sensor-board)

### STMicroelectronics
 * [NUCLEO-WB55RG](https://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/stm32-nucleo-expansion-boards/p-nucleo-wb55.html)

## Installing in PlatformIO (WIP)

You can use this Arduino core in PlatformIO. 

To do so, create any new project in the PIO Home screen (e.g., "Board: Arduino Uno" + "Framework: Arduino"), then overwrite the `platformio.ini` of the generated project with:

```ini
[env:nucleo_wb55rg]
; use forked platform
platform = https://github.com/maxgerhardt/platform-ststm32.git#stm32wb
board = grumpyoldpizza_nucleo_wb55rg
framework = arduino
build_flags = 
    -DPIO_FRAMEWORK_ARDUINO_STORAGE_TYPE_NONE
; if you need to source the core from a different repo
;platform_packages = framework-arduinoststm32wb@https://github.com/maxgerhardt/ArduinoCore-stm32wb.git
; if you need to source the core from the local filesystem
;platform_packages = framework-arduinoststm32wb@symlink://C:\Users\User\Desktop\dev\ArduinoCore-stm32wb
```

Also see example at https://github.com/maxgerhardt/pio-grumpyoldpizza-stm32wb-test.

Available `board` selection values:
* grumpyoldpizza_firefly_wb55cg
* grumpyoldpizza_katydid_wb55cg
* grumpyoldpizza_mothra_wb5mmg
* grumpyoldpizza_nucleo_wb55rg
* grumpyoldpizza_snoopy6_wb5mmg

CPU frequency selection example:
```ini
; 16 MHz (No USB)
board_build.f_cpu = 16000000L
```

The default USB type is "Serial" if the board supports USB (currently all except Nucleo WB55RG).
This can be changed by activating one of these macros through the `build_flags` of the `platformoi.ini`.
Remember that to combine multiple flags into one `build_flags` expression if you want to activate multiple flags.

```ini
; USB: "Serial" (default if USB available)
build_flags = -D PIO_FRAMEWORK_ARDUINO_ENABLE_CDC
; USB: "Serial + Mass Storage"
build_flags = -D PIO_FRAMEWORK_ARDUINO_ENABLE_CDC_WITH_MSC
; USB: "No USB"
build_flags = -D PIO_FRAMEWORK_ARDUINO_NO_USB
```

Configuration for "External Storage":

```ini
; Ext. Storage: None (default)
build_flags = -D PIO_FRAMEWORK_ARDUINO_STORAGE_TYPE_NONE
; Ext. Storage: SFLASH
build_flags = -D PIO_FRAMEWORK_ARDUINO_STORAGE_TYPE_SFLASH
; Ext. Storage: SDCARD
build_flags = -D PIO_FRAMEWORK_ARDUINO_STORAGE_TYPE_SDCARD
```

Example for combined values:
```ini
build_flags = 
   -D PIO_FRAMEWORK_ARDUINO_STORAGE_TYPE_SDCARD
   -D PIO_FRAMEWORK_ARDUINO_ENABLE_CDC_WITH_MSC
```

In case of platform or package updates, use
```
pio pkg update -g -p "https://github.com/maxgerhardt/platform-ststm32.git#stm32wb"
pio pkg update -g -t "https://github.com/maxgerhardt/ArduinoCore-stm32wb.git"
```

on [the CLI](https://docs.platformio.org/en/stable/integration/ide/vscode.html#platformio-core-cli).

## Installing in the Arduino IDE

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

 STM32WB uses encrypted/signed firmware images for the BLE stack. Please flash the `FWUpdate` sketch under the `STM32WB` examples before using BLE. The update will take up few seconds/minutes. During that time the LED will stay on, while the serial monitor will say disconnected. When done the LED will blink if the update was succesful, or simply be turned off if an error was encountered. The serial monitor should pop up again and report back the newly updated firmware versions. The `FWInfo` sketch can be used to verify what version is installed. At this time it should report back 1.10.0 for the BLE stack.

## Recovering from a faulty sketch for Tlera Corp Boards

 Sometimes a faulty sketch can render the normal USB Serial based integration into the Arduindo IDE not working. In this case plugin the STM32WB board and toggle the RESET button while holding down the BOOT button and program a known to be working sketch to go ack to a working USB Serial setup.

## Credits

This core is based on the [Arduino SAMD Core](https://github.com/arduino/ArduinoCore-samd)

