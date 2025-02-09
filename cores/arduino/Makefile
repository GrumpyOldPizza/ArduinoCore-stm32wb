#TOOLS    = ~/.arduino15/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/
TOOLS    = /opt/arm-none-eabi-gcc/10.3-2021.10
CPP      = $(TOOLS)/bin/arm-none-eabi-cpp
CC       = $(TOOLS)/bin/arm-none-eabi-gcc
CXX      = $(TOOLS)/bin/arm-none-eabi-g++
AS       = $(TOOLS)/bin/arm-none-eabi-as
AR       = $(TOOLS)/bin/arm-none-eabi-ar
LD       = $(TOOLS)/bin/arm-none-eabi-ld
OBJCOPY  = $(TOOLS)/bin/arm-none-eabi-objcopy
CFLAGS   = -c -g3 -Os $(WARNINGS) -std=gnu11 -ffast-math -ffunction-sections -fdata-sections -nostdlib -MMD $(EXTRAS) $(DEFINES) $(INCLUDES)
CXXFLAGS = -c -g3 -Os $(WARNINGS) -std=gnu++11 -ffast-math -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib -fno-use-cxa-atexit -fno-rtti -fno-exceptions -MMD $(EXTRAS) $(DEFINES) $(INCLUDES)
ASFLAGS  = -c -g -x assembler-with-cpp $(EXTRAS) $(DEFINES) $(INCLUDES)
#LDFLAGS  = -g -Os $(EXTRAS) -Wl,--gc-sections,--no-undefined -T../../system/STM32WBxx/LdScripts/STM32WB55xx_FLASH.ld --specs=nano.specs --specs=nosys.specs -Wl,--defsym=__RTC_EPOCH__=$(shell date +"%s")
LDFLAGS  = -g -Os $(EXTRAS) -Wl,--gc-sections,--no-undefined -T../../system/STM32WBxx/LdScripts/STM32WB55xx_COMPOSITE.ld --specs=nano.specs -Wl,--defsym=__RTC_EPOCH__=$(shell date +"%s")
WARNINGS = -Wall -Wextra
EXTRAS   = -mcpu=cortex-m4 -march=armv7e-m+fp -mthumb -mabi=aapcs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -falign-functions=6
#DEFINES  = -DSTM32WB55xx -D__SYSTEM_CORE_CLOCK__=64000000 -DSTORAGE_TYPE=2 -DARDUINO_MAKEFILE
DEFINES  = -DSTM32WB55xx -D__SYSTEM_CORE_CLOCK__=64000000 -DSTORAGE_TYPE=0 -DARDUINO_MAKEFILE -DUSB_VID=0x1209 -DUSB_PID=0x6677 -DUSB_MANUFACTURER="\"Sparkfun\"" -DUSB_PRODUCT="\"MicroMod-WB5MMG\"" -DUSB_TYPE=1
EPOCH    = $(shell date +"%s")

INCLUDES = \
	-I../../system/CMSIS/Core/Include \
	-I../../system/CMSIS/Device/ST/STM32WBxx/Include \
	-I../../system/CMSIS/RTOS2/Include \
	-I../../system/STM32WBxx/Include \
	-I../../variants/MicroMod-WB5MMG \
	-I../../libraries/BLE/src \
	-I../../libraries/DOSFS/src \
	-I../../libraries/EEPROM/src \
	-I../../libraries/GNSS/src \
	-I../../libraries/GNSS/src/utility \
	-I../../libraries/I2S/src \
	-I../../libraries/PDM/src \
	-I../../libraries/RTC/src \
	-I../../libraries/Servo/src \
	-I../../libraries/SFLASH/src \
	-I../../libraries/SPI/src \
	-I../../libraries/STM32WB/src \
	-I../../libraries/TimerMillis/src \
	-I../../libraries/USBStorage/src \
	-I../../libraries/Wire/src \
	-I.
SRCS = \
	../../system/CMSIS/RTOS2/Source/cmsis_os2.c \
	../../libraries/BLE/src/BLE.cpp \
	../../libraries/DOSFS/src/DOSFS.cpp \
	../../libraries/GNSS/src/GNSS.cpp \
	../../libraries/GNSS/src/utility/gnss_core.c \
	../../libraries/I2S/src/I2S.cpp \
	../../libraries/PDM/src/PDM.cpp \
	../../libraries/RTC/src/RTC.cpp \
	../../libraries/Servo/src/Servo.cpp \
	../../libraries/SFLASH/src/SFLASH.cpp \
	../../libraries/SPI/src/SPI.cpp \
	../../libraries/STM32WB/src/STM32WB.cpp \
	../../libraries/TimerMillis/src/TimerMillis.cpp \
	../../libraries/USBStorage/src/USBStorage.cpp \
	../../libraries/Wire/src/Wire.cpp \
	../../variants/MicroMod-WB5MMG/variant.cpp \
	avr/dtostrf.c \
	avr/eeprom.c \
	Callback.cpp \
	CDC.cpp \
	HardwareSerial.cpp \
	HardwareSerial1.cpp \
	HardwareSerial2.cpp \
	IPAddress.cpp \
	Print.cpp \
	RingBuffer.cpp \
	Stream.cpp \
	Tone.cpp \
	Uart.cpp \
	USBCore.cpp \
	WInterrupts.cpp \
	WMath.cpp \
	WString.cpp \
	abi.cpp \
	delay.c \
	hooks.c \
	itoa.c \
	new.cpp \
	syscalls_stm32wb.c \
	wiring.c \
	wiring_analog.cpp \
	wiring_digital.c \
	wiring_pulse.c \
	wiring_shift.c

OBJS = \
	../../system/CMSIS/RTOS2/Source/cmsis_os2.o \
	../../libraries/BLE/src/BLE.o \
	../../libraries/DOSFS/src/DOSFS.o \
	../../libraries/GNSS/src/GNSS.o \
	../../libraries/GNSS/src/utility/gnss_core.o \
	../../libraries/I2S/src/I2S.o \
	../../libraries/PDM/src/PDM.o \
	../../libraries/RTC/src/RTC.o \
	../../libraries/Servo/src/Servo.o \
	../../libraries/SFLASH/src/SFLASH.o \
	../../libraries/SPI/src/SPI.o \
	../../libraries/STM32WB/src/STM32WB.o \
	../../libraries/TimerMillis/src/TimerMillis.o \
	../../libraries/USBStorage/src/USBStorage.o \
	../../libraries/Wire/src/Wire.o \
	../../variants/MicroMod-WB5MMG/variant.o \
	avr/dtostrf.o \
	avr/eeprom.o \
	Callback.o \
	CDC.o \
	HardwareSerial.o \
	HardwareSerial1.o \
	HardwareSerial2.o \
	IPAddress.o \
	Print.o \
	RingBuffer.o \
	Stream.o \
	Tone.o \
	Uart.o \
	USBCore.o \
	WInterrupts.o \
	WMath.o \
	WString.o \
	abi.o \
	delay.o \
	hooks.o \
	itoa.o \
	new.o \
	syscalls_stm32wb.o \
	wiring.o \
	wiring_analog.o \
	wiring_digital.o \
	wiring_pulse.o \
	wiring_shift.o

all:: flash.elf flash.bin

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.S
	$(CC) $(ASFLAGS) -c $< -o $@

libarduino.a:: $(OBJS)
	$(AR) rcs $@ $^

flash.elf: main.o libarduino.a ../../system/STM32WBxx/Source/_out/stm32wb55xx/stm32wb_boot.o ../../system/STM32WBxx/Lib/libstm32wb55xx.a
	$(CC) $(LDFLAGS) main.o ../../system/STM32WBxx/Source/_out/stm32wb55xx/stm32wb_boot.o -Wl,--gc-sections,--no-undefined,-Map=flash.map -Wl,--start-group -L. -larduino -L../../system/STM32WBxx/Lib -lstm32wb55xx -lm -lc -Wl,--end-group -o flash.elf
#	$(CC) $(LDFLAGS) main.o ../../system/STM32WBxx/Source/_out/stm32wb55xx/stm32wb_boot.o -Wl,--gc-sections,--no-undefined,-Map=flash.map -Wl,--start-group -L. -larduino -L../../system/STM32WBxx/Lib -lstm32wb55xx_debug -lm -lc -Wl,--end-group -o flash.elf
#	$(CC) $(LDFLAGS) main.o -Wl,--gc-sections,--no-undefined,-Map=flash.map -Wl,--start-group -L. -larduino -L../../system/STM32WBxx/Lib -lstm32wb55xx_debug -lm -lc -Wl,--end-group -o flash.elf
#	../../tools/linux/elfsign -u eafee6d0-b575-4412-aeb4-c30ecb0d3c6e -v 1209 -p 6675 -e ../../libraries/STM32WB/keys/aes128_key.bin -s ../../libraries/STM32WB/keys/ecc256_key.pem -b flash.bin -d flash.dfu -o flash.ota -x flash.hex flash.elf flash.elf
	../../tools/linux/elfsign -u eafee6d0-b575-4412-aeb4-c30ecb0d3c6e -v 1209 -p 6675 -b flash.bin -d flash.dfu -o flash.ota -x flash.hex flash.elf flash.elf
#	../../tools/linux/elfsign -u 07fcdd24-aa7d-4bd5-93b1-9923e90c457c -v 1209 -p 6675 -b flash.bin -d flash.dfu -o flash.ota -x flash.hex flash.elf flash.elf

clean::
	rm -f flash.elf flash.bin flash.dfu flash.ota flash.hex flash.map libarduino.a *~ *.o *.d main.o $(OBJS)

-include $(OBJS:.o=.d)
