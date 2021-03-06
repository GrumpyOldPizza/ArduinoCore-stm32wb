TOOLS    = ~/.arduino15/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/
CPP      = $(TOOLS)/bin/arm-none-eabi-cpp
CC       = $(TOOLS)/bin/arm-none-eabi-gcc
CXX      = $(TOOLS)/bin/arm-none-eabi-g++
AS       = $(TOOLS)/bin/arm-none-eabi-as
AR       = $(TOOLS)/bin/arm-none-eabi-ar
LD       = $(TOOLS)/bin/arm-none-eabi-ld
CFLAGS   = -mcpu=cortex-m4 -mthumb -c -g3 -Os $(WARNINGS) -std=gnu11 -ffunction-sections -fdata-sections -nostdlib -MMD $(EXTRAS) $(DEFINES) $(INCLUDES)
CXXFLAGS = -mcpu=cortex-m4 -mthumb -c -g3 -Os $(WARNINGS) -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib -fno-use-cxa-atexit -fno-rtti -fno-exceptions -MMD $(EXTRAS) $(DEFINES) $(INCLUDES)
ASFLAGS  = -c -g -x assembler-with-cpp $(EXTRAS) $(DEFINES) $(INCLUDES)
LDFLAGS = -g -Os -mcpu=cortex-m4 -march=armv7e-m -mthumb -mabi=aapcs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Wl,--gc-sections,--no-undefined -T../../system/STM32WBxx/Lib/STM32WB55xx_FLASH.ld --specs=nano.specs --specs=nosys.specs -Wl,--defsym=__RTC_EPOCH__=0 -Wl,--defsym=__RTC_ZONE__=0 -Wl,--defsym=__RTC_DST__=0
WARNINGS = -Wall -Wextra
EXTRAS   = -DSTM32WB55xx -mcpu=cortex-m4 -march=armv7e-m -mthumb -mabi=aapcs -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFINES  = -D__SYSTEM_CORE_CLOCK__=64000000 -DDOSFS_SDCARD=0 -DDOSFS_SFLASH=0 -DARDUINO_MAKEFILE
#DEFINES  = -D__SYSTEM_CORE_CLOCK__=64000000 -DDOSFS_SDCARD=0 -DDOSFS_SFLASH=0 -DARDUINO_MAKEFILE -DSTORAGE_TYPE=2
#DEFINES  = -D__SYSTEM_CORE_CLOCK__=64000000 -DDOSFS_SDCARD=0 -DDOSFS_SFLASH=0 -DARDUINO_MAKEFILE -DUSB_TYPE=0
#DEFINES  = -D__SYSTEM_CORE_CLOCK__=32000000 -DDOSFS_SDCARD=0 -DDOSFS_SFLASH=0 -DARDUINO_MAKEFILE -DUSB_TYPE=0
#DEFINES  = -D__SYSTEM_CORE_CLOCK__=16000000 -DDOSFS_SDCARD=0 -DDOSFS_SFLASH=0 -DARDUINO_MAKEFILE -DUSB_TYPE=0
#DEFINES  = -D__SYSTEM_CORE_CLOCK__=2000000 -DDOSFS_SDCARD=0 -DDOSFS_SFLASH=0 -DARDUINO_MAKEFILE -DUSB_TYPE=0
INCLUDES = \
	-I../../system/CMSIS/Include \
	-I../../system/CMSIS/Device/ST/STM32WBxx/Include \
	-I../../system/STM32WBxx/Include \
	-I../../variants/Firefly-WB55CG \
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
	-I../../libraries/Wire/src \
	-I.
SRCS = \
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
	../../libraries/Wire/src/Wire.cpp \
	../../variants/Firefly-WB55CG/variant.cpp \
	avr/dtostrf.c \
	avr/eeprom.c \
	avr/fdevopen.c \
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
	main.cpp \
	new.cpp \
	syscalls_stm32wb.c \
	wiring.c \
	wiring_analog.c \
	wiring_digital.c \
	wiring_pulse.c \
	wiring_shift.c

OBJS = \
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
	../../libraries/Wire/src/Wire.o \
	../../variants/Firefly-WB55CG/variant.o \
	avr/dtostrf.o \
	avr/eeprom.o \
	avr/fdevopen.o \
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
	main.o \
	new.o \
	syscalls_stm32wb.o \
	wiring.o \
	wiring_analog.o \
	wiring_digital.o \
	wiring_pulse.o \
	wiring_shift.o

all:: flash.elf

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.S
	$(CC) $(ASFLAGS) -c $< -o $@

flash.elf: $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -L../../system/STM32WBxx/Lib -lstm32wb55xx -lm -lc -lgcc -lstm32wb55xx -o flash.elf

clean::
	rm -f flash.elf flash.bin *~ *.o *.d $(OBJS)

-include $(OBJS:.o=.d)
