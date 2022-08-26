#TOOLS    = ~/.arduino15/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/
#TOOLS    = ~/.arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/
TOOLS    = /opt/arm-none-eabi-gcc/10.3-2021.10
CPP      = $(TOOLS)/bin/arm-none-eabi-cpp
CC       = $(TOOLS)/bin/arm-none-eabi-gcc
CXX      = $(TOOLS)/bin/arm-none-eabi-g++
AS       = $(TOOLS)/bin/arm-none-eabi-as
AR       = $(TOOLS)/bin/arm-none-eabi-ar
LD       = $(TOOLS)/bin/arm-none-eabi-ld
CFLAGS   = -mcpu=cortex-m4 -mthumb -c -g -Os $(WARNINGS) -std=gnu11 -ffunction-sections -fdata-sections -nostdlib -MMD $(EXTRAS) $(DEFINES) $(INCLUDES)
CXXFLAGS = -mcpu=cortex-m4 -mthumb -c -g -Os $(WARNINGS) -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib -fno-rtti -fno-exceptions -MMD $(EXTRAS) $(DEFINES) $(INCLUDES)
ASFLAGS  = -c -g -x assembler-with-cpp $(EXTRAS) $(DEFINES) $(INCLUDES)
WARNINGS = -Wall -Wextra -Wno-unused-parameter
EXTRAS   = -mcpu=cortex-m4 -march=armv7e-m -mthumb -mabi=aapcs -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFINES  = 
INCLUDES = \
	-I../../../system/CMSIS/Include \
	-I../../../system/CMSIS/Device/ST/STM32WBxx/Include \
	-I../../../system/STM32WBxx/Include \
	-I. 
SRCS = \
	BLE/ble_events.c \
	BLE/ble_gap_aci.c \
	BLE/ble_gatt_aci.c \
	BLE/ble_hal_aci.c \
	BLE/ble_hci_le.c \
	BLE/ble_l2cap_aci.c \
	armv7m_atomic.c \
	armv7m_core.c \
	armv7m_pdm.S \
	armv7m_pendsv.c \
	armv7m_rtlib.S \
	armv7m_rtos.c \
	armv7m_rtt.c \
	armv7m_svcall.c \
	armv7m_systick.c \
	dosfs_core.c \
	dosfs_device.c \
	dosfs_sflash.c \
	dosfs_storage.c \
	stm32wb_adc.c \
	stm32wb_dma.c \
	stm32wb_eeprom.c \
	stm32wb_exti.c \
	stm32wb_flash.c \
	stm32wb_gpio.c \
	stm32wb_hsem.c \
	stm32wb_i2c.c \
	stm32wb_ipcc.c \
	stm32wb_iwdg.c \
	stm32wb_lptim.c \
	stm32wb_random.c \
	stm32wb_rtc.c \
	stm32wb_sai.c \
	stm32wb_sdspi.c \
	stm32wb_sflash.c \
	stm32wb_sfspi.c \
	stm32wb_sfsqi.c \
	stm32wb_spi.c \
	stm32wb_servo.c \
	stm32wb_system.c \
	stm32wb_tim.c \
	stm32wb_uart.c \
	stm32wb_usbd.c \
	stm32wb_usbd_cdc.c \
	stm32wb_usbd_dfu.c \
	stm32wb_usbd_dcd.c \
	stm32wb_usbd_info.c \
	stm32wb_usbd_msc.c

SRCS_WB55 = $(SRCS) startup_stm32wb55xx.S
OBJS_WB55 = $(patsubst %.c,_out/stm32wb55xx/%.o,$(filter-out %.S,$(SRCS_WB55))) $(patsubst %.S,_out/stm32wb55xx/%.o,$(filter-out %.c,$(SRCS_WB55)))

all:: libstm32wb55xx.a

libstm32wb55xx.a:: $(OBJS_WB55)
	$(AR) rcs $@ $^

_out/stm32wb55xx/%.o: %.c
	-@mkdir -p $(dir $@)
	$(CC) -DSTM32WB55xx $(CFLAGS) -c $< -o $@

_out/stm32wb55xx/%.o: %.S
	-@mkdir -p $(dir $@)
	$(CC) -DSTM32WB55xx $(ASFLAGS) -c $< -o $@

clean::
	rm -rf libstm32wb55xx.a _out *~

-include $(OBJS_WB55:.o=.d)


