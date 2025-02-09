#TOOLS    = ~/.arduino15/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/
#TOOLS    = ~/.arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/
TOOLS    = /opt/arm-none-eabi-gcc/10.3-2021.10
CPP      = $(TOOLS)/bin/arm-none-eabi-cpp
CC       = $(TOOLS)/bin/arm-none-eabi-gcc
CXX      = $(TOOLS)/bin/arm-none-eabi-g++
AS       = $(TOOLS)/bin/arm-none-eabi-as
AR       = $(TOOLS)/bin/arm-none-eabi-ar
LD       = $(TOOLS)/bin/arm-none-eabi-ld
OBJCOPY  = $(TOOLS)/bin/arm-none-eabi-objcopy
CFLAGS   = -c -g -Os $(WARNINGS) -std=gnu11 -ffast-math -ffunction-sections -fdata-sections -nostdlib -MMD $(EXTRAS) $(DEFINES) $(INCLUDES)
GCFLAGS  = -c -g3 -Og $(WARNINGS) -std=gnu11 -ffast-math -ffunction-sections -fdata-sections -nostdlib -MMD $(EXTRAS) $(DEFINES) $(INCLUDES) -DDEBUG
ASFLAGS  = -c -g -x assembler-with-cpp $(EXTRAS) $(DEFINES) $(INCLUDES)
WARNINGS = -Wall -Wextra -Wno-unused-parameter
EXTRAS   = -mcpu=cortex-m4 -march=armv7e-m+fp -mthumb -mabi=aapcs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -falign-functions=6
DEFINES  = 
INCLUDES = \
	-I../../../system/CMSIS/Core/Include \
	-I../../../system/CMSIS/Device/ST/STM32WBxx/Include \
	-I../../../system/STM32WBxx/Include \
	-I.

BSRCS = \
	stm32wb_boot.c

LSRCS = \
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
	stm32wb_fwu.c \
	stm32wb_gpio.c \
	stm32wb_hsem.c \
	stm32wb_i2c.c \
	stm32wb_ipcc.c \
	stm32wb_iwdg.c \
	stm32wb_lptim.c \
	stm32wb_otp.c \
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

BSRCS_WB55 = $(BSRCS)
BOBJS_WB55 = $(patsubst %.c,_out/stm32wb55xx/%.o,$(filter-out %.S,$(BSRCS_WB55))) $(patsubst %.S,_out/stm32wb55xx/%.o,$(filter-out %.c,$(BSRCS_WB55)))

LSRCS_WB55 = $(LSRCS) startup_stm32wb55xx.S
LOBJS_WB55 = _out/stm32wb55xx/boot_stm32wb55xx.o $(patsubst %.c,_out/stm32wb55xx/%.o,$(filter-out %.S,$(LSRCS_WB55))) $(patsubst %.S,_out/stm32wb55xx/%.o,$(filter-out %.c,$(LSRCS_WB55)))
GOBJS_WB55 = _out/stm32wb55xx/boot_stm32wb55xx.o $(patsubst %.c,_out/stm32wb55xx/%_debug.o,$(filter-out %.S,$(LSRCS_WB55))) $(patsubst %.S,_out/stm32wb55xx/%_debug.o,$(filter-out %.c,$(LSRCS_WB55)))

all:: ../Lib/libstm32wb55xx.a  ../Lib/libstm32wb55xx_debug.a

../Lib/libstm32wb55xx.a:: $(LOBJS_WB55)
	rm -f $@
	$(AR) rcs $@ $^

../Lib/libstm32wb55xx_debug.a:: $(GOBJS_WB55)
	rm -f $@
	$(AR) rcs $@ $^

_out/stm32wb55xx/boot_stm32wb55xx.elf:: $(BOBJS_WB55)
	$(CC) $(LDFLAGS) $(BOBJS_WB55) -nostdlib -Wl,--gc-sections,--no-undefined,-Map=boot_stm32wb55xx.map -T../LdScripts/STM32WB55xx_BOOT.ld -o $@

_out/stm32wb55xx/boot_stm32wb55xx.bin:: _out/stm32wb55xx/boot_stm32wb55xx.elf
	$(OBJCOPY) -O binary $< $@

_out/stm32wb55xx/boot_stm32wb55xx.o:: _out/stm32wb55xx/boot_stm32wb55xx.bin
	$(OBJCOPY) -S -I binary -O elf32-littlearm --rename-section .data=.boot --add-symbol __boot_base__=.boot:0 $< $@

_out/stm32wb55xx/stm32wb_boot.o: stm32wb_boot.c
	-@mkdir -p $(dir $@)
	$(CC) -DSTM32WB55xx -c -g -Os $(WARNINGS) -std=gnu11 -ffast-math -ffunction-sections -fdata-sections -nostdlib -MMD $(EXTRAS) -mpure-code $(DEFINES) $(INCLUDES) $< -o $@
#	$(CC) -DSTM32WB55xx -c -g -Os $(WARNINGS) -std=gnu11 -ffast-math -ffunction-sections -fdata-sections -nostdlib -MMD $(EXTRAS) $(DEFINES) $(INCLUDES) $< -o $@

_out/stm32wb55xx/%.o: %.c
	-@mkdir -p $(dir $@)
	$(CC) -DSTM32WB55xx $(CFLAGS) -c $< -o $@

_out/stm32wb55xx/%.o: %.S
	-@mkdir -p $(dir $@)
	$(CC) -DSTM32WB55xx $(ASFLAGS) -c $< -o $@

_out/stm32wb55xx/%_debug.o: %.c
	-@mkdir -p $(dir $@)
	$(CC) -DSTM32WB55xx $(GCFLAGS) -c $< -o $@

_out/stm32wb55xx/%_debug.o: %.S
	-@mkdir -p $(dir $@)
	$(CC) -DSTM32WB55xx $(ASFLAGS) -c $< -o $@

clean::
	rm -rf _out *.map *~

-include $(BOBJS_WB55:.o=.d) $(LOBJS_WB55:.o=.d) $(GOBJS_WB55:.o=.d)


