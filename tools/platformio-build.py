# Copyright 2022-present Maximilian Gerhardt <maxmimilian.gerhardt@rub.de>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import time    
from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()
platform = env.PioPlatform()
board = env.BoardConfig()

framework_package = "framework-arduinoststm32wb"
FRAMEWORK_DIR = platform.get_package_dir(framework_package)
BUILD_CORE = "arduino"

assert os.path.isdir(FRAMEWORK_DIR)

# make RTC infos available
# see https://github.com/arduino/arduino-cli/blob/63f1e1855aa4c91415480be40fc49fd50d597e35/legacy/builder/setup_build_properties.go#L121-L125
# https://arduino.github.io/arduino-cli/0.29/platform-specification/#global-predefined-properties
# in this core
# -Wl,--defsym=__RTC_EPOCH__={extra.time.utc}
# -Wl,--defsym=__RTC_ZONE__={extra.time.zone}
# -Wl,--defsym=__RTC_DST__={extra.time.dst}
# -Wl,--defsym=__RTC_LEAP_SECONDS=18
# extra.time.utc
time_utc = int(time.time())
# extra.time.zone
time_zone = -time.timezone
is_dst = time.daylight and time.localtime().tm_isdst > 0
# extra.time.dst
time_dst = - (time.altzone if is_dst else 0)

variants_dir = os.path.join(
    "$PROJECT_DIR", board.get("build.variants_dir")) if board.get(
        "build.variants_dir", "") else os.path.join(FRAMEWORK_DIR, "variants")

machine_flags = [
    "-mcpu=%s" % board.get("build.cpu"),
    "-march=armv7e-m",
    "-mthumb", 
    "-mabi=aapcs",
]

env.Append(
    ASFLAGS=machine_flags,
    ASPPFLAGS=[
        "-x", "assembler-with-cpp",
    ],

    CFLAGS=[
        "-std=gnu11"
    ],

    CCFLAGS=machine_flags + [
        "-Os",  # optimize for size
        "-ffunction-sections",  # place each function in its own section
        "-fdata-sections",
        "-Wall",
        "-nostdlib",
        "--param", "max-inline-insns-single=500"
    ],

    CXXFLAGS=[
        "-fno-threadsafe-statics",
        "-fno-rtti",
        "-fno-exceptions",
        "-std=gnu++11",
        "-fno-threadsafe-statics",
        "-fsingle-precision-constant"
    ],

    CPPDEFINES=[
        ("ARDUINO", 10819),
        ("__SYSTEM_CORE_CLOCK__", "$BOARD_F_CPU"), # take value from board_build.f_cpu = ...
        "ARDUINO_ARCH_STM32WB"
    ],

    CPPPATH=[
        os.path.join(FRAMEWORK_DIR, "cores", BUILD_CORE),
        os.path.join(FRAMEWORK_DIR, "system", "CMSIS", "Core", "Include"), 
        os.path.join(FRAMEWORK_DIR, "system", "CMSIS", "DSP", "Include"),
        os.path.join(FRAMEWORK_DIR, "system", "CMSIS", "Device", "ST", "STM32WBxx", "Include"),
        os.path.join(FRAMEWORK_DIR, "system", "STM32WBxx", "Include") 
    ],

    LIBSOURCE_DIRS=[
        os.path.join(FRAMEWORK_DIR, "libraries")
    ],

    LIBPATH=[
        os.path.join(FRAMEWORK_DIR, "system", "STM32WBxx", "Lib"),
        os.path.join(FRAMEWORK_DIR, "system", "CMSIS", "DSP", "Lib")
    ],

    LINKFLAGS=machine_flags + [
        "-Os",
        "--specs=nosys.specs",
        "--specs=nano.specs",
        "-Wl,--gc-sections",
        "-Wl,--check-sections",
        "-Wl,--unresolved-symbols=report-all",
        "-Wl,--warn-common",
        "-Wl,--warn-section-align",
        "-Wl,--defsym=__RTC_EPOCH__=%d" % time_utc,
        "-Wl,--defsym=__RTC_ZONE__=%d" % time_zone,
        "-Wl,--defsym=__RTC_DST__=%d" % time_dst,
        "-Wl,--defsym=__RTC_LEAP_SECONDS=18" # constant
    ],

    LIBS=["m", "stm32wb55xx"],
)

# no custom linker script given? give it the default one.
if not board.get("build.ldscript", ""):
    env.Append(
        LIBPATH=[
            os.path.join(FRAMEWORK_DIR, "system", "STM32WBxx", "LinkScripts")
        ]
    )
    env.Replace(LDSCRIPT_PATH="STM32WB55xx_FLASH.ld")

# process USB flags
cpp_defines = env.Flatten(env.get("CPPDEFINES", []))
board_is_usb_capable = "build.usb_product" in board
# if not already set externally and board is USB capable
if board_is_usb_capable and "USB_TYPE" not in cpp_defines:
    # default value: "Serial"
    usb_type = 1
    # "Serial"
    if "PIO_FRAMEWORK_ARDUINO_ENABLE_CDC" in cpp_defines:
        usb_type = 1
    # "Serial + Mass Storage"
    elif "PIO_FRAMEWORK_ARDUINO_ENABLE_CDC_WITH_MSC" in cpp_defines:
        usb_type = 2
    # "No USB"
    elif "PIO_FRAMEWORK_ARDUINO_NO_USB" in cpp_defines:
        usb_type = 0
    env.Append(CPPDEFINES=[("USB_TYPE", usb_type)])

# process storage type if not given externally:
if "STORAGE_TYPE" not in cpp_defines:
    # default value: "None"
    storage_type = 0
    # "None"
    if "PIO_FRAMEWORK_ARDUINO_STORAGE_TYPE_NONE" in cpp_defines:
        storage_type = 0
    # "SFLASH"
    elif "PIO_FRAMEWORK_ARDUINO_STORAGE_TYPE_SFLASH" in cpp_defines:
        storage_type = 1
    # "SDCARD"
    elif "PIO_FRAMEWORK_ARDUINO_STORAGE_TYPE_SDCARD" in cpp_defines:
        storage_type = 2
    env.Append(CPPDEFINES=[("STORAGE_TYPE", storage_type)])

if board_is_usb_capable:
    env.Append(
        CPPDEFINES=[
            ("USB_VID", board.get("build.hwids")[0][0]),
            ("USB_PID", board.get("build.hwids")[0][1]),
            # default DID is 0x0100 unless given in board
            # right now every board has the same DID
            ("USB_DID", board.get("build.usb_did", "0x0100")),
            ("USB_PRODUCT", '\\"%s\\"' %
             board.get("build.usb_product", "").replace('"', "")),
            ("USB_MANUFACTURER", '\\"%s\\"' %
             board.get("vendor", "").replace('"', ""))
        ]
    )

# Flags for Cortex-M4 FPU
env.Prepend(
    CCFLAGS=[
        "-mfloat-abi=hard",
        "-mfpu=fpv4-sp-d16"
    ],

    LINKFLAGS=[
        "-mfloat-abi=hard",
        "-mfpu=fpv4-sp-d16"
    ],

    LIBS=["arm_cortexM4lf_math"]
)

#
# Target: Build Core Library
#

libs = []

if "build.variant" in board:
    env.Append(
        CPPPATH=[os.path.join(variants_dir, board.get("build.variant"))]
    )
    # explicitly use BuildSources so that weak functions are linked correctly with object files instead of archives
    env.BuildSources(
        os.path.join("$BUILD_DIR", "FrameworkArduinoVariant"),
        os.path.join(variants_dir, board.get("build.variant"))
    )

libs.append(env.BuildLibrary(
    os.path.join("$BUILD_DIR", "FrameworkArduino"),
    os.path.join(FRAMEWORK_DIR, "cores", BUILD_CORE)
))

env.Prepend(LIBS=libs)
