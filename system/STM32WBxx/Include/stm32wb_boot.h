/*
 * Copyright (c) 2022-2024 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#if !defined(_STM32WB_BOOT_H)
#define _STM32WB_BOOT_H

#include "armv7m.h"

#ifdef __cplusplus
extern "C" {
#endif

// extern const uint32_t stm32wb_boot_vectors[];

typedef struct _stm32wb_boot_vectors_t {
    uint32_t                      reserved_0[8];
    uint32_t                      magic;             // TAG, OPTION, IDCODE
    uint32_t                      base;              // 0x08000000
    uint32_t                      size;              // 16384
    uint32_t                      reserved_1[2];
    uint32_t                      offset;            // offset to boot_info
    uint32_t                      reserved_2[2];
} stm32wb_boot_vectors_t;

typedef struct _stm32wb_boot_ecc256_key_t {
    uint32_t                      x[8];
    uint32_t                      y[8];
} stm32wb_boot_ecc256_key_t;

 typedef struct _stm32wb_boot_rsa2048_key_t {
    uint32_t                      n[64];
    uint32_t                      r2[64];   /* R^2 as little endian array */
    uint32_t                      u;        /* -1 / n mod 2^32 */
} stm32wb_boot_rsa2048_key_t;
  
#define STM32WB_BOOT_TAG_MASK                        0xffff0000
#define STM32WB_BOOT_TAG_SHIFT                       16
#define STM32WB_BOOT_TAG_BOOT                        0x54560000 // 'V', 'T'
#define STM32WB_BOOT_TYPE_MASK                       0x00008000
#define STM32WB_BOOT_TYPE_SHIFT                      15
#define STM32WB_BOOT_TYPE_BOOT                       0x00000000
#define STM32WB_BOOT_OPTION_MASK                     0x00007000
#define STM32WB_BOOT_OPTION_SHIFT                    12
#define STM32WB_BOOT_OPTION_PROTECTED                0x00004000
#define STM32WB_BOOT_OPTION_SIGNATURE_MASK           0x00003000
#define STM32WB_BOOT_OPTION_SIGNATURE_NONE           0x00000000
#define STM32WB_BOOT_OPTION_SIGNATURE_ECC256         0x00001000
#define STM32WB_BOOT_OPTION_SIGNATURE_RSA2048        0x00002000
#define STM32WB_BOOT_IDCODE_MASK                     0x00000fff
#define STM32WB_BOOT_IDCODE_SHIFT                    0
#define STM32WB_BOOT_IDCODE_STM32WB55                0x00000495

#define STM32WB_BOOT_BASE                            0x08000000
#define STM32WB_BOOT_SIZE                            0x00004000

#define STM32WB_BOOT_AES128_KEY                      0x08003fb0
#define STM32WB_BOOT_HASH                            0x08003fe0

typedef struct _stm32wb_boot_version_t {
    uint8_t                       major;
    uint8_t                       minor;
    uint16_t                      revision;
} stm32wb_boot_version_t;
  
typedef struct _stm32wb_boot_info_t {
    uint8_t                       uuid[16];
    stm32wb_boot_version_t        version;
    uint32_t                      application_base;
    uint32_t                      application_limit;
    uint32_t                      fwu_base;
    uint32_t                      fwu_limit;
    uint32_t                      fwu_status;
    uint32_t                      wireless_base;
    union {
        uint32_t                      ecc256_key;
        uint32_t                      rsa2048_key;
    };
    uint32_t                      options;
    uint32_t                      hseclk;
    uint16_t                      lseclk;
    struct {
        uint8_t                       status;        /* STATUS        */
        uint8_t                       boost;         /* BOOST         */
        uint8_t                       dfu;           /* ALT_BOOT      */
        uint8_t                       usb_vbus;      /* USB_VBUS      */
        uint16_t                      uart_rx;       /* UART_RX       */
        uint16_t                      uart_tx;       /* UART_TX       */
        uint16_t                      sflash_cs;     /* SFLASH_CS     */
        uint16_t                      sflash_clk;    /* SFLASH_CLK    */
        uint16_t                      sflash_mosi;   /* SFLASH_IO0    */
        uint16_t                      sflash_miso;   /* SFLASH_IO1    */
        uint16_t                      sflash_wp;     /* SFLASH_IO2    */
        uint16_t                      sflash_hold;   /* SFLASH_IO3    */
        uint16_t                      sflash_enable; /* SFLASH_ENABLE */
    }                             pins;
} stm32wb_boot_info_t;

#define STM32WB_BOOT_VERSION { .major = 1, .minor = 0, .revision = 0 }

typedef struct _stm32wb_application_vectors_t {
    uint32_t                      reserved_0[8];
    uint32_t                      magic;             // TAG, OPTION, IDCODE
    uint32_t                      base;              // 0x08004000
    uint32_t                      size;              // application size (signature not included)
    uint32_t                      reserved_1[2];
    uint32_t                      offset;            // offset to system_info
    uint32_t                      reserved_2[2];
} stm32wb_application_vectors_t;

#define STM32WB_APPLICATION_TAG_MASK                 0xffff0000
#define STM32WB_APPLICATION_TAG_SHIFT                16
#define STM32WB_APPLICATION_TAG_APPLICATION          0x54560000 // 'V', 'T'
#define STM32WB_APPLICATION_TYPE_MASK                0x00008000
#define STM32WB_APPLICATION_TYPE_SHIFT               15
#define STM32WB_APPLICATION_TYPE_APPLICATION         0x00008000
#define STM32WB_APPLICATION_OPTION_MASK              0x00007000
#define STM32WB_APPLICATION_OPTION_SHIFT             12
#define STM32WB_APPLICATION_OPTION_PROTECTED         0x00004000
#define STM32WB_APPLICATION_OPTION_SIGNATURE_MASK    0x00003000
#define STM32WB_APPLICATION_OPTION_SIGNATURE_NONE    0x00000000
#define STM32WB_APPLICATION_OPTION_SIGNATURE_ECC256  0x00001000
#define STM32WB_APPLICATION_OPTION_SIGNATURE_RSA2048 0x00002000
#define STM32WB_APPLICATION_IDCODE_MASK              0x00000fff
#define STM32WB_APPLICATION_IDCODE_SHIFT             0
#define STM32WB_APPLICATION_IDCODE_STM32WB55         0x00000495

#define STM32WB_APPLICATION_BASE                     0x08004000

#define STM32WB_APPLICATION_SIGNATURE_SIZE_NONE      (0)
#define STM32WB_APPLICATION_SIGNATURE_SIZE_ECC256    (64)
#define STM32WB_APPLICATION_SIGNATURE_SIZE_RSA2048   (256)

#if 0
typedef struct _stm32wb_application_version_t {
    uint8_t                       major;
    uint8_t                       minor;
    uint16_t                      revision;
    uint32_t                      build;
} stm32wb_application_version_t;
  
typedef struct _stm32wb_application_info_t {
    uint8_t                       uuid[16];
    stm32wb_application_version_t version;
    uint32_t                      sequence;
    uint32_t                      epoch;
} stm32wb_application_info_t;

#define STM32WB_APPLICATION_UUID(_u15, _u14, _u13, _u12, _u11, _u10, _u9, _u8, _u7, _u6, _u5, _u4, _u3, _u2, _u1, _u0)        \
    const uint8_t __attribute__((used, section(".info.uuid"))) __application_uuid__[16] = {                                   \
        (uint8_t)(_u0),  (uint8_t)(_u1),  (uint8_t)(_u2),  (uint8_t)(_u3),                                                    \
        (uint8_t)(_u4),  (uint8_t)(_u5),  (uint8_t)(_u6),  (uint8_t)(_u7),                                                    \
        (uint8_t)(_u8),  (uint8_t)(_u9),  (uint8_t)(_u10), (uint8_t)(_u11),                                                   \
        (uint8_t)(_u12), (uint8_t)(_u13), (uint8_t)(_u14), (uint8_t)(_u15)                                                    \
    }

#define STM32WB_APPLICATION_VERSION(_major,_minor,_revision,_build)                                                           \
    const uint32_t __attribute__((used, section(".info.version"))) __application_version__[2] = {                             \
        ((((uint32_t)(_major) & 0xff) << 0) | (((uint32_t)(_minor) & 0xff) << 8) | (((uint32_t)(_revision) & 0xffff) << 16)), \
        (uint32_t)(_build)                                                                                                    \
    }
    
#define STM32WB_APPLICATION_SEQUENCE(_sequence)                                                                               \
    const uint32_t __attribute__((used, section(".info.sequence"))) __application_sequence__[1] = {                           \
        ((uint32_t)(_sequence))                                                                                               \
    }
  
#endif

typedef struct _stm32wb_application_version_t {
    uint8_t                       major;
    uint8_t                       minor;
    uint16_t                      revision;
} stm32wb_application_version_t;
  
typedef struct _stm32wb_application_info_t {
    uint8_t                       uuid[16];
    stm32wb_application_version_t version;
    uint32_t                      sequence;
    uint32_t                      epoch;
    uint32_t                      crc32;
} stm32wb_application_info_t;
  
#define STM32WB_APPLICATION_UUID(_u15, _u14, _u13, _u12, _u11, _u10, _u9, _u8, _u7, _u6, _u5, _u4, _u3, _u2, _u1, _u0)        \
    const uint8_t __attribute__((used, section(".info.uuid"))) __application_uuid__[16] = {                                   \
        (uint8_t)(_u0),  (uint8_t)(_u1),  (uint8_t)(_u2),  (uint8_t)(_u3),                                                    \
        (uint8_t)(_u4),  (uint8_t)(_u5),  (uint8_t)(_u6),  (uint8_t)(_u7),                                                    \
        (uint8_t)(_u8),  (uint8_t)(_u9),  (uint8_t)(_u10), (uint8_t)(_u11),                                                   \
        (uint8_t)(_u12), (uint8_t)(_u13), (uint8_t)(_u14), (uint8_t)(_u15)                                                    \
    }

#define STM32WB_APPLICATION_VERSION(_major,_minor,_revision)                                                                  \
    const uint32_t __attribute__((used, section(".info.version"))) __application_version__[1] = {                             \
        ((((uint32_t)(_major) & 0xff) << 0) | (((uint32_t)(_minor) & 0xff) << 8) | (((uint32_t)(_revision) & 0xffff) << 16)), \
    }
    
#define STM32WB_APPLICATION_SEQUENCE(_sequence)                                                                               \
    const uint32_t __attribute__((used, section(".info.sequence"))) __application_sequence__[1] = {                           \
        ((uint32_t)(_sequence))                                                                                               \
    }
    
typedef struct _stm32wb_fwu_prefix_t {
    uint32_t                      magic;             // TAG, OPTION, IDCODE
    uint32_t                      base;              // 0x08004000
    uint32_t                      size;              // application size (signature not included)
    uint32_t                      length;            // image length
    uint32_t                      nonce[3];          // image nonce
    uint32_t                      crc32;             // image crc32
} stm32wb_fwu_prefix_t;

#define STM32WB_FWU_TAG_MASK                         0xffff0000
#define STM32WB_FWU_TAG_SHIFT                        16
#define STM32WB_FWU_TAG_FWU                          0x55530000 // 'S', 'U'
#define STM32WB_FWU_TYPE_MASK                        0x00008000
#define STM32WB_FWU_TYPE_SHIFT                       15
#define STM32WB_FWU_TYPE_UNCOMPRESSED                0x00000000
#define STM32WB_FWU_TYPE_COMPRESSED                  0x00008000
#define STM32WB_FWU_OPTION_MASK                      0x00007000
#define STM32WB_FWU_OPTION_SHIFT                     12
#define STM32WB_FWU_OPTION_PROTECTED                 0x00004000
#define STM32WB_FWU_OPTION_SIGNATURE_MASK            0x00003000
#define STM32WB_FWU_OPTION_SIGNATURE_NONE            0x00000000
#define STM32WB_FWU_OPTION_SIGNATURE_ECC256          0x00001000
#define STM32WB_FWU_OPTION_SIGNATURE_RSA2048         0x00002000
#define STM32WB_FWU_IDCODE_MASK                      0x00000fff
#define STM32WB_FWU_IDCODE_SHIFT                     0
#define STM32WB_FWU_IDCODE_STM32WB55                 0x00000495

#define STM32WB_FWU_PREFIX_SIZE                      32

/* A WIRELESS stack update is stored in the APPLICATION/FWU area
 * with a 64 byte prefix, so that it neither conflicts with an
 * APPLICATION image or a FWU image.
 */
#define STM32WB_BOOT_WIRELESS_PREFIX_MAGIC           0x53570495
#define STM32WB_BOOT_WIRELESS_PREFIX_OFFSET_STATUS   16
#define STM32WB_BOOT_WIRELESS_PREFIX_OFFSET_MAGIC    56
#define STM32WB_BOOT_WIRELESS_PREFIX_OFFSET_SIZE     60
#define STM32WB_BOOT_WIRELESS_PREFIX_SIZE            64
  
#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_BOOT_H */
