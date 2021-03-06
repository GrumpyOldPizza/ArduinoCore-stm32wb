/*
 * Copyright (c) 2016-2020 Thomas Roell.  All rights reserved.
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

#if !defined(_DOSFS_SFLASH_H)
#define _DOSFS_SFLASH_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "dosfs_device.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DOSFS_SFLASH_INFO_HEAD_MAGIC               0
#define DOSFS_SFLASH_INFO_HEAD_DATA_START          1
#define DOSFS_SFLASH_INFO_HEAD_DATA_SIZE           2
#define DOSFS_SFLASH_INFO_HEAD_ERASE_COUNT         3
#define DOSFS_SFLASH_INFO_HEAD_OFFSET              0
#define DOSFS_SFLASH_INFO_HEAD_COUNT               4
#define DOSFS_SFLASH_INFO_HEAD_SIZE                16

#define DOSFS_SFLASH_INFO_TAIL_MAGIC               0
#define DOSFS_SFLASH_INFO_TAIL_OFFSET              508
#define DOSFS_SFLASH_INFO_TAIL_COUNT               1
#define DOSFS_SFLASH_INFO_TAIL_SIZE                4

#define DOSFS_SFLASH_INFO_ERASE_COUNT_OFFSET       12
#define DOSFS_SFLASH_INFO_ERASE_COUNT_SIZE         4
#define DOSFS_SFLASH_INFO_RECLAIM_OFFSET           16
#define DOSFS_SFLASH_INFO_RECLAIM_SIZE             16
  
#define DOSFS_SFLASH_INFO_DIR_OFFSET               124
#define DOSFS_SFLASH_INFO_DIR_SIZE                 384
  
#define DOSFS_SFLASH_MAGIC_HEAD                    0x5346544c     /* "SFTL" */
#define DOSFS_SFLASH_MAGIC_TAIL                    0x45563033     /* "EV03" */

#define DOSFS_SFLASH_DATA_SIZE                     0x02000000
#define DOSFS_SFLASH_BLOCK_SIZE                    0x00010000
#define DOSFS_SFLASH_PAGE_SIZE                     0x00000100

extern void dosfs_sflash_initialize(void);

#ifdef __cplusplus
}
#endif

#endif /*_DOSFS_SFLASH_H */
