/*
 * Copyright (c) 2019-2020 Thomas Roell.  All rights reserved.
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

#if !defined(_DOSFS_STORAGE_H)
#define _DOSFS_STORAGE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "dosfs_core.h"
  
typedef struct _dosfs_object_t dosfs_object_t;
  
typedef void (*dosfs_object_read_callback_t)(dosfs_object_t *object, uint32_t offset, uint8_t *data, uint32_t count);
typedef void (*dosfs_object_write_callback_t)(dosfs_object_t *object, uint32_t offset, const uint8_t *data, uint32_t count);
typedef void (*dosfs_object_delete_callback_t)(dosfs_object_t *object);

typedef struct _dosfs_object_t {
    const char                     *name;
    bool                           read_only;
    uint32_t                       size;
    dosfs_object_read_callback_t   read_callback;
    dosfs_object_write_callback_t  write_callback;
    dosfs_object_delete_callback_t delete_callback;
} dosfs_object_t;

typedef struct _dosfs_storage_t {
    dosfs_device_t               device;
    uint32_t                     serial;
    uint8_t                      write_protect;
    uint8_t                      label[11];
    uint8_t                      type;
    uint8_t                      spt;
    uint16_t                     hpc;
    uint32_t                     tot_sec;
    uint32_t                     fat_blkno;
    uint32_t                     fat_blkcnt;
    uint32_t                     root_blkno;
    uint32_t                     root_blkcnt;
    uint32_t                     data_blkno;
    uint32_t                     data_blkcnt;
    uint32_t                     object_mask;
    uint32_t                     object_count;
    dosfs_object_t               **object_table;
    dosfs_dir_t                  root_data[16];
    uint32_t                     cache_data[1024 / 4];
} dosfs_storage_t;

extern bool dosfs_storage_init(dosfs_storage_t *storage, bool write_protect, const char *label, uint32_t object_count, dosfs_object_t **object_table);
extern bool dosfs_storage_deinit(dosfs_storage_t *storage);
  
#ifdef __cplusplus
}
#endif

#endif /*_DOSFS_STORAGE_H */
