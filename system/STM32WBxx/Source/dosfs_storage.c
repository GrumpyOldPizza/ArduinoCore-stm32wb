/*
 * Copyright (c) 2014-2020 Thomas Roell.  All rights reserved.
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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "armv7m.h"
#include "stm32wb_rtc.h"
#include "stm32wb_usbd.h"
#include "stm32wb_usbd_msc.h"
#include "dosfs_core.h"
#include "dosfs_storage.h"

#define DOSFS_STORAGE_TYPE_FAT12              0
#define DOSFS_STORAGE_TYPE_FAT16              1

#define DOSFS_STORAGE_CLS_SIZE                32768

static inline unsigned int dosfs_name_ascii_upcase(unsigned int cc)
{
    unsigned int uc;

    if ((cc <= 0x1f) || (cc > 0x7f))
    {
        uc = 0x0000;
    }
    else
    {
        if ((cc >= 'a') && (cc <= 'z'))
        {
            uc = cc - ('a' - 'A');
        }
        else
        {
            uc = cc;
        }
    }

    return uc;
}

static const char *dosfs_name_cstring_to_label(const char *cstring, uint8_t *label)
{
    unsigned int cc, n, n_e;

    memset(label, ' ', 11);

    n = 0;
    n_e = 11;

    cc = *cstring++;

    if (cc == ' ')
    {
        cstring = NULL;
    }
    else
    {
        do
        {
            if (n == n_e)
            {
                cstring = NULL;
            }
            else
            {
                if ((cc <= 0x001f) ||
                    (cc >= 0x0080) ||
                    (cc == '"') ||
                    (cc == '*') ||
                    (cc == '+') ||
                    (cc == ',') ||
                    (cc == '.') ||
                    (cc == '/') ||
                    (cc == ':') ||
                    (cc == ';') ||
                    (cc == '<') ||
                    (cc == '=') ||
                    (cc == '>') ||
                    (cc == '?') ||
                    (cc == '[') ||
                    (cc == '\\') ||
                    (cc == ']') ||
                    (cc == '|'))
                {
                    cstring = NULL;
                }
                else
                {
                    label[n++] = dosfs_name_ascii_upcase(cc);
                            
                    cc = *cstring++;
                }
            }
        }
        while ((cstring != NULL) && (cc != '\0'));
    }
    
    return cstring;
}

static const char *dosfs_name_cstring_to_dosname(const char *cstring, uint8_t *dosname)
{
    unsigned int cc, n, n_e;

    memset(dosname, ' ', 11);

    n = 0;
    n_e = 8;

    cc = *cstring++;

    if (cc == ' ')
    {
        cstring = NULL;
    }
    else
    {
        /* A leading sequence of "." or ".." is handled varbatim.
         */
    
        if (cc == '.')
        {
            dosname[n++] = cc;

            cc = *cstring++;

            if (cc == '.')
            {
                dosname[n++] = cc;

                cc = *cstring++;
            }

            if ((cc != '/') && (cc != '\\') && (cc != '\0'))
            {
                cstring = NULL;
            }
        }

        while ((cstring != NULL) && (cc != '/') && (cc != '\\') && (cc != '\0'))
        {
            if (cc == '.')
            {
                if (n_e == 11)
                {
                    cstring = NULL;
                }
                else
                {
                    n = 8;
                    n_e = 11;
                        
                    cc = *cstring++;
                }
            }
            else
            {
                if (n == n_e)
                {
                    cstring = NULL;
                }
                else
                {
                    if ((cc <= 0x001f) ||
                        (cc >= 0x0080) ||
                        (cc == '"') ||
                        (cc == '*') ||
                        (cc == '+') ||
                        (cc == ',') ||
                        (cc == '/') ||
                        (cc == ':') ||
                        (cc == ';') ||
                        (cc == '<') ||
                        (cc == '=') ||
                        (cc == '>') ||
                        (cc == '?') ||
                        (cc == '[') ||
                        (cc == '\\') ||
                        (cc == ']') ||
                        (cc == '|'))
                    {
                        cstring = NULL;
                    }
                    else
                    {
                        dosname[n++] = dosfs_name_ascii_upcase(cc);
                            
                        cc = *cstring++;
                    }
                }
            }
        }

        if (n == 0)
        {
            cstring = NULL;
        }
    }

    return cstring;
}

#define Y2K_TO_GPS_OFFSET    630720000

static void dosfs_storage_reset(dosfs_storage_t *storage)
{
    uint32_t clsno, index;
    stm32wb_rtc_tod_t tod;
    uint32_t seconds, ticks, date, time;
    int32_t leap_seconds;
    dosfs_dir_t *dir;
    dosfs_object_t *object;

    stm32wb_rtc_time_read(&seconds, &ticks);

    leap_seconds = stm32wb_rtc_get_leap_seconds();

    seconds -= Y2K_TO_GPS_OFFSET;

    stm32wb_rtc_time_to_tod(seconds - leap_seconds, ticks, &tod);
    
    time = ((tod.seconds >> 1) | (tod.minutes << 5) | (tod.hours << 11));
    date = ((tod.day << 0) | (tod.month << 5) | ((tod.year + 20) << 9));

    memset(storage->root_data, 0, sizeof(storage->root_data));
    
    dir = &storage->root_data[0];
    
    memcpy(dir->dir_name, storage->label, sizeof(dir->dir_name));

    dir->dir_attr = DOSFS_DIR_ATTR_VOLUME_ID;
    dir->dir_nt_reserved = 0;
    dir->dir_crt_time_tenth = 0;
    dir->dir_crt_time = time;
    dir->dir_crt_date = date;
    dir->dir_acc_date = date;
    dir->dir_clsno_hi = 0;
    dir->dir_wrt_time = time;
    dir->dir_wrt_date = date;
    dir->dir_clsno_lo = 0;
    dir->dir_file_size = 0;
    dir++;

    for (clsno = 2, index = 0; index < storage->object_count; index++)
    {
        object = storage->object_table[index];
        
        dosfs_name_cstring_to_dosname(object->name, dir->dir_name);

        dir->dir_attr = object->read_only ? DOSFS_DIR_ATTR_READ_ONLY : 0;
        dir->dir_nt_reserved = 0;
        dir->dir_crt_time_tenth = 0;
        dir->dir_crt_time = time;
        dir->dir_crt_date = date;
        dir->dir_acc_date = date;
        dir->dir_clsno_hi = 0;
        dir->dir_wrt_time = time;
        dir->dir_wrt_date = date;
        dir->dir_clsno_lo = clsno;
        dir->dir_file_size = object->size;
        dir++;
        
        clsno += ((object->size + (DOSFS_STORAGE_CLS_SIZE-1)) / (DOSFS_STORAGE_CLS_SIZE-1));
    }

    storage->serial = seconds;
    storage->object_mask = 0xffffffff >> (32 - storage->object_count);
}

static void dosfs_storage_bpb_read(dosfs_storage_t *storage, uint32_t address, uint8_t *data)
{
    dosfs_boot_t *boot;

    // armv7m_rtt_printf("BPB %08x\n", address);
    
    boot = (dosfs_boot_t *)data;

    boot->bpb40.bs_jmp_boot[0]   = 0xeb;
    boot->bpb40.bs_jmp_boot[1]   = 0x00;
    boot->bpb40.bs_jmp_boot[2]   = 0x90;
    boot->bpb40.bs_oem_name[0]   = 'M';
    boot->bpb40.bs_oem_name[1]   = 'S';
    boot->bpb40.bs_oem_name[2]   = 'D';
    boot->bpb40.bs_oem_name[3]   = 'O';
    boot->bpb40.bs_oem_name[4]   = 'S';
    boot->bpb40.bs_oem_name[5]   = '5';
    boot->bpb40.bs_oem_name[6]   = '.';
    boot->bpb40.bs_oem_name[7]   = '0';
    boot->bpb40.bpb_byts_per_sec = DOSFS_HTOFS(DOSFS_BLK_SIZE);
    boot->bpb40.bpb_sec_per_clus = (DOSFS_STORAGE_CLS_SIZE / DOSFS_BLK_SIZE);
    boot->bpb40.bpb_rsvd_sec_cnt = DOSFS_HTOFS(storage->fat_blkno);
    boot->bpb40.bpb_num_fats     = 1;
    boot->bpb40.bpb_root_ent_cnt = DOSFS_HTOFS(16);
    boot->bpb40.bpb_tot_sec_16   = DOSFS_HTOFS((storage->tot_sec >= 65536) ? 0 : storage->tot_sec);
    boot->bpb40.bpb_media        = 0xf8;
    boot->bpb40.bpb_fat_sz_16    = DOSFS_HTOFS(storage->fat_blkcnt);
    boot->bpb40.bpb_sec_per_trk  = storage->spt;
    boot->bpb40.bpb_num_heads    = storage->hpc;
    boot->bpb40.bpb_hidd_sec_32  = DOSFS_HTOFL(0);
    boot->bpb40.bpb_tot_sec_32   = DOSFS_HTOFS((storage->tot_sec >= 65536) ? storage->tot_sec : 0);
    boot->bpb40.bs_drv_num       = 0x80;
    boot->bpb40.bs_nt_reserved   = 0x00;
    boot->bpb40.bs_boot_sig      = 0x29;
    boot->bpb40.bs_vol_id        = DOSFS_HTOFL(storage->serial);
    boot->bpb40.bs_trail_sig     = DOSFS_HTOFS(0xaa55);

    memcpy(boot->bpb40.bs_vol_lab, storage->label, sizeof(boot->bpb40.bs_vol_lab));

    if (storage->type == DOSFS_STORAGE_TYPE_FAT16)
    {
        memcpy(boot->bpb40.bs_fil_sys_type, "FAT16   ", sizeof(boot->bpb40.bs_fil_sys_type));
    }
    else
    {
        memcpy(boot->bpb40.bs_fil_sys_type, "FAT12   ", sizeof(boot->bpb40.bs_fil_sys_type));
    }
}

static void dosfs_storage_fat_read(dosfs_storage_t *storage, uint32_t address, uint8_t *data)
{
    uint8_t *fat;
    uint32_t clsno, clsno_e, index, offset, offset_l, offset_h, clsdata, clsdata_l, clsdata_h;
    dosfs_object_t *object;
    
    // armv7m_rtt_printf("FAT %08x\n", address);
    
    address = (address - storage->fat_blkno) * DOSFS_BLK_SIZE;

    fat = data - address;

    if (storage->type == DOSFS_STORAGE_TYPE_FAT16)
    {
        if (address == 0)
        {
            fat[0] = 0xf8;
            fat[1] = 0xff;
            fat[2] = 0xff;
            fat[3] = 0xff;
        }

        for (clsno = 2, index = 0; index < storage->object_count; index++)
        {
            object = storage->object_table[index];
        
            for (clsno_e = clsno + ((object->size + (DOSFS_STORAGE_CLS_SIZE-1)) / DOSFS_STORAGE_CLS_SIZE) - 1; clsno <= clsno_e; clsno++)
            {
                offset = clsno * 2;

                if (((offset+1) < address) || ((offset+0) >= (address + DOSFS_BLK_SIZE)))
                {
                    continue;
                }
            
                clsdata = (clsno == clsno_e) ? 0xffff : (clsno+1);
            
                fat[offset+0] = clsdata;
                fat[offset+1] = clsdata >> 8;
            }
        }
    }
    else
    {
        if (address == 0)
        {
            fat[0] = 0xf8;
            fat[1] = 0xff;
            fat[2] = 0xff;
        }

        for (clsno = 2, index = 0; index < storage->object_count; index++)
        {
            object = storage->object_table[index];
        
            for (clsno_e = clsno + ((object->size + (DOSFS_STORAGE_CLS_SIZE-1)) / DOSFS_STORAGE_CLS_SIZE) - 1; clsno <= clsno_e; clsno++)
            {
                offset = clsno + (clsno >> 1);

                offset_l = offset + 0;
                offset_h = offset + 1;

                if ((offset_h < address) || (offset_l >= (address + DOSFS_BLK_SIZE)))
                {
                    continue;
                }
            
                clsdata = (clsno == clsno_e) ? 0xfff : (clsno+1);
            
                if (clsno & 1)
                {
                    clsdata_l = (clsdata << 4);
                    clsdata_h = (clsdata >> 4);
                }
                else
                {
                    clsdata_l = (clsdata << 0);
                    clsdata_h = (clsdata >> 8);
                }

                if ((offset_l >= address) && (offset_l < (address + DOSFS_BLK_SIZE)))
                {
                    fat[offset_l] |= clsdata_l;
                }

                if ((offset_h >= address) && (offset_h < (address + DOSFS_BLK_SIZE)))
                {
                    fat[offset_h] |= clsdata_h;
                }
            }
        }
    }
}

static void dosfs_storage_root_read(dosfs_storage_t *storage, uint32_t address, uint8_t *data)
{
    // armv7m_rtt_printf("ROOT %08x\n", address);
  
    memcpy(data, storage->root_data, DOSFS_BLK_SIZE);
}

static void dosfs_storage_root_write(dosfs_storage_t *storage, uint32_t address, const uint8_t *data)
{
    const dosfs_dir_t *n_dir;
    dosfs_object_t *object;
    dosfs_dir_t *o_dir;
    uint32_t index;

    // armv7m_rtt_printf("ROOT %08x\n", address);
    
    o_dir = storage->root_data + 1;
    n_dir = (const dosfs_dir_t *)data + 1;

    for (index = 0; index < storage->object_count; index++, o_dir++, n_dir++)
    {
        object = storage->object_table[index];
        
        if (storage->object_mask & (1 << index))
        {
            if ((n_dir->dir_name[0] == 0xe5) || (n_dir->dir_name[0] == 0x00))
            {
                storage->object_mask &= ~(1 << index);
                
                // armv7m_rtt_printf("DELETED %d\n", index);

                if (object->delete_callback)
                {
                    (*object->delete_callback)(object);
                }
                
                continue;
            }

            if (memcmp(o_dir->dir_name, n_dir->dir_name, sizeof(o_dir->dir_name)) ||
                (o_dir->dir_clsno_lo != n_dir->dir_clsno_lo) ||
                (o_dir->dir_file_size != n_dir->dir_file_size))
            {
                storage->object_mask &= ~(1 << index);
                
                // armv7m_rtt_printf("UNMAP %d (%d %d -> %d %d)\n", index, o_dir->dir_file_size, o_dir->dir_clsno_lo, n_dir->dir_file_size, n_dir->dir_clsno_lo);

                continue;
            }
        }
    }

    memcpy(storage->root_data, data, DOSFS_BLK_SIZE);
}

static int dosfs_storage_start(void *context, uint8_t *p_media, uint8_t *p_write_protected, uint32_t *p_block_count, uint32_t *p_au_size, uint32_t *p_serial)
{
    dosfs_storage_t *storage = (dosfs_storage_t*)context;
    
    *p_media = DOSFS_MEDIA_SFLASH;
    *p_write_protected = storage->write_protect;
    *p_block_count = storage->tot_sec;
    *p_au_size = 1;
    *p_serial = 0;
    
    return F_NO_ERROR;
}

static int dosfs_storage_stop(void *context, bool eject)
{
    dosfs_storage_t *storage = (dosfs_storage_t*)context;

    if (eject)
    {
        dosfs_storage_reset(storage);
    }
    
    return F_NO_ERROR;
}

static int dosfs_storage_format(void *context, uint32_t size)
{
    return F_NO_ERROR;
}

static int dosfs_storage_erase(void *context, uint32_t address, uint32_t length)
{
    return F_NO_ERROR;
}

static int dosfs_storage_discard(void *context, uint32_t address, uint32_t length)
{
    return F_NO_ERROR;
}

static int dosfs_storage_read(void *context, uint32_t address, uint8_t *data, uint32_t length)
{
    dosfs_storage_t *storage = (dosfs_storage_t*)context;
    dosfs_object_t *object;
    uint32_t index, offset, size, blkno, blkno_e;
    int status = F_NO_ERROR;

    // armv7m_rtt_printf("READ %08x/%d\n", address, length);
    
    while (length--)
    {
        memset(&data[0], 0x00, DOSFS_BLK_SIZE);

        if (address == 0)
        {
            dosfs_storage_bpb_read(storage, address, data);
            
            break;
        }
        
        if ((address >= storage->fat_blkno) && (address < (storage->fat_blkno + storage->fat_blkcnt)))
        {
            dosfs_storage_fat_read(storage, address, data);
            
            break;
        }
        
        if ((address >= storage->root_blkno) && (address < (storage->root_blkno + storage->root_blkcnt)))
        {
            dosfs_storage_root_read(storage, address, data);
            
            break;
        }

        if (address >= storage->data_blkno)
        {
            for (blkno = storage->data_blkno, index = 0; index < storage->object_count; index++)
            {
                object = storage->object_table[index];
                
                blkno_e = blkno + ((object->size + (DOSFS_BLK_SIZE-1)) >> DOSFS_BLK_SHIFT);
                
                if ((address >= blkno) && (address < blkno_e))
                {
                    if (storage->object_mask & (1 << index))
                    {
                        offset = (address - blkno) * DOSFS_BLK_SIZE;
                        size = DOSFS_BLK_SIZE;
                        
                        if (size > (object->size - offset))
                        {
                            size = object->size - offset;
                        }
                        
                        if (size)
                        {
                            // armv7m_rtt_printf("DATA %08x (%08x,%d)\n", address, offset, size);
                            
                            if (object->read_callback)
                            {
                                (*object->read_callback)(object, offset, data, size);
                            }
                            else   
                            {
                                status = F_ERR_READ;
                            }
                        }
                    }
                    
                    break;
                }

                blkno += (((object->size + (DOSFS_STORAGE_CLS_SIZE-1)) & ~(DOSFS_STORAGE_CLS_SIZE-1)) >> DOSFS_BLK_SHIFT);
            }
        }

        data += DOSFS_BLK_SIZE;

        address++;
    }
    
    return status;
}

static int dosfs_storage_write(void *context, uint32_t address, const uint8_t *data, uint32_t length, bool sync)
{
    dosfs_storage_t *storage = (dosfs_storage_t*)context;
    dosfs_object_t *object;
    uint32_t index, offset, size, blkno, blkno_e;
    int status = F_NO_ERROR;

    // armv7m_rtt_printf("WRITE %08x/%d\n", address, length);
    
    while (length--)
    {
        if ((address >= storage->root_blkno) && (address < (storage->root_blkno + storage->root_blkcnt)))
        {
            dosfs_storage_root_write(storage, address, data);
            
            break;
        }

        if (address >= storage->data_blkno)
        {
            for (blkno = storage->data_blkno, index = 0; index < storage->object_count; index++)
            {
                object = storage->object_table[index];
                
                blkno_e = blkno + ((object->size + (DOSFS_BLK_SIZE-1)) >> DOSFS_BLK_SHIFT);
                
                if ((address >= blkno) && (address < blkno_e))
                {
                    if ((storage->object_mask & (1 << index)) && !object->read_only)
                    {
                        offset = (address - blkno) * DOSFS_BLK_SIZE;
                        size = DOSFS_BLK_SIZE;
                        
                        if (size > (object->size - offset))
                        {
                            size = object->size - offset;
                        }
                        
                        if (size)
                        {
                            // armv7m_rtt_printf("DATA %08x (%08x,%d)\n", address, offset, size);

                            if (object->write_callback)
                            {
                                (*object->write_callback)(object, offset, data, size);
                            }
                            else
                            {
                                status = F_ERR_WRITE;
                            }
                        }
                    }
                    
                    break;
                }

                blkno += (((object->size + (DOSFS_STORAGE_CLS_SIZE-1)) & ~(DOSFS_STORAGE_CLS_SIZE-1)) >> DOSFS_BLK_SHIFT);
            }
        }

        
        data += DOSFS_BLK_SIZE;

        address++;
    }
    
    return status;
}

static int dosfs_storage_sync(void *context)
{
    return F_NO_ERROR;
}

static const dosfs_device_interface_t dosfs_storage_interface = {
    dosfs_storage_start,
    dosfs_storage_stop,
    dosfs_storage_format,
    dosfs_storage_erase,
    dosfs_storage_discard,
    dosfs_storage_read,
    dosfs_storage_write,
    dosfs_storage_sync,
};

bool dosfs_storage_init(dosfs_storage_t *storage, bool write_protect, const char *label, uint32_t object_count, dosfs_object_t **object_table)
{
    uint32_t clscnt, hpc, spt, index;
    uint32_t type, fat_blkno, fat_blkcnt, root_blkno, root_blkcnt, data_blkno, data_blkcnt, tot_sec;
    
    if (object_count > 15)
    {
        return false;
    }
    
    for (clscnt = 0, index = 0; index < object_count; index++)
    {
        clscnt += ((object_table[index]->size + (DOSFS_STORAGE_CLS_SIZE-1)) / DOSFS_STORAGE_CLS_SIZE);
    }

    if (clscnt >= 4085)
    {
        type = DOSFS_STORAGE_TYPE_FAT16;

        fat_blkcnt  = (((clscnt + 2) * 2) + (DOSFS_BLK_SIZE -1)) >> DOSFS_BLK_SHIFT;
    }
    else
    {
        type = DOSFS_STORAGE_TYPE_FAT12;
        
        fat_blkcnt  = (((((clscnt + 2) * 3) + 1) / 2) + (DOSFS_BLK_SIZE -1)) >> DOSFS_BLK_SHIFT;
    }

    root_blkcnt = 1;
    data_blkcnt = clscnt * (DOSFS_STORAGE_CLS_SIZE / DOSFS_BLK_SIZE);

    data_blkno  = (1 + fat_blkcnt + root_blkcnt + ((DOSFS_STORAGE_CLS_SIZE / DOSFS_BLK_SIZE)-1)) & ~((DOSFS_STORAGE_CLS_SIZE / DOSFS_BLK_SIZE)-1);
    root_blkno  = data_blkno - root_blkcnt;
    fat_blkno   = root_blkno - fat_blkcnt;
    
    tot_sec = data_blkno + data_blkcnt;

    if (tot_sec <= (1024 * 2 * 16))
    {
        hpc = 2;
        spt = 16;
    }
    else
    {
        hpc = 2;
        spt = 32;
        
        while (hpc < 128)
        {
            if (tot_sec <= (1024 * hpc * spt))
            {
                break;
            }
            
            hpc <<= 1;
        }
    }

    storage->device.lock = 0;
    storage->device.interface = &dosfs_storage_interface;
    storage->device.context = storage;
    storage->device.cache = (uint8_t*)&storage->cache_data[0];

    storage->write_protect = write_protect;
    storage->type          = type;
    storage->spt           = spt;
    storage->hpc           = hpc;
    storage->tot_sec       = tot_sec;
    storage->fat_blkno     = fat_blkno;
    storage->fat_blkcnt    = fat_blkcnt;
    storage->root_blkno    = root_blkno;
    storage->root_blkcnt   = root_blkcnt;
    storage->data_blkno    = data_blkno;
    storage->data_blkcnt   = data_blkcnt;
    storage->object_count  = object_count;
    storage->object_table  = object_table;
    
    if (!label || !dosfs_name_cstring_to_label(label, storage->label))
    {
        memcpy(storage->label, "NO NAME    ", sizeof(storage->label));
    }

    dosfs_storage_reset(storage);

    return true;
}

