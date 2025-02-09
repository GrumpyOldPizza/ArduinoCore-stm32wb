/*
 * Copyright (c) 2016-2021 Thomas Roell.  All rights reserved.
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

#include "armv7m.h"
#include "stm32wb_sflash.h"
#include "dosfs_sflash.h"

#define DOSFS_SFLASH_DIR_ENTRY_SIZE               3
#define DOSFS_SFLASH_DIR_ENTRY_MASK               0xffffff
#define DOSFS_SFLASH_DIR_ENTRY_NOT_WRITTEN        0xffffff
#define DOSFS_SFLASH_DIR_TAG_MASK                 0xff0000
#define DOSFS_SFLASH_DIR_TAG_OFFSET               2
#define DOSFS_SFLASH_DIR_TAG_SIZE                 1
#define DOSFS_SFLASH_DIR_TAG_SHIFT                16
#define DOSFS_SFLASH_DIR_TAG_NOT_WRITTEN          0xff0000
#define DOSFS_SFLASH_DIR_TAG_TYPE_MASK            0xf00000
#define DOSFS_SFLASH_DIR_TAG_TYPE_NONE            0xf00000
#define DOSFS_SFLASH_DIR_TAG_TYPE_RECLAIM         0x700000
#define DOSFS_SFLASH_DIR_TAG_TYPE_INFO            0x300000
#define DOSFS_SFLASH_DIR_TAG_TYPE_DATA            0x200000
#define DOSFS_SFLASH_DIR_TAG_TYPE_XLATE_SECONDARY 0x100000
#define DOSFS_SFLASH_DIR_TAG_TYPE_XLATE_PRIMARY   0x000000
#define DOSFS_SFLASH_DIR_TAG_STATE_MASK           0x0f0000
#define DOSFS_SFLASH_DIR_TAG_STATE_NOT_COMMITTED  0x080000
#define DOSFS_SFLASH_DIR_TAG_STATE_NOT_WRITTEN    0x040000
#define DOSFS_SFLASH_DIR_TAG_STATE_NOT_TOUCHED    0x020000
#define DOSFS_SFLASH_DIR_TAG_STATE_NOT_FINISHED   0x010000
#define DOSFS_SFLASH_DIR_DATA_MASK                0x00ffff
#define DOSFS_SFLASH_DIR_DATA_SHIFT               0
#define DOSFS_SFLASH_DIR_DATA_SIZE                2

#define DOSFS_SFLASH_SECTOR_SIZE                  512
#define DOSFS_SFLASH_ADDRESS_UNDEFINED            0xffff

#define DOSFS_SFLASH_CACHE_TAG_MASK               0xfff0
#define DOSFS_SFLASH_CACHE_TAG_SHIFT              4
#define DOSFS_SFLASH_CACHE_BLOCK_MASK             0x0070
#define DOSFS_SFLASH_CACHE_BLOCK_SHIFT            4
#define DOSFS_SFLASH_CACHE_BLOCK_COUNT            8
#define DOSFS_SFLASH_CACHE_INDEX_MASK             0x000f
#define DOSFS_SFLASH_CACHE_INDEX_SHIFT            0
#define DOSFS_SFLASH_CACHE_INDEX_COUNT            16

#define DOSFS_SFLASH_CACHE_TAG_UNDEFINED          0xffff

#define DOSFS_SFLASH_XLATE_OFFSET                 64
#define DOSFS_SFLASH_XLATE_SLOT_COUNT             (DOSFS_SFLASH_DATA_SIZE / (DOSFS_SFLASH_SECTOR_SIZE * (DOSFS_SFLASH_SECTOR_SIZE / sizeof(uint16_t))))
#define DOSFS_SFLASH_XLATE_SLOT_MASK              0xff00
#define DOSFS_SFLASH_XLATE_SLOT_SHIFT             8
#define DOSFS_SFLASH_XLATE_INDEX_MASK             0x00ff
#define DOSFS_SFLASH_XLATE_INDEX_SHIFT            0
#define DOSFS_SFLASH_XLATE_INDEX_COUNT            256
#define DOSFS_SFLASH_XLATE_SLOT_UNDEFINED         0xffff
#define DOSFS_SFLASH_XLATE_ENTRY_UNDEFINED        0xffff

#define DOSFS_SFLASH_ENTRY_BLOCK_MASK             0xff80
#define DOSFS_SFLASH_ENTRY_BLOCK_SHIFT            7
#define DOSFS_SFLASH_ENTRY_SECTOR_MASK            0x007f
#define DOSFS_SFLASH_ENTRY_SECTOR_SHIFT           0
#define DOSFS_SFLASH_ENTRY_UNDEFINED              0xffff
#define DOSFS_SFLASH_ENTRY_DISCARDED              0x0000

#define DOSFS_SFLASH_BLOCK_COUNT                  (DOSFS_SFLASH_DATA_SIZE / DOSFS_SFLASH_BLOCK_SIZE)
#define DOSFS_SFLASH_BLOCK_UNDEFINED              (DOSFS_SFLASH_BLOCK_COUNT-1)

#define DOSFS_SFLASH_DATA_OFFSET_UNDEFINED        (DOSFS_SFLASH_BLOCK_UNDEFINED * DOSFS_SFLASH_BLOCK_SIZE)
#define DOSFS_SFLASH_DATA_SIZE_MIN                (4 * DOSFS_SFLASH_BLOCK_SIZE)
#define DOSFS_SFLASH_RECLAIM_THRESHOLD            2
#define DOSFS_SFLASH_ERASE_THRESHOLD              256

#define DOSFS_SFLASH_STATE_NONE                   0
#define DOSFS_SFLASH_STATE_NOT_READY              1
#define DOSFS_SFLASH_STATE_READY                  2

//#define DOSFS_SFLASH_ASSERT(_condition)           { if (!(_condition)) { __BKPT(); } }
#define DOSFS_SFLASH_ASSERT(_condition)           /**/


/* [16M] 128 + 256 + 512 + 256 == 1152
 * [32M] 128 + 512 + 512 + 768 == 1920
 *
 * For >= 16M uses even/odd block lookup: block_table[block] * 2 + (block & 1). Than requires 2 reclaim blocks to have the even/odd mapping.
 */
typedef struct _dosfs_sflash_device_t {
    uint8_t  state;
    uint16_t block_count;
    uint16_t sector_count;
    uint32_t reclaim_offset; 
    uint16_t erase_count;
    uint16_t alloc_block;                                                        /* current block                                                    */
    uint16_t alloc_count;                                                        /* overall current free sector count                                        */
    uint16_t alloc_index;                                                        /* current offset (next to be looked at)                            */
    uint32_t alloc_mask[DOSFS_SFLASH_BLOCK_SIZE / (DOSFS_SFLASH_SECTOR_SIZE * 32)];
    uint16_t xlate_slot;                                                         /* sector slot the xlate secondary is associated with               */
    uint16_t xlate_entry;                                                        /* logical addresses the xlate secondry                             */
    uint16_t xlate_direct[DOSFS_SFLASH_XLATE_OFFSET];                            /* direct mapped sectors                                            */
    uint16_t xlate_table[DOSFS_SFLASH_XLATE_SLOT_COUNT];                         /* logical addresses of the primary xlate sectors                   */
    uint16_t cache_tag[2][DOSFS_SFLASH_CACHE_BLOCK_COUNT];
    uint16_t cache_data[2][DOSFS_SFLASH_CACHE_BLOCK_COUNT][DOSFS_SFLASH_CACHE_INDEX_COUNT];
#if (DOSFS_SFLASH_BLOCK_COUNT <= 256)
    uint8_t  block_table[DOSFS_SFLASH_BLOCK_COUNT];                              /* block remapping table (logical to offset mapping)                */
#else /* (DOSFS_SFLASH_BLOCK_COUNT <= 256) */
    uint8_t  block_table[DOSFS_SFLASH_BLOCK_COUNT + ((DOSFS_SFLASH_BLOCK_COUNT + 7) / 8)];
#endif /* (DOSFS_SFLASH_BLOCK_COUNT <= 256) */

    const stm32wb_sflash_interface_t *interface;
    const stm32wb_sflash_info_t *info;
    uint32_t serial;
    uint32_t data_start;
    uint32_t data_size;
} dosfs_sflash_device_t;

static dosfs_sflash_device_t dosfs_sflash_device;

static void dosfs_sflash_nor_acquire()
{
    const stm32wb_sflash_interface_t *interface = dosfs_sflash_device.interface;
    
    (*interface->acquire)();
}

static inline void dosfs_sflash_nor_release()
{
    const stm32wb_sflash_interface_t *interface = dosfs_sflash_device.interface;

    (*interface->release)();
}

static void dosfs_sflash_nor_erase(uint32_t offset)
{
    const stm32wb_sflash_interface_t *interface = dosfs_sflash_device.interface;

    DOSFS_SFLASH_ASSERT((offset + DOSFS_SFLASH_BLOCK_SIZE) <= dosfs_sflash_device.data_size);
    
    (*interface->erase)(dosfs_sflash_device.data_start + offset, NULL);

    while ((*interface->busy)()) { };
}

static void dosfs_sflash_nor_program(uint32_t offset, const uint8_t *data, uint32_t size)
{
    const stm32wb_sflash_interface_t *interface = dosfs_sflash_device.interface;

    DOSFS_SFLASH_ASSERT((offset + size) <= dosfs_sflash_device.data_size);

    (*interface->program)(dosfs_sflash_device.data_start + offset, data, size, NULL);

    while ((*interface->busy)()) { };
}

static void dosfs_sflash_nor_read(uint32_t offset, uint8_t *data, uint32_t size)
{
    const stm32wb_sflash_interface_t *interface = dosfs_sflash_device.interface;

    DOSFS_SFLASH_ASSERT((offset + size) <= dosfs_sflash_device.data_size);

    (*interface->read)(dosfs_sflash_device.data_start + offset, data, size);
}

static const uint8_t dosfs_sflash_ftl_zeros[256] = {
    8, 7, 7, 6, 7, 6, 6, 5, 7, 6, 6, 5, 6, 5, 5, 4, 
    7, 6, 6, 5, 6, 5, 5, 4, 6, 5, 5, 4, 5, 4, 4, 3, 
    7, 6, 6, 5, 6, 5, 5, 4, 6, 5, 5, 4, 5, 4, 4, 3, 
    6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2, 
    7, 6, 6, 5, 6, 5, 5, 4, 6, 5, 5, 4, 5, 4, 4, 3, 
    6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2, 
    6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2, 
    5, 4, 4, 3, 4, 3, 3, 2, 4, 3, 3, 2, 3, 2, 2, 1, 
    7, 6, 6, 5, 6, 5, 5, 4, 6, 5, 5, 4, 5, 4, 4, 3, 
    6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2, 
    6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2, 
    5, 4, 4, 3, 4, 3, 3, 2, 4, 3, 3, 2, 3, 2, 2, 1, 
    6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2, 
    5, 4, 4, 3, 4, 3, 3, 2, 4, 3, 3, 2, 3, 2, 2, 1, 
    5, 4, 4, 3, 4, 3, 3, 2, 4, 3, 3, 2, 3, 2, 2, 1, 
    4, 3, 3, 2, 3, 2, 2, 1, 3, 2, 2, 1, 2, 1, 1, 0, 
};

static uint32_t dosfs_sflash_block_offset(uint32_t block)
{
    uint32_t offset;

    DOSFS_SFLASH_ASSERT(block < dosfs_sflash_device.block_count);
    
    offset = dosfs_sflash_device.block_table[block] * DOSFS_SFLASH_BLOCK_SIZE;

#if (DOSFS_SFLASH_BLOCK_COUNT > 256)
    if (dosfs_sflash_device.block_table[DOSFS_SFLASH_BLOCK_COUNT + (block / 8)] & (1 << (block & 7)))
    {
	offset += (256 * DOSFS_SFLASH_BLOCK_SIZE);
    }
#endif /* (DOSFS_SFLASH_BLOCK_COUNT <= 256) */
    
    return offset;
}

static void dosfs_sflash_block_assign(uint32_t block, uint32_t offset)
{
    DOSFS_SFLASH_ASSERT(block < dosfs_sflash_device.block_count);
    DOSFS_SFLASH_ASSERT(offset < dosfs_sflash_device.data_size);

    dosfs_sflash_device.block_table[block] = offset / DOSFS_SFLASH_BLOCK_SIZE;

#if (DOSFS_SFLASH_BLOCK_COUNT > 256)
    if (offset >= (256 * DOSFS_SFLASH_BLOCK_SIZE))
    {
	dosfs_sflash_device.block_table[DOSFS_SFLASH_BLOCK_COUNT + (block / 8)] |= (1 << (block & 7));
    }
    else
    {
	dosfs_sflash_device.block_table[DOSFS_SFLASH_BLOCK_COUNT + (block / 8)] &= ~(1 << (block & 7));
    }
#endif /* (DOSFS_SFLASH_BLOCK_COUNT <= 256) */
}

static uint32_t dosfs_sflash_dir_tag_data_extract(const uint8_t *info, uint32_t sector)
{
    const uint8_t *data;

    DOSFS_SFLASH_ASSERT(sector <= (DOSFS_SFLASH_BLOCK_SIZE / DOSFS_SFLASH_SECTOR_SIZE));

    data = &info[DOSFS_SFLASH_INFO_DIR_OFFSET + (sector * DOSFS_SFLASH_DIR_ENTRY_SIZE)];

    return ((data[0] << 0) | (data[1] << 8) | (data[2] << 16));
}

static void dosfs_sflash_dir_tag_write(uint32_t entry, uint32_t tag)
{
    uint32_t block, sector, offset;
    uint8_t data;

    data = (uint8_t)(tag >> DOSFS_SFLASH_DIR_TAG_SHIFT);

    block = (entry & DOSFS_SFLASH_ENTRY_BLOCK_MASK) >> DOSFS_SFLASH_ENTRY_BLOCK_SHIFT;
    sector = (entry & DOSFS_SFLASH_ENTRY_SECTOR_MASK) >> DOSFS_SFLASH_ENTRY_SECTOR_SHIFT;

    DOSFS_SFLASH_ASSERT(block < dosfs_sflash_device.block_count);

    offset = dosfs_sflash_block_offset(block) + DOSFS_SFLASH_INFO_DIR_OFFSET + (sector * DOSFS_SFLASH_DIR_ENTRY_SIZE) + DOSFS_SFLASH_DIR_TAG_OFFSET;

    dosfs_sflash_nor_program(offset, &data, DOSFS_SFLASH_DIR_TAG_SIZE);
}

static void dosfs_sflash_dir_tag_data_write(uint32_t entry, uint32_t tag_data)
{
    uint32_t block, sector, offset;

    block = (entry & DOSFS_SFLASH_ENTRY_BLOCK_MASK) >> DOSFS_SFLASH_ENTRY_BLOCK_SHIFT;
    sector = (entry & DOSFS_SFLASH_ENTRY_SECTOR_MASK) >> DOSFS_SFLASH_ENTRY_SECTOR_SHIFT;

    DOSFS_SFLASH_ASSERT(block < dosfs_sflash_device.block_count);
    
    offset = dosfs_sflash_block_offset(block) + DOSFS_SFLASH_INFO_DIR_OFFSET + (sector * DOSFS_SFLASH_DIR_ENTRY_SIZE);

    dosfs_sflash_nor_program(offset, (uint8_t*)&tag_data, DOSFS_SFLASH_DIR_ENTRY_SIZE);
}

static uint32_t dosfs_sflash_ftl_offset(uint32_t entry)
{
    uint32_t block, sector, offset;

    block = (entry & DOSFS_SFLASH_ENTRY_BLOCK_MASK) >> DOSFS_SFLASH_ENTRY_BLOCK_SHIFT;
    sector = (entry & DOSFS_SFLASH_ENTRY_SECTOR_MASK) >> DOSFS_SFLASH_ENTRY_SECTOR_SHIFT;

    DOSFS_SFLASH_ASSERT(block < dosfs_sflash_device.block_count);
    DOSFS_SFLASH_ASSERT(sector != 0);
    
    offset = dosfs_sflash_device.block_table[block] * DOSFS_SFLASH_BLOCK_SIZE;

#if (DOSFS_SFLASH_BLOCK_COUNT > 256)
    if (dosfs_sflash_device.block_table[DOSFS_SFLASH_BLOCK_COUNT + (block / 8)] & (1 << (block & 7)))
    {
	offset += (256 * DOSFS_SFLASH_BLOCK_SIZE);
    }
#endif /* (DOSFS_SFLASH_BLOCK_COUNT <= 256) */
    
    return (offset + (sector * DOSFS_SFLASH_SECTOR_SIZE));
}

static void dosfs_sflash_ftl_flush()
{
    memset(&dosfs_sflash_device.cache_tag[0][0], 0xff, sizeof(dosfs_sflash_device.cache_tag));
}

static uint32_t dosfs_sflash_ftl_allocate(uint32_t tag_data)
{
    uint32_t sector, index, entry, offset;
    uint8_t *info;

    DOSFS_SFLASH_ASSERT(dosfs_sflash_device.erase_count);

    while (1)
    {
	if (!dosfs_sflash_device.alloc_count)
	{
	    dosfs_sflash_ftl_flush();

	    info = (uint8_t*)&dosfs_sflash_device.cache_data[0][0][0];
	    
	    dosfs_sflash_device.alloc_index = 0;
	    memset(&dosfs_sflash_device.alloc_mask[0], 0, sizeof(dosfs_sflash_device.alloc_mask));

	    do
	    {
		dosfs_sflash_device.alloc_block++;
		
		if (dosfs_sflash_device.alloc_block == dosfs_sflash_device.block_count)
		{
		    dosfs_sflash_device.alloc_block = 0;
		}
		
		offset = dosfs_sflash_block_offset(dosfs_sflash_device.alloc_block) + DOSFS_SFLASH_INFO_DIR_OFFSET;
		
		dosfs_sflash_nor_read(offset, &info[DOSFS_SFLASH_INFO_DIR_OFFSET], DOSFS_SFLASH_DIR_ENTRY_SIZE);

		if (dosfs_sflash_dir_tag_data_extract(info, 0) & DOSFS_SFLASH_DIR_TAG_STATE_NOT_WRITTEN)
		{
		    dosfs_sflash_nor_read(offset + DOSFS_SFLASH_DIR_ENTRY_SIZE, &info[DOSFS_SFLASH_INFO_DIR_OFFSET + DOSFS_SFLASH_DIR_ENTRY_SIZE], (DOSFS_SFLASH_INFO_DIR_SIZE - DOSFS_SFLASH_DIR_ENTRY_SIZE));

		    for (sector = 1; sector < (DOSFS_SFLASH_BLOCK_SIZE / DOSFS_SFLASH_SECTOR_SIZE); sector++)
		    {
			if (dosfs_sflash_dir_tag_data_extract(info, sector) == DOSFS_SFLASH_DIR_ENTRY_NOT_WRITTEN)
			{
			    dosfs_sflash_device.alloc_mask[sector / 32] |= (1 << (sector & 31));

			    dosfs_sflash_device.alloc_count++;
			}
		    }
		}
	    }
	    while (!dosfs_sflash_device.alloc_count);
	}
	
	for (index = dosfs_sflash_device.alloc_index; index < ((DOSFS_SFLASH_BLOCK_SIZE / DOSFS_SFLASH_SECTOR_SIZE) / 32); index++)
	{
	    if (dosfs_sflash_device.alloc_mask[index])
	    {
  	        dosfs_sflash_device.alloc_index = index;
  	        dosfs_sflash_device.alloc_count--;
		dosfs_sflash_device.erase_count--;

		sector = __builtin_ctz(dosfs_sflash_device.alloc_mask[index]);
		dosfs_sflash_device.alloc_mask[index] &= ~(1 << sector);
		
		entry = (dosfs_sflash_device.alloc_block << DOSFS_SFLASH_ENTRY_BLOCK_SHIFT) | ((sector + (index * 32)) << DOSFS_SFLASH_ENTRY_SECTOR_SHIFT);
		
		dosfs_sflash_dir_tag_data_write(entry, tag_data);

		if (!dosfs_sflash_device.alloc_count)
		{
		    /* If the last entry gets allocated, make the block as fully allocated.
		     */
		    dosfs_sflash_dir_tag_write((dosfs_sflash_device.alloc_block << DOSFS_SFLASH_ENTRY_BLOCK_SHIFT), ~DOSFS_SFLASH_DIR_TAG_STATE_NOT_WRITTEN);
		}
		
		return entry;
	    }
	}
    }
}

static void dosfs_sflash_ftl_deallocate(uint32_t entry)
{
    uint32_t entry_block, entry_sector;
    uint8_t mask;

    entry_block = (entry & DOSFS_SFLASH_ENTRY_BLOCK_MASK) >> DOSFS_SFLASH_ENTRY_BLOCK_SHIFT;
    entry_sector = (entry & DOSFS_SFLASH_ENTRY_SECTOR_MASK) >> DOSFS_SFLASH_ENTRY_SECTOR_SHIFT;
    
    mask = ~(1 << (entry_sector & 7));
    
    dosfs_sflash_nor_program((dosfs_sflash_block_offset(entry_block) + DOSFS_SFLASH_INFO_RECLAIM_OFFSET + (entry_sector / 8)), &mask, 1);
}

static uint32_t dosfs_sflash_ftl_merge(uint32_t xlate_entry)
{
    uint32_t offset, n;
    uint16_t *xlate_primary, *xlate_secondary;
    
    if (xlate_entry != DOSFS_SFLASH_XLATE_ENTRY_UNDEFINED)
    {
	dosfs_sflash_ftl_flush();

	dosfs_sflash_dir_tag_write(dosfs_sflash_device.xlate_entry, DOSFS_SFLASH_DIR_TAG_TYPE_XLATE_PRIMARY | DOSFS_SFLASH_DIR_TAG_STATE_NOT_COMMITTED);

	xlate_primary = &dosfs_sflash_device.cache_data[0][0][0];
	xlate_secondary = &dosfs_sflash_device.cache_data[1][0][0];
	
	for (offset = 0; offset < DOSFS_SFLASH_SECTOR_SIZE; offset += DOSFS_SFLASH_PAGE_SIZE)
	{
	    dosfs_sflash_nor_read(dosfs_sflash_ftl_offset(xlate_entry) + offset, (uint8_t*)&xlate_primary[0], DOSFS_SFLASH_PAGE_SIZE);
	    dosfs_sflash_nor_read(dosfs_sflash_ftl_offset(dosfs_sflash_device.xlate_entry) + offset, (uint8_t*)&xlate_secondary[0], DOSFS_SFLASH_PAGE_SIZE);
	    
	    for (n = 0; n < DOSFS_SFLASH_PAGE_SIZE / sizeof(uint16_t); n++)
	    {
	        if (xlate_primary[n] == DOSFS_SFLASH_ENTRY_DISCARDED)
		{
		    xlate_primary[n] = xlate_secondary[n];
		}
	    }
	    
	    dosfs_sflash_nor_program(dosfs_sflash_ftl_offset(dosfs_sflash_device.xlate_entry) + offset, (uint8_t*)&xlate_primary[0], DOSFS_SFLASH_PAGE_SIZE);
	}
	
	dosfs_sflash_ftl_deallocate(xlate_entry);
    }

    xlate_entry = dosfs_sflash_device.xlate_entry;
    
    dosfs_sflash_dir_tag_write(xlate_entry, DOSFS_SFLASH_DIR_TAG_TYPE_XLATE_PRIMARY);
    
    dosfs_sflash_device.xlate_table[dosfs_sflash_device.xlate_slot] = xlate_entry;
    
    dosfs_sflash_device.xlate_slot = DOSFS_SFLASH_XLATE_SLOT_UNDEFINED;
    dosfs_sflash_device.xlate_entry = DOSFS_SFLASH_XLATE_ENTRY_UNDEFINED;

    return xlate_entry;
}

static uint32_t dosfs_sflash_ftl_lookup(uint32_t address)
{
    uint32_t entry_direct, entry_primary, entry_secondary, cache_tag, cache_block, cache_index, xlate_entry, xlate_address, xlate_slot, xlate_index, xlate_offset;

    DOSFS_SFLASH_ASSERT(address < dosfs_sflash_device.sector_count);

    if (address < DOSFS_SFLASH_XLATE_OFFSET)
    {
	entry_direct = dosfs_sflash_device.xlate_direct[address];

	return entry_direct;
    }

    xlate_address = address - DOSFS_SFLASH_XLATE_OFFSET;
    xlate_slot = xlate_address >> DOSFS_SFLASH_XLATE_SLOT_SHIFT;
    xlate_index = xlate_address & DOSFS_SFLASH_XLATE_INDEX_MASK;
    xlate_entry = dosfs_sflash_device.xlate_table[xlate_slot];

    if (xlate_entry == DOSFS_SFLASH_XLATE_ENTRY_UNDEFINED)
    {
	return DOSFS_SFLASH_ENTRY_UNDEFINED;
    }
    
    cache_tag = xlate_address >> DOSFS_SFLASH_CACHE_TAG_SHIFT;
    cache_block = cache_tag & (DOSFS_SFLASH_CACHE_BLOCK_MASK >> DOSFS_SFLASH_CACHE_BLOCK_SHIFT);
    cache_index = xlate_address & DOSFS_SFLASH_CACHE_INDEX_MASK;

    if (dosfs_sflash_device.cache_tag[0][cache_block] != cache_tag)
    {
	dosfs_sflash_device.cache_tag[0][cache_block] = cache_tag;
	
	xlate_offset = (xlate_index & ~DOSFS_SFLASH_CACHE_INDEX_MASK) * sizeof(uint16_t);
	    
	dosfs_sflash_nor_read(dosfs_sflash_ftl_offset(xlate_entry) + xlate_offset, (uint8_t*)&dosfs_sflash_device.cache_data[0][cache_block][0], (DOSFS_SFLASH_CACHE_INDEX_COUNT * sizeof(uint16_t)));
    }

    entry_primary = dosfs_sflash_device.cache_data[0][cache_block][cache_index];

    if (entry_primary != DOSFS_SFLASH_ENTRY_DISCARDED)
    {
	return entry_primary;
    }

    if (dosfs_sflash_device.xlate_slot != xlate_slot)
    {
	return DOSFS_SFLASH_ENTRY_UNDEFINED;
    }

    if (dosfs_sflash_device.cache_tag[1][cache_block] != cache_tag)
    {
	dosfs_sflash_device.cache_tag[1][cache_block] = cache_tag;

	xlate_offset = (xlate_index & ~DOSFS_SFLASH_CACHE_INDEX_MASK) * sizeof(uint16_t);
	    
	dosfs_sflash_nor_read(dosfs_sflash_ftl_offset(dosfs_sflash_device.xlate_entry) + xlate_offset, (uint8_t*)&dosfs_sflash_device.cache_data[1][cache_block][0], (DOSFS_SFLASH_CACHE_INDEX_COUNT * sizeof(uint16_t)));
    }
    
    entry_secondary = dosfs_sflash_device.cache_data[1][cache_block][cache_index];

    if (entry_secondary == DOSFS_SFLASH_ENTRY_DISCARDED)
    {
	entry_secondary = DOSFS_SFLASH_ENTRY_UNDEFINED;
    }

    return entry_secondary;
}

static void dosfs_sflash_ftl_replace(uint32_t address, uint32_t entry_replace)
{
    uint32_t entry_direct, entry_primary, entry_secondary, entry_discarded, cache_tag, cache_block, cache_index, xlate_entry, xlate_address, xlate_slot, xlate_index, xlate_offset;

    DOSFS_SFLASH_ASSERT(address < dosfs_sflash_device.sector_count);
    
    DOSFS_SFLASH_ASSERT(entry_replace != DOSFS_SFLASH_ENTRY_UNDEFINED);
    DOSFS_SFLASH_ASSERT(entry_replace != DOSFS_SFLASH_ENTRY_DISCARDED);
    
    if (address < DOSFS_SFLASH_XLATE_OFFSET)
    {
	entry_direct = dosfs_sflash_device.xlate_direct[address];

	if (entry_direct != DOSFS_SFLASH_ENTRY_UNDEFINED)
	{
	    dosfs_sflash_ftl_deallocate(entry_direct);
	}

	dosfs_sflash_device.xlate_direct[address] = entry_replace;
    }
    else
    {
	xlate_address = address - DOSFS_SFLASH_XLATE_OFFSET;
	xlate_slot = xlate_address >> DOSFS_SFLASH_XLATE_SLOT_SHIFT;
	xlate_index = xlate_address & DOSFS_SFLASH_XLATE_INDEX_MASK;
	xlate_entry = dosfs_sflash_device.xlate_table[xlate_slot];
    
	cache_tag = xlate_address >> DOSFS_SFLASH_CACHE_TAG_SHIFT;
	cache_block = cache_tag & (DOSFS_SFLASH_CACHE_BLOCK_MASK >> DOSFS_SFLASH_CACHE_BLOCK_SHIFT);
	cache_index = xlate_address & DOSFS_SFLASH_CACHE_INDEX_MASK;
    
	if (xlate_entry == DOSFS_SFLASH_XLATE_ENTRY_UNDEFINED)
	{
	    xlate_entry = dosfs_sflash_device.xlate_table[xlate_slot] = dosfs_sflash_ftl_allocate(DOSFS_SFLASH_DIR_TAG_TYPE_XLATE_PRIMARY | xlate_slot);
	
	    dosfs_sflash_nor_program(dosfs_sflash_ftl_offset(xlate_entry) + (xlate_index * sizeof(uint16_t)), (uint8_t*)&entry_replace, sizeof(uint16_t));
	}
	else
	{
	    if (dosfs_sflash_device.cache_tag[0][cache_block] != cache_tag)
	    {
		dosfs_sflash_device.cache_tag[0][cache_block] = cache_tag;
	
		xlate_offset = (xlate_index & ~DOSFS_SFLASH_CACHE_INDEX_MASK) * sizeof(uint16_t);
	
		dosfs_sflash_nor_read(dosfs_sflash_ftl_offset(xlate_entry) + xlate_offset, (uint8_t*)&dosfs_sflash_device.cache_data[0][cache_block][0], (DOSFS_SFLASH_CACHE_INDEX_COUNT * sizeof(uint16_t)));
	    }
    
	    entry_primary = dosfs_sflash_device.cache_data[0][cache_block][cache_index];

	    if (entry_primary == DOSFS_SFLASH_ENTRY_UNDEFINED)
	    {
		dosfs_sflash_device.cache_data[0][cache_block][cache_index] = entry_replace;
	
		dosfs_sflash_nor_program(dosfs_sflash_ftl_offset(xlate_entry) + (xlate_index * sizeof(uint16_t)), (uint8_t*)&entry_replace, sizeof(uint16_t));
	    }
	    else
	    {
		/* Below here the xlate secondary needs to be updated. entry_secondary hence points at the end to the deallocated entry.
		 */

		entry_secondary = entry_primary;
    
		if (entry_primary != DOSFS_SFLASH_ENTRY_DISCARDED)
		{
		    dosfs_sflash_device.cache_data[0][cache_block][cache_index] = DOSFS_SFLASH_ENTRY_DISCARDED;

		    dosfs_sflash_ftl_deallocate(entry_primary);
	
		    entry_discarded = DOSFS_SFLASH_ENTRY_DISCARDED;
		
		    dosfs_sflash_nor_program(dosfs_sflash_ftl_offset(xlate_entry) + (xlate_index * sizeof(uint16_t)), (uint8_t*)&entry_discarded, sizeof(uint16_t));

		    if (dosfs_sflash_device.xlate_slot != xlate_slot)
		    {
			if (dosfs_sflash_device.xlate_slot != DOSFS_SFLASH_XLATE_SLOT_UNDEFINED)
			{
			    dosfs_sflash_ftl_merge(dosfs_sflash_device.xlate_table[dosfs_sflash_device.xlate_slot]);
			}
		    }
		}
		else
		{
		    if (dosfs_sflash_device.xlate_slot == xlate_slot)
		    {
			if (dosfs_sflash_device.cache_tag[1][cache_block] != cache_tag)
			{
			    dosfs_sflash_device.cache_tag[1][cache_block] = cache_tag;
		
			    xlate_offset = (xlate_index & ~DOSFS_SFLASH_CACHE_INDEX_MASK) * sizeof(uint16_t);
		
			    dosfs_sflash_nor_read(dosfs_sflash_ftl_offset(dosfs_sflash_device.xlate_entry) + xlate_offset, (uint8_t*)&dosfs_sflash_device.cache_data[1][cache_block][0], (DOSFS_SFLASH_CACHE_INDEX_COUNT * sizeof(uint16_t)));
			}
	    
			entry_secondary = dosfs_sflash_device.cache_data[1][cache_block][cache_index];
	    
			if (entry_secondary != DOSFS_SFLASH_ENTRY_UNDEFINED)
			{
			    if (entry_secondary != DOSFS_SFLASH_ENTRY_DISCARDED)
			    {
				dosfs_sflash_ftl_deallocate(entry_secondary);
		    
				entry_discarded = DOSFS_SFLASH_ENTRY_DISCARDED;
		    
				dosfs_sflash_nor_program(dosfs_sflash_ftl_offset(dosfs_sflash_device.xlate_entry) + (xlate_index * sizeof(uint16_t)), (uint8_t*)&entry_discarded, sizeof(uint16_t));
			    }
		
			    dosfs_sflash_ftl_merge(xlate_entry);
			}
		    }
		    else
		    {
			if (dosfs_sflash_device.xlate_slot != DOSFS_SFLASH_XLATE_SLOT_UNDEFINED)
			{
			    dosfs_sflash_ftl_merge(dosfs_sflash_device.xlate_table[dosfs_sflash_device.xlate_slot]);
			}
		    }
		}

		if (dosfs_sflash_device.xlate_slot == DOSFS_SFLASH_XLATE_SLOT_UNDEFINED)
		{
		    dosfs_sflash_device.xlate_slot = xlate_slot;
		    dosfs_sflash_device.xlate_entry = dosfs_sflash_ftl_allocate(DOSFS_SFLASH_DIR_TAG_TYPE_XLATE_SECONDARY | DOSFS_SFLASH_DIR_TAG_STATE_NOT_COMMITTED | xlate_slot);
		}
		else
		{
		    DOSFS_SFLASH_ASSERT(dosfs_sflash_device.xlate_slot == xlate_slot);

		    if (dosfs_sflash_device.cache_tag[1][cache_block] == cache_tag)
		    {
			dosfs_sflash_device.cache_data[1][cache_block][cache_index] = entry_replace;
		    }
		}
    
		dosfs_sflash_nor_program(dosfs_sflash_ftl_offset(dosfs_sflash_device.xlate_entry) + (xlate_index * sizeof(uint16_t)), (uint8_t*)&entry_replace, sizeof(uint16_t));
	    }
	}
    }

    dosfs_sflash_dir_tag_write(entry_replace, ~DOSFS_SFLASH_DIR_TAG_STATE_NOT_COMMITTED);
		
    return;
}

static void dosfs_sflash_ftl_discard(uint32_t address)
{
    uint32_t entry_direct, entry_primary, entry_secondary, entry_discarded, cache_tag, cache_block, cache_index, xlate_entry, xlate_address, xlate_slot, xlate_index, xlate_offset;

    DOSFS_SFLASH_ASSERT(address < dosfs_sflash_device.sector_count);

    if (address < DOSFS_SFLASH_XLATE_OFFSET)
    {
	entry_direct = dosfs_sflash_device.xlate_direct[address];

	if (entry_direct != DOSFS_SFLASH_ENTRY_UNDEFINED)
	{
	    dosfs_sflash_ftl_deallocate(entry_direct);
	}
	
	dosfs_sflash_device.xlate_direct[address] = DOSFS_SFLASH_ENTRY_UNDEFINED;

	return;
    }

    xlate_address = address - DOSFS_SFLASH_XLATE_OFFSET;
    xlate_slot = xlate_address >> DOSFS_SFLASH_XLATE_SLOT_SHIFT;
    xlate_index = xlate_address & DOSFS_SFLASH_XLATE_INDEX_MASK;
    xlate_entry = dosfs_sflash_device.xlate_table[xlate_slot];

    if (xlate_entry == DOSFS_SFLASH_XLATE_ENTRY_UNDEFINED)
    {
	return;
    }

    cache_tag = xlate_address >> DOSFS_SFLASH_CACHE_TAG_SHIFT;
    cache_block = cache_tag & (DOSFS_SFLASH_CACHE_BLOCK_MASK >> DOSFS_SFLASH_CACHE_BLOCK_SHIFT);
    cache_index = xlate_address & DOSFS_SFLASH_CACHE_INDEX_MASK;
    
    if (dosfs_sflash_device.cache_tag[0][cache_block] != cache_tag)
    {
	dosfs_sflash_device.cache_tag[0][cache_block] = cache_tag;

	xlate_offset = (xlate_index & ~DOSFS_SFLASH_CACHE_INDEX_MASK) * sizeof(uint16_t);
	    
	dosfs_sflash_nor_read(dosfs_sflash_ftl_offset(xlate_entry) + xlate_offset, (uint8_t*)&dosfs_sflash_device.cache_data[0][cache_block][0], (DOSFS_SFLASH_CACHE_INDEX_COUNT * sizeof(uint16_t)));
    }
    
    entry_primary = dosfs_sflash_device.cache_data[0][cache_block][cache_index];

    if (entry_primary == DOSFS_SFLASH_ENTRY_UNDEFINED)
    {
	return;
    }

    if (entry_primary != DOSFS_SFLASH_ENTRY_DISCARDED)
    {
	dosfs_sflash_device.cache_data[0][cache_block][cache_index] = DOSFS_SFLASH_ENTRY_DISCARDED;

	dosfs_sflash_ftl_deallocate(entry_primary);

	entry_discarded = DOSFS_SFLASH_ENTRY_DISCARDED;
		
	dosfs_sflash_nor_program(dosfs_sflash_ftl_offset(xlate_entry) + (xlate_index * sizeof(uint16_t)), (uint8_t*)&entry_discarded, sizeof(uint16_t));
    }
    else
    {
	if (dosfs_sflash_device.xlate_slot == xlate_slot)
	{
	    if (dosfs_sflash_device.cache_tag[1][cache_block] != cache_tag)
	    {
		dosfs_sflash_device.cache_tag[1][cache_block] = cache_tag;

		xlate_offset = (xlate_index & ~DOSFS_SFLASH_CACHE_INDEX_MASK) * sizeof(uint16_t);
	
		dosfs_sflash_nor_read(dosfs_sflash_ftl_offset(dosfs_sflash_device.xlate_entry) + xlate_offset, (uint8_t*)&dosfs_sflash_device.cache_data[1][cache_block][0], (DOSFS_SFLASH_CACHE_INDEX_COUNT * sizeof(uint16_t)));
	    }

	    entry_secondary = dosfs_sflash_device.cache_data[1][cache_block][cache_index];

	    if (entry_secondary != DOSFS_SFLASH_ENTRY_UNDEFINED)
	    {
		if (entry_secondary != DOSFS_SFLASH_ENTRY_DISCARDED)
		{
		    dosfs_sflash_device.cache_data[1][cache_block][cache_index] = DOSFS_SFLASH_ENTRY_DISCARDED;
		    
		    dosfs_sflash_ftl_deallocate(entry_secondary);
		    
		    entry_discarded = DOSFS_SFLASH_ENTRY_DISCARDED;
		    
		    dosfs_sflash_nor_program(dosfs_sflash_ftl_offset(dosfs_sflash_device.xlate_entry) + (xlate_index * sizeof(uint16_t)), (uint8_t*)&entry_discarded, sizeof(uint16_t));
		}
	    }
	}
    }
}

static uint32_t dosfs_sflash_ftl_victim()
{
    uint32_t block, index, offset, erase_count, erase_count_min, erase_count_max, reclaim_count;
    uint32_t victim_l_block, victim_l_erase_count, victim_l_reclaim_count, victim_h_block, victim_h_erase_count, victim_h_reclaim_count;
    uint32_t *info_head;
    uint8_t *info;
    
    erase_count_min = ~0;    
    erase_count_max = 0;    

    victim_l_block = 0;
    victim_l_erase_count = ~0;
    victim_l_reclaim_count = 0;

    victim_h_block = 0;
    victim_h_erase_count = ~0;
    victim_h_reclaim_count = 0;

    dosfs_sflash_ftl_flush();

    info = (uint8_t*)&dosfs_sflash_device.cache_data[0][0][0];
    info_head = (uint32_t*)&info[DOSFS_SFLASH_INFO_HEAD_OFFSET];
    
    for (block = 0; block < dosfs_sflash_device.block_count; block++)
    {
	offset = dosfs_sflash_block_offset(block) + DOSFS_SFLASH_INFO_ERASE_COUNT_OFFSET;
	
	dosfs_sflash_nor_read(offset, &info[DOSFS_SFLASH_INFO_ERASE_COUNT_OFFSET], DOSFS_SFLASH_INFO_ERASE_COUNT_SIZE + DOSFS_SFLASH_INFO_RECLAIM_SIZE);

	erase_count = info_head[DOSFS_SFLASH_INFO_HEAD_ERASE_COUNT];

	if (erase_count_min > erase_count)
	{
	    erase_count_min = erase_count;
	}
	
	if (erase_count_max < erase_count)
	{
	    erase_count_max = erase_count;
	}

	for (reclaim_count = 0, index = 0; index < 16; index++)
	{
	    reclaim_count += dosfs_sflash_ftl_zeros[info[DOSFS_SFLASH_INFO_RECLAIM_OFFSET + index]];
	}
	
	if (erase_count < victim_l_erase_count)
	{
	    victim_l_block = block;
	    victim_l_erase_count = erase_count;
	    victim_l_reclaim_count = reclaim_count;
	}
	else
	{
	    if ((erase_count == victim_l_erase_count) && (reclaim_count > victim_l_reclaim_count))
	    {
		victim_l_block = block;
		victim_l_erase_count = erase_count;
		victim_l_reclaim_count = reclaim_count;
	    }
	}

	if (reclaim_count > victim_h_reclaim_count)
	{
	    victim_h_block = block;
	    victim_h_erase_count = erase_count;
	    victim_h_reclaim_count = reclaim_count;
	}
	else
	{
	    if ((reclaim_count == victim_h_reclaim_count) && (erase_count < victim_h_erase_count))
	    {
		victim_h_block = block;
		victim_h_erase_count = erase_count;
		victim_h_reclaim_count = reclaim_count;
	    }
	}
    }

    if ((erase_count_max - erase_count_min) < DOSFS_SFLASH_ERASE_THRESHOLD)
    {
        return victim_h_block;
    }
    else
    {
	return victim_l_block;
    }
}

static void dosfs_sflash_ftl_reclaim()
{
    uint32_t sector, alloc_count, info_tag, info_tag_data, reclaim_offset, reclaim_erase_count, victim_block, victim_offset, victim_erase_count;
    uint32_t *info_head, *info_tail;
    uint8_t *info, *data;
    uint32_t reclaim_mask[DOSFS_SFLASH_BLOCK_SIZE / (DOSFS_SFLASH_SECTOR_SIZE * 32)];
      
    if (dosfs_sflash_device.erase_count >= DOSFS_SFLASH_RECLAIM_THRESHOLD)
    {
	return;
    }

    dosfs_sflash_ftl_flush();
    
    info = (uint8_t*)&dosfs_sflash_device.cache_data[0][0][0];
    info_head = (uint32_t*)&info[DOSFS_SFLASH_INFO_HEAD_OFFSET];
    info_tail = (uint32_t*)&info[DOSFS_SFLASH_INFO_TAIL_OFFSET];

    data = (uint8_t*)&dosfs_sflash_device.cache_data[0][0][0];
    
    do
    {
	victim_block = dosfs_sflash_ftl_victim();
	victim_offset = dosfs_sflash_block_offset(victim_block);
	reclaim_offset = dosfs_sflash_device.reclaim_offset;

	if (dosfs_sflash_device.alloc_block == victim_block)
	{
	    dosfs_sflash_device.alloc_count = 0;
	}
	
	/* Clear the STATE_NOT_WRITTEN flag in the RECLAIM block.
	 */
	dosfs_sflash_nor_read(reclaim_offset + DOSFS_SFLASH_INFO_ERASE_COUNT_OFFSET, (uint8_t*)&reclaim_erase_count, DOSFS_SFLASH_INFO_ERASE_COUNT_SIZE);
	
	dosfs_sflash_nor_read(victim_offset, &info[0], DOSFS_SFLASH_SECTOR_SIZE);
	
	victim_erase_count = info_head[DOSFS_SFLASH_INFO_HEAD_ERASE_COUNT];

	memset(&reclaim_mask[0], 0, sizeof(reclaim_mask));
	
	for (alloc_count = 0, sector = 1; sector < (DOSFS_SFLASH_BLOCK_SIZE / DOSFS_SFLASH_SECTOR_SIZE); sector++)
	{
	    if (info[DOSFS_SFLASH_INFO_RECLAIM_OFFSET + (sector / 8)] & (1 << (sector & 7)))
	    {
		info_tag_data = dosfs_sflash_dir_tag_data_extract(info, sector);
		
		if (info_tag_data != DOSFS_SFLASH_DIR_ENTRY_NOT_WRITTEN)
		{
		    reclaim_mask[(sector / 32)] |= (1 << (sector & 31));
		}
		else
		{
		    alloc_count++;
		}
	    }
	    else
	    {
		info[DOSFS_SFLASH_INFO_RECLAIM_OFFSET + (sector / 8)] |= (1 << (sector & 7));
		
		info[DOSFS_SFLASH_INFO_DIR_OFFSET + (sector * DOSFS_SFLASH_DIR_ENTRY_SIZE) +0] = 0xff;
		info[DOSFS_SFLASH_INFO_DIR_OFFSET + (sector * DOSFS_SFLASH_DIR_ENTRY_SIZE) +1] = 0xff;
		info[DOSFS_SFLASH_INFO_DIR_OFFSET + (sector * DOSFS_SFLASH_DIR_ENTRY_SIZE) +2] = 0xff;
		
		alloc_count++;
	    }
	}

    	info_tag_data = (DOSFS_SFLASH_DIR_TAG_TYPE_INFO | DOSFS_SFLASH_DIR_TAG_STATE_NOT_COMMITTED | victim_block);

	if (alloc_count)
	{
	    info_tag_data |= DOSFS_SFLASH_DIR_TAG_STATE_NOT_WRITTEN;
	}
			 
	info_head[DOSFS_SFLASH_INFO_HEAD_ERASE_COUNT] = reclaim_erase_count;
	
	info[DOSFS_SFLASH_INFO_DIR_OFFSET +0] = (uint8_t)(info_tag_data >> 0);
	info[DOSFS_SFLASH_INFO_DIR_OFFSET +1] = (uint8_t)(info_tag_data >> 8);
	info[DOSFS_SFLASH_INFO_DIR_OFFSET +2] = (uint8_t)(info_tag_data >> 16);
	
	dosfs_sflash_nor_program(reclaim_offset, &info[0], DOSFS_SFLASH_PAGE_SIZE);
	dosfs_sflash_nor_program(reclaim_offset + DOSFS_SFLASH_PAGE_SIZE, &info[DOSFS_SFLASH_PAGE_SIZE], DOSFS_SFLASH_PAGE_SIZE);
	
	for (sector = 1; sector < (DOSFS_SFLASH_BLOCK_SIZE / DOSFS_SFLASH_SECTOR_SIZE); sector++)
	{
	    if (reclaim_mask[(sector / 32)] & (1 << (sector & 31)))
	    {
		dosfs_sflash_nor_read(victim_offset + (sector * DOSFS_SFLASH_SECTOR_SIZE), &data[0], DOSFS_SFLASH_SECTOR_SIZE);
		
		dosfs_sflash_nor_program(reclaim_offset + (sector * DOSFS_SFLASH_SECTOR_SIZE), &data[0], DOSFS_SFLASH_PAGE_SIZE);
		dosfs_sflash_nor_program(reclaim_offset + (sector * DOSFS_SFLASH_SECTOR_SIZE) + DOSFS_SFLASH_PAGE_SIZE, &data[DOSFS_SFLASH_PAGE_SIZE], DOSFS_SFLASH_PAGE_SIZE);
	    }
	}
	
	/* Convert the victim (INFO) block into a RECLAIM block.
	 */
	info_head[DOSFS_SFLASH_INFO_HEAD_MAGIC] = DOSFS_SFLASH_MAGIC_HEAD;
	info_head[DOSFS_SFLASH_INFO_HEAD_DATA_START] = dosfs_sflash_device.data_start;
	info_head[DOSFS_SFLASH_INFO_HEAD_DATA_SIZE] = dosfs_sflash_device.data_size;
	info_head[DOSFS_SFLASH_INFO_HEAD_ERASE_COUNT] = victim_erase_count +1;
	
    	info_tag_data = DOSFS_SFLASH_DIR_TAG_TYPE_RECLAIM | DOSFS_SFLASH_DIR_TAG_STATE_NOT_COMMITTED | DOSFS_SFLASH_DIR_TAG_STATE_NOT_WRITTEN | DOSFS_SFLASH_BLOCK_UNDEFINED;
	
	info_tail[DOSFS_SFLASH_INFO_TAIL_MAGIC] = DOSFS_SFLASH_MAGIC_TAIL;
	
	dosfs_sflash_nor_erase(victim_offset);
	
	dosfs_sflash_nor_program(victim_offset + DOSFS_SFLASH_INFO_HEAD_OFFSET, (const uint8_t*)&info_head[0], DOSFS_SFLASH_INFO_HEAD_SIZE);
	dosfs_sflash_nor_program(victim_offset + DOSFS_SFLASH_INFO_DIR_OFFSET, (const uint8_t*)&info_tag_data, DOSFS_SFLASH_DIR_ENTRY_SIZE);
	dosfs_sflash_nor_program(victim_offset + DOSFS_SFLASH_INFO_TAIL_OFFSET, (const uint8_t*)&info_tail[0], DOSFS_SFLASH_INFO_TAIL_SIZE);
	
    	info_tag = ~DOSFS_SFLASH_DIR_TAG_STATE_NOT_COMMITTED >> DOSFS_SFLASH_DIR_TAG_SHIFT;
	dosfs_sflash_nor_program(reclaim_offset + DOSFS_SFLASH_INFO_DIR_OFFSET + DOSFS_SFLASH_DIR_TAG_OFFSET, (const uint8_t*)&info_tag, DOSFS_SFLASH_DIR_TAG_SIZE);
	
	dosfs_sflash_block_assign(victim_block, reclaim_offset);
	
	dosfs_sflash_device.reclaim_offset = victim_offset;
	
	dosfs_sflash_device.erase_count += alloc_count;
    }
    while (dosfs_sflash_device.erase_count < DOSFS_SFLASH_RECLAIM_THRESHOLD);
}

static void dosfs_sflash_ftl_verify(uint32_t xlate_entry)
{
    uint32_t index, entry, block, sector, offset_cache, offset_mask, offset_reclaim;
    uint16_t *cache;
    uint32_t mask;

    cache = (uint16_t*)&dosfs_sflash_device.cache_data[0][0][0];

    offset_cache = dosfs_sflash_ftl_offset(xlate_entry);
    
    dosfs_sflash_nor_read(offset_cache, (uint8_t*)&cache[0], DOSFS_SFLASH_SECTOR_SIZE);

    offset_mask = DOSFS_SFLASH_DATA_SIZE;
    
    for (index = 0; index < (DOSFS_SFLASH_SECTOR_SIZE / sizeof(uint16_t)); index++)
    {
	entry = cache[index];
	
	if ((entry != DOSFS_SFLASH_ENTRY_DISCARDED) && (entry != DOSFS_SFLASH_ENTRY_UNDEFINED))
	{
	    block = (entry & DOSFS_SFLASH_ENTRY_BLOCK_MASK) >> DOSFS_SFLASH_ENTRY_BLOCK_SHIFT;
	    sector = (entry & DOSFS_SFLASH_ENTRY_SECTOR_MASK) >> DOSFS_SFLASH_ENTRY_SECTOR_SHIFT;
	    
	    offset_reclaim = dosfs_sflash_block_offset(block) + DOSFS_SFLASH_INFO_RECLAIM_OFFSET + (sector / 32);
	    
	    if (offset_mask != offset_reclaim)
	    {
		offset_mask = offset_reclaim;
		
		dosfs_sflash_nor_read(offset_mask, (uint8_t*)&mask, sizeof(mask));
	    }
	    
	    if (!(mask & (1 << (sector & 31))))
	    {
		entry = DOSFS_SFLASH_ENTRY_DISCARDED;
		
		dosfs_sflash_nor_program(offset_cache + (index * sizeof(uint16_t)), (uint8_t*)&entry, sizeof(uint16_t));
	    }
	}
    }
}

static void dosfs_sflash_ftl_format()
{
    uint32_t offset, erase_count, erase_count_max, info_tag_data;
    uint32_t *info_head, *info_tail;
    uint8_t *info;

    info = (uint8_t*)&dosfs_sflash_device.cache_data[0][0][0];
    info_head = (uint32_t*)&info[DOSFS_SFLASH_INFO_HEAD_OFFSET];
    info_tail = (uint32_t*)&info[DOSFS_SFLASH_INFO_TAIL_OFFSET];
    
    for (erase_count_max = 0, offset = 0; offset < dosfs_sflash_device.data_size; offset += DOSFS_SFLASH_BLOCK_SIZE)
    {
	dosfs_sflash_nor_read(offset + DOSFS_SFLASH_INFO_HEAD_OFFSET, (uint8_t*)&info_head[0], DOSFS_SFLASH_INFO_HEAD_SIZE);
	dosfs_sflash_nor_read(offset + DOSFS_SFLASH_INFO_TAIL_OFFSET, (uint8_t*)&info_tail[0], DOSFS_SFLASH_INFO_TAIL_SIZE);

	if ((info_head[DOSFS_SFLASH_INFO_HEAD_MAGIC] == DOSFS_SFLASH_MAGIC_HEAD) && (info_tail[DOSFS_SFLASH_INFO_TAIL_MAGIC] == DOSFS_SFLASH_MAGIC_TAIL))
	{
	    if (erase_count_max < info_head[DOSFS_SFLASH_INFO_HEAD_ERASE_COUNT])
	    {
		erase_count_max = info_head[DOSFS_SFLASH_INFO_HEAD_ERASE_COUNT];
	    }
	}
    }
    
    for (offset = 0; offset < dosfs_sflash_device.data_size; offset += DOSFS_SFLASH_BLOCK_SIZE)
    {
	dosfs_sflash_nor_read(offset + DOSFS_SFLASH_INFO_HEAD_OFFSET, (uint8_t*)&info_head[0], DOSFS_SFLASH_INFO_HEAD_SIZE);
	dosfs_sflash_nor_read(offset + DOSFS_SFLASH_INFO_TAIL_OFFSET, (uint8_t*)&info_tail[0], DOSFS_SFLASH_INFO_TAIL_SIZE);

	if ((info_head[DOSFS_SFLASH_INFO_HEAD_MAGIC] == DOSFS_SFLASH_MAGIC_HEAD) && (info_tail[DOSFS_SFLASH_INFO_TAIL_MAGIC] == DOSFS_SFLASH_MAGIC_TAIL))
	{
	    erase_count = info_head[DOSFS_SFLASH_INFO_HEAD_ERASE_COUNT];
	}
	else
	{
	    erase_count = erase_count_max +1;
	}

	info_head[DOSFS_SFLASH_INFO_HEAD_MAGIC] = DOSFS_SFLASH_MAGIC_HEAD;
	info_head[DOSFS_SFLASH_INFO_HEAD_DATA_START] = dosfs_sflash_device.data_start;
	info_head[DOSFS_SFLASH_INFO_HEAD_DATA_SIZE] = dosfs_sflash_device.data_size;
	info_head[DOSFS_SFLASH_INFO_HEAD_ERASE_COUNT] = erase_count;

	if (offset != (dosfs_sflash_device.data_size - DOSFS_SFLASH_BLOCK_SIZE))
	{
	    info_tag_data = DOSFS_SFLASH_DIR_TAG_TYPE_INFO | DOSFS_SFLASH_DIR_TAG_STATE_NOT_WRITTEN | (offset / DOSFS_SFLASH_BLOCK_SIZE);
	}
	else
	{
	    info_tag_data = DOSFS_SFLASH_DIR_TAG_TYPE_RECLAIM | DOSFS_SFLASH_DIR_TAG_STATE_NOT_COMMITTED | DOSFS_SFLASH_DIR_TAG_STATE_NOT_WRITTEN | DOSFS_SFLASH_BLOCK_UNDEFINED;
	}
	
	info_tail[DOSFS_SFLASH_INFO_TAIL_MAGIC] = DOSFS_SFLASH_MAGIC_TAIL;

	dosfs_sflash_nor_erase(offset);

	dosfs_sflash_nor_program(offset + DOSFS_SFLASH_INFO_HEAD_OFFSET, (const uint8_t*)&info_head[0], DOSFS_SFLASH_INFO_HEAD_SIZE);
	dosfs_sflash_nor_program(offset + DOSFS_SFLASH_INFO_DIR_OFFSET, (const uint8_t*)&info_tag_data, DOSFS_SFLASH_DIR_ENTRY_SIZE);
	dosfs_sflash_nor_program(offset + DOSFS_SFLASH_INFO_TAIL_OFFSET, (const uint8_t*)&info_tail[0], DOSFS_SFLASH_INFO_TAIL_SIZE);
    }
}

static bool dosfs_sflash_ftl_start()
{
    uint32_t offset, block, sector, slot, address, alloc_count, erase_count_max, reclaim_offset, reclaim_block, victim_offset, victim_erase_count, xlate_slot, xlate_entry;
    uint32_t *info_head, *info_tail, info_tag_data, info_tag;
    uint8_t *info;

    if (!dosfs_sflash_device.data_size)
    {
	return false;
    }
    
    dosfs_sflash_device.block_count = (dosfs_sflash_device.data_size / DOSFS_SFLASH_BLOCK_SIZE) -1;

    /* This is kind of wining it. It is assumed that per block there are 127 useable sectors. There needed to be
     * at least 0.5 sectors per block to map it (each xlate sector has 256 entries). There also needs to be space
     * for one xlate secondary entry. But at the end of the day there needs to be a litte bit of extra to avoid
     * tooo many reclaim operations. Hence here as a rought guess, the system is populated with 120/128 == 93.75%.
     */
    dosfs_sflash_device.sector_count = dosfs_sflash_device.block_count * ((DOSFS_SFLASH_BLOCK_SIZE / DOSFS_SFLASH_SECTOR_SIZE) - 8);
      
    dosfs_sflash_device.reclaim_offset = DOSFS_SFLASH_DATA_OFFSET_UNDEFINED;

    dosfs_sflash_device.erase_count = 0;
    dosfs_sflash_device.alloc_block = dosfs_sflash_device.block_count -1;
    dosfs_sflash_device.alloc_count = 0;
    dosfs_sflash_device.alloc_index = 0;

    dosfs_sflash_device.xlate_slot = DOSFS_SFLASH_XLATE_SLOT_UNDEFINED;
    dosfs_sflash_device.xlate_entry = DOSFS_SFLASH_XLATE_ENTRY_UNDEFINED;

    dosfs_sflash_ftl_flush();

    memset(&dosfs_sflash_device.xlate_direct[0], 0xff, sizeof(dosfs_sflash_device.xlate_direct));
    memset(&dosfs_sflash_device.xlate_table[0], 0xff, sizeof(dosfs_sflash_device.xlate_table));
    memset(&dosfs_sflash_device.block_table[0], 0xff, sizeof(dosfs_sflash_device.block_table));

    info = (uint8_t*)&dosfs_sflash_device.cache_data[0][0][0];
    info_head = (uint32_t*)&info[DOSFS_SFLASH_INFO_HEAD_OFFSET];
    info_tail = (uint32_t*)&info[DOSFS_SFLASH_INFO_TAIL_OFFSET];

    reclaim_offset = DOSFS_SFLASH_DATA_OFFSET_UNDEFINED;
    victim_offset = DOSFS_SFLASH_DATA_OFFSET_UNDEFINED;
    xlate_slot = DOSFS_SFLASH_XLATE_SLOT_UNDEFINED;
    
    for (erase_count_max = 0, offset = 0; offset < dosfs_sflash_device.data_size; offset += DOSFS_SFLASH_BLOCK_SIZE)
    {
	dosfs_sflash_nor_read(offset, (uint8_t*)&info[0], DOSFS_SFLASH_SECTOR_SIZE);

	if ((info_head[DOSFS_SFLASH_INFO_HEAD_MAGIC] != DOSFS_SFLASH_MAGIC_HEAD) ||
	    (info_head[DOSFS_SFLASH_INFO_HEAD_DATA_START] != dosfs_sflash_device.data_start) ||
	    (info_head[DOSFS_SFLASH_INFO_HEAD_DATA_SIZE] != dosfs_sflash_device.data_size) ||
	    (info_tail[DOSFS_SFLASH_INFO_TAIL_MAGIC] != DOSFS_SFLASH_MAGIC_TAIL))
	{
	    if (victim_offset != DOSFS_SFLASH_DATA_OFFSET_UNDEFINED)
	    {
		return false;
	    }

	    victim_offset = offset;
	}
	else
	{
	    if (erase_count_max < info_head[DOSFS_SFLASH_INFO_HEAD_ERASE_COUNT])
	    {
		erase_count_max = info_head[DOSFS_SFLASH_INFO_HEAD_ERASE_COUNT];
	    }

	    info_tag_data = dosfs_sflash_dir_tag_data_extract(info, 0);

	    if ((info_tag_data & DOSFS_SFLASH_DIR_TAG_TYPE_MASK) == DOSFS_SFLASH_DIR_TAG_TYPE_RECLAIM)
	    {
		if (dosfs_sflash_device.reclaim_offset != DOSFS_SFLASH_DATA_OFFSET_UNDEFINED)
		{
		    return false;
		}
		
		dosfs_sflash_device.reclaim_offset = offset;
	    }

	    else if ((info_tag_data & DOSFS_SFLASH_DIR_TAG_TYPE_MASK) == DOSFS_SFLASH_DIR_TAG_TYPE_INFO)
	    {
		block = info_tag_data & DOSFS_SFLASH_DIR_DATA_MASK;

		if (block >= dosfs_sflash_device.block_count)
		{
		    return false;
		}
		
		if (info_tag_data & DOSFS_SFLASH_DIR_TAG_STATE_NOT_COMMITTED)
		{
		    if (reclaim_offset != DOSFS_SFLASH_DATA_OFFSET_UNDEFINED)
		    {
			return false;
		    }

		    reclaim_offset = offset;
		    reclaim_block = block;
		}
		else
		{
		    dosfs_sflash_block_assign(block, offset);

		    for (alloc_count = 0, sector = 1; sector < (DOSFS_SFLASH_BLOCK_SIZE / DOSFS_SFLASH_SECTOR_SIZE); sector++)
		    {
			if (info[DOSFS_SFLASH_INFO_RECLAIM_OFFSET + (sector / 8)] & (1 << (sector & 7)))
			{
			    info_tag_data = dosfs_sflash_dir_tag_data_extract(info, sector);

			    if (info_tag_data != DOSFS_SFLASH_DIR_ENTRY_NOT_WRITTEN)
			    {
				if ((info_tag_data & DOSFS_SFLASH_DIR_TAG_TYPE_MASK) == DOSFS_SFLASH_DIR_TAG_TYPE_XLATE_SECONDARY)
				{
				    slot = info_tag_data & DOSFS_SFLASH_DIR_DATA_MASK;

				    if (slot > DOSFS_SFLASH_XLATE_SLOT_COUNT)
				    {
					return false;
				    }
				    
				    if (dosfs_sflash_device.xlate_slot != DOSFS_SFLASH_XLATE_SLOT_UNDEFINED)
				    {
					return false;
				    }
				
				    dosfs_sflash_device.xlate_slot = slot;
				    dosfs_sflash_device.xlate_entry = (block << DOSFS_SFLASH_ENTRY_BLOCK_SHIFT) | (sector << DOSFS_SFLASH_ENTRY_SECTOR_SHIFT);
				}

				else if ((info_tag_data & DOSFS_SFLASH_DIR_TAG_TYPE_MASK) == DOSFS_SFLASH_DIR_TAG_TYPE_XLATE_PRIMARY)
				{
				    slot = info_tag_data & DOSFS_SFLASH_DIR_DATA_MASK;

				    if (slot > DOSFS_SFLASH_XLATE_SLOT_COUNT)
				    {
					return false;
				    }

				    if (info_tag_data & DOSFS_SFLASH_DIR_TAG_STATE_NOT_COMMITTED)
				    {
					if (xlate_slot != DOSFS_SFLASH_XLATE_SLOT_UNDEFINED)
					{
					    return false;
					}
					
					xlate_slot = slot;
					xlate_entry = (block << DOSFS_SFLASH_ENTRY_BLOCK_SHIFT) | (sector << DOSFS_SFLASH_ENTRY_SECTOR_SHIFT);
				    }
				    else
				    {
					if (dosfs_sflash_device.xlate_table[slot] != DOSFS_SFLASH_XLATE_SLOT_UNDEFINED)
					{
					    return false;
					}
					
					dosfs_sflash_device.xlate_table[slot] = (block << DOSFS_SFLASH_ENTRY_BLOCK_SHIFT) | (sector << DOSFS_SFLASH_ENTRY_SECTOR_SHIFT);
				    }
				}

				else if ((info_tag_data & DOSFS_SFLASH_DIR_TAG_TYPE_MASK) == DOSFS_SFLASH_DIR_TAG_TYPE_DATA)
				{
				    address = info_tag_data & DOSFS_SFLASH_DIR_DATA_MASK;

				    if (address >= dosfs_sflash_device.sector_count)
				    {
					return false;
				    }
				    
				    if (info_tag_data & DOSFS_SFLASH_DIR_TAG_STATE_NOT_COMMITTED)
				    {
					info[DOSFS_SFLASH_INFO_RECLAIM_OFFSET + (sector / 8)] &= ~(1 << (sector & 7));
				
					dosfs_sflash_nor_program(offset + DOSFS_SFLASH_INFO_RECLAIM_OFFSET + (sector / 8), &info[DOSFS_SFLASH_INFO_RECLAIM_OFFSET + (sector / 8)], 1);
				    }
				    else
				    {
					if (address < DOSFS_SFLASH_XLATE_OFFSET)
					{
					    if (dosfs_sflash_device.xlate_direct[address] != DOSFS_SFLASH_XLATE_SLOT_UNDEFINED)
					    {
						return false;
					    }
					    
					    dosfs_sflash_device.xlate_direct[address] = (block << DOSFS_SFLASH_ENTRY_BLOCK_SHIFT) | (sector << DOSFS_SFLASH_ENTRY_SECTOR_SHIFT);
					}
				    }
				}
				else
				{
				    return false;
				}
			    }
			    else
			    {
				alloc_count++;
			    }
			}
		    }

		    if (!alloc_count)
		    {
			info_tag_data = dosfs_sflash_dir_tag_data_extract(info, 0);

			if (info_tag_data & DOSFS_SFLASH_DIR_TAG_STATE_NOT_WRITTEN)
			{
			    dosfs_sflash_dir_tag_write((block << DOSFS_SFLASH_ENTRY_BLOCK_SHIFT), ~(DOSFS_SFLASH_DIR_TAG_STATE_NOT_WRITTEN));
			}
		    }

		    dosfs_sflash_device.erase_count += alloc_count;
		}
	    }
	}
    }

    if (reclaim_offset != DOSFS_SFLASH_DATA_OFFSET_UNDEFINED)
    {
	victim_erase_count = erase_count_max;

	if (victim_offset == DOSFS_SFLASH_DATA_OFFSET_UNDEFINED)
	{
	    victim_offset = dosfs_sflash_block_offset(reclaim_block);

	    if (victim_offset != DOSFS_SFLASH_DATA_OFFSET_UNDEFINED)
	    {
		dosfs_sflash_nor_read(victim_offset + DOSFS_SFLASH_INFO_ERASE_COUNT_OFFSET, (uint8_t*)&victim_erase_count, DOSFS_SFLASH_INFO_ERASE_COUNT_SIZE);
		victim_erase_count++;
	    }
	}
	
	if (victim_offset != DOSFS_SFLASH_DATA_OFFSET_UNDEFINED)
	{
	    info_head[DOSFS_SFLASH_INFO_HEAD_MAGIC] = DOSFS_SFLASH_MAGIC_HEAD;
	    info_head[DOSFS_SFLASH_INFO_HEAD_DATA_START] = dosfs_sflash_device.data_start;
	    info_head[DOSFS_SFLASH_INFO_HEAD_DATA_SIZE] = dosfs_sflash_device.data_size;
	    info_head[DOSFS_SFLASH_INFO_HEAD_ERASE_COUNT] = victim_erase_count;
	
	    info_tag_data = DOSFS_SFLASH_DIR_TAG_TYPE_RECLAIM | DOSFS_SFLASH_DIR_TAG_STATE_NOT_COMMITTED | DOSFS_SFLASH_DIR_TAG_STATE_NOT_WRITTEN | DOSFS_SFLASH_BLOCK_UNDEFINED;
	    
	    info_tail[DOSFS_SFLASH_INFO_TAIL_MAGIC] = DOSFS_SFLASH_MAGIC_TAIL;
	    
	    dosfs_sflash_nor_erase(victim_offset);
	    
	    dosfs_sflash_nor_program(victim_offset + DOSFS_SFLASH_INFO_HEAD_OFFSET, (const uint8_t*)&info_head[0], DOSFS_SFLASH_INFO_HEAD_SIZE);
	    dosfs_sflash_nor_program(victim_offset + DOSFS_SFLASH_INFO_DIR_OFFSET, (const uint8_t*)&info_tag_data, DOSFS_SFLASH_DIR_ENTRY_SIZE);
	    dosfs_sflash_nor_program(victim_offset + DOSFS_SFLASH_INFO_TAIL_OFFSET, (const uint8_t*)&info_tail[0], DOSFS_SFLASH_INFO_TAIL_SIZE);
	    
	    dosfs_sflash_device.reclaim_offset = victim_offset;
	}

    	info_tag = ~DOSFS_SFLASH_DIR_TAG_STATE_NOT_COMMITTED >> DOSFS_SFLASH_DIR_TAG_SHIFT;
	dosfs_sflash_nor_program(reclaim_offset + DOSFS_SFLASH_INFO_DIR_OFFSET + DOSFS_SFLASH_DIR_TAG_OFFSET, (uint8_t*)&info_tag, DOSFS_SFLASH_DIR_TAG_SIZE);

	dosfs_sflash_block_assign(reclaim_block, reclaim_offset);
    }

    if (xlate_slot != DOSFS_SFLASH_XLATE_SLOT_UNDEFINED)
    {
	if (dosfs_sflash_device.xlate_slot != DOSFS_SFLASH_XLATE_SLOT_UNDEFINED)
	{
	    return false;
	}

	dosfs_sflash_device.xlate_slot = xlate_slot;
	dosfs_sflash_device.xlate_entry = xlate_entry;

	dosfs_sflash_ftl_merge(dosfs_sflash_device.xlate_table[xlate_slot]);
    }

    if (dosfs_sflash_device.xlate_slot != DOSFS_SFLASH_XLATE_SLOT_UNDEFINED)
    {
	dosfs_sflash_ftl_verify(dosfs_sflash_device.xlate_entry);
    }

    for (slot = 0; slot < DOSFS_SFLASH_XLATE_SLOT_COUNT; slot++)
    {
	xlate_entry = dosfs_sflash_device.xlate_table[slot];
	
	if (xlate_entry != DOSFS_SFLASH_XLATE_ENTRY_UNDEFINED)
	{
	    dosfs_sflash_ftl_verify(xlate_entry);
	}
    }
    
    return true;
}

static void dosfs_sflash_ftl_stop()
{
    const stm32wb_sflash_interface_t *interface = dosfs_sflash_device.interface;
    
    (*interface->disable)();
}

static int dosfs_sflash_start(void *context, uint8_t *p_media, uint8_t *p_write_protected, uint32_t *p_block_count, uint32_t *p_au_size, uint32_t *p_serial)
{
    const stm32wb_sflash_interface_t *interface = dosfs_sflash_device.interface;
    int status = F_NO_ERROR;

    if (dosfs_sflash_device.state <= DOSFS_SFLASH_STATE_NOT_READY)
    {
        status = F_ERR_ONDRIVE;
    }
    else
    {
        (*interface->enable)();
        
        *p_media = DOSFS_MEDIA_SFLASH;
        *p_write_protected = false;
        *p_block_count = dosfs_sflash_device.sector_count;
        *p_au_size = 8;
	*p_serial = dosfs_sflash_device.serial;
    }
    
    return status;
}

static int dosfs_sflash_stop(void *context, bool eject)
{
    int status = F_NO_ERROR;

    dosfs_sflash_ftl_stop();

    return status;
}

static int dosfs_sflash_format(void *context, uint32_t size)
{
    int status = F_NO_ERROR;

    if (dosfs_sflash_device.state < DOSFS_SFLASH_STATE_NOT_READY)
    {
        status = F_ERR_ONDRIVE;
    }
    else
    {
        dosfs_sflash_ftl_stop();

	if (size > dosfs_sflash_device.info->capacity)
	{
	    size = dosfs_sflash_device.info->capacity;
	}

	if (size > DOSFS_SFLASH_DATA_SIZE)
	{
	    size = DOSFS_SFLASH_DATA_SIZE;
	}
	
	if (size <= DOSFS_SFLASH_DATA_SIZE_MIN)
	{
	    size = 0;
	}
	
	size = (size + (DOSFS_SFLASH_BLOCK_SIZE -1)) & ~(DOSFS_SFLASH_BLOCK_SIZE -1);

	dosfs_sflash_device.data_start = dosfs_sflash_device.info->capacity - size;
	dosfs_sflash_device.data_size = size;
	
	stm32wb_sflash_resize(dosfs_sflash_device.data_start);
	
	if (!dosfs_sflash_device.data_size)
	{
            dosfs_sflash_device.state = DOSFS_SFLASH_STATE_NOT_READY;

	    status = F_ERR_INVALIDMEDIA;
	}
	else
	{
	    dosfs_sflash_nor_acquire();

	    dosfs_sflash_ftl_format();

	    if (!dosfs_sflash_ftl_start())
	    {
		dosfs_sflash_device.state = DOSFS_SFLASH_STATE_NOT_READY;
		
		status = F_ERR_ONDRIVE;
	    }
	    else
	    {
		dosfs_sflash_device.state = DOSFS_SFLASH_STATE_READY;
	    }
	}
	
        dosfs_sflash_nor_release();
    }
    
    return status;
}

static int dosfs_sflash_erase(void *context, uint32_t address, uint32_t length)
{
    return F_NO_ERROR;
}

static int dosfs_sflash_discard(void *context, uint32_t address, uint32_t length)
{
    int status = F_NO_ERROR;

    if (dosfs_sflash_device.state != DOSFS_SFLASH_STATE_READY)
    {
        status = F_ERR_ONDRIVE;
    }
    else
    {
        dosfs_sflash_nor_acquire();

        while (length--)
        {
	    dosfs_sflash_ftl_discard(address);

            address++;
        }

        dosfs_sflash_nor_release();
    }

    return status;
}

static int dosfs_sflash_read(void *context, uint32_t address, uint8_t *data, uint32_t length)
{
    int status = F_NO_ERROR;
    uint32_t entry;
    
    if (dosfs_sflash_device.state != DOSFS_SFLASH_STATE_READY)
    {
        status = F_ERR_ONDRIVE;
    }
    else
    {
        dosfs_sflash_nor_acquire();

        while (length--)
        {
	    entry = dosfs_sflash_ftl_lookup(address);
	    
	    if (entry != DOSFS_SFLASH_ENTRY_UNDEFINED)
	    {
		dosfs_sflash_nor_read(dosfs_sflash_ftl_offset(entry), &data[0], DOSFS_SFLASH_SECTOR_SIZE);
	    }
	    else
	    {
		memset(&data[0], 0xff, DOSFS_SFLASH_SECTOR_SIZE);
	    }
	    
	    data += DOSFS_SFLASH_SECTOR_SIZE;

            address++;
        }

        dosfs_sflash_nor_release();
    }

    return status;
}

static int dosfs_sflash_write(void *context, uint32_t address, const uint8_t *data, uint32_t length, bool sync)
{
    int status = F_NO_ERROR;
    uint32_t entry;
    
    if (dosfs_sflash_device.state != DOSFS_SFLASH_STATE_READY)
    {
        status = F_ERR_ONDRIVE;
    }
    else
    {
        dosfs_sflash_nor_acquire();

        while (length--)
        {
	    dosfs_sflash_ftl_reclaim();

	    entry = dosfs_sflash_ftl_allocate(DOSFS_SFLASH_DIR_TAG_TYPE_DATA | DOSFS_SFLASH_DIR_TAG_STATE_NOT_COMMITTED | address);

	    dosfs_sflash_ftl_replace(address, entry);

	    dosfs_sflash_nor_program(dosfs_sflash_ftl_offset(entry), &data[0], DOSFS_SFLASH_PAGE_SIZE);
	    dosfs_sflash_nor_program(dosfs_sflash_ftl_offset(entry) + DOSFS_SFLASH_PAGE_SIZE, &data[DOSFS_SFLASH_PAGE_SIZE], DOSFS_SFLASH_PAGE_SIZE);
	    
	    data += DOSFS_SFLASH_SECTOR_SIZE;
	    
            address++;
        }

        dosfs_sflash_nor_release();
    }

    return status;
}

static int dosfs_sflash_sync(void *context)
{
    int status = F_NO_ERROR;

    if (dosfs_sflash_device.state != DOSFS_SFLASH_STATE_READY)
    {
        status = F_ERR_ONDRIVE;
    }

    return status;
}

static const dosfs_device_interface_t dosfs_sflash_interface = {
    dosfs_sflash_start,
    dosfs_sflash_stop,
    dosfs_sflash_format,
    dosfs_sflash_erase,
    dosfs_sflash_discard,
    dosfs_sflash_read,
    dosfs_sflash_write,
    dosfs_sflash_sync,
};

void dosfs_sflash_initialize(void)
{
    uint32_t start, size;

    if (dosfs_sflash_device.state == DOSFS_SFLASH_STATE_READY)
    {
	return;
    }
    
    if (!stm32wb_sflash_query(&dosfs_sflash_device.interface, &dosfs_sflash_device.info, &start))
    {
	return;
    }

    size = dosfs_sflash_device.info->capacity - start;

    if (size < DOSFS_SFLASH_DATA_SIZE_MIN)
    {
	size = 0;
    }

    dosfs_device.lock = DOSFS_DEVICE_LOCK_INIT;
    dosfs_device.context = (void*)&dosfs_sflash_device;
    dosfs_device.interface = &dosfs_sflash_interface;
    
    dosfs_sflash_device.serial = 0;
    dosfs_sflash_device.data_start = start;
    dosfs_sflash_device.data_size = size;
    
    dosfs_sflash_nor_acquire();

    if (dosfs_sflash_ftl_start())
    {
	dosfs_sflash_device.state = DOSFS_SFLASH_STATE_READY;
    }
    else
    {
	dosfs_sflash_device.state = DOSFS_SFLASH_STATE_NOT_READY;
    }

    dosfs_sflash_nor_release();
        
    dosfs_device.lock = 0;
}
