/*****************************************************************************
 * @file    ble_const.h
 * @author  MDG
 * @brief   This file contains the definitions which are compiler dependent.
 *****************************************************************************
 * @attention
 *
 * Copyright (c) 2018-2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 *****************************************************************************
 */

#ifndef BLE_CONST_H__
#define BLE_CONST_H__


#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "BLE/ble_std.h"
#include "BLE/ble_defs.h"

#include "stm32wb_ipcc.h"

/* Default BLE variant */
#ifndef BASIC_FEATURES
#define BASIC_FEATURES 0
#endif
#ifndef SLAVE_ONLY
#define SLAVE_ONLY 0
#endif
#ifndef LL_ONLY
#define LL_ONLY 0
#endif
#ifndef LL_ONLY_BASIC
#define LL_ONLY_BASIC 0
#endif
#ifndef BEACON_ONLY
#define BEACON_ONLY 0
#endif



/* Size of command/events buffers:
 *
 * To change the size of commands and events parameters used in the
 * auto-generated files, you need to update 2 defines:
 *
 *  - BLE_CMD_MAX_PARAM_LEN          
 *  - BLE_EVT_MAX_PARAM_LEN          
 *
 * These 2 defines are set below with default values and can be changed.
 *
 * To compute the value to support a characteristic of 512 bytes for a specific
 * command or an event, you need to look in "ble_types.h".
 *
 * Here are 2 examples, one with a command and one with an event:
 *
 * - aci_gatt_update_char_value_ext_cp0
 *   ----------------------------------
 *
 *   we have in the structure:
 *
 *      uint8_t Value[(BLE_CMD_MAX_PARAM_LEN- 12)/sizeof(uint8_t)];
 *
 *   so to support a 512 byte value, we need to have
 *
 *   BLE_CMD_MAX_PARAM_LEN at least equal to: 512 + 12 = 524
 *
 * - aci_gatt_read_handle_value_rp0
 *   ------------------------------
 *
 *   we have in the structure:
 *
 *     uint8_t Value[((BLE_EVT_MAX_PARAM_LEN - 3) - 5)/sizeof(uint8_t)];
 *
 *   so to support a 512 byte value, we need to have
 *
 *   BLE_EVT_MAX_PARAM_LEN at least equal to: 512 + 3 + 5 = 520
 *
 * If you need several events or commands with 512-size values, you need to
 * take the maximum values for BLE_EVT_MAX_PARAM_LEN and BLE_CMD_MAX_PARAM_LEN.
 *
 */

/* Maximum parameter size of BLE commands.
 * Change this value if needed. */
#define BLE_CMD_MAX_PARAM_LEN          HCI_COMMAND_MAX_PARAM_LEN

/* Maximum parameter size of BLE responses/events.
 * Change this value if needed. */
#define BLE_EVT_MAX_PARAM_LEN          HCI_EVENT_MAX_PARAM_LEN

/* Callback function to send command and receive response */
#define hci_request _stm32wb_ipcc_ble_command_t 

extern int hci_send_req( struct hci_request* req, bool async );


/**
 * This function copies size number of bytes from a 
 * memory location pointed by src to a destination 
 * memory location pointed by dest
 * 
 * @param[in] dest Destination address
 * @param[in] src  Source address
 * @param[in] size size in the bytes  
 * 
 * @return  Address of the destination
 */

#define Osal_MemCpy(_dest, _src, _size) memcpy((_dest), (_src), (_size))

/**
 * This function sets first number of bytes, specified
 * by size, to the destination memory pointed by ptr 
 * to the specified value
 * 
 * @param[in] ptr    Destination address
 * @param[in] value  Value to be set
 * @param[in] size   Size in the bytes  
 * 
 * @return  Address of the destination
 */
 
#define Osal_MemSet(_ptr, _value, _size) memset((_ptr), (_value), (_size))

/**
 * This function compares n bytes of two regions of memory
 * 
 * @param[in] s1    First buffer to compare.
 * @param[in] s2    Second buffer to compare.
 * @param[in] size  Number of bytes to compare.   
 * 
 * @return  0 if the two buffers are equal, 1 otherwise
 */
#define Osal_MemCmp(_s1, _s2, _size) memcmp((_s1), (_s2), (_size))



#ifndef FALSE
#define FALSE 0
#endif

#ifndef MIN
#define MIN( a, b )            (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define MAX( a, b )            (((a) > (b)) ? (a) : (b))
#endif

#ifndef DIVC
#define DIVC( x, y )           (((x)+(y)-1)/(y))
#endif


#ifndef __WEAK
#define __WEAK __attribute__((weak))
#endif

/**
 * @brief  PACKED
 *         Use the PACKED macro for variables that needs to be packed.
 *         Usage:  PACKED(struct) myStruct_s
 *                 PACKED(union) myStruct_s
 */
#define PACKED(decl)                    decl __attribute__((packed))

/**
 * @brief  SECTION
 *         Use the SECTION macro to assign data or code in a specific section.
 *         Usage:  SECTION(".my_section")
 */
#define SECTION(name)                   __attribute__((section(name)))

/**
 * @brief  ALIGN_DEF
 *         Use the ALIGN_DEF macro to specify the alignment of a variable.
 *         Usage:  ALIGN_DEF(4)
 */
#define ALIGN_DEF(N)                    __attribute__((aligned(N)))

/**
 * @brief  NO_INIT
 *         Use the NO_INIT macro to declare a not initialized variable.
 *         Usage:  NO_INIT(int my_no_init_var)
 *         Usage:  NO_INIT(uint16_t my_no_init_array[10])
 */
#define NO_INIT(var)                    var  __attribute__((section(".noinit")))

#endif /* BLE_CONST_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE***/
