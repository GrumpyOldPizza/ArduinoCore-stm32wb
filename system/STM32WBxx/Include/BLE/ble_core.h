/*****************************************************************************
 * @file    ble_core.h
 * @author  MDG
 * @brief   This file contains the definitions for BLE stack
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

#ifndef BLE_CORE_H__
#define BLE_CORE_H__


/* BLE standard definitions */
#include "BLE/ble_std.h"

/* BLE stack API definitions */
#include "BLE/ble_defs.h"
#include "BLE/ble_gap_aci.h"
#include "BLE/ble_gatt_aci.h"
#include "BLE/ble_hal_aci.h"
#include "BLE/ble_hci_le.h"
#include "BLE/ble_l2cap_aci.h"
#include "BLE/ble_events.h"

/* BLE stack buffer size definitions */
#include "BLE/ble_bufsize.h"

/* BLE stack legacy definitions */
#include "BLE/ble_legacy.h"


#endif /* BLE_CORE_H__ */
