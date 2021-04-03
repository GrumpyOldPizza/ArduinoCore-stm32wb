/*
 * Copyright (c) 2020-2021 Thomas Roell.  All rights reserved.
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
#include "stm32wbxx.h"

#include "stm32wb_ipcc.h"
#include "stm32wb_flash.h"
#include "stm32wb_system.h"
#include "stm32wb_gpio.h"

#include "BLE/ble_defs.h"

#define STM32WB_IPCC_TRACE_SUPPORTED 0

#if (STM32WB_IPCC_TRACE_SUPPORTED == 1)

#include <stdio.h>

typedef struct _stm32wb_hci_command_table_t {
    uint16_t opcode;
    const char *cstring;
} stm32wb_hci_command_table_t;

typedef struct _stm32wb_hci_event_table_t {
    uint16_t evtcode;
    const char *cstring;
} stm32wb_hci_event_table_t;

const stm32wb_hci_command_table_t hci_command_table[] = {
    { 0x0406, "HCI_DISCONNECT" },
    { 0x041D, "HCI_READ_REMOTE_VERSION_INFORMATION" },
    { 0x0C01, "HCI_SET_EVENT_MASK" },
    { 0x0C03, "HCI_RESET" },
    { 0x0C2D, "HCI_READ_TRANSMIT_POWER_LEVEL" },
    { 0x0C31, "HCI_SET_CONTROLLER_TO_HOST_FLOW_CONTROL" },
    { 0x0C33, "HCI_HOST_BUFFER_SIZE" },
    { 0x0C35, "HCI_HOST_NUMBER_OF_COMPLETED_PACKETS" },
    { 0x1001, "HCI_READ_LOCAL_VERSION_INFORMATION" },
    { 0x1002, "HCI_READ_LOCAL_SUPPORTED_COMMANDS" },
    { 0x1003, "HCI_READ_LOCAL_SUPPORTED_FEATURES" },
    { 0x1009, "HCI_READ_BD_ADDR" },
    { 0x1405, "HCI_READ_RSSI" },
    { 0x2001, "HCI_LE_SET_EVENT_MASK" },
    { 0x2002, "HCI_LE_READ_BUFFER_SIZE" },
    { 0x2003, "HCI_LE_READ_LOCAL_SUPPORTED_FEATURES" },
    { 0x2005, "HCI_LE_SET_RANDOM_ADDRESS" },
    { 0x2006, "HCI_LE_SET_ADVERTISING_PARAMETERS" },
    { 0x2007, "HCI_LE_READ_ADVERTISING_CHANNEL_TX_POWER" },
    { 0x2008, "HCI_LE_SET_ADVERTISING_DATA" },
    { 0x2009, "HCI_LE_SET_SCAN_RESPONSE_DATA" },
    { 0x200A, "HCI_LE_SET_ADVERTISE_ENABLE" },
    { 0x200B, "HCI_LE_SET_SCAN_PARAMETERS" },
    { 0x200C, "HCI_LE_SET_SCAN_ENABLE" },
    { 0x200D, "HCI_LE_CREATE_CONNECTION" },
    { 0x200E, "HCI_LE_CREATE_CONNECTION_CANCEL" },
    { 0x200F, "HCI_LE_READ_WHITE_LIST_SIZE" },
    { 0x2010, "HCI_LE_CLEAR_WHITE_LIST" },
    { 0x2011, "HCI_LE_ADD_DEVICE_TO_WHITE_LIST" },
    { 0x2012, "HCI_LE_REMOVE_DEVICE_FROM_WHITE_LIST" },
    { 0x2013, "HCI_LE_CONNECTION_UPDATE" },
    { 0x2014, "HCI_LE_SET_HOST_CHANNEL_CLASSIFICATION" },
    { 0x2015, "HCI_LE_READ_CHANNEL_MAP" },
    { 0x2016, "HCI_LE_READ_REMOTE_FEATURES" },
    { 0x2017, "HCI_LE_ENCRYPT" },
    { 0x2018, "HCI_LE_RAND" },
    { 0x2019, "HCI_LE_START_ENCRYPTION" },
    { 0x201A, "HCI_LE_LONG_TERM_KEY_REQUEST_REPLY" },
    { 0x201B, "HCI_LE_LONG_TERM_KEY_REQUESTED_NEGATIVE_REPLY" },
    { 0x201C, "HCI_LE_READ_SUPPORTED_STATES" },
    { 0x2022, "HCI_LE_SET_DATA_LENGTH" },
    { 0x2023, "HCI_LE_READ_SUGGESTED_DEFAULT_DATA_LENGTH" },
    { 0x2024, "HCI_LE_WRITE_SUGGESTED_DEFAULT_DATA_LENGTH" },
    { 0x2025, "HCI_LE_READ_LOCAL_P256_PUBLIC_KEY" },
    { 0x2026, "HCI_LE_GENERATE_DHKEY" },
    { 0x2027, "HCI_LE_ADD_DEVICE_TO_RESOLVING_LIST" },
    { 0x2028, "HCI_LE_REMOVE_DEVICE_FROM_RESOLVING_LIST" },
    { 0x2029, "HCI_LE_CLEAR_RESOLVING_LIST" },
    { 0x202A, "HCI_LE_READ_RESOLVING_LIST_SIZE" },
    { 0x202B, "HCI_LE_READ_PEER_RESOLVABLE_ADDRESS" },
    { 0x202C, "HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS" },
    { 0x202D, "HCI_LE_SET_ADDRESS_RESOLUTION_ENABLE" },
    { 0x202E, "HCI_LE_SET_RESOLVABLE_PRIVATE_ADDRESS_TIMEOUT" },
    { 0x202F, "HCI_LE_READ_MAXIMUM_DATA_LENGTH" },
    { 0x2030, "HCI_LE_READ_PHY" },
    { 0x2031, "HCI_LE_SET_DEFAULT_PHY" },
    { 0x2032, "HCI_LE_SET_PHY" },
    { 0x204E, "HCI_LE_SET_PRIVACY_MODE" },
    { 0x201D, "HCI_LE_RECEIVER_TEST" },
    { 0x201E, "HCI_LE_TRANSMITTER_TEST" },
    { 0x201F, "HCI_LE_TEST_END" },
    { 0x2033, "HCI_LE_ENHANCED_RECEIVER_TEST" },
    { 0x2034, "HCI_LE_ENHANCED_TRANSMITTER_TEST" },
    { 0xFC00, "ACI_HAL_GET_FW_BUILD_NUMBER" },
    { 0xFC0C, "ACI_HAL_WRITE_CONFIG_DATA" },
    { 0xFC0D, "ACI_HAL_READ_CONFIG_DATA" },
    { 0xFC0F, "ACI_HAL_SET_TX_POWER_LEVEL" },
    { 0xFC14, "ACI_HAL_LE_TX_TEST_PACKET_NUMBER" },
    { 0xFC15, "ACI_HAL_TONE_START" },
    { 0xFC16, "ACI_HAL_TONE_STOP" },
    { 0xFC17, "ACI_HAL_GET_LINK_STATUS" },
    { 0xFC18, "ACI_HAL_SET_RADIO_ACTIVITY_MASK" },
    { 0xFC19, "ACI_HAL_GET_ANCHOR_PERIOD" },
    { 0xFC1A, "ACI_HAL_SET_EVENT_MASK" },
    { 0xFC1B, "ACI_HAL_SET_SMP_ENG_CONFIG" },
    { 0xFC1C, "ACI_HAL_GET_PM_DEBUG_INFO" },
    { 0xFC30, "ACI_HAL_READ_RADIO_REG" },
    { 0xFC31, "ACI_HAL_WRITE_RADIO_REG" },
    { 0xFC32, "ACI_HAL_READ_RAW_RSSI" },
    { 0xFC33, "ACI_HAL_RX_START" },
    { 0xFC34, "ACI_HAL_RX_STOP" },
    { 0xFC3B, "ACI_HAL_STACK_RESET" },
    { 0xFC81, "ACI_GAP_SET_NON_DISCOVERABLE" },
    { 0xFC82, "ACI_GAP_SET_LIMITED_DISCOVERABLE" },
    { 0xFC83, "ACI_GAP_SET_DISCOVERABLE" },
    { 0xFC84, "ACI_GAP_SET_DIRECT_CONNECTABLE" },
    { 0xFC85, "ACI_GAP_SET_IO_CAPABILITY" },
    { 0xFC86, "ACI_GAP_SET_AUTHENTICATION_REQUIREMENT" },
    { 0xFC87, "ACI_GAP_SET_AUTHORIZATION_REQUIREMENT" },
    { 0xFC88, "ACI_GAP_PASS_KEY_RESP" },
    { 0xFC89, "ACI_GAP_AUTHORIZATION_RESP" },
    { 0xFC8A, "ACI_GAP_INIT" },
    { 0xFC8B, "ACI_GAP_SET_NON_CONNECTABLE" },
    { 0xFC8C, "ACI_GAP_SET_UNDIRECTED_CONNECTABLE" },
    { 0xFC8D, "ACI_GAP_SLAVE_SECURITY_REQ" },
    { 0xFC8E, "ACI_GAP_UPDATE_ADV_DATA" },
    { 0xFC8F, "ACI_GAP_DELETE_AD_TYPE" },
    { 0xFC90, "ACI_GAP_GET_SECURITY_LEVEL" },
    { 0xFC91, "ACI_GAP_SET_EVENT_MASK" },
    { 0xFC92, "ACI_GAP_CONFIGURE_WHITELIST" },
    { 0xFC93, "ACI_GAP_TERMINATE" },
    { 0xFC94, "ACI_GAP_CLEAR_SECURITY_DB" },
    { 0xFC95, "ACI_GAP_ALLOW_REBOND" },
    { 0xFC96, "ACI_GAP_START_LIMITED_DISCOVERY_PROC" },
    { 0xFC97, "ACI_GAP_START_GENERAL_DISCOVERY_PROC" },
    { 0xFC98, "ACI_GAP_START_NAME_DISCOVERY_PROC" },
    { 0xFC99, "ACI_GAP_START_AUTO_CONNECTION_ESTABLISH_PROC" },
    { 0xFC9A, "ACI_GAP_START_GENERAL_CONNECTION_ESTABLISH_PROC" },
    { 0xFC9B, "ACI_GAP_START_SELECTIVE_CONNECTION_ESTABLISH_PROC" },
    { 0xFC9C, "ACI_GAP_CREATE_CONNECTION" },
    { 0xFC9D, "ACI_GAP_TERMINATE_GAP_PROC" },
    { 0xFC9E, "ACI_GAP_START_CONNECTION_UPDATE" },
    { 0xFC9F, "ACI_GAP_SEND_PAIRING_REQ" },
    { 0xFCA0, "ACI_GAP_RESOLVE_PRIVATE_ADDR" },
    { 0xFCA1, "ACI_GAP_SET_BROADCAST_MODE" },
    { 0xFCA2, "ACI_GAP_START_OBSERVATION_PROC" },
    { 0xFCA3, "ACI_GAP_GET_BONDED_DEVICES" },
    { 0xFCA4, "ACI_GAP_IS_DEVICE_BONDED" },
    { 0xFCA5, "ACI_GAP_NUMERIC_COMPARISON_VALUE_CONFIRM_YESNO" },
    { 0xFCA6, "ACI_GAP_PASSKEY_INPUT" },
    { 0xFCA7, "ACI_GAP_GET_OOB_DATA" },
    { 0xFCA8, "ACI_GAP_SET_OOB_DATA" },
    { 0xFCA9, "ACI_GAP_ADD_DEVICES_TO_RESOLVING_LIST" },
    { 0xFCAA, "ACI_GAP_REMOVE_BONDED_DEVICE" },
    { 0xFD01, "ACI_GATT_INIT" },
    { 0xFD02, "ACI_GATT_ADD_SERVICE" },
    { 0xFD03, "ACI_GATT_INCLUDE_SERVICE" },
    { 0xFD04, "ACI_GATT_ADD_CHAR" },
    { 0xFD05, "ACI_GATT_ADD_CHAR_DESC" },
    { 0xFD06, "ACI_GATT_UPDATE_CHAR_VALUE" },
    { 0xFD07, "ACI_GATT_DEL_CHAR" },
    { 0xFD08, "ACI_GATT_DEL_SERVICE" },
    { 0xFD09, "ACI_GATT_DEL_INCLUDE_SERVICE" },
    { 0xFD0A, "ACI_GATT_SET_EVENT_MASK" },
    { 0xFD0B, "ACI_GATT_EXCHANGE_CONFIG" },
    { 0xFD0C, "ACI_ATT_FIND_INFO_REQ" },
    { 0xFD0D, "ACI_ATT_FIND_BY_TYPE_VALUE_REQ" },
    { 0xFD0E, "ACI_ATT_READ_BY_TYPE_REQ" },
    { 0xFD0F, "ACI_ATT_READ_BY_GROUP_TYPE_REQ" },
    { 0xFD10, "ACI_ATT_PREPARE_WRITE_REQ" },
    { 0xFD11, "ACI_ATT_EXECUTE_WRITE_REQ" },
    { 0xFD12, "ACI_GATT_DISC_ALL_PRIMARY_SERVICES" },
    { 0xFD13, "ACI_GATT_DISC_PRIMARY_SERVICE_BY_UUID" },
    { 0xFD14, "ACI_GATT_FIND_INCLUDED_SERVICES" },
    { 0xFD15, "ACI_GATT_DISC_ALL_CHAR_OF_SERVICE" },
    { 0xFD16, "ACI_GATT_DISC_CHAR_BY_UUID" },
    { 0xFD17, "ACI_GATT_DISC_ALL_CHAR_DESC" },
    { 0xFD18, "ACI_GATT_READ_CHAR_VALUE" },
    { 0xFD19, "ACI_GATT_READ_USING_CHAR_UUID" },
    { 0xFD1A, "ACI_GATT_READ_LONG_CHAR_VALUE" },
    { 0xFD1B, "ACI_GATT_READ_MULTIPLE_CHAR_VALUE" },
    { 0xFD1C, "ACI_GATT_WRITE_CHAR_VALUE" },
    { 0xFD1D, "ACI_GATT_WRITE_LONG_CHAR_VALUE" },
    { 0xFD1E, "ACI_GATT_WRITE_CHAR_RELIABLE" },
    { 0xFD1F, "ACI_GATT_WRITE_LONG_CHAR_DESC" },
    { 0xFD20, "ACI_GATT_READ_LONG_CHAR_DESC" },
    { 0xFD21, "ACI_GATT_WRITE_CHAR_DESC" },
    { 0xFD22, "ACI_GATT_READ_CHAR_DESC" },
    { 0xFD23, "ACI_GATT_WRITE_WITHOUT_RESP" },
    { 0xFD24, "ACI_GATT_SIGNED_WRITE_WITHOUT_RESP" },
    { 0xFD25, "ACI_GATT_CONFIRM_INDICATION" },
    { 0xFD26, "ACI_GATT_WRITE_RESP" },
    { 0xFD27, "ACI_GATT_ALLOW_READ" },
    { 0xFD28, "ACI_GATT_SET_SECURITY_PERMISSION" },
    { 0xFD29, "ACI_GATT_SET_DESC_VALUE" },
    { 0xFD2A, "ACI_GATT_READ_HANDLE_VALUE" },
    { 0xFD2C, "ACI_GATT_UPDATE_CHAR_VALUE_EXT" },
    { 0xFD2D, "ACI_GATT_DENY_READ" },
    { 0xFD2E, "ACI_GATT_SET_ACCESS_PERMISSION" },
    { 0xFD81, "ACI_L2CAP_CONNECTION_PARAMETER_UPDATE_REQ" },
    { 0xFD82, "ACI_L2CAP_CONNECTION_PARAMETER_UPDATE_RESP" },

};

const stm32wb_hci_event_table_t hci_event_table[] = {
    { 0X0005, "HCI_DISCONNECTION_COMPLETE_EVENT" },
    { 0X0008, "HCI_ENCRYPTION_CHANGE_EVENT" },
    { 0X000C, "HCI_READ_REMOTE_VERSION_INFORMATION_COMPLETE_EVENT" },
    { 0X000E, "HCI_COMMAND_COMPLETE_EVENT" },
    { 0X000F, "HCI_COMMAND_STATUS_EVENT" },
    { 0X0010, "HCI_HARDWARE_ERROR_EVENT" },
    { 0X0013, "HCI_NUMBER_OF_COMPLETED_PACKETS_EVENT" },
    { 0X0030, "HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT" },
};

const stm32wb_hci_event_table_t hci_le_event_table[] = {
    { 0X0001, "HCI_LE_CONNECTION_COMPLETE_EVENT" },
    { 0X0002, "HCI_LE_ADVERTISING_REPORT_EVENT" },
    { 0X0003, "HCI_LE_CONNECTION_UPDATE_COMPLETE_EVENT" },
    { 0X0004, "HCI_LE_READ_REMOTE_FEATURES_COMPLETE_EVENT" },
    { 0X0005, "HCI_LE_LONG_TERM_KEY_REQUEST_EVENT" },
    { 0X0007, "HCI_LE_DATA_LENGTH_CHANGE_EVENT" },
    { 0X0008, "HCI_LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE_EVENT" },
    { 0X0009, "HCI_LE_GENERATE_DHKEY_COMPLETE_EVENT" },
    { 0X000A, "HCI_LE_ENHANCED_CONNECTION_COMPLETE_EVENT" },
    { 0X000B, "HCI_LE_DIRECT_ADVERTISING_REPORT_EVENT" },
    { 0X000C, "HCI_LE_PHY_UPDATE_COMPLETE_EVENT" },
};

const stm32wb_hci_event_table_t hci_vs_event_table[] = {
    { 0X0004, "ACI_HAL_END_OF_RADIO_ACTIVITY_EVENT" },
    { 0X0005, "ACI_HAL_SCAN_REQ_REPORT_EVENT" },
    { 0X0006, "ACI_HAL_FW_ERROR_EVENT" },
    { 0X0400, "ACI_GAP_LIMITED_DISCOVERABLE_EVENT" },
    { 0X0401, "ACI_GAP_PAIRING_COMPLETE_EVENT" },
    { 0X0402, "ACI_GAP_PASS_KEY_REQ_EVENT" },
    { 0X0403, "ACI_GAP_AUTHORIZATION_REQ_EVENT" },
    { 0X0404, "ACI_GAP_SLAVE_SECURITY_INITIATED_EVENT" },
    { 0X0405, "ACI_GAP_BOND_LOST_EVENT" },
    { 0X0407, "ACI_GAP_PROC_COMPLETE_EVENT" },
    { 0X0408, "ACI_GAP_ADDR_NOT_RESOLVED_EVENT" },
    { 0X0409, "ACI_GAP_NUMERIC_COMPARISON_VALUE_EVENT" },
    { 0X040A, "ACI_GAP_KEYPRESS_NOTIFICATION_EVENT" },
    { 0X0800, "ACI_L2CAP_CONNECTION_UPDATE_RESP_EVENT" },
    { 0X0801, "ACI_L2CAP_PROC_TIMEOUT_EVENT" },
    { 0X0802, "ACI_L2CAP_CONNECTION_UPDATE_REQ_EVENT" },
    { 0X080A, "ACI_L2CAP_COMMAND_REJECT_EVENT" },
    { 0X0C01, "ACI_GATT_ATTRIBUTE_MODIFIED_EVENT" },
    { 0X0C02, "ACI_GATT_PROC_TIMEOUT_EVENT" },
    { 0X0C03, "ACI_ATT_EXCHANGE_MTU_RESP_EVENT" },
    { 0X0C04, "ACI_ATT_FIND_INFO_RESP_EVENT" },
    { 0X0C05, "ACI_ATT_FIND_BY_TYPE_VALUE_RESP_EVENT" },
    { 0X0C06, "ACI_ATT_READ_BY_TYPE_RESP_EVENT" },
    { 0X0C07, "ACI_ATT_READ_RESP_EVENT" },
    { 0X0C08, "ACI_ATT_READ_BLOB_RESP_EVENT" },
    { 0X0C09, "ACI_ATT_READ_MULTIPLE_RESP_EVENT" },
    { 0X0C0A, "ACI_ATT_READ_BY_GROUP_TYPE_RESP_EVENT" },
    { 0X0C0C, "ACI_ATT_PREPARE_WRITE_RESP_EVENT" },
    { 0X0C0D, "ACI_ATT_EXEC_WRITE_RESP_EVENT" },
    { 0X0C0E, "ACI_GATT_INDICATION_EVENT" },
    { 0X0C0F, "ACI_GATT_NOTIFICATION_EVENT" },
    { 0X0C10, "ACI_GATT_PROC_COMPLETE_EVENT" },
    { 0X0C11, "ACI_GATT_ERROR_RESP_EVENT" },
    { 0X0C12, "ACI_GATT_DISC_READ_CHAR_BY_UUID_RESP_EVENT" },
    { 0X0C13, "ACI_GATT_WRITE_PERMIT_REQ_EVENT" },
    { 0X0C14, "ACI_GATT_READ_PERMIT_REQ_EVENT" },
    { 0X0C15, "ACI_GATT_READ_MULTI_PERMIT_REQ_EVENT" },
    { 0X0C16, "ACI_GATT_TX_POOL_AVAILABLE_EVENT" },
    { 0X0C17, "ACI_GATT_SERVER_CONFIRMATION_EVENT" },
    { 0X0C18, "ACI_GATT_PREPARE_WRITE_PERMIT_REQ_EVENT" },
    { 0X0C1D, "ACI_GATT_READ_EXT_EVENT" },
    { 0X0C1E, "ACI_GATT_INDICATION_EXT_EVENT" },
    { 0X0C1F, "ACI_GATT_NOTIFICATION_EXT_EVENT" },
};


#endif

#define HCI_COMMAND_DATA_PACKET        0x01
#define HCI_ACL_DATA_PACKET            0x02
#define HCI_EVENT_PACKET               0x04

#define SYS_COMMAND_PACKET             0x10
#define SYS_RESPONSE_PACKET            0x11
#define SYS_EVENT_PACKET               0x12

/*
 * IPCC Channels:
 *
 * BLE: CH1_TX COMMAND
 *      CH1_RX EVENT
 *
 * SYS: CH2_TX COMMAND/RESPONSE
 *      CH2_RX EVENT
 *
 * MM:  CH4_TX RELEASE
 *
 * ACL: CH6_TX ACLDATA
 */

typedef struct _MB_Queue_t {
    struct _MB_Node_t             *head;
    struct _MB_Node_t             *tail;
} MB_Queue_t;

typedef struct _MB_Node_t {
    struct _MB_Node_t             *next;
    struct _MB_Node_t             *prev;
} MB_Node_t;

typedef struct _MB_SysCommand_t {
    MB_Node_t                     node;
  // uint8_t                       data[68]; /* type (1), opcode (2), length(1), payload(64) */
    uint8_t                       data[260]; /* type (1), opcode (2), length(1), payload(255), pad(1) */
} MB_SysCommand_t;

typedef struct _MB_SysEvent_t {
    MB_Node_t                     node;
  //    uint8_t                       data[24]; /* type (1), evtcode (1), length(1), payload(20), pad(1) */
    uint8_t                       data[260]; /* type (1), evtcode (1), length(1), payload(255), pad(1) */
} MB_SysEvent_t;

typedef struct _MB_BleCommand_t {
    MB_Node_t                     node;
    uint8_t                       data[260]; /* type (1), opcode (2), length(1), payload(255), pad(1) */
} MB_BleCommand_t;

typedef struct _MB_BleAclData_t {
    MB_Node_t                     node;
    uint8_t                       data[256]; /* type (1), handle (2), length(2), payload(251) */
} MB_BleAclData_t;

typedef struct _MB_BleEvent_t {
    MB_Node_t                     node;
    uint8_t                       data[260]; /* type (1), evtcode (1), length(1), payload(255), pad(1) */
} MB_BleEvent_t;

typedef struct _MB_BleCsEvent_t {
    MB_Node_t                     node;
    uint8_t                       data[8]; /* type (1), evtcode (1), length(1), payload(4), pad(1) */
} MB_BleCsEvent_t;


/**
 * Version
 * [0:3]   = Build - 0: Untracked - 15:Released - x: Tracked version
 * [4:7]   = branch - 0: Mass Market - x: ...
 * [8:15]  = Subversion
 * [16:23] = Version minor
 * [24:31] = Version major
 *
 * Memory Size
 * [0:7]   = Flash ( Number of 4k sector)
 * [8:15]  = Reserved ( Shall be set to 0 - may be used as flash extension )
 * [16:23] = SRAM2b ( Number of 1k sector)
 * [24:31] = SRAM2a ( Number of 1k sector)
 */

typedef struct _MB_FUS_DeviceInfoTable_t {
    uint32_t                      DeviceInfoTableState;
    uint8_t                       Reserved1;
    uint8_t                       LastFusActiveState;
    uint8_t                       LastWirelessStackState;
    uint8_t                       CurrentWirelessStackType;
    uint32_t                      SafeBootVersion;
    uint32_t                      FusVersion;
    uint32_t                      FusMemorySize;
    uint32_t                      WirelessStackVersion;
    uint32_t                      WirelessStackMemorySize;
    uint32_t                      WirelessFirmwareBleInfo;
    uint32_t                      WirelessFirmwareThreadInfo;
    uint32_t                      Reserved2;
    uint64_t                      UID64;
    uint16_t                      DeviceId;
} MB_FUS_DeviceInfoTable_t;

typedef struct _MB_SafeBootInfoTable_t {
    uint32_t                      Version;
} MB_SafeBootInfoTable_t;

typedef struct _MB_FusInfoTable_t {
    uint32_t                      Version;
    uint32_t                      MemorySize;
    uint32_t                      FusInfo;
} MB_FusInfoTable_t;

typedef struct _MB_WirelessFwInfoTable_t {
    uint32_t                      Version;
    uint32_t                      MemorySize;
    uint32_t                      InfoStack;
    uint32_t                      Reserved;
} MB_WirelessFwInfoTable_t;

typedef struct _MB_Wireless_DeviceInfoTable_t {
    MB_SafeBootInfoTable_t        SafeBootInfoTable;
    MB_FusInfoTable_t             FusInfoTable;
    MB_WirelessFwInfoTable_t      WirelessFwInfoTable;
} MB_Wireless_DeviceInfoTable_t;

typedef union _MB_DeviceInfoTable_t {
    MB_FUS_DeviceInfoTable_t      Fus;
    MB_Wireless_DeviceInfoTable_t Wireless;
} MB_DeviceInfoTable_t;

typedef struct _MB_BleTable_t {
    MB_BleCommand_t               *pcmd_buffer;
    MB_BleCsEvent_t               *pcs_buffer;
    MB_Queue_t                    *pevt_queue;
    MB_BleAclData_t               *pacl_data_buffer;
} MB_BleTable_t;

typedef struct _MB_SysTable_t {
    MB_SysCommand_t               *pcmd_buffer;
    MB_Queue_t                    *pevt_queue;
} MB_SysTable_t;

typedef struct _MB_MemManagerTable_t {
    MB_BleEvent_t                 *spare_ble_buffer;
    MB_SysEvent_t                 *spare_sys_buffer;
    uint8_t                       *blepool;
    uint32_t                      blepoolsize;
    MB_Queue_t                    *pevt_free_buffer_queue;
    uint8_t                       *traces_evt_pool;
    uint32_t                      tracespoolsize;
} MB_MemManagerTable_t;

typedef struct _MB_TracesTable_t {
    MB_Queue_t                    *traces_queue;
} MB_TracesTable_t;

typedef struct _MB_RefTable_t {
    union  _MB_DeviceInfoTable_t  *p_device_info_table;
    struct _MB_BleTable_t         *p_ble_table;
    struct _MB_ThreadTable_t      *p_thread_table;
    struct _MB_SysTable_t         *p_sys_table;
    struct _MB_MemManagerTable_t  *p_mem_manager_table;
    struct _MB_TracesTable_t      *p_traces_table;
    struct _MB_Mac_802_15_4_t     *p_mac_802_15_4_table;
    struct _MB_ZigbeeTable_t      *p_zigbee_table;
    struct _MB_LldTestsTable_t    *p_lld_tests_table;
    struct _MB_LldBleTable_t      *p_lld_ble_table;
} MB_RefTable_t;

#define STM32WB_IPCC_BLE_EVENT_POOL_ENTRIES 5

static __attribute__((section(".ipcc.ref_table"), used)) MB_RefTable_t MB_RefTable;
static __attribute__((section(".ipcc.sys_state"), used)) volatile uint32_t MB_SysState;
static __attribute__((section(".ipcc.info_table"))) MB_DeviceInfoTable_t MB_DeviceInfoTable;
static __attribute__((section(".ipcc.sys_table"))) MB_SysTable_t MB_SysTable;
static __attribute__((section(".ipcc.ble_table"))) MB_BleTable_t MB_BleTable;
static __attribute__((section(".ipcc.mm_table"))) MB_MemManagerTable_t MB_MemManagerTable;
static __attribute__((section(".ipcc"))) MB_Queue_t MB_SysEvtQueue;
static __attribute__((section(".ipcc"))) MB_SysEvent_t MB_SpareSysBuffer;
static __attribute__((section(".ipcc"))) MB_SysCommand_t MB_SysCmdBuffer;
static __attribute__((section(".ipcc"))) MB_Queue_t MB_BleEvtQueue;
static __attribute__((section(".ipcc"))) MB_BleEvent_t MB_SpareBleBuffer;
static __attribute__((section(".ipcc"))) MB_Queue_t MB_EvtFreeBufferQueue;
static __attribute__((section(".ipcc"))) MB_BleCommand_t MB_BleCmdBuffer;
static __attribute__((section(".ipcc"))) MB_BleCsEvent_t MB_BleCsBuffer;
static __attribute__((section(".ipcc"))) MB_BleAclData_t MB_BleAclDataBuffer;
static __attribute__((section(".ipcc"))) uint8_t MB_BlePool[((sizeof(MB_BleEvent_t) + 3) & ~3) * STM32WB_IPCC_BLE_EVENT_POOL_ENTRIES];

static void MB_QueueInit(MB_Queue_t *queue)
{
    queue->head = (MB_Node_t*)queue;
    queue->tail = (MB_Node_t*)queue;
}

static bool MB_QueueIsEmpty(MB_Queue_t *queue)
{
    return (((MB_Node_t*)queue)->next == ((MB_Node_t*)queue));
}

static void MB_QueueInsert(MB_Queue_t *queue, MB_Node_t *node)
{
    node->next = (MB_Node_t*)queue;
    node->prev = queue->tail;
    
    node->next->prev = node;
    node->prev->next = node;
}

static MB_Node_t* MB_QueueRemove(MB_Queue_t *queue)
{
    MB_Node_t *node = queue->head;

    node->next->prev = node->prev;
    node->prev->next = node->next;

    return node;
}

static void MB_QueueCopy(MB_Queue_t *destination, MB_Queue_t *source)
{
    destination->head = source->head;
    destination->tail = source->tail;

    destination->head->prev = (MB_Node_t*)destination;
    destination->tail->next = (MB_Node_t*)destination;

    source->head = (MB_Node_t*)source;
    source->tail = (MB_Node_t*)source;
}

/*********************************************************************************************/

#define STM32WB_IPCC_STATE_NONE      0
#define STM32WB_IPCC_STATE_NOT_READY 1
#define STM32WB_IPCC_STATE_READY     2
#define STM32WB_IPCC_STATE_WIRELESS  3

typedef struct _stm32wb_ipcc_device_t {
    volatile uint8_t                      state;
    volatile uint8_t                      ble_busy;
    volatile uint8_t                      acl_busy;
    volatile uint8_t                      mm_busy;
    stm32wb_ipcc_ble_command_t            *ble_head;
    stm32wb_ipcc_ble_command_t            *ble_tail;
    stm32wb_ipcc_ble_command_t * volatile ble_submit;
    stm32wb_ipcc_ble_command_t            *ble_current;
    volatile uint8_t                      *acl_status;
    stm32wb_ipcc_ble_acldata_callback_t   acl_callback;
    void                                  *acl_context;
    stm32wb_ipcc_ble_event_callback_t     evt_callback;
    void                                  *evt_context;
    MB_BleEvent_t                         *evt_head;
    MB_BleEvent_t                         *evt_tail;
    MB_BleEvent_t * volatile              evt_acquire;
    MB_BleEvent_t * volatile              evt_release;
    MB_BleEvent_t                         *evt_current;
    MB_Queue_t                            mm_queue;
} stm32wb_ipcc_device_t;

static stm32wb_ipcc_device_t stm32wb_ipcc_device;

/*********************************************************************************************/

void __stm32wb_ipcc_initialize(void)
{
    if (MB_RefTable.p_device_info_table == NULL)
    {
        MB_QueueInit(&MB_SysEvtQueue);
        MB_QueueInit(&MB_BleEvtQueue);
        MB_QueueInit(&MB_EvtFreeBufferQueue);

        MB_BleTable.pcmd_buffer = &MB_BleCmdBuffer; 
        MB_BleTable.pcs_buffer = &MB_BleCsBuffer; 
        MB_BleTable.pevt_queue = &MB_BleEvtQueue; 
        MB_BleTable.pacl_data_buffer = &MB_BleAclDataBuffer;
        
        MB_SysTable.pcmd_buffer = &MB_SysCmdBuffer; 
        MB_SysTable.pevt_queue = &MB_SysEvtQueue; 
        
        MB_MemManagerTable.spare_ble_buffer = &MB_SpareBleBuffer;
        MB_MemManagerTable.spare_sys_buffer = &MB_SpareSysBuffer;
        MB_MemManagerTable.pevt_free_buffer_queue = &MB_EvtFreeBufferQueue;
        MB_MemManagerTable.blepool = MB_BlePool;
        MB_MemManagerTable.blepoolsize = sizeof(MB_BlePool);

        MB_RefTable.p_device_info_table = &MB_DeviceInfoTable;
        MB_RefTable.p_ble_table = &MB_BleTable;
        MB_RefTable.p_sys_table = &MB_SysTable;
        MB_RefTable.p_mem_manager_table = &MB_MemManagerTable;
    }
}

bool stm32wb_ipcc_sys_enable(void)
{
    if (stm32wb_ipcc_device.state != STM32WB_IPCC_STATE_NONE)
    {
        return false;
    }
    
    stm32wb_ipcc_device.state = (MB_SysState == STM32WB_IPCC_SYS_STATE_NONE) ? STM32WB_IPCC_STATE_NOT_READY : STM32WB_IPCC_STATE_READY;
    stm32wb_ipcc_device.ble_busy = false;
    stm32wb_ipcc_device.acl_busy = false;
    stm32wb_ipcc_device.mm_busy = false;

    stm32wb_ipcc_device.ble_head = NULL;
    stm32wb_ipcc_device.ble_tail = NULL;
    stm32wb_ipcc_device.ble_submit = NULL;
    stm32wb_ipcc_device.ble_current = NULL;

    stm32wb_ipcc_device.evt_head = NULL;
    stm32wb_ipcc_device.evt_tail = NULL;
    stm32wb_ipcc_device.evt_acquire = NULL;
    stm32wb_ipcc_device.evt_release = NULL;
    stm32wb_ipcc_device.evt_current = NULL;
    
    MB_QueueInit(&stm32wb_ipcc_device.mm_queue);
    
    stm32wb_system_periph_enable(STM32WB_SYSTEM_PERIPH_IPCC);
    
    NVIC_SetPriority(IPCC_C1_RX_IRQn, STM32WB_IPCC_IRQ_PRIORITY);
    NVIC_SetPriority(IPCC_C1_TX_IRQn, STM32WB_IPCC_IRQ_PRIORITY);
    NVIC_EnableIRQ(IPCC_C1_RX_IRQn);
    NVIC_EnableIRQ(IPCC_C1_TX_IRQn);

    IPCC->C1CR |= (IPCC_C1CR_TXFIE | IPCC_C1CR_RXOIE);

    armv7m_atomic_and(&IPCC->C1MR, ~IPCC_C1MR_CH2OM);

    armv7m_atomic_or(&EXTI->IMR2, EXTI_IMR2_IM36);

    if (!stm32wb_system_cpu2_boot())
    {
        stm32wb_ipcc_sys_disable();

        return false;
    }

    return true;
}

void stm32wb_ipcc_sys_disable(void)
{
    armv7m_atomic_and(&EXTI->IMR2, ~EXTI_IMR2_IM36);

    armv7m_atomic_or(&IPCC->C1MR, IPCC_C1MR_CH2OM);

    IPCC->C1CR &= ~(IPCC_C1CR_TXFIE | IPCC_C1CR_RXOIE);

    NVIC_DisableIRQ(IPCC_C1_RX_IRQn);
    NVIC_DisableIRQ(IPCC_C1_TX_IRQn);

    stm32wb_system_periph_disable(STM32WB_SYSTEM_PERIPH_IPCC);
    
    stm32wb_ipcc_device.state = STM32WB_IPCC_STATE_NONE;
}

uint32_t stm32wb_ipcc_sys_state(void)
{
    if (stm32wb_ipcc_device.state <= STM32WB_IPCC_STATE_NOT_READY)
    {
        return STM32WB_IPCC_SYS_STATE_NONE;
    }

    return MB_SysState;
}

bool stm32wb_ipcc_sys_info(stm32wb_ipcc_sys_info_t *p_info_return)
{
    if (stm32wb_ipcc_device.state <= STM32WB_IPCC_STATE_NOT_READY)
    {
        return false;
    }
    
    if (MB_SysState == STM32WB_IPCC_SYS_STATE_FUS)
    {
        p_info_return->FusVersion = MB_DeviceInfoTable.Fus.FusVersion;
        p_info_return->FusMemorySize = MB_DeviceInfoTable.Fus.FusMemorySize;
        p_info_return->WirelessStackVersion = MB_DeviceInfoTable.Fus.WirelessStackVersion;
        p_info_return->WirelessStackMemorySize = MB_DeviceInfoTable.Fus.WirelessStackMemorySize;
        p_info_return->WirelessStackType = MB_DeviceInfoTable.Fus.WirelessFirmwareBleInfo & 0x000000ff;
    }
    else
    {
        p_info_return->FusVersion = MB_DeviceInfoTable.Wireless.FusInfoTable.Version;
        p_info_return->FusMemorySize = MB_DeviceInfoTable.Wireless.FusInfoTable.MemorySize;
        p_info_return->WirelessStackVersion = MB_DeviceInfoTable.Wireless.WirelessFwInfoTable.Version;
        p_info_return->WirelessStackMemorySize = MB_DeviceInfoTable.Wireless.WirelessFwInfoTable.MemorySize;
        p_info_return->WirelessStackType = MB_DeviceInfoTable.Wireless.WirelessFwInfoTable.InfoStack & 0x000000ff;
    }

    return true;
}


static bool __svc_stm32wb_ipcc_sys_command(uint32_t control, const void *cparam, void *rparam)
{
    uint32_t opcode, clen, rsize, rlen;
    volatile uint8_t *MB_SysRspBuffer;

    if (stm32wb_ipcc_device.state <= STM32WB_IPCC_STATE_NOT_READY)
    {
        return false;
    }

    opcode = (control >>  0) & 0xffff;
    clen   = (control >> 16) & 0xff;
    rsize  = (control >> 24) & 0xff;
    
    MB_SysCmdBuffer.data[0] = SYS_COMMAND_PACKET;
    MB_SysCmdBuffer.data[1] = (opcode >> 0);
    MB_SysCmdBuffer.data[2] = (opcode >> 8);
    MB_SysCmdBuffer.data[3] = clen;

    if (clen)
    {
        memcpy(&MB_SysCmdBuffer.data[4], cparam, clen);
    }

    IPCC->C1SCR = IPCC_C1SCR_CH2S;

    while (IPCC->C1TOC2SR & IPCC_C1TOC2SR_CH2F)
    {
    }
    
    MB_SysRspBuffer = (volatile uint8_t*)&MB_SysCmdBuffer;

    if (rparam)
    {
        rlen = MB_SysRspBuffer[2] - 3;

        if (rlen > rsize)
        {
            ((uint8_t*)rparam)[0] = 0xff;
        }
        else
        {
            memcpy(rparam, (const void*)&MB_SysRspBuffer[6], rlen);
        }
    }

    return true;
}

bool stm32wb_ipcc_sys_command(uint16_t opcode, const void *cparam, uint8_t clen, void *rparam, uint8_t rsize)
{
    uint32_t control;

    control = (opcode << 0) | (clen << 16) | (rsize << 24);
    
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_3((uint32_t)&__svc_stm32wb_ipcc_sys_command, (uint32_t)control, (uint32_t)cparam, (uint32_t)rparam);
    }

    if (armv7m_core_is_in_pendsv_or_svcall())
    {
        return __svc_stm32wb_ipcc_sys_command(control, cparam, rparam);
    }

    return false;
}

bool stm32wb_ipcc_sys_firmware(uint32_t version, uint32_t type, uint32_t address, const uint8_t *image, uint32_t size, const uint8_t *fus_1_0_2, const uint8_t *fus_1_1_0, uint32_t *p_code_return)
{
    static uint8_t firmware_data[1024];
    const uint8_t *firmware_image;
    uint32_t firmware_address, firmware_size, firmware_offset, firmware_chunk, firmware_index;
    static uint8_t fus_status;
    static stm32wb_ipcc_sys_info_t sys_info;
    static stm32wb_ipcc_sys_fus_state_t fus_state;
    stm32wb_flash_request_t request;
    bool success = true;
    
    if (!stm32wb_ipcc_sys_info(&sys_info))
    {
        *p_code_return = 0xffff0000;

        return false;
    }

    if (((sys_info.FusVersion & 0xffff0000) != 0x01010000) && (!fus_1_0_2 || !fus_1_1_0))
    {
        *p_code_return = 0xffff0001;

        return false;
    }

    *p_code_return = 0x00000000;
    
    if (((sys_info.FusVersion & 0xffff0000) != 0x01010000) ||
        ((sys_info.WirelessStackType & 0x000000ff) != type) ||
        ((sys_info.WirelessStackVersion & 0xffffff00) != version))
    {
        if (stm32wb_ipcc_sys_state() == STM32WB_IPCC_SYS_STATE_WIRELESS)
        {
            stm32wb_ipcc_sys_command(STM32WB_IPCC_SYS_OPCODE_FUS_GET_STATE, NULL, 0, NULL, 0);
            stm32wb_ipcc_sys_command(STM32WB_IPCC_SYS_OPCODE_FUS_GET_STATE, NULL, 0, NULL, 0);
            
            while (1) { __WFE(); }
        }
        
        stm32wb_ipcc_sys_command(STM32WB_IPCC_SYS_OPCODE_FUS_GET_STATE, NULL, 0, &fus_state, sizeof(fus_state));

        while ((fus_state.state != STM32WB_IPCC_SYS_FUS_STATE_IDLE) && (fus_state.error_code == STM32WB_IPCC_SYS_FUS_ERROR_CODE_NO_ERROR))
        {
            armv7m_core_udelay(250000);

            stm32wb_ipcc_sys_command(STM32WB_IPCC_SYS_OPCODE_FUS_GET_STATE, NULL, 0, &fus_state, sizeof(fus_state));
        }
        
        if (fus_state.state != STM32WB_IPCC_SYS_FUS_STATE_IDLE)
        {
            if ((fus_state.state == STM32WB_IPCC_SYS_FUS_STATE_ERROR) && (fus_state.error_code >= 0xf0))
            {
                stm32wb_system_reset();
            }
            
            *p_code_return = (STM32WB_IPCC_SYS_OPCODE_FUS_GET_STATE << 16) | (fus_state.state << 8) | (fus_state.error_code << 0);

            success = false;
        }
        else
        {
            if (sys_info.WirelessStackMemorySize)
            {
                stm32wb_ipcc_sys_command(STM32WB_IPCC_SYS_OPCODE_FUS_FW_DELETE, NULL, 0, &fus_status, sizeof(fus_status));
                    
                if (fus_status == STM32WB_IPCC_SYS_FUS_STATUS_SUCCESS)
                {
                    while (1) { __WFE(); }
                }

                *p_code_return = (STM32WB_IPCC_SYS_OPCODE_FUS_FW_DELETE << 16) | fus_status;

                success = false;
            }
            else
            {
                if ((sys_info.FusVersion & 0xffff0000) != 0x01010000)
                {
                    if ((sys_info.FusVersion & 0xffff0000) != 0x01000000)
                    {
                        firmware_image = fus_1_0_2;
                    }
                    else
                    {
                        firmware_image = fus_1_1_0;
                    }

                    firmware_address = 0x080ec000;
                    firmware_size = 24496;
                }
                else
                {
                    firmware_address = address;
                    firmware_image = (const uint8_t*)image;
                    firmware_size = size;
                }

                request.next = NULL;
                request.control = STM32WB_FLASH_CONTROL_ERASE | STM32WB_FLASH_CONTROL_FLUSH;
                request.address = firmware_address;
                request.count = ((FLASH_BASE + ((FLASH->SFR & FLASH_SFR_SFSA) * 4096)) - firmware_address);
                request.data = NULL;
                request.callback = NULL;
                request.context = NULL;
                
                stm32wb_flash_request(&request);

                while (request.status == STM32WB_FLASH_STATUS_BUSY) { __WFE(); }
                    
                if (request.status != STM32WB_FLASH_STATUS_SUCCESS)
                {
                    *p_code_return = 0xffff0002;

                    success = false;
                }
                else
                {
                    for (firmware_offset = 0; firmware_offset < firmware_size; firmware_offset += firmware_chunk)
                    {
                        firmware_chunk = firmware_size - firmware_offset;
                                
                        if (firmware_chunk > sizeof(firmware_data)) {
                            firmware_chunk = sizeof(firmware_data);
                        }
                                
                        for (firmware_index = 0; firmware_index < firmware_chunk; firmware_index++) {
                            firmware_data[firmware_index] = firmware_image[firmware_offset + firmware_index] ^ 0xff;
                        }
                                
                        request.next = NULL;
                        request.control = STM32WB_FLASH_CONTROL_PROGRAM | STM32WB_FLASH_CONTROL_FLUSH;
                        request.address = firmware_address + firmware_offset;
                        request.count = firmware_chunk;
                        request.data = (const uint8_t*)&firmware_data[0];
                        request.callback = NULL;
                        request.context = NULL;
                        
                        stm32wb_flash_request(&request);
                        
                        while (request.status == STM32WB_FLASH_STATUS_BUSY) { __WFE(); }
                        
                        if (request.status != STM32WB_FLASH_STATUS_SUCCESS)
                        {
                            *p_code_return = 0xffff0003;

                            success = false;

                            break;
                        }
                    }

                    if (success)
                    {
                        stm32wb_ipcc_sys_command(STM32WB_IPCC_SYS_OPCODE_FUS_FW_UPGRADE, NULL, 0, &fus_status, sizeof(fus_status));

                        if (fus_status == STM32WB_IPCC_SYS_FUS_STATUS_SUCCESS)
                        {
                            while (1) {  __WFE(); }
                        }

                        *p_code_return = (STM32WB_IPCC_SYS_OPCODE_FUS_FW_UPGRADE << 16) | fus_status;

                        success = false;
                    }
                }
            }
        }
    }
    
    return success;
}

bool stm32wb_ipcc_ble_enable(const stm32wb_ipcc_ble_init_params_t *params, stm32wb_ipcc_ble_event_callback_t callback, void *context)
{
    uint8_t sys_status;

    if (stm32wb_ipcc_device.state != STM32WB_IPCC_STATE_READY)
    {
        return false;
    }
    
    if (MB_SysState != STM32WB_IPCC_SYS_STATE_WIRELESS)
    {
        return false;
    }

    if (!stm32wb_system_wireless_enable())
    {
        return false;
    }
    
    stm32wb_ipcc_device.state = STM32WB_IPCC_STATE_WIRELESS;

    stm32wb_ipcc_device.evt_callback = callback;
    stm32wb_ipcc_device.evt_context = context;
    
    armv7m_atomic_and(&IPCC->C1MR, ~IPCC_C1MR_CH1OM);

    stm32wb_ipcc_sys_command(STM32WB_IPCC_SYS_OPCODE_BLE_INIT, params, sizeof(stm32wb_ipcc_ble_init_params_t), &sys_status, sizeof(sys_status));
    
    if (sys_status != 0x00)
    {
        armv7m_atomic_or(&IPCC->C1MR, IPCC_C1MR_CH1OM);
        
        stm32wb_system_wireless_disable();

        return false;
    }
    
    return true;
}

bool stm32wb_ipcc_ble_disable(void)
{
    if (stm32wb_ipcc_device.state != STM32WB_IPCC_STATE_WIRELESS)
    {
        return false;
    }
    
    armv7m_atomic_or(&IPCC->C1MR, IPCC_C1MR_CH1OM);
        
    stm32wb_system_wireless_disable();

    stm32wb_ipcc_device.state = STM32WB_IPCC_STATE_READY;
    
    return true;
}

bool stm32wb_ipcc_ble_command(stm32wb_ipcc_ble_command_t *command)
{
    stm32wb_ipcc_ble_command_t *command_submit;
    
    if (stm32wb_ipcc_device.state != STM32WB_IPCC_STATE_WIRELESS)
    {
        return false;
    }

    command->status = STM32WB_IPCC_BLE_STATUS_BUSY;

    do
    {
        command_submit = stm32wb_ipcc_device.ble_submit;

        armv7m_atomic_store((volatile uint32_t*)&command->next, (uint32_t)command_submit);
    }
    while ((stm32wb_ipcc_ble_command_t*)armv7m_atomic_cas((volatile uint32_t*)&stm32wb_ipcc_device.ble_submit, (uint32_t)command_submit, (uint32_t)command) != command_submit);
    
    if (command_submit == NULL)
    {
        if (__current_irq() != IPCC_C1_RX_IRQn)
        {
            NVIC_SetPendingIRQ(IPCC_C1_RX_IRQn);
        }
    }
            
    return true;
}

bool stm32wb_ipcc_ble_acldata(const uint8_t *data, uint32_t count, volatile uint8_t *p_status_return, stm32wb_ipcc_ble_acldata_callback_t callback, void *context)
{
    if (count > (sizeof(MB_BleAclDataBuffer.data) -1))
    {
        return false;
    }

    if (stm32wb_ipcc_device.state != STM32WB_IPCC_STATE_WIRELESS)
    {
        return false;
    }
    
    if (armv7m_atomic_casb(&stm32wb_ipcc_device.acl_busy, false, true) != false)
    {
        return false;
    }

    stm32wb_ipcc_device.acl_status = p_status_return;
    stm32wb_ipcc_device.acl_callback = callback;
    stm32wb_ipcc_device.acl_context = context;

    if (p_status_return)
    {
        *p_status_return = STM32WB_IPCC_BLE_STATUS_BUSY;
    }

    MB_BleAclDataBuffer.data[0] = HCI_ACL_DATA_PACKET;
    memcpy(&MB_BleAclDataBuffer.data[1], data, count);

    IPCC->C1SCR = IPCC_C1SCR_CH6S;
            
    armv7m_atomic_and(&IPCC->C1MR, ~IPCC_C1MR_CH6FM);

    return true;
}

uint8_t *stm32wb_ipcc_ble_event(void)
{
    MB_BleEvent_t *ble_event, *ble_event_next, *ble_event_head, *ble_event_tail, *ble_event_acquire, *ble_event_release;
    
    if (stm32wb_ipcc_device.state != STM32WB_IPCC_STATE_WIRELESS)
    {
        return false;
    }

    if (stm32wb_ipcc_device.evt_current)
    {
        ble_event = stm32wb_ipcc_device.evt_current;

        stm32wb_ipcc_device.evt_current = NULL;

        do
        {
            ble_event_release = stm32wb_ipcc_device.evt_release;

            armv7m_atomic_store((volatile uint32_t*)&ble_event->node.next, (uint32_t)&ble_event_release->node);
        }
        while ((MB_BleEvent_t*)armv7m_atomic_cas((volatile uint32_t*)&stm32wb_ipcc_device.evt_release, (uint32_t)ble_event_release, (uint32_t)ble_event) != ble_event_release);

        if (ble_event_release == NULL)
        {
            if (__current_irq() != IPCC_C1_RX_IRQn)
            {
                NVIC_SetPendingIRQ(IPCC_C1_RX_IRQn);
            }
        }
    }

    if (stm32wb_ipcc_device.evt_acquire)
    {
        ble_event_acquire = (MB_BleEvent_t*)armv7m_atomic_swap((volatile uint32_t*)&stm32wb_ipcc_device.evt_acquire, (uint32_t)NULL);

        for (ble_event_head = NULL, ble_event_tail = ble_event_acquire; ble_event_acquire != NULL; ble_event_acquire = ble_event_next)
        {
            ble_event_next = (MB_BleEvent_t*)ble_event_acquire->node.next;

            armv7m_atomic_store((volatile uint32_t*)&ble_event_acquire->node.next, (uint32_t)&ble_event_head->node);

            ble_event_head = ble_event_acquire;
        }
        
        if (!stm32wb_ipcc_device.evt_head)
        {
            stm32wb_ipcc_device.evt_head = ble_event_head;
        }
        else
        {
            armv7m_atomic_store((volatile uint32_t*)&stm32wb_ipcc_device.evt_tail->node.next, (uint32_t)&ble_event_head->node);
        }
        
        stm32wb_ipcc_device.evt_tail = ble_event_tail;
    }
    
    if (stm32wb_ipcc_device.evt_head)
    {
        ble_event = stm32wb_ipcc_device.evt_head;
        
        if (stm32wb_ipcc_device.evt_head == stm32wb_ipcc_device.evt_tail)
        {
            stm32wb_ipcc_device.evt_head = NULL;
            stm32wb_ipcc_device.evt_tail = NULL;
        }
        else
        {
            stm32wb_ipcc_device.evt_head = (MB_BleEvent_t*)ble_event->node.next;
        }
        
        stm32wb_ipcc_device.evt_current = ble_event;

#if (STM32WB_IPCC_TRACE_SUPPORTED == 1)
        if (ble_event->data[0] == HCI_EVENT_PACKET)
        {
            unsigned int index, evtcode;
            
            if (ble_event->data[1] == HCI_VENDOR_SPECIFIC_EVENT)
            {
                evtcode = (ble_event->data[3] << 0) | (ble_event->data[4] << 8);

                for (index = 0; index < (sizeof(hci_vs_event_table) / sizeof(hci_vs_event_table[0])); index++)
                {
                    if (hci_vs_event_table[index].evtcode == evtcode)
                    {
                        printf("== %s\r\n", hci_vs_event_table[index].cstring);
                        break;
                    }
                }

                if (index == (sizeof(hci_vs_event_table) / sizeof(hci_vs_event_table[0])))
                {
                    printf("?? UNEXPECTED HCI VS EVENT = %04x\r\n", evtcode);
                }
            }
            else if (ble_event->data[1] == HCI_LE_META_EVENT)
            {
                evtcode = ble_event->data[3];

                for (index = 0; index < (sizeof(hci_le_event_table) / sizeof(hci_le_event_table[0])); index++)
                {
                    if (hci_le_event_table[index].evtcode == evtcode)
                    {
                        printf("== %s\r\n", hci_le_event_table[index].cstring);
                        break;
                    }
                }

                if (index == (sizeof(hci_le_event_table) / sizeof(hci_le_event_table[0])))
                {
                    printf("?? UNEXPECTED HCI LE EVENT = %02x\r\n", evtcode);
                }
            }
            else
            {
                evtcode = ble_event->data[1];
                
                for (index = 0; index < (sizeof(hci_event_table) / sizeof(hci_event_table[0])); index++)
                {
                    if (hci_event_table[index].evtcode == evtcode)
                    {
                        printf("== %s\r\n", hci_event_table[index].cstring);
                        break;
                    }
                }

                if (index == (sizeof(hci_event_table) / sizeof(hci_event_table[0])))
                {
                    printf("?? UNEXPECTED HCI EVENT = %02x\r\n", evtcode);
                }
            }
        }
        else
        {
            printf("?? UNEXPECTED EVENT PACKET = %02x\r\n", ble_event->data[0]);
        }
#endif /* (STM32WB_IPCC_TRACE_SUPPORTED == 1) */
        
        return &ble_event->data[0];
    }

    return NULL;
}

void IPCC_C1_RX_IRQHandler(void)
{
    uint32_t ipcc_c1mr, ipcc_c2toc1sr, evt_code, evt_count, length;
    uint16_t opcode;
    const uint8_t *data;
    stm32wb_ipcc_ble_command_t *command, *command_submit, *command_next, *command_head, *command_tail;
    stm32wb_ipcc_ble_command_callback_t callback;
    void *context;
    MB_BleEvent_t *ble_event, *ble_event_next, *ble_event_acquire, *ble_event_release;
    MB_SysEvent_t *sys_event;
    
    ipcc_c1mr = IPCC->C1MR;
    ipcc_c2toc1sr = IPCC->C2TOC1SR;

    if (!(ipcc_c1mr & IPCC_C1MR_CH1OM) && (ipcc_c2toc1sr & IPCC_C2TOC1SR_CH1F))
    {
        /* BLE RX EVT */

        evt_count = 0;
        
        while (!MB_QueueIsEmpty(MB_BleTable.pevt_queue))
        {
            ble_event = (MB_BleEvent_t*)MB_QueueRemove(MB_BleTable.pevt_queue);

            data = &ble_event->data[0];

            if (data[0] == HCI_EVENT_PACKET)
            {
                if (data[1] == HCI_COMMAND_COMPLETE_EVENT)
                {
                    /* type, evt, length, packets, opcode[0], opcode[1], params */

                    stm32wb_ipcc_device.ble_busy = (data[3] == 0);
                    
                    opcode = (data[4] << 0) | (data[5] << 8);

                    command = stm32wb_ipcc_device.ble_current;

                    if (command && (command->opcode == opcode))
                    {
                        stm32wb_ipcc_device.ble_current = NULL;

                        if (command->rparam)
                        {
                            length = (data[2] - 3);
                            
                            if (length > command->rsize)
                            {
                                length = command->rsize;
                            }
                            
                            if (length)
                            {
                                memcpy(command->rparam, &data[6], length);
                            }
                            
                            command->rlen = length;
                            
                            if (((void*)ble_event >= (void*)(&MB_BlePool[0])) && ((void*)ble_event < (void*)(&MB_BlePool[0] + sizeof(MB_BlePool))))
                            {
                                MB_QueueInsert(&stm32wb_ipcc_device.mm_queue, &ble_event->node);
                            }
                            
                            ble_event = NULL;
                        }

#if (STM32WB_IPCC_TRACE_SUPPORTED == 1)
                        printf("<< HCI_COMMAND_COMPLETE_EVENT = %02x\r\n", data[6]);
#endif /* (STM32WB_IPCC_TRACE_SUPPORTED == 1) */
                        
                        callback = command->callback;
                        context = command->context;

                        command->status = STM32WB_IPCC_BLE_STATUS_SUCCESS;
                        
                        if (callback)
                        {
                            (*callback)(context);
                        }
                    }
                }

                if (data[1] == HCI_COMMAND_STATUS_EVENT)
                {
                    /* type, evt, length, status, packets, opcode[0], opcode[1] */

                    stm32wb_ipcc_device.ble_busy = (data[4] == 0);

                    opcode = (data[5] << 0) | (data[6] << 8);
                    
                    command = stm32wb_ipcc_device.ble_current;

                    if (command && (command->opcode == opcode))
                    {
                        stm32wb_ipcc_device.ble_current = NULL;

                        if (command->rparam)
                        {
                            length = 1;

                            if (length > command->rsize)
                            {
                                length = command->rsize;
                            }
                            
                            if (length)
                            {
                                ((uint8_t*)command->rparam)[0] = data[3];
                            }
                            
                            command->rlen = length;

                            if (((void*)ble_event >= (void*)(&MB_BlePool[0])) && ((void*)ble_event < (void*)(&MB_BlePool[0] + sizeof(MB_BlePool))))
                            {
                                MB_QueueInsert(&stm32wb_ipcc_device.mm_queue, &ble_event->node);
                            }
                                
                            ble_event = NULL;
                        }

#if (STM32WB_IPCC_TRACE_SUPPORTED == 1)
                        printf("<< HCI_COMMAND_STATUS_EVENT = %02x\r\n", data[3]);
#endif /* (STM32WB_IPCC_TRACE_SUPPORTED == 1) */
                        
                        callback = command->callback;
                        context = command->context;

                        command->status = STM32WB_IPCC_BLE_STATUS_SUCCESS;
                            
                        if (callback)
                        {
                            (*callback)(context);
                        }
                    }
                }
            }
            
            if (ble_event)
            {
                do
                {
                    ble_event_acquire = stm32wb_ipcc_device.evt_acquire;
                    
                    armv7m_atomic_store((volatile uint32_t*)&ble_event->node.next, (uint32_t)&ble_event_acquire->node);
                }
                while ((MB_BleEvent_t*)armv7m_atomic_cas((volatile uint32_t*)&stm32wb_ipcc_device.evt_acquire, (uint32_t)ble_event_acquire, (uint32_t)ble_event) != ble_event_acquire);

                evt_count++;
            }
        }

        IPCC->C1SCR = IPCC_C1SCR_CH1C;

        if (evt_count)
        {
            if (stm32wb_ipcc_device.evt_callback)
            {
                (*stm32wb_ipcc_device.evt_callback)(stm32wb_ipcc_device.evt_context);
            }
        }
    }
    
    if (!(ipcc_c1mr & IPCC_C1MR_CH2OM) && (ipcc_c2toc1sr & IPCC_C2TOC1SR_CH2F))
    {
        /* SYS RX EVT */

        while (!MB_QueueIsEmpty(&MB_SysEvtQueue))
        {
            sys_event = (MB_SysEvent_t*)MB_QueueRemove(&MB_SysEvtQueue);

            data = &sys_event->data[0];
            
            if (data[0] == SYS_EVENT_PACKET)
            {
                if (data[1] == 0xff)
                {
                    evt_code = (data[3] << 0) | (data[4] << 8);

                    if (evt_code == 0x9200)
                    {
                        if (data[5] == 0x00)
                        {
                            MB_SysState = STM32WB_IPCC_SYS_STATE_WIRELESS;
                        }

                        if (data[5] == 0x01)
                        {
                            MB_SysState = STM32WB_IPCC_SYS_STATE_FUS;
                        }

                        stm32wb_ipcc_device.state = STM32WB_IPCC_STATE_READY;
                    }
                }
            }

            if (((void*)sys_event >= (void*)(&MB_BlePool[0])) && ((void*)sys_event < (void*)(&MB_BlePool[0] + sizeof(MB_BlePool))))
            {
                MB_QueueInsert(&stm32wb_ipcc_device.mm_queue, &sys_event->node);
            }
        }

        IPCC->C1SCR = IPCC_C1SCR_CH2C;
    }

    if (stm32wb_ipcc_device.ble_submit)
    {
        command_submit = (stm32wb_ipcc_ble_command_t*)armv7m_atomic_swap((volatile uint32_t*)&stm32wb_ipcc_device.ble_submit, (uint32_t)NULL);

        for (command_head = NULL, command_tail = command_submit; command_submit != NULL; command_submit = command_next)
        {
            command_next = command_submit->next;

            armv7m_atomic_store((volatile uint32_t*)&command_submit->next, (uint32_t)command_head);

            command_head = command_submit;
        }
        
        if (!stm32wb_ipcc_device.ble_head)
        {
            stm32wb_ipcc_device.ble_head = command_head;
        }
        else
        {
            armv7m_atomic_store((volatile uint32_t*)&stm32wb_ipcc_device.ble_tail->next, (uint32_t)command_head);
        }
        
        stm32wb_ipcc_device.ble_tail = command_tail;
    }
    
    if (!stm32wb_ipcc_device.ble_current)
    {
        if (stm32wb_ipcc_device.ble_head && !stm32wb_ipcc_device.ble_busy)
        {
            command = stm32wb_ipcc_device.ble_head;
            
            if (stm32wb_ipcc_device.ble_head == stm32wb_ipcc_device.ble_tail)
            {
                stm32wb_ipcc_device.ble_head = NULL;
                stm32wb_ipcc_device.ble_tail = NULL;
            }
            else
            {
                stm32wb_ipcc_device.ble_head = command->next;
            }
            
            stm32wb_ipcc_device.ble_current = command;

#if (STM32WB_IPCC_TRACE_SUPPORTED == 1)
            {
                unsigned int index, opcode;

                opcode = command->opcode;
                
                for (index = 0; index < (sizeof(hci_command_table) / sizeof(hci_command_table[0])); index++)
                {
                    if (hci_command_table[index].opcode == opcode)
                    {
                        printf(">> %s\r\n", hci_command_table[index].cstring);
                        break;
                    }
                }

                if (index == (sizeof(hci_command_table) / sizeof(hci_command_table[0])))
                {
                    printf("?? UNEXPECTED HCI COMMAND = %04x\r\n", opcode);
                }
            }
#endif /* (STM32WB_IPCC_TRACE_SUPPORTED == 1) */
            
            MB_BleCmdBuffer.data[0] = HCI_COMMAND_DATA_PACKET;
            MB_BleCmdBuffer.data[1] = (uint8_t)(command->opcode >> 0);
            MB_BleCmdBuffer.data[2] = (uint8_t)(command->opcode >> 8);
            MB_BleCmdBuffer.data[3] = (uint8_t)command->clen;
            
            if (command->clen)
            {
                memcpy(&MB_BleCmdBuffer.data[4], command->cparam, command->clen);
            }
            
            IPCC->C1SCR = IPCC_C1SCR_CH1S;
        }
    }

    if (stm32wb_ipcc_device.evt_release)
    {
        ble_event_release = (MB_BleEvent_t*)armv7m_atomic_swap((volatile uint32_t*)&stm32wb_ipcc_device.evt_release, (uint32_t)NULL);

        for (ble_event = ble_event_release; ble_event; ble_event = ble_event_next)
        {
            ble_event_next = (MB_BleEvent_t*)ble_event->node.next;

            if (((void*)ble_event >= (void*)(&MB_BlePool[0])) && ((void*)ble_event < (void*)(&MB_BlePool[0] + sizeof(MB_BlePool))))
            {
                MB_QueueInsert(&stm32wb_ipcc_device.mm_queue, &ble_event->node);
            }
        }
    }
    
    if (!stm32wb_ipcc_device.mm_busy)
    {
        if (!MB_QueueIsEmpty(&stm32wb_ipcc_device.mm_queue))
        {
            MB_QueueCopy(&MB_EvtFreeBufferQueue, &stm32wb_ipcc_device.mm_queue);

            stm32wb_ipcc_device.mm_busy = true;
            
            IPCC->C1SCR = IPCC_C1SCR_CH4S;
            
            armv7m_atomic_and(&IPCC->C1MR, ~IPCC_C1MR_CH4FM);
        }
    }
    
    __DSB();
}

void IPCC_C1_TX_IRQHandler(void)
{
  uint32_t ipcc_c1mr, ipcc_c1toc2sr;
    volatile uint8_t *acl_status;
    stm32wb_ipcc_ble_acldata_callback_t acl_callback;
    void *acl_context;

    ipcc_c1mr = IPCC->C1MR;
    ipcc_c1toc2sr = IPCC->C1TOC2SR;
   
    if (!(ipcc_c1mr & IPCC_C1MR_CH4FM) && !(ipcc_c1toc2sr & IPCC_C1TOC2SR_CH4F))
    {
        /* MM TX ACK */

        armv7m_atomic_or(&IPCC->C1MR, IPCC_C1MR_CH4FM);

        stm32wb_ipcc_device.mm_busy = false;
    }

    if (!(ipcc_c1mr & IPCC_C1MR_CH6FM) && !(ipcc_c1toc2sr & IPCC_C1TOC2SR_CH6F))
    {
        /* ACL TX ACK */

        armv7m_atomic_or(&IPCC->C1MR, IPCC_C1MR_CH6FM);

        acl_status = stm32wb_ipcc_device.acl_status;
        acl_callback = stm32wb_ipcc_device.acl_callback;
        acl_context = stm32wb_ipcc_device.acl_context;
        
        armv7m_atomic_storeb(&stm32wb_ipcc_device.acl_busy, false);

        if (acl_status)
        {
            *acl_status = STM32WB_IPCC_BLE_STATUS_SUCCESS;
        }

        if (acl_callback)
        {
            (*acl_callback)(acl_context);
        }
    }
    
    if (!stm32wb_ipcc_device.mm_busy)
    {
        if (!MB_QueueIsEmpty(&stm32wb_ipcc_device.mm_queue))
        {
            MB_QueueCopy(&MB_EvtFreeBufferQueue, &stm32wb_ipcc_device.mm_queue);

            stm32wb_ipcc_device.mm_busy = true;
            
            IPCC->C1SCR = IPCC_C1SCR_CH4S;
            
            armv7m_atomic_and(&IPCC->C1MR, ~IPCC_C1MR_CH4FM);
        }
    }

    __DSB();
}
