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
    { 0x041d, "HCI_READ_REMOTE_VERSION_INFORMATION" },
    { 0x0c01, "HCI_SET_EVENT_MASK" },
    { 0x0c03, "HCI_RESET" },
    { 0x0c2d, "HCI_READ_TRANSMIT_POWER_LEVEL" },
    { 0x0c31, "HCI_SET_CONTROLLER_TO_HOST_FLOW_CONTROL" },
    { 0x0c33, "HCI_HOST_BUFFER_SIZE" },
    { 0x0c35, "HCI_HOST_NUMBER_OF_COMPLETED_PACKETS" },
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
    { 0x2007, "HCI_LE_READ_ADVERTISING_PHYSICAL_CHANNEL_TX_POWER" },
    { 0x2008, "HCI_LE_SET_ADVERTISING_DATA" },
    { 0x2009, "HCI_LE_SET_SCAN_RESPONSE_DATA" },
    { 0x200a, "HCI_LE_SET_ADVERTISING_ENABLE" },
    { 0x200b, "HCI_LE_SET_SCAN_PARAMETERS" },
    { 0x200c, "HCI_LE_SET_SCAN_ENABLE" },
    { 0x200d, "HCI_LE_CREATE_CONNECTION" },
    { 0x200e, "HCI_LE_CREATE_CONNECTION_CANCEL" },
    { 0x200f, "HCI_LE_READ_FILTER_ACCEPT_LIST_SIZE" },
    { 0x2010, "HCI_LE_CLEAR_FILTER_ACCEPT_LIST" },
    { 0x2011, "HCI_LE_ADD_DEVICE_TO_FILTER_ACCEPT_LIST" },
    { 0x2012, "HCI_LE_REMOVE_DEVICE_FROM_FILTER_ACCEPT_LIST" },
    { 0x2013, "HCI_LE_CONNECTION_UPDATE" },
    { 0x2014, "HCI_LE_SET_HOST_CHANNEL_CLASSIFICATION" },
    { 0x2015, "HCI_LE_READ_CHANNEL_MAP" },
    { 0x2016, "HCI_LE_READ_REMOTE_FEATURES" },
    { 0x2017, "HCI_LE_ENCRYPT" },
    { 0x2018, "HCI_LE_RAND" },
    { 0x2019, "HCI_LE_ENABLE_ENCRYPTION" },
    { 0x201a, "HCI_LE_LONG_TERM_KEY_REQUEST_REPLY" },
    { 0x201b, "HCI_LE_LONG_TERM_KEY_REQUEST_NEGATIVE_REPLY" },
    { 0x201c, "HCI_LE_READ_SUPPORTED_STATES" },
    { 0x201d, "HCI_LE_RECEIVER_TEST" },
    { 0x201e, "HCI_LE_TRANSMITTER_TEST" },
    { 0x201f, "HCI_LE_TEST_END" },
    { 0x2022, "HCI_LE_SET_DATA_LENGTH" },
    { 0x2023, "HCI_LE_READ_SUGGESTED_DEFAULT_DATA_LENGTH" },
    { 0x2024, "HCI_LE_WRITE_SUGGESTED_DEFAULT_DATA_LENGTH" },
    { 0x2025, "HCI_LE_READ_LOCAL_P256_PUBLIC_KEY" },
    { 0x2026, "HCI_LE_GENERATE_DHKEY" },
    { 0x2027, "HCI_LE_ADD_DEVICE_TO_RESOLVING_LIST" },
    { 0x2028, "HCI_LE_REMOVE_DEVICE_FROM_RESOLVING_LIST" },
    { 0x2029, "HCI_LE_CLEAR_RESOLVING_LIST" },
    { 0x202a, "HCI_LE_READ_RESOLVING_LIST_SIZE" },
    { 0x202b, "HCI_LE_READ_PEER_RESOLVABLE_ADDRESS" },
    { 0x202c, "HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS" },
    { 0x202d, "HCI_LE_SET_ADDRESS_RESOLUTION_ENABLE" },
    { 0x202e, "HCI_LE_SET_RESOLVABLE_PRIVATE_ADDRESS_TIMEOUT" },
    { 0x202f, "HCI_LE_READ_MAXIMUM_DATA_LENGTH" },
    { 0x2030, "HCI_LE_READ_PHY" },
    { 0x2031, "HCI_LE_SET_DEFAULT_PHY" },
    { 0x2032, "HCI_LE_SET_PHY" },
    { 0x2033, "HCI_LE_RECEIVER_TEST_V2" },
    { 0x2034, "HCI_LE_TRANSMITTER_TEST_V2" },
    { 0x2035, "HCI_LE_SET_ADVERTISING_SET_RANDOM_ADDRESS" },
    { 0x2036, "HCI_LE_SET_EXTENDED_ADVERTISING_PARAMETERS" },
    { 0x2037, "HCI_LE_SET_EXTENDED_ADVERTISING_DATA" },
    { 0x2038, "HCI_LE_SET_EXTENDED_SCAN_RESPONSE_DATA" },
    { 0x2039, "HCI_LE_SET_EXTENDED_ADVERTISING_ENABLE" },
    { 0x203a, "HCI_LE_READ_MAXIMUM_ADVERTISING_DATA_LENGTH" },
    { 0x203b, "HCI_LE_READ_NUMBER_OF_SUPPORTED_ADVERTISING_SETS" },
    { 0x203c, "HCI_LE_REMOVE_ADVERTISING_SET" },
    { 0x203d, "HCI_LE_CLEAR_ADVERTISING_SETS" },
    { 0x2041, "HCI_LE_SET_EXTENDED_SCAN_PARAMETERS" },
    { 0x2042, "HCI_LE_SET_EXTENDED_SCAN_ENABLE" },
    { 0x2043, "HCI_LE_EXTENDED_CREATE_CONNECTION" },
    { 0x204b, "HCI_LE_READ_TRANSMIT_POWER" },
    { 0x204c, "HCI_LE_READ_RF_PATH_COMPENSATION" },
    { 0x204d, "HCI_LE_WRITE_RF_PATH_COMPENSATION" },
    { 0x204e, "HCI_LE_SET_PRIVACY_MODE" },
    { 0xfc00, "ACI_HAL_GET_FW_BUILD_NUMBER" },
    { 0xfc0c, "ACI_HAL_WRITE_CONFIG_DATA" },
    { 0xfc0d, "ACI_HAL_READ_CONFIG_DATA" },
    { 0xfc0f, "ACI_HAL_SET_TX_POWER_LEVEL" },
    { 0xfc14, "ACI_HAL_LE_TX_TEST_PACKET_NUMBER" },
    { 0xfc15, "ACI_HAL_TONE_START" },
    { 0xfc16, "ACI_HAL_TONE_STOP" },
    { 0xfc17, "ACI_HAL_GET_LINK_STATUS" },
    { 0xfc18, "ACI_HAL_SET_RADIO_ACTIVITY_MASK" },
    { 0xfc19, "ACI_HAL_GET_ANCHOR_PERIOD" },
    { 0xfc1a, "ACI_HAL_SET_EVENT_MASK" },
    { 0xfc1c, "ACI_HAL_GET_PM_DEBUG_INFO" },
    { 0xfc20, "ACI_HAL_SET_SLAVE_LATENCY" },
    { 0xfc22, "ACI_HAL_READ_RSSI" },
    { 0xfc30, "ACI_HAL_READ_RADIO_REG" },
    { 0xfc31, "ACI_HAL_WRITE_RADIO_REG" },
    { 0xfc32, "ACI_HAL_READ_RAW_RSSI" },
    { 0xfc33, "ACI_HAL_RX_START" },
    { 0xfc34, "ACI_HAL_RX_STOP" },
    { 0xfc3b, "ACI_HAL_STACK_RESET" },
    { 0xfc81, "ACI_GAP_SET_NON_DISCOVERABLE" },
    { 0xfc82, "ACI_GAP_SET_LIMITED_DISCOVERABLE" },
    { 0xfc83, "ACI_GAP_SET_DISCOVERABLE" },
    { 0xfc84, "ACI_GAP_SET_DIRECT_CONNECTABLE" },
    { 0xfc85, "ACI_GAP_SET_IO_CAPABILITY" },
    { 0xfc86, "ACI_GAP_SET_AUTHENTICATION_REQUIREMENT" },
    { 0xfc87, "ACI_GAP_SET_AUTHORIZATION_REQUIREMENT" },
    { 0xfc88, "ACI_GAP_PASS_KEY_RESP" },
    { 0xfc89, "ACI_GAP_AUTHORIZATION_RESP" },
    { 0xfc8a, "ACI_GAP_INIT" },
    { 0xfc8b, "ACI_GAP_SET_NON_CONNECTABLE" },
    { 0xfc8c, "ACI_GAP_SET_UNDIRECTED_CONNECTABLE" },
    { 0xfc8d, "ACI_GAP_SLAVE_SECURITY_REQ" },
    { 0xfc8e, "ACI_GAP_UPDATE_ADV_DATA" },
    { 0xfc8f, "ACI_GAP_DELETE_AD_TYPE" },
    { 0xfc90, "ACI_GAP_GET_SECURITY_LEVEL" },
    { 0xfc91, "ACI_GAP_SET_EVENT_MASK" },
    { 0xfc92, "ACI_GAP_CONFIGURE_WHITELIST" },
    { 0xfc93, "ACI_GAP_TERMINATE" },
    { 0xfc94, "ACI_GAP_CLEAR_SECURITY_DB" },
    { 0xfc95, "ACI_GAP_ALLOW_REBOND" },
    { 0xfc96, "ACI_GAP_START_LIMITED_DISCOVERY_PROC" },
    { 0xfc97, "ACI_GAP_START_GENERAL_DISCOVERY_PROC" },
    { 0xfc99, "ACI_GAP_START_AUTO_CONNECTION_ESTABLISH_PROC" },
    { 0xfc9a, "ACI_GAP_START_GENERAL_CONNECTION_ESTABLISH_PROC" },
    { 0xfc9b, "ACI_GAP_START_SELECTIVE_CONNECTION_ESTABLISH_PROC" },
    { 0xfc9c, "ACI_GAP_CREATE_CONNECTION" },
    { 0xfc9d, "ACI_GAP_TERMINATE_GAP_PROC" },
    { 0xfc9e, "ACI_GAP_START_CONNECTION_UPDATE" },
    { 0xfc9f, "ACI_GAP_SEND_PAIRING_REQ" },
    { 0xfca0, "ACI_GAP_RESOLVE_PRIVATE_ADDR" },
    { 0xfca1, "ACI_GAP_SET_BROADCAST_MODE" },
    { 0xfca2, "ACI_GAP_START_OBSERVATION_PROC" },
    { 0xfca3, "ACI_GAP_GET_BONDED_DEVICES" },
    { 0xfca4, "ACI_GAP_IS_DEVICE_BONDED" },
    { 0xfca5, "ACI_GAP_NUMERIC_COMPARISON_VALUE_CONFIRM_YESNO" },
    { 0xfca6, "ACI_GAP_PASSKEY_INPUT" },
    { 0xfca7, "ACI_GAP_GET_OOB_DATA" },
    { 0xfca8, "ACI_GAP_SET_OOB_DATA" },
    { 0xfca9, "ACI_GAP_ADD_DEVICES_TO_RESOLVING_LIST" },
    { 0xfcaa, "ACI_GAP_REMOVE_BONDED_DEVICE" },
    { 0xfcab, "ACI_GAP_ADD_DEVICES_TO_LIST" },
    { 0xfcb0, "ACI_GAP_ADDITIONAL_BEACON_START" },
    { 0xfcb1, "ACI_GAP_ADDITIONAL_BEACON_STOP" },
    { 0xfcb2, "ACI_GAP_ADDITIONAL_BEACON_SET_DATA" },
    { 0xfcc0, "ACI_GAP_ADV_SET_CONFIGURATION" },
    { 0xfcc1, "ACI_GAP_ADV_SET_ENABLE" },
    { 0xfcc2, "ACI_GAP_ADV_SET_ADV_DATA" },
    { 0xfcc3, "ACI_GAP_ADV_SET_SCAN_RESP_DATA" },
    { 0xfcc4, "ACI_GAP_ADV_REMOVE_SET" },
    { 0xfcc5, "ACI_GAP_ADV_CLEAR_SETS" },
    { 0xfcc6, "ACI_GAP_ADV_SET_RANDOM_ADDRESS" },
    { 0xfd01, "ACI_GATT_INIT" },
    { 0xfd02, "ACI_GATT_ADD_SERVICE" },
    { 0xfd03, "ACI_GATT_INCLUDE_SERVICE" },
    { 0xfd04, "ACI_GATT_ADD_CHAR" },
    { 0xfd05, "ACI_GATT_ADD_CHAR_DESC" },
    { 0xfd06, "ACI_GATT_UPDATE_CHAR_VALUE" },
    { 0xfd07, "ACI_GATT_DEL_CHAR" },
    { 0xfd08, "ACI_GATT_DEL_SERVICE" },
    { 0xfd09, "ACI_GATT_DEL_INCLUDE_SERVICE" },
    { 0xfd0a, "ACI_GATT_SET_EVENT_MASK" },
    { 0xfd0b, "ACI_GATT_EXCHANGE_CONFIG" },
    { 0xfd0c, "ACI_ATT_FIND_INFO_REQ" },
    { 0xfd0d, "ACI_ATT_FIND_BY_TYPE_VALUE_REQ" },
    { 0xfd0e, "ACI_ATT_READ_BY_TYPE_REQ" },
    { 0xfd0f, "ACI_ATT_READ_BY_GROUP_TYPE_REQ" },
    { 0xfd10, "ACI_ATT_PREPARE_WRITE_REQ" },
    { 0xfd11, "ACI_ATT_EXECUTE_WRITE_REQ" },
    { 0xfd12, "ACI_GATT_DISC_ALL_PRIMARY_SERVICES" },
    { 0xfd13, "ACI_GATT_DISC_PRIMARY_SERVICE_BY_UUID" },
    { 0xfd14, "ACI_GATT_FIND_INCLUDED_SERVICES" },
    { 0xfd15, "ACI_GATT_DISC_ALL_CHAR_OF_SERVICE" },
    { 0xfd16, "ACI_GATT_DISC_CHAR_BY_UUID" },
    { 0xfd17, "ACI_GATT_DISC_ALL_CHAR_DESC" },
    { 0xfd18, "ACI_GATT_READ_CHAR_VALUE" },
    { 0xfd19, "ACI_GATT_READ_USING_CHAR_UUID" },
    { 0xfd1a, "ACI_GATT_READ_LONG_CHAR_VALUE" },
    { 0xfd1b, "ACI_GATT_READ_MULTIPLE_CHAR_VALUE" },
    { 0xfd1c, "ACI_GATT_WRITE_CHAR_VALUE" },
    { 0xfd1d, "ACI_GATT_WRITE_LONG_CHAR_VALUE" },
    { 0xfd1e, "ACI_GATT_WRITE_CHAR_RELIABLE" },
    { 0xfd1f, "ACI_GATT_WRITE_LONG_CHAR_DESC" },
    { 0xfd20, "ACI_GATT_READ_LONG_CHAR_DESC" },
    { 0xfd21, "ACI_GATT_WRITE_CHAR_DESC" },
    { 0xfd22, "ACI_GATT_READ_CHAR_DESC" },
    { 0xfd23, "ACI_GATT_WRITE_WITHOUT_RESP" },
    { 0xfd24, "ACI_GATT_SIGNED_WRITE_WITHOUT_RESP" },
    { 0xfd25, "ACI_GATT_CONFIRM_INDICATION" },
    { 0xfd26, "ACI_GATT_WRITE_RESP" },
    { 0xfd27, "ACI_GATT_ALLOW_READ" },
    { 0xfd28, "ACI_GATT_SET_SECURITY_PERMISSION" },
    { 0xfd29, "ACI_GATT_SET_DESC_VALUE" },
    { 0xfd2a, "ACI_GATT_READ_HANDLE_VALUE" },
    { 0xfd2c, "ACI_GATT_UPDATE_CHAR_VALUE_EXT" },
    { 0xfd2d, "ACI_GATT_DENY_READ" },
    { 0xfd2e, "ACI_GATT_SET_ACCESS_PERMISSION" },
    { 0xfd30, "ACI_GATT_STORE_DB" },
    { 0xfd31, "ACI_GATT_SEND_MULT_NOTIFICATION" },
    { 0xfd32, "ACI_GATT_READ_MULTIPLE_VAR_CHAR_VALUE" },
    { 0xfd81, "ACI_L2CAP_CONNECTION_PARAMETER_UPDATE_REQ" },
    { 0xfd82, "ACI_L2CAP_CONNECTION_PARAMETER_UPDATE_RESP" },
    { 0xfd88, "ACI_L2CAP_COC_CONNECT" },
    { 0xfd89, "ACI_L2CAP_COC_CONNECT_CONFIRM" },
    { 0xfd8a, "ACI_L2CAP_COC_RECONF" },
    { 0xfd8b, "ACI_L2CAP_COC_RECONF_CONFIRM" },
    { 0xfd8c, "ACI_L2CAP_COC_DISCONNECT" },
    { 0xfd8d, "ACI_L2CAP_COC_FLOW_CONTROL" },
    { 0xfd8e, "ACI_L2CAP_COC_TX_DATA" },
};

const stm32wb_hci_event_table_t hci_event_table[] = {
    { 0x0005, "HCI_DISCONNECTION_COMPLETE_EVENT" },
    { 0x0008, "HCI_ENCRYPTION_CHANGE_EVENT" },
    { 0x000c, "HCI_READ_REMOTE_VERSION_INFORMATION_COMPLETE_EVENT" },
    { 0x000e, "HCI_COMMAND_COMPLETE_EVENT" },
    { 0x000f, "HCI_COMMAND_STATUS_EVENT" },
    { 0x0010, "HCI_HARDWARE_ERROR_EVENT" },
    { 0x0013, "HCI_NUMBER_OF_COMPLETED_PACKETS_EVENT" },
    { 0x0030, "HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT" },
};

const stm32wb_hci_event_table_t hci_le_event_table[] = {
    { 0x0001, "HCI_LE_CONNECTION_COMPLETE_EVENT" },
    { 0x0002, "HCI_LE_ADVERTISING_REPORT_EVENT" },
    { 0x0003, "HCI_LE_CONNECTION_UPDATE_COMPLETE_EVENT" },
    { 0x0004, "HCI_LE_READ_REMOTE_FEATURES_COMPLETE_EVENT" },
    { 0x0005, "HCI_LE_LONG_TERM_KEY_REQUEST_EVENT" },
    { 0x0007, "HCI_LE_DATA_LENGTH_CHANGE_EVENT" },
    { 0x0008, "HCI_LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE_EVENT" },
    { 0x0009, "HCI_LE_GENERATE_DHKEY_COMPLETE_EVENT" },
    { 0x000a, "HCI_LE_ENHANCED_CONNECTION_COMPLETE_EVENT" },
    { 0x000b, "HCI_LE_DIRECTED_ADVERTISING_REPORT_EVENT" },
    { 0x000c, "HCI_LE_PHY_UPDATE_COMPLETE_EVENT" },
    { 0x000d, "HCI_LE_EXTENDED_ADVERTISING_REPORT_EVENT" },
    { 0x0011, "HCI_LE_SCAN_TIMEOUT_EVENT" },
    { 0x0012, "HCI_LE_ADVERTISING_SET_TERMINATED_EVENT" },
    { 0x0013, "HCI_LE_SCAN_REQUEST_RECEIVED_EVENT" },
    { 0x0014, "HCI_LE_CHANNEL_SELECTION_ALGORITHM_EVENT" },
};

const stm32wb_hci_event_table_t hci_vs_event_table[] = {
    { 0x0004, "ACI_HAL_END_OF_RADIO_ACTIVITY_EVENT" },
    { 0x0005, "ACI_HAL_SCAN_REQ_REPORT_EVENT" },
    { 0x0006, "ACI_HAL_FW_ERROR_EVENT" },
    { 0x0400, "ACI_GAP_LIMITED_DISCOVERABLE_EVENT" },
    { 0x0401, "ACI_GAP_PAIRING_COMPLETE_EVENT" },
    { 0x0402, "ACI_GAP_PASS_KEY_REQ_EVENT" },
    { 0x0403, "ACI_GAP_AUTHORIZATION_REQ_EVENT" },
    { 0x0404, "ACI_GAP_SLAVE_SECURITY_INITIATED_EVENT" },
    { 0x0405, "ACI_GAP_BOND_LOST_EVENT" },
    { 0x0407, "ACI_GAP_PROC_COMPLETE_EVENT" },
    { 0x0408, "ACI_GAP_ADDR_NOT_RESOLVED_EVENT" },
    { 0x0409, "ACI_GAP_NUMERIC_COMPARISON_VALUE_EVENT" },
    { 0x040a, "ACI_GAP_KEYPRESS_NOTIFICATION_EVENT" },
    { 0x0800, "ACI_L2CAP_CONNECTION_UPDATE_RESP_EVENT" },
    { 0x0801, "ACI_L2CAP_PROC_TIMEOUT_EVENT" },
    { 0x0802, "ACI_L2CAP_CONNECTION_UPDATE_REQ_EVENT" },
    { 0x080a, "ACI_L2CAP_COMMAND_REJECT_EVENT" },
    { 0x0810, "ACI_L2CAP_COC_CONNECT_EVENT" },
    { 0x0811, "ACI_L2CAP_COC_CONNECT_CONFIRM_EVENT" },
    { 0x0812, "ACI_L2CAP_COC_RECONF_EVENT" },
    { 0x0813, "ACI_L2CAP_COC_RECONF_CONFIRM_EVENT" },
    { 0x0814, "ACI_L2CAP_COC_DISCONNECT_EVENT" },
    { 0x0815, "ACI_L2CAP_COC_FLOW_CONTROL_EVENT" },
    { 0x0816, "ACI_L2CAP_COC_RX_DATA_EVENT" },
    { 0x0817, "ACI_L2CAP_COC_TX_POOL_AVAILABLE_EVENT" },
    { 0x0c01, "ACI_GATT_ATTRIBUTE_MODIFIED_EVENT" },
    { 0x0c02, "ACI_GATT_PROC_TIMEOUT_EVENT" },
    { 0x0c03, "ACI_ATT_EXCHANGE_MTU_RESP_EVENT" },
    { 0x0c04, "ACI_ATT_FIND_INFO_RESP_EVENT" },
    { 0x0c05, "ACI_ATT_FIND_BY_TYPE_VALUE_RESP_EVENT" },
    { 0x0c06, "ACI_ATT_READ_BY_TYPE_RESP_EVENT" },
    { 0x0c07, "ACI_ATT_READ_RESP_EVENT" },
    { 0x0c08, "ACI_ATT_READ_BLOB_RESP_EVENT" },
    { 0x0c09, "ACI_ATT_READ_MULTIPLE_RESP_EVENT" },
    { 0x0c0a, "ACI_ATT_READ_BY_GROUP_TYPE_RESP_EVENT" },
    { 0x0c0c, "ACI_ATT_PREPARE_WRITE_RESP_EVENT" },
    { 0x0c0d, "ACI_ATT_EXEC_WRITE_RESP_EVENT" },
    { 0x0c0e, "ACI_GATT_INDICATION_EVENT" },
    { 0x0c0f, "ACI_GATT_NOTIFICATION_EVENT" },
    { 0x0c10, "ACI_GATT_PROC_COMPLETE_EVENT" },
    { 0x0c11, "ACI_GATT_ERROR_RESP_EVENT" },
    { 0x0c12, "ACI_GATT_DISC_READ_CHAR_BY_UUID_RESP_EVENT" },
    { 0x0c13, "ACI_GATT_WRITE_PERMIT_REQ_EVENT" },
    { 0x0c14, "ACI_GATT_READ_PERMIT_REQ_EVENT" },
    { 0x0c15, "ACI_GATT_READ_MULTI_PERMIT_REQ_EVENT" },
    { 0x0c16, "ACI_GATT_TX_POOL_AVAILABLE_EVENT" },
    { 0x0c17, "ACI_GATT_SERVER_CONFIRMATION_EVENT" },
    { 0x0c18, "ACI_GATT_PREPARE_WRITE_PERMIT_REQ_EVENT" },
    { 0x0c19, "ACI_GATT_EATT_BEARER_EVENT" },
    { 0x0c1a, "ACI_GATT_MULT_NOTIFICATION_EVENT" },
    { 0x0c1d, "ACI_GATT_READ_EXT_EVENT" },
    { 0x0c1e, "ACI_GATT_INDICATION_EXT_EVENT" },
    { 0x0c1f, "ACI_GATT_NOTIFICATION_EXT_EVENT" },
};

#endif

#define HCI_COMMAND_DATA_PACKET        0x01
#define HCI_ACL_DATA_PACKET            0x02
#define HCI_EVENT_PACKET               0x04

#define SYS_COMMAND_PACKET             0x10
#define SYS_RESPONSE_PACKET            0x11
#define SYS_EVENT_PACKET               0x12

#define STM32WB_IPCC_SYS_COMMAND_SENTINEL ((stm32wb_ipcc_sys_command_t*)0x00000001)
#define STM32WB_IPCC_BLE_COMMAND_SENTINEL ((stm32wb_ipcc_ble_command_t*)0x00000001)
#define STM32WB_IPCC_BLE_ACLDATA_SENTINEL ((stm32wb_ipcc_ble_acldata_t*)0x00000001)

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
 // uint8_t                       data[132]; /* type (1), opcode (2), length(1), payload(128) */
    uint8_t                       data[260]; /* type (1), opcode (2), length(1), payload(255), pad(1) */
} MB_SysCommand_t;

typedef struct _MB_SysEvent_t {
    MB_Node_t                     node;
 // uint8_t                       data[36];  /* type (1), evtcode (1), length(1), payload(32) */
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

typedef struct _MB_Data_t {
    MB_RefTable_t                 MB_RefTable;
    volatile uint32_t             MB_SysState;
    MB_DeviceInfoTable_t          MB_DeviceInfoTable;
    MB_BleTable_t                 MB_BleTable;
    MB_SysTable_t                 MB_SysTable;
    MB_MemManagerTable_t          MB_MemManagerTable;
    MB_Queue_t                    MB_SysEvtQueue;
    MB_SysEvent_t                 MB_SpareSysBuffer;
    MB_SysCommand_t               MB_SysCmdBuffer;
    MB_Queue_t                    MB_EvtFreeBufferQueue;
    MB_Queue_t                    MB_BleEvtQueue;
    MB_BleEvent_t                 MB_SpareBleBuffer;
    MB_BleCommand_t               MB_BleCmdBuffer;
    MB_BleCsEvent_t               MB_BleCsBuffer;
    uint8_t                       MB_BlePool[((sizeof(MB_BleEvent_t) + 3) & ~3) * STM32WB_IPCC_BLE_EVENT_POOL_ENTRIES];
#if (STM32WB_IPCC_BLE_ACLDATA_SUPPORTED == 1)
    MB_BleAclData_t               MB_BleAclDataBuffer;
#endif /* STM32WB_IPCC_BLE_ACLDATA_SUPPORTED == 1 */
} MB_Data_t;

#if !defined(__STM32WB_BOOT_CODE__)
static __attribute__((section(".ipcc"), used)) MB_Data_t MB_Data;
#else /* defined(__STM32WB_BOOT_CODE__) */
#define MB_Data (*((MB_Data_t*)STM32WB_IPCC_MB_BASE))
#endif /* !defined(__STM32WB_BOOT_CODE__) */

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
#if !defined(__STM32WB_BOOT_CODE__)
    volatile uint8_t                      mm_busy;
    MB_Queue_t                            mm_queue;
    stm32wb_ipcc_sys_command_t            *sys_current;
#if (STM32WB_IPCC_BLE_SUPPORTED == 1)
    stm32wb_ipcc_ble_command_t            *ble_head;
    stm32wb_ipcc_ble_command_t            *ble_tail;
    stm32wb_ipcc_ble_command_t * volatile ble_submit;
    stm32wb_ipcc_ble_command_t            *ble_current;
#if (STM32WB_IPCC_BLE_ACLDATA_SUPPORTED == 1)
    stm32wb_ipcc_ble_acldata_t            *acl_current;
#endif /* STM32WB_IPCC_BLE_ACLDATA_SUPPORTED == 1 */
    stm32wb_ipcc_ble_event_callback_t     evt_callback;
    void                                  *evt_context;
    MB_BleEvent_t                         *evt_head;
    MB_BleEvent_t                         *evt_tail;
    MB_BleEvent_t * volatile              evt_acquire;
    MB_BleEvent_t * volatile              evt_release;
    MB_BleEvent_t                         *evt_current;
#endif /* STM32WB_IPCC_BLE_SUPPORTED == 1 */
#endif /* !defined(__STM32WB_BOOT_CODE__) */
} stm32wb_ipcc_device_t;

static stm32wb_ipcc_device_t stm32wb_ipcc_device;

/*********************************************************************************************/

bool stm32wb_ipcc_sys_enable(void)
{
    if (!(PWR->CR4 & PWR_CR4_C2BOOT))
    {
#if !defined(__STM32WB_BOOT_CODE__)
        memset((void*)STM32WB_IPCC_MB_BASE, 0, STM32WB_IPCC_MB_SIZE);
#else /* defined(__STM32WB_BOOT_CODE__) */
        stm32wb_boot_memset((void*)STM32WB_IPCC_MB_BASE, 0, STM32WB_IPCC_MB_SIZE);
#endif /* !defined(__STM32WB_BOOT_CODE__) */

        MB_QueueInit(&MB_Data.MB_BleEvtQueue);
        MB_QueueInit(&MB_Data.MB_SysEvtQueue);
        MB_QueueInit(&MB_Data.MB_EvtFreeBufferQueue);

        MB_Data.MB_RefTable.p_device_info_table = &MB_Data.MB_DeviceInfoTable;
        MB_Data.MB_RefTable.p_ble_table = &MB_Data.MB_BleTable;
        MB_Data.MB_RefTable.p_sys_table = &MB_Data.MB_SysTable;
        MB_Data.MB_RefTable.p_mem_manager_table = &MB_Data.MB_MemManagerTable;

        MB_Data.MB_BleTable.pcmd_buffer = &MB_Data.MB_BleCmdBuffer; 
        MB_Data.MB_BleTable.pcs_buffer = &MB_Data.MB_BleCsBuffer; 
        MB_Data.MB_BleTable.pevt_queue = &MB_Data.MB_BleEvtQueue; 
#if (STM32WB_IPCC_BLE_ACLDATA_SUPPORTED == 1)
        MB_Data.MB_BleTable.pacl_data_buffer = &MB_Data.MB_BleAclDataBuffer;
#endif /* STM32WB_IPCC_BLE_ACLDATA_SUPPORTED == 1 */
        
        MB_Data.MB_SysTable.pcmd_buffer = &MB_Data.MB_SysCmdBuffer; 
        MB_Data.MB_SysTable.pevt_queue = &MB_Data.MB_SysEvtQueue; 

        MB_Data.MB_MemManagerTable.spare_sys_buffer = &MB_Data.MB_SpareSysBuffer;
        MB_Data.MB_MemManagerTable.spare_ble_buffer = &MB_Data.MB_SpareBleBuffer;
        MB_Data.MB_MemManagerTable.pevt_free_buffer_queue = &MB_Data.MB_EvtFreeBufferQueue;
        MB_Data.MB_MemManagerTable.blepool = MB_Data.MB_BlePool;
        MB_Data.MB_MemManagerTable.blepoolsize = sizeof(MB_Data.MB_BlePool);

        MB_Data.MB_DeviceInfoTable.Fus.DeviceInfoTableState = 0;
    }

    if (stm32wb_ipcc_device.state != STM32WB_IPCC_STATE_NONE)
    {
        return false;
    }
    
    stm32wb_ipcc_device.state = (MB_Data.MB_SysState == STM32WB_IPCC_SYS_STATE_NONE) ? STM32WB_IPCC_STATE_NOT_READY : STM32WB_IPCC_STATE_READY;

#if !defined(__STM32WB_BOOT_CODE__)
    stm32wb_ipcc_device.mm_busy = false;
    MB_QueueInit(&stm32wb_ipcc_device.mm_queue);

    stm32wb_ipcc_device.sys_current = NULL;
#endif /* !defined(__STM32WB_BOOT_CODE__) */
    
#if (STM32WB_IPCC_BLE_SUPPORTED == 1)
    stm32wb_ipcc_device.ble_head = STM32WB_IPCC_BLE_COMMAND_SENTINEL;
    stm32wb_ipcc_device.ble_tail = STM32WB_IPCC_BLE_COMMAND_SENTINEL;
    stm32wb_ipcc_device.ble_submit = STM32WB_IPCC_BLE_COMMAND_SENTINEL;
    stm32wb_ipcc_device.ble_current = NULL;

#if (STM32WB_IPCC_BLE_ACLDATA_SUPPORTED == 1)
    stm32wb_ipcc_device.acl_current = NULL;
#endif /* STM32WB_IPCC_BLE_ACLDATA_SUPPORTED == 1 */
    
    stm32wb_ipcc_device.evt_head = NULL;
    stm32wb_ipcc_device.evt_tail = NULL;
    stm32wb_ipcc_device.evt_acquire = NULL;
    stm32wb_ipcc_device.evt_release = NULL;
    stm32wb_ipcc_device.evt_current = NULL;
#endif /* STM32WB_IPCC_BLE_SUPPORTED == 1 */

    armv7m_atomic_or(&RCC->AHB3ENR, RCC_AHB3ENR_IPCCEN);
    RCC->AHB3ENR;
    
#if !defined(__STM32WB_BOOT_CODE__)
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
#else /* !defined(__STM32WB_BOOT_CODE__) */
    RCC->EXTCFGR = (RCC->EXTCFGR & ~(RCC_EXTCFGR_C2HPRE | RCC_EXTCFGR_C2HPREF)) | RCC_EXTCFGR_C2HPRE_DIV2;
    
    while ((RCC->EXTCFGR & (RCC_EXTCFGR_C2HPREF)) != (RCC_EXTCFGR_C2HPREF))
    {
    }

    PWR->CR4 |= PWR_CR4_C2BOOT;
#endif /* !defined(__STM32WB_BOOT_CODE__) */

    while (stm32wb_ipcc_device.state != STM32WB_IPCC_STATE_READY)
    {
        if ((volatile uint32_t)MB_Data.MB_DeviceInfoTable.Fus.DeviceInfoTableState == 0xa94656b9)
        {
            MB_Data.MB_SysState = STM32WB_IPCC_SYS_STATE_FUS;
            
            stm32wb_ipcc_device.state = STM32WB_IPCC_STATE_READY;
        }
    }
    
    return true;
}

void stm32wb_ipcc_sys_disable(void)
{
#if !defined(__STM32WB_BOOT_CODE__)
    armv7m_atomic_and(&EXTI->IMR2, ~EXTI_IMR2_IM36);

    armv7m_atomic_or(&IPCC->C1MR, IPCC_C1MR_CH2OM);

    IPCC->C1CR &= ~(IPCC_C1CR_TXFIE | IPCC_C1CR_RXOIE);

    NVIC_DisableIRQ(IPCC_C1_RX_IRQn);
    NVIC_DisableIRQ(IPCC_C1_TX_IRQn);
#endif /* !defined(__STM32WB_BOOT_CODE__) */

    armv7m_atomic_and(&RCC->AHB3ENR, ~RCC_AHB3ENR_IPCCEN);
    
    stm32wb_ipcc_device.state = STM32WB_IPCC_STATE_NONE;
}

uint32_t stm32wb_ipcc_sys_state(void)
{
    if (stm32wb_ipcc_device.state <= STM32WB_IPCC_STATE_NOT_READY)
    {
        return STM32WB_IPCC_SYS_STATE_NONE;
    }

    return MB_Data.MB_SysState;
}

bool stm32wb_ipcc_sys_info(stm32wb_ipcc_sys_info_t *p_info_return)
{
    if (stm32wb_ipcc_device.state <= STM32WB_IPCC_STATE_NOT_READY)
    {
        return false;
    }
    
    if (MB_Data.MB_SysState == STM32WB_IPCC_SYS_STATE_FUS)
    {
        p_info_return->SafeBootVersion = MB_Data.MB_DeviceInfoTable.Fus.SafeBootVersion;
        p_info_return->FusVersion = MB_Data.MB_DeviceInfoTable.Fus.FusVersion;
        p_info_return->FusMemorySize = MB_Data.MB_DeviceInfoTable.Fus.FusMemorySize;
        p_info_return->WirelessStackVersion = MB_Data.MB_DeviceInfoTable.Fus.WirelessStackVersion;
        p_info_return->WirelessStackMemorySize = MB_Data.MB_DeviceInfoTable.Fus.WirelessStackMemorySize;
        p_info_return->WirelessStackType = MB_Data.MB_DeviceInfoTable.Fus.CurrentWirelessStackType;
    }
    else
    {
        p_info_return->SafeBootVersion = MB_Data.MB_DeviceInfoTable.Wireless.SafeBootInfoTable.Version;
        p_info_return->FusVersion = MB_Data.MB_DeviceInfoTable.Wireless.FusInfoTable.Version;
        p_info_return->FusMemorySize = MB_Data.MB_DeviceInfoTable.Wireless.FusInfoTable.MemorySize;
        p_info_return->WirelessStackVersion = MB_Data.MB_DeviceInfoTable.Wireless.WirelessFwInfoTable.Version;
        p_info_return->WirelessStackMemorySize = MB_Data.MB_DeviceInfoTable.Wireless.WirelessFwInfoTable.MemorySize;
        p_info_return->WirelessStackType = MB_Data.MB_DeviceInfoTable.Wireless.WirelessFwInfoTable.InfoStack & 0x000000ff;
    }

    return true;
}

bool stm32wb_ipcc_sys_command(stm32wb_ipcc_sys_command_t *command)
{
#if defined(__STM32WB_BOOT_CODE__)
    uint32_t length;
    const uint8_t *data;
#endif /* defined(__STM32WB_BOOT_CODE__) */
    
#if !defined(__STM32WB_BOOT_CODE__)
    if (stm32wb_ipcc_device.state <= STM32WB_IPCC_STATE_NOT_READY)
    {
        return false;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&stm32wb_ipcc_device.sys_current, (uint32_t)NULL, (uint32_t)command) != (uint32_t)NULL)
    {
        return false;
    }

    command->status = STM32WB_IPCC_SYS_COMMAND_STATUS_BUSY;
    
    MB_Data.MB_SysCmdBuffer.data[0] = SYS_COMMAND_PACKET;
    MB_Data.MB_SysCmdBuffer.data[1] = (command->opcode >> 0);
    MB_Data.MB_SysCmdBuffer.data[2] = (command->opcode >> 8);
    MB_Data.MB_SysCmdBuffer.data[3] = command->clen;
    
    if (command->clen)
    {
        memcpy(&MB_Data.MB_SysCmdBuffer.data[4], command->cparam, command->clen);
    }
    
    IPCC->C1SCR = IPCC_C1SCR_CH2S;
    
    armv7m_atomic_and(&IPCC->C1MR, ~IPCC_C1MR_CH2FM);
#else /* !defined(__STM32WB_BOOT_CODE__) */
    if ((stm32wb_ipcc_device.state < STM32WB_IPCC_STATE_NOT_READY) || ((stm32wb_ipcc_device.state == STM32WB_IPCC_STATE_NOT_READY) && (command->opcode != STM32WB_IPCC_SYS_OPCODE_FUS_GET_STATE)))
    {
        return false;
    }

    command->status = STM32WB_IPCC_SYS_COMMAND_STATUS_BUSY;
    
    MB_Data.MB_SysCmdBuffer.data[0] = SYS_COMMAND_PACKET;
    MB_Data.MB_SysCmdBuffer.data[1] = (command->opcode >> 0);
    MB_Data.MB_SysCmdBuffer.data[2] = (command->opcode >> 8);
    MB_Data.MB_SysCmdBuffer.data[3] = command->clen;
    
    if (command->clen)
    {
        stm32wb_boot_memcpy(&MB_Data.MB_SysCmdBuffer.data[4], command->cparam, command->clen);
    }

    IPCC->C1SCR = IPCC_C1SCR_CH2S;

    while (IPCC->C1TOC2SR & IPCC_C1TOC2SR_CH2F)
    {
    }

    data = (const uint8_t*)&MB_Data.MB_SysCmdBuffer;

    if (command->opcode == STM32WB_IPCC_SYS_OPCODE_FUS_GET_STATE)
    {
        if (stm32wb_ipcc_device.state == STM32WB_IPCC_STATE_NOT_READY)
        {
            if ((data[6] == STM32WB_IPCC_FUS_STATE_ERROR) && (data[7] == STM32WB_IPCC_FUS_ERROR_CODE_NOT_RUNNING))
            {
                MB_Data.MB_SysState = STM32WB_IPCC_SYS_STATE_WIRELESS;
            }
            else
            {
                MB_Data.MB_SysState = STM32WB_IPCC_SYS_STATE_FUS;
            }
            
            stm32wb_ipcc_device.state = STM32WB_IPCC_STATE_READY;
        }
        
        command->status = STM32WB_IPCC_SYS_COMMAND_STATUS_SUCCESS;
    }
    else
    {
        command->status = data[6] ? STM32WB_IPCC_SYS_COMMAND_STATUS_FAILURE : STM32WB_IPCC_SYS_COMMAND_STATUS_SUCCESS;
    }
    
    if (command->rparam)
    {
        length = data[2] - 3;
                
        if (length > command->rsize)
        {
            length = command->rsize;
        }
        
        if (length)
        {
            stm32wb_boot_memcpy(command->rparam, (const void*)&data[6], length);
        }
        
        command->rlen = length;
    }
    
#endif /* !defined(__STM32WB_BOOT_CODE__) */
    
    return true;
}

bool stm32wb_ipcc_fus_state(stm32wb_ipcc_fus_state_t *p_state_return)
{
    stm32wb_ipcc_sys_command_t command;
    
    command.opcode = STM32WB_IPCC_SYS_OPCODE_FUS_GET_STATE;
    command.cparam = NULL;
    command.clen = 0;
    command.rparam = (void*)p_state_return;
    command.rsize = sizeof(stm32wb_ipcc_fus_state_t);
#if !defined(__STM32WB_BOOT_CODE__)
    command.callback = NULL;
    command.context = NULL;
            
    stm32wb_ipcc_sys_command(&command);

    while (command.status == STM32WB_IPCC_SYS_COMMAND_STATUS_BUSY)
    {
    }
#else /* !defined(__STM32WB_BOOT_CODE__) */
    stm32wb_ipcc_sys_command(&command);
#endif /* !defined(__STM32WB_BOOT_CODE__) */

    return (command.status == STM32WB_IPCC_SYS_COMMAND_STATUS_SUCCESS);
}

bool stm32wb_ipcc_fus_command(uint16_t opcode)
{
    stm32wb_ipcc_sys_command_t command;
    
    command.opcode = opcode;
    command.cparam = NULL;
    command.clen = 0;
    command.rparam = NULL;
    command.rsize = 0;
#if !defined(__STM32WB_BOOT_CODE__)
    command.callback = NULL;
    command.context = NULL;
            
    stm32wb_ipcc_sys_command(&command);

    while (command.status == STM32WB_IPCC_SYS_COMMAND_STATUS_BUSY)
    {
    }
#else /* !defined(__STM32WB_BOOT_CODE__) */
    stm32wb_ipcc_sys_command(&command);
#endif /* !defined(__STM32WB_BOOT_CODE__) */

    return (command.status == STM32WB_IPCC_SYS_COMMAND_STATUS_SUCCESS);
}

#if !defined(__STM32WB_BOOT_CODE__)

bool stm32wb_ipcc_sys_firmware(uint32_t version, uint32_t type, uint32_t address, const uint8_t *image, uint32_t size, const uint8_t *fus, const uint8_t *fus_for_0_5_3, uint32_t *p_code_return)
{
    static uint8_t firmware_data[1024];
    const uint8_t *firmware_image;
    uint32_t firmware_address, firmware_size, firmware_offset, firmware_chunk, firmware_index;
    stm32wb_ipcc_fus_state_t fus_state;
    stm32wb_ipcc_sys_info_t sys_info;
    stm32wb_flash_request_t request;

    if (!stm32wb_ipcc_sys_info(&sys_info))
    {
        *p_code_return = 0xffff0000;

        return false;
    }

    if (((sys_info.FusVersion & 0xffff0000) < 0x01020000) && (!fus || !fus_for_0_5_3))
    {
        *p_code_return = 0xffff0001;

        return false;
    }

    request.next = NULL;
    request.callback = NULL;
    request.context = NULL;
    
    *p_code_return = 0x00000000;
    
    if (((sys_info.FusVersion & 0xffff0000) < 0x01020000) ||
        ((sys_info.WirelessStackType & 0x000000ff) != type) ||
        ((sys_info.WirelessStackVersion & 0xffffff00) != version))
    {
        if (stm32wb_ipcc_sys_state() == STM32WB_IPCC_SYS_STATE_WIRELESS)
        {
            stm32wb_ipcc_fus_command(STM32WB_IPCC_SYS_OPCODE_FUS_GET_STATE);
            stm32wb_ipcc_fus_command(STM32WB_IPCC_SYS_OPCODE_FUS_GET_STATE);
            
            while (1)
            {
            }
        }

        do
        {
            stm32wb_ipcc_fus_state(&fus_state);

            if (fus_state.State != STM32WB_IPCC_FUS_STATE_IDLE)
            {
                if ((fus_state.State >= STM32WB_IPCC_FUS_STATE_FW_UPGRD_ONGOING_START) && (fus_state.State <= STM32WB_IPCC_FUS_STATE_SERVICE_ONGOING_END))
                {
                    armv7m_core_udelay(100000);
                }
                else
                {
                    stm32wb_system_reset();
                }
            }
        }
        while (fus_state.State != STM32WB_IPCC_FUS_STATE_IDLE);

        if (sys_info.WirelessStackMemorySize)
        {
            if (!stm32wb_ipcc_fus_command(STM32WB_IPCC_SYS_OPCODE_FUS_FW_DELETE))
            {
                *p_code_return = (STM32WB_IPCC_SYS_OPCODE_FUS_FW_DELETE << 16);

                return false;
            }

            while (1)
            {
                stm32wb_ipcc_fus_state(&fus_state);

                if ((fus_state.State >= STM32WB_IPCC_FUS_STATE_FW_UPGRD_ONGOING_START) && (fus_state.State <= STM32WB_IPCC_FUS_STATE_SERVICE_ONGOING_END))
                {
                    armv7m_core_udelay(100000);
                }
                else
                {
                    stm32wb_system_reset();
                }
            }
        }
                
        if ((sys_info.FusVersion & 0xffff0000) < 0x01020000)
        {
            if ((sys_info.FusVersion & 0xff000000) != 0x00000000)
            {
                firmware_image = fus;
            }
            else
            {
                firmware_image = fus_for_0_5_3;
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
        
        request.control = STM32WB_FLASH_CONTROL_ERASE | STM32WB_FLASH_CONTROL_FLUSH;
        request.address = firmware_address;
        request.count = ((FLASH_BASE + ((FLASH->SFR & FLASH_SFR_SFSA) * 4096)) - firmware_address);
        request.data = NULL;
        
        stm32wb_flash_request(&request);

        while (request.status == STM32WB_FLASH_STATUS_BUSY)
        {
        }
                    
        if (request.status != STM32WB_FLASH_STATUS_SUCCESS)
        {
            *p_code_return = 0xffff0002;

            return false;
        }

        for (firmware_offset = 0; firmware_offset < firmware_size; firmware_offset += firmware_chunk)
        {
            firmware_chunk = firmware_size - firmware_offset;
            
            if (firmware_chunk > sizeof(firmware_data))
            {
                firmware_chunk = sizeof(firmware_data);
            }
                                
            for (firmware_index = 0; firmware_index < firmware_chunk; firmware_index++)
            {
                firmware_data[firmware_index] = firmware_image[firmware_offset + firmware_index] ^ 0xff;
            }
            
            request.control = STM32WB_FLASH_CONTROL_PROGRAM | STM32WB_FLASH_CONTROL_FLUSH;
            request.address = firmware_address + firmware_offset;
            request.count = firmware_chunk;
            request.data = (const uint8_t*)&firmware_data[0];
            
            stm32wb_flash_request(&request);
            
            while (request.status == STM32WB_FLASH_STATUS_BUSY)
            {
            }
            
            if (request.status != STM32WB_FLASH_STATUS_SUCCESS)
            {
                *p_code_return = 0xffff0003;

                return false;
            }
        }
        
        if (!stm32wb_ipcc_fus_command(STM32WB_IPCC_SYS_OPCODE_FUS_FW_UPGRADE))
        {
            *p_code_return = (STM32WB_IPCC_SYS_OPCODE_FUS_FW_UPGRADE << 16);

            return false;
        }

        while (1)
        {
            stm32wb_ipcc_fus_state(&fus_state);

            if ((fus_state.State >= STM32WB_IPCC_FUS_STATE_FW_UPGRD_ONGOING_START) && (fus_state.State <= STM32WB_IPCC_FUS_STATE_SERVICE_ONGOING_END))
            {
                armv7m_core_udelay(100000);
            }
            else
            {
                stm32wb_system_reset();
            }
        }
    }
    else
    {
        if (stm32wb_ipcc_sys_state() != STM32WB_IPCC_SYS_STATE_WIRELESS)
        {
            if (!stm32wb_ipcc_fus_command(STM32WB_IPCC_SYS_OPCODE_FUS_START_WS))
            {
                *p_code_return = (STM32WB_IPCC_SYS_OPCODE_FUS_START_WS << 16);

                return false;
            }

            while (1)
            {
            }
        }
    }
    
    return true;
}

#if (STM32WB_IPCC_BLE_SUPPORTED == 1)

bool stm32wb_ipcc_ble_enable(const stm32wb_ipcc_ble_init_params_t *params, stm32wb_ipcc_ble_event_callback_t callback, void *context)
{
    uint8_t activity_control;
    stm32wb_ipcc_sys_command_t command;

    if (stm32wb_ipcc_device.state != STM32WB_IPCC_STATE_READY)
    {
        return false;
    }
    
    if (MB_Data.MB_SysState == STM32WB_IPCC_SYS_STATE_FUS)
    {
        if (!MB_Data.MB_DeviceInfoTable.Fus.WirelessStackMemorySize)
        {
            return false;
        }
        
        if (!stm32wb_ipcc_fus_command(STM32WB_IPCC_SYS_OPCODE_FUS_START_WS))
        {
            return false;
        }

        while ((volatile uint32_t)MB_Data.MB_DeviceInfoTable.Fus.DeviceInfoTableState == 0xa94656b9)
        {
        }

        MB_Data.MB_SysState = STM32WB_IPCC_SYS_STATE_WIRELESS;
    }

    if (!stm32wb_system_wireless_enable())
    {
        return false;
    }

    activity_control = STM32WB_IPCC_SYS_SET_FLASH_ACTIVITY_CONTROL_HSEM;
    
    command.opcode = STM32WB_IPCC_SYS_OPCODE_SET_FLASH_ACTIVITY_CONTROL; 
    command.event = 0;
    command.cparam = &activity_control;
    command.clen = sizeof(activity_control);
    command.rparam = NULL;
    command.rsize = 0;
    command.callback = NULL;
    command.context = NULL;
    
    stm32wb_ipcc_sys_command(&command);

    while (command.status == STM32WB_IPCC_SYS_COMMAND_STATUS_BUSY) { __WFE(); }
    
    if (command.status != STM32WB_IPCC_SYS_COMMAND_STATUS_SUCCESS)
    {
        stm32wb_system_wireless_disable();

        return false;
    }

    stm32wb_ipcc_device.evt_callback = callback;
    stm32wb_ipcc_device.evt_context = context;
    
    armv7m_atomic_and(&IPCC->C1MR, ~IPCC_C1MR_CH1OM);

    command.opcode = STM32WB_IPCC_SYS_OPCODE_BLE_INIT;
    command.event = 0;
    command.cparam = params;
    command.clen = sizeof(stm32wb_ipcc_ble_init_params_t);
    command.rparam = NULL;
    command.rsize = 0;
    command.callback = NULL;
    command.context = NULL;
    
    stm32wb_ipcc_sys_command(&command);

    while (command.status == STM32WB_IPCC_SYS_COMMAND_STATUS_BUSY) { __WFE(); }
    
    if (command.status != STM32WB_IPCC_SYS_COMMAND_STATUS_SUCCESS)
    {
        armv7m_atomic_or(&IPCC->C1MR, IPCC_C1MR_CH1OM);
        
        stm32wb_system_wireless_disable();

        return false;
    }
    
    stm32wb_ipcc_device.state = STM32WB_IPCC_STATE_WIRELESS;
    
    return true;
}

bool stm32wb_ipcc_ble_disable(void)
{
    if (stm32wb_ipcc_device.state != STM32WB_IPCC_STATE_WIRELESS)
    {
        return false;
    }

    stm32wb_ipcc_device.state = STM32WB_IPCC_STATE_READY;
    
    armv7m_atomic_or(&IPCC->C1MR, IPCC_C1MR_CH1OM);
        
    stm32wb_system_wireless_disable();
    
    return true;
}

bool stm32wb_ipcc_ble_command(stm32wb_ipcc_ble_command_t *command)
{
    stm32wb_ipcc_ble_command_t *command_submit;
    
    if (stm32wb_ipcc_device.state != STM32WB_IPCC_STATE_WIRELESS)
    {
        return false;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&command->next, (uint32_t)NULL, (uint32_t)STM32WB_IPCC_BLE_COMMAND_SENTINEL) != (uint32_t)NULL)
    {
        return false;
    }

    command->status = STM32WB_IPCC_BLE_COMMAND_STATUS_BUSY;

    do
    {
        command_submit = stm32wb_ipcc_device.ble_submit;

        command->next = command_submit;
    }
    while ((stm32wb_ipcc_ble_command_t*)armv7m_atomic_cas((volatile uint32_t*)&stm32wb_ipcc_device.ble_submit, (uint32_t)command_submit, (uint32_t)command) != command_submit);
    
    if (command_submit == STM32WB_IPCC_BLE_COMMAND_SENTINEL)
    {
        if (__current_irq() != IPCC_C1_RX_IRQn)
        {
            NVIC_SetPendingIRQ(IPCC_C1_RX_IRQn);
        }
    }
            
    return true;
}

#if (STM32WB_IPCC_BLE_ACLDATA_SUPPORTED == 1)

bool stm32wb_ipcc_ble_acldata(stm32wb_ipcc_ble_acldata_t *acl_data)
{
    if (acl_data->count > (sizeof(MB_Data.MB_BleAclDataBuffer.data) -1))
    {
        return false;
    }

    if (stm32wb_ipcc_device.state != STM32WB_IPCC_STATE_WIRELESS)
    {
        return false;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&stm32wb_ipcc_device.acl_current, (uint32_t)NULL, (uint32_t)acl_data) != (uint32_t)NULL)
    {
        return false;
    }
 
    acl_data->status = STM32WB_IPCC_BLE_ACLDATA_STATUS_BUSY;

    MB_Data.MB_BleAclDataBuffer.data[0] = HCI_ACL_DATA_PACKET;
    memcpy(&MB_Data.MB_BleAclDataBuffer.data[1], acl_data->data, acl_data->count);

    IPCC->C1SCR = IPCC_C1SCR_CH6S;
            
    armv7m_atomic_and(&IPCC->C1MR, ~IPCC_C1MR_CH6FM);

    return true;
}

#endif /* STM32WB_IPCC_BLE_ACLDATA_SUPPORTED == 1 */

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

            ble_event->node.next = &ble_event_release->node;
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

            ble_event_acquire->node.next = &ble_event_head->node;

            ble_event_head = ble_event_acquire;
        }
        
        if (!stm32wb_ipcc_device.evt_head)
        {
            stm32wb_ipcc_device.evt_head = ble_event_head;
        }
        else
        {
            stm32wb_ipcc_device.evt_tail->node.next = &ble_event_head->node;
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
                        armv7m_rtt_printf("== %s\r\n", hci_vs_event_table[index].cstring);
                        break;
                    }
                }

                if (index == (sizeof(hci_vs_event_table) / sizeof(hci_vs_event_table[0])))
                {
                    armv7m_rtt_printf("?? UNEXPECTED HCI VS EVENT = %04x\r\n", evtcode);
                }
            }
            else if (ble_event->data[1] == HCI_LE_META_EVENT)
            {
                evtcode = ble_event->data[3];

                for (index = 0; index < (sizeof(hci_le_event_table) / sizeof(hci_le_event_table[0])); index++)
                {
                    if (hci_le_event_table[index].evtcode == evtcode)
                    {
                        armv7m_rtt_printf("== %s\r\n", hci_le_event_table[index].cstring);
                        break;
                    }
                }

                if (index == (sizeof(hci_le_event_table) / sizeof(hci_le_event_table[0])))
                {
                    armv7m_rtt_printf("?? UNEXPECTED HCI LE EVENT = %02x\r\n", evtcode);
                }
            }
            else
            {
                evtcode = ble_event->data[1];
                
                for (index = 0; index < (sizeof(hci_event_table) / sizeof(hci_event_table[0])); index++)
                {
                    if (hci_event_table[index].evtcode == evtcode)
                    {
                        armv7m_rtt_printf("== %s\r\n", hci_event_table[index].cstring);
                        break;
                    }
                }

                if (index == (sizeof(hci_event_table) / sizeof(hci_event_table[0])))
                {
                    armv7m_rtt_printf("?? UNEXPECTED HCI EVENT = %02x\r\n", evtcode);
                }
            }
        }
        else
        {
            armv7m_rtt_printf("?? UNEXPECTED EVENT PACKET = %02x\r\n", ble_event->data[0]);
        }
#endif /* STM32WB_IPCC_TRACE_SUPPORTED == 1 */
        
        return &ble_event->data[0];
    }

    return NULL;
}

#endif /* STM32WB_IPCC_BLE_SUPPORTED == 1 */

static void stm32wb_ipcc_rx_interrupt(void)
{
    uint32_t ipcc_c1mr, ipcc_c2toc1sr, evt_code, evt_count, length;
    const uint8_t *data;
    MB_SysEvent_t *sys_event;
#if (STM32WB_IPCC_BLE_SUPPORTED == 1)
    uint16_t opcode;
    stm32wb_ipcc_ble_command_t *ble_command, *ble_command_submit, *ble_command_next, *ble_command_head, *ble_command_tail;
    stm32wb_ipcc_ble_command_callback_t ble_callback;
    void *ble_context;
    MB_BleEvent_t *ble_event, *ble_event_next, *ble_event_acquire, *ble_event_release;
#endif /* STM32WB_IPCC_BLE_SUPPORTED == 1 */
    
    ipcc_c1mr = IPCC->C1MR;
    ipcc_c2toc1sr = IPCC->C2TOC1SR;

#if (STM32WB_IPCC_BLE_SUPPORTED == 1)
    if (!(ipcc_c1mr & IPCC_C1MR_CH1OM) && (ipcc_c2toc1sr & IPCC_C2TOC1SR_CH1F))
    {
        /* BLE RX EVT */

        evt_count = 0;
        
        while (!MB_QueueIsEmpty(MB_Data.MB_BleTable.pevt_queue))
        {
            ble_event = (MB_BleEvent_t*)MB_QueueRemove(MB_Data.MB_BleTable.pevt_queue);

            data = &ble_event->data[0];

            if (data[0] == HCI_EVENT_PACKET)
            {
                if (data[1] == HCI_COMMAND_COMPLETE_EVENT)
                {
                    /* type, evt, length, packets, opcode[0], opcode[1], params */

                    opcode = (data[4] << 0) | (data[5] << 8);

                    ble_command = stm32wb_ipcc_device.ble_current;

                    if (ble_command && (ble_command->opcode == opcode))
                    {
                        stm32wb_ipcc_device.ble_current = NULL;

                        if (ble_command->rparam)
                        {
                            length = (data[2] - 3);
                            
                            if (length > ble_command->rsize)
                            {
                                length = ble_command->rsize;
                            }
                            
                            if (length)
                            {
                                memcpy(ble_command->rparam, &data[6], length);
                            }
                            
                            ble_command->rlen = length;
                            
                            if (((void*)ble_event >= (void*)(&MB_Data.MB_BlePool[0])) && ((void*)ble_event < (void*)(&MB_Data.MB_BlePool[0] + sizeof(MB_Data.MB_BlePool))))
                            {
                                MB_QueueInsert(&stm32wb_ipcc_device.mm_queue, &ble_event->node);
                            }
                            
                            ble_event = NULL;
                        }

#if (STM32WB_IPCC_TRACE_SUPPORTED == 1)
                        armv7m_rtt_printf("<< HCI_COMMAND_COMPLETE_EVENT = %02x\r\n", data[6]);
#endif /* (STM32WB_IPCC_TRACE_SUPPORTED == 1) */
                        
                        ble_callback = ble_command->callback;
                        ble_context = ble_command->context;

                        ble_command->next = NULL;
                        
                        ble_command->status = STM32WB_IPCC_BLE_COMMAND_STATUS_SUCCESS;
                        
                        if (ble_callback)
                        {
                            (*ble_callback)(ble_context);
                        }
                    }
                }

                if (data[1] == HCI_COMMAND_STATUS_EVENT)
                {
                    /* type, evt, length, status, packets, opcode[0], opcode[1] */

                    opcode = (data[5] << 0) | (data[6] << 8);
                    
                    ble_command = stm32wb_ipcc_device.ble_current;

                    if (ble_command && (ble_command->opcode == opcode))
                    {
                        stm32wb_ipcc_device.ble_current = NULL;

                        if (ble_command->rparam)
                        {
                            length = 1;

                            if (length > ble_command->rsize)
                            {
                                length = ble_command->rsize;
                            }
                            
                            if (length)
                            {
                                ((uint8_t*)ble_command->rparam)[0] = data[3];
                            }
                            
                            ble_command->rlen = length;

                            if (((void*)ble_event >= (void*)(&MB_Data.MB_BlePool[0])) && ((void*)ble_event < (void*)(&MB_Data.MB_BlePool[0] + sizeof(MB_Data.MB_BlePool))))
                            {
                                MB_QueueInsert(&stm32wb_ipcc_device.mm_queue, &ble_event->node);
                            }
                                
                            ble_event = NULL;
                        }

#if (STM32WB_IPCC_TRACE_SUPPORTED == 1)
                        armv7m_rtt_printf("<< HCI_COMMAND_STATUS_EVENT = %02x\r\n", data[3]);
#endif /* (STM32WB_IPCC_TRACE_SUPPORTED == 1) */
                        
                        ble_callback = ble_command->callback;
                        ble_context = ble_command->context;

                        ble_command->next = NULL;
                        
                        ble_command->status = STM32WB_IPCC_BLE_COMMAND_STATUS_SUCCESS;
                            
                        if (ble_callback)
                        {
                            (*ble_callback)(ble_context);
                        }
                    }
                }
            }
            
            if (ble_event)
            {
                do
                {
                    ble_event_acquire = stm32wb_ipcc_device.evt_acquire;
                    
                    ble_event->node.next = &ble_event_acquire->node;
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
#endif /* STM32WB_IPCC_BLE_SUPPORTED == 1 */
    
    if (!(ipcc_c1mr & IPCC_C1MR_CH2OM) && (ipcc_c2toc1sr & IPCC_C2TOC1SR_CH2F))
    {
        /* SYS RX EVT */

        while (!MB_QueueIsEmpty(&MB_Data.MB_SysEvtQueue))
        {
            sys_event = (MB_SysEvent_t*)MB_QueueRemove(&MB_Data.MB_SysEvtQueue);

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
                            MB_Data.MB_SysState = STM32WB_IPCC_SYS_STATE_WIRELESS;
                        }

                        if (data[5] == 0x01)
                        {
                            MB_Data.MB_SysState = STM32WB_IPCC_SYS_STATE_FUS;
                        }

                        stm32wb_ipcc_device.state = STM32WB_IPCC_STATE_READY;
                    }
                }
            }

            if (((void*)sys_event >= (void*)(&MB_Data.MB_BlePool[0])) && ((void*)sys_event < (void*)(&MB_Data.MB_BlePool[0] + sizeof(MB_Data.MB_BlePool))))
            {
                MB_QueueInsert(&stm32wb_ipcc_device.mm_queue, &sys_event->node);
            }
        }

        IPCC->C1SCR = IPCC_C1SCR_CH2C;
    }

#if (STM32WB_IPCC_BLE_SUPPORTED == 1)
    if (stm32wb_ipcc_device.ble_submit != STM32WB_IPCC_BLE_COMMAND_SENTINEL)
    {
        ble_command_submit = (stm32wb_ipcc_ble_command_t*)armv7m_atomic_swap((volatile uint32_t*)&stm32wb_ipcc_device.ble_submit, (uint32_t)STM32WB_IPCC_BLE_COMMAND_SENTINEL);

        if (ble_command_submit->next == STM32WB_IPCC_BLE_COMMAND_SENTINEL)
        {
            ble_command_head = ble_command_submit;
            ble_command_tail = ble_command_submit;
        }
        else
        {
            for (ble_command_head = STM32WB_IPCC_BLE_COMMAND_SENTINEL, ble_command_tail = ble_command_submit; ble_command_submit != STM32WB_IPCC_BLE_COMMAND_SENTINEL; ble_command_submit = ble_command_next)
            {
                ble_command_next = ble_command_submit->next;
                
                ble_command_submit->next = ble_command_head;
                
                ble_command_head = ble_command_submit;
            }
        }
        
        if (stm32wb_ipcc_device.ble_head == STM32WB_IPCC_BLE_COMMAND_SENTINEL)
        {
            stm32wb_ipcc_device.ble_head = ble_command_head;
        }
        else
        {
            stm32wb_ipcc_device.ble_tail->next = ble_command_head;
        }
        
        stm32wb_ipcc_device.ble_tail = ble_command_tail;
    }
    
    if (!stm32wb_ipcc_device.ble_current)
    {
        ble_command = stm32wb_ipcc_device.ble_head;

        if (ble_command != STM32WB_IPCC_BLE_COMMAND_SENTINEL)
        {
            if (stm32wb_ipcc_device.ble_head == stm32wb_ipcc_device.ble_tail)
            {
                stm32wb_ipcc_device.ble_head = STM32WB_IPCC_BLE_COMMAND_SENTINEL;
                stm32wb_ipcc_device.ble_tail = STM32WB_IPCC_BLE_COMMAND_SENTINEL;
            }
            else
            {
                stm32wb_ipcc_device.ble_head = ble_command->next;
            }
            
            stm32wb_ipcc_device.ble_current = ble_command;

#if (STM32WB_IPCC_TRACE_SUPPORTED == 1)
            {
                unsigned int index, opcode;

                opcode = ble_command->opcode;
                
                for (index = 0; index < (sizeof(hci_command_table) / sizeof(hci_command_table[0])); index++)
                {
                    if (hci_command_table[index].opcode == opcode)
                    {
                        armv7m_rtt_printf(">> %s\r\n", hci_command_table[index].cstring);
                        break;
                    }
                }

                if (index == (sizeof(hci_command_table) / sizeof(hci_command_table[0])))
                {
                    armv7m_rtt_printf("?? UNEXPECTED HCI COMMAND = %04x\r\n", opcode);
                }
            }
#endif /* (STM32WB_IPCC_TRACE_SUPPORTED == 1) */
            
            MB_Data.MB_BleCmdBuffer.data[0] = HCI_COMMAND_DATA_PACKET;
            MB_Data.MB_BleCmdBuffer.data[1] = (uint8_t)(ble_command->opcode >> 0);
            MB_Data.MB_BleCmdBuffer.data[2] = (uint8_t)(ble_command->opcode >> 8);
            MB_Data.MB_BleCmdBuffer.data[3] = (uint8_t)ble_command->clen;
            
            if (ble_command->clen)
            {
                memcpy(&MB_Data.MB_BleCmdBuffer.data[4], ble_command->cparam, ble_command->clen);
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

            if (((void*)ble_event >= (void*)(&MB_Data.MB_BlePool[0])) && ((void*)ble_event < (void*)(&MB_Data.MB_BlePool[0] + sizeof(MB_Data.MB_BlePool))))
            {
                MB_QueueInsert(&stm32wb_ipcc_device.mm_queue, &ble_event->node);
            }
        }
    }
#endif /* (STM32WB_IPCC_BLE_SUPPORTED == 1) */
    
    if (!stm32wb_ipcc_device.mm_busy)
    {
        if (!MB_QueueIsEmpty(&stm32wb_ipcc_device.mm_queue))
        {
            MB_QueueCopy(&MB_Data.MB_EvtFreeBufferQueue, &stm32wb_ipcc_device.mm_queue);

            stm32wb_ipcc_device.mm_busy = true;
            
            IPCC->C1SCR = IPCC_C1SCR_CH4S;
            
            armv7m_atomic_and(&IPCC->C1MR, ~IPCC_C1MR_CH4FM);
        }
    }

    __DSB();
}

static void stm32wb_ipcc_tx_interrupt(void)
{
    uint32_t ipcc_c1mr, ipcc_c1toc2sr, status, length;
    uint16_t opcode;
    const uint8_t *data;
    stm32wb_ipcc_sys_command_t *sys_command;
    stm32wb_ipcc_sys_command_callback_t sys_callback;
    void *sys_context;
#if (STM32WB_IPCC_BLE_SUPPORTED == 1)
#if (STM32WB_IPCC_BLE_ACLDATA_SUPPORTED == 1)
    stm32wb_ipcc_ble_acldata_t *acl_data;
    stm32wb_ipcc_ble_acldata_callback_t acl_callback;
    void *acl_context;
#endif /* STM32WB_IPCC_BLE_ACLDATA_SUPPORTED == 1 */
#endif /*STM32WB_IPCC_BLE_SUPPORTED == 1 */

    ipcc_c1mr = IPCC->C1MR;
    ipcc_c1toc2sr = IPCC->C1TOC2SR;

    if (!(ipcc_c1mr & IPCC_C1MR_CH2FM) && !(ipcc_c1toc2sr & IPCC_C1TOC2SR_CH2F))
    {
        /* SYS TX ACK */

        armv7m_atomic_or(&IPCC->C1MR, IPCC_C1MR_CH2FM);

        sys_command = stm32wb_ipcc_device.sys_current;

        if (sys_command)
        {
            stm32wb_ipcc_device.sys_current = NULL;

            data = (const uint8_t*)&MB_Data.MB_SysCmdBuffer;

            opcode = (data[4] << 0) | (data[5] << 8);

            if (opcode == STM32WB_IPCC_SYS_OPCODE_FUS_GET_STATE)
            {
                status = STM32WB_IPCC_SYS_COMMAND_STATUS_SUCCESS;
            }
            else
            {
                status = data[6] ? STM32WB_IPCC_SYS_COMMAND_STATUS_FAILURE : STM32WB_IPCC_SYS_COMMAND_STATUS_SUCCESS;
            }
            
            if (sys_command->rparam)
            {
                length = data[2] - 3;
                
                if (length > sys_command->rsize)
                {
                    length = sys_command->rsize;
                }
                
                if (length)
                {
                    memcpy(sys_command->rparam, (const void*)&data[6], length);
                }

                sys_command->rlen = length;
            }
            
            sys_callback = sys_command->callback;
            sys_context = sys_command->context;

            sys_command->status = status;

            if (sys_callback)
            {
                (*sys_callback)(sys_context);
            }
        }
    }

    if (!(ipcc_c1mr & IPCC_C1MR_CH4FM) && !(ipcc_c1toc2sr & IPCC_C1TOC2SR_CH4F))
    {
        /* MM TX ACK */

        armv7m_atomic_or(&IPCC->C1MR, IPCC_C1MR_CH4FM);

        stm32wb_ipcc_device.mm_busy = false;
    }
    
#if (STM32WB_IPCC_BLE_SUPPORTED == 1)
#if (STM32WB_IPCC_BLE_ACLDATA_SUPPORTED == 1)
    if (!(ipcc_c1mr & IPCC_C1MR_CH6FM) && !(ipcc_c1toc2sr & IPCC_C1TOC2SR_CH6F))
    {
        /* ACL TX ACK */

        armv7m_atomic_or(&IPCC->C1MR, IPCC_C1MR_CH6FM);

        acl_data = stm32wb_ipcc_device.acl_current;

        if (acl_data)
        {
            stm32wb_ipcc_device.acl_current = NULL;
            
            acl_callback = acl_data->callback;
            acl_context = acl_data->context;
        
            acl_data->status = STM32WB_IPCC_BLE_ACLDATA_STATUS_SUCCESS;

            if (acl_callback)
            {
                (*acl_callback)(acl_context);
            }
        }
    }
#endif /* STM32WB_IPCC_BLE_ACLDATA_SUPPORTED == 1 */
#endif /* (STM32WB_IPCC_BLE_SUPPORTED == 1) */
    
    if (!stm32wb_ipcc_device.mm_busy)
    {
        if (!MB_QueueIsEmpty(&stm32wb_ipcc_device.mm_queue))
        {
            MB_QueueCopy(&MB_Data.MB_EvtFreeBufferQueue, &stm32wb_ipcc_device.mm_queue);

            stm32wb_ipcc_device.mm_busy = true;
            
            IPCC->C1SCR = IPCC_C1SCR_CH4S;
            
            armv7m_atomic_and(&IPCC->C1MR, ~IPCC_C1MR_CH4FM);
        }
    }

    __DSB();
}

void IPCC_C1_RX_IRQHandler(void)
{
    stm32wb_ipcc_rx_interrupt();
}


void IPCC_C1_TX_IRQHandler(void)
{
    stm32wb_ipcc_tx_interrupt();
}

#endif /* !defined(__STM32WB_BOOT_CODE__) */
