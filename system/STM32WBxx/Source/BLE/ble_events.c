/******************************************************************************
 * @file    ble_events.c
 * @author  MCD
 * @brief   STM32WB BLE API (event callbacks)
 *          Auto-generated file: do not edit!
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */

#include "BLE/ble_events.h"

/* Event process functions declaration */
static void hci_disconnection_complete_event_process( const uint8_t* in );
static void hci_encryption_change_event_process( const uint8_t* in );
static void hci_read_remote_version_information_complete_event_process( const uint8_t* in );
static void hci_hardware_error_event_process( const uint8_t* in );
static void hci_number_of_completed_packets_event_process( const uint8_t* in );
static void hci_encryption_key_refresh_complete_event_process( const uint8_t* in );
static void hci_le_connection_complete_event_process( const uint8_t* in );
static void hci_le_advertising_report_event_process( const uint8_t* in );
static void hci_le_connection_update_complete_event_process( const uint8_t* in );
static void hci_le_read_remote_features_complete_event_process( const uint8_t* in );
static void hci_le_long_term_key_request_event_process( const uint8_t* in );
static void hci_le_data_length_change_event_process( const uint8_t* in );
static void hci_le_read_local_p256_public_key_complete_event_process( const uint8_t* in );
static void hci_le_generate_dhkey_complete_event_process( const uint8_t* in );
static void hci_le_enhanced_connection_complete_event_process( const uint8_t* in );
static void hci_le_direct_advertising_report_event_process( const uint8_t* in );
static void hci_le_phy_update_complete_event_process( const uint8_t* in );
static void aci_hal_end_of_radio_activity_event_process( const uint8_t* in );
static void aci_hal_scan_req_report_event_process( const uint8_t* in );
static void aci_hal_fw_error_event_process( const uint8_t* in );
static void aci_gap_limited_discoverable_event_process( const uint8_t* in );
static void aci_gap_pairing_complete_event_process( const uint8_t* in );
static void aci_gap_pass_key_req_event_process( const uint8_t* in );
static void aci_gap_authorization_req_event_process( const uint8_t* in );
static void aci_gap_slave_security_initiated_event_process( const uint8_t* in );
static void aci_gap_bond_lost_event_process( const uint8_t* in );
static void aci_gap_proc_complete_event_process( const uint8_t* in );
static void aci_gap_addr_not_resolved_event_process( const uint8_t* in );
static void aci_gap_numeric_comparison_value_event_process( const uint8_t* in );
static void aci_gap_keypress_notification_event_process( const uint8_t* in );
static void aci_l2cap_connection_update_resp_event_process( const uint8_t* in );
static void aci_l2cap_proc_timeout_event_process( const uint8_t* in );
static void aci_l2cap_connection_update_req_event_process( const uint8_t* in );
static void aci_l2cap_command_reject_event_process( const uint8_t* in );
static void aci_gatt_attribute_modified_event_process( const uint8_t* in );
static void aci_gatt_proc_timeout_event_process( const uint8_t* in );
static void aci_att_exchange_mtu_resp_event_process( const uint8_t* in );
static void aci_att_find_info_resp_event_process( const uint8_t* in );
static void aci_att_find_by_type_value_resp_event_process( const uint8_t* in );
static void aci_att_read_by_type_resp_event_process( const uint8_t* in );
static void aci_att_read_resp_event_process( const uint8_t* in );
static void aci_att_read_blob_resp_event_process( const uint8_t* in );
static void aci_att_read_multiple_resp_event_process( const uint8_t* in );
static void aci_att_read_by_group_type_resp_event_process( const uint8_t* in );
static void aci_att_prepare_write_resp_event_process( const uint8_t* in );
static void aci_att_exec_write_resp_event_process( const uint8_t* in );
static void aci_gatt_indication_event_process( const uint8_t* in );
static void aci_gatt_notification_event_process( const uint8_t* in );
static void aci_gatt_proc_complete_event_process( const uint8_t* in );
static void aci_gatt_error_resp_event_process( const uint8_t* in );
static void aci_gatt_disc_read_char_by_uuid_resp_event_process( const uint8_t* in );
static void aci_gatt_write_permit_req_event_process( const uint8_t* in );
static void aci_gatt_read_permit_req_event_process( const uint8_t* in );
static void aci_gatt_read_multi_permit_req_event_process( const uint8_t* in );
static void aci_gatt_tx_pool_available_event_process( const uint8_t* in );
static void aci_gatt_server_confirmation_event_process( const uint8_t* in );
static void aci_gatt_prepare_write_permit_req_event_process( const uint8_t* in );
static void aci_gatt_read_ext_event_process( const uint8_t* in );
static void aci_gatt_indication_ext_event_process( const uint8_t* in );
static void aci_gatt_notification_ext_event_process( const uint8_t* in );

/* HCI event process functions table */
const hci_event_table_t hci_event_table[HCI_EVENT_TABLE_SIZE] =
{
  { 0x0005U, hci_disconnection_complete_event_process },
  { 0x0008U, hci_encryption_change_event_process },
  { 0x000CU, hci_read_remote_version_information_complete_event_process },
  { 0x0010U, hci_hardware_error_event_process },
  { 0x0013U, hci_number_of_completed_packets_event_process },
  { 0x0030U, hci_encryption_key_refresh_complete_event_process },
};

/* HCI LE event process functions table */
const hci_event_table_t hci_le_event_table[HCI_LE_EVENT_TABLE_SIZE] =
{
  { 0x0001U, hci_le_connection_complete_event_process },
  { 0x0002U, hci_le_advertising_report_event_process },
  { 0x0003U, hci_le_connection_update_complete_event_process },
  { 0x0004U, hci_le_read_remote_features_complete_event_process },
  { 0x0005U, hci_le_long_term_key_request_event_process },
  { 0x0007U, hci_le_data_length_change_event_process },
  { 0x0008U, hci_le_read_local_p256_public_key_complete_event_process },
  { 0x0009U, hci_le_generate_dhkey_complete_event_process },
  { 0x000AU, hci_le_enhanced_connection_complete_event_process },
  { 0x000BU, hci_le_direct_advertising_report_event_process },
  { 0x000CU, hci_le_phy_update_complete_event_process },
};

/* HCI VS event process functions table */
const hci_event_table_t hci_vs_event_table[HCI_VS_EVENT_TABLE_SIZE] =
{
  { 0x0004U, aci_hal_end_of_radio_activity_event_process },
  { 0x0005U, aci_hal_scan_req_report_event_process },
  { 0x0006U, aci_hal_fw_error_event_process },
  { 0x0400U, aci_gap_limited_discoverable_event_process },
  { 0x0401U, aci_gap_pairing_complete_event_process },
  { 0x0402U, aci_gap_pass_key_req_event_process },
  { 0x0403U, aci_gap_authorization_req_event_process },
  { 0x0404U, aci_gap_slave_security_initiated_event_process },
  { 0x0405U, aci_gap_bond_lost_event_process },
  { 0x0407U, aci_gap_proc_complete_event_process },
  { 0x0408U, aci_gap_addr_not_resolved_event_process },
  { 0x0409U, aci_gap_numeric_comparison_value_event_process },
  { 0x040AU, aci_gap_keypress_notification_event_process },
  { 0x0800U, aci_l2cap_connection_update_resp_event_process },
  { 0x0801U, aci_l2cap_proc_timeout_event_process },
  { 0x0802U, aci_l2cap_connection_update_req_event_process },
  { 0x080AU, aci_l2cap_command_reject_event_process },
  { 0x0C01U, aci_gatt_attribute_modified_event_process },
  { 0x0C02U, aci_gatt_proc_timeout_event_process },
  { 0x0C03U, aci_att_exchange_mtu_resp_event_process },
  { 0x0C04U, aci_att_find_info_resp_event_process },
  { 0x0C05U, aci_att_find_by_type_value_resp_event_process },
  { 0x0C06U, aci_att_read_by_type_resp_event_process },
  { 0x0C07U, aci_att_read_resp_event_process },
  { 0x0C08U, aci_att_read_blob_resp_event_process },
  { 0x0C09U, aci_att_read_multiple_resp_event_process },
  { 0x0C0AU, aci_att_read_by_group_type_resp_event_process },
  { 0x0C0CU, aci_att_prepare_write_resp_event_process },
  { 0x0C0DU, aci_att_exec_write_resp_event_process },
  { 0x0C0EU, aci_gatt_indication_event_process },
  { 0x0C0FU, aci_gatt_notification_event_process },
  { 0x0C10U, aci_gatt_proc_complete_event_process },
  { 0x0C11U, aci_gatt_error_resp_event_process },
  { 0x0C12U, aci_gatt_disc_read_char_by_uuid_resp_event_process },
  { 0x0C13U, aci_gatt_write_permit_req_event_process },
  { 0x0C14U, aci_gatt_read_permit_req_event_process },
  { 0x0C15U, aci_gatt_read_multi_permit_req_event_process },
  { 0x0C16U, aci_gatt_tx_pool_available_event_process },
  { 0x0C17U, aci_gatt_server_confirmation_event_process },
  { 0x0C18U, aci_gatt_prepare_write_permit_req_event_process },
  { 0x0C1DU, aci_gatt_read_ext_event_process },
  { 0x0C1EU, aci_gatt_indication_ext_event_process },
  { 0x0C1FU, aci_gatt_notification_ext_event_process },
};

/* HCI_DISCONNECTION_COMPLETE_EVENT callback function */
__WEAK void hci_disconnection_complete_event( uint8_t Status,
                                              uint16_t Connection_Handle,
                                              uint8_t Reason )
{
}

/* HCI_DISCONNECTION_COMPLETE_EVENT process function */
static void hci_disconnection_complete_event_process( const uint8_t* in )
{
  hci_disconnection_complete_event_rp0 *rp0 = (void*)in;
  hci_disconnection_complete_event( rp0->Status,
                                    rp0->Connection_Handle,
                                    rp0->Reason );
}

/* HCI_ENCRYPTION_CHANGE_EVENT callback function */
__WEAK void hci_encryption_change_event( uint8_t Status,
                                         uint16_t Connection_Handle,
                                         uint8_t Encryption_Enabled )
{
}

/* HCI_ENCRYPTION_CHANGE_EVENT process function */
static void hci_encryption_change_event_process( const uint8_t* in )
{
  hci_encryption_change_event_rp0 *rp0 = (void*)in;
  hci_encryption_change_event( rp0->Status,
                               rp0->Connection_Handle,
                               rp0->Encryption_Enabled );
}

/* HCI_READ_REMOTE_VERSION_INFORMATION_COMPLETE_EVENT callback function */
__WEAK void hci_read_remote_version_information_complete_event( uint8_t Status,
                                                                uint16_t Connection_Handle,
                                                                uint8_t Version,
                                                                uint16_t Manufacturer_Name,
                                                                uint16_t Subversion )
{
}

/* HCI_READ_REMOTE_VERSION_INFORMATION_COMPLETE_EVENT process function */
static void hci_read_remote_version_information_complete_event_process( const uint8_t* in )
{
  hci_read_remote_version_information_complete_event_rp0 *rp0 = (void*)in;
  hci_read_remote_version_information_complete_event( rp0->Status,
                                                      rp0->Connection_Handle,
                                                      rp0->Version,
                                                      rp0->Manufacturer_Name,
                                                      rp0->Subversion );
}

/* HCI_HARDWARE_ERROR_EVENT callback function */
__WEAK void hci_hardware_error_event( uint8_t Hardware_Code )
{
}

/* HCI_HARDWARE_ERROR_EVENT process function */
static void hci_hardware_error_event_process( const uint8_t* in )
{
  hci_hardware_error_event_rp0 *rp0 = (void*)in;
  hci_hardware_error_event( rp0->Hardware_Code );
}

/* HCI_NUMBER_OF_COMPLETED_PACKETS_EVENT callback function */
__WEAK void hci_number_of_completed_packets_event( uint8_t Number_of_Handles,
                                                   const Handle_Packets_Pair_Entry_t* Handle_Packets_Pair_Entry )
{
}

/* HCI_NUMBER_OF_COMPLETED_PACKETS_EVENT process function */
static void hci_number_of_completed_packets_event_process( const uint8_t* in )
{
  hci_number_of_completed_packets_event_rp0 *rp0 = (void*)in;
  hci_number_of_completed_packets_event( rp0->Number_of_Handles,
                                         rp0->Handle_Packets_Pair_Entry );
}

/* HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT callback function */
__WEAK void hci_encryption_key_refresh_complete_event( uint8_t Status,
                                                       uint16_t Connection_Handle )
{
}

/* HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT process function */
static void hci_encryption_key_refresh_complete_event_process( const uint8_t* in )
{
  hci_encryption_key_refresh_complete_event_rp0 *rp0 = (void*)in;
  hci_encryption_key_refresh_complete_event( rp0->Status,
                                             rp0->Connection_Handle );
}

/* HCI_LE_CONNECTION_COMPLETE_EVENT callback function */
__WEAK void hci_le_connection_complete_event( uint8_t Status,
                                              uint16_t Connection_Handle,
                                              uint8_t Role,
                                              uint8_t Peer_Address_Type,
                                              const uint8_t* Peer_Address,
                                              uint16_t Conn_Interval,
                                              uint16_t Conn_Latency,
                                              uint16_t Supervision_Timeout,
                                              uint8_t Master_Clock_Accuracy )
{
}

/* HCI_LE_CONNECTION_COMPLETE_EVENT process function */
static void hci_le_connection_complete_event_process( const uint8_t* in )
{
  hci_le_connection_complete_event_rp0 *rp0 = (void*)in;
  hci_le_connection_complete_event( rp0->Status,
                                    rp0->Connection_Handle,
                                    rp0->Role,
                                    rp0->Peer_Address_Type,
                                    rp0->Peer_Address,
                                    rp0->Conn_Interval,
                                    rp0->Conn_Latency,
                                    rp0->Supervision_Timeout,
                                    rp0->Master_Clock_Accuracy );
}

/* HCI_LE_ADVERTISING_REPORT_EVENT callback function */
__WEAK void hci_le_advertising_report_event( uint8_t Num_Reports,
                                             const Advertising_Report_t* Advertising_Report )
{
}

/* HCI_LE_ADVERTISING_REPORT_EVENT process function */
static void hci_le_advertising_report_event_process( const uint8_t* in )
{
  hci_le_advertising_report_event_rp0 *rp0 = (void*)in;
  Advertising_Report_t Advertising_Report[1];
  int i;
  for ( i = 0; i < rp0->Num_Reports; i++ ) 
  {
    in += 1;
    Osal_MemCpy( (void*)&Advertising_Report[0], (const void*)in, 9 );
    Advertising_Report[0].Data = &in[9];
    in += 9 + in[8];
    Advertising_Report[0].RSSI = in[0];
    hci_le_advertising_report_event( 1, Advertising_Report );
  }
}

/* HCI_LE_CONNECTION_UPDATE_COMPLETE_EVENT callback function */
__WEAK void hci_le_connection_update_complete_event( uint8_t Status,
                                                     uint16_t Connection_Handle,
                                                     uint16_t Conn_Interval,
                                                     uint16_t Conn_Latency,
                                                     uint16_t Supervision_Timeout )
{
}

/* HCI_LE_CONNECTION_UPDATE_COMPLETE_EVENT process function */
static void hci_le_connection_update_complete_event_process( const uint8_t* in )
{
  hci_le_connection_update_complete_event_rp0 *rp0 = (void*)in;
  hci_le_connection_update_complete_event( rp0->Status,
                                           rp0->Connection_Handle,
                                           rp0->Conn_Interval,
                                           rp0->Conn_Latency,
                                           rp0->Supervision_Timeout );
}

/* HCI_LE_READ_REMOTE_FEATURES_COMPLETE_EVENT callback function */
__WEAK void hci_le_read_remote_features_complete_event( uint8_t Status,
                                                        uint16_t Connection_Handle,
                                                        const uint8_t* LE_Features )
{
}

/* HCI_LE_READ_REMOTE_FEATURES_COMPLETE_EVENT process function */
static void hci_le_read_remote_features_complete_event_process( const uint8_t* in )
{
  hci_le_read_remote_features_complete_event_rp0 *rp0 = (void*)in;
  hci_le_read_remote_features_complete_event( rp0->Status,
                                              rp0->Connection_Handle,
                                              rp0->LE_Features );
}

/* HCI_LE_LONG_TERM_KEY_REQUEST_EVENT callback function */
__WEAK void hci_le_long_term_key_request_event( uint16_t Connection_Handle,
                                                const uint8_t* Random_Number,
                                                uint16_t Encrypted_Diversifier )
{
}

/* HCI_LE_LONG_TERM_KEY_REQUEST_EVENT process function */
static void hci_le_long_term_key_request_event_process( const uint8_t* in )
{
  hci_le_long_term_key_request_event_rp0 *rp0 = (void*)in;
  hci_le_long_term_key_request_event( rp0->Connection_Handle,
                                      rp0->Random_Number,
                                      rp0->Encrypted_Diversifier );
}

/* HCI_LE_DATA_LENGTH_CHANGE_EVENT callback function */
__WEAK void hci_le_data_length_change_event( uint16_t Connection_Handle,
                                             uint16_t MaxTxOctets,
                                             uint16_t MaxTxTime,
                                             uint16_t MaxRxOctets,
                                             uint16_t MaxRxTime )
{
}

/* HCI_LE_DATA_LENGTH_CHANGE_EVENT process function */
static void hci_le_data_length_change_event_process( const uint8_t* in )
{
  hci_le_data_length_change_event_rp0 *rp0 = (void*)in;
  hci_le_data_length_change_event( rp0->Connection_Handle,
                                   rp0->MaxTxOctets,
                                   rp0->MaxTxTime,
                                   rp0->MaxRxOctets,
                                   rp0->MaxRxTime );
}

/* HCI_LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE_EVENT callback function */
__WEAK void hci_le_read_local_p256_public_key_complete_event( uint8_t Status,
                                                              const uint8_t* Local_P256_Public_Key )
{
}

/* HCI_LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE_EVENT process function */
static void hci_le_read_local_p256_public_key_complete_event_process( const uint8_t* in )
{
  hci_le_read_local_p256_public_key_complete_event_rp0 *rp0 = (void*)in;
  hci_le_read_local_p256_public_key_complete_event( rp0->Status,
                                                    rp0->Local_P256_Public_Key );
}

/* HCI_LE_GENERATE_DHKEY_COMPLETE_EVENT callback function */
__WEAK void hci_le_generate_dhkey_complete_event( uint8_t Status,
                                                  const uint8_t* DHKey )
{
}

/* HCI_LE_GENERATE_DHKEY_COMPLETE_EVENT process function */
static void hci_le_generate_dhkey_complete_event_process( const uint8_t* in )
{
  hci_le_generate_dhkey_complete_event_rp0 *rp0 = (void*)in;
  hci_le_generate_dhkey_complete_event( rp0->Status,
                                        rp0->DHKey );
}

/* HCI_LE_ENHANCED_CONNECTION_COMPLETE_EVENT callback function */
__WEAK void hci_le_enhanced_connection_complete_event( uint8_t Status,
                                                       uint16_t Connection_Handle,
                                                       uint8_t Role,
                                                       uint8_t Peer_Address_Type,
                                                       const uint8_t* Peer_Address,
                                                       const uint8_t* Local_Resolvable_Private_Address,
                                                       const uint8_t* Peer_Resolvable_Private_Address,
                                                       uint16_t Conn_Interval,
                                                       uint16_t Conn_Latency,
                                                       uint16_t Supervision_Timeout,
                                                       uint8_t Master_Clock_Accuracy )
{
}

/* HCI_LE_ENHANCED_CONNECTION_COMPLETE_EVENT process function */
static void hci_le_enhanced_connection_complete_event_process( const uint8_t* in )
{
  hci_le_enhanced_connection_complete_event_rp0 *rp0 = (void*)in;
  hci_le_enhanced_connection_complete_event( rp0->Status,
                                             rp0->Connection_Handle,
                                             rp0->Role,
                                             rp0->Peer_Address_Type,
                                             rp0->Peer_Address,
                                             rp0->Local_Resolvable_Private_Address,
                                             rp0->Peer_Resolvable_Private_Address,
                                             rp0->Conn_Interval,
                                             rp0->Conn_Latency,
                                             rp0->Supervision_Timeout,
                                             rp0->Master_Clock_Accuracy );
}

/* HCI_LE_DIRECT_ADVERTISING_REPORT_EVENT callback function */
__WEAK void hci_le_direct_advertising_report_event( uint8_t Num_Reports,
                                                    const Direct_Advertising_Report_t* Direct_Advertising_Report )
{
}

/* HCI_LE_DIRECT_ADVERTISING_REPORT_EVENT process function */
static void hci_le_direct_advertising_report_event_process( const uint8_t* in )
{
  hci_le_direct_advertising_report_event_rp0 *rp0 = (void*)in;
  hci_le_direct_advertising_report_event( rp0->Num_Reports,
                                          rp0->Direct_Advertising_Report );
}

/* HCI_LE_PHY_UPDATE_COMPLETE_EVENT callback function */
__WEAK void hci_le_phy_update_complete_event( uint8_t Status,
                                              uint16_t Connection_Handle,
                                              uint8_t TX_PHY,
                                              uint8_t RX_PHY )
{
}

/* HCI_LE_PHY_UPDATE_COMPLETE_EVENT process function */
static void hci_le_phy_update_complete_event_process( const uint8_t* in )
{
  hci_le_phy_update_complete_event_rp0 *rp0 = (void*)in;
  hci_le_phy_update_complete_event( rp0->Status,
                                    rp0->Connection_Handle,
                                    rp0->TX_PHY,
                                    rp0->RX_PHY );
}

/* ACI_HAL_END_OF_RADIO_ACTIVITY_EVENT callback function */
__WEAK void aci_hal_end_of_radio_activity_event( uint8_t Last_State,
                                                 uint8_t Next_State,
                                                 uint32_t Next_State_SysTime )
{
}

/* ACI_HAL_END_OF_RADIO_ACTIVITY_EVENT process function */
static void aci_hal_end_of_radio_activity_event_process( const uint8_t* in )
{
  aci_hal_end_of_radio_activity_event_rp0 *rp0 = (void*)in;
  aci_hal_end_of_radio_activity_event( rp0->Last_State,
                                       rp0->Next_State,
                                       rp0->Next_State_SysTime );
}

/* ACI_HAL_SCAN_REQ_REPORT_EVENT callback function */
__WEAK void aci_hal_scan_req_report_event( uint8_t RSSI,
                                           uint8_t Peer_Address_Type,
                                           const uint8_t* Peer_Address )
{
}

/* ACI_HAL_SCAN_REQ_REPORT_EVENT process function */
static void aci_hal_scan_req_report_event_process( const uint8_t* in )
{
  aci_hal_scan_req_report_event_rp0 *rp0 = (void*)in;
  aci_hal_scan_req_report_event( rp0->RSSI,
                                 rp0->Peer_Address_Type,
                                 rp0->Peer_Address );
}

/* ACI_HAL_FW_ERROR_EVENT callback function */
__WEAK void aci_hal_fw_error_event( uint8_t FW_Error_Type,
                                    uint8_t Data_Length,
                                    const uint8_t* Data )
{
}

/* ACI_HAL_FW_ERROR_EVENT process function */
static void aci_hal_fw_error_event_process( const uint8_t* in )
{
  aci_hal_fw_error_event_rp0 *rp0 = (void*)in;
  aci_hal_fw_error_event( rp0->FW_Error_Type,
                          rp0->Data_Length,
                          rp0->Data );
}

/* ACI_GAP_LIMITED_DISCOVERABLE_EVENT callback function */
__WEAK void aci_gap_limited_discoverable_event( void )
{
}

/* ACI_GAP_LIMITED_DISCOVERABLE_EVENT process function */
static void aci_gap_limited_discoverable_event_process( const uint8_t* in )
{
  aci_gap_limited_discoverable_event( );
}

/* ACI_GAP_PAIRING_COMPLETE_EVENT callback function */
__WEAK void aci_gap_pairing_complete_event( uint16_t Connection_Handle,
                                            uint8_t Status,
                                            uint8_t Reason )
{
}

/* ACI_GAP_PAIRING_COMPLETE_EVENT process function */
static void aci_gap_pairing_complete_event_process( const uint8_t* in )
{
  aci_gap_pairing_complete_event_rp0 *rp0 = (void*)in;
  aci_gap_pairing_complete_event( rp0->Connection_Handle,
                                  rp0->Status,
                                  rp0->Reason );
}

/* ACI_GAP_PASS_KEY_REQ_EVENT callback function */
__WEAK void aci_gap_pass_key_req_event( uint16_t Connection_Handle )
{
}

/* ACI_GAP_PASS_KEY_REQ_EVENT process function */
static void aci_gap_pass_key_req_event_process( const uint8_t* in )
{
  aci_gap_pass_key_req_event_rp0 *rp0 = (void*)in;
  aci_gap_pass_key_req_event( rp0->Connection_Handle );
}

/* ACI_GAP_AUTHORIZATION_REQ_EVENT callback function */
__WEAK void aci_gap_authorization_req_event( uint16_t Connection_Handle )
{
}

/* ACI_GAP_AUTHORIZATION_REQ_EVENT process function */
static void aci_gap_authorization_req_event_process( const uint8_t* in )
{
  aci_gap_authorization_req_event_rp0 *rp0 = (void*)in;
  aci_gap_authorization_req_event( rp0->Connection_Handle );
}

/* ACI_GAP_SLAVE_SECURITY_INITIATED_EVENT callback function */
__WEAK void aci_gap_slave_security_initiated_event( void )
{
}

/* ACI_GAP_SLAVE_SECURITY_INITIATED_EVENT process function */
static void aci_gap_slave_security_initiated_event_process( const uint8_t* in )
{
  aci_gap_slave_security_initiated_event( );
}

/* ACI_GAP_BOND_LOST_EVENT callback function */
__WEAK void aci_gap_bond_lost_event( void )
{
}

/* ACI_GAP_BOND_LOST_EVENT process function */
static void aci_gap_bond_lost_event_process( const uint8_t* in )
{
  aci_gap_bond_lost_event( );
}

/* ACI_GAP_PROC_COMPLETE_EVENT callback function */
__WEAK void aci_gap_proc_complete_event( uint8_t Procedure_Code,
                                         uint8_t Status,
                                         uint8_t Data_Length,
                                         const uint8_t* Data )
{
}

/* ACI_GAP_PROC_COMPLETE_EVENT process function */
static void aci_gap_proc_complete_event_process( const uint8_t* in )
{
  aci_gap_proc_complete_event_rp0 *rp0 = (void*)in;
  aci_gap_proc_complete_event( rp0->Procedure_Code,
                               rp0->Status,
                               rp0->Data_Length,
                               rp0->Data );
}

/* ACI_GAP_ADDR_NOT_RESOLVED_EVENT callback function */
__WEAK void aci_gap_addr_not_resolved_event( uint16_t Connection_Handle )
{
}

/* ACI_GAP_ADDR_NOT_RESOLVED_EVENT process function */
static void aci_gap_addr_not_resolved_event_process( const uint8_t* in )
{
  aci_gap_addr_not_resolved_event_rp0 *rp0 = (void*)in;
  aci_gap_addr_not_resolved_event( rp0->Connection_Handle );
}

/* ACI_GAP_NUMERIC_COMPARISON_VALUE_EVENT callback function */
__WEAK void aci_gap_numeric_comparison_value_event( uint16_t Connection_Handle,
                                                    uint32_t Numeric_Value )
{
}

/* ACI_GAP_NUMERIC_COMPARISON_VALUE_EVENT process function */
static void aci_gap_numeric_comparison_value_event_process( const uint8_t* in )
{
  aci_gap_numeric_comparison_value_event_rp0 *rp0 = (void*)in;
  aci_gap_numeric_comparison_value_event( rp0->Connection_Handle,
                                          rp0->Numeric_Value );
}

/* ACI_GAP_KEYPRESS_NOTIFICATION_EVENT callback function */
__WEAK void aci_gap_keypress_notification_event( uint16_t Connection_Handle,
                                                 uint8_t Notification_Type )
{
}

/* ACI_GAP_KEYPRESS_NOTIFICATION_EVENT process function */
static void aci_gap_keypress_notification_event_process( const uint8_t* in )
{
  aci_gap_keypress_notification_event_rp0 *rp0 = (void*)in;
  aci_gap_keypress_notification_event( rp0->Connection_Handle,
                                       rp0->Notification_Type );
}

/* ACI_L2CAP_CONNECTION_UPDATE_RESP_EVENT callback function */
__WEAK void aci_l2cap_connection_update_resp_event( uint16_t Connection_Handle,
                                                    uint16_t Result )
{
}

/* ACI_L2CAP_CONNECTION_UPDATE_RESP_EVENT process function */
static void aci_l2cap_connection_update_resp_event_process( const uint8_t* in )
{
  aci_l2cap_connection_update_resp_event_rp0 *rp0 = (void*)in;
  aci_l2cap_connection_update_resp_event( rp0->Connection_Handle,
                                          rp0->Result );
}

/* ACI_L2CAP_PROC_TIMEOUT_EVENT callback function */
__WEAK void aci_l2cap_proc_timeout_event( uint16_t Connection_Handle,
                                          uint8_t Data_Length,
                                          const uint8_t* Data )
{
}

/* ACI_L2CAP_PROC_TIMEOUT_EVENT process function */
static void aci_l2cap_proc_timeout_event_process( const uint8_t* in )
{
  aci_l2cap_proc_timeout_event_rp0 *rp0 = (void*)in;
  aci_l2cap_proc_timeout_event( rp0->Connection_Handle,
                                rp0->Data_Length,
                                rp0->Data );
}

/* ACI_L2CAP_CONNECTION_UPDATE_REQ_EVENT callback function */
__WEAK void aci_l2cap_connection_update_req_event( uint16_t Connection_Handle,
                                                   uint8_t Identifier,
                                                   uint16_t L2CAP_Length,
                                                   uint16_t Interval_Min,
                                                   uint16_t Interval_Max,
                                                   uint16_t Slave_Latency,
                                                   uint16_t Timeout_Multiplier )
{
}

/* ACI_L2CAP_CONNECTION_UPDATE_REQ_EVENT process function */
static void aci_l2cap_connection_update_req_event_process( const uint8_t* in )
{
  aci_l2cap_connection_update_req_event_rp0 *rp0 = (void*)in;
  aci_l2cap_connection_update_req_event( rp0->Connection_Handle,
                                         rp0->Identifier,
                                         rp0->L2CAP_Length,
                                         rp0->Interval_Min,
                                         rp0->Interval_Max,
                                         rp0->Slave_Latency,
                                         rp0->Timeout_Multiplier );
}

/* ACI_L2CAP_COMMAND_REJECT_EVENT callback function */
__WEAK void aci_l2cap_command_reject_event( uint16_t Connection_Handle,
                                            uint8_t Identifier,
                                            uint16_t Reason,
                                            uint8_t Data_Length,
                                            const uint8_t* Data )
{
}

/* ACI_L2CAP_COMMAND_REJECT_EVENT process function */
static void aci_l2cap_command_reject_event_process( const uint8_t* in )
{
  aci_l2cap_command_reject_event_rp0 *rp0 = (void*)in;
  aci_l2cap_command_reject_event( rp0->Connection_Handle,
                                  rp0->Identifier,
                                  rp0->Reason,
                                  rp0->Data_Length,
                                  rp0->Data );
}

/* ACI_GATT_ATTRIBUTE_MODIFIED_EVENT callback function */
__WEAK void aci_gatt_attribute_modified_event( uint16_t Connection_Handle,
                                               uint16_t Attr_Handle,
                                               uint16_t Offset,
                                               uint16_t Attr_Data_Length,
                                               const uint8_t* Attr_Data )
{
}

/* ACI_GATT_ATTRIBUTE_MODIFIED_EVENT process function */
static void aci_gatt_attribute_modified_event_process( const uint8_t* in )
{
  aci_gatt_attribute_modified_event_rp0 *rp0 = (void*)in;
  aci_gatt_attribute_modified_event( rp0->Connection_Handle,
                                     rp0->Attr_Handle,
                                     rp0->Offset,
                                     rp0->Attr_Data_Length,
                                     rp0->Attr_Data );
}

/* ACI_GATT_PROC_TIMEOUT_EVENT callback function */
__WEAK void aci_gatt_proc_timeout_event( uint16_t Connection_Handle )
{
}

/* ACI_GATT_PROC_TIMEOUT_EVENT process function */
static void aci_gatt_proc_timeout_event_process( const uint8_t* in )
{
  aci_gatt_proc_timeout_event_rp0 *rp0 = (void*)in;
  aci_gatt_proc_timeout_event( rp0->Connection_Handle );
}

/* ACI_ATT_EXCHANGE_MTU_RESP_EVENT callback function */
__WEAK void aci_att_exchange_mtu_resp_event( uint16_t Connection_Handle,
                                             uint16_t Server_RX_MTU )
{
}

/* ACI_ATT_EXCHANGE_MTU_RESP_EVENT process function */
static void aci_att_exchange_mtu_resp_event_process( const uint8_t* in )
{
  aci_att_exchange_mtu_resp_event_rp0 *rp0 = (void*)in;
  aci_att_exchange_mtu_resp_event( rp0->Connection_Handle,
                                   rp0->Server_RX_MTU );
}

/* ACI_ATT_FIND_INFO_RESP_EVENT callback function */
__WEAK void aci_att_find_info_resp_event( uint16_t Connection_Handle,
                                          uint8_t Format,
                                          uint8_t Event_Data_Length,
                                          const uint8_t* Handle_UUID_Pair )
{
}

/* ACI_ATT_FIND_INFO_RESP_EVENT process function */
static void aci_att_find_info_resp_event_process( const uint8_t* in )
{
  aci_att_find_info_resp_event_rp0 *rp0 = (void*)in;
  aci_att_find_info_resp_event( rp0->Connection_Handle,
                                rp0->Format,
                                rp0->Event_Data_Length,
                                rp0->Handle_UUID_Pair );
}

/* ACI_ATT_FIND_BY_TYPE_VALUE_RESP_EVENT callback function */
__WEAK void aci_att_find_by_type_value_resp_event( uint16_t Connection_Handle,
                                                   uint8_t Num_of_Handle_Pair,
                                                   const Attribute_Group_Handle_Pair_t* Attribute_Group_Handle_Pair )
{
}

/* ACI_ATT_FIND_BY_TYPE_VALUE_RESP_EVENT process function */
static void aci_att_find_by_type_value_resp_event_process( const uint8_t* in )
{
  aci_att_find_by_type_value_resp_event_rp0 *rp0 = (void*)in;
  aci_att_find_by_type_value_resp_event( rp0->Connection_Handle,
                                         rp0->Num_of_Handle_Pair,
                                         rp0->Attribute_Group_Handle_Pair );
}

/* ACI_ATT_READ_BY_TYPE_RESP_EVENT callback function */
__WEAK void aci_att_read_by_type_resp_event( uint16_t Connection_Handle,
                                             uint8_t Handle_Value_Pair_Length,
                                             uint8_t Data_Length,
                                             const uint8_t* Handle_Value_Pair_Data )
{
}

/* ACI_ATT_READ_BY_TYPE_RESP_EVENT process function */
static void aci_att_read_by_type_resp_event_process( const uint8_t* in )
{
  aci_att_read_by_type_resp_event_rp0 *rp0 = (void*)in;
  aci_att_read_by_type_resp_event( rp0->Connection_Handle,
                                   rp0->Handle_Value_Pair_Length,
                                   rp0->Data_Length,
                                   rp0->Handle_Value_Pair_Data );
}

/* ACI_ATT_READ_RESP_EVENT callback function */
__WEAK void aci_att_read_resp_event( uint16_t Connection_Handle,
                                     uint8_t Event_Data_Length,
                                     const uint8_t* Attribute_Value )
{
}

/* ACI_ATT_READ_RESP_EVENT process function */
static void aci_att_read_resp_event_process( const uint8_t* in )
{
  aci_att_read_resp_event_rp0 *rp0 = (void*)in;
  aci_att_read_resp_event( rp0->Connection_Handle,
                           rp0->Event_Data_Length,
                           rp0->Attribute_Value );
}

/* ACI_ATT_READ_BLOB_RESP_EVENT callback function */
__WEAK void aci_att_read_blob_resp_event( uint16_t Connection_Handle,
                                          uint8_t Event_Data_Length,
                                          const uint8_t* Attribute_Value )
{
}

/* ACI_ATT_READ_BLOB_RESP_EVENT process function */
static void aci_att_read_blob_resp_event_process( const uint8_t* in )
{
  aci_att_read_blob_resp_event_rp0 *rp0 = (void*)in;
  aci_att_read_blob_resp_event( rp0->Connection_Handle,
                                rp0->Event_Data_Length,
                                rp0->Attribute_Value );
}

/* ACI_ATT_READ_MULTIPLE_RESP_EVENT callback function */
__WEAK void aci_att_read_multiple_resp_event( uint16_t Connection_Handle,
                                              uint8_t Event_Data_Length,
                                              const uint8_t* Set_Of_Values )
{
}

/* ACI_ATT_READ_MULTIPLE_RESP_EVENT process function */
static void aci_att_read_multiple_resp_event_process( const uint8_t* in )
{
  aci_att_read_multiple_resp_event_rp0 *rp0 = (void*)in;
  aci_att_read_multiple_resp_event( rp0->Connection_Handle,
                                    rp0->Event_Data_Length,
                                    rp0->Set_Of_Values );
}

/* ACI_ATT_READ_BY_GROUP_TYPE_RESP_EVENT callback function */
__WEAK void aci_att_read_by_group_type_resp_event( uint16_t Connection_Handle,
                                                   uint8_t Attribute_Data_Length,
                                                   uint8_t Data_Length,
                                                   const uint8_t* Attribute_Data_List )
{
}

/* ACI_ATT_READ_BY_GROUP_TYPE_RESP_EVENT process function */
static void aci_att_read_by_group_type_resp_event_process( const uint8_t* in )
{
  aci_att_read_by_group_type_resp_event_rp0 *rp0 = (void*)in;
  aci_att_read_by_group_type_resp_event( rp0->Connection_Handle,
                                         rp0->Attribute_Data_Length,
                                         rp0->Data_Length,
                                         rp0->Attribute_Data_List );
}

/* ACI_ATT_PREPARE_WRITE_RESP_EVENT callback function */
__WEAK void aci_att_prepare_write_resp_event( uint16_t Connection_Handle,
                                              uint16_t Attribute_Handle,
                                              uint16_t Offset,
                                              uint8_t Part_Attribute_Value_Length,
                                              const uint8_t* Part_Attribute_Value )
{
}

/* ACI_ATT_PREPARE_WRITE_RESP_EVENT process function */
static void aci_att_prepare_write_resp_event_process( const uint8_t* in )
{
  aci_att_prepare_write_resp_event_rp0 *rp0 = (void*)in;
  aci_att_prepare_write_resp_event( rp0->Connection_Handle,
                                    rp0->Attribute_Handle,
                                    rp0->Offset,
                                    rp0->Part_Attribute_Value_Length,
                                    rp0->Part_Attribute_Value );
}

/* ACI_ATT_EXEC_WRITE_RESP_EVENT callback function */
__WEAK void aci_att_exec_write_resp_event( uint16_t Connection_Handle )
{
}

/* ACI_ATT_EXEC_WRITE_RESP_EVENT process function */
static void aci_att_exec_write_resp_event_process( const uint8_t* in )
{
  aci_att_exec_write_resp_event_rp0 *rp0 = (void*)in;
  aci_att_exec_write_resp_event( rp0->Connection_Handle );
}

/* ACI_GATT_INDICATION_EVENT callback function */
__WEAK void aci_gatt_indication_event( uint16_t Connection_Handle,
                                       uint16_t Attribute_Handle,
                                       uint8_t Attribute_Value_Length,
                                       const uint8_t* Attribute_Value )
{
}

/* ACI_GATT_INDICATION_EVENT process function */
static void aci_gatt_indication_event_process( const uint8_t* in )
{
  aci_gatt_indication_event_rp0 *rp0 = (void*)in;
  aci_gatt_indication_event( rp0->Connection_Handle,
                             rp0->Attribute_Handle,
                             rp0->Attribute_Value_Length,
                             rp0->Attribute_Value );
}

/* ACI_GATT_NOTIFICATION_EVENT callback function */
__WEAK void aci_gatt_notification_event( uint16_t Connection_Handle,
                                         uint16_t Attribute_Handle,
                                         uint8_t Attribute_Value_Length,
                                         const uint8_t* Attribute_Value )
{
}

/* ACI_GATT_NOTIFICATION_EVENT process function */
static void aci_gatt_notification_event_process( const uint8_t* in )
{
  aci_gatt_notification_event_rp0 *rp0 = (void*)in;
  aci_gatt_notification_event( rp0->Connection_Handle,
                               rp0->Attribute_Handle,
                               rp0->Attribute_Value_Length,
                               rp0->Attribute_Value );
}

/* ACI_GATT_PROC_COMPLETE_EVENT callback function */
__WEAK void aci_gatt_proc_complete_event( uint16_t Connection_Handle,
                                          uint8_t Error_Code )
{
}

/* ACI_GATT_PROC_COMPLETE_EVENT process function */
static void aci_gatt_proc_complete_event_process( const uint8_t* in )
{
  aci_gatt_proc_complete_event_rp0 *rp0 = (void*)in;
  aci_gatt_proc_complete_event( rp0->Connection_Handle,
                                rp0->Error_Code );
}

/* ACI_GATT_ERROR_RESP_EVENT callback function */
__WEAK void aci_gatt_error_resp_event( uint16_t Connection_Handle,
                                       uint8_t Req_Opcode,
                                       uint16_t Attribute_Handle,
                                       uint8_t Error_Code )
{
}

/* ACI_GATT_ERROR_RESP_EVENT process function */
static void aci_gatt_error_resp_event_process( const uint8_t* in )
{
  aci_gatt_error_resp_event_rp0 *rp0 = (void*)in;
  aci_gatt_error_resp_event( rp0->Connection_Handle,
                             rp0->Req_Opcode,
                             rp0->Attribute_Handle,
                             rp0->Error_Code );
}

/* ACI_GATT_DISC_READ_CHAR_BY_UUID_RESP_EVENT callback function */
__WEAK void aci_gatt_disc_read_char_by_uuid_resp_event( uint16_t Connection_Handle,
                                                        uint16_t Attribute_Handle,
                                                        uint8_t Attribute_Value_Length,
                                                        const uint8_t* Attribute_Value )
{
}

/* ACI_GATT_DISC_READ_CHAR_BY_UUID_RESP_EVENT process function */
static void aci_gatt_disc_read_char_by_uuid_resp_event_process( const uint8_t* in )
{
  aci_gatt_disc_read_char_by_uuid_resp_event_rp0 *rp0 = (void*)in;
  aci_gatt_disc_read_char_by_uuid_resp_event( rp0->Connection_Handle,
                                              rp0->Attribute_Handle,
                                              rp0->Attribute_Value_Length,
                                              rp0->Attribute_Value );
}

/* ACI_GATT_WRITE_PERMIT_REQ_EVENT callback function */
__WEAK void aci_gatt_write_permit_req_event( uint16_t Connection_Handle,
                                             uint16_t Attribute_Handle,
                                             uint8_t Data_Length,
                                             const uint8_t* Data )
{
}

/* ACI_GATT_WRITE_PERMIT_REQ_EVENT process function */
static void aci_gatt_write_permit_req_event_process( const uint8_t* in )
{
  aci_gatt_write_permit_req_event_rp0 *rp0 = (void*)in;
  aci_gatt_write_permit_req_event( rp0->Connection_Handle,
                                   rp0->Attribute_Handle,
                                   rp0->Data_Length,
                                   rp0->Data );
}

/* ACI_GATT_READ_PERMIT_REQ_EVENT callback function */
__WEAK void aci_gatt_read_permit_req_event( uint16_t Connection_Handle,
                                            uint16_t Attribute_Handle,
                                            uint16_t Offset )
{
}

/* ACI_GATT_READ_PERMIT_REQ_EVENT process function */
static void aci_gatt_read_permit_req_event_process( const uint8_t* in )
{
  aci_gatt_read_permit_req_event_rp0 *rp0 = (void*)in;
  aci_gatt_read_permit_req_event( rp0->Connection_Handle,
                                  rp0->Attribute_Handle,
                                  rp0->Offset );
}

/* ACI_GATT_READ_MULTI_PERMIT_REQ_EVENT callback function */
__WEAK void aci_gatt_read_multi_permit_req_event( uint16_t Connection_Handle,
                                                  uint8_t Number_of_Handles,
                                                  const Handle_Item_t* Handle_Item )
{
}

/* ACI_GATT_READ_MULTI_PERMIT_REQ_EVENT process function */
static void aci_gatt_read_multi_permit_req_event_process( const uint8_t* in )
{
  aci_gatt_read_multi_permit_req_event_rp0 *rp0 = (void*)in;
  aci_gatt_read_multi_permit_req_event( rp0->Connection_Handle,
                                        rp0->Number_of_Handles,
                                        rp0->Handle_Item );
}

/* ACI_GATT_TX_POOL_AVAILABLE_EVENT callback function */
__WEAK void aci_gatt_tx_pool_available_event( uint16_t Connection_Handle,
                                              uint16_t Available_Buffers )
{
}

/* ACI_GATT_TX_POOL_AVAILABLE_EVENT process function */
static void aci_gatt_tx_pool_available_event_process( const uint8_t* in )
{
  aci_gatt_tx_pool_available_event_rp0 *rp0 = (void*)in;
  aci_gatt_tx_pool_available_event( rp0->Connection_Handle,
                                    rp0->Available_Buffers );
}

/* ACI_GATT_SERVER_CONFIRMATION_EVENT callback function */
__WEAK void aci_gatt_server_confirmation_event( uint16_t Connection_Handle )
{
}

/* ACI_GATT_SERVER_CONFIRMATION_EVENT process function */
static void aci_gatt_server_confirmation_event_process( const uint8_t* in )
{
  aci_gatt_server_confirmation_event_rp0 *rp0 = (void*)in;
  aci_gatt_server_confirmation_event( rp0->Connection_Handle );
}

/* ACI_GATT_PREPARE_WRITE_PERMIT_REQ_EVENT callback function */
__WEAK void aci_gatt_prepare_write_permit_req_event( uint16_t Connection_Handle,
                                                     uint16_t Attribute_Handle,
                                                     uint16_t Offset,
                                                     uint8_t Data_Length,
                                                     const uint8_t* Data )
{
}

/* ACI_GATT_PREPARE_WRITE_PERMIT_REQ_EVENT process function */
static void aci_gatt_prepare_write_permit_req_event_process( const uint8_t* in )
{
  aci_gatt_prepare_write_permit_req_event_rp0 *rp0 = (void*)in;
  aci_gatt_prepare_write_permit_req_event( rp0->Connection_Handle,
                                           rp0->Attribute_Handle,
                                           rp0->Offset,
                                           rp0->Data_Length,
                                           rp0->Data );
}

/* ACI_GATT_READ_EXT_EVENT callback function */
__WEAK void aci_gatt_read_ext_event( uint16_t Connection_Handle,
                                     uint16_t Offset,
                                     uint16_t Event_Data_Length,
                                     const uint8_t* Attribute_Value )
{
}

/* ACI_GATT_READ_EXT_EVENT process function */
static void aci_gatt_read_ext_event_process( const uint8_t* in )
{
  aci_gatt_read_ext_event_rp0 *rp0 = (void*)in;
  aci_gatt_read_ext_event( rp0->Connection_Handle,
                           rp0->Offset,
                           rp0->Event_Data_Length,
                           rp0->Attribute_Value );
}

/* ACI_GATT_INDICATION_EXT_EVENT callback function */
__WEAK void aci_gatt_indication_ext_event( uint16_t Connection_Handle,
                                           uint16_t Attribute_Handle,
                                           uint16_t Offset,
                                           uint16_t Attribute_Value_Length,
                                           const uint8_t* Attribute_Value )
{
}

/* ACI_GATT_INDICATION_EXT_EVENT process function */
static void aci_gatt_indication_ext_event_process( const uint8_t* in )
{
  aci_gatt_indication_ext_event_rp0 *rp0 = (void*)in;
  aci_gatt_indication_ext_event( rp0->Connection_Handle,
                                 rp0->Attribute_Handle,
                                 rp0->Offset,
                                 rp0->Attribute_Value_Length,
                                 rp0->Attribute_Value );
}

/* ACI_GATT_NOTIFICATION_EXT_EVENT callback function */
__WEAK void aci_gatt_notification_ext_event( uint16_t Connection_Handle,
                                             uint16_t Attribute_Handle,
                                             uint16_t Offset,
                                             uint16_t Attribute_Value_Length,
                                             const uint8_t* Attribute_Value )
{
}

/* ACI_GATT_NOTIFICATION_EXT_EVENT process function */
static void aci_gatt_notification_ext_event_process( const uint8_t* in )
{
  aci_gatt_notification_ext_event_rp0 *rp0 = (void*)in;
  aci_gatt_notification_ext_event( rp0->Connection_Handle,
                                   rp0->Attribute_Handle,
                                   rp0->Offset,
                                   rp0->Attribute_Value_Length,
                                   rp0->Attribute_Value );
}

