/*
 * Copyright (c) 2020 Thomas Roell.  All rights reserved.
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

#if !defined(_STM32WB_IPCC_H)
#define _STM32WB_IPCC_H

#ifdef __cplusplus
extern "C" {
#endif

#define STM32WB_IPCC_IRQ_PRIORITY       ARMV7M_IRQ_PRIORITY_IPCC

#define STM32WB_IPCC_SYS_STATE_NONE                             0
#define STM32WB_IPCC_SYS_STATE_FUS                              1
#define STM32WB_IPCC_SYS_STATE_WIRELESS                         2
  
#define STM32WB_IPCC_SYS_OPCODE_FUS_GET_STATE                   0xfc52
#define STM32WB_IPCC_SYS_OPCODE_FUS_FW_UPGRADE                  0xfc54
#define STM32WB_IPCC_SYS_OPCODE_FUS_FW_DELETE                   0xfc55
#define STM32WB_IPCC_SYS_OPCODE_BLE_INIT                        0xfc66
#define STM32WB_IPCC_SYS_OPCODE_FLASH_ERASE_ACTIVITY            0xfc69
#define STM32WB_IPCC_SYS_OPCODE_SET_FLASH_ACTIVITY_CONTROL      0xfc73

#define STM32WB_IPCC_SYS_FUS_STATUS_SUCCESS                     0
#define STM32WB_IPCC_SYS_FUS_STATUS_FAILURE                     1
  
#define STM32WB_IPCC_SYS_FUS_STATE_MASK                         0xf0
#define STM32WB_IPCC_SYS_FUS_STATE_IDLE                         0x00
#define STM32WB_IPCC_SYS_FUS_STATE_FW_UPGRD_ONGOING             0x10
#define STM32WB_IPCC_SYS_FUS_STATE_FUS_UPGRD_ONGOING            0x20
#define STM32WB_IPCC_SYS_FUS_STATE_SERVICE_ONGOING              0x30
#define STM32WB_IPCC_SYS_FUS_STATE_ERROR                        0xff

#define STM32WB_IPCC_SYS_FUS_ERROR_CODE_NO_ERROR                0x00
#define STM32WB_IPCC_SYS_FUS_ERROR_CODE_IMG_NOT_FOUND           0x01
#define STM32WB_IPCC_SYS_FUS_ERROR_CODE_IMG_CORRUPT             0x02
#define STM32WB_IPCC_SYS_FUS_ERROR_CODE_IMG_NOT_AUTHENTIC       0x03
#define STM32WB_IPCC_SYS_FUS_ERROR_CODE_IMG_NOT_ENOUGH_SPACE    0x04
#define STM32WB_IPCC_SYS_FUS_ERROR_CODE_IMG_USR_ABORT           0x05
#define STM32WB_IPCC_SYS_FUS_ERROR_CODE_IMG_ERASE               0x06
#define STM32WB_IPCC_SYS_FUS_ERROR_CODE_IMG_PROGRAM             0x07
#define STM32WB_IPCC_SYS_FUS_ERROR_CODE_AUTH_TAG_ST_NOT_FOUND   0x08
#define STM32WB_IPCC_SYS_FUS_ERROR_CODE_AUTH_TAG_CUST_NOT_FOUND 0x09
#define STM32WB_IPCC_SYS_FUS_ERROR_CODE_AUTH_KEY_LOCKED         0x0a
#define STM32WB_IPCC_SYS_FUS_ERROR_CODE_FW_ROLLBACK             0x11
#define STM32WB_IPCC_SYS_FUS_ERROR_CODE_NOT_RUNNING             0xfe
#define STM32WB_IPCC_SYS_FUS_ERROR_CODE_ERROR_UNKNOWN           0xff

#define STM32WB_IPCC_SYS_ERASE_ACTIVITY_OFF                     0  
#define STM32WB_IPCC_SYS_ERASE_ACTIVITY_ON                      1  

#define STM32WB_IPCC_SYS_SET_FLASH_ACTIVITY_CONTROL_PES         0  
#define STM32WB_IPCC_SYS_SET_FLASH_ACTIVITY_CONTROL_HSEM        1  

#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_NONE                   0x00
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_BLE_FULL               0x01
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_BLE_HCI                0x02
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_BLE_LIGHT              0x03
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_BLE_BEACON             0x04
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_THREAD_FTD             0x10
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_THREAD_MTD             0x11
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_ZIGBEE_FFD             0x30
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_ZIGBEE_RFD             0x31
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_MAC                    0x40
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_BLE_THREAD_FTD_STATIC  0x50
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_BLE_THREAD_FTD_DYAMIC  0x51
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_802154_LLD_TESTS       0x60
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_802154_PHY_VALID       0x61
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_BLE_PHY_VALID          0x62
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_BLE_LLD_TESTS          0x63
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_BLE_RLV                0x64
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_802154_RLV             0x65
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_BLE_ZIGBEE_FFD_STATIC  0x70
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_BLE_ZIGBEE_RFD_STATIC  0x71
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_BLE_ZIGBEE_FFD_DYNAMIC 0x78
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_BLE_ZIGBEE_RFD_DYNAMIC 0x79
#define STM32WB_IPCC_SYS_INFO_STACK_TYPE_RLV                    0x80

typedef struct _stm32wb_ipcc_sys_info_t {
    uint32_t FusVersion;
    uint32_t FusMemorySize;
    uint32_t WirelessStackVersion;
    uint32_t WirelessStackMemorySize;
    uint32_t WirelessStackType;
} stm32wb_ipcc_sys_info_t;

typedef struct _stm32wb_ipcc_sys_fus_state_t {
    uint8_t  state;
    uint8_t  error_code;
} stm32wb_ipcc_sys_fus_state_t;

typedef void (*stm32wb_ipcc_sys_command_callback_t)(void *context);

#define STM32WB_IPCC_SYS_COMMAND_STATUS_SUCCESS                 0
#define STM32WB_IPCC_SYS_COMMAND_STATUS_FAILURE                 1
#define STM32WB_IPCC_SYS_COMMAND_STATUS_BUSY                    255
  
typedef struct _stm32wb_ipcc_sys_command_t {
    struct _stm32wb_ipcc_sys_command_t  *next;
    union {
	struct {
	    uint16_t                        ocf : 10;
	    uint16_t                        ogf : 6;
	};
        uint16_t                            opcode;
    };
    uint16_t                            event; // MUST BE ZERO
    const void                          *cparam;
    void                                *rparam;
    uint8_t                             clen;
    uint8_t                             rsize;
    volatile uint8_t                    rlen;
    volatile uint8_t                    status;
    stm32wb_ipcc_sys_command_callback_t callback;
    void                                *context;
} stm32wb_ipcc_sys_command_t;
  
extern void __stm32wb_ipcc_initialize(void);
  
extern bool stm32wb_ipcc_sys_enable(void);
extern void stm32wb_ipcc_sys_disable(void);
extern uint32_t stm32wb_ipcc_sys_state(void);
extern bool stm32wb_ipcc_sys_info(stm32wb_ipcc_sys_info_t *p_info_return);
extern bool stm32wb_ipcc_sys_command(stm32wb_ipcc_sys_command_t *command);
extern bool stm32wb_ipcc_sys_firmware(uint32_t version, uint32_t type, uint32_t address, const uint8_t *image, uint32_t size, const uint8_t *fus, const uint8_t *fus_for_0_5_3, uint32_t *p_code_return);
  
typedef struct __attribute__((packed)) _stm32wb_ipcc_ble_init_params_t {
    uint8_t *pBleBufferAddress;   /**< NOT USED CURRENTLY */
    uint32_t BleBufferSize;       /**< Size of the Buffer allocated in pBleBufferAddress  */
    uint16_t NumAttrRecord;
    uint16_t NumAttrServ;
    uint16_t AttrValueArrSize;
    uint8_t  NumOfLinks;
    uint8_t  ExtendedPacketLengthEnable;
    uint8_t  PrWriteListSize;
    uint8_t  MblockCount;
    uint16_t AttMtu;
    uint16_t SlaveSca;
    uint8_t  MasterSca;
    uint8_t  LsSource;
    uint32_t MaxConnEventLength;
    uint16_t HsStartupTime;
    uint8_t  ViterbiEnable;
    uint8_t  Options;
    uint8_t  HwVersion;
    uint8_t  MaxCOCInitiatorNbr;
    int8_t   MinTxPower;
    int8_t   MaxTxPower;
    uint8_t  RxModel;
    uint8_t  MaxAdvSetNbr;
    uint16_t MaxAdvDataLen;
    int16_t  TxPathCompensation;
    int16_t  RxPathCompensation;
} stm32wb_ipcc_ble_init_params_t;
  
typedef void (*stm32wb_ipcc_ble_event_callback_t)(void *context);
typedef void (*stm32wb_ipcc_ble_command_callback_t)(void *context);
typedef void (*stm32wb_ipcc_ble_acldata_callback_t)(void *context);

#define STM32WB_IPCC_BLE_STATUS_SUCCESS                         0
#define STM32WB_IPCC_BLE_STATUS_FAILURE                         1
#define STM32WB_IPCC_BLE_STATUS_BUSY                            255

typedef struct _stm32wb_ipcc_ble_command_t {
    struct _stm32wb_ipcc_ble_command_t  *next;
    union {
	struct {
	    uint16_t                        ocf : 10;
	    uint16_t                        ogf : 6;
	};
        uint16_t                            opcode;
    };
    uint16_t                            event; // MUST BE ZERO
    const void                          *cparam;
    void                                *rparam;
    uint8_t                             clen;
    uint8_t                             rsize;
    volatile uint8_t                    rlen;
    volatile uint8_t                    status;
    stm32wb_ipcc_ble_command_callback_t callback;
    void                                *context;
} stm32wb_ipcc_ble_command_t;
  
extern bool stm32wb_ipcc_ble_enable(const stm32wb_ipcc_ble_init_params_t *params, stm32wb_ipcc_ble_event_callback_t callback, void *context);
extern bool stm32wb_ipcc_ble_disable(void);
extern bool stm32wb_ipcc_ble_command(stm32wb_ipcc_ble_command_t *command);
extern bool stm32wb_ipcc_ble_acldata(const uint8_t *data, uint32_t count, volatile uint8_t *p_status_return, stm32wb_ipcc_ble_acldata_callback_t callback, void *context);
extern uint8_t *stm32wb_ipcc_ble_event(void);
  
#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_IPCC_H */


