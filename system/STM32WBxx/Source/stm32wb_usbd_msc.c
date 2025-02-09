/*
 * Copyright (c) 2019-2021 Thomas Roell.  All rights reserved.
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
#include "stm32wb_usbd.h"
#include "stm32wb_usbd_dcd.h"
#include "stm32wb_usbd_msc.h"
#include "dosfs_core.h"
#include "dosfs_storage.h"

/***********************************************************************************************************************/

#define SCSI_OPCODE_TEST_UNIT_READY                               0x00
#define SCSI_OPCODE_REQUEST_SENSE                                 0x03
#define SCSI_OPCODE_INQUIRY                                       0x12
#define SCSI_OPCODE_MODE_SENSE_6                                  0x1a
#define SCSI_OPCODE_START_STOP_UNIT                               0x1b
#define SCSI_OPCODE_PREVENT_ALLOW_MEDIUM_REMOVAL                  0x1e
#define SCSI_OPCODE_READ_CAPACITY_10                              0x25
#define SCSI_OPCODE_READ_10                                       0x28
#define SCSI_OPCODE_WRITE_10                                      0x2a
#define SCSI_OPCODE_SYNCHRONIZE_CACHE_10                          0x35

#define SCSI_STATE_READY                                          0
#define SCSI_STATE_STARTED                                        1
#define SCSI_STATE_STOPPED                                        2
#define SCSI_STATE_EJECTED                                        3

#define SCSI_STATUS_CSW_CMD_PASSED                                0
#define SCSI_STATUS_CSW_CMD_FAILED                                1
#define SCSI_STATUS_DATA                                          3
#define SCSI_STATUS_DATA_IN_CSW_CMD_PASSED                        MSC_BOT_ABORT_DATA_IN_CSW_CMD_PASSED
#define SCSI_STATUS_DATA_IN_CSW_CMD_FAILED                        MSC_BOT_ABORT_DATA_IN_CSW_CMD_FAILED
#define SCSI_STATUS_DATA_IN_CSW_PHASE_ERROR                       MSC_BOT_ABORT_DATA_IN_CSW_PHASE_ERROR
#define SCSI_STATUS_DATA_OUT_CSW_CMD_PASSED                       MSC_BOT_ABORT_DATA_OUT_CSW_CMD_PASSED
#define SCSI_STATUS_DATA_OUT_CSW_CMD_FAILED                       MSC_BOT_ABORT_DATA_OUT_CSW_CMD_FAILED
#define SCSI_STATUS_DATA_OUT_CSW_PHASE_ERROR                      MSC_BOT_ABORT_DATA_OUT_CSW_PHASE_ERROR

#define SCSI_DATA_LENGTH_INQUIRE_STANDARD                         36
#define SCSI_DATA_LENGTH_REQUEST_SENSE                            18
#define SCSI_DATA_LENGTH_MODE_SENSE_6                             4
#define SCSI_DATA_LENGTH_READ_CAPACITY_10                         8

#define SCSI_SKEY_NO_SENSE                                        0x00
#define SCSI_SKEY_RECOVERED_ERROR                                 0x01
#define SCSI_SKEY_NOT_READY                                       0x02
#define SCSI_SKEY_MEDIUM_ERROR                                    0x03
#define SCSI_SKEY_HARDWARE_ERROR                                  0x04
#define SCSI_SKEY_ILLEGAL_REQUEST                                 0x05
#define SCSI_SKEY_UNIT_ATTENTION                                  0x06
#define SCSI_SKEY_DATA_PROTECT                                    0x07
#define SCSI_SKEY_BLANK_CHECK                                     0x08
#define SCSI_SKEY_VENDOR_SPECIFIC                                 0x09
#define SCSI_SKEY_ABORTED_COMMAND                                 0x0b
#define SCSI_SKEY_VOLUME_OVERFLOW                                 0x0d
#define SCSI_SKEY_MISCOMPARE                                      0x0e

#define SCSI_ASC_NO_SENSE                                         0x0000
#define SCSI_ASC_NO_SEEK_COMPLETE                                 0x0200
#define SCSI_ASC_WRITE_FAULT                                      0x0300
#define SCSI_ASC_LOGICAL_DRIVE_NOT_READY_CAUSE_NOT_REPORTABLE     0x0400
#define SCSI_ASC_LOGICAL_DRIVE_NOT_READY_BECOMING_READY           0x0401
#define SCSI_ASC_LOGICAL_UNIT_NOT_READY_INITIALIZING_COMMAND_REQUIRED 0x0402
#define SCSI_ASC_LOGICAL_UNIT_NOT_READY_MANUAL_INTERVENTION_REQUIRED  0x0403
#define SCSI_ASC_LOGICAL_UNIT_NOT_READY_FORMAT_IN_PROGRESS            0x0404
#define SCSI_ASC_LOGICAL_DRIVE_NOT_READY_DEVICE_IS_BUSY               0x04FF
#define SCSI_ASC_NO_REFERENCE_POSITION_FOUND                          0x0600
#define SCSI_ASC_LOGICAL_UNIT_COMMUNICATION_FAILURE                   0x0800
#define SCSI_ASC_LOGICAL_UNIT_COMMUNICATION_TIMEOUT                   0x0801
#define SCSI_ASC_LOGICAL_UNIT_COMMUNICATION_OVERRUN                   0x0880
#define SCSI_ASC_ID_CRC_ERROR                                         0x1000
#define SCSI_ASC_UNRECOVERED_READ_ERROR                               0x1100
#define SCSI_ASC_ADDRESS_MARK_NOT_FOUND_FOR_ID_FIELD                  0x1200
#define SCSI_ASC_ADDRESS_MARK_NOT_FOUND_FOR_DATA_FIELD                0x1300
#define SCSI_ASC_RECORDED_ENTITY_NOT_FOUND                            0x1400
#define SCSI_ASC_RECOVERED_DATA_WITH_RETRIES                          0x1701
#define SCSI_ASC_RECOVERED_DATA_WITH_ECC                              0x1800
#define SCSI_ASC_PARAMETER_LIST_LENGTH_ERROR                          0x1A00
#define SCSI_ASC_INVALID_COMMAND_OPERATION_CODE                       0x2000
#define SCSI_ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE                   0x2100
#define SCSI_ASC_INVALID_FIELD_IN_COMMAND_PACKET                      0x2400
#define SCSI_ASC_LOGICAL_UNIT_NOT_SUPPORTED                           0x2500
#define SCSI_ASC_INVALID_FIELD_IN_PARAMETER_LIST                      0x2600
#define SCSI_ASC_PARAMETER_NOT_SUPPORTED                              0x2601
#define SCSI_ASC_PARAMETER_VALUE_INVALID                              0x2602
#define SCSI_ASC_WRITE_PROTECTED_MEDIA                                0x2700
#define SCSI_ASC_NOT_READY_TO_READY_TRANSITION_MEDIA_CHANGED          0x2800
#define SCSI_ASC_POWER_ON_RESET_OR_BUS_DEVICE_RESET_OCCURRED          0x2900
#define SCSI_ASC_COMMANDS_CLEARED_BY_ANOTHER_INITIATOR                0x2F00
#define SCSI_ASC_CANNOT_READ_MEDIUM_UNKNOWN_FORMAT                    0x3001
#define SCSI_ASC_FORMAT_COMMAND_FAILED                                0x3101
#define SCSI_ASC_SAVING_PARAMETERS_NOT_SUPPORTED                      0x3900
#define SCSI_ASC_MEDIUM_NOT_PRESENT                                   0x3A00
#define SCSI_ASC_INTERNAL_TARGET_FAILURE                              0x4400
#define SCSI_ASC_OVERLAPPED_COMMAND_ATTEMPTED                         0x4E00
#define SCSI_ASC_MEDIA_REMOVAL_PREVENTED                              0x5302
#define SCSI_ASC_USB_TO_HOST_SYSTEM_INTERFACE_FAILURE                 0x5400
#define SCSI_ASC_INSUFFICIENT_RESOURCES                               0x8000
#define SCSI_ASC_UNKNOWN_ERROR                                        0xFFFF

static void SCSI_ProcessStart(void);
static void SCSI_ProcessStop(void);
static void SCSI_ProcessReset(void);
static void SCSI_ProcessAttach(void);
static void SCSI_ProcessDetach(void);
static void SCSI_ProcessCommand(void);
static void SCSI_ProcessRead(void);
static void SCSI_ProcessWrite(void);

/***********************************************************************************************************************/

#define MSC_BOT_STATE_NONE                     0       /* None */
#define MSC_BOT_STATE_IDLE                     1       /* Idle */
#define MSC_BOT_STATE_RECOVERY_RESET           2       /* Waiting for reset */
#define MSC_BOT_STATE_RECOVERY_HALT            3       /* Waiting for clear feature halt on data in or out*/
#define MSC_BOT_STATE_HALT_DATA_IN             4       /* Waiting for clear feature halt on data in */
#define MSC_BOT_STATE_HALT_DATA_OUT            5       /* Waiting for clear feature halt on data out */
#define MSC_BOT_STATE_DATA_IN                  6       /* Data In state */
#define MSC_BOT_STATE_DATA_IN_LAST             7       /* Last Data In Last (passed) */
#define MSC_BOT_STATE_DATA_IN_LAST_STALL       8       /* Last Data In Last (failed or phase error) */
#define MSC_BOT_STATE_DATA_OUT                 9       /* Data Out state */

#define MSC_BOT_ABORT_DATA_IN_CSW_CMD_PASSED   4
#define MSC_BOT_ABORT_DATA_IN_CSW_CMD_FAILED   5
#define MSC_BOT_ABORT_DATA_IN_CSW_PHASE_ERROR  6
#define MSC_BOT_ABORT_DATA_OUT_CSW_CMD_PASSED  7
#define MSC_BOT_ABORT_DATA_OUT_CSW_CMD_FAILED  8
#define MSC_BOT_ABORT_DATA_OUT_CSW_PHASE_ERROR 9
   
#define MSC_BOT_CBW_SIGNATURE                  0x43425355
#define MSC_BOT_CSW_SIGNATURE                  0x53425355
#define MSC_BOT_CBW_LENGTH                     31
#define MSC_BOT_CSW_LENGTH                     13

#define MSC_BOT_CSW_CMD_PASSED                 0
#define MSC_BOT_CSW_CMD_FAILED                 1
#define MSC_BOT_CSW_PHASE_ERROR                2

#define MSC_BOT_GET_MAX_LUN                    0xfe
#define MSC_BOT_RESET                          0xff

#define MSC_BOT_EVENT_CONFIGURE                0x00000001
#define MSC_BOT_EVENT_START                    0x00000002
#define MSC_BOT_EVENT_STOP                     0x00000004
#define MSC_BOT_EVENT_RESET                    0x00000008
#define MSC_BOT_EVENT_ATTACH                   0x00000010
#define MSC_BOT_EVENT_DETACH                   0x00000020
#define MSC_BOT_EVENT_CLEAR_HALT_DATA_IN       0x00000040
#define MSC_BOT_EVENT_CLEAR_HALT_DATA_OUT      0x00000080
#define MSC_BOT_EVENT_TRANSMIT_0               0x00000100
#define MSC_BOT_EVENT_TRANSMIT_1               0x00000200
#define MSC_BOT_EVENT_RECEIVE_0                0x00000400
#define MSC_BOT_EVENT_RECEIVE_1                0x00000800
#define MSC_BOT_EVENT_RESPONSE                 0x00001000
#define MSC_BOT_EVENT_CSW                      0x00002000
#define MSC_BOT_EVENT_CBW                      0x00004000

#define MSC_BOT_NOTIFY_START                   0x00000001
#define MSC_BOT_NOTIFY_STOP                    0x00000002
#define MSC_BOT_NOTIFY_EJECT                   0x00000004
#define MSC_BOT_NOTIFY_ATTACH                  0x00000008
#define MSC_BOT_NOTIFY_DETACH                  0x00000010

static void MSC_BOT_Event(uint32_t events);
static void MSC_BOT_Routine(void *context) __attribute__((noreturn));

static void MSC_BOT_Start(void);
static void MSC_BOT_Stop(void);
static void MSC_BOT_Reset(void);
static void MSC_BOT_Abort(uint8_t abort);
static void MSC_BOT_Stall(uint8_t ep_addr);
static void MSC_BOT_ClearHaltDataIn(void);
static void MSC_BOT_ClearHaltDataOut(void);
static void MSC_BOT_SendCSW(uint8_t status);
static void MSC_BOT_Response(const uint8_t *data, uint32_t length);

static void MSC_BOT_SetupTransmit(uint32_t length, uint8_t status);
static void MSC_BOT_ContinueTransmit(uint32_t length, uint8_t status);

static void MSC_BOT_SetupReceive(uint32_t length);
static void MSC_BOT_ContinueReceive(uint32_t length);
static void MSC_BOT_FinishReceive(uint8_t status);

static void MSC_BOT_Notify(uint32_t notify);

/***********************************************************************************************************************/
/***********************************************************************************************************************/
   
typedef struct
{
    uint32_t dSignature;
    uint32_t dTag;
    uint32_t dDataLength;
    uint8_t  bmFlags;
    uint8_t  bLUN;
    uint8_t  bCBLength;
    uint8_t  CB[16];
} USBD_MSC_BOT_CBWTypeDef;

typedef struct
{
    uint32_t dSignature;
    uint32_t dTag;
    uint32_t dDataResidue;
    uint8_t  bStatus;
} USBD_MSC_BOT_CSWTypeDef;

typedef struct _stm32wb_usbd_msc_control_t {
    dosfs_device_t                    *device;
    stm32wb_usbd_msc_event_callback_t callback;
    void                              *context;

    uint8_t                  interface;  
    uint8_t                  max_lun;  

    uint8_t                  bot_state;
    USBD_MSC_BOT_CBWTypeDef  bot_cbw;
    USBD_MSC_BOT_CSWTypeDef  bot_csw;

    volatile uint8_t         bot_index;
    volatile uint8_t         bot_count;
    volatile uint8_t         bot_tx_busy;
    volatile uint8_t         bot_rx_busy;
    volatile uint32_t        bot_tx_length;
    volatile uint32_t        bot_rx_length;
  
    uint8_t                  scsi_state;
    uint8_t                  scsi_sense_skey;
    uint16_t                 scsi_sense_asc;
    uint8_t                  scsi_sense_data[SCSI_DATA_LENGTH_REQUEST_SENSE];
    uint8_t                  scsi_medium_present;
    uint8_t                  scsi_medium_removal;
    uint8_t                  scsi_write_protected;
    uint16_t                 scsi_blk_size;
    uint32_t                 scsi_blk_count;
    uint32_t                 scsi_blk_address;
    uint32_t                 scsi_blk_length;
    uint8_t                  scsi_blk_fua;
    uint8_t                  scsi_blk_busy;
    uint8_t                  scsi_blk_fault;
    uint8_t                  scsi_blk_index;
    uint8_t                  *scsi_data[2];
} stm32wb_usbd_msc_control_t;

static stm32wb_usbd_msc_control_t stm32wb_usbd_msc_control;

static k_task_t stm32wb_usbd_msc_task;

static __attribute__((aligned(8))) uint8_t stm32wb_usbd_msc_stack[1024];

/***********************************************************************************************************************/

static void stm32wb_usbd_msc_transmit_data(void *context, uint8_t ep_addr);
static void stm32wb_usbd_msc_transmit_response(void *context, uint8_t ep_addr);
static void stm32wb_usbd_msc_transmit_csw(void *context, uint8_t ep_addr);

static void stm32wb_usbd_msc_receive_data(void *context, uint8_t ep_addr, uint32_t count);
static void stm32wb_usbd_msc_receive_cbw(void *context, uint8_t ep_addr, uint32_t count);

/***********************************************************************************************************************/

#define STANDARD_INQUIRY_DATA_LEN 0x24

static const uint8_t scsi_inquiry_data[] = {
  /* LUN 0 */
  0x00,         
  0x80,         
  0x06,         
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00, 
  0x00,
  'S', 'T', 'M', '3', '2', 'W', 'B', ' ', /* Manufacturer : 8 bytes */
  'U', 'S', 'B', ' ', 'S', 'T', 'O', 'R', /* Product      : 16 Bytes */
  'A', 'G', 'E', ' ', '2', '.', '0', ' ',
  ' ', ' ', ' ' ,' ',                     /* Version      : 4 Bytes */
}; 

/*
 * This code is more complex than expected. USB/MSC uses really only one logical connection
 * over 2 pipe. First the host sends a CBW (Command Block Wrapper), then optionally data is
 * exchanged, then finally the device sends a CSW (Command Status Wrapper) back. If there
 * a mismatch between what the host or devices expects one has to properly resync. If the
 * the CBW is invalid, the device stalls both IN and OUT pipe. If the host expects to
 * receive data, the IN pipe gets stalled, and if the host expects to send data the OUT
 * pipe gets stalled. If both pipes are stalled, the host issues a reset sequence
 * (reset token, then clear halt on IN, then clear halt on OUT). Otherwise the halt on
 * IN or OUT are cleared as needed. When that is done, the devices sends the CSV.
 *
 * The spec lays this out in terms of Hi (host receives), Ho (host wants to send) and
 * Hn (host neither wants to receiver or send). Same for the device, Di, Do, Dn.
 *
 * Hi, Ho & Hn are coded in the CBW:
 *
 * if (dCBWDataTransferLength == 0) {
 *     Hn;
 * } else {
 *     if (bmCBWFlags & 0x80) {
 *         Hi(dCBWDataTransferLength);
 *     } else {
 *         Ho(dCBWDataTransferLength);
 *     }
 * }
 *
 * For the device side it dependends up the command:
 *
 * TEST_UNIT_READY                   Dn
 * START_STOP_UNIT                   Dn
 * PREVENT_ALLOW_MEDIUM_REMOVAL      Dn
 * REQUEST_SENSE                     Do (Dn if "Allocation Length" is 0)
 * INQUIRY                           Do (Dn if "Allocation Length" is 0)
 * MODE_SENSE_6                      Do
 * READ_CAPACITY_10                  Do
 * READ_10                           Do (Dn if "Transfer Length" is 0)
 * WRITE_10                          Di (Dn if "Transfer Length" is 0)
 * <illegal command>                 Dn
 *
 * In general up to dCBWDataTransferLength get transferred; if there is less data, less gets
 * transfered, and dCSWDataResidue contains the delta.
 *
 * The spec identifies 13 cases:
 *
 *                  CSW               STALL     HOST            BOT STATE
 *  (1) Hn = Dn     PASSED / FAILED   -         -               IDLE
 *  (2) Hn < Di     PHASE_ERROR       DATA_IN   RESET_RECOVERY  RECOVERY_RESET
 *  (3) Hn < Do     PHASE_ERROR       DATA_IN   RESET_RECOVERY  RECOVERY_RESET
 *  (4) Hi > Dn     PASSED / FAILED   DATA_IN   -               HALT_DATA_IN
 *  (5) Hi > Di     PASSED / FAILED   DATA_IN   -               HALT_DATA_IN
 *  (6) Hi = Di     PASSED / FAILED   -         -               IDLE
 *  (7) Hi < Di     PHASE_ERROR       DATA_IN   RESET_RECOVERY  RECOVERY_RESET
 *  (8) Hi <> Do    PHASE_ERROR       DATA_IN   RESET_RECOVERY  RECOVERY_RESET
 *  (9) Ho > Dn     PASSED / FAILED   DATA_OUT  -               HALT_DATA_OUT
 * (10) Ho <> Di    PHASE_ERROR       DATA_OUT  RESET_RECOVERY  RECOVERY_RESET
 * (11) Ho > Do     PASSED / FAILED   DATA_OUT  -               HALT_DATA_OUT
 * (12) Ho = Do     PASSED / FAILED   -         -               IDLE
 * (13) Ho < Do     PHASE_ERROR       DATA_OUT  RESET_RECOVERY  RECOVERY_RESET
 *
 * The spec does not say what CSW to send on a invalid CBW. Bot stalled are DATA_IN/DATA_OUT
 * followed by a RESET_RECOVERY.
 */

static void SCSI_SenseCode(uint8_t SKey, uint16_t ASC)
{
    if (stm32wb_usbd_msc_control.scsi_sense_skey == SCSI_SKEY_NO_SENSE)
    {
        stm32wb_usbd_msc_control.scsi_sense_skey = SKey;
        stm32wb_usbd_msc_control.scsi_sense_asc = ASC;
    }
}

static void SCSI_SenseStatus(int status)
{
    if (status == F_ERR_READ)
    {
        SCSI_SenseCode(SCSI_SKEY_HARDWARE_ERROR, SCSI_ASC_UNRECOVERED_READ_ERROR);
    }

    if (status == F_ERR_WRITE)
    {
        SCSI_SenseCode(SCSI_SKEY_HARDWARE_ERROR, SCSI_ASC_WRITE_FAULT);
    }
        
    if (status == F_ERR_CARDREMOVED)
    {
        SCSI_SenseCode(SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT);
    }
    
    if (status == F_ERR_ONDRIVE)
    {
        SCSI_SenseCode(SCSI_SKEY_HARDWARE_ERROR, SCSI_ASC_INTERNAL_TARGET_FAILURE);
    }

#if 0
    if (status == F_ERR_IO)
    {
        SCSI_SenseCode(SCSI_SKEY_HARDWARE_ERROR, SCSI_ASC_LOGICAL_UNIT_COMMUNICATION_FAILURE);
    }
#endif    
    // #### FALLTHROU ?
}

static int SCSI_CommandPassed(void)
{
    if (stm32wb_usbd_msc_control.bot_cbw.dDataLength != 0)
    {
        if (stm32wb_usbd_msc_control.bot_cbw.bmFlags & 0x80)
        {
            /* case (4) Hi > Dn */
            return SCSI_STATUS_DATA_IN_CSW_CMD_PASSED;
        }
        else
        {
            /* case (9) Ho > Dn */
            return SCSI_STATUS_DATA_OUT_CSW_CMD_PASSED;
        }
    }  
    else
    {
        /* case (1) Hn = Dn */
        return SCSI_STATUS_CSW_CMD_PASSED;
    }
}

static int SCSI_CommandFailed(void)
{
    if (stm32wb_usbd_msc_control.bot_cbw.dDataLength == 0)
    {
        /* case (1) Hn = Dn */
        return SCSI_STATUS_CSW_CMD_FAILED;
    }
    else
    {
        if (stm32wb_usbd_msc_control.bot_cbw.bmFlags & 0x80)
        {
            /* case (4) Hi > Dn */
            return SCSI_STATUS_DATA_IN_CSW_CMD_FAILED;
        }
        else
        {
            /* case (9) Ho > Dn */
            return SCSI_STATUS_DATA_OUT_CSW_CMD_FAILED;
        }
    }
}

static bool SCSI_CheckLength(uint32_t length, int *p_status_return)
{
    if (stm32wb_usbd_msc_control.bot_cbw.dDataLength == 0)
    {
        /* case (1) Hn = Dn */
        *p_status_return = SCSI_STATUS_CSW_CMD_PASSED;

        return false;
    }
    else
    {
        if (!(stm32wb_usbd_msc_control.bot_cbw.bmFlags & 0x80))
        {
            /* case (10) Ho <> Di */
            *p_status_return = SCSI_STATUS_DATA_OUT_CSW_PHASE_ERROR;
                
            return false;
        }
    }

    return true;
}

static bool SCSI_CheckAllocation(uint32_t allocation, uint32_t length, int *p_status_return)
{
    if (allocation)
    {
        if (stm32wb_usbd_msc_control.bot_cbw.dDataLength == 0)
        {
            /* case (2) Hn < Di */
            *p_status_return = SCSI_STATUS_DATA_IN_CSW_PHASE_ERROR;

            return false;
        }
        else
        {
            if (stm32wb_usbd_msc_control.bot_cbw.bmFlags & 0x80)
            {
                if (stm32wb_usbd_msc_control.bot_cbw.dDataLength != allocation)
                {
                    SCSI_SenseCode(SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_INVALID_FIELD_IN_COMMAND_PACKET);

                    *p_status_return = SCSI_CommandFailed();

                    return false;
                }
            }
            else
            {
                /* case (10) Ho <> Di */
                *p_status_return = SCSI_STATUS_DATA_OUT_CSW_PHASE_ERROR;
                
                return false;
            }
        }
    }
    else
    {
        if (stm32wb_usbd_msc_control.bot_cbw.dDataLength == 0)
        {
            /* case (1) Hn = Dn */
            *p_status_return = SCSI_STATUS_CSW_CMD_PASSED;
        }
        else
        {
            if (stm32wb_usbd_msc_control.bot_cbw.bmFlags & 0x80)
            {
                /* case (4) Hi > Dn */
                *p_status_return = SCSI_STATUS_DATA_IN_CSW_CMD_PASSED;
            }
            else
            {
                /* case (9) Ho > Dn */
                *p_status_return =  SCSI_STATUS_DATA_OUT_CSW_CMD_PASSED;
            }
        }

        return false;
    }

    return true;
}

static dosfs_device_t * SCSI_QueryDevice(void)
{
    dosfs_device_t *device;
    uint8_t media, write_protected;
    uint32_t blk_count, au_size, serial;
    int status = F_NO_ERROR;

    if (!stm32wb_usbd_msc_control.scsi_medium_present)
    {
        SCSI_SenseCode(SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT);

        return NULL;
    }

    if (stm32wb_usbd_msc_control.scsi_state == SCSI_STATE_EJECTED)
    {
        // SCSI_SenseCode(SCSI_SKEY_NOT_READY, SCSI_ASC_LOGICAL_UNIT_NOT_READY_MANUAL_INTERVENTION_REQUIRED);
        SCSI_SenseCode(SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT);

        return NULL;
    }

    if (stm32wb_usbd_msc_control.scsi_state == SCSI_STATE_STOPPED)
    {
        SCSI_SenseCode(SCSI_SKEY_NOT_READY, SCSI_ASC_LOGICAL_UNIT_NOT_READY_INITIALIZING_COMMAND_REQUIRED);

        return NULL;
    }
    
    device = stm32wb_usbd_msc_control.device;
    
    if (stm32wb_usbd_msc_control.scsi_state != SCSI_STATE_STARTED)
    {
        status = (*device->interface->start)(device->context, &media, &write_protected, &blk_count, &au_size, &serial);
        
        if (status != F_NO_ERROR)
        {
            SCSI_SenseStatus(status);

            return NULL;
        }

        stm32wb_usbd_msc_control.scsi_state = SCSI_STATE_STARTED;

        stm32wb_usbd_msc_control.scsi_write_protected = write_protected;
        stm32wb_usbd_msc_control.scsi_blk_count = blk_count;
        stm32wb_usbd_msc_control.scsi_blk_size = 512;

        MSC_BOT_Notify(MSC_BOT_NOTIFY_START);
    }

    return device;
}

static int SCSI_TestUnitReady(const uint8_t *params)
{
    dosfs_device_t *device;

    // armv7m_rtt_printf("SCSI_TestUnitReady()\n");

    device = SCSI_QueryDevice();

    if (!device)
    {
        return SCSI_CommandFailed();
    }

    return SCSI_CommandPassed();
}

static int SCSI_StartStopUnit(const uint8_t *params)
{
    dosfs_device_t *device;
    bool start, loej;
    uint8_t media, write_protected;
    uint32_t blk_count, au_size, serial;
    int status = F_NO_ERROR;

    start = !!(params[4] & 0x01);
    loej = !!(params[4] & 0x02);

    // armv7m_rtt_printf("SCSI_StartStopUnit(start=%d, loej=%d)\n", start, loej);

    if (!stm32wb_usbd_msc_control.scsi_medium_present)
    {
        SCSI_SenseCode(SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT);

        return SCSI_CommandFailed();
    }

    if (stm32wb_usbd_msc_control.scsi_state == SCSI_STATE_EJECTED)
    {
        // SCSI_SenseCode(SCSI_SKEY_NOT_READY, SCSI_ASC_LOGICAL_UNIT_NOT_READY_MANUAL_INTERVENTION_REQUIRED);
        SCSI_SenseCode(SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT);

        return SCSI_CommandFailed();
    }

    if (!start && loej && !stm32wb_usbd_msc_control.scsi_medium_removal)
    {
        SCSI_SenseCode(SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_MEDIA_REMOVAL_PREVENTED);

        return SCSI_CommandFailed();
    }

    device = stm32wb_usbd_msc_control.device;

    if (start)
    {
        if (stm32wb_usbd_msc_control.scsi_state != SCSI_STATE_STARTED)
        {
            status = (*device->interface->start)(device->context, &media, &write_protected, &blk_count, &au_size, &serial);
            
            if (status != F_NO_ERROR)
            {
                SCSI_SenseStatus(status);

                return SCSI_CommandFailed();
            }
            
            stm32wb_usbd_msc_control.scsi_state = SCSI_STATE_STARTED;
            
            stm32wb_usbd_msc_control.scsi_write_protected = write_protected;
            stm32wb_usbd_msc_control.scsi_blk_count = blk_count;
            stm32wb_usbd_msc_control.scsi_blk_size = 512;

            MSC_BOT_Notify(MSC_BOT_NOTIFY_START);
        }
    }
    else
    {
        if (stm32wb_usbd_msc_control.scsi_state != SCSI_STATE_STOPPED)
        {
            status = (*device->interface->stop)(device->context, !!loej);
            
            if (status != F_NO_ERROR)
            {
                SCSI_SenseStatus(status);

                return SCSI_CommandFailed();
            }

            stm32wb_usbd_msc_control.scsi_state = (loej ? SCSI_STATE_EJECTED : SCSI_STATE_STOPPED);

            MSC_BOT_Notify(loej ? MSC_BOT_NOTIFY_EJECT : MSC_BOT_NOTIFY_STOP);
        }
    }
        
    return SCSI_CommandPassed();
}

static int SCSI_PreventAllowMediumRemoval(const uint8_t *params)
{
    dosfs_device_t *device;
    bool prevent;
    int status = F_NO_ERROR;
    
    prevent = !!(params[4] & 0x01);
    
    // armv7m_rtt_printf("SCSI_PreventAllowMediumRemoval(prevent=%d)\n", prevent);

    device = SCSI_QueryDevice();

    if (!device)
    {
        return SCSI_CommandFailed();
    }

    if (!prevent && !stm32wb_usbd_msc_control.scsi_medium_removal)
    {
        status = (*device->interface->sync)(device->context);
            
        if (status != F_NO_ERROR)
        {
            SCSI_SenseStatus(status);
            
            return SCSI_CommandFailed();
        }
    }
    
    stm32wb_usbd_msc_control.scsi_medium_removal = !prevent;
    
    return SCSI_CommandPassed();
}

static int SCSI_Inquiry(const uint8_t *params)
{
    uint32_t length, allocation;
    int status;

    
    // armv7m_rtt_printf("SCSI_Inquiry()\n");
    
    allocation = params[4];

    if ((params[1] & 0x01) || (params[2] != 0x00))
    {
        SCSI_SenseCode(SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_INVALID_FIELD_IN_COMMAND_PACKET);

        return SCSI_CommandFailed();
    }

    length = scsi_inquiry_data[4] + 5;

    if (!SCSI_CheckAllocation(allocation, length, &status))
    {
        return status;
    }

    if (length > allocation)
    {
	length = allocation;
    }
    
    MSC_BOT_Response(scsi_inquiry_data, length);
    
    return SCSI_STATUS_DATA;
}

static int SCSI_RequestSense(const uint8_t *params)
{
    uint8_t *data;
    uint32_t length, allocation;
    int status;
    
    // armv7m_rtt_printf("SCSI_RequestSense() -> SKEY=%02x, ASC=%04x\n", stm32wb_usbd_msc_control.scsi_sense_skey, stm32wb_usbd_msc_control.scsi_sense_asc);
    
    allocation = params[4];

    data = stm32wb_usbd_msc_control.scsi_sense_data;

    data[0]  = 0x70;
    data[1]  = 0x00;
    data[2]  = stm32wb_usbd_msc_control.scsi_sense_skey;
    data[3]  = 0x00;
    data[4]  = 0x00;
    data[5]  = 0x00;
    data[6]  = 0x00;
    data[7]  = 0x0a;
    data[8]  = 0x00;
    data[9]  = 0x00;
    data[10] = 0x00;
    data[11] = 0x00;
    data[12] = (uint8_t)(stm32wb_usbd_msc_control.scsi_sense_asc >> 8);
    data[13] = (uint8_t)(stm32wb_usbd_msc_control.scsi_sense_asc >> 0);
    data[14] = 0x00;
    data[15] = 0x00;
    data[16] = 0x00;
    data[17] = 0x00;
    
    stm32wb_usbd_msc_control.scsi_sense_skey = SCSI_SKEY_NO_SENSE;
    stm32wb_usbd_msc_control.scsi_sense_asc = SCSI_ASC_NO_SENSE;

    length = SCSI_DATA_LENGTH_REQUEST_SENSE;
    
    if (!SCSI_CheckAllocation(allocation, length, &status))
    {
        return status;
    }

    if (length > allocation)
    {
	length = allocation;
    }

    MSC_BOT_Response(data, length);
    
    return SCSI_STATUS_DATA;
}

static int SCSI_ModeSense6(const uint8_t *params)
{
    dosfs_device_t *device;
    uint8_t *data;
    uint8_t pc, page_code;
    uint32_t length;
    int status;

    pc = params[2] >> 6;
    page_code = params[2] & 0x3f;

    // armv7m_rtt_printf("SCSI_ModeSense6(pc=%d, page_code=%d)\n", pc, page_code);

    if (pc == 3)
    {
        SCSI_SenseCode(SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_SAVING_PARAMETERS_NOT_SUPPORTED);

        return SCSI_CommandFailed();
    }

    device = SCSI_QueryDevice();

    if (!device)
    {
        return SCSI_CommandFailed();
    }
    
    data = stm32wb_usbd_msc_control.scsi_data[0];
    length = SCSI_DATA_LENGTH_MODE_SENSE_6;
    
    data[0] = 0x03;
    data[1] = 0x00;
    data[2] = stm32wb_usbd_msc_control.scsi_write_protected ? 0x90 : 0x10;
    data[3] = 0x00;

    if ((page_code == 0x08) || (page_code == 0x3f))
    {
        data[4+0]  = 0x08;
        data[4+1]  = 0x12;

        if (pc == 1)
        {
            data[4+2]  = 0x00;
            data[4+3]  = 0x00;
            data[4+4]  = 0x00;
            data[4+5]  = 0x00;
            data[4+6]  = 0x00;
            data[4+7]  = 0x00;
            data[4+8]  = 0x00;
            data[4+9]  = 0x00;
            data[4+10] = 0x00;
            data[4+11] = 0x00;
        }
        else
        {
            data[4+2]  = 0x04; // WCE
            data[4+3]  = 0x00;
            data[4+4]  = 0xff;
            data[4+5]  = 0xff;
            data[4+6]  = 0x00;
            data[4+7]  = 0x00;
            data[4+8]  = 0xff;
            data[4+9]  = 0xff;
            data[4+10] = 0xff;
            data[4+11] = 0xff;
        }

        data[4+12] = 0x00;
        data[4+13] = 0x00;
        data[4+14] = 0x00;
        data[4+15] = 0x00;
        data[4+16] = 0x00;
        data[4+17] = 0x00;
        data[4+18] = 0x00;
        data[4+19] = 0x00;
        
        length += 20;
    }

    if (!SCSI_CheckLength(length, &status))
    {
        return status;
    }

    MSC_BOT_Response(data, length);
        
    return SCSI_STATUS_DATA;
}

static int SCSI_ReadCapacity10(const uint8_t *params)
{
    dosfs_device_t *device;
    uint8_t *data;
    int status;
    
    // armv7m_rtt_printf("SCSI_ReadCapacity10()\n");

    device = SCSI_QueryDevice();
    
    if (!device)
    {
        return SCSI_CommandFailed();
    }
    
    data = stm32wb_usbd_msc_control.scsi_data[0];
    
    data[0] = (uint8_t)((stm32wb_usbd_msc_control.scsi_blk_count -1) >> 24);
    data[1] = (uint8_t)((stm32wb_usbd_msc_control.scsi_blk_count -1) >> 16);
    data[2] = (uint8_t)((stm32wb_usbd_msc_control.scsi_blk_count -1) >>  8);
    data[3] = (uint8_t)((stm32wb_usbd_msc_control.scsi_blk_count -1) >>  0);
    data[4] = (uint8_t)(stm32wb_usbd_msc_control.scsi_blk_size >> 24);
    data[5] = (uint8_t)(stm32wb_usbd_msc_control.scsi_blk_size >> 16);
    data[6] = (uint8_t)(stm32wb_usbd_msc_control.scsi_blk_size >>  8);
    data[7] = (uint8_t)(stm32wb_usbd_msc_control.scsi_blk_size >>  0);

    if (!SCSI_CheckLength(SCSI_DATA_LENGTH_READ_CAPACITY_10, &status))
    {
        return status;
    }

    MSC_BOT_Response(data, SCSI_DATA_LENGTH_READ_CAPACITY_10);
    
    return SCSI_STATUS_DATA;
}

static int SCSI_Read10(const uint8_t *params)
{
    dosfs_device_t *device;
    int status = F_NO_ERROR;
    
    stm32wb_usbd_msc_control.scsi_blk_address = (params[2] << 24) | (params[3] << 16) | (params[4] << 8) | (params[5] << 0);
    stm32wb_usbd_msc_control.scsi_blk_length = (params[7] << 8) | (params[8] << 0);

    // armv7m_rtt_printf("SCSI_Read10(address=%08x, length=%d)\n", stm32wb_usbd_msc_control.scsi_blk_address, stm32wb_usbd_msc_control.scsi_blk_length);

    device = SCSI_QueryDevice();

    if (!device)
    {
        return SCSI_CommandFailed();
    }
    
    /* Check address range */
    if ((stm32wb_usbd_msc_control.scsi_blk_address + stm32wb_usbd_msc_control.scsi_blk_length) > stm32wb_usbd_msc_control.scsi_blk_count)
    {
        SCSI_SenseCode(SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE);

        return SCSI_CommandFailed();
    }

    if (stm32wb_usbd_msc_control.bot_cbw.dDataLength != (stm32wb_usbd_msc_control.scsi_blk_length * stm32wb_usbd_msc_control.scsi_blk_size))
    {
        SCSI_SenseCode(SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_INVALID_FIELD_IN_COMMAND_PACKET);

        return SCSI_CommandFailed();
    }

    if (stm32wb_usbd_msc_control.scsi_blk_length == 0)
    {
        return SCSI_CommandPassed();
    }

    if (!(stm32wb_usbd_msc_control.bot_cbw.bmFlags & 0x80))
    {
        /* case (10) Ho <> Di */
        return SCSI_STATUS_DATA_OUT_CSW_PHASE_ERROR;
    }

    stm32wb_usbd_msc_control.scsi_blk_busy = true;
    stm32wb_usbd_msc_control.scsi_blk_fault = false;
    stm32wb_usbd_msc_control.scsi_blk_index = 0;

    status = (*device->interface->read)(device->context,
                                        stm32wb_usbd_msc_control.scsi_blk_address,
                                        stm32wb_usbd_msc_control.scsi_data[stm32wb_usbd_msc_control.scsi_blk_index],
                                        1);

    if (status != F_NO_ERROR)
    {
        SCSI_SenseStatus(status);

        stm32wb_usbd_msc_control.scsi_blk_fault = true;
    }
    
    MSC_BOT_SetupTransmit(stm32wb_usbd_msc_control.scsi_blk_length, (stm32wb_usbd_msc_control.scsi_blk_fault ? MSC_BOT_CSW_CMD_FAILED : MSC_BOT_CSW_CMD_PASSED));

    stm32wb_usbd_msc_control.scsi_blk_address += 1;
    stm32wb_usbd_msc_control.scsi_blk_length -= 1;
    stm32wb_usbd_msc_control.scsi_blk_index ^= 1;
    
    if (!stm32wb_usbd_msc_control.scsi_blk_length)
    {
        stm32wb_usbd_msc_control.scsi_blk_busy = false;
    }
    else
    {
        if (!stm32wb_usbd_msc_control.scsi_blk_fault)
        {
            status = (*device->interface->read)(device->context,
                                                stm32wb_usbd_msc_control.scsi_blk_address,
                                                stm32wb_usbd_msc_control.scsi_data[stm32wb_usbd_msc_control.scsi_blk_index],
                                                1);
            
            if (status != F_NO_ERROR)
            {
                SCSI_SenseStatus(status);
                
                stm32wb_usbd_msc_control.scsi_blk_fault = true;
            }
        }
	    
        MSC_BOT_ContinueTransmit(stm32wb_usbd_msc_control.scsi_blk_length, (stm32wb_usbd_msc_control.scsi_blk_fault ? MSC_BOT_CSW_CMD_FAILED : MSC_BOT_CSW_CMD_PASSED));
	    
        stm32wb_usbd_msc_control.scsi_blk_address += 1;
        stm32wb_usbd_msc_control.scsi_blk_length -= 1;
        stm32wb_usbd_msc_control.scsi_blk_index ^= 1;
        
        if (!stm32wb_usbd_msc_control.scsi_blk_length)
        {
            stm32wb_usbd_msc_control.scsi_blk_busy = false;
	}
    }
    
    return SCSI_STATUS_DATA;
}

static int SCSI_Write10(const uint8_t *params)
{
    dosfs_device_t *device;

    stm32wb_usbd_msc_control.scsi_blk_address = (params[2] << 24) | (params[3] << 16) | (params[4] << 8) | (params[5] << 0);
    stm32wb_usbd_msc_control.scsi_blk_length = (params[7] << 8) | (params[8] << 0);
    stm32wb_usbd_msc_control.scsi_blk_fua = !!(params[1] & 0x08);

    // armv7m_rtt_printf("SCSI_Write10(address=%08x, length=%d, fua=%d)\n", stm32wb_usbd_msc_control.scsi_blk_address, stm32wb_usbd_msc_control.scsi_blk_length, stm32wb_usbd_msc_control.scsi_blk_fua);
    
    device = SCSI_QueryDevice();

    if (!device)
    {
        return SCSI_CommandFailed();
    }
    
    /* Check If media is write-protected */
    if (stm32wb_usbd_msc_control.scsi_write_protected)
    {
        SCSI_SenseCode(SCSI_SKEY_NOT_READY, SCSI_ASC_WRITE_PROTECTED_MEDIA);

        return SCSI_CommandFailed();
    } 
    
    /* Check address range */
    if ((stm32wb_usbd_msc_control.scsi_blk_address + stm32wb_usbd_msc_control.scsi_blk_length) > stm32wb_usbd_msc_control.scsi_blk_count)
    {
        SCSI_SenseCode(SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE);

        return SCSI_CommandFailed();
    }

    if (stm32wb_usbd_msc_control.bot_cbw.dDataLength != (stm32wb_usbd_msc_control.scsi_blk_length * stm32wb_usbd_msc_control.scsi_blk_size))
    {
        SCSI_SenseCode(SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_INVALID_FIELD_IN_COMMAND_PACKET);

        return SCSI_CommandFailed();
    }

    if (stm32wb_usbd_msc_control.scsi_blk_length == 0)
    {
        return SCSI_CommandPassed();
    }

    if (stm32wb_usbd_msc_control.bot_cbw.bmFlags & 0x80)
    {
        /* case (8) Hi <> Do */
        return SCSI_STATUS_DATA_IN_CSW_PHASE_ERROR;
    }
    
    stm32wb_usbd_msc_control.scsi_blk_busy = true;
    stm32wb_usbd_msc_control.scsi_blk_fault = false;
    stm32wb_usbd_msc_control.scsi_blk_index = 0;
    
    MSC_BOT_SetupReceive(stm32wb_usbd_msc_control.scsi_blk_length);

    return SCSI_STATUS_DATA;
}

static int SCSI_SynchronizeCache10(const uint8_t *params)
{
    dosfs_device_t *device;
    int status = F_NO_ERROR;
    
    // armv7m_rtt_printf("SCSI_SynchronizeCache10()\n");
    
    device = SCSI_QueryDevice();

    if (!device)
    {
        return SCSI_CommandFailed();
    }

    status = (*device->interface->sync)(device->context);
    
    if (status != F_NO_ERROR)
    {
        SCSI_SenseStatus(status);
        
        return SCSI_CommandFailed();
    }
    
    return SCSI_CommandPassed();
}

static void SCSI_ProcessStart(void)
{
    stm32wb_usbd_msc_control.scsi_state = SCSI_STATE_READY;
    stm32wb_usbd_msc_control.scsi_sense_skey = SCSI_SKEY_NO_SENSE;
    stm32wb_usbd_msc_control.scsi_sense_asc = SCSI_ASC_NO_SENSE;
    
    stm32wb_usbd_msc_control.scsi_medium_removal = true;
    
    stm32wb_usbd_msc_control.scsi_write_protected = false;
    stm32wb_usbd_msc_control.scsi_blk_size = 512;
    stm32wb_usbd_msc_control.scsi_blk_count = 0;
    stm32wb_usbd_msc_control.scsi_blk_address = 0;
    stm32wb_usbd_msc_control.scsi_blk_length = 0;
    stm32wb_usbd_msc_control.scsi_blk_fua = false;
    stm32wb_usbd_msc_control.scsi_blk_busy = false;
    stm32wb_usbd_msc_control.scsi_blk_fault = false;
    stm32wb_usbd_msc_control.scsi_blk_index = 0;
}

static void SCSI_ProcessStop(void)
{
    if (stm32wb_usbd_msc_control.scsi_medium_present)
    {
        if ((stm32wb_usbd_msc_control.scsi_state == SCSI_STATE_STARTED) || (stm32wb_usbd_msc_control.scsi_state == SCSI_STATE_STOPPED))
        {
            (*stm32wb_usbd_msc_control.device->interface->stop)(stm32wb_usbd_msc_control.device->context, true);

            MSC_BOT_Notify(MSC_BOT_NOTIFY_EJECT);
        }
    }
}

static void SCSI_ProcessReset(void)
{
    stm32wb_usbd_msc_control.scsi_blk_fault = true;
}

static void SCSI_ProcessAttach(void)
{
    uint8_t *cache_data;

    cache_data = stm32wb_usbd_msc_control.device->cache;
    
    stm32wb_usbd_msc_control.scsi_data[0] = cache_data;
    stm32wb_usbd_msc_control.scsi_data[1] = cache_data + STM32WB_USBD_MSC_DATA_BLOCK_SIZE;

    stm32wb_usbd_msc_control.scsi_medium_present = true;

    MSC_BOT_Notify(MSC_BOT_NOTIFY_ATTACH);
}

static void SCSI_ProcessDetach(void)
{
    stm32wb_usbd_msc_control.scsi_blk_fault = true;

    stm32wb_usbd_msc_control.scsi_medium_present = false;

    stm32wb_usbd_msc_control.scsi_write_protected = false;
    stm32wb_usbd_msc_control.scsi_blk_size = 512;
    stm32wb_usbd_msc_control.scsi_blk_count = 0;

    if (stm32wb_usbd_msc_control.device)
    {
        armv7m_atomic_and(&stm32wb_usbd_msc_control.device->lock, ~DOSFS_DEVICE_LOCK_SCSI);
    }

    MSC_BOT_Notify(MSC_BOT_NOTIFY_DETACH);
}

static void SCSI_ProcessCommand(void)
{
    const uint8_t *params = (const uint8_t*)&stm32wb_usbd_msc_control.bot_cbw.CB[0];
    int status;
    
    switch (params[0]) {
    case SCSI_OPCODE_TEST_UNIT_READY:
        status = SCSI_TestUnitReady(params);
        break;
        
    case SCSI_OPCODE_START_STOP_UNIT:
        status = SCSI_StartStopUnit(params);
        break;
    
    case SCSI_OPCODE_PREVENT_ALLOW_MEDIUM_REMOVAL:
        status = SCSI_PreventAllowMediumRemoval(params);
        break;
    
    case SCSI_OPCODE_INQUIRY:
        status = SCSI_Inquiry(params);
        break;
    
    case SCSI_OPCODE_REQUEST_SENSE:
        status = SCSI_RequestSense(params);
        break;

    case SCSI_OPCODE_MODE_SENSE_6:
        status = SCSI_ModeSense6(params);
        break;
        
    case SCSI_OPCODE_READ_CAPACITY_10:
        status = SCSI_ReadCapacity10(params);
        break;
    
    case SCSI_OPCODE_READ_10:
        status = SCSI_Read10(params); 
        break;
    
    case SCSI_OPCODE_WRITE_10:
        status = SCSI_Write10(params);
        break;

    case SCSI_OPCODE_SYNCHRONIZE_CACHE_10:
        status = SCSI_SynchronizeCache10(params);
        break;
        
    default:
        // armv7m_rtt_printf("SCSI_OPCODE = %02x\n", params[0]);

        SCSI_SenseCode(SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_INVALID_COMMAND_OPERATION_CODE);

        status = SCSI_CommandFailed();
        break;
    }

    switch (status) {
    case SCSI_STATUS_CSW_CMD_PASSED:
    case SCSI_STATUS_CSW_CMD_FAILED:
        MSC_BOT_SendCSW(status);
        break;

    case SCSI_STATUS_DATA:
        break;

    case SCSI_STATUS_DATA_IN_CSW_CMD_PASSED:
    case SCSI_STATUS_DATA_IN_CSW_CMD_FAILED:
    case SCSI_STATUS_DATA_IN_CSW_PHASE_ERROR:
    case SCSI_STATUS_DATA_OUT_CSW_CMD_PASSED:
    case SCSI_STATUS_DATA_OUT_CSW_CMD_FAILED:
    case SCSI_STATUS_DATA_OUT_CSW_PHASE_ERROR:
        MSC_BOT_Abort(status);
        break;

    default:
        break;
    }
}

static void SCSI_ProcessRead(void)
{
    dosfs_device_t *device;
    int status = F_NO_ERROR;
    
    if (stm32wb_usbd_msc_control.scsi_blk_busy)
    {
        if (!stm32wb_usbd_msc_control.scsi_blk_fault)
        {
            device = stm32wb_usbd_msc_control.device;

            status = (*device->interface->read)(device->context,
                                                stm32wb_usbd_msc_control.scsi_blk_address,
                                                stm32wb_usbd_msc_control.scsi_data[stm32wb_usbd_msc_control.scsi_blk_index],
                                                1);

            if (status != F_NO_ERROR)
            {
                SCSI_SenseStatus(status);
                
                stm32wb_usbd_msc_control.scsi_blk_fault = true;
            }
        }
        
        MSC_BOT_ContinueTransmit(stm32wb_usbd_msc_control.scsi_blk_length, (stm32wb_usbd_msc_control.scsi_blk_fault ? MSC_BOT_CSW_CMD_FAILED : MSC_BOT_CSW_CMD_PASSED));
        
        stm32wb_usbd_msc_control.scsi_blk_address += 1;
        stm32wb_usbd_msc_control.scsi_blk_length -= 1;
        stm32wb_usbd_msc_control.scsi_blk_index ^= 1;
        
        if (!stm32wb_usbd_msc_control.scsi_blk_length)
        {
            stm32wb_usbd_msc_control.scsi_blk_busy = false;
        }
    }
}

static void SCSI_ProcessWrite(void)
{
    dosfs_device_t *device;
    int status = F_NO_ERROR;

    if (stm32wb_usbd_msc_control.scsi_blk_busy)
    {
        if (!stm32wb_usbd_msc_control.scsi_blk_fault)
        {
            device = stm32wb_usbd_msc_control.device;

            status = (*device->interface->write)(device->context,
                                                 stm32wb_usbd_msc_control.scsi_blk_address,
                                                 stm32wb_usbd_msc_control.scsi_data[stm32wb_usbd_msc_control.scsi_blk_index],
                                                 1,
                                                 ((stm32wb_usbd_msc_control.scsi_blk_length == 1) ? stm32wb_usbd_msc_control.scsi_blk_fua : false));

            if (status != F_NO_ERROR)
            {
                SCSI_SenseStatus(status);
                
                stm32wb_usbd_msc_control.scsi_blk_fault = true;
            }
        }
        
        stm32wb_usbd_msc_control.scsi_blk_address += 1;
        stm32wb_usbd_msc_control.scsi_blk_length -= 1;
        stm32wb_usbd_msc_control.scsi_blk_index ^= 1;
        
        if (stm32wb_usbd_msc_control.scsi_blk_length)
        {
            MSC_BOT_ContinueReceive(stm32wb_usbd_msc_control.scsi_blk_length);
        }
        else
        {
            MSC_BOT_FinishReceive((stm32wb_usbd_msc_control.scsi_blk_fault ? MSC_BOT_CSW_CMD_FAILED : MSC_BOT_CSW_CMD_PASSED));
            
            stm32wb_usbd_msc_control.scsi_blk_busy = false;
        }
    }
}

/***********************************************************************************************************************/

static void MSC_BOT_Event(uint32_t events)
{
    k_event_send(&stm32wb_usbd_msc_task, events);
}

static __attribute__((noreturn)) void MSC_BOT_Routine(void *context) 
{
    uint32_t events;

    while (1)
    {
        k_event_receive(~0, (K_EVENT_ANY | K_EVENT_CLEAR), K_TIMEOUT_FOREVER, &events);

	if (events & MSC_BOT_EVENT_CONFIGURE)
	{
	}
    
	if (events & MSC_BOT_EVENT_START)
	{
	    MSC_BOT_Start();
    
	    SCSI_ProcessStart();
	}

	if (events & MSC_BOT_EVENT_STOP)
	{
	    events = 0;
	
	    SCSI_ProcessStop();

	    MSC_BOT_Stop();
	}

	if (events & MSC_BOT_EVENT_RESET)
	{
            events &= ~(MSC_BOT_EVENT_CLEAR_HALT_DATA_IN | MSC_BOT_EVENT_CLEAR_HALT_DATA_OUT | MSC_BOT_EVENT_CBW);

	    MSC_BOT_Reset();

	    SCSI_ProcessReset();
	}

	if (events & MSC_BOT_EVENT_ATTACH)
	{
	    SCSI_ProcessAttach();
        }

	if (events & MSC_BOT_EVENT_DETACH)
	{
	    SCSI_ProcessDetach();
        }
        
	if (events & MSC_BOT_EVENT_CLEAR_HALT_DATA_IN)
	{
	    MSC_BOT_ClearHaltDataIn();
	}

	if (events & MSC_BOT_EVENT_CLEAR_HALT_DATA_OUT)
	{
	    MSC_BOT_ClearHaltDataOut();
	}

	if (events & MSC_BOT_EVENT_TRANSMIT_0)
	{
            SCSI_ProcessRead();
	}

	if (events & MSC_BOT_EVENT_TRANSMIT_1)
	{
            SCSI_ProcessRead();
	}
        
	if (events & MSC_BOT_EVENT_RECEIVE_0)
	{
            SCSI_ProcessWrite();
	}

	if (events & MSC_BOT_EVENT_RECEIVE_1)
	{
            SCSI_ProcessWrite();
	}
        
	if (events & MSC_BOT_EVENT_RESPONSE)
	{
	    if (stm32wb_usbd_msc_control.bot_state == MSC_BOT_STATE_DATA_IN_LAST)
	    {
		MSC_BOT_SendCSW(MSC_BOT_CSW_CMD_PASSED);
	    }
	    else
	    {
		MSC_BOT_Stall(STM32WB_USBD_MSC_DATA_IN_EP_ADDR);

		if (stm32wb_usbd_msc_control.bot_csw.bStatus == MSC_BOT_CSW_PHASE_ERROR)
		{
		    stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_RECOVERY_RESET;
		}
		else
		{
		    stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_HALT_DATA_IN;
		}
	    }
	}

	if (events & MSC_BOT_EVENT_CSW)
	{
            stm32wb_usbd_msc_control.bot_csw.dSignature = ~MSC_BOT_CSW_SIGNATURE;
	}
    
	if (events & MSC_BOT_EVENT_CBW)
	{
	    stm32wb_usbd_msc_control.bot_csw.dSignature = MSC_BOT_CSW_SIGNATURE;
	    stm32wb_usbd_msc_control.bot_csw.dTag = stm32wb_usbd_msc_control.bot_cbw.dTag;
	    stm32wb_usbd_msc_control.bot_csw.dDataResidue = stm32wb_usbd_msc_control.bot_cbw.dDataLength;

            SCSI_ProcessCommand();
	}
    }
}

static void __svc_MSC_BOT_Start(void)
{
    stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_IDLE;
    stm32wb_usbd_msc_control.bot_csw.dSignature = ~MSC_BOT_CSW_SIGNATURE;

    stm32wb_usbd_dcd_ep_enable(STM32WB_USBD_MSC_DATA_OUT_EP_ADDR);
    stm32wb_usbd_dcd_ep_enable(STM32WB_USBD_MSC_DATA_IN_EP_ADDR);
    
    stm32wb_usbd_dcd_ep_receive(STM32WB_USBD_MSC_DATA_OUT_EP_ADDR, (uint8_t *)&stm32wb_usbd_msc_control.bot_cbw, MSC_BOT_CBW_LENGTH, stm32wb_usbd_msc_receive_cbw, NULL);    
}

static void __svc_MSC_BOT_Stop(void)
{
    stm32wb_usbd_dcd_ep_disable(STM32WB_USBD_MSC_DATA_OUT_EP_ADDR);
    stm32wb_usbd_dcd_ep_disable(STM32WB_USBD_MSC_DATA_IN_EP_ADDR);

    stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_NONE;
}

static void __svc_MSC_BOT_Reset(void)
{
    stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_RECOVERY_HALT;
}

static void __svc_MSC_BOT_Abort(uint8_t abort)
{
    switch (abort) {
    case MSC_BOT_ABORT_DATA_IN_CSW_CMD_PASSED:
    case MSC_BOT_ABORT_DATA_IN_CSW_CMD_FAILED:
    case MSC_BOT_ABORT_DATA_IN_CSW_PHASE_ERROR:
        stm32wb_usbd_dcd_ep_stall(STM32WB_USBD_MSC_DATA_IN_EP_ADDR);

        stm32wb_usbd_msc_control.bot_csw.bStatus = (abort - MSC_BOT_ABORT_DATA_IN_CSW_CMD_PASSED);

        stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_HALT_DATA_IN;
        break;

    case MSC_BOT_ABORT_DATA_OUT_CSW_CMD_PASSED:
    case MSC_BOT_ABORT_DATA_OUT_CSW_CMD_FAILED:
    case MSC_BOT_ABORT_DATA_OUT_CSW_PHASE_ERROR:
        stm32wb_usbd_dcd_ep_stall(STM32WB_USBD_MSC_DATA_OUT_EP_ADDR);

        stm32wb_usbd_msc_control.bot_csw.bStatus = (abort - MSC_BOT_ABORT_DATA_OUT_CSW_CMD_PASSED);

        stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_HALT_DATA_OUT;
        break;

    default:
        break;
    }
}

static void __svc_MSC_BOT_Stall(uint8_t ep_addr)
{      
    stm32wb_usbd_dcd_ep_stall(ep_addr);
}

static void __svc_MSC_BOT_ClearHaltDataIn(void)
{      
    uint8_t bot_state_previous = stm32wb_usbd_msc_control.bot_state;

    if (stm32wb_usbd_msc_control.bot_state == MSC_BOT_STATE_RECOVERY_HALT)
    {
	stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_HALT_DATA_OUT;
    }
    
    if (stm32wb_usbd_msc_control.bot_state == MSC_BOT_STATE_HALT_DATA_IN)
    {
	stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_IDLE;
    }

    if ((stm32wb_usbd_msc_control.bot_state != bot_state_previous) && (stm32wb_usbd_msc_control.bot_state == MSC_BOT_STATE_IDLE))
    {
        if (stm32wb_usbd_msc_control.bot_csw.dSignature == MSC_BOT_CSW_SIGNATURE)
        {
#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
            stm32wb_usbd_dcd_lpm_reference(STM32WB_USBD_DCD_EP_MASK(STM32WB_USBD_MSC_DATA_IN_EP_ADDR));
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */

            stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_MSC_DATA_IN_EP_ADDR, (uint8_t *)&stm32wb_usbd_msc_control.bot_csw, MSC_BOT_CSW_LENGTH, stm32wb_usbd_msc_transmit_csw, NULL);
        }
        else
        {
            stm32wb_usbd_dcd_ep_receive(STM32WB_USBD_MSC_DATA_OUT_EP_ADDR, (uint8_t *)&stm32wb_usbd_msc_control.bot_cbw, MSC_BOT_CBW_LENGTH, stm32wb_usbd_msc_receive_cbw, NULL);    
        }
    }
}

static void __svc_MSC_BOT_ClearHaltDataOut(void)
{      
    uint8_t bot_state_previous = stm32wb_usbd_msc_control.bot_state;

    if (stm32wb_usbd_msc_control.bot_state == MSC_BOT_STATE_RECOVERY_HALT)
    {
	stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_HALT_DATA_IN;
    }
    
    if (stm32wb_usbd_msc_control.bot_state == MSC_BOT_STATE_HALT_DATA_OUT)
    {
	stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_IDLE;
    }

    if ((stm32wb_usbd_msc_control.bot_state != bot_state_previous) && (stm32wb_usbd_msc_control.bot_state == MSC_BOT_STATE_IDLE))
    {
        if (stm32wb_usbd_msc_control.bot_csw.dSignature == MSC_BOT_CSW_SIGNATURE)
        {
#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
            stm32wb_usbd_dcd_lpm_reference(STM32WB_USBD_DCD_EP_MASK(STM32WB_USBD_MSC_DATA_IN_EP_ADDR));
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */

            stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_MSC_DATA_IN_EP_ADDR, (uint8_t *)&stm32wb_usbd_msc_control.bot_csw, MSC_BOT_CSW_LENGTH, stm32wb_usbd_msc_transmit_csw, NULL);
        }
        else
        {
            stm32wb_usbd_dcd_ep_receive(STM32WB_USBD_MSC_DATA_OUT_EP_ADDR, (uint8_t *)&stm32wb_usbd_msc_control.bot_cbw, MSC_BOT_CBW_LENGTH, stm32wb_usbd_msc_receive_cbw, NULL);    
        }
    }
}

static void __svc_MSC_BOT_SendCSW(uint8_t status)
{
    stm32wb_usbd_msc_control.bot_csw.bStatus = status;
    stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_IDLE;

    stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_MSC_DATA_IN_EP_ADDR, (uint8_t *)&stm32wb_usbd_msc_control.bot_csw, MSC_BOT_CSW_LENGTH, stm32wb_usbd_msc_transmit_csw, NULL);
}

static void __svc_MSC_BOT_Response(const uint8_t *data, uint32_t length)
{
    if (stm32wb_usbd_msc_control.bot_csw.dDataResidue > length)
    {
	/* case (5) Hi > Di */
	stm32wb_usbd_msc_control.bot_csw.bStatus = MSC_BOT_CSW_CMD_PASSED;

	stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_DATA_IN_LAST_STALL;
	
	stm32wb_usbd_msc_control.bot_csw.dDataResidue -= length;
    }
    else
    {
        if (length > stm32wb_usbd_msc_control.bot_csw.dDataResidue)
        {
            /* case (7) Hi < Di */
            stm32wb_usbd_msc_control.bot_csw.bStatus = MSC_BOT_CSW_PHASE_ERROR;
            
            stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_DATA_IN_LAST_STALL;

            length = stm32wb_usbd_msc_control.bot_csw.dDataResidue;
        }
        else
        {
            /* case (6) Hi == Di */
        }
        
        stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_DATA_IN_LAST;

        stm32wb_usbd_msc_control.bot_csw.dDataResidue = 0;
    }
    
    stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_MSC_DATA_IN_EP_ADDR, (uint8_t*)data, length, stm32wb_usbd_msc_transmit_response, NULL);
}

static void __svc_MSC_BOT_SetupTransmit(uint32_t length, uint8_t status)
{
    if (length == 1)
    {
	stm32wb_usbd_msc_control.bot_csw.bStatus = status;

	stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_IDLE;
    }
    else
    {
	stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_DATA_IN;
    }

    stm32wb_usbd_msc_control.bot_index = 0;
    stm32wb_usbd_msc_control.bot_count = 1;

    stm32wb_usbd_msc_control.bot_tx_length = length;
    stm32wb_usbd_msc_control.bot_tx_busy = true;

    stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_MSC_DATA_IN_EP_ADDR, (uint8_t *)&stm32wb_usbd_msc_control.scsi_data[stm32wb_usbd_msc_control.bot_index][0], STM32WB_USBD_MSC_DATA_BLOCK_SIZE, stm32wb_usbd_msc_transmit_data, NULL);
}

static void __svc_MSC_BOT_ContinueTransmit(uint32_t length, uint8_t status)
{
    if (length == 1)
    {
	stm32wb_usbd_msc_control.bot_csw.bStatus = status;

	stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_IDLE;
    }
    
    armv7m_atomic_incb(&stm32wb_usbd_msc_control.bot_count);

    if (!stm32wb_usbd_msc_control.bot_tx_busy)
    {
	stm32wb_usbd_msc_control.bot_tx_busy = true;
		
	stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_MSC_DATA_IN_EP_ADDR, (uint8_t *)&stm32wb_usbd_msc_control.scsi_data[stm32wb_usbd_msc_control.bot_index][0], STM32WB_USBD_MSC_DATA_BLOCK_SIZE, stm32wb_usbd_msc_transmit_data, NULL);
    }
}

static void __svc_MSC_BOT_SetupReceive(uint32_t length)
{
    stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_DATA_OUT;  

    stm32wb_usbd_msc_control.bot_index = 0;
    stm32wb_usbd_msc_control.bot_count = 0;

    stm32wb_usbd_msc_control.bot_rx_length = length;
    stm32wb_usbd_msc_control.bot_rx_busy = true;
    
    stm32wb_usbd_dcd_ep_receive(STM32WB_USBD_MSC_DATA_OUT_EP_ADDR, (uint8_t *)&stm32wb_usbd_msc_control.scsi_data[stm32wb_usbd_msc_control.bot_index][0], STM32WB_USBD_MSC_DATA_BLOCK_SIZE, stm32wb_usbd_msc_receive_data, NULL);
}

static void __svc_MSC_BOT_ContinueReceive(uint32_t length)
{
    armv7m_atomic_decb(&stm32wb_usbd_msc_control.bot_count);
    
    if (stm32wb_usbd_msc_control.bot_rx_length)
    {
	if (!stm32wb_usbd_msc_control.bot_rx_busy)
	{
	    stm32wb_usbd_msc_control.bot_rx_busy = true;
	    
	    stm32wb_usbd_dcd_ep_receive(STM32WB_USBD_MSC_DATA_OUT_EP_ADDR, (uint8_t *)&stm32wb_usbd_msc_control.scsi_data[stm32wb_usbd_msc_control.bot_index][0], STM32WB_USBD_MSC_DATA_BLOCK_SIZE, stm32wb_usbd_msc_receive_data, NULL);
	}
    }
}

static void __svc_MSC_BOT_FinishReceive(uint8_t status)
{
    armv7m_atomic_decb(&stm32wb_usbd_msc_control.bot_count);

    __svc_MSC_BOT_SendCSW(status);
}

static void __svc_MSC_BOT_Notify(uint32_t notify)
{
    if (stm32wb_usbd_msc_control.callback)
    {
        (*stm32wb_usbd_msc_control.callback)(stm32wb_usbd_msc_control.context, notify);
    }
}

static void MSC_BOT_Start(void)
{
    armv7m_svcall_0((uint32_t)&__svc_MSC_BOT_Start);
}

static void MSC_BOT_Stop(void)
{
    armv7m_svcall_0((uint32_t)&__svc_MSC_BOT_Stop);
}

static void MSC_BOT_Reset(void)
{
    armv7m_svcall_0((uint32_t)&__svc_MSC_BOT_Reset);
}

static void MSC_BOT_Abort(uint8_t abort)
{
    armv7m_svcall_1((uint32_t)&__svc_MSC_BOT_Abort, (uint32_t)abort);
}

static void MSC_BOT_Stall(uint8_t ep_addr)
{
    armv7m_svcall_1((uint32_t)&__svc_MSC_BOT_Stall, (uint32_t)ep_addr);
}

static void MSC_BOT_ClearHaltDataIn(void)
{
    armv7m_svcall_0((uint32_t)&__svc_MSC_BOT_ClearHaltDataIn);
}

static void MSC_BOT_ClearHaltDataOut(void)
{
    armv7m_svcall_0((uint32_t)&__svc_MSC_BOT_ClearHaltDataOut);
}

static void MSC_BOT_SendCSW(uint8_t status)
{
    armv7m_svcall_1((uint32_t)&__svc_MSC_BOT_SendCSW, (uint32_t)status);
}

static void MSC_BOT_Response(const uint8_t *data, uint32_t length)
{
    armv7m_svcall_2((uint32_t)&__svc_MSC_BOT_Response, (uint32_t)data, (uint32_t)length);
}

static void MSC_BOT_SetupTransmit(uint32_t length, uint8_t status)
{
    armv7m_svcall_2((uint32_t)&__svc_MSC_BOT_SetupTransmit, (uint32_t)length, (uint32_t)status);
}

static void MSC_BOT_ContinueTransmit(uint32_t length, uint8_t status)
{
    armv7m_svcall_2((uint32_t)&__svc_MSC_BOT_ContinueTransmit, (uint32_t)length, (uint32_t)status);
}

static void MSC_BOT_SetupReceive(uint32_t length)
{
    armv7m_svcall_1((uint32_t)&__svc_MSC_BOT_SetupReceive, (uint32_t)length);
}

static void MSC_BOT_ContinueReceive(uint32_t length)
{
    armv7m_svcall_1((uint32_t)&__svc_MSC_BOT_ContinueReceive, (uint32_t)length);
}

static void MSC_BOT_FinishReceive(uint8_t status)
{
    armv7m_svcall_1((uint32_t)&__svc_MSC_BOT_FinishReceive, (uint32_t)status);
}

static void MSC_BOT_Notify(uint32_t notify)
{
    armv7m_svcall_1((uint32_t)&__svc_MSC_BOT_Notify, (uint32_t)notify);
}

/***********************************************************************************************************************/

static void __attribute__((optimize("O3"))) stm32wb_usbd_msc_transmit_data(void *context, uint8_t ep_addr)
{
    if ((stm32wb_usbd_msc_control.bot_tx_length > 1) && (stm32wb_usbd_msc_control.bot_count > 1))
    {
        stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_MSC_DATA_IN_EP_ADDR, (uint8_t *)&stm32wb_usbd_msc_control.scsi_data[stm32wb_usbd_msc_control.bot_index ^ 1][0], STM32WB_USBD_MSC_DATA_BLOCK_SIZE, stm32wb_usbd_msc_transmit_data, NULL);

        stm32wb_usbd_msc_control.bot_tx_busy = true;
    }
    else
    {
        stm32wb_usbd_msc_control.bot_tx_busy = false;
    }

    MSC_BOT_Event((stm32wb_usbd_msc_control.bot_index & 1) ? MSC_BOT_EVENT_TRANSMIT_1 : MSC_BOT_EVENT_TRANSMIT_0);
    
    __armv7m_atomic_dec(&stm32wb_usbd_msc_control.bot_tx_length);
    
    stm32wb_usbd_msc_control.bot_csw.dDataResidue -= stm32wb_usbd_msc_control.scsi_blk_size;
    
    stm32wb_usbd_msc_control.bot_index ^= 1;
    
    __armv7m_atomic_decb(&stm32wb_usbd_msc_control.bot_count);

    if (!stm32wb_usbd_msc_control.bot_tx_length)
    {
        stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_MSC_DATA_IN_EP_ADDR, (uint8_t *)&stm32wb_usbd_msc_control.bot_csw, MSC_BOT_CSW_LENGTH, stm32wb_usbd_msc_transmit_csw, NULL);
    }

#if 0    
    MSC_BOT_Event((stm32wb_usbd_msc_control.bot_index & 1) ? MSC_BOT_EVENT_TRANSMIT_1 : MSC_BOT_EVENT_TRANSMIT_0);
    
    __armv7m_atomic_dec(&stm32wb_usbd_msc_control.bot_tx_length);
    
    stm32wb_usbd_msc_control.bot_csw.dDataResidue -= stm32wb_usbd_msc_control.scsi_blk_size;
    
    stm32wb_usbd_msc_control.bot_index ^= 1;
    
    __armv7m_atomic_decb(&stm32wb_usbd_msc_control.bot_count);

    if (stm32wb_usbd_msc_control.bot_tx_length)
    {
        if (stm32wb_usbd_msc_control.bot_count != 0)
        {
            stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_MSC_DATA_IN_EP_ADDR, (uint8_t *)&stm32wb_usbd_msc_control.scsi_data[stm32wb_usbd_msc_control.bot_index][0], STM32WB_USBD_MSC_DATA_BLOCK_SIZE, stm32wb_usbd_msc_transmit_data, NULL);
        }
        else
        {
            stm32wb_usbd_msc_control.bot_tx_busy = false;
        }
    }
    else
    {
        stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_MSC_DATA_IN_EP_ADDR, (uint8_t *)&stm32wb_usbd_msc_control.bot_csw, MSC_BOT_CSW_LENGTH, stm32wb_usbd_msc_transmit_csw, NULL);

        stm32wb_usbd_msc_control.bot_tx_busy = false;
    }
#endif    
}

static void __attribute__((optimize("O3"))) stm32wb_usbd_msc_transmit_response(void *context, uint8_t ep_addr)
{
    MSC_BOT_Event(MSC_BOT_EVENT_RESPONSE);
}

static void __attribute__((optimize("O3"))) stm32wb_usbd_msc_transmit_csw(void *context, uint8_t ep_addr)
{
#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
    stm32wb_usbd_dcd_lpm_unreference(STM32WB_USBD_DCD_EP_MASK(STM32WB_USBD_MSC_DATA_IN_EP_ADDR));
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */

    stm32wb_usbd_dcd_ep_receive(STM32WB_USBD_MSC_DATA_OUT_EP_ADDR, (uint8_t *)&stm32wb_usbd_msc_control.bot_cbw, MSC_BOT_CBW_LENGTH, stm32wb_usbd_msc_receive_cbw, NULL);    
    
    MSC_BOT_Event(MSC_BOT_EVENT_CSW);
}

static void __attribute__((optimize("O3"))) stm32wb_usbd_msc_receive_data(void *context, uint8_t ep_addr, uint32_t count)
{
    bool rx_busy;

    if ((stm32wb_usbd_msc_control.bot_rx_length > 1) && (stm32wb_usbd_msc_control.bot_count < 1))
    {
        stm32wb_usbd_dcd_ep_receive(STM32WB_USBD_MSC_DATA_OUT_EP_ADDR, (uint8_t *)&stm32wb_usbd_msc_control.scsi_data[stm32wb_usbd_msc_control.bot_index ^ 1][0], STM32WB_USBD_MSC_DATA_BLOCK_SIZE, stm32wb_usbd_msc_receive_data, NULL);

        rx_busy = true;
    }
    else
    {
        rx_busy = false;
    }
    
    MSC_BOT_Event((stm32wb_usbd_msc_control.bot_index & 1) ? MSC_BOT_EVENT_RECEIVE_1 : MSC_BOT_EVENT_RECEIVE_0);
    
    __armv7m_atomic_dec(&stm32wb_usbd_msc_control.bot_rx_length);
    
    stm32wb_usbd_msc_control.bot_csw.dDataResidue -= stm32wb_usbd_msc_control.scsi_blk_size;
    
    stm32wb_usbd_msc_control.bot_index ^= 1;
    
    __armv7m_atomic_incb(&stm32wb_usbd_msc_control.bot_count);

    stm32wb_usbd_msc_control.bot_rx_busy = rx_busy;
        
#if 0  
    MSC_BOT_Event((stm32wb_usbd_msc_control.bot_index & 1) ? MSC_BOT_EVENT_RECEIVE_1 : MSC_BOT_EVENT_RECEIVE_0);
    
    __armv7m_atomic_dec(&stm32wb_usbd_msc_control.bot_rx_length);
    
    stm32wb_usbd_msc_control.bot_csw.dDataResidue -= stm32wb_usbd_msc_control.scsi_blk_size;
    
    stm32wb_usbd_msc_control.bot_index ^= 1;
    
    __armv7m_atomic_incb(&stm32wb_usbd_msc_control.bot_count);
    
    stm32wb_usbd_msc_control.bot_rx_busy = false;

    if (stm32wb_usbd_msc_control.bot_rx_length)
    {
	if (stm32wb_usbd_msc_control.bot_count != 2)
	{
	    stm32wb_usbd_msc_control.bot_rx_busy = true;
	    
	    stm32wb_usbd_dcd_ep_receive(STM32WB_USBD_MSC_DATA_OUT_EP_ADDR, (uint8_t *)&stm32wb_usbd_msc_control.scsi_data[stm32wb_usbd_msc_control.bot_index][0], STM32WB_USBD_MSC_DATA_BLOCK_SIZE, stm32wb_usbd_msc_receive_data, NULL);
	}
    }
#endif    
}

static void __attribute__((optimize("O3"))) stm32wb_usbd_msc_receive_cbw(void *context, uint8_t ep_addr, uint32_t count)
{
#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
    stm32wb_usbd_dcd_lpm_reference(STM32WB_USBD_DCD_EP_MASK(STM32WB_USBD_MSC_DATA_IN_EP_ADDR));
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */

    if ((count != MSC_BOT_CBW_LENGTH) ||
        (stm32wb_usbd_msc_control.bot_cbw.dSignature != MSC_BOT_CBW_SIGNATURE) ||
        (stm32wb_usbd_msc_control.bot_cbw.bLUN > stm32wb_usbd_msc_control.max_lun) || 
        (stm32wb_usbd_msc_control.bot_cbw.bCBLength == 0) || 
        (stm32wb_usbd_msc_control.bot_cbw.bCBLength > 16))
    {
        stm32wb_usbd_dcd_ep_stall(STM32WB_USBD_MSC_DATA_IN_EP_ADDR);
        stm32wb_usbd_dcd_ep_stall(STM32WB_USBD_MSC_DATA_OUT_EP_ADDR);

        stm32wb_usbd_msc_control.bot_state = MSC_BOT_STATE_RECOVERY_RESET;
    }
    else
    {
        MSC_BOT_Event(MSC_BOT_EVENT_CBW);
    }
}

static void stm32wb_usbd_msc_configure(void *context, uint8_t interface)
{
    stm32wb_usbd_msc_control.interface = interface;
    
    k_task_create(&stm32wb_usbd_msc_task, "USB/MSC", MSC_BOT_Routine, NULL, 15, &stm32wb_usbd_msc_stack[0], sizeof(stm32wb_usbd_msc_stack), 0);

    stm32wb_usbd_dcd_ep_configure(STM32WB_USBD_MSC_DATA_IN_EP_ADDR, STM32WB_USBD_DCD_EP_TYPE_BULK, STM32WB_USBD_MSC_DATA_MAX_PACKET_SIZE);
    stm32wb_usbd_dcd_ep_configure(STM32WB_USBD_MSC_DATA_OUT_EP_ADDR, STM32WB_USBD_DCD_EP_TYPE_BULK, STM32WB_USBD_MSC_DATA_MAX_PACKET_SIZE);
    
    MSC_BOT_Event(MSC_BOT_EVENT_CONFIGURE);
}

static void stm32wb_usbd_msc_start(void *context)
{
    MSC_BOT_Event(MSC_BOT_EVENT_START);
}

static void stm32wb_usbd_msc_stop(void *context)
{
    MSC_BOT_Event(MSC_BOT_EVENT_STOP);
}

static int stm32wb_usbd_msc_request(void *context, int state, const stm32wb_usbd_request_t *request, uint8_t *data, const uint8_t **p_data_return, uint16_t *p_length_return, stm32wb_usbd_status_routine_t *p_status_routine_return)
{
    uint8_t ep_addr;
    int status;

    status = STM32WB_USBD_REQUEST_STATUS_UNHANDLED;
    
    switch (request->bmRequestType & USB_REQ_RECIPIENT_MASK) {
    case USB_REQ_RECIPIENT_INTERFACE: {
        switch (request->bmRequestType & USB_REQ_TYPE_MASK) {
        case USB_REQ_TYPE_CLASS: {
            switch (request->bRequest) {
            case MSC_BOT_GET_MAX_LUN: {
                if ((request->wValue == 0) && (request->wLength == 1))
                {
                    *p_data_return = data;
                    *p_length_return = 1;
                        
                    data[0] = stm32wb_usbd_msc_control.max_lun;
                        
                    status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                }
                break;
            }
		    
            case MSC_BOT_RESET: {
                if ((request->wValue == 0) && (request->wLength == 0))
                {
                    MSC_BOT_Event(MSC_BOT_EVENT_RESET);

                    status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                }
                break;
            }

            default:
                break;
            }
            break;
        }

        default:
            break;
        }

	break;
    }

    case USB_REQ_RECIPIENT_ENDPOINT: {
	ep_addr = request->wIndex & 255;
	
        switch (request->bmRequestType & USB_REQ_TYPE_MASK) {
        case USB_REQ_TYPE_STANDARD: {
            switch (request->bRequest) {
            case USB_REQ_CODE_CLEAR_FEATURE: {
                if (request->wValue == USB_FEATURE_ENDPOINT_HALT)
                {
                    if (ep_addr == STM32WB_USBD_MSC_DATA_IN_EP_ADDR)
                    {
                        if ((stm32wb_usbd_msc_control.bot_state != MSC_BOT_STATE_RECOVERY_RESET) && stm32wb_usbd_dcd_ep_is_stalled(STM32WB_USBD_MSC_DATA_IN_EP_ADDR))
                        {
                            stm32wb_usbd_dcd_ep_unstall(STM32WB_USBD_MSC_DATA_IN_EP_ADDR);
                        
                            MSC_BOT_Event(MSC_BOT_EVENT_CLEAR_HALT_DATA_IN);

                            status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                        }
                        else
                        {
                            status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                        }
                    }
                    else
                    {
                        if ((stm32wb_usbd_msc_control.bot_state != MSC_BOT_STATE_RECOVERY_RESET) && stm32wb_usbd_dcd_ep_is_stalled(STM32WB_USBD_MSC_DATA_OUT_EP_ADDR))
                        {
                            stm32wb_usbd_dcd_ep_unstall(STM32WB_USBD_MSC_DATA_OUT_EP_ADDR);
                        
                            MSC_BOT_Event(MSC_BOT_EVENT_CLEAR_HALT_DATA_OUT);

                            status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                        }
                        else
                        {
                            status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                        }
                    }
                }
                break;
            }

            default:
                break;
            }
            break;
        }

        default:
            break;
        }
	break;
    }
	
    default:
	break;
    }

    return status;
}

static void stm32wb_usbd_msc_suspend(void *context)
{
}

static void stm32wb_usbd_msc_resume(void *context)
{
}

const stm32wb_usbd_class_t stm32wb_usbd_msc_class = {
    stm32wb_usbd_msc_configure,
    stm32wb_usbd_msc_start,
    stm32wb_usbd_msc_stop,
    stm32wb_usbd_msc_request,
    stm32wb_usbd_msc_suspend,
    stm32wb_usbd_msc_resume,
    NULL,
    STM32WB_USBD_MSC_INTERFACE_COUNT,
    (STM32WB_USBD_EP_MASK(STM32WB_USBD_MSC_DATA_IN_EP_ADDR) | STM32WB_USBD_EP_MASK(STM32WB_USBD_MSC_DATA_OUT_EP_ADDR)),
};

/************************************************************************************************************************************************************/

bool stm32wb_usbd_msc_attach(dosfs_device_t *device, stm32wb_usbd_msc_event_callback_t callback, void *context)
{
    uint32_t o_lock, n_lock;

    while (1)
    {
        o_lock = device->lock;
        
        if (o_lock & (DOSFS_DEVICE_LOCK_INIT | DOSFS_DEVICE_LOCK_SFLASH | DOSFS_DEVICE_LOCK_VOLUME | DOSFS_DEVICE_LOCK_SCSI))
        {
            return false;
        }

        n_lock = o_lock | DOSFS_DEVICE_LOCK_SCSI;
        
        if (armv7m_atomic_cas(&device->lock, o_lock, n_lock) == o_lock)
        {
            break;
        }
    }

    stm32wb_usbd_msc_control.device = device;
    stm32wb_usbd_msc_control.callback = callback;
    stm32wb_usbd_msc_control.context = context;
    
    MSC_BOT_Event(MSC_BOT_EVENT_ATTACH);

    return true;
}

bool stm32wb_usbd_msc_detach(void)
{
    MSC_BOT_Event(MSC_BOT_EVENT_DETACH);
    
    return true;
}


/************************************************************************************************************************************************************/
