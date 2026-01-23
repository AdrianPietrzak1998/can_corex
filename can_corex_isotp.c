/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Author: Adrian Pietrzak
 * GitHub: https://github.com/AdrianPietrzak1998
 * Created: Jan 22, 2026
 */

#include "can_corex_isotp.h"
#include "string.h"
#include <assert.h>

/* External tick getter from can_corex.c */
#if CCX_TICK_FROM_FUNC
extern CCX_TIME_t (*CCX_get_tick)(void);
#define CCX_GET_TICK ((CCX_get_tick != NULL) ? CCX_get_tick() : ((CCX_TIME_t)0))
#else
extern CCX_TIME_t *CCX_tick;
#define CCX_GET_TICK (*(CCX_tick))
#endif

/* ISO-TP frame constants */
#define ISOTP_SF_MAX_DATA 7    /* Single Frame max data bytes */
#define ISOTP_FF_DATA_BYTES 6  /* First Frame data bytes (after 2-byte length) */
#define ISOTP_CF_DATA_BYTES 7  /* Consecutive Frame data bytes */
#define ISOTP_FC_SIZE 3        /* Flow Control frame size (PCI, FS, BS, STmin) */

#define ISOTP_MAX_WAIT_FRAMES 10 /* Maximum number of WAIT FC frames to tolerate */

/* Global RX instance pointer for parser callback */
static CCX_ISOTP_RX_t *g_ISOTP_RX_Instance = NULL;

/* Global TX instance pointer for FC parser callback */
static CCX_ISOTP_TX_t *g_ISOTP_TX_Instance = NULL;

/* ========================================================================
 * HELPER FUNCTIONS
 * ======================================================================== */

static inline uint8_t ISOTP_GetPCI(const CCX_message_t *msg)
{
    return msg->Data[0] & 0xF0;
}

static inline uint8_t ISOTP_GetSFDataLength(const CCX_message_t *msg)
{
    return msg->Data[0] & 0x0F;
}

static inline uint16_t ISOTP_GetFFDataLength(const CCX_message_t *msg)
{
    return (uint16_t)(((msg->Data[0] & 0x0F) << 8) | msg->Data[1]);
}

static inline uint8_t ISOTP_GetCFSequenceNumber(const CCX_message_t *msg)
{
    return msg->Data[0] & 0x0F;
}

static inline uint8_t ISOTP_GetFCFlowStatus(const CCX_message_t *msg)
{
    return msg->Data[1];
}

static inline uint8_t ISOTP_GetFCBlockSize(const CCX_message_t *msg)
{
    return msg->Data[2];
}

static inline uint8_t ISOTP_GetFCSTmin(const CCX_message_t *msg)
{
    return msg->Data[3];
}

static inline void ISOTP_SendCANMessage(CCX_instance_t *CanInstance, uint32_t ID, uint8_t IDE_flag, const uint8_t *Data,
                                        uint8_t DLC, const CCX_ISOTP_Padding_t *Padding)
{
    CCX_message_t msg;
    msg.ID = ID;
    msg.IDE_flag = IDE_flag;
    msg.DLC = DLC;
    memcpy(msg.Data, Data, DLC);

    /* Apply padding if enabled */
    if (Padding->Enable && DLC < 8)
    {
        for (uint8_t i = DLC; i < 8; i++)
        {
            msg.Data[i] = Padding->PaddingByte;
        }
        msg.DLC = 8;
    }

    CCX_TX_PushMsg(CanInstance, &msg);
}

/* ========================================================================
 * TX FUNCTIONS
 * ======================================================================== */

CCX_ISOTP_Status_t CCX_ISOTP_TX_Init(CCX_ISOTP_TX_t *Instance, const CCX_ISOTP_TX_Config_t *Config)
{
    if (NULL == Instance || NULL == Config)
    {
        return CCX_ISOTP_ERROR_NULL_PTR;
    }

    if (NULL == Config->CanInstance)
    {
        return CCX_ISOTP_ERROR_NULL_PTR;
    }

    memcpy(&Instance->Config, Config, sizeof(CCX_ISOTP_TX_Config_t));
    Instance->State = CCX_ISOTP_TX_STATE_IDLE;
    Instance->TxData = NULL;
    Instance->TxDataLength = 0;
    Instance->TxDataOffset = 0;
    Instance->SequenceNumber = 0;
    Instance->BlockCounter = 0;
    Instance->LastTick = 0;
    Instance->WaitFramesRemaining = ISOTP_MAX_WAIT_FRAMES;

    /* Register this instance for FC parser */
    CCX_ISOTP_TX_SetInstance(Instance);

    return CCX_ISOTP_OK;
}

CCX_ISOTP_Status_t CCX_ISOTP_Transmit(CCX_ISOTP_TX_t *Instance, const uint8_t *Data, uint16_t Length)
{
    if (NULL == Instance || NULL == Data)
    {
        return CCX_ISOTP_ERROR_NULL_PTR;
    }

    if (Length == 0 || Length > CCX_ISOTP_MAX_DATA_SIZE)
    {
        return CCX_ISOTP_ERROR_INVALID_ARG;
    }

    if (Instance->State != CCX_ISOTP_TX_STATE_IDLE)
    {
        return CCX_ISOTP_ERROR_BUSY;
    }

    Instance->TxData = Data;
    Instance->TxDataLength = Length;
    Instance->TxDataOffset = 0;
    Instance->SequenceNumber = 0;
    Instance->BlockCounter = 0;
    Instance->WaitFramesRemaining = ISOTP_MAX_WAIT_FRAMES;

    /* Single Frame */
    if (Length <= ISOTP_SF_MAX_DATA)
    {
        uint8_t frame[8];
        frame[0] = (uint8_t)(CCX_ISOTP_PCI_SF | Length);
        memcpy(&frame[1], Data, Length);

        ISOTP_SendCANMessage(Instance->Config.CanInstance, Instance->Config.TxID, Instance->Config.IDE_TxID, frame,
                             (uint8_t)(1 + Length), &Instance->Config.Padding);

        Instance->State = CCX_ISOTP_TX_STATE_SENDING_SF;
        Instance->LastTick = CCX_GET_TICK;

        /* SF completes immediately */
        Instance->State = CCX_ISOTP_TX_STATE_IDLE;
        if (Instance->Config.OnTransmitComplete != NULL)
        {
            Instance->Config.OnTransmitComplete(Instance);
        }

        return CCX_ISOTP_OK;
    }

    /* Multi-frame: send First Frame */
    uint8_t frame[8];
    frame[0] = (uint8_t)(CCX_ISOTP_PCI_FF | ((Length >> 8) & 0x0F));
    frame[1] = (uint8_t)(Length & 0xFF);
    memcpy(&frame[2], Data, ISOTP_FF_DATA_BYTES);

    ISOTP_SendCANMessage(Instance->Config.CanInstance, Instance->Config.TxID, Instance->Config.IDE_TxID, frame, 8,
                         &Instance->Config.Padding);

    Instance->TxDataOffset = ISOTP_FF_DATA_BYTES;
    Instance->SequenceNumber = 1;
    Instance->State = CCX_ISOTP_TX_STATE_WAIT_FC;
    Instance->LastTick = CCX_GET_TICK;

    return CCX_ISOTP_OK;
}

static inline void CCX_ISOTP_TX_HandleFlowControl(CCX_ISOTP_TX_t *Instance, const CCX_message_t *msg)
{
    assert(Instance != NULL);
    assert(msg != NULL);

    if (Instance->State != CCX_ISOTP_TX_STATE_WAIT_FC && Instance->State != CCX_ISOTP_TX_STATE_SENDING_CF)
    {
        return;
    }

    uint8_t fs = ISOTP_GetFCFlowStatus(msg);

    switch (fs)
    {
    case CCX_ISOTP_FC_CTS:
        /* Continue to send */
        Instance->BlockCounter = ISOTP_GetFCBlockSize(msg);
        Instance->State = CCX_ISOTP_TX_STATE_SENDING_CF;
        Instance->LastTick = CCX_GET_TICK;
        Instance->WaitFramesRemaining = ISOTP_MAX_WAIT_FRAMES;
        break;

    case CCX_ISOTP_FC_WAIT:
        /* Wait for next FC */
        Instance->WaitFramesRemaining--;
        if (Instance->WaitFramesRemaining == 0)
        {
            /* Too many WAIT frames */
            Instance->State = CCX_ISOTP_TX_STATE_IDLE;
            if (Instance->Config.OnError != NULL)
            {
                Instance->Config.OnError(Instance, CCX_ISOTP_ERROR_TIMEOUT);
            }
        }
        Instance->LastTick = CCX_GET_TICK;
        break;

    case CCX_ISOTP_FC_OVFLW:
        /* Receiver buffer overflow */
        Instance->State = CCX_ISOTP_TX_STATE_IDLE;
        if (Instance->Config.OnError != NULL)
        {
            Instance->Config.OnError(Instance, CCX_ISOTP_ERROR_OVERFLOW);
        }
        break;

    default:
        /* Invalid flow status */
        Instance->State = CCX_ISOTP_TX_STATE_IDLE;
        if (Instance->Config.OnError != NULL)
        {
            Instance->Config.OnError(Instance, CCX_ISOTP_ERROR_SEQUENCE);
        }
        break;
    }
}

static inline void CCX_ISOTP_TX_SendConsecutiveFrame(CCX_ISOTP_TX_t *Instance)
{
    assert(Instance != NULL);

    uint16_t remaining = Instance->TxDataLength - Instance->TxDataOffset;
    uint8_t data_len = (remaining > ISOTP_CF_DATA_BYTES) ? ISOTP_CF_DATA_BYTES : (uint8_t)remaining;

    uint8_t frame[8];
    frame[0] = (uint8_t)(CCX_ISOTP_PCI_CF | (Instance->SequenceNumber & 0x0F));
    memcpy(&frame[1], &Instance->TxData[Instance->TxDataOffset], data_len);

    ISOTP_SendCANMessage(Instance->Config.CanInstance, Instance->Config.TxID, Instance->Config.IDE_TxID, frame,
                         (uint8_t)(1 + data_len), &Instance->Config.Padding);

    Instance->TxDataOffset += data_len;
    Instance->SequenceNumber = (Instance->SequenceNumber + 1) & 0x0F;

    /* Check if all data sent */
    if (Instance->TxDataOffset >= Instance->TxDataLength)
    {
        Instance->State = CCX_ISOTP_TX_STATE_IDLE;
        if (Instance->Config.OnTransmitComplete != NULL)
        {
            Instance->Config.OnTransmitComplete(Instance);
        }
    }
    else
    {
        /* Check block counter */
        if (Instance->BlockCounter > 0)
        {
            Instance->BlockCounter--;
            if (Instance->BlockCounter == 0)
            {
                /* Wait for next FC */
                Instance->State = CCX_ISOTP_TX_STATE_WAIT_FC;
            }
        }
    }

    Instance->LastTick = CCX_GET_TICK;
}

void CCX_ISOTP_TX_Poll(CCX_ISOTP_TX_t *Instance)
{
    if (NULL == Instance)
    {
        return;
    }

    CCX_TIME_t current_tick = CCX_GET_TICK;

    switch (Instance->State)
    {
    case CCX_ISOTP_TX_STATE_IDLE:
        /* Nothing to do */
        break;

    case CCX_ISOTP_TX_STATE_WAIT_FC:
        /* Check N_Bs timeout */
        if (current_tick - Instance->LastTick >= Instance->Config.N_Bs)
        {
            Instance->State = CCX_ISOTP_TX_STATE_IDLE;
            if (Instance->Config.OnError != NULL)
            {
                Instance->Config.OnError(Instance, CCX_ISOTP_ERROR_TIMEOUT);
            }
        }
        break;

    case CCX_ISOTP_TX_STATE_SENDING_CF:
        /* Check N_Cs timeout and send next CF */
        if (current_tick - Instance->LastTick >= Instance->Config.N_Cs)
        {
            CCX_ISOTP_TX_SendConsecutiveFrame(Instance);
        }
        break;

    default:
        break;
    }
}

/* ========================================================================
 * RX FUNCTIONS
 * ======================================================================== */

CCX_ISOTP_Status_t CCX_ISOTP_RX_Init(CCX_ISOTP_RX_t *Instance, const CCX_ISOTP_RX_Config_t *Config)
{
    if (NULL == Instance || NULL == Config)
    {
        return CCX_ISOTP_ERROR_NULL_PTR;
    }

    if (NULL == Config->CanInstance || NULL == Config->RxBuffer)
    {
        return CCX_ISOTP_ERROR_NULL_PTR;
    }

    if (Config->RxBufferSize == 0)
    {
        return CCX_ISOTP_ERROR_INVALID_ARG;
    }

    memcpy(&Instance->Config, Config, sizeof(CCX_ISOTP_RX_Config_t));
    Instance->State = CCX_ISOTP_RX_STATE_IDLE;
    Instance->RxDataLength = 0;
    Instance->RxDataOffset = 0;
    Instance->SequenceNumber = 0;
    Instance->BlockCounter = 0;
    Instance->LastTick = 0;
    Instance->LastProgressCallback = 0;

    /* Register this instance for parser */
    CCX_ISOTP_RX_SetInstance(Instance);

    return CCX_ISOTP_OK;
}

static inline void CCX_ISOTP_RX_SendFlowControl(CCX_ISOTP_RX_t *Instance, CCX_ISOTP_FC_FS_t FlowStatus)
{
    assert(Instance != NULL);

    uint8_t frame[8];
    frame[0] = CCX_ISOTP_PCI_FC;
    frame[1] = FlowStatus;
    frame[2] = Instance->Config.BS;
    frame[3] = Instance->Config.STmin;

    /* Use configured padding for FC */
    ISOTP_SendCANMessage(Instance->Config.CanInstance, Instance->Config.TxID, Instance->Config.IDE_TxID, frame,
                         ISOTP_FC_SIZE + 1, &Instance->Config.Padding);
}

static inline void CCX_ISOTP_RX_HandleSingleFrame(CCX_ISOTP_RX_t *Instance, const CCX_message_t *msg)
{
    assert(Instance != NULL);
    assert(msg != NULL);

    uint8_t data_len = ISOTP_GetSFDataLength(msg);

    if (data_len == 0 || data_len > ISOTP_SF_MAX_DATA)
    {
        if (Instance->Config.OnError != NULL)
        {
            Instance->Config.OnError(Instance, CCX_ISOTP_ERROR_INVALID_ARG);
        }
        return;
    }

    if (data_len > Instance->Config.RxBufferSize)
    {
        if (Instance->Config.OnError != NULL)
        {
            Instance->Config.OnError(Instance, CCX_ISOTP_ERROR_BUFFER_TOO_SMALL);
        }
        return;
    }

    /* Copy data */
    memcpy(Instance->Config.RxBuffer, &msg->Data[1], data_len);

    /* Call complete callback */
    if (Instance->Config.OnReceiveComplete != NULL)
    {
        Instance->Config.OnReceiveComplete(Instance, Instance->Config.RxBuffer, data_len);
    }
}

static inline void CCX_ISOTP_RX_HandleFirstFrame(CCX_ISOTP_RX_t *Instance, const CCX_message_t *msg)
{
    assert(Instance != NULL);
    assert(msg != NULL);

    if (Instance->State != CCX_ISOTP_RX_STATE_IDLE)
    {
        /* Already receiving, abort previous */
        if (Instance->Config.OnError != NULL)
        {
            Instance->Config.OnError(Instance, CCX_ISOTP_ERROR_SEQUENCE);
        }
    }

    uint16_t total_len = ISOTP_GetFFDataLength(msg);

    if (total_len <= ISOTP_SF_MAX_DATA || total_len > CCX_ISOTP_MAX_DATA_SIZE)
    {
        if (Instance->Config.OnError != NULL)
        {
            Instance->Config.OnError(Instance, CCX_ISOTP_ERROR_INVALID_ARG);
        }
        return;
    }

    if (total_len > Instance->Config.RxBufferSize)
    {
        /* Buffer overflow */
        CCX_ISOTP_RX_SendFlowControl(Instance, CCX_ISOTP_FC_OVFLW);
        if (Instance->Config.OnError != NULL)
        {
            Instance->Config.OnError(Instance, CCX_ISOTP_ERROR_BUFFER_TOO_SMALL);
        }
        return;
    }

    /* Copy first frame data */
    memcpy(Instance->Config.RxBuffer, &msg->Data[2], ISOTP_FF_DATA_BYTES);

    Instance->RxDataLength = total_len;
    Instance->RxDataOffset = ISOTP_FF_DATA_BYTES;
    Instance->SequenceNumber = 1;
    Instance->BlockCounter = Instance->Config.BS;
    Instance->State = CCX_ISOTP_RX_STATE_RECEIVING_CF;
    Instance->LastTick = CCX_GET_TICK;
    Instance->LastProgressCallback = 0;

    /* Send Flow Control CTS */
    CCX_ISOTP_RX_SendFlowControl(Instance, CCX_ISOTP_FC_CTS);
}

static inline void CCX_ISOTP_RX_HandleConsecutiveFrame(CCX_ISOTP_RX_t *Instance, const CCX_message_t *msg)
{
    assert(Instance != NULL);
    assert(msg != NULL);

    if (Instance->State != CCX_ISOTP_RX_STATE_RECEIVING_CF)
    {
        /* Not expecting CF */
        return;
    }

    uint8_t sn = ISOTP_GetCFSequenceNumber(msg);

    if (sn != Instance->SequenceNumber)
    {
        /* Sequence error */
        Instance->State = CCX_ISOTP_RX_STATE_IDLE;
        if (Instance->Config.OnError != NULL)
        {
            Instance->Config.OnError(Instance, CCX_ISOTP_ERROR_SEQUENCE);
        }
        return;
    }

    /* Copy data */
    uint16_t remaining = Instance->RxDataLength - Instance->RxDataOffset;
    uint8_t data_len = (remaining > ISOTP_CF_DATA_BYTES) ? ISOTP_CF_DATA_BYTES : (uint8_t)remaining;

    memcpy(&Instance->Config.RxBuffer[Instance->RxDataOffset], &msg->Data[1], data_len);

    Instance->RxDataOffset += data_len;
    Instance->SequenceNumber = (Instance->SequenceNumber + 1) & 0x0F;
    Instance->LastTick = CCX_GET_TICK;

    /* Check progress callback */
    if (Instance->Config.ProgressCallbackInterval > 0 && Instance->Config.OnReceiveProgress != NULL)
    {
        uint16_t bytes_since_last =
            Instance->RxDataOffset - Instance->LastProgressCallback;
        if (bytes_since_last >= Instance->Config.ProgressCallbackInterval)
        {
            Instance->Config.OnReceiveProgress(Instance, bytes_since_last, Instance->RxDataLength);
            Instance->LastProgressCallback = Instance->RxDataOffset;
        }
    }

    /* Check if all data received */
    if (Instance->RxDataOffset >= Instance->RxDataLength)
    {
        Instance->State = CCX_ISOTP_RX_STATE_IDLE;

        /* Call progress callback for remaining bytes */
        if (Instance->Config.OnReceiveProgress != NULL)
        {
            uint16_t bytes_since_last = Instance->RxDataOffset - Instance->LastProgressCallback;
            if (bytes_since_last > 0)
            {
                Instance->Config.OnReceiveProgress(Instance, bytes_since_last, Instance->RxDataLength);
            }
        }

        /* Call complete callback */
        if (Instance->Config.OnReceiveComplete != NULL)
        {
            Instance->Config.OnReceiveComplete(Instance, Instance->Config.RxBuffer, Instance->RxDataLength);
        }
        return;
    }

    /* Check block counter */
    if (Instance->Config.BS > 0)
    {
        Instance->BlockCounter--;
        if (Instance->BlockCounter == 0)
        {
            /* Send next FC */
            Instance->BlockCounter = Instance->Config.BS;
            CCX_ISOTP_RX_SendFlowControl(Instance, CCX_ISOTP_FC_CTS);
        }
    }
}

void CCX_ISOTP_RX_Parser(const CCX_instance_t *CanInstance, CCX_message_t *Msg, uint16_t Slot)
{
    (void)CanInstance;
    (void)Slot;

    if (NULL == g_ISOTP_RX_Instance || NULL == Msg)
    {
        return;
    }

    uint8_t pci = ISOTP_GetPCI(Msg);

    switch (pci)
    {
    case CCX_ISOTP_PCI_SF:
        CCX_ISOTP_RX_HandleSingleFrame(g_ISOTP_RX_Instance, Msg);
        break;

    case CCX_ISOTP_PCI_FF:
        CCX_ISOTP_RX_HandleFirstFrame(g_ISOTP_RX_Instance, Msg);
        break;

    case CCX_ISOTP_PCI_CF:
        CCX_ISOTP_RX_HandleConsecutiveFrame(g_ISOTP_RX_Instance, Msg);
        break;

    default:
        break;
    }
}

void CCX_ISOTP_RX_Poll(CCX_ISOTP_RX_t *Instance)
{
    if (NULL == Instance)
    {
        return;
    }

    if (Instance->State == CCX_ISOTP_RX_STATE_IDLE)
    {
        return;
    }

    CCX_TIME_t current_tick = CCX_GET_TICK;

    /* Check N_Cr timeout */
    if (current_tick - Instance->LastTick >= Instance->Config.N_Cr)
    {
        Instance->State = CCX_ISOTP_RX_STATE_IDLE;
        if (Instance->Config.OnError != NULL)
        {
            Instance->Config.OnError(Instance, CCX_ISOTP_ERROR_TIMEOUT);
        }
    }
}

void CCX_ISOTP_RX_SetInstance(CCX_ISOTP_RX_t *Instance)
{
    g_ISOTP_RX_Instance = Instance;
}

void CCX_ISOTP_TX_FC_Parser(const CCX_instance_t *CanInstance, CCX_message_t *Msg, uint16_t Slot)
{
    (void)CanInstance;
    (void)Slot;

    if (NULL == g_ISOTP_TX_Instance || NULL == Msg)
    {
        return;
    }

    uint8_t pci = ISOTP_GetPCI(Msg);

    if (pci == CCX_ISOTP_PCI_FC)
    {
        CCX_ISOTP_TX_HandleFlowControl(g_ISOTP_TX_Instance, Msg);
    }
}

void CCX_ISOTP_TX_SetInstance(CCX_ISOTP_TX_t *Instance)
{
    g_ISOTP_TX_Instance = Instance;
}
