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

#define ISOTP_PCI_TYPE_MASK 0xF0U
#define ISOTP_PCI_VALUE_MASK 0x0FU
#define ISOTP_STANDARD_SF_HEADER_SIZE 1U
#define ISOTP_EXTENDED_SF_HEADER_SIZE 2U
#define ISOTP_STANDARD_FF_HEADER_SIZE 2U
#define ISOTP_EXTENDED_FF_HEADER_SIZE 6U
#define ISOTP_CF_HEADER_SIZE 1U
#define ISOTP_FC_FRAME_DATA_SIZE 3U
#define ISOTP_CLASSIC_CAN_DL 8U
#define ISOTP_FD_DEFAULT_TX_DL CCX_ISOTP_TX_DL_64
#define ISOTP_DEFAULT_MAX_WAIT_FRAMES 10U
#define ISOTP_STANDARD_FF_MAX_LENGTH 4095UL
#define ISOTP_EXTENDED_FF_ESCAPE 0x00U
#define ISOTP_EXTENDED_SF_ESCAPE 0x00U
#define ISOTP_INVALID_CAN_DL 0U
#define ISOTP_SN_MASK 0x0FU

#if CCX_ENABLE_CANFD
#define ISOTP_CAN_FRAME_MAX_PAYLOAD CCX_ISOTP_TX_DL_64
#else
#define CCX_FRAME_FORMAT_CLASSIC 0U
#define ISOTP_CAN_FRAME_MAX_PAYLOAD ISOTP_CLASSIC_CAN_DL
#endif

static inline uint8_t ISOTP_GetMessageDataLength(const CCX_message_t *msg)
{
#if CCX_ENABLE_CANFD
    return CCX_MsgPayloadLen(msg);
#else
    return msg->DLC;
#endif
}

static inline uint8_t ISOTP_GetMessageFrameFormat(const CCX_message_t *msg)
{
#if CCX_ENABLE_CANFD
    return msg->FrameFormat;
#else
    (void)msg;
    return CCX_FRAME_FORMAT_CLASSIC;
#endif
}

static inline uint8_t ISOTP_IsFDFrameFormat(uint8_t frame_format)
{
#if CCX_ENABLE_CANFD
    return (uint8_t)(frame_format == CCX_FRAME_FORMAT_FD || frame_format == CCX_FRAME_FORMAT_FD_BRS);
#else
    (void)frame_format;
    return 0U;
#endif
}

static inline uint8_t ISOTP_FrameFormatsCompatible(uint8_t expected, uint8_t actual)
{
    if (expected == CCX_FRAME_FORMAT_CLASSIC)
    {
        return (uint8_t)(actual == CCX_FRAME_FORMAT_CLASSIC);
    }

    return ISOTP_IsFDFrameFormat(actual);
}

static inline uint8_t ISOTP_IsValidTxDL(uint8_t tx_dl)
{
#if CCX_ENABLE_CANFD
    switch (tx_dl)
    {
    case CCX_ISOTP_TX_DL_8:
    case CCX_ISOTP_TX_DL_12:
    case CCX_ISOTP_TX_DL_16:
    case CCX_ISOTP_TX_DL_20:
    case CCX_ISOTP_TX_DL_24:
    case CCX_ISOTP_TX_DL_32:
    case CCX_ISOTP_TX_DL_48:
    case CCX_ISOTP_TX_DL_64:
        return 1U;
    default:
        return 0U;
    }
#else
    return (uint8_t)(tx_dl == ISOTP_CLASSIC_CAN_DL);
#endif
}

static inline uint8_t ISOTP_RoundUpFDLength(uint8_t used_len)
{
#if CCX_ENABLE_CANFD
    if (used_len <= CCX_ISOTP_TX_DL_8)
    {
        return used_len;
    }
    if (used_len <= CCX_ISOTP_TX_DL_12)
    {
        return CCX_ISOTP_TX_DL_12;
    }
    if (used_len <= CCX_ISOTP_TX_DL_16)
    {
        return CCX_ISOTP_TX_DL_16;
    }
    if (used_len <= CCX_ISOTP_TX_DL_20)
    {
        return CCX_ISOTP_TX_DL_20;
    }
    if (used_len <= CCX_ISOTP_TX_DL_24)
    {
        return CCX_ISOTP_TX_DL_24;
    }
    if (used_len <= CCX_ISOTP_TX_DL_32)
    {
        return CCX_ISOTP_TX_DL_32;
    }
    if (used_len <= CCX_ISOTP_TX_DL_48)
    {
        return CCX_ISOTP_TX_DL_48;
    }
    if (used_len <= CCX_ISOTP_TX_DL_64)
    {
        return CCX_ISOTP_TX_DL_64;
    }
#endif

    (void)used_len;
    return ISOTP_INVALID_CAN_DL;
}

static inline uint8_t ISOTP_GetConfiguredTxDL(uint8_t frame_format, uint8_t configured_tx_dl)
{
#if CCX_ENABLE_CANFD
    if (!ISOTP_IsFDFrameFormat(frame_format))
    {
        return ISOTP_CLASSIC_CAN_DL;
    }

    if (configured_tx_dl == 0U)
    {
        return ISOTP_FD_DEFAULT_TX_DL;
    }

    return configured_tx_dl;
#else
    (void)frame_format;
    (void)configured_tx_dl;
    return ISOTP_CLASSIC_CAN_DL;
#endif
}

static inline uint8_t ISOTP_GetConfiguredMaxWaitFrames(uint8_t configured_max_wait_frames)
{
    return (configured_max_wait_frames == 0U) ? ISOTP_DEFAULT_MAX_WAIT_FRAMES : configured_max_wait_frames;
}

static inline uint8_t ISOTP_GetSingleFrameMaxPayload(uint8_t frame_format, uint8_t can_dl)
{
    if (!ISOTP_IsFDFrameFormat(frame_format) || can_dl <= ISOTP_CLASSIC_CAN_DL)
    {
        return (uint8_t)(ISOTP_CLASSIC_CAN_DL - ISOTP_STANDARD_SF_HEADER_SIZE);
    }

    if (can_dl <= ISOTP_EXTENDED_SF_HEADER_SIZE)
    {
        return 0U;
    }

    return (uint8_t)(can_dl - ISOTP_EXTENDED_SF_HEADER_SIZE);
}

static inline uint8_t ISOTP_GetFirstFrameHeaderSize(CCX_ISOTP_LengthFormat_t length_format)
{
    return (length_format == CCX_ISOTP_LENGTH_FORMAT_EXTENDED) ? ISOTP_EXTENDED_FF_HEADER_SIZE
                                                               : ISOTP_STANDARD_FF_HEADER_SIZE;
}

static inline uint8_t ISOTP_GetFirstFrameDataCapacity(uint8_t can_dl, CCX_ISOTP_LengthFormat_t length_format)
{
    uint8_t header_size = ISOTP_GetFirstFrameHeaderSize(length_format);

    if (can_dl <= header_size)
    {
        return 0U;
    }

    return (uint8_t)(can_dl - header_size);
}

static inline uint8_t ISOTP_GetConsecutiveFrameDataCapacity(uint8_t can_dl)
{
    if (can_dl <= ISOTP_CF_HEADER_SIZE)
    {
        return 0U;
    }

    return (uint8_t)(can_dl - ISOTP_CF_HEADER_SIZE);
}

static inline uint8_t ISOTP_GetTransmitCanDL(uint8_t frame_format, uint8_t tx_dl, uint8_t used_len,
                                             const CCX_ISOTP_Padding_t *padding, uint8_t fixed_length)
{
    if (!ISOTP_IsFDFrameFormat(frame_format))
    {
        if (padding->Enable && used_len < ISOTP_CLASSIC_CAN_DL)
        {
            return ISOTP_CLASSIC_CAN_DL;
        }

        return used_len;
    }

    if (fixed_length || padding->Enable)
    {
        return tx_dl;
    }

    return ISOTP_RoundUpFDLength(used_len);
}

static inline uint8_t ISOTP_GetPCI(const CCX_message_t *msg)
{
    return (uint8_t)(msg->Data[0] & ISOTP_PCI_TYPE_MASK);
}

static inline uint8_t ISOTP_GetStandardSFDataLength(const CCX_message_t *msg)
{
    return (uint8_t)(msg->Data[0] & ISOTP_PCI_VALUE_MASK);
}

static inline CCX_ISOTP_Length_t ISOTP_GetExtendedSFDataLength(const CCX_message_t *msg)
{
    return (CCX_ISOTP_Length_t)msg->Data[1];
}

static inline CCX_ISOTP_Length_t ISOTP_GetStandardFFDataLength(const CCX_message_t *msg)
{
    return (CCX_ISOTP_Length_t)(((CCX_ISOTP_Length_t)(msg->Data[0] & ISOTP_PCI_VALUE_MASK) << 8) | msg->Data[1]);
}

static inline CCX_ISOTP_Length_t ISOTP_GetExtendedFFDataLength(const CCX_message_t *msg)
{
    return (CCX_ISOTP_Length_t)(((CCX_ISOTP_Length_t)msg->Data[2] << 24) | ((CCX_ISOTP_Length_t)msg->Data[3] << 16) |
                                ((CCX_ISOTP_Length_t)msg->Data[4] << 8) | (CCX_ISOTP_Length_t)msg->Data[5]);
}

static inline uint8_t ISOTP_GetCFSequenceNumber(const CCX_message_t *msg)
{
    return (uint8_t)(msg->Data[0] & ISOTP_SN_MASK);
}

static inline uint8_t ISOTP_GetFCFlowStatus(const CCX_message_t *msg)
{
    return (uint8_t)(msg->Data[0] & ISOTP_PCI_VALUE_MASK);
}

static inline uint8_t ISOTP_GetFCBlockSize(const CCX_message_t *msg)
{
    return msg->Data[1];
}

static inline uint8_t ISOTP_GetFCSTmin(const CCX_message_t *msg)
{
    return msg->Data[2];
}

static inline uint8_t ISOTP_STminUsesHighRes(uint8_t stmin_raw)
{
    return (uint8_t)(stmin_raw >= 0xF1U && stmin_raw <= 0xF9U);
}

static inline CCX_TIME_BASE_SCALAR ISOTP_ConvertSTminToBaseTicks(uint8_t stmin_raw)
{
    if (stmin_raw <= 0x7FU)
    {
        return (CCX_TIME_t)stmin_raw;
    }

    return (CCX_TIME_t)127U;
}

static inline CCX_HR_TIME_BASE_SCALAR ISOTP_ConvertSTminToHighResTicks(uint8_t stmin_raw)
{
    if (ISOTP_STminUsesHighRes(stmin_raw))
    {
        return CCX_HR_TIME((uint32_t)(stmin_raw - 0xF0U) * 100U);
    }

    return (CCX_HR_TIME_t)0U;
}

static inline void ISOTP_SetSTminRuntime(CCX_ISOTP_TX_t *Instance, uint8_t stmin_raw)
{
    assert(Instance != NULL);

    Instance->STminUsesHighRes = ISOTP_STminUsesHighRes(stmin_raw);
    if (Instance->STminUsesHighRes)
    {
        Instance->STminHighResTicks = ISOTP_ConvertSTminToHighResTicks(stmin_raw);
        Instance->STminTicks = 0U;
        return;
    }

    Instance->STminTicks = ISOTP_ConvertSTminToBaseTicks(stmin_raw);
    Instance->STminHighResTicks = 0U;
}

static inline CCX_Status_t ISOTP_SendCANMessage(CCX_instance_t *CanInstance, uint32_t ID, uint8_t IDE_flag,
                                                const uint8_t *Data, uint8_t used_len, uint8_t target_can_dl,
                                                const CCX_ISOTP_Padding_t *Padding
#if CCX_ENABLE_CANFD
                                                ,
                                                uint8_t frame_format
#endif
)
{
    CCX_message_t msg = {0};
    uint8_t copy_len = used_len;

    msg.ID = ID;
    msg.IDE_flag = IDE_flag;
#if CCX_ENABLE_CANFD
    msg.FrameFormat = frame_format;
    msg.DLC = ISOTP_IsFDFrameFormat(frame_format) ? CCX_FD_LenToDLC(target_can_dl) : target_can_dl;
#else
    msg.DLC = target_can_dl;
#endif

    if (copy_len > target_can_dl)
    {
        copy_len = target_can_dl;
    }

    memcpy(msg.Data, Data, copy_len);

    if (Padding->Enable && target_can_dl > used_len)
    {
        for (uint8_t i = used_len; i < target_can_dl; i++)
        {
            msg.Data[i] = Padding->PaddingByte;
        }
    }

    return CCX_TX_PushMsg(CanInstance, &msg);
}

static inline uint8_t ISOTP_IsExtendedSingleFrame(uint8_t frame_format, uint8_t can_dl, CCX_ISOTP_Length_t length)
{
    if (!ISOTP_IsFDFrameFormat(frame_format) || can_dl <= ISOTP_CLASSIC_CAN_DL)
    {
        return 0U;
    }

    return (uint8_t)((length > (CCX_ISOTP_Length_t)(ISOTP_CLASSIC_CAN_DL - ISOTP_STANDARD_SF_HEADER_SIZE)) &&
                     (length <= (CCX_ISOTP_Length_t)ISOTP_GetSingleFrameMaxPayload(frame_format, can_dl)));
}

static inline uint8_t ISOTP_IsExtendedFirstFrameLength(CCX_ISOTP_Length_t length)
{
#if CCX_ENABLE_CANFD
    return (uint8_t)(length > ISOTP_STANDARD_FF_MAX_LENGTH);
#else
    (void)length;
    return 0U;
#endif
}

static inline CCX_ISOTP_LengthFormat_t ISOTP_SelectLengthFormat(CCX_ISOTP_Length_t length)
{
    return ISOTP_IsExtendedFirstFrameLength(length) ? CCX_ISOTP_LENGTH_FORMAT_EXTENDED
                                                    : CCX_ISOTP_LENGTH_FORMAT_STANDARD;
}

static inline uint8_t ISOTP_IsValidReceiveFrameFormat(const CCX_ISOTP_RX_t *Instance, const CCX_message_t *msg)
{
#if CCX_ENABLE_CANFD
    return ISOTP_FrameFormatsCompatible(Instance->Config.FrameFormat, ISOTP_GetMessageFrameFormat(msg));
#else
    (void)Instance;
    (void)msg;
    return 1U;
#endif
}

static inline uint8_t ISOTP_IsValidFlowControlFrameFormat(const CCX_ISOTP_TX_t *Instance, const CCX_message_t *msg)
{
#if CCX_ENABLE_CANFD
    return ISOTP_FrameFormatsCompatible(Instance->Config.FrameFormat, ISOTP_GetMessageFrameFormat(msg));
#else
    (void)Instance;
    (void)msg;
    return 1U;
#endif
}

static inline void ISOTP_ResetTXTransfer(CCX_ISOTP_TX_t *Instance)
{
    Instance->TxData = NULL;
    Instance->TxDataLength = 0U;
    Instance->TxDataOffset = 0U;
    Instance->SequenceNumber = 0U;
    Instance->BlockCounter = 0U;
    Instance->WaitFramesRemaining = Instance->MaxWaitFrames;
    Instance->ActiveTxDL = ISOTP_CLASSIC_CAN_DL;
    Instance->LengthFormat = CCX_ISOTP_LENGTH_FORMAT_STANDARD;
}

static inline CCX_ISOTP_Status_t ISOTP_AbortTXOnCANQueueError(CCX_ISOTP_TX_t *Instance, CCX_Status_t CanStatus)
{
    (void)CanStatus;

    Instance->State = CCX_ISOTP_TX_STATE_IDLE;
    ISOTP_ResetTXTransfer(Instance);
    if (Instance->Config.OnError != NULL)
    {
        Instance->Config.OnError(Instance, CCX_ISOTP_ERROR_BUSY, Instance->Config.UserData);
    }

    return CCX_ISOTP_ERROR_BUSY;
}

static inline void ISOTP_ResetRXTransfer(CCX_ISOTP_RX_t *Instance)
{
    Instance->CallbackToken = NULL;
    Instance->RxDataLength = 0U;
    Instance->RxDataOffset = 0U;
    Instance->SequenceNumber = 0U;
    Instance->BlockCounter = 0U;
    Instance->LastProgressCallback = 0U;
    Instance->ActiveRxDL = ISOTP_CLASSIC_CAN_DL;
    Instance->LengthFormat = CCX_ISOTP_LENGTH_FORMAT_STANDARD;
}

static void ISOTP_FinishTXError(CCX_ISOTP_TX_t *Instance, CCX_ISOTP_Status_t Error)
{
    Instance->State = CCX_ISOTP_TX_STATE_IDLE;
    ISOTP_ResetTXTransfer(Instance);
    if (Instance->Config.OnError != NULL)
    {
        Instance->Config.OnError(Instance, Error, Instance->Config.UserData);
    }
}

static uint8_t ISOTP_FinishRXError(CCX_ISOTP_RX_t *Instance, CCX_ISOTP_Status_t Error)
{
    uint8_t callback_token;
    Instance->State = CCX_ISOTP_RX_STATE_IDLE;
    ISOTP_ResetRXTransfer(Instance);
    if (Instance->Config.OnError != NULL)
    {
        /* cppcheck-suppress autoVariables
         * Compared only during this callback, never dereferenced. Init/Abort invalidates
         * the identity; otherwise it is cleared below before the local goes out of scope.
         */
        Instance->CallbackToken = &callback_token;
        Instance->Config.OnError(Instance, Error, Instance->Config.UserData);
        if (Instance->CallbackToken != &callback_token)
        {
            return 0U;
        }
        Instance->CallbackToken = NULL;
    }
    return 1U;
}

static uint8_t ISOTP_NotifyRXProgress(CCX_ISOTP_RX_t *Instance, CCX_ISOTP_Length_t Delta)
{
    uint8_t callback_token;
    Instance->LastProgressCallback = Instance->RxDataOffset;
    /* cppcheck-suppress autoVariables
     * Scoped callback identity, cleared or invalidated before this function returns.
     */
    Instance->CallbackToken = &callback_token;
    Instance->Config.OnReceiveProgress(Instance, Delta, Instance->RxDataLength, Instance->Config.UserData);
    if (Instance->CallbackToken != &callback_token)
    {
        return 0U;
    }
    Instance->CallbackToken = NULL;
    return 1U;
}

static uint8_t ISOTP_IsValidAddress(uint32_t ID, uint8_t IDE_flag)
{
    return (uint8_t)(IDE_flag <= 1U && ID <= (IDE_flag ? 0x1FFFFFFFUL : 0x7FFUL));
}

static uint8_t ISOTP_IsValidCANFrame(const CCX_message_t *Msg)
{
#if CCX_ENABLE_CANFD
    if (Msg->FrameFormat > CCX_FRAME_FORMAT_FD_BRS)
    {
        return 0U;
    }
    if (Msg->FrameFormat != CCX_FRAME_FORMAT_CLASSIC)
    {
        return 1U;
    }
#endif
    return (uint8_t)(Msg->DLC <= ISOTP_CLASSIC_CAN_DL);
}

void CCX_ISOTP_TX_Config_Init(CCX_ISOTP_TX_Config_t *Config, uint32_t TxID, uint8_t IDE_TxID, CCX_TIME_t N_Bs,
                              CCX_TIME_t N_Cs, CCX_ISOTP_Padding_t Padding, void *UserData,
                              void (*OnTransmitComplete)(CCX_ISOTP_TX_t *Instance, void *UserData),
                              void (*OnError)(CCX_ISOTP_TX_t *Instance, CCX_ISOTP_Status_t Error, void *UserData))
{
    if (NULL == Config)
    {
        return;
    }

    memset(Config, 0, sizeof(CCX_ISOTP_TX_Config_t));
    Config->TxID = TxID;
    Config->IDE_TxID = IDE_TxID;
#if CCX_ENABLE_CANFD
    Config->FrameFormat = CCX_FRAME_FORMAT_CLASSIC;
    Config->TxDL = 0U;
#endif
    Config->N_Bs = N_Bs;
    Config->N_Cs = N_Cs;
    Config->MaxWaitFrames = 0U;
    Config->Padding = Padding;
    Config->UserData = UserData;
    Config->OnTransmitComplete = OnTransmitComplete;
    Config->OnError = OnError;
}

#if CCX_ENABLE_CANFD
void CCX_ISOTP_TX_Config_InitFD(CCX_ISOTP_TX_Config_t *Config, uint32_t TxID, uint8_t IDE_TxID,
                                CCX_frame_format_t FrameFormat, uint8_t TxDL, CCX_TIME_t N_Bs, CCX_TIME_t N_Cs,
                                CCX_ISOTP_Padding_t Padding, void *UserData,
                                void (*OnTransmitComplete)(CCX_ISOTP_TX_t *Instance, void *UserData),
                                void (*OnError)(CCX_ISOTP_TX_t *Instance, CCX_ISOTP_Status_t Error, void *UserData))
{
    CCX_ISOTP_TX_Config_Init(Config, TxID, IDE_TxID, N_Bs, N_Cs, Padding, UserData, OnTransmitComplete, OnError);
    if (NULL == Config)
    {
        return;
    }

    Config->FrameFormat = FrameFormat;
    Config->TxDL = TxDL;
}
#endif

CCX_ISOTP_Status_t CCX_ISOTP_TX_Init(CCX_ISOTP_TX_t *Instance, CCX_instance_t *CanInstance,
                                     const CCX_ISOTP_TX_Config_t *Config)
{
    CCX_ISOTP_TX_Config_t config = {0};
    if (Instance == NULL)
    {
        return CCX_ISOTP_ERROR_NULL_PTR;
    }
    if (Config != NULL)
    {
        config = *Config;
    }
    memset(Instance, 0, sizeof(*Instance));
    if (CanInstance == NULL || Config == NULL)
    {
        return CCX_ISOTP_ERROR_NULL_PTR;
    }
    if (!CCX_IsPrimaryTickRegistered()
#ifndef CCX_DISABLE_HIGH_RES_TIMEBASE
        || !CCX_IsHighResTickRegistered()
#endif
    )
    {
        return CCX_ISOTP_ERROR_MISSING_TIMEBASE;
    }
    if (!ISOTP_IsValidAddress(config.TxID, config.IDE_TxID))
    {
        return CCX_ISOTP_ERROR_INVALID_ARG;
    }
#if CCX_ENABLE_CANFD
    if (config.FrameFormat > CCX_FRAME_FORMAT_FD_BRS ||
        !ISOTP_IsValidTxDL(ISOTP_GetConfiguredTxDL(config.FrameFormat, config.TxDL)))
    {
        return CCX_ISOTP_ERROR_INVALID_ARG;
    }
#endif
    Instance->Config = config;
    Instance->CanInstance = CanInstance;
    Instance->MaxWaitFrames = ISOTP_GetConfiguredMaxWaitFrames(config.MaxWaitFrames);
    Instance->InitValid = 1U;
    ISOTP_ResetTXTransfer(Instance);
    return CCX_ISOTP_OK;
}

CCX_ISOTP_Status_t CCX_ISOTP_TX_Abort(CCX_ISOTP_TX_t *Instance)
{
    uint8_t was_active;

    if (NULL == Instance)
    {
        return CCX_ISOTP_ERROR_NULL_PTR;
    }

    if (!Instance->InitValid)
    {
        return CCX_ISOTP_ERROR_INVALID_ARG;
    }
    was_active = (uint8_t)(Instance->State != CCX_ISOTP_TX_STATE_IDLE);
    Instance->State = CCX_ISOTP_TX_STATE_IDLE;
    ISOTP_ResetTXTransfer(Instance);

    if (was_active && Instance->Config.OnError != NULL)
    {
        Instance->Config.OnError(Instance, CCX_ISOTP_ERROR_ABORTED, Instance->Config.UserData);
    }

    return CCX_ISOTP_OK;
}

CCX_ISOTP_Status_t CCX_ISOTP_Transmit(CCX_ISOTP_TX_t *Instance, const uint8_t *Data, CCX_ISOTP_Length_t Length)
{
    uint8_t frame[ISOTP_CAN_FRAME_MAX_PAYLOAD] = {0};
    uint8_t frame_format;
    uint8_t tx_dl;
    uint8_t single_frame_max;
    uint8_t used_len;
    uint8_t target_can_dl;
    uint8_t first_frame_data_capacity;

    if (NULL == Instance || NULL == Data)
    {
        return CCX_ISOTP_ERROR_NULL_PTR;
    }

    if (!Instance->InitValid)
    {
        return CCX_ISOTP_ERROR_INVALID_ARG;
    }

    if (Instance->State != CCX_ISOTP_TX_STATE_IDLE)
    {
        return CCX_ISOTP_ERROR_BUSY;
    }

#if CCX_ENABLE_CANFD
    frame_format = Instance->Config.FrameFormat;
    tx_dl = ISOTP_GetConfiguredTxDL(frame_format, Instance->Config.TxDL);
#else
    frame_format = CCX_FRAME_FORMAT_CLASSIC;
    tx_dl = ISOTP_CLASSIC_CAN_DL;
#endif

    if (Length == 0U)
    {
        return CCX_ISOTP_ERROR_INVALID_ARG;
    }

#if CCX_ENABLE_CANFD
    if (ISOTP_IsFDFrameFormat(frame_format))
    {
        if (Length > (CCX_ISOTP_Length_t)CCX_ISOTP_MAX_FD_DATA_SIZE)
        {
            return CCX_ISOTP_ERROR_INVALID_ARG;
        }
    }
    else
#endif
    {
        if (Length > (CCX_ISOTP_Length_t)CCX_ISOTP_MAX_CLASSIC_DATA_SIZE)
        {
            return CCX_ISOTP_ERROR_INVALID_ARG;
        }
    }

    single_frame_max = ISOTP_GetSingleFrameMaxPayload(frame_format, tx_dl);

    Instance->TxData = Data;
    Instance->TxDataLength = Length;
    Instance->TxDataOffset = 0U;
    Instance->SequenceNumber = 0U;
    Instance->BlockCounter = 0U;
    Instance->WaitFramesRemaining = Instance->MaxWaitFrames;
    Instance->ActiveTxDL = tx_dl;
    Instance->LengthFormat = CCX_ISOTP_LENGTH_FORMAT_STANDARD;

    if (Length <= (CCX_ISOTP_Length_t)single_frame_max)
    {
        if (ISOTP_IsExtendedSingleFrame(frame_format, tx_dl, Length) ||
            (ISOTP_IsFDFrameFormat(frame_format) && tx_dl > ISOTP_CLASSIC_CAN_DL && Instance->Config.Padding.Enable))
        {
            frame[0] = ISOTP_EXTENDED_SF_ESCAPE;
            frame[1] = (uint8_t)Length;
            memcpy(&frame[ISOTP_EXTENDED_SF_HEADER_SIZE], Data, (size_t)Length);
            used_len = (uint8_t)(ISOTP_EXTENDED_SF_HEADER_SIZE + (uint8_t)Length);
        }
        else
        {
            frame[0] = (uint8_t)(CCX_ISOTP_PCI_SF | (uint8_t)Length);
            memcpy(&frame[ISOTP_STANDARD_SF_HEADER_SIZE], Data, (size_t)Length);
            used_len = (uint8_t)(ISOTP_STANDARD_SF_HEADER_SIZE + (uint8_t)Length);
        }

        target_can_dl = ISOTP_GetTransmitCanDL(frame_format, tx_dl, used_len, &Instance->Config.Padding, 0U);
        if (target_can_dl == ISOTP_INVALID_CAN_DL || target_can_dl > tx_dl)
        {
            ISOTP_ResetTXTransfer(Instance);
            return CCX_ISOTP_ERROR_INVALID_ARG;
        }

        CCX_Status_t can_status =
            ISOTP_SendCANMessage(Instance->CanInstance, Instance->Config.TxID, Instance->Config.IDE_TxID, frame,
                                 used_len, target_can_dl, &Instance->Config.Padding
#if CCX_ENABLE_CANFD
                                 ,
                                 frame_format
#endif
            );
        if (can_status != CCX_OK)
        {
            return ISOTP_AbortTXOnCANQueueError(Instance, can_status);
        }

        Instance->State = CCX_ISOTP_TX_STATE_IDLE;
        ISOTP_ResetTXTransfer(Instance);
        if (Instance->Config.OnTransmitComplete != NULL)
        {
            Instance->Config.OnTransmitComplete(Instance, Instance->Config.UserData);
        }
        return CCX_ISOTP_OK;
    }

    Instance->LengthFormat = ISOTP_SelectLengthFormat(Length);
    first_frame_data_capacity = ISOTP_GetFirstFrameDataCapacity(tx_dl, Instance->LengthFormat);
    if (first_frame_data_capacity == 0U || Length <= (CCX_ISOTP_Length_t)first_frame_data_capacity)
    {
        ISOTP_ResetTXTransfer(Instance);
        return CCX_ISOTP_ERROR_INVALID_ARG;
    }

    if (Instance->LengthFormat == CCX_ISOTP_LENGTH_FORMAT_EXTENDED)
    {
#if CCX_ENABLE_CANFD
        frame[0] = CCX_ISOTP_PCI_FF;
        frame[1] = ISOTP_EXTENDED_FF_ESCAPE;
        frame[2] = (uint8_t)((Length >> 24) & 0xFFU);
        frame[3] = (uint8_t)((Length >> 16) & 0xFFU);
        frame[4] = (uint8_t)((Length >> 8) & 0xFFU);
        frame[5] = (uint8_t)(Length & 0xFFU);
#else
        ISOTP_ResetTXTransfer(Instance);
        return CCX_ISOTP_ERROR_INVALID_ARG;
#endif
    }
    else
    {
        frame[0] = (uint8_t)(CCX_ISOTP_PCI_FF | (uint8_t)((Length >> 8) & ISOTP_PCI_VALUE_MASK));
        frame[1] = (uint8_t)(Length & 0xFFU);
    }

    memcpy(&frame[ISOTP_GetFirstFrameHeaderSize(Instance->LengthFormat)], Data, (size_t)first_frame_data_capacity);
    used_len = (uint8_t)(ISOTP_GetFirstFrameHeaderSize(Instance->LengthFormat) + first_frame_data_capacity);

    CCX_Status_t can_status =
        ISOTP_SendCANMessage(Instance->CanInstance, Instance->Config.TxID, Instance->Config.IDE_TxID, frame, used_len,
                             tx_dl, &Instance->Config.Padding
#if CCX_ENABLE_CANFD
                             ,
                             frame_format
#endif
        );
    if (can_status != CCX_OK)
    {
        return ISOTP_AbortTXOnCANQueueError(Instance, can_status);
    }

    Instance->TxDataOffset = first_frame_data_capacity;
    Instance->SequenceNumber = 1U;
    Instance->State = CCX_ISOTP_TX_STATE_WAIT_FC;
    Instance->LastTick = CCX_GetPrimaryTick();
    Instance->LastHighResTick = CCX_GetHighResTick();

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

    if (ISOTP_GetMessageDataLength(msg) < ISOTP_FC_FRAME_DATA_SIZE)
    {
        ISOTP_FinishTXError(Instance, CCX_ISOTP_ERROR_INVALID_ARG);
        return;
    }

    switch (ISOTP_GetFCFlowStatus(msg))
    {
    case CCX_ISOTP_FC_CTS:
        Instance->BlockCounter = ISOTP_GetFCBlockSize(msg);
        ISOTP_SetSTminRuntime(Instance, ISOTP_GetFCSTmin(msg));
        Instance->State = CCX_ISOTP_TX_STATE_SENDING_CF;
        Instance->LastTick = CCX_GetPrimaryTick();
        Instance->LastHighResTick = CCX_GetHighResTick();
        Instance->WaitFramesRemaining = Instance->MaxWaitFrames;
        break;

    case CCX_ISOTP_FC_WAIT:
        Instance->LastTick = CCX_GetPrimaryTick();
        if (Instance->WaitFramesRemaining > 0U)
        {
            Instance->WaitFramesRemaining--;
        }
        if (Instance->WaitFramesRemaining == 0U)
        {
            ISOTP_FinishTXError(Instance, CCX_ISOTP_ERROR_WAIT_EXCEEDED);
        }
        break;

    case CCX_ISOTP_FC_OVFLW:
        ISOTP_FinishTXError(Instance, CCX_ISOTP_ERROR_OVERFLOW);
        break;

    default:
        ISOTP_FinishTXError(Instance, CCX_ISOTP_ERROR_SEQUENCE);
        break;
    }
}

static inline void CCX_ISOTP_TX_SendConsecutiveFrame(CCX_ISOTP_TX_t *Instance)
{
    uint8_t frame[ISOTP_CAN_FRAME_MAX_PAYLOAD] = {0};
    CCX_ISOTP_Length_t remaining;
    uint8_t data_capacity;
    uint8_t data_len;
    uint8_t used_len;
    uint8_t target_can_dl;
    uint8_t frame_format;

    assert(Instance != NULL);

    remaining = Instance->TxDataLength - Instance->TxDataOffset;
    data_capacity = ISOTP_GetConsecutiveFrameDataCapacity(Instance->ActiveTxDL);
    data_len = (remaining > (CCX_ISOTP_Length_t)data_capacity) ? data_capacity : (uint8_t)remaining;

    if (Instance->TxData == NULL)
    {
        ISOTP_FinishTXError(Instance, CCX_ISOTP_ERROR_INVALID_ARG);
        return;
    }

#if CCX_ENABLE_CANFD
    frame_format = Instance->Config.FrameFormat;
#else
    frame_format = CCX_FRAME_FORMAT_CLASSIC;
#endif

    frame[0] = (uint8_t)(CCX_ISOTP_PCI_CF | (Instance->SequenceNumber & ISOTP_SN_MASK));
    memcpy(&frame[ISOTP_CF_HEADER_SIZE], &Instance->TxData[Instance->TxDataOffset], data_len);

    used_len = (uint8_t)(ISOTP_CF_HEADER_SIZE + data_len);
    target_can_dl = ISOTP_GetTransmitCanDL(frame_format, Instance->ActiveTxDL, used_len, &Instance->Config.Padding,
                                           (uint8_t)(data_len == data_capacity));
    if (target_can_dl == ISOTP_INVALID_CAN_DL)
    {
        ISOTP_FinishTXError(Instance, CCX_ISOTP_ERROR_INVALID_ARG);
        return;
    }

    CCX_Status_t can_status =
        ISOTP_SendCANMessage(Instance->CanInstance, Instance->Config.TxID, Instance->Config.IDE_TxID, frame, used_len,
                             target_can_dl, &Instance->Config.Padding
#if CCX_ENABLE_CANFD
                             ,
                             frame_format
#endif
        );
    if (can_status != CCX_OK)
    {
        (void)ISOTP_AbortTXOnCANQueueError(Instance, can_status);
        return;
    }

    Instance->TxDataOffset += data_len;
    Instance->SequenceNumber = (uint8_t)((Instance->SequenceNumber + 1U) & ISOTP_SN_MASK);
    Instance->LastTick = CCX_GetPrimaryTick();
    Instance->LastHighResTick = CCX_GetHighResTick();

    if (Instance->TxDataOffset >= Instance->TxDataLength)
    {
        Instance->State = CCX_ISOTP_TX_STATE_IDLE;
        ISOTP_ResetTXTransfer(Instance);
        if (Instance->Config.OnTransmitComplete != NULL)
        {
            Instance->Config.OnTransmitComplete(Instance, Instance->Config.UserData);
        }
        return;
    }

    if (Instance->BlockCounter > 0U)
    {
        Instance->BlockCounter--;
        if (Instance->BlockCounter == 0U)
        {
            Instance->State = CCX_ISOTP_TX_STATE_WAIT_FC;
        }
    }
}

void CCX_ISOTP_TX_Poll(CCX_ISOTP_TX_t *Instance)
{
    CCX_TIME_t current_tick;

    if (NULL == Instance)
    {
        return;
    }

    if (!Instance->InitValid)
    {
        return;
    }

    current_tick = CCX_GetPrimaryTick();

    switch (Instance->State)
    {
    case CCX_ISOTP_TX_STATE_IDLE:
        break;

    case CCX_ISOTP_TX_STATE_WAIT_FC: {
        CCX_TIME_t base_delta = (CCX_TIME_t)(current_tick - Instance->LastTick);
        if (Instance->Config.N_Bs > 0U && base_delta >= Instance->Config.N_Bs)
        {
            ISOTP_FinishTXError(Instance, CCX_ISOTP_ERROR_TIMEOUT_FC);
        }
        break;
    }

    case CCX_ISOTP_TX_STATE_SENDING_CF: {
        CCX_TIME_t base_delta = (CCX_TIME_t)(current_tick - Instance->LastTick);
        if (Instance->Config.N_Cs > 0U && base_delta > Instance->Config.N_Cs)
        {
            ISOTP_FinishTXError(Instance, CCX_ISOTP_ERROR_TIMEOUT_CF_TX);
            break;
        }

        if (Instance->STminUsesHighRes)
        {
            CCX_HR_TIME_t current_high_res_tick = CCX_GetHighResTick();
            CCX_HR_TIME_t hr_delta = (CCX_HR_TIME_t)(current_high_res_tick - Instance->LastHighResTick);
            if (hr_delta >= Instance->STminHighResTicks)
            {
                CCX_ISOTP_TX_SendConsecutiveFrame(Instance);
            }
        }
        else if (base_delta >= Instance->STminTicks)
        {
            CCX_ISOTP_TX_SendConsecutiveFrame(Instance);
        }
        break;
    }

    default:
        break;
    }
}

void CCX_ISOTP_RX_Config_Init(CCX_ISOTP_RX_Config_t *Config, uint32_t TxID, uint8_t IDE_TxID, uint8_t BS, uint8_t STmin,
                              CCX_TIME_t N_Cr, uint8_t *RxBuffer, CCX_ISOTP_Length_t RxBufferSize,
                              CCX_ISOTP_Padding_t Padding, CCX_ISOTP_Length_t ProgressCallbackInterval, void *UserData,
                              void (*OnReceiveStart)(CCX_ISOTP_RX_t *Instance, CCX_ISOTP_Length_t TotalLength,
                                                     void *UserData),
                              void (*OnReceiveComplete)(CCX_ISOTP_RX_t *Instance, const uint8_t *Data,
                                                        CCX_ISOTP_Length_t Length, void *UserData),
                              void (*OnReceiveProgress)(CCX_ISOTP_RX_t *Instance, CCX_ISOTP_Length_t BytesReceived,
                                                        CCX_ISOTP_Length_t TotalLength, void *UserData),
                              void (*OnError)(CCX_ISOTP_RX_t *Instance, CCX_ISOTP_Status_t Error, void *UserData))
{
    if (NULL == Config)
    {
        return;
    }

    memset(Config, 0, sizeof(CCX_ISOTP_RX_Config_t));
    Config->TxID = TxID;
    Config->IDE_TxID = IDE_TxID;
#if CCX_ENABLE_CANFD
    Config->FrameFormat = CCX_FRAME_FORMAT_CLASSIC;
    Config->FC_TxDL = 0U;
#endif
    Config->BS = BS;
    Config->STmin = STmin;
    Config->N_Cr = N_Cr;
    Config->Padding = Padding;
    Config->RxBuffer = RxBuffer;
    Config->RxBufferSize = RxBufferSize;
    Config->ProgressCallbackInterval = ProgressCallbackInterval;
    Config->UserData = UserData;
    Config->OnReceiveStart = OnReceiveStart;
    Config->OnReceiveComplete = OnReceiveComplete;
    Config->OnReceiveProgress = OnReceiveProgress;
    Config->OnError = OnError;
}

#if CCX_ENABLE_CANFD
void CCX_ISOTP_RX_Config_InitFD(
    CCX_ISOTP_RX_Config_t *Config, uint32_t TxID, uint8_t IDE_TxID, CCX_frame_format_t FrameFormat, uint8_t FC_TxDL,
    uint8_t BS, uint8_t STmin, CCX_TIME_t N_Cr, uint8_t *RxBuffer, CCX_ISOTP_Length_t RxBufferSize,
    CCX_ISOTP_Padding_t Padding, CCX_ISOTP_Length_t ProgressCallbackInterval, void *UserData,
    void (*OnReceiveStart)(CCX_ISOTP_RX_t *Instance, CCX_ISOTP_Length_t TotalLength, void *UserData),
    void (*OnReceiveComplete)(CCX_ISOTP_RX_t *Instance, const uint8_t *Data, CCX_ISOTP_Length_t Length, void *UserData),
    void (*OnReceiveProgress)(CCX_ISOTP_RX_t *Instance, CCX_ISOTP_Length_t BytesReceived,
                              CCX_ISOTP_Length_t TotalLength, void *UserData),
    void (*OnError)(CCX_ISOTP_RX_t *Instance, CCX_ISOTP_Status_t Error, void *UserData))
{
    CCX_ISOTP_RX_Config_Init(Config, TxID, IDE_TxID, BS, STmin, N_Cr, RxBuffer, RxBufferSize, Padding,
                             ProgressCallbackInterval, UserData, OnReceiveStart, OnReceiveComplete, OnReceiveProgress,
                             OnError);
    if (NULL == Config)
    {
        return;
    }

    Config->FrameFormat = FrameFormat;
    Config->FC_TxDL = FC_TxDL;
}
#endif

CCX_ISOTP_Status_t CCX_ISOTP_RX_Init(CCX_ISOTP_RX_t *Instance, CCX_instance_t *CanInstance,
                                     const CCX_ISOTP_RX_Config_t *Config)
{
    CCX_ISOTP_RX_Config_t config = {0};
    if (Instance == NULL)
    {
        return CCX_ISOTP_ERROR_NULL_PTR;
    }
    if (Config != NULL)
    {
        config = *Config;
    }
    memset(Instance, 0, sizeof(*Instance));
    if (CanInstance == NULL || Config == NULL || config.RxBuffer == NULL)
    {
        return CCX_ISOTP_ERROR_NULL_PTR;
    }
    if (config.RxBufferSize == 0U || !ISOTP_IsValidAddress(config.TxID, config.IDE_TxID))
    {
        return CCX_ISOTP_ERROR_INVALID_ARG;
    }
    if (!CCX_IsPrimaryTickRegistered())
    {
        return CCX_ISOTP_ERROR_MISSING_TIMEBASE;
    }
#if CCX_ENABLE_CANFD
    if (config.FrameFormat > CCX_FRAME_FORMAT_FD_BRS ||
        !ISOTP_IsValidTxDL(ISOTP_GetConfiguredTxDL(config.FrameFormat, config.FC_TxDL)))
    {
        return CCX_ISOTP_ERROR_INVALID_ARG;
    }
#endif
    Instance->Config = config;
    Instance->CanInstance = CanInstance;
    Instance->InitValid = 1U;
    ISOTP_ResetRXTransfer(Instance);
    return CCX_ISOTP_OK;
}

CCX_ISOTP_Status_t CCX_ISOTP_RX_Abort(CCX_ISOTP_RX_t *Instance)
{
    uint8_t was_active;

    if (NULL == Instance)
    {
        return CCX_ISOTP_ERROR_NULL_PTR;
    }

    if (!Instance->InitValid)
    {
        return CCX_ISOTP_ERROR_INVALID_ARG;
    }

    was_active = (uint8_t)(Instance->State != CCX_ISOTP_RX_STATE_IDLE);
    Instance->State = CCX_ISOTP_RX_STATE_IDLE;
    ISOTP_ResetRXTransfer(Instance);

    if (was_active && Instance->Config.OnError != NULL)
    {
        Instance->Config.OnError(Instance, CCX_ISOTP_ERROR_ABORTED, Instance->Config.UserData);
    }

    return CCX_ISOTP_OK;
}

static CCX_Status_t CCX_ISOTP_RX_SendFlowControl(CCX_ISOTP_RX_t *Instance, CCX_ISOTP_FC_FS_t FlowStatus)
{
    uint8_t frame[ISOTP_CAN_FRAME_MAX_PAYLOAD] = {0};
    uint8_t frame_format;
    uint8_t tx_dl;
#if CCX_ENABLE_CANFD
    frame_format = Instance->Config.FrameFormat;
    tx_dl = ISOTP_GetConfiguredTxDL(frame_format, Instance->Config.FC_TxDL);
#else
    frame_format = CCX_FRAME_FORMAT_CLASSIC;
    tx_dl = ISOTP_CLASSIC_CAN_DL;
#endif
    frame[0] = (uint8_t)(CCX_ISOTP_PCI_FC | (FlowStatus & ISOTP_PCI_VALUE_MASK));
    frame[1] = Instance->Config.BS;
    frame[2] = Instance->Config.STmin;
    uint8_t target_can_dl =
        ISOTP_GetTransmitCanDL(frame_format, tx_dl, ISOTP_FC_FRAME_DATA_SIZE, &Instance->Config.Padding, 0U);
    if (target_can_dl == ISOTP_INVALID_CAN_DL)
    {
        return CCX_WRONG_ARG;
    }
    return ISOTP_SendCANMessage(Instance->CanInstance, Instance->Config.TxID, Instance->Config.IDE_TxID, frame,
                                ISOTP_FC_FRAME_DATA_SIZE, target_can_dl, &Instance->Config.Padding
#if CCX_ENABLE_CANFD
                                ,
                                frame_format
#endif
    );
}

static void CCX_ISOTP_RX_HandleSingleFrame(CCX_ISOTP_RX_t *Instance, const CCX_message_t *msg)
{
    uint8_t can_dl = ISOTP_GetMessageDataLength(msg);
    uint8_t frame_format = ISOTP_GetMessageFrameFormat(msg);
    uint8_t header_size = ISOTP_STANDARD_SF_HEADER_SIZE;
    CCX_ISOTP_Length_t data_len;
    if (ISOTP_IsFDFrameFormat(frame_format) && can_dl > ISOTP_CLASSIC_CAN_DL)
    {
        if (msg->Data[0] != ISOTP_EXTENDED_SF_ESCAPE)
        {
            (void)ISOTP_FinishRXError(Instance, CCX_ISOTP_ERROR_INVALID_ARG);
            return;
        }
        header_size = ISOTP_EXTENDED_SF_HEADER_SIZE;
        data_len = ISOTP_GetExtendedSFDataLength(msg);
    }
    else
    {
        data_len = ISOTP_GetStandardSFDataLength(msg);
        if (data_len > ISOTP_CLASSIC_CAN_DL - ISOTP_STANDARD_SF_HEADER_SIZE)
        {
            (void)ISOTP_FinishRXError(Instance, CCX_ISOTP_ERROR_INVALID_ARG);
            return;
        }
    }
    if (can_dl < header_size || data_len == 0U || data_len > (CCX_ISOTP_Length_t)(can_dl - header_size))
    {
        (void)ISOTP_FinishRXError(Instance, CCX_ISOTP_ERROR_INVALID_ARG);
        return;
    }
    if (data_len > Instance->Config.RxBufferSize)
    {
        (void)ISOTP_FinishRXError(Instance, CCX_ISOTP_ERROR_BUFFER_TOO_SMALL);
        return;
    }
    if (Instance->State != CCX_ISOTP_RX_STATE_IDLE && !ISOTP_FinishRXError(Instance, CCX_ISOTP_ERROR_SEQUENCE))
    {
        return;
    }
    memcpy(Instance->Config.RxBuffer, &msg->Data[header_size], (size_t)data_len);
    Instance->State = CCX_ISOTP_RX_STATE_IDLE;
    ISOTP_ResetRXTransfer(Instance);
    if (Instance->Config.OnReceiveComplete != NULL)
    {
        Instance->Config.OnReceiveComplete(Instance, Instance->Config.RxBuffer, data_len, Instance->Config.UserData);
    }
}

static inline void CCX_ISOTP_RX_StartFirstFrame(CCX_ISOTP_RX_t *Instance, const CCX_message_t *msg,
                                                CCX_ISOTP_Length_t total_len, CCX_ISOTP_LengthFormat_t length_format)
{
    uint8_t can_dl = ISOTP_GetMessageDataLength(msg);
    uint8_t header_size = ISOTP_GetFirstFrameHeaderSize(length_format);
    uint8_t first_payload_bytes = ISOTP_GetFirstFrameDataCapacity(can_dl, length_format);

    if (Instance->State != CCX_ISOTP_RX_STATE_IDLE && !ISOTP_FinishRXError(Instance, CCX_ISOTP_ERROR_SEQUENCE))
    {
        return;
    }

    if (total_len > Instance->Config.RxBufferSize)
    {
        CCX_Status_t status = CCX_ISOTP_RX_SendFlowControl(Instance, CCX_ISOTP_FC_OVFLW);
        (void)ISOTP_FinishRXError(Instance, status == CCX_OK ? CCX_ISOTP_ERROR_BUFFER_TOO_SMALL : CCX_ISOTP_ERROR_BUSY);
        return;
    }

    Instance->CallbackToken = NULL;
    memcpy(Instance->Config.RxBuffer, &msg->Data[header_size], first_payload_bytes);

    Instance->RxDataLength = total_len;
    Instance->RxDataOffset = first_payload_bytes;
    Instance->SequenceNumber = 1U;
    Instance->BlockCounter = Instance->Config.BS;
    Instance->State = CCX_ISOTP_RX_STATE_RECEIVING_CF;
    Instance->LastTick = CCX_GetPrimaryTick();
    Instance->LastProgressCallback = 0U;
    Instance->ActiveRxDL = can_dl;
    Instance->LengthFormat = length_format;

    if (Instance->Config.OnReceiveStart != NULL)
    {
        uint8_t callback_token;
        /* cppcheck-suppress autoVariables
         * Scoped callback identity, cleared or invalidated before this function returns.
         */
        Instance->CallbackToken = &callback_token;
        Instance->Config.OnReceiveStart(Instance, total_len, Instance->Config.UserData);
        if (Instance->CallbackToken != &callback_token)
        {
            return;
        }
        Instance->CallbackToken = NULL;
    }
    if (CCX_ISOTP_RX_SendFlowControl(Instance, CCX_ISOTP_FC_CTS) != CCX_OK)
    {
        (void)ISOTP_FinishRXError(Instance, CCX_ISOTP_ERROR_BUSY);
    }
}

static inline void CCX_ISOTP_RX_HandleFirstFrame(CCX_ISOTP_RX_t *Instance, const CCX_message_t *msg)
{
    CCX_ISOTP_Length_t total_len;
    uint8_t can_dl;
    uint8_t frame_format;
    uint8_t sf_max;
    uint8_t standard_ff_payload;

    assert(Instance != NULL);
    assert(msg != NULL);

    frame_format = ISOTP_GetMessageFrameFormat(msg);
    can_dl = ISOTP_GetMessageDataLength(msg);
    sf_max = ISOTP_GetSingleFrameMaxPayload(frame_format, can_dl);
    if (can_dl < ISOTP_STANDARD_FF_HEADER_SIZE)
    {
        (void)ISOTP_FinishRXError(Instance, CCX_ISOTP_ERROR_INVALID_ARG);
        return;
    }
    total_len = ISOTP_GetStandardFFDataLength(msg);

    if (total_len == 0U)
    {
#if CCX_ENABLE_CANFD
        if (!ISOTP_IsFDFrameFormat(frame_format))
        {
            (void)ISOTP_FinishRXError(Instance, CCX_ISOTP_ERROR_INVALID_ARG);
            return;
        }

        if (can_dl < ISOTP_EXTENDED_FF_HEADER_SIZE)
        {
            (void)ISOTP_FinishRXError(Instance, CCX_ISOTP_ERROR_INVALID_ARG);
            return;
        }
        total_len = ISOTP_GetExtendedFFDataLength(msg);
        if (can_dl <= ISOTP_EXTENDED_FF_HEADER_SIZE || total_len <= (CCX_ISOTP_Length_t)ISOTP_STANDARD_FF_MAX_LENGTH ||
            total_len > (CCX_ISOTP_Length_t)CCX_ISOTP_MAX_FD_DATA_SIZE)
        {
            (void)ISOTP_FinishRXError(Instance, CCX_ISOTP_ERROR_INVALID_ARG);
            return;
        }

        CCX_ISOTP_RX_StartFirstFrame(Instance, msg, total_len, CCX_ISOTP_LENGTH_FORMAT_EXTENDED);
#else
        (void)ISOTP_FinishRXError(Instance, CCX_ISOTP_ERROR_INVALID_ARG);
#endif
        return;
    }

    standard_ff_payload = ISOTP_GetFirstFrameDataCapacity(can_dl, CCX_ISOTP_LENGTH_FORMAT_STANDARD);
    if (can_dl <= ISOTP_STANDARD_FF_HEADER_SIZE || total_len <= (CCX_ISOTP_Length_t)sf_max ||
        total_len <= (CCX_ISOTP_Length_t)standard_ff_payload ||
        total_len > (CCX_ISOTP_Length_t)CCX_ISOTP_MAX_CLASSIC_DATA_SIZE)
    {
        (void)ISOTP_FinishRXError(Instance, CCX_ISOTP_ERROR_INVALID_ARG);
        return;
    }

    CCX_ISOTP_RX_StartFirstFrame(Instance, msg, total_len, CCX_ISOTP_LENGTH_FORMAT_STANDARD);
}

static inline uint8_t ISOTP_IsValidConsecutiveFrameLength(const CCX_ISOTP_RX_t *Instance, const CCX_message_t *msg)
{
    CCX_ISOTP_Length_t remaining = Instance->RxDataLength - Instance->RxDataOffset;
    uint8_t can_dl = ISOTP_GetMessageDataLength(msg);
    uint8_t full_cf_data = ISOTP_GetConsecutiveFrameDataCapacity(Instance->ActiveRxDL);

    if (full_cf_data == 0U || can_dl <= ISOTP_CF_HEADER_SIZE)
    {
        return 0U;
    }

    if (remaining > (CCX_ISOTP_Length_t)full_cf_data)
    {
        return (uint8_t)(can_dl == Instance->ActiveRxDL);
    }

    return (uint8_t)(can_dl <= Instance->ActiveRxDL &&
                     (CCX_ISOTP_Length_t)(can_dl - ISOTP_CF_HEADER_SIZE) >= remaining);
}

static inline void CCX_ISOTP_RX_HandleConsecutiveFrame(CCX_ISOTP_RX_t *Instance, const CCX_message_t *msg)
{
    CCX_ISOTP_Length_t remaining;
    CCX_ISOTP_Length_t bytes_since_last;
    uint8_t sn;
    uint8_t data_len;
    uint8_t can_dl;

    assert(Instance != NULL);
    assert(msg != NULL);

    if (Instance->State != CCX_ISOTP_RX_STATE_RECEIVING_CF)
    {
        return;
    }

    sn = ISOTP_GetCFSequenceNumber(msg);
    if (sn != Instance->SequenceNumber)
    {
        (void)ISOTP_FinishRXError(Instance, CCX_ISOTP_ERROR_SEQUENCE);
        return;
    }

    if (!ISOTP_IsValidConsecutiveFrameLength(Instance, msg))
    {
        (void)ISOTP_FinishRXError(Instance, CCX_ISOTP_ERROR_INVALID_ARG);
        return;
    }

    remaining = Instance->RxDataLength - Instance->RxDataOffset;
    can_dl = ISOTP_GetMessageDataLength(msg);
    data_len = (remaining > (CCX_ISOTP_Length_t)ISOTP_GetConsecutiveFrameDataCapacity(can_dl))
                   ? ISOTP_GetConsecutiveFrameDataCapacity(can_dl)
                   : (uint8_t)remaining;

    memcpy(&Instance->Config.RxBuffer[Instance->RxDataOffset], &msg->Data[ISOTP_CF_HEADER_SIZE], data_len);

    Instance->RxDataOffset += data_len;
    Instance->SequenceNumber = (uint8_t)((Instance->SequenceNumber + 1U) & ISOTP_SN_MASK);
    Instance->LastTick = CCX_GetPrimaryTick();

    if (Instance->Config.ProgressCallbackInterval > 0U && Instance->Config.OnReceiveProgress != NULL)
    {
        bytes_since_last = Instance->RxDataOffset - Instance->LastProgressCallback;
        if (bytes_since_last >= Instance->Config.ProgressCallbackInterval)
        {
            if (!ISOTP_NotifyRXProgress(Instance, bytes_since_last))
            {
                return;
            }
        }
    }

    if (Instance->RxDataOffset >= Instance->RxDataLength)
    {
        if (Instance->Config.OnReceiveProgress != NULL)
        {
            bytes_since_last = Instance->RxDataOffset - Instance->LastProgressCallback;
            if (bytes_since_last > 0U)
            {
                if (!ISOTP_NotifyRXProgress(Instance, bytes_since_last))
                {
                    return;
                }
            }
        }

        CCX_ISOTP_Length_t completed_length = Instance->RxDataLength;
        Instance->State = CCX_ISOTP_RX_STATE_IDLE;
        ISOTP_ResetRXTransfer(Instance);
        if (Instance->Config.OnReceiveComplete != NULL)
        {
            Instance->Config.OnReceiveComplete(Instance, Instance->Config.RxBuffer, completed_length,
                                               Instance->Config.UserData);
        }
        return;
    }

    if (Instance->Config.BS > 0U)
    {
        Instance->BlockCounter--;
        if (Instance->BlockCounter == 0U)
        {
            Instance->BlockCounter = Instance->Config.BS;
            if (CCX_ISOTP_RX_SendFlowControl(Instance, CCX_ISOTP_FC_CTS) != CCX_OK)
            {
                (void)ISOTP_FinishRXError(Instance, CCX_ISOTP_ERROR_BUSY);
            }
        }
    }
}

/* cppcheck-suppress constParameterPointer
 * Public parser signature must match CAN CoreX RX callback ABI (`CCX_message_t *Msg`).
 */
void CCX_ISOTP_RX_Parser(const CCX_instance_t *CanInstance, CCX_message_t *Msg, uint16_t Slot, void *UserData)
{
    CCX_ISOTP_RX_t *Instance = (CCX_ISOTP_RX_t *)UserData;

    (void)CanInstance;
    (void)Slot;

    if (NULL == Instance || NULL == Msg)
    {
        return;
    }

    if (!Instance->InitValid)
    {
        return;
    }

    if (!ISOTP_IsValidReceiveFrameFormat(Instance, Msg))
    {
        return;
    }

    if (!ISOTP_IsValidCANFrame(Msg) || ISOTP_GetMessageDataLength(Msg) == 0U)
    {
        (void)ISOTP_FinishRXError(Instance, CCX_ISOTP_ERROR_INVALID_ARG);
        return;
    }
    switch (ISOTP_GetPCI(Msg))
    {
    case CCX_ISOTP_PCI_SF:
        CCX_ISOTP_RX_HandleSingleFrame(Instance, Msg);
        break;

    case CCX_ISOTP_PCI_FF:
        CCX_ISOTP_RX_HandleFirstFrame(Instance, Msg);
        break;

    case CCX_ISOTP_PCI_CF:
        CCX_ISOTP_RX_HandleConsecutiveFrame(Instance, Msg);
        break;

    default:
        break;
    }
}

void CCX_ISOTP_RX_Poll(CCX_ISOTP_RX_t *Instance)
{
    CCX_TIME_t current_tick;

    if (NULL == Instance)
    {
        return;
    }

    if (!Instance->InitValid)
    {
        return;
    }

    if (Instance->State == CCX_ISOTP_RX_STATE_IDLE)
    {
        return;
    }

    current_tick = CCX_GetPrimaryTick();
    if (Instance->Config.N_Cr > 0U && (CCX_TIME_t)(current_tick - Instance->LastTick) >= Instance->Config.N_Cr)
    {
        (void)ISOTP_FinishRXError(Instance, CCX_ISOTP_ERROR_TIMEOUT_CF_RX);
    }
}

/* cppcheck-suppress constParameterPointer
 * Public parser signature must match CAN CoreX RX callback ABI (`CCX_message_t *Msg`).
 */
void CCX_ISOTP_TX_FC_Parser(const CCX_instance_t *CanInstance, CCX_message_t *Msg, uint16_t Slot, void *UserData)
{
    CCX_ISOTP_TX_t *Instance = (CCX_ISOTP_TX_t *)UserData;

    (void)CanInstance;
    (void)Slot;

    if (NULL == Instance || NULL == Msg)
    {
        return;
    }

    if (!Instance->InitValid)
    {
        return;
    }

    if (!ISOTP_IsValidFlowControlFrameFormat(Instance, Msg))
    {
        return;
    }

    if (!ISOTP_IsValidCANFrame(Msg) || ISOTP_GetMessageDataLength(Msg) == 0U)
    {
        if (Instance->State != CCX_ISOTP_TX_STATE_IDLE)
        {
            ISOTP_FinishTXError(Instance, CCX_ISOTP_ERROR_INVALID_ARG);
        }
        return;
    }
    if (ISOTP_GetPCI(Msg) == CCX_ISOTP_PCI_FC)
    {
        CCX_ISOTP_TX_HandleFlowControl(Instance, Msg);
    }
}
