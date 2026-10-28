/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Author: Adrian Pietrzak
 * GitHub: https://github.com/AdrianPietrzak1998
 * Created: Aug 20, 2025
 */

#include "can_corex.h"
#include "assert.h"
#include "string.h"
#include <stddef.h>

/* Hash table configuration for CCX_RX_SEARCH_HASH mode */
#if defined(CCX_RX_SEARCH_HASH)
#define HASH_EMPTY 0xFFFF
#endif

#if CCX_TICK_FROM_FUNC

CCX_TIME_BASE_SCALAR (*CCX_get_tick)(void) = NULL;

void CCX_tick_function_register(CCX_TIME_BASE_SCALAR (*Function)(void))
{
    assert(Function != NULL);

    CCX_get_tick = Function;
}

#else

CCX_TIME_t *CCX_tick = NULL;

void CCX_tick_variable_register(CCX_TIME_t *Variable)
{
    assert(Variable != NULL);

    CCX_tick = Variable;
}

#endif

#ifndef CCX_DISABLE_HIGH_RES_TIMEBASE
#if CCX_HR_TICK_FROM_FUNC
CCX_HR_TIME_BASE_SCALAR (*CCX_get_high_res_tick)(void) = NULL;

void CCX_high_res_tick_function_register(CCX_HR_TIME_BASE_SCALAR (*Function)(void))
{
    assert(Function != NULL);

    CCX_get_high_res_tick = Function;
}
#else
CCX_HR_TIME_t *CCX_high_res_tick = NULL;

void CCX_high_res_tick_variable_register(CCX_HR_TIME_t *Variable)
{
    assert(Variable != NULL);

    CCX_high_res_tick = Variable;
}
#endif
#endif

uint8_t CCX_IsPrimaryTickRegistered(void)
{
#if CCX_TICK_FROM_FUNC
    return (uint8_t)(CCX_get_tick != NULL);
#else
    return (uint8_t)(CCX_tick != NULL);
#endif
}

uint8_t CCX_IsHighResTickRegistered(void)
{
#ifdef CCX_DISABLE_HIGH_RES_TIMEBASE
    return CCX_IsPrimaryTickRegistered();
#elif CCX_HR_TICK_FROM_FUNC
    return (uint8_t)(CCX_get_high_res_tick != NULL);
#else
    return (uint8_t)(CCX_high_res_tick != NULL);
#endif
}

extern void CCX_net_push(const CCX_instance_t *Instance, const CCX_message_t *msg, uint8_t FromTxFunc);
extern void CCX_BusMonitor_Update(CCX_instance_t *Instance);

#if CCX_ENABLE_CANFD
const uint8_t CCX_FD_DLC_TO_LEN[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

uint8_t CCX_FD_LenToDLC(uint8_t len)
{
    for (uint8_t i = 0; i < 16; i++)
    {
        if (CCX_FD_DLC_TO_LEN[i] >= len)
        {
            return i;
        }
    }
    return 15;
}
#endif

static uint8_t CCX_IsValidFrame(uint32_t ID, uint8_t IDE_flag, uint8_t DLC
#if CCX_ENABLE_CANFD
                                ,
                                uint8_t FrameFormat
#endif
)
{
    if (ID > (IDE_flag ? 0x1FFFFFFFUL : 0x7FFUL))
    {
        return 0U;
    }
#if CCX_ENABLE_CANFD
    if (FrameFormat > CCX_FRAME_FORMAT_FD_BRS)
    {
        return 0U;
    }
    return (uint8_t)(FrameFormat != CCX_FRAME_FORMAT_CLASSIC || DLC <= 8U);
#else
    return (uint8_t)(DLC <= 8U);
#endif
}

static CCX_Status_t CCX_ValidateTXEntry(const CCX_TX_table_t *Entry)
{
    if (!CCX_IsValidFrame(Entry->ID, Entry->IDE_flag, Entry->DLC
#if CCX_ENABLE_CANFD
                          ,
                          Entry->FrameFormat
#endif
                          ))
    {
        return CCX_WRONG_ARG;
    }
    return (Entry->DLC > 0U && Entry->Data == NULL) ? CCX_NULL_PTR : CCX_OK;
}

static uint16_t CCX_GetRingDepth(uint16_t head, uint16_t tail, uint16_t size)
{
    if (head >= tail)
    {
        return (uint16_t)(head - tail);
    }

    return (uint16_t)(size - tail + head);
}

static inline void CCX_UpdatePeakDepth(uint16_t depth, uint16_t *peak)
{
    if (depth > *peak)
    {
        *peak = depth;
    }
}

static inline void CCX_UpdateRxPeakDepth(CCX_instance_t *Instance)
{
    if (Instance == NULL)
    {
        return;
    }

    CCX_UpdatePeakDepth(CCX_GetRingDepth(Instance->RxHead, Instance->RxTail, CCX_RX_BUFFER_SIZE),
                        &Instance->GlobalStats.peak_rx_depth);
}

static inline void CCX_UpdateTxPeakDepth(CCX_instance_t *Instance)
{
    if (Instance == NULL)
    {
        return;
    }

    CCX_UpdatePeakDepth(CCX_GetRingDepth(Instance->TxHead, Instance->TxTail, CCX_TX_BUFFER_SIZE),
                        &Instance->GlobalStats.peak_tx_depth);
}

CCX_Status_t CCX_RX_PushMsg(CCX_instance_t *Instance, const CCX_message_t *msg)
{
    if (NULL == Instance || NULL == msg)
    {
        return CCX_NULL_PTR;
    }
    if (!CCX_IsValidFrame(msg->ID, msg->IDE_flag, msg->DLC
#if CCX_ENABLE_CANFD
                          ,
                          msg->FrameFormat
#endif
                          ))
    {
        return CCX_WRONG_ARG;
    }

    uint16_t next_head = Instance->RxHead + 1;
    if (next_head >= CCX_RX_BUFFER_SIZE)
    {
        next_head = 0;
    }

    if (next_head == Instance->RxTail)
    {
        Instance->GlobalStats.rx_buffer_overflows++; /* Track overflow (v1.3.0) */
        return CCX_BUS_TOO_BUSY;
    }

    /* Publish the new head only after the slot is fully written. */
    memcpy(&Instance->RxBuf[next_head], msg, sizeof(CCX_message_t));
    Instance->RxReceivedTick[next_head] = CCX_GetPrimaryTick();
    Instance->RxHead = next_head;
    CCX_UpdateRxPeakDepth(Instance);

    Instance->GlobalStats.total_rx_messages++; /* Increment RX counter (v1.3.0) */

    CCX_net_push(Instance, &Instance->RxBuf[Instance->RxHead], 0);

    return CCX_OK;
}

#if defined(CCX_RX_SEARCH_HASH)
/**
 * @brief Calculate hash value for CAN message ID
 *
 * Simple hash function using XOR and modulo.
 */
static inline uint16_t CCX_Hash(uint32_t ID)
{
    return (uint16_t)((ID ^ (ID >> 16)) % CCX_RX_HASH_SIZE);
}

/**
 * @brief Build hash table for RX message lookup
 *
 * Clears the hash table and rebuilds it from the current RX table.
 * Uses linear probing for collision resolution.
 */
static void CCX_RX_BuildHash(CCX_instance_t *Instance)
{
    assert(Instance != NULL);

    /* Clear hash table */
    for (uint16_t i = 0; i < CCX_RX_HASH_SIZE; i++)
    {
        Instance->RxHashTable[i] = HASH_EMPTY;
    }

    if (Instance->CCX_RX_table == NULL || Instance->RxTableSize > CCX_RX_HASH_SIZE)
    {
        return;
    }

    /* Build hash - linear probing for collisions */
    for (uint16_t i = 0; i < Instance->RxTableSize; i++)
    {
        uint16_t hash = CCX_Hash(Instance->CCX_RX_table[i].ID);

        uint16_t probes = 0U;
        while (Instance->RxHashTable[hash] != HASH_EMPTY && probes < CCX_RX_HASH_SIZE)
        {
            hash = (hash + 1) % CCX_RX_HASH_SIZE;
            probes++;
        }
        if (probes == CCX_RX_HASH_SIZE)
        {
            return;
        }

        Instance->RxHashTable[hash] = i;
    }
}
#endif

/**
 * @brief Helper function to process matched RX message
 */
static inline CCX_MsgRegStatus_t CCX_RX_ProcessMatch(CCX_instance_t *Instance, CCX_message_t *Msg, uint16_t index)
{
    uint8_t dlc_match = 0;
#if CCX_ENABLE_CANFD
    /* FD and FD_BRS both count as "FD" for format matching — entry FD matches any FD frame */
    uint8_t entry_is_fd = (Instance->CCX_RX_table[index].FrameFormat != CCX_FRAME_FORMAT_CLASSIC);
    uint8_t msg_is_fd = (Msg->FrameFormat != CCX_FRAME_FORMAT_CLASSIC);
    uint8_t format_match = (entry_is_fd == msg_is_fd);

    if (Instance->CCX_RX_table[index].DLC == CCX_DLC_ANY)
    {
        dlc_match = 1;
    }
    else
    {
        dlc_match = (Instance->CCX_RX_table[index].DLC == Msg->DLC);
    }

    if (dlc_match && format_match && (Instance->CCX_RX_table[index].IDE_flag == Msg->IDE_flag))
#else
    if (Instance->CCX_RX_table[index].DLC == CCX_DLC_ANY)
    {
        dlc_match = 1;
    }
    else
    {
        dlc_match = (Instance->CCX_RX_table[index].DLC == Msg->DLC);
    }

    if (dlc_match && (Instance->CCX_RX_table[index].IDE_flag == Msg->IDE_flag))
#endif
    {
        if (NULL != Instance->CCX_RX_table[index].Parser)
        {
            Instance->GlobalStats.parser_calls_count++; /* Track parser calls (v1.3.0) */
            Instance->CCX_RX_table[index].Parser(Instance, Msg, index, Instance->CCX_RX_table[index].UserData);
        }
        Instance->CCX_RX_table[index].LastTick = Instance->RxReceivedTick[Instance->RxTail];
        return CCX_MSG_REG;
    }

    return CCX_MSG_UNREG;
}

static inline CCX_MsgRegStatus_t CCX_RX_MsgFromTables(CCX_instance_t *Instance, CCX_message_t *Msg)
{
    assert(Instance != NULL);

    if (NULL == Instance->CCX_RX_table || Instance->RxTableSize == 0U)
    {
        return CCX_MSG_UNREG;
    }

#if defined(CCX_RX_SEARCH_HASH)
    /* Hash table search with linear probing */
    uint16_t hash = CCX_Hash(Msg->ID);
    uint16_t start_hash = hash;

    do
    {
        if (Instance->RxHashTable[hash] == HASH_EMPTY)
        {
            /* Empty slot - message not registered */
            return CCX_MSG_UNREG;
        }

        uint16_t index = Instance->RxHashTable[hash];

        if (index < Instance->RxTableSize && Instance->CCX_RX_table[index].ID == Msg->ID)
        {
            /* ID match - check DLC and IDE */
            CCX_MsgRegStatus_t result = CCX_RX_ProcessMatch(Instance, Msg, index);
            if (result == CCX_MSG_REG)
            {
                return CCX_MSG_REG;
            }
        }

        /* Try next slot (linear probing) */
        hash = (hash + 1) % CCX_RX_HASH_SIZE;

    } while (hash != start_hash);

    return CCX_MSG_UNREG;

#elif defined(CCX_RX_SEARCH_BINARY)
    /* Binary search - requires sorted RX table by ID */
    uint16_t left = 0;
    uint16_t right = Instance->RxTableSize;

    while (left < right)
    {
        uint16_t mid = left + (right - left) / 2;

        if (Instance->CCX_RX_table[mid].ID < Msg->ID)
        {
            left = mid + 1;
        }
        else
        {
            right = mid;
        }
    }

    /* Start at the first matching ID to preserve table-order precedence. */
    for (uint16_t i = left; i < Instance->RxTableSize && Instance->CCX_RX_table[i].ID == Msg->ID; i++)
    {
        if (CCX_RX_ProcessMatch(Instance, Msg, i) == CCX_MSG_REG)
        {
            return CCX_MSG_REG;
        }
    }

    return CCX_MSG_UNREG;

#else
    /* Linear search (default) */
    for (uint16_t i = 0; i < Instance->RxTableSize; i++)
    {
        if (Instance->CCX_RX_table[i].ID == Msg->ID)
        {
            /* ID match - check DLC and IDE */
            CCX_MsgRegStatus_t result = CCX_RX_ProcessMatch(Instance, Msg, i);
            if (result == CCX_MSG_REG)
            {
                return CCX_MSG_REG;
            }
        }
    }

    return CCX_MSG_UNREG;
#endif
}

static inline void CCX_Timeout_Check(CCX_instance_t *Instance)
{
    if (NULL != Instance->CCX_RX_table)
    {
        for (uint16_t i = 0; i < Instance->RxTableSize; i++)
        {
            CCX_TIME_t elapsed = (CCX_TIME_t)(CCX_GetPrimaryTick() - Instance->CCX_RX_table[i].LastTick);
            if ((0 != Instance->CCX_RX_table[i].TimeOut) && (elapsed >= Instance->CCX_RX_table[i].TimeOut))
            {
                Instance->CCX_RX_table[i].LastTick = CCX_GetPrimaryTick();
                if (NULL != Instance->CCX_RX_table[i].TimeoutCallback)
                {
                    Instance->GlobalStats.timeout_calls_count++; /* Track timeout calls (v1.3.0) */
                    Instance->CCX_RX_table[i].TimeoutCallback(Instance, i, Instance->CCX_RX_table[i].UserData);
                }
            }
        }
    }
}

static inline void CCX_RX_Poll(CCX_instance_t *Instance)
{
    assert(Instance != NULL);

    while (Instance->RxHead != Instance->RxTail)
    {
        Instance->RxTail++;
        if (Instance->RxTail >= CCX_RX_BUFFER_SIZE)
        {
            Instance->RxTail = 0;
        }

        if ((CCX_RX_MsgFromTables(Instance, &Instance->RxBuf[Instance->RxTail]) != CCX_MSG_REG) &&
            NULL != Instance->Parser_unreg_msg)
        {
            Instance->Parser_unreg_msg(Instance, &Instance->RxBuf[Instance->RxTail]);
        }
    }

    CCX_Timeout_Check(Instance);
}

CCX_Status_t CCX_TX_PushMsg(CCX_instance_t *Instance, const CCX_message_t *msg)
{
    if (NULL == Instance || NULL == msg)
    {
        return CCX_NULL_PTR;
    }
    if (!CCX_IsValidFrame(msg->ID, msg->IDE_flag, msg->DLC
#if CCX_ENABLE_CANFD
                          ,
                          msg->FrameFormat
#endif
                          ))
    {
        return CCX_WRONG_ARG;
    }
    if (Instance->SendFunction == NULL || Instance->BusCheck == NULL)
    {
        return CCX_NULL_PTR;
    }

    uint16_t next_head = Instance->TxHead + 1;
    if (next_head >= CCX_TX_BUFFER_SIZE)
    {
        next_head = 0;
    }

    if (next_head == Instance->TxTail)
    {
        Instance->GlobalStats.tx_buffer_overflows++; /* Track overflow (v1.3.0) */
        return CCX_BUS_TOO_BUSY;
    }

    /* Publish the new head only after the slot is fully written. */
    memcpy(&Instance->TxBuf[next_head], msg, sizeof(CCX_message_t));
    Instance->TxHead = next_head;
    CCX_UpdateTxPeakDepth(Instance);

    CCX_net_push(Instance, &Instance->TxBuf[next_head], 1);

    return CCX_OK;
}

uint16_t CCX_RX_GetDepth(const CCX_instance_t *Instance)
{
    if (Instance == NULL)
    {
        return 0U;
    }

    return CCX_GetRingDepth(Instance->RxHead, Instance->RxTail, CCX_RX_BUFFER_SIZE);
}

uint16_t CCX_TX_GetDepth(const CCX_instance_t *Instance)
{
    if (Instance == NULL)
    {
        return 0U;
    }

    return CCX_GetRingDepth(Instance->TxHead, Instance->TxTail, CCX_TX_BUFFER_SIZE);
}

uint16_t CCX_RX_GetFree(const CCX_instance_t *Instance)
{
    if (Instance == NULL)
    {
        return 0U;
    }

    return (uint16_t)((CCX_RX_BUFFER_SIZE - 1U) - CCX_RX_GetDepth(Instance));
}

uint16_t CCX_TX_GetFree(const CCX_instance_t *Instance)
{
    if (Instance == NULL)
    {
        return 0U;
    }

    return (uint16_t)((CCX_TX_BUFFER_SIZE - 1U) - CCX_TX_GetDepth(Instance));
}

void CCX_FlushRx(CCX_instance_t *Instance)
{
    if (Instance == NULL)
    {
        return;
    }

    Instance->RxHead = 0U;
    Instance->RxTail = 0U;
    for (uint16_t i = 0; i < CCX_RX_BUFFER_SIZE; i++)
    {
        Instance->RxReceivedTick[i] = 0U;
    }
}

void CCX_FlushTx(CCX_instance_t *Instance)
{
    if (Instance == NULL)
    {
        return;
    }

    Instance->TxHead = 0U;
    Instance->TxTail = 0U;
}

void CCX_Flush(CCX_instance_t *Instance)
{
    if (Instance == NULL)
    {
        return;
    }

    CCX_FlushRx(Instance);
    CCX_FlushTx(Instance);
}

void CCX_Reset(CCX_instance_t *Instance)
{
    if (Instance == NULL)
    {
        return;
    }

    CCX_Flush(Instance);
    CCX_ResetGlobalStats(Instance);
}

static CCX_Status_t CCX_TX_MsgFromTables(CCX_instance_t *Instance)
{
    assert(NULL != Instance);
#if CCX_ENABLE_CANFD
    uint8_t Tmp[64];
    memset(Tmp, 0x00, 64);
#else
    uint8_t Tmp[8];
    memset(Tmp, 0x00, 8);
#endif

    for (uint16_t i = 0; i < Instance->TxTableSize; i++)
    {
        CCX_Status_t status = CCX_ValidateTXEntry(&Instance->CCX_TX_table[i]);
        if (status != CCX_OK)
        {
            return status;
        }
        CCX_TIME_t elapsed = (CCX_TIME_t)(CCX_GetPrimaryTick() - Instance->CCX_TX_table[i].LastTick);
        if (elapsed >= Instance->CCX_TX_table[i].SendFreq)
        {
            CCX_TIME_t current_tick = CCX_GetPrimaryTick();

#if CCX_ENABLE_CANFD
            CCX_frame_format_t table_fmt = Instance->CCX_TX_table[i].FrameFormat;
            uint8_t len = (table_fmt != CCX_FRAME_FORMAT_CLASSIC) ? CCX_FD_DLC_TO_LEN[Instance->CCX_TX_table[i].DLC]
                                                                  : Instance->CCX_TX_table[i].DLC;
#else
            uint8_t len = Instance->CCX_TX_table[i].DLC;
#endif
            if (len > 0U)
            {
                memcpy(Tmp, Instance->CCX_TX_table[i].Data, len);
            }
            if (NULL != Instance->CCX_TX_table[i].Parser)
            {
                Instance->CCX_TX_table[i].Parser(Instance, Tmp, i, Instance->CCX_TX_table[i].UserData);
            }

#if CCX_ENABLE_CANFD
            CCX_message_t msg = {.ID = Instance->CCX_TX_table[i].ID,
                                 .DLC = Instance->CCX_TX_table[i].DLC,
                                 .IDE_flag = Instance->CCX_TX_table[i].IDE_flag,
                                 .FrameFormat = table_fmt};
#else
            CCX_message_t msg = {.ID = Instance->CCX_TX_table[i].ID,
                                 .DLC = Instance->CCX_TX_table[i].DLC,
                                 .IDE_flag = Instance->CCX_TX_table[i].IDE_flag};
#endif
            memcpy(msg.Data, Tmp, len);

            if (CCX_TX_PushMsg(Instance, &msg) == CCX_OK)
            {
                Instance->CCX_TX_table[i].LastTick = current_tick;
            }
        }
    }
    return CCX_OK;
}

static CCX_Status_t CCX_TX_Poll(CCX_instance_t *Instance)
{
    assert(Instance != NULL);

    if (Instance->BusCheck == NULL || Instance->SendFunction == NULL)
    {
        return (Instance->TxTableSize == 0U && Instance->TxHead == Instance->TxTail) ? CCX_OK : CCX_NULL_PTR;
    }

    CCX_Status_t status = CCX_TX_MsgFromTables(Instance);
    if (status != CCX_OK)
    {
        return status;
    }

    while ((Instance->TxHead != Instance->TxTail) && (Instance->BusCheck(Instance) == CCX_BUS_FREE))
    {
        Instance->TxTail++;
        if (Instance->TxTail >= CCX_TX_BUFFER_SIZE)
        {
            Instance->TxTail = 0;
        }

        assert(NULL != Instance->SendFunction);
        Instance->SendFunction(Instance, &Instance->TxBuf[Instance->TxTail]);
    }
    return CCX_OK;
}

CCX_Status_t CCX_Poll(CCX_instance_t *Instance)
{
    if (NULL == Instance)
    {
        return CCX_NULL_PTR;
    }

    /* Automatic bus monitoring update (v1.3.0) */
    if (Instance->BusMonitor != NULL)
    {
        CCX_BusMonitor_Update(Instance);
    }

    CCX_RX_Poll(Instance);
    return CCX_TX_Poll(Instance);
}

CCX_Status_t CCX_Init(CCX_instance_t *Instance, CCX_RX_table_t *CCX_RX_table, CCX_TX_table_t *CCX_TX_table,
                      uint16_t RxTableSize, uint16_t TxTableSize,
                      void (*SendFunction)(const CCX_instance_t *Instance, const CCX_message_t *msg),
                      CCX_BusIsFree_t (*BusCheck)(const CCX_instance_t *Instance),
                      void (*ParserUnregMsg)(const CCX_instance_t *Instance, CCX_message_t *Msg))
{
    /* asserts here */
    if ((NULL == CCX_RX_table && RxTableSize > 0) ||
        ((NULL == CCX_TX_table || NULL == SendFunction || NULL == BusCheck) && TxTableSize > 0) || NULL == Instance)
    {
        return CCX_NULL_PTR;
    }

    if (CCX_IsPrimaryTickRegistered() == 0U)
    {
        return CCX_MISSING_TIMEBASE;
    }

    if ((SendFunction == NULL) != (BusCheck == NULL))
    {
        return CCX_NULL_PTR;
    }
#if defined(CCX_RX_SEARCH_HASH)
    if (RxTableSize > CCX_RX_HASH_SIZE)
    {
        return CCX_WRONG_ARG;
    }
#endif
    for (uint16_t i = 0U; i < TxTableSize; i++)
    {
        CCX_Status_t status = CCX_ValidateTXEntry(&CCX_TX_table[i]);
        if (status != CCX_OK)
        {
            return status;
        }
    }
    for (uint16_t i = 0U; i < RxTableSize; i++)
    {
        uint8_t dlc = CCX_RX_table[i].DLC;
        if (!CCX_IsValidFrame(CCX_RX_table[i].ID, CCX_RX_table[i].IDE_flag, dlc == CCX_DLC_ANY ? 0U : dlc
#if CCX_ENABLE_CANFD
                              ,
                              CCX_RX_table[i].FrameFormat
#endif
                              )
#if CCX_ENABLE_CANFD
            || dlc > CCX_DLC_ANY
#endif
        )
        {
            return CCX_WRONG_ARG;
        }
#if defined(CCX_RX_SEARCH_BINARY)
        if (i > 0U && CCX_RX_table[i - 1U].ID > CCX_RX_table[i].ID)
        {
            return CCX_WRONG_ARG;
        }
#endif
    }

    Instance->CCX_TX_table = CCX_TX_table;
    Instance->CCX_RX_table = CCX_RX_table;
    Instance->RxTableSize = RxTableSize;
    Instance->TxTableSize = TxTableSize;
    Instance->SendFunction = SendFunction;
    Instance->BusCheck = BusCheck;
    Instance->Parser_unreg_msg = ParserUnregMsg;

    Instance->RxHead = 0;
    Instance->RxTail = 0;
    Instance->TxHead = 0;
    Instance->TxTail = 0;

    memset(Instance->RxBuf, 0, sizeof(Instance->RxBuf));
    memset(Instance->TxBuf, 0, sizeof(Instance->TxBuf));

    for (uint16_t i = 0; i < CCX_RX_BUFFER_SIZE; i++)
    {
        Instance->RxReceivedTick[i] = 0;
    }

    CCX_TIME_t current_tick = CCX_GetPrimaryTick();

    if (Instance->CCX_RX_table != NULL)
    {
        for (uint16_t i = 0; i < RxTableSize; i++)
        {
            Instance->CCX_RX_table[i].LastTick = current_tick;
        }
    }

    if (Instance->CCX_TX_table != NULL)
    {
        for (uint16_t i = 0; i < TxTableSize; i++)
        {
            Instance->CCX_TX_table[i].LastTick = current_tick;
        }
    }

    /* Initialize global statistics (v1.3.0) */
    memset(&Instance->GlobalStats, 0, sizeof(CCX_GlobalStats_t));

    /* Bus monitoring disabled by default (v1.3.0) */
    Instance->BusMonitor = NULL;
    Instance->OnMessageTransmitted = NULL;

#if defined(CCX_RX_SEARCH_HASH)
    /* Build hash table for RX messages */
    CCX_RX_BuildHash(Instance);
#endif

    return CCX_OK;
}

const CCX_GlobalStats_t *CCX_GetGlobalStats(const CCX_instance_t *Instance)
{
    if (Instance == NULL)
    {
        static const CCX_GlobalStats_t empty_stats = {0};
        return &empty_stats;
    }

    return &Instance->GlobalStats;
}

void CCX_ResetGlobalStats(CCX_instance_t *Instance)
{
    if (Instance == NULL)
    {
        return;
    }

    memset(&Instance->GlobalStats, 0, sizeof(CCX_GlobalStats_t));
}

void CCX_OnMessageTransmitted(CCX_instance_t *Instance, const CCX_message_t *msg)
{
    if (Instance == NULL)
    {
        return;
    }

    /* Automatic increment */
    Instance->GlobalStats.total_tx_messages++;

    /* Optional user callback for notification */
    if (Instance->OnMessageTransmitted != NULL)
    {
        Instance->OnMessageTransmitted(Instance, msg);
    }
}

void CCX_RX_RebuildHash(CCX_instance_t *Instance)
{
#if defined(CCX_RX_SEARCH_HASH)
    if (Instance != NULL)
    {
        CCX_RX_BuildHash(Instance);
    }
#else
    (void)Instance; /* Suppress unused parameter warning */
#endif
}
