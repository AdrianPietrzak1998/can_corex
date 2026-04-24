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

static inline void CopyBuf(const uint8_t *restrict src, uint8_t *restrict dst, size_t size)
{
    assert((src != NULL) && (dst != NULL));

    for (size_t i = 0; i < size; i++)
    {
        dst[i] = src[i];
    }
}

CCX_Status_t CCX_RX_PushMsg(CCX_instance_t *Instance, const CCX_message_t *msg)
{
    if (NULL == Instance || NULL == msg)
    {
        return CCX_NULL_PTR;
    }
#if CCX_ENABLE_CANFD
    if (msg->FrameFormat == CCX_FRAME_FORMAT_CLASSIC && msg->DLC > 8)
    {
        return CCX_WRONG_ARG;
    }
    /* FD frames: DLC 0-15 all valid */
#else
    if (msg->DLC > 8)
    {
        return CCX_WRONG_ARG;
    }
#endif

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
    assert(Instance->RxTableSize < CCX_RX_HASH_SIZE);

    /* Clear hash table */
    for (uint16_t i = 0; i < CCX_RX_HASH_SIZE; i++)
    {
        Instance->RxHashTable[i] = HASH_EMPTY;
    }

    /* Build hash - linear probing for collisions */
    for (uint16_t i = 0; i < Instance->RxTableSize; i++)
    {
        uint16_t hash = CCX_Hash(Instance->CCX_RX_table[i].ID);

        /* Linear probing until we find an empty slot */
        while (Instance->RxHashTable[hash] != HASH_EMPTY)
        {
            hash = (hash + 1) % CCX_RX_HASH_SIZE;
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
    uint8_t msg_is_fd   = (Msg->FrameFormat != CCX_FRAME_FORMAT_CLASSIC);
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

    if (NULL == Instance->CCX_RX_table)
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

        if (Instance->CCX_RX_table[index].ID == Msg->ID)
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
        else if (Instance->CCX_RX_table[mid].ID > Msg->ID)
        {
            right = mid;
        }
        else
        {
            /* ID match - check DLC and IDE */
            return CCX_RX_ProcessMatch(Instance, Msg, mid);
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
            if ((0 != Instance->CCX_RX_table[i].TimeOut) &&
                (elapsed >= Instance->CCX_RX_table[i].TimeOut))
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
#if CCX_ENABLE_CANFD
    if (msg->FrameFormat == CCX_FRAME_FORMAT_CLASSIC && msg->DLC > 8)
    {
        return CCX_WRONG_ARG;
    }
    /* FD frames: DLC 0-15 all valid */
#else
    if (msg->DLC > 8)
    {
        return CCX_WRONG_ARG;
    }
#endif

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

    CCX_net_push(Instance, &Instance->TxBuf[next_head], 1);

    return CCX_OK;
}

static inline void CCX_TX_MsgFromTables(CCX_instance_t *Instance)
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
        CCX_TIME_t elapsed = (CCX_TIME_t)(CCX_GetPrimaryTick() - Instance->CCX_TX_table[i].LastTick);
        if (elapsed >= Instance->CCX_TX_table[i].SendFreq)
        {
            Instance->CCX_TX_table[i].LastTick = CCX_GetPrimaryTick();

#if CCX_ENABLE_CANFD
            CCX_frame_format_t table_fmt = Instance->CCX_TX_table[i].FrameFormat;
            uint8_t len = (table_fmt != CCX_FRAME_FORMAT_CLASSIC)
                              ? CCX_FD_DLC_TO_LEN[Instance->CCX_TX_table[i].DLC]
                              : Instance->CCX_TX_table[i].DLC;
#else
            uint8_t len = Instance->CCX_TX_table[i].DLC;
#endif
            CopyBuf(Instance->CCX_TX_table[i].Data, Tmp, len);
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

            CCX_TX_PushMsg(Instance, &msg);
        }
    }
}

static inline void CCX_TX_Poll(CCX_instance_t *Instance)
{
    assert((NULL != Instance) && (NULL != Instance->BusCheck));

    CCX_TX_MsgFromTables(Instance);

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
    CCX_TX_Poll(Instance);

    return CCX_OK;
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
    if (Instance->CCX_RX_table != NULL && RxTableSize > 0)
    {
        CCX_RX_BuildHash(Instance);
    }
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
    if (Instance != NULL && Instance->CCX_RX_table != NULL && Instance->RxTableSize > 0)
    {
        CCX_RX_BuildHash(Instance);
    }
#else
    (void)Instance; /* Suppress unused parameter warning */
#endif
}

