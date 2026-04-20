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

/* Forward declaration for bus monitoring (v1.3.0) */
static void CCX_BusMonitor_Update(CCX_instance_t *Instance);

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

    Instance->RxHead = next_head;

    memcpy(&Instance->RxBuf[Instance->RxHead], msg, sizeof(CCX_message_t));
    Instance->RxReceivedTick[Instance->RxHead] = CCX_GetPrimaryTick();

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
            if ((0 != Instance->CCX_RX_table[i].TimeOut) &&
                (CCX_GetPrimaryTick() - Instance->CCX_RX_table[i].LastTick >= Instance->CCX_RX_table[i].TimeOut))
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

    Instance->TxHead = next_head;

    memcpy(&Instance->TxBuf[Instance->TxHead], msg, sizeof(CCX_message_t));

    CCX_net_push(Instance, &Instance->TxBuf[Instance->TxHead], 1);

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
        if (CCX_GetPrimaryTick() - Instance->CCX_TX_table[i].LastTick >= Instance->CCX_TX_table[i].SendFreq)
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

/* ========================================================================
 * BUS MONITORING IMPLEMENTATION (v1.3.0)
 * ======================================================================== */

/**
 * @brief Internal bus monitoring update function
 *
 * Called automatically by CCX_Poll() when bus monitoring is enabled.
 * Handles state transitions, recovery logic, and callbacks.
 */
static void CCX_BusMonitor_Update(CCX_instance_t *Instance)
{
    assert(Instance != NULL);
    assert(Instance->BusMonitor != NULL);

    CCX_BusMonitor_t *mon = Instance->BusMonitor;
    CCX_TIME_t current_tick = CCX_GetPrimaryTick();

    /* Get current bus state from hardware */
    CCX_BusState_t new_state = mon->GetBusState(Instance);
    CCX_BusState_t old_state = mon->current_state;

    /* Update error counters if available */
    if (mon->GetErrorCounters != NULL)
    {
        CCX_ErrorCounters_t counters;
        mon->GetErrorCounters(Instance, &counters);

        mon->stats.error_counters = counters;

        /* Track peak values */
        if (counters.TEC > mon->stats.peak_error_counters.TEC)
        {
            mon->stats.peak_error_counters.TEC = counters.TEC;
        }
        if (counters.REC > mon->stats.peak_error_counters.REC)
        {
            mon->stats.peak_error_counters.REC = counters.REC;
        }

        /* Callback for error counter updates */
        if (mon->OnErrorCountersUpdate != NULL)
        {
            mon->OnErrorCountersUpdate(Instance, &counters, mon->UserData);
        }
    }

    /* State change detection */
    if (new_state != old_state)
    {
        mon->current_state = new_state;

        /* Track state-specific events */
        if (new_state == CCX_BUS_STATE_OFF)
        {
            mon->stats.bus_off_count++;
            mon->bus_off_entry_time = current_tick;
            mon->recovery_start_time = current_tick;
            mon->recovery_start_time_hr = CCX_GetHighResTick();
            mon->stats.last_bus_off_time = current_tick;

            /* Only reset attempts if not in grace period */
            if (!mon->in_grace_period)
            {
                mon->recovery_attempts = 0;
            }
        }
        else if (new_state == CCX_BUS_STATE_WARNING)
        {
            mon->stats.error_warning_count++;
        }
        else if (new_state == CCX_BUS_STATE_PASSIVE)
        {
            mon->stats.error_passive_count++;
        }
        else if (new_state == CCX_BUS_STATE_ACTIVE && old_state == CCX_BUS_STATE_OFF)
        {
            /* Successful recovery from bus-off */
            mon->stats.total_bus_off_duration += (current_tick - mon->bus_off_entry_time);
            mon->last_successful_recovery = current_tick;
            mon->in_grace_period = 0; /* Exit grace period */
        }

        /* User callback */
        if (mon->OnBusStateChange != NULL)
        {
            mon->OnBusStateChange(Instance, old_state, new_state, mon->UserData);
        }
    }

    /* Auto-recovery logic for bus-off */
    if (new_state == CCX_BUS_STATE_OFF)
    {
        /* Check if in grace period after failed recovery */
        if (mon->in_grace_period)
        {
            CCX_TIME_t grace_elapsed = current_tick - mon->grace_period_start;

            if (grace_elapsed >= mon->successful_run_time)
            {
                /* Grace period expired - try again */
                mon->recovery_attempts = 0; /* Reset counter */
                mon->in_grace_period = 0;
                mon->recovery_start_time = current_tick;
                mon->recovery_start_time_hr = CCX_GetHighResTick();

                /* Trigger recovery immediately */
                mon->recovery_attempts++;
                mon->RequestRecovery(Instance);

                if (mon->OnRecoveryAttempt != NULL)
                {
                    mon->OnRecoveryAttempt(Instance, mon->recovery_attempts, mon->UserData);
                }
            }
        }
        else
        {
            /* Normal auto-recovery mode */
            if (mon->auto_recovery_enabled)
            {
                uint8_t recovery_due = 0U;

                if (mon->recovery_delay.UsesHighRes)
                {
                    CCX_HR_TIME_t time_in_bus_off_hr = CCX_GetHighResTick() - mon->recovery_start_time_hr;
                    recovery_due = (uint8_t)(time_in_bus_off_hr >= mon->recovery_delay.HighResDelay);
                }
                else
                {
                    CCX_TIME_t time_in_bus_off = current_tick - mon->recovery_start_time;
                    recovery_due = (uint8_t)(time_in_bus_off >= mon->recovery_delay.BaseDelay);
                }

                if (recovery_due)
                {
                    /* Check if we can still attempt recovery */
                    if (mon->max_recovery_attempts == 0 || mon->recovery_attempts < mon->max_recovery_attempts)
                    {
                        mon->recovery_attempts++;
                        mon->RequestRecovery(Instance);

                        if (mon->OnRecoveryAttempt != NULL)
                        {
                            mon->OnRecoveryAttempt(Instance, mon->recovery_attempts, mon->UserData);
                        }

                        mon->recovery_start_time = current_tick;
                        mon->recovery_start_time_hr = CCX_GetHighResTick();
                    }
                    else
                    {
                        /* Max attempts reached - enter grace period */
                        if (mon->OnRecoveryFailed != NULL)
                        {
                            mon->OnRecoveryFailed(Instance, mon->UserData);
                        }

                        mon->in_grace_period = 1;
                        mon->grace_period_start = current_tick;
                    }
                }
            }
        }
    }

    /* Reset recovery counter after successful run time (when ACTIVE) */
    if (new_state == CCX_BUS_STATE_ACTIVE && mon->recovery_attempts > 0)
    {
        CCX_TIME_t time_since_recovery = current_tick - mon->last_successful_recovery;

        if (time_since_recovery >= mon->successful_run_time)
        {
            mon->recovery_attempts = 0; /* Reset counter after stable operation */
        }
    }
}

CCX_Status_t CCX_BusMonitor_Init(CCX_instance_t *Instance, CCX_BusMonitor_t *Monitor,
                                 CCX_BusState_t (*GetBusState)(const CCX_instance_t *),
                                 void (*GetErrorCounters)(const CCX_instance_t *, CCX_ErrorCounters_t *),
                                 void (*RequestRecovery)(const CCX_instance_t *), CCX_BusRecoveryDelay_t recovery_delay,
                                 CCX_TIME_t successful_run_time, uint8_t auto_recovery_enabled,
                                 uint8_t max_recovery_attempts)
{
    if (Instance == NULL || Monitor == NULL || GetBusState == NULL || RequestRecovery == NULL)
    {
        return CCX_NULL_PTR;
    }

#ifndef CCX_DISABLE_HIGH_RES_TIMEBASE
    if (recovery_delay.UsesHighRes && !CCX_IsHighResTickRegistered())
    {
        memset(Monitor, 0, sizeof(CCX_BusMonitor_t));
        return CCX_MISSING_TIMEBASE;
    }
#endif

    /* Initialize monitor structure */
    memset(Monitor, 0, sizeof(CCX_BusMonitor_t));

    Monitor->current_state = CCX_BUS_STATE_ACTIVE;
    Monitor->recovery_delay = recovery_delay;
    Monitor->successful_run_time = successful_run_time;
    Monitor->auto_recovery_enabled = auto_recovery_enabled;
    Monitor->max_recovery_attempts = max_recovery_attempts;

    Monitor->GetBusState = GetBusState;
    Monitor->GetErrorCounters = GetErrorCounters;
    Monitor->RequestRecovery = RequestRecovery;

    /* Link to instance */
    Instance->BusMonitor = Monitor;

    /* Initialize state from hardware */
    Monitor->current_state = GetBusState(Instance);

    /* Initialize error counters if available */
    if (GetErrorCounters != NULL)
    {
        GetErrorCounters(Instance, &Monitor->stats.error_counters);
        Monitor->stats.peak_error_counters = Monitor->stats.error_counters;
    }

    return CCX_OK;
}

CCX_Status_t CCX_BusMonitor_TriggerRecovery(CCX_instance_t *Instance)
{
    if (Instance == NULL || Instance->BusMonitor == NULL)
    {
        return CCX_NULL_PTR;
    }

    CCX_BusMonitor_t *mon = Instance->BusMonitor;

    if (mon->current_state != CCX_BUS_STATE_OFF)
    {
        return CCX_WRONG_ARG; /* Not in bus-off */
    }

    /* Manual recovery resets everything */
    mon->recovery_attempts = 0;
    mon->in_grace_period = 0;
    mon->recovery_start_time = CCX_GetPrimaryTick();
    mon->recovery_start_time_hr = CCX_GetHighResTick();

    /* Trigger first attempt */
    mon->recovery_attempts++;
    mon->RequestRecovery(Instance);

    if (mon->OnRecoveryAttempt != NULL)
    {
        mon->OnRecoveryAttempt(Instance, mon->recovery_attempts, mon->UserData);
    }

    return CCX_OK;
}

CCX_BusState_t CCX_BusMonitor_GetState(const CCX_instance_t *Instance)
{
    if (Instance == NULL || Instance->BusMonitor == NULL)
    {
        return CCX_BUS_STATE_ACTIVE;
    }

    return Instance->BusMonitor->current_state;
}

void CCX_BusMonitor_ResetStats(CCX_instance_t *Instance)
{
    if (Instance == NULL || Instance->BusMonitor == NULL)
    {
        return;
    }

    memset(&Instance->BusMonitor->stats, 0, sizeof(CCX_BusStats_t));

    /* Re-initialize current error counters if available */
    if (Instance->BusMonitor->GetErrorCounters != NULL)
    {
        Instance->BusMonitor->GetErrorCounters(Instance, &Instance->BusMonitor->stats.error_counters);
        Instance->BusMonitor->stats.peak_error_counters = Instance->BusMonitor->stats.error_counters;
    }
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

