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

#if CCX_TICK_FROM_FUNC

CCX_TIME_t (*CCX_get_tick)(void) = NULL;

#define CCX_GET_TICK ((CCX_get_tick != NULL) ? CCX_get_tick() : ((CCX_TIME_t)0))

void CCX_tick_function_register(CCX_TIME_t (*Function)(void))
{
    assert(Function != NULL);

    CCX_get_tick = Function;
}

#else

CCX_TIME_t *CCX_tick = NULL;

#define CCX_GET_TICK (*(CCX_tick))

void CCX_tick_variable_register(CCX_TIME_t *Variable)
{
    assert(Variable != NULL);

    CCX_tick = Variable;
}

#endif

extern void CCX_net_push(const CCX_instance_t *Instance, const CCX_message_t *msg, uint8_t FromTxFunc);

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
    if (msg->DLC > 8)
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

    Instance->RxHead = next_head;

    memcpy(&Instance->RxBuf[Instance->RxHead], msg, sizeof(CCX_message_t));
    Instance->RxReceivedTick[Instance->RxHead] = CCX_GET_TICK;

    Instance->GlobalStats.total_rx_messages++; /* Increment RX counter (v1.3.0) */

    CCX_net_push(Instance, &Instance->RxBuf[Instance->RxHead], 0);

    return CCX_OK;
}

static inline CCX_MsgRegStatus_t CCX_RX_MsgFromTables(CCX_instance_t *Instance, CCX_message_t *Msg)
{
    assert(Instance != NULL);

    if (NULL == Instance->CCX_RX_table)
    {
        return CCX_MSG_UNREG;
    }

    for (uint16_t i = 0; i < Instance->RxTableSize; i++)
    {
        if (Instance->CCX_RX_table[i].ID == Msg->ID)
        {
            /* Check DLC: CCX_DLC_ANY accepts any DLC (0-8), otherwise exact match */
            uint8_t dlc_match = 0;
            if (Instance->CCX_RX_table[i].DLC == CCX_DLC_ANY)
            {
                dlc_match = 1; /* Any DLC accepted */
            }
            else
            {
                dlc_match = (Instance->CCX_RX_table[i].DLC == Msg->DLC); /* Exact match */
            }

            if (dlc_match && (Instance->CCX_RX_table[i].IDE_flag == Msg->IDE_flag))

            {
                if (NULL != Instance->CCX_RX_table[i].Parser)
                {
                    Instance->GlobalStats.parser_calls_count++; /* Track parser calls (v1.3.0) */
                    Instance->CCX_RX_table[i].Parser(Instance, Msg, i, Instance->CCX_RX_table[i].UserData);
                }
                Instance->CCX_RX_table[i].LastTick = Instance->RxReceivedTick[Instance->RxTail];
                return CCX_MSG_REG;
            }
        }
    }
    return CCX_MSG_UNREG;
}

static inline void CCX_Timeout_Check(CCX_instance_t *Instance)
{
    if (NULL != Instance->CCX_RX_table)
    {
        for (uint16_t i = 0; i < Instance->RxTableSize; i++)
        {
            if ((0 != Instance->CCX_RX_table[i].TimeOut) &&
                (CCX_GET_TICK - Instance->CCX_RX_table[i].LastTick >= Instance->CCX_RX_table[i].TimeOut))
            {
                Instance->CCX_RX_table[i].LastTick = CCX_GET_TICK;
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
    if (msg->DLC > 8)
    {
        return CCX_WRONG_ARG;
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

    Instance->TxHead = next_head;

    memcpy(&Instance->TxBuf[Instance->TxHead], msg, sizeof(CCX_message_t));

    CCX_net_push(Instance, &Instance->TxBuf[Instance->TxHead], 1);

    return CCX_OK;
}

static inline void CCX_TX_MsgFromTables(CCX_instance_t *Instance)
{
    assert(NULL != Instance);
    uint8_t Tmp[8];
    memset(Tmp, 0x00, 8);

    for (uint16_t i = 0; i < Instance->TxTableSize; i++)
    {
        if (CCX_GET_TICK - Instance->CCX_TX_table[i].LastTick >= Instance->CCX_TX_table[i].SendFreq)
        {
            Instance->CCX_TX_table[i].LastTick = CCX_GET_TICK;
            CopyBuf(Instance->CCX_TX_table[i].Data, Tmp, Instance->CCX_TX_table[i].DLC);
            if (NULL != Instance->CCX_TX_table[i].Parser)
            {
                Instance->CCX_TX_table[i].Parser(Instance, Tmp, i, Instance->CCX_TX_table[i].UserData);
            }
            CCX_message_t msg = {.ID = Instance->CCX_TX_table[i].ID,
                                 .DLC = Instance->CCX_TX_table[i].DLC,
                                 .IDE_flag = Instance->CCX_TX_table[i].IDE_flag};

            memcpy(msg.Data, Tmp, 8);

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

    CCX_TIME_t current_tick = CCX_GET_TICK;

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
            mon->bus_off_entry_time = CCX_GET_TICK;
            mon->recovery_start_time = CCX_GET_TICK;
            mon->stats.last_bus_off_time = CCX_GET_TICK;

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
            mon->stats.total_bus_off_duration += (CCX_GET_TICK - mon->bus_off_entry_time);
            mon->last_successful_recovery = CCX_GET_TICK;
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
            CCX_TIME_t grace_elapsed = CCX_GET_TICK - mon->grace_period_start;

            if (grace_elapsed >= mon->successful_run_time)
            {
                /* Grace period expired - try again */
                mon->recovery_attempts = 0; /* Reset counter */
                mon->in_grace_period = 0;
                mon->recovery_start_time = CCX_GET_TICK;

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
                CCX_TIME_t time_in_bus_off = CCX_GET_TICK - mon->recovery_start_time;

                if (time_in_bus_off >= mon->recovery_delay)
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

                        mon->recovery_start_time = CCX_GET_TICK;
                    }
                    else
                    {
                        /* Max attempts reached - enter grace period */
                        if (mon->OnRecoveryFailed != NULL)
                        {
                            mon->OnRecoveryFailed(Instance, mon->UserData);
                        }

                        mon->in_grace_period = 1;
                        mon->grace_period_start = CCX_GET_TICK;
                    }
                }
            }
        }
    }

    /* Reset recovery counter after successful run time (when ACTIVE) */
    if (new_state == CCX_BUS_STATE_ACTIVE && mon->recovery_attempts > 0)
    {
        CCX_TIME_t time_since_recovery = CCX_GET_TICK - mon->last_successful_recovery;

        if (time_since_recovery >= mon->successful_run_time)
        {
            mon->recovery_attempts = 0; /* Reset counter after stable operation */
        }
    }
}

CCX_Status_t CCX_BusMonitor_Init(CCX_instance_t *Instance, CCX_BusMonitor_t *Monitor,
                                 CCX_BusState_t (*GetBusState)(const CCX_instance_t *),
                                 void (*GetErrorCounters)(const CCX_instance_t *, CCX_ErrorCounters_t *),
                                 void (*RequestRecovery)(const CCX_instance_t *), CCX_TIME_t recovery_delay,
                                 CCX_TIME_t successful_run_time, uint8_t auto_recovery_enabled,
                                 uint8_t max_recovery_attempts)
{
    if (Instance == NULL || Monitor == NULL || GetBusState == NULL || RequestRecovery == NULL)
    {
        return CCX_NULL_PTR;
    }

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
    mon->recovery_start_time = CCX_GET_TICK;

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