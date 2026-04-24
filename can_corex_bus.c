/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Author: Adrian Pietrzak
 * GitHub: https://github.com/AdrianPietrzak1998
 * Created: Apr 24, 2026
 */

#include "can_corex.h"
#include "assert.h"
#include "string.h"

void CCX_BusMonitor_Update(CCX_instance_t *Instance)
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
            mon->stats.total_bus_off_duration += (CCX_TIME_t)(current_tick - mon->bus_off_entry_time);
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
            CCX_TIME_t grace_elapsed = (CCX_TIME_t)(current_tick - mon->grace_period_start);

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
                    CCX_HR_TIME_t time_in_bus_off_hr =
                        (CCX_HR_TIME_t)(CCX_GetHighResTick() - mon->recovery_start_time_hr);
                    recovery_due = (uint8_t)(time_in_bus_off_hr >= mon->recovery_delay.HighResDelay);
                }
                else
                {
                    CCX_TIME_t time_in_bus_off = (CCX_TIME_t)(current_tick - mon->recovery_start_time);
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
        CCX_TIME_t time_since_recovery = (CCX_TIME_t)(current_tick - mon->last_successful_recovery);

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
