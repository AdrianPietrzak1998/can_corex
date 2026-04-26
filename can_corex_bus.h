/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Author: Adrian Pietrzak
 * GitHub: https://github.com/AdrianPietrzak1998
 * Created: Apr 24, 2026
 */

#ifndef CAN_COREX_CAN_COREX_BUS_H_
#define CAN_COREX_CAN_COREX_BUS_H_

#if !defined(CAN_COREX_CAN_COREX_H_) && !defined(DOXYGEN)
#error "Include can_corex.h instead of can_corex_bus.h directly."
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file can_corex_bus.h
 * @brief Bus monitoring public API.
 *
 * @defgroup ccx_bus Bus Monitoring
 * @brief CAN bus state tracking, bus-off recovery, and bus health statistics.
 * @{
 */

typedef struct
{
    CCX_HR_TIME_t HighResDelay;
    CCX_TIME_t BaseDelay;
    uint8_t UsesHighRes;
} CCX_BusRecoveryDelay_t;

#ifdef CCX_DISABLE_HIGH_RES_TIMEBASE
#define CCX_BUS_RECOVERY_MS(ms)                                                                                        \
    ((CCX_BusRecoveryDelay_t){.HighResDelay = 0U, .BaseDelay = (CCX_TIME_t)(ms), .UsesHighRes = 0U})
#define CCX_BUS_RECOVERY_US(us)                                                                                        \
    ((CCX_BusRecoveryDelay_t){                                                                                         \
        .HighResDelay = 0U, .BaseDelay = (CCX_TIME_t)CCX_INTERNAL_MS_FROM_US_CEIL(us), .UsesHighRes = 0U})
#else
#define CCX_BUS_RECOVERY_MS(ms)                                                                                        \
    ((CCX_BusRecoveryDelay_t){.HighResDelay = 0U, .BaseDelay = (CCX_TIME_t)(ms), .UsesHighRes = 0U})
#define CCX_BUS_RECOVERY_US(us)                                                                                        \
    ((CCX_BusRecoveryDelay_t){.HighResDelay = ((us) <= 3000U) ? CCX_HR_TIME(us) : 0U,                                  \
                              .BaseDelay = (CCX_TIME_t)CCX_INTERNAL_MS_FROM_US_CEIL(us),                               \
                              .UsesHighRes = (uint8_t)((us) <= 3000U)})
#endif

/* ============================================================
 * ISO 11898-1 BUS-OFF recovery timing
 *
 * Condition:
 *   128 x 11 recessive bits = 1408 bits
 *
 * Classic CAN constants are expressed directly in milliseconds because they
 * do not benefit from the HR domain. Short recovery delays can still be
 * expressed through CCX_BUS_RECOVERY_US(us). Values up to 3 ms use the HR
 * timebase when available; longer values are tracked in the base time domain.
 * The _MS suffix is kept for API stability.
 * ============================================================ */

#define CAN_COREX_BUS_OFF_RECOVERY_10KBPS_MS CCX_BUS_RECOVERY_MS(141U)
#define CAN_COREX_BUS_OFF_RECOVERY_20KBPS_MS CCX_BUS_RECOVERY_MS(71U)
#define CAN_COREX_BUS_OFF_RECOVERY_50KBPS_MS CCX_BUS_RECOVERY_MS(29U)
#define CAN_COREX_BUS_OFF_RECOVERY_83K3BPS_MS CCX_BUS_RECOVERY_MS(17U)
#define CAN_COREX_BUS_OFF_RECOVERY_100KBPS_MS CCX_BUS_RECOVERY_MS(15U)
#define CAN_COREX_BUS_OFF_RECOVERY_125KBPS_MS CCX_BUS_RECOVERY_MS(12U)
#define CAN_COREX_BUS_OFF_RECOVERY_250KBPS_MS CCX_BUS_RECOVERY_MS(6U)
#define CAN_COREX_BUS_OFF_RECOVERY_500KBPS_MS CCX_BUS_RECOVERY_US(2816U)
#define CAN_COREX_BUS_OFF_RECOVERY_800KBPS_MS CCX_BUS_RECOVERY_US(1760U)
#define CAN_COREX_BUS_OFF_RECOVERY_1000KBPS_MS CCX_BUS_RECOVERY_US(1408U)

#if CCX_ENABLE_CANFD
#define CAN_COREX_BUS_OFF_RECOVERY_FD_2M_MS CCX_BUS_RECOVERY_US(704U)
#define CAN_COREX_BUS_OFF_RECOVERY_FD_5M_MS CCX_BUS_RECOVERY_US(282U)
#define CAN_COREX_BUS_OFF_RECOVERY_FD_8M_MS CCX_BUS_RECOVERY_US(176U)
#endif

/**
 * @brief CAN bus state according to ISO 11898-1
 *
 * Represents the current error state of the CAN controller.
 * State transitions are based on Transmit Error Counter (TEC) and Receive Error Counter (REC).
 *
 * Values:
 * - CCX_BUS_STATE_ACTIVE: Error Active (TEC < 96 && REC < 96) - normal operation
 * - CCX_BUS_STATE_WARNING: Error Warning (TEC > 96 || REC > 96) - degraded performance
 * - CCX_BUS_STATE_PASSIVE: Error Passive (TEC > 127 || REC > 127) - cannot send active error frames
 * - CCX_BUS_STATE_OFF: Bus Off (TEC > 255) - disconnected from bus
 */
typedef enum
{
    CCX_BUS_STATE_ACTIVE = 0,
    CCX_BUS_STATE_WARNING,
    CCX_BUS_STATE_PASSIVE,
    CCX_BUS_STATE_OFF
} CCX_BusState_t;

/**
 * @brief CAN error counters from hardware controller
 *
 * According to ISO 11898-1, CAN controllers maintain two error counters:
 * - TEC (Transmit Error Counter): Incremented on transmission errors
 * - REC (Receive Error Counter): Incremented on reception errors
 *
 * These counters determine the bus state:
 * - Error Active: TEC < 96 && REC < 96
 * - Error Warning: TEC > 96 || REC > 96
 * - Error Passive: TEC > 127 || REC > 127
 * - Bus Off: TEC > 255
 */
typedef struct
{
    uint8_t TEC; /**< Transmit Error Counter (0-255) */
    uint8_t REC; /**< Receive Error Counter (0-255) */
} CCX_ErrorCounters_t;

/**
 * @brief Bus monitoring statistics
 *
 * Tracks detailed bus health metrics including error states and recovery attempts.
 * Statistics are accumulated over the lifetime of the bus monitor.
 */
typedef struct
{
    uint32_t bus_off_count;                  /**< Number of bus-off events */
    uint32_t error_warning_count;            /**< Number of error warning events (TEC/REC > 96) */
    uint32_t error_passive_count;            /**< Number of error passive events (TEC/REC > 127) */
    CCX_TIME_t last_bus_off_time;            /**< Timestamp of last bus-off occurrence in base ticks */
    CCX_TIME_t total_bus_off_duration;       /**< Cumulative time spent in bus-off state in base ticks */
    CCX_ErrorCounters_t error_counters;      /**< Current TEC/REC values from hardware */
    CCX_ErrorCounters_t peak_error_counters; /**< Peak TEC/REC values since initialization */
} CCX_BusStats_t;

/**
 * @brief Bus monitor configuration and state
 *
 * Provides automatic bus-off detection and recovery with configurable retry strategy.
 * Recovery process has two phases:
 * 1. Active recovery: Attempts recovery up to max_recovery_attempts times
 * 2. Grace period: After max attempts, waits successful_run_time before trying again
 */
typedef struct
{
    CCX_BusState_t current_state; /**< Current bus state */
    CCX_BusStats_t stats;         /**< Accumulated statistics */

    /* Recovery parameters */
    CCX_BusRecoveryDelay_t
        recovery_delay;             /**< Delay between recovery attempts, in base or HR ticks depending on macro used */
    CCX_TIME_t successful_run_time; /**< Time to run successfully before resetting counter, in base ticks */
    uint8_t auto_recovery_enabled;  /**< Enable automatic bus-off recovery */
    uint8_t max_recovery_attempts;  /**< Max attempts before grace period (0 = unlimited) */
    uint8_t recovery_attempts;      /**< Current recovery attempt counter */

    /* Internal state */
    CCX_TIME_t recovery_start_time; /**< Base-domain timestamp for current recovery cycle */
    CCX_HR_TIME_t
        recovery_start_time_hr; /**< HR-domain timestamp for current recovery cycle when sub-ms recovery is used */
    CCX_TIME_t last_successful_recovery; /**< When last recovery succeeded */
    CCX_TIME_t bus_off_entry_time;       /**< When bus-off state was entered */
    uint8_t in_grace_period;             /**< 1 = waiting in grace period after max attempts */
    CCX_TIME_t grace_period_start;       /**< When grace period started */

    /* Hardware interface - user implements these */
    CCX_BusState_t (*GetBusState)(const CCX_instance_t *Instance); /**< Read bus state from hardware */
    void (*GetErrorCounters)(const CCX_instance_t *Instance,
                             CCX_ErrorCounters_t *Counters); /**< Read TEC/REC (can be NULL) */
    void (*RequestRecovery)(const CCX_instance_t *Instance); /**< Trigger recovery in hardware */

    /* User callbacks */
    void (*OnBusStateChange)(CCX_instance_t *Instance, CCX_BusState_t OldState, CCX_BusState_t NewState,
                             void *UserData); /**< Called on state transition */
    void (*OnRecoveryAttempt)(CCX_instance_t *Instance, uint8_t AttemptNumber,
                              void *UserData);                          /**< Called before each recovery attempt */
    void (*OnRecoveryFailed)(CCX_instance_t *Instance, void *UserData); /**< Called when max attempts reached */
    void (*OnErrorCountersUpdate)(CCX_instance_t *Instance, const CCX_ErrorCounters_t *Counters,
                                  void *UserData); /**< Called when TEC/REC updated */
    void *UserData;                                /**< User context pointer for callbacks */
} CCX_BusMonitor_t;

/* ========================================================================
 * BUS MONITORING API (v1.3.0)
 * ======================================================================== */

CCX_Status_t CCX_BusMonitor_Init(CCX_instance_t *Instance, CCX_BusMonitor_t *Monitor,
                                 CCX_BusState_t (*GetBusState)(const CCX_instance_t *),
                                 void (*GetErrorCounters)(const CCX_instance_t *, CCX_ErrorCounters_t *),
                                 void (*RequestRecovery)(const CCX_instance_t *), CCX_BusRecoveryDelay_t recovery_delay,
                                 CCX_TIME_t successful_run_time, uint8_t auto_recovery_enabled,
                                 uint8_t max_recovery_attempts);

CCX_Status_t CCX_BusMonitor_TriggerRecovery(CCX_instance_t *Instance);
CCX_BusState_t CCX_BusMonitor_GetState(const CCX_instance_t *Instance);
void CCX_BusMonitor_ResetStats(CCX_instance_t *Instance);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CAN_COREX_CAN_COREX_BUS_H_ */
