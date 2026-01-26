/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Author: Adrian Pietrzak
 * GitHub: https://github.com/AdrianPietrzak1998
 * Created: Aug 20, 2025
 */

#ifndef CAN_COREX_CAN_COREX_H_
#define CAN_COREX_CAN_COREX_H_

#include <limits.h>
#include <stdint.h>

/**
 * @def CCX_TICK_FROM_FUNC
 * @brief Enables system tick retrieval via a function call.
 *
 * If set to 1, the system time base is obtained by calling a function
 * that returns the current tick count.
 * If set to 0, the system time base is read directly from a variable.
 */
#define CCX_TICK_FROM_FUNC 0

/**
 * @def CCX_RX_BUFFER_SIZE
 * @brief Size of the CAN receive buffer.
 *
 * Defines the number of messages that can be stored in the CAN RX buffer.
 */
#define CCX_RX_BUFFER_SIZE 48

/**
 * @def CCX_TX_BUFFER_SIZE
 * @brief Size of the CAN transmit buffer.
 *
 * Defines the number of messages that can be stored in the CAN TX buffer.
 */
#define CCX_TX_BUFFER_SIZE 48

/**
 * @def CCX_DLC_ANY
 * @brief Special DLC value for wildcard matching in RX table.
 *
 * When this value is used in CCX_RX_table_t.DLC field, the parser will be called
 * for any received DLC (0-8), regardless of the actual message DLC.
 * This is useful for protocols that may send messages with variable DLC.
 *
 * Example:
 * CCX_RX_table_t rx_table[] = {
 *     {.ID = 0x100, .DLC = 8, .Parser = my_parser},        // Exact match: only DLC=8
 *     {.ID = 0x200, .DLC = CCX_DLC_ANY, .Parser = my_parser} // Any DLC: 0-8
 * };
 */
#define CCX_DLC_ANY 15

/**
 * @brief System time base type and maximum timeout definition.
 *
 * By default, the system tick type is `volatile uint32_t`.
 * If you want to use a different type, define `CC_TIME_BASE_TYPE_CUSTOM` as that type
 * and also define the corresponding `_IS_` macro
 * (e.g., `CC_TIME_BASE_TYPE_CUSTOM_IS_UINT16`) to properly set `CC_MAX_TIMEOUT`.
 */
#ifndef CCX_TIME_BASE_TYPE_CUSTOM

#define CCX_MAX_TIMEOUT UINT32_MAX
typedef volatile uint32_t CCX_TIME_t;

#else

typedef CCX_TIME_BASE_TYPE_CUSTOM CC_TIME_t;

#if defined(CCX_TIME_BASE_TYPE_CUSTOM_IS_UINT8)
#define CCX_MAX_TIMEOUT UINT8_MAX
#elif defined(CCX_TIME_BASE_TYPE_CUSTOM_IS_UINT16)
#define CCX_MAX_TIMEOUT UINT16_MAX
#elif defined(CCX_TIME_BASE_TYPE_CUSTOM_IS_UINT32)
#define CCX_MAX_TIMEOUT UINT32_MAX
#elif defined(CCX_TIME_BASE_TYPE_CUSTOM_IS_UINT64)
#define CCX_MAX_TIMEOUT UINT64_MAX
#elif defined(CCX_TIME_BASE_TYPE_CUSTOM_IS_INT8)
#define CCX_MAX_TIMEOUT INT8_MAX
#elif defined(CCX_TIME_BASE_TYPE_CUSTOM_IS_INT16)
#define CCX_MAX_TIMEOUT INT16_MAX
#elif defined(CCX_TIME_BASE_TYPE_CUSTOM_IS_INT32)
#define CCX_MAX_TIMEOUT INT32_MAX
#elif defined(CCX_TIME_BASE_TYPE_CUSTOM_IS_INT64)
#define CCX_MAX_TIMEOUT INT64_MAX
#else
#error "CCX_MAX_TIMEOUT: Unknown CCX_TIME_BASE_TYPE_CUSTOM or missing _IS_* define"
#endif

#endif

/**
 * @brief Enumeration indicating the status of the CAN bus.
 *
 * This enum represents whether the CAN bus is free for transmitting a new message
 * or if it is currently busy.
 *
 * Values:
 * - CCX_BUS_BUSY: The CAN bus is currently busy and cannot accept new messages.
 * - CCX_BUS_FREE: The CAN bus is free and ready for new message transmission.
 */
typedef enum
{
    CCX_BUS_BUSY = 0,
    CCX_BUS_FREE
} CCX_BusIsFree_t;

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
 * @brief Global statistics for CAN instance
 *
 * These statistics are always enabled and have minimal performance overhead.
 * All counters are automatically maintained by the library.
 *
 * Usage:
 * @code
 * const CCX_GlobalStats_t *stats = CCX_GetGlobalStats(&can_instance);
 * printf("RX: %lu, TX: %lu, Overflows: %lu\n",
 *        stats->total_rx_messages,
 *        stats->total_tx_messages,
 *        stats->rx_buffer_overflows);
 * @endcode
 */
typedef struct
{
    uint32_t total_rx_messages; /**< Total messages received and pushed to RX buffer */
    uint32_t total_tx_messages; /**< Total messages successfully transmitted (call CCX_OnMessageTransmitted from ISR) */
    uint32_t rx_buffer_overflows; /**< Number of times RX buffer was full */
    uint32_t tx_buffer_overflows; /**< Number of times TX buffer was full */
    uint32_t parser_calls_count;  /**< Total number of parser function invocations */
    uint32_t timeout_calls_count; /**< Total number of timeout callback invocations */
} CCX_GlobalStats_t;

typedef enum
{
    CCX_MSG_UNREG,
    CCX_MSG_REG
} CCX_MsgRegStatus_t;

typedef enum
{
    CCX_OK = 0,
    CCX_NULL_PTR,
    CCX_WRONG_ARG,
    CCX_BUS_TOO_BUSY
} CCX_Status_t;

/**
 * @brief Structure representing a CAN message.
 *
 * Fields:
 * - ID: CAN message identifier.
 * - Data: CAN message data payload (up to 8 bytes).
 * - DLC: Data Length Code, number of valid data bytes (0-8).
 * - IDE_flag: Identifier Extension flag (0 = standard, 1 = extended).
 */
typedef struct
{
    uint32_t ID;
    uint8_t Data[8];
    uint8_t DLC : 4;
    uint8_t IDE_flag : 1;
} CCX_message_t;

typedef struct CCX_instance_t CCX_instance_t;

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
    CCX_TIME_t last_bus_off_time;            /**< Timestamp of last bus-off occurrence */
    CCX_TIME_t total_bus_off_duration;       /**< Cumulative time spent in bus-off state (ms) */
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
 *
 * Example usage:
 * @code
 * CCX_BusMonitor_t bus_monitor;
 *
 * CCX_BusMonitor_Init(
 *     &can_instance,
 *     &bus_monitor,
 *     my_get_bus_state,        // Read state from hardware
 *     my_get_error_counters,   // Read TEC/REC
 *     my_request_recovery,     // Trigger recovery
 *     10,      // recovery_delay: 10ms between attempts
 *     60000,   // successful_run_time: 60s grace period
 *     1,       // auto_recovery_enabled
 *     5        // max_recovery_attempts: 5 tries before grace period
 * );
 *
 * bus_monitor.OnBusStateChange = my_state_callback;
 * bus_monitor.OnRecoveryFailed = my_failed_callback;
 * @endcode
 */
typedef struct
{
    CCX_BusState_t current_state; /**< Current bus state */
    CCX_BusStats_t stats;         /**< Accumulated statistics */

    /* Recovery parameters */
    CCX_TIME_t recovery_delay;      /**< Delay between recovery attempts (ms, default: 10) */
    CCX_TIME_t successful_run_time; /**< Time to run successfully before resetting counter (ms, default: 60000) */
    uint8_t auto_recovery_enabled;  /**< Enable automatic bus-off recovery */
    uint8_t max_recovery_attempts;  /**< Max attempts before grace period (0 = unlimited) */
    uint8_t recovery_attempts;      /**< Current recovery attempt counter */

    /* Internal state */
    CCX_TIME_t recovery_start_time;      /**< When current recovery cycle started */
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

/**
 * @brief CAN RX table entry structure
 *
 * Defines a single entry in the RX message table for filtering and parsing incoming CAN messages.
 *
 * @note UserData Example:
 * @code
 * typedef struct {
 *     int counter;
 *     const char *name;
 * } MessageContext_t;
 *
 * MessageContext_t msg_ctx = {.counter = 0, .name = "RPM"};
 *
 * void rpm_parser(const CCX_instance_t *Instance, CCX_message_t *Msg,
 *                 uint16_t Slot, void *UserData) {
 *     MessageContext_t *ctx = (MessageContext_t *)UserData;
 *     ctx->counter++;
 *     printf("%s message #%d received\n", ctx->name, ctx->counter);
 * }
 *
 * CCX_RX_table_t rx_table[] = {
 *     {
 *         .ID = 0x200,
 *         .DLC = 8,
 *         .IDE_flag = 0,
 *         .UserData = &msg_ctx,
 *         .TimeOut = 1000,
 *         .Parser = rpm_parser,
 *         .TimeoutCallback = rpm_timeout
 *     }
 * };
 * @endcode
 */
typedef struct
{
    uint32_t ID;
    uint8_t DLC : 4;
    uint8_t IDE_flag : 1;
    void *UserData;
    CCX_TIME_t TimeOut;
    void (*Parser)(const CCX_instance_t *Instance, CCX_message_t *Msg, uint16_t Slot, void *UserData);
    void (*TimeoutCallback)(CCX_instance_t *Instance, uint16_t Slot, void *UserData);
    CCX_TIME_t LastTick;
} CCX_RX_table_t;

/**
 * @brief CAN TX table entry structure
 *
 * Defines a single entry in the TX message table for periodic message transmission.
 *
 * @note UserData Example:
 * @code
 * typedef struct {
 *     uint16_t *sensor_value;  // Pointer to live sensor data
 *     uint8_t scaling_factor;
 * } SensorContext_t;
 *
 * uint16_t temperature = 25;
 * SensorContext_t temp_ctx = {.sensor_value = &temperature, .scaling_factor = 10};
 *
 * void temp_tx_parser(const CCX_instance_t *Instance, uint8_t *DataToSend,
 *                     uint16_t Slot, void *UserData) {
 *     SensorContext_t *ctx = (SensorContext_t *)UserData;
 *     uint16_t scaled = *(ctx->sensor_value) * ctx->scaling_factor;
 *     DataToSend[0] = (uint8_t)(scaled >> 8);
 *     DataToSend[1] = (uint8_t)(scaled & 0xFF);
 * }
 *
 * uint8_t tx_data[8] = {0};
 * CCX_TX_table_t tx_table[] = {
 *     {
 *         .ID = 0x300,
 *         .Data = tx_data,
 *         .DLC = 8,
 *         .IDE_flag = 0,
 *         .UserData = &temp_ctx,
 *         .SendFreq = 100,  // Send every 100ms
 *         .Parser = temp_tx_parser
 *     }
 * };
 * @endcode
 */
typedef struct
{
    uint32_t ID;
    uint8_t *Data;
    uint8_t DLC : 4;
    uint8_t IDE_flag : 1;
    void *UserData;
    CCX_TIME_t SendFreq;
    void (*Parser)(const CCX_instance_t *Instance, uint8_t *DataToSend, uint16_t Slot, void *UserData);
    CCX_TIME_t LastTick;
} CCX_TX_table_t;

struct CCX_instance_t
{

    void (*SendFunction)(const CCX_instance_t *Instance, const CCX_message_t *msg);
    CCX_BusIsFree_t (*BusCheck)(const CCX_instance_t *Instance);

    CCX_message_t RxBuf[CCX_RX_BUFFER_SIZE];
    CCX_message_t TxBuf[CCX_TX_BUFFER_SIZE];
    uint16_t RxTail, RxHead, TxTail, TxHead;
    CCX_TIME_t RxReceivedTick[CCX_RX_BUFFER_SIZE];

    CCX_RX_table_t *CCX_RX_table;
    CCX_TX_table_t *CCX_TX_table;
    uint16_t RxTableSize, TxTableSize;
    void (*Parser_unreg_msg)(const CCX_instance_t *Instance, CCX_message_t *Msg);

    /* New fields for v1.3.0 - added at the end for compatibility */
    CCX_GlobalStats_t GlobalStats; /**< Global statistics (always enabled) */
    CCX_BusMonitor_t *BusMonitor;  /**< Bus monitoring (NULL = disabled) */
    void (*OnMessageTransmitted)(
        CCX_instance_t *Instance,
        const CCX_message_t *msg); /**< Callback for TX complete (optional, for user notification) */
};

CCX_Status_t CCX_RX_PushMsg(CCX_instance_t *Instance, const CCX_message_t *msg);
CCX_Status_t CCX_TX_PushMsg(CCX_instance_t *Instance, const CCX_message_t *msg);
CCX_Status_t CCX_Poll(CCX_instance_t *Instance);
CCX_Status_t CCX_Init(CCX_instance_t *Instance, CCX_RX_table_t *CCX_RX_table, CCX_TX_table_t *CCX_TX_table,
                      uint16_t RxTableSize, uint16_t TxTableSize,
                      void (*SendFunction)(const CCX_instance_t *Instance, const CCX_message_t *msg),
                      CCX_BusIsFree_t (*BusCheck)(const CCX_instance_t *Instance),
                      void (*ParserUnregMsg)(const CCX_instance_t *Instance, CCX_message_t *Msg));

#if CCX_TICK_FROM_FUNC
/**
 * @brief Registers the system tick source function.
 *
 * When CCX_TICK_FROM_FUNC is set to 1, this function registers a user-provided
 * function that returns the current system tick value.
 *
 * @param Function Pointer to a function that returns the current system tick (CCX_TIME_t).
 */
void CCX_tick_function_register(CCX_TIME_t (*Function)(void));
#else
/**
 * @brief Registers the system tick source variable.
 *
 * When CCX_TICK_FROM_FUNC is set to 0, this function registers a pointer
 * to a variable that holds the current system tick value.
 *
 * @param Variable Pointer to a volatile variable of type CCX_TIME_t representing the system tick.
 */
void CCX_tick_variable_register(CCX_TIME_t *Variable);
#endif

/* ========================================================================
 * BUS MONITORING API (v1.3.0)
 * ======================================================================== */

/**
 * @brief Initialize bus monitoring for a CAN instance
 *
 * Enables automatic bus-off detection and recovery with configurable retry strategy.
 * The recovery process works in two phases:
 * 1. Active recovery: Attempts recovery up to max_recovery_attempts times with recovery_delay between attempts
 * 2. Grace period: After max attempts, waits successful_run_time before trying again
 *
 * @param Instance CAN instance to monitor
 * @param Monitor Bus monitor structure (must persist during operation)
 * @param GetBusState Function to read current bus state from hardware (required)
 * @param GetErrorCounters Function to read TEC/REC from hardware (optional, can be NULL)
 * @param RequestRecovery Function to trigger bus-off recovery in hardware (required)
 * @param recovery_delay Delay between recovery attempts in milliseconds (recommended: 10ms minimum per ISO 11898-1)
 * @param successful_run_time Time to run successfully before resetting recovery counter in milliseconds (recommended:
 * 60000ms)
 * @param auto_recovery_enabled 1 = enable automatic recovery, 0 = manual recovery only
 * @param max_recovery_attempts Maximum recovery attempts before entering grace period (0 = unlimited)
 * @return CCX_OK on success, CCX_NULL_PTR if Instance/Monitor/callbacks are NULL
 *
 * @note After initialization, CCX_Poll() automatically calls CCX_BusMonitor_Update()
 * @note Set callbacks in Monitor structure after initialization (OnBusStateChange, OnRecoveryFailed, etc.)
 *
 * @code
 * CCX_BusMonitor_t bus_monitor;
 * CCX_BusMonitor_Init(&can_inst, &bus_monitor, my_get_state, my_get_tec_rec,
 *                     my_recovery, 10, 60000, 1, 5);
 * bus_monitor.OnBusStateChange = my_callback;
 * @endcode
 */
CCX_Status_t CCX_BusMonitor_Init(CCX_instance_t *Instance, CCX_BusMonitor_t *Monitor,
                                 CCX_BusState_t (*GetBusState)(const CCX_instance_t *),
                                 void (*GetErrorCounters)(const CCX_instance_t *, CCX_ErrorCounters_t *),
                                 void (*RequestRecovery)(const CCX_instance_t *), CCX_TIME_t recovery_delay,
                                 CCX_TIME_t successful_run_time, uint8_t auto_recovery_enabled,
                                 uint8_t max_recovery_attempts);

/**
 * @brief Manually trigger bus-off recovery
 *
 * Resets recovery attempt counter and immediately triggers recovery.
 * This is useful during grace period to retry earlier than grace period timeout.
 *
 * @param Instance CAN instance
 * @return CCX_OK on success
 * @return CCX_NULL_PTR if Instance or BusMonitor is NULL
 * @return CCX_WRONG_ARG if bus is not in bus-off state
 *
 * @note Calling this during grace period resets the counter and restarts active recovery phase
 */
CCX_Status_t CCX_BusMonitor_TriggerRecovery(CCX_instance_t *Instance);

/**
 * @brief Get current bus state
 *
 * @param Instance CAN instance
 * @return Current bus state, or CCX_BUS_STATE_ACTIVE if monitoring is disabled
 */
CCX_BusState_t CCX_BusMonitor_GetState(const CCX_instance_t *Instance);

/**
 * @brief Reset bus monitoring statistics
 *
 * Resets all counters in CCX_BusStats_t to zero.
 * Does not affect current bus state or recovery state.
 *
 * @param Instance CAN instance
 */
void CCX_BusMonitor_ResetStats(CCX_instance_t *Instance);

/**
 * @brief Get global statistics
 *
 * Global statistics are always enabled and track basic operational metrics.
 *
 * @param Instance CAN instance
 * @return Pointer to global statistics structure (always available, never NULL)
 */
const CCX_GlobalStats_t *CCX_GetGlobalStats(const CCX_instance_t *Instance);

/**
 * @brief Reset global statistics
 *
 * Resets all global counters to zero.
 *
 * @param Instance CAN instance
 */
void CCX_ResetGlobalStats(CCX_instance_t *Instance);

/**
 * @brief Notify library that a message was successfully transmitted
 *
 * Call this function from your CAN TX complete interrupt handler.
 * Automatically increments GlobalStats.total_tx_messages and calls OnMessageTransmitted callback if set.
 *
 * @param Instance CAN instance
 * @param msg Pointer to transmitted message (optional, can be NULL)
 *
 * @note This is the ONLY way to properly track transmitted messages
 * @note OnMessageTransmitted callback is for user notification only
 *
 * @code
 * // In STM32 HAL
 * void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
 *     CCX_OnMessageTransmitted(&can_instance, &last_sent_msg);
 * }
 *
 * // In bare-metal ISR
 * void CAN1_TX_IRQHandler(void) {
 *     if (CAN1->TSR & CAN_TSR_RQCP0) {
 *         CAN1->TSR |= CAN_TSR_RQCP0;
 *         CCX_OnMessageTransmitted(&can_instance, NULL);
 *     }
 * }
 * @endcode
 */
void CCX_OnMessageTransmitted(CCX_instance_t *Instance, const CCX_message_t *msg);

#endif /* CAN_COREX_CAN_COREX_H_ */