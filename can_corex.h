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
#ifndef CCX_TICK_FROM_FUNC
#define CCX_TICK_FROM_FUNC 0
#endif

/**
 * @def CCX_HR_TICK_FROM_FUNC
 * @brief Enables high-resolution tick retrieval via a function call.
 *
 * This switch is independent from `CCX_TICK_FROM_FUNC`.
 */
#ifndef CCX_HR_TICK_FROM_FUNC
#define CCX_HR_TICK_FROM_FUNC 0
#endif

/**
 * @def CCX_RX_BUFFER_SIZE
 * @brief Size of the CAN receive buffer.
 *
 * Defines the number of messages that can be stored in the CAN RX buffer.
 */
#ifndef CCX_RX_BUFFER_SIZE
#define CCX_RX_BUFFER_SIZE 48
#endif

/**
 * @def CCX_TX_BUFFER_SIZE
 * @brief Size of the CAN transmit buffer.
 *
 * Defines the number of messages that can be stored in the CAN TX buffer.
 */
#ifndef CCX_TX_BUFFER_SIZE
#define CCX_TX_BUFFER_SIZE 48
#endif

/**
 * @def CCX_ENABLE_CANFD
 * @brief Enables CAN FD support (64-byte payload, BRS, ESI, FDF).
 *
 * When 0 (default): zero RAM/code overhead, ABI identical to v1.x.
 * When 1: CCX_message_t grows to 64-byte payload; CCX_frame_format_t and CCX_ide_t enums available.
 */
#ifndef CCX_ENABLE_CANFD
#define CCX_ENABLE_CANFD 0
#endif

/**
 * @def CCX_DLC_ANY
 * @brief Special DLC value for wildcard matching in RX table.
 *
 * When set in CCX_RX_table_t.DLC the parser is called for any received DLC.
 * In classic builds (CCX_ENABLE_CANFD=0) the sentinel is 15 (fits in 4-bit DLC field,
 * classic frames are DLC 0-8).
 * In FD builds (CCX_ENABLE_CANFD=1) the sentinel is 16 (max valid FD DLC is 15,
 * stored in the 5-bit DLC field).  FrameFormat in the table entry controls which
 * frame format is matched: CLASSIC → classic wildcard, FD/FD_BRS → FD wildcard.
 *
 * Example (FD build):
 * CCX_RX_table_t rx_table[] = {
 *     {.ID = 0x100, .DLC = 8,           .FrameFormat = CCX_FRAME_FORMAT_CLASSIC, .Parser = exact_parser},
 *     {.ID = 0x200, .DLC = CCX_DLC_ANY, .FrameFormat = CCX_FRAME_FORMAT_FD,      .Parser = fd_any_parser},
 * };
 */
#if CCX_ENABLE_CANFD
#define CCX_DLC_ANY 16
#else
#define CCX_DLC_ANY 15
#endif

/**
 * @def CCX_RX_HASH_SIZE
 * @brief Size of the hash table for RX message lookup (used when CCX_RX_SEARCH_HASH is defined).
 *
 * Default value is 64. Can be overridden before including this header.
 * Larger values reduce collisions but increase memory usage.
 * Recommended to use a power of 2 or prime number.
 */
#ifndef CCX_RX_HASH_SIZE
#define CCX_RX_HASH_SIZE 64
#endif

/* ============================================================
 * Time base type configuration
 *
 * Primary time base:
 *   - used by core CAN RX/TX logic
 *   - expressed by CCX_TIME_t / CCX_MAX_TIMEOUT
 *   - public unit is always milliseconds
 *
 * High-resolution time base:
 *   - used by HR-aware modules such as ISO-TP and bus monitor
 *   - expressed by CCX_HR_TIME_t / CCX_HR_MAX_TIMEOUT
 *   - public unit is microseconds when HR is enabled
 *   - public unit falls back to milliseconds when HR is disabled
 *
 * Configuration is width-based and unsigned only.
 * Signed tick types are intentionally not supported because the library relies
 * on wraparound subtraction for timeout arithmetic.
 *
 * If no custom width macro is defined, uint32_t is used by default.
 * ============================================================ */

#ifdef CCX_TIME_BASE_TYPE_CUSTOM
#error                                                                                                                 \
    "CCX_TIME_BASE_TYPE_CUSTOM is no longer supported. Define exactly one of CCX_TIME_BASE_TYPE_CUSTOM_IS_UINT8/16/32/64."
#endif

#ifdef CCX_HR_TIME_BASE_TYPE_CUSTOM
#error                                                                                                                 \
    "CCX_HR_TIME_BASE_TYPE_CUSTOM is no longer supported. Define exactly one of CCX_HR_TIME_BASE_TYPE_CUSTOM_IS_UINT8/16/32/64."
#endif

#if defined(CCX_TIME_BASE_TYPE_CUSTOM_IS_INT8) || defined(CCX_TIME_BASE_TYPE_CUSTOM_IS_INT16) ||                       \
    defined(CCX_TIME_BASE_TYPE_CUSTOM_IS_INT32) || defined(CCX_TIME_BASE_TYPE_CUSTOM_IS_INT64)
#error                                                                                                                 \
    "Signed primary time base types were a bug and must not be used. Use exactly one of CCX_TIME_BASE_TYPE_CUSTOM_IS_UINT16/32/64."
#endif

#if defined(CCX_HR_TIME_BASE_TYPE_CUSTOM_IS_INT8) || defined(CCX_HR_TIME_BASE_TYPE_CUSTOM_IS_INT16) ||                 \
    defined(CCX_HR_TIME_BASE_TYPE_CUSTOM_IS_INT32) || defined(CCX_HR_TIME_BASE_TYPE_CUSTOM_IS_INT64)
#error                                                                                                                 \
    "Signed high-resolution time base types were a bug and must not be used. Use exactly one of CCX_HR_TIME_BASE_TYPE_CUSTOM_IS_UINT16/32/64."
#endif

#if defined(CCX_TIME_BASE_TYPE_CUSTOM_IS_UINT8)
#error                                                                                                                 \
    "CCX_TIME_BASE_TYPE_CUSTOM_IS_UINT8 is not supported because uint8_t has too small a range for this library. Use exactly one of CCX_TIME_BASE_TYPE_CUSTOM_IS_UINT16/32/64."
typedef volatile uint16_t CCX_TIME_t;
#define CCX_TIME_BASE_SCALAR uint16_t
#define CCX_MAX_TIMEOUT UINT16_MAX
#elif defined(CCX_TIME_BASE_TYPE_CUSTOM_IS_UINT16)
typedef volatile uint16_t CCX_TIME_t;
#define CCX_TIME_BASE_SCALAR uint16_t
#define CCX_MAX_TIMEOUT UINT16_MAX
#elif defined(CCX_TIME_BASE_TYPE_CUSTOM_IS_UINT32)
typedef volatile uint32_t CCX_TIME_t;
#define CCX_TIME_BASE_SCALAR uint32_t
#define CCX_MAX_TIMEOUT UINT32_MAX
#elif defined(CCX_TIME_BASE_TYPE_CUSTOM_IS_UINT64)
typedef volatile uint64_t CCX_TIME_t;
#define CCX_TIME_BASE_SCALAR uint64_t
#define CCX_MAX_TIMEOUT UINT64_MAX
#else
typedef volatile uint32_t CCX_TIME_t;
#define CCX_TIME_BASE_SCALAR uint32_t
#define CCX_MAX_TIMEOUT UINT32_MAX
#endif

#ifndef CCX_DISABLE_HIGH_RES_TIMEBASE

#if defined(CCX_HR_TIME_BASE_TYPE_CUSTOM_IS_UINT8)
#error                                                                                                                 \
    "CCX_HR_TIME_BASE_TYPE_CUSTOM_IS_UINT8 is not supported because uint8_t has too small a range for this library. Use exactly one of CCX_HR_TIME_BASE_TYPE_CUSTOM_IS_UINT16/32/64."
typedef volatile uint16_t CCX_HR_TIME_t;
#define CCX_HR_TIME_BASE_SCALAR uint16_t
#define CCX_HR_MAX_TIMEOUT UINT16_MAX
#elif defined(CCX_HR_TIME_BASE_TYPE_CUSTOM_IS_UINT16)
typedef volatile uint16_t CCX_HR_TIME_t;
#define CCX_HR_TIME_BASE_SCALAR uint16_t
#define CCX_HR_MAX_TIMEOUT UINT16_MAX
#elif defined(CCX_HR_TIME_BASE_TYPE_CUSTOM_IS_UINT32)
typedef volatile uint32_t CCX_HR_TIME_t;
#define CCX_HR_TIME_BASE_SCALAR uint32_t
#define CCX_HR_MAX_TIMEOUT UINT32_MAX
#elif defined(CCX_HR_TIME_BASE_TYPE_CUSTOM_IS_UINT64)
typedef volatile uint64_t CCX_HR_TIME_t;
#define CCX_HR_TIME_BASE_SCALAR uint64_t
#define CCX_HR_MAX_TIMEOUT UINT64_MAX
#else
typedef volatile uint16_t CCX_HR_TIME_t;
#define CCX_HR_TIME_BASE_SCALAR uint16_t
#define CCX_HR_MAX_TIMEOUT UINT16_MAX
#endif

#else

typedef CCX_TIME_t CCX_HR_TIME_t;
#define CCX_HR_TIME_BASE_SCALAR CCX_TIME_BASE_SCALAR
#define CCX_HR_MAX_TIMEOUT CCX_MAX_TIMEOUT

#endif

#define CCX_INTERNAL_MS_FROM_US_CEIL(us) (((us) == 0U) ? 0U : (((us)-1U) / 1000U) + 1U)

#ifdef CCX_DISABLE_HIGH_RES_TIMEBASE
#define CCX_HR_TIME(us) ((CCX_HR_TIME_t)CCX_INTERNAL_MS_FROM_US_CEIL(us))
#else
#define CCX_HR_TIME(us) ((CCX_HR_TIME_t)(us))
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
 * @brief CAN identifier type (standard 11-bit or extended 29-bit)
 */
typedef enum
{
    CCX_ID_STANDARD = 0, /**< Standard 11-bit identifier */
    CCX_ID_EXTENDED = 1, /**< Extended 29-bit identifier */
} CCX_ide_t;

#if CCX_ENABLE_CANFD
/**
 * @brief CAN frame format selection
 *
 * Used in message, RX table, and TX table entries to specify the frame format.
 * A single 2-bit field replaces the separate FDF and BRS bitfields from v1.x.
 *
 * For RX table entries:
 *   - CCX_FRAME_FORMAT_CLASSIC: entry matches only classic (non-FD) frames
 *   - CCX_FRAME_FORMAT_FD / CCX_FRAME_FORMAT_FD_BRS: entry matches only FD frames
 *
 * For TX table / message entries:
 *   - Controls the frame format transmitted on the bus
 */
typedef enum
{
    CCX_FRAME_FORMAT_CLASSIC = 0, /**< Classic CAN 2.0, max 8-byte payload */
    CCX_FRAME_FORMAT_FD = 1,      /**< CAN FD without bit-rate switch (BRS=0) */
    CCX_FRAME_FORMAT_FD_BRS = 2,  /**< CAN FD with bit-rate switch (BRS=1) */
} CCX_frame_format_t;
#endif

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
    uint16_t peak_rx_depth;       /**< Highest RX buffer depth observed since stats reset */
    uint16_t peak_tx_depth;       /**< Highest TX buffer depth observed since stats reset */
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
    CCX_BUS_TOO_BUSY,
    CCX_MISSING_TIMEBASE
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
#if CCX_ENABLE_CANFD
    uint8_t Data[64];
#else
    uint8_t Data[8];
#endif
    uint8_t DLC : 4;
    uint8_t IDE_flag : 1; /* use CCX_ide_t values */
#if CCX_ENABLE_CANFD
    uint8_t FrameFormat : 2; /* use CCX_frame_format_t values */
    uint8_t ESI : 1;         /* error state indicator (set by RX hardware) */
#endif
} CCX_message_t;

typedef struct CCX_instance_t CCX_instance_t;

#include "can_corex_bus.h"

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
#if CCX_ENABLE_CANFD
    uint8_t DLC : 5;         /* 0-15 = exact FD DLC; 16 (CCX_DLC_ANY) = wildcard */
    uint8_t IDE_flag : 1;    /* use CCX_ide_t values */
    uint8_t FrameFormat : 2; /* CLASSIC→match classic frames; FD/FD_BRS→match FD frames */
#else
    uint8_t DLC : 4;
    uint8_t IDE_flag : 1; /* use CCX_ide_t values */
#endif
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
    uint8_t IDE_flag : 1; /* use CCX_ide_t values */
#if CCX_ENABLE_CANFD
    uint8_t FrameFormat : 2; /* use CCX_frame_format_t values */
#endif
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

#if defined(CCX_RX_SEARCH_HASH)
    uint16_t RxHashTable[CCX_RX_HASH_SIZE]; /**< Hash table for fast RX message lookup */
#endif
};

CCX_Status_t CCX_RX_PushMsg(CCX_instance_t *Instance, const CCX_message_t *msg);
CCX_Status_t CCX_TX_PushMsg(CCX_instance_t *Instance, const CCX_message_t *msg);
uint16_t CCX_RX_GetDepth(const CCX_instance_t *Instance);
uint16_t CCX_TX_GetDepth(const CCX_instance_t *Instance);
uint16_t CCX_RX_GetFree(const CCX_instance_t *Instance);
uint16_t CCX_TX_GetFree(const CCX_instance_t *Instance);
void CCX_FlushRx(CCX_instance_t *Instance);
void CCX_FlushTx(CCX_instance_t *Instance);
void CCX_Flush(CCX_instance_t *Instance);
void CCX_Reset(CCX_instance_t *Instance);
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
void CCX_tick_function_register(CCX_TIME_BASE_SCALAR (*Function)(void));
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

#ifndef CCX_DISABLE_HIGH_RES_TIMEBASE
#if CCX_HR_TICK_FROM_FUNC
void CCX_high_res_tick_function_register(CCX_HR_TIME_BASE_SCALAR (*Function)(void));
#else
void CCX_high_res_tick_variable_register(CCX_HR_TIME_t *Variable);
#endif
#endif

uint8_t CCX_IsPrimaryTickRegistered(void);
uint8_t CCX_IsHighResTickRegistered(void);

#if CCX_TICK_FROM_FUNC
extern CCX_TIME_BASE_SCALAR (*CCX_get_tick)(void);
#define CCX_GetPrimaryTick() ((CCX_get_tick != NULL) ? (CCX_TIME_t)CCX_get_tick() : ((CCX_TIME_t)0))
#else
extern CCX_TIME_t *CCX_tick;
#define CCX_GetPrimaryTick() ((CCX_tick != NULL) ? (*CCX_tick) : ((CCX_TIME_t)0))
#endif

#ifdef CCX_DISABLE_HIGH_RES_TIMEBASE
#define CCX_GetHighResTick() ((CCX_HR_TIME_t)CCX_GetPrimaryTick())
#else
#if CCX_HR_TICK_FROM_FUNC
extern CCX_HR_TIME_BASE_SCALAR (*CCX_get_high_res_tick)(void);
#define CCX_GetHighResTick()                                                                                           \
    ((CCX_get_high_res_tick != NULL) ? (CCX_HR_TIME_t)CCX_get_high_res_tick() : ((CCX_HR_TIME_t)0))
#else
extern CCX_HR_TIME_t *CCX_high_res_tick;
#define CCX_GetHighResTick() ((CCX_high_res_tick != NULL) ? (*CCX_high_res_tick) : ((CCX_HR_TIME_t)0))
#endif
#endif

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

/**
 * @brief Rebuild hash table for RX message lookup
 *
 * This function rebuilds the internal hash table used for fast RX message lookup
 * when CCX_RX_SEARCH_HASH is defined at compile time.
 *
 * Call this function after modifying the RX table (CCX_RX_table) to update the hash table.
 * The hash table is automatically built during CCX_Init(), so manual rebuild is only needed
 * if you dynamically modify the RX table after initialization.
 *
 * @param Instance CAN instance
 *
 * @note This function does nothing if CCX_RX_SEARCH_HASH is not defined
 * @note Not needed for linear search or binary search modes
 *
 * @code
 * // After modifying RX table
 * rx_table[5].ID = 0x456;
 * CCX_RX_RebuildHash(&can_instance);
 * @endcode
 */
void CCX_RX_RebuildHash(CCX_instance_t *Instance);

/* ========================================================================
 * CAN FD API (v2.0, requires CCX_ENABLE_CANFD=1)
 * ======================================================================== */

#if CCX_ENABLE_CANFD

/**
 * @brief Named CAN FD DLC values
 *
 * Use these when setting the DLC field of CCX_message_t or CCX_TX_table_t
 * to make the intended payload length explicit.
 */
typedef enum
{
    CCX_FD_DLC_0B = 0,   /* 0 bytes  */
    CCX_FD_DLC_1B = 1,   /* 1 byte   */
    CCX_FD_DLC_2B = 2,   /* 2 bytes  */
    CCX_FD_DLC_3B = 3,   /* 3 bytes  */
    CCX_FD_DLC_4B = 4,   /* 4 bytes  */
    CCX_FD_DLC_5B = 5,   /* 5 bytes  */
    CCX_FD_DLC_6B = 6,   /* 6 bytes  */
    CCX_FD_DLC_7B = 7,   /* 7 bytes  */
    CCX_FD_DLC_8B = 8,   /* 8 bytes  */
    CCX_FD_DLC_12B = 9,  /* 12 bytes */
    CCX_FD_DLC_16B = 10, /* 16 bytes */
    CCX_FD_DLC_20B = 11, /* 20 bytes */
    CCX_FD_DLC_24B = 12, /* 24 bytes */
    CCX_FD_DLC_32B = 13, /* 32 bytes */
    CCX_FD_DLC_48B = 14, /* 48 bytes */
    CCX_FD_DLC_64B = 15, /* 64 bytes */
} CCX_FD_DLC_t;

/** DLC-to-payload-length lookup table: index 0-15, values {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64} */
extern const uint8_t CCX_FD_DLC_TO_LEN[16];

/**
 * @brief Convert byte length to the smallest valid CAN FD DLC
 *
 * Rounds up to the nearest valid FD payload length.
 * Lengths > 64 return DLC 15 (64 bytes).
 *
 * @param len Desired payload length in bytes
 * @return CAN FD DLC value (0-15)
 */
uint8_t CCX_FD_LenToDLC(uint8_t len);

/**
 * @brief Return actual payload length of a CAN message
 *
 * For FD frames uses the DLC-to-length LUT; for classic frames DLC equals length.
 *
 * @param msg Pointer to CAN message
 * @return Number of valid payload bytes
 */
static inline uint8_t CCX_MsgPayloadLen(const CCX_message_t *msg)
{
    return (msg->FrameFormat != CCX_FRAME_FORMAT_CLASSIC) ? CCX_FD_DLC_TO_LEN[msg->DLC] : msg->DLC;
}

#endif /* CCX_ENABLE_CANFD */

#endif /* CAN_COREX_CAN_COREX_H_ */
