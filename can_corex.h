/*
 * can_corex.h
 *
 *  Created on: Aug 20, 2025
 *      Author: Adrian
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


typedef enum
{
    CCX_MSG_UNREG,
    CCX_MSG_REG
} CCX_MsgRegStatus_t;

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

typedef struct
{
    uint32_t ID;
    uint8_t DLC : 4;
    uint8_t IDE_flag : 1;
    CCX_TIME_t TimeOut;
    void (*Parser)(const CCX_instance_t *Instance, CCX_message_t *Msg, uint16_t Slot);
    CCX_TIME_t LastTick;
} CCX_RX_table_t;

typedef struct
{
    uint32_t ID;
    uint8_t *Data;
    uint8_t DLC : 4;
    uint8_t IDE_flag : 1;
    CCX_TIME_t SendFreq;
    void (*Parser)(const CCX_instance_t *Instance, uint8_t *DataToSend, uint16_t Slot);
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

	CCX_RX_table_t* CCX_RX_table;
	CCX_TX_table_t* CCX_TX_table;
	uint16_t RxTableSize, TxTableSize;
	void (*TimeoutCallback)(CCX_instance_t *Instance, uint16_t Slot);
	void (*Parser_unreg_msg)(const CCX_instance_t *Instance, CCX_message_t *Msg);
};


void CCX_RX_PushMsg(CCX_instance_t *Instance, uint32_t ID, const uint8_t *Data, uint8_t DLC, uint8_t IDE_flag);
void CCX_TX_PushMsg(CCX_instance_t *Instance, uint32_t ID, const uint8_t *Data, uint8_t DLC, uint8_t IDE_flag);
void CCX_Poll(CCX_instance_t *Instance);
void CCX_Init(CCX_instance_t *Instance, CCX_RX_table_t* CCX_RX_table, CCX_TX_table_t* CCX_TX_table, uint16_t RxTableSize,
		uint16_t TxTableSize, void (*SendFunction)(const CCX_instance_t *Instance, const CCX_message_t *msg),
		CCX_BusIsFree_t (*BusCheck)(const CCX_instance_t *Instance), void (*TimeoutCallback)(CCX_instance_t *Instance, uint16_t Slot),
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


#endif /* CAN_COREX_CAN_COREX_H_ */
