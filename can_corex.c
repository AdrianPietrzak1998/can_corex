/*
 * can_corex.c
 *
 *  Created on: Aug 20, 2025
 *      Author: Adrian
 */

#include "can_corex.h"
#include "assert.h"
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

extern void CCX_net_push(const CCX_instance_t* Instance, const CCX_message_t* msg);

static inline void CopyBuf(const uint8_t *restrict src, uint8_t *restrict dst, size_t size)
{
    assert((src != NULL) && (dst != NULL));

    for (size_t i = 0; i < size; i++)
    {
        dst[i] = src[i];
    }
}






void CCX_RX_PushMsg(CCX_instance_t *Instance, uint32_t ID, const uint8_t *Data, uint8_t DLC, uint8_t IDE_flag)
{
    assert(Instance != NULL);

    uint16_t next_head = Instance->RxHead + 1;
    if (next_head >= CCX_RX_BUFFER_SIZE)
    {
        next_head = 0;
    }

    if (next_head == Instance->RxTail)
    {
        return;
    }

    Instance->RxHead = next_head;

    Instance->RxBuf[Instance->RxHead].ID = ID;
    Instance->RxBuf[Instance->RxHead].DLC = DLC;
    Instance->RxBuf[Instance->RxHead].IDE_flag = IDE_flag;
    Instance->RxReceivedTick[Instance->RxHead] = CCX_GET_TICK;

    if ((DLC > 0) && (DLC <= 8))
    {
        CopyBuf(Data, Instance->RxBuf[Instance->RxHead].Data, DLC);
    }

    CCX_net_push(Instance, &Instance->RxBuf[Instance->RxHead]);
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
            if ((Instance->CCX_RX_table[i].DLC == Msg->DLC) && (Instance->CCX_RX_table[i].IDE_flag == Msg->IDE_flag))
            {
                Instance->CCX_RX_table[i].Parser(Instance, Msg, i);
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
                if (NULL != Instance->TimeoutCallback)
                {
                    Instance->TimeoutCallback(Instance, i);
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


void CCX_TX_PushMsg(CCX_instance_t *Instance, uint32_t ID, const uint8_t *Data, uint8_t DLC, uint8_t IDE_flag)
{
    assert(Instance != NULL);

    uint16_t next_head = Instance->TxHead + 1;
    if (next_head >= CCX_RX_BUFFER_SIZE)
    {
        next_head = 0;
    }

    if (next_head == Instance->TxTail)
    {
        return;
    }

    Instance->TxHead = next_head;

    Instance->TxBuf[Instance->TxHead].ID = ID;
    Instance->TxBuf[Instance->TxHead].DLC = DLC;
    Instance->TxBuf[Instance->TxHead].IDE_flag = IDE_flag;

    if ((DLC > 0) && (DLC <= 8))
    {
        CopyBuf(Data, Instance->TxBuf[Instance->TxHead].Data, DLC);
    }

    CCX_net_push(Instance, &Instance->TxBuf[Instance->TxHead]);
}


static inline void CCX_TX_MsgFromTables(CCX_instance_t *Instance)
{
    assert(NULL != Instance);
    uint8_t Tmp[8];

    for (uint16_t i = 0; i < Instance->TxTableSize; i++)
    {
        if (CCX_GET_TICK - Instance->CCX_TX_table[i].LastTick >= Instance->CCX_TX_table[i].SendFreq)
        {
            Instance->CCX_TX_table[i].LastTick = CCX_GET_TICK;
            CopyBuf(Instance->CCX_TX_table[i].Data, Tmp, Instance->CCX_TX_table[i].DLC);
            if (NULL != Instance->CCX_TX_table[i].Parser)
            {
                Instance->CCX_TX_table[i].Parser(Instance, Tmp, i);
            }
            CCX_TX_PushMsg(Instance, Instance->CCX_TX_table[i].ID, Tmp, Instance->CCX_TX_table[i].DLC,
                          Instance->CCX_TX_table[i].IDE_flag);
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

void CCX_Poll(CCX_instance_t *Instance)
{
	CCX_RX_Poll(Instance);
	CCX_TX_Poll(Instance);
}

void CCX_Init(CCX_instance_t *Instance, CCX_RX_table_t* CCX_RX_table, CCX_TX_table_t* CCX_TX_table, uint16_t RxTableSize,
		uint16_t TxTableSize, void (*SendFunction)(const CCX_instance_t *Instance, const CCX_message_t *msg),
		CCX_BusIsFree_t (*BusCheck)(const CCX_instance_t *Instance), void (*TimeoutCallback)(CCX_instance_t *Instance, uint16_t Slot),
		void (*ParserUnregMsg)(const CCX_instance_t *Instance, CCX_message_t *Msg))
{
	/* asserts here */
	Instance->CCX_TX_table = CCX_TX_table;
	Instance->CCX_RX_table = CCX_RX_table;
	Instance->RxTableSize = RxTableSize;
	Instance->TxTableSize = TxTableSize;
	Instance->SendFunction = SendFunction;
	Instance->BusCheck = BusCheck;
	Instance->TimeoutCallback = TimeoutCallback;
	Instance->Parser_unreg_msg = ParserUnregMsg;
}






