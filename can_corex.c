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
        return CCX_BUS_TOO_BUSY;
    }

    Instance->RxHead = next_head;

    memcpy(&Instance->RxBuf[Instance->RxHead], msg, sizeof(CCX_message_t));
    Instance->RxReceivedTick[Instance->RxHead] = CCX_GET_TICK; /* FIXED: populate timestamp */

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
            if ((Instance->CCX_RX_table[i].DLC == Msg->DLC) && (Instance->CCX_RX_table[i].IDE_flag == Msg->IDE_flag))
            {
                if (NULL != Instance->CCX_RX_table[i].Parser) /* FIXED: check before calling */
                {
                    Instance->CCX_RX_table[i].Parser(Instance, Msg, i);
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
                Instance->CCX_TX_table[i].Parser(Instance, Tmp, i);
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

    CCX_RX_Poll(Instance);
    CCX_TX_Poll(Instance);

    return CCX_OK;
}

CCX_Status_t CCX_Init(CCX_instance_t *Instance, CCX_RX_table_t *CCX_RX_table, CCX_TX_table_t *CCX_TX_table,
                      uint16_t RxTableSize, uint16_t TxTableSize,
                      void (*SendFunction)(const CCX_instance_t *Instance, const CCX_message_t *msg),
                      CCX_BusIsFree_t (*BusCheck)(const CCX_instance_t *Instance),
                      void (*TimeoutCallback)(CCX_instance_t *Instance, uint16_t Slot),
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
    Instance->TimeoutCallback = TimeoutCallback;
    Instance->Parser_unreg_msg = ParserUnregMsg;

    Instance->RxHead = 0;
    Instance->RxTail = 0;
    Instance->TxHead = 0;
    Instance->TxTail = 0;

    memset(Instance->RxBuf, 0, sizeof(Instance->RxBuf));
    memset(Instance->TxBuf, 0, sizeof(Instance->TxBuf));
    memset(Instance->RxReceivedTick, 0, sizeof(Instance->RxReceivedTick));

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

    return CCX_OK;
}