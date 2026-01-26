/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Author: Adrian Pietrzak
 * GitHub: https://github.com/AdrianPietrzak1998
 * Created: Aug 26, 2025
 */

#include "can_corex_net.h"
#include "string.h"
#include <assert.h>

/**
 * @brief Global list of all CAN networks
 *
 * Head of linked list containing all initialized networks.
 * NULL when no networks exist.
 */
CCX_net_t *CCX_nets = NULL;

CCX_net_status_t CCX_net_clear_nodes(CCX_net_t *net)
{
    if (NULL == net)
    {
        return CCX_NET_NULL;
    }

    for (uint16_t i = 0; i < CCX_MAX_INSTANCE_IN_NETWORK; i++)
    {
        net->NodeList[i].NodeInstance = NULL;
        net->NodeList[i].NodeSettings.Replication = CCX_NET_TX_REPLICATION;
        net->NodeList[i].NodeSettings.NodeType = CCX_NET_NODE_IN_NET;
    }

    return CCX_NET_OK;
}

/**
 * @brief Internal helper - Push message to TX buffer of a CAN instance
 *
 * Used by network replication to forward messages to other nodes.
 * Bypasses normal CCX_TX_PushMsg to avoid circular replication.
 *
 * @param Instance CAN instance to push to
 * @param msg Message to push
 */
static void CCX_net_TX_PushMsg(CCX_instance_t *Instance, const CCX_message_t *msg)
{
    assert(Instance != NULL);
    assert(msg != NULL);

    uint16_t next_head = Instance->TxHead + 1;
    if (next_head >= CCX_TX_BUFFER_SIZE)
    {
        next_head = 0;
    }

    if (next_head == Instance->TxTail)
    {
        return;
    }

    Instance->TxHead = next_head;

    memcpy(&Instance->TxBuf[Instance->TxHead], msg, sizeof(CCX_message_t));
}

/**
 * @brief Internal helper - Push message to RX buffer of a CAN instance
 *
 * Used by network replication for TX_RX_REPLICATION mode.
 * Bypasses normal CCX_RX_PushMsg to avoid circular replication.
 *
 * @param Instance CAN instance to push to
 * @param msg Message to push
 */
static void CCX_net_RX_PushMsg(CCX_instance_t *Instance, const CCX_message_t *msg)
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

    memcpy(&Instance->RxBuf[Instance->RxHead], msg, sizeof(CCX_message_t));
}

CCX_net_status_t CCX_net_init(CCX_net_t *net)
{

    CCX_net_status_t status = CCX_NET_OK;

    if (NULL == net)
    {
        return CCX_NET_NULL;
    }

    if (CCX_nets == NULL)
    {
        CCX_nets = net;
        CCX_nets->next = NULL;

        status = CCX_NET_OK;
    }
    else
    {
        CCX_net_t *last = CCX_nets;
        while (last->next != NULL)
        {
            if (last == net)
            {
                return CCX_NET_ALREDY_EXISTING;
            }
            last = last->next;
        }
        last->next = net;
        net->next = NULL;

        status = CCX_NET_OK;
    }

    return status;
}

CCX_net_status_t CCX_net_deinit(CCX_net_t *net)
{
    if (net == NULL)
    {
        return CCX_NET_NULL;
    }

    if (CCX_nets == net)
    {
        CCX_nets = CCX_nets->next;
        net->next = NULL;
        return CCX_NET_OK;
    }

    CCX_net_t *prev = CCX_nets;
    while (prev->next != NULL)
    {
        if (prev->next == net)
        {
            prev->next = net->next;
            net->next = NULL;
            return CCX_NET_OK;
        }
        prev = prev->next;
    }

    return CCX_NET_DOES_NOT_EXISTING;
}

/**
 * @brief Network message distribution function
 *
 * Called automatically by CCX_RX_PushMsg and CCX_TX_PushMsg to replicate
 * messages across network nodes.
 *
 * @param Instance Source CAN instance that received/transmitted the message
 * @param msg Message to distribute
 * @param FromTxFunc 1 if called from TX function, 0 if from RX function
 *
 * @note This function is called internally by CAN CoreX and should not be called directly
 */
void CCX_net_push(const CCX_instance_t *Instance, const CCX_message_t *msg, uint8_t FromTxFunc)
{
    if (CCX_nets == NULL)
    {
        return;
    }

    CCX_net_t *net = CCX_nets;
    do
    {
        for (uint16_t i = 0; i < CCX_MAX_INSTANCE_IN_NETWORK; i++)
        {
            if (Instance == net->NodeList[i].NodeInstance &&
                (!FromTxFunc || (CCX_NET_NODE_IN_NET == net->NodeList[i].NodeSettings.NodeType)))
            {
                for (uint16_t j = 0; j < CCX_MAX_INSTANCE_IN_NETWORK; j++)
                {
                    if ((Instance != net->NodeList[j].NodeInstance) && NULL != net->NodeList[j].NodeInstance)
                    {
                        switch (net->NodeList[j].NodeSettings.Replication)
                        {
                        case CCX_NET_TX_REPLICATION:
                            CCX_net_TX_PushMsg(net->NodeList[j].NodeInstance, msg);
                            break;
                        case CCX_NET_TX_RX_REPLICATION:
                            CCX_net_TX_PushMsg(net->NodeList[j].NodeInstance, msg);
                            CCX_net_RX_PushMsg(net->NodeList[j].NodeInstance, msg);
                            break;
                        }
                    }
                }

                break;
            }
        }
        net = net->next;
    } while (net != NULL);
}
