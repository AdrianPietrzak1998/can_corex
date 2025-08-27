/*
 * can_corex_net.c
 *
 *  Created on: Aug 26, 2025
 *      Author: Adrian
 */

#include "can_corex_net.h"
#include "string.h"


CCX_net_t* CCX_nets = NULL;

static CCX_net_status_t CCX_net_clear_nodes(CCX_net_t* net)
{
	if (NULL == net)
	{
		return CCX_NET_NULL;
	}

	for (uint16_t i = 0; i < CCX_MAX_INSTANCE_IN_NETWORK; i++)
	{
		net->NodeList[i].NodeInstance = NULL;
		net->NodeList[i].NodeSettings.Replication = CCX_NET_TX_REPLICATION;
	}

	return CCX_NET_OK;
}

static void CCX_net_TX_PushMsg(CCX_instance_t *Instance, const CCX_message_t* msg)
{
    assert(Instance != NULL);
    assert(msg != NULL);

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

    memcpy(&Instance->TxBuf[Instance->TxHead], msg, sizeof(CCX_message_t));
}

static void CCX_net_RX_PushMsg(CCX_instance_t *Instance, const CCX_message_t* msg)
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

CCX_net_status_t CCX_net_init(CCX_net_t* net)
{

	CCX_net_status_t status = CCX_NET_OK;

	if (NULL == net)
	{
		return CCX_NET_NULL;
	}

	status = CCX_net_clear_nodes(net);

	if (status != CCX_NET_OK)
	{
		return status;
	}

	if (CCX_nets == NULL)
	{
		CCX_nets = net;
		CCX_nets->next = NULL;

		status = CCX_NET_OK;
	}
	else
	{
        CCX_net_t* last = CCX_nets;
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


CCX_net_status_t CCX_net_deinit(CCX_net_t* net)
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

    CCX_net_t* prev = CCX_nets;
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

void CCX_net_push(const CCX_instance_t* Instance, const CCX_message_t* msg)
{
	if (CCX_nets == NULL)
	{
		return;
	}

	CCX_net_t* net = CCX_nets;
	do
	{
		for (uint16_t i = 0; i < CCX_MAX_INSTANCE_IN_NETWORK; i++)
		{
			if (Instance == net->NodeList[i].NodeInstance)
			{
				for (uint16_t j = 0; j < CCX_MAX_INSTANCE_IN_NETWORK; j++)
				{
					if ((Instance != net->NodeList[j].NodeInstance) && NULL != net->NodeList[j].NodeInstance)
					{
						switch(net->NodeList[j].NodeSettings.Replication)
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
	} while(net != NULL);

}












