/*
 * can_corex_net.h
 *
 *  Created on: Aug 26, 2025
 *      Author: Adrian
 */

#ifndef CAN_COREX_CAN_COREX_NET_H_
#define CAN_COREX_CAN_COREX_NET_H_

#include "can_corex.h"

#define CCX_MAX_INSTANCE_IN_NETWORK 12

typedef enum
{
	CCX_NET_OK = 0,
	CCX_NET_ALREDY_EXISTING,
	CCX_NET_DOES_NOT_EXISTING,
	CCX_NET_NULL
} CCX_net_status_t;

typedef enum
{
	CCX_NET_TX_REPLICATION = 0,
	CCX_NET_TX_RX_REPLICATION,
} CCX_net_replication_t;

typedef struct
{
	CCX_net_replication_t Replication;
} CCX_net_node_settings_t;

typedef struct
{
	CCX_instance_t* NodeInstance;
	CCX_net_node_settings_t NodeSettings;
} CCX_net_node_t;

typedef struct CCX_net_t CCX_net_t;

struct CCX_net_t
{
	CCX_net_node_t NodeList[CCX_MAX_INSTANCE_IN_NETWORK];
	CCX_net_t* next;
};



CCX_net_status_t CCX_net_init(CCX_net_t* net);
CCX_net_status_t CCX_net_deinit(CCX_net_t* net);




#endif /* CAN_COREX_CAN_COREX_NET_H_ */
