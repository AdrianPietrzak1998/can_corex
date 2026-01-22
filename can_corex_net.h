/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Author: Adrian Pietrzak
 * GitHub: https://github.com/AdrianPietrzak1998
 * Created: Aug 26, 2025
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
    CCX_NET_TX_REPLICATION =
        0, /* If another node in the network receives a message, that node will only forward the message on its TX. */
    CCX_NET_TX_RX_REPLICATION, /* If another node in the network receives a message, that node will forward it on its TX
                                  and also treat it as received by itself. */
} CCX_net_replication_t;

typedef enum
{
    CCX_NET_NODE_IN_NET = 0, /* Messages sent by this node reach the network as if it were connected to the network. */
    CCX_NET_NODE_REPEATER, /* Messages sent by this node are transmitted only through the physical layer assigned to it
                              and do not affect the network it is connected to, but the network also makes use of its
                              physical layer. */
} CCX_net_node_type_t;

typedef struct
{
    CCX_net_replication_t Replication;
    CCX_net_node_type_t NodeType;
} CCX_net_node_settings_t;

typedef struct
{
    CCX_instance_t *NodeInstance;
    CCX_net_node_settings_t NodeSettings;
} CCX_net_node_t;

typedef struct CCX_net_t CCX_net_t;

struct CCX_net_t
{
    CCX_net_node_t NodeList[CCX_MAX_INSTANCE_IN_NETWORK];
    CCX_net_t *next;
};

CCX_net_status_t CCX_net_init(CCX_net_t *net);
CCX_net_status_t CCX_net_deinit(CCX_net_t *net);

#endif /* CAN_COREX_CAN_COREX_NET_H_ */