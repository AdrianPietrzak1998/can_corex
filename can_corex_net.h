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

/**
 * @def CCX_MAX_INSTANCE_IN_NETWORK
 * @brief Maximum number of CAN instances that can be connected to a single network
 */
#define CCX_MAX_INSTANCE_IN_NETWORK 12

/**
 * @brief Network operation status codes
 */
typedef enum
{
    CCX_NET_OK = 0,            /* Operation successful */
    CCX_NET_ALREDY_EXISTING,   /* Network already exists in global list */
    CCX_NET_DOES_NOT_EXISTING, /* Network not found in global list */
    CCX_NET_NULL               /* NULL pointer provided */
} CCX_net_status_t;

/**
 * @brief Message replication modes for network nodes
 *
 * Defines how messages from other nodes in the network are handled.
 *
 * @note Example:
 * Node A sends message → Node B receives it:
 * - TX_REPLICATION: Message appears only in Node B's TX buffer (forwarded to bus)
 * - TX_RX_REPLICATION: Message appears in both TX and RX buffers (forwarded + processed)
 */
typedef enum
{
    CCX_NET_TX_REPLICATION = 0, /* Forward to TX buffer only (physical layer forwarding) */
    CCX_NET_TX_RX_REPLICATION,  /* Forward to TX and RX buffers (act as if received locally) */
} CCX_net_replication_t;

/**
 * @brief Network node types
 *
 * Defines whether a node's transmitted messages propagate to the network.
 *
 * @note Example:
 * - NODE_IN_NET: Node A transmits → message reaches Node B, C, D
 * - NODE_REPEATER: Node A transmits → message stays local, but Node A still repeats
 *                  messages from B, C, D to its physical bus
 */
typedef enum
{
    CCX_NET_NODE_IN_NET = 0, /* Full participant - TX messages reach network */
    CCX_NET_NODE_REPEATER,   /* Repeater only - TX stays local, RX is repeated */
} CCX_net_node_type_t;

/**
 * @brief Network node settings
 */
typedef struct
{
    CCX_net_replication_t Replication; /* How to handle messages from other nodes */
    CCX_net_node_type_t NodeType;      /* Full participant vs repeater */
} CCX_net_node_settings_t;

/**
 * @brief Network node entry
 */
typedef struct
{
    CCX_instance_t *NodeInstance;         /* CAN instance for this node */
    CCX_net_node_settings_t NodeSettings; /* Node behavior configuration */
} CCX_net_node_t;

typedef struct CCX_net_t CCX_net_t;

/**
 * @brief CAN network structure
 *
 * Represents a virtual CAN network connecting multiple CAN instances.
 * Networks are managed as a global linked list.
 *
 * @note Architecture:
 * - Multiple networks can exist simultaneously (global linked list)
 * - Each network can have up to CCX_MAX_INSTANCE_IN_NETWORK nodes
 * - Messages are automatically replicated between nodes based on settings
 * - Network simulation happens in CCX_RX_PushMsg and CCX_TX_PushMsg
 *
 * @note Example - ECU Network Simulation:
 * @code
 * // Create 3 ECU instances
 * CCX_instance_t ecu_engine, ecu_transmission, ecu_dashboard;
 *
 * // Initialize network
 * CCX_net_t vehicle_network;
 * CCX_net_init(&vehicle_network);
 *
 * // Add ECUs to network
 * vehicle_network.NodeList[0].NodeInstance = &ecu_engine;
 * vehicle_network.NodeList[0].NodeSettings.Replication = CCX_NET_TX_RX_REPLICATION;
 * vehicle_network.NodeList[0].NodeSettings.NodeType = CCX_NET_NODE_IN_NET;
 *
 * vehicle_network.NodeList[1].NodeInstance = &ecu_transmission;
 * vehicle_network.NodeList[1].NodeSettings.Replication = CCX_NET_TX_RX_REPLICATION;
 * vehicle_network.NodeList[1].NodeSettings.NodeType = CCX_NET_NODE_IN_NET;
 *
 * vehicle_network.NodeList[2].NodeInstance = &ecu_dashboard;
 * vehicle_network.NodeList[2].NodeSettings.Replication = CCX_NET_TX_REPLICATION;
 * vehicle_network.NodeList[2].NodeSettings.NodeType = CCX_NET_NODE_IN_NET;
 *
 * // Now: engine sends RPM → transmission and dashboard receive it automatically
 * @endcode
 */
struct CCX_net_t
{
    CCX_net_node_t NodeList[CCX_MAX_INSTANCE_IN_NETWORK]; /* Array of nodes in this network */
    CCX_net_t *next;                                      /* Next network in global linked list */
};

/**
 * @brief Initialize a CAN network
 *
 * Adds the network to the global network list. Networks are managed as a linked list
 * to support multiple independent networks.
 *
 * @param net Pointer to network structure to initialize
 * @return CCX_NET_OK on success
 * @return CCX_NET_NULL if net is NULL
 * @return CCX_NET_ALREDY_EXISTING if network already in list
 *
 * @note Network node list is NOT cleared - call CCX_net_clear_nodes() first if needed
 */
CCX_net_status_t CCX_net_init(CCX_net_t *net);

/**
 * @brief Remove a CAN network from global list
 *
 * Removes the network from the global linked list. Does not affect node instances.
 *
 * @param net Pointer to network to remove
 * @return CCX_NET_OK on success
 * @return CCX_NET_NULL if net is NULL
 * @return CCX_NET_DOES_NOT_EXISTING if network not found in list
 */
CCX_net_status_t CCX_net_deinit(CCX_net_t *net);

/**
 * @brief Clear all nodes from a network
 *
 * Resets all node entries to NULL with default settings:
 * - NodeInstance: NULL
 * - Replication: CCX_NET_TX_REPLICATION
 * - NodeType: CCX_NET_NODE_IN_NET
 *
 * @param net Pointer to network to clear
 * @return CCX_NET_OK on success
 * @return CCX_NET_NULL if net is NULL
 *
 * @note This does NOT remove the network from the global list (use CCX_net_deinit for that)
 */
CCX_net_status_t CCX_net_clear_nodes(CCX_net_t *net);

#endif /* CAN_COREX_CAN_COREX_NET_H_ */
