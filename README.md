# CAN CoreX

[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)
[![Version](https://img.shields.io/badge/Version-2.0.0-blue.svg)](CHANGELOG.md)
[![Language: C](https://img.shields.io/badge/Language-C-blue.svg)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Platform: Embedded](https://img.shields.io/badge/Platform-Embedded-orange.svg)]()
[![Tests](https://img.shields.io/badge/Tests-53%2F53%20passing-success.svg)]()
[![GitHub stars](https://img.shields.io/github/stars/AdrianPietrzak1998/can_corex.svg?style=social&label=Star)](https://github.com/AdrianPietrzak1998/can_corex)
[![GitHub forks](https://img.shields.io/github/forks/AdrianPietrzak1998/can_corex.svg?style=social&label=Fork)](https://github.com/AdrianPietrzak1998/can_corex/fork)

## Overview

CAN CoreX is a lightweight, modular CAN bus communication library designed for embedded systems. It provides buffer management, message routing, timeout detection, and network replication capabilities.

## Table of Contents

1. [Key Features](#key-features)
2. [API Reference](#api-reference)
3. [Error Codes](#error-codes)
4. [Data Structures](#data-structures)
5. [Usage Examples](#usage-examples)
6. [Best Practices](#best-practices)

---

## Key Features

- **Circular Buffer Management**: Efficient RX and TX message buffering
- **Error Handling**: Comprehensive return codes for all operations
- **Input Validation**: DLC and NULL pointer checks
- **Timeout Detection**: Configurable timeout monitoring for RX messages
- **Periodic Transmission**: Automatic periodic message sending via TX tables
- **Network Replication**: Multi-instance message routing and replication
- **Timestamp Tracking**: Automatic message receive time recording

---

## API Reference

### Initialization and Configuration

#### `CCX_Init`

```c
CCX_Status_t CCX_Init(
    CCX_instance_t *Instance,
    CCX_RX_table_t *CCX_RX_table,
    CCX_TX_table_t *CCX_TX_table,
    uint16_t RxTableSize,
    uint16_t TxTableSize,
    void (*SendFunction)(const CCX_instance_t *Instance, const CCX_message_t *msg),
    CCX_BusIsFree_t (*BusCheck)(const CCX_instance_t *Instance),
    void (*TimeoutCallback)(CCX_instance_t *Instance, uint16_t Slot),
    void (*ParserUnregMsg)(const CCX_instance_t *Instance, CCX_message_t *Msg)
);
```

**Description**: Initializes a CAN CoreX instance with specified tables and callbacks.

**Parameters**:
- `Instance`: Pointer to instance structure to initialize
- `CCX_RX_table`: Pointer to RX message table (can be NULL if RxTableSize is 0)
- `CCX_TX_table`: Pointer to TX message table (can be NULL if TxTableSize is 0)
- `RxTableSize`: Number of entries in RX table
- `TxTableSize`: Number of entries in TX table
- `SendFunction`: Callback to physically send messages (required if TxTableSize > 0)
- `BusCheck`: Callback to check if bus is free (required if TxTableSize > 0)
- `TimeoutCallback`: Callback when RX timeout occurs (optional)
- `ParserUnregMsg`: Callback for unregistered messages (optional)

**Returns**:
- `CCX_OK`: Initialization successful
- `CCX_NULL_PTR`: Invalid parameter combination

**Important Notes**:
- Must call `CCX_tick_variable_register()` BEFORE `CCX_Init()`
- Initializes all buffer pointers to zero
- Clears all buffers
- Sets LastTick in tables to current time

**Example**:
```c
CCX_instance_t can_instance;
CCX_RX_table_t rx_table[5];
CCX_TX_table_t tx_table[3];

// Register tick source first
CCX_tick_variable_register(&system_tick);

// Initialize instance
CCX_Status_t status = CCX_Init(
    &can_instance,
    rx_table, tx_table,
    5, 3,
    my_send_function,
    my_bus_check,
    my_timeout_callback,
    NULL
);
```

---

#### `CCX_tick_variable_register`

```c
void CCX_tick_variable_register(CCX_TIME_t *Variable);
```

**Description**: Registers the system tick variable used for timing.

**Parameters**:
- `Variable`: Pointer to volatile tick counter variable

**Important**: Must be called BEFORE `CCX_Init()`.

**Example**:
```c
volatile uint32_t system_tick_ms = 0;
CCX_tick_variable_register(&system_tick_ms);
```

---

### Message Operations

#### `CCX_RX_PushMsg`

```c
CCX_Status_t CCX_RX_PushMsg(
    CCX_instance_t *Instance,
    const CCX_message_t *msg
);
```

**Description**: Adds a received message to the RX buffer.

**Parameters**:
- `Instance`: Pointer to CAN instance
- `msg`: Pointer to message to add

**Returns**:
- `CCX_OK`: Message added successfully
- `CCX_NULL_PTR`: NULL pointer provided
- `CCX_WRONG_ARG`: Invalid DLC (> 8)
- `CCX_BUS_TOO_BUSY`: Buffer is full

**Behavior**:
- Validates DLC (must be 0-8)
- Records receive timestamp
- Triggers network replication if configured
- Returns error if buffer full (non-blocking)

**Example**:
```c
CCX_message_t msg = {
    .ID = 0x123,
    .DLC = 8,
    .IDE_flag = 0,
    .Data = {1, 2, 3, 4, 5, 6, 7, 8}
};

CCX_Status_t status = CCX_RX_PushMsg(&can_instance, &msg);
if (status == CCX_OK) {
    // Message accepted
} else if (status == CCX_BUS_TOO_BUSY) {
    // Buffer full, message dropped
}
```

---

#### `CCX_TX_PushMsg`

```c
CCX_Status_t CCX_TX_PushMsg(
    CCX_instance_t *Instance,
    const CCX_message_t *msg
);
```

**Description**: Adds a message to the TX buffer for transmission.

**Parameters**:
- `Instance`: Pointer to CAN instance
- `msg`: Pointer to message to transmit

**Returns**:
- `CCX_OK`: Message queued successfully
- `CCX_NULL_PTR`: NULL pointer provided
- `CCX_WRONG_ARG`: Invalid DLC (> 8)
- `CCX_BUS_TOO_BUSY`: Buffer is full

**Behavior**:
- Validates DLC (must be 0-8)
- Queues message for transmission
- Triggers network replication if configured
- Returns error if buffer full (non-blocking)

**Example**:
```c
CCX_message_t msg = {
    .ID = 0x456,
    .DLC = 4,
    .IDE_flag = 0,
    .Data = {0xAA, 0xBB, 0xCC, 0xDD}
};

CCX_Status_t status = CCX_TX_PushMsg(&can_instance, &msg);
```

---

#### `CCX_Poll`

```c
CCX_Status_t CCX_Poll(CCX_instance_t *Instance);
```

**Description**: Processes RX/TX buffers and performs timeout checks. Must be called periodically (e.g., in main loop or timer).

**Parameters**:
- `Instance`: Pointer to CAN instance

**Returns**:
- `CCX_OK`: Poll completed successfully
- `CCX_NULL_PTR`: NULL instance provided

**Behavior**:
1. Processes all messages in RX buffer
   - Matches against RX table
   - Calls parser callbacks
   - Updates LastTick on match
2. Checks for RX timeouts
3. Generates periodic TX messages from TX table
4. Sends queued TX messages if bus is free

**Example**:
```c
while (1) {
    CCX_Poll(&can_instance);
    // Other main loop tasks
}
```

---

## Error Codes

```c
typedef enum {
    CCX_OK = 0,           // Operation successful
    CCX_NULL_PTR,         // NULL pointer provided
    CCX_WRONG_ARG,        // Invalid argument (e.g., DLC > 8)
    CCX_BUS_TOO_BUSY      // Buffer full, message dropped
} CCX_Status_t;
```

---

## Data Structures

### `CCX_message_t`

```c
typedef struct {
    uint32_t ID;          // CAN message ID
    uint8_t Data[8];      // Message data (0-8 bytes)
    uint8_t DLC : 4;      // Data Length Code (0-8)
    uint8_t IDE_flag : 1; // 0=Standard, 1=Extended ID
} CCX_message_t;
```

---

### `CCX_RX_table_t`

```c
typedef struct {
    uint32_t ID;          // Expected message ID
    uint8_t DLC : 4;      // Expected DLC
    uint8_t IDE_flag : 1; // Expected ID type
    CCX_TIME_t TimeOut;   // Timeout period (0 = disabled)
    void (*Parser)(const CCX_instance_t *Instance, 
                   CCX_message_t *Msg, 
                   uint16_t Slot);
    CCX_TIME_t LastTick;  // Last receive time (auto-managed)
} CCX_RX_table_t;
```

**Usage**:
- Define expected messages with ID, DLC, and IDE_flag
- Set `TimeOut` to enable timeout detection (in ticks)
- Provide `Parser` callback to process matched messages
- `LastTick` is automatically updated by library

---

### `CCX_TX_table_t`

```c
typedef struct {
    uint32_t ID;          // Message ID to send
    uint8_t *Data;        // Pointer to data buffer
    uint8_t DLC : 4;      // Data length
    uint8_t IDE_flag : 1; // ID type
    CCX_TIME_t SendFreq;  // Send period in ticks
    void (*Parser)(const CCX_instance_t *Instance,
                   uint8_t *DataToSend,
                   uint16_t Slot);
    CCX_TIME_t LastTick;  // Last send time (auto-managed)
} CCX_TX_table_t;
```

**Usage**:
- Define periodic messages with ID, DLC, IDE_flag
- Set `SendFreq` to transmission period (in ticks)
- `Data` points to data buffer
- Optional `Parser` callback to update data before sending
- `LastTick` is automatically updated by library

---

## Usage Examples

### Basic Setup

```c
#include "can_corex.h"

// System tick (incremented by timer interrupt)
volatile uint32_t system_tick_ms = 0;

// CAN instance
CCX_instance_t can1;

// Callback: Send message to hardware
void hw_send_can_message(const CCX_instance_t *inst, const CCX_message_t *msg) {
    // Write to CAN hardware registers
    CAN->TX_ID = msg->ID;
    CAN->TX_DLC = msg->DLC;
    memcpy(CAN->TX_DATA, msg->Data, msg->DLC);
    CAN->TX_REQ = 1; // Trigger transmission
}

// Callback: Check if CAN bus is free
CCX_BusIsFree_t hw_can_bus_check(const CCX_instance_t *inst) {
    return (CAN->STATUS & CAN_TX_BUSY) ? CCX_BUS_BUSY : CCX_BUS_FREE;
}

int main(void) {
    // Initialize hardware
    can_hardware_init();
    
    // Register tick source
    CCX_tick_variable_register(&system_tick_ms);
    
    // Initialize CAN CoreX
    CCX_Init(&can1, NULL, NULL, 0, 0, 
             hw_send_can_message, hw_can_bus_check, NULL, NULL);
    
    while (1) {
        // Poll CAN library
        CCX_Poll(&can1);
        
        // Send a message
        CCX_message_t msg = {
            .ID = 0x100,
            .DLC = 2,
            .IDE_flag = 0,
            .Data = {0xAA, 0xBB}
        };
        CCX_TX_PushMsg(&can1, &msg);
        
        delay_ms(10);
    }
}

// Timer interrupt - increment tick
void SysTick_Handler(void) {
    system_tick_ms++;
}
```

---

### RX Table with Timeout

```c
// Parser callback for specific message
void parse_sensor_data(const CCX_instance_t *inst, 
                       CCX_message_t *msg, 
                       uint16_t slot) {
    uint16_t sensor_value = (msg->Data[0] << 8) | msg->Data[1];
    process_sensor_value(sensor_value);
}

// Timeout callback
void sensor_timeout_handler(CCX_instance_t *inst, uint16_t slot) {
    printf("Sensor timeout on slot %u!\n", slot);
    activate_failsafe_mode();
}

// Define RX table
CCX_RX_table_t rx_table[] = {
    {
        .ID = 0x200,
        .DLC = 2,
        .IDE_flag = 0,
        .TimeOut = 1000,  // 1000ms timeout
        .Parser = parse_sensor_data
    }
};

// Initialize with RX table
CCX_tick_variable_register(&system_tick_ms);
CCX_Init(&can1, rx_table, NULL, 1, 0,
         hw_send_can_message, hw_can_bus_check,
         sensor_timeout_handler, NULL);
```

---

### TX Table for Periodic Messages

```c
// Data buffer for heartbeat message
uint8_t heartbeat_data[2] = {0x00, 0x00};

// Parser to update data before sending
void update_heartbeat(const CCX_instance_t *inst,
                      uint8_t *data,
                      uint16_t slot) {
    static uint8_t counter = 0;
    data[0] = counter++;
    data[1] = get_system_status();
}

// Define TX table
CCX_TX_table_t tx_table[] = {
    {
        .ID = 0x100,
        .Data = heartbeat_data,
        .DLC = 2,
        .IDE_flag = 0,
        .SendFreq = 100,  // Send every 100ms
        .Parser = update_heartbeat
    }
};

// Initialize with TX table
CCX_Init(&can1, NULL, tx_table, 0, 1,
         hw_send_can_message, hw_can_bus_check, NULL, NULL);

// Heartbeat will be sent automatically by CCX_Poll()
```

---

### Network Replication

```c
// Two CAN instances on different physical buses
CCX_instance_t can1, can2;

// Network structure
CCX_net_t can_network;

// Initialize both instances
CCX_Init(&can1, NULL, NULL, 0, 0, hw_send_can1, hw_bus_check1, NULL, NULL);
CCX_Init(&can2, NULL, NULL, 0, 0, hw_send_can2, hw_bus_check2, NULL, NULL);

// Configure network
can_network.NodeList[0].NodeInstance = &can1;
can_network.NodeList[0].NodeSettings.Replication = CCX_NET_TX_RX_REPLICATION;
can_network.NodeList[0].NodeSettings.NodeType = CCX_NET_NODE_IN_NET;

can_network.NodeList[1].NodeInstance = &can2;
can_network.NodeList[1].NodeSettings.Replication = CCX_NET_TX_REPLICATION;
can_network.NodeList[1].NodeSettings.NodeType = CCX_NET_NODE_IN_NET;

// Initialize network
CCX_net_init(&can_network);

// Now messages received on can1 are automatically forwarded to can2
```

---

## Best Practices

### 1. Always Register Tick Source First

```c
// CORRECT
CCX_tick_variable_register(&system_tick);
CCX_Init(&can1, ...);

// WRONG - will cause undefined behavior
CCX_Init(&can1, ...);
CCX_tick_variable_register(&system_tick);
```

### 2. Handle Return Codes

```c
CCX_Status_t status = CCX_TX_PushMsg(&can1, &msg);
if (status == CCX_BUS_TOO_BUSY) {
    // Implement retry logic or drop message
    log_error("TX buffer full");
}
```

### 3. Call Poll Regularly

```c
// In main loop (non-RTOS)
while (1) {
    CCX_Poll(&can1);
    // Poll frequency should be >> highest TX frequency
}

// Or in timer (RTOS)
void timer_callback(void) {
    CCX_Poll(&can1);
}
```

### 4. Validate DLC Before Creating Messages

```c
uint8_t dlc = user_input;
if (dlc > 8) {
    dlc = 8;  // Clamp to maximum
}
msg.DLC = dlc;
```

### 5. Initialize Tables Properly

```c
// For TX table, ensure Data pointer is valid
static uint8_t data_buffer[8];  // Static or global, not stack!

CCX_TX_table_t tx_table[] = {
    {
        .Data = data_buffer,  // Valid for entire lifetime
        // ...
    }
};
```

### 6. Use Timeouts for Critical Messages

```c
// For safety-critical sensors, always enable timeout
rx_table[0].TimeOut = 100;  // 100ms max between messages
```

### 7. Buffer Sizing

```c
// Size buffers based on traffic:
// RX buffer >= max burst size
// TX buffer >= (max burst + periodic messages)

#define CCX_RX_BUFFER_SIZE 48  // e.g., 48 messages
#define CCX_TX_BUFFER_SIZE 48
```

---

## Timing Considerations

### Tick Resolution

- Tick source should match timing requirements
- For 1ms timeouts, use 1ms tick
- Higher resolution = more precise timeouts

### Poll Frequency

- Call `CCX_Poll()` at least 2x faster than shortest timeout
- Example: 100ms timeout → poll every 50ms or faster

### Timeout Accuracy

- Timeout triggers when: `current_tick - LastTick >= TimeOut`
- Actual timeout = TimeOut ± poll_period
- For 100ms timeout with 10ms poll: actual = 100-110ms

---

## Thread Safety

CAN CoreX is **not thread-safe** by default. If using RTOS:

### Option 1: Single Thread Access
```c
void can_task(void *param) {
    while (1) {
        CCX_Poll(&can1);
        vTaskDelay(10);
    }
}
```

### Option 2: Mutex Protection
```c
SemaphoreHandle_t can_mutex;

void user_code(void) {
    xSemaphoreTake(can_mutex, portMAX_DELAY);
    CCX_TX_PushMsg(&can1, &msg);
    xSemaphoreGive(can_mutex);
}

void can_task(void *param) {
    while (1) {
        xSemaphoreTake(can_mutex, portMAX_DELAY);
        CCX_Poll(&can1);
        xSemaphoreGive(can_mutex);
        vTaskDelay(10);
    }
}
```

---

## Limitations

1. **Buffer Size**: Fixed at compile time (CCX_RX_BUFFER_SIZE, CCX_TX_BUFFER_SIZE)
2. **DLC Range**: 0-8 only (CAN 2.0 standard)
3. **Timeout Range**: Limited by CCX_TIME_t type (default: uint32_t)
4. **Not Thread-Safe**: Requires external synchronization in multi-threaded environments

---

## License

Mozilla Public License 2.0 - see LICENSE file for details.

## Author

**Adrian Pietrzak**
- GitHub: [@AdrianPietrzak1998](https://github.com/AdrianPietrzak1998)

## Changelog

### Previous Release: v1.0.0 (2025-04-26)
- 🎉 Initial release
---
