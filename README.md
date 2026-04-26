# CAN CoreX

[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)
[![Version](https://img.shields.io/badge/Version-2.2.2-blue.svg)](#changelog)
[![Language: C](https://img.shields.io/badge/Language-C-blue.svg)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Platform: Embedded](https://img.shields.io/badge/Platform-Embedded-orange.svg)]()
[![Tests](https://img.shields.io/badge/Tests-622%20passing-success.svg)]()
[![ISO-TP validated with PEAK PCAN-ISO-TP API](https://img.shields.io/badge/ISO--TP-validated%20with%20PEAK%20PCAN--ISO--TP%20API-informational.svg)](#iso-tp)
[![GitHub stars](https://img.shields.io/github/stars/AdrianPietrzak1998/can_corex?style=social)](https://github.com/AdrianPietrzak1998/can_corex/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/AdrianPietrzak1998/can_corex?style=social)](https://github.com/AdrianPietrzak1998/can_corex/network/members)


## Overview

CAN CoreX is built around one practical idea: most CAN applications end up maintaining the same RX parsers, TX messages, timeouts, and periodic traffic by hand. This library moves that work into explicit RX/TX tables and a small polling model.

The main value of the library is:

- table-driven RX handling with parser callbacks
- table-driven periodic TX handling
- one place for message metadata, IDs, DLC, timeouts, and callbacks
- ISO-TP support for both classic CAN and CAN FD
- optional bus monitoring and runtime statistics

Version `2.2.2` keeps the `2.2.x` public API compatible and adds queue observability, queue flush/reset helpers, peak RX/TX queue usage statistics, stronger network replication overflow accounting, and safer ISO-TP TX enqueue error handling.

## Table of Contents

1. [Core CAN Model](#core-can-model)
2. [ISO-TP](#iso-tp)
3. [Bus Monitoring & Statistics](#bus-monitoring--statistics)
4. [Quick Start](#quick-start)
5. [API Reference](#api-reference)
6. [Error Codes](#error-codes)
7. [Data Structures](#data-structures)
8. [Usage Examples](#usage-examples)
9. [Advanced Build-Time Options](#advanced-build-time-options)
10. [Best Practices](#best-practices)
11. [Changelog](#changelog)

---

## Core CAN Model

The core of CAN CoreX is not a transport layer or diagnostics module. It is the RX/TX table model.

### What You Actually Work With

In a typical integration you define:

- an RX table: what IDs you accept, what DLC you expect, which parser handles the message, and whether a timeout matters
- a TX table: what messages should be sent periodically, how often, and whether data should be refreshed just before sending
- one `CCX_instance_t` per CAN controller or logical CAN channel

That gives you a clean split between:

- message definitions
- application logic
- hardware send/receive glue

### Why It Matters

Without this model, most projects gradually accumulate:

- hand-written `switch(ID)` parsing logic
- duplicated timeout handling
- ad-hoc periodic transmission code
- per-message state scattered across the application

With CAN CoreX, that state lives in the tables.

### Core Features

- **RX tables**: parser dispatch, optional timeout callback, wildcard DLC support when needed
- **TX tables**: periodic traffic with optional pre-send parser callback
- **Classic CAN and CAN FD**: one model, with strict `FrameFormat` matching in FD builds
- **Buffered operation**: RX/TX queues decouple hardware callbacks from the application loop
- **Runtime statistics**: message counters, parser calls, timeout calls, buffer overflows

### Minimal Example

```c
CCX_RX_table_t rx_table[] = {
    {.ID = 0x200, .DLC = 8, .IDE_flag = 0, .Parser = parse_status, .TimeOut = 1000, .TimeoutCallback = on_status_timeout}
};

CCX_TX_table_t tx_table[] = {
    {.ID = 0x300, .Data = heartbeat_data, .DLC = 2, .IDE_flag = 0, .SendFreq = 100, .Parser = update_heartbeat}
};

CCX_tick_variable_register(&system_tick_ms);
CCX_Init(&can1, rx_table, tx_table, 1, 1, hw_send_can_message, hw_can_bus_check, NULL);

while (1) {
    CCX_Poll(&can1);
}
```

The point is not fewer lines at all costs. The point is that message handling becomes explicit and centralized.

---

## ISO-TP

The second big reason to use this library is the ISO-TP layer.

### Why It Is Important Here

The library already gives you structured CAN message handling. ISO-TP extends that model to payloads that do not fit into a single CAN frame, without forcing a separate stack or a completely different integration style.

### What You Get

- ISO-TP for classic CAN and CAN FD
- TX and RX instances with explicit callbacks and state handling
- padding control
- `TxDL` / `FC_TxDL` support for FD sessions
- timeout diagnostics split by phase
- abort APIs
- support for sub-millisecond `STmin` when HR timing is enabled

### Practical Integration Model

You still work with:

- one CAN CoreX instance
- RX table entries for ISO-TP data or flow-control traffic
- one ISO-TP TX instance and/or one ISO-TP RX instance
- polling from the main loop

That makes ISO-TP feel like an extension of the same library, not a second subsystem bolted on from somewhere else.

ISO-TP interoperability was additionally validated with PEAK PCAN-ISO-TP API tooling.

---

## Bus Monitoring & Statistics

Bus monitoring is useful, but it is not the main reason to adopt the library. It is an operational layer on top of the core CAN model.

What it provides:

- bus state tracking: active, warning, passive, off
- TEC/REC tracking
- configurable recovery callback flow
- recovery delay helpers
- runtime counters and stats reset APIs

In `2.2.0`, timing in this area is also more explicit:

- `successful_run_time` uses the primary timebase
- `recovery_delay` can use HR only for short delays via `CCX_BUS_RECOVERY_US(...)`

Header organization in `2.2.1` and newer:

- `can_corex.h` remains the main public header
- `can_corex_bus.h` contains bus-monitoring types, macros, and API declarations
- including only `can_corex.h` remains sufficient for normal integrations

---

## Quick Start

Typical integration flow:

1. Register the primary tick source.
2. Initialize one `CCX_instance_t` with RX/TX tables and hardware callbacks.
3. Push RX frames from the ISR or driver callback.
4. Call `CCX_Poll()` regularly from the main loop or task.
5. If you use ISO-TP, also poll the ISO-TP TX/RX instances.

Minimal sketch:

```c
volatile uint32_t system_tick_ms = 0;
CCX_instance_t can1;

CCX_tick_variable_register(&system_tick_ms);

CCX_Init(&can1, rx_table, tx_table, RX_TABLE_SIZE, TX_TABLE_SIZE,
         hw_send_can_message, hw_can_bus_check, NULL);

while (1) {
    CCX_Poll(&can1);
    CCX_ISOTP_TX_Poll(&isotp_tx);
    CCX_ISOTP_RX_Poll(&isotp_rx);
}
```

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
- `ParserUnregMsg`: Callback for unregistered messages (optional)

**Returns**:
- `CCX_OK`: Initialization successful
- `CCX_NULL_PTR`: Invalid parameter combination

**Important Notes**:
- Must call `CCX_tick_variable_register()` BEFORE `CCX_Init()`
- Initializes all buffer pointers to zero
- Clears all buffers
- Sets LastTick in tables to current time
- In FD builds, does **not** zero `FrameFormat` in RX table entries - `CCX_FRAME_FORMAT_CLASSIC=0` is the natural C aggregate-initializer default; if using field-by-field assignment (no aggregate initializer), call `memset(rx_table, 0, sizeof(rx_table))` before populating

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
    NULL  /* ParserUnregMsg - optional unregistered message handler */
);
```

---

#### `CCX_FD_LenToDLC` / `CCX_MsgPayloadLen` (CAN FD builds only)

```c
uint8_t CCX_FD_LenToDLC(uint8_t len);
uint8_t CCX_MsgPayloadLen(const CCX_message_t *msg);
```

**`CCX_FD_LenToDLC`**: Converts a byte length (0-64) to the corresponding CAN FD DLC code (0-15).

**`CCX_MsgPayloadLen`**: Returns the actual payload length of a message in bytes (uses `DLC` directly for classic frames; applies the FD length table for FD frames).

**Example**:
```c
uint8_t dlc = CCX_FD_LenToDLC(48);    // returns 14 (CCX_FD_DLC_48B)
uint8_t len = CCX_MsgPayloadLen(&msg); // returns actual byte count
```

---

#### `CCX_RX_RebuildHash`

```c
void CCX_RX_RebuildHash(CCX_instance_t *Instance);
```

**Description**: Rebuilds the internal hash table for RX lookup when the hash mode is enabled.

**Parameters**:
- `Instance`: Pointer to CAN instance

**Important Notes**:
- Only functional when compiled with `-DCCX_RX_SEARCH_HASH`
- Hash table is automatically built during `CCX_Init()`
- Call this function only if you modify RX table after initialization
- Does nothing in linear or binary search modes

**When to use**:
```c
// Initial setup - hash built automatically
CCX_Init(&can_instance, rx_table, ...);

// Later: modify RX table at runtime
rx_table[5].ID = 0x456;
rx_table[5].Parser = new_parser;

// Rebuild hash to reflect changes
CCX_RX_RebuildHash(&can_instance);
```

---

#### Primary Tick Registration

```c
#if CCX_TICK_FROM_FUNC
void CCX_tick_function_register(CCX_TIME_BASE_SCALAR (*Function)(void));
#else
void CCX_tick_variable_register(CCX_TIME_t *Variable);
#endif
```

**Description**: Registers the primary system tick source used by core CAN RX/TX logic and all base-domain timeouts.

**Parameters**:
- `Function`: Callback returning the current primary tick when `CCX_TICK_FROM_FUNC=1`
- `Variable`: Pointer to a volatile primary tick variable when `CCX_TICK_FROM_FUNC=0`

**Important**: Must be called BEFORE `CCX_Init()`.

**Example**:
```c
volatile uint32_t system_tick_ms = 0;
CCX_tick_variable_register(&system_tick_ms);
```

Or with a callback:

```c
static uint32_t get_system_tick_ms(void)
{
    return system_tick_ms;
}

CCX_tick_function_register(get_system_tick_ms);
```

#### High-Resolution Tick Registration

When `CCX_DISABLE_HIGH_RES_TIMEBASE` is **not** defined, HR-aware parts of the library can use a separate high-resolution tick source.

```c
#if CCX_HR_TICK_FROM_FUNC
void CCX_high_res_tick_function_register(CCX_HR_TIME_BASE_SCALAR (*Function)(void));
#else
void CCX_high_res_tick_variable_register(CCX_HR_TIME_t *Variable);
#endif
```

**Important semantics**:
- `CCX_TICK_FROM_FUNC` and `CCX_HR_TICK_FROM_FUNC` are independent
- core CAN logic always uses the primary timebase
- ISO-TP uses HR only for sub-millisecond `STmin` values (`0xF1..0xF9`)
- bus monitor uses HR only when recovery delay is configured in `us` and the value is `<= 3000 us`
- `CCX_IsPrimaryTickRegistered()` and `CCX_IsHighResTickRegistered()` expose registration state for tests and diagnostics

#### Timebase Type Configuration

The library supports separate unsigned type selection for the primary and HR timebases:

- primary: `CCX_TIME_BASE_TYPE_CUSTOM_IS_UINT16`, `..._UINT32`, `..._UINT64`
- HR: `CCX_HR_TIME_BASE_TYPE_CUSTOM_IS_UINT16`, `..._UINT32`, `..._UINT64`

The following are intentionally rejected at compile time:

- `uint8_t` timebases: range is too small for this library
- signed timebase macros (`..._IS_INT8/16/32/64`): kept only as hard errors because they were an old bug and break timeout arithmetic

If no custom width macro is defined, both domains default to `uint32_t`.

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
- `CCX_WRONG_ARG`: Invalid DLC (> 8 in classic build; > 15 in FD build); also returned for FD frames pushed to a classic instance
- `CCX_BUS_TOO_BUSY`: Buffer is full

**Behavior**:
- Validates DLC (0-8 in classic; 0-15 in FD)
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
- `CCX_WRONG_ARG`: Invalid DLC (> 8 in classic build; > 15 in FD build); also returned for FD frames pushed to a classic instance
- `CCX_BUS_TOO_BUSY`: Buffer is full

**Behavior**:
- Validates DLC (0-8 in classic; 0-15 in FD)
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
    CCX_WRONG_ARG,        // Invalid argument (e.g., DLC > 8 in classic, DLC > 15 in FD)
    CCX_BUS_TOO_BUSY      // Buffer full, message dropped
} CCX_Status_t;
```

---

## Data Structures

### `CCX_message_t`

**Classic build** (`CCX_ENABLE_CANFD=0`, default):
```c
typedef struct {
    uint32_t ID;          // CAN message ID
    uint8_t Data[8];      // Message data (0-8 bytes)
    uint8_t DLC : 4;      // Data Length Code (0-8)
    uint8_t IDE_flag : 1; // 0=Standard, 1=Extended ID
} CCX_message_t;
```

**FD build** (`CCX_ENABLE_CANFD=1`):
```c
typedef struct {
    uint32_t ID;             // CAN message ID
    uint8_t Data[64];        // Message data (up to 64 bytes)
    uint8_t DLC : 4;         // DLC code (0-15; use CCX_FD_DLC_t constants)
    uint8_t IDE_flag : 1;    // use CCX_ide_t: CCX_ID_STANDARD / CCX_ID_EXTENDED
    uint8_t FrameFormat : 2; // use CCX_frame_format_t: CLASSIC / FD / FD_BRS
    uint8_t ESI : 1;         // Error State Indicator (set by RX hardware)
} CCX_message_t;
```

**`CCX_FD_DLC_t` enum** (FD build only) - named DLC constants:
```c
typedef enum {
    CCX_FD_DLC_0B  = 0,   CCX_FD_DLC_1B  = 1,
    CCX_FD_DLC_2B  = 2,   CCX_FD_DLC_3B  = 3,
    CCX_FD_DLC_4B  = 4,   CCX_FD_DLC_5B  = 5,
    CCX_FD_DLC_6B  = 6,   CCX_FD_DLC_7B  = 7,
    CCX_FD_DLC_8B  = 8,   CCX_FD_DLC_12B = 9,
    CCX_FD_DLC_16B = 10,  CCX_FD_DLC_20B = 11,
    CCX_FD_DLC_24B = 12,  CCX_FD_DLC_32B = 13,
    CCX_FD_DLC_48B = 14,  CCX_FD_DLC_64B = 15,
} CCX_FD_DLC_t;
```

---

### `CCX_RX_table_t`

**Classic build** (`CCX_ENABLE_CANFD=0`, default):
```c
typedef struct {
    uint32_t ID;          // Expected message ID
    uint8_t DLC : 4;      // Expected DLC (0-8); CCX_DLC_ANY (15) = accept any
    uint8_t IDE_flag : 1; // Expected ID type
    void *UserData;       // User context pointer passed to Parser and TimeoutCallback
    CCX_TIME_t TimeOut;   // Timeout period (0 = disabled)
    void (*Parser)(const CCX_instance_t *Instance,
                   CCX_message_t *Msg,
                   uint16_t Slot,
                   void *UserData);
    void (*TimeoutCallback)(CCX_instance_t *Instance, uint16_t Slot, void *UserData);
    CCX_TIME_t LastTick;  // Last receive time (auto-managed)
} CCX_RX_table_t;
```

**FD build** (`CCX_ENABLE_CANFD=1`):
```c
typedef struct {
    uint32_t ID;             // Expected message ID
    uint8_t DLC : 5;         // Expected DLC (0-15); CCX_DLC_ANY (16) = accept any
    uint8_t IDE_flag : 1;    // use CCX_ide_t: CCX_ID_STANDARD / CCX_ID_EXTENDED
    uint8_t FrameFormat : 2; // use CCX_frame_format_t: CLASSIC->match classic; FD/FD_BRS->match FD
    void *UserData;          // User context pointer passed to Parser and TimeoutCallback
    CCX_TIME_t TimeOut;      // Timeout period (0 = disabled)
    void (*Parser)(const CCX_instance_t *Instance,
                   CCX_message_t *Msg,
                   uint16_t Slot,
                   void *UserData);
    void (*TimeoutCallback)(CCX_instance_t *Instance, uint16_t Slot, void *UserData);
    CCX_TIME_t LastTick;     // Last receive time (auto-managed)
} CCX_RX_table_t;
```

**Usage**:
- Define expected messages with ID, DLC, and IDE_flag
- Set `TimeOut` to enable timeout detection (in ticks)
- Provide `Parser` callback to process matched messages
- Provide `TimeoutCallback` to handle timeout events (optional)
- `LastTick` is automatically updated by library
- In FD builds: `FrameFormat` defaults to `CCX_FRAME_FORMAT_CLASSIC=0` via C aggregate initialization (`{.ID=..., .DLC=...}`); if using field-by-field assignment, call `memset(rx_table, 0, sizeof(rx_table))` first
- **All** entries check `FrameFormat` - a CLASSIC entry will not match an FD frame and vice versa

---

### `CCX_TX_table_t`

**Classic build** (`CCX_ENABLE_CANFD=0`, default):
```c
typedef struct {
    uint32_t ID;          // Message ID to send
    uint8_t *Data;        // Pointer to data buffer
    uint8_t DLC : 4;      // Data length (0-8)
    uint8_t IDE_flag : 1; // ID type
    void *UserData;       // User context pointer passed to Parser
    CCX_TIME_t SendFreq;  // Send period in ticks
    void (*Parser)(const CCX_instance_t *Instance,
                   uint8_t *DataToSend,
                   uint16_t Slot,
                   void *UserData);
    CCX_TIME_t LastTick;  // Last send time (auto-managed)
} CCX_TX_table_t;
```

**FD build** (`CCX_ENABLE_CANFD=1`):
```c
typedef struct {
    uint32_t ID;             // Message ID to send
    uint8_t *Data;           // Pointer to data buffer (up to 64 bytes)
    uint8_t DLC : 4;         // DLC code (0-15)
    uint8_t IDE_flag : 1;    // use CCX_ide_t: CCX_ID_STANDARD / CCX_ID_EXTENDED
    uint8_t FrameFormat : 2; // use CCX_frame_format_t: CLASSIC / FD / FD_BRS
    void *UserData;          // User context pointer passed to Parser
    CCX_TIME_t SendFreq;     // Send period in ticks
    void (*Parser)(const CCX_instance_t *Instance,
                   uint8_t *DataToSend,
                   uint16_t Slot,
                   void *UserData);
    CCX_TIME_t LastTick;     // Last send time (auto-managed)
} CCX_TX_table_t;
```

**Usage**:
- Define periodic messages with ID, DLC, IDE_flag
- Set `SendFreq` to transmission period (in ticks)
- `Data` points to data buffer (must be at least `CCX_FD_DLC_TO_LEN[DLC]` bytes in FD builds)
- Optional `Parser` callback to update data before sending
- `LastTick` is automatically updated by library
- In FD builds: set `FrameFormat = CCX_FRAME_FORMAT_FD` or `CCX_FRAME_FORMAT_FD_BRS` for FD periodic frames

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
             hw_send_can_message, hw_can_bus_check, NULL);
    
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
                       uint16_t slot,
                       void *user_data) {
    (void)inst; (void)slot; (void)user_data;
    uint16_t sensor_value = (msg->Data[0] << 8) | msg->Data[1];
    process_sensor_value(sensor_value);
}

// Timeout callback
void sensor_timeout_handler(CCX_instance_t *inst, uint16_t slot, void *user_data) {
    (void)inst; (void)user_data;
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
        .Parser = parse_sensor_data,
        .TimeoutCallback = sensor_timeout_handler
    }
};

// Initialize with RX table
CCX_tick_variable_register(&system_tick_ms);
CCX_Init(&can1, rx_table, NULL, 1, 0,
         hw_send_can_message, hw_can_bus_check, NULL);
```

---

### TX Table for Periodic Messages

```c
// Data buffer for heartbeat message
uint8_t heartbeat_data[2] = {0x00, 0x00};

// Parser to update data before sending
void update_heartbeat(const CCX_instance_t *inst,
                      uint8_t *data,
                      uint16_t slot,
                      void *user_data) {
    (void)inst; (void)slot; (void)user_data;
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
         hw_send_can_message, hw_can_bus_check, NULL);

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
CCX_Init(&can1, NULL, NULL, 0, 0, hw_send_can1, hw_bus_check1, NULL);
CCX_Init(&can2, NULL, NULL, 0, 0, hw_send_can2, hw_bus_check2, NULL);

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

## ISO-TP Transport Protocol

CAN CoreX includes ISO 15765-2 (ISO-TP) for classic CAN and CAN FD transports.

### Features

- **Classic CAN instances**: Standard `SF` up to 7 bytes, standard `FF + CF` up to 4095 bytes
- **CAN FD instances**: Standard `SF` up to 7 bytes, extended `SF` for larger single-frame payloads, standard `FF` up to 4095 bytes, extended `FF` above 4095 bytes
- **Flow Control (FC)**: CTS/WAIT/OVFLW support
- **Configurable Padding**:
  - classic instances pad to `8`
  - FD instances pad to configured `TxDL`
- **Timeout Monitoring**: enforced `N_Bs`, `N_Cs`, `N_Cr`
- **Separated Timing Domains**:
  - `N_Bs`, `N_Cs`, `N_Cr` use the primary `ms` timebase
  - `STmin` values `0x00..0x7F` use the primary `ms` timebase
  - `STmin` values `0xF1..0xF9` (100-900 `us`) use the HR timebase when enabled
- **Lifecycle Hooks**: `OnReceiveStart`, `OnReceiveProgress`, `OnReceiveComplete`, `OnError`
- **Abort API**: `CCX_ISOTP_TX_Abort()` / `CCX_ISOTP_RX_Abort()`
- **Separate TX/RX Instances**: Independent transmit and receive handling
- **Extended ID Support**: Full support for both Standard (11-bit) and Extended (29-bit) CAN identifiers
- **Per-instance length limits**:
  - classic instance: `CCX_ISOTP_MAX_CLASSIC_DATA_SIZE` (`4095` by default)
  - FD instance: `CCX_ISOTP_MAX_FD_DATA_SIZE` (`UINT32_MAX` by default, bounded by application buffers)

### Important Semantics

- Header selection is based on ISO-TP payload length **before padding**
- Padding changes CAN frame payload length, not reported PDU length
- In FD builds, `CCX_ISOTP_Length_t` is `uint32_t`; in classic-only builds it remains `uint16_t`
- In FD builds, a classic ISO-TP instance still keeps the classic `4095`-byte limit
- `OnReceiveProgress` reports a delta since the previous callback, not an absolute offset
- To print `received/total`, accumulate `BytesReceived` yourself or read `Instance->RxDataOffset`; do not interpret `BytesReceived` as the total bytes received so far
- `N_Cr` measures inactivity between CF frames, not whether the next received CF is the expected one
- If one CF is lost but a later CF arrives before `N_Cr` expires, RX will typically end with `CCX_ISOTP_ERROR_SEQUENCE`, not `CCX_ISOTP_ERROR_TIMEOUT_CF_RX`
- `OnReceiveStart` fires once after a valid `FF` is accepted and the total payload length is known
- `CCX_ISOTP_ERROR_TIMEOUT` remains a legacy alias of `CCX_ISOTP_ERROR_TIMEOUT_FC`
- In dual-timebase builds, sub-millisecond `STmin` requires a registered HR tick source
- In single-timebase builds (`CCX_DISABLE_HIGH_RES_TIMEBASE`), RX configuration must not request sub-millisecond `STmin`; incoming FC values in the `0xF1..0xF9` range are still rounded up to the primary `ms` tick on TX

### Basic Usage

#### Transmitter Node (Sending ISO-TP Messages)

```c
#include "can_corex.h"
#include "can_corex_isotp.h"

// Initialize CAN CoreX
CCX_instance_t can;
CCX_tick_variable_register(&system_tick);

// Create ISO-TP TX instance
CCX_ISOTP_TX_t isotp_tx;

// RX table - only for receiving Flow Control frames
CCX_RX_table_t rx_table[] = {
    CCX_ISOTP_TX_FC_TABLE_ENTRY(&isotp_tx, 0x321, 0)   // Receive FC on 0x321 (Standard ID)
};

CCX_Init(&can, rx_table, NULL, 1, 0, hw_send, hw_bus_check, NULL);

// Configure ISO-TP TX
CCX_ISOTP_TX_Config_t tx_cfg;
CCX_ISOTP_TX_Config_Init(&tx_cfg, 0x123, CCX_ID_STANDARD, 1000, 1000, CCX_ISOTP_PADDING(0xAA), NULL, tx_complete_callback,
                         tx_error_callback);
tx_cfg.MaxWaitFrames = 10;     // Optional FC.WAIT tolerance (0 = library default)
CCX_ISOTP_TX_Init(&isotp_tx, &can, &tx_cfg);

// Transmit large message
uint8_t data[200];
// ... fill data ...
CCX_ISOTP_Transmit(&isotp_tx, data, 200);

// Main loop
while (1) {
    CCX_Poll(&can);              // Process CAN messages
    CCX_ISOTP_TX_Poll(&isotp_tx); // Handle ISO-TP TX state machine
}
```

#### CAN FD ISO-TP Example

Use the `_EX` table macros when the ISO-TP session uses FD frames:

```c
CCX_RX_table_t rx_table[] = {
    CCX_ISOTP_TX_FC_TABLE_ENTRY_EX(&isotp_tx, 0x321, 0, CCX_FRAME_FORMAT_FD_BRS)
};

CCX_ISOTP_TX_Config_t tx_cfg;
CCX_ISOTP_TX_Config_InitFD(&tx_cfg, 0x123, CCX_ID_STANDARD, CCX_FRAME_FORMAT_FD_BRS, CCX_ISOTP_TX_DL_64, 1000,
                           1000, CCX_ISOTP_PADDING(0xAA), NULL, tx_complete_callback, tx_error_callback);
tx_cfg.MaxWaitFrames = 10;
```

`TxDL` accepts only legal FD payload sizes: `8, 12, 16, 20, 24, 32, 48, 64`.

#### Receiver Node (Receiving ISO-TP Messages)

```c
#include "can_corex.h"
#include "can_corex_isotp.h"

// Initialize CAN CoreX
CCX_instance_t can;
CCX_tick_variable_register(&system_tick);

// Create ISO-TP RX instance
CCX_ISOTP_RX_t isotp_rx;
uint8_t rx_buffer[512];

// RX table - for receiving data frames
CCX_RX_table_t rx_table[] = {
    CCX_ISOTP_RX_TABLE_ENTRY(&isotp_rx, 0x123, 0)     // Receive data on 0x123 (Standard ID)
};

CCX_Init(&can, rx_table, NULL, 1, 0, hw_send, hw_bus_check, NULL);

// Configure ISO-TP RX
CCX_ISOTP_RX_Config_t rx_cfg;
CCX_ISOTP_RX_Config_Init(&rx_cfg, 0x321, CCX_ID_STANDARD, 0, 10, 1000, rx_buffer, sizeof(rx_buffer),
                         CCX_ISOTP_PADDING(0xAA), 0, NULL, NULL, rx_complete_callback, NULL, rx_error_callback);
CCX_ISOTP_RX_Init(&isotp_rx, &can, &rx_cfg);

// Main loop
while (1) {
    CCX_Poll(&can);              // Process CAN messages
    CCX_ISOTP_RX_Poll(&isotp_rx); // Handle ISO-TP RX timeouts
}
```

**Complete Callback Example**:
```c
void rx_complete_callback(CCX_ISOTP_RX_t *Instance, const uint8_t *Data, CCX_ISOTP_Length_t Length, void *UserData) {
    (void)Instance;
    (void)UserData;
    printf("Received %d bytes via ISO-TP\n", Length);
    // Process received data...
}

void tx_complete_callback(CCX_ISOTP_TX_t *Instance, void *UserData) {
    (void)Instance;
    (void)UserData;
    printf("Transmission complete!\n");
}
```

#### CAN FD ISO-TP Receiver Example

In FD sessions the RX-side `FC_TxDL` controls only transmitted Flow Control frames:

```c
CCX_ISOTP_RX_Config_t rx_cfg;
CCX_ISOTP_RX_Config_InitFD(&rx_cfg, 0x321, CCX_ID_STANDARD, CCX_FRAME_FORMAT_FD_BRS, CCX_ISOTP_TX_DL_64, 0, 0,
                           1000, rx_buffer, sizeof(rx_buffer), CCX_ISOTP_PADDING(0xAA), 512, NULL,
                           rx_start_callback, rx_complete_callback, rx_progress_callback, rx_error_callback);
```

### Flow Control Frame Layout (ISO 15765-2)

Flow Control frames follow the ISO 15765-2 standard layout:

```
Data[0] = 0x3N    (PCI=0x30 | Flow Status in lower nibble: 0=CTS, 1=WAIT, 2=OVFLW)
Data[1] = BS      (Block Size: 0=unlimited, 1-255=CF count before next FC)
Data[2] = STmin   (Separation Time minimum: 0-127ms or 0xF1-0xF9 for 100-900us)
```

Helper macros are available for the sub-millisecond range:

```c
CCX_ISOTP_STMIN_100US
CCX_ISOTP_STMIN_200US
CCX_ISOTP_STMIN_300US
CCX_ISOTP_STMIN_400US
CCX_ISOTP_STMIN_500US
CCX_ISOTP_STMIN_600US
CCX_ISOTP_STMIN_700US
CCX_ISOTP_STMIN_800US
CCX_ISOTP_STMIN_900US
```

Example:

```c
.STmin = CCX_ISOTP_STMIN_500US
```

In single-timebase builds (`CCX_DISABLE_HIGH_RES_TIMEBASE`), these helper macros still exist, but `CCX_ISOTP_RX_Init()` rejects sub-millisecond `STmin` values in RX configuration with `CCX_ISOTP_ERROR_INVALID_ARG`. Incoming FC values in the `0xF1..0xF9` range are still accepted on TX and rounded up to the primary `ms` tick.

FC frames respect the `Padding` configuration from RX:

```c
// With padding - FC will have DLC=8
.Padding = CCX_ISOTP_PADDING(0xAA)
// Example with BS=0, STmin=10: [30 00 0A AA AA AA AA AA]

// Without padding - FC will have DLC=3
.Padding = CCX_ISOTP_NO_PADDING
// Example with BS=0, STmin=10: [30 00 0A]
```

Both variants work correctly thanks to `CCX_DLC_ANY` wildcard matching in the RX table macros.

### Config Init Helpers

Use `CCX_ISOTP_TX_Config_Init()` and `CCX_ISOTP_RX_Config_Init()` to seed the config with the common defaults:

```c
CCX_ISOTP_TX_Config_t tx_cfg;
CCX_ISOTP_TX_Config_Init(&tx_cfg, 0x700, CCX_ID_STANDARD, 1000U, 1000U, CCX_ISOTP_NO_PADDING, user_ctx,
                         tx_complete_cb, tx_error_cb);

CCX_ISOTP_RX_Config_t rx_cfg;
CCX_ISOTP_RX_Config_Init(&rx_cfg, 0x701, CCX_ID_STANDARD, 0, 0, 1000U, rx_buffer, sizeof(rx_buffer),
                         CCX_ISOTP_NO_PADDING, 0U, user_ctx, rx_start_cb, rx_complete_cb, rx_progress_cb,
                         rx_error_cb);
```

These helpers now also accept `Padding`, and RX helpers accept `ProgressCallbackInterval`.

Defaults typically used by callers:

- TX: classic frame format, default `MaxWaitFrames`
- RX: classic frame format

Override fields directly after init when needed, for example `MaxWaitFrames`.

In FD-enabled builds, use `CCX_ISOTP_TX_Config_InitFD()` and `CCX_ISOTP_RX_Config_InitFD()` when the session itself
is FD, so `FrameFormat` and `TxDL` / `FC_TxDL` stay encapsulated in the helper call.

### Abort and Timeout Diagnostics

- `CCX_ISOTP_TX_Abort()` and `CCX_ISOTP_RX_Abort()` cancel an active transfer and reset the instance to `IDLE`
- Active aborts emit `OnError(..., CCX_ISOTP_ERROR_ABORTED, ...)`
- Timeout errors are phase-specific:
  - `CCX_ISOTP_ERROR_TIMEOUT_FC` for `N_Bs`
  - `CCX_ISOTP_ERROR_TIMEOUT_CF_TX` for `N_Cs`
  - `CCX_ISOTP_ERROR_TIMEOUT_CF_RX` for `N_Cr`
  - `CCX_ISOTP_ERROR_WAIT_EXCEEDED` for exhausted `FC.WAIT`
- `CCX_ISOTP_ERROR_TIMEOUT` is kept as a legacy alias of `CCX_ISOTP_ERROR_TIMEOUT_FC`

### Progress Monitoring

For large transfers, you can monitor reception progress:

```c
CCX_ISOTP_RX_Config_t rx_cfg = {
    // ... other config ...
    .ProgressCallbackInterval = 512,  // Call every 512 bytes
    .OnReceiveProgress = rx_progress_callback,
};

void rx_progress_callback(CCX_ISOTP_RX_t *Instance, CCX_ISOTP_Length_t BytesReceived,
                          CCX_ISOTP_Length_t TotalLength, void *UserData) {
    (void)Instance; (void)UserData;
    static CCX_ISOTP_Length_t accumulated = 0;
    accumulated += BytesReceived;  // BytesReceived is a delta from the previous callback
    printf("Progress: %u/%u bytes (%.1f%%)\n",
           (unsigned)accumulated, (unsigned)TotalLength,
           (100.0 * accumulated) / TotalLength);
}
```

Incorrect usage example:

```c
printf("Progress: %u/%u\n", (unsigned)(TotalLength - BytesReceived), (unsigned)TotalLength);
```

This is wrong because `BytesReceived` is a delta, so the output will jump around instead of showing monotonic progress.

### Extended ID Support

ISO-TP supports both Standard (11-bit) and Extended (29-bit) CAN identifiers.

**Standard ID (11-bit) - Range: 0x000 to 0x7FF**:
```c
CCX_RX_table_t rx_table[] = {
    CCX_ISOTP_RX_TABLE_ENTRY(&isotp_rx, 0x123, 0),     // Standard ID
    CCX_ISOTP_TX_FC_TABLE_ENTRY(&isotp_tx, 0x321, 0)   // Standard ID
};

CCX_ISOTP_TX_Config_t tx_cfg = {
    .TxID = 0x123,
    .IDE_TxID = 0,        // Standard ID
    // ... other config
};

CCX_ISOTP_RX_Config_t rx_cfg = {
    .TxID = 0x321,
    .IDE_TxID = 0,        // Standard ID
    // ... other config
};
```

**Extended ID (29-bit) - Range: 0x00000000 to 0x1FFFFFFF**:
```c
CCX_RX_table_t rx_table[] = {
    CCX_ISOTP_RX_TABLE_ENTRY(&isotp_rx, 0x18DA00F1, 1),     // Extended ID
    CCX_ISOTP_TX_FC_TABLE_ENTRY(&isotp_tx, 0x18DAF100, 1)   // Extended ID
};

CCX_ISOTP_TX_Config_t tx_cfg = {
    .TxID = 0x18DA00F1,
    .IDE_TxID = 1,        // Extended ID
    // ... other config
};

CCX_ISOTP_RX_Config_t rx_cfg = {
    .TxID = 0x18DAF100,
    .IDE_TxID = 1,        // Extended ID
    // ... other config
};
```

**Mixed Configuration** (TX uses Standard, RX uses Extended):
```c
CCX_RX_table_t rx_table[] = {
    CCX_ISOTP_RX_TABLE_ENTRY(&isotp_rx, 0x18DA00F1, 1),     // Extended ID
    CCX_ISOTP_TX_FC_TABLE_ENTRY(&isotp_tx, 0x321, 0)        // Standard ID
};

CCX_ISOTP_TX_Config_t tx_cfg = {
    .TxID = 0x123,
    .IDE_TxID = 0,        // Standard ID
    // ... other config
};

CCX_ISOTP_RX_Config_t rx_cfg = {
    .TxID = 0x18DAF100,
    .IDE_TxID = 1,        // Extended ID for FC
    // ... other config
};
```

### Limitations

- **Normal addressing only**: Extended and Mixed addressing modes not yet implemented
- **Single-timebase builds**: Submillisecond `STmin` values are rounded up to the primary `ms` tick

---

## Wildcard DLC Matching

CAN CoreX supports wildcard DLC matching in RX tables using `CCX_DLC_ANY`.

### Overview

By default, RX table entries match exact DLC values. Using `CCX_DLC_ANY` allows accepting messages with any DLC:

```c
CCX_RX_table_t rx_table[] = {
    // Exact match - only DLC=8
    {.ID = 0x100, .DLC = 8, .IDE_flag = 0, .Parser = my_parser},

    // Wildcard - accepts any DLC
    {.ID = 0x200, .DLC = CCX_DLC_ANY, .IDE_flag = 0, .Parser = my_parser}
};
```

### `CCX_DLC_ANY` value

| Build | `CCX_DLC_ANY` | Matches |
|-------|--------------|---------|
| Classic (`CCX_ENABLE_CANFD=0`) | 15 | Any DLC 0-8 |
| FD (`CCX_ENABLE_CANFD=1`) | 16 | Any DLC 0-15 (format filtered by `FrameFormat`) |

The sentinel value is always one above the maximum valid DLC for the build, so it can never collide with a real frame length code.

### Classic build wildcard

```c
CCX_message_t msg1 = {.ID = 0x200, .DLC = 3, .Data = {1, 2, 3}};
CCX_message_t msg2 = {.ID = 0x200, .DLC = 8, .Data = {1, 2, 3, 4, 5, 6, 7, 8}};

CCX_RX_PushMsg(&can, &msg1);  // Parser called (DLC=3 accepted)
CCX_RX_PushMsg(&can, &msg2);  // Parser called (DLC=8 accepted)
```

### FD build wildcard with frame-format filter

In FD builds, `CCX_DLC_ANY` combined with the `FrameFormat` field selects which frame format the wildcard matches:

```c
// Matches any FD frame (with BRS) on ID 0x300
CCX_RX_table_t rx_table[] = {
    {.ID = 0x100, .DLC = 8,           .FrameFormat = CCX_FRAME_FORMAT_CLASSIC, .Parser = classic_parser},
    {.ID = 0x300, .DLC = CCX_DLC_ANY, .FrameFormat = CCX_FRAME_FORMAT_FD_BRS,  .Parser = fd_brs_parser},
    {.ID = 0x400, .DLC = CCX_DLC_ANY, .FrameFormat = CCX_FRAME_FORMAT_CLASSIC, .Parser = classic_any_parser},
};
CCX_Init(&can, rx_table, NULL, 3, 0, send_fn, bus_fn, NULL);
// FrameFormat is part of the aggregate initializer - no post-Init fixup needed
```

`FrameFormat` uses the `CCX_frame_format_t` enum values:
- `CCX_FRAME_FORMAT_CLASSIC` (0) - matches classic (non-FD) frames only
- `CCX_FRAME_FORMAT_FD` (1) - matches FD frames (any BRS setting)
- `CCX_FRAME_FORMAT_FD_BRS` (2) - matches FD frames with BRS

### Exact-DLC entries

All entries check `FrameFormat` - a CLASSIC entry will not match an FD frame carrying the same DLC, and vice versa.

### Use Cases

1. **Higher-level protocols**: ISO-TP, J1939 where message DLC varies
2. **Padding flexibility**: Accept messages with or without padding
3. **Monitoring/debugging**: Capture all variants of a message ID
4. **FD/classic separation**: Different parsers for FD vs classic traffic on the same ID

### Important Notes

- ISO-TP macros automatically use `CCX_DLC_ANY` for flexibility
- In FD builds, DLC values 0-15 are all valid; only 16 is the wildcard sentinel
- `CCX_FRAME_FORMAT_CLASSIC=0` is the natural C aggregate-initializer default; field-by-field tables require `memset` before population


---

## Detailed Bus Management & Statistics

CAN CoreX includes comprehensive bus health monitoring and statistics tracking.

### Global Statistics

Always-on statistics tracking with minimal overhead. Automatically maintained by the library.

**Available Metrics:**
- `total_rx_messages` - Total messages received and pushed to RX buffer
- `total_tx_messages` - Total messages successfully transmitted (requires `CCX_OnMessageTransmitted()` call from ISR)
- `rx_buffer_overflows` - Number of times RX buffer was full
- `tx_buffer_overflows` - Number of times TX buffer was full
- `peak_rx_depth` - Highest observed RX queue depth since statistics reset
- `peak_tx_depth` - Highest observed TX queue depth since statistics reset
- `parser_calls_count` - Total parser function invocations
- `timeout_calls_count` - Total timeout callback invocations

**Usage:**
```c
// Get statistics
const CCX_GlobalStats_t *stats = CCX_GetGlobalStats(&can_instance);
printf("RX: %lu, TX: %lu, Overflows: %lu/%lu\n",
       stats->total_rx_messages,
       stats->total_tx_messages,
       stats->rx_buffer_overflows,
       stats->tx_buffer_overflows);
printf("Peak queue depth: RX=%u, TX=%u\n",
       stats->peak_rx_depth,
       stats->peak_tx_depth);

// Reset statistics
CCX_ResetGlobalStats(&can_instance);
```

**Important:** For accurate TX counting, call `CCX_OnMessageTransmitted()` from your CAN TX complete interrupt:
```c
void CAN_TX_IRQHandler(void) {
    // Clear interrupt flag
    // ...
    
    // Notify library
    CCX_OnMessageTransmitted(&can_instance, NULL);
}
```

### Queue State Helpers

The RX/TX queues can be inspected without touching queue internals:

```c
uint16_t rx_depth = CCX_RX_GetDepth(&can_instance);
uint16_t tx_depth = CCX_TX_GetDepth(&can_instance);
uint16_t rx_free = CCX_RX_GetFree(&can_instance);
uint16_t tx_free = CCX_TX_GetFree(&can_instance);
```

All four helpers return `0` for `NULL`.

Queue contents can also be cleared explicitly:

```c
CCX_FlushRx(&can_instance);  // clear only RX queue indices
CCX_FlushTx(&can_instance);  // clear only TX queue indices
CCX_Flush(&can_instance);    // clear RX and TX queues
CCX_Reset(&can_instance);    // clear RX/TX queues and reset global stats
```

`CCX_Flush*()` does not reset global statistics or ISO-TP session state. `CCX_Reset()` resets global statistics, including `peak_rx_depth` and `peak_tx_depth`.

### Bus Monitoring

Automatic bus-off detection and recovery with configurable retry strategy according to ISO 11898-1.

**Features:**
- Automatic bus state monitoring (Active/Warning/Passive/Off)
- TEC/REC error counter tracking
- Configurable recovery delay and retry attempts
- Grace period after max recovery attempts
- State transition callbacks
- Manual recovery trigger

**Bus States (ISO 11898-1):**
- `CCX_BUS_STATE_ACTIVE` - Normal operation (TEC < 96 && REC < 96)
- `CCX_BUS_STATE_WARNING` - Degraded performance (TEC > 96 || REC > 96)
- `CCX_BUS_STATE_PASSIVE` - Cannot send active error frames (TEC > 127 || REC > 127)
- `CCX_BUS_STATE_OFF` - Disconnected from bus (TEC > 255)

**Initialization:**
```c
CCX_BusMonitor_t bus_monitor;

CCX_BusMonitor_Init(
    &can_instance,
    &bus_monitor,
    my_get_bus_state,        // Read state from hardware
    my_get_error_counters,   // Read TEC/REC (optional)
    my_request_recovery,     // Trigger recovery
    CCX_BUS_RECOVERY_MS(10), // recovery_delay: 10ms between attempts
    60000,   // successful_run_time: 60s grace period
    1,       // auto_recovery_enabled
    5        // max_recovery_attempts before grace period
);

// Optional callbacks
bus_monitor.OnBusStateChange = my_state_callback;
bus_monitor.OnRecoveryAttempt = my_recovery_callback;
bus_monitor.OnRecoveryFailed = my_failed_callback;
bus_monitor.OnErrorCountersUpdate = my_counters_callback;
```

**Recovery Strategy:**

1. **Active Recovery Phase:**
   - Attempts recovery up to `max_recovery_attempts` times
   - Waits `recovery_delay` between attempts
   - Calls `OnRecoveryAttempt` before each try
   - Use `CCX_BUS_RECOVERY_MS(x)` for base-domain recovery delays
   - Use `CCX_BUS_RECOVERY_US(x)` for short delays that should use HR when `x <= 3000 us`

2. **Grace Period:**
   - After max attempts, waits `successful_run_time` before trying again
   - Calls `OnRecoveryFailed` when entering grace period
   - Prevents aggressive retry loops

3. **Counter Reset:**
   - After successful operation for `successful_run_time`, attempt counter resets to 0

**Recovery delay selection:**
- Classic CAN constants up to `250 kbps` use `CCX_BUS_RECOVERY_MS(...)`
- `500 kbps`, `800 kbps`, `1000 kbps`, and fast FD data-phase constants use `CCX_BUS_RECOVERY_US(...)`
- `CCX_BUS_RECOVERY_US(x)` uses HR only for `x <= 3000 us`; above that it falls back to base `ms`
- `successful_run_time` always uses the primary `ms` timebase

**Manual Recovery:**
```c
// Trigger recovery manually (resets counter and grace period)
CCX_Status_t status = CCX_BusMonitor_TriggerRecovery(&can_instance);
```

**Statistics:**
```c
// Access bus statistics
const CCX_BusStats_t *stats = &bus_monitor.stats;
printf("Bus-off count: %lu\n", stats->bus_off_count);
printf("TEC: %u, REC: %u\n", 
       stats->error_counters.TEC,
       stats->error_counters.REC);
printf("Peak TEC: %u, Peak REC: %u\n",
       stats->peak_error_counters.TEC,
       stats->peak_error_counters.REC);

// Reset bus monitoring statistics
CCX_BusMonitor_ResetStats(&can_instance);
```

**Hardware Interface Functions:**

You must implement three functions for your hardware:

```c
// Read current bus state
CCX_BusState_t my_get_bus_state(const CCX_instance_t *inst) {
    // Read from CAN controller registers
    // Return CCX_BUS_STATE_ACTIVE/WARNING/PASSIVE/OFF
}

// Read error counters (optional - can be NULL)
void my_get_error_counters(const CCX_instance_t *inst, CCX_ErrorCounters_t *cnt) {
    // Read TEC/REC from CAN controller
    cnt->TEC = /* read transmit error counter */;
    cnt->REC = /* read receive error counter */;
}

// Trigger bus-off recovery
void my_request_recovery(const CCX_instance_t *inst) {
    // Trigger recovery in CAN controller
    // e.g., clear bus-off bit, request re-initialization
}
```

**Callbacks:**
```c
void my_state_callback(CCX_instance_t *inst, 
                       CCX_BusState_t old_state,
                       CCX_BusState_t new_state,
                       void *user_data) {
    if (new_state == CCX_BUS_STATE_OFF) {
        // Bus-off occurred - recovery will start automatically
    } else if (new_state == CCX_BUS_STATE_ACTIVE && 
               old_state == CCX_BUS_STATE_OFF) {
        // Recovery successful
    }
}

void my_recovery_callback(CCX_instance_t *inst,
                          uint8_t attempt,
                          void *user_data) {
    printf("Recovery attempt %u\n", attempt);
}

void my_failed_callback(CCX_instance_t *inst, void *user_data) {
    printf("Max recovery attempts reached - entering grace period\n");
}
```

**Integration with CCX_Poll:**

Bus monitoring is automatic - just call `CCX_Poll()` as usual:
```c
while (1) {
    CCX_Poll(&can_instance);  // Automatically updates bus monitor
    // ...
}
```

---

## Advanced Build-Time Options

Most users should stay with the default build and only decide whether CAN FD is needed.

Main build switch:

- `CCX_ENABLE_CANFD=1` enables CAN FD message support, FD DLC helpers, FD-aware RX/TX tables, and FD ISO-TP sessions

Less common build-time options:

- `CCX_RX_SEARCH_BINARY` switches RX lookup to binary search and requires the RX table to stay sorted by CAN ID
- `CCX_RX_SEARCH_HASH` enables hashed RX lookup and `CCX_RX_RebuildHash()` for runtime RX table changes
- `CCX_TICK_FROM_FUNC` and `CCX_HR_TICK_FROM_FUNC` switch tick registration from variables to callbacks
- custom primary / HR tick widths can be selected with `CCX_TIME_BASE_TYPE_CUSTOM_IS_UINT16/32/64` and `CCX_HR_TIME_BASE_TYPE_CUSTOM_IS_UINT16/32/64`

Recommendation:

- treat RX lookup mode selection as an advanced optimization knob
- choose it only when you have a measured reason, not as part of the default integration path

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
rx_table[0].TimeoutCallback = critical_sensor_timeout_handler;
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

### Timeout Accuracy

- Timeout triggers when: `current_tick - LastTick >= TimeOut`
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

1. **Buffer Size**: Fixed at compile time (`CCX_RX_BUFFER_SIZE`, `CCX_TX_BUFFER_SIZE`)
2. **ISO-TP effective payload limit depends on instance format**:
   - classic instance: `CCX_ISOTP_MAX_CLASSIC_DATA_SIZE` (`4095` by default)
   - FD instance: `CCX_ISOTP_MAX_FD_DATA_SIZE` (`UINT32_MAX` by default, still bounded by application buffer sizes and available memory)
3. **Timeout Range**: Limited by `CCX_TIME_t` type (default: `uint32_t`)
4. **Not Thread-Safe**: Requires external synchronization in multi-threaded environments


---

## License

Mozilla Public License 2.0 - see LICENSE file for details.

## Author

**Adrian Pietrzak**
- GitHub: [@AdrianPietrzak1998](https://github.com/AdrianPietrzak1998)

## Changelog

### Current Release: v2.2.2 (2026-04-26)
- **Queue observability**:
  - added `CCX_RX_GetDepth()`, `CCX_TX_GetDepth()`, `CCX_RX_GetFree()`, and `CCX_TX_GetFree()`
  - added `peak_rx_depth` and `peak_tx_depth` to global statistics
- **Queue control**:
  - added `CCX_FlushRx()`, `CCX_FlushTx()`, `CCX_Flush()`, and `CCX_Reset()`
  - flush helpers clear queue indices without resetting statistics
  - `CCX_Reset()` clears RX/TX queues and global statistics
- **Network replication accounting**:
  - failed internal replication pushes now increment existing RX/TX overflow counters
  - successful internal replication updates target queue peak usage
- **ISO-TP robustness**:
  - TX enqueue failures for SF, FF, and CF now abort transmission, return/report failure, and raise `CCX_ISOTP_ERROR_BUSY`
  - RX initialization accepts `STmin` values `0xF1..0xF9` and advertises the raw value in Flow Control frames
  - TX behavior for sub-millisecond `STmin` in single-timebase builds remains rounded up to the nearest full millisecond
- **Breaking Changes**: None to existing function signatures, enum values, or initialization APIs

### Previous Release: v2.2.1 (2026-04-24)
- **Bus-monitoring header split**:
  - bus-monitoring public types, macros, and API declarations moved from `can_corex.h` to `can_corex_bus.h`
  - `can_corex.h` still includes `can_corex_bus.h`, so existing integrations that include only `can_corex.h` remain valid
- **Bus-monitoring implementation split**:
  - bus-monitoring runtime logic now lives in `can_corex_bus.c`
  - `can_corex.c` keeps core RX/TX runtime and global statistics
- **Documentation**:
  - README and implementation guide updated to reflect the new public module layout
- **Breaking Changes**: None for normal users of the public API

### Previous Release: v2.2.0 (2026-04-21)
- **Timebase contract cleanup**:
  - core CAN RX/TX logic now always uses the primary `ms` timebase
  - optional HR timing is a separate domain with its own type family and independent tick registration
  - `CCX_TICK_FROM_FUNC` and `CCX_HR_TICK_FROM_FUNC` are fully independent
  - `CCX_IsPrimaryTickRegistered()` and `CCX_IsHighResTickRegistered()` expose registration state
- **Timebase type restrictions**:
  - allowed custom widths are now `uint16_t`, `uint32_t`, `uint64_t`
  - `uint8_t` timebases are rejected because their range is too small for this library
  - old signed timebase selection macros are rejected with a hard compile-time error because they were a historical bug and break timeout arithmetic
- **ISO-TP timing cleanup**:
  - enforced ISO-TP runtime timers `N_Bs`, `N_Cs`, and `N_Cr` use the primary `ms` timebase
  - HR timing is used only for sub-millisecond `STmin` values (`0xF1..0xF9`)
  - in single-timebase builds, incoming FC `STmin` values in the `0xF1..0xF9` range are rounded up to the primary `ms` tick
- **Bus monitoring timing cleanup**:
  - `successful_run_time` now always uses the primary `ms` timebase
  - `recovery_delay` is now configured through `CCX_BUS_RECOVERY_MS(...)` or `CCX_BUS_RECOVERY_US(...)`
  - `CCX_BUS_RECOVERY_US(x)` uses HR only for `x <= 3000 us`; above that it falls back to base `ms`
  - classic recovery constants up to `250 kbps` are expressed in `ms`; faster classic/FD constants use precise `us` values where it matters
  - `500 kbps` recovery constant corrected to `2816 us`
- **Testing**:
  - added explicit bus recovery threshold coverage for `2999 us`, `3000 us`, and `3001 us`
  - current verified totals: classic linear `352`, FD linear `529`

### Previous Release: v2.1.0 (2026-04-19)
- **CAN FD Support** (`-DCCX_ENABLE_CANFD=1`): 64-byte payloads, BRS, ESI - zero overhead when disabled
  - `CCX_frame_format_t` (3 values): `CCX_FRAME_FORMAT_CLASSIC=0`, `CCX_FRAME_FORMAT_FD=1`, `CCX_FRAME_FORMAT_FD_BRS=2` - replaces separate `FDF:1 + BRS:1` bitfields; present in `CCX_message_t`, `CCX_RX_table_t`, `CCX_TX_table_t`; per-message/per-entry (not per-instance)
  - `CCX_ide_t` enum: `CCX_ID_STANDARD=0`, `CCX_ID_EXTENDED=1` - named constants for all `IDE_flag` fields
  - `CCX_RX_table_t` extended: 5-bit `DLC` field (sentinel `CCX_DLC_ANY=16`); `FrameFormat` controls which format the wildcard matches
  - **All** RX entries check `FrameFormat` - CLASSIC entry will not match FD frame and vice versa
  - `CCX_Init` does **not** zero `FrameFormat` - `CCX_FRAME_FORMAT_CLASSIC=0` is the C aggregate-init default; field-by-field tables require `memset(rx_table, 0, sizeof(rx_table))` before population
  - `CCX_Init_Ex()` **removed** - no per-instance frame format; use `CCX_Init()` for all instances
  - `CCX_FD_DLC_t` enum: named constants `CCX_FD_DLC_0B` ... `CCX_FD_DLC_64B`
  - `CCX_FD_LenToDLC()` / `CCX_MsgPayloadLen()`: helpers for FD length <-> DLC conversion
  - `CCX_FD_DLC_TO_LEN[16]` LUT: compile-time array mapping DLC codes to byte lengths
- **Network layer** (`can_corex_net`): no FD-to-classic frame filtering - FD frames replicate freely; `dropped_mixed` stat removed
- **ISO-TP** (`can_corex_isotp`):
  - unified classic/FD API
  - `FrameFormat` selects classic / FD / FD_BRS session behavior
  - TX uses `TxDL`; RX uses `FC_TxDL` for transmitted Flow Control frames
  - `TxDL` / `FC_TxDL` select FD link-layer payload size (`8/12/16/20/24/32/48/64`)
  - classic instances keep the standard `4095`-byte limit
  - FD instances support extended `SF`, extended `FF`, and payload lengths above `4095`
  - `CCX_ISOTP_Length_t` is `uint32_t` in FD builds
  - padding is format-aware: classic pads to `8`, FD pads to `TxDL`
  - `CCX_ISOTP_TX_Abort()` / `CCX_ISOTP_RX_Abort()` allow explicit transfer cancellation
  - `OnReceiveStart` fires after a valid `FF`; `OnReceiveProgress` reports deltas
  - phase-specific timeout diagnostics: `TIMEOUT_FC`, `TIMEOUT_CF_TX`, `TIMEOUT_CF_RX`, `WAIT_EXCEEDED`
  - RX configuration advertises `BS` and `STmin` through FC; TX enforces the values received in FC at runtime
- **Test suite**: 7 build flavors
  - classic linear / binary / hash
  - FD linear / binary / hash
  - optional FD 32-bit stress build for the full `UINT32_MAX` ISO-TP path
  - current verified totals: classic linear/binary `340`, FD linear/binary `516`, FD hash `520`
- **Breaking Changes** (FD builds only): `FDF`/`BRS` fields replaced by `FrameFormat`; `CCX_Init_Ex` removed; all RX entries now enforce FrameFormat matching
- **Breaking Changes** (FD ISO-TP users): ISO-TP length-related APIs now use `CCX_ISOTP_Length_t`
- **No breaking changes** for classic (`CCX_ENABLE_CANFD=0`) builds - fully backward compatible with v1.4.x

### Previous Release: v2.0.0 (2026-04-18)
- **CAN FD Support** (`-DCCX_ENABLE_CANFD=1`): 64-byte payloads, BRS, ESI support added to the CAN core
  - `CCX_frame_format_t` introduced with `CCX_FRAME_FORMAT_CLASSIC`, `CCX_FRAME_FORMAT_FD`, and `CCX_FRAME_FORMAT_FD_BRS`
  - `CCX_message_t`, `CCX_RX_table_t`, and `CCX_TX_table_t` extended for FD operation
  - strict `FrameFormat` matching added on RX entries
  - `CCX_FD_DLC_t`, `CCX_FD_LenToDLC()`, `CCX_MsgPayloadLen()`, and `CCX_FD_DLC_TO_LEN[16]` added for FD DLC handling
- **Network layer** (`can_corex_net`): FD frames can be replicated between instances; no FD-to-classic filtering in the network layer
- **ISO-TP status in 2.0.0**:
  - classic ISO-TP sessions on FD-capable instances were supported
  - FD payload ISO-TP was **not** implemented yet and returned `CCX_ISOTP_ERROR_FD_NOT_SUPPORTED`
- **Test suite**: first FD test coverage added alongside the existing classic builds
- **Breaking Changes** (FD builds only):
  - `FDF` / `BRS` bitfields replaced by `FrameFormat`
  - `CCX_Init_Ex()` removed
  - all RX entries began enforcing frame-format matching

### v1.4.4 (2026-04-01)
- **Bug Fixes**:
  - **ISO-TP N_Cs timeout logic (CRITICAL)**: Fixed N_Cs timeout check incorrectly nested inside STmin condition
    - When `STmin > N_Cs`, the timeout never fired - CF was sent before N_Cs could trigger
    - When `N_Cs = 0`, any elapsed tick immediately triggered a false timeout instead of sending CF
    - N_Cs is now checked independently of STmin, with `N_Cs == 0` treated as disabled
  - **ISO-TP FC.WAIT LastTick ordering (MODERATE)**: Fixed `LastTick` being updated after `OnError` callback when FC.WAIT frames are exhausted
    - If a new transmission was started inside the `OnError` callback, its `LastTick` was immediately overwritten
    - `LastTick` is now updated before invoking the callback
  - **Network cyclic list (CRITICAL)**: Fixed `CCX_net_init()` not checking the last node for duplicates
    - Re-adding the last node in the linked list set `node->next = node`, creating a cycle
    - Any subsequent `CCX_net_push()` call would then hang in an infinite loop
    - Added duplicate check for the final node after the traversal loop
- **Test Coverage**: Added 4 previously missing tests - 100% of public API now explicitly tested
  - `CCX_BusMonitor_GetState()` - all states (ACTIVE, WARNING, PASSIVE, OFF) and NULL safety
  - `CCX_BusMonitor_ResetStats()` - counter zeroing, state preservation, NULL safety
  - `CCX_RX_RebuildHash()` - dynamic table modification and hash rebuild in hash mode; no-op in other modes
  - ISO-TP 400-byte payload - multi-frame segmentation and data integrity for large transfers
- **Breaking Changes**: None - all changes are internal fixes, fully backward compatible
- **Testing**: 275/275 tests passing (linear, binary); 279/279 tests passing (hash)

### Previous Release: v1.4.3 (2026-02-13)
- **ISO-TP Flow Control Frame Fix (CRITICAL)**: Fixed FC frame layout to comply with ISO 15765-2
  - Flow Status now encoded in lower nibble of PCI byte (`Data[0] = 0x3N`) instead of separate `Data[1]`
  - Block Size moved from `Data[2]` to `Data[1]`, STmin moved from `Data[3]` to `Data[2]`
  - FC DLC without padding corrected from 4 to 3 bytes
  - **Previous versions were incompatible with all external ISO-TP implementations** (CANoe, PCAN, any standards-compliant ECU)
  - Internal loopback tests passed before because both TX and RX had the same bug
- **Custom Time Type Fix**: Fixed typedef `CC_TIME_t` -> `CCX_TIME_t` in `can_corex.h`
  - Any user defining `CCX_TIME_BASE_TYPE_CUSTOM` would get a compilation error in v1.4.0-v1.4.2
- **Header Cleanup**: Removed leftover `#define DCCX_RX_SEARCH_HASH` from `can_corex.h`
- **Network RX Replication Fix**: `CCX_net_RX_PushMsg` now sets `RxReceivedTick`
  - Timeout detection for messages received via network replication (`CCX_NET_TX_RX_REPLICATION`) was broken - `LastTick` was never updated because `RxReceivedTick` stayed at 0
- **New Test**: ISO 15765-2 FC frame layout validation with non-zero BS and STmin values
  - Verifies byte-level FC format, TX-side parsing of BS/STmin, and end-to-end transfer with BS=5
- **Breaking Changes**: ISO-TP FC wire format changed - **not backward compatible** with v1.2.0-v1.4.2 ISO-TP peers
  - Now compatible with all standards-compliant ISO-TP implementations
- **Testing**: 253/253 tests passing across all three search modes (linear, binary, hash)

### Previous Release: v1.4.2 (2026-02-10)
- **ISO-TP Protocol Fixes**: Critical timing implementation corrections
  - **STmin Implementation**: Fixed TX consecutive frame timing to use STmin from Flow Control instead of N_Cs
    - STmin (from FC) now correctly used as minimum delay between consecutive frames
    - N_Cs now properly used as timeout protection (maximum allowed time), not as delay
    - Fixes extremely slow transfers (256 bytes taking ~3 minutes instead of <1 second)
    - Complies with ISO 15765-2 specification for separation time handling
  - **Timeout Boundary Condition**: Fixed false timeout at exact N_Cs timing
    - Changed condition from `>=` to `>` for proper timeout detection
    - Allows full use of configured timeout window (e.g., 10ms timeout no longer triggers at exactly 10ms)
    - Fixes test failures in boundary conditions
  - **Time Type System**: Added `CCX_TIME_VALUE_t` typedef for non-volatile time values
    - Eliminates compiler warning: "type qualifiers ignored on function return type"
    - Preserves configurable time type system (uint8/16/32/64 support)
    - `CCX_TIME_t` (volatile) for tick variables, `CCX_TIME_VALUE_t` (non-volatile) for values/return types
    - Maintains type consistency across custom time base configurations
- **Breaking Changes**: None - all fixes are internal implementation corrections
- **Performance Impact**: Massive improvement in ISO-TP transfer speed (100-200x faster for typical configurations)
- **Testing**: All 241 tests passing, including previously failing ISO-TP boundary condition tests
- **Hardware Validation**: Confirmed working on STM32 CAN peripherals

### Previous Release: v1.4.1 (2026-02-05)
- **Bug-fix**
  - **Hash table size:** assert if RxTableSize >= CCX_RX_HASH_SIZE

### Previous Release: v1.4.0 (2026-02-05)
- **Compile-Time RX Lookup Strategies**: Configurable message search methods
  - **Linear Search** (default): O(n), no extra memory, best for < 15 messages
  - **Binary Search** (`-DCCX_RX_SEARCH_BINARY`): O(log n), requires sorted RX table, ideal for 15-50+ messages
  - **Hash Table** (`-DCCX_RX_SEARCH_HASH`): O(1) average, configurable size (default 64 entries/128 bytes), best for 30+ messages
  - Compile-time selection via preprocessor flags - no runtime overhead
- **Hash Table Management**:
  - `CCX_RX_RebuildHash()` - Rebuild hash after runtime RX table modifications
  - Automatic hash table initialization during `CCX_Init()`
  - Configurable hash size via `CCX_RX_HASH_SIZE` define (default: 64)
  - Linear probing collision resolution
- **Performance Improvements**:
  - Up to 10x faster RX message lookup for large tables (50+ messages)
  - Hash table provides consistent O(1) performance regardless of table size
  - Binary search provides 2-3x speedup over linear for medium tables (15-50 messages)
- **Memory Efficiency**:
  - Hash table overhead: only 2 bytes per hash entry (default: 128 bytes total)
  - Binary search: zero additional memory overhead
  - Linear search: unchanged, still the most compact option
- **API Additions**:
  - `CCX_RX_RebuildHash()` - Rebuild hash table (no-op for non-hash modes)
  - `CCX_RX_HASH_SIZE` define for hash table sizing
- **Documentation**:
  - Comprehensive RX lookup strategy guide with performance comparisons
  - Hash table sizing recommendations and best practices
  - Compilation examples for all three search modes
- **Testing**: All 241 tests passing across all three search modes
- **Breaking Changes**: None - fully backward compatible with v1.3.0
  - Default behavior unchanged (linear search)
  - Opt-in via compilation flags only

### Previous Release: v1.3.0 (2026-01-27)
- **Bus Management**: Automatic bus-off detection and recovery
  - Configurable retry strategy with grace period
  - TEC/REC error counter monitoring according to ISO 11898-1
  - State transition callbacks (OnBusStateChange, OnRecoveryAttempt, OnRecoveryFailed)
  - Manual recovery trigger via `CCX_BusMonitor_TriggerRecovery()`
  - Detailed statistics tracking (bus-off count, error states, total bus-off duration)
- **Global Statistics**: Always-on operational metrics
  - RX/TX message counters
  - Buffer overflow tracking
  - Parser/timeout call counters
  - `CCX_OnMessageTransmitted()` function for accurate TX counting from ISR
  - Minimal performance overhead
- **API Additions**:
  - `CCX_BusMonitor_Init()` - Initialize bus monitoring
  - `CCX_BusMonitor_TriggerRecovery()` - Manual recovery trigger
  - `CCX_BusMonitor_GetState()` - Get current bus state
  - `CCX_BusMonitor_ResetStats()` - Reset bus statistics
  - `CCX_GetGlobalStats()` - Get global statistics
  - `CCX_ResetGlobalStats()` - Reset global statistics
  - `CCX_OnMessageTransmitted()` - Notify library of successful transmission
- **Enhanced Testing**: 225/225 tests passing
  - Global statistics validation (RX/TX counters, buffer overflows)
  - Bus monitoring state machine tests
  - Auto-recovery and grace period verification
  - Manual recovery trigger tests
  - TEC/REC tracking validation
- **Breaking Changes**: None - fully backward compatible with v1.2.0

### Previous Release: v1.2.0 (2026-01-26)
- **UserData Context in Parsers**: Added `void *UserData` to RX/TX table structures
  - Enables multiple ISO-TP instances without global variables
  - Parser callbacks receive context pointer for flexible instance management
  - Breaking change: Parser signatures updated (added `void *UserData` parameter)
  - TimeoutCallback also receives UserData for consistency
- **ISO-TP Protocol**: Full ISO 15765-2 transport layer implementation
  - Single Frame and Multi-Frame support (up to 4095 bytes)
  - Flow Control with CTS/WAIT/OVFLW
  - Configurable padding and timeouts
  - Progress callbacks for large transfers
  - **Extended ID Support**: Full support for both Standard (11-bit) and Extended (29-bit) CAN identifiers
  - Multiple concurrent instances supported via UserData context
  - **UserData in Callbacks**: All ISO-TP callbacks now receive UserData parameter for context
    - Breaking change: All callback signatures updated (OnTransmitComplete, OnReceiveComplete, OnReceiveProgress, OnError)
    - Consistent with CAN CoreX parser/callback design pattern
- **Wildcard DLC Matching**: `CCX_DLC_ANY` constant for accepting any DLC in RX table
  - Enables flexible protocol handling (ISO-TP, J1939, etc.)
  - Useful for messages with variable padding
  - Comprehensive test coverage added
- **Enhanced Testing**: 143/143 tests passing
  - TimeoutCallback with UserData validation
  - CCX_DLC_ANY wildcard matching tests (all DLC values 0-8)
  - Extended ID support in basic CAN CoreX (Standard vs Extended ID filtering)
  - ISO-TP UserData callback verification
  - Full protocol compliance testing
  - Network initialization and message replication tests
- **Improved Documentation**: 
  - Detailed code comments for all ISO-TP helper functions
  - UserData usage examples in structure documentation
  - Complete API reference with practical code examples
  - Clear separation of Standard ID (11-bit) and Extended ID (29-bit) usage
  - **CAN Network (can_corex_net) fully documented**:
    - Complete API documentation for all functions
    - Network architecture explained (linked list, node types, replication modes)
    - Practical examples (ECU network simulation)
    - Added missing `CCX_net_clear_nodes()` to public API

### Previous Release: v1.1.0 (2026-01-22)
- **Per-message timeout callbacks**: `TimeoutCallback` moved from `CCX_instance_t` to `CCX_RX_table_t` for better flexibility
- **API Change**: `CCX_Init()` parameter count reduced from 9 to 8 (removed global `TimeoutCallback` parameter)
- **Documentation**: Updated all examples and best practices to reflect new timeout callback approach
- **Performance**: Reduced overhead - timeout callbacks now only called when needed per message

### Initial Release: v1.0.0 (2025-04-26)
- Initial release

---





