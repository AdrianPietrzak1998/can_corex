# CAN CoreX Implementation Guide

**Version:** 1.3.0  
**Author:** Adrian Pietrzak  
**Date:** January 2026

---

## Table of Contents

1. [Recommended File Structure](#recommended-file-structure)
2. [Platform: STM32 HAL (CAN 2.0)](#platform-stm32-hal-can-20)
   - [1.1 Basic TX/RX Implementation](#11-basic-txrx-implementation)
   - [1.2 Bus Monitoring & Statistics](#12-bus-monitoring--statistics)
3. [Platform: STM32 HAL (FDCAN as CAN 2.0)](#platform-stm32-hal-fdcan-as-can-20)
   - [2.1 Basic TX/RX Implementation](#21-basic-txrx-implementation-1)
   - [2.2 Bus Monitoring & Statistics](#22-bus-monitoring--statistics-1)
4. [Platform: TI Connectivity Manager (CAN 2.0)](#platform-ti-connectivity-manager-can-20)
   - [3.1 Basic TX/RX Implementation](#31-basic-txrx-implementation)
   - [3.2 Bus Monitoring & Statistics](#32-bus-monitoring--statistics)

---

## Recommended File Structure

For maintainable CAN application code, we recommend splitting your implementation into the following files:

```
project/
├── can_app/
│   ├── can_app.c                   # Main CAN initialization and routine
│   ├── can_app.h                   # Public API
│   ├── can_app_frame_types.h      # CAN frame structures (unions)
│   ├── can_app_frames_api.h       # External declarations for frame instances
│   ├── can_app_msg_rx.c           # RX table and parsers
│   ├── can_app_msg_rx.h           # RX table declarations
│   ├── can_app_msg_tx.c           # TX table and frame instances
│   └── can_app_msg_tx.h           # TX table declarations
└── can_corex/                      # CAN CoreX library
    ├── can_corex.c
    ├── can_corex.h
    ├── can_corex_net.c
    ├── can_corex_net.h
    ├── can_corex_isotp.c
    └── can_corex_isotp.h
```

### File Relationships Diagram

```
┌─────────────────┐
│   can_app.c     │  ◄── Main application entry point
└────────┬────────┘
         │ includes
         ├──────────────────┬──────────────────┐
         ▼                  ▼                  ▼
┌──────────────┐   ┌──────────────┐   ┌──────────────┐
│can_app_msg_  │   │can_app_msg_  │   │can_app_frames│
│   rx.c/.h    │   │   tx.c/.h    │   │   _api.h     │
└──────┬───────┘   └──────┬───────┘   └──────┬───────┘
       │ includes         │ includes         │ includes
       └──────────────────┴──────────────────┘
                          │
                          ▼
                 ┌─────────────────┐
                 │can_app_frame_   │
                 │   types.h       │  ◄── Frame definitions
                 └─────────────────┘
                          │
                          │ used by
                          ▼
                 ┌─────────────────┐
                 │  can_corex.h    │  ◄── CAN CoreX library
                 └─────────────────┘
```

**Key principles:**
- `can_app_frame_types.h` contains **only** frame structure definitions (unions)
- `can_app_frames_api.h` contains **extern** declarations for global frame instances
- `can_app_msg_tx.c` contains frame **instances** and TX table
- `can_app_msg_rx.c` contains RX table and parser **implementations**
- `can_app.c` orchestrates initialization and polling

---

## Platform: STM32 HAL (CAN 2.0)

### Prerequisites

- STM32 HAL library
- CAN peripheral configured in STM32CubeMX
- **Required interrupts enabled in NVIC:**
  - `CAN1_RX0_IRQn` (FIFO0 reception)
  - `CAN1_RX1_IRQn` (FIFO1 reception) - if using CAN1
  - `CAN2_RX0_IRQn` (FIFO0 reception) - if using CAN2
  - `CAN2_RX1_IRQn` (FIFO1 reception) - if using CAN2
- System tick configured (SysTick or any timer incrementing tick variable)

---

### 1.1 Basic TX/RX Implementation

This example demonstrates:
- Two independent CAN instances (CAN1 and CAN2)
- Standard ID (11-bit) and Extended ID (29-bit) support
- Separate interrupt handlers for each CAN peripheral
- Basic message transmission and reception

#### Step 1: Frame Type Definitions

**File: `can_app_frame_types.h`**

```c
#ifndef CAN_APP_FRAME_TYPES_H_
#define CAN_APP_FRAME_TYPES_H_

#include <stdint.h>

/* Example frame structures */

typedef union {
    uint8_t frame[8];
    struct {
        uint16_t speed;
        uint16_t torque;
        uint8_t  status;
        uint8_t  temperature;
        uint16_t reserved;
    } __attribute__((packed));
} CAN_MotorStatus_t;

typedef union {
    uint8_t frame[8];
    struct {
        uint8_t  enable : 1;
        uint8_t  direction : 1;
        uint8_t  reserved : 6;
        uint16_t speed_setpoint;
        uint16_t torque_limit;
        uint8_t  mode;
    } __attribute__((packed));
} CAN_MotorControl_t;

typedef union {
    uint8_t frame[8];
    struct {
        uint32_t timestamp;
        uint16_t voltage;
        uint16_t current;
    } __attribute__((packed));
} CAN_PowerMeasurement_t;

typedef union {
    uint8_t frame[8];
    struct {
        uint16_t voltage_bat1;
        uint16_t voltage_bat2;
        uint16_t voltage_bat3;
        int16_t  temperature;
    } __attribute__((packed));
} CAN_BatteryVoltage_t;

typedef union {
    uint8_t frame[8];
    struct {
        uint32_t error_flags;
        uint16_t warning_flags;
        uint8_t  system_state;
        uint8_t  reserved;
    } __attribute__((packed));
} CAN_ErrorFlags_t;

typedef union {
    uint8_t frame[8];
    struct {
        uint8_t  command_type;
        uint16_t param1;
        uint16_t param2;
        uint8_t  config_byte;
        uint16_t reserved;
    } __attribute__((packed));
} CAN_BatteryCommand_t;

typedef union {
    uint8_t frame[8];
    struct {
        uint16_t config_id;
        uint32_t config_value;
        uint16_t checksum;
    } __attribute__((packed));
} CAN_ConfigMessage_t;

typedef union {
    uint8_t frame[8];
    struct {
        uint32_t debug_value1;
        uint32_t debug_value2;
    } __attribute__((packed));
} CAN_DebugMessage_t;

/* Extended ID frame examples */
typedef union {
    uint8_t frame[8];
    struct {
        uint8_t  module_id;
        uint8_t  command;
        uint16_t parameter1;
        uint16_t parameter2;
        uint16_t checksum;
    } __attribute__((packed));
} CAN_ExtendedCommand_t;

typedef union {
    uint8_t frame[8];
    struct {
        uint8_t  response_code;
        uint8_t  data_length;
        uint8_t  data[6];
    } __attribute__((packed));
} CAN_DiagnosticResp_t;

typedef union {
    uint8_t frame[8];
    struct {
        uint16_t status_word;
        uint16_t counter;
        uint32_t timestamp;
    } __attribute__((packed));
} CAN_StatusWord_t;

typedef union {
    uint8_t frame[8];
    struct {
        uint8_t  device_id;
        uint8_t  status_flags;
        uint16_t error_code;
        uint32_t uptime;
    } __attribute__((packed));
} CAN_ExtendedStatus_t;

typedef union {
    uint8_t frame[8];
    struct {
        uint16_t response_id;
        uint8_t  result_code;
        uint8_t  data[5];
    } __attribute__((packed));
} CAN_StandardResponse_t;

#endif /* CAN_APP_FRAME_TYPES_H_ */
```

#### Step 2: Frame API Declarations

**File: `can_app_frames_api.h`**

```c
#ifndef CAN_APP_FRAMES_API_H_
#define CAN_APP_FRAMES_API_H_

#include "can_app_frame_types.h"

/* TX frame instances - defined in can_app_msg_tx.c */
extern CAN_MotorStatus_t      CAN_MotorStatus;
extern CAN_PowerMeasurement_t CAN_PowerMeasurement;
extern CAN_BatteryVoltage_t   CAN_BatteryVoltage;
extern CAN_ErrorFlags_t       CAN_ErrorFlags;
extern CAN_ExtendedCommand_t  CAN_ExtCommand;
extern CAN_DiagnosticResp_t   CAN_DiagnosticResp;
extern CAN_StatusWord_t       CAN_StatusWord;

/* RX frame instances - defined in can_app_msg_rx.c */
extern CAN_MotorControl_t     CAN_MotorControl;
extern CAN_BatteryCommand_t   CAN_BatteryCommand;
extern CAN_ConfigMessage_t    CAN_ConfigMessage;
extern CAN_DebugMessage_t     CAN_DebugMessage;
extern CAN_ExtendedStatus_t   CAN_ExtendedStatus;
extern CAN_StandardResponse_t CAN_StandardResponse;

#endif /* CAN_APP_FRAMES_API_H_ */
```

#### Step 3: TX Message Table

**File: `can_app_msg_tx.h`**

```c
#ifndef CAN_APP_MSG_TX_H_
#define CAN_APP_MSG_TX_H_

#include "can_corex.h"
#include <stdint.h>

/* TX table indices for CAN1 */
enum {
    CAN1_TX_MOTOR_STATUS = 0,
    CAN1_TX_POWER_MEAS,
    CAN1_TX_BATTERY_VOLTAGE,
    CAN1_TX_ERROR_FLAGS,
    CAN1_TX_END
};

/* TX table indices for CAN2 */
enum {
    CAN2_TX_EXT_COMMAND = 0,
    CAN2_TX_DIAGNOSTIC_RESP,
    CAN2_TX_STATUS_WORD,
    CAN2_TX_END
};

extern CCX_TX_table_t CAN1_tx_table[];
extern CCX_TX_table_t CAN2_tx_table[];

#endif /* CAN_APP_MSG_TX_H_ */
```

**File: `can_app_msg_tx.c`**

```c
#include "can_app_msg_tx.h"
#include "can_app_frames_api.h"

/* Frame instances */
CAN_MotorStatus_t      CAN_MotorStatus;
CAN_PowerMeasurement_t CAN_PowerMeasurement;
CAN_ExtendedCommand_t  CAN_ExtCommand;

/* Optional: TX parser callback example */
static void motor_status_tx_parser(const CCX_instance_t *Instance, 
                                   uint8_t *DataToSend, 
                                   uint16_t Slot, 
                                   void *UserData)
{
    (void)Instance;
    (void)Slot;
    (void)UserData;
    
    /* Update frame data just before transmission */
    static uint8_t counter = 0;
    CAN_MotorStatus_t *frame = (CAN_MotorStatus_t *)DataToSend;
    frame->status = counter++;
}

/* CAN1 TX table - Standard IDs */
CCX_TX_table_t CAN1_tx_table[] = {
    {0x200, CAN_MotorStatus.frame, 8, 0, NULL, 100, motor_status_tx_parser},
    {0x210, CAN_PowerMeasurement.frame, 8, 0, NULL, 200, NULL},
    {0x220, CAN_BatteryVoltage.frame, 8, 0, NULL, 500, NULL},
    {0x230, CAN_ErrorFlags.frame, 8, 0, NULL, 1000, NULL}
};

/* CAN2 TX table - Mix of Standard and Extended IDs */
CCX_TX_table_t CAN2_tx_table[] = {
    {0x18DA00F1, CAN_ExtCommand.frame, 8, 1, NULL, 500, NULL},        /* Extended ID */
    {0x18DAF100, CAN_DiagnosticResp.frame, 8, 1, NULL, 1000, NULL},   /* Extended ID */
    {0x300, CAN_StatusWord.frame, 8, 0, NULL, 100, NULL}              /* Standard ID */
};
```

#### Step 4: RX Message Table

**File: `can_app_msg_rx.h`**

```c
#ifndef CAN_APP_MSG_RX_H_
#define CAN_APP_MSG_RX_H_

#include "can_corex.h"
#include <stdint.h>

/* RX table indices for CAN1 */
enum {
    CAN1_RX_MOTOR_CONTROL = 0,
    CAN1_RX_BATTERY_COMMAND,
    CAN1_RX_CONFIG_MESSAGE,
    CAN1_RX_DEBUG_MESSAGE,
    CAN1_RX_END
};

/* RX table indices for CAN2 */
enum {
    CAN2_RX_EXT_STATUS = 0,
    CAN2_RX_EXT_COMMAND,
    CAN2_RX_STANDARD_RESP,
    CAN2_RX_END
};

extern CCX_RX_table_t CAN1_rx_table[];
extern CCX_RX_table_t CAN2_rx_table[];

/* Unregistered message parsers */
void CAN1_rx_unreg_parser(const CCX_instance_t *Instance, CCX_message_t *Msg);
void CAN2_rx_unreg_parser(const CCX_instance_t *Instance, CCX_message_t *Msg);

/* Timeout callbacks */
void CAN1_rx_timeout(CCX_instance_t *Instance, uint16_t Slot, void *UserData);
void CAN2_rx_timeout(CCX_instance_t *Instance, uint16_t Slot, void *UserData);

#endif /* CAN_APP_MSG_RX_H_ */
```

**File: `can_app_msg_rx.c`**

```c
#include "can_app_msg_rx.h"
#include "can_app_frames_api.h"
#include <stdio.h>

/* Frame instances for RX */
CAN_MotorControl_t     CAN_MotorControl;
CAN_BatteryCommand_t   CAN_BatteryCommand;
CAN_ConfigMessage_t    CAN_ConfigMessage;
CAN_DebugMessage_t     CAN_DebugMessage;
CAN_ExtendedStatus_t   CAN_ExtendedStatus;
CAN_StandardResponse_t CAN_StandardResponse;

/* Parser callback for motor control message */
static void motor_control_parser(const CCX_instance_t *Instance,
                                 CCX_message_t *Msg,
                                 uint16_t Slot,
                                 void *UserData)
{
    (void)Instance;
    (void)Slot;
    (void)UserData;
    
    /* Copy data to frame structure */
    for (uint8_t i = 0; i < 8; i++) {
        CAN_MotorControl.frame[i] = Msg->Data[i];
    }
    
    /* Process received data */
    if (CAN_MotorControl.enable) {
        /* Handle enable command */
        printf("Motor enabled, speed setpoint: %d\n", 
               CAN_MotorControl.speed_setpoint);
    }
}

/* Parser for battery command */
static void battery_command_parser(const CCX_instance_t *Instance,
                                   CCX_message_t *Msg,
                                   uint16_t Slot,
                                   void *UserData)
{
    (void)Instance;
    (void)Slot;
    (void)UserData;
    
    for (uint8_t i = 0; i < 8; i++) {
        CAN_BatteryCommand.frame[i] = Msg->Data[i];
    }
}

/* Parser for configuration message */
static void config_parser(const CCX_instance_t *Instance,
                         CCX_message_t *Msg,
                         uint16_t Slot,
                         void *UserData)
{
    (void)Instance;
    (void)Slot;
    (void)UserData;
    
    for (uint8_t i = 0; i < 8; i++) {
        CAN_ConfigMessage.frame[i] = Msg->Data[i];
    }
}

/* Parser for debug message */
static void debug_parser(const CCX_instance_t *Instance,
                        CCX_message_t *Msg,
                        uint16_t Slot,
                        void *UserData)
{
    (void)Instance;
    (void)Slot;
    (void)UserData;
    
    for (uint8_t i = 0; i < Msg->DLC; i++) {  /* Use actual DLC */
        CAN_DebugMessage.frame[i] = Msg->Data[i];
    }
}

/* Parser for extended status (Extended ID) */
static void ext_status_parser(const CCX_instance_t *Instance,
                              CCX_message_t *Msg,
                              uint16_t Slot,
                              void *UserData)
{
    (void)Instance;
    (void)Slot;
    (void)UserData;
    
    for (uint8_t i = 0; i < 8; i++) {
        CAN_ExtendedStatus.frame[i] = Msg->Data[i];
    }
}

/* Parser for extended command (Extended ID) */
static void ext_command_parser(const CCX_instance_t *Instance,
                               CCX_message_t *Msg,
                               uint16_t Slot,
                               void *UserData)
{
    (void)Instance;
    (void)Slot;
    (void)UserData;
    
    for (uint8_t i = 0; i < 8; i++) {
        CAN_ExtendedCommand.frame[i] = Msg->Data[i];
    }
}

/* Parser for standard response */
static void standard_resp_parser(const CCX_instance_t *Instance,
                                 CCX_message_t *Msg,
                                 uint16_t Slot,
                                 void *UserData)
{
    (void)Instance;
    (void)Slot;
    (void)UserData;
    
    for (uint8_t i = 0; i < 8; i++) {
        CAN_StandardResponse.frame[i] = Msg->Data[i];
    }
}

/* CAN1 RX table - Standard ID */
CCX_RX_table_t CAN1_rx_table[] = {
    {0x100, 8, 0, NULL, 1000, motor_control_parser, CAN1_rx_timeout},
    {0x110, 8, 0, NULL, 500, battery_command_parser, CAN1_rx_timeout},
    {0x120, 8, 0, NULL, 2000, config_parser, NULL},
    {0x130, CCX_DLC_ANY, 0, NULL, 0, debug_parser, NULL}  /* Accept any DLC, no timeout */
};

/* CAN2 RX table - Mix of Standard and Extended IDs */
CCX_RX_table_t CAN2_rx_table[] = {
    {0x18DAF100, 8, 1, NULL, 2000, ext_status_parser, CAN2_rx_timeout},     /* Extended ID */
    {0x18DA00F1, 8, 1, NULL, 1000, ext_command_parser, CAN2_rx_timeout},    /* Extended ID */
    {0x200, 8, 0, NULL, 500, standard_resp_parser, NULL}                    /* Standard ID */
};

/* Unregistered message handler for CAN1 */
void CAN1_rx_unreg_parser(const CCX_instance_t *Instance, CCX_message_t *Msg)
{
    (void)Instance;
    printf("CAN1 Unregistered message ID: 0x%03lX, DLC: %d\n", 
           Msg->ID, Msg->DLC);
}

/* Unregistered message handler for CAN2 */
void CAN2_rx_unreg_parser(const CCX_instance_t *Instance, CCX_message_t *Msg)
{
    (void)Instance;
    printf("CAN2 Unregistered message ID: 0x%08lX, DLC: %d, Extended: %d\n",
           Msg->ID, Msg->DLC, Msg->IDE_flag);
}

/* Timeout handler for CAN1 */
void CAN1_rx_timeout(CCX_instance_t *Instance, uint16_t Slot, void *UserData)
{
    (void)Instance;
    (void)UserData;
    printf("CAN1 RX Timeout on slot %d\n", Slot);
}

/* Timeout handler for CAN2 */
void CAN2_rx_timeout(CCX_instance_t *Instance, uint16_t Slot, void *UserData)
{
    (void)Instance;
    (void)UserData;
    printf("CAN2 RX Timeout on slot %d\n", Slot);
}
```

#### Step 5: Main Application

**File: `can_app.h`**

```c
#ifndef CAN_APP_H_
#define CAN_APP_H_

void CAN_App_Init(void);
void CAN_App_Process(void);

#endif /* CAN_APP_H_ */
```

**File: `can_app.c`**

```c
#include "can_app.h"
#include "can_app_msg_rx.h"
#include "can_app_msg_tx.h"
#include "can_corex.h"
#include "main.h"  /* For HAL CAN handles */

/* CAN CoreX instances */
CCX_instance_t CAN1_instance;
CCX_instance_t CAN2_instance;

/* System tick variable */
volatile uint32_t system_tick_ms = 0;

/* External HAL handles - generated by STM32CubeMX */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* ========================================================================
 * CALLBACK OPTION 1: Separate callback functions for each CAN instance
 * ======================================================================== */

/* CAN1 send function */
static void CAN1_send_message(const CCX_instance_t *Instance, 
                              const CCX_message_t *msg)
{
    (void)Instance;
    
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    
    if (msg->IDE_flag) {
        TxHeader.IDE = CAN_ID_EXT;
        TxHeader.ExtId = msg->ID;
    } else {
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.StdId = msg->ID;
    }
    
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = msg->DLC;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (uint8_t *)msg->Data, &TxMailbox);
}

/* CAN2 send function */
static void CAN2_send_message(const CCX_instance_t *Instance,
                              const CCX_message_t *msg)
{
    (void)Instance;
    
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    
    if (msg->IDE_flag) {
        TxHeader.IDE = CAN_ID_EXT;
        TxHeader.ExtId = msg->ID;
    } else {
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.StdId = msg->ID;
    }
    
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = msg->DLC;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, (uint8_t *)msg->Data, &TxMailbox);
}

/* CAN1 bus status check */
static CCX_BusIsFree_t CAN1_bus_check(const CCX_instance_t *Instance)
{
    (void)Instance;
    return HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0 ? 
           CCX_BUS_FREE : CCX_BUS_BUSY;
}

/* CAN2 bus status check */
static CCX_BusIsFree_t CAN2_bus_check(const CCX_instance_t *Instance)
{
    (void)Instance;
    return HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0 ?
           CCX_BUS_FREE : CCX_BUS_BUSY;
}

/* ========================================================================
 * CALLBACK OPTION 2: Shared callback functions with instance differentiation
 * 
 * Alternative approach - use this instead of Option 1 if you prefer
 * ======================================================================== */

/* Shared send function for both CAN instances */
static void CAN_send_message_shared(const CCX_instance_t *Instance,
                                    const CCX_message_t *msg)
{
    CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    
    /* Determine which CAN peripheral based on instance pointer */
    if (Instance == &CAN1_instance) {
        hcan = &hcan1;
    } else if (Instance == &CAN2_instance) {
        hcan = &hcan2;
    } else {
        return;  /* Invalid instance */
    }
    
    if (msg->IDE_flag) {
        TxHeader.IDE = CAN_ID_EXT;
        TxHeader.ExtId = msg->ID;
    } else {
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.StdId = msg->ID;
    }
    
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = msg->DLC;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    HAL_CAN_AddTxMessage(hcan, &TxHeader, (uint8_t *)msg->Data, &TxMailbox);
}

/* Shared bus check function for both CAN instances */
static CCX_BusIsFree_t CAN_bus_check_shared(const CCX_instance_t *Instance)
{
    CAN_HandleTypeDef *hcan;
    
    /* Determine which CAN peripheral based on instance pointer */
    if (Instance == &CAN1_instance) {
        hcan = &hcan1;
    } else if (Instance == &CAN2_instance) {
        hcan = &hcan2;
    } else {
        return CCX_BUS_BUSY;  /* Invalid instance */
    }
    
    return HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0 ?
           CCX_BUS_FREE : CCX_BUS_BUSY;
}

/*
 * When using OPTION 2 (shared callbacks), change CCX_Init() calls to:
 * 
 * CCX_Init(&CAN1_instance, ..., CAN_send_message_shared, CAN_bus_check_shared, ...);
 * CCX_Init(&CAN2_instance, ..., CAN_send_message_shared, CAN_bus_check_shared, ...);
 */

/* ========================================================================
 * Initialization
 * ======================================================================== */

void CAN_App_Init(void)
{
    /* Register system tick variable */
    CCX_tick_variable_register(&system_tick_ms);
    
    /* Initialize CAN1 instance */
    CCX_Init(&CAN1_instance,
             CAN1_rx_table,
             CAN1_tx_table,
             CAN1_RX_END,
             CAN1_TX_END,
             CAN1_send_message,
             CAN1_bus_check,
             CAN1_rx_unreg_parser);
    
    /* Initialize CAN2 instance */
    CCX_Init(&CAN2_instance,
             CAN2_rx_table,
             CAN2_tx_table,
             CAN2_RX_END,
             CAN2_TX_END,
             CAN2_send_message,
             CAN2_bus_check,
             CAN2_rx_unreg_parser);
    
    /* Configure CAN1 filter to accept all Standard IDs */
    CAN_FilterTypeDef filter1;
    filter1.FilterIdHigh = 0x0000;
    filter1.FilterIdLow = 0x0000;
    filter1.FilterMaskIdHigh = 0x0000;
    filter1.FilterMaskIdLow = 0x0000;
    filter1.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter1.FilterBank = 0;
    filter1.FilterMode = CAN_FILTERMODE_IDMASK;
    filter1.FilterScale = CAN_FILTERSCALE_32BIT;
    filter1.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(&hcan1, &filter1);
    
    /* Configure CAN2 filter to accept all Extended IDs */
    CAN_FilterTypeDef filter2;
    filter2.FilterIdHigh = 0x0000;
    filter2.FilterIdLow = 0x0000;
    filter2.FilterMaskIdHigh = 0x0000;
    filter2.FilterMaskIdLow = 0x0000;
    filter2.FilterFIFOAssignment = CAN_RX_FIFO1;
    filter2.FilterBank = 14;  /* CAN2 starts at bank 14 */
    filter2.FilterMode = CAN_FILTERMODE_IDMASK;
    filter2.FilterScale = CAN_FILTERSCALE_32BIT;
    filter2.FilterActivation = ENABLE;
    filter2.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &filter2);
    
    /* Activate RX notifications */
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
    
    /* Start CAN peripherals */
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
}

/* ========================================================================
 * Main processing loop
 * ======================================================================== */

void CAN_App_Process(void)
{
    /* Poll both CAN instances */
    CCX_Poll(&CAN1_instance);
    CCX_Poll(&CAN2_instance);
}

/* ========================================================================
 * HAL CAN RX Interrupt Callbacks
 * ======================================================================== */

/* CAN1 FIFO0 reception callback */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) {
        CAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8];
        
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
            CCX_message_t msg;
            
            if (RxHeader.IDE == CAN_ID_EXT) {
                msg.ID = RxHeader.ExtId;
                msg.IDE_flag = 1;
            } else {
                msg.ID = RxHeader.StdId;
                msg.IDE_flag = 0;
            }
            
            msg.DLC = RxHeader.DLC;
            for (uint8_t i = 0; i < RxHeader.DLC; i++) {
                msg.Data[i] = RxData[i];
            }
            
            CCX_RX_PushMsg(&CAN1_instance, &msg);
        }
    }
}

/* CAN2 FIFO1 reception callback */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN2) {
        CAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8];
        
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK) {
            CCX_message_t msg;
            
            if (RxHeader.IDE == CAN_ID_EXT) {
                msg.ID = RxHeader.ExtId;
                msg.IDE_flag = 1;
            } else {
                msg.ID = RxHeader.StdId;
                msg.IDE_flag = 0;
            }
            
            msg.DLC = RxHeader.DLC;
            for (uint8_t i = 0; i < RxHeader.DLC; i++) {
                msg.Data[i] = RxData[i];
            }
            
            CCX_RX_PushMsg(&CAN2_instance, &msg);
        }
    }
}

/* ========================================================================
 * System Tick Handler
 * ======================================================================== */

/* Call this from SysTick_Handler or timer interrupt */
void CAN_App_SysTick(void)
{
    system_tick_ms++;
}
```

**File: `main.c` (excerpt)**

```c
#include "main.h"
#include "can_app.h"

/* Handles generated by STM32CubeMX */
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    
    /* Initialize CAN peripherals (generated by CubeMX) */
    MX_CAN1_Init();
    MX_CAN2_Init();
    
    /* Initialize CAN application */
    CAN_App_Init();
    
    while (1)
    {
        /* Process CAN messages */
        CAN_App_Process();
        
        /* Other application tasks */
        HAL_Delay(1);
    }
}

/* SysTick handler */
void SysTick_Handler(void)
{
    HAL_IncTick();
    CAN_App_SysTick();  /* Increment CAN CoreX tick */
}
```

---

### 1.2 Bus Monitoring & Statistics

This section extends the basic implementation with bus health monitoring and operational statistics.

#### Features Added:
- Automatic bus-off detection and recovery
- TEC/REC error counter tracking
- Global statistics (RX/TX counters, buffer overflows)
- State transition callbacks

#### Step 1: Hardware Interface Functions

Add these functions to `can_app.c`:

```c
/* ========================================================================
 * Bus Monitoring Hardware Interface Functions
 * ======================================================================== */

/* Get current bus state from CAN peripheral */
static CCX_BusState_t CAN1_get_bus_state(const CCX_instance_t *Instance)
{
    (void)Instance;
    
    uint32_t esr = hcan1.Instance->ESR;
    
    /* Check for Bus-Off */
    if (esr & CAN_ESR_BOFF) {
        return CCX_BUS_STATE_OFF;
    }
    
    /* Extract error counters */
    uint8_t tec = (esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos;
    uint8_t rec = (esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos;
    
    /* Determine state based on ISO 11898-1 */
    if (tec > 127 || rec > 127) {
        return CCX_BUS_STATE_PASSIVE;
    } else if (tec > 96 || rec > 96) {
        return CCX_BUS_STATE_WARNING;
    }
    
    return CCX_BUS_STATE_ACTIVE;
}

static CCX_BusState_t CAN2_get_bus_state(const CCX_instance_t *Instance)
{
    (void)Instance;
    
    uint32_t esr = hcan2.Instance->ESR;
    
    if (esr & CAN_ESR_BOFF) {
        return CCX_BUS_STATE_OFF;
    }
    
    uint8_t tec = (esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos;
    uint8_t rec = (esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos;
    
    if (tec > 127 || rec > 127) {
        return CCX_BUS_STATE_PASSIVE;
    } else if (tec > 96 || rec > 96) {
        return CCX_BUS_STATE_WARNING;
    }
    
    return CCX_BUS_STATE_ACTIVE;
}

/* Read TEC/REC error counters */
static void CAN1_get_error_counters(const CCX_instance_t *Instance,
                                    CCX_ErrorCounters_t *Counters)
{
    (void)Instance;
    
    uint32_t esr = hcan1.Instance->ESR;
    Counters->TEC = (esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos;
    Counters->REC = (esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos;
}

static void CAN2_get_error_counters(const CCX_instance_t *Instance,
                                    CCX_ErrorCounters_t *Counters)
{
    (void)Instance;
    
    uint32_t esr = hcan2.Instance->ESR;
    Counters->TEC = (esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos;
    Counters->REC = (esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos;
}

/* Request bus-off recovery */
static void CAN1_request_recovery(const CCX_instance_t *Instance)
{
    (void)Instance;
    
    /* Clear BOFF flag by resetting CAN peripheral */
    HAL_CAN_Stop(&hcan1);
    HAL_CAN_Start(&hcan1);
}

static void CAN2_request_recovery(const CCX_instance_t *Instance)
{
    (void)Instance;
    
    HAL_CAN_Stop(&hcan2);
    HAL_CAN_Start(&hcan2);
}
```

#### Step 2: Bus Monitor Callbacks

Add callback functions to handle bus state changes:

```c
/* ========================================================================
 * Bus Monitoring Callbacks
 * ======================================================================== */

/* Called when bus state changes */
static void CAN1_bus_state_changed(CCX_instance_t *Instance,
                                   CCX_BusState_t OldState,
                                   CCX_BusState_t NewState,
                                   void *UserData)
{
    (void)Instance;
    (void)UserData;
    
    const char *state_names[] = {
        "ACTIVE", "WARNING", "PASSIVE", "OFF"
    };
    
    printf("CAN1 State: %s -> %s\n", 
           state_names[OldState], 
           state_names[NewState]);
    
    if (NewState == CCX_BUS_STATE_OFF) {
        /* Bus-off occurred - recovery will start automatically */
        printf("CAN1 Bus-Off detected! Auto-recovery starting...\n");
    }
}

static void CAN2_bus_state_changed(CCX_instance_t *Instance,
                                   CCX_BusState_t OldState,
                                   CCX_BusState_t NewState,
                                   void *UserData)
{
    (void)Instance;
    (void)UserData;
    
    const char *state_names[] = {
        "ACTIVE", "WARNING", "PASSIVE", "OFF"
    };
    
    printf("CAN2 State: %s -> %s\n",
           state_names[OldState],
           state_names[NewState]);
}

/* Called before each recovery attempt */
static void CAN1_recovery_attempt(CCX_instance_t *Instance,
                                  uint8_t AttemptNumber,
                                  void *UserData)
{
    (void)Instance;
    (void)UserData;
    
    printf("CAN1 Recovery attempt #%d\n", AttemptNumber);
}

/* Called when max recovery attempts reached */
static void CAN1_recovery_failed(CCX_instance_t *Instance,
                                 void *UserData)
{
    (void)Instance;
    (void)UserData;
    
    printf("CAN1 Recovery failed - entering grace period\n");
}

/* Called when error counters update */
static void CAN1_error_counters_updated(CCX_instance_t *Instance,
                                        const CCX_ErrorCounters_t *Counters,
                                        void *UserData)
{
    (void)Instance;
    (void)UserData;
    
    /* Only print when counters are significant */
    if (Counters->TEC > 50 || Counters->REC > 50) {
        printf("CAN1 Errors - TEC: %d, REC: %d\n", 
               Counters->TEC, Counters->REC);
    }
}
```

#### Step 3: Initialize Bus Monitoring

Update `CAN_App_Init()` to include bus monitoring:

```c
void CAN_App_Init(void)
{
    /* Register system tick variable */
    CCX_tick_variable_register(&system_tick_ms);
    
    /* Initialize CAN1 instance */
    CCX_Init(&CAN1_instance,
             CAN1_rx_table,
             CAN1_tx_table,
             CAN1_RX_END,
             CAN1_TX_END,
             CAN1_send_message,
             CAN1_bus_check,
             CAN1_rx_unreg_parser);
    
    /* Initialize CAN2 instance */
    CCX_Init(&CAN2_instance,
             CAN2_rx_table,
             CAN2_tx_table,
             CAN2_RX_END,
             CAN2_TX_END,
             CAN2_send_message,
             CAN2_bus_check,
             CAN2_rx_unreg_parser);
    
    /* ====================================================================
     * Initialize Bus Monitoring for CAN1
     * ==================================================================== */
    
    static CCX_BusMonitor_t CAN1_monitor;
    
    CCX_BusMonitor_Init(
        &CAN1_instance,
        &CAN1_monitor,
        CAN1_get_bus_state,
        CAN1_get_error_counters,
        CAN1_request_recovery,
        10,      /* recovery_delay: 10ms between attempts */
        60000,   /* successful_run_time: 60s before resetting counter */
        1,       /* auto_recovery_enabled */
        5        /* max_recovery_attempts before grace period */
    );
    
    /* Set callbacks */
    CAN1_monitor.OnBusStateChange = CAN1_bus_state_changed;
    CAN1_monitor.OnRecoveryAttempt = CAN1_recovery_attempt;
    CAN1_monitor.OnRecoveryFailed = CAN1_recovery_failed;
    CAN1_monitor.OnErrorCountersUpdate = CAN1_error_counters_updated;
    
    /* ====================================================================
     * Initialize Bus Monitoring for CAN2 (simplified - no callbacks)
     * ==================================================================== */
    
    static CCX_BusMonitor_t CAN2_monitor;
    
    CCX_BusMonitor_Init(
        &CAN2_instance,
        &CAN2_monitor,
        CAN2_get_bus_state,
        CAN2_get_error_counters,
        CAN2_request_recovery,
        10,
        60000,
        1,
        5
    );
    
    CAN2_monitor.OnBusStateChange = CAN2_bus_state_changed;
    
    /* Configure filters and start CAN (same as before) */
    /* ... filter configuration ... */
    
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
}
```

#### Step 4: Access Statistics

Add a function to periodically check and display statistics:

```c
/* ========================================================================
 * Statistics Reporting
 * ======================================================================== */

void CAN_App_PrintStats(void)
{
    /* Get global statistics */
    const CCX_GlobalStats_t *stats1 = CCX_GetGlobalStats(&CAN1_instance);
    const CCX_GlobalStats_t *stats2 = CCX_GetGlobalStats(&CAN2_instance);
    
    printf("\n=== CAN1 Statistics ===\n");
    printf("RX messages:      %lu\n", stats1->total_rx_messages);
    printf("TX messages:      %lu\n", stats1->total_tx_messages);
    printf("RX overflows:     %lu\n", stats1->rx_buffer_overflows);
    printf("TX overflows:     %lu\n", stats1->tx_buffer_overflows);
    printf("Parser calls:     %lu\n", stats1->parser_calls_count);
    printf("Timeout calls:    %lu\n", stats1->timeout_calls_count);
    
    printf("\n=== CAN2 Statistics ===\n");
    printf("RX messages:      %lu\n", stats2->total_rx_messages);
    printf("TX messages:      %lu\n", stats2->total_tx_messages);
    printf("RX overflows:     %lu\n", stats2->rx_buffer_overflows);
    printf("TX overflows:     %lu\n", stats2->tx_buffer_overflows);
    
    /* Get bus monitoring statistics */
    CCX_BusState_t state1 = CCX_BusMonitor_GetState(&CAN1_instance);
    const char *state_names[] = {"ACTIVE", "WARNING", "PASSIVE", "OFF"};
    
    printf("\n=== CAN1 Bus Health ===\n");
    printf("Current state:    %s\n", state_names[state1]);
    
    if (CAN1_instance.BusMonitor) {
        printf("Bus-off count:    %lu\n", 
               CAN1_instance.BusMonitor->stats.bus_off_count);
        printf("Current TEC:      %d\n",
               CAN1_instance.BusMonitor->stats.error_counters.TEC);
        printf("Current REC:      %d\n",
               CAN1_instance.BusMonitor->stats.error_counters.REC);
        printf("Peak TEC:         %d\n",
               CAN1_instance.BusMonitor->stats.peak_error_counters.TEC);
        printf("Peak REC:         %d\n",
               CAN1_instance.BusMonitor->stats.peak_error_counters.REC);
    }
}

/* Optional: Reset statistics */
void CAN_App_ResetStats(void)
{
    CCX_ResetGlobalStats(&CAN1_instance);
    CCX_ResetGlobalStats(&CAN2_instance);
    
    CCX_BusMonitor_ResetStats(&CAN1_instance);
    CCX_BusMonitor_ResetStats(&CAN2_instance);
    
    printf("Statistics reset for both CAN instances\n");
}
```

#### Step 5: Call Statistics from Main Loop

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    
    MX_CAN1_Init();
    MX_CAN2_Init();
    
    CAN_App_Init();
    
    uint32_t stats_print_tick = 0;
    
    while (1)
    {
        /* Process CAN messages */
        CAN_App_Process();
        
        /* Print statistics every 10 seconds */
        if (system_tick_ms - stats_print_tick >= 10000) {
            stats_print_tick = system_tick_ms;
            CAN_App_PrintStats();
        }
        
        HAL_Delay(1);
    }
}
```

#### Step 6: Manual Recovery Trigger

You can also trigger recovery manually:

```c
/* Trigger manual recovery (e.g., from button press or command) */
void CAN_App_TriggerManualRecovery(uint8_t can_number)
{
    CCX_Status_t status;
    
    if (can_number == 1) {
        status = CCX_BusMonitor_TriggerRecovery(&CAN1_instance);
        if (status == CCX_OK) {
            printf("CAN1 Manual recovery triggered\n");
        } else if (status == CCX_WRONG_ARG) {
            printf("CAN1 Not in bus-off state\n");
        }
    } else if (can_number == 2) {
        status = CCX_BusMonitor_TriggerRecovery(&CAN2_instance);
        if (status == CCX_OK) {
            printf("CAN2 Manual recovery triggered\n");
        }
    }
}
```

---

## Platform: STM32 HAL (FDCAN as CAN 2.0)

### Prerequisites

- STM32 with FDCAN peripheral (e.g., STM32G4, STM32H7, STM32U5)
- FDCAN configured in **Classic CAN mode** (not FD mode)
- **Required interrupts enabled in NVIC:**
  - `FDCANx_IT0_IRQn` or `FDCANx_IT1_IRQn` (RX FIFO interrupts)
- System tick configured

**Note:** FDCAN peripheral supports both Classic CAN 2.0 and CAN FD modes. This implementation uses FDCAN configured as Classic CAN 2.0.

---

### 2.1 Basic TX/RX Implementation

The FDCAN peripheral has different register structure and API compared to bxCAN, but provides the same functionality.

#### Key Differences from bxCAN:
- Uses RX FIFO0/FIFO1 instead of mailboxes
- Different filter configuration (uses dedicated RAM)
- TX buffer management differs
- Error counter access through different registers

#### Step 1-3: Frame Types, API, Tables

**Use the same frame type definitions, API declarations, and table structures as STM32 HAL (CAN 2.0) example.**

The only difference is in the hardware interface layer.

#### Step 4: Main Application for FDCAN

**File: `can_app.c`**

```c
#include "can_app.h"
#include "can_app_msg_rx.h"
#include "can_app_msg_tx.h"
#include "can_corex.h"
#include "main.h"

/* CAN CoreX instances */
CCX_instance_t FDCAN1_instance;
CCX_instance_t FDCAN2_instance;

/* System tick variable */
volatile uint32_t system_tick_ms = 0;

/* External HAL handles - generated by STM32CubeMX */
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

/* ========================================================================
 * CALLBACK OPTION 1: Separate callback functions for each FDCAN instance
 * ======================================================================== */

/* FDCAN1 send function */
static void FDCAN1_send_message(const CCX_instance_t *Instance,
                                const CCX_message_t *msg)
{
    (void)Instance;
    
    FDCAN_TxHeaderTypeDef TxHeader;
    
    if (msg->IDE_flag) {
        TxHeader.Identifier = msg->ID;
        TxHeader.IdType = FDCAN_EXTENDED_ID;
    } else {
        TxHeader.Identifier = msg->ID;
        TxHeader.IdType = FDCAN_STANDARD_ID;
    }
    
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = msg->DLC << 16;  /* DLC in upper 16 bits */
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, (uint8_t *)msg->Data);
}

/* FDCAN2 send function */
static void FDCAN2_send_message(const CCX_instance_t *Instance,
                                const CCX_message_t *msg)
{
    (void)Instance;
    
    FDCAN_TxHeaderTypeDef TxHeader;
    
    if (msg->IDE_flag) {
        TxHeader.Identifier = msg->ID;
        TxHeader.IdType = FDCAN_EXTENDED_ID;
    } else {
        TxHeader.Identifier = msg->ID;
        TxHeader.IdType = FDCAN_STANDARD_ID;
    }
    
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = msg->DLC << 16;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, (uint8_t *)msg->Data);
}

/* FDCAN1 bus status check */
static CCX_BusIsFree_t FDCAN1_bus_check(const CCX_instance_t *Instance)
{
    (void)Instance;
    return HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0 ?
           CCX_BUS_FREE : CCX_BUS_BUSY;
}

/* FDCAN2 bus status check */
static CCX_BusIsFree_t FDCAN2_bus_check(const CCX_instance_t *Instance)
{
    (void)Instance;
    return HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0 ?
           CCX_BUS_FREE : CCX_BUS_BUSY;
}

/* ========================================================================
 * CALLBACK OPTION 2: Shared callback functions with instance differentiation
 * 
 * Alternative approach - use this instead of Option 1 if you prefer
 * ======================================================================== */

/* Shared send function for both FDCAN instances */
static void FDCAN_send_message_shared(const CCX_instance_t *Instance,
                                      const CCX_message_t *msg)
{
    FDCAN_HandleTypeDef *hfdcan;
    FDCAN_TxHeaderTypeDef TxHeader;
    
    /* Determine which FDCAN peripheral based on instance pointer */
    if (Instance == &FDCAN1_instance) {
        hfdcan = &hfdcan1;
    } else if (Instance == &FDCAN2_instance) {
        hfdcan = &hfdcan2;
    } else {
        return;  /* Invalid instance */
    }
    
    if (msg->IDE_flag) {
        TxHeader.Identifier = msg->ID;
        TxHeader.IdType = FDCAN_EXTENDED_ID;
    } else {
        TxHeader.Identifier = msg->ID;
        TxHeader.IdType = FDCAN_STANDARD_ID;
    }
    
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = msg->DLC << 16;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, (uint8_t *)msg->Data);
}

/* Shared bus check function for both FDCAN instances */
static CCX_BusIsFree_t FDCAN_bus_check_shared(const CCX_instance_t *Instance)
{
    FDCAN_HandleTypeDef *hfdcan;
    
    /* Determine which FDCAN peripheral based on instance pointer */
    if (Instance == &FDCAN1_instance) {
        hfdcan = &hfdcan1;
    } else if (Instance == &FDCAN2_instance) {
        hfdcan = &hfdcan2;
    } else {
        return CCX_BUS_BUSY;  /* Invalid instance */
    }
    
    return HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0 ?
           CCX_BUS_FREE : CCX_BUS_BUSY;
}

/*
 * When using OPTION 2 (shared callbacks), change CCX_Init() calls to:
 * 
 * CCX_Init(&FDCAN1_instance, ..., FDCAN_send_message_shared, FDCAN_bus_check_shared, ...);
 * CCX_Init(&FDCAN2_instance, ..., FDCAN_send_message_shared, FDCAN_bus_check_shared, ...);
 */

/* ========================================================================
 * Initialization
 * ======================================================================== */

void CAN_App_Init(void)
{
    /* Register system tick variable */
    CCX_tick_variable_register(&system_tick_ms);
    
    /* Initialize FDCAN1 instance */
    CCX_Init(&FDCAN1_instance,
             CAN1_rx_table,
             CAN1_tx_table,
             CAN1_RX_END,
             CAN1_TX_END,
             FDCAN1_send_message,
             FDCAN1_bus_check,
             CAN1_rx_unreg_parser);
    
    /* Initialize FDCAN2 instance */
    CCX_Init(&FDCAN2_instance,
             CAN2_rx_table,
             CAN2_tx_table,
             CAN2_RX_END,
             CAN2_TX_END,
             FDCAN2_send_message,
             FDCAN2_bus_check,
             CAN2_rx_unreg_parser);
    
    /* Configure FDCAN1 filter to accept all Standard IDs */
    FDCAN_FilterTypeDef filter1;
    filter1.IdType = FDCAN_STANDARD_ID;
    filter1.FilterIndex = 0;
    filter1.FilterType = FDCAN_FILTER_MASK;
    filter1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter1.FilterID1 = 0x000;
    filter1.FilterID2 = 0x000;  /* Mask: accept all */
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter1);
    
    /* Configure FDCAN1 filter to accept all Extended IDs */
    FDCAN_FilterTypeDef filter1_ext;
    filter1_ext.IdType = FDCAN_EXTENDED_ID;
    filter1_ext.FilterIndex = 0;
    filter1_ext.FilterType = FDCAN_FILTER_MASK;
    filter1_ext.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter1_ext.FilterID1 = 0x00000000;
    filter1_ext.FilterID2 = 0x00000000;  /* Mask: accept all */
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter1_ext);
    
    /* Configure FDCAN2 filter to accept all Standard IDs */
    FDCAN_FilterTypeDef filter2;
    filter2.IdType = FDCAN_STANDARD_ID;
    filter2.FilterIndex = 0;
    filter2.FilterType = FDCAN_FILTER_MASK;
    filter2.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    filter2.FilterID1 = 0x000;
    filter2.FilterID2 = 0x000;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &filter2);
    
    /* Configure FDCAN2 filter to accept all Extended IDs */
    FDCAN_FilterTypeDef filter2_ext;
    filter2_ext.IdType = FDCAN_EXTENDED_ID;
    filter2_ext.FilterIndex = 0;
    filter2_ext.FilterType = FDCAN_FILTER_MASK;
    filter2_ext.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    filter2_ext.FilterID1 = 0x00000000;
    filter2_ext.FilterID2 = 0x00000000;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &filter2_ext);
    
    /* Activate RX FIFO notifications */
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    
    /* Start FDCAN peripherals */
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);
}

/* ========================================================================
 * Main processing loop
 * ======================================================================== */

void CAN_App_Process(void)
{
    /* Poll both FDCAN instances */
    CCX_Poll(&FDCAN1_instance);
    CCX_Poll(&FDCAN2_instance);
}

/* ========================================================================
 * HAL FDCAN RX Interrupt Callbacks
 * ======================================================================== */

/* FDCAN1 RX FIFO0 callback */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        if (hfdcan->Instance == FDCAN1)
        {
            FDCAN_RxHeaderTypeDef RxHeader;
            uint8_t RxData[8];
            
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
            {
                CCX_message_t msg;
                
                msg.ID = RxHeader.Identifier;
                msg.IDE_flag = (RxHeader.IdType == FDCAN_EXTENDED_ID) ? 1 : 0;
                msg.DLC = (RxHeader.DataLength >> 16) & 0x0F;
                
                for (uint8_t i = 0; i < msg.DLC; i++) {
                    msg.Data[i] = RxData[i];
                }
                
                CCX_RX_PushMsg(&FDCAN1_instance, &msg);
            }
        }
    }
}

/* FDCAN2 RX FIFO1 callback */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0)
    {
        if (hfdcan->Instance == FDCAN2)
        {
            FDCAN_RxHeaderTypeDef RxHeader;
            uint8_t RxData[8];
            
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
            {
                CCX_message_t msg;
                
                msg.ID = RxHeader.Identifier;
                msg.IDE_flag = (RxHeader.IdType == FDCAN_EXTENDED_ID) ? 1 : 0;
                msg.DLC = (RxHeader.DataLength >> 16) & 0x0F;
                
                for (uint8_t i = 0; i < msg.DLC; i++) {
                    msg.Data[i] = RxData[i];
                }
                
                CCX_RX_PushMsg(&FDCAN2_instance, &msg);
            }
        }
    }
}

/* ========================================================================
 * System Tick Handler
 * ======================================================================== */

/* Call this from SysTick_Handler or timer interrupt */
void CAN_App_SysTick(void)
{
    system_tick_ms++;
}
```

**File: `main.c` (excerpt)**

```c
#include "main.h"
#include "can_app.h"

/* Handles generated by STM32CubeMX */
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    
    /* Initialize FDCAN peripherals (generated by CubeMX) */
    MX_FDCAN1_Init();
    MX_FDCAN2_Init();
    
    /* Initialize CAN application */
    CAN_App_Init();
    
    while (1)
    {
        /* Process CAN messages */
        CAN_App_Process();
        
        /* Other application tasks */
        HAL_Delay(1);
    }
}

/* SysTick handler */
void SysTick_Handler(void)
{
    HAL_IncTick();
    CAN_App_SysTick();  /* Increment CAN CoreX tick */
}
```

---

### 2.2 Bus Monitoring & Statistics

Bus monitoring for FDCAN requires different register access compared to bxCAN.

#### Step 1: Hardware Interface Functions

Add these functions to `can_app.c`:

```c
/* ========================================================================
 * Bus Monitoring Hardware Interface Functions for FDCAN
 * ======================================================================== */

/* Get current bus state from FDCAN peripheral */
static CCX_BusState_t FDCAN1_get_bus_state(const CCX_instance_t *Instance)
{
    (void)Instance;
    
    uint32_t psr = hfdcan1.Instance->PSR;
    
    /* Check for Bus-Off */
    if (psr & FDCAN_PSR_BO) {
        return CCX_BUS_STATE_OFF;
    }
    
    /* Extract error counters */
    uint32_t ecr = hfdcan1.Instance->ECR;
    uint8_t tec = (ecr & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos;
    uint8_t rec = (ecr & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos;
    
    /* Determine state based on ISO 11898-1 */
    if (tec > 127 || rec > 127) {
        return CCX_BUS_STATE_PASSIVE;
    } else if (tec > 96 || rec > 96) {
        return CCX_BUS_STATE_WARNING;
    }
    
    return CCX_BUS_STATE_ACTIVE;
}

static CCX_BusState_t FDCAN2_get_bus_state(const CCX_instance_t *Instance)
{
    (void)Instance;
    
    uint32_t psr = hfdcan2.Instance->PSR;
    
    if (psr & FDCAN_PSR_BO) {
        return CCX_BUS_STATE_OFF;
    }
    
    uint32_t ecr = hfdcan2.Instance->ECR;
    uint8_t tec = (ecr & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos;
    uint8_t rec = (ecr & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos;
    
    if (tec > 127 || rec > 127) {
        return CCX_BUS_STATE_PASSIVE;
    } else if (tec > 96 || rec > 96) {
        return CCX_BUS_STATE_WARNING;
    }
    
    return CCX_BUS_STATE_ACTIVE;
}

/* Read TEC/REC error counters */
static void FDCAN1_get_error_counters(const CCX_instance_t *Instance,
                                      CCX_ErrorCounters_t *Counters)
{
    (void)Instance;
    
    uint32_t ecr = hfdcan1.Instance->ECR;
    Counters->TEC = (ecr & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos;
    Counters->REC = (ecr & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos;
}

static void FDCAN2_get_error_counters(const CCX_instance_t *Instance,
                                      CCX_ErrorCounters_t *Counters)
{
    (void)Instance;
    
    uint32_t ecr = hfdcan2.Instance->ECR;
    Counters->TEC = (ecr & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos;
    Counters->REC = (ecr & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos;
}

/* Request bus-off recovery */
static void FDCAN1_request_recovery(const CCX_instance_t *Instance)
{
    (void)Instance;
    
    /* For FDCAN, restart by stop and start */
    HAL_FDCAN_Stop(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan1);
}

static void FDCAN2_request_recovery(const CCX_instance_t *Instance)
{
    (void)Instance;
    
    HAL_FDCAN_Stop(&hfdcan2);
    HAL_FDCAN_Start(&hfdcan2);
}

/* ========================================================================
 * Bus Monitoring Callbacks (same as CAN 2.0 example)
 * ======================================================================== */

/* Use the same callback functions as shown in section 1.2 */
```

#### Step 2: Initialize Bus Monitoring

Update `CAN_App_Init()`:

```c
void CAN_App_Init(void)
{
    /* Register system tick variable */
    CCX_tick_variable_register(&system_tick_ms);
    
    /* Initialize FDCAN instances */
    CCX_Init(&FDCAN1_instance,
             CAN1_rx_table,
             CAN1_tx_table,
             CAN1_RX_END,
             CAN1_TX_END,
             FDCAN1_send_message,
             FDCAN1_bus_check,
             CAN1_rx_unreg_parser);
    
    CCX_Init(&FDCAN2_instance,
             CAN2_rx_table,
             CAN2_tx_table,
             CAN2_RX_END,
             CAN2_TX_END,
             FDCAN2_send_message,
             FDCAN2_bus_check,
             CAN2_rx_unreg_parser);
    
    /* ====================================================================
     * Initialize Bus Monitoring for FDCAN1
     * ==================================================================== */
    
    static CCX_BusMonitor_t FDCAN1_monitor;
    
    CCX_BusMonitor_Init(
        &FDCAN1_instance,
        &FDCAN1_monitor,
        FDCAN1_get_bus_state,
        FDCAN1_get_error_counters,
        FDCAN1_request_recovery,
        10,      /* recovery_delay: 10ms */
        60000,   /* successful_run_time: 60s */
        1,       /* auto_recovery_enabled */
        5        /* max_recovery_attempts */
    );
    
    /* Set callbacks (reuse from section 1.2) */
    FDCAN1_monitor.OnBusStateChange = CAN1_bus_state_changed;
    FDCAN1_monitor.OnRecoveryAttempt = CAN1_recovery_attempt;
    FDCAN1_monitor.OnRecoveryFailed = CAN1_recovery_failed;
    FDCAN1_monitor.OnErrorCountersUpdate = CAN1_error_counters_updated;
    
    /* ====================================================================
     * Initialize Bus Monitoring for FDCAN2
     * ==================================================================== */
    
    static CCX_BusMonitor_t FDCAN2_monitor;
    
    CCX_BusMonitor_Init(
        &FDCAN2_instance,
        &FDCAN2_monitor,
        FDCAN2_get_bus_state,
        FDCAN2_get_error_counters,
        FDCAN2_request_recovery,
        10,
        60000,
        1,
        5
    );
    
    FDCAN2_monitor.OnBusStateChange = CAN2_bus_state_changed;
    
    /* Configure filters and start FDCAN (same as section 2.1) */
    /* ... */
    
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);
}
```

#### Key Differences from bxCAN:

1. **Register Access:**
   - `PSR` register for protocol status (instead of `ESR`)
   - `ECR` register for error counters
   - Different bit positions

2. **Recovery:**
   - Use `HAL_FDCAN_Stop()` / `HAL_FDCAN_Start()` instead of full reset

3. **Statistics:**
   - Global statistics work identically
   - Use same `CCX_GetGlobalStats()` and `CCX_ResetGlobalStats()`

**All other features (callbacks, statistics reporting, manual recovery) work identically to bxCAN example in section 1.2.**

---

## Platform: TI Connectivity Manager (CAN 2.0)

### Prerequisites

- TI C2000 device with Connectivity Manager subsystem
- Driverlib CAN driver
- System tick configured (interrupt-driven timer)
- **Required interrupts enabled in PIE:**
  - CAN RX interrupt for message reception
  - Optional: CAN TX interrupt for transmission complete notification

**Note:** The following examples are for TI Connectivity Manager (Cortex-M4 subsystem), not C2000 control core.

---

### 3.1 Basic TX/RX Implementation

This example demonstrates CAN communication on TI Connectivity Manager with Extended ID support.

#### Step 1: Frame Type Definitions

**File: `can_app_frame_types.h`**

Same structure definitions as STM32 example - reuse the frame types.

#### Step 2: TX Message Table

**File: `can_app_msg_tx.c`**

```c
#include "can_app_msg_tx.h"
#include "can_app_frames_api.h"

/* Frame instances */
CAN_MotorStatus_t      CAN_MotorStatus;
CAN_PowerMeasurement_t CAN_PowerMeasurement;
CAN_BatteryVoltage_t   CAN_BatteryVoltage;
CAN_ExtendedCommand_t  CAN_ExtCommand;

/* CAN1 TX table - Standard and Extended IDs */
CCX_TX_table_t CAN1_tx_table[] = {
    {0x200, CAN_MotorStatus.frame, 8, 0, NULL, 100, NULL},
    {0x210, CAN_PowerMeasurement.frame, 8, 0, NULL, 200, NULL},
    {0x18DA00F1, CAN_ExtCommand.frame, 8, 1, NULL, 500, NULL}  /* Extended ID */
};
```

#### Step 3: RX Message Table

**File: `can_app_msg_rx.c`**

```c
#include "can_app_msg_rx.h"
#include "can_app_frames_api.h"

CAN_MotorControl_t CAN_MotorControl;

static void motor_control_parser(const CCX_instance_t *Instance,
                                 CCX_message_t *Msg,
                                 uint16_t Slot,
                                 void *UserData)
{
    (void)Instance;
    (void)Slot;
    (void)UserData;
    
    for (uint8_t i = 0; i < 8; i++) {
        CAN_MotorControl.frame[i] = Msg->Data[i];
    }
}

/* CAN1 RX table - Standard and Extended IDs */
CCX_RX_table_t CAN1_rx_table[] = {
    {0x100, 8, 0, NULL, 1000, motor_control_parser, NULL},
    {0x18DAF100, 8, 1, NULL, 2000, NULL, NULL}  /* Extended ID */
};

void CAN1_rx_unreg_parser(const CCX_instance_t *Instance, CCX_message_t *Msg)
{
    (void)Instance;
    (void)Msg;
    /* Handle unregistered messages */
}
```

#### Step 4: Main Application

**File: `can_app.c`**

```c
#include "can_app.h"
#include "can_app_msg_rx.h"
#include "can_app_msg_tx.h"
#include "can_corex.h"
#include "driverlib.h"
#include "cm.h"

/* CAN CoreX instance */
CCX_instance_t CAN1_instance;

/* System tick variable */
volatile uint32_t system_tick_ms = 0;

/* TX mailbox tracking */
#define TX_MSG_OBJ_1  1
#define TX_MSG_OBJ_2  2
#define TX_MSG_OBJ_3  3
#define TX_MSG_OBJ_4  4
#define RX_MSG_OBJ_ID 5

static uint32_t free_tx_mailbox = TX_MSG_OBJ_1;

/* ========================================================================
 * Hardware Interface Functions
 * ======================================================================== */

/* Send message to CAN peripheral */
static void CAN_send_message(const CCX_instance_t *Instance,
                             const CCX_message_t *msg)
{
    (void)Instance;
    
    CAN_MsgFrameType frameType = msg->IDE_flag ? 
                                 CAN_MSG_FRAME_EXT : 
                                 CAN_MSG_FRAME_STD;
    
    /* Setup and send message */
    CAN_sendMessage_16bit(CANA_BASE,
                         free_tx_mailbox,
                         msg->DLC,
                         msg->ID,
                         frameType,
                         (uint16_t *)msg->Data);
}

/* Check if TX mailbox is available */
static CCX_BusIsFree_t CAN_bus_check(const CCX_instance_t *Instance)
{
    (void)Instance;
    
    /* Check if interface is busy */
    if ((HWREGH(CANA_BASE + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY) == CAN_IF1CMD_BUSY) {
        return CCX_BUS_BUSY;
    }
    
    /* Check each TX mailbox */
    if (!(HWREG(CANA_BASE + CAN_O_TXRQ_21) & (1UL << (TX_MSG_OBJ_1 - 1)))) {
        free_tx_mailbox = TX_MSG_OBJ_1;
        return CCX_BUS_FREE;
    }
    else if (!(HWREG(CANA_BASE + CAN_O_TXRQ_21) & (1UL << (TX_MSG_OBJ_2 - 1)))) {
        free_tx_mailbox = TX_MSG_OBJ_2;
        return CCX_BUS_FREE;
    }
    else if (!(HWREG(CANA_BASE + CAN_O_TXRQ_21) & (1UL << (TX_MSG_OBJ_3 - 1)))) {
        free_tx_mailbox = TX_MSG_OBJ_3;
        return CCX_BUS_FREE;
    }
    else if (!(HWREG(CANA_BASE + CAN_O_TXRQ_21) & (1UL << (TX_MSG_OBJ_4 - 1)))) {
        free_tx_mailbox = TX_MSG_OBJ_4;
        return CCX_BUS_FREE;
    }
    
    return CCX_BUS_BUSY;
}

/* ========================================================================
 * CAN Interrupt Handler
 * ======================================================================== */

__interrupt void CAN_ISR(void)
{
    uint32_t status = CAN_getInterruptCause(CANA_BASE);
    
    if (status == RX_MSG_OBJ_ID)
    {
        CAN_MsgFrameType frameType;
        uint32_t msgID;
        uint16_t rxData[4];  /* CAN driver uses 16-bit array */
        
        /* Read message */
        CAN_readMessage_16bit(CANA_BASE, 
                             RX_MSG_OBJ_ID,
                             &frameType,
                             &msgID,
                             rxData);
        
        /* Convert to CCX_message_t */
        CCX_message_t msg;
        msg.ID = msgID;
        msg.DLC = 8;  /* Assuming 8 bytes - adjust if needed */
        msg.IDE_flag = (frameType == CAN_MSG_FRAME_EXT) ? 1 : 0;
        
        /* Copy data (convert from 16-bit to 8-bit array) */
        uint8_t *pData = (uint8_t *)rxData;
        for (uint8_t i = 0; i < 8; i++) {
            msg.Data[i] = pData[i];
        }
        
        /* Push to CAN CoreX */
        CCX_RX_PushMsg(&CAN1_instance, &msg);
        
        /* Clear interrupt */
        CAN_clearInterruptStatus(CANA_BASE, RX_MSG_OBJ_ID);
    }
    
    /* Clear global interrupt */
    CAN_clearGlobalInterruptStatus(CANA_BASE, CAN_GLOBAL_INT_CANINT0);
    
    /* Acknowledge PIE interrupt */
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

/* ========================================================================
 * Initialization
 * ======================================================================== */

void CAN_App_Init(void)
{
    /* Register system tick */
    CCX_tick_variable_register(&system_tick_ms);
    
    /* Initialize CAN peripheral */
    CAN_initModule(CANA_BASE);
    
    /* Set bitrate: 500 kbit/s */
    CAN_setBitRate(CANA_BASE, 
                   CM_CLK_FREQ,
                   500000,  /* 500 kbit/s */
                   16);     /* Time quanta */
    
    /* Enable interrupts */
    CAN_enableInterrupt(CANA_BASE, 
                        CAN_INT_IE0 | CAN_INT_ERROR | CAN_INT_STATUS);
    
    /* Register interrupt handler */
    Interrupt_register(INT_CANA0, CAN_ISR);
    Interrupt_enable(INT_CANA0);
    
    /* Enable global CAN interrupt */
    CAN_enableGlobalInterrupt(CANA_BASE, CAN_GLOBAL_INT_CANINT0);
    
    /* Setup RX message object - accept all Standard IDs */
    CAN_setupMessageObject(CANA_BASE,
                          RX_MSG_OBJ_ID,
                          0x000,                    /* ID (don't care with mask) */
                          CAN_MSG_FRAME_STD,
                          CAN_MSG_OBJ_TYPE_RX,
                          0x000,                    /* Mask (accept all) */
                          CAN_MSG_OBJ_RX_INT_ENABLE | 
                          CAN_MSG_OBJ_USE_ID_FILTER,
                          8);
    
    /* Setup RX message object for Extended IDs */
    CAN_setupMessageObject(CANA_BASE,
                          RX_MSG_OBJ_ID + 1,
                          0x00000000,               /* ID (don't care) */
                          CAN_MSG_FRAME_EXT,        /* Extended frame */
                          CAN_MSG_OBJ_TYPE_RX,
                          0x00000000,               /* Mask (accept all) */
                          CAN_MSG_OBJ_RX_INT_ENABLE |
                          CAN_MSG_OBJ_USE_ID_FILTER |
                          CAN_MSG_OBJ_USE_EXT_FILTER,
                          8);
    
    /* Initialize CAN CoreX */
    CCX_Init(&CAN1_instance,
             CAN1_rx_table,
             CAN1_tx_table,
             CAN1_RX_END,
             CAN1_TX_END,
             CAN_send_message,
             CAN_bus_check,
             CAN1_rx_unreg_parser);
    
    /* Start CAN module */
    CAN_startModule(CANA_BASE);
}

/* ========================================================================
 * Main Processing Loop
 * ======================================================================== */

void CAN_App_Process(void)
{
    CCX_Poll(&CAN1_instance);
}

/* ========================================================================
 * System Tick (call from timer interrupt)
 * ======================================================================== */

void CAN_App_SysTick(void)
{
    system_tick_ms++;
}
```

---

### 3.2 Bus Monitoring & Statistics

Add bus monitoring for TI platform:

```c
/* ========================================================================
 * Bus Monitoring Hardware Interface
 * ======================================================================== */

static CCX_BusState_t CAN_get_bus_state(const CCX_instance_t *Instance)
{
    (void)Instance;
    
    uint32_t status = CAN_getStatus(CANA_BASE);
    
    /* Check for Bus-Off */
    if (status & CAN_STATUS_BUS_OFF) {
        return CCX_BUS_STATE_OFF;
    }
    
    /* Get error counters */
    uint32_t esr = HWREG(CANA_BASE + CAN_O_ERR);
    uint8_t tec = (esr & CAN_ERR_TEC_M) >> CAN_ERR_TEC_S;
    uint8_t rec = (esr & CAN_ERR_REC_M) >> CAN_ERR_REC_S;
    
    if (tec > 127 || rec > 127) {
        return CCX_BUS_STATE_PASSIVE;
    } else if (tec > 96 || rec > 96) {
        return CCX_BUS_STATE_WARNING;
    }
    
    return CCX_BUS_STATE_ACTIVE;
}

static void CAN_get_error_counters(const CCX_instance_t *Instance,
                                   CCX_ErrorCounters_t *Counters)
{
    (void)Instance;
    
    uint32_t esr = HWREG(CANA_BASE + CAN_O_ERR);
    Counters->TEC = (esr & CAN_ERR_TEC_M) >> CAN_ERR_TEC_S;
    Counters->REC = (esr & CAN_ERR_REC_M) >> CAN_ERR_REC_S;
}

static void CAN_request_recovery(const CCX_instance_t *Instance)
{
    (void)Instance;
    
    /* Reset CAN module to recover from bus-off */
    CAN_initModule(CANA_BASE);
    CAN_setBitRate(CANA_BASE, CM_CLK_FREQ, 500000, 16);
    CAN_startModule(CANA_BASE);
}

/* Initialize bus monitoring in CAN_App_Init() */
void CAN_App_Init(void)
{
    /* ... existing initialization ... */
    
    static CCX_BusMonitor_t CAN1_monitor;
    
    CCX_BusMonitor_Init(
        &CAN1_instance,
        &CAN1_monitor,
        CAN_get_bus_state,
        CAN_get_error_counters,
        CAN_request_recovery,
        10,
        60000,
        1,
        5
    );
    
    /* ... start CAN module ... */
}
```

---

## Summary

This implementation guide covers:

1. **Recommended file structure** for maintainable CAN applications
2. **STM32 HAL implementation** with:
   - Dual CAN instance support (CAN1/CAN2)
   - Standard and Extended ID handling
   - Two callback styles (separate vs. shared)
   - Complete bus monitoring integration
3. **TI Connectivity Manager implementation** with:
   - Extended ID support
   - Hardware-specific message object handling
   - Bus monitoring for TI platform

**Key takeaways:**
- Always enable required interrupts in your HAL/Driverlib configuration
- Use separate instances for independent CAN peripherals
- Implement hardware interface functions specific to your platform
- Bus monitoring provides automatic recovery and health tracking
- Statistics help diagnose communication issues

For advanced features like ISO-TP and Network replication, refer to `ADVANCED_FEATURES.md` (future document).
