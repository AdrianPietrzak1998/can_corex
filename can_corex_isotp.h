/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Author: Adrian Pietrzak
 * GitHub: https://github.com/AdrianPietrzak1998
 * Created: Jan 22, 2026
 */

#ifndef CAN_COREX_CAN_COREX_ISOTP_H_
#define CAN_COREX_CAN_COREX_ISOTP_H_

#include "can_corex.h"

/**
 * @def CCX_ISOTP_MAX_DATA_SIZE
 * @brief Maximum size of ISO-TP message payload in bytes
 *
 * Default: 4095 bytes (ISO 15765-2 standard maximum)
 */
#ifndef CCX_ISOTP_MAX_DATA_SIZE
#define CCX_ISOTP_MAX_DATA_SIZE 4095
#endif

/**
 * @brief ISO-TP frame types (PCI - Protocol Control Information)
 */
typedef enum
{
    CCX_ISOTP_PCI_SF = 0x00, /* Single Frame: [0x0N | data] */
    CCX_ISOTP_PCI_FF = 0x10, /* First Frame: [0x1L LL | data] */
    CCX_ISOTP_PCI_CF = 0x20, /* Consecutive Frame: [0x2N | data] */
    CCX_ISOTP_PCI_FC = 0x30  /* Flow Control: [0x30 | FS | BS | STmin] */
} CCX_ISOTP_PCI_t;

/**
 * @brief ISO-TP Flow Control Flow Status values
 */
typedef enum
{
    CCX_ISOTP_FC_CTS = 0x00,   /* Continue To Send */
    CCX_ISOTP_FC_WAIT = 0x01,  /* Wait */
    CCX_ISOTP_FC_OVFLW = 0x02  /* Overflow */
} CCX_ISOTP_FC_FS_t;

/**
 * @brief ISO-TP error codes
 */
typedef enum
{
    CCX_ISOTP_OK = 0,
    CCX_ISOTP_ERROR_NULL_PTR,
    CCX_ISOTP_ERROR_INVALID_ARG,
    CCX_ISOTP_ERROR_TIMEOUT,
    CCX_ISOTP_ERROR_OVERFLOW,
    CCX_ISOTP_ERROR_BUSY,
    CCX_ISOTP_ERROR_SEQUENCE,
    CCX_ISOTP_ERROR_BUFFER_TOO_SMALL
} CCX_ISOTP_Status_t;

/**
 * @brief ISO-TP TX state machine states
 */
typedef enum
{
    CCX_ISOTP_TX_STATE_IDLE = 0,
    CCX_ISOTP_TX_STATE_SENDING_SF,
    CCX_ISOTP_TX_STATE_SENDING_FF,
    CCX_ISOTP_TX_STATE_WAIT_FC,
    CCX_ISOTP_TX_STATE_SENDING_CF
} CCX_ISOTP_TX_State_t;

/**
 * @brief ISO-TP RX state machine states
 */
typedef enum
{
    CCX_ISOTP_RX_STATE_IDLE = 0,
    CCX_ISOTP_RX_STATE_RECEIVING_CF
} CCX_ISOTP_RX_State_t;

/**
 * @brief ISO-TP padding configuration
 */
typedef struct
{
    uint8_t Enable;      /* 1 = pad frames to 8 bytes, 0 = no padding */
    uint8_t PaddingByte; /* Byte value used for padding (e.g., 0xAA, 0xCC, 0x55) */
} CCX_ISOTP_Padding_t;

typedef struct CCX_ISOTP_TX_t CCX_ISOTP_TX_t;
typedef struct CCX_ISOTP_RX_t CCX_ISOTP_RX_t;

/**
 * @brief ISO-TP TX configuration structure
 */
typedef struct
{
    CCX_instance_t *CanInstance; /* Pointer to CAN CoreX instance */
    uint32_t TxID;                /* CAN ID for transmitting data */
    uint32_t RxID_FC;             /* CAN ID for receiving Flow Control frames */
    uint8_t IDE_TxID : 1;         /* 0 = Standard ID (11-bit), 1 = Extended ID (29-bit) for TxID */
    uint8_t IDE_RxID_FC : 1;      /* 0 = Standard ID (11-bit), 1 = Extended ID (29-bit) for RxID_FC */
    uint8_t BS;                   /* Block Size: number of CF before expecting FC (0 = no limit) */
    uint8_t STmin;                /* Separation Time minimum (0-127ms or 0xF1-0xF9 for 100-900us) */
    CCX_TIME_t N_As;              /* Timeout for TX of SF/FF/CF (default: 1000ms) */
    CCX_TIME_t N_Bs;              /* Timeout for reception of FC after FF/CF (default: 1000ms) */
    CCX_TIME_t N_Cs;              /* Timeout between CF transmissions (default: 1000ms) */
    CCX_ISOTP_Padding_t Padding;  /* Padding configuration */

    /* Callbacks */
    void (*OnTransmitComplete)(CCX_ISOTP_TX_t *Instance);
    void (*OnError)(CCX_ISOTP_TX_t *Instance, CCX_ISOTP_Status_t Error);
} CCX_ISOTP_TX_Config_t;

/**
 * @brief ISO-TP RX configuration structure
 */
typedef struct
{
    CCX_instance_t *CanInstance; /* Pointer to CAN CoreX instance */
    uint32_t RxID;                /* CAN ID for receiving data */
    uint32_t TxID;                /* CAN ID for transmitting Flow Control */
    uint8_t IDE_RxID : 1;         /* 0 = Standard ID (11-bit), 1 = Extended ID (29-bit) for RxID */
    uint8_t IDE_TxID : 1;         /* 0 = Standard ID (11-bit), 1 = Extended ID (29-bit) for TxID */
    uint8_t BS;                   /* Block Size to request in FC (0 = no limit) */
    uint8_t STmin;                /* Separation Time minimum to request in FC */
    CCX_TIME_t N_Ar;              /* Timeout for RX of SF/FF/CF (default: 1000ms) */
    CCX_TIME_t N_Br;              /* Timeout for transmission of FC after FF/CF (default: 1000ms) */
    CCX_TIME_t N_Cr;              /* Timeout between received CF (default: 1000ms) */
    CCX_ISOTP_Padding_t Padding;  /* Padding configuration */

    uint8_t *RxBuffer;                    /* Buffer for received data */
    uint16_t RxBufferSize;                /* Size of RX buffer */
    uint16_t ProgressCallbackInterval;    /* Call progress callback every N bytes (0 = disabled) */

    /* Callbacks */
    void (*OnReceiveComplete)(CCX_ISOTP_RX_t *Instance, const uint8_t *Data, uint16_t Length);
    void (*OnReceiveProgress)(CCX_ISOTP_RX_t *Instance, uint16_t BytesReceived, uint16_t TotalLength);
    void (*OnError)(CCX_ISOTP_RX_t *Instance, CCX_ISOTP_Status_t Error);
} CCX_ISOTP_RX_Config_t;

/**
 * @brief ISO-TP TX instance structure
 */
struct CCX_ISOTP_TX_t
{
    CCX_ISOTP_TX_Config_t Config;
    CCX_ISOTP_TX_State_t State;

    const uint8_t *TxData;         /* Pointer to data being transmitted */
    uint16_t TxDataLength;         /* Total length of data to transmit */
    uint16_t TxDataOffset;         /* Current offset in TxData */
    uint8_t SequenceNumber;        /* CF sequence number (0-15) */
    uint8_t BlockCounter;          /* Counter for BS */
    CCX_TIME_t LastTick;           /* Last activity timestamp */
    uint8_t WaitFramesRemaining;   /* Number of WAIT FC frames we can tolerate */
};

/**
 * @brief ISO-TP RX instance structure
 */
struct CCX_ISOTP_RX_t
{
    CCX_ISOTP_RX_Config_t Config;
    CCX_ISOTP_RX_State_t State;

    uint16_t RxDataLength;           /* Total expected length */
    uint16_t RxDataOffset;           /* Current offset in RxBuffer */
    uint8_t SequenceNumber;          /* Expected CF sequence number (0-15) */
    uint8_t BlockCounter;            /* Counter for BS */
    CCX_TIME_t LastTick;             /* Last activity timestamp */
    uint16_t LastProgressCallback;   /* Offset at last progress callback */
};

/**
 * @brief Initialize ISO-TP TX instance
 *
 * @param Instance Pointer to TX instance structure
 * @param Config Pointer to TX configuration
 * @return CCX_ISOTP_OK on success, error code otherwise
 */
CCX_ISOTP_Status_t CCX_ISOTP_TX_Init(CCX_ISOTP_TX_t *Instance, const CCX_ISOTP_TX_Config_t *Config);

/**
 * @brief Transmit data using ISO-TP
 *
 * @param Instance Pointer to TX instance
 * @param Data Pointer to data to transmit
 * @param Length Length of data in bytes (1-4095)
 * @return CCX_ISOTP_OK on success, error code otherwise
 *
 * @note Data pointer must remain valid until OnTransmitComplete is called
 */
CCX_ISOTP_Status_t CCX_ISOTP_Transmit(CCX_ISOTP_TX_t *Instance, const uint8_t *Data, uint16_t Length);

/**
 * @brief Poll ISO-TP TX instance (call periodically from main loop)
 *
 * @param Instance Pointer to TX instance
 */
void CCX_ISOTP_TX_Poll(CCX_ISOTP_TX_t *Instance);

/**
 * @brief Parser function for Flow Control frames (for TX side)
 *
 * This function should be called from CAN CoreX RX table parser.
 * Use CCX_ISOTP_TX_FC_TABLE_ENTRY macro to generate RX table entry.
 *
 * @param CanInstance CAN CoreX instance (unused, for compatibility)
 * @param Msg Received CAN message (Flow Control)
 * @param Slot RX table slot (unused, for compatibility)
 */
void CCX_ISOTP_TX_FC_Parser(const CCX_instance_t *CanInstance, CCX_message_t *Msg, uint16_t Slot, void *UserData);

/**
 * @brief Initialize ISO-TP RX instance
 *
 * @param Instance Pointer to RX instance structure
 * @param Config Pointer to RX configuration
 * @return CCX_ISOTP_OK on success, error code otherwise
 */
CCX_ISOTP_Status_t CCX_ISOTP_RX_Init(CCX_ISOTP_RX_t *Instance, const CCX_ISOTP_RX_Config_t *Config);

/**
 * @brief Parser function for CAN CoreX RX table
 *
 * This function should be called from CAN CoreX RX table parser.
 * Use CCX_ISOTP_RX_TABLE_ENTRY macro to generate RX table entry.
 *
 * @param CanInstance CAN CoreX instance (unused, for compatibility)
 * @param Msg Received CAN message
 * @param Slot RX table slot (unused, for compatibility)
 */
void CCX_ISOTP_RX_Parser(const CCX_instance_t *CanInstance, CCX_message_t *Msg, uint16_t Slot, void *UserData);

/**
 * @brief Poll ISO-TP RX instance (call periodically from main loop)
 *
 * @param Instance Pointer to RX instance
 */
void CCX_ISOTP_RX_Poll(CCX_ISOTP_RX_t *Instance);

/**
 * @brief Macro to generate CAN CoreX RX table entry for ISO-TP RX instance
 *
 * @param isotp_rx_ptr Pointer to CCX_ISOTP_RX_t instance
 * @param can_id CAN ID for receiving ISO-TP data frames
 * @param ide_flag 0 = Standard ID (11-bit), 1 = Extended ID (29-bit)
 *
 * Note: Uses CCX_DLC_ANY to accept any DLC (wildcard matching in CAN CoreX)
 *
 * Usage example:
 * CCX_ISOTP_RX_t isotp_rx;
 * CCX_RX_table_t rx_table[] = {
 *     CCX_ISOTP_RX_TABLE_ENTRY(&isotp_rx, 0x123, 0),           // Standard ID
 *     CCX_ISOTP_RX_TABLE_ENTRY(&isotp_rx, 0x18DA00F1, 1)       // Extended ID
 * };
 */
#define CCX_ISOTP_RX_TABLE_ENTRY(isotp_rx_ptr, can_id, ide_flag)                                                      \
    {                                                                                                                  \
        .ID = (can_id), .DLC = CCX_DLC_ANY, .IDE_flag = (ide_flag), .UserData = (isotp_rx_ptr), .TimeOut = 0,        \
        .Parser = CCX_ISOTP_RX_Parser, .TimeoutCallback = NULL, .LastTick = 0                                         \
    }

/**
 * @brief Macro to generate CAN CoreX RX table entry for ISO-TP TX Flow Control
 *
 * @param isotp_tx_ptr Pointer to CCX_ISOTP_TX_t instance
 * @param fc_can_id CAN ID for receiving Flow Control frames
 * @param ide_flag 0 = Standard ID (11-bit), 1 = Extended ID (29-bit)
 *
 * Note: Uses CCX_DLC_ANY - FC can be sent with or without padding
 *
 * Usage example:
 * CCX_ISOTP_TX_t isotp_tx;
 * CCX_ISOTP_RX_t isotp_rx;
 * CCX_RX_table_t rx_table[] = {
 *     CCX_ISOTP_RX_TABLE_ENTRY(&isotp_rx, 0x123, 0),           // Standard ID
 *     CCX_ISOTP_TX_FC_TABLE_ENTRY(&isotp_tx, 0x321, 0)         // Standard ID for FC
 * };
 */
#define CCX_ISOTP_TX_FC_TABLE_ENTRY(isotp_tx_ptr, fc_can_id, ide_flag)                                                \
    {                                                                                                                  \
        .ID = (fc_can_id), .DLC = CCX_DLC_ANY, .IDE_flag = (ide_flag), .UserData = (isotp_tx_ptr), .TimeOut = 0,     \
        .Parser = CCX_ISOTP_TX_FC_Parser, .TimeoutCallback = NULL, .LastTick = 0                                      \
    }

#endif /* CAN_COREX_CAN_COREX_ISOTP_H_ */
