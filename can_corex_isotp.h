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
 * @def CCX_ISOTP_MAX_CLASSIC_DATA_SIZE
 * @brief Maximum ISO-TP payload for classic CAN instances
 *
 * Applies to sessions configured with
 * `FrameFormat = CCX_FRAME_FORMAT_CLASSIC`,
 * even in builds where `CCX_ENABLE_CANFD=1`.
 */
#ifndef CCX_ISOTP_MAX_CLASSIC_DATA_SIZE
#define CCX_ISOTP_MAX_CLASSIC_DATA_SIZE 4095U
#endif

/**
 * @def CCX_ISOTP_MAX_FD_DATA_SIZE
 * @brief Maximum ISO-TP payload for CAN FD instances
 *
 * This limit applies only to ISO-TP sessions configured for
 * `FrameFormat = CCX_FRAME_FORMAT_FD` or `CCX_FRAME_FORMAT_FD_BRS`.
 * Override it if the application wants to cap FD transfers below the
 * protocol's 32-bit length field.
 */
#if CCX_ENABLE_CANFD
#ifndef CCX_ISOTP_MAX_FD_DATA_SIZE
#define CCX_ISOTP_MAX_FD_DATA_SIZE UINT32_MAX
#endif
#endif

/**
 * @def CCX_ISOTP_MAX_DATA_SIZE
 * @brief Build-wide compatibility alias for the active default limit
 */
#ifndef CCX_ISOTP_MAX_DATA_SIZE
#if CCX_ENABLE_CANFD
#define CCX_ISOTP_MAX_DATA_SIZE CCX_ISOTP_MAX_FD_DATA_SIZE
#else
#define CCX_ISOTP_MAX_DATA_SIZE CCX_ISOTP_MAX_CLASSIC_DATA_SIZE
#endif
#endif

#if CCX_ENABLE_CANFD
typedef uint32_t CCX_ISOTP_Length_t;
#else
typedef uint16_t CCX_ISOTP_Length_t;
#endif

/**
 * @brief ISO-TP length type
 *
 * In classic-only builds this stays `uint16_t`.
 * In FD builds it becomes `uint32_t`, allowing payload lengths above `4095`
 * on FD-configured ISO-TP instances.
 */

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

typedef enum
{
    CCX_ISOTP_LENGTH_FORMAT_STANDARD = 0,
    CCX_ISOTP_LENGTH_FORMAT_EXTENDED
} CCX_ISOTP_LengthFormat_t;

typedef enum
{
    CCX_ISOTP_TX_DL_8 = 8,
    CCX_ISOTP_TX_DL_12 = 12,
    CCX_ISOTP_TX_DL_16 = 16,
    CCX_ISOTP_TX_DL_20 = 20,
    CCX_ISOTP_TX_DL_24 = 24,
    CCX_ISOTP_TX_DL_32 = 32,
    CCX_ISOTP_TX_DL_48 = 48,
    CCX_ISOTP_TX_DL_64 = 64
} CCX_ISOTP_TxDL_t;

/**
 * @brief Legal CAN FD link-layer payload sizes for ISO-TP sessions
 *
 * `TxDL` is used only for FD ISO-TP sessions.
 * Classic ISO-TP sessions always use `8` bytes as the link-layer payload.
 */

/**
 * @brief ISO-TP Flow Control Flow Status values
 */
typedef enum
{
    CCX_ISOTP_FC_CTS = 0x00,  /* Continue To Send */
    CCX_ISOTP_FC_WAIT = 0x01, /* Wait */
    CCX_ISOTP_FC_OVFLW = 0x02 /* Overflow */
} CCX_ISOTP_FC_FS_t;

/**
 * @brief ISO-TP error codes
 */
typedef enum
{
    CCX_ISOTP_OK = 0,
    CCX_ISOTP_ERROR_NULL_PTR,
    CCX_ISOTP_ERROR_INVALID_ARG,
    CCX_ISOTP_ERROR_TIMEOUT_FC,
    CCX_ISOTP_ERROR_TIMEOUT = CCX_ISOTP_ERROR_TIMEOUT_FC, /* Legacy alias */
    CCX_ISOTP_ERROR_OVERFLOW,
    CCX_ISOTP_ERROR_BUSY,
    CCX_ISOTP_ERROR_SEQUENCE,
    CCX_ISOTP_ERROR_BUFFER_TOO_SMALL,
    CCX_ISOTP_ERROR_ABORTED,
    CCX_ISOTP_ERROR_WAIT_EXCEEDED,
    CCX_ISOTP_ERROR_TIMEOUT_CF_TX,
    CCX_ISOTP_ERROR_TIMEOUT_CF_RX,
#if CCX_ENABLE_CANFD
    CCX_ISOTP_ERROR_FD_NOT_SUPPORTED, /* Reserved for legacy compatibility */
#endif
} CCX_ISOTP_Status_t;

/**
 * @brief ISO-TP TX state machine states
 */
typedef enum
{
    CCX_ISOTP_TX_STATE_IDLE = 0,
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
 *
 * Padding affects CAN frame payload length, not the reported ISO-TP PDU length.
 * Classic instances pad to `8`. FD instances pad to the configured `TxDL`.
 */
typedef struct
{
    uint8_t Enable;      /* 1 = pad frames to target CAN payload length, 0 = no padding */
    uint8_t PaddingByte; /* Byte value used for padding (e.g., 0xAA, 0xCC, 0x55) */
} CCX_ISOTP_Padding_t;

typedef struct CCX_ISOTP_TX_t CCX_ISOTP_TX_t;
typedef struct CCX_ISOTP_RX_t CCX_ISOTP_RX_t;

/**
 * @brief ISO-TP TX configuration structure
 *
 * The effective transmit payload limit depends on `FrameFormat`:
 * - `CCX_FRAME_FORMAT_CLASSIC`: `CCX_ISOTP_MAX_CLASSIC_DATA_SIZE`
 * - `CCX_FRAME_FORMAT_FD` / `CCX_FRAME_FORMAT_FD_BRS`: `CCX_ISOTP_MAX_FD_DATA_SIZE`
 *
 * @note UserData Example:
 * @code
 * typedef struct {
 *     int session_id;
 *     void (*log_function)(const char *msg);
 * } MyContext_t;
 *
 * MyContext_t my_ctx = {.session_id = 42, .log_function = printf};
 *
 * CCX_ISOTP_TX_Config_t cfg = {
 *     // ... other fields ...
 *     .UserData = &my_ctx,
 *     .OnTransmitComplete = my_tx_callback,
 *     .OnError = my_error_callback
 * };
 *
 * void my_tx_callback(CCX_ISOTP_TX_t *Instance, void *UserData) {
 *     MyContext_t *ctx = (MyContext_t *)UserData;
 *     ctx->log_function("Session %d: TX complete\n", ctx->session_id);
 * }
 * @endcode
 */
typedef struct
{
    CCX_instance_t *CanInstance; /* Pointer to CAN CoreX instance */
    uint32_t TxID;               /* CAN ID for transmitting data */
    uint32_t RxID_FC;            /* CAN ID for receiving Flow Control frames */
    uint8_t IDE_TxID : 1;        /* use CCX_ide_t values: CCX_ID_STANDARD / CCX_ID_EXTENDED */
    uint8_t IDE_RxID_FC : 1;     /* use CCX_ide_t values: CCX_ID_STANDARD / CCX_ID_EXTENDED */
#if CCX_ENABLE_CANFD
    CCX_frame_format_t FrameFormat : 2; /* Session transport format: CLASSIC / FD / FD_BRS */
    uint8_t TxDL;                       /* FD only: link-layer payload 8,12,16,20,24,32,48,64 */
#endif
    uint8_t BS;                  /* Block Size: number of CF before expecting FC (0 = no limit) */
    uint8_t STmin;               /* Separation Time minimum (0-127ms or 0xF1-0xF9 for 100-900us) */
    CCX_TIME_t N_As;             /* Informational timeout for TX of SF/FF/CF; not enforced internally */
    CCX_TIME_t N_Bs;             /* Timeout for reception of FC after FF/CF (default: 1000ms) */
    CCX_TIME_t N_Cs;             /* Timeout between CF transmissions (default: 1000ms) */
    uint8_t MaxWaitFrames;       /* Number of FC.WAIT frames tolerated before aborting (0 = default 10) */
    CCX_ISOTP_Padding_t Padding; /* Padding configuration */

    void *UserData; /* User context pointer passed to callbacks */

    /* Callbacks */
    void (*OnTransmitComplete)(CCX_ISOTP_TX_t *Instance, void *UserData);
    void (*OnError)(CCX_ISOTP_TX_t *Instance, CCX_ISOTP_Status_t Error, void *UserData);
} CCX_ISOTP_TX_Config_t;

/**
 * @brief ISO-TP RX configuration structure
 *
 * The effective receive payload limit depends on `FrameFormat`:
 * - `CCX_FRAME_FORMAT_CLASSIC`: `CCX_ISOTP_MAX_CLASSIC_DATA_SIZE`
 * - `CCX_FRAME_FORMAT_FD` / `CCX_FRAME_FORMAT_FD_BRS`: `CCX_ISOTP_MAX_FD_DATA_SIZE`
 *
 * @note UserData and Progress Callback Example:
 * @code
 * typedef struct {
 *     FILE *log_file;
 *     uint32_t total_bytes_received;
 * } RxContext_t;
 *
 * RxContext_t rx_ctx = {.log_file = fopen("isotp.log", "w"), .total_bytes_received = 0};
 *
 * CCX_ISOTP_RX_Config_t cfg = {
 *     // ... other fields ...
 *     .ProgressCallbackInterval = 512,  // Report every 512 bytes
 *     .UserData = &rx_ctx,
 *     .OnReceiveComplete = my_rx_complete,
 *     .OnReceiveProgress = my_rx_progress
 * };
 *
 * void my_rx_progress(CCX_ISOTP_RX_t *Instance, CCX_ISOTP_Length_t BytesReceived,
 * CCX_ISOTP_Length_t TotalLength,
 * void *UserData) {
 *     RxContext_t *ctx = (RxContext_t *)UserData; ctx->total_bytes_received += BytesReceived;
 *     fprintf(ctx->log_file, "Progress: %d/%d bytes\n", ctx->total_bytes_received, TotalLength);
 * }
 * @endcode
 *
 * @note `OnReceiveStart` is called once after a valid First Frame is accepted,
 * the total payload length is known, and the transfer state is initialized.
 *
 * @note `OnReceiveProgress` receives `BytesReceived` as the delta since the previous
 * callback, not an absolute offset from the start of the transfer.
 */
typedef struct
{
    CCX_instance_t *CanInstance; /* Pointer to CAN CoreX instance */
    uint32_t RxID;               /* CAN ID for receiving data */
    uint32_t TxID;               /* CAN ID for transmitting Flow Control */
    uint8_t IDE_RxID : 1;        /* use CCX_ide_t values: CCX_ID_STANDARD / CCX_ID_EXTENDED */
    uint8_t IDE_TxID : 1;        /* use CCX_ide_t values: CCX_ID_STANDARD / CCX_ID_EXTENDED */
#if CCX_ENABLE_CANFD
    CCX_frame_format_t FrameFormat : 2; /* Expected transport format of received ISO-TP frames */
    uint8_t FC_TxDL;                    /* FD only: link-layer payload used for transmitted FC frames */
#endif
    uint8_t BS;                  /* Block Size to request in FC (0 = no limit) */
    uint8_t STmin;               /* Separation Time minimum to request in FC */
    CCX_TIME_t N_Ar;             /* Informational timeout for RX of SF/FF/CF; not enforced internally */
    CCX_TIME_t N_Br;             /* Informational timeout for FC transmission after FF/CF; not enforced internally */
    CCX_TIME_t N_Cr;             /* Timeout between received CF (default: 1000ms) */
    CCX_ISOTP_Padding_t Padding; /* Padding configuration */

    uint8_t *RxBuffer;                           /* Buffer for received data */
    CCX_ISOTP_Length_t RxBufferSize;             /* Size of RX buffer */
    CCX_ISOTP_Length_t ProgressCallbackInterval; /* Call progress callback every N newly received bytes (0 = disabled) */

    void *UserData; /* User context pointer passed to callbacks */

    /* Callbacks */
    void (*OnReceiveStart)(CCX_ISOTP_RX_t *Instance, CCX_ISOTP_Length_t TotalLength, void *UserData);
    void (*OnReceiveComplete)(CCX_ISOTP_RX_t *Instance, const uint8_t *Data, CCX_ISOTP_Length_t Length, void *UserData);
    void (*OnReceiveProgress)(CCX_ISOTP_RX_t *Instance, CCX_ISOTP_Length_t BytesReceived,
                              CCX_ISOTP_Length_t TotalLength, void *UserData);
    void (*OnError)(CCX_ISOTP_RX_t *Instance, CCX_ISOTP_Status_t Error, void *UserData);
} CCX_ISOTP_RX_Config_t;

/**
 * @brief ISO-TP TX instance structure
 */
struct CCX_ISOTP_TX_t
{
    CCX_ISOTP_TX_Config_t Config;
    CCX_ISOTP_TX_State_t State;

    const uint8_t *TxData;           /* Pointer to data being transmitted */
    CCX_ISOTP_Length_t TxDataLength; /* Total length of data to transmit */
    CCX_ISOTP_Length_t TxDataOffset; /* Current offset in TxData */
    uint8_t SequenceNumber;          /* CF sequence number (0-15) */
    uint8_t BlockCounter;            /* Counter for BS */
    CCX_TIME_t LastTick;             /* Last activity timestamp */
    uint8_t WaitFramesRemaining;     /* Number of WAIT FC frames we can tolerate */
    CCX_TIME_VALUE_t STmin_ms;       /* STmin value from FC converted to milliseconds */
    uint8_t ActiveTxDL;              /* Active link-layer payload for TX */
    uint8_t MaxWaitFrames;           /* Effective FC.WAIT tolerance for this instance */
    CCX_ISOTP_LengthFormat_t LengthFormat;
};

/**
 * @brief ISO-TP RX instance structure
 */
struct CCX_ISOTP_RX_t
{
    CCX_ISOTP_RX_Config_t Config;
    CCX_ISOTP_RX_State_t State;

    CCX_ISOTP_Length_t RxDataLength;         /* Total expected length */
    CCX_ISOTP_Length_t RxDataOffset;         /* Current offset in RxBuffer */
    uint8_t SequenceNumber;                  /* Expected CF sequence number (0-15) */
    uint8_t BlockCounter;                    /* Counter for BS */
    CCX_TIME_t LastTick;                     /* Last activity timestamp */
    CCX_ISOTP_Length_t LastProgressCallback; /* Offset at last progress callback */
    uint8_t ActiveRxDL;                      /* CAN_DL announced by FF */
    CCX_ISOTP_LengthFormat_t LengthFormat;
};

/**
 * @brief Initialize ISO-TP TX instance
 *
 * @param Instance Pointer to TX instance structure
 * @param Config Pointer to TX configuration
 * @return CCX_ISOTP_OK on success, error code otherwise
 *
 * @note In FD builds, `Config->TxDL` is validated only for FD sessions.
 * Classic sessions keep the classic `4095`-byte transport limit even when
 * `CCX_ENABLE_CANFD=1`.
 */
CCX_ISOTP_Status_t CCX_ISOTP_TX_Init(CCX_ISOTP_TX_t *Instance, const CCX_ISOTP_TX_Config_t *Config);
CCX_ISOTP_Status_t CCX_ISOTP_TX_Abort(CCX_ISOTP_TX_t *Instance);

/**
 * @brief Transmit data using ISO-TP
 *
 * @param Instance Pointer to TX instance
 * @param Data Pointer to data to transmit
 * @param Length Length of data in bytes
 * @return CCX_ISOTP_OK on success, error code
 * otherwise
 *
 * @note Data pointer must remain valid until OnTransmitComplete is called
 * @note Header selection is based on ISO-TP payload length before padding.
 * For example, in FD mode payloads `1..7` still use the standard single-frame
 * header even if CAN padding expands the frame to a larger `TxDL`.
 */
CCX_ISOTP_Status_t CCX_ISOTP_Transmit(CCX_ISOTP_TX_t *Instance, const uint8_t *Data, CCX_ISOTP_Length_t Length);

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
 *
 * @note In FD builds, `Config->FC_TxDL` is used only for transmitted Flow Control
 * frames when the RX session itself is configured for FD.
 */
CCX_ISOTP_Status_t CCX_ISOTP_RX_Init(CCX_ISOTP_RX_t *Instance, const CCX_ISOTP_RX_Config_t *Config);
CCX_ISOTP_Status_t CCX_ISOTP_RX_Abort(CCX_ISOTP_RX_t *Instance);

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
 * and defaults to classic frame format in FD-capable builds.
 *
 * Usage example:
 * CCX_ISOTP_RX_t isotp_rx;
 * CCX_RX_table_t rx_table[] = {
 *     CCX_ISOTP_RX_TABLE_ENTRY(&isotp_rx, 0x123, 0),           // Standard ID
 *     CCX_ISOTP_RX_TABLE_ENTRY(&isotp_rx, 0x18DA00F1, 1)       // Extended ID
 * };
 */
#if CCX_ENABLE_CANFD
#define CCX_ISOTP_RX_TABLE_ENTRY(isotp_rx_ptr, can_id, ide_flag)                                                       \
    {                                                                                                                  \
        .ID = (can_id), .DLC = CCX_DLC_ANY, .IDE_flag = (ide_flag), .FrameFormat = CCX_FRAME_FORMAT_CLASSIC,           \
        .UserData = (isotp_rx_ptr), .TimeOut = 0, .Parser = CCX_ISOTP_RX_Parser, .TimeoutCallback = NULL,              \
        .LastTick = 0                                                                                                  \
    }

/**
 * @brief Macro to generate CAN CoreX RX table entry for ISO-TP TX Flow Control
 *
 * @param isotp_tx_ptr Pointer to
 * CCX_ISOTP_TX_t instance
 * @param fc_can_id CAN ID for receiving Flow Control frames
 * @param ide_flag 0 = Standard
 * ID (11-bit), 1 = Extended ID (29-bit)
 *
 * Note: Uses CCX_DLC_ANY - FC can be sent with or without padding.
 * In FD-capable builds this macro defaults to classic frame format.
 *
 *
 * Usage example:
 * CCX_ISOTP_TX_t isotp_tx;
 * CCX_ISOTP_RX_t isotp_rx;
 * CCX_RX_table_t rx_table[] = {
 *
 * CCX_ISOTP_RX_TABLE_ENTRY(&isotp_rx, 0x123, 0),           // Standard ID
 *     CCX_ISOTP_TX_FC_TABLE_ENTRY(&isotp_tx,
 * 0x321, 0)         // Standard ID for FC
 * };
 */
#define CCX_ISOTP_TX_FC_TABLE_ENTRY(isotp_tx_ptr, fc_can_id, ide_flag)                                                 \
    {                                                                                                                  \
        .ID = (fc_can_id), .DLC = CCX_DLC_ANY, .IDE_flag = (ide_flag), .FrameFormat = CCX_FRAME_FORMAT_CLASSIC,        \
        .UserData = (isotp_tx_ptr), .TimeOut = 0, .Parser = CCX_ISOTP_TX_FC_Parser, .TimeoutCallback = NULL,           \
        .LastTick = 0                                                                                                  \
    }

#define CCX_ISOTP_RX_TABLE_ENTRY_EX(isotp_rx_ptr, can_id, ide_flag, frame_format)                                      \
    {                                                                                                                  \
        .ID = (can_id), .DLC = CCX_DLC_ANY, .IDE_flag = (ide_flag), .FrameFormat = (frame_format),                     \
        .UserData = (isotp_rx_ptr), .TimeOut = 0, .Parser = CCX_ISOTP_RX_Parser, .TimeoutCallback = NULL,              \
        .LastTick = 0                                                                                                  \
    }

#define CCX_ISOTP_TX_FC_TABLE_ENTRY_EX(isotp_tx_ptr, fc_can_id, ide_flag, frame_format)                                \
    {                                                                                                                  \
        .ID = (fc_can_id), .DLC = CCX_DLC_ANY, .IDE_flag = (ide_flag), .FrameFormat = (frame_format),                  \
        .UserData = (isotp_tx_ptr), .TimeOut = 0, .Parser = CCX_ISOTP_TX_FC_Parser, .TimeoutCallback = NULL,           \
        .LastTick = 0                                                                                                  \
    }
#else
#define CCX_ISOTP_RX_TABLE_ENTRY(isotp_rx_ptr, can_id, ide_flag)                                                       \
    {                                                                                                                  \
        .ID = (can_id), .DLC = CCX_DLC_ANY, .IDE_flag = (ide_flag), .UserData = (isotp_rx_ptr), .TimeOut = 0,          \
        .Parser = CCX_ISOTP_RX_Parser, .TimeoutCallback = NULL, .LastTick = 0                                          \
    }

#define CCX_ISOTP_TX_FC_TABLE_ENTRY(isotp_tx_ptr, fc_can_id, ide_flag)                                                 \
    {                                                                                                                  \
        .ID = (fc_can_id), .DLC = CCX_DLC_ANY, .IDE_flag = (ide_flag), .UserData = (isotp_tx_ptr), .TimeOut = 0,       \
        .Parser = CCX_ISOTP_TX_FC_Parser, .TimeoutCallback = NULL, .LastTick = 0                                       \
    }
#endif

#endif /* CAN_COREX_CAN_COREX_ISOTP_H_ */
