/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Author: Adrian Pietrzak
 * GitHub: https://github.com/AdrianPietrzak1998
 * Created: May 18, 2026
 */

#include "can_corex_utils.h"
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define CCX_UTILS_DOUBLE_U64_MAX_ROUND_DOWN 18446744073709549568.0
#define CCX_UTILS_DOUBLE_S64_MAX_ROUND_DOWN 9223372036854774784.0
#define CCX_UTILS_DOUBLE_S64_MIN_EXACT (-9223372036854775808.0)

#define CCX_UTILS_FLOAT_U64_MAX_ROUND_DOWN 18446744073709551616.0f
#define CCX_UTILS_FLOAT_S64_MAX_ROUND_DOWN 9223372036854775808.0f
#define CCX_UTILS_FLOAT_S64_MIN_EXACT (-9223372036854775808.0f)

static char CCX_UTILS_HexDigit(uint8_t value)
{
    return (value < 10U) ? (char)('0' + value) : (char)('A' + (value - 10U));
}

static int8_t CCX_UTILS_HexValue(char ch)
{
    if ((ch >= '0') && (ch <= '9'))
    {
        return (int8_t)(ch - '0');
    }

    if ((ch >= 'A') && (ch <= 'F'))
    {
        return (int8_t)(ch - 'A' + 10);
    }

    if ((ch >= 'a') && (ch <= 'f'))
    {
        return (int8_t)(ch - 'a' + 10);
    }

    return -1;
}

static CCX_UTILS_Status_t CCX_UTILS_ParseHexU32(const char *text, uint8_t count, uint32_t *value)
{
    uint32_t parsed = 0U;

    for (uint8_t i = 0U; i < count; i++)
    {
        int8_t nibble = CCX_UTILS_HexValue(text[i]);

        if (nibble < 0)
        {
            return CCX_UTILS_WRONG_ARG;
        }

        parsed = (parsed << 4U) | (uint32_t)nibble;
    }

    *value = parsed;
    return CCX_UTILS_OK;
}

static CCX_UTILS_Status_t CCX_UTILS_CheckLineEnd(const char *line)
{
    if (line[0] == '\0')
    {
        return CCX_UTILS_OK;
    }

    if (line[0] == '\r')
    {
        if ((line[1] == '\0') || ((line[1] == '\n') && (line[2] == '\0')))
        {
            return CCX_UTILS_OK;
        }
    }

    if ((line[0] == '\n') && (line[1] == '\0'))
    {
        return CCX_UTILS_OK;
    }

    return CCX_UTILS_WRONG_ARG;
}

CCX_UTILS_Status_t CCX_SLCAN_Parse(const char *line, CCX_message_t *msg)
{
    uint8_t id_chars;
    uint8_t data_start;
    uint32_t id;
    int8_t dlc_nibble;
    CCX_UTILS_Status_t status;
    uint8_t data[8] = {0U};

    if ((line == NULL) || (msg == NULL))
    {
        return CCX_UTILS_NULL_PTR;
    }

    if ((line[0] == 'r') || (line[0] == 'R') || (line[0] == 'd') || (line[0] == 'D') || (line[0] == 'b') ||
        (line[0] == 'B'))
    {
        return CCX_UTILS_UNSUPPORTED;
    }

    if (line[0] == 't')
    {
        id_chars = 3U;
        data_start = 5U;
    }
    else if (line[0] == 'T')
    {
        id_chars = 8U;
        data_start = 10U;
    }
    else
    {
        return CCX_UTILS_WRONG_ARG;
    }

    status = CCX_UTILS_ParseHexU32(&line[1], id_chars, &id);
    if (status != CCX_UTILS_OK)
    {
        return status;
    }

    dlc_nibble = CCX_UTILS_HexValue(line[1U + id_chars]);
    if ((dlc_nibble < 0) || (dlc_nibble > 8))
    {
        return CCX_UTILS_WRONG_ARG;
    }

    if (((line[0] == 't') && (id > 0x7FFUL)) || ((line[0] == 'T') && (id > 0x1FFFFFFFUL)))
    {
        return CCX_UTILS_WRONG_ARG;
    }

    for (uint8_t i = 0U; i < (uint8_t)dlc_nibble; i++)
    {
        int8_t high = CCX_UTILS_HexValue(line[data_start + ((uint8_t)(i * 2U))]);
        int8_t low;

        if (high < 0)
        {
            return CCX_UTILS_WRONG_ARG;
        }

        low = CCX_UTILS_HexValue(line[data_start + ((uint8_t)(i * 2U)) + 1U]);
        if (low < 0)
        {
            return CCX_UTILS_WRONG_ARG;
        }

        data[i] = (uint8_t)(((uint8_t)high << 4U) | (uint8_t)low);
    }

    status = CCX_UTILS_CheckLineEnd(&line[data_start + ((uint8_t)dlc_nibble * 2U)]);
    if (status != CCX_UTILS_OK)
    {
        return status;
    }

    memset(msg, 0, sizeof(*msg));
    msg->ID = id;
    msg->DLC = (uint8_t)dlc_nibble;
    msg->IDE_flag = (line[0] == 'T') ? CCX_ID_EXTENDED : CCX_ID_STANDARD;
    memcpy(msg->Data, data, (uint8_t)dlc_nibble);
#if CCX_ENABLE_CANFD
    msg->FrameFormat = CCX_FRAME_FORMAT_CLASSIC;
    msg->ESI = 0U;
#endif

    return CCX_UTILS_OK;
}

CCX_UTILS_Status_t CCX_SLCAN_Format(const CCX_message_t *msg, char *out, size_t out_size)
{
    uint8_t id_chars;
    uint8_t data_start;
    uint32_t id_limit;
    size_t required;

    if ((msg == NULL) || (out == NULL))
    {
        return CCX_UTILS_NULL_PTR;
    }

#if CCX_ENABLE_CANFD
    if (msg->FrameFormat != CCX_FRAME_FORMAT_CLASSIC)
    {
        return CCX_UTILS_UNSUPPORTED;
    }
#endif

    if (msg->DLC > 8U)
    {
        return CCX_UTILS_WRONG_ARG;
    }

    if (msg->IDE_flag == CCX_ID_EXTENDED)
    {
        id_chars = 8U;
        data_start = 10U;
        id_limit = 0x1FFFFFFFUL;
    }
    else if (msg->IDE_flag == CCX_ID_STANDARD)
    {
        id_chars = 3U;
        data_start = 5U;
        id_limit = 0x7FFUL;
    }
    else
    {
        return CCX_UTILS_WRONG_ARG;
    }

    if (msg->ID > id_limit)
    {
        return CCX_UTILS_WRONG_ARG;
    }

    required = (size_t)data_start + ((size_t)msg->DLC * 2U) + 2U;
    if (out_size < required)
    {
        return CCX_UTILS_BUFFER_TOO_SMALL;
    }

    out[0] = (msg->IDE_flag == CCX_ID_EXTENDED) ? 'T' : 't';

    for (uint8_t i = 0U; i < id_chars; i++)
    {
        uint8_t shift = (uint8_t)((id_chars - 1U - i) * 4U);
        out[1U + i] = CCX_UTILS_HexDigit((uint8_t)((msg->ID >> shift) & 0x0FU));
    }

    out[1U + id_chars] = CCX_UTILS_HexDigit(msg->DLC);

    for (uint8_t i = 0U; i < msg->DLC; i++)
    {
        out[data_start + (i * 2U)] = CCX_UTILS_HexDigit((uint8_t)(msg->Data[i] >> 4U));
        out[data_start + (i * 2U) + 1U] = CCX_UTILS_HexDigit((uint8_t)(msg->Data[i] & 0x0FU));
    }

    out[data_start + (msg->DLC * 2U)] = '\r';
    out[data_start + (msg->DLC * 2U) + 1U] = '\0';

    return CCX_UTILS_OK;
}

CCX_UTILS_Status_t CCX_BytesToHex(const uint8_t *bytes, size_t byte_count, char *out, size_t out_size)
{
    if ((bytes == NULL) || (out == NULL))
    {
        return CCX_UTILS_NULL_PTR;
    }

    if (byte_count > ((SIZE_MAX - 1U) / 2U))
    {
        return CCX_UTILS_WRONG_ARG;
    }

    if (out_size < ((byte_count * 2U) + 1U))
    {
        return CCX_UTILS_BUFFER_TOO_SMALL;
    }

    for (size_t i = 0U; i < byte_count; i++)
    {
        out[i * 2U] = CCX_UTILS_HexDigit((uint8_t)(bytes[i] >> 4U));
        out[(i * 2U) + 1U] = CCX_UTILS_HexDigit((uint8_t)(bytes[i] & 0x0FU));
    }

    out[byte_count * 2U] = '\0';
    return CCX_UTILS_OK;
}

CCX_UTILS_Status_t CCX_HexToBytes(const char *hex, size_t hex_len, uint8_t *out, size_t out_size, size_t *bytes_written)
{
    size_t byte_count;

    if ((hex == NULL) || (out == NULL))
    {
        return CCX_UTILS_NULL_PTR;
    }

    if ((hex_len & 1U) != 0U)
    {
        return CCX_UTILS_WRONG_ARG;
    }

    byte_count = hex_len / 2U;
    if (out_size < byte_count)
    {
        return CCX_UTILS_BUFFER_TOO_SMALL;
    }

    for (size_t i = 0U; i < byte_count; i++)
    {
        int8_t high = CCX_UTILS_HexValue(hex[i * 2U]);
        int8_t low = CCX_UTILS_HexValue(hex[(i * 2U) + 1U]);

        if ((high < 0) || (low < 0))
        {
            return CCX_UTILS_WRONG_ARG;
        }

        out[i] = (uint8_t)(((uint8_t)high << 4U) | (uint8_t)low);
    }

    if (bytes_written != NULL)
    {
        *bytes_written = byte_count;
    }

    return CCX_UTILS_OK;
}

static double CCX_UTILS_RoundDouble(double value)
{
    return (value >= 0.0) ? (value + 0.5) : (value - 0.5);
}

static float CCX_UTILS_RoundFloat(float value)
{
    return (value >= 0.0f) ? (value + 0.5f) : (value - 0.5f);
}

static double CCX_UTILS_EncodeDoubleRaw(double physical, double factor, double offset)
{
    if (factor == 0.0)
    {
        return 0.0;
    }

    return CCX_UTILS_RoundDouble((physical - offset) / factor);
}

static bool CCX_UTILS_IsNanDouble(double value)
{
    return value != value;
}

static float CCX_UTILS_EncodeFloatRaw(float physical, float factor, float offset)
{
    if (factor == 0.0f)
    {
        return 0.0f;
    }

    return CCX_UTILS_RoundFloat((physical - offset) / factor);
}

static bool CCX_UTILS_IsNanFloat(float value)
{
    return value != value;
}

uint8_t CCX_EncodeLinearU8_Clamped(double physical, double factor, double offset)
{
    double raw = CCX_UTILS_EncodeDoubleRaw(physical, factor, offset);

    if (CCX_UTILS_IsNanDouble(raw))
    {
        return 0U;
    }
    if (raw <= 0.0)
    {
        return 0U;
    }
    if (raw >= (double)UINT8_MAX)
    {
        return UINT8_MAX;
    }
    return (uint8_t)raw;
}

uint16_t CCX_EncodeLinearU16_Clamped(double physical, double factor, double offset)
{
    double raw = CCX_UTILS_EncodeDoubleRaw(physical, factor, offset);

    if (CCX_UTILS_IsNanDouble(raw))
    {
        return 0U;
    }
    if (raw <= 0.0)
    {
        return 0U;
    }
    if (raw >= (double)UINT16_MAX)
    {
        return UINT16_MAX;
    }
    return (uint16_t)raw;
}

uint32_t CCX_EncodeLinearU32_Clamped(double physical, double factor, double offset)
{
    double raw = CCX_UTILS_EncodeDoubleRaw(physical, factor, offset);

    if (CCX_UTILS_IsNanDouble(raw))
    {
        return 0UL;
    }
    if (raw <= 0.0)
    {
        return 0UL;
    }
    if (raw >= (double)UINT32_MAX)
    {
        return UINT32_MAX;
    }
    return (uint32_t)raw;
}

uint64_t CCX_EncodeLinearU64_Clamped(double physical, double factor, double offset)
{
    double raw = CCX_UTILS_EncodeDoubleRaw(physical, factor, offset);

    if (CCX_UTILS_IsNanDouble(raw))
    {
        return 0ULL;
    }
    if (raw <= 0.0)
    {
        return 0ULL;
    }
    if (raw >= CCX_UTILS_DOUBLE_U64_MAX_ROUND_DOWN)
    {
        return UINT64_MAX;
    }
    return (uint64_t)raw;
}

int8_t CCX_EncodeLinearS8_Clamped(double physical, double factor, double offset)
{
    double raw = CCX_UTILS_EncodeDoubleRaw(physical, factor, offset);

    if (CCX_UTILS_IsNanDouble(raw))
    {
        return INT8_MIN;
    }
    if (raw <= (double)INT8_MIN)
    {
        return INT8_MIN;
    }
    if (raw >= (double)INT8_MAX)
    {
        return INT8_MAX;
    }
    return (int8_t)raw;
}

int16_t CCX_EncodeLinearS16_Clamped(double physical, double factor, double offset)
{
    double raw = CCX_UTILS_EncodeDoubleRaw(physical, factor, offset);

    if (CCX_UTILS_IsNanDouble(raw))
    {
        return INT16_MIN;
    }
    if (raw <= (double)INT16_MIN)
    {
        return INT16_MIN;
    }
    if (raw >= (double)INT16_MAX)
    {
        return INT16_MAX;
    }
    return (int16_t)raw;
}

int32_t CCX_EncodeLinearS32_Clamped(double physical, double factor, double offset)
{
    double raw = CCX_UTILS_EncodeDoubleRaw(physical, factor, offset);

    if (CCX_UTILS_IsNanDouble(raw))
    {
        return INT32_MIN;
    }
    if (raw <= (double)INT32_MIN)
    {
        return INT32_MIN;
    }
    if (raw >= (double)INT32_MAX)
    {
        return INT32_MAX;
    }
    return (int32_t)raw;
}

int64_t CCX_EncodeLinearS64_Clamped(double physical, double factor, double offset)
{
    double raw = CCX_UTILS_EncodeDoubleRaw(physical, factor, offset);

    if (CCX_UTILS_IsNanDouble(raw))
    {
        return INT64_MIN;
    }
    if (raw <= CCX_UTILS_DOUBLE_S64_MIN_EXACT)
    {
        return INT64_MIN;
    }
    if (raw >= CCX_UTILS_DOUBLE_S64_MAX_ROUND_DOWN)
    {
        return INT64_MAX;
    }
    return (int64_t)raw;
}

double CCX_DecodeLinearU8(uint8_t raw, double factor, double offset)
{
    return ((double)raw * factor) + offset;
}

double CCX_DecodeLinearU16(uint16_t raw, double factor, double offset)
{
    return ((double)raw * factor) + offset;
}

double CCX_DecodeLinearU32(uint32_t raw, double factor, double offset)
{
    return ((double)raw * factor) + offset;
}

double CCX_DecodeLinearU64(uint64_t raw, double factor, double offset)
{
    return ((double)raw * factor) + offset;
}

double CCX_DecodeLinearS8(int8_t raw, double factor, double offset)
{
    return ((double)raw * factor) + offset;
}

double CCX_DecodeLinearS16(int16_t raw, double factor, double offset)
{
    return ((double)raw * factor) + offset;
}

double CCX_DecodeLinearS32(int32_t raw, double factor, double offset)
{
    return ((double)raw * factor) + offset;
}

double CCX_DecodeLinearS64(int64_t raw, double factor, double offset)
{
    return ((double)raw * factor) + offset;
}

uint8_t CCX_EncodeLinearU8F_Clamped(float physical, float factor, float offset)
{
    float raw = CCX_UTILS_EncodeFloatRaw(physical, factor, offset);

    if (CCX_UTILS_IsNanFloat(raw))
    {
        return 0U;
    }
    if (raw <= 0.0f)
    {
        return 0U;
    }
    if (raw >= (float)UINT8_MAX)
    {
        return UINT8_MAX;
    }
    return (uint8_t)raw;
}

uint16_t CCX_EncodeLinearU16F_Clamped(float physical, float factor, float offset)
{
    float raw = CCX_UTILS_EncodeFloatRaw(physical, factor, offset);

    if (CCX_UTILS_IsNanFloat(raw))
    {
        return 0U;
    }
    if (raw <= 0.0f)
    {
        return 0U;
    }
    if (raw >= (float)UINT16_MAX)
    {
        return UINT16_MAX;
    }
    return (uint16_t)raw;
}

uint32_t CCX_EncodeLinearU32F_Clamped(float physical, float factor, float offset)
{
    float raw = CCX_UTILS_EncodeFloatRaw(physical, factor, offset);

    if (CCX_UTILS_IsNanFloat(raw))
    {
        return 0UL;
    }
    if (raw <= 0.0f)
    {
        return 0UL;
    }
    if (raw >= (float)UINT32_MAX)
    {
        return UINT32_MAX;
    }
    return (uint32_t)raw;
}

uint64_t CCX_EncodeLinearU64F_Clamped(float physical, float factor, float offset)
{
    float raw = CCX_UTILS_EncodeFloatRaw(physical, factor, offset);

    if (CCX_UTILS_IsNanFloat(raw))
    {
        return 0ULL;
    }
    if (raw <= 0.0f)
    {
        return 0ULL;
    }
    if (raw >= CCX_UTILS_FLOAT_U64_MAX_ROUND_DOWN)
    {
        return UINT64_MAX;
    }
    return (uint64_t)raw;
}

int8_t CCX_EncodeLinearS8F_Clamped(float physical, float factor, float offset)
{
    float raw = CCX_UTILS_EncodeFloatRaw(physical, factor, offset);

    if (CCX_UTILS_IsNanFloat(raw))
    {
        return INT8_MIN;
    }
    if (raw <= (float)INT8_MIN)
    {
        return INT8_MIN;
    }
    if (raw >= (float)INT8_MAX)
    {
        return INT8_MAX;
    }
    return (int8_t)raw;
}

int16_t CCX_EncodeLinearS16F_Clamped(float physical, float factor, float offset)
{
    float raw = CCX_UTILS_EncodeFloatRaw(physical, factor, offset);

    if (CCX_UTILS_IsNanFloat(raw))
    {
        return INT16_MIN;
    }
    if (raw <= (float)INT16_MIN)
    {
        return INT16_MIN;
    }
    if (raw >= (float)INT16_MAX)
    {
        return INT16_MAX;
    }
    return (int16_t)raw;
}

int32_t CCX_EncodeLinearS32F_Clamped(float physical, float factor, float offset)
{
    float raw = CCX_UTILS_EncodeFloatRaw(physical, factor, offset);

    if (CCX_UTILS_IsNanFloat(raw))
    {
        return INT32_MIN;
    }
    if (raw <= (float)INT32_MIN)
    {
        return INT32_MIN;
    }
    if (raw >= (float)INT32_MAX)
    {
        return INT32_MAX;
    }
    return (int32_t)raw;
}

int64_t CCX_EncodeLinearS64F_Clamped(float physical, float factor, float offset)
{
    float raw = CCX_UTILS_EncodeFloatRaw(physical, factor, offset);

    if (CCX_UTILS_IsNanFloat(raw))
    {
        return INT64_MIN;
    }
    if (raw <= CCX_UTILS_FLOAT_S64_MIN_EXACT)
    {
        return INT64_MIN;
    }
    if (raw >= CCX_UTILS_FLOAT_S64_MAX_ROUND_DOWN)
    {
        return INT64_MAX;
    }
    return (int64_t)raw;
}

float CCX_DecodeLinearU8F(uint8_t raw, float factor, float offset)
{
    return ((float)raw * factor) + offset;
}

float CCX_DecodeLinearU16F(uint16_t raw, float factor, float offset)
{
    return ((float)raw * factor) + offset;
}

float CCX_DecodeLinearU32F(uint32_t raw, float factor, float offset)
{
    return ((float)raw * factor) + offset;
}

float CCX_DecodeLinearU64F(uint64_t raw, float factor, float offset)
{
    return ((float)raw * factor) + offset;
}

float CCX_DecodeLinearS8F(int8_t raw, float factor, float offset)
{
    return ((float)raw * factor) + offset;
}

float CCX_DecodeLinearS16F(int16_t raw, float factor, float offset)
{
    return ((float)raw * factor) + offset;
}

float CCX_DecodeLinearS32F(int32_t raw, float factor, float offset)
{
    return ((float)raw * factor) + offset;
}

float CCX_DecodeLinearS64F(int64_t raw, float factor, float offset)
{
    return ((float)raw * factor) + offset;
}
