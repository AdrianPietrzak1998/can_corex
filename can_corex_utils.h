/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Author: Adrian Pietrzak
 * GitHub: https://github.com/AdrianPietrzak1998
 * Created: May 18, 2026
 */

#ifndef CAN_COREX_CAN_COREX_UTILS_H_
#define CAN_COREX_CAN_COREX_UTILS_H_

#include "can_corex.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @file can_corex_utils.h
     * @brief Optional utility helpers for CAN CoreX applications.
     *
     * @defgroup ccx_utils Utilities
     * @brief Allocation-free helpers for SLCAN text conversion, hex conversion,
     * and linear signal scaling.
     * @{
     */

    /**
     * @brief Status values returned by CAN CoreX utility helpers.
     */
    typedef enum
    {
        CCX_UTILS_OK = 0,           /**< Operation completed successfully. */
        CCX_UTILS_NULL_PTR,         /**< A required pointer argument was NULL. */
        CCX_UTILS_WRONG_ARG,        /**< Input format or numeric argument was invalid. */
        CCX_UTILS_BUFFER_TOO_SMALL, /**< Caller-provided output buffer is too small. */
        CCX_UTILS_UNSUPPORTED       /**< The input uses a feature intentionally not supported by this helper. */
    } CCX_UTILS_Status_t;

/**
 * @brief Maximum NUL-terminated classic SLCAN data-frame line length.
 *
 * The longest supported line is an extended 29-bit data frame with 8 data bytes:
 * `T` + 8 hex ID chars + 1 DLC char + 16 hex data chars + `\r` + `\0`.
 */
#define CCX_SLCAN_CLASSIC_MAX_LINE_LEN 28U

    /**
     * @brief Parse a classic SLCAN data-frame line into a CAN CoreX message.
     *
     * Supported input forms:
     * - `tIIILDD...` for standard 11-bit data frames
     * - `TIIIIIIIILDD...` for extended 29-bit data frames
     *
     * The parser accepts a trailing `\r`, `\n`, `\r\n`, or the end of the C string
     * exactly after the last expected frame character. Remote frames (`r`/`R`) are
     * not represented by CCX_message_t and are returned as CCX_UTILS_UNSUPPORTED.
     *
     * @param line NUL-terminated SLCAN line.
     * @param msg Destination message.
     * @return CCX_UTILS_OK on success, otherwise an error status.
     */
    CCX_UTILS_Status_t CCX_SLCAN_Parse(const char *line, CCX_message_t *msg);

    /**
     * @brief Format a CAN CoreX classic CAN message as an SLCAN data-frame line.
     *
     * The output is always terminated with `\r` and `\0`. Only classic CAN data
     * frames are supported.
     *
     * @param msg Source message.
     * @param out Caller-owned output buffer.
     * @param out_size Size of @p out in chars.
     * @return CCX_UTILS_OK on success, otherwise an error status.
     */
    CCX_UTILS_Status_t CCX_SLCAN_Format(const CCX_message_t *msg, char *out, size_t out_size);

    /**
     * @brief Convert bytes to an uppercase hex string.
     *
     * No separators are inserted. The output is NUL-terminated.
     *
     * @param bytes Source bytes.
     * @param byte_count Number of bytes to convert.
     * @param out Caller-owned output buffer.
     * @param out_size Size of @p out in chars.
     * @return CCX_UTILS_OK on success, otherwise an error status.
     */
    CCX_UTILS_Status_t CCX_BytesToHex(const uint8_t *bytes, size_t byte_count, char *out, size_t out_size);

    /**
     * @brief Convert a hex string to bytes.
     *
     * The input length must be even. Uppercase and lowercase hex digits are accepted.
     *
     * @param hex Source hex characters.
     * @param hex_len Number of hex characters to parse.
     * @param out Caller-owned output byte buffer.
     * @param out_size Capacity of @p out in bytes.
     * @param bytes_written Optional destination for the number of bytes written.
     * @return CCX_UTILS_OK on success, otherwise an error status.
     */
    CCX_UTILS_Status_t CCX_HexToBytes(const char *hex, size_t hex_len, uint8_t *out, size_t out_size,
                                      size_t *bytes_written);

    /**
     * @name Double-based linear signal helpers
     *
     * These helpers implement the common CAN signal scaling model:
     * - decode: `physical = raw * factor + offset`
     * - encode: `raw = round((physical - offset) / factor)`
     *
     * Encode/decode preserves the physical value exactly when the input physical
     * value lies on the representable signal grid and the floating-point type can
     * represent the intermediate values exactly enough. Off-grid physical values
     * are quantized to the nearest raw integer first; decoding returns that nearest
     * representable physical value, not necessarily the original input.
     *
     * The 64-bit helpers are useful for large counters and scaled values, but
     * `double` cannot represent every 64-bit integer exactly. Exact round-trips are
     * guaranteed only within the precision limits of the floating-point type and
     * the selected factor/offset.
     *
     * If factor is zero, encode returns the minimum value for the target type.
     * @{
     */
    uint8_t CCX_EncodeLinearU8_Clamped(double physical, double factor, double offset);
    uint16_t CCX_EncodeLinearU16_Clamped(double physical, double factor, double offset);
    uint32_t CCX_EncodeLinearU32_Clamped(double physical, double factor, double offset);
    uint64_t CCX_EncodeLinearU64_Clamped(double physical, double factor, double offset);

    int8_t CCX_EncodeLinearS8_Clamped(double physical, double factor, double offset);
    int16_t CCX_EncodeLinearS16_Clamped(double physical, double factor, double offset);
    int32_t CCX_EncodeLinearS32_Clamped(double physical, double factor, double offset);
    int64_t CCX_EncodeLinearS64_Clamped(double physical, double factor, double offset);

    double CCX_DecodeLinearU8(uint8_t raw, double factor, double offset);
    double CCX_DecodeLinearU16(uint16_t raw, double factor, double offset);
    double CCX_DecodeLinearU32(uint32_t raw, double factor, double offset);
    double CCX_DecodeLinearU64(uint64_t raw, double factor, double offset);

    double CCX_DecodeLinearS8(int8_t raw, double factor, double offset);
    double CCX_DecodeLinearS16(int16_t raw, double factor, double offset);
    double CCX_DecodeLinearS32(int32_t raw, double factor, double offset);
    double CCX_DecodeLinearS64(int64_t raw, double factor, double offset);
    /** @} */

    /**
     * @name Float-based linear signal helpers
     *
     * Same behavior as the double-based family, but all physical values, factors,
     * offsets, and decoded values use `float`. This is useful on MCUs where `float`
     * is the preferred application type. Precision is lower than the double family:
     * not all 32-bit or 64-bit raw values can be represented exactly as `float`.
     *
     * If factor is zero, encode returns the minimum value for the target type.
     * @{
     */
    uint8_t CCX_EncodeLinearU8F_Clamped(float physical, float factor, float offset);
    uint16_t CCX_EncodeLinearU16F_Clamped(float physical, float factor, float offset);
    uint32_t CCX_EncodeLinearU32F_Clamped(float physical, float factor, float offset);
    uint64_t CCX_EncodeLinearU64F_Clamped(float physical, float factor, float offset);

    int8_t CCX_EncodeLinearS8F_Clamped(float physical, float factor, float offset);
    int16_t CCX_EncodeLinearS16F_Clamped(float physical, float factor, float offset);
    int32_t CCX_EncodeLinearS32F_Clamped(float physical, float factor, float offset);
    int64_t CCX_EncodeLinearS64F_Clamped(float physical, float factor, float offset);

    float CCX_DecodeLinearU8F(uint8_t raw, float factor, float offset);
    float CCX_DecodeLinearU16F(uint16_t raw, float factor, float offset);
    float CCX_DecodeLinearU32F(uint32_t raw, float factor, float offset);
    float CCX_DecodeLinearU64F(uint64_t raw, float factor, float offset);

    float CCX_DecodeLinearS8F(int8_t raw, float factor, float offset);
    float CCX_DecodeLinearS16F(int16_t raw, float factor, float offset);
    float CCX_DecodeLinearS32F(int32_t raw, float factor, float offset);
    float CCX_DecodeLinearS64F(int64_t raw, float factor, float offset);
    /** @} */

    /** @} */

#ifdef __cplusplus
}
#endif

#endif /* CAN_COREX_CAN_COREX_UTILS_H_ */
