/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

/**
 * @file at_utils.h
 *
 * @defgroup at_cmd_parser_utils AT Command utility functions.
 * @ingroup at_cmd_parser
 * @{
 * @brief AT parser utility functions to deal with strings.
 */
#ifndef AT_UTILS_H__
#define AT_UTILS_H__

#include <zephyr/types.h>
#include <stddef.h>

/**
 * @brief Remove spaces from beginning of returned string.
 *
 * Skips spaces from the beginning of the string and moves pointer to point
 * first non-space character. Caller should maintain pointer to block start for
 * deallocation purposes.
 *
 * @param[in,out] str Address of string pointer as input. Pointer changed to
 *                    point to first non-space character as output.
 *
 * @return Number of removed spaces from the beginning.
 */
u32_t at_params_get_space_count(const char **str);

/**
 * @brief Get the length of an AT command or event.
 *
 * An AT command or event can be terminated by '\0', '?' or ';'.
 *
 * @param[in] str Pointer to AT command or event string.
 *
 * @return Length of first AT command or event.
 */
size_t at_params_get_cmd_length(const char *const str);

/** @} */

#endif /* AT_UTILS_H__ */
