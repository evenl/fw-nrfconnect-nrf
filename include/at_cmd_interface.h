/**
 * @file at_cmd_interface.h
 *
 * @brief Public APIs for the AT command interface driver.
 */

/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */
#ifndef ZEPHYR_INCLUDE_AT_INTERFACE_H_
#define ZEPHYR_INCLUDE_AT_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>

/**
 * @brief AT command return codes
 */
enum at_cmd_return_code {
	AT_CMD_OK,
	AT_CMD_ERROR,
	AT_CMD_CMS,
	AT_CMD_CME,
	AT_CMD_NOT_RETURN_CODE,
};

/**
 * @typedefs at_cmd_interface_handler_t
 * @brief Callback API when a message is received from the mode
 *
 * @param "char * response" Null terminated string containing the modem message
 * @param "size_t response_len" Length of string
 *
 */
typedef void (*at_cmd_interface_handler_t)(char *response,
					   size_t response_len);

/**
 * @brief Function to send an AT command to the modem
 *
 * @param cmd Pointer to null terminated AT command string
 */
enum at_cmd_return_code at_cmd_interface_write(const char *const cmd);

/**
 * @brief Function to set AT command message handler
 *
 * @param handler Pointer to a received messages handler function
 */
void                    at_cmd_interface_set_handler(at_cmd_interface_handler_t
							handler);

/**
 * @brief Function to get the last received CMS or CME error code
 *
 */
int                     at_cmd_interface_get_last_error(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_AT_INTERFACE_H_ */
