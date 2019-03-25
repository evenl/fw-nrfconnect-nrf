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
#ifndef ZEPHYR_INCLUDE_AT_CMD_INTERFACE_H_
#define ZEPHYR_INCLUDE_AT_CMD_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>

/**
 * @brief AT command return codes
 */
enum at_cmd_rc {
	AT_CMD_OK,
	AT_CMD_ERROR,
	AT_CMD_CMS,
	AT_CMD_CME,
	AT_CMD_NO_RETURN_CODE,
};

/**
 * @typedefs at_cmd_interface_handler_t
 * @brief Callback API when a message is received from the mode
 *
 * @param "char * response" Null terminated string containing the modem message
 * @param "size_t response_len" Length of string
 *
 */
typedef void (*at_cmd_handler_t)(char *response,
				 size_t response_len);

/**
 * @brief Function to send an AT command to the modem
 *
 * @param cmd Pointer to null terminated AT command string
 */
enum at_cmd_rc at_cmd_write(const char *const cmd);

/**
 * @brief Function to send an AT command and receive response immediately
 *
 * For AT commands that will return a response immediately before the
 * the command return code can be called by this function to get the response
 * string in the supplied buffer if it's large enough. If the buffer is not
 * large enough the driver will fall back on the handler.
 *
 * @param cmd Pointer to null terminated AT command string
 * @param buf Buffer to put the response in
 * @param buf_len Length of response buffer
 *
 */
enum at_cmd_rc at_cmd_write_with_response(const char *const cmd,
					  char *buf,
					  u32_t buf_len);

/**
 * @brief Function to set AT command message handler
 *
 * @param handler Pointer to a received messages handler function
 */
void at_cmd_set_handler(at_cmd_handler_t handler);

/**
 * @brief Function to get the last received CMS or CME error code
 *
 */
int at_cmd_get_last_error(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_AT_CMD_INTERFACE_H_ */
