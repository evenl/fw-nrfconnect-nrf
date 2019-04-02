/**
 * @file at_cmd_interface.h
 *
 * @defgroup at_cmd_interface AT Command interface driver
 *
 * @{
 *
 * @brief Public APIs for the AT command interface driver.
 */

#ifndef ZEPHYR_INCLUDE_AT_CMD_INTERFACE_H_
#define ZEPHYR_INCLUDE_AT_CMD_INTERFACE_H_
/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

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
 * @typedefs at_cmd_handler_t
 * @brief Callback API when a message is received from the mode
 *
 * @param response Null terminated string containing the modem message
 * @param response_len Length of string
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
 * This function can be used for AT commands that return a response
 * immediately. The response will returned in the supplied buffer.
 * This function will return an empty buffer for function used to enable
 * notifications. The function will log an error if the buffer is not to small.
 *
 * @param cmd Pointer to null terminated AT command string
 * @param buf Buffer to put the response in
 * @param buf_len Length of response buffer
 *
 */
enum at_cmd_rc at_cmd_write_with_response(const char *const cmd,
					  char *buf,
					  size_t buf_len);

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

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_AT_CMD_INTERFACE_H_ */
