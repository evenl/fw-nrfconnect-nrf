/**
 * @file at_decoder.h
 *
 * @brief Public APIs for the AT command interface driver.
 */

/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */
#ifndef ZEPHYR_INCLUDE_AT_DECODER_H_
#define ZEPHYR_INCLUDE_AT_DECODER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>
#include <at_cmd_data_models.h>

/**
 * @typedefs at_cmd_decoder_handler_t
 * @brief Callback API when a message is parsed
 *
 * @param "enum at_cmd_models model" Which data model is used to store the
 *				     the message payload
 * @param "void * model_ptr" Pointer to data model
 */

typedef void(*at_cmd_decoder_handler_t)(enum at_cmd_models model,
					void *model_ptr);

/**
 * @brief Function to initalize the AT command decoder
 */
int at_cmd_decoder_init(void);

/**
 * @brief Function free up resources used by the decoder
 */
int at_cmd_decoder_deinit(void);

/**
 * @brief Function to set handler to receive the processed result from decoder
 */
int at_cmd_decoder_set_handler(at_cmd_decoder_handler_t handler);

/**
 * @brief Function parse a message from the modem.
 *
 * @param at_message A string containing an AT command message
 */
int at_cmd_decode(char *at_message);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_AT_DECODER_H_ */

