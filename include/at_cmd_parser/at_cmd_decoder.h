/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

/**
 * @file at_cmd_decoder.h
 *
 * @defgroup at_cmd_parser_decoder AT commands decoder
 * @ingroup at_cmd_parser
 * @{
 * @brief Public APIs for the AT command decoder driver.
 */
#ifndef AT_CMD_DECODER_H__
#define AT_CMD_DECODER_H__

#include <zephyr/types.h>

#include <at_cmd_parser/at_cmd_data_models.h>
#include <at_cmd_parser/at_params.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AT_CMD_DECODER_LIST_START(name)                                        \
	const struct at_cmd_decoder_list name[] = {

#define AT_CMD_DECODER_LIST_END()                                              \
	{                                                                      \
		.cmd_str = "", .model = AT_CMD_NO_MODEL,                       \
		.at_cmd_decoder = NULL,                                        \
	}                                                                      \
	}

#define AT_CMD_DECODER_LIST_ENTRY(name, id, cb_fn) { name, id, cb_fn },

/**
 * @typedefs at_cmd_decoder_handler_t
 * @brief Callback API when a message is parsed.
 *
 * @param[in] model     Data model is used to store the the message payload.
 * @param[in] model_ptr Pointer to the data model.
 */
typedef void (*at_cmd_decoder_handler_t)(enum at_cmd_models model,
					 void *model_ptr);

struct at_cmd_decoder_list {
	/**
	 * @brief Name of the AT command received as a response.
	 */
	const char *cmd_str;

	enum at_cmd_models model;

	/**
	 * @brief Function used to decode the AT command response parameters.
	 *
	 * @param[in] p_at_params   Parameters of the AT command as a String.
	 *                          Cannot be null.
	 * @param[out] p_out        Pointer to the output structure to populate.
	 *                          Can be null if not used.
	 */
	void *(*at_cmd_decoder)(struct at_param_list *param_list,
				u32_t valid_params);
};

/**
 * @brief Function to initialize the AT command decoder.
 */
int at_cmd_decoder_init(struct at_cmd_decoder_list const *decoder_list);

/**
 * @brief Function free up resources used by the decoder.
 */
int at_cmd_decoder_deinit(void);

/**
 * @brief Function to set handler to receive the processed result from decoder.
 */
int at_cmd_decoder_set_handler(at_cmd_decoder_handler_t handler);

/**
 * @brief Function parse a message from the modem.
 *
 * @param at_message A string containing an AT command message.
 */
int at_cmd_decode(const char *at_message);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* AT_CMD_DECODER_H__ */
