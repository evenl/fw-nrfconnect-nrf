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
 * @brief Public API for the AT command decoder driver.
 *
 *        This driver is used to parameterize an AT string using the AT parser
 *        library and then process the parameter arrays and categorize them
 *        based on the content of the first parameter in the array. A user
 *        specific function (decoder) is then called based on the first
 *        parameter in the string which is used to identify the data in the
 *        rest of the string.
 *
 *        Two concepts for passing data between producer (decoder) and
 *        consumer (application) is supported. Either the decoder can return
 *        a memory object in dynamic memory containing the produced data which
 *        will passed to the handler through the model_ptr parameter
 *        @ref at_cmd_decoder_handler_t. This object will automatically be freed
 *        after the handler has returned, which means that this object can not
 *        be in static memory.
 *
 *        The second concept let the user reference an object in memory
 *        (static or dynamic) using the user_data pointer member in the
 *        @ref at_cmd_decoder_list struct. This reference will be passed to the
 *        handler through the user_data pointer parameter.
 *
 *        A default list of decoders is used to decode the AT payload if the
 *        @ref at_cmd_decoder_init function is called with a NULL pointer
 *        parameter. */
#ifndef AT_CMD_DECODER_H__
#define AT_CMD_DECODER_H__

#include <zephyr/types.h>

#include <at_cmd_parser/at_cmd_data_models.h>
#include <at_cmd_parser/at_params.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AT_CMD_DECODER_LIST_START(name)                \
	const struct at_cmd_decoder_list name[] = {

#define AT_CMD_DECODER_LIST_END()                      \
	}

#define AT_CMD_DECODER_LIST_ADD(name, user, cb_fn)   \
	{ name, (void*)user, cb_fn },

/**
 * @typedefs at_cmd_decoder_handler_t
 * @brief Callback function that will be called when the decoder is done
 *        processing the AT string.
 *
 * @param[in] model_ptr Points to the data stored by the decoder
 * @param[in] user_data Points to user data that is the same as the user_data
 *                      field in the @ref at_cmd_decoder_list struct. 
 */
typedef void (*at_cmd_decoder_handler_t)(void *model_ptr,
					 void *user_data);

/**
 * @brief Struct that define one element in the list of decoders.
 *
 * This struct is used in an array to define a decoder element in the list
 * of decoders. The AT_CMD_DECODER_LIST_START, AT_CMD_DECODER_LIST_ADD, and
 * AT_CMD_DECODER_LIST_END can be used to define such array. This array is
 * then used with the @ref at_cmd_decoder_init function.
 *
 */
struct at_cmd_decoder_list {
	/**
	 * @brief Name of the data identifier including the +/% character
	 * Eg. +CEREG
	 */
	const char *identifier;

	/**
	 * @bref Pointer that can point to application specific data. Should be
	 * NULL if not used.
	 */
	void       *user_data;

	/**
	 * @brief Function used to decode the AT command response parameters.
	 *
	 * @param[in] param_list    List of parameters
	 * @param[in] valid_params  How many valid parameters in the list
	 * @param[in] user_data     Reference to user_data defined in the
	 *                          decoder list
	 *
	 * @param[out] p_out        Object in dynamic memory containing
	 *                          processed data. Can be NULL if not used.
	 */
	void *(*at_cmd_decoder)(struct at_param_list *param_list,
				u32_t valid_params,
				void  *user_data);
};

/**
 * @brief Struct to store the context of one decoder instance.
 *
 * This struct is used as the store the decoder instance specific data
 *
 */
struct at_cmd_decoder {
	/**
	 * @brief Length of decoder list
	 */
	u32_t                            list_len;

	/**
	 * @brief List of decoders
	 */
	struct at_cmd_decoder_list const *list;

	/**
	 * @brief List of parameterized AT string
	 */
	struct at_param_list             param_list;

	/**
	 * @brief Callback function called when the AT string is completely
	 *        processed by the decoder
	 */
	at_cmd_decoder_handler_t         handler;
};

/**
 * @brief Function to initialize the AT command decoder.
 *
 * @param decoder      Pointer to @ref at_cmd_decoder struct to hold the
 *                     decoder instance context.
 * @param decoder_list Pointer to a list of AT response decoders, can be NULL
 *                     if the default decoder list should be used
 * @param list_length  Length of list (ie. Number of entries). This number is
 *                     not used if decoder_list is NULL;
 *
 */
int at_cmd_decoder_init(struct at_cmd_decoder            *decoder,
			struct at_cmd_decoder_list const *decoder_list,
			u32_t                             list_length);

/**
 * @brief Function free up resources used by the decoder.
 *
 * @param decoder      Pointer to @ref at_cmd_decoder struct to hold the
 *                     decoder instance context.
 */
int at_cmd_decoder_deinit(struct at_cmd_decoder *decoder);

/**
 * @brief Function to set handler to receive the processed result from decoder.
 *
 * @param decoder      Pointer to @ref at_cmd_decoder struct to hold the
 *                     decoder instance context.
 * @param handler Pointer to a handler function of type
 *                @ref at_cmd_decoder_handler_t
 */
void at_cmd_decoder_set_handler(struct at_cmd_decoder    *decoder,
				at_cmd_decoder_handler_t handler);

/**
 * @brief Function to decode an AT string
 *
 * This function is used to process AT string and detect which notification type
 * it is. If a function is registered in the decoder list for the notification
 * type detected, this function will be called with a reference to the
 * processed payload. In some cases do an AT string contain multiple
 * notifications, the decoder will automatically split the string into
 * individual notifications, and process them separately. Then handler, if
 * registered, will be called for each notification detected.
 *
 * @param decoder      Pointer to @ref at_cmd_decoder struct to hold the
 *                     decoder instance context.
 * @param at_message A string containing an AT command message.
 *
 * @retval EIO   Generic error when trying to parse AT string
 * @retval E2BIG More than 10 parameters detected in the AT string
 *
 */
int at_cmd_decode(struct at_cmd_decoder *decoder,
		  const char *at_message);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* AT_CMD_DECODER_H__ */
