/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <string.h>

#include <at_cmd_parser/at_cmd_parser.h>
#include <at_cmd_parser/at_cmd_decoder.h>

/* Forward declarations. */
static void *at_CEREG_decode(struct at_param_list *param_list,
			     u32_t valid_params,
			     void *user_data);

static void *at_CESQ_decode(struct at_param_list *param_list,
			    u32_t valid_params,
			    void *user_data);

static void *at_NOT_CESQ_decode(struct at_param_list *param_list,
				u32_t valid_params,
				void *user_data);

static void *at_CMT_decode(struct at_param_list *param_list,
			   u32_t valid_params,
			   void *user_data);

static void *at_CNUM_decode(struct at_param_list *param_list,
			    u32_t valid_params,
			    void *user_data);

AT_CMD_DECODER_LIST_START(default_decoder_list)
AT_CMD_DECODER_LIST_ADD("+CESQ", AT_CMD_CESQ_MODEL, at_CESQ_decode)
AT_CMD_DECODER_LIST_ADD("%CESQ", AT_NOT_CESQ_MODEL, at_NOT_CESQ_decode)
AT_CMD_DECODER_LIST_ADD("+CEREG", AT_CMD_CEREG_MODEL, at_CEREG_decode)
AT_CMD_DECODER_LIST_ADD("+CMT", AT_CMD_CMT_MODEL, at_CMT_decode)
AT_CMD_DECODER_LIST_ADD("+CNUM", AT_CMD_CNUM_MODEL, at_CNUM_decode)
AT_CMD_DECODER_LIST_END();

static void *at_CMT_decode(struct at_param_list *param_list,
			   u32_t valid_params,
			   void *user_data)
{
	int err;

	if (valid_params != 4) {
		return NULL;
	}

	struct at_cmd_model_cmt *model =
		k_calloc(1, sizeof(struct at_cmd_model_cmt));

	if (!model) {
		return NULL;
	}

	err = at_params_string_get(param_list, 1, model->sender_addr, 32);
	err |= at_params_short_get(param_list, 2, &model->pdu_length);
	err |= at_params_string_get(param_list, 3, model->pdu_data, 160);

	if (err < 0) {
		k_free(model);
		return NULL;
	} else {
		return model;
	}
}

static void *at_CESQ_decode(struct at_param_list *param_list,
			    u32_t valid_params,
			    void *user_data)
{
	int err;

	if (valid_params != 7) {
		return NULL;
	}

	struct at_cmd_model_cesq *model =
		k_malloc(sizeof(struct at_cmd_model_cesq));

	if (!model) {
		return NULL;
	}

	err = at_params_short_get(param_list, 1, &model->rxlev);
	err |= at_params_short_get(param_list, 2, &model->ber);
	err |= at_params_short_get(param_list, 3, &model->rscp);
	err |= at_params_short_get(param_list, 4, &model->ecno);
	err |= at_params_short_get(param_list, 5, &model->rsrq);
	err |= at_params_short_get(param_list, 6, &model->rsrp);

	if (err) {
		k_free(model);
		return NULL;
	} else {
		return model;
	}
}

static void *at_NOT_CESQ_decode(struct at_param_list *param_list,
				u32_t valid_params,
				void *user_data)
{
	int err;

	if (valid_params != 3) {
		return NULL;
	}

	struct at_not_model_cesq *model =
		k_malloc(sizeof(struct at_not_model_cesq));

	if (!model) {
		return NULL;
	}

	err = at_params_short_get(param_list, 1, &model->rsrp);
	err |= at_params_short_get(param_list, 2, &model->threshold_index);

	if (err) {
		k_free(model);
		return NULL;
	} else {
		return model;
	}
}

static void *at_CNUM_decode(struct at_param_list *param_list,
			    u32_t valid_params,
			    void *user_data)
{
	int err;

	if (valid_params != 3) {
		return NULL;
	}

	struct at_cmd_model_cnum *model =
		k_calloc(1, sizeof(struct at_cmd_model_cnum));

	if (!model) {
		return NULL;
	}

	err = at_params_string_get(param_list, 1, model->numberx, 32);
	err |= at_params_short_get(param_list, 2, &model->typex);

	if (err < 0) {
		k_free(model);
		return NULL;
	} else {
		return model;
	}
}

static void *at_CEREG_decode(struct at_param_list *param_list,
			     u32_t valid_params,
			     void *user_data)
{
	int err;
	int ret;

	if (valid_params < 2) {
		return NULL;
	}

	struct at_cmd_model_cereg *model =
		k_calloc(1, sizeof(struct at_cmd_model_cereg));

	if (!model) {
		return NULL;
	}

	err = at_params_short_get(param_list, 1, &model->stat);

	if (valid_params > 2) {
		ret = at_params_string_get(param_list, 2, model->tac, 5);
		if (ret < 0) {
			err |= ret;
		}

		ret = at_params_string_get(param_list, 3, model->ci, 9);
		if (ret < 0) {
			err |= ret;
		}

		err |= at_params_short_get(param_list, 4, &model->act);
	}

	if (err) {
		k_free(model);
		return NULL;
	} else {
		return model;
	}
}

static int get_at_cmd_decode_handler_index(struct at_cmd_decoder *decoder,
					   const char *const p_atstring)
{
	for (size_t i = 0; i < decoder->list_len; ++i) {
		size_t len = strlen(decoder->list[i].identifier);

		if (memcmp(decoder->list[i].identifier,
			   p_atstring, len) == 0) {
			return i;
		}
	}

	return -EIO;
}

int at_cmd_decode(struct at_cmd_decoder *decoder,
		  const char *at_message)
{
	u32_t valid_params;
	void *model_ptr;
	char id[16];
	int err;
	int parse_err;

	do {
		parse_err = at_parser_params_from_str(&at_message,
						      &decoder->param_list);

		if ((parse_err != 0) &&
		    (parse_err != EAGAIN)) {
			return parse_err;
		}
		valid_params = at_params_valid_count_get(&decoder->param_list);

		if (valid_params == 0) {
			return -EIO;
		}

		err = at_params_string_get(&decoder->param_list, 0, id, 16);
		int model_index = get_at_cmd_decode_handler_index(decoder, id);

		if (model_index < 0) {
			return -EIO;
		}

		model_ptr = decoder->list[model_index].at_cmd_decoder(
					&decoder->param_list,
					valid_params,
					decoder->list[model_index].user_data);

		if (decoder->handler) {
			decoder->handler(model_ptr,
					 decoder->list[model_index].user_data);
		}

		if (model_ptr) {
			k_free(model_ptr);
		}
	} while(parse_err == EAGAIN);

	return 0;
}

int at_cmd_decoder_init(struct at_cmd_decoder            *decoder,
			struct at_cmd_decoder_list const *decoder_list,
			u32_t                             list_length)
{
	at_params_list_init(&decoder->param_list, 10);

	if (decoder_list != NULL) {
		decoder->list     = decoder_list;
		decoder->list_len = list_length;
		
	} else {
		decoder->list     = default_decoder_list;
		decoder->list_len = sizeof(default_decoder_list);
	}

	return 0;
}

int at_cmd_decoder_deinit(struct at_cmd_decoder *decoder)
{
	at_params_list_free(&decoder->param_list);

	return 0;
}

void at_cmd_decoder_set_handler(struct at_cmd_decoder    *decoder,
				at_cmd_decoder_handler_t handler)
{
	decoder->handler = handler;
}
