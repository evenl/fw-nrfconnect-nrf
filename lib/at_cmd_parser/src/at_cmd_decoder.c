#include <zephyr.h>

#include <at_cmd_parser.h>
#include <at_params.h>
#include <at_cmd_decoder.h>

#include <string.h>

struct at_cmd_cb {
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
	void* (*at_cmd_decode_handler)(struct at_param_list *param_list,
				       u32_t valid_params);
};

static at_cmd_decoder_handler_t at_cmd_decoder_handler;

/* Forward declarations. */
static void *at_CEREG_decode(struct at_param_list *param_list,
			     u32_t valid_params);

static void *at_CESQ_decode(struct at_param_list *param_list,
			    u32_t valid_params);

static void *at_NOT_CESQ_decode(struct at_param_list *param_list,
				u32_t valid_params);

static void *at_CMT_decode(struct at_param_list *param_list,
			   u32_t valid_params);

static void *at_CNUM_decode(struct at_param_list *param_list,
			    u32_t valid_params);

static const struct at_cmd_cb m_at_cmds[] = {
	{"+CESQ", AT_CMD_CESQ_MODEL, at_CESQ_decode},
	{"%CESQ", AT_NOT_CESQ_MODEL, at_NOT_CESQ_decode},
	{"+CEREG:", AT_CMD_CEREG_MODEL, at_CEREG_decode},
	{"+CMT:", AT_CMD_CMT_MODEL, at_CMT_decode},
	{"+CNUM", AT_CMD_CNUM_MODEL, at_CNUM_decode},
};

static struct at_param_list param_list;

static void *at_CMT_decode(struct at_param_list *param_list,
			   u32_t valid_params)
{
	int err;

	if (valid_params != 3) {
		return NULL;
	}

	struct at_cmd_model_cmt *model =
			k_malloc(sizeof(struct at_cmd_model_cmt));

	if (!model) {
		return NULL;
	}
	memset(model, 0, sizeof(struct at_cmd_model_cmt));


	err = at_params_get_string(param_list, 0, model->sender_addr, 32);
	err |= at_params_get_short(param_list, 1, &model->pdu_length);
	err |= at_params_get_string(param_list, 2, model->pdu_data, 160);

	return model;
}

static void *at_CESQ_decode(struct at_param_list *param_list,
			    u32_t valid_params)
{
	int err;

	if (valid_params != 6) {
		return NULL;
	}

	struct at_cmd_model_cesq *model =
			k_malloc(sizeof(struct at_cmd_model_cesq));

	if (!model) {
		return NULL;
	}
	memset(model, 0, sizeof(struct at_cmd_model_cesq));

	err = at_params_get_short(param_list, 0, &model->rxlev);
	err |= at_params_get_short(param_list, 1, &model->ber);
	err |= at_params_get_short(param_list, 2, &model->rscp);
	err |= at_params_get_short(param_list, 3, &model->ecno);
	err |= at_params_get_short(param_list, 4, &model->rsrq);
	err |= at_params_get_short(param_list, 5, &model->rsrp);

	if (err) {
		k_free(model);
		return NULL;
	} else {
		return model;
	}
}

static void *at_NOT_CESQ_decode(struct at_param_list *param_list,
			       u32_t valid_params)
{
	int err;

	if (valid_params != 2) {
		return NULL;
	}

	struct at_not_model_cesq *model =
			k_malloc(sizeof(struct at_not_model_cesq));

	if (!model) {
		return NULL;
	}
	memset(model, 0, sizeof(struct at_not_model_cesq));

	err = at_params_get_short(param_list, 0, &model->rsrp);
	err |= at_params_get_short(param_list, 1, &model->threshold_index);

	if (err) {
		k_free(model);
		return NULL;
	} else {
		return model;
	}
}

static void *at_CNUM_decode(struct at_param_list *param_list,
			   u32_t valid_params)
{
	int err;

	if (valid_params != 2) {
		return NULL;
	}

	struct at_cmd_model_cnum *model =
			k_malloc(sizeof(struct at_cmd_model_cnum));

	if (!model) {
		return NULL;
	}
	memset(model, 0, sizeof(struct at_cmd_model_cnum));

	err = at_params_get_string(param_list, 0, model->numberx, 32);
	err |= at_params_get_short(param_list, 1, &model->typex);

	if (err) {
		k_free(model);
		return NULL;
	} else {
		return model;
	}

}

static void *at_CEREG_decode(struct at_param_list *param_list,
			     u32_t valid_params)
{
	int err;
	int ret;

	if (valid_params == 0) {
		return NULL;
	}

	struct at_cmd_model_cereg *model =
			k_malloc(sizeof(struct at_cmd_model_cereg));

	if (!model) {
		return NULL;
	}
	memset(model, 0, sizeof(struct at_cmd_model_cereg));

	err = at_params_get_short(param_list, 0, &model->stat);

	if (valid_params > 1) {
		ret = at_params_get_string(param_list, 1, model->tac, 5);
		if (ret < 0) {
			err |= ret;
		}

		ret = at_params_get_string(param_list, 2, model->ci, 9);
		if (ret < 0) {
			err |= ret;
		}

		err |= at_params_get_short(param_list, 3, &model->act);
	}

	if (err) {
		k_free(model);
		return NULL;
	} else {
		return model;
	}
}

static int get_at_cmd_decode_handler_index(const char * const p_atstring)
{
	const struct at_cmd_cb *cb;

	for (u8_t i = 0; i < ARRAY_SIZE(m_at_cmds); ++i) {
		cb = &(m_at_cmds[i]);
		size_t len = strlen(cb->cmd_str);

		if (!strncmp(cb->cmd_str, p_atstring, len)) {
			return i;
		}
	}

	return -EIO;
}

int at_cmd_decode(char *at_message)
{
	int model_index = get_at_cmd_decode_handler_index(at_message);

	if (model_index < 0) {
		return -EIO;
	}

	u32_t valid_params;
	u32_t ofs = strlen(m_at_cmds[model_index].cmd_str)+1;
	void  *model_ptr;

	/* Parse response parameters. */
	int err = at_parser_params_from_str(&at_message[ofs], &param_list);

	if (err != 0) {
		return err;
	}
	valid_params = at_params_get_valid_count(&param_list);

	/* Check if the response is valid from the parser output. */
	if (valid_params == 0) {
		return -EIO;
	}

	model_ptr = m_at_cmds[model_index].at_cmd_decode_handler(&param_list,
							      valid_params);

	if (at_cmd_decoder_handler && model_ptr) {
		at_cmd_decoder_handler(m_at_cmds[model_index].model,
				       model_ptr);
	}

	k_free(model_ptr);

	return 0;
}

int at_cmd_decoder_init(void)
{
	at_params_list_init(&param_list, 10);

	return 0;
}

int at_cmd_decoder_deinit(void)
{
	at_params_list_free(&param_list);

	return 0;
}

int at_cmd_decoder_set_handler(at_cmd_decoder_handler_t handler)
{
	at_cmd_decoder_handler = handler;

	return 0;
}

