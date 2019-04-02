/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

/**
 * @file at_cmd_data_models.h
 *
 * @defgroup at_data_models Store AT commands in data models
 * @ingroup  at_cmd_parser
 * @{
 * @brief Data model of objects given by the Modem.
 */
#ifndef AT_CMD_DATA_MODELS_H__
#define AT_CMD_DATA_MODELS_H__

#include <zephyr/types.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum at_cmd_models {
	AT_CMD_NO_MODEL,
	AT_CMD_CESQ_MODEL,
	AT_NOT_CESQ_MODEL,
	AT_CMD_CEREG_MODEL,
	AT_CMD_CMT_MODEL,
	AT_CMD_CNUM_MODEL,
};

struct at_cmd_model_cesq {
	u16_t rxlev;
	u16_t ber;
	u16_t rscp;
	u16_t ecno;
	u16_t rsrq;
	u16_t rsrp;
};

struct at_not_model_cesq {
	u16_t rsrp;
	u16_t threshold_index;
};

struct at_cmd_model_cereg {
	u8_t n;
	u16_t stat;
	char tac[4 + 1];
	char ci[8 + 1];
	u16_t act;
	u16_t cause_type;
	u16_t reject_cause;
	u16_t active_time;
	u16_t periodic_tau;
};

struct at_cmd_model_cmt {
	char sender_addr[32];
	u16_t pdu_length;
	char pdu_data[160];
};

struct at_cmd_model_cnum {
	char numberx[32];
	u16_t typex;
};

#ifdef __cplusplus
}
#endif

#endif /* AT_CMD_DATA_MODELS_H__ */

/**@} */
