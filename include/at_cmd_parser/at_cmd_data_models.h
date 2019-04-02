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

/**
 * @file at_cmd_data_models.h
 *
 * @brief In this file are all datamodels used by the default configuration of
 *        the at_cmd_decoder defined. Custom decoders and datamodels can be
 *        used if a separate decoder table is defined, check the
 *        @ref at_cmd_decoder.h documentation. Documentation for the different
 *        AT commands, notifications and responses can be found in the
 *        'nrf91 AT Commands' document found on https://www.nordicsemi.com
 */

/** List of available data models for AT responses and notifications */
enum at_cmd_models {
	/** A no model entry */
	AT_CMD_NO_MODEL,
	/** A +CESQ reply datamodel */
	AT_CMD_CESQ_MODEL,
	/** A %CESQ notification datamodel */
	AT_NOT_CESQ_MODEL,
	/** A +CEREG reply datamodel */
	AT_CMD_CEREG_MODEL,
	/** A +CEREG notification datamodel */
	AT_CMD_CMT_MODEL,
	/** A CNUM reply datamode */
	AT_CMD_CNUM_MODEL,
};

/** CESQ response data model definition */
struct at_cmd_model_cesq {
	u16_t rxlev;
	u16_t ber;
	u16_t rscp;
	u16_t ecno;
	u16_t rsrq;
	u16_t rsrp;
};

/** CESQ notification data model definition */
struct at_not_model_cesq {
	u16_t rsrp;
	u16_t threshold_index;
};

/** CEREG notification data model definition */
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

/** CMT notification data model definition */
struct at_cmd_model_cmt {
	char sender_addr[32];
	u16_t pdu_length;
	char pdu_data[160];
};

/** CNUM response data model definition */
struct at_cmd_model_cnum {
	char numberx[32];
	u16_t typex;
};

#ifdef __cplusplus
}
#endif

#endif /* AT_CMD_DATA_MODELS_H__ */

/**@} */
