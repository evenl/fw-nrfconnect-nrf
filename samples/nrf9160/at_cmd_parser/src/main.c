/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <at_cmd_parser/at_cmd_decoder.h>


char *cereg_str = "+CEREG: 2,\"76C1\",\"0102DA04\",7";
char *cesq_str1 = "%CESQ: 62,3";
char *cesq_str2 = "+CESQ: 99,99,255,255,255,62";
char *cmt_str   = "+CMT: \"+4797664513\", 24\r\n"
		  	"06917429000171040A91747966543100009160402143708006C8329BFD0601";

void process_model(enum at_cmd_models model, void *model_ptr)
{
	static u32_t count;

	printk("------Message %04d-------\n", count++);

	switch (model) {
	case AT_CMD_CEREG_MODEL:
		printk("CEREG Stat: %d\n",
			((struct at_cmd_model_cereg *)model_ptr)->stat);
		printk("CEREG TAC:  %s\n",
			((struct at_cmd_model_cereg *)model_ptr)->tac);
		printk("CEREG CI:   %s\n",
			((struct at_cmd_model_cereg *)model_ptr)->ci);
		printk("CEREG Act:  %d\n",
			((struct at_cmd_model_cereg *)model_ptr)->act);
		break;
	case AT_CMD_CMT_MODEL:
		printk("CMT Origin number: %s\n",
			((struct at_cmd_model_cmt *)model_ptr)->sender_addr);
		printk("CMT PDU lenght: %d\n",
			((struct at_cmd_model_cmt *)model_ptr)->pdu_length);
		printk("CMT PDU payload: %s\n",
			((struct at_cmd_model_cmt *)model_ptr)->pdu_data);
		break;
	case AT_CMD_CESQ_MODEL:
		printk("CESQ Rxlev: %d\n",
			((struct at_cmd_model_cesq *)model_ptr)->rxlev);
		printk("CESQ ber:   %d\n",
			((struct at_cmd_model_cesq *)model_ptr)->ber);
		printk("CESQ rscp:  %d\n",
			((struct at_cmd_model_cesq *)model_ptr)->rscp);
		printk("CESQ ecno:  %d\n",
			((struct at_cmd_model_cesq *)model_ptr)->ecno);
		printk("CESQ rsrq:  %d\n",
			((struct at_cmd_model_cesq *)model_ptr)->rsrq);
		printk("CESQ rsrp:  %d\n",
			((struct at_cmd_model_cesq *)model_ptr)->rsrp);
		break;
	case AT_NOT_CESQ_MODEL:
		printk("Notification CESQ rsrp: %d\n",
			((struct at_not_model_cesq *)model_ptr)->rsrp);
		printk("Notification CESQ threshold index: %d\n",
			((struct at_not_model_cesq *)
				model_ptr)->threshold_index);
		break;
	default:
		break;
	}

	printk("-------------------------\n");
}

void main(void)
{
	printk("The AT command parser sample started\n");

	at_cmd_decoder_init(NULL);
	at_cmd_decoder_set_handler(process_model);

	at_cmd_decode(cereg_str);
	at_cmd_decode(cesq_str1);
	at_cmd_decode(cesq_str2);
	at_cmd_decode(cmt_str);
}
