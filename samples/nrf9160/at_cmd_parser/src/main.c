/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>

#include <at_cmd_parser/at_cmd_parser.h>
#include <at_cmd_parser/at_params.h>

const char *str1 = "+CEREG: 2,\"76C1\",\"0102DA04\",65537";
const char *str2 = "+CEREG: -2, \"76C1\", \"0102DA04\", 65537";
const char *str3 = "CEREG: 2, \"76C1\", \"0102DA04\", 7";
const char *str4 = "+CEREG: 2, \"76C1, \"0102DA04\", 7";
const char *str5 = "+CEREG: 2, \"76C1, \"0102DA04, 7";
const char *str6 = "+CMT: \"+4797664513\", 24\r\n"
	     "06917429000171040A91747966543100009160402143708006C8329BFD0601";
const char *str7 = "mfw_nrf9160_0.7.0-23.prealpha";
const char *str8 = "+CPSMS: 1,,,\"10101111\",\"01101100\"";
const char *str9 = "+CGEQOSRDP: 0,0,,\r\n"
	     "+CGEQOSRDP: 1,2,,\r\n"
	     "+CGEQOSRDP: 2,4,,,1,65280000\r\n";
const char *str10 = "%CMNG: 12345678, 0, \"978C...02C4\","
	      "\"-----BEGIN CERTIFICATE-----"
	      "MIIBc464..."
	      "...bW9aAa4"
	      "-----END CERTIFICATE-----\"";

const char *str11 = "%XCBAND: (1,2,3,4,12,13)";
const char *str12 = "+CESQ: (99),(99),(255),(255),(255),(0-97,255)";



static struct at_param_list param_list;

void print_cereg()
{
	char   buf[32];
	u16_t  short_val;
	u32_t  int_val;
	size_t len;

	len = at_params_string_get(&param_list, 0, buf, 32);
	buf[len] = '\0';
	printk("Notification ID: %s\n", buf);

	at_params_short_get(&param_list, 1, &short_val);
	printk("Stat: %u\n", short_val);

	len = at_params_string_get(&param_list, 2, buf, 32);
	buf[len] = '\0';
	printk("TAC: %s\n", buf);

	len = at_params_string_get(&param_list, 3, buf, 32);
	buf[len] = '\0';
	printk("CI: %s\n", buf);

	at_params_int_get(&param_list, 4, &int_val);
	printk("ACT: %u\n", int_val);
}

void print_cmt()
{
	char   buf[64];
	u16_t  short_val;
	u32_t  int_val;
	size_t len;

	len = at_params_string_get(&param_list, 0, buf, 64);
	buf[len] = '\0';
	printk("Notification ID: %s\n", buf);

	len = at_params_string_get(&param_list, 1, buf, 64);
	buf[len] = '\0';
	printk("Number: %s\n", buf);

	at_params_short_get(&param_list, 2, &short_val);
	printk("PDU length: %u\n", short_val);

	len = at_params_string_get(&param_list, 3, buf, 64);
	buf[len] = '\0';
	printk("PDU payload: %s\n", buf);
}

void print_cgmr()
{
	char   buf[64];
	size_t len;

	len = at_params_string_get(&param_list, 0, buf, 64);
	buf[len] = '\0';
	printk("Firmware: %s\n", buf);
}

void print_cpsms()
{
	char buf[32];
	size_t len;
	u16_t  val;

	len = at_params_string_get(&param_list, 0, buf, 32);
	buf[len] = '\0';
	printk("Notification ID: %s\n", buf);

	at_params_short_get(&param_list, 1, &val);
	printk("Mode: %u\n", val);

	len = at_params_string_get(&param_list, 4, buf, 32);
	buf[len] = '\0';
	printk("Req periodic TAU: %s\n", buf);

	len = at_params_string_get(&param_list, 5, buf, 32);
	buf[len] = '\0';
	printk("Req Active time: %s\n", buf);
}

void print_qos(int param_count)
{
	char buf[32];
	size_t len;
	u16_t  val1;
	u32_t  val2;

	len = at_params_string_get(&param_list, 0, buf, 32);
	buf[len] = '\0';
	printk("Notification ID: %s\n", buf);

	at_params_short_get(&param_list, 1, &val1);
	printk("Param 1: %u\n", val1);

	at_params_short_get(&param_list, 2, &val1);
	printk("Param 2: %u\n", val1);

	if (at_params_get_type(&param_list, 3) == AT_PARAM_TYPE_EMPTY) {
		printk("Param 3 is empty\n");
	} else {
		len = at_params_string_get(&param_list, 3, buf, 32);
		buf[len] = '\0';
		printk("Param 3: %s (length: %d)\n", buf, len);
	}
	if (at_params_get_type(&param_list, 4) == AT_PARAM_TYPE_EMPTY) {
		printk("Param 4 is empty\n");
	} else {
		len = at_params_string_get(&param_list, 4, buf, 32);
		buf[len] = '\0';
		printk("Param 4: %s (length: %d)\n", buf, len);
	}

	if (param_count == 7) {
		at_params_short_get(&param_list, 5, &val1);
		printk("Param 5: %u\n", val1);

		at_params_int_get(&param_list, 6, &val2);
		printk("Param 6: %u\n", val2);		
	}
}

void print_cmng()
{
	char buf[128];
	size_t len;
	u32_t  val;

	len = at_params_string_get(&param_list, 0, buf, 32);
	buf[len] = '\0';
	printk("Notification ID: %s\n", buf);

	at_params_int_get(&param_list, 1, &val);
	printk("Param 1: %u\n", val);

	at_params_int_get(&param_list, 2, &val);
	printk("Param 2: %u\n", val);

	len = at_params_string_get(&param_list, 3, buf, 32);
	buf[len] = '\0';
	printk("Param 3: %s (length: %d)\n", buf, len);

	len = at_params_string_get(&param_list, 4, buf, 128);
	buf[len] = '\0';
	printk("Param 4: %s (length: %d)\n", buf, len);
}

void print_xcband()
{
	char buf[32];
	size_t len;
	u32_t  val[16];

	len = at_params_string_get(&param_list, 0, buf, 31);
	buf[len] = '\0';
	printk("Notification ID: %s\n", buf);

	len = at_params_array_get(&param_list, 1, val, sizeof(val));

	for (int i=0;i<(len / sizeof(u32_t));++i) {
		printk("Value %u: %u\n",i, val[i]);
	}
}

void print_cesq()
{
	char buf[32];
	size_t len;
	u32_t  val[16];

	len = at_params_string_get(&param_list, 0, buf, 31);
	buf[len] = '\0';
	printk("Notification ID: %s\n", buf);

	len = at_params_array_get(&param_list, 1, val, sizeof(val));

	for (int i=0;i<(len / sizeof(u32_t));++i) {
		printk("Value %u: %u\n",i, val[i]);
	}

	len = at_params_array_get(&param_list, 2, val, sizeof(val));

	for (int i=0;i<(len / sizeof(u32_t));++i) {
		printk("Value %u: %u\n",i, val[i]);
	}

	len = at_params_array_get(&param_list, 3, val, sizeof(val));

	for (int i=0;i<(len / sizeof(u32_t));++i) {
		printk("Value %u: %u\n",i, val[i]);
	}

	len = at_params_array_get(&param_list, 4, val, sizeof(val));

	for (int i=0;i<(len / sizeof(u32_t));++i) {
		printk("Value %u: %u\n",i, val[i]);
	}

	len = at_params_array_get(&param_list, 5, val, sizeof(val));

	for (int i=0;i<(len / sizeof(u32_t));++i) {
		printk("Value %u: %u\n",i, val[i]);
	}

	len = at_params_array_get(&param_list, 6, val, sizeof(val));

	for (int i=0;i<(len / sizeof(u32_t));++i) {
		printk("Value %u: %u\n",i, val[i]);
	}
}

void main(void)
{
	int     param_count;
	int     err;

	printk("The AT command parser sample started\n");

	at_params_list_init(&param_list, 10);

/*	err = at_parser_params_from_str(&str1, &param_list);
	printk("Str1 valid params: %d (err: %d)\n", at_params_valid_count_get(&param_list), err);
	print_cereg();

	err = at_parser_params_from_str(&str2, &param_list);
	printk("Str2 valid params: %d (err: %d)\n", at_params_valid_count_get(&param_list), err);
	print_cereg();

	err = at_parser_params_from_str(&str3, &param_list);
	printk("Str3 valid params: %d (err: %d)\n", at_params_valid_count_get(&param_list), err);
	print_cereg();

	err = at_parser_params_from_str(&str4, &param_list);
	printk("Str4 valid params: %d (err: %d)\n", at_params_valid_count_get(&param_list), err);
	print_cereg();

	err = at_parser_params_from_str(&str5, &param_list);
	printk("Str5 valid params: %d (err: %d)\n", at_params_valid_count_get(&param_list), err);
	print_cereg();

	err = at_parser_params_from_str(&str6, &param_list);
	printk("Str6 valid params: %d (err: %d)\n", at_params_valid_count_get(&param_list), err);
	print_cmt();

	err = at_parser_params_from_str(&str7, &param_list);
	printk("Str7 valid params: %d (err: %d)\n", at_params_valid_count_get(&param_list), err);
	print_cgmr();

	err = at_parser_params_from_str(&str8, &param_list);
	printk("Str8 valid params: %d (err: %d)\n", at_params_valid_count_get(&param_list), err);
	print_cpsms();

	do {
		err         = at_parser_params_from_str(&str9, &param_list);
		param_count = at_params_valid_count_get(&param_list);

		printk("Remainding str9: %s\n", str9);
		printk("Str9 valid params: %d (err: %d)\n", param_count, err);
		print_qos(param_count);
	} while (err == EAGAIN);

	err = at_parser_params_from_str(&str10, &param_list);
	printk("Str10 valid params %d (err: %d)\n", at_params_valid_count_get(&param_list), err);
	print_cmng();*/

	err = at_parser_params_from_str(&str11, &param_list);
	printk("Str11 valid params %d (err: %d)\n", at_params_valid_count_get(&param_list), err);
	print_xcband();

	err = at_parser_params_from_str(&str12, &param_list);
	printk("Str12 valid params %d (err: %d)\n", at_params_valid_count_get(&param_list), err);
	print_cesq();
}
