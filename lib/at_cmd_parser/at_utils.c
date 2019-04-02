/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <at_cmd_parser/at_utils.h>

u32_t at_params_get_space_count(const char **str)
{
	if ((!str) || (!(*str))) {
		return 0;
	}

	u32_t space_count = 0;

	while (isspace(**str) && (**str)) {
		space_count++;
		(*str)++;
	}
	return space_count;
}

size_t at_params_get_cmd_length(const char *const str)
{
	if (str == NULL) {
		return 0;
	}

	const char *cmd = (const char *)str;
	size_t len = 0;

	while (*cmd && (*cmd != '?') && (*cmd != ';')) {
		len++;
		cmd++;
	}
	return len;
}
