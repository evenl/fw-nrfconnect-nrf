/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <ctype.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr.h>
#include <zephyr/types.h>

#include <at_cmd_parser/at_cmd_parser.h>
#include <at_cmd_parser/at_utils.h>

#define AT_PARAM_SEPARATOR              ','
#define AT_CMD_SEPARATOR                ':'
#define AT_CMD_BUFFER_TERMINATOR        0
#define AT_CMD_STRING_IDENTIFIER        '\"'
#define AT_STANDARD_NOTIFICATION_PREFIX '+'
#define AT_PROP_NOTIFICATION_PREFX      '%'

enum at_parser_state {
	DETECT_TYPE,
	SEARCH_NEXT_PARAM,
	ARRAY,
	STRING,
	NUMBER,
	SMS_PDU,
	NOTIFICATION,
	OPTIONAL,
};

static enum at_parser_state state;
static enum at_parser_state prev_state;

static inline bool is_notification(char chr)
{
	if ((chr == AT_STANDARD_NOTIFICATION_PREFIX) ||
	    (chr == AT_PROP_NOTIFICATION_PREFX)) {
		return true;
	}

	return false;
}

static inline bool is_valid_notification_char(char chr)
{
	chr = toupper(chr);

	if ((chr >= 'A') && (chr <= 'Z')) {
		return true;
	}

	return false;
}

static inline bool is_terminated(char chr)
{
	if (chr == AT_CMD_BUFFER_TERMINATOR) {
		return true;
	}

	return false;
}

static inline bool is_separator(char chr)
{
	if ((chr == AT_PARAM_SEPARATOR) ||
	    (chr == AT_CMD_SEPARATOR)) {
		return true;
	}

	return false;
}

static inline bool is_lfcr(char chr)
{
	if ((chr == '\r') ||
	    (chr == '\n')) {
		return true;
	}

	return false;
}

static inline bool is_dblquote(char chr)
{
	if (chr == '"') {
		return true;
	}

	return false;
}

static inline bool is_array_start(char chr)
{
	if (chr == '(') {
		return true;
	}

	return false;
}

static inline bool is_array_stop(char chr)
{
	if (chr == ')') {
		return true;
	}

	return false;
}

static inline bool is_number(char chr)
{
	if (isdigit(chr) ||
	    (chr == '-') ||
	    (chr == '+')) {
		return true;
	}

	return false;
}

static inline void set_new_state(enum at_parser_state new_state)
{
	if ((state != SEARCH_NEXT_PARAM) &&
	    (state != DETECT_TYPE)) {
		prev_state = state;
	}

	state     = new_state;
}

static inline void reset_state(void)
{
	prev_state = DETECT_TYPE;
	state      = DETECT_TYPE;
}

/*
 * Internal function.
 * Parameters cannot be null. String must be null terminated.
 */
static int at_parse_param(const char ** at_params_str,
			  struct at_param_list * const list,
			  const size_t max_params)
{
	int index = 0;
	const char *str = *at_params_str;

	reset_state();

	while ((!is_terminated(*str)) &&
	       (index < max_params)) {

		str += isspace(*str);

		if (state == DETECT_TYPE) {
			if ((index == 0) &&
			    is_notification(*str)) {
				/* Only first parameter in the string can be
				   notification ID, (eg +CEREG:) */
				set_new_state(NOTIFICATION);
			} else if ((index > 0) &&
				is_notification(*str)) {
				/* If notifications is detected later in the
				   string we should stop parsing and return
				   EAGAIN */
				break;
			} else if (is_number(*str)) {
				set_new_state(NUMBER);

			} else if (is_dblquote(*str)) {
				set_new_state(STRING);
				str++;
			} else if (is_array_start(*str)) {
				set_new_state(ARRAY);
				str++;
			} else if (is_lfcr(*str) &&
			          (prev_state == NUMBER)) {
				/* If \n or \r is detected in the string and the
				   previous param was a number we assume the
				   next parameter is PDU data */

				while (is_lfcr(*str)) {
					str++;
				}

				set_new_state(SMS_PDU);
			} else if ((index == 0) &&
				    !is_notification(*str)) {
				/* If the string start without an notification
				   ID, we treat the whole string as one string
				   parameter */
				set_new_state(STRING);
			} else {
				if (is_separator(*str)) {
				/* If a separator is detected we have detected
				   and empty optional parameter */
					set_new_state(OPTIONAL);
				} else {
				/* The rule set is exhausted, and cannot
				   continue. Break the loop and return an error
				*/
					break;
				}
			}
		} else if (state == NOTIFICATION) {
			const char *start_ptr = str++;

			while (is_valid_notification_char(*str)) {
				str++;
			}

			at_params_string_put(list,
					     index, start_ptr,
					     str - start_ptr);

			set_new_state(SEARCH_NEXT_PARAM);
		} else if (state == OPTIONAL) {
			at_params_empty_put(list, index);
			str++;

			set_new_state(SEARCH_NEXT_PARAM);
		} else if (state == STRING) {
			const char *start_ptr = str;

			while (!is_dblquote(*str) &&
			       !is_terminated(*str)) {
				str++;
			}

			at_params_string_put(list,
					     index,
					     start_ptr, str - start_ptr);

			set_new_state(SEARCH_NEXT_PARAM);
		} else if (state == ARRAY) {
			size_t i = 0;
			u32_t  tmparray[16];

			tmparray[i++] = atoi(str++);

			while (!is_array_stop(*str) &&
			       !is_terminated(*str)) {

				if (is_separator(*str)) {
					tmparray[i++] = atoi(++str);
				} else {
					str++;
				}
			}

			at_params_array_put(list, index,
					    tmparray,
					    i * sizeof(u32_t));
			set_new_state(SEARCH_NEXT_PARAM);
		} else if (state == NUMBER) {
			int value = atoi(str);

			if (value <= USHRT_MAX) {
				at_params_short_put(list, index, (u16_t)value);
			} else {
				at_params_int_put(list, index, value);
			}

			set_new_state(SEARCH_NEXT_PARAM);
		} else if (state == SMS_PDU) {
			const char *start_ptr = str;

			while (isxdigit(*str)) {
				str++;
			}

			at_params_string_put(list,
					     index,
					     start_ptr, str - start_ptr);

			set_new_state(SEARCH_NEXT_PARAM);
		} else if (state == SEARCH_NEXT_PARAM) {

			while (!is_separator(*str) &&
			       !is_terminated(*str) &&
			       !is_lfcr(*str)) {
				str++;
			}

			if (is_terminated(*str)) {
				break;
			}

			if (!is_separator(*(str + 1)) &&
			    !is_lfcr(*(str + 1))) {
				str++;
			}

			index++;
			set_new_state(DETECT_TYPE);
		}
	}

	*at_params_str = str;

	if ((index == max_params) &&
	    (!is_terminated(*str))) {
		return E2BIG;
	}

	if (!is_terminated(*str)) {
		return EAGAIN;
	}

	return 0;
}

int at_parser_params_from_str(const char **at_params_str,
			      struct at_param_list * const list)
{
	return at_parser_max_params_from_str(at_params_str, list,
					     list->param_count);
}

int at_parser_max_params_from_str(const char **at_params_str,
				  struct at_param_list * const list,
				  size_t max_params_count)
{
	if (at_params_str == NULL || *at_params_str == NULL ||
	    list == NULL || list->params == NULL) {
		return -EINVAL;
	}

	at_params_list_clear(list);

	max_params_count = MIN(max_params_count, list->param_count);

	return at_parse_param(at_params_str, list, max_params_count);
}
