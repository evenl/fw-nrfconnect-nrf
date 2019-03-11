/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(at_cmd_interface);

#include <zephyr.h>
#include <stdio.h>
#include <net/socket.h>
#include <init.h>
#include <bsd_limits.h>

#include <at_cmd_interface.h>

#define RESPONSE_MAX_LEN  CONFIG_AT_CMD_RESPONSE_MAX_LEN
#define THREAD_PRIORITY   K_PRIO_PREEMPT(CONFIG_AT_CMD_THREAD_PRIO)

#define AT_CMD_OK_STR    "OK"
#define AT_CMD_ERROR_STR "ERROR"
#define AT_CMD_CMS_STR   "+CMS:"
#define AT_CMD_CME_STR   "+CME:"

static K_THREAD_STACK_DEFINE(socket_thread_stack, 2048);
K_MSGQ_DEFINE(return_code_msq, sizeof(enum at_cmd_rc), 2, 4);
static K_SEM_DEFINE(cmd_pending, 1, 1);

static int              common_socket_fd;
static int              last_error;
static struct k_thread  socket_thread;
static at_cmd_handler_t received_data_handler;
static char             *response_buf;
static u32_t            response_buf_len;

struct callback_work_item {
	struct k_work work;
	u32_t         data_len;
	char          data[RESPONSE_MAX_LEN];
};

static enum at_cmd_rc get_return_code(char *buf)
{
	enum at_cmd_rc return_code = AT_CMD_NO_RETURN_CODE;
	char *tmpstr = NULL;

	printk("%s\n", buf);

	do {
		tmpstr = strstr(buf, AT_CMD_OK_STR);
		if (tmpstr) {
			return_code = AT_CMD_OK;
			break;
		}

		tmpstr = strstr(buf, AT_CMD_ERROR_STR);
		if (tmpstr) {
			return_code = AT_CMD_ERROR;
			break;
		}

		tmpstr = strstr(buf, AT_CMD_CMS_STR);
		if (tmpstr) {
			return_code = AT_CMD_CMS;
			break;
		}

		tmpstr = strstr(buf, AT_CMD_CME_STR);
		if (tmpstr) {
			return_code = AT_CMD_CME;
			break;
		}
	} while (0);

	if ((return_code == AT_CMD_CMS) || (return_code == AT_CMD_CME)) {
		char tmpstr[5];

		memcpy(tmpstr, &buf[strlen(AT_CMD_CMS_STR)], 3);
		last_error = atoi(tmpstr);
	} else {
		last_error = 0;
	}

	if (tmpstr) {
		buf[tmpstr - buf] = '\0';
	}

	return return_code;
}

static void callback_worker(struct k_work *item)
{
	struct callback_work_item *data =
		CONTAINER_OF(item, struct callback_work_item, work);

	received_data_handler(data->data, data->data_len);

	k_free(data);
}


static void socket_thread_fn(void *arg1, void *arg2, void *arg3)
{
	int bytes_read;
	char buf[RESPONSE_MAX_LEN];
	enum at_cmd_rc return_code;
	bool write_with_response = false;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	LOG_DBG("Socket thread started");

	for (;;) {
		bytes_read = recv(common_socket_fd, buf, sizeof(buf), 0);

		return_code = get_return_code(buf);

		if (return_code != AT_CMD_NO_RETURN_CODE) {
			if (response_buf_len > 0) {
				if (response_buf_len >= strlen(buf)) {
					strcpy(response_buf, buf);
					response_buf_len = 0;
					write_with_response = true;
				} else {
					response_buf = NULL;
				}
			}

			k_msgq_put(&return_code_msq, &return_code, K_FOREVER);
		} else {
			write_with_response = false;
		}

		if (strlen(buf) == 0) {
			continue;
		}

		if (received_data_handler) {
			if (!write_with_response) {
				struct callback_work_item *item =
				k_malloc(sizeof(struct callback_work_item));

				k_work_init(&item->work, callback_worker);
				item->data_len = strlen(buf);
				memcpy(item->data, buf, item->data_len);

				k_work_submit(&item->work);
			}
		}  else {
			LOG_DBG("No handler registerd for received data");
		}
	}
}

enum at_cmd_rc at_cmd_write(const char *const cmd)
{
	int bytes_sent;
	enum at_cmd_rc return_code;

	k_sem_take(&cmd_pending, K_FOREVER);

	LOG_DBG("Sending command %s", cmd);
	bytes_sent = send(common_socket_fd, cmd, strlen(cmd), 0);

	k_msgq_get(&return_code_msq, &return_code, K_FOREVER);

	LOG_DBG("Bytes sent: %d", bytes_sent);

	k_sem_give(&cmd_pending);

	return return_code;
}

enum at_cmd_rc at_cmd_write_with_response(const char *const cmd,
					  char *buf,
					  u32_t buf_len)
{
	enum at_cmd_rc return_code;

	response_buf     = buf;
	response_buf_len = buf_len;

	return_code = at_cmd_write(cmd);

	return return_code;
}

void at_cmd_set_handler(at_cmd_handler_t handler)
{
	received_data_handler = handler;
}

int at_cmd_get_last_error(void)
{
	return last_error;
}

static int at_cmd_init(struct device *dev)
{
	ARG_UNUSED(dev);

	common_socket_fd = socket(AF_LTE, 0, NPROTO_AT);

	if (common_socket_fd == -1) {
		LOG_ERR("Socket could not be established.");
		return -EFAULT;
	}

	k_thread_create(&socket_thread, socket_thread_stack,
			K_THREAD_STACK_SIZEOF(socket_thread_stack),
			socket_thread_fn,
			NULL, NULL, NULL,
			THREAD_PRIORITY, 0, K_NO_WAIT);

	return 0;
}

SYS_INIT(at_cmd_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

