/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr.h>
#include <bsd.h>
#include <lte_lc.h>
#include <net/bsdlib.h>
#include <net/socket.h>
#include <nrf_socket.h>
#include <at_cmd.h>
#include <download_client.h>
#include <logging/log.h>
#include <logging/log_ctrl.h>
#include <shell/shell.h>

LOG_MODULE_REGISTER(app);

/* Use this URL for IPv6 */
/* http://s3.dualstack.us-east-1.amazonaws.com/ */

#define HOST "s3.dualstack.us-east-1.amazonaws.com"
#define DELTA_12 "nordic-firmware-files/2acc3b5f-4cc5-417f-b0a2-4871c4ff09d5"
#define DELTA_23 "nordic-firmware-files/6dbbdc33-d44b-48fd-97e3-f82320e7b5eb"
#define DELTA_31 "nordic-firmware-files/1f6b08c8-576f-4ebd-8c3e-67ab547bdbd2"
#define TEST "humor/JOKES/100.txt" // "textfiles.com"

/* Socket fd used for modem DFU. */
static int fd;
static uint32_t offset;
static struct download_client dfu;
static struct k_sem bsdlib_sem;
static struct k_delayed_work delete_dwork;

extern int cert_provision(void);

static int download_client_callback(const struct download_client_evt *);


/**@brief Recoverable BSD library error. */
void bsd_recoverable_error_handler(uint32_t err)
{
	printk("bsd_recoverable_error_handler\n"
	       "Unexpected modem fault: err %u\n", err);
}

/**@brief Irrecoverable BSD library error. */
void bsd_irrecoverable_error_handler(uint32_t err)
{
	printk("bsd_irrecoverable_error_handler\n"
	       "Unexpected modem fault: err %u\n", err);
}

static nrf_dfu_err_t modem_error_get(void)
{
	int rc;
	nrf_dfu_err_t err;
	nrf_socklen_t len;

	len = sizeof(err);
	rc = nrf_getsockopt(fd, NRF_SOL_DFU, NRF_SO_DFU_ERROR, &err, &len);
	if (rc) {
		LOG_ERR("Unable to fetch modem error, errno %d", errno);
	}

	return err;
}
/* Print a message on the console when the firmware image has been deleted. */
static void delete_task(struct k_work *w)
{
	int rc;
	uint32_t off;
	nrf_socklen_t len;

	len = sizeof(off);
	rc = nrf_getsockopt(fd, NRF_SOL_DFU, NRF_SO_DFU_OFFSET, &off, &len);
	if (rc < 0) {
		k_delayed_work_submit(&delete_dwork, K_SECONDS(1));
	} else {
		printk("Firmware image deleted\n");
	}
}

static int bsdlib_sem_give(const struct shell *shell, size_t argc, char **argv)
{
	/* Give a semaphore and let the main thread run bsdlib_init(),
	 * which would otherwise overflow the stack of the shell thread.
	 */
	k_sem_give(&bsdlib_sem);

	return 0;
}

static int modem_connect(const struct shell *shell, size_t argc, char **argv)
{
	printk("Connecting..\n");
	at_cmd_init();
	lte_lc_init_and_connect();

	return 0;
}

static int fw_offsets(const struct shell *shell, size_t argc, char **argv)
{
	int rc;
	uint32_t off;
	nrf_socklen_t optlen;

	optlen = sizeof(off);
	rc = nrf_getsockopt(fd, NRF_SOL_DFU, NRF_SO_DFU_OFFSET, &off, &optlen);
	if (rc < 0) {
		printk("Offset request failed, errno %d\n", errno);
		return 0;
	}

	printk("Offset retrieved: %u\n", off);
	rc = nrf_setsockopt(fd, NRF_SOL_DFU, NRF_SO_DFU_OFFSET, &off,
			    sizeof(off));
	if (rc < 0) {
		printk("Failed to reset offset, errno %d\n", errno);
		return 0;
	}

	offset = off;
	printk("Offset set to %u.\n", offset);

	return 0;
}

static int fw_version(const struct shell *shell, size_t argc, char **argv)
{
	int rc;
	nrf_socklen_t len;
	nrf_dfu_fw_version_t version;

	len = sizeof(version);
	rc = nrf_getsockopt(fd, NRF_SOL_DFU, NRF_SO_DFU_FW_VERSION, &version,
			    &len);
	if (rc < 0) {
		printk("Firmware version request failed, errno %d\n", errno);
	} else {
		*((uint8_t *)version + sizeof(version) - 1) = '\0';
		printk("Modem firmware version: %s\n", version);
	}

	return 0;
}

static int fw_flash_size(const struct shell *shell, size_t argc, char **argv)
{
	int rc;
	nrf_socklen_t len;
	nrf_dfu_resources_t flash;

	len = sizeof(flash);
	rc = nrf_getsockopt(fd, NRF_SOL_DFU, NRF_SO_DFU_RESOURCES, &flash,
			    &len);
	if (rc < 0) {
		printk("Resource request failed, errno %d\n", errno);
	} else {
		printk("Flash memory available: %u\n", flash);
	}

	return 0;
}

static int fw_update(const struct shell *shell, size_t argc, char **argv)
{
	int rc;

	printk("Scheduling firmware update at next boot\n");
	rc = nrf_setsockopt(fd, NRF_SOL_DFU, NRF_SO_DFU_APPLY, NULL, 0);
	if (rc < 0) {
		printk("Failed to schedule update, errno %d\n", errno);
	}

	return 0;
}

static int fw_revert(const struct shell *shell, size_t argc, char **argv)
{
	int rc;

	printk("Scheduling firmware rollback at next boot\n");
	rc = nrf_setsockopt(fd, NRF_SOL_DFU, NRF_SO_DFU_REVERT, NULL, 0);
	if (rc < 0) {
		printk("Failed to schedule rollback, errno %d\n", errno);
	}

	return 0;
}

static int fw_delete(const struct shell *shell, size_t argc, char **argv)
{
	int rc;

	printk("Deleting firmware image, please wait...\n");
	rc = nrf_setsockopt(fd, NRF_SOL_DFU, NRF_SO_DFU_BACKUP_DELETE, NULL, 0);
	if (rc < 0) {
		printk("Failed to delete backup, errno %d\n", errno);
		return 0;
	}

	/* Reset offset used for downloading */
	offset = 0;

	/* Check when the operation has completed */
	k_delayed_work_submit(&delete_dwork, K_SECONDS(1));

	return 0;
}

static int dfusock_connect(const struct shell *shell, size_t argc, char **argv)
{
	printk("Creating DFU socket..\n");

	/* Create a socket for firmware update */
	fd = nrf_socket(NRF_AF_LOCAL, NRF_SOCK_STREAM, NRF_PROTO_DFU);
	if (fd < 0) {
		printk("Failed to open DFU socket.\n");
	}

	return 0;
}

static int dfusock_close(const struct shell *shell, size_t argc, char **argv)
{
	int err;

	printk("Closing DFU socket..\n");

	err = close(fd);
	if (err) {
		printk("Failed to close DFU socket\n");
	}

	return 0;
}

static int httpsock_connect(const struct shell *shell, size_t argc, char **argv)
{
	int err;
	struct download_client_cfg config = {
		.sec_tag = 0xA,
	};

	err = download_client_connect(&dfu, HOST, &config);
	if (err) {
		printk("Error %d\n", err);
	}

	return 0;
}

static int httpsock_close(const struct shell *shell, size_t argc, char **argv)
{
	int err;

	err = download_client_disconnect(&dfu);
	if (err) {
		printk("Error %d\n", err);
	}

	return 0;
}

static int reboot(const struct shell *shell, size_t argc, char **argv)
{
	printk("Rebooting..\n");

	log_panic();
	NVIC_SystemReset();
}

static int pause(const struct shell *shell, size_t argc, char **argv)
{
	printk("Pause download\n");

	download_client_pause(&dfu);
	return 0;
}

static int resume(const struct shell *shell, size_t argc, char **argv)
{
	printk("Resume download\n");

	download_client_resume(&dfu);
	return 0;
}

static int download_delta(char *file)
{
	int err;
	struct download_client_cfg config = {
		.sec_tag = 0xA,
	};

	err = download_client_connect(&dfu, HOST, &config);
	if (err) {
		printk("Failed to open HTTP socket, err %d\n", err);
	}

	err = download_client_start(&dfu, file, offset);
	if (err) {
		printk("Failed to start download, err %d\n", err);
	}

	return err;
}

static int dl_delta12(const struct shell *shell, size_t argc, char **argv)
{
	download_delta(DELTA_12);
	return 0;
}

static int dl_delta23(const struct shell *shell, size_t argc, char **argv)
{
	download_delta(DELTA_23);
	return 0;
}

static int dl_delta31(const struct shell *shell, size_t argc, char **argv)
{
	download_delta(DELTA_31);
	return 0;
}

/**@brief Initialize DFU socket. */
static int dfu_socket_init(void)
{
	int rc;
	nrf_socklen_t len;
	nrf_dfu_fw_version_t version;
	nrf_dfu_resources_t resource;
	nrf_dfu_fw_offset_t off;

	/* Create a socket for firmware update */
	fd = nrf_socket(NRF_AF_LOCAL, NRF_SOCK_STREAM, NRF_PROTO_DFU);
	if (fd < 0) {
		printk("Failed to open DFU socket.\n");
		return -1;
	}

	printk("Socket created\n");

	len = sizeof(version);
	rc = nrf_getsockopt(fd, NRF_SOL_DFU, NRF_SO_DFU_FW_VERSION, &version,
			    &len);
	if (rc < 0) {
		printk("Firmware version request failed, errno %d\n", errno);
		return -1;
	}

	*((uint8_t *)version + sizeof(version) - 1) = '\0';
	printk("Modem firmware version: %s\n", version);

	len = sizeof(resource);
	rc = nrf_getsockopt(fd, NRF_SOL_DFU, NRF_SO_DFU_RESOURCES, &resource,
			    &len);
	if (rc < 0) {
		printk("Resource request failed, errno %d\n", errno);
		return -1;
	}

	printk("Flash memory available: %u\n", resource);

	len = sizeof(off);
	rc = nrf_getsockopt(fd, NRF_SOL_DFU, NRF_SO_DFU_OFFSET, &off, &len);
	if (rc < 0) {
		printk("Offset request failed, errno %d\n", errno);
		return -1;
	}

	offset = off;
	printk("Offset retrieved: %u\n", offset);

	return rc;
}

static int download_client_callback(const struct download_client_evt *event)
{
	int err;

	switch (event->id) {
	case DOWNLOAD_CLIENT_EVT_FRAGMENT: {
		int sent;

		printk("Sending fragment (%u) to modem..\n",
		       event->fragment.len);

		sent = send(fd, event->fragment.buf, event->fragment.len, 0);
		if (sent < 0) {
			printk("Modem refused fragment, errno %d, dfu err %d\n",
			       errno, modem_error_get());
			download_client_disconnect(&dfu);
			printk("Socket closed\n");
			return -1;
		}
		break;
	}
	case DOWNLOAD_CLIENT_EVT_DONE: {
		printk("Download completed\n");
		err = download_client_disconnect(&dfu);
		if (err) {
			printk("Failed to close socket!\n");
		}
		printk("Socket closed\n");
		break;
	}
	case DOWNLOAD_CLIENT_EVT_ERROR: {
		printk("Download error %d\n", event->error);
		err = download_client_disconnect(&dfu);
		if (err) {
			printk("Failed to close socket!\n");
		}
		printk("Socket closed\n");
		break;
	}
	default:
		break;
	}

	return 0;
}

int main(void)
{
	int err;

	//modem_trace_enable();

	k_sem_init(&bsdlib_sem, 0, 1);
	k_delayed_work_init(&delete_dwork, delete_task);

	printk("Use `bsdinit` to initialize communication with the modem.\n");
	printk("When the modem firmware is being updated, "
	       "the operation can take up to a minute\n");

	/* Wait for use to manually request to initialize bsdlib */
	k_sem_take(&bsdlib_sem, K_FOREVER);

	printk("Initializing bsdlib, please wait..\n");
	err = bsdlib_init();
	switch (err) {
	case MODEM_DFU_RESULT_OK:
		printk("Modem firmware update successful!\n");
		printk("Modem will run the new firmware after reboot\n");
		k_thread_suspend(k_current_get());
		break;
	case MODEM_DFU_RESULT_UUID_ERROR:
	case MODEM_DFU_RESULT_AUTH_ERROR:
		printk("Modem firmware update failed\n");
		printk("Modem will run non-updated firmware on reboot.\n");
		break;
	case MODEM_DFU_RESULT_HARDWARE_ERROR:
	case MODEM_DFU_RESULT_INTERNAL_ERROR:
		printk("Modem firmware update failed\n");
		printk("Fatal error.\n");
		break;

	default:
		break;
	}

	cert_provision();

	printk("Initializing DFU socket\n");
	err = dfu_socket_init();
	if (err) {
		return 0;
	}

	err = download_client_init(&dfu, download_client_callback);
	if (err) {
		printk("download_client_init() failed, err: %d\n", err);
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(fw_subcmd,
	SHELL_CMD(offset, NULL, "Update modem firmware offset", fw_offsets),
	SHELL_CMD(version, NULL, "Request modem firmware version", fw_version),
	SHELL_CMD(flash, NULL, "Request modem resources", fw_flash_size),
	SHELL_CMD(update, NULL, "Update modem firmware", fw_update),
	SHELL_CMD(revert, NULL, "Rollback modem firmware", fw_revert),
	SHELL_CMD(delete, NULL, "Delete modem firmware", fw_delete),
	/* Firmware deltas */
	SHELL_CMD(d1, NULL, "Download modem firmware (1)", dl_delta12),
	SHELL_CMD(d2, NULL, "Download modem firmware (2)", dl_delta23),
	SHELL_CMD(d3, NULL, "Download modem firmware (3)", dl_delta31),
	SHELL_SUBCMD_SET_END,
);

SHELL_STATIC_SUBCMD_SET_CREATE(dfusock_cmd,
	SHELL_CMD(connect, NULL, "connect() DFU socket", dfusock_connect),
	SHELL_CMD(close, NULL, "close() DFU socket", dfusock_close),
	SHELL_SUBCMD_SET_END,
);

SHELL_STATIC_SUBCMD_SET_CREATE(httpsock_cmd,
	SHELL_CMD(connect, NULL, "connect() HTTP socket", httpsock_connect),
	SHELL_CMD(close, NULL, "close() HTTP socket", httpsock_close),
	SHELL_SUBCMD_SET_END,
);

SHELL_CMD_REGISTER(fw, &fw_subcmd, "Modem DFU commands", NULL);
SHELL_CMD_REGISTER(dfusock, &dfusock_cmd, "Manipulate DFU socket", NULL);
SHELL_CMD_REGISTER(httpsock, &httpsock_cmd, "Manipulate HTTP socket", NULL);

SHELL_CMD_REGISTER(bsdinit, NULL, "Initialize bsdlib", bsdlib_sem_give);
SHELL_CMD_REGISTER(linkon, NULL, "Connect", modem_connect);
SHELL_CMD_REGISTER(resume, NULL, "Resume download", resume);
SHELL_CMD_REGISTER(pause, NULL, "Pause download", pause);
SHELL_CMD_REGISTER(reboot, NULL, "Reboot", reboot);
