/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(net_http_client_sample, LOG_LEVEL_DBG);

#define RUNNER_THREAD_STACK 2048
#define RUNNER_PRIORITY     5

#include <zephyr.h>
#include <net/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <shell/shell.h>
#include <drivers/gpio.h>
#include <modem/at_cmd.h>
#include <modem/bsdlib.h>
#include <modem/lte_lc.h>
#include <modem/modem_key_mgmt.h>
#include <net/download_client.h>
#include <drivers/gps.h>

#include <net/mqtt.h>

#define HTTP_PATH "/"
#define HTTP_GET_TEMPLATE "GET %s HTTP/1.1\r\n\"Host: %s\"\r\n\r\n"

#define SECURITY_TAG 0x7A

K_THREAD_STACK_DEFINE(runner_stack_area, RUNNER_THREAD_STACK);

static char     config_url[80]     = "";
static char     config_ip[80]      = "192.168.12.55";
static uint16_t config_port        = 9087;
static uint16_t config_buffer_size = 256;
static uint16_t config_sleep       = 100;
static bool     config_tls_enable  = false;

static uint8_t mqtt_rx_buffer[1024];
static uint8_t mqtt_tx_buffer[1024];
static uint8_t mqtt_payload_buf[1024];

static struct mqtt_client      mqtt_client;
static struct sockaddr_storage mqtt_broker;
static bool                    mqtt_connected;

static int            ip_fd = -1;

int client_fd;

struct k_work   run_job;
struct k_work_q runner_work_queue;

struct download_client     client;

atomic_t test_ctrl = ATOMIC_INIT(0);

#define TEST_CTRL_RUN_BIT    0

void modem_trace_enable(void)
{
    /* GPIO configurations for trace and debug */
    #define CS_PIN_CFG_TRACE_CLK    21 //GPIO_OUT_PIN21_Pos
    #define CS_PIN_CFG_TRACE_DATA0  22 //GPIO_OUT_PIN22_Pos
    #define CS_PIN_CFG_TRACE_DATA1  23 //GPIO_OUT_PIN23_Pos
    #define CS_PIN_CFG_TRACE_DATA2  24 //GPIO_OUT_PIN24_Pos
    #define CS_PIN_CFG_TRACE_DATA3  25 //GPIO_OUT_PIN25_Pos

    // Configure outputs.
    // CS_PIN_CFG_TRACE_CLK
    NRF_P0_NS->PIN_CNF[CS_PIN_CFG_TRACE_CLK] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                               (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    // CS_PIN_CFG_TRACE_DATA0
    NRF_P0_NS->PIN_CNF[CS_PIN_CFG_TRACE_DATA0] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                 (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    // CS_PIN_CFG_TRACE_DATA1
    NRF_P0_NS->PIN_CNF[CS_PIN_CFG_TRACE_DATA1] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                 (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    // CS_PIN_CFG_TRACE_DATA2
    NRF_P0_NS->PIN_CNF[CS_PIN_CFG_TRACE_DATA2] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                 (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    // CS_PIN_CFG_TRACE_DATA3
    NRF_P0_NS->PIN_CNF[CS_PIN_CFG_TRACE_DATA3] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                 (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    NRF_P0_NS->DIR = 0xFFFFFFFF;
}

void bsd_recoverable_error_handler(uint32_t error) {
	printk("RECOVERABLE ERROR\n");
}

static int resolve_dns(struct sockaddr_storage *info)
{
	int err;
	struct addrinfo *result;
	struct addrinfo *addr;
	struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM
	};

	err = getaddrinfo(config_url, NULL, &hints, &result);
	if (err) {
		printk("ERROR: getaddrinfo failed %d\n", err);

		return -1;
	}

	addr = result;
	err = -ENOENT;

	/* Look for address of the broker. */
	while (addr != NULL) {
		/* IPv4 Address. */
		if (addr->ai_addrlen == sizeof(struct sockaddr_in)) {
			struct sockaddr_in *broker4 =
				((struct sockaddr_in *)info);
			char ipv4_addr[NET_IPV4_ADDR_LEN];

			broker4->sin_addr.s_addr =
				((struct sockaddr_in *)addr->ai_addr)
				->sin_addr.s_addr;
			broker4->sin_family = AF_INET;
			broker4->sin_port = htons(config_port);

			inet_ntop(AF_INET, &broker4->sin_addr.s_addr,
				  ipv4_addr, sizeof(ipv4_addr));
			printk("IPv4 Address found %s\n", ipv4_addr);

			break;
		} else {
			printk("ai_addrlen = %u should be %u or %u\n",
				(unsigned int)addr->ai_addrlen,
				(unsigned int)sizeof(struct sockaddr_in),
				(unsigned int)sizeof(struct sockaddr_in6));
		}

		addr = addr->ai_next;
		break;
	}

	/* Free the address. */
	freeaddrinfo(result);

	return 0;
}

static int setup_socket(sa_family_t family, int *sock, struct sockaddr *addr, socklen_t addr_len,
		        enum net_ip_protocol protocol, enum net_sock_type type)
{
        const char *family_str = family == AF_INET ? "IPv4" : "IPv6";
        int ret = 0;

        memset(addr, 0, addr_len);

        if (family == AF_INET) {
                net_sin(addr)->sin_family = AF_INET;
                net_sin(addr)->sin_port = htons(config_port);
                inet_pton(family, config_ip, &net_sin(addr)->sin_addr);
        } else {
                net_sin6(addr)->sin6_family = AF_INET6;
                net_sin6(addr)->sin6_port = htons(config_port);
                inet_pton(family, config_ip, &net_sin6(addr)->sin6_addr);
        }

        *sock = socket(family, type, protocol);

        if (*sock < 0) {
                LOG_ERR("Failed to create %s HTTP socket (%d)", family_str,
                        -errno);
		ret = -1;
        }

        return ret;
}

static int connect_socket(sa_family_t family, int *sock, enum net_ip_protocol protocol, enum net_sock_type type)
{
        int ret;
	static struct sockaddr_in addr;

        ret = setup_socket(family, sock, (struct sockaddr*)&addr, sizeof(struct sockaddr), protocol, type);
        if (ret < 0 || *sock < 0) {
                return -1;
        }

        ret = connect(*sock, (struct sockaddr*)&addr, sizeof(struct sockaddr));
        if (ret < 0) {
                LOG_ERR("Cannot connect to %s remote (%d)",
                        family == AF_INET ? "IPv4" : "IPv6",
                        -errno);
                ret = -errno;
        }

        return ret;
}

int download_client_callback(const struct download_client_evt *event)
{
	return 0;
}

void app_tcp_start(struct k_work *item)
{
	int  read;
	static char buf[1024];
	bool run;
	uint32_t connects=0;

	LOG_INF("Starting TCP client");

	memset(buf, 0, 1024);

	atomic_clear(&test_ctrl);
	atomic_set_bit(&test_ctrl, TEST_CTRL_RUN_BIT);

	do {
		connects++;
		if (ip_fd == -1) {
			LOG_INF("Connecting");
			if (connect_socket(AF_INET, &ip_fd, IPPROTO_TCP, SOCK_STREAM) == -ENETUNREACH)
			{
				LOG_ERR("Not able to connect, network not avaiable");
				atomic_clear_bit(&test_ctrl, TEST_CTRL_RUN_BIT);
				break;
			}
		} else {
			LOG_INF("Allready connected, skipping");
		}

		if (ip_fd < 0)
		{
			LOG_ERR("Not able to connect to server, errno: %d\n", errno);
			atomic_clear_bit(&test_ctrl, TEST_CTRL_RUN_BIT);
			break;
		} else {
			LOG_INF("Connected");
		}

		LOG_INF("Download started");

		uint32_t total_bytes  = 0;
		uint32_t error_count  = 0;
		uint32_t validate_pos = 0;
		uint8_t  validate_idx = 0;
		uint32_t i;
		uint32_t c=0;

		run = true;

		send(ip_fd, "Hello", 6, 0);

		do {
			read = recv(ip_fd, buf, config_buffer_size, MSG_DONTWAIT);
			if (read > 0) {
				total_bytes += read;

				LOG_INF("Validating current %d bytes chunck, total bytes: %d, connects: %d", read, total_bytes, connects);
				for(i=0;i<read;i++)
				{
					if (buf[i] != (c%256)) {
						if ((c%256) == 0) {
							if (buf[i] != validate_idx) {
								LOG_ERR("Validation failed, expected index: %d, got: %d", validate_idx, buf[i]);
								error_count++;
							}
							validate_idx++;
						} else {
							LOG_ERR("Validation failed, expected: %d, got: %d", c%256, buf[i]);
							error_count++;
						}
					} else if (buf[i] == 0) {
						validate_idx++;
					}

					c++;
				}

			} else if (read == 0) {
				LOG_INF("Received NULL packet");
				run = false;
			} else {
				if (errno != EAGAIN) {
					error_count++;
					if (error_count > 5) 
					{
						run = false;
					}
					LOG_ERR("Not handled errno: %d, bytes: %d", errno, total_bytes);
				}
			}

			if (config_sleep > 0) {
				k_sleep(K_MSEC(config_sleep));
			}
		} while (run && atomic_test_bit(&test_ctrl, TEST_CTRL_RUN_BIT));

		LOG_INF("Download done");
		LOG_INF("Total bytes: %d", total_bytes);
		close(ip_fd);
		ip_fd = -1;
		k_sleep(K_MSEC(1000));

	} while (atomic_test_bit(&test_ctrl, TEST_CTRL_RUN_BIT));

end:
	close(ip_fd);
	ip_fd = -1;
}

static void gps_handler(struct device *dev, struct gps_event *evt)
{
	ARG_UNUSED(dev);

	switch (evt->type) {
	case GPS_EVT_SEARCH_STARTED:
		LOG_DBG("GPS_EVT_SEARCH_STARTED");
		break;
	case GPS_EVT_SEARCH_STOPPED:
		LOG_DBG("GPS_EVT_SEARCH_STOPPED");
		break;
	case GPS_EVT_SEARCH_TIMEOUT:
		LOG_DBG("GPS_EVT_SEARCH_TIMEOUT");
		break;
	case GPS_EVT_OPERATION_BLOCKED:
		LOG_DBG("GPS_EVT_OPERATION_BLOCKED");
		break;
	case GPS_EVT_OPERATION_UNBLOCKED:
		LOG_DBG("GPS_EVT_OPERATION_UNBLOCKED");
		break;
	case GPS_EVT_AGPS_DATA_NEEDED:
		LOG_DBG("GPS_EVT_AGPS_DATA_NEEDED");
		break;
	case GPS_EVT_PVT:
		LOG_DBG("GPS_EVT_PVT");
		break;
	case GPS_EVT_PVT_FIX:
		LOG_DBG("GPS_EVT_PVT_FIX");
		break;
	case GPS_EVT_NMEA_FIX:
		LOG_DBG("GPS_EVT_NMEA_FIX");
		break;
	default:
		break;
	}
}

static int mqtt_test_subscribe(void)
{
	struct mqtt_topic subscribe_topic = {
		.topic = {
			.utf8 = "/even-debug-client/subscribe/topic",
			.size = strlen("/even-debug-client/subscribe/topic"),
		},
		.qos = MQTT_QOS_1_AT_LEAST_ONCE
	};

	const struct mqtt_subscription_list subscription_list = {
		.list = &subscribe_topic,
		.list_count = 1,
		.message_id = 1234
	};

	LOG_INF("Subscring to topic");

	return mqtt_subscribe(&mqtt_client, &subscription_list);
}

void mqtt_evt_handler(struct mqtt_client *const c,
		      const struct mqtt_evt *evt)
{
	int err=0;

	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		if (evt->result != 0) {
			LOG_INF("MQTT connect failed %d", evt->result);
			break;
		}

		LOG_INF("[%s:%d] MQTT client connected!", __func__, __LINE__);
		mqtt_test_subscribe();
		break;

	case MQTT_EVT_DISCONNECT:
		LOG_INF("[%s:%d] MQTT client disconnected %d", __func__,
		       __LINE__, evt->result);

		atomic_clear_bit(&test_ctrl, TEST_CTRL_RUN_BIT);
		break;

	case MQTT_EVT_PUBLISH: {
		const struct mqtt_publish_param *p = &evt->param.publish;

		LOG_INF("[%s:%d] MQTT PUBLISH result=%d len=%d", __func__,
		       __LINE__, evt->result, p->message.payload.len);
//		err = publish_get_payload(c, p->message.payload.len);
		if (err >= 0) {
//			data_print("Received: ", payload_buf,
//				p->message.payload.len);
			/* Echo back received data */
//			data_publish(&client, MQTT_QOS_1_AT_LEAST_ONCE,
//				payload_buf, p->message.payload.len);
		} else {
			LOG_INF("mqtt_read_publish_payload: Failed! %d", err);
			LOG_INF("Disconnecting MQTT client...");

			err = mqtt_disconnect(c);
			if (err) {
				LOG_INF("Could not disconnect: %d", err);
			}
		}
	} break;

	case MQTT_EVT_PUBACK:
		if (evt->result != 0) {
			LOG_INF("MQTT PUBACK error %d", evt->result);
			break;
		}

		LOG_INF("[%s:%d] PUBACK packet id: %u", __func__, __LINE__,
				evt->param.puback.message_id);
		break;

	case MQTT_EVT_SUBACK:
		if (evt->result != 0) {
			LOG_INF("MQTT SUBACK error %d", evt->result);
			break;
		}

		LOG_INF("[%s:%d] SUBACK packet id: %u", __func__, __LINE__,
				evt->param.suback.message_id);
		break;

	default:
		LOG_INF("[%s:%d] default: %d", __func__, __LINE__,
				evt->type);
		break;
	}
}

void app_mqtt_start(struct k_work *item)
{
	int err;
	static struct pollfd fds;

	if (mqtt_client.transport.type == MQTT_TRANSPORT_NON_SECURE) {
		fds.fd = mqtt_client.transport.tcp.sock;
	} else {
#if defined(CONFIG_MQTT_LIB_TLS)
		fds.fd = mqtt_client.transport.tls.sock;
#else
		return -ENOTSUP;
#endif
	}

	fds.events = POLLIN;

	atomic_set_bit(&test_ctrl, TEST_CTRL_RUN_BIT);

	while(atomic_test_bit(&test_ctrl, TEST_CTRL_RUN_BIT) == 1)
	{
		err = poll(&fds, 1, mqtt_keepalive_time_left(&mqtt_client));
		if (err < 0) {
			LOG_INF("ERROR: poll %d", errno);
			break;
		}

		err = mqtt_live(&mqtt_client);
		if ((err != 0) && (err != -EAGAIN)) {
			LOG_INF("ERROR: mqtt_live %d", err);
			break;
		}

		if ((fds.revents & POLLIN) == POLLIN) {
			err = mqtt_input(&mqtt_client);
			if (err != 0) {
				LOG_INF("ERROR: mqtt_input %d", err);
				break;
			}
		}

		if ((fds.revents & POLLERR) == POLLERR) {
			LOG_INF("POLLERR");
			break;
		}

		if ((fds.revents & POLLNVAL) == POLLNVAL) {
			LOG_INF("POLLNVAL");
			break;
		}
	}

	LOG_INF("MQTT thread is dead");
}

int main()
{
	int err;
	char buf[2048];
	static struct device *gps_dev;

	k_work_q_start(&runner_work_queue, runner_stack_area,
		       K_THREAD_STACK_SIZEOF(runner_stack_area), RUNNER_PRIORITY);

	lte_lc_init_and_connect();

	LOG_DBG("Ready");

	at_cmd_write("AT+CGMR", buf, sizeof(buf),  NULL);
	LOG_INF("%s", buf);

	at_cmd_write("AT%XSYSTEMMODE=1,0,1,0", NULL, 0, NULL);
	at_cmd_write("AT+CFUN=1", NULL, 0, NULL);

	gps_dev = device_get_binding("NRF9160_GPS");
	if (gps_dev == NULL) {
		LOG_ERR("Could not get binding to nRF9160 GPS");
		return -1;
	}

	err = gps_init(gps_dev, gps_handler);
	if (err) {
		LOG_ERR("Could not initialize GPS, error: %d", err);
		return -1;
	}

	while (1)
	{
		k_sleep(K_MSEC(1000));
	}

	return 0;
}

static int cmd_config_print(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "URL:         %s", config_url);
	shell_print(shell, "IP:          %s", config_ip);
	shell_print(shell, "Port:        %d", config_port);
	shell_print(shell, "Buffer size: %d", config_buffer_size);
	shell_print(shell, "Sleep time:  %d", config_sleep);
	shell_print(shell, "TLS:         %s", config_tls_enable == 1 ? "enabled" : "disabled");

	shell_print(shell, "Errno: %d", errno);

	return 0;
}

static int cmd_config_set_url(const struct shell *shell, size_t argc, char **argv)
{
	if (argc == 2)
	{
		strcpy(config_url, argv[1]);
	} else {
		shell_print(shell, "Wrong amount of params, should be one");
		return -1;
	}

	return 0;
}

static int cmd_config_set_ip(const struct shell *shell, size_t argc, char **argv)
{
	if (argc == 2)
	{
		strcpy(config_ip, argv[1]);
	} else {
		shell_print(shell, "Wrong amount of params, should be one");
		return -1;
	}

	return 0;
}
static int cmd_config_set_port(const struct shell *shell, size_t argc, char **argv)
{
	if (argc == 2)
	{
		config_port = atoi(argv[1]);
	} else {
		shell_print(shell, "Wrong amount of params, should be one");
		return -1;
	}

	return 0;
}
static int cmd_config_set_buffer_size(const struct shell *shell, size_t argc, char **argv)
{
	if (argc == 2)
	{
		config_buffer_size = atoi(argv[1]);
	} else {
		shell_print(shell, "Wrong amount of params, should be one");
		return -1;
	}

	return 0;
}

static int cmd_config_set_sleep(const struct shell *shell, size_t argc, char **argv)
{
	if (argc == 2)
	{
		config_sleep = atoi(argv[1]);
	} else {
		shell_print(shell, "Wrong amount of params, should be one");
		return -1;
	}

	return 0;
}

static int cmd_config_set_trace(const struct shell *shell, size_t argc, char **argv)
{
	if (argc == 2)
	{
		if (atoi(argv[1]) == 1)
		{
			modem_trace_enable();
			//at_cmd_write("AT%XMODEMTRACE=2,,3,\"0000000c000000000000000000300200000000000000000000000000000000000000000000\"", buf, sizeof(buf), NULL);

			if (at_cmd_write("AT%XMODEMTRACE=1,1", NULL, 0, NULL) != 0) {
				LOG_ERR("Writing AT command failed");
			}
		} else {
			LOG_ERR("Trace disable not implemented yet");
			return -1;
		}
	} else {
		shell_print(shell, "Wrong amount of params, should be one");
		return -1;
	}

	return 0;
}

static int cmd_config_set_tls(const struct shell *shell, size_t argc, char **argv)
{
	if (argc == 2) {
		if (atoi(argv[1]) == 1) {
			config_tls_enable = 1;
		} else if (atoi(argv[1]) == 0) {
			config_tls_enable = 0;
		} else {
			LOG_ERR("Value can either be 1 (enable) or 0 (disable)");
			return -1;
		}
	} else {
		shell_print(shell, "Wrong amount of params, should be one");
		return -1;
	}

	return 0;
}

static int cmd_shutdown_modem(const struct shell *shell, size_t argc, char **argv)
{
	at_cmd_write("AT+CFUN=0", NULL, 0, NULL);

	int ret = bsdlib_shutdown();

	LOG_INF("BSDlib shutdown returned %d", ret);

	return 0;
}

static int cmd_init_modem(const struct shell *shell, size_t argc, char **argv)
{
	int ret = bsdlib_init();
	LOG_INF("BSDLib init returned %d", ret);

	while(at_cmd_write("AT%XSYSTEMMODE=1,0,1,0", NULL, 0, NULL) == -EHOSTDOWN)
	{
		k_sleep(K_MSEC(100));
	}
	at_cmd_write("AT+CFUN=1", NULL, 0, NULL);

	return 0;
}

static int cmd_start_gps(const struct shell *shell, size_t argc, char **argv)
{
	int err;
	struct gps_config gps_cfg = {
		.nav_mode = GPS_NAV_MODE_CONTINUOUS,
		.power_mode = GPS_POWER_MODE_DISABLED,
		.timeout = 120,
		.interval = 240,
	};
	static struct device *gps_dev;

	gps_dev = device_get_binding("NRF9160_GPS");
	err = gps_start(gps_dev, &gps_cfg);
	if (err) {
		LOG_ERR("Failed to start GPS, error: %d", err);
		return -1;
	}

	return 0;
}

static int cmd_stop_gps(const struct shell *shell, size_t argc, char **argv)
{
	int err;
	static struct device *gps_dev;

	gps_dev = device_get_binding("NRF9160_GPS");
	err = gps_stop(gps_dev);
	if (err) {
		LOG_ERR("Failed to stop GPS, error: %d", err);
		return -1;
	}

	return 0;
}

static int cmd_ip_connect(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		LOG_ERR("Wrong amount of arguments");
	}

	if (strcmp("tcp", argv[1]) == 0) {
		connect_socket(AF_INET, &ip_fd, IPPROTO_TCP, SOCK_STREAM);
		LOG_INF("Socket open: %d", ip_fd);
	} else {
		LOG_ERR("Protocol not recognized");
	}

	return 0;
}

static int cmd_at(const struct shell *shell, size_t argc, char **argv)
{
	char tmpstr[1024];
	int ret;

	if (argc != 2) {
		LOG_ERR("Wrong amount of arguments");
	}

	ret = at_cmd_write(argv[1], tmpstr, 1024, NULL);
	if (ret == AT_CMD_OK) {
		LOG_INF("OK");
		LOG_INF("%s", tmpstr);
	} else {
		LOG_INF("ERROR (err: %d)", ret);
	}

	return 0;
}

static int cmd_run_tcp_test(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		LOG_ERR("Wrong amount of arguments");
	}

	k_work_init(&run_job, app_tcp_start);
	k_work_submit(&run_job);

	return 0;
}

void cmd_run_http_download_test(const struct shell *shell, size_t argc, char **argv)
{
	struct download_client_cfg cfg = {
		.sec_tag = -1,
	};

	download_client_init(&client, download_client_callback);
	download_client_connect(&client, "212.183.159.230", &cfg);
	download_client_start(&client, "5MB.zip", 0);
}


static int cmd_stop_test(const struct shell *shell, size_t argc, char **argv)
{
	atomic_clear_bit(&test_ctrl, TEST_CTRL_RUN_BIT);

	return 0;
}

static int cmd_mqtt_connect(const struct shell *shell, size_t argc, char **argv)
{

	if (atomic_test_bit(&test_ctrl, TEST_CTRL_RUN_BIT) == 1) {
		LOG_WRN("Allready connected");
		return -1;
	}

	mqtt_client_init(&mqtt_client);
	resolve_dns(&mqtt_broker);

	mqtt_client.broker           = &mqtt_broker;
	mqtt_client.evt_cb           = mqtt_evt_handler;
	mqtt_client.client_id.utf8   = "even-debug-client";
	mqtt_client.client_id.size   = strlen("even-debug-client");
	mqtt_client.password         = NULL;
	mqtt_client.user_name        = NULL;
	mqtt_client.protocol_version = MQTT_VERSION_3_1_1;

	mqtt_client.rx_buf      = mqtt_rx_buffer;
	mqtt_client.rx_buf_size = sizeof(mqtt_rx_buffer);
	mqtt_client.tx_buf      = mqtt_rx_buffer;
	mqtt_client.tx_buf_size = sizeof(mqtt_tx_buffer);

	if (config_tls_enable) {
		mqtt_client.transport.type = MQTT_TRANSPORT_SECURE;
	} else {
		mqtt_client.transport.type = MQTT_TRANSPORT_NON_SECURE;
	}

	int err = mqtt_connect(&mqtt_client);
	if (err != 0) {
		LOG_ERR("Could not connect to MQTT broker (err: %d)", err);
	} else {
		LOG_INF("Connected to MQTT broker");
		mqtt_connected = true;
	}

	k_work_init(&run_job, app_mqtt_start);
	k_work_submit(&run_job);

	return 0;
}

static int cmd_mqtt_disconnect(const struct shell *shell, size_t argc, char **argv)
{
	int err;

	if (atomic_test_bit(&test_ctrl, TEST_CTRL_RUN_BIT) == 0) {
		LOG_WRN("Not connected");
		return -1;
	}

	err = mqtt_disconnect(&mqtt_client);
	LOG_INF("Closing MQTT connection with err: %d", err);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_config,
	SHELL_CMD(print,       NULL, "Print configuration", cmd_config_print),
	SHELL_CMD(url,         NULL, "Set download URL", cmd_config_set_url),
	SHELL_CMD(ip,          NULL, "Set download IP", cmd_config_set_ip),
	SHELL_CMD(port,        NULL, "Set download port", cmd_config_set_port),
	SHELL_CMD(buffer_size, NULL, "Set reception buffer size", cmd_config_set_buffer_size),
	SHELL_CMD(sleep,       NULL, "Set sleep between receive", cmd_config_set_sleep),
	SHELL_CMD(tls,         NULL, "Enable/Disable TLS",        cmd_config_set_tls),
	SHELL_CMD(trace,       NULL, "Enable or disable fido trace", cmd_config_set_trace),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_run,
		SHELL_CMD(tcp, NULL, "Run TCP client test", cmd_run_tcp_test),
		SHELL_CMD(http_download, NULL, "Run HTTP download test", cmd_run_http_download_test),
		SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_modem,
		SHELL_CMD(shutdown, NULL, "Modem shutdown", cmd_shutdown_modem),
		SHELL_CMD(init, NULL, "Modem initialize", cmd_init_modem),
		SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_gps,
		SHELL_CMD(start, NULL, "GPS start", cmd_start_gps),
		SHELL_CMD(stop, NULL, "GPS stop", cmd_stop_gps),
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_ip,
		SHELL_CMD(connect, NULL, "Connect socket", cmd_ip_connect),
		SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_modify,
		SHELL_CMD(stop, NULL, "Stop test", cmd_stop_test),
		SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_mqtt,
		SHELL_CMD(connect, NULL, "Connect to MQTT server", cmd_mqtt_connect),
		SHELL_CMD(disconnect, NULL, "Disconnect from MQTT server", cmd_mqtt_disconnect),
		SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(run, &sub_run, "Run download tests", NULL);
SHELL_CMD_REGISTER(modify, &sub_modify, "Modify running test", NULL);
SHELL_CMD_REGISTER(config, &sub_config, "Configure IP client", NULL);
SHELL_CMD_REGISTER(modem, &sub_modem, "Modem control", NULL);
SHELL_CMD_REGISTER(gps, &sub_gps, "GPS control", NULL);
SHELL_CMD_REGISTER(ip, &sub_ip, "IP control",  NULL);
SHELL_CMD_REGISTER(at, NULL, "AT command", cmd_at);
SHELL_CMD_REGISTER(mqtt, &sub_mqtt, "MQTT control", NULL);


