/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <nrf_socket.h>
#include <net/socket.h>
#include <stdio.h>
#include <modem/at_cmd.h>
#include <modem/at_notif.h>

#include "nrf_gnss.h"

#ifdef CONFIG_SUPL_CLIENT_LIB
#include <supl_os_client.h>
#include <supl_session.h>
#include "supl_support.h"
#endif

#define AT_XSYSTEMMODE      "AT\%XSYSTEMMODE=1,0,1,0"
#define AT_ACTIVATE_GPS     "AT+CFUN=31"
#define AT_ACTIVATE_LTE     "AT+CFUN=21"
#define AT_DEACTIVATE_LTE   "AT+CFUN=20"

#define GNSS_INIT_AND_START 1
#define GNSS_STOP           2
#define GNSS_RESTART        3

#define AT_CMD_SIZE(x) (sizeof(x) - 1)

#ifdef CONFIG_BOARD_NRF9160DK_NRF9160NS
#define AT_MAGPIO      "AT\%XMAGPIO=1,0,0,1,1,1574,1577"
#ifdef CONFIG_GPS_SAMPLE_ANTENNA_ONBOARD
#define AT_COEX0       "AT\%XCOEX0=1,1,1565,1586"
#elif CONFIG_GPS_SAMPLE_ANTENNA_EXTERNAL
#define AT_COEX0       "AT\%XCOEX0"
#endif
#endif /* CONFIG_BOARD_NRF9160DK_NRF9160NS */

#ifdef CONFIG_BOARD_THINGY91_NRF9160NS
#define AT_MAGPIO      "AT\%XMAGPIO=1,1,1,7,1,746,803,2,698,748,2,1710,2200," \
			"3,824,894,4,880,960,5,791,849,7,1565,1586"
#ifdef CONFIG_GPS_SAMPLE_ANTENNA_ONBOARD
#define AT_COEX0       "AT\%XCOEX0=1,1,1565,1586"
#elif CONFIG_GPS_SAMPLE_ANTENNA_EXTERNAL
#define AT_COEX0       "AT\%XCOEX0"
#endif
#endif /* CONFIG_BOARD_THINGY91_NRF9160NS */

static const char update_indicator[] = {'\\', '|', '/', '-'};
static const char *const at_commands[] = {
	AT_XSYSTEMMODE,
#if defined(CONFIG_BOARD_NRF9160DK_NRF9160NS) || \
	defined(CONFIG_BOARD_THINGY91_NRF9160NS)
	AT_MAGPIO,
	AT_COEX0,
#endif
	AT_ACTIVATE_GPS
};

static char                  nmea_strings[10][NRF_GNSS_NMEA_MAX_LEN];
static uint32_t              nmea_string_cnt;

static uint64_t                    fix_timestamp;
static nrf_gnss_pvt_data_frame_t  last_pvt;
static nrf_gnss_agps_data_frame_t last_agps;
static bool                        dirty;
static bool                        agps;
static bool                        got_fix;

K_SEM_DEFINE(lte_ready, 0, 1);

void bsd_recoverable_error_handler(uint32_t error)
{
	printf("Err: %lu\n", (unsigned long)error);
}

static int setup_modem(void)
{
	for (int i = 0; i < ARRAY_SIZE(at_commands); i++) {

		if (at_cmd_write(at_commands[i], NULL, 0, NULL) != 0) {
			return -1;
		}
	}

	return 0;
}

#ifdef CONFIG_SUPL_CLIENT_LIB
/* Accepted network statuses read from modem */
static const char status1[] = "+CEREG: 1";
static const char status2[] = "+CEREG:1";
static const char status3[] = "+CEREG: 5";
static const char status4[] = "+CEREG:5";


static void wait_for_lte(void *context, const char *response)
{
	if (!memcmp(status1, response, AT_CMD_SIZE(status1)) ||
		!memcmp(status2, response, AT_CMD_SIZE(status2)) ||
		!memcmp(status3, response, AT_CMD_SIZE(status3)) ||
		!memcmp(status4, response, AT_CMD_SIZE(status4))) {
		k_sem_give(&lte_ready);
	}
}

static int activate_lte(bool activate)
{
	if (activate) {
		if (at_cmd_write(AT_ACTIVATE_LTE, NULL, 0, NULL) != 0) {
			return -1;
		}

		at_notif_register_handler(NULL, wait_for_lte);
		if (at_cmd_write("AT+CEREG=2", NULL, 0, NULL) != 0) {
			return -1;
		}

		k_sem_take(&lte_ready, K_FOREVER);

		at_notif_deregister_handler(NULL, wait_for_lte);
		if (at_cmd_write("AT+CEREG=0", NULL, 0, NULL) != 0) {
			return -1;
		}
	} else {
		if (at_cmd_write(AT_DEACTIVATE_LTE, NULL, 0, NULL) != 0) {
			return -1;
		}
	}

	return 0;
}
#endif

void modem_trace_enable(void)
{
#define CS_PIN_CFG_TRACE_CLK    21 //GPIO_OUT_PIN21_Pos
#define CS_PIN_CFG_TRACE_DATA0  22 //GPIO_OUT_PIN22_Pos
#define CS_PIN_CFG_TRACE_DATA1  23 //GPIO_OUT_PIN23_Pos
#define CS_PIN_CFG_TRACE_DATA2  24 //GPIO_OUT_PIN24_Pos
#define CS_PIN_CFG_TRACE_DATA3  25 //GPIO_OUT_PIN25_Pos

	NRF_P0_NS->PIN_CNF[CS_PIN_CFG_TRACE_CLK] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
	                                           (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

	NRF_P0_NS->PIN_CNF[CS_PIN_CFG_TRACE_DATA0] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
	                                             (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

	NRF_P0_NS->PIN_CNF[CS_PIN_CFG_TRACE_DATA1] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
	                                             (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

	NRF_P0_NS->PIN_CNF[CS_PIN_CFG_TRACE_DATA2] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
	                                             (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

	NRF_P0_NS->PIN_CNF[CS_PIN_CFG_TRACE_DATA3] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
	                                             (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

	NRF_P0_NS->DIR = 0xFFFFFFFF;
}

void gnss_event_handler(int event)
{
	switch (event) {
		case NRF_GNSS_PVT_NTF:
			nrf_gnss_read(&last_pvt, sizeof(last_pvt), event);
			nmea_string_cnt = 0;
			dirty = false;
			break;

		case NRF_GNSS_FIX_NTF:
			got_fix = true;
			nmea_string_cnt = 0;
			fix_timestamp = k_uptime_get();
			break;

		case NRF_GNSS_NMEA_NTF:
			nrf_gnss_read(&nmea_strings[nmea_string_cnt++], sizeof(nmea_strings[nmea_string_cnt]), event);
			break;

		case NRF_GNSS_AGPS_NTF:
#ifdef CONFIG_SUPL_CLIENT_LIB
			nrf_gnss_read(&last_agps, sizeof(last_agps), event);
			agps = true;
#endif
			break;
		case NRF_GNSS_BLOCKED_NTF:
			break;

		case NRF_GNSS_UNBLOCKED_NTF:
			break;

		default:
			break;
	}
}

static int gnss_ctrl(uint32_t ctrl)
{
	struct nrf_gnss_config config = {
			.elevation_mask = -1,
			.use_case       = -1,
			.system         = -1,
			.fix_interval   = 1,
			.fix_retry      = 0,
			.nmea_mask      = NRF_GNSS_NMEA_GSV_MASK | NRF_GNSS_NMEA_GSA_MASK | NRF_GNSS_NMEA_GLL_MASK | NRF_GNSS_NMEA_GGA_MASK | NRF_GNSS_NMEA_RMC_MASK,
			.power_mode     = -1,
	};

	if (ctrl == GNSS_INIT_AND_START) {
		int retval = nrf_gnss_set_configuration(&config);
		retval |= nrf_gnss_set_handler(gnss_event_handler);

		if(retval != 0) {
			printk("Failed to initalize GNSS interface");
			return -1;
		}
	}

	if ((ctrl == GNSS_INIT_AND_START) ||
	    (ctrl == GNSS_RESTART)) {
		if (nrf_gnss_send_cmd(NRF_GNSS_CMD_START, 0) != 0) {
			printk("Failed to start GPS\n");
			return -1;
		}
	}

	if (ctrl == GNSS_STOP) {
		if (nrf_gnss_send_cmd(NRF_GNSS_CMD_STOP, 0) != 0) {
			printk("Failed to stop GPS\n");
			return -1;
		}
	}

	return 0;
}

static int init_app(void)
{
	int retval;

	if (setup_modem() != 0) {
		printk("Failed to initialize modem\n");
		return -1;
	}

	retval = gnss_ctrl(GNSS_INIT_AND_START);

	return retval;
}

static void print_satellite_stats(nrf_gnss_pvt_data_frame_t *pvt_data)
{
	uint8_t  tracked          = 0;
	uint8_t  in_fix           = 0;
	uint8_t  unhealthy        = 0;

	for (int i = 0; i < NRF_GNSS_MAX_SATELLITES; ++i) {

		if ((pvt_data->sv[i].sv > 0) &&
		    (pvt_data->sv[i].sv < 33)) {

			tracked++;

			if (pvt_data->sv[i].flags &
					NRF_GNSS_SV_FLAG_USED_IN_FIX) {
				in_fix++;
			}

			if (pvt_data->sv[i].flags &
					NRF_GNSS_SV_FLAG_UNHEALTHY) {
				unhealthy++;
			}
		}
	}

	printk("Tracking: %d Using: %d Unhealthy: %d\n", tracked,
							 in_fix,
							 unhealthy);
}

static void print_gnss_stats(nrf_gnss_pvt_data_frame_t *pvt_data)
{
	if (pvt_data->flags & NRF_GNSS_PVT_FLAG_DEADLINE_MISSED) {
		printk("GNSS notification deadline missed\n");
	}
	if (pvt_data->flags & NRF_GNSS_PVT_FLAG_NOT_ENOUGH_WINDOW_TIME) {
		printk("GNSS operation blocked by insufficient time windows\n");
	}
}

static void print_fix_data(nrf_gnss_pvt_data_frame_t *pvt_data)
{
	printf("Longitude:  %f\n", pvt_data->longitude);
	printf("Latitude:   %f\n", pvt_data->latitude);
	printf("Altitude:   %f\n", pvt_data->altitude);
	printf("Speed:      %f\n", pvt_data->speed);
	printf("Heading:    %f\n", pvt_data->heading);
	printk("Date:       %02u-%02u-%02u\n", pvt_data->datetime.year,
					       pvt_data->datetime.month,
					       pvt_data->datetime.day);
	printk("Time (UTC): %02u:%02u:%02u\n", pvt_data->datetime.hour,
					       pvt_data->datetime.minute,
					      pvt_data->datetime.seconds);
}

static void print_nmea_data(void)
{
	for (int i = 0; i < nmea_string_cnt; ++i) {
		printk("%s", nmea_strings[i]);
	}
}

#ifdef CONFIG_SUPL_CLIENT_LIB
int inject_agps_type(void *agps,
		     size_t agps_size,
		     nrf_gnss_agps_data_type_t type,
		     void *user_data)
{
	ARG_UNUSED(user_data);
	int retval = nrf_gnss_write(agps, agps_size, type);

	if (retval != 0) {
		printk("Failed to send AGNSS data, type: %d (err: %d)\n",
		       type,
		       errno);
		return -1;
	}

	printk("Injected AGPS data, flags: %d, size: %d\n", type, agps_size);

	return 0;
}
#endif


int main(void)
{
	uint8_t		      cnt = 0;

#ifdef CONFIG_SUPL_CLIENT_LIB
	static struct supl_api supl_api = {
		.read       = supl_read,
		.write      = supl_write,
		.handler    = inject_agps_type,
		.logger     = supl_logger,
		.counter_ms = k_uptime_get
	};
#endif
	modem_trace_enable();

	if (at_cmd_write("AT%XMODEMTRACE=2,,3,\"ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\"", NULL, 0, NULL) == 0)
	{
		printk("Trace enabled\n");
	} else {
		printk("Failed to enable trace\n");
	}

	printk("Starting GPS application\n");

	if (init_app() != 0) {
		return -1;
	}


#ifdef CONFIG_SUPL_CLIENT_LIB
	int rc = supl_init(&supl_api);

	if (rc != 0) {
		return rc;
	}
#endif

	printk("Getting GPS data...\n");

	while (1) {

		if (IS_ENABLED(CONFIG_GPS_SAMPLE_NMEA_ONLY)) {
			print_nmea_data();
			nmea_string_cnt = 0;
		} else {
			if (!dirty) {
				printk("\033[1;1H");
				printk("\033[2J");
				print_satellite_stats(&last_pvt);
				print_gnss_stats(&last_pvt);
				printk("---------------------------------\n");

				if (!got_fix) {
					printk("Seconds since last fix: %lld\n",
							(k_uptime_get() - fix_timestamp) / 1000);
					cnt++;
					printk("Searching [%c]\n",
							update_indicator[cnt%4]);
				} else {
					print_fix_data(&last_pvt);
				}

				printk("\nNMEA strings:\n\n");
				print_nmea_data();
				printk("---------------------------------");
				dirty = true;
				got_fix = false;
			} else if (agps) {

#ifdef CONFIG_SUPL_CLIENT_LIB
				printk("\033[1;1H");
				printk("\033[2J");
				printk("New AGPS data requested, contacting SUPL server, flags %d\n",
						last_agps.data_flags);
				gnss_ctrl(GNSS_STOP);
				activate_lte(true);
				printk("Established LTE link\n");
				if (open_supl_socket() == 0) {
					printf("Starting SUPL session\n");
					supl_session(&last_agps);
					printk("Done\n");
					close_supl_socket();
				}
				activate_lte(false);
				gnss_ctrl(GNSS_RESTART);
				k_sleep(K_MSEC(2000));

				agps = false;
#endif
			}
		}

		k_sleep(K_MSEC(1000));
	}

	return 0;
}
