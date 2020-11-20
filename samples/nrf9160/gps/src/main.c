/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr.h>
#include <nrf_socket.h>
#include <net/socket.h>
#include <stdio.h>
#include <modem/at_cmd.h>
#include <modem/at_notif.h>

#include "nrf_modem_gnss.h"

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

static struct nrf_modem_gnss_pvt_data_frame  last_pvt;
#ifdef CONFIG_SUPL_CLIENT_LIB
static struct nrf_modem_gnss_agps_data_frame last_agps;
#endif
K_MSGQ_DEFINE(nmea_queue, sizeof(struct nrf_modem_gnss_nmea_data_frame *), 10, 4);

static uint64_t fix_timestamp;

static bool new_data;
static bool agps_requested;
static bool gps_blocked;
static bool got_fix;

K_SEM_DEFINE(lte_ready, 0, 1);

void nrf_modem_recoverable_error_handler(uint32_t error)
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

void gnss_event_handler(int event)
{
	int retval;

	switch (event) {
		case NRF_MODEM_GNSS_PVT_NTF:
			nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT);
			new_data = true;
			break;

		case NRF_MODEM_GNSS_FIX_NTF:
			got_fix = true;
			fix_timestamp = k_uptime_get();
			break;

		case NRF_MODEM_GNSS_NMEA_NTF:
		{
			struct nrf_modem_gnss_nmea_data_frame *data =
					k_malloc(sizeof(struct nrf_modem_gnss_nmea_data_frame));

			retval = nrf_modem_gnss_read(data,
						     sizeof(struct nrf_modem_gnss_nmea_data_frame),
						     NRF_MODEM_GNSS_DATA_NMEA);
			if (retval == 0) {
				retval = k_msgq_put(&nmea_queue, &data, K_NO_WAIT);
			}

			if (retval != 0) {
				k_free(data);
			}
			break;
		}

		case NRF_MODEM_GNSS_AGPS_REQ_NTF:
#ifdef CONFIG_SUPL_CLIENT_LIB
			retval = nrf_modem_gnss_read(&last_agps,
						     sizeof(last_agps),
						     NRF_MODEM_GNSS_DATA_AGPS_REQ);
			if (retval == 0) {
				agps_requested = true;
			}
#endif
			break;
		case NRF_MODEM_GNSS_BLOCKED_NTF:
			gps_blocked = true;
			break;

		case NRF_MODEM_GNSS_UNBLOCKED_NTF:
			gps_blocked = false;
			break;

		default:
			break;
	}
}

static int gnss_ctrl(uint32_t ctrl)
{
	int retval;

	if (ctrl == GNSS_INIT_AND_START) {
		retval = nrf_modem_gnss_nmea_mask_set(NRF_MODEM_GNSS_NMEA_RMC_MASK |
						      NRF_MODEM_GNSS_NMEA_GGA_MASK |
						      NRF_MODEM_GNSS_NMEA_GLL_MASK |
						      NRF_MODEM_GNSS_NMEA_GSA_MASK |
						      NRF_MODEM_GNSS_NMEA_GSV_MASK);
		retval |= nrf_modem_gnss_fix_retry_set(0);
		retval |= nrf_modem_gnss_fix_interval_set(1);
		retval |= nrf_modem_gnss_handler_set(gnss_event_handler);

		if(retval != 0) {
			printk("Failed to initalize GNSS interface");
			return -1;
		}
	}

	if ((ctrl == GNSS_INIT_AND_START) ||
	    (ctrl == GNSS_RESTART)) {
		if (nrf_modem_gnss_cmd_send(NRF_MODEM_GNSS_CMD_START, 0) != 0) {
			printk("Failed to start GPS\n");
			return -1;
		}
	}

	if (ctrl == GNSS_STOP) {
		if (nrf_modem_gnss_cmd_send(NRF_MODEM_GNSS_CMD_STOP, 0) != 0) {
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

	nrf_modem_gnss_init();
	retval = gnss_ctrl(GNSS_INIT_AND_START);

	return retval;
}

static void print_satellite_stats(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	uint8_t  tracked          = 0;
	uint8_t  in_fix           = 0;
	uint8_t  unhealthy        = 0;

	for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; ++i) {

		if ((pvt_data->sv[i].sv > 0) &&
		    (pvt_data->sv[i].sv < 33)) {

			tracked++;

			if (pvt_data->sv[i].flags &
					NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX) {
				in_fix++;
			}

			if (pvt_data->sv[i].flags &
					NRF_MODEM_GNSS_SV_FLAG_UNHEALTHY) {
				unhealthy++;
			}
		}
	}

	printk("Tracking: %d Using: %d Unhealthy: %d\n", tracked,
							 in_fix,
							 unhealthy);
}

static void print_gnss_stats(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	if (gps_blocked) {
		printk("GPS is blocked from creating a fix\n");
	}
}

static void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
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
	struct nrf_modem_gnss_nmea_data_frame *data = NULL;

	if (k_msgq_num_used_get(&nmea_queue) > 0) {
		printk("\nNMEA strings:\n\n");

		while (k_msgq_get(&nmea_queue, &data, K_NO_WAIT) == 0) {
			printk("%s", data->nmea_str);
			k_free(data);
		}
	}
}

#ifdef CONFIG_SUPL_CLIENT_LIB
int inject_agps_type(void *agps,
		     size_t agps_size,
		     uint16_t type,
		     void *user_data)
{
	ARG_UNUSED(user_data);
	int retval = nrf_modem_gnss_agps_write(agps, agps_size, type);

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
		} else {

			if (new_data) {
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

				print_nmea_data();

				printk("---------------------------------");
				new_data = false;
				got_fix = false;
			}

			if (agps_requested) {
				printk("\033[1;1H");
				printk("\033[2J");
#ifdef CONFIG_SUPL_CLIENT_LIB
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

				agps_requested = false;
#endif
			}
		}

		k_sleep(K_MSEC(1000));
	}

	return 0;
}
