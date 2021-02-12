/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/gps.h>
#include <init.h>
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <logging/log.h>
#include <nrf_gnss.h>
#include <nrf_socket.h>
#include <modem/at_cmd.h>
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>

LOG_MODULE_REGISTER(nrf9160_gps, CONFIG_NRF9160_GPS_LOG_LEVEL);

#define NMEA_STRS_MAX 10 /* Max number of stored NMEA strings */

struct gps_drv_data {
	const struct device *dev;
	gps_event_handler_t handler;
	struct gps_config current_cfg;
	atomic_t is_init;
	atomic_t is_active;
	atomic_t is_shutdown;
	atomic_t timeout_occurred;
	atomic_t has_fix;
	int64_t  fix_timestamp;
	int32_t  fix_interval;
	atomic_t got_evt;
	char     nmea_strs[NMEA_STRS_MAX][NRF_GNSS_NMEA_MAX_LEN];
	atomic_t nmea_nxt;
	atomic_t nmea_fix_idx;
	atomic_t blocked;
	nrf_gnss_pvt_data_frame_t  last_pvt;
#ifdef CONFIG_NRF9160_AGPS
	nrf_gnss_agps_data_frame_t last_agps;
#endif

	K_THREAD_STACK_MEMBER(thread_stack,
			      CONFIG_NRF9160_GPS_THREAD_STACK_SIZE);
	struct k_thread thread;
	k_tid_t thread_id;
	struct k_sem thread_run_sem;
	struct k_sem gps_evt_sem;
	struct k_delayed_work stop_work;
	struct k_delayed_work timeout_work;
	struct k_delayed_work pvt_ntf_work;
	struct k_delayed_work sat_stats_work;
	struct k_delayed_work blocked_ntf_work;
	struct k_delayed_work unblocked_ntf_work;
	struct k_delayed_work nmea_ntf_work;
#ifdef CONFIG_NRF9160_AGPS
	struct k_delayed_work agps_ntf_work;
#endif
};

struct nrf9160_gps_config {
	nrf_gnss_fix_retry_t retry;
	nrf_gnss_fix_interval_t interval;
	nrf_gnss_nmea_mask_t nmea_mask;
	nrf_gnss_delete_mask_t delete_mask;
	nrf_gnss_power_save_mode_t power_mode;
	nrf_gnss_use_case_t use_case;
	bool priority;
};

/* To keep the GNSS event handler and the GPS thread in sync */
static struct device *dev_hook;

static void copy_pvt(struct gps_pvt *dest, nrf_gnss_pvt_data_frame_t *src)
{
	dest->latitude = src->latitude;
	dest->longitude = src->longitude;
	dest->altitude = src->altitude;
	dest->accuracy = src->accuracy;
	dest->speed = src->speed;
	dest->heading = src->heading;
	dest->datetime.year = src->datetime.year;
	dest->datetime.month = src->datetime.month;
	dest->datetime.day = src->datetime.day;
	dest->datetime.hour = src->datetime.hour;
	dest->datetime.minute = src->datetime.minute;
	dest->datetime.seconds = src->datetime.seconds;
	dest->datetime.ms = src->datetime.ms;
	dest->pdop = src->pdop;
	dest->hdop = src->hdop;
	dest->vdop = src->vdop;
	dest->tdop = src->tdop;

	for (size_t i = 0;
	     i < MIN(NRF_GNSS_MAX_SATELLITES, GPS_PVT_MAX_SV_COUNT); i++) {
		dest->sv[i].sv = src->sv[i].sv;
		dest->sv[i].cn0 = src->sv[i].cn0;
		dest->sv[i].elevation = src->sv[i].elevation;
		dest->sv[i].azimuth = src->sv[i].azimuth;
		dest->sv[i].signal = src->sv[i].signal;
		dest->sv[i].in_fix =
			(src->sv[i].flags & NRF_GNSS_SV_FLAG_USED_IN_FIX) ==
			NRF_GNSS_SV_FLAG_USED_IN_FIX;
		dest->sv[i].unhealthy =
			(src->sv[i].flags & NRF_GNSS_SV_FLAG_UNHEALTHY) ==
			NRF_GNSS_SV_FLAG_UNHEALTHY;
	}
}

static bool is_fix(nrf_gnss_pvt_data_frame_t *pvt)
{
	return ((pvt->flags & NRF_GNSS_PVT_FLAG_FIX_VALID_BIT)
		== NRF_GNSS_PVT_FLAG_FIX_VALID_BIT);
}

static void tracking_stats(nrf_gnss_pvt_data_frame_t *pvt_data,
				  int64_t fix_timestamp)
{
	uint8_t  n_tracked = 0;
	uint8_t  n_used = 0;
	uint8_t  n_unhealthy = 0;

	for (int i = 0; i < NRF_GNSS_MAX_SATELLITES; ++i) {
		uint8_t sv = pvt_data->sv[i].sv;
		bool used = (pvt_data->sv[i].flags &
			     NRF_GNSS_SV_FLAG_USED_IN_FIX) ? true : false;
		bool unhealthy = (pvt_data->sv[i].flags &
				  NRF_GNSS_SV_FLAG_UNHEALTHY) ? true : false;

		if (sv) { /* SV number 0 indicates no satellite */
			n_tracked++;
			if (used) {
				n_used++;
			}
			if (unhealthy) {
				n_unhealthy++;
			}
		}
	}

	int64_t present = k_uptime_get();

	if (fix_timestamp < present) {

		LOG_DBG("%2u SVs:  %2u used, %2u unhealthy, %lld %s",
			n_tracked, n_used, n_unhealthy,
			((present - fix_timestamp) / MSEC_PER_SEC),
			"secs since last fix");
	} else {

		LOG_DBG("%2u SVs:  %2u used, %2u unhealthy, %lld %s",
			n_tracked, n_used, n_unhealthy,
			(present / MSEC_PER_SEC),
			"secs and no fix");
	}
}

static void notify_event(const struct device *dev, struct gps_event *evt)
{
	struct gps_drv_data *drv_data = dev->data;

	if (drv_data->handler) {
		drv_data->handler(dev, evt);
	}
}

static void cancel_works(struct gps_drv_data *drv_data)
{
	k_delayed_work_cancel(&drv_data->timeout_work);
	k_delayed_work_cancel(&drv_data->pvt_ntf_work);
	k_delayed_work_cancel(&drv_data->sat_stats_work);
	k_delayed_work_cancel(&drv_data->blocked_ntf_work);
	k_delayed_work_cancel(&drv_data->unblocked_ntf_work);
	k_delayed_work_cancel(&drv_data->nmea_ntf_work);
#ifdef CONFIG_NRF9160_AGPS
	k_delayed_work_cancel(&drv_data->agps_ntf_work);
#endif
}

static void nrf91_gnss_event_handler(int event)
{
	struct gps_drv_data *drv_data = dev_hook->data;
	atomic_t nxt;
	atomic_t idx;

	switch (event) {
	case NRF_GNSS_PVT_NTF:
		atomic_clear(&drv_data->has_fix);

		if (atomic_get(&drv_data->blocked)) {
			/* Avoid spamming the logs and app. */
			goto out;
		}

		nrf_gnss_read(&drv_data->last_pvt, sizeof(drv_data->last_pvt),
			      event);

		k_delayed_work_submit(&drv_data->pvt_ntf_work, K_NO_WAIT);
		break;

	case NRF_GNSS_FIX_NTF:
		atomic_set(&drv_data->has_fix, true);
		drv_data->fix_timestamp = k_uptime_get();
		break;

	case NRF_GNSS_NMEA_NTF:
		/* Might get overwritten but will not overflow */
		idx = atomic_inc(&drv_data->nmea_nxt) % NMEA_STRS_MAX;
		nxt = atomic_get(&drv_data->nmea_nxt) % NMEA_STRS_MAX;
		atomic_set(&drv_data->nmea_nxt, nxt);

		nrf_gnss_read(drv_data->nmea_strs[idx],
			      GPS_NMEA_SENTENCE_MAX_LENGTH,
			      event);

		drv_data->nmea_fix_idx = idx;
		k_delayed_work_submit(&drv_data->nmea_ntf_work, K_NO_WAIT);
		break;

	case NRF_GNSS_AGPS_NTF:
#ifdef CONFIG_NRF9160_AGPS
		nrf_gnss_read(&drv_data->last_agps,
				sizeof(drv_data->last_agps),
				event);

		k_delayed_work_submit(&drv_data->agps_ntf_work, K_NO_WAIT);
#endif /* CONFIG_NRF9160_AGPS */
		break;
	case NRF_GNSS_BLOCKED_NTF:
		atomic_set(&drv_data->blocked, true);
		k_delayed_work_cancel(&drv_data->timeout_work);
		k_delayed_work_submit(&drv_data->blocked_ntf_work, K_NO_WAIT);
		break;
	case NRF_GNSS_UNBLOCKED_NTF:
		atomic_clear(&drv_data->blocked);
		k_delayed_work_submit(&drv_data->unblocked_ntf_work, K_NO_WAIT);
		break;
	default:
		break;
	}

out:
	atomic_set(&drv_data->got_evt, true);
	k_sem_give(&drv_data->gps_evt_sem);
}

static void nrf91_gnss_thread(int dev_ptr)
{
	struct device *dev = INT_TO_POINTER(dev_ptr);
	struct gps_drv_data *drv_data = dev->data;
	struct gps_event evt = {
		.type = GPS_EVT_SEARCH_STARTED
	};
	atomic_clear(&drv_data->has_fix);
	uint32_t timeout;

wait:
	k_sem_take(&drv_data->thread_run_sem, K_FOREVER);

	evt.type = GPS_EVT_SEARCH_STARTED;
	notify_event(dev, &evt);

	while (true) {
		timeout = drv_data->fix_interval ? drv_data->fix_interval : 10;

		/** There is no way of knowing if an event has not been received
		 *  or the GPS has gotten a fix. This check makes sure that
		 *  a GPS_EVT_SEARCH_TIMEOUT is not propagated upon a fix.
		 */
		if (!atomic_get(&drv_data->has_fix)) {
			k_delayed_work_submit(&drv_data->timeout_work,
					      K_SECONDS(timeout));
		}

		k_sem_take(&drv_data->gps_evt_sem, K_SECONDS(timeout+1));

		/* No need for timeout if an event occurred early enough */
		k_delayed_work_cancel(&drv_data->timeout_work);

		if (atomic_clear(&drv_data->got_evt)) {
			continue;
		}

		/* Is the GPS stopped, causing timeout? */
		if (!atomic_get(&drv_data->is_active)) {
			goto wait;
		}

		/* If blocked lets wait at least for one fix interval */
		if (!atomic_get(&drv_data->blocked)) {
			k_sleep(K_SECONDS(drv_data->fix_interval));
		}
	}
}

static int init_thread(const struct device *dev)
{
	struct gps_drv_data *drv_data = dev->data;

	drv_data->thread_id = k_thread_create(
			&drv_data->thread, drv_data->thread_stack,
			K_THREAD_STACK_SIZEOF(drv_data->thread_stack),
			(k_thread_entry_t)nrf91_gnss_thread, (void *)dev,
			NULL, NULL,
			K_PRIO_PREEMPT(CONFIG_NRF9160_GPS_THREAD_PRIORITY),
			0, K_NO_WAIT);

	return 0;
}

#ifdef CONFIG_NRF9160_GPS_HANDLE_MODEM_CONFIGURATION
static int enable_gps(const struct device *dev)
{
	int err;
	enum lte_lc_system_mode system_mode;
	enum lte_lc_func_mode functional_mode;

	err = lte_lc_system_mode_get(&system_mode);
	if (err) {
		LOG_ERR("Could not get modem system mode, error: %d", err);
		return err;
	}

	if ((system_mode != LTE_LC_SYSTEM_MODE_GPS) &&
	    (system_mode != LTE_LC_SYSTEM_MODE_LTEM_GPS) &&
	    (system_mode != LTE_LC_SYSTEM_MODE_NBIOT_GPS)) {
		enum lte_lc_system_mode new_mode = LTE_LC_SYSTEM_MODE_GPS;

		if (system_mode == LTE_LC_SYSTEM_MODE_LTEM) {
			new_mode = LTE_LC_SYSTEM_MODE_LTEM_GPS;
		} else if (system_mode == LTE_LC_SYSTEM_MODE_NBIOT) {
			new_mode = LTE_LC_SYSTEM_MODE_NBIOT_GPS;
		}

		LOG_DBG("GPS mode is not enabled, attempting to enable it");

		err = lte_lc_system_mode_set(new_mode);
		if (err) {
			LOG_ERR("Could not enable GPS mode, error: %d", err);
			return err;
		}
	}

	LOG_DBG("GPS mode is enabled");

	err = lte_lc_func_mode_get(&functional_mode);
	if (err) {
		LOG_ERR("Could not get modem's functional mode, error: %d",
			err);
		return err;
	}

	if (functional_mode != LTE_LC_FUNC_MODE_NORMAL) {
		LOG_ERR("GPS is not supported in current functional mode");
		return -EIO;
	}

	return err;
}
#endif

static void set_nmea_mask(nrf_gnss_nmea_mask_t *nmea_mask)
{
	*nmea_mask = 0;

#ifdef CONFIG_NRF9160_GPS_NMEA_GSV
	*nmea_mask |= NRF_GNSS_NMEA_GSV_MASK;
#endif
#ifdef CONFIG_NRF9160_GPS_NMEA_GSA
	*nmea_mask |= NRF_GNSS_NMEA_GSA_MASK;
#endif
#ifdef CONFIG_NRF9160_GPS_NMEA_GLL
	*nmea_mask |= NRF_GNSS_NMEA_GLL_MASK;
#endif
#ifdef CONFIG_NRF9160_GPS_NMEA_GGA
	*nmea_mask |= NRF_GNSS_NMEA_GGA_MASK;
#endif
#ifdef CONFIG_NRF9160_GPS_NMEA_RMC
	*nmea_mask |= NRF_GNSS_NMEA_RMC_MASK;
#endif
}

static int parse_cfg(struct gps_config *cfg_src,
		     struct nrf9160_gps_config *cfg_dst)
{
	switch (cfg_src->nav_mode) {
	case GPS_NAV_MODE_SINGLE_FIX:
		cfg_dst->interval = 0;
		cfg_dst->retry = cfg_src->timeout < 0 ? 0 : cfg_src->timeout;
		break;
	case GPS_NAV_MODE_CONTINUOUS:
		cfg_dst->retry = 0;
		cfg_dst->interval = 1;
		break;
	case GPS_NAV_MODE_PERIODIC:
		if (cfg_src->interval < 10) {
			LOG_ERR("Minimum periodic interval is 10 sec");
			return -EINVAL;
		}

		cfg_dst->retry = cfg_src->timeout;
		cfg_dst->interval = cfg_src->interval;
		break;
	default:
		LOG_ERR("Invalid operation mode (%d), GPS will not start",
			cfg_src->nav_mode);
		return -EINVAL;
	}

	if (cfg_src->delete_agps_data) {
		cfg_dst->delete_mask = 0x7F;
	}

	set_nmea_mask(&cfg_dst->nmea_mask);

	if (cfg_src->power_mode == GPS_POWER_MODE_PERFORMANCE) {
		cfg_dst->power_mode = NRF_GNSS_PSM_DUTY_CYCLING_PERFORMANCE;
	} else if (cfg_src->power_mode == GPS_POWER_MODE_SAVE) {
		cfg_dst->power_mode = NRF_GNSS_PSM_DUTY_CYCLING_POWER;
	}

	cfg_dst->priority = cfg_src->priority;

	if (cfg_src->use_case == GPS_USE_CASE_SINGLE_COLD_START) {
		cfg_dst->use_case = NRF_GNSS_USE_CASE_SINGLE_COLD_START;
	} else if (cfg_src->use_case == GPS_USE_CASE_MULTIPLE_HOT_START) {
		cfg_dst->use_case = NRF_GNSS_USE_CASE_MULTIPLE_HOT_START;
	}

	if (cfg_src->accuracy == GPS_ACCURACY_NORMAL) {
		cfg_dst->use_case |= NRF_GNSS_USE_CASE_NORMAL_ACCURACY;
	} else if (cfg_src->accuracy == GPS_ACCURACY_LOW) {
		cfg_dst->use_case |= NRF_GNSS_USE_CASE_LOW_ACCURACY;
	}

	return 0;
}

static int nrf91_gnss_ctrl(uint32_t ctrl, struct nrf_gnss_config *config)
{
	if (ctrl == NRF_GNSS_CMD_START) {
		int err = nrf_gnss_set_configuration(config);

		err |= nrf_gnss_set_handler(nrf91_gnss_event_handler);

		if (err) {
			LOG_ERR("Failed to initialize GNSS interface");
			return -1;
		}

		if (nrf_gnss_send_cmd(ctrl, 0) != 0) {
			LOG_ERR("Failed to start GPS\n");
			return -1;
		}
	}

	if (ctrl == NRF_GNSS_CMD_STOP) {
		if (nrf_gnss_send_cmd(ctrl, 0) != 0) {
			LOG_ERR("Failed to stop GPS\n");
			return -1;
		}
	}

	return 0;
}

static int start(const struct device *dev, struct gps_config *cfg)
{
	int err;
	struct gps_drv_data *drv_data = dev->data;
	struct nrf9160_gps_config gps_cfg = { 0 };

	if (atomic_get(&drv_data->is_shutdown) == 1) {
		return -EHOSTDOWN;
	}

	if (atomic_get(&drv_data->is_active)) {
		LOG_DBG("GPS is already active. Clean up before restart");
		cancel_works(drv_data);
	}

	if (atomic_get(&drv_data->is_init) != 1) {
		LOG_WRN("GPS must be initialized first");
		return -ENODEV;
	}

	err = parse_cfg(cfg, &gps_cfg);
	if (err) {
		LOG_ERR("Invalid GPS configuration");
		return err;
	}

	/* Don't copy config if it points to the internal one */
	if (cfg != &drv_data->current_cfg) {
		memcpy(&drv_data->current_cfg, cfg,
		       sizeof(struct gps_config));
	}

#ifdef CONFIG_NRF9160_GPS_HANDLE_MODEM_CONFIGURATION
	if (enable_gps(dev) != 0) {
		LOG_ERR("Failed to enable GPS");
		return -EIO;
	}
#endif
	/* Needs to be set before the event handler is registered */
	dev_hook = (struct device *)dev;

	struct nrf_gnss_config gnss_cfg = {
			.elevation_mask = -1,
			.use_case       = -1,
			.system         = -1,
			.fix_interval   = (int32_t)gps_cfg.interval,
			.fix_retry      = (int32_t)gps_cfg.retry,
			.nmea_mask      = (int32_t)gps_cfg.nmea_mask,
			.power_mode     = (int16_t)gps_cfg.power_mode,
	};

	err = nrf91_gnss_ctrl(NRF_GNSS_CMD_START, &gnss_cfg);
	if (err) {
		LOG_ERR("Failed to initialize GNSS interface");
		return -1;
	}


	atomic_set(&drv_data->is_active, 1);
	atomic_set(&drv_data->fix_interval, gnss_cfg.fix_interval);
	atomic_set(&drv_data->timeout_occurred, 0);
	k_sem_give(&drv_data->thread_run_sem);

	LOG_DBG("GPS operational");

	return 0;
}

static int setup(const struct device *dev)
{
	struct gps_drv_data *drv_data = dev->data;

	drv_data->dev = dev;

	atomic_set(&drv_data->is_active, 0);
	atomic_set(&drv_data->timeout_occurred, 0);
	drv_data->fix_timestamp = INT64_MAX; /* In the future -> no fix yet */
	atomic_clear(&drv_data->got_evt);
	atomic_clear(&drv_data->has_fix);
	atomic_clear(&drv_data->nmea_nxt);
	atomic_clear(&drv_data->nmea_fix_idx);
	atomic_set(&drv_data->blocked, false);
	memset(&drv_data->last_pvt, 0, sizeof(nrf_gnss_pvt_data_frame_t));

	return 0;
}

static int configure_antenna(void)
{
	int err = 0;

#if CONFIG_NRF9160_GPS_SET_MAGPIO
	err = at_cmd_write(CONFIG_NRF9160_GPS_MAGPIO_STRING,
				NULL, 0, NULL);
	if (err) {
		LOG_ERR("Could not configure MAGPIO, error: %d", err);
		return err;
	}

	LOG_DBG("MAGPIO set: %s",
		log_strdup(CONFIG_NRF9160_GPS_MAGPIO_STRING));
#endif /* CONFIG_NRF9160_GPS_SET_MAGPIO */

#if CONFIG_NRF9160_GPS_SET_COEX0
	err = at_cmd_write(CONFIG_NRF9160_GPS_COEX0_STRING,
				NULL, 0, NULL);
	if (err) {
		LOG_ERR("Could not configure COEX0, error: %d", err);
		return err;
	}

	LOG_DBG("COEX0 set: %s",
		log_strdup(CONFIG_NRF9160_GPS_COEX0_STRING));
#endif /* CONFIG_NRF9160_GPS_SET_COEX0 */

	return err;
}

static int stop(const struct device *dev)
{
	int err = 0;
	struct gps_drv_data *drv_data = dev->data;

	k_sem_reset(&drv_data->thread_run_sem);

	if (atomic_get(&drv_data->is_shutdown)) {
		return -EHOSTDOWN;
	}

	cancel_works(drv_data);

	if (!atomic_get(&drv_data->is_active)) {
		/* The GPS is already stopped, attempting to stop it again would
		 * result in an error. Instead, notify that it's stopped.
		 */
		goto notify;
	}

	atomic_clear(&drv_data->is_active);

	err = nrf91_gnss_ctrl(NRF_GNSS_CMD_STOP, NULL);
	if (err) {
		LOG_ERR("Failed to stop GPS");
		return -EIO;
	}

notify:
	/* Offloading event dispatching to workqueue, as also other events
	 * in this driver are sent from context different than the calling
	 * context.
	 */
	k_delayed_work_submit(&drv_data->stop_work, K_NO_WAIT);

	return 0;
}

static void stop_work_fn(struct k_work *work)
{
	struct gps_drv_data *drv_data =
		CONTAINER_OF(work, struct gps_drv_data, stop_work);
	const struct device *dev = drv_data->dev;
	struct gps_event evt = {
		.type = GPS_EVT_SEARCH_STOPPED
	};

	notify_event(dev, &evt);
}

static void pvt_ntf_work_fn(struct k_work *work)
{
	struct gps_drv_data *drv_data =
		CONTAINER_OF(work, struct gps_drv_data, pvt_ntf_work);
	const struct device *dev = drv_data->dev;
	struct gps_event evt = {};

	copy_pvt(&evt.pvt, &drv_data->last_pvt);

	if (is_fix(&drv_data->last_pvt)) {
		evt.type = GPS_EVT_PVT_FIX;
		drv_data->fix_timestamp = k_uptime_get();
		atomic_set(&drv_data->has_fix, true);
	} else {
		evt.type = GPS_EVT_PVT;
	}

	notify_event(dev, &evt);

	if (!k_delayed_work_remaining_get(&drv_data->sat_stats_work)) {
		k_delayed_work_submit(&drv_data->sat_stats_work, K_SECONDS(1));
	}
}

#ifdef CONFIG_NRF9160_AGPS
static void agps_ntf_work_fn(struct k_work *work)
{
	struct gps_drv_data *drv_data =
			CONTAINER_OF(work, struct gps_drv_data, agps_ntf_work);
	const struct device *dev = drv_data->dev;
	struct gps_event evt = {};

	evt.type = GPS_EVT_AGPS_DATA_NEEDED;
	evt.agps_request.sv_mask_ephe = drv_data->last_agps.sv_mask_ephe;
	evt.agps_request.sv_mask_alm = drv_data->last_agps.sv_mask_alm;
	evt.agps_request.utc = drv_data->last_agps.data_flags &
			BIT(NRF_GNSS_AGPS_GPS_UTC_REQUEST) ? 1 : 0;
	evt.agps_request.klobuchar = drv_data->last_agps.data_flags &
			BIT(NRF_GNSS_AGPS_KLOBUCHAR_REQUEST) ? 1 : 0;
	evt.agps_request.nequick = drv_data->last_agps.data_flags &
			BIT(NRF_GNSS_AGPS_NEQUICK_REQUEST) ? 1 : 0;
	evt.agps_request.system_time_tow = drv_data->last_agps.data_flags &
			BIT(NRF_GNSS_AGPS_SYS_TIME_AND_SV_TOW_REQUEST) ?
					1 : 0;
	evt.agps_request.position = drv_data->last_agps.data_flags &
			BIT(NRF_GNSS_AGPS_POSITION_REQUEST) ? 1 : 0;
	evt.agps_request.integrity = drv_data->last_agps.data_flags &
			BIT(NRF_GNSS_AGPS_INTEGRITY_REQUEST) ? 1 : 0;

	notify_event(dev, &evt);
}
#endif

static void sat_stats_work_fn(struct k_work *work)
{
	struct gps_drv_data *drv_data =
		CONTAINER_OF(work, struct gps_drv_data, sat_stats_work);

	/* Stats get printed already by this call */
	k_delayed_work_cancel(&drv_data->sat_stats_work);

	tracking_stats(&drv_data->last_pvt, drv_data->fix_timestamp);
}

static void nmea_ntf_work_fn(struct k_work *work)
{
	struct gps_drv_data *drv_data =
		CONTAINER_OF(work, struct gps_drv_data, nmea_ntf_work);
	const struct device *dev = drv_data->dev;
	struct gps_event evt;
	int idx = drv_data->nmea_fix_idx;

	memcpy(evt.nmea.buf, drv_data->nmea_strs[idx],
	       GPS_NMEA_SENTENCE_MAX_LENGTH);

	/* Don't count null terminator. */
	evt.nmea.len = strlen(evt.nmea.buf);

	if (atomic_get(&drv_data->has_fix)) {
		LOG_DBG("NMEA fix data: %s", evt.nmea.buf);
		evt.type = GPS_EVT_NMEA_FIX;
	} else {
		evt.type = GPS_EVT_NMEA;
	}

	notify_event(dev, &evt);
}

static void blocked_ntf_work_fn(struct k_work *work)
{
	struct gps_drv_data *drv_data =
		CONTAINER_OF(work, struct gps_drv_data, blocked_ntf_work);
	const struct device *dev = drv_data->dev;
	struct gps_event evt = { .type = GPS_EVT_OPERATION_BLOCKED};

	notify_event(dev, &evt);
}

static void unblocked_ntf_work_fn(struct k_work *work)
{
	struct gps_drv_data *drv_data =
		CONTAINER_OF(work, struct gps_drv_data, unblocked_ntf_work);
	const struct device *dev = drv_data->dev;
	struct gps_event evt = { .type = GPS_EVT_OPERATION_UNBLOCKED};

	notify_event(dev, &evt);
}

static void timeout_work_fn(struct k_work *work)
{
	struct gps_drv_data *drv_data =
		CONTAINER_OF(work, struct gps_drv_data, timeout_work);
	const struct device *dev = drv_data->dev;
	struct gps_event evt = {
		.type = GPS_EVT_SEARCH_TIMEOUT
	};

	if (!atomic_get(&drv_data->blocked)) {
		atomic_set(&drv_data->timeout_occurred, 1);
		notify_event(dev, &evt);
	}
}

static int agps_write(const struct device *dev, enum gps_agps_type type,
		      void *data, size_t data_len)
{
	int err;
	int nrf_type;
	(void)dev->data; // Was used with GNSS API v1

	switch (type) {
	case GPS_AGPS_UTC_PARAMETERS:
		nrf_type = NRF_GNSS_AGPS_UTC_PARAMETERS;
		break;
	case GPS_AGPS_EPHEMERIDES:
		nrf_type = NRF_GNSS_AGPS_EPHEMERIDES;
		break;
	case GPS_AGPS_ALMANAC:
		nrf_type = NRF_GNSS_AGPS_ALMANAC;
		break;
	case GPS_AGPS_KLOBUCHAR_CORRECTION:
		nrf_type = NRF_GNSS_AGPS_KLOBUCHAR_IONOSPHERIC_CORRECTION;
		break;
	case GPS_AGPS_NEQUICK_CORRECTION:
		nrf_type = NRF_GNSS_AGPS_NEQUICK_IONOSPHERIC_CORRECTION;
		break;
	case GPS_AGPS_GPS_SYSTEM_CLOCK_AND_TOWS:
		nrf_type = NRF_GNSS_AGPS_GPS_SYSTEM_CLOCK_AND_TOWS;
		break;
	case GPS_AGPS_LOCATION:
		nrf_type = NRF_GNSS_AGPS_LOCATION;
		break;
	case GPS_AGPS_INTEGRITY:
		nrf_type = NRF_GNSS_AGPS_INTEGRITY;
		break;
	default:
		LOG_ERR("Invalid argument");
		return -EINVAL;
	};

	err = nrf_gnss_write(data, data_len, nrf_type);
	if (err < 0) {
		LOG_ERR("Failed to send A-GPS data to modem, errno: %d", errno);
		return -errno;
	}

	LOG_DBG("Sent A-GPS data to modem, type: %d", type);

	return 0;
}

static int init(const struct device *dev, gps_event_handler_t handler)
{
	struct gps_drv_data *drv_data = dev->data;
	int err;

	if (atomic_get(&drv_data->is_init)) {
		LOG_WRN("GPS is already initialized");

		return -EALREADY;
	}

	if (handler == NULL) {
		LOG_ERR("No event handler provided");
		return -EINVAL;
	}

	drv_data->handler = handler;

	err = configure_antenna();
	if (err) {
		return err;
	}

	k_delayed_work_init(&drv_data->stop_work, stop_work_fn);
	k_delayed_work_init(&drv_data->timeout_work, timeout_work_fn);
	k_delayed_work_init(&drv_data->pvt_ntf_work, pvt_ntf_work_fn);
	k_delayed_work_init(&drv_data->sat_stats_work, sat_stats_work_fn);
	k_delayed_work_init(&drv_data->nmea_ntf_work, nmea_ntf_work_fn);
	k_delayed_work_init(&drv_data->blocked_ntf_work, blocked_ntf_work_fn);
	k_delayed_work_init(&drv_data->unblocked_ntf_work,
			    unblocked_ntf_work_fn);
#ifdef CONFIG_NRF9160_AGPS
	k_delayed_work_init(&drv_data->agps_ntf_work, agps_ntf_work_fn);
#endif
	k_sem_init(&drv_data->thread_run_sem, 0, 1);
	k_sem_init(&drv_data->gps_evt_sem, 0, 1);

	err = init_thread(dev);
	if (err) {
		LOG_ERR("Could not initialize GPS thread, error: %d",
			err);
		return err;
	}

	atomic_set(&drv_data->is_init, 1);

	return 0;
}

static struct gps_drv_data gps_drv_data = {};

static const struct gps_driver_api gps_api_funcs = {
	.init = init,
	.start = start,
	.stop = stop,
	.agps_write = agps_write,
};

DEVICE_DEFINE(nrf9160_gps, CONFIG_NRF9160_GPS_DEV_NAME,
	      setup, device_pm_control_nop,
	      &gps_drv_data, NULL, POST_KERNEL,
	      CONFIG_NRF9160_GPS_INIT_PRIO, &gps_api_funcs);
