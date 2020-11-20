/*$$$LICENCE_NORDIC_STANDARD<2016>$$$*/
/**
 * @file gnss_interface.h
 *
 * @defgroup interface_at Socket interface for GNSS clients.
 * @{
 * @ingroup interfaces
 * @brief Interface for GNSS clients used by the socket layer.
 */
#ifndef GNSS2_INTERFACE_H__
#define GNSS2_INTERFACE_H__

#include <stdint.h>
#include <stdbool.h>
#include "compiler_abstraction.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GNSS_PVT_NTF     1
#define GNSS_GPS_FIX_NTF 2
#define GNSS_NMEA_NTF    3
#define GNSS_AGPS_NTF    4
#define GNSS_BLOCKED_NTF 5

#define GNSS_MAGIC_HANDLE 0x49765443

typedef int (*gnss2_callback_type_t)(int event);

struct gnss_config {
	int16_t elevation_mask;
	int16_t use_case;
	int16_t system;
	int32_t fix_interval;
	int32_t fix_retry;
	int32_t nmea_mask;
	int16_t duty_cycling;
};

int32_t gnss2_init(void);
int32_t gnss2_deinit(void);
int32_t gnss2_stop(void);
int32_t gnss2_start(void);
int32_t gnss2_set_configuration(struct gnss_config * config);
int32_t gnss2_set_callback(gnss2_callback_type_t callback);
int32_t gnss2_interface_read(void * buf, int32_t buf_len, int type);

#ifdef __cplusplus
}
#endif

#endif
