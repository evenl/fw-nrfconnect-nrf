
#include <logging/log.h>
#include <nrf_inbuilt_key.h>
#include <nrf_key_mgmt.h>

#if 0
static const char amazon[] = {
	#include "../certs/s3.amazonaws.com"
};
#endif

static const char digicert[] = {
	#include "../certs/DigiCert Baltimore CA-2 G2"
};

#if 0
static const char baltimore[] = {
	#include "../certs/Baltimore CyberTrust Root"
};
#endif

LOG_MODULE_DECLARE(app);

int cert_provision(void)
{
	int err;

	nrf_sec_tag_t sec_tag = 0xA;

	#if 0
	/* Delete certificates */

	for (nrf_key_mgnt_cred_type_t type = 0; type < 5; type++) {
		err = nrf_inbuilt_key_delete(sec_tag, type);
		LOG_DBG("nrf_inbuilt_key_delete(%u, %d) => result=%d", sec_tag,
			type, err);
	}
	#endif

	err = nrf_inbuilt_key_write(sec_tag, NRF_KEY_MGMT_CRED_TYPE_CA_CHAIN,
				    (char*)digicert, sizeof(digicert) - 1);
	if (err) {
		LOG_ERR("NRF_CLOUD_CA_CERTIFICATE digicert err: %d", err);
		return err;
	}

	return 0;
}