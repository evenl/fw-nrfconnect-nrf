#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: BSD-5-Clause-Nordic
#

zephyr_library()

zephyr_library_sources(
  mqtt_decoder.c
  mqtt_encoder.c
  mqtt_rx.c
  mqtt_transport_socket_tcp.c
  mqtt_transport.c
  mqtt.c
  )

zephyr_library_sources_ifdef(CONFIG_MQTT_LIB_TLS
  mqtt_transport_socket_tls.c
  )
