#
# Copyright (c) 2019 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

zephyr_library()
zephyr_library_sources(
	at_cmd_parser.c
	at_utils.c
	at_params.c
	at_cmd_decoder.c
)

zephyr_include_directories(include)
