#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

if(CONFIG_AT_CMD_INTERFACE)
    zephyr_library()
    zephyr_library_sources(at_cmd_interface.c)
endif()
