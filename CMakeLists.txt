# ST25Rx00 NFC Driver CMakeLists.txt
# Copyright (c) 2025 NeuraSignal Inc.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_ST25RX00)

# Create st25rx00 library
zephyr_library_named(st25rx00)

# Zephyr platform abstraction layer
zephyr_library_sources(
    zephyr/rfal_platform.c
    zephyr/st25rx00_init.c
    zephyr/st25rx00_api.c
)

# RFAL core source files
# Note: rfal_dlma.c excluded - DLMA requires Listen Mode which ST25R200 doesn't support
zephyr_library_sources(
    rfal/source/rfal_analogConfig.c
    rfal/source/rfal_cd.c
    rfal/source/rfal_crc.c
    rfal/source/rfal_dpo.c
    rfal/source/rfal_iso15693_2.c
    rfal/source/rfal_isoDep.c
    rfal/source/rfal_nfc.c
    rfal/source/rfal_nfca.c
    rfal/source/rfal_nfcb.c
    rfal/source/rfal_nfcDep.c
    rfal/source/rfal_nfcf.c
    rfal/source/rfal_nfcv.c
    rfal/source/rfal_st25tb.c
    rfal/source/rfal_st25xv.c
    rfal/source/rfal_t1t.c
    rfal/source/rfal_t2t.c
    rfal/source/rfal_t4t.c
)

# ST25R200-specific source files
zephyr_library_sources(
    rfal/source/st25r200/rfal_rfst25r200.c
    rfal/source/st25r200/st25r200.c
    rfal/source/st25r200/st25r200_com.c
    rfal/source/st25r200/st25r200_irq.c
)

# Include directories - use zephyr_include_directories for global visibility
zephyr_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/zephyr
    ${CMAKE_CURRENT_SOURCE_DIR}/rfal/include
    ${CMAKE_CURRENT_SOURCE_DIR}/rfal/source
    ${CMAKE_CURRENT_SOURCE_DIR}/rfal/source/st25r200
)

# Compile definitions
zephyr_compile_definitions(
    ST25R200
    RFAL_USE_I2C=0
)

# Suppress warnings from ST library code (vendor code style)
zephyr_library_compile_options(
    -Wno-unused-parameter
    -Wno-sign-compare
)

endif() # CONFIG_ST25RX00
