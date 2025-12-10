/*
 * Copyright (c) 2025 NeuraSignal Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file st25rx00_init.c
 * @brief ST25Rx00 NFC Reader Driver Initialization
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "rfal_platform.h"
#include "st25r200.h"
#include "st25r200_irq.h"
#include "rfal_rf.h"
#include "rfal_nfc.h"

LOG_MODULE_REGISTER(st25rx00, CONFIG_ST25RX00_LOG_LEVEL);

/*
 ******************************************************************************
 * DEVICE TREE CONFIGURATION
 ******************************************************************************
 */

#define DT_DRV_COMPAT st_st25rx00

/*
 ******************************************************************************
 * DRIVER INITIALIZATION
 ******************************************************************************
 */

static int st25rx00_init(const struct device *dev)
{
    int ret;
    uint8_t chip_rev;

    LOG_INF("Initializing ST25Rx00 NFC Reader");

    /* Initialize platform layer (SPI, GPIOs) */
    ret = rfal_platform_init(dev);
    if (ret < 0) {
        LOG_ERR("Platform initialization failed: %d", ret);
        return ret;
    }

    /* Perform hardware reset sequence */
    LOG_DBG("Performing hardware reset");

    /* Assert reset */
    rfal_platform_gpio_set(0);
    k_msleep(10);

    /* Release reset */
    rfal_platform_gpio_clear(0);
    k_msleep(10);

    /* Initialize the ST25R200 chip */
    ReturnCode err = st25r200Initialize();
    if (err != RFAL_ERR_NONE) {
        LOG_ERR("ST25R200 initialization failed: %d", err);
        return -EIO;
    }

    /* Verify chip ID */
    if (!st25r200CheckChipID(&chip_rev)) {
        LOG_ERR("Chip ID verification failed");
        return -ENODEV;
    }
    LOG_INF("ST25R200 detected, revision: 0x%02X", chip_rev);

    /* Initialize interrupt handling */
    st25r200InitInterrupts();

    LOG_INF("ST25Rx00 initialized successfully");
    return 0;
}

/*
 ******************************************************************************
 * DEVICE INSTANTIATION
 ******************************************************************************
 */

/* SPI mode: CPOL=0, CPHA=0 (Mode 0) for ST25R200 */
#define ST25RX00_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

#define ST25RX00_INIT(inst)                                                     \
    static struct st25rx00_data st25rx00_data_##inst;                           \
                                                                                \
    static const struct st25rx00_config st25rx00_config_##inst = {              \
        .spi = SPI_DT_SPEC_INST_GET(inst, ST25RX00_SPI_OPERATION, 0),            \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),                      \
        .reset_gpio = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),                  \
        .led_field_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, led_field_gpios, {0}),  \
        .led_rx_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, led_rx_gpios, {0}),        \
        .led_tx_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, led_tx_gpios, {0}),        \
        .led_error_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, led_error_gpios, {0}),  \
    };                                                                          \
                                                                                \
    DEVICE_DT_INST_DEFINE(inst,                                                 \
                          st25rx00_init,                                        \
                          NULL,                                                 \
                          &st25rx00_data_##inst,                                \
                          &st25rx00_config_##inst,                              \
                          POST_KERNEL,                                          \
                          CONFIG_ST25RX00_INIT_PRIORITY,                        \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(ST25RX00_INIT)
