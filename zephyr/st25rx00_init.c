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
    uint8_t chip_id;
    const struct st25rx00_config *config = dev->config;

    LOG_INF("Initializing ST25Rx00 NFC Reader");

    /* Initialize platform layer (SPI, GPIOs) */
    ret = rfal_platform_init(dev);
    if (ret < 0) {
        LOG_ERR("Platform initialization failed: %d", ret);
        return ret;
    }

    /* Perform hardware reset sequence with proper timing */
    LOG_INF("Performing hardware reset sequence");

    /* Check reset pin state */
    LOG_INF("Reset GPIO port: %s, pin: %d", config->reset_gpio.port->name, config->reset_gpio.pin);

    /* ST25R100 RESET is ACTIVE HIGH:
     * - To reset: drive pin HIGH (logic 1)
     * - Normal operation: drive pin LOW (logic 0)
     * Pull-down ensures valid LOW state when released.
     */

    /* Ensure reset is released first (LOW = not in reset) */
    gpio_pin_set_dt(&config->reset_gpio, 0);
    k_msleep(10);

    /* Assert reset (HIGH = reset active) */
    LOG_INF("Asserting reset (driving HIGH)...");
    gpio_pin_set_dt(&config->reset_gpio, 1);
    k_msleep(5);  /* Hold reset for 5ms */

    /* Release reset (LOW = normal operation) - chip needs time for crystal to stabilize */
    LOG_INF("Releasing reset (driving LOW for normal operation)...");
    gpio_pin_set_dt(&config->reset_gpio, 0);
    k_msleep(100);  /* Give crystal time to stabilize (27.12 MHz with 4pF load) */

    LOG_INF("Reset sequence complete, waiting for oscillator...");

    /* Verify reset pin is LOW (released) */
    int reset_state = gpio_pin_get_dt(&config->reset_gpio);
    LOG_INF("Reset pin current state: %d (should be 0 for released/normal operation)", reset_state);

    /* Check IRQ pin - should be high (inactive) after reset if chip is ready */
    int irq_state = gpio_pin_get_dt(&config->irq_gpio);
    LOG_INF("IRQ pin current state: %d (should be 1 for active-low inactive)", irq_state);

    /* Initialize the ST25R200/ST25R100 chip */
    ReturnCode err = st25r200Initialize();
    if (err != RFAL_ERR_NONE) {
        LOG_ERR("ST25Rx00 initialization failed: %d", err);
        return -EIO;
    }

    /* Verify chip ID */
    if (!st25r200CheckChipID(&chip_id)) {
        LOG_ERR("Chip ID verification failed (ID=0x%02X)", chip_id);
        return -ENODEV;
    }
    LOG_INF("ST25Rx00 detected, IC ID: 0x%02X", chip_id);

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

/* SPI mode: CPOL=0, CPHA=1 (Mode 1) for ST25R100/ST25R200 - per ST datasheet */
#define ST25RX00_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA)

/*
 * SPI configuration without automatic CS control.
 * CS is manually controlled by rfal_platform.c to support multi-byte
 * SPI sequences required by the RFAL library.
 */
#define ST25RX00_SPI_CONFIG(inst)  {                                            \
    .bus = DEVICE_DT_GET(DT_INST_BUS(inst)),                                    \
    .config = {                                                                  \
        .frequency = DT_INST_PROP(inst, spi_max_frequency),                     \
        .operation = ST25RX00_SPI_OPERATION,                                    \
        .slave = 0,                                                              \
        .cs = { .gpio = { .port = NULL }, .delay = 0 },                         \
    },                                                                           \
}

#define ST25RX00_INIT(inst)                                                     \
    static struct st25rx00_data st25rx00_data_##inst;                           \
                                                                                \
    static const struct st25rx00_config st25rx00_config_##inst = {              \
        .spi = ST25RX00_SPI_CONFIG(inst),                                       \
        .cs_gpio = GPIO_DT_SPEC_INST_GET(inst, cs_gpios),                        \
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
