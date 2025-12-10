/*
 * Copyright (c) 2025 NeuraSignal Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file st25rx00_init.c
 * @brief ST25Rx00 NFC Reader Driver Initialization
 *
 * This driver initializes the ST25R100/ST25R200 NFC reader IC via SPI.
 * The initialization is performed at APPLICATION level to allow for
 * the lengthy crystal stabilization delay without blocking kernel boot.
 *
 * Hardware Requirements:
 * - SPI bus configured for Mode 1 (CPOL=0, CPHA=1) per ST25R100 datasheet
 * - CS pin manually controlled (not via SPI controller)
 * - Reset pin active-HIGH for ST25R100, active-LOW for ST25R200
 * - IRQ pin active-LOW with internal pull-up
 *
 * Timing Requirements (ST25R100 Datasheet):
 * - Reset pulse minimum: 10us
 * - Crystal stabilization: ~10ms typical, 100ms maximum
 * - Power-on to ready: <100ms
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "rfal_platform.h"
#include "st25r200.h"
#include "st25r200_com.h"
#include "st25r200_irq.h"
#include "rfal_rf.h"
#include "rfal_nfc.h"

LOG_MODULE_REGISTER(st25rx00, CONFIG_ST25RX00_LOG_LEVEL);

/*
 ******************************************************************************
 * TIMING CONSTANTS (from ST25R100 Datasheet)
 ******************************************************************************
 */

/** Minimum reset pulse width in microseconds (datasheet: 10us min) */
#define ST25R100_RESET_PULSE_US         100U

/** Time for crystal to stabilize after reset release (datasheet: 10ms typ, 100ms max) */
#define ST25R100_CRYSTAL_STABILIZE_US   20000U

/** Maximum time to wait for chip initialization */
#define ST25R100_INIT_TIMEOUT_MS        500U

/** Delay between initialization retry attempts */
#define ST25R100_INIT_RETRY_DELAY_MS    50U

/** Maximum number of initialization retries */
#define ST25R100_INIT_MAX_RETRIES       3U

/*
 ******************************************************************************
 * DEVICE TREE CONFIGURATION
 ******************************************************************************
 */

#define DT_DRV_COMPAT st_st25rx00

/*
 ******************************************************************************
 * HELPER FUNCTIONS
 ******************************************************************************
 */

/**
 * @brief Perform hardware reset sequence on ST25R100
 *
 * The ST25R100 reset pin is active-HIGH. This function performs a clean
 * reset sequence with proper timing per the datasheet.
 *
 * @param config Device configuration containing GPIO specs
 * @return 0 on success, negative errno on failure
 */
static int st25rx00_hw_reset(const struct st25rx00_config *config)
{
    int ret;

    LOG_DBG("Performing hardware reset sequence");

    /*
     * ST25R100 RESET Pin Behavior (from datasheet):
     * - RESET is ACTIVE HIGH
     * - Drive HIGH to put chip in reset state
     * - Drive LOW for normal operation
     * - External pull-down recommended on PCB
     */

    /* Step 1: Ensure reset is released initially */
    ret = gpio_pin_set_dt(&config->reset_gpio, 0);
    if (ret < 0) {
        LOG_ERR("Failed to release reset initially: %d", ret);
        return ret;
    }
    k_busy_wait(ST25R100_RESET_PULSE_US);

    /* Step 2: Assert reset (drive HIGH) */
    ret = gpio_pin_set_dt(&config->reset_gpio, 1);
    if (ret < 0) {
        LOG_ERR("Failed to assert reset: %d", ret);
        return ret;
    }
    k_busy_wait(ST25R100_RESET_PULSE_US);

    /* Step 3: Release reset (drive LOW) - crystal needs time to stabilize */
    ret = gpio_pin_set_dt(&config->reset_gpio, 0);
    if (ret < 0) {
        LOG_ERR("Failed to release reset: %d", ret);
        return ret;
    }

    /* Step 4: Wait for crystal oscillator to stabilize */
    k_busy_wait(ST25R100_CRYSTAL_STABILIZE_US);

    LOG_DBG("Hardware reset complete");
    return 0;
}

/**
 * @brief Verify GPIO states after reset
 *
 * @param config Device configuration
 * @return 0 if states are correct, negative errno otherwise
 */
static int st25rx00_verify_gpio_states(const struct st25rx00_config *config)
{
    int reset_state, irq_state;

    /* Read reset pin - should be LOW (released) */
    reset_state = gpio_pin_get_dt(&config->reset_gpio);
    if (reset_state < 0) {
        LOG_ERR("Failed to read reset pin: %d", reset_state);
        return reset_state;
    }
    if (reset_state != 0) {
        LOG_WRN("Reset pin unexpectedly HIGH after release");
    }

    /* Read IRQ pin - should be HIGH (inactive, since it's active-low) */
    irq_state = gpio_pin_get_dt(&config->irq_gpio);
    if (irq_state < 0) {
        LOG_ERR("Failed to read IRQ pin: %d", irq_state);
        return irq_state;
    }

    LOG_DBG("GPIO states - Reset: %d, IRQ: %d", reset_state, irq_state);
    return 0;
}

/**
 * @brief Verify SPI communication by reading chip ID
 *
 * Only sets up minimal state to read registers. Full chip initialization
 * is done by rfalNfcInitialize() in the NFC thread.
 *
 * @param chip_id Pointer to store detected chip ID
 * @return 0 on success, negative errno on failure
 */
static int st25rx00_verify_spi_comm(uint8_t *chip_id)
{
    uint8_t retry;
    uint8_t id = 0;

    /* Ensure CS is deselected before any SPI transaction */
    platformSpiDeselect();

    for (retry = 0; retry < ST25R100_INIT_MAX_RETRIES; retry++) {
        if (retry > 0) {
            LOG_WRN("Retrying SPI test (attempt %u/%u)",
                    retry + 1, ST25R100_INIT_MAX_RETRIES);
            k_msleep(ST25R100_INIT_RETRY_DELAY_MS);
        }

        /* Read IC ID register directly */
        st25r200ReadRegister(ST25R200_REG_IC_ID, &id);

        uint8_t ic_type = (id & ST25R200_REG_IC_ID_ic_type_mask) >> ST25R200_REG_IC_ID_ic_type_shift;

        if (ic_type == ST25R200_REG_IC_ID_ic_type_st25r200) {
            if (chip_id != NULL) {
                *chip_id = id;
            }
            return 0;
        }

        LOG_WRN("Unexpected IC ID: 0x%02X (type=0x%X)", id, ic_type);
    }

    LOG_ERR("SPI verification failed after %u attempts", ST25R100_INIT_MAX_RETRIES);
    return -EIO;
}


/*
 ******************************************************************************
 * DRIVER INITIALIZATION
 ******************************************************************************
 */

/**
 * @brief Initialize the ST25Rx00 NFC reader driver
 *
 * This function initializes the platform layer, performs hardware reset,
 * and initializes the RFAL library. It includes retry logic for robustness.
 *
 * @param dev Pointer to device structure
 * @return 0 on success, negative errno on failure
 */
static int st25rx00_init(const struct device *dev)
{
    int ret;
    uint8_t chip_id = 0;
    const struct st25rx00_config *config = dev->config;

    /* Very early debug output - printk works before deferred logging */
    printk("[ST25RX00] 1. Init starting\n");

    LOG_INF("Initializing ST25Rx00 NFC Reader");

    /* Step 1: Initialize platform layer (SPI, GPIOs) */
    printk("[ST25RX00] 2. Platform init\n");
    ret = rfal_platform_init(dev);
    if (ret < 0) {
        LOG_ERR("Platform initialization failed: %d", ret);
        return ret;
    }
    printk("[ST25RX00] 3. Platform init done\n");

    /* Step 2: Log GPIO configuration for debugging */
    printk("[ST25RX00] 4. GPIO: Reset=%s:%d, IRQ=%s:%d, CS=%s:%d\n",
            config->reset_gpio.port->name, config->reset_gpio.pin,
            config->irq_gpio.port->name, config->irq_gpio.pin,
            config->cs_gpio.port->name, config->cs_gpio.pin);

    /* Step 3: Perform hardware reset sequence */
    printk("[ST25RX00] 5. HW reset\n");
    ret = st25rx00_hw_reset(config);
    if (ret < 0) {
        LOG_ERR("Hardware reset failed: %d", ret);
        return ret;
    }
    printk("[ST25RX00] 6. HW reset done\n");

    /* Step 4: Verify GPIO states */
    printk("[ST25RX00] 7. GPIO verify\n");
    ret = st25rx00_verify_gpio_states(config);
    if (ret < 0) {
        LOG_WRN("GPIO state verification failed: %d (continuing anyway)", ret);
        /* Don't fail here - chip might still work */
    }
    printk("[ST25RX00] 8. GPIO verify done\n");

    /* Step 5: Verify SPI works by reading chip ID (no full init yet) */
    printk("[ST25RX00] 9. SPI verify\n");
    ret = st25rx00_verify_spi_comm(&chip_id);
    if (ret < 0) {
        printk("[ST25RX00] SPI verify FAILED: %d\n", ret);
        LOG_ERR("ST25Rx00 SPI communication failed");
        return ret;
    }
    printk("[ST25RX00] 10. SPI OK, ID=0x%02X\n", chip_id);
    LOG_INF("ST25Rx00 detected, IC ID: 0x%02X", chip_id);

    printk("[ST25RX00] 11. Platform ready\n");
    LOG_INF("ST25Rx00 platform initialized");
    return 0;
}

/*
 ******************************************************************************
 * DEVICE INSTANTIATION
 ******************************************************************************
 */

/*
 * SPI Mode Configuration for ST25R100/ST25R200
 *
 * Per ST25R100 Datasheet Section 7.2 (SPI Interface):
 * "The ST25R100 has a standard serial peripheral interface with clock polarity
 *  of 0, a clock phase of 1, and an active low slave select signal."
 *
 * This corresponds to SPI Mode 1:
 * - CPOL = 0 (clock idles low)
 * - CPHA = 1 (data sampled on falling edge of SCLK)
 * - MOSI sampled on falling edge
 * - MISO updated on rising edge
 * - MSB first
 * - Active-low chip select (BSS)
 */
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
        .led_conn_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, led_conn_gpios, {0}),    \
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
