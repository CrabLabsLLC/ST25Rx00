/*
 * Copyright (c) 2025 NeuraSignal Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file rfal_platform.c
 * @brief RFAL Platform Abstraction Layer Implementation for Zephyr
 */

#include "rfal_platform.h"
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(rfal_platform, CONFIG_ST25RX00_LOG_LEVEL);

/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */

/* Pointer to the device instance - set during initialization */
static const struct device *st25rx00_dev;
static struct k_spinlock platform_lock;
static unsigned int saved_irq_key;

/*
 ******************************************************************************
 * PLATFORM INITIALIZATION
 ******************************************************************************
 */

int rfal_platform_init(const struct device *dev)
{
    const struct st25rx00_config *config = dev->config;
    struct st25rx00_data *data = dev->data;
    int ret;

    LOG_INF("Initializing RFAL platform for %s", dev->name);

    /* Store device pointer for platform functions */
    st25rx00_dev = dev;

    /* Spinlocks are zero-initialized by default in Zephyr */

    /* Check SPI bus is ready */
    if (!spi_is_ready_dt(&config->spi)) {
        LOG_ERR("SPI bus not ready");
        return -ENODEV;
    }

    /* Configure reset GPIO */
    if (!gpio_is_ready_dt(&config->reset_gpio)) {
        LOG_ERR("Reset GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure reset GPIO: %d", ret);
        return ret;
    }

    /* Configure IRQ GPIO */
    if (!gpio_is_ready_dt(&config->irq_gpio)) {
        LOG_ERR("IRQ GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure IRQ GPIO: %d", ret);
        return ret;
    }

    /* Configure optional LED GPIOs */
    if (config->led_field_gpio.port != NULL) {
        if (gpio_is_ready_dt(&config->led_field_gpio)) {
            gpio_pin_configure_dt(&config->led_field_gpio, GPIO_OUTPUT_INACTIVE);
        }
    }

    if (config->led_rx_gpio.port != NULL) {
        if (gpio_is_ready_dt(&config->led_rx_gpio)) {
            gpio_pin_configure_dt(&config->led_rx_gpio, GPIO_OUTPUT_INACTIVE);
        }
    }

    if (config->led_tx_gpio.port != NULL) {
        if (gpio_is_ready_dt(&config->led_tx_gpio)) {
            gpio_pin_configure_dt(&config->led_tx_gpio, GPIO_OUTPUT_INACTIVE);
        }
    }

    if (config->led_error_gpio.port != NULL) {
        if (gpio_is_ready_dt(&config->led_error_gpio)) {
            gpio_pin_configure_dt(&config->led_error_gpio, GPIO_OUTPUT_INACTIVE);
        }
    }

    data->initialized = true;
    LOG_INF("RFAL platform initialized successfully");
    return 0;
}

/*
 ******************************************************************************
 * INTERRUPT PROTECTION
 ******************************************************************************
 */

void rfal_platform_protect_comm(void)
{
    saved_irq_key = irq_lock();
}

void rfal_platform_unprotect_comm(void)
{
    irq_unlock(saved_irq_key);
}

void rfal_platform_protect_irq_status(void)
{
    rfal_platform_protect_comm();
}

void rfal_platform_unprotect_irq_status(void)
{
    rfal_platform_unprotect_comm();
}

/*
 ******************************************************************************
 * GPIO FUNCTIONS
 ******************************************************************************
 */

void rfal_platform_gpio_set(uint32_t pin)
{
    if (st25rx00_dev == NULL) {
        return;
    }

    const struct st25rx00_config *config = st25rx00_dev->config;

    /* This is called for RESET pin - active high to reset */
    gpio_pin_set_dt(&config->reset_gpio, 1);
}

void rfal_platform_gpio_clear(uint32_t pin)
{
    if (st25rx00_dev == NULL) {
        return;
    }

    const struct st25rx00_config *config = st25rx00_dev->config;

    /* This is called for RESET pin - clear to release reset */
    gpio_pin_set_dt(&config->reset_gpio, 0);
}

bool rfal_platform_gpio_is_high(uint32_t pin)
{
    if (st25rx00_dev == NULL) {
        return false;
    }

    const struct st25rx00_config *config = st25rx00_dev->config;

    /* This is called for IRQ pin */
    return (gpio_pin_get_dt(&config->irq_gpio) != 0);
}

/*
 ******************************************************************************
 * INTERRUPT FUNCTIONS
 ******************************************************************************
 */

static void rfal_platform_irq_handler(const struct device *dev,
                                      struct gpio_callback *cb,
                                      uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(pins);

    struct st25rx00_data *data = CONTAINER_OF(cb, struct st25rx00_data, irq_cb);

    /* Call the RFAL ISR callback if set */
    if (data->isr_callback != NULL) {
        data->isr_callback();
    }
}

void rfal_platform_irq_init(void)
{
    if (st25rx00_dev == NULL) {
        LOG_ERR("Device not initialized");
        return;
    }

    const struct st25rx00_config *config = st25rx00_dev->config;
    struct st25rx00_data *data = st25rx00_dev->data;
    int ret;

    /* Initialize GPIO callback */
    gpio_init_callback(&data->irq_cb, rfal_platform_irq_handler,
                       BIT(config->irq_gpio.pin));

    /* Add callback */
    ret = gpio_add_callback(config->irq_gpio.port, &data->irq_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add IRQ callback: %d", ret);
        return;
    }

    /* Configure interrupt on falling edge (IRQ is active LOW) */
    ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure IRQ interrupt: %d", ret);
        return;
    }

    LOG_DBG("IRQ pin configured successfully");
}

void rfal_platform_irq_set_callback(void (*cb)(void))
{
    if (st25rx00_dev == NULL) {
        return;
    }

    struct st25rx00_data *data = st25rx00_dev->data;
    data->isr_callback = cb;
}

/*
 ******************************************************************************
 * TIMER FUNCTIONS
 ******************************************************************************
 */

uint32_t rfal_platform_timer_create(uint16_t time_ms)
{
    /* Return the expiration time in milliseconds */
    return (uint32_t)(k_uptime_get() + time_ms);
}

bool rfal_platform_timer_is_expired(uint32_t timer)
{
    return ((uint32_t)k_uptime_get() >= timer);
}

uint32_t rfal_platform_timer_get_remaining(uint32_t timer)
{
    uint32_t now = (uint32_t)k_uptime_get();
    if (now >= timer) {
        return 0;
    }
    return (timer - now);
}

/*
 ******************************************************************************
 * SPI FUNCTIONS
 ******************************************************************************
 */

void rfal_platform_spi_select(void)
{
    /* CS is handled automatically by SPI subsystem via device tree */
}

void rfal_platform_spi_deselect(void)
{
    /* CS is handled automatically by SPI subsystem via device tree */
}

int rfal_platform_spi_txrx(const uint8_t *txBuf, uint8_t *rxBuf, uint16_t len)
{
    if (st25rx00_dev == NULL) {
        return -ENODEV;
    }

    const struct st25rx00_config *config = st25rx00_dev->config;
    int ret;

    struct spi_buf tx_buf = {
        .buf = (void *)txBuf,
        .len = len
    };
    struct spi_buf_set tx_bufs = {
        .buffers = &tx_buf,
        .count = 1
    };

    struct spi_buf rx_buf = {
        .buf = rxBuf,
        .len = len
    };
    struct spi_buf_set rx_bufs = {
        .buffers = &rx_buf,
        .count = 1
    };

    /* Handle different transfer modes */
    if (txBuf != NULL && rxBuf != NULL) {
        /* Full duplex transfer */
        ret = spi_transceive_dt(&config->spi, &tx_bufs, &rx_bufs);
    } else if (txBuf != NULL) {
        /* TX only */
        ret = spi_write_dt(&config->spi, &tx_bufs);
    } else if (rxBuf != NULL) {
        /* RX only */
        ret = spi_read_dt(&config->spi, &rx_bufs);
    } else {
        return -EINVAL;
    }

    if (ret < 0) {
        LOG_ERR("SPI transfer failed: %d", ret);
    }

    return ret;
}

/*
 ******************************************************************************
 * LED FUNCTIONS
 ******************************************************************************
 */

void rfal_platform_led_on(void)
{
    if (st25rx00_dev == NULL) {
        return;
    }

    const struct st25rx00_config *config = st25rx00_dev->config;

    if (config->led_field_gpio.port != NULL) {
        gpio_pin_set_dt(&config->led_field_gpio, 1);
    }
}

void rfal_platform_led_off(void)
{
    if (st25rx00_dev == NULL) {
        return;
    }

    const struct st25rx00_config *config = st25rx00_dev->config;

    if (config->led_field_gpio.port != NULL) {
        gpio_pin_set_dt(&config->led_field_gpio, 0);
    }
}

/*
 ******************************************************************************
 * ERROR HANDLING
 ******************************************************************************
 */

void rfal_platform_error_handle(void)
{
    LOG_ERR("RFAL platform error occurred!");

    /* In a production system, you might want to:
     * - Reset the NFC reader
     * - Log detailed error information
     * - Notify the application
     * - Enter a safe state
     */

    __ASSERT(0, "RFAL fatal error");
}
