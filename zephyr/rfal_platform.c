/*
 * Copyright (c) 2025 NeuraSignal Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file rfal_platform.c
 * @brief RFAL Platform Abstraction Layer Implementation for Zephyr
 */

#include "rfal_platform.h"
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(rfal_platform, CONFIG_ST25RX00_LOG_LEVEL);

/*
 ******************************************************************************
 * CONSTANTS
 ******************************************************************************
 */

/**
 * SPI temporary buffer size for same-buffer TX/RX operations.
 * Must be larger than ST25R200 FIFO depth (256 bytes) plus command overhead.
 * Used when ST25R_COM_SINGLETXRX mode requires TX and RX in the same buffer.
 */
#define SPI_TEMP_BUF_SIZE               (256U + 4U)

/**
 * Maximum timer duration supported (in milliseconds).
 * Using signed comparison for wraparound handling limits us to ~24 days.
 * This is sufficient for all NFC timing requirements.
 */
#define PLATFORM_TIMER_MAX_MS           (INT32_MAX / 2)

/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */

/** Pointer to the device instance - set during initialization */
static const struct device *st25rx00_dev;

/*
 * THREAD SAFETY:
 *
 * This driver is NOT thread-safe. All NFC operations must be called from
 * a single thread context. The ST RFAL library makes nested/re-entrant SPI
 * calls internally which would deadlock with mutex protection.
 *
 * IRQ Handling Architecture (Interrupt Mode - default):
 *
 * The ST RFAL library has blocking spin loops that don't yield to the
 * scheduler. During these waits, the library polls platformGpioIsHigh()
 * to check for pending interrupts. We use this as an opportunity to
 * process interrupts synchronously when detected.
 *
 * Flow:
 * 1. Hardware IRQ fires -> GPIO ISR gives semaphore
 * 2. NFC thread wakes from semaphore wait
 * 3. NFC thread calls rfalWorker() which processes interrupts via SPI
 * 4. RFAL state machine advances
 *
 * The semaphore provides proper kernel-level wake/sleep for low power.
 * The platformGpioIsHigh() fallback ensures interrupts are processed
 * even during tight RFAL spin loops that don't yield.
 */

#ifndef CONFIG_ST25RX00_IRQ_POLLING
/* Interrupt mode state */
static void (* volatile isr_callback)(void);  /* Pointer is volatile, not return type */
static atomic_t irq_pending;  /* Atomic flag for IRQ pending state */
static struct k_sem irq_sem;  /* Semaphore for thread wakeup */
static bool irq_initialized;  /* Guard against double-init */
#endif

/** Temporary buffer for same-buffer SPI TX/RX operations */
static uint8_t spi_temp_buf[SPI_TEMP_BUF_SIZE];

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

    /* Check SPI bus is ready */
    if (!spi_is_ready_dt(&config->spi)) {
        LOG_ERR("SPI bus not ready");
        return -ENODEV;
    }

    /* Configure CS GPIO - manually controlled for multi-byte transactions */
    if (!gpio_is_ready_dt(&config->cs_gpio)) {
        LOG_ERR("CS GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure CS GPIO: %d", ret);
        return ret;
    }
    LOG_DBG("CS GPIO configured: %s pin %d",
            config->cs_gpio.port->name, config->cs_gpio.pin);

    /* Configure reset GPIO */
    if (!gpio_is_ready_dt(&config->reset_gpio)) {
        LOG_ERR("Reset GPIO not ready");
        return -ENODEV;
    }

    /* Configure reset as output, inactive state.
     * ST25R100: Reset is active-HIGH. External pull-down on PCB ensures chip
     * stays out of reset when MCU is in reset/boot. */
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
    /* No-op - driver is single-threaded by design (see THREAD SAFETY note above) */
}

void rfal_platform_unprotect_comm(void)
{
    /* No-op - driver is single-threaded by design */
}

void rfal_platform_protect_irq_status(void)
{
    /* For IRQ status protection, we don't need to do anything special
     * since we're using a cooperative threading model */
}

void rfal_platform_unprotect_irq_status(void)
{
    /* Nothing to do */
}

/*
 ******************************************************************************
 * GPIO FUNCTIONS
 ******************************************************************************
 */

void rfal_platform_gpio_set(uint32_t pin)
{
    ARG_UNUSED(pin);

    if (st25rx00_dev == NULL) {
        return;
    }

    const struct st25rx00_config *config = st25rx00_dev->config;

    /* This is called for RESET pin - active high to reset */
    gpio_pin_set_dt(&config->reset_gpio, 1);
}

void rfal_platform_gpio_clear(uint32_t pin)
{
    ARG_UNUSED(pin);

    if (st25rx00_dev == NULL) {
        return;
    }

    const struct st25rx00_config *config = st25rx00_dev->config;

    /* This is called for RESET pin - clear to release reset */
    gpio_pin_set_dt(&config->reset_gpio, 0);
}

bool rfal_platform_gpio_is_high(uint32_t pin)
{
    ARG_UNUSED(pin);

    if (st25rx00_dev == NULL) {
        return false;
    }

    const struct st25rx00_config *config = st25rx00_dev->config;

    /*
     * Check if the IRQ pin is active (interrupt pending).
     *
     * The ST25R IRQ is active-LOW on hardware. With GPIO_ACTIVE_LOW flag,
     * gpio_pin_get_dt() returns 1 when active (physically LOW).
     *
     * This function is called by st25r200CheckForReceivedInterrupts() in
     * polling mode to check for pending interrupts. In interrupt mode,
     * it's also called during spin waits, giving us an opportunity to
     * process any pending interrupts synchronously.
     */
#ifdef CONFIG_ST25RX00_IRQ_POLLING
    /* Polling mode: read GPIO directly */
    int gpio_val = gpio_pin_get_dt(&config->irq_gpio);
    if (gpio_val < 0) {
        LOG_ERR("GPIO read failed: %d", gpio_val);
        return false;
    }
    return (gpio_val != 0);
#else
    /*
     * Interrupt mode: Check both the pending flag and GPIO state.
     *
     * irq_pending is set by the GPIO ISR on falling edge.
     * We also check GPIO directly as fallback for:
     * - Interrupts pending before ISR was configured
     * - Level-triggered behavior (IRQ stays low until serviced)
     *
     * Use atomic_clear() to atomically read and clear the flag.
     * This prevents race condition where ISR fires between read and clear.
     */
    bool was_pending = atomic_clear(&irq_pending);

    /* Read GPIO with proper error handling */
    int gpio_val = gpio_pin_get_dt(&config->irq_gpio);
    if (gpio_val < 0) {
        LOG_ERR("GPIO read failed: %d", gpio_val);
        return was_pending;  /* Return pending flag even on error */
    }
    bool gpio_active = (gpio_val != 0);
    bool active = was_pending || gpio_active;

    if (active && isr_callback != NULL) {
        /*
         * Process interrupt synchronously. This is critical because:
         * 1. RFAL's spin loops don't yield to the scheduler
         * 2. We must update interrupt status during the spin wait
         * 3. SPI reads are safe here - we're in thread context
         *
         * The callback (st25r200Isr) will:
         * - Read IRQ status registers via SPI
         * - Clear hardware interrupt flags
         * - Update RFAL's internal interrupt status
         */
        isr_callback();
    }

    return active;
#endif
}

/*
 ******************************************************************************
 * INTERRUPT FUNCTIONS
 ******************************************************************************
 */

#ifdef CONFIG_ST25RX00_IRQ_POLLING

/*
 * Polling Mode Implementation
 * RFAL library polls GPIO directly - minimal stubs for API compatibility
 */

void rfal_platform_irq_init(void)
{
    LOG_DBG("IRQ mode: polling");
}

void rfal_platform_irq_set_callback(void (*cb)(void))
{
    ARG_UNUSED(cb);
}

void rfal_platform_irq_clear(void)
{
    /* Nothing to clear in polling mode */
}

#else /* Interrupt Mode */

/*
 * Interrupt Mode Implementation
 *
 * Minimal ISR that just sets a flag. All actual interrupt processing
 * happens synchronously in platformGpioIsHigh() when RFAL polls it.
 * This avoids the need for work queue deferral and its complexity.
 */

/**
 * GPIO IRQ handler - runs in ISR context
 * Sets atomic flag and gives semaphore to wake NFC thread
 */
static void rfal_platform_irq_handler(const struct device *dev,
                                      struct gpio_callback *cb,
                                      uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    atomic_set(&irq_pending, 1);
    k_sem_give(&irq_sem);
}

void rfal_platform_irq_init(void)
{
    if (st25rx00_dev == NULL) {
        LOG_ERR("Device not initialized");
        return;
    }

    /* Guard against multiple initializations.
     * RFAL calls this during st25r200Initialize() which may be called
     * multiple times (driver init + rfalNfcInitialize). */
    if (irq_initialized) {
        LOG_DBG("IRQ already initialized, skipping");
        return;
    }

    const struct st25rx00_config *config = st25rx00_dev->config;
    struct st25rx00_data *data = st25rx00_dev->data;
    int ret;

    /* Reset state using atomic operations */
    atomic_clear(&irq_pending);
    isr_callback = NULL;

    /* Initialize semaphore for thread synchronization */
    k_sem_init(&irq_sem, 0, 1);

    /* Initialize and add GPIO callback */
    gpio_init_callback(&data->irq_cb, rfal_platform_irq_handler,
                       BIT(config->irq_gpio.pin));

    ret = gpio_add_callback(config->irq_gpio.port, &data->irq_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add IRQ callback: %d", ret);
        return;
    }

    /* Configure edge-triggered interrupt on falling edge.
     * GPIO_INT_EDGE_TO_ACTIVE with GPIO_ACTIVE_LOW means falling edge. */
    ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure IRQ: %d", ret);
        return;
    }

    irq_initialized = true;
    LOG_DBG("IRQ mode: interrupt (edge-triggered)");
}

void rfal_platform_irq_set_callback(void (*cb)(void))
{
    isr_callback = cb;
}

void rfal_platform_irq_clear(void)
{
    atomic_clear(&irq_pending);
}

#endif /* CONFIG_ST25RX00_IRQ_POLLING */

/*
 ******************************************************************************
 * TIMER FUNCTIONS
 ******************************************************************************
 *
 * Timer Implementation Notes:
 *
 * The RFAL library uses timers for NFC protocol timing. Timers are created
 * with a duration in milliseconds and can be checked for expiration.
 *
 * Implementation uses k_uptime_get() which returns monotonically increasing
 * milliseconds since boot. We store the expiration time and use signed
 * comparison to handle 32-bit wraparound correctly.
 *
 * Wraparound Handling:
 * - k_uptime_get() wraps at ~49 days (2^32 ms)
 * - Using signed comparison handles wraparound for durations up to ~24 days
 * - NFC timings are typically microseconds to seconds, so this is safe
 */

/**
 * @brief Create a timer that expires after specified milliseconds
 *
 * @param time_ms Duration in milliseconds (max ~24 days for wraparound safety)
 * @return Timer handle (expiration timestamp)
 */
uint32_t rfal_platform_timer_create(uint16_t time_ms)
{
    uint32_t now = (uint32_t)k_uptime_get();
    return now + (uint32_t)time_ms;
}

/**
 * @brief Check if a timer has expired
 *
 * Uses signed comparison to correctly handle 32-bit wraparound.
 * The comparison (now - expiration) >= 0 works because:
 * - If not expired: (now - expiration) is negative (large positive unsigned)
 * - If expired: (now - expiration) is positive or zero
 *
 * @param timer Timer handle from rfal_platform_timer_create()
 * @return true if timer has expired, false otherwise
 */
bool rfal_platform_timer_is_expired(uint32_t timer)
{
    uint32_t now = (uint32_t)k_uptime_get();
    int32_t diff = (int32_t)(now - timer);
    return (diff >= 0);
}

/**
 * @brief Get remaining time until timer expiration
 *
 * @param timer Timer handle from rfal_platform_timer_create()
 * @return Remaining milliseconds, 0 if already expired
 */
uint32_t rfal_platform_timer_get_remaining(uint32_t timer)
{
    uint32_t now = (uint32_t)k_uptime_get();
    int32_t remaining = (int32_t)(timer - now);

    if (remaining <= 0) {
        return 0U;
    }
    return (uint32_t)remaining;
}

/*
 ******************************************************************************
 * SPI FUNCTIONS
 ******************************************************************************
 */

void rfal_platform_spi_select(void)
{
    if (st25rx00_dev == NULL) {
        return;
    }

    const struct st25rx00_config *config = st25rx00_dev->config;

    /* Drive CS LOW (active) - gpio_pin_set_dt handles GPIO_ACTIVE_LOW flag */
    gpio_pin_set_dt(&config->cs_gpio, 1);
}

void rfal_platform_spi_deselect(void)
{
    if (st25rx00_dev == NULL) {
        return;
    }

    const struct st25rx00_config *config = st25rx00_dev->config;

    /* Drive CS HIGH (inactive) - gpio_pin_set_dt handles GPIO_ACTIVE_LOW flag */
    gpio_pin_set_dt(&config->cs_gpio, 0);
}

int rfal_platform_spi_txrx(const uint8_t *txBuf, uint8_t *rxBuf, uint16_t len)
{
    if (st25rx00_dev == NULL) {
        return -ENODEV;
    }

    const struct st25rx00_config *config = st25rx00_dev->config;
    int ret;

    /* Handle different transfer modes */
    if (txBuf != NULL && rxBuf != NULL) {
        /* Full duplex transfer */
        if (txBuf == rxBuf) {
            /* Same buffer for TX and RX (ST25R_COM_SINGLETXRX mode)
             * Copy TX data to temp buffer, transceive, then the original
             * buffer will have the RX data (which is what RFAL expects) */
            if (len > SPI_TEMP_BUF_SIZE) {
                LOG_ERR("SPI buffer too large: %d > %d", len, SPI_TEMP_BUF_SIZE);
                return -ENOMEM;
            }
            memcpy(spi_temp_buf, txBuf, len);

            struct spi_buf tx_buf = { .buf = spi_temp_buf, .len = len };
            struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };
            struct spi_buf rx_buf = { .buf = rxBuf, .len = len };
            struct spi_buf_set rx_bufs = { .buffers = &rx_buf, .count = 1 };

            ret = spi_transceive_dt(&config->spi, &tx_bufs, &rx_bufs);
        } else {
            /* Different buffers - standard full duplex */
            struct spi_buf tx_buf = { .buf = (void *)txBuf, .len = len };
            struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };
            struct spi_buf rx_buf = { .buf = rxBuf, .len = len };
            struct spi_buf_set rx_bufs = { .buffers = &rx_buf, .count = 1 };

            ret = spi_transceive_dt(&config->spi, &tx_bufs, &rx_bufs);
        }
    } else if (txBuf != NULL) {
        /* TX only */
        struct spi_buf tx_buf = { .buf = (void *)txBuf, .len = len };
        struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };
        ret = spi_write_dt(&config->spi, &tx_bufs);
    } else if (rxBuf != NULL) {
        /* RX only */
        struct spi_buf rx_buf = { .buf = rxBuf, .len = len };
        struct spi_buf_set rx_bufs = { .buffers = &rx_buf, .count = 1 };
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
 *
 * LED control uses the device tree configuration from the st25rx00 node.
 * All LED GPIOs are optional - if not defined in device tree, the LED
 * functions become no-ops.
 */

static const struct gpio_dt_spec *get_led_gpio(uint32_t pin)
{
    if (st25rx00_dev == NULL) {
        return NULL;
    }

    const struct st25rx00_config *config = st25rx00_dev->config;

    switch (pin) {
    case PLATFORM_LED_RX_PIN:
        return (config->led_rx_gpio.port != NULL) ? &config->led_rx_gpio : NULL;
    case PLATFORM_LED_FIELD_PIN:
        return (config->led_field_gpio.port != NULL) ? &config->led_field_gpio : NULL;
    case PLATFORM_LED_CONN_PIN:
        return (config->led_conn_gpio.port != NULL) ? &config->led_conn_gpio : NULL;
    default:
        return NULL;
    }
}

void rfal_platform_leds_init(void)
{
    /* LEDs are already configured in rfal_platform_init() */
}

void rfal_platform_led_on(uint32_t pin)
{
    const struct gpio_dt_spec *gpio = get_led_gpio(pin);
    if (gpio != NULL) {
        gpio_pin_set_dt(gpio, 1);
    }
}

void rfal_platform_led_off(uint32_t pin)
{
    const struct gpio_dt_spec *gpio = get_led_gpio(pin);
    if (gpio != NULL) {
        gpio_pin_set_dt(gpio, 0);
    }
}

void rfal_platform_led_toggle(uint32_t pin)
{
    const struct gpio_dt_spec *gpio = get_led_gpio(pin);
    if (gpio != NULL) {
        gpio_pin_toggle_dt(gpio);
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

    /* Don't halt - allow caller to handle error */
}

/*
 ******************************************************************************
 * IRQ WAIT API (for thread-based NFC processing)
 ******************************************************************************
 */

int st25rx00_wait_for_irq(k_timeout_t timeout)
{
#ifdef CONFIG_ST25RX00_IRQ_POLLING
    /* In polling mode, just yield briefly - no semaphore */
    k_sleep(K_MSEC(1));
    return 0;
#else
    /* Wait on IRQ semaphore - woken by GPIO ISR */
    return k_sem_take(&irq_sem, timeout);
#endif
}
