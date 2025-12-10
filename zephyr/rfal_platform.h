/*
 * Copyright (c) 2025 NeuraSignal Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file rfal_platform.h
 * @brief RFAL Platform Abstraction Layer for Zephyr RTOS
 *
 * This file implements the platform-specific functions required by the
 * ST RFAL (RF Abstraction Layer) library for Zephyr RTOS.
 */

#ifndef RFAL_PLATFORM_H
#define RFAL_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/*
 ******************************************************************************
 * GLOBAL DEFINES
 ******************************************************************************
 */

/* ST25R200 is defined via CMakeLists.txt compile definitions */

/* Use SPI interface (not I2C) */
#define RFAL_USE_I2C                    0

/* Single SPI transaction mode for better performance */
#define ST25R_COM_SINGLETXRX

/*
 ******************************************************************************
 * RFAL FEATURE CONFIGURATION (from Kconfig)
 ******************************************************************************
 */

/* Convert Kconfig booleans to RFAL true/false format */
#ifdef CONFIG_ST25RX00_FEATURE_LISTEN_MODE
#define RFAL_FEATURE_LISTEN_MODE               true
#else
#define RFAL_FEATURE_LISTEN_MODE               false
#endif

#ifdef CONFIG_ST25RX00_FEATURE_WAKEUP_MODE
#define RFAL_FEATURE_WAKEUP_MODE               true
#else
#define RFAL_FEATURE_WAKEUP_MODE               false
#endif

#ifdef CONFIG_ST25RX00_FEATURE_LOWPOWER_MODE
#define RFAL_FEATURE_LOWPOWER_MODE             true
#else
#define RFAL_FEATURE_LOWPOWER_MODE             false
#endif

#ifdef CONFIG_ST25RX00_FEATURE_NFCA
#define RFAL_FEATURE_NFCA                      true
#else
#define RFAL_FEATURE_NFCA                      false
#endif

#ifdef CONFIG_ST25RX00_FEATURE_NFCB
#define RFAL_FEATURE_NFCB                      true
#else
#define RFAL_FEATURE_NFCB                      false
#endif

#ifdef CONFIG_ST25RX00_FEATURE_NFCF
#define RFAL_FEATURE_NFCF                      true
#else
#define RFAL_FEATURE_NFCF                      false
#endif

#ifdef CONFIG_ST25RX00_FEATURE_NFCV
#define RFAL_FEATURE_NFCV                      true
#else
#define RFAL_FEATURE_NFCV                      false
#endif

#ifdef CONFIG_ST25RX00_FEATURE_T1T
#define RFAL_FEATURE_T1T                       true
#else
#define RFAL_FEATURE_T1T                       false
#endif

#ifdef CONFIG_ST25RX00_FEATURE_T2T
#define RFAL_FEATURE_T2T                       true
#else
#define RFAL_FEATURE_T2T                       false
#endif

#ifdef CONFIG_ST25RX00_FEATURE_T4T
#define RFAL_FEATURE_T4T                       true
#else
#define RFAL_FEATURE_T4T                       false
#endif

#ifdef CONFIG_ST25RX00_FEATURE_ST25TB
#define RFAL_FEATURE_ST25TB                    true
#else
#define RFAL_FEATURE_ST25TB                    false
#endif

#ifdef CONFIG_ST25RX00_FEATURE_ST25XV
#define RFAL_FEATURE_ST25xV                    true
#else
#define RFAL_FEATURE_ST25xV                    false
#endif

/* DPO (Dynamic Power Output) - not supported on ST25R100/R200 */
#define RFAL_FEATURE_DPO                       false

#ifdef CONFIG_ST25RX00_FEATURE_ISO_DEP
#define RFAL_FEATURE_ISO_DEP                   true
#define RFAL_FEATURE_ISO_DEP_POLL              true
#else
#define RFAL_FEATURE_ISO_DEP                   false
#define RFAL_FEATURE_ISO_DEP_POLL              false
#endif

/* ISO-DEP Listen requires Listen Mode */
#if defined(CONFIG_ST25RX00_FEATURE_ISO_DEP) && defined(CONFIG_ST25RX00_FEATURE_LISTEN_MODE)
#define RFAL_FEATURE_ISO_DEP_LISTEN            true
#else
#define RFAL_FEATURE_ISO_DEP_LISTEN            false
#endif

#ifdef CONFIG_ST25RX00_FEATURE_NFC_DEP
#define RFAL_FEATURE_NFC_DEP                   true
#define RFAL_FEATURE_NFC_DEP_POLL              true
#else
#define RFAL_FEATURE_NFC_DEP                   false
#define RFAL_FEATURE_NFC_DEP_POLL              false
#endif

/* NFC-DEP Listen requires Listen Mode */
#if defined(CONFIG_ST25RX00_FEATURE_NFC_DEP) && defined(CONFIG_ST25RX00_FEATURE_LISTEN_MODE)
#define RFAL_FEATURE_NFC_DEP_LISTEN            true
#else
#define RFAL_FEATURE_NFC_DEP_LISTEN            false
#endif

/* DLMA requires Listen Mode - ST25R100/R200 don't support it */
#define RFAL_FEATURE_DLMA                      false

#ifdef CONFIG_ST25RX00_FEATURE_CD
#define RFAL_FEATURE_CD                        true
#else
#define RFAL_FEATURE_CD                        false
#endif

/*
 ******************************************************************************
 * BUFFER SIZE CONFIGURATION (from Kconfig)
 ******************************************************************************
 */
#define RFAL_FEATURE_ISO_DEP_IBLOCK_MAX_LEN    CONFIG_ST25RX00_ISO_DEP_IBLOCK_MAX_LEN
#define RFAL_FEATURE_ISO_DEP_APDU_MAX_LEN      CONFIG_ST25RX00_ISO_DEP_APDU_MAX_LEN
#define RFAL_FEATURE_NFC_DEP_BLOCK_MAX_LEN     254U    /*!< NFC-DEP Block/Payload length */
#define RFAL_FEATURE_NFC_DEP_PDU_MAX_LEN       512U    /*!< NFC-DEP PDU max length */
#define RFAL_FEATURE_NFC_RF_BUF_LEN            CONFIG_ST25RX00_NFC_RF_BUF_LEN

/* GPIO port/pin definitions - placeholder values, actual from device tree */
#define ST25R_INT_PORT                  NULL
#define ST25R_INT_PIN                   0U
#define ST25R_RESET_PORT                NULL
#define ST25R_RESET_PIN                 0U

/*
 ******************************************************************************
 * GLOBAL MACROS
 ******************************************************************************
 */

#define platformProtectST25RComm()              rfal_platform_protect_comm()
#define platformUnprotectST25RComm()            rfal_platform_unprotect_comm()
#define platformProtectST25RIrqStatus()         rfal_platform_protect_irq_status()
#define platformUnprotectST25RIrqStatus()       rfal_platform_unprotect_irq_status()
#define platformProtectWorker()                 /* Not needed in single-threaded polling */
#define platformUnprotectWorker()               /* Not needed in single-threaded polling */

#define platformIrqST25RPinInitialize()         rfal_platform_irq_init()
#define platformIrqST25RSetCallback(cb)         rfal_platform_irq_set_callback(cb)

#define platformLedsInitialize()                /* LEDs initialized by Zephyr */
#define platformLedOff(port, pin)               rfal_platform_led_off()
#define platformLedOn(port, pin)                rfal_platform_led_on()
#define platformLedToggle(port, pin)            /* Not implemented */

#define platformGpioSet(port, pin)              rfal_platform_gpio_set(pin)
#define platformGpioClear(port, pin)            rfal_platform_gpio_clear(pin)
#define platformGpioIsHigh(port, pin)           rfal_platform_gpio_is_high(pin)
#define platformGpioIsLow(port, pin)            (!rfal_platform_gpio_is_high(pin))

#define platformTimerCreate(t)                  rfal_platform_timer_create(t)
#define platformTimerIsExpired(t)               rfal_platform_timer_is_expired(t)
#define platformTimerDestroy(t)                 /* No action needed */
#define platformTimerGetRemaining(t)            rfal_platform_timer_get_remaining(t)

#define platformDelay(t)                        k_msleep(t)
#define platformGetSysTick()                    ((uint32_t)k_uptime_get())

#define platformErrorHandle()                   rfal_platform_error_handle()

#define platformSpiSelect()                     rfal_platform_spi_select()
#define platformSpiDeselect()                   rfal_platform_spi_deselect()
#define platformSpiTxRx(txBuf, rxBuf, len)      rfal_platform_spi_txrx(txBuf, rxBuf, len)

#define platformLog(...)                        /* Disabled by default */

#ifndef platformAssert
#define platformAssert(exp)                     __ASSERT(exp, "RFAL Assert Failed")
#endif

/* Memory operations */
#define platformFlushCache()                    /* No cache to flush */

/*
 ******************************************************************************
 * GLOBAL TYPES
 ******************************************************************************
 */

/** ST25Rx00 device configuration (from device tree) */
struct st25rx00_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec cs_gpio;           /**< Chip select (manually controlled) */
    struct gpio_dt_spec irq_gpio;
    struct gpio_dt_spec reset_gpio;
    struct gpio_dt_spec led_field_gpio;
    struct gpio_dt_spec led_rx_gpio;
    struct gpio_dt_spec led_tx_gpio;
    struct gpio_dt_spec led_error_gpio;
};

/** ST25Rx00 device runtime data */
struct st25rx00_data {
    struct gpio_callback irq_cb;
    bool initialized;
};

/*
 ******************************************************************************
 * GLOBAL FUNCTION PROTOTYPES
 ******************************************************************************
 */

/**
 * @brief Initialize the platform layer
 * @param dev Pointer to device structure
 * @return 0 on success, negative errno on failure
 */
int rfal_platform_init(const struct device *dev);

/**
 * @brief Protect communication with ST25R (disable interrupts)
 */
void rfal_platform_protect_comm(void);

/**
 * @brief Unprotect communication with ST25R (enable interrupts)
 */
void rfal_platform_unprotect_comm(void);

/**
 * @brief Protect IRQ status variable access
 */
void rfal_platform_protect_irq_status(void);

/**
 * @brief Unprotect IRQ status variable access
 */
void rfal_platform_unprotect_irq_status(void);

/**
 * @brief Initialize IRQ pin and configure interrupt
 */
void rfal_platform_irq_init(void);

/**
 * @brief Set IRQ callback function
 * @param cb Callback function pointer
 */
void rfal_platform_irq_set_callback(void (*cb)(void));

/**
 * @brief Set GPIO pin high (reset pin)
 * @param pin Pin number (ignored, uses device tree)
 */
void rfal_platform_gpio_set(uint32_t pin);

/**
 * @brief Set GPIO pin low (reset pin)
 * @param pin Pin number (ignored, uses device tree)
 */
void rfal_platform_gpio_clear(uint32_t pin);

/**
 * @brief Check if GPIO pin is high (IRQ pin)
 * @param pin Pin number (ignored, uses device tree)
 * @return true if pin is high, false otherwise
 */
bool rfal_platform_gpio_is_high(uint32_t pin);

/**
 * @brief Create/start a timer
 * @param time_ms Duration in milliseconds
 * @return Timer handle (expiration time in ms)
 */
uint32_t rfal_platform_timer_create(uint16_t time_ms);

/**
 * @brief Check if timer has expired
 * @param timer Timer handle
 * @return true if expired, false otherwise
 */
bool rfal_platform_timer_is_expired(uint32_t timer);

/**
 * @brief Get remaining time until timer expiration
 * @param timer Timer handle
 * @return Remaining time in milliseconds (0 if expired)
 */
uint32_t rfal_platform_timer_get_remaining(uint32_t timer);

/**
 * @brief Select SPI device (assert CS)
 */
void rfal_platform_spi_select(void);

/**
 * @brief Deselect SPI device (deassert CS)
 */
void rfal_platform_spi_deselect(void);

/**
 * @brief SPI transfer function
 * @param txBuf Transmit buffer (can be NULL for RX-only)
 * @param rxBuf Receive buffer (can be NULL for TX-only)
 * @param len Length of transfer
 * @return 0 on success, negative errno on failure
 */
int rfal_platform_spi_txrx(const uint8_t *txBuf, uint8_t *rxBuf, uint16_t len);

/**
 * @brief Platform error handler
 */
void rfal_platform_error_handle(void);

/**
 * @brief Turn on field LED
 */
void rfal_platform_led_on(void);

/**
 * @brief Turn off field LED
 */
void rfal_platform_led_off(void);

#ifdef __cplusplus
}
#endif

#endif /* RFAL_PLATFORM_H */
