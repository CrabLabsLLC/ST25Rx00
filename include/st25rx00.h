/*
 * Copyright (c) 2025 NeuraSignal Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file st25rx00.h
 * @brief ST25Rx00 NFC Reader Public API
 *
 * This header provides a clean application-facing API for the ST25Rx00
 * NFC reader driver. Applications can use these functions for high-level
 * NFC operations or access the RFAL library directly for advanced use.
 */

#ifndef ST25RX00_H
#define ST25RX00_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

/* Include RFAL headers for direct access to RFAL API */
#include "rfal_rf.h"
#include "rfal_nfc.h"
#include "rfal_nfca.h"
#include "rfal_nfcb.h"
#include "rfal_nfcv.h"
#include "rfal_isoDep.h"

/*
 ******************************************************************************
 * CONSTANTS
 ******************************************************************************
 */

/** Maximum UID length for NFC tags */
#define ST25RX00_MAX_UID_LEN            10

/** Maximum NDEF message size supported */
#define ST25RX00_MAX_NDEF_SIZE          256

/*
 ******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************
 */

/** NFC technology types */
enum st25rx00_tech {
    ST25RX00_TECH_NONE = 0,
    ST25RX00_TECH_NFCA,     /**< NFC-A (ISO14443A) */
    ST25RX00_TECH_NFCB,     /**< NFC-B (ISO14443B) */
    ST25RX00_TECH_NFCF,     /**< NFC-F (FeliCa) */
    ST25RX00_TECH_NFCV,     /**< NFC-V (ISO15693) */
};

/** NFC device type (for NFC-A) */
enum st25rx00_nfca_type {
    ST25RX00_NFCA_T1T,      /**< Type 1 Tag (Topaz) */
    ST25RX00_NFCA_T2T,      /**< Type 2 Tag (NTAG, Ultralight) */
    ST25RX00_NFCA_T4T,      /**< Type 4 Tag (ISO-DEP) */
    ST25RX00_NFCA_NFCDEP,   /**< NFC-DEP (P2P) */
    ST25RX00_NFCA_OTHER,    /**< Other/unknown */
};

/** Tag information structure */
struct st25rx00_tag_info {
    enum st25rx00_tech tech;
    uint8_t uid[ST25RX00_MAX_UID_LEN];
    uint8_t uid_len;

    /* NFC-A specific */
    uint16_t sens_res;      /**< ATQA for NFC-A */
    uint8_t sel_res;        /**< SAK for NFC-A */
    enum st25rx00_nfca_type nfca_type;

    /* Raw RFAL device - for advanced use */
    union {
        rfalNfcaListenDevice nfca;
        rfalNfcbListenDevice nfcb;
        rfalNfcvListenDevice nfcv;
    } dev;
};

/** Discovery configuration */
struct st25rx00_discover_cfg {
    bool detect_nfca;       /**< Enable NFC-A detection */
    bool detect_nfcb;       /**< Enable NFC-B detection */
    bool detect_nfcf;       /**< Enable NFC-F detection */
    bool detect_nfcv;       /**< Enable NFC-V detection */
    uint16_t timeout_ms;    /**< Discovery timeout in ms */
    uint8_t max_devices;    /**< Maximum devices to detect */
};

/*
 ******************************************************************************
 * PUBLIC API FUNCTIONS
 ******************************************************************************
 */

/**
 * @brief Initialize the high-level NFC API
 *
 * This function initializes the RFAL NFC layer and prepares for
 * polling operations. The driver must already be initialized via
 * device tree.
 *
 * @return 0 on success, negative errno on failure
 */
int st25rx00_nfc_init(void);

/**
 * @brief Deinitialize the NFC API
 *
 * Shuts down the NFC stack and puts the reader in low-power mode.
 *
 * @return 0 on success, negative errno on failure
 */
int st25rx00_nfc_deinit(void);

/**
 * @brief Start NFC discovery (polling)
 *
 * Starts polling for NFC tags with specified configuration.
 * This is non-blocking - use st25rx00_worker() to process.
 *
 * @param cfg Discovery configuration (NULL for defaults)
 * @return 0 on success, negative errno on failure
 */
int st25rx00_discover_start(const struct st25rx00_discover_cfg *cfg);

/**
 * @brief Stop NFC discovery
 *
 * Stops the polling operation and returns to idle state.
 *
 * @return 0 on success, negative errno on failure
 */
int st25rx00_discover_stop(void);

/**
 * @brief NFC state machine worker
 *
 * This function must be called periodically to process the NFC
 * state machine. Call this in a loop or work queue handler.
 *
 * @param tag_info Pointer to structure to fill with tag info (can be NULL)
 * @return true if a tag was detected and activated, false otherwise
 */
bool st25rx00_worker(struct st25rx00_tag_info *tag_info);

/**
 * @brief Get currently activated device
 *
 * Returns information about the currently activated NFC device.
 *
 * @param tag_info Pointer to structure to fill
 * @return 0 on success, -ENODEV if no device active
 */
int st25rx00_get_active_device(struct st25rx00_tag_info *tag_info);

/**
 * @brief Deactivate current device
 *
 * Releases the currently activated tag and returns to discovery.
 *
 * @return 0 on success, negative errno on failure
 */
int st25rx00_deactivate(void);

/**
 * @brief Control RF field
 *
 * Manually enable or disable the RF field.
 *
 * @param enable true to enable field, false to disable
 * @return 0 on success, negative errno on failure
 */
int st25rx00_field_control(bool enable);

/**
 * @brief Transceive raw data with activated tag
 *
 * Send and receive raw data with the currently activated tag.
 * This is useful for custom protocols or advanced operations.
 *
 * @param tx_buf Transmit buffer
 * @param tx_len Transmit length in bytes
 * @param rx_buf Receive buffer
 * @param rx_buf_len Receive buffer size
 * @param rx_len Pointer to store actual received length
 * @param timeout_ms Timeout in milliseconds
 * @return 0 on success, negative errno on failure
 */
int st25rx00_transceive(const uint8_t *tx_buf, size_t tx_len,
                        uint8_t *rx_buf, size_t rx_buf_len,
                        size_t *rx_len, uint32_t timeout_ms);

/**
 * @brief Read ISO-DEP data
 *
 * Exchange ISO-DEP (ISO14443-4) APDUs with an activated ISO-DEP tag.
 *
 * @param tx_buf APDU command to send
 * @param tx_len Command length
 * @param rx_buf Buffer for response
 * @param rx_buf_len Response buffer size
 * @param rx_len Pointer to store actual response length
 * @return 0 on success, negative errno on failure
 */
int st25rx00_isodep_transceive(const uint8_t *tx_buf, size_t tx_len,
                               uint8_t *rx_buf, size_t rx_buf_len,
                               size_t *rx_len);

/*
 ******************************************************************************
 * RFAL DIRECT ACCESS
 ******************************************************************************
 *
 * For advanced use cases, applications can directly access the RFAL API
 * by including the appropriate rfal_*.h headers. The driver initialization
 * sets up the platform layer, so RFAL functions can be called directly.
 *
 * Example:
 *   #include "rfal_nfc.h"
 *
 *   rfalNfcDiscover(&discParam);
 *   while (rfalNfcGetState() != RFAL_NFC_STATE_ACTIVATED) {
 *       rfalNfcWorker();
 *   }
 */

#ifdef __cplusplus
}
#endif

#endif /* ST25RX00_H */
