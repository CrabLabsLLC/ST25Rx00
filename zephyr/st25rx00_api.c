/*
 * Copyright (c) 2025 NeuraSignal Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file st25rx00_api.c
 * @brief ST25Rx00 NFC Reader Public API Implementation
 */

#include "st25rx00.h"
#include "rfal_nfc.h"
#include "rfal_rf.h"
#include "rfal_isoDep.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(st25rx00_api, CONFIG_ST25RX00_LOG_LEVEL);

/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */

/* NFC discovery parameters */
static rfalNfcDiscoverParam discover_param;

/* Current NFC device */
static rfalNfcDevice *nfc_device;

/* API initialization state */
static bool api_initialized = false;

/* ISO-DEP buffers - protected by requiring single-threaded access to API */
static rfalIsoDepApduBufFormat isodep_tx_apdu_buf;
static rfalIsoDepApduBufFormat isodep_rx_apdu_buf;
static rfalIsoDepBufFormat isodep_tmp_buf;

/*
 ******************************************************************************
 * HELPER FUNCTIONS
 ******************************************************************************
 */

static enum st25rx00_nfca_type get_nfca_type(uint8_t sel_res)
{
    /* Determine NFC-A tag type from SAK (SEL_RES) */
    if ((sel_res & 0x60) == 0x00) {
        return ST25RX00_NFCA_T2T;  /* Type 2 Tag */
    } else if ((sel_res & 0x20) == 0x20) {
        return ST25RX00_NFCA_T4T;  /* Type 4 Tag (ISO-DEP) */
    } else if ((sel_res & 0x40) == 0x40) {
        return ST25RX00_NFCA_NFCDEP;  /* NFC-DEP */
    }
    return ST25RX00_NFCA_OTHER;
}

/**
 * @brief Safely copy UID with bounds checking
 *
 * @param dst Destination buffer
 * @param dst_size Size of destination buffer
 * @param src Source buffer
 * @param src_len Length of source data
 * @return Actual number of bytes copied
 */
static uint8_t safe_uid_copy(uint8_t *dst, size_t dst_size,
                             const uint8_t *src, size_t src_len)
{
    size_t copy_len = (src_len <= dst_size) ? src_len : dst_size;
    memcpy(dst, src, copy_len);
    return (uint8_t)copy_len;
}

/*
 ******************************************************************************
 * PUBLIC API IMPLEMENTATION
 ******************************************************************************
 */

int st25rx00_nfc_init(void)
{
    ReturnCode err;

    if (api_initialized) {
        return 0;  /* Already initialized */
    }

    /* Initialize RFAL NFC layer */
    err = rfalNfcInitialize();
    if (err != RFAL_ERR_NONE) {
        LOG_ERR("RFAL NFC initialization failed: %d", err);
        return -EIO;
    }

    /* Set default discovery parameters */
    memset(&discover_param, 0, sizeof(discover_param));
    discover_param.compMode = RFAL_COMPLIANCE_MODE_NFC;
    discover_param.techs2Find = RFAL_NFC_POLL_TECH_A |
                                RFAL_NFC_POLL_TECH_B |
                                RFAL_NFC_POLL_TECH_V;
    discover_param.totalDuration = 1000;  /* 1 second */
    discover_param.devLimit = 1;
    discover_param.wakeupEnabled = false;
    discover_param.wakeupConfigDefault = true;
    discover_param.nfcfBR = RFAL_BR_212;
    discover_param.ap2pBR = RFAL_BR_424;
    discover_param.notifyCb = NULL;

    api_initialized = true;
    LOG_INF("ST25Rx00 NFC API initialized");
    return 0;
}

int st25rx00_nfc_deinit(void)
{
    if (!api_initialized) {
        return 0;
    }

    rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_IDLE);
    api_initialized = false;

    LOG_INF("ST25Rx00 NFC API deinitialized");
    return 0;
}

int st25rx00_discover_start(const struct st25rx00_discover_cfg *cfg)
{
    ReturnCode err;

    if (!api_initialized) {
        LOG_ERR("API not initialized");
        return -EINVAL;
    }

    /* Apply custom configuration if provided */
    if (cfg != NULL) {
        discover_param.techs2Find = 0;

        if (cfg->detect_nfca) {
            discover_param.techs2Find |= RFAL_NFC_POLL_TECH_A;
        }
        if (cfg->detect_nfcb) {
            discover_param.techs2Find |= RFAL_NFC_POLL_TECH_B;
        }
        if (cfg->detect_nfcf) {
            discover_param.techs2Find |= RFAL_NFC_POLL_TECH_F;
        }
        if (cfg->detect_nfcv) {
            discover_param.techs2Find |= RFAL_NFC_POLL_TECH_V;
        }

        if (cfg->timeout_ms > 0) {
            discover_param.totalDuration = cfg->timeout_ms;
        }
        if (cfg->max_devices > 0) {
            discover_param.devLimit = cfg->max_devices;
        }
    }

    err = rfalNfcDiscover(&discover_param);
    if (err != RFAL_ERR_NONE) {
        LOG_ERR("Failed to start discovery: %d", err);
        return -EIO;
    }

    LOG_DBG("NFC discovery started");
    return 0;
}

int st25rx00_discover_stop(void)
{
    ReturnCode err;

    err = rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_IDLE);
    if (err != RFAL_ERR_NONE) {
        LOG_WRN("Stop discovery failed: %d", err);
        return -EIO;
    }

    nfc_device = NULL;
    LOG_DBG("NFC discovery stopped");
    return 0;
}

bool st25rx00_worker(struct st25rx00_tag_info *tag_info)
{
    rfalNfcState state;
    ReturnCode err;

    /* Run the NFC state machine */
    rfalNfcWorker();

    /* Get current state */
    state = rfalNfcGetState();

    /* Check if we have found and activated a device */
    if (state == RFAL_NFC_STATE_ACTIVATED) {
        err = rfalNfcGetActiveDevice(&nfc_device);
        if (err != RFAL_ERR_NONE || nfc_device == NULL) {
            LOG_ERR("Failed to get active device: %d", err);
            return false;
        }

        /* Fill tag_info if provided */
        if (tag_info != NULL) {
            memset(tag_info, 0, sizeof(*tag_info));

            switch (nfc_device->type) {
            case RFAL_NFC_LISTEN_TYPE_NFCA:
                tag_info->tech = ST25RX00_TECH_NFCA;
                tag_info->uid_len = safe_uid_copy(
                    tag_info->uid, sizeof(tag_info->uid),
                    nfc_device->dev.nfca.nfcId1,
                    nfc_device->dev.nfca.nfcId1Len);
                tag_info->sens_res = nfc_device->dev.nfca.sensRes.anticollisionInfo;
                tag_info->sel_res = nfc_device->dev.nfca.selRes.sak;
                tag_info->nfca_type = get_nfca_type(tag_info->sel_res);
                memcpy(&tag_info->dev.nfca, &nfc_device->dev.nfca,
                       sizeof(rfalNfcaListenDevice));
                LOG_INF("NFC-A tag detected, UID len: %d, SAK: 0x%02X",
                        tag_info->uid_len, tag_info->sel_res);
                break;

            case RFAL_NFC_LISTEN_TYPE_NFCB:
                tag_info->tech = ST25RX00_TECH_NFCB;
                tag_info->uid_len = safe_uid_copy(
                    tag_info->uid, sizeof(tag_info->uid),
                    nfc_device->dev.nfcb.sensbRes.nfcid0,
                    RFAL_NFCB_NFCID0_LEN);
                memcpy(&tag_info->dev.nfcb, &nfc_device->dev.nfcb,
                       sizeof(rfalNfcbListenDevice));
                LOG_INF("NFC-B tag detected");
                break;

            case RFAL_NFC_LISTEN_TYPE_NFCV:
                tag_info->tech = ST25RX00_TECH_NFCV;
                tag_info->uid_len = safe_uid_copy(
                    tag_info->uid, sizeof(tag_info->uid),
                    nfc_device->dev.nfcv.InvRes.UID,
                    RFAL_NFCV_UID_LEN);
                memcpy(&tag_info->dev.nfcv, &nfc_device->dev.nfcv,
                       sizeof(rfalNfcvListenDevice));
                LOG_INF("NFC-V tag detected");
                break;

            default:
                tag_info->tech = ST25RX00_TECH_NONE;
                LOG_WRN("Unknown NFC technology: %d", nfc_device->type);
                break;
            }
        }

        return true;
    }

    return false;
}

int st25rx00_get_active_device(struct st25rx00_tag_info *tag_info)
{
    if (nfc_device == NULL) {
        return -ENODEV;
    }

    if (tag_info == NULL) {
        return -EINVAL;
    }

    /* Re-populate tag info from current device */
    return st25rx00_worker(tag_info) ? 0 : -ENODEV;
}

int st25rx00_deactivate(void)
{
    ReturnCode err;

    err = rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_DISCOVERY);
    if (err != RFAL_ERR_NONE) {
        LOG_ERR("Deactivation failed: %d", err);
        return -EIO;
    }

    nfc_device = NULL;
    return 0;
}

int st25rx00_field_control(bool enable)
{
    ReturnCode err;

    if (enable) {
        err = rfalFieldOnAndStartGT();
    } else {
        err = rfalFieldOff();
    }

    if (err != RFAL_ERR_NONE) {
        LOG_ERR("Field control failed: %d", err);
        return -EIO;
    }

    LOG_DBG("RF field %s", enable ? "enabled" : "disabled");
    return 0;
}

int st25rx00_transceive(const uint8_t *tx_buf, size_t tx_len,
                        uint8_t *rx_buf, size_t rx_buf_len,
                        size_t *rx_len, uint32_t timeout_ms)
{
    ReturnCode err;
    uint16_t actual_rx_len = 0;

    if (nfc_device == NULL) {
        return -ENODEV;
    }

    err = rfalTransceiveBlockingTxRx(
        (uint8_t *)tx_buf,
        tx_len,
        rx_buf,
        rx_buf_len,
        &actual_rx_len,
        RFAL_TXRX_FLAGS_DEFAULT,
        rfalConvMsTo1fc(timeout_ms)
    );

    if (err != RFAL_ERR_NONE) {
        LOG_ERR("Transceive failed: %d", err);
        return -EIO;
    }

    if (rx_len != NULL) {
        *rx_len = actual_rx_len;
    }

    return 0;
}

int st25rx00_isodep_transceive(const uint8_t *tx_buf, size_t tx_len,
                               uint8_t *rx_buf, size_t rx_buf_len,
                               size_t *rx_len)
{
    ReturnCode err;
    uint16_t actual_rx_len = 0;
    rfalIsoDepApduTxRxParam param;

    if (nfc_device == NULL) {
        return -ENODEV;
    }

    /* Check if device supports ISO-DEP */
    if (nfc_device->rfInterface != RFAL_NFC_INTERFACE_ISODEP) {
        LOG_ERR("Device does not support ISO-DEP");
        return -ENOTSUP;
    }

    /* Check TX data fits in buffer */
    if (tx_len > sizeof(isodep_tx_apdu_buf.apdu)) {
        LOG_ERR("TX data too large: %zu > %zu", tx_len,
                sizeof(isodep_tx_apdu_buf.apdu));
        return -ENOMEM;
    }

    /* Copy TX data to APDU buffer */
    memcpy(isodep_tx_apdu_buf.apdu, tx_buf, tx_len);

    /* Setup transceive parameters using ISO-DEP device info from proto union */
    memset(&param, 0, sizeof(param));
    param.txBuf = &isodep_tx_apdu_buf;
    param.txBufLen = tx_len;
    param.rxBuf = &isodep_rx_apdu_buf;
    param.rxLen = &actual_rx_len;
    param.tmpBuf = &isodep_tmp_buf;
    param.FWT = nfc_device->proto.isoDep.info.FWT;
    param.dFWT = nfc_device->proto.isoDep.info.dFWT;
    param.FSx = nfc_device->proto.isoDep.info.FSx;
    param.ourFSx = RFAL_ISODEP_FSX_256;
    param.DID = nfc_device->proto.isoDep.info.DID;

    /* Start async transceive */
    err = rfalIsoDepStartApduTransceive(param);
    if (err != RFAL_ERR_NONE) {
        LOG_ERR("ISO-DEP start transceive failed: %d", err);
        return -EIO;
    }

    /* Poll for completion */
    do {
        rfalWorker();
        err = rfalIsoDepGetApduTransceiveStatus();
    } while (err == RFAL_ERR_BUSY);

    if (err != RFAL_ERR_NONE) {
        LOG_ERR("ISO-DEP transceive failed: %d", err);
        return -EIO;
    }

    /* Check RX buffer size and copy received data */
    if (actual_rx_len > rx_buf_len) {
        LOG_ERR("RX buffer too small: need %u, have %zu", actual_rx_len, rx_buf_len);
        return -ENOMEM;
    }
    memcpy(rx_buf, isodep_rx_apdu_buf.apdu, actual_rx_len);

    if (rx_len != NULL) {
        *rx_len = actual_rx_len;
    }

    return 0;
}
