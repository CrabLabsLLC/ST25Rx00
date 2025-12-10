/*
 * Copyright (c) 2025 NeuraSignal Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file st25rx00_api.c
 * @brief ST25Rx00 NFC Reader Public API Implementation
 *
 * This module provides a simple API for NFC operations using the ST RFAL library.
 *
 * Design Notes:
 * - Single-threaded operation - all functions must be called from the same thread
 * - The worker function drives the RFAL state machine and must be called frequently
 * - No mutex protection as RFAL library makes nested SPI calls that would deadlock
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

/** NFC discovery parameters */
static rfalNfcDiscoverParam discover_param;

/** Current NFC device pointer (owned by RFAL) */
static rfalNfcDevice *nfc_device;

/** API initialization state */
static bool api_initialized;

/** ISO-DEP buffers for APDU transceive */
static rfalIsoDepApduBufFormat isodep_tx_buf;
static rfalIsoDepApduBufFormat isodep_rx_buf;
static rfalIsoDepBufFormat isodep_tmp_buf;

/** NFC thread control */
static volatile bool nfc_thread_running;
static st25rx00_tag_callback_t tag_callback;

/*
 ******************************************************************************
 * HELPER FUNCTIONS
 ******************************************************************************
 */

static enum st25rx00_nfca_type get_nfca_type(uint8_t sel_res)
{
    if ((sel_res & 0x60) == 0x00) {
        return ST25RX00_NFCA_T2T;
    } else if ((sel_res & 0x20) == 0x20) {
        return ST25RX00_NFCA_T4T;
    } else if ((sel_res & 0x40) == 0x40) {
        return ST25RX00_NFCA_NFCDEP;
    }
    return ST25RX00_NFCA_OTHER;
}

static uint8_t copy_uid(uint8_t *dst, size_t dst_size, const uint8_t *src, size_t src_len)
{
    size_t len = (src_len <= dst_size) ? src_len : dst_size;
    memcpy(dst, src, len);
    return (uint8_t)len;
}

static void fill_tag_info(struct st25rx00_tag_info *info, const rfalNfcDevice *dev)
{
    memset(info, 0, sizeof(*info));

    switch (dev->type) {
    case RFAL_NFC_LISTEN_TYPE_NFCA:
        info->tech = ST25RX00_TECH_NFCA;
        info->uid_len = copy_uid(info->uid, sizeof(info->uid),
                                  dev->dev.nfca.nfcId1,
                                  dev->dev.nfca.nfcId1Len);
        info->sens_res = dev->dev.nfca.sensRes.anticollisionInfo;
        info->sel_res = dev->dev.nfca.selRes.sak;
        info->nfca_type = get_nfca_type(info->sel_res);
        memcpy(&info->dev.nfca, &dev->dev.nfca, sizeof(rfalNfcaListenDevice));
        break;

    case RFAL_NFC_LISTEN_TYPE_NFCB:
        info->tech = ST25RX00_TECH_NFCB;
        info->uid_len = copy_uid(info->uid, sizeof(info->uid),
                                  dev->dev.nfcb.sensbRes.nfcid0,
                                  RFAL_NFCB_NFCID0_LEN);
        memcpy(&info->dev.nfcb, &dev->dev.nfcb, sizeof(rfalNfcbListenDevice));
        break;

    case RFAL_NFC_LISTEN_TYPE_NFCV:
        info->tech = ST25RX00_TECH_NFCV;
        info->uid_len = copy_uid(info->uid, sizeof(info->uid),
                                  dev->dev.nfcv.InvRes.UID,
                                  RFAL_NFCV_UID_LEN);
        memcpy(&info->dev.nfcv, &dev->dev.nfcv, sizeof(rfalNfcvListenDevice));
        break;

    default:
        info->tech = ST25RX00_TECH_NONE;
        break;
    }
}

/*
 ******************************************************************************
 * PUBLIC API
 ******************************************************************************
 */

int st25rx00_nfc_init(void)
{
    ReturnCode err;

    if (api_initialized) {
        return 0;
    }

    LOG_INF("Calling rfalNfcInitialize()...");
    err = rfalNfcInitialize();
    LOG_INF("rfalNfcInitialize() returned %d", err);
    if (err != RFAL_ERR_NONE) {
        LOG_ERR("RFAL init failed: %d", err);
        return -EIO;
    }

    /* Set default discovery parameters */
    memset(&discover_param, 0, sizeof(discover_param));
    discover_param.compMode = RFAL_COMPLIANCE_MODE_NFC;
    discover_param.techs2Find = RFAL_NFC_POLL_TECH_A;  /* Start with just NFC-A for MIFARE tag */
    discover_param.totalDuration = 1000U;
    discover_param.devLimit = 1U;
    discover_param.wakeupEnabled = false;
    discover_param.wakeupConfigDefault = true;
    discover_param.nfcfBR = RFAL_BR_212;
    discover_param.ap2pBR = RFAL_BR_424;
    discover_param.notifyCb = NULL;

    api_initialized = true;
    LOG_INF("NFC API initialized");
    return 0;
}

int st25rx00_nfc_deinit(void)
{
    if (!api_initialized) {
        return 0;
    }

    rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_IDLE);
    nfc_device = NULL;
    api_initialized = false;
    return 0;
}

int st25rx00_discover_start(const struct st25rx00_discover_cfg *cfg)
{
    ReturnCode err;

    if (!api_initialized) {
        LOG_ERR("API not initialized");
        return -EINVAL;
    }

    /* Apply custom config if provided */
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
        LOG_ERR("Discovery start failed: %d", err);
        return -EIO;
    }

    LOG_DBG("Discovery started (techs=0x%04X)", discover_param.techs2Find);
    return 0;
}

int st25rx00_discover_stop(void)
{
    ReturnCode err;

    err = rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_IDLE);
    nfc_device = NULL;

    if (err != RFAL_ERR_NONE) {
        LOG_WRN("Discovery stop failed: %d", err);
        return -EIO;
    }
    return 0;
}

bool st25rx00_worker(struct st25rx00_tag_info *tag_info)
{
    rfalNfcState state;
    ReturnCode err;

    /* Drive the RFAL state machine */
    rfalNfcWorker();

    state = rfalNfcGetState();

    /* Check if a device was activated */
    if (state == RFAL_NFC_STATE_ACTIVATED) {
        err = rfalNfcGetActiveDevice(&nfc_device);
        if (err != RFAL_ERR_NONE || nfc_device == NULL) {
            LOG_ERR("Get active device failed: %d", err);
            return false;
        }

        if (tag_info != NULL) {
            fill_tag_info(tag_info, nfc_device);
            LOG_INF("Tag detected: tech=%d, UID len=%d",
                    tag_info->tech, tag_info->uid_len);
        }
        return true;
    }

    return false;
}

int st25rx00_deactivate(void)
{
    ReturnCode err;

    err = rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_DISCOVERY);
    nfc_device = NULL;

    if (err != RFAL_ERR_NONE) {
        LOG_ERR("Deactivate failed: %d", err);
        return -EIO;
    }
    return 0;
}

int st25rx00_field_control(bool enable)
{
    ReturnCode err;

    err = enable ? rfalFieldOnAndStartGT() : rfalFieldOff();
    if (err != RFAL_ERR_NONE) {
        LOG_ERR("Field control failed: %d", err);
        return -EIO;
    }

    LOG_DBG("RF field %s", enable ? "ON" : "OFF");
    return 0;
}

int st25rx00_get_active_device(struct st25rx00_tag_info *tag_info)
{
    if (tag_info == NULL || nfc_device == NULL) {
        return -ENODEV;
    }

    fill_tag_info(tag_info, nfc_device);
    return 0;
}

int st25rx00_transceive(const uint8_t *tx_buf, size_t tx_len,
                        uint8_t *rx_buf, size_t rx_buf_len,
                        size_t *rx_len, uint32_t timeout_ms)
{
    ReturnCode err;
    uint16_t actual_len = 0;

    if (nfc_device == NULL) {
        return -ENODEV;
    }

    err = rfalTransceiveBlockingTxRx(
        (uint8_t *)tx_buf, tx_len,
        rx_buf, rx_buf_len, &actual_len,
        RFAL_TXRX_FLAGS_DEFAULT,
        rfalConvMsTo1fc(timeout_ms)
    );

    if (err != RFAL_ERR_NONE) {
        LOG_ERR("Transceive failed: %d", err);
        return -EIO;
    }

    if (rx_len != NULL) {
        *rx_len = actual_len;
    }
    return 0;
}

int st25rx00_isodep_transceive(const uint8_t *tx_buf, size_t tx_len,
                               uint8_t *rx_buf, size_t rx_buf_len,
                               size_t *rx_len)
{
    ReturnCode err;
    uint16_t actual_len = 0;
    rfalIsoDepApduTxRxParam param;

    if (nfc_device == NULL) {
        return -ENODEV;
    }

    if (nfc_device->rfInterface != RFAL_NFC_INTERFACE_ISODEP) {
        LOG_ERR("Device doesn't support ISO-DEP");
        return -ENOTSUP;
    }

    if (tx_len > sizeof(isodep_tx_buf.apdu)) {
        LOG_ERR("TX data too large");
        return -ENOMEM;
    }

    memcpy(isodep_tx_buf.apdu, tx_buf, tx_len);

    memset(&param, 0, sizeof(param));
    param.txBuf = &isodep_tx_buf;
    param.txBufLen = tx_len;
    param.rxBuf = &isodep_rx_buf;
    param.rxLen = &actual_len;
    param.tmpBuf = &isodep_tmp_buf;
    param.FWT = nfc_device->proto.isoDep.info.FWT;
    param.dFWT = nfc_device->proto.isoDep.info.dFWT;
    param.FSx = nfc_device->proto.isoDep.info.FSx;
    param.ourFSx = RFAL_ISODEP_FSX_256;
    param.DID = nfc_device->proto.isoDep.info.DID;

    err = rfalIsoDepStartApduTransceive(param);
    if (err != RFAL_ERR_NONE) {
        LOG_ERR("ISO-DEP start failed: %d", err);
        return -EIO;
    }

    do {
        rfalWorker();
        err = rfalIsoDepGetApduTransceiveStatus();
    } while (err == RFAL_ERR_BUSY);

    if (err != RFAL_ERR_NONE) {
        LOG_ERR("ISO-DEP transceive failed: %d", err);
        return -EIO;
    }

    if (actual_len > rx_buf_len) {
        LOG_ERR("RX buffer too small");
        return -ENOMEM;
    }

    memcpy(rx_buf, isodep_rx_buf.apdu, actual_len);
    if (rx_len != NULL) {
        *rx_len = actual_len;
    }
    return 0;
}

/*
 ******************************************************************************
 * NFC THREAD FUNCTIONS
 ******************************************************************************
 */

void st25rx00_set_tag_callback(st25rx00_tag_callback_t cb)
{
    tag_callback = cb;
}

void st25rx00_thread_stop(void)
{
    nfc_thread_running = false;
}

void st25rx00_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct st25rx00_tag_info tag;
    int ret;

    LOG_INF("NFC thread starting");

    /* Initialize NFC API */
    ret = st25rx00_nfc_init();
    if (ret != 0) {
        LOG_ERR("Failed to initialize NFC: %d", ret);
        return;
    }

    /* Start discovery */
    ret = st25rx00_discover_start(NULL);
    if (ret != 0) {
        LOG_ERR("Failed to start discovery: %d", ret);
        return;
    }

    LOG_INF("NFC thread running - waiting for tags");
    nfc_thread_running = true;

    uint32_t loop_count = 0;
    rfalNfcState last_state = RFAL_NFC_STATE_NOTINIT;

    LOG_INF("Entering main loop");

    while (nfc_thread_running) {
        /* Brief delay between iterations */
        k_msleep(10);

        /* Process RFAL state machine */
        bool detected = st25rx00_worker(&tag);

        /* Debug: Log state changes */
        rfalNfcState state = rfalNfcGetState();
        if (state != last_state) {
            LOG_INF("State: %d -> %d", last_state, state);
            last_state = state;
        }

        /* Periodic status (every ~2 sec) */
        if (++loop_count >= 200) {
            loop_count = 0;
            LOG_INF("Polling... state=%d", state);
        }

        if (detected) {
            LOG_INF("Tag detected: tech=%d, UID len=%d", tag.tech, tag.uid_len);

            /* Invoke user callback if registered */
            if (tag_callback != NULL) {
                tag_callback(&tag);
            }

            /* Deactivate and restart discovery */
            st25rx00_deactivate();
            k_msleep(200);
        }
    }

    LOG_INF("NFC thread stopping");
    st25rx00_discover_stop();
}
