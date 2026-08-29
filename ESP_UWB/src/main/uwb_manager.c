#include "uwb_manager.h"

#include "app_config.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"

#include "dw3000_hw.h"
#include "dwhw.h"
#include "dwmac.h"
#include "dwphy.h"
#include "dwproto.h"
#include "ranging.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <deca_device_api.h>
#include <inttypes.h>
#include <string.h>

static const char *TAG = "uwb_manager";

#define APP_UWB_PAN_ID             0xDECAu
#define APP_UWB_ANCHOR_MAC16       0x0001u
#define APP_UWB_TAG_MAC16          0x0002u
#define APP_UWB_TWR_PERIOD_MS      500u
#define APP_UWB_TWR_PROCESS_US     2000u
#define APP_UWB_DISTANCE_INVALID   UINT16_MAX
#define APP_UWB_PDOA_TX_PERIOD_MS  200u
#define APP_UWB_PDOA_TX_TIMEOUT_MS 20u
#define APP_UWB_PDOA_FRAME_FCS_LEN 2u

static bool s_available;
static bool s_is_anchor;
static uint16_t s_self_mac16;
static uint16_t s_peer_mac16;
static uint16_t s_result_seq;
static int64_t s_last_twr_start_ms;
static volatile bool s_has_result;
static uwb_result_t s_last_result;
static volatile bool s_has_pdoa_sample;
static volatile int16_t s_pdoa_raw;
static volatile int16_t s_pdoa_sts_quality;
static volatile uint16_t s_pdoa_rx_len;
static uint8_t s_pdoa_tx_msg[] = {0xC5, 0, 'U', 'W', 'B', '-', 'X', '3', 'A', 'O', 'A'};
static int64_t s_last_pdoa_tx_ms;

static dwt_txconfig_t s_pdoa_tx_config = {
    .PGdly = 0x34,
    .power = 0xfdfdfdfd,
    .PGcount = 0x0,
};

#if APP_ENABLE_UWB_MOCK
static uint16_t s_mock_seq;
#endif

static uint16_t mac_to_role_selector(const uint8_t mac[6])
{
    return ((uint16_t)mac[4] << 8) | mac[5];
}

static float pdoa_raw_to_phase_deg(int16_t pdoa_raw)
{
    /*
     * DW3000 PDOA is signed s[1:-11] radians.
     * This is phase difference in degrees, not yet calibrated AoA degrees.
     */
    return ((float)pdoa_raw / 2048.0f) * 57.2957795f;
}

#if APP_ENABLE_UWB_PDOA_TEST
static void pdoa_rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    int16_t sts_quality = 0;
    const int sts_status = dwt_readstsquality(&sts_quality, 0);

    if (sts_status >= 0) {
        s_pdoa_raw = dwt_readpdoa();
        s_pdoa_sts_quality = sts_quality;
        s_pdoa_rx_len = cb_data ? cb_data->datalength : 0;
        s_has_pdoa_sample = true;
    }

    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void pdoa_rx_err_cb(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void pdoa_start_rx_mode(void)
{
    static dwt_callbacks_s callbacks;
    memset(&callbacks, 0, sizeof(callbacks));
    callbacks.cbRxOk = pdoa_rx_ok_cb;
    callbacks.cbRxTo = pdoa_rx_err_cb;
    callbacks.cbRxErr = pdoa_rx_err_cb;

    dwt_setcallbacks(&callbacks);
    dwt_setinterrupt(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR, 0, DWT_ENABLE_INT);
    dwt_writesysstatuslo(DWT_INT_RCINIT_BIT_MASK | DWT_INT_SPIRDY_BIT_MASK);
    dwt_forcetrxoff();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    ESP_LOGI(TAG, "UWB-X3-AOA PDOA RX mode start; rotate/move tag and watch pdoa_phase");
}

static void pdoa_tx_once(void)
{
    const uint16_t tx_len_no_fcs = sizeof(s_pdoa_tx_msg);
    const uint16_t frame_len = tx_len_no_fcs + APP_UWB_PDOA_FRAME_FCS_LEN;

    dwt_writetxdata(tx_len_no_fcs, s_pdoa_tx_msg, 0);
    dwt_writetxfctrl(frame_len, 0, 0);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
        ESP_LOGW(TAG, "UWB-X3-AOA PDOA TX start failed");
        return;
    }

    const int64_t start_ms = esp_timer_get_time() / 1000;
    while ((dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK) == 0) {
        if ((esp_timer_get_time() / 1000) - start_ms > APP_UWB_PDOA_TX_TIMEOUT_MS) {
            ESP_LOGW(TAG, "UWB-X3-AOA PDOA TX timeout");
            dwt_forcetrxoff();
            return;
        }
    }

    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    ESP_LOGI(TAG, "UWB-X3-AOA PDOA TX seq=%u", s_pdoa_tx_msg[1]);
    s_pdoa_tx_msg[1]++;
}
#endif

static void twr_done_cb(uint64_t src, uint64_t dst, uint16_t dist_cm, uint16_t num)
{
    const int64_t now_ms = esp_timer_get_time() / 1000;
    const int16_t pdoa_raw = dwt_readpdoa();
    const float pdoa_phase_deg = pdoa_raw_to_phase_deg(pdoa_raw);

    if (dist_cm == APP_UWB_DISTANCE_INVALID) {
        ESP_LOGW(TAG, "UWB TWR failed src=0x%04" PRIx16 " dst=0x%04" PRIx16
                      " num=%u pdoa_raw=%d pdoa_phase=%.2f deg",
                 (uint16_t)src, (uint16_t)dst, num, pdoa_raw, (double)pdoa_phase_deg);
        return;
    }

    uwb_result_t result = {
        .target_uwb_id = (uint32_t)(uint16_t)dst,
        .distance_m = (float)dist_cm / 100.0f,
        .angle_deg = pdoa_phase_deg,
        .fp_power = 0.0f,
        .rx_power = 0.0f,
        .rssi = 0,
        .timestamp_ms = (uint32_t)now_ms,
        .seq = s_result_seq++,
        .valid = 1,
    };

    s_last_result = result;
    s_has_result = true;

    ESP_LOGI(TAG, "UWB TWR done src=0x%04" PRIx16 " dst=0x%04" PRIx16
                  " distance=%.2f m raw=%u cm num=%u pdoa_raw=%d pdoa_phase=%.2f deg",
             (uint16_t)src, (uint16_t)dst, (double)result.distance_m, dist_cm, num,
             pdoa_raw, (double)pdoa_phase_deg);
}

static esp_err_t uwb_libdeca_init(void)
{
    uint8_t mac[6] = {0};
    ESP_RETURN_ON_ERROR(esp_efuse_mac_get_default(mac), TAG, "read base MAC failed");

    /*
     * Same firmware on both boards:
     * COM3 observed MAC 14:c1:9f:c1:26:8c -> selector 0x268c -> anchor.
     * COM5 observed MAC 14:c1:9f:c1:2f:18 -> selector 0x2f18 -> tag.
     */
    const uint16_t selector = mac_to_role_selector(mac);
    s_is_anchor = selector <= 0x268cu;
    s_self_mac16 = s_is_anchor ? APP_UWB_ANCHOR_MAC16 : APP_UWB_TAG_MAC16;
    s_peer_mac16 = s_is_anchor ? APP_UWB_TAG_MAC16 : APP_UWB_ANCHOR_MAC16;

    ESP_LOGI(TAG, "UWB role=%s base_mac=%02x:%02x:%02x:%02x:%02x:%02x mac16=0x%04x peer=0x%04x",
             s_is_anchor ? "ANCHOR/RESPONDER" : "TAG/INITIATOR",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
             s_self_mac16, s_peer_mac16);

    if (dw3000_hw_init() != ESP_OK) {
        ESP_LOGE(TAG, "dw3000_hw_init failed");
        return ESP_FAIL;
    }
    dw3000_hw_reset();
    if (dw3000_hw_init_interrupt() != ESP_OK) {
        ESP_LOGE(TAG, "dw3000_hw_init_interrupt failed");
        return ESP_FAIL;
    }
    if (!dwhw_init()) {
        ESP_LOGE(TAG, "dwhw_init failed");
        return ESP_FAIL;
    }
    if (!dwphy_config()) {
        ESP_LOGE(TAG, "dwphy_config failed");
        return ESP_FAIL;
    }

#if APP_ENABLE_UWB_PDOA_TEST
    dwt_setlnapamode(DWT_TXRX_EN);
    dwt_configure_rf_port(DWT_RF_PORT_AUTO_1_2);
    dwt_configuretxrf(&s_pdoa_tx_config);
#endif

    dwphy_set_antenna_delay(DWPHY_ANTENNA_DELAY);
#if APP_ENABLE_UWB_PDOA_TEST
    ESP_LOGI(TAG, "UWB-X3-AOA PDoA test enabled: simple TX/RX, no TWR; angle_deg is raw phase deg");
    if (s_is_anchor) {
        pdoa_start_rx_mode();
    } else {
        ESP_LOGI(TAG, "UWB-X3-AOA PDOA TX mode ready");
    }
    return ESP_OK;
#endif

    if (!dwmac_init(APP_UWB_PAN_ID, s_self_mac16, dwprot_rx_handler, NULL, NULL)) {
        ESP_LOGE(TAG, "dwmac_init failed");
        return ESP_FAIL;
    }
    dwmac_set_frame_filter();

    twr_init(APP_UWB_TWR_PROCESS_US, true);
    if (!s_is_anchor) {
        twr_set_observer(twr_done_cb);
    }

    if (s_is_anchor) {
        ESP_LOGI(TAG, "UWB anchor RX start");
        dwmac_set_rx_reenable(true);
        dwt_forcetrxoff();
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    } else {
        ESP_LOGI(TAG, "UWB tag ready; ranging target=0x%04x", s_peer_mac16);
    }

    return ESP_OK;
}

esp_err_t uwb_manager_init(void)
{
#if APP_ENABLE_UWB
#if APP_ENABLE_UWB_MOCK
    s_available = true;
    ESP_LOGW(TAG, "UWB mock mode enabled");
    return ESP_OK;
#else
    esp_err_t err = uwb_libdeca_init();
    s_available = err == ESP_OK;
    if (s_available) {
        ESP_LOGI(TAG, "DW3000 libdeca init OK");
        return ESP_OK;
    }
    ESP_LOGW(TAG, "DW3000 libdeca init failed: %s", esp_err_to_name(err));
    return err;
#endif
#else
    s_available = false;
    ESP_LOGI(TAG, "UWB disabled by app_config.h");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t uwb_manager_poll(uwb_result_t *out_result)
{
    if (out_result == NULL || !s_available) {
        return ESP_ERR_INVALID_STATE;
    }

#if APP_ENABLE_UWB_MOCK
    const int64_t now_ms = esp_timer_get_time() / 1000;
    out_result->target_uwb_id = 0xA002u;
    out_result->distance_m = 2.0f + (float)((now_ms / 500) % 5) * 0.25f;
    out_result->angle_deg = -12.0f + (float)((now_ms / 300) % 8);
    out_result->fp_power = -82.0f;
    out_result->rx_power = -78.0f;
    out_result->rssi = -70;
    out_result->timestamp_ms = (uint32_t)now_ms;
    out_result->seq = s_mock_seq++;
    out_result->valid = 1;
    return ESP_OK;
#else
#if APP_ENABLE_UWB_PDOA_TEST
    if (!s_is_anchor) {
        const int64_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms - s_last_pdoa_tx_ms >= APP_UWB_PDOA_TX_PERIOD_MS) {
            s_last_pdoa_tx_ms = now_ms;
            pdoa_tx_once();
        }
        return ESP_ERR_NOT_FOUND;
    }

    if (s_has_pdoa_sample) {
        s_has_pdoa_sample = false;
        const int16_t pdoa_raw = s_pdoa_raw;
        const int16_t sts_quality = s_pdoa_sts_quality;
        const uint16_t rx_len = s_pdoa_rx_len;
        const float pdoa_phase_deg = pdoa_raw_to_phase_deg(pdoa_raw);
        const int64_t now_ms = esp_timer_get_time() / 1000;

        uwb_result_t result = {
            .target_uwb_id = APP_UWB_TAG_MAC16,
            .distance_m = 0.0f,
            .angle_deg = pdoa_phase_deg,
            .fp_power = 0.0f,
            .rx_power = 0.0f,
            .rssi = 0,
            .timestamp_ms = (uint32_t)now_ms,
            .seq = s_result_seq++,
            .valid = 1,
        };

        s_last_result = result;
        s_has_result = true;

        ESP_LOGI(TAG, "UWB-X3-AOA PDOA rx len=%u sts_quality=%d pdoa_raw=%d pdoa_phase=%.2f deg",
                 rx_len, sts_quality, pdoa_raw, (double)pdoa_phase_deg);
    }

    if (!s_has_result) {
        return ESP_ERR_NOT_FOUND;
    }

    *out_result = s_last_result;
    s_has_result = false;
    return ESP_OK;
#else
    if (s_is_anchor) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    const int64_t now_ms = esp_timer_get_time() / 1000;
    if (!twr_in_progress() && now_ms - s_last_twr_start_ms >= APP_UWB_TWR_PERIOD_MS) {
        s_last_twr_start_ms = now_ms;
        if (!twr_start(s_peer_mac16)) {
            ESP_LOGW(TAG, "UWB TWR start failed target=0x%04x", s_peer_mac16);
        } else {
            ESP_LOGI(TAG, "UWB TWR start target=0x%04x", s_peer_mac16);
        }
    }

    if (!s_has_result) {
        return ESP_ERR_NOT_FOUND;
    }

    *out_result = s_last_result;
    s_has_result = false;
    return ESP_OK;
#endif
#endif
}

bool uwb_manager_is_available(void)
{
    return s_available;
}
