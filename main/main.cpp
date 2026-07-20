#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp32LGFXLib.hpp"
#include "nvs_flash.h"

#include "esp_mac.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"

#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "ping/ping_sock.h"

#include "esp_csi_gain_ctrl.h"
#include "esp_event.h"

#define CONFIG_SEND_FREQUENCY      100
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
#define CSI_FORCE_LLTF                      0
#endif
#define CONFIG_FORCE_GAIN                   0

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
#define CONFIG_GAIN_CONTROL                 1
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#define ESP_IF_WIFI_STA ESP_MAC_WIFI_STA
#endif

static const char *TAG = "csi-visualizer";

#define WIFI_SSID      "BananaFi-2G"
#define WIFI_PASS      "haruhiyo01040312"

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

#if CONFIG_IDF_TARGET_ESP32C5
#define NUM_CSI_PARTS = 1
#define CSI_SIZE_L_LTF 54
#define CSI_SIZE_HT_LTF 57
#define CSI_SIZE_HE_LTF 245
#define CSI_SUBCARRIER_START_L 0
#define CSI_SUBCARRIER_END_L   120
#define CSI_SUBCARRIER_START_H 123
#define CSI_SUBCARRIER_END_H   244
#else
// For ESP32
#define NUM_CSI_PARTS = 3
#define CSI_SIZE_PER_LTF 64
#define CSI_SUBCARRIER_START_H 1
#define CSI_SUBCARRIER_END_H   25
#define CSI_SUBCARRIER_START_L 38
#define CSI_SUBCARRIER_END_L   62
#endif

SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

typedef struct comp {
    float re;
    float im;
} csi_t;

static csi_t heltf_raw[CSI_SIZE_HE_LTF], heltf[CSI_SIZE_HE_LTF];
static float heltf_amp = -1;
static unsigned int rssi;
static unsigned int non_he_pkt_count = 0;

const float CSI_ZOOM_RATIO = 50.0;
const float CSI_AMP_THRESHOLD = 10.0;

SemaphoreHandle_t xCsiMutex;

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI("wifi", "Disconnected. Reconnecting...");
            esp_wifi_connect();
            break;
        }
    }

    if (event_base == IP_EVENT &&
        event_id == IP_EVENT_STA_GOT_IP) {

        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

        ESP_LOGI("wifi",
                 "Got IP: " IPSTR,
                 IP2STR(&event->ip_info.ip));

        xEventGroupSetBits(wifi_event_group,
                           WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &wifi_event_handler,
            NULL,
            NULL));

    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &wifi_event_handler,
            NULL,
            NULL));

    wifi_config_t wifi_config = {};

    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, WIFI_PASS);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(
        esp_wifi_set_config(WIFI_IF_STA,
                            &wifi_config));

    ESP_ERROR_CHECK(esp_wifi_start());

    xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY);
}


void cancel_lo_ratation(csi_t *csi_raw, csi_t *csi, int csi_size)
{
    // Get average phase
    double sum_re = 0.0, sum_im = 0.0;
    for (int k = CSI_SUBCARRIER_START_H; k < CSI_SUBCARRIER_END_H; k++) {
        sum_re += csi_raw[k].re;
        sum_im += csi_raw[k].im;
    }
    for (int k = CSI_SUBCARRIER_START_L; k < CSI_SUBCARRIER_END_L; k++) {
        sum_re += csi_raw[k].re;
        sum_im += csi_raw[k].im;
    }
    double theta = atan2(sum_im, sum_re);

    // Cancel the average phase (get rid of random rotation comes from Local Oscillator)
    double cos_t = cos(-theta);
    double sin_t = sin(-theta);
    for (int k = 0; k < csi_size; k++) {
        csi[k].re = csi_raw[k].re * cos_t - csi_raw[k].im * sin_t;
        csi[k].im = csi_raw[k].re * sin_t + csi_raw[k].im * cos_t;
    }
}

float normalize_packet_rms(csi_t *csi)
{
    double pwr = 0.0; int cnt = 0;
    for (int k = CSI_SUBCARRIER_START_H; k < CSI_SUBCARRIER_END_H; k++) {
        pwr += csi[k].re*csi[k].re + csi[k].im*csi[k].im;
        cnt++;
    }
    for (int k = CSI_SUBCARRIER_START_L; k < CSI_SUBCARRIER_END_L; k++) {
        pwr += csi[k].re*csi[k].re + csi[k].im*csi[k].im;
        cnt++;
    }
    if (cnt == 0) return -1.0;
    double rms = sqrt(pwr / (double)cnt);
    if (rms < 1e-12) return -1.0;
    double inv = 1.0 / rms;
    for (int k = CSI_SUBCARRIER_START_H; k < CSI_SUBCARRIER_END_H; k++) {
        csi[k].re *= inv;
        csi[k].im *= inv;
    }
    for (int k = CSI_SUBCARRIER_START_L; k < CSI_SUBCARRIER_END_L; k++) {
        csi[k].re *= inv;
        csi[k].im *= inv;
    }
    return rms;
}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) {
        ESP_LOGW(TAG, "<%s> wifi_csi_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }

    if (memcmp(info->mac, ctx, 6)) {
        return;
    }

    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;
    static int s_count = 0;
    float compensate_gain = 1.0f;
    static uint8_t agc_gain = 0;
    static int8_t fft_gain = 0;
#if CONFIG_GAIN_CONTROL
    static uint8_t agc_gain_baseline = 0;
    static int8_t fft_gain_baseline = 0;
    esp_csi_gain_ctrl_get_rx_gain(rx_ctrl, &agc_gain, &fft_gain);
    if (s_count < 100) {
        esp_csi_gain_ctrl_record_rx_gain(agc_gain, fft_gain);
    } else if (s_count == 100) {
        esp_csi_gain_ctrl_get_rx_gain_baseline(&agc_gain_baseline, &fft_gain_baseline);
#if CONFIG_FORCE_GAIN
        esp_csi_gain_ctrl_set_rx_force_gain(agc_gain_baseline, fft_gain_baseline);
        ESP_LOGI(TAG, "fft_force %d, agc_force %d", fft_gain_baseline, agc_gain_baseline);
#endif
    }
    esp_csi_gain_ctrl_get_gain_compensation(&compensate_gain, agc_gain, fft_gain);
    ESP_LOGD(TAG, "compensate_gain %f, agc_gain %d, fft_gain %d", compensate_gain, agc_gain, fft_gain);
#endif

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    if (!s_count) {
        ESP_LOGI(TAG, "================ CSI RECV ================");
        ets_printf("type,seq,mac,rssi,rate,noise_floor,fft_gain,agc_gain,channel,local_timestamp,sig_len,rx_format,len,first_word,data\n");
    }
    /*ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d",
               s_count, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate,
               rx_ctrl->noise_floor, fft_gain, agc_gain, rx_ctrl->channel,
               rx_ctrl->timestamp, rx_ctrl->sig_len, rx_ctrl->cur_bb_format);*/
#else
    if (!s_count) {
        ESP_LOGI(TAG, "================ CSI RECV ================");
        ets_printf("type,id,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,not_sounding,aggregation,stbc,fec_coding,sgi,noise_floor,ampdu_cnt,channel,secondary_channel,local_timestamp,ant,sig_len,rx_format,len,first_word,data\n");
    }
    ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
               s_count, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate, rx_ctrl->sig_mode,
               rx_ctrl->mcs, rx_ctrl->cwb, rx_ctrl->smoothing, rx_ctrl->not_sounding,
               rx_ctrl->aggregation, rx_ctrl->stbc, rx_ctrl->fec_coding, rx_ctrl->sgi,
               rx_ctrl->noise_floor, rx_ctrl->ampdu_cnt, rx_ctrl->channel, rx_ctrl->secondary_channel,
               rx_ctrl->timestamp, rx_ctrl->ant, rx_ctrl->sig_len, rx_ctrl->sig_mode);
#endif

#if (CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61) && CSI_FORCE_LLTF

    int16_t csi = ((int16_t)(((((uint16_t)info->buf[1]) << 8) | info->buf[0]) << 4) >> 4);
    ets_printf(",%d,%d,\"[%d", (info->len - 2) / 2, info->first_word_invalid, (int16_t)(compensate_gain * csi));
    for (int i = 2; i < (info->len - 2); i += 2) {
        csi = ((int16_t)(((((uint16_t)info->buf[i + 1]) << 8) | info->buf[i]) << 4) >> 4);
        ets_printf(",%d", (int16_t)(compensate_gain * csi));
    }

#else
    //ets_printf(",%d,%d,\"[%d", info->len, info->first_word_invalid, (int16_t)(compensate_gain * info->buf[0]));
    /*for (int i = 1; i < info->len; i++) {
        ets_printf(",%d", (int16_t)(compensate_gain * info->buf[i]));
    }*/
#endif
    //ets_printf("]\"\n");
    s_count++;

    if (rx_ctrl->cur_bb_format != RX_BB_FORMAT_HE_SU) {
        non_he_pkt_count++;
        return;
    }

    for (int k = 0; k < CSI_SIZE_HE_LTF; k++) {
        // HE-LTF
        heltf_raw[k].re = (float) info->buf[k * 2];
        heltf_raw[k].im = (float) info->buf[(k * 2) + 1];
    }
    rssi = rx_ctrl->rssi;
    if (xCsiMutex != NULL) {
        if (xSemaphoreTake(xCsiMutex, portMAX_DELAY) == pdTRUE) {
            cancel_lo_ratation(heltf_raw, heltf, CSI_SIZE_HE_LTF);
            heltf_amp = normalize_packet_rms(heltf);
            xSemaphoreGive(xCsiMutex);
        }
    }
}

static void wifi_csi_init()
{
    /**
     * @brief In order to ensure the compatibility of routers, only LLTF sub-carriers are selected.
     */
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
    wifi_csi_config_t csi_config = {
        .enable                   = true,
        .acquire_csi_legacy       = true,
        .acquire_csi_force_lltf   = CSI_FORCE_LLTF,
        .acquire_csi_ht20         = true,
        .acquire_csi_ht40         = true,
        .acquire_csi_vht          = true,
        .acquire_csi_su           = true,
        .acquire_csi_mu           = true,
        .acquire_csi_dcm          = true,
        .acquire_csi_beamformed   = true,
        .acquire_csi_he_stbc_mode = 2,
        .val_scale_cfg            = 0,
        .dump_ack_en              = false,
        .lltf_bit_mode            = false,
        .reserved                 = false
    };
#elif CONFIG_IDF_TARGET_ESP32C6
    wifi_csi_config_t csi_config = {
        .enable                 = true,
        .acquire_csi_legacy     = false,
        .acquire_csi_ht20       = true,
        .acquire_csi_ht40       = true,
        .acquire_csi_su         = false,
        .acquire_csi_mu         = false,
        .acquire_csi_dcm        = false,
        .acquire_csi_beamformed = false,
        .acquire_csi_he_stbc    = 2,
        .val_scale_cfg          = false,
        .dump_ack_en            = false,
        .lltf_bit_mode          = false,
        .reserved               = false
    };
#else
    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = true,
        .channel_filter_en = true,
        .manu_scale        = true,
        .shift             = true,
        .dump_ack_en       = false
    };
#endif
    static wifi_ap_record_t s_ap_info = {};
    ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&s_ap_info));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, s_ap_info.bssid));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

static esp_err_t wifi_ping_router_start()
{
    static esp_ping_handle_t ping_handle = NULL;

    esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
    ping_config.count             = 0;
    ping_config.interval_ms       = 1000 / CONFIG_SEND_FREQUENCY;
    ping_config.timeout_ms        = 20;
    ping_config.task_stack_size   = 3072;
    ping_config.data_size         = 1;

    esp_netif_ip_info_t local_ip;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &local_ip);
    ESP_LOGI(TAG, "got ip:" IPSTR ", gw: " IPSTR, IP2STR(&local_ip.ip), IP2STR(&local_ip.gw));
    ping_config.target_addr.u_addr.ip4.addr = ip4_addr_get_u32(&local_ip.gw);
    ping_config.target_addr.type = ESP_IPADDR_TYPE_V4;

    esp_ping_callbacks_t cbs = {};
    esp_ping_new_session(&ping_config, &cbs, &ping_handle);
    esp_ping_start(ping_handle);

    return ESP_OK;
}

void plot_csi_in_phasor(csi_t *csi, int color, float amplitude) {
  if (amplitude <= CSI_AMP_THRESHOLD) {
    color = lcd.color565(64,64,64);
  }
  // Positive sub-carrier
  for (int k = CSI_SUBCARRIER_START_H; k < CSI_SUBCARRIER_END_H; k++) {
    spr.drawPixel(_hw + (csi[k].re * CSI_ZOOM_RATIO), _hh + (csi[k].im * CSI_ZOOM_RATIO), color);
  }
  // Negative sub-carrier
  for (int k = CSI_SUBCARRIER_START_L; k < CSI_SUBCARRIER_END_L; k++) {
    spr.drawPixel(_hw + (csi[k].re * CSI_ZOOM_RATIO), _hh + (csi[k].im * CSI_ZOOM_RATIO), color);
  }
}

void connect_csi_plots(csi_t *csi, int color, float amplitude) {
  int x, y, last_x, last_y;
  if (amplitude <= CSI_AMP_THRESHOLD) {
    color = lcd.color565(64,64,64);
  }
  // Negative sub-carrier
  for (int k = CSI_SUBCARRIER_START_L; k < CSI_SUBCARRIER_END_L; k++) {
        x = _hw + (csi[k].re * CSI_ZOOM_RATIO);
    y = _hh + (csi[k].im * CSI_ZOOM_RATIO);
    if (k != CSI_SUBCARRIER_START_L) {
      spr.drawLine(last_x, last_y, x, y, color);
    }
    last_x = x;
    last_y = y;
  }
  // Positive sub-carrier
  for (int k = CSI_SUBCARRIER_START_H; k < CSI_SUBCARRIER_END_H; k++) {
    x = _hw + (csi[k].re * CSI_ZOOM_RATIO);
    y = _hh + (csi[k].im * CSI_ZOOM_RATIO);
    if (k != CSI_SUBCARRIER_START_H) {
      spr.drawLine(last_x, last_y, x, y, color);
      //spr.drawLine(last_x, last_y, x, y, TFT_YELLOW);
    }
    last_x = x;
    last_y = y;
  }
}

extern "C" void app_main(void)
{
  xCsiMutex = xSemaphoreCreateMutex();
  lcd.init();
  setupLGFX(16, ROT0);
  uint32_t sz = _w * _h;

  printf("w=%ld h=%ld\n",
       lcd.width(),
       lcd.height());

  spr.setColorDepth(DEPTH_8BIT);
  if (!spr.createSprite(_w, _h)) {
    printf("ERROR: malloc error (tmpspr:%ldByte)\n", sz);
    while(1) {
      sleep(1);
    };
  }
  printf("init done\n");
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_sta();

  wifi_csi_init();
  wifi_ping_router_start();

  while(1) {
    spr.fillSprite(TFT_BLACK);
    spr.drawLine(0, _hh, _w, _hh, TFT_DARKGRAY);spr.drawLine(_hw, 0, _hw, _h, TFT_DARKGRAY);
    spr.setCursor(0,0);
    spr.setTextColor(TFT_WHITE);spr.setTextSize(1);spr.printf("RSSI=%d\n", rssi);
    spr.printf("AMP_HE-LTF = %2.2f\n", heltf_amp);
    spr.printf("NON-HE PKTS = %d\n", non_he_pkt_count);
    connect_csi_plots(heltf, TFT_MAGENTA, heltf_amp);
    spr.pushSprite(&lcd, 0, 0);

    vTaskDelay(10);
  }
}
