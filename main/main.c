#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_hf_client_api.h"
#include "driver/i2s.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"

#define TAG                     "BT_AUDIO_COMBINED"
#define BT_DEVICE_NAME          "ESP32-BT-AUDIO"

// I2S MIC config
#define I2S_MIC_PORT            I2S_NUM_0
#define I2S_MIC_SAMPLE_RATE     16000
#define I2S_MIC_SDA_PIN         12
#define I2S_MIC_SCK_PIN         21
#define I2S_MIC_WS_PIN          22

// I2S Speaker config
#define I2S_SPK_PORT            I2S_NUM_1
#define I2S_SPK_SAMPLE_RATE     44100
#define I2S_SPK_BCLK            27
#define I2S_SPK_LRCK            26
#define I2S_SPK_DOUT            25

#define I2S_BUFFER_SIZE         512
#define A2DP_RINGBUF_SIZE       (16 * 1024)
#define HF_RINGBUF_SIZE         (4 * 1024)
#define ESPNOW_MAX_LEN          250

// Secondary MAC for ESP‑NOW
static const uint8_t secondary_mac[6] = {0xA0,0xB7,0x65,0x04,0x8F,0xA0};

static RingbufHandle_t a2dp_rb = NULL;
static RingbufHandle_t hf_audio_rb = NULL;
static SemaphoreHandle_t i2s_spk_mutex = NULL;
static TaskHandle_t a2dp_task_handle = NULL;
static TaskHandle_t mic_task_handle = NULL;
volatile bool hf_call_active = false;

typedef struct { int frequency; int duration; } note_t;
static const note_t melody[] = {
    {523,300},{587,300},{659,300},{698,300},
    {784,300},{880,300},{988,300},{1047,600},{0,200}
};
#define MELODY_LENGTH (sizeof(melody)/sizeof(melody[0]))

// Function prototypes
static void i2s_spk_init(void);
static void reinit_i2s_spk(int new_rate);
static void i2s_mic_init(void);

static void init_bt(void);
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
static void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len);
static void bt_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);

static uint32_t outgoing_cb(uint8_t *p_buf, uint32_t sz);
static void hf_incoming_data_cb(const uint8_t *data, uint32_t len);
static void hf_client_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param);

static void a2dp_task(void *arg);
static void mic_loopback_task(void *arg);
static void play_fancy_tune(void);

static void wifi_init_sta(void);
static void init_esp_now(void);
static void on_data_send(const uint8_t *mac_addr, esp_now_send_status_t status);
static void on_data_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len);

// ---------- ESPNOW ----------

static void on_data_send(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGD(TAG, "ESPNOW send to %02x:%02x:%02x:%02x:%02x:%02x status=%d",
             mac_addr[0],mac_addr[1],mac_addr[2],mac_addr[3],mac_addr[4],mac_addr[5],
             status);
}

static void on_data_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    ESP_LOGI(TAG, "ESPNOW recv %d bytes from %02x:%02x:%02x:%02x:%02x:%02x",
             len,
             info->src_addr[0],info->src_addr[1],info->src_addr[2],
             info->src_addr[3],info->src_addr[4],info->src_addr[5]);
}

static void init_esp_now(void) {
    esp_err_t ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESPNOW init failed: %d", ret);
        return;
    }
    esp_now_register_send_cb(on_data_send);
    esp_now_register_recv_cb(on_data_recv);
    esp_now_peer_info_t peer = { 0 };
    memcpy(peer.peer_addr, secondary_mac, 6);
    peer.channel = 1;
    peer.ifidx = ESP_IF_WIFI_STA;
    peer.encrypt = false;
    ret = esp_now_add_peer(&peer);
    ESP_LOGI(TAG, "ESPNOW add peer: %s", ret==ESP_OK ? "OK" : "FAIL");
}

// ---------- I2S init ----------

static void i2s_spk_init(void) {
    i2s_config_t cfg = {
        .mode = I2S_MODE_MASTER|I2S_MODE_TX,
        .sample_rate = I2S_SPK_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .dma_desc_num = 8,
        .dma_frame_num = 64,
        .use_apll = false,
        .tx_desc_auto_clear = true
    };
    i2s_pin_config_t pins = {
        .bck_io_num = I2S_SPK_BCLK,
        .ws_io_num  = I2S_SPK_LRCK,
        .data_out_num = I2S_SPK_DOUT,
        .data_in_num  = I2S_PIN_NO_CHANGE,
        .mck_io_num   = I2S_PIN_NO_CHANGE
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_SPK_PORT, &cfg, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_SPK_PORT, &pins));
    i2s_zero_dma_buffer(I2S_SPK_PORT);
    ESP_LOGI(TAG, "🔊 I2S Speaker initialized");
}

static void reinit_i2s_spk(int new_rate) {
    ESP_ERROR_CHECK(i2s_driver_uninstall(I2S_SPK_PORT));
    i2s_config_t cfg = {0};
    if (new_rate == 44100) {
        cfg.mode = I2S_MODE_MASTER|I2S_MODE_TX;
        cfg.sample_rate = 44100;
        cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
        cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
        cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
        cfg.dma_desc_num = 8;
        cfg.dma_frame_num = 64;
        cfg.use_apll = false;
        cfg.tx_desc_auto_clear = true;
        ESP_LOGI(TAG, "🔄 I2S → A2DP 44.1kHz");
    } else {
        cfg.mode = I2S_MODE_MASTER|I2S_MODE_TX;
        cfg.sample_rate = 16000;
        cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
        cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
        cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
        cfg.dma_desc_num = 8;
        cfg.dma_frame_num = 64;
        cfg.use_apll = false;
        cfg.tx_desc_auto_clear = true;
        ESP_LOGI(TAG, "🔄 I2S → HFP 16kHz");
    }
    i2s_pin_config_t pins = {
        .bck_io_num = I2S_SPK_BCLK,
        .ws_io_num  = I2S_SPK_LRCK,
        .data_out_num = I2S_SPK_DOUT,
        .data_in_num  = I2S_PIN_NO_CHANGE,
        .mck_io_num   = I2S_PIN_NO_CHANGE
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_SPK_PORT, &cfg, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_SPK_PORT, &pins));
    i2s_zero_dma_buffer(I2S_SPK_PORT);
}

static void i2s_mic_init(void) {
    i2s_config_t cfg = {
        .mode = I2S_MODE_MASTER|I2S_MODE_RX,
        .sample_rate = I2S_MIC_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .dma_desc_num = 4,
        .dma_frame_num = 256,
        .use_apll = false
    };
    i2s_pin_config_t pins = {
        .bck_io_num = I2S_MIC_SCK_PIN,
        .ws_io_num  = I2S_MIC_WS_PIN,
        .data_in_num  = I2S_MIC_SDA_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_MIC_PORT, &cfg, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_MIC_PORT, &pins));
    i2s_zero_dma_buffer(I2S_MIC_PORT);
    ESP_LOGI(TAG, "🎤 I2S Mic initialized");
}

// ---------- BT & HFP init & callbacks ----------

static void init_bt(void) {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    esp_bt_gap_set_device_name(BT_DEVICE_NAME);
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    esp_avrc_ct_init();
    esp_avrc_ct_register_callback(bt_avrc_ct_cb);
    esp_a2d_register_callback(bt_app_a2d_cb);
    esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);
    esp_a2d_sink_init();
    esp_hf_client_register_callback(hf_client_cb);
    esp_hf_client_init();
}

void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    ESP_LOGI(TAG, "A2DP event: %d", event);
}

void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len) {
    if (!hf_call_active && a2dp_rb) {
        xRingbufferSend(a2dp_rb, data, len, 0);
    }
}

void bt_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param) {
    if (event == ESP_AVRC_CT_CONNECTION_STATE_EVT && param->conn_stat.connected) {
        esp_avrc_ct_send_metadata_cmd(0,
            ESP_AVRC_MD_ATTR_TITLE|ESP_AVRC_MD_ATTR_ARTIST|ESP_AVRC_MD_ATTR_ALBUM);
    } else if (event == ESP_AVRC_CT_METADATA_RSP_EVT) {
        ESP_LOGI(TAG, "🎵 %.*s",
                 param->meta_rsp.attr_length, param->meta_rsp.attr_text);
    }
}

static uint32_t outgoing_cb(uint8_t *p_buf, uint32_t sz) {
    size_t item_size;
    uint8_t *data = xRingbufferReceiveUpTo(hf_audio_rb, &item_size, 0, sz);
    if (!data) {
        memset(p_buf, 0, sz);
        return sz;
    }
    memcpy(p_buf, data, item_size);
    vRingbufferReturnItem(hf_audio_rb, data);
    return item_size;
}

static void hf_incoming_data_cb(const uint8_t *data, uint32_t len) {
    if (xSemaphoreTake(i2s_spk_mutex, portMAX_DELAY)==pdTRUE) {
        size_t wrote;
        i2s_write(I2S_SPK_PORT, data, len, &wrote, portMAX_DELAY);
        xSemaphoreGive(i2s_spk_mutex);
    }
}

void hf_client_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param) {
    if (event == ESP_HF_CLIENT_AUDIO_STATE_EVT) {
        bool connected = (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC ||
                          param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED);
        if (connected) {
            hf_call_active = true;
            reinit_i2s_spk(16000);
            if (!hf_audio_rb) {
                hf_audio_rb = xRingbufferCreate(HF_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
            }
            esp_hf_client_register_data_callback(hf_incoming_data_cb, outgoing_cb);
            xTaskCreatePinnedToCore(mic_loopback_task, "mic_loopback", 4096, NULL,
                                    configMAX_PRIORITIES-1, &mic_task_handle, 0);
        } else {
            hf_call_active = false;
            if (mic_task_handle) {
                vTaskDelete(mic_task_handle);
                mic_task_handle = NULL;
            }
            if (hf_audio_rb) {
                vRingbufferDelete(hf_audio_rb);
                hf_audio_rb = NULL;
            }
            reinit_i2s_spk(44100);
            i2s_zero_dma_buffer(I2S_SPK_PORT);
        }
    }
}

// ---------- Tasks ----------

static void a2dp_task(void *arg) {
    size_t item_size;
    uint8_t *data_ptr;
    while (1) {
        if (hf_call_active) {
            while ((data_ptr = xRingbufferReceiveUpTo(a2dp_rb, &item_size, 0, 0)) != NULL) {
                vRingbufferReturnItem(a2dp_rb, data_ptr);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        data_ptr = xRingbufferReceive(a2dp_rb, &item_size, portMAX_DELAY);
        if (data_ptr && item_size > 0) {
            // local
            size_t written;
            if (xSemaphoreTake(i2s_spk_mutex, portMAX_DELAY)==pdTRUE) {
                i2s_write(I2S_SPK_PORT, data_ptr, item_size, &written, portMAX_DELAY);
                xSemaphoreGive(i2s_spk_mutex);
            }
            // ESPNOW fragment
            size_t offset = 0;
            while (offset < item_size) {
                size_t chunk = item_size - offset;
                if (chunk > ESPNOW_MAX_LEN) chunk = ESPNOW_MAX_LEN;
                esp_now_send(secondary_mac, data_ptr + offset, chunk);
                offset += chunk;
            }
            vRingbufferReturnItem(a2dp_rb, data_ptr);
        }
    }
}

static void mic_loopback_task(void *arg) {
    uint8_t buffer[I2S_BUFFER_SIZE];
    size_t bytes_read;
    while (1) {
        if (i2s_read(I2S_MIC_PORT, buffer, I2S_BUFFER_SIZE, &bytes_read, portMAX_DELAY)==ESP_OK && bytes_read>0) {
            if (xRingbufferSend(hf_audio_rb, buffer, bytes_read, 0)!=pdTRUE) {
                size_t dummy;
                uint8_t *old = xRingbufferReceiveUpTo(hf_audio_rb, &dummy, 0, bytes_read);
                if (old) vRingbufferReturnItem(hf_audio_rb, old);
                xRingbufferSend(hf_audio_rb, buffer, bytes_read, 0);
            }
            esp_hf_client_outgoing_data_ready();
            // ESPNOW fragment
            size_t sent=0;
            while (sent < bytes_read) {
                size_t chunk = bytes_read - sent;
                if (chunk > ESPNOW_MAX_LEN) chunk = ESPNOW_MAX_LEN;
                esp_now_send(secondary_mac, buffer + sent, chunk);
                sent += chunk;
            }
        }
    }
}

static void play_fancy_tune(void) {
    const int sr = I2S_SPK_SAMPLE_RATE;
    const float amp = 10000;
    for (int i = 0; i < MELODY_LENGTH; i++) {
        int freq = melody[i].frequency;
        int dur  = melody[i].duration;
        int samples = (sr * dur) / 1000;
        int16_t *tone = malloc(samples * sizeof(int16_t));
        if (!tone) return;
        if (freq == 0) {
            memset(tone, 0, samples * sizeof(int16_t));
        } else {
            for (int j = 0; j < samples; j++) {
                tone[j] = (int16_t)(amp * sinf(2.0f * M_PI * freq * j / sr));
            }
        }
        size_t bw;
        if (xSemaphoreTake(i2s_spk_mutex, portMAX_DELAY)==pdTRUE) {
            i2s_write(I2S_SPK_PORT, tone, samples * sizeof(int16_t), &bw, portMAX_DELAY);
            xSemaphoreGive(i2s_spk_mutex);
        }
        free(tone);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGI(TAG, "🔔 Fancy tune played");
}

// ---------- Wi‑Fi STA ----------

static void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ---------- app_main ----------

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    init_bt();
    i2s_mic_init();
    i2s_spk_init();
    i2s_spk_mutex = xSemaphoreCreateMutex();
    a2dp_rb     = xRingbufferCreate(A2DP_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    hf_audio_rb = xRingbufferCreate(HF_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    xTaskCreatePinnedToCore(a2dp_task, "a2dp_task", 4096, NULL, configMAX_PRIORITIES-1, &a2dp_task_handle, 1);
    play_fancy_tune();
    ESP_LOGI(TAG, "ESPNOW: waiting to init...");
    wifi_init_sta();
    init_esp_now();
    ESP_LOGI(TAG, "✅ Primary ready (A2DP + HFP + ESPNOW)");
}
