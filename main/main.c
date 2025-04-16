/******************************************************************************
 * INCLUDES & DEFINES
 ******************************************************************************/
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

// Bluetooth headers
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_hf_client_api.h"

// I2S driver (for mic and speaker)
#include "driver/i2s.h"

// Wi-Fi & ESP-NOW headers
#include "esp_wifi.h"
#include "esp_now.h"

/* Device settings */
#define TAG                     "BT_AUDIO_COMBINED"
#define BT_DEVICE_NAME          "ESP32-BT-AUDIO"

// BT discoverability/connectability
#define BT_SCAN_MODE_CONNECTABLE       ESP_BT_CONNECTABLE
#define BT_SCAN_MODE_DISCOVERABLE      ESP_BT_GENERAL_DISCOVERABLE

// I2S microphone configuration
#define I2S_MIC_PORT            I2S_NUM_0
#define I2S_MIC_SAMPLE_RATE     16000
#define I2S_MIC_SDA_PIN         12
#define I2S_MIC_SCK_PIN         21
#define I2S_MIC_WS_PIN          22

// I2S speaker configuration
#define I2S_SPK_PORT            I2S_NUM_1
#define I2S_SPK_SAMPLE_RATE     44100  // For normal A2DP playback
#define I2S_SPK_BCLK            27
#define I2S_SPK_LRCK            26
#define I2S_SPK_DOUT            25

// Buffer sizes
#define I2S_BUFFER_SIZE         512
#define A2DP_RINGBUF_SIZE       (16 * 1024)
#define HF_RINGBUF_SIZE         (4 * 1024)

// Simple beep tune delay (in ms)
#define FANCY_TUNE_NOTE_DELAY   50

// ESP-NOW configuration
#define ESPNOW_WIFI_IF          WIFI_IF_STA
#define ESPNOW_CHANNEL          1

/******************************************************************************
 * DYNAMIC ROLE DEFINITION
 ******************************************************************************/
typedef enum {
    ROLE_UNDECIDED,
    ROLE_PRIMARY,
    ROLE_SECONDARY
} device_role_t;

// Start with an undecided role.
static device_role_t g_role = ROLE_UNDECIDED;

/******************************************************************************
 * GLOBALS (SHARED)
 ******************************************************************************/
static RingbufHandle_t a2dp_rb = NULL;    // For A2DP audio samples
static RingbufHandle_t hf_audio_rb = NULL;  // For HFP call audio

static TaskHandle_t a2dp_task_handle = NULL;
static TaskHandle_t mic_task_handle = NULL;

static SemaphoreHandle_t i2s_spk_mutex = NULL;

// ESPNOW flags for audio forwarding
volatile bool espnow_peer_connected = false;
volatile bool forwarding_enabled = false;

// HFP call status flag
volatile bool hf_call_active = false;

/******************************************************************************
 * FORWARD DECLARATIONS & CALLBACKS
 ******************************************************************************/
static void init_esp_now(void);
static esp_err_t espnow_send_audio(const uint8_t *data, size_t len);

// I2S functions
static void i2s_mic_init(void);
static void i2s_spk_init(void);
static void reinit_i2s_spk(int new_sample_rate);

// Fancy tune function
static void play_fancy_tune(void);

// Bluetooth (A2DP/HFP) callbacks and tasks:
static void a2dp_task(void *arg);
void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len);
void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
void bt_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);

static uint32_t outgoing_cb(uint8_t *p_buf, uint32_t sz);
static void hf_incoming_data_cb(const uint8_t *data, uint32_t len);
static void mic_loopback_task(void *arg);
static void hf_client_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param);

// Role-switch functions.
static void switch_to_primary(void);
static void switch_to_secondary(void);

// A task to periodically send handshake messages (only in primary)
static void espnow_timeout_task(void *arg)
{
    while (g_role == ROLE_PRIMARY) {
        if (!espnow_peer_connected) {
            const char *hs = "HANDSHAKE";
            esp_err_t ret = esp_now_send((uint8_t*)"\xFF\xFF\xFF\xFF\xFF\xFF", 
                                         (const uint8_t *)hs, strlen(hs));
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send handshake in timeout task: 0x%x", ret);
            }
            vTaskDelay(pdMS_TO_TICKS(10000));  // Wait 10 seconds.
            if (!espnow_peer_connected) {
                forwarding_enabled = false;
                ESP_LOGI(TAG, "No ESPNOW peer found within 10 sec; BT audio remains local.");
            } else {
                forwarding_enabled = true;
                ESP_LOGI(TAG, "ESPNOW peer detected; forwarding enabled.");
                break;
            }
        } else {
            forwarding_enabled = true;
            break;
        }
    }
    vTaskDelete(NULL);
}

/******************************************************************************
 * ESPNOW CALLBACKS
 ******************************************************************************/
// Updated ESPNOW receive callback with new signature.
// The first parameter provides meta info (including sender's MAC) via recv_info->src_addr.
static void espnow_receive_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len)
{
    ESP_LOGI(TAG, "ESPNOW received from %02x:%02x:%02x:%02x:%02x:%02x, len: %d, data: %.*s",
             recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
             recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5],
             data_len, data_len, data);

    if (!data || data_len <= 0)
        return;
    
    // Check for a role-message.
    if (strncmp((const char *)data, "ROLE_PRIMARY", strlen("ROLE_PRIMARY")) == 0) {
        ESP_LOGI(TAG, "Received ROLE_PRIMARY message; switching to SECONDARY");
        switch_to_secondary();
        return;
    }
    
    // In primary mode, a handshake marks that a peer is connected.
    if (g_role == ROLE_PRIMARY) {
        if (data_len >= 9 && strncmp((const char *)data, "HANDSHAKE", 9) == 0) {
            espnow_peer_connected = true;
            forwarding_enabled = true;
            ESP_LOGI(TAG, "ESPNOW handshake received; peer connection established.");
            // Optionally add the peer for unicast messaging.
            esp_now_peer_info_t peer_info = {0};
            memcpy(peer_info.peer_addr, recv_info->src_addr, 6);
            peer_info.channel = ESPNOW_CHANNEL;
            peer_info.encrypt = false;
            esp_err_t ret = esp_now_add_peer(&peer_info);
            if (ret == ESP_OK || ret == ESP_ERR_ESPNOW_EXIST) {
                ESP_LOGI(TAG, "Added secondary peer: %02x:%02x:%02x:%02x:%02x:%02x",
                         recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
                         recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
            } else {
                ESP_LOGE(TAG, "Failed to add secondary peer, error: 0x%x", ret);
            }
        }
    }
    
    // In secondary mode, any received audio is written directly to I2S.
    if (g_role == ROLE_SECONDARY) {
        size_t bytes_written;
        if (xSemaphoreTake(i2s_spk_mutex, portMAX_DELAY) == pdTRUE) {
            i2s_write(I2S_SPK_PORT, data, data_len, &bytes_written, portMAX_DELAY);
            xSemaphoreGive(i2s_spk_mutex);
        }
    }
}

// Initialize ESPNOW (using Wi-Fi STA).
static void init_esp_now(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_receive_cb));
}

// Send audio data via ESPNOW (from primary).
static esp_err_t espnow_send_audio(const uint8_t *data, size_t len)
{
    uint8_t broadcast_addr[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    return esp_now_send(broadcast_addr, data, len);
}

/******************************************************************************
 * ROLE SWITCH FUNCTIONS
 ******************************************************************************/
static void switch_to_primary(void)
{
    if (g_role == ROLE_PRIMARY)
        return;
    ESP_LOGI(TAG, "Switching to PRIMARY role");
    g_role = ROLE_PRIMARY;
    
    // Deinitialize BLE if necessary.
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    // Initialize Bluetooth Classic.
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
 
    esp_bt_dev_set_device_name(BT_DEVICE_NAME);
    esp_bt_gap_set_scan_mode(BT_SCAN_MODE_CONNECTABLE, BT_SCAN_MODE_DISCOVERABLE);
 
    // Initialize AVRCP and register callback.
    esp_avrc_ct_init();
    esp_avrc_ct_register_callback(bt_avrc_ct_cb);
 
    // Initialize A2DP sink.
    esp_a2d_register_callback(&bt_app_a2d_cb);
    esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);
    esp_a2d_sink_init();
 
    // Initialize HFP client.
    esp_hf_client_register_callback(hf_client_cb);
    esp_hf_client_init();
 
    // Initialize microphone for primary.
    i2s_mic_init();
 
    // Create ring buffer for A2DP audio if not already created.
    if (a2dp_rb == NULL) {
        a2dp_rb = xRingbufferCreate(A2DP_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    }
 
    // Create the A2DP playback task if not running.
    if (a2dp_task_handle == NULL) {
        xTaskCreatePinnedToCore(a2dp_task, "a2dp_task", 4096, NULL,
                                configMAX_PRIORITIES - 1, &a2dp_task_handle, 1);
    }
 
    // Broadcast our new role to peers.
    const char *role_msg = "ROLE_PRIMARY";
    esp_err_t ret = esp_now_send((uint8_t*)"\xFF\xFF\xFF\xFF\xFF\xFF",
                                 (const uint8_t *)role_msg, strlen(role_msg));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send ROLE_PRIMARY message: 0x%x", ret);
    }
 
    // Start a task to periodically check for ESPNOW peer connection.
    xTaskCreate(&espnow_timeout_task, "espnow_timeout_task", 2048, NULL,
                configMAX_PRIORITIES - 2, NULL);
}

static void switch_to_secondary(void)
{
    if (g_role == ROLE_SECONDARY)
        return;
    ESP_LOGI(TAG, "Switching to SECONDARY role");
    g_role = ROLE_SECONDARY;
    // Disable BT connectability/discoverability on secondary to prevent BT connection requests
    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    if (mic_task_handle) {
        vTaskDelete(mic_task_handle);
        mic_task_handle = NULL;
    }
}

/******************************************************************************
 * I2S SPEAKER & MICROPHONE FUNCTIONS
 ******************************************************************************/
static void i2s_spk_init(void)
{
    i2s_config_t config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = I2S_SPK_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .intr_alloc_flags = 0,
        .fixed_mclk = 0
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SPK_BCLK,
        .ws_io_num = I2S_SPK_LRCK,
        .data_out_num = I2S_SPK_DOUT,
        .data_in_num = I2S_PIN_NO_CHANGE,
        .mck_io_num = I2S_PIN_NO_CHANGE
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_SPK_PORT, &config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_SPK_PORT, &pin_config));
    i2s_zero_dma_buffer(I2S_SPK_PORT);
    ESP_LOGI(TAG, "🔊 I2S SPEAKER initialized (44100Hz stereo)");
}

static void reinit_i2s_spk(int new_sample_rate)
{
    ESP_ERROR_CHECK(i2s_driver_uninstall(I2S_SPK_PORT));
    i2s_config_t config = {0};
    if (new_sample_rate == 44100) {
        config.mode = I2S_MODE_MASTER | I2S_MODE_TX;
        config.sample_rate = new_sample_rate;
        config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
        config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
        config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
        config.dma_buf_count = 8;
        config.dma_buf_len = 64;
        config.use_apll = false;
        config.tx_desc_auto_clear = true;
        config.intr_alloc_flags = 0;
        config.fixed_mclk = 0;
        ESP_LOGI(TAG, "🔄 Reinit I2S for A2DP (44100Hz stereo)");
    } else if (new_sample_rate == 16000) {
        config.mode = I2S_MODE_MASTER | I2S_MODE_TX;
        config.sample_rate = new_sample_rate;
        config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
        config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
        config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
        config.dma_buf_count = 8;
        config.dma_buf_len = 64;
        config.use_apll = false;
        config.tx_desc_auto_clear = true;
        config.intr_alloc_flags = 0;
        config.fixed_mclk = 0;
        ESP_LOGI(TAG, "🔄 Reinit I2S for HFP (16000Hz mono)");
    }
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SPK_BCLK,
        .ws_io_num = I2S_SPK_LRCK,
        .data_out_num = I2S_SPK_DOUT,
        .data_in_num = I2S_PIN_NO_CHANGE,
        .mck_io_num = I2S_PIN_NO_CHANGE
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_SPK_PORT, &config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_SPK_PORT, &pin_config));
    i2s_zero_dma_buffer(I2S_SPK_PORT);
    ESP_LOGI(TAG, "🔊 I2S SPEAKER reinitialized to: %d Hz", new_sample_rate);
}

static void i2s_mic_init(void)
{
    i2s_config_t config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = I2S_MIC_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 256,
        .use_apll = false
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_MIC_SCK_PIN,
        .ws_io_num = I2S_MIC_WS_PIN,
        .data_in_num = I2S_MIC_SDA_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_MIC_PORT, &config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_MIC_PORT, &pin_config));
    i2s_zero_dma_buffer(I2S_MIC_PORT);
    ESP_LOGI(TAG, "🎤 I2S MIC initialized");
}

/******************************************************************************
 * FANCY TUNE PLAYBACK
 ******************************************************************************/
typedef struct {
    int frequency;
    int duration;
} note_t;

const note_t melody[] = {
    {523, 300}, {587, 300}, {659, 300}, {698, 300},
    {784, 300}, {880, 300}, {988, 300}, {1047, 600}, {0, 200}
};
#define MELODY_LENGTH (sizeof(melody) / sizeof(melody[0]))

static void play_fancy_tune(void)
{
    const int sample_rate = I2S_SPK_SAMPLE_RATE;
    const float amplitude = 10000;
    const uint16_t note_delay = FANCY_TUNE_NOTE_DELAY;

    for (int i = 0; i < MELODY_LENGTH; i++) {
        int freq = melody[i].frequency;
        int duration = melody[i].duration;
        int samples = (sample_rate * duration) / 1000;

        int16_t *tone = malloc(samples * sizeof(int16_t));
        if (!tone)
            return;

        if (freq == 0) {
            memset(tone, 0, samples * sizeof(int16_t));
        } else {
            for (int j = 0; j < samples; j++) {
                tone[j] = (int16_t)(amplitude * sinf((2.0f * M_PI * freq * j) / sample_rate));
            }
        }

        size_t bytes_written;
        if (xSemaphoreTake(i2s_spk_mutex, portMAX_DELAY) == pdTRUE) {
            i2s_write(I2S_SPK_PORT, tone, samples * sizeof(int16_t), &bytes_written, portMAX_DELAY);
            xSemaphoreGive(i2s_spk_mutex);
        }
        free(tone);
        vTaskDelay(pdMS_TO_TICKS(note_delay));
    }
    ESP_LOGI(TAG, "🔔 Fancy tune played");
}

/******************************************************************************
 * A2DP CALLBACKS & TASKS
 ******************************************************************************/
void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len)
{
    if (hf_call_active)
        return;
    if (a2dp_rb) {
        xRingbufferSend(a2dp_rb, data, len, 0);
    }
    // Forward audio via ESPNOW when enabled.
    if (g_role == ROLE_PRIMARY && forwarding_enabled) {
        espnow_send_audio(data, len);
    }
}

void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    ESP_LOGI(TAG, "A2DP event: %d", event);
    if (event == ESP_A2D_CONNECTION_STATE_EVT && param->conn_stat.state == 1) { // CONNECTED
        switch_to_primary();
    }
}

// A2DP playback task (for primary).
static void a2dp_task(void *arg)
{
    size_t item_size;
    uint8_t *data_ptr;
    size_t bytes_written;
    while (1) {
        if (hf_call_active) {
            size_t dummy;
            while ((data_ptr = (uint8_t *)xRingbufferReceiveUpTo(a2dp_rb, &dummy, 0, 0)) != NULL) {
                vRingbufferReturnItem(a2dp_rb, data_ptr);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        data_ptr = (uint8_t *)xRingbufferReceive(a2dp_rb, &item_size, portMAX_DELAY);
        if (data_ptr && item_size > 0) {
            if (xSemaphoreTake(i2s_spk_mutex, portMAX_DELAY) == pdTRUE) {
                i2s_write(I2S_SPK_PORT, data_ptr, item_size, &bytes_written, portMAX_DELAY);
                xSemaphoreGive(i2s_spk_mutex);
            }
            vRingbufferReturnItem(a2dp_rb, data_ptr);
        }
    }
}

/******************************************************************************
 * AVRCP CALLBACK
 ******************************************************************************/
void bt_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    if (event == ESP_AVRC_CT_CONNECTION_STATE_EVT && param->conn_stat.connected) {
        esp_avrc_ct_send_metadata_cmd(0,
            ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST | ESP_AVRC_MD_ATTR_ALBUM);
    }
    else if (event == ESP_AVRC_CT_METADATA_RSP_EVT) {
        ESP_LOGI(TAG, "🎵 Metadata: %.*s", param->meta_rsp.attr_length, param->meta_rsp.attr_text);
    }
}

/******************************************************************************
 * HFP CALLBACKS & TASKS
 ******************************************************************************/
static void hf_incoming_data_cb(const uint8_t *data, uint32_t len)
{
    size_t bytes_written;
    if (xSemaphoreTake(i2s_spk_mutex, portMAX_DELAY) == pdTRUE) {
        i2s_write(I2S_SPK_PORT, data, len, &bytes_written, portMAX_DELAY);
        xSemaphoreGive(i2s_spk_mutex);
    }
    if (g_role == ROLE_PRIMARY && forwarding_enabled) {
        espnow_send_audio(data, len);
    }
}

static uint32_t outgoing_cb(uint8_t *p_buf, uint32_t sz)
{
    size_t item_size;
    uint8_t *data = (uint8_t *)xRingbufferReceiveUpTo(hf_audio_rb, &item_size, 0, sz);
    if (!data) {
        memset(p_buf, 0, sz);
        return sz;
    }
    memcpy(p_buf, data, item_size);
    vRingbufferReturnItem(hf_audio_rb, data);
    return item_size;
}

static void mic_loopback_task(void *arg)
{
    uint8_t buffer[I2S_BUFFER_SIZE];
    size_t bytes_read;
    while (1) {
        if (i2s_read(I2S_MIC_PORT, buffer, I2S_BUFFER_SIZE, &bytes_read, portMAX_DELAY) == ESP_OK && bytes_read > 0) {
            if (xRingbufferSend(hf_audio_rb, buffer, bytes_read, 0) != pdTRUE) {
                size_t dummy;
                uint8_t *old = (uint8_t *)xRingbufferReceiveUpTo(hf_audio_rb, &dummy, 0, bytes_read);
                if (old) {
                    vRingbufferReturnItem(hf_audio_rb, old);
                }
                xRingbufferSend(hf_audio_rb, buffer, bytes_read, 0);
            }
            esp_hf_client_outgoing_data_ready();
        }
    }
}

static void hf_client_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param)
{
    ESP_LOGI(TAG, "HFP event: %d", event);
    if (event == ESP_HF_CLIENT_AUDIO_STATE_EVT) {
        ESP_LOGI(TAG, "Negotiated codec: %s",
                 (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) ?
                 "mSBC (HD)" : "CVSD");
        
        if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) {
            ESP_LOGW(TAG, "mSBC connection detected; for stability consider forcing CVSD mode.");
        }
 
        if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED ||
            param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) {
 
            hf_call_active = true;
            reinit_i2s_spk(16000);
 
            if (hf_audio_rb == NULL) {
                hf_audio_rb = xRingbufferCreate(HF_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
            }
            esp_hf_client_register_data_callback(hf_incoming_data_cb, outgoing_cb);
 
            if (g_role == ROLE_PRIMARY && mic_task_handle == NULL) {
                xTaskCreatePinnedToCore(mic_loopback_task, "mic_loopback", 4096, NULL,
                                        configMAX_PRIORITIES - 1, &mic_task_handle, 0);
            }
 
            switch_to_primary();
 
        } else if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_DISCONNECTED) {
            ESP_LOGI(TAG, "HFP Audio Disconnected");
            hf_call_active = false;
 
            if (mic_task_handle) {
                vTaskDelete(mic_task_handle);
                mic_task_handle = NULL;
            }
            if (hf_audio_rb) {
                vRingbufferDelete(hf_audio_rb);
                hf_audio_rb = NULL;
            }
 
            if (a2dp_rb) {
                size_t dummy;
                uint8_t *leftover;
                while ((leftover = (uint8_t *)xRingbufferReceiveUpTo(a2dp_rb, &dummy, 0, 0)) != NULL) {
                    vRingbufferReturnItem(a2dp_rb, leftover);
                }
            }
 
            reinit_i2s_spk(44100);
            i2s_zero_dma_buffer(I2S_SPK_PORT);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

/******************************************************************************
 * APP MAIN
 ******************************************************************************/
void app_main(void)
{
    // Initialize NVS.
    ESP_ERROR_CHECK(nvs_flash_init());
 
    // Initialize ESPNOW (this creates the Wi-Fi STA, starts Wi-Fi, sets channel, and registers ESPNOW callback).
    init_esp_now();
 
    // Initialize I2S speaker and create its semaphore.
    i2s_spk_init();
    i2s_spk_mutex = xSemaphoreCreateMutex();
 
    // --- Initialize Bluetooth stack in UNDECIDED mode ---
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if (esp_bt_controller_init(&bt_cfg) == ESP_OK) {
        if (esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT) == ESP_OK) {
            if (esp_bluedroid_init() == ESP_OK && esp_bluedroid_enable() == ESP_OK) {
                esp_bt_dev_set_device_name(BT_DEVICE_NAME);
                esp_bt_gap_set_scan_mode(BT_SCAN_MODE_CONNECTABLE, BT_SCAN_MODE_DISCOVERABLE);
                ESP_LOGI(TAG, "Bluetooth stack initialized in UNDECIDED mode");
            }
        }
    } else {
        ESP_LOGE(TAG, "Bluetooth initialization failed");
    }
    // --- End BT initialization ---
 
    // Add broadcast peer (for handshake messages).
    esp_now_peer_info_t broadcast_peer = {0};
    memset(broadcast_peer.peer_addr, 0xFF, 6);
    broadcast_peer.channel = ESPNOW_CHANNEL;
    broadcast_peer.encrypt = false;
    esp_err_t peer_ret = esp_now_add_peer(&broadcast_peer);
    if (peer_ret != ESP_OK && peer_ret != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "Failed to add broadcast peer: 0x%x", peer_ret);
    }
 
    // Send initial handshake to discover and connect to the other ESP32.
    const char *handshake = "HANDSHAKE";
    esp_err_t ret = esp_now_send(broadcast_peer.peer_addr, (const uint8_t *)handshake, strlen(handshake));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send handshake: 0x%x", ret);
    }
 
    ESP_LOGI(TAG, "✅ Device started in UNDECIDED mode; waiting for BT connection and ESPNOW handshake...");
 
    // Optional: Play a startup fancy tune.
    play_fancy_tune();
 
    // Main loop - tasks and callbacks will manage audio streaming, role switching, and inter-device ESPNOW communication.
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
/******************************************************************************
 * END OF FILE
 ******************************************************************************/
// This code is a combination of Bluetooth A2DP and HFP audio streaming with ESP-NOW for inter-device communication.