#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"         // New header for network interface initialization
#include "nvs_flash.h"
#include "esp_now.h"
#include "esp_log.h"

#define ESPNOW_WIFI_IF WIFI_IF_STA
#define CHANNEL         1

static const char *TAG = "NODE2";

// Updated callback signature for receiving ESP‑NOW data.
// The parameter 'recv_info' contains the sender's MAC address via recv_info->src_addr.
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    char mac_str[18];
    // Use recv_info->src_addr to access the sender's MAC address
    snprintf(mac_str, sizeof(mac_str),
             "%02x:%02x:%02x:%02x:%02x:%02x",
             recv_info->src_addr[0], recv_info->src_addr[1],
             recv_info->src_addr[2], recv_info->src_addr[3],
             recv_info->src_addr[4], recv_info->src_addr[5]);
    ESP_LOGI(TAG, "Data received from %s, len: %d", mac_str, len);
    ESP_LOGI(TAG, "Message: %.*s", len, data);
}

static void init_esp_now(void) {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
}

static void wifi_init(void) {
    // Initialize TCP/IP stack via esp_netif and create the default event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // Create the default Wi‑Fi station instance
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    // Start Wi‑Fi before setting the channel
    ESP_ERROR_CHECK(esp_wifi_start());
    // Now it's safe to set the channel
    ESP_ERROR_CHECK(esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE));
}

void app_main(void) {
    // Initialize NVS — required for Wi‑Fi initialization
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
    wifi_init();
    init_esp_now();

    // Optionally add Node 1 as a peer if two‑way communication is desired.
    esp_now_peer_info_t peer_info = {0};
    uint8_t node1_mac[6] = {0xa0, 0xb7, 0x65, 0x04, 0x8f, 0xa0};
    memcpy(peer_info.peer_addr, node1_mac, 6);
    peer_info.channel = CHANNEL;
    peer_info.ifidx = ESPNOW_WIFI_IF;
    peer_info.encrypt = false;

    if (esp_now_add_peer(&peer_info) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add peer; not critical if only receiving");
    } else {
        ESP_LOGI(TAG, "Peer (Node1) added successfully");
    }
    
    // Loop indefinitely while waiting for incoming data
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
