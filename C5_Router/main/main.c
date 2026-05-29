/*
 * ESP32-C5 独立 SoftAP 路由器固件
 *
 * 为双路 P4 WebRTC H.264 图传系统提供 5GHz SoftAP。
 * P4 使用固定 IP（.2/.3），手机等设备 DHCP 自动获取（.4+）。
 * C5 SoftAP 默认启用 L2 桥接，STA 之间可直接通信。
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/ip4_addr.h"

static const char *TAG = "c5_ap";

void app_main(void)
{
    // 1. 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // 2. 初始化网络接口
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 3. 创建 AP 网络接口
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(ap_netif);

    // 4. 初始化 WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 5. 设置 AP 静态 IP 并启动 DHCP
    //    P4#1 固定 192.168.4.2，P4#2 固定 192.168.4.3
    //    手机等设备 DHCP 自动获取 192.168.4.4+
    //    （lwIP DHCP 有 ARP 探测，不会冲突 P4 的固定 IP）
    esp_netif_ip_info_t ip_info = {};
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);

    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    // 6. 设置国家码（在 start 之前，锁住 5GHz 信道 149）
    ESP_ERROR_CHECK(esp_wifi_set_country_code("CN", true));

    // 7. 设置 WiFi 模式为 SoftAP
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    // 8. 配置 SoftAP 参数
    wifi_config_t ap_config = {
        .ap = {
            .ssid = "ROBOT_CAM",
            .password = "",
            .channel = 149,
            .authmode = WIFI_AUTH_OPEN,
            .max_connection = 4,
            .ssid_hidden = 0,
            .beacon_interval = 100,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

    // 9. 启动 WiFi
    ESP_ERROR_CHECK(esp_wifi_start());

    // 10. 打印网络信息
    wifi_config_t config_read;
    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_AP, &config_read));
    ESP_LOGI(TAG, "SoftAP started:");
    ESP_LOGI(TAG, "  SSID:     %s", config_read.ap.ssid);
    ESP_LOGI(TAG, "  Password: %s", config_read.ap.password);
    ESP_LOGI(TAG, "  Channel:  %d", config_read.ap.channel);
    ESP_LOGI(TAG, "  IP:       192.168.4.1");
    ESP_LOGI(TAG, "  P4 #1:    192.168.4.2 (static)");
    ESP_LOGI(TAG, "  P4 #2:    192.168.4.3 (static)");
    ESP_LOGI(TAG, "  DHCP:     192.168.4.4+ (auto)");

    // 11. 定时打印已连接客户端数
    wifi_sta_list_t sta_list;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        if (esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK) {
            ESP_LOGI(TAG, "Connected clients: %d", sta_list.num);
        }
    }
}
