# ESP32-C5 独立 SoftAP 固件

## 背景

现有一套 ESP32-P4 + C5 双摄像头图传系统（WebRTC H.264），架构为 P4 通过 SDIO 控制 C5 做 WiFi（ESP-Hosted）。已遇到以下问题：

- **C5 固件限制**：不支持 STA 之间通信桥接（L2 forwarding），导致浏览器无法直接连到 STA 设备的 HTTP/WebRTC 服务
- **P4 内部 DRAM 不足**：P4 仅有 ~400KB 内部 RAM，无法同时运行两个 esp_peer（DTLS+AES 需要大量 DMA 缓冲）

**解决方案**：用一块独立的 ESP32-C5 开发板作为纯 SoftAP 路由器，两台 P4 都作为 STA 连接该 C5。C5 的 ESP-IDF SoftAP 默认使能 L2 桥接，STA 之间可直接通信。

## 架构

```
┌─────────────────────────────────────────────────────┐
│             独立 C5  SoftAP 路由器                      │
│             SSID: ROBOT_CAM                           │
│             信道: 149 (5GHz)                          │
│             IP: 192.168.4.1                          │
│             DHCP: 192.168.4.10 ~ 192.168.4.20         │
└──────────┬──────────────────────────┬────────────────┘
           │                          │
      ┌────▼────┐              ┌────▼────┐
      │ P4 #1   │              │ P4 #2   │
      │ STA     │              │ STA     │
      │ 相机A   │              │ 相机B   │
│      │ 192.168.4.2(固定)│    │ 192.168.4.3(固定)│
      └─────────┘              └─────────┘
               浏览器 (连 C5 WiFi)
               打开 http://192.168.4.2 或 http://192.168.4.3
```

## 功能需求

1. **5GHz SoftAP**，信道 149，国家码 CN
2. **DHCP 服务器**，地址池 192.168.4.2 ~ 192.168.4.10
3. **开放 WiFi**（无密码，authmode=WIFI_AUTH_OPEN）
4. **SSID: "ROBOT_CAM"**
5. **L2 桥接**（ESP-IDF SoftAP 默认启用，客户端之间可直接路由）
6. **最多连接 4 个 STA**

## 代码实现

### 文件结构

```
c5_softap/
├── CMakeLists.txt
├── main/
│   ├── CMakeLists.txt
│   └── main.c
├── sdkconfig
└── sdkconfig.defaults
```

### CMakeLists.txt（顶层）

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(c5_softap)
```

### main/CMakeLists.txt

```cmake
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS ".")
```

### main/main.c

```c
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"

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

    // 3. 创建 AP 接口
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(ap_netif);

    // 4. 初始化 WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 5. 设置 SoftAP 参数
    //    先设置 AP 的 IP，确保 DHCP 在正确网段
    esp_netif_ip_info_t ip_info = {};
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    esp_netif_dhcps_stop(ap_netif);  // 先停 DHCP 才能改 IP 池
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));

    //    DHCP 地址池：192.168.4.10 ~ 192.168.4.20
    //    (留出 .2 ~ .9 给 P4 固定 IP，手机自动拿 .10+)
    esp_netif_dhcps_option_t dhcp_op = {};
    dhcp_op.enable = true;
    dhcp_op.start_ip.addr = (192 << 24) | (168 << 16) | (4 << 8) | 10; // 192.168.4.10
    dhcp_op.end_ip.addr   = (192 << 24) | (168 << 16) | (4 << 8) | 20; // 192.168.4.20
    ESP_ERROR_CHECK(esp_netif_dhcps_option(ap_netif,
        ESP_NETIF_OP_SET, ESP_NETIF_SUBNET_IP_RANGE, &dhcp_op, sizeof(dhcp_op)));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    // 6. 设置国家码（5GHz 信道需要）
    ESP_ERROR_CHECK(esp_wifi_set_country_code("CN", true));

    // 7. 配置 SoftAP
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
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

    // 8. 启动 WiFi
    ESP_ERROR_CHECK(esp_wifi_start());

    // 9. 打印网络信息
    wifi_config_t config_read;
    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_AP, &config_read));
    ESP_LOGI(TAG, "SoftAP started:");
    ESP_LOGI(TAG, "  SSID:     %s", config_read.ap.ssid);
    ESP_LOGI(TAG, "  Password: %s", config_read.ap.password);
    ESP_LOGI(TAG, "  Channel:  %d", config_read.ap.channel);
    ESP_LOGI(TAG, "  IP:       192.168.4.1");
    ESP_LOGI(TAG, "  DHCP:     192.168.4.10 ~ 192.168.4.20");

    // 10. 定时打印已连接客户端数（可选调试）
    wifi_sta_list_t sta_list;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        if (esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK) {
            ESP_LOGI(TAG, "Connected clients: %d", sta_list.num);
        }
    }
}
```

## sdkconfig 关键配置

```
CONFIG_ESP_WIFI_SOFTAP_SUPPORT=y
CONFIG_ESP_WIFI_REMOTE_ENABLED=n              # 非 ESP-Hosted，直接控制 WiFi
CONFIG_LWIP_IP_FORWARD=n                       # 不需要 IP 转发（SoftAP L2 就够了）
CONFIG_ESP_MAIN_TASK_STACK_SIZE=4096
```

### sdkconfig.defaults（推荐）

```
CONFIG_ESP_WIFI_SOFTAP_SUPPORT=y
CONFIG_ESP_WIFI_REMOTE_ENABLED=n
CONFIG_LWIP_IP_FORWARD=n
CONFIG_ESP_MAIN_TASK_STACK_SIZE=4096
CONFIG_LOG_DEFAULT_LEVEL_INFO=y
```

## 编译和烧录

```bash
# 设置 IDF 环境（使用与 P4 项目一致的 v5.5）
cd c5_softap
export IDF_PATH=C:/esp/v5.5/esp-idf

# 配置目标为 ESP32-C5
idf.py set-target esp32c5

# 编译
idf.py build

# 烧录（根据实际串口修改）
idf.py -p COM?? flash monitor
```

## 验证

1. 打开串口监视器，确认日志显示 `SoftAP started`
2. 用手机/电脑扫描 WiFi，应看到 `ROBOT_CAM`（无密码）
3. 连接后自动获取 IP（192.168.4.x）
4. 两台 P4 分别连上 C5 的 WiFi，稳定在各自固定 IP（.2 和 .3）
5. 手机连上 C5 WiFi，自动获取 192.168.4.10+ 的 IP
6. 浏览器打开 `http://192.168.4.2:80/webrtc/signal` 应能收到 SSE

## 注意事项

1. **5GHz 信道 149 需要国家码 CN**，否则 ESP-IDF 会回退到 1 信道
2. **C5 的 SoftAP 默认启用 L2 桥接**，不需要额外配置 `CONFIG_LWIP_IP_FORWARD`
3. **DHCP 地址池范围要够大**，至少容纳 4-6 个客户端
4. 这个 C5 只做 AP，不再跑 P4 的 SDIO/ESP-Hosted，所以 `CONFIG_ESP_WIFI_REMOTE_ENABLED=n`
5. C5 的 5GHz 性能约 30-50Mbps 实际吞吐，对 H.264 WebRTC 双路流（每路 ~6Mbps）足够
6. 如果双路视频卡顿，可以在 P4 端降低码率（例如每路 3Mbps）
