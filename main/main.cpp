/*
 * WebRTC H.264 Video Streaming
 * ESP32-P4 + OV5647 + HW H.264 Encoder + esp_peer (ICE/DTLS/SRTP)
 * Browser: RTCPeerConnection (iOS Safari / Chrome / Firefox)
 */

// ──── Device Role: uncomment ONE ────
// #define DEVICE_ROLE_AP       //AP模式
#define DEVICE_ROLE_STA         //STA模式

// ──── STA Debug Level (uncomment ONE) ────
// #define STA_DBG_BARE_WIFI        // ① bare WiFi only ✅ PASSED
// #define STA_DBG_CAMERA_ONLY   // ② + camera, no WebRTC ✅ PASSED
#define STA_DBG_FULL          // ③ full stack (expect crash)

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_video_init.h"
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <lwip/ip4_addr.h>
#define CACHE_LINE_SIZE 128
#include "camera/app_video.h"

// WebRTC video interface
void webrtc_on_yuv_frame(const uint8_t *yuv_data, size_t yuv_len);
esp_err_t webrtc_video_init(int camera_fd);
void webrtc_set_ap_mode(bool is_ap);

static const char *TAG = "main";

// ── Camera callback: feed YUV420 frames to WebRTC pipeline ──
static uint32_t g_cam_fc = 0;
static int64_t  g_cam_last = 0;

static void camera_cb(uint8_t *buf, uint8_t index, uint32_t w, uint32_t h, size_t len)
{
    g_cam_fc++;

    // Feed every frame — double-buffering handles async safety
    webrtc_on_yuv_frame(buf, len);

    // FPS display every 100 frames
    int64_t now = esp_timer_get_time();
    if (g_cam_fc % 100 == 0 && g_cam_last > 0) {
        float fps = 100.f * 1000000.f / (float)(now - g_cam_last);
        ESP_LOGI(TAG, "Camera: %.1f fps, frame=%" PRIu32 " size=%u",
            fps, g_cam_fc, (unsigned)len);
    }
    if (g_cam_fc % 100 == 0) g_cam_last = now;
}

// ── WiFi Setup ──

#ifdef DEVICE_ROLE_AP

static void wifi_ap(void)
{
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t c = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&c));

    // Use country code "CN" — firmware handles channel ranges internally
    esp_wifi_set_country_code("CN", true);

    wifi_config_t a = {
        .ap = {
            .ssid = "ROBOT_CAM", .password = "",
            .channel = 149, .authmode = WIFI_AUTH_OPEN, .max_connection = 6,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &a));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_config_t a_read;
    esp_wifi_get_config(WIFI_IF_AP, &a_read);
    ESP_LOGI(TAG, "WiFi AP started on channel %d", a_read.ap.channel);
}

#else // DEVICE_ROLE_STA

static SemaphoreHandle_t g_sta_ip_got = NULL;
static char g_sta_ip[16] = "";

static void sta_got_ip_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
    esp_ip4addr_ntoa(&evt->ip_info.ip, g_sta_ip, sizeof(g_sta_ip));
    ESP_LOGI(TAG, "STA got IP: %s", g_sta_ip);
    xSemaphoreGive(g_sta_ip_got);
}

static void sta_wifi_event_handler(void *arg, esp_event_base_t base,
                                    int32_t id, void *data)
{
    if (id == WIFI_EVENT_STA_START)
        ESP_LOGI(TAG, "WiFi event: STA START");
    else if (id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "WiFi event: STA CONNECTED");
    }
    else if (id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *evt = (wifi_event_sta_disconnected_t *)data;
        ESP_LOGW(TAG, "WiFi event: STA DISCONNECTED, reason=%d", evt->reason);
    }
}

static void wifi_init(void)
{
    g_sta_ip_got = xSemaphoreCreateBinary();

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    // Static IP — avoids DHCP conflict with phone
    // P4 #1: keep as 192.168.4.10. P4 #2: change to 192.168.4.11.
    esp_netif_dhcpc_stop(sta_netif);
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 11);  // <-- P4 #1=10, P4 #2=11
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    esp_netif_set_ip_info(sta_netif, &ip_info);
    strcpy(g_sta_ip, "192.168.4.11");  // <-- match above

    wifi_init_config_t c = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&c));

    // Use country code "CN" — firmware handles channel ranges internally
    esp_wifi_set_country_code("CN", true);

    wifi_config_t s = {
        .sta = {
            .ssid = "ROBOT_CAM", .password = "",
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &s));

    // Register event handlers BEFORE starting WiFi
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               sta_got_ip_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               sta_wifi_event_handler, NULL));

    ESP_LOGI(TAG, "WiFi STA connecting to ROBOT_CAM...");
    ESP_ERROR_CHECK(esp_wifi_start());
    vTaskDelay(pdMS_TO_TICKS(500));  // let C5 WiFi stack settle
    esp_wifi_connect();               // initiate actual connection

    // Block until STA connects (or timeout after 30s)
    if (xSemaphoreTake(g_sta_ip_got, pdMS_TO_TICKS(30000)) == pdTRUE) {
        ESP_LOGI(TAG, "WiFi connected, IP: %s", g_sta_ip);
    } else {
        ESP_LOGW(TAG, "WiFi STA did not connect within 30s, continuing anyway");
        strcpy(g_sta_ip, "192.168.4.x"); // fallback
    }
}

#endif

// ── Main ──
extern "C" void app_main(void)
{
    // Initialize NVS
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Initialize I2C for camera
    i2c_master_bus_handle_t i2c;
    i2c_master_bus_config_t ic = {
        .i2c_port = 1,
        .sda_io_num = GPIO_NUM_7,
        .scl_io_num = GPIO_NUM_8,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = true },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&ic, &i2c));
    ESP_ERROR_CHECK(app_video_main(i2c));

    // Open camera in YUV420 mode
    int fd = app_video_open((char *)ESP_VIDEO_MIPI_CSI_DEVICE_NAME, APP_VIDEO_FMT_YUV420);
    assert(fd >= 0);

    // Read back actual camera resolution (CSI driver may have accepted or rejected 1080p)
    int cam_w = 1280, cam_h = 960;
    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) == 0) {
        cam_w = fmt.fmt.pix.width;
        cam_h = fmt.fmt.pix.height;
    }
    ESP_LOGI(TAG, "Camera resolution: %dx%d", cam_w, cam_h);

    // Allocate buffers
    size_t buf_size = cam_w * cam_h * 3 / 2;
    void *fb[3] = { 0 };
    for (int i = 0; i < 3; i++) {
        fb[i] = heap_caps_aligned_alloc(CACHE_LINE_SIZE, buf_size,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
        assert(fb[i]);
    }
    ESP_ERROR_CHECK(app_video_set_bufs(fd, 3, (const void **)fb));

    // Initialize WiFi
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
#ifdef DEVICE_ROLE_AP
    webrtc_set_ap_mode(true);
    wifi_ap();
#else
    webrtc_set_ap_mode(false);
    wifi_init();
#endif

    // WebRTC init — skip for BARE_WIFI and CAMERA_ONLY
#if !defined(DEVICE_ROLE_STA) || (defined(STA_DBG_FULL))
    ESP_ERROR_CHECK(webrtc_video_init(fd));
    ESP_ERROR_CHECK(app_video_register_frame_operation_cb(camera_cb));
#endif

    // Camera stream — skip only for BARE_WIFI
#if !defined(DEVICE_ROLE_STA) || !defined(STA_DBG_BARE_WIFI)
    ESP_ERROR_CHECK(app_video_stream_task_start(fd, 0));
#endif

#if defined(DEVICE_ROLE_STA)
 #ifdef STA_DBG_BARE_WIFI
    ESP_LOGI(TAG, "STA BARE WIFI — no camera, no WebRTC");
 #elif defined(STA_DBG_CAMERA_ONLY)
    ESP_LOGI(TAG, "STA CAMERA ONLY — streaming, no WebRTC");
 #else
    ESP_LOGI(TAG, "STA FULL — camera + WebRTC");
 #endif
#endif

    char url[64];
#ifdef DEVICE_ROLE_AP
    snprintf(url, sizeof(url), "http://192.168.4.1");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  WebRTC ready!");
    ESP_LOGI(TAG, "  1. Connect to WiFi: ROBOT_CAM");
    ESP_LOGI(TAG, "  2. Open %s", url);
    ESP_LOGI(TAG, "========================================");
#else
    snprintf(url, sizeof(url), "http://%s", g_sta_ip);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  WebRTC ready!");
    ESP_LOGI(TAG, "  STA IP: %s", g_sta_ip);
    ESP_LOGI(TAG, "  Open %s in browser", url);
    ESP_LOGI(TAG, "========================================");
#endif

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
