/*
 * WebRTC H.264 Video Streaming
 * ESP32-P4 + OV5647 + HW H.264 Encoder + esp_peer (ICE/DTLS/SRTP)
 * Browser: RTCPeerConnection (iOS Safari / Chrome / Firefox)
 */

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
#define CACHE_LINE_SIZE 128
#include "camera/app_video.h"

// WebRTC video interface
void webrtc_on_yuv_frame(const uint8_t *yuv_data, size_t yuv_len);
esp_err_t webrtc_video_init(int camera_fd);

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

// ── WiFi SoftAP Setup ──
static void wifi_ap(void)
{
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t c = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&c));

    // Unlock 5.8GHz channels for China regulatory domain
    wifi_country_t country = {
        .cc = "CN",
        .schan = 1,
        .nchan = 13,
        .policy = WIFI_COUNTRY_POLICY_MANUAL,
    };
    esp_wifi_set_country(&country);

    wifi_config_t a = {
        .ap = {
            .ssid = "ROBOT_CAM",
            .password = "",
            .ssid_len = 0,
            .channel = 149,          // 5.745GHz, CN domain unlocked
            .authmode = WIFI_AUTH_OPEN,
            .max_connection = 4,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    esp_wifi_set_band(WIFI_BAND_5G);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &a));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Confirm actual channel
    wifi_config_t a_read;
    esp_wifi_get_config(WIFI_IF_AP, &a_read);
    ESP_LOGI(TAG, "WiFi AP started on channel %d", a_read.ap.channel);
}

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

    // Initialize WiFi SoftAP
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_ap();

    // Initialize WebRTC + H.264 FIRST (DTLS cert needs fresh PSRAM before camera DMA)
    ESP_ERROR_CHECK(webrtc_video_init(fd));

    // Start camera streaming AFTER WebRTC init
    ESP_ERROR_CHECK(app_video_register_frame_operation_cb(camera_cb));
    ESP_ERROR_CHECK(app_video_stream_task_start(fd, 0));

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  WebRTC ready!");
    ESP_LOGI(TAG, "  1. Connect to WiFi: ROBOT_CAM");
    ESP_LOGI(TAG, "  2. Open http://192.168.4.1");
    ESP_LOGI(TAG, "========================================");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
