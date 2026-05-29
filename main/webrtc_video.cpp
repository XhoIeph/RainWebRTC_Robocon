/*
 * WebRTC Video Streaming (H.264 over DTLS-SRTP)
 * 
 * Architecture:
 *   Camera(ISP YUV420 O_UYY_E_VYY) ¡ú H.264 HW Encoder ¡ú esp_peer_send_video()
 *   ¡ú DTLS-SRTP ¡ú RTP/UDP ¡ú Browser RTCPeerConnection
 * 
 * Signaling: HTTP SSE (ESP¡úBrowser) + POST (Browser¡úESP)
 */

#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <errno.h>
#include <unistd.h>
#include <lwip/sockets.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "esp_h264_enc_single_hw.h"
#include "esp_h264_enc_param_hw.h"
#include "esp_peer_default.h"
#include "cJSON.h"
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#define TAG "WEBRTC"

/* ?? Configuration ??????????????????????????????????????????????? */
#define WEBRTC_VIDEO_WIDTH      1280
#define WEBRTC_VIDEO_HEIGHT     720         // 16:9 crop from 960p sensor (full H-FOV)
#define WEBRTC_VIDEO_FPS          30          // 30fps — buttery smooth
#define WEBRTC_VIDEO_GOP          15          // Keyframe every 1s
#define WEBRTC_VIDEO_BITRATE  (6 * 1024 * 1024)  // 6 Mbps — faster WiFi throughput
#define WEBRTC_VIDEO_QP_MIN       28          // Tighter compression for speed
#define WEBRTC_VIDEO_QP_MAX       35          // Motion scenes stay clean
#define WEBRTC_YUV_SIZE       (WEBRTC_VIDEO_WIDTH * WEBRTC_VIDEO_HEIGHT * 3 / 2)
#define WEBRTC_H264_MAX_SIZE  (2048 * 1024)  // 2MB — IDR frame headroom
#define WEBRTC_SSE_QUEUE_LEN      16        // Max pending messages in SSE queue (browser signaling)
#define WEBRTC_SSE_HEARTBEAT_MS  3000       // If no messages sent for this long, send heartbeat to keep connection alive

/* ©¤©¤ Global State ©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤ */
static esp_peer_handle_t    g_peer = NULL;
static esp_h264_enc_handle_t g_h264_enc = NULL;
static httpd_handle_t        g_httpd = NULL;
static QueueHandle_t         g_sse_queue = NULL;
static bool                  g_peer_connected = false;
static bool                  g_need_idr = false;  // Force IDR on new connection
static bool                  g_sse_connected = false;
static bool                  g_sse_stopping = false;
static httpd_req_t          *g_sse_req = NULL;
static SemaphoreHandle_t     g_frame_sem = NULL;  // unused — kept for compat
static TaskHandle_t          g_peer_task = NULL;
static TaskHandle_t          g_encoder_task = NULL;
static QueueHandle_t         g_frame_queue = NULL;  // Camera ? Encoder: frame pointers
static int                   g_camera_fd = -1;      // Camera fd for V4L2 controls
static bool                  g_is_ap_mode = false;   // Set by main.cpp: AP vs STA

/* ?? CAM2 (AP-side relay for STA device) ?? */
static esp_peer_handle_t    g_peer2 = NULL;
static bool                  g_peer2_connected = false;
static bool                  g_need_idr2 = false;
static QueueHandle_t         g_sse_queue2 = NULL;
static bool                  g_sse2_connected = false;
static bool                  g_sse2_stopping = false;
static httpd_req_t          *g_sse2_req = NULL;
static TaskHandle_t          g_peer2_task = NULL;
static SemaphoreHandle_t     g_peer_open_mutex = NULL;  // serialize esp_peer_open

/* UDP push: STA ? AP (192.168.4.1:9999) */
#define UDP_PUSH_PORT 9999
static int                   g_udp_push_sock = -1;
static struct sockaddr_in    g_udp_push_dest = {};

void webrtc_set_ap_mode(bool is_ap) { g_is_ap_mode = is_ap; }

/* Frame info passed through queue */
typedef struct {
    uint8_t  *yuv_data;
    size_t    yuv_len;
} frame_item_t;

/* H.264 output double-buffer (async send safety) */
static uint8_t              *g_h264_out[2] = {NULL, NULL};
static int                   g_h264_idx = 0;  // Toggle 0?1 each frame

/* Frame counter for FPS display */
static uint32_t g_frame_count = 0;
static int64_t  g_last_fps_time = 0;

/* ©¤©¤ Forward Declarations ©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤ */
static int peer_on_state(esp_peer_state_t state, void *ctx);
static int peer_on_msg(esp_peer_msg_t *msg, void *ctx);
static int peer_on_video_info(esp_peer_video_stream_info_t *info, void *ctx);
static esp_err_t webrtc_peer_reopen(void);

/* ================================================================
 *  SSE (Server-Sent Events) for ESP¡úBrowser signaling
 * ================================================================ */

static int sse_send(httpd_req_t *req, const char *data)
{
    int len = strlen(data) + strlen("data: ") + strlen("\n\n") + 1;
    char *buf = (char *)malloc(len);
    if (!buf) return -1;
    len = snprintf(buf, len, "data: %s\n\n", data);
    int ret = httpd_resp_send_chunk(req, buf, len);
    free(buf);
    return ret;
}

static void sse_send_task(void *arg)
{
    int64_t last_hb = esp_timer_get_time() / 1000;
    while (!g_sse_stopping) {
        char *msg = NULL;
        if (xQueueReceive(g_sse_queue, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (msg) {
                int ret = sse_send(g_sse_req, msg);
                free(msg);
                if (ret != ESP_OK) break;
            }
        }
        // Heartbeat every WEBRTC_SSE_HEARTBEAT_MS
        int64_t now = esp_timer_get_time() / 1000;
        if (now - last_hb > WEBRTC_SSE_HEARTBEAT_MS) {
            last_hb = now;
            if (sse_send(g_sse_req, "{\"type\":\"heartbeat\"}") != ESP_OK) break;
        }
    }
    httpd_req_async_handler_complete(g_sse_req);
    g_sse_req = NULL;
    g_sse_connected = false;
    g_sse_stopping = false;
    // Only reset peer if it was NOT yet connected (in negotiation phase)
    // If already streaming, let it continue — SSE is just for signaling
    if (g_peer && !g_peer_connected) {
        ESP_LOGI(TAG, "SSE disconnected during negotiation, resetting peer");
        esp_peer_disconnect(g_peer);
    } else if (g_peer && g_peer_connected) {
        ESP_LOGI(TAG, "SSE disconnected but peer still connected (streaming continues)");
    }
    ESP_LOGI(TAG, "SSE task exited");
    vTaskDelete(NULL);
}

static esp_err_t sse_get_handler(httpd_req_t *req)
{
    // Debug: log client IP on every SSE attempt
    int sockfd = httpd_req_to_sockfd(req);
    char client_ip[16] = "?.?.?.?";
    if (sockfd >= 0) {
        struct sockaddr_storage addr;
        socklen_t addr_len = sizeof(addr);
        if (getpeername(sockfd, (struct sockaddr *)&addr, &addr_len) == 0) {
            if (addr.ss_family == AF_INET) {
                inet_ntop(AF_INET, &((struct sockaddr_in *)&addr)->sin_addr, client_ip, sizeof(client_ip));
            }
        }
    }
    ESP_LOGI(TAG, "SSE connect from %s (g_sse_connected=%d)", client_ip, (int)g_sse_connected);

    httpd_resp_set_type(req, "text/event-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    if (g_sse_connected) {
        sse_send(req, "{\"error\":\"Only one SSE connection allowed\"}");
        httpd_resp_send_chunk(req, NULL, 0);
        return ESP_OK;
    }

    // If peer already streaming, tell browser to reuse existing connection
    // If not, tell browser to create a new peer and start negotiation
    if (g_peer && g_peer_connected) {
        sse_send(req, "{\"type\":\"status\",\"state\":\"connected\"}");
        ESP_LOGI(TAG, "SSE client connected, peer already streaming -- reusing");
    } else {
        sse_send(req, "{\"type\":\"connected\"}");
    }
    g_sse_connected = true;
    g_sse_stopping = false;
    httpd_req_async_handler_begin(req, &g_sse_req);

    if (g_sse_req) {
        xTaskCreate(sse_send_task, "sse_send", 4096, NULL, 5, NULL);
    }

    // Start or restart WebRTC when browser connects and peer isn't streaming
    if (g_peer && !g_peer_connected) {
        ESP_LOGI(TAG, "SSE client connected, starting WebRTC negotiation...");
        // Let old peer cleanup settle before creating new one
        esp_peer_close(g_peer);
        g_peer = NULL;
        vTaskDelay(pdMS_TO_TICKS(200));
        if (webrtc_peer_reopen() == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(50));
            esp_peer_new_connection(g_peer);
            ESP_LOGI(TAG, "Reconnect initiated — waiting for browser answer...");
        } else {
            ESP_LOGE(TAG, "Peer reopen failed — will retry on next SSE connect");
        }
    }
    return ESP_OK;
}

/* ================================================================
 *  POST /webrtc/signal ¡ª Browser¡úESP (SDP offer/answer, ICE candidates)
 * ================================================================ */

static esp_err_t signal_post_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    if (req->method == HTTP_OPTIONS) {
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "POST /signal len=%d", req->content_len);

    if (req->content_len > 16 * 1024) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too long");
        return ESP_OK;
    }

    char *buf = (char *)malloc(req->content_len + 1);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No memory");
        return ESP_OK;
    }

    int total = 0;
    while (total < req->content_len) {
        int r = httpd_req_recv(req, buf + total, req->content_len - total);
        if (r <= 0) {
            if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Receive failed");
            free(buf);
            return ESP_FAIL;
        }
        total += r;
    }
    buf[total] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        free(buf);
        return ESP_OK;
    }

    cJSON *type = cJSON_GetObjectItem(root, "type");
    if (!type || !type->valuestring) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing type");
        cJSON_Delete(root);
        free(buf);
        return ESP_OK;
    }

    esp_peer_msg_t msg = {};
    bool should_send = false;
    char *sdp_modified = NULL;  // Modified SDP (mDNS .local ? real IP)

    if (strcmp(type->valuestring, "offer") == 0 || strcmp(type->valuestring, "answer") == 0) {
        cJSON *sdp = cJSON_GetObjectItem(root, "sdp");
        if (!sdp || !sdp->valuestring) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing SDP");
            cJSON_Delete(root);
            free(buf);
            return ESP_OK;
        }

        // Get client IP for mDNS ? IP replacement
        // ESP DHCP assigns 192.168.4.2 as the first client address
        char client_ip[16] = {0};
        int sockfd = httpd_req_to_sockfd(req);
        if (sockfd >= 0) {
            struct sockaddr_storage addr;
            socklen_t addr_len = sizeof(addr);
            if (getpeername(sockfd, (struct sockaddr *)&addr, &addr_len) == 0) {
                if (addr.ss_family == AF_INET) {
                    struct sockaddr_in *addr_in = (struct sockaddr_in *)&addr;
                    inet_ntop(AF_INET, &addr_in->sin_addr, client_ip, sizeof(client_ip));
                }
            }
        }
        // Fallback: ESP DHCP always assigns 192.168.4.2 first
        if (!client_ip[0]) {
            strcpy(client_ip, "192.168.4.2");
            ESP_LOGW(TAG, "getpeername failed, using fallback IP: %s", client_ip);
        } else {
            ESP_LOGI(TAG, "Client IP: %s", client_ip);
        }

        // Make mutable copy of SDP; replace mDNS .local hostnames with real IP
        sdp_modified = strdup(sdp->valuestring);
        if (sdp_modified && client_ip[0]) {
            char *p = sdp_modified;
            while ((p = strstr(p, ".local")) != NULL) {
                // Find start of hostname (the mDNS name before .local)
                char *host_start = p;
                while (host_start > sdp_modified && host_start[-1] != ' ' && host_start[-1] != '\n' && host_start[-1] != '\r') {
                    host_start--;
                }
                // Find end of hostname+".local" (space or newline after .local)
                char *host_end = p + 6;  // after ".local"
                size_t tail_len = strlen(host_end);
                size_t ip_len = strlen(client_ip);
                // Overwrite hostname.local with IP, then shift tail
                memcpy(host_start, client_ip, ip_len);
                memmove(host_start + ip_len, host_end, tail_len + 1);  // +1 for null terminator
                p = host_start + ip_len;  // continue search right after the IP we just inserted
            }
        }

        msg.type = ESP_PEER_MSG_TYPE_SDP;
        msg.data = (uint8_t *)(sdp_modified ? sdp_modified : sdp->valuestring);
        msg.size = strlen((char *)msg.data);
        should_send = true;

        // Log browser's answer/offer for debugging
        ESP_LOGI(TAG, "=== Remote SDP (%s, %d bytes) ===", type->valuestring, msg.size);
        char *ans_copy = strdup((char *)msg.data);
        if (ans_copy) {
            char *save = NULL;
            char *line = strtok_r(ans_copy, "\r\n", &save);
            while (line) { ESP_LOGI(TAG, "  %s", line); line = strtok_r(NULL, "\r\n", &save); }
            free(ans_copy);
        }

        // Extract ICE candidates from (possibly modified) answer SDP
        if (strcmp(type->valuestring, "answer") == 0) {
            char *sdp_copy = strdup((char *)msg.data);
            if (sdp_copy) {
                char *save = NULL;
                char *line = strtok_r(sdp_copy, "\r\n", &save);
                while (line) {
                    if (strncmp(line, "a=candidate:", 12) == 0) {
                        esp_peer_msg_t cand_msg = {};
                        cand_msg.type = ESP_PEER_MSG_TYPE_CANDIDATE;
                        cand_msg.data = (uint8_t *)(line + 2); // strip "a="
                        cand_msg.size = strlen(line) - 2;
                        esp_peer_send_msg(g_peer, &cand_msg);
                    }
                    line = strtok_r(NULL, "\r\n", &save);
                }
                free(sdp_copy);
            }
        }
    } else if (strcmp(type->valuestring, "candidate") == 0) {
        cJSON *cand = cJSON_GetObjectItem(root, "candidate");
        if (!cand || !cand->valuestring) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing candidate");
            cJSON_Delete(root);
            free(buf);
            return ESP_OK;
        }
        msg.type = ESP_PEER_MSG_TYPE_CANDIDATE;
        msg.data = (uint8_t *)cand->valuestring;
        msg.size = strlen(cand->valuestring);
        should_send = true;
    } else if (strcmp(type->valuestring, "bye") == 0) {
        esp_peer_disconnect(g_peer);
        g_peer_connected = false;
    }

    if (should_send && g_peer) {
        int ret = esp_peer_send_msg(g_peer, &msg);
        if (ret != ESP_PEER_ERR_NONE) {
            ESP_LOGW(TAG, "esp_peer_send_msg failed: %d", ret);
        }
    }

    httpd_resp_sendstr(req, "OK");
    cJSON_Delete(root);
    free(buf);
    if (sdp_modified) free(sdp_modified);
    return ESP_OK;
}

/* ================================================================
 *  Push SDP/Candidate to browser via SSE queue
 * ================================================================ */

static void sse_push_json(const char *json)
{
    if (g_sse_connected && g_sse_queue) {
        char *cpy = strdup(json);
        if (cpy) {
            xQueueSend(g_sse_queue, &cpy, 0);
        }
    }
}

static void sse_push_sdp(const char *sdp)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "offer");  // ESP is controlling: sends offer
    cJSON_AddStringToObject(root, "sdp", sdp);
    char *json = cJSON_PrintUnformatted(root);
    if (json) {
        sse_push_json(json);
        free(json);
    }
    cJSON_Delete(root);
}

static void sse_push_candidate(const char *candidate)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "candidate");
    cJSON_AddStringToObject(root, "candidate", candidate);
    char *json = cJSON_PrintUnformatted(root);
    if (json) {
        sse_push_json(json);
        free(json);
    }
    cJSON_Delete(root);
}

/* ================================================================
 *  esp_peer Callbacks
 * ================================================================ */

static int peer_on_state(esp_peer_state_t state, void *ctx)
{
    ESP_LOGI(TAG, "Peer state: %d", (int)state);
    switch (state) {
    case ESP_PEER_STATE_CONNECTED:
        g_peer_connected = true;
        g_need_idr = true;  // Force IDR for new connection decode start
        ESP_LOGI(TAG, "=== WebRTC CONNECTED - video streaming ===");
        break;
    case ESP_PEER_STATE_DISCONNECTED:
    case ESP_PEER_STATE_CONNECT_FAILED:
        g_peer_connected = false;
        ESP_LOGI(TAG, "=== WebRTC %s ===",
            state == ESP_PEER_STATE_DISCONNECTED ? "DISCONNECTED" : "CONNECT FAILED");
        // Let browser re-trigger reconnect via SSE — don't auto-reconnect here
        break;
    default:
        break;
    }
    return 0;
}

static int peer_on_msg(esp_peer_msg_t *msg, void *ctx)
{
    if (!msg || !g_sse_connected) return 0;

    if (msg->type == ESP_PEER_MSG_TYPE_SDP) {
        // Make a mutable copy for modification
        char *sdp_mod = strndup((char *)msg->data, msg->size);
        if (!sdp_mod) return 0;

        // Replace Main Profile (4d001f) with Constrained Baseline (42e01f)
        // Also upgrade H.264 level: 42e01f (Lev3.1 max 720p) ? 42e028 (Lev4.0 max 1080p)
        // Browser decoder reads real 1920×1080 from H.264 SPS in bitstream
        char *pos = sdp_mod;
        while ((pos = strstr(pos, "profile-level-id=")) != NULL) {
            if (memcmp(pos + 17, "4d001f", 6) == 0) {
                memcpy(pos + 17, "42e028", 6);  // Main ? CBP Lev4.0
            } else if (memcmp(pos + 17, "42e01f", 6) == 0) {
                memcpy(pos + 17, "42e028", 6);  // CBP Lev3.1 ? Lev4.0
            }
            pos += 23;
        }

        // Log full SDP (possibly modified)
        char *sdp_log = strdup(sdp_mod);
        if (sdp_log) {
            ESP_LOGI(TAG, "=== SDP (%d bytes) ===", msg->size);
            char *save = NULL;
            char *line = strtok_r(sdp_log, "\r\n", &save);
            while (line) {
                ESP_LOGI(TAG, "%s", line);
                line = strtok_r(NULL, "\r\n", &save);
            }
            free(sdp_log);
        }
        // Send (possibly modified) SDP to browser via SSE
        sse_push_sdp(sdp_mod);
        free(sdp_mod);
    } else if (msg->type == ESP_PEER_MSG_TYPE_CANDIDATE) {
        ESP_LOGI(TAG, "ICE candidate: %.*s", msg->size, (char *)msg->data);
        sse_push_candidate((char *)msg->data);
    }
    return 0;
}

static int peer_on_video_info(esp_peer_video_stream_info_t *info, void *ctx)
{
    ESP_LOGI(TAG, "Video info: %dx%d codec=%d", info->width, info->height, (int)info->codec);
    return 0;
}

/* ================================================================
 *  Peer Connection Setup
 * ================================================================ */

static void peer_task(void *arg)
{
    ESP_LOGI(TAG, "Peer task started");
    while (1) {
        if (g_peer) {
            esp_peer_main_loop(g_peer);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static esp_err_t webrtc_peer_init(void)
{
    // Pre-generate DTLS certificate (takes ~2 seconds, do once)
    esp_peer_pre_generate_cert();

    esp_peer_default_cfg_t extra = {};
    extra.agent_recv_timeout = 1000;
    extra.rtp_cfg.send_pool_size = 512 * 1024;
    extra.rtp_cfg.send_queue_num = 128;
    extra.alive_binding_retries = 10;
    extra.ice_use_lite_mode = true;

    esp_peer_cfg_t cfg = {
        .role = ESP_PEER_ROLE_CONTROLLING,
        .audio_info = {
            .codec = ESP_PEER_AUDIO_CODEC_G711A,
            .sample_rate = 8000,
            .channel = 1,
        },
        .video_info = {
            .codec = ESP_PEER_VIDEO_CODEC_H264,
            .width = 1280,
            .height = 960,
            .fps = WEBRTC_VIDEO_FPS,
        },
        .audio_dir = ESP_PEER_MEDIA_DIR_SEND_ONLY,
        .video_dir = ESP_PEER_MEDIA_DIR_SEND_ONLY,
        .no_auto_reconnect = true,
        .enable_data_channel = false,
        .extra_cfg = &extra,
        .extra_size = sizeof(extra),
        .ctx = NULL,
        .on_state = peer_on_state,
        .on_msg = peer_on_msg,
        .on_video_info = peer_on_video_info,
    };

    xSemaphoreTake(g_peer_open_mutex, portMAX_DELAY);
    int ret = esp_peer_open(&cfg, esp_peer_get_default_impl(), &g_peer);
    xSemaphoreGive(g_peer_open_mutex);
    if (ret != ESP_PEER_ERR_NONE) {
        ESP_LOGE(TAG, "esp_peer_open failed: %d", ret);
        return ESP_FAIL;
    }

    // Create peer task on CPU0 (shares core with WiFi/lwIP, priority 12)
    if (xTaskCreatePinnedToCore(peer_task, "webrtc_peer", 8192, NULL, 12, &g_peer_task, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create peer task");
        esp_peer_close(g_peer);
        g_peer = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "WebRTC peer initialized");
    return ESP_OK;
}

/* Re-open peer without creating duplicate task (for auto-reconnect) */
static esp_err_t webrtc_peer_reopen(void)
{
    esp_peer_default_cfg_t default_cfg = {
        .agent_recv_timeout = 500,
        .rtp_cfg = {
            .audio_recv_jitter = { .cache_size = 1024 },
            .video_recv_jitter = { .cache_size = 1024 },
            .send_pool_size = 512 * 1024,
            .send_queue_num = 128,
        },
        .alive_binding_retries = 10,
        .ice_use_lite_mode = true,
    };

    esp_peer_cfg_t cfg = {
        .role = ESP_PEER_ROLE_CONTROLLING,
        .audio_info = { .codec = ESP_PEER_AUDIO_CODEC_G711A, .sample_rate = 8000, .channel = 1 },
        .video_info = { .codec = ESP_PEER_VIDEO_CODEC_H264, .width = 1280, .height = 960, .fps = WEBRTC_VIDEO_FPS },
        .audio_dir = ESP_PEER_MEDIA_DIR_SEND_ONLY,
        .video_dir = ESP_PEER_MEDIA_DIR_SEND_ONLY,
        .no_auto_reconnect = true,
        .enable_data_channel = false,
        .extra_cfg = &default_cfg,
        .extra_size = sizeof(esp_peer_default_cfg_t),
        .ctx = NULL,
        .on_state = peer_on_state,
        .on_msg = peer_on_msg,
        .on_video_info = peer_on_video_info,
    };

    xSemaphoreTake(g_peer_open_mutex, portMAX_DELAY);
    int ret = esp_peer_open(&cfg, esp_peer_get_default_impl(), &g_peer);
    xSemaphoreGive(g_peer_open_mutex);
    if (ret != ESP_PEER_ERR_NONE) {
        ESP_LOGE(TAG, "Peer reopen failed: %d", ret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Peer reopened — existing task will pick it up");
    return ESP_OK;
}

/* ================================================================
 *  H.264 Hardware Encoder Setup
 * ================================================================ */

static esp_err_t h264_encoder_init(void)
{
    esp_h264_enc_cfg_hw_t cfg = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
        .gop = WEBRTC_VIDEO_GOP,
        .fps = WEBRTC_VIDEO_FPS,
        .res = {
            .width = WEBRTC_VIDEO_WIDTH,
            .height = WEBRTC_VIDEO_HEIGHT,
        },
        .rc = {
            .bitrate = WEBRTC_VIDEO_BITRATE,
            .qp_min = WEBRTC_VIDEO_QP_MIN,
            .qp_max = WEBRTC_VIDEO_QP_MAX,
        },
    };

    esp_h264_err_t ret = esp_h264_enc_hw_new(&cfg, &g_h264_enc);
    if (ret != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "H.264 encoder create failed: %d", ret);
        return ESP_FAIL;
    }

    ret = esp_h264_enc_open(g_h264_enc);
    if (ret != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "H.264 encoder open failed: %d", ret);
        return ESP_FAIL;
    }

    // Allocate TWO output buffers (128-byte aligned, true double-buffering)
    // This prevents async send from overwriting buffer while in use
    for (int i = 0; i < 2; i++) {
        g_h264_out[i] = (uint8_t *)heap_caps_aligned_alloc(128, WEBRTC_H264_MAX_SIZE,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!g_h264_out[i]) {
            ESP_LOGE(TAG, "H.264 output buffer[%d] alloc failed", i);
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "H.264 encoder ready (%dx%d @%dfps, %dkbps)",
        WEBRTC_VIDEO_WIDTH, WEBRTC_VIDEO_HEIGHT,
        WEBRTC_VIDEO_FPS, (int)(WEBRTC_VIDEO_BITRATE / 1024));
    return ESP_OK;
}

/* ================================================================
 *  Frame Callback ¡ª called from camera task with YUV420 frame
 * ================================================================ */

/* Frame queue: camera callback ? encoder task */
#define FRAME_QUEUE_LEN  6  // 6 slots — absorb bursts without dropping

void webrtc_on_yuv_frame(const uint8_t *yuv_data, size_t yuv_len)
{
    if (!g_frame_queue) return;

    // Pass pointer directly — camera's 4 DMA buffers provide enough time
    // for the encoder task to process before the buffer is reused
    frame_item_t item = { .yuv_data = (uint8_t *)yuv_data, .yuv_len = yuv_len };
    xQueueSend(g_frame_queue, &item, 0);  // Non-blocking, drop if full
}

/* ?? Dedicated encoder task: always encode the latest frame, skip backlog ?? */
static void encoder_task(void *arg)
{
    frame_item_t item;
    static bool first_send_logged = false;

    while (1) {
        if (xQueueReceive(g_frame_queue, &item, pdMS_TO_TICKS(10)) != pdTRUE) {
            continue;
        }

        // Drain queue to get the freshest frame (skip stale backlog)
        // This prevents cascade slowdown when encoder is slower than camera
        frame_item_t latest;
        while (xQueueReceive(g_frame_queue, &latest, 0) == pdTRUE) {
            item = latest;
        }

        if (!g_h264_enc || !g_h264_out[0]) continue;

        // Double-buffer toggle
        int idx = g_h264_idx;
        g_h264_idx ^= 1;

        // Encode
        esp_h264_enc_in_frame_t in_frame = {
            .raw_data = { .buffer = item.yuv_data, .len = (uint32_t)item.yuv_len },
            .pts = (uint32_t)(esp_timer_get_time() / 1000),
        };
        esp_h264_enc_out_frame_t out_frame = {
            .raw_data = { .buffer = g_h264_out[idx], .len = WEBRTC_H264_MAX_SIZE },
        };

        esp_h264_err_t ret = esp_h264_enc_process(g_h264_enc, &in_frame, &out_frame);
        if (ret != ESP_H264_ERR_OK) {
            if (ret != ESP_H264_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "H.264 encode error: %d", ret);
            }
            continue;
        }

        // Wait for IDR on new connection (browser can't decode P-frames without reference)
        if (g_need_idr && out_frame.frame_type != ESP_H264_FRAME_TYPE_IDR) {
            continue;  // Skip P-frames until IDR arrives
        }
        g_need_idr = false;

        // No frame-size drop — QP 18-28 + 6Mbps bitrate control handles frame sizing naturally

        // Send
        if (g_peer_connected && out_frame.length > 0) {
            esp_peer_video_frame_t vf = {
                .pts = out_frame.pts,
                .data = out_frame.raw_data.buffer,
                .size = (int)out_frame.length,
            };
            int sr = esp_peer_send_video(g_peer, &vf);
            if (sr == ESP_PEER_ERR_WOULD_BLOCK) {
                // Drop silently
            } else if (sr != ESP_PEER_ERR_NONE) {
                if (!first_send_logged)
                    ESP_LOGW(TAG, "Send err: %d", sr);
            } else if (!first_send_logged) {
                ESP_LOGI(TAG, "First video frame sent: %d B, type=%s",
                    (int)out_frame.length,
                    out_frame.frame_type == ESP_H264_FRAME_TYPE_IDR ? "IDR" : "P");
                first_send_logged = true;
            }
        }

        // UDP push disabled — C5 AP handles routing independently
        if (0 && !g_is_ap_mode && out_frame.length > 0 && (g_frame_count & 1)) {
            if (g_udp_push_sock < 0) {
                g_udp_push_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
                if (g_udp_push_sock >= 0) {
                    g_udp_push_dest.sin_family = AF_INET;
                    g_udp_push_dest.sin_port = htons(UDP_PUSH_PORT);
                    inet_pton(AF_INET, "192.168.4.1", &g_udp_push_dest.sin_addr);
                }
            }
            if (g_udp_push_sock >= 0) {
                // Send with 4-byte header: [uint32_t length]
                uint32_t hdr = (uint32_t)out_frame.length;
                struct iovec iov[2] = {
                    { .iov_base = &hdr, .iov_len = 4 },
                    { .iov_base = out_frame.raw_data.buffer, .iov_len = out_frame.length },
                };
                struct msghdr msg = {};
                msg.msg_name = &g_udp_push_dest;
                msg.msg_namelen = sizeof(g_udp_push_dest);
                msg.msg_iov = iov;
                msg.msg_iovlen = 2;
                sendmsg(g_udp_push_sock, &msg, 0);
            }
        }

        // FPS stats
        g_frame_count++;
        int64_t now = esp_timer_get_time();
        if (g_last_fps_time == 0) g_last_fps_time = now;
        if (now - g_last_fps_time >= 5000000) {
            float fps = (float)g_frame_count * 1000000.0f / (float)(now - g_last_fps_time);
            ESP_LOGI(TAG, "WebRTC: %.1f fps, H.264: %u B, type=%s",
                fps, (unsigned)out_frame.length,
                out_frame.frame_type == ESP_H264_FRAME_TYPE_IDR ? "IDR" :
                out_frame.frame_type == ESP_H264_FRAME_TYPE_I ? "I" : "P");
            g_frame_count = 0;
            g_last_fps_time = now;
        }
    }
}

/* ================================================================
 *  Camera V4L2 Control Helpers (OV5647 sensor via ioctl)
 *  ?? Only V4L2_CID_EXPOSURE_ABSOLUTE SET is confirmed working.
 *  ?? Most GET / other CIDs fail on this sensor; we cache SET values.
 * ================================================================ */

/* Cached control values (sensor GET fails for most CIDs after stream starts) */
typedef struct {
    const char *name;
    uint32_t    cid;
    const char *desc;
    int32_t     cached;       // Last known value
    bool        set_works;    // Confirmed SET works
    int32_t     min_val;
    int32_t     max_val;
    int32_t     step;
} camera_ctrl_t;

static camera_ctrl_t g_ctrls[] = {
    {"exposure",   V4L2_CID_EXPOSURE_ABSOLUTE, "Exposure (x100us)", 5,  true,  1,   50, 1},       //
};
#define NUM_CTRLS (sizeof(g_ctrls) / sizeof(g_ctrls[0]))

/* Find control by name */
static camera_ctrl_t *ctrl_find(const char *name)
{
    for (int i = 0; i < (int)NUM_CTRLS; i++) {
        if (strcmp(g_ctrls[i].name, name) == 0) return &g_ctrls[i];
    }
    return NULL;
}

/* Set a single V4L2 control (quietly probe on first use) */
static esp_err_t camera_ctrl_set(camera_ctrl_t *c, int32_t value)
{
    if (g_camera_fd < 0) return ESP_ERR_INVALID_STATE;

    struct v4l2_ext_controls ctrls = {};
    struct v4l2_ext_control ctrl  = {};
    ctrls.ctrl_class = V4L2_CTRL_CLASS_USER;
    ctrls.count      = 1;
    ctrls.controls   = &ctrl;
    ctrl.id          = c->cid;
    ctrl.value       = value;

    if (ioctl(g_camera_fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
        if (c->set_works) {
            // Was working before — now failing, warn
            ESP_LOGW(TAG, "cam ctrl SET %s val=%ld FAILED errno=%d", c->name, (long)value, errno);
        }
        // First failure is silent — we just mark it unavailable
        c->set_works = false;
        return ESP_FAIL;
    }
    c->set_works = true;
    c->cached = value;
    ESP_LOGI(TAG, "Cam: %s = %ld OK", c->name, (long)value);
    return ESP_OK;
}

/* ?? POST /camera/control ?? */
static esp_err_t camera_control_post_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    if (req->method == HTTP_OPTIONS) {
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }

    if (req->content_len > 512) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Too long");
        return ESP_OK;
    }

    char buf[512];
    int total = 0;
    while (total < req->content_len) {
        int r = httpd_req_recv(req, buf + total, req->content_len - total);
        if (r <= 0) { if (r == HTTPD_SOCK_ERR_TIMEOUT) continue; return ESP_FAIL; }
        total += r;
    }
    buf[total] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad JSON"); return ESP_OK; }

    cJSON *cid_j = cJSON_GetObjectItem(root, "cid");
    cJSON *val_j = cJSON_GetObjectItem(root, "value");
    if (!cid_j || !cid_j->valuestring || !val_j) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Need cid+value");
        cJSON_Delete(root); return ESP_OK;
    }

    camera_ctrl_t *c = ctrl_find(cid_j->valuestring);
    if (!c) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unknown cid");
        cJSON_Delete(root); return ESP_OK;
    }

    int32_t value = (int32_t)(val_j->valuedouble);
    // Clamp to range
    if (value < c->min_val) value = c->min_val;
    if (value > c->max_val) value = c->max_val;

    esp_err_t ret = camera_ctrl_set(c, value);

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddStringToObject(resp, "result", (ret == ESP_OK) ? "ok" : (c->set_works ? "ioerr" : "unsupported"));
    cJSON_AddStringToObject(resp, "cid", c->name);
    cJSON_AddNumberToObject(resp, "value", (ret == ESP_OK) ? value : c->cached);
    char *js = cJSON_PrintUnformatted(resp);
    if (js) { httpd_resp_sendstr(req, js); free(js); }
    cJSON_Delete(resp);
    cJSON_Delete(root);
    return ESP_OK;
}

/* ?? GET /camera/status ?? */
static esp_err_t camera_status_get_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json; charset=utf-8");

    cJSON *root = cJSON_CreateObject();
    cJSON *arr  = cJSON_CreateArray();

    for (int i = 0; i < (int)NUM_CTRLS; i++) {
        camera_ctrl_t *c = &g_ctrls[i];
        cJSON *item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "cid", c->name);
        cJSON_AddStringToObject(item, "desc", c->desc);
        cJSON_AddNumberToObject(item, "value", c->cached);
        cJSON_AddNumberToObject(item, "min", c->min_val);
        cJSON_AddNumberToObject(item, "max", c->max_val);
        cJSON_AddNumberToObject(item, "step", c->step);
        cJSON_AddStringToObject(item, "status", c->set_works ? "rw" : "ro");  // ro = untested/unsupported
        cJSON_AddItemToArray(arr, item);
    }

    cJSON_AddItemToObject(root, "controls", arr);
    cJSON_AddStringToObject(root, "camera", "OV5647");

    char *js = cJSON_PrintUnformatted(root);
    if (js) { httpd_resp_sendstr(req, js); free(js); }
    cJSON_Delete(root);
    return ESP_OK;
}

/* ?? POST /encoder/control ?? bitrate/gop/force_idr ?? */
static esp_err_t encoder_control_post_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    if (req->method == HTTP_OPTIONS) {
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }

    if (req->content_len > 256) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Too long");
        return ESP_OK;
    }

    char buf[256];
    int total = 0;
    while (total < req->content_len) {
        int r = httpd_req_recv(req, buf + total, req->content_len - total);
        if (r <= 0) { if (r == HTTPD_SOCK_ERR_TIMEOUT) continue; return ESP_FAIL; }
        total += r;
    }
    buf[total] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad JSON"); return ESP_OK; }

    cJSON *cmd_j = cJSON_GetObjectItem(root, "cmd");
    cJSON *val_j = cJSON_GetObjectItem(root, "value");
    if (!cmd_j || !cmd_j->valuestring) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Need cmd");
        cJSON_Delete(root); return ESP_OK;
    }

    // Get param handle for runtime control
    esp_h264_enc_param_hw_handle_t param_hw = NULL;
    if (g_h264_enc) {
        esp_h264_enc_hw_get_param_hd(g_h264_enc, &param_hw);
    }
    esp_h264_enc_param_handle_t param = (esp_h264_enc_param_handle_t)param_hw;

    const char *cmd = cmd_j->valuestring;
    cJSON *resp = cJSON_CreateObject();
    cJSON_AddStringToObject(resp, "cmd", cmd);

    if (strcmp(cmd, "force_idr") == 0) {
        g_need_idr = true;
        cJSON_AddStringToObject(resp, "result", "ok");
        ESP_LOGI(TAG, "Encoder: forced IDR");
    } else if (strcmp(cmd, "bitrate") == 0 && val_j) {
        int bitrate = (int)(val_j->valuedouble);
        if (bitrate < 500000) bitrate = 500000;
        if (bitrate > 15000000) bitrate = 15000000;
        if (param) {
            esp_h264_enc_set_bitrate(param, bitrate);
            cJSON_AddStringToObject(resp, "result", "ok");
            ESP_LOGI(TAG, "Encoder: bitrate set to %d bps", bitrate);
        } else {
            cJSON_AddStringToObject(resp, "result", "error");
            cJSON_AddStringToObject(resp, "msg", "encoder not ready");
        }
        cJSON_AddNumberToObject(resp, "value", bitrate);
    } else if (strcmp(cmd, "gop") == 0 && val_j) {
        int gop = (int)(val_j->valuedouble);
        if (gop < 1) gop = 1;
        if (gop > 255) gop = 255;
        if (param) {
            esp_h264_enc_set_gop(param, (uint8_t)gop);
            cJSON_AddStringToObject(resp, "result", "ok");
            ESP_LOGI(TAG, "Encoder: GOP set to %d", gop);
        } else {
            cJSON_AddStringToObject(resp, "result", "error");
            cJSON_AddStringToObject(resp, "msg", "encoder not ready");
        }
        cJSON_AddNumberToObject(resp, "value", gop);
    } else {
        cJSON_AddStringToObject(resp, "result", "error");
        cJSON_AddStringToObject(resp, "msg", "unknown cmd");
    }

    char *js = cJSON_PrintUnformatted(resp);
    if (js) { httpd_resp_sendstr(req, js); free(js); }
    cJSON_Delete(resp);
    cJSON_Delete(root);
    return ESP_OK;
}

/* ?? GET /encoder/status ?? current bitrate/gop/fps ?? */
static esp_err_t encoder_status_get_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json; charset=utf-8");

    cJSON *root = cJSON_CreateObject();
    if (g_h264_enc) {
        esp_h264_enc_param_hw_handle_t param_hw = NULL;
        esp_h264_enc_hw_get_param_hd(g_h264_enc, &param_hw);
        esp_h264_enc_param_handle_t param = (esp_h264_enc_param_handle_t)param_hw;
        if (param) {
            uint32_t bitrate = 0;
            uint8_t gop = 0, fps = 0;
            esp_h264_enc_get_bitrate(param, &bitrate);
            esp_h264_enc_get_gop(param, &gop);
            esp_h264_enc_get_fps(param, &fps);
            cJSON_AddNumberToObject(root, "bitrate", bitrate);
            cJSON_AddNumberToObject(root, "gop", gop);
            cJSON_AddNumberToObject(root, "fps", fps);
            cJSON_AddNumberToObject(root, "width", WEBRTC_VIDEO_WIDTH);
            cJSON_AddNumberToObject(root, "height", WEBRTC_VIDEO_HEIGHT);
        }
    }
    cJSON_AddStringToObject(root, "status", g_h264_enc ? "ok" : "no_encoder");

    char *js = cJSON_PrintUnformatted(root);
    if (js) { httpd_resp_sendstr(req, js); free(js); }
    cJSON_Delete(root);
    return ESP_OK;
}

/* ================================================================
 *  Proxy endpoints for STA device (192.168.4.2)
 *  Device A (AP) proxies all HTTP traffic to Device B (STA)
 *  because ESP-Hosted C5 firmware does NOT support STA-to-STA IP forwarding.
 * ================================================================ */

// Forward decl
static int proxy_connect_to_sta(void);

#define STA_PROXY_IP   "192.168.4.2"
#define STA_PROXY_PORT 80

/* POST proxy: forward POST /proxy/sta/signal -> http://192.168.4.2/webrtc/signal */
static esp_err_t proxy_sta_post_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    if (req->method == HTTP_OPTIONS) {
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }

    // Read request body
    if (req->content_len > 64 * 1024) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Too large");
        return ESP_OK;
    }
    char *body = (char *)malloc(req->content_len + 1);
    if (!body) return ESP_FAIL;
    int total = 0;
    while (total < req->content_len) {
        int r = httpd_req_recv(req, body + total, req->content_len - total);
        if (r <= 0) { if (r == HTTPD_SOCK_ERR_TIMEOUT) continue; free(body); return ESP_FAIL; }
        total += r;
    }
    body[total] = '\0';

    // Forward to STA device via proxy helper (with ARP fix + retry)
    int sock = proxy_connect_to_sta();
    if (sock < 0) { free(body); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "STA unreachable"); return ESP_OK; }

    // Build HTTP POST request
    char hdr[1024];
    int hdr_len = snprintf(hdr, sizeof(hdr),
        "POST /webrtc/signal HTTP/1.0\r\n"
        "Host: %s\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n\r\n",
        STA_PROXY_IP, total);
    send(sock, hdr, hdr_len, 0);
    send(sock, body, total, 0);
    free(body);

    // Read response headers
    char resp_buf[4096];
    int resp_total = 0;
    while (resp_total < (int)sizeof(resp_buf) - 1) {
        int r = recv(sock, resp_buf + resp_total, sizeof(resp_buf) - 1 - resp_total, 0);
        if (r <= 0) break;
        resp_total += r;
    }
    resp_buf[resp_total] = '\0';
    close(sock);

    // Find body after \r\n\r\n
    char *body_start = strstr(resp_buf, "\r\n\r\n");
    if (body_start) {
        body_start += 4;
        httpd_resp_send(req, body_start, resp_total - (body_start - resp_buf));
    } else {
        httpd_resp_sendstr(req, "{}");
    }
    return ESP_OK;
}

/* Helper: connect to STA device with retry */
static int proxy_connect_to_sta(void)
{
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) return -1;

    struct timeval tv = { .tv_sec = 3, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    struct sockaddr_in sta_addr = {};
    sta_addr.sin_family = AF_INET;
    sta_addr.sin_port = htons(STA_PROXY_PORT);
    inet_pton(AF_INET, STA_PROXY_IP, &sta_addr.sin_addr);

    // Retry up to 5 times with increasing delay
    for (int retry = 0; retry < 5; retry++) {
        if (connect(sock, (struct sockaddr *)&sta_addr, sizeof(sta_addr)) == 0) {
            return sock;
        }
        ESP_LOGW(TAG, "STA connect retry %d/5: errno=%d", retry + 1, errno);
        vTaskDelay(pdMS_TO_TICKS(500 + retry * 200));
        // New socket each retry
        if (retry < 4) {
            close(sock);
            sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            if (sock < 0) return -1;
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        }
    }
    ESP_LOGE(TAG, "STA connect FAILED after 5 retries: errno=%d", errno);
    close(sock);
    return -1;
}

static esp_err_t proxy_sta_sse_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/event-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    int sock = proxy_connect_to_sta();
    if (sock < 0) {
        httpd_resp_sendstr(req, "data: {\"error\":\"sta_unreachable\"}\n\n");
        return ESP_OK;
    }

    // Send SSE request
    const char *req_str = "GET /webrtc/signal HTTP/1.0\r\nHost: 192.168.4.2\r\nAccept: text/event-stream\r\nConnection: close\r\n\r\n";
    send(sock, req_str, strlen(req_str), 0);

    // Relay SSE events
    char buf[4096];
    int chunk_start = 0;
    int received = 0;
    for (int i = 0; i < 6000; i++) {  // max ~10 minutes
        int r = recv(sock, buf + received, sizeof(buf) - 1 - received, 0);
        if (r <= 0) break;
        received += r;
        buf[received] = '\0';

        // Send each complete line
        char *line_start = buf + chunk_start;
        char *p = line_start;
        while (p < buf + received) {
            if (*p == '\n') {
                *p = '\0';
                httpd_resp_send_chunk(req, line_start, strlen(line_start));
                httpd_resp_send_chunk(req, "\n", 1);
                line_start = p + 1;
            }
            p++;
        }
        // Keep unprocessed remainder
        if (line_start > buf) {
            int remain = (buf + received) - line_start;
            if (remain > 0) memmove(buf, line_start, remain);
            received = remain;
        } else {
            received = 0;
        }
        chunk_start = 0;
    }

    close(sock);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* ================================================================
 *  HTTP Server Setup
 * ================================================================ */

// Forward declarations for CAM2 & proxy
static int proxy_connect_to_sta(void);
static esp_err_t cam2_sse_get_handler(httpd_req_t *req);
static esp_err_t cam2_signal_post_handler(httpd_req_t *req);
static esp_err_t cam2_peer_init(void);
static void udp_relay_task(void *arg);

// Serve the embedded WebRTC test HTML page
static esp_err_t html_get_handler(httpd_req_t *req)
{
    extern const unsigned char webrtc_test_html_start[] asm("_binary_webrtc_test_html_start");
    extern const unsigned char webrtc_test_html_end[] asm("_binary_webrtc_test_html_end");
    const size_t webrtc_test_html_size = (webrtc_test_html_end - webrtc_test_html_start);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)webrtc_test_html_start, webrtc_test_html_size);
    return ESP_OK;
}

static esp_err_t webrtc_http_init(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 20;  // + CAM2 + proxy
    config.stack_size = 7168;
    config.lru_purge_enable = true;

    esp_err_t ret = httpd_start(&g_httpd, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed on port 80: %d, trying port 8080", (int)ret);
        config.server_port = 8080;
        ret = httpd_start(&g_httpd, &config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "HTTP server start failed on port 8080: %d", (int)ret);
            return ret;
        }
        ESP_LOGI(TAG, "HTTP server started on port 8080 instead");
    }

    // WebRTC test page: GET /
    httpd_uri_t html_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = html_get_handler,
    };
    httpd_register_uri_handler(g_httpd, &html_uri);

    // WebRTC test page: GET /webrtc
    httpd_uri_t webrtc_html_uri = {
        .uri = "/webrtc",
        .method = HTTP_GET,
        .handler = html_get_handler,
    };
    httpd_register_uri_handler(g_httpd, &webrtc_html_uri);

    // SSE endpoint: GET /webrtc/signal
    httpd_uri_t sse_uri = {
        .uri = "/webrtc/signal",
        .method = HTTP_GET,
        .handler = sse_get_handler,
    };
    httpd_register_uri_handler(g_httpd, &sse_uri);

    // POST endpoint: POST /webrtc/signal
    httpd_uri_t post_uri = {
        .uri = "/webrtc/signal",
        .method = HTTP_POST,
        .handler = signal_post_handler,
    };
    httpd_register_uri_handler(g_httpd, &post_uri);

    // OPTIONS for CORS preflight
    httpd_uri_t options_uri = {
        .uri = "/webrtc/signal",
        .method = HTTP_OPTIONS,
        .handler = signal_post_handler,  // same handler handles OPTIONS
    };
    httpd_register_uri_handler(g_httpd, &options_uri);

    // Camera control: POST /camera/control
    httpd_uri_t cam_ctrl_uri = {
        .uri = "/camera/control",
        .method = HTTP_POST,
        .handler = camera_control_post_handler,
    };
    httpd_register_uri_handler(g_httpd, &cam_ctrl_uri);

    // Camera control: OPTIONS /camera/control (CORS preflight)
    httpd_uri_t cam_ctrl_options_uri = {
        .uri = "/camera/control",
        .method = HTTP_OPTIONS,
        .handler = camera_control_post_handler,
    };
    httpd_register_uri_handler(g_httpd, &cam_ctrl_options_uri);

    // Camera status: GET /camera/status
    httpd_uri_t cam_status_uri = {
        .uri = "/camera/status",
        .method = HTTP_GET,
        .handler = camera_status_get_handler,
    };
    httpd_register_uri_handler(g_httpd, &cam_status_uri);

    // Encoder control: POST /encoder/control
    httpd_uri_t enc_ctrl_uri = {
        .uri = "/encoder/control",
        .method = HTTP_POST,
        .handler = encoder_control_post_handler,
    };
    httpd_register_uri_handler(g_httpd, &enc_ctrl_uri);

    // Encoder control: OPTIONS /encoder/control (CORS prefight)
    httpd_uri_t enc_ctrl_options_uri = {
        .uri = "/encoder/control",
        .method = HTTP_OPTIONS,
        .handler = encoder_control_post_handler,
    };
    httpd_register_uri_handler(g_httpd, &enc_ctrl_options_uri);

    // Encoder status: GET /encoder/status
    httpd_uri_t enc_status_uri = {
        .uri = "/encoder/status",
        .method = HTTP_GET,
        .handler = encoder_status_get_handler,
    };
    httpd_register_uri_handler(g_httpd, &enc_status_uri);

    // ?? STA Proxy endpoints (device A ? device B relay) ??
    // POST /proxy/sta/signal
    httpd_uri_t proxy_post_uri = {
        .uri = "/proxy/sta/signal",
        .method = HTTP_POST,
        .handler = proxy_sta_post_handler,
    };
    httpd_register_uri_handler(g_httpd, &proxy_post_uri);

    // OPTIONS /proxy/sta/signal (CORS)
    httpd_uri_t proxy_post_opt_uri = {
        .uri = "/proxy/sta/signal",
        .method = HTTP_OPTIONS,
        .handler = proxy_sta_post_handler,
    };
    httpd_register_uri_handler(g_httpd, &proxy_post_opt_uri);

    // GET /proxy/sta/signal (SSE relay)
    httpd_uri_t proxy_sse_uri = {
        .uri = "/proxy/sta/signal",
        .method = HTTP_GET,
        .handler = proxy_sta_sse_get_handler,
    };
    httpd_register_uri_handler(g_httpd, &proxy_sse_uri);

    // ?? CAM2 endpoints (camera B via UDP relay) ??
    // GET /cam2/signal (SSE)
    httpd_uri_t cam2_sse_uri = {
        .uri = "/cam2/signal",
        .method = HTTP_GET,
        .handler = cam2_sse_get_handler,
    };
    httpd_register_uri_handler(g_httpd, &cam2_sse_uri);

    // POST /cam2/signal
    httpd_uri_t cam2_post_uri = {
        .uri = "/cam2/signal",
        .method = HTTP_POST,
        .handler = cam2_signal_post_handler,
    };
    httpd_register_uri_handler(g_httpd, &cam2_post_uri);

    // OPTIONS /cam2/signal (CORS)
    httpd_uri_t cam2_opt_uri = {
        .uri = "/cam2/signal",
        .method = HTTP_OPTIONS,
        .handler = cam2_signal_post_handler,
    };
    httpd_register_uri_handler(g_httpd, &cam2_opt_uri);

    ESP_LOGI(TAG, "WebRTC HTTP signaling + camera controls ready");
    return ESP_OK;
}

/* ================================================================
 *  Public Init Function  (camera_fd for V4L2 exposure/gain control)
 * ================================================================ */

esp_err_t webrtc_video_init(int camera_fd)
{
    ESP_LOGI(TAG, "=== WebRTC Video Init ===");

    // Store camera fd for exposure/gain V4L2 controls
    g_camera_fd = camera_fd;

    // Suppress noisy internal library debug logs that kill video performance
    esp_log_level_set("PEER_DEF", ESP_LOG_WARN);
    esp_log_level_set("AGENT", ESP_LOG_NONE);
    esp_log_level_set("UDP", ESP_LOG_NONE);

    // Create SSE message queue
    g_sse_queue = xQueueCreate(WEBRTC_SSE_QUEUE_LEN, sizeof(char *));
    if (!g_sse_queue) {
        ESP_LOGE(TAG, "SSE queue create failed");
        return ESP_FAIL;
    }

    // Mutex to serialize esp_peer_open (shared impl not thread-safe)
    g_peer_open_mutex = xSemaphoreCreateMutex();
    if (!g_peer_open_mutex) {
        ESP_LOGE(TAG, "Peer open mutex create failed");
        return ESP_FAIL;
    }

    // Create CAM2 SSE message queue (for browser ? CAM2 peer signaling)
    g_sse_queue2 = xQueueCreate(WEBRTC_SSE_QUEUE_LEN, sizeof(char *));
    if (!g_sse_queue2) {
        ESP_LOGE(TAG, "SSE2 queue create failed");
        return ESP_FAIL;
    }

    // Initialize WebRTC peer FIRST (needs fresh PSRAM before encoder's internal alloc)
    if (webrtc_peer_init() != ESP_OK) {
        ESP_LOGE(TAG, "Peer init failed");
        return ESP_FAIL;
    }

    // Initialize H.264 encoder (internal buffers for 1080p may fragment PSRAM)
    if (h264_encoder_init() != ESP_OK) {
        ESP_LOGE(TAG, "H.264 init failed");
        return ESP_FAIL;
    }

    // Create frame queue & encoder task (decoupled from camera callback)
    g_frame_queue = xQueueCreate(FRAME_QUEUE_LEN, sizeof(frame_item_t));
    if (!g_frame_queue) {
        ESP_LOGE(TAG, "Frame queue create failed");
        return ESP_FAIL;
    }
    // Encoder on CPU1 (isolated from WiFi/peer), priority 15 (highest, no preemption)
    if (xTaskCreatePinnedToCore(encoder_task, "webrtc_enc", 12288, NULL, 15, &g_encoder_task, 1) != pdPASS) {
        ESP_LOGE(TAG, "Encoder task create failed");
        return ESP_FAIL;
    }

    // Start HTTP signaling server
    if (webrtc_http_init() != ESP_OK) {
        ESP_LOGE(TAG, "HTTP init failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "=== WebRTC HTTP signaling ready (check main for IP) ===");

    // CAM2: UDP relay listener receives H.264 from STA, forwards via CAM2 peer
    if (g_is_ap_mode) {
        xTaskCreatePinnedToCore(udp_relay_task, "udp_relay", 8192, NULL, 12, NULL, 0);
        ESP_LOGI(TAG, "UDP relay listening on :%d (CAM2 active)", UDP_PUSH_PORT);
    }

    return ESP_OK;
}

/* ================================================================
 *  Start Peer Connection (call after SSE client connects)
 * ================================================================ */

void webrtc_start_connection(void)
{
    if (g_peer) {
        ESP_LOGI(TAG, "Starting new WebRTC connection...");
        esp_peer_new_connection(g_peer);
    }
}

/* ================================================================
 *  Second Peer (CAM2): relays STA device's H.264 via UDP to browser
 * ================================================================ */

// ?? CAM2 SSE helpers ??
static int sse2_send(const char *data)
{
    if (!g_sse2_req || !g_sse2_connected) return -1;
    int len = strlen(data) + strlen("data: ") + strlen("\n\n") + 1;
    char *buf = (char *)malloc(len);
    if (!buf) return -1;
    len = snprintf(buf, len, "data: %s\n\n", data);
    int ret = httpd_resp_send_chunk(g_sse2_req, buf, len);
    free(buf);
    return ret;
}

static void sse2_push_json(const char *json)
{
    if (g_sse2_connected && g_sse_queue2) {
        char *cpy = strdup(json);
        if (cpy) xQueueSend(g_sse_queue2, &cpy, 0);
    }
}

static void sse2_push_sdp(const char *sdp) {
    cJSON *r = cJSON_CreateObject();
    cJSON_AddStringToObject(r, "type", "offer");
    cJSON_AddStringToObject(r, "sdp", sdp);
    char *j = cJSON_PrintUnformatted(r);
    if (j) { sse2_push_json(j); free(j); }
    cJSON_Delete(r);
}

static void sse2_push_candidate(const char *c) {
    cJSON *r = cJSON_CreateObject();
    cJSON_AddStringToObject(r, "type", "candidate");
    cJSON_AddStringToObject(r, "candidate", c);
    char *j = cJSON_PrintUnformatted(r);
    if (j) { sse2_push_json(j); free(j); }
    cJSON_Delete(r);
}

static void sse2_send_task(void *arg)
{
    int64_t last_hb = esp_timer_get_time() / 1000;
    while (!g_sse2_stopping) {
        char *msg = NULL;
        if (xQueueReceive(g_sse_queue2, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (msg) { sse2_send(msg); free(msg); }
        }
        int64_t now = esp_timer_get_time() / 1000;
        if (now - last_hb > WEBRTC_SSE_HEARTBEAT_MS) {
            last_hb = now;
            sse2_send("{\"type\":\"heartbeat\"}");
        }
    }
    httpd_req_async_handler_complete(g_sse2_req);
    g_sse2_req = NULL;
    g_sse2_connected = false;
    g_sse2_stopping = false;
    vTaskDelete(NULL);
}

// ?? CAM2 callbacks ??
static int peer2_on_state(esp_peer_state_t state, void *ctx)
{
    ESP_LOGI(TAG, "CAM2 Peer state: %d", (int)state);
    if (state == ESP_PEER_STATE_CONNECTED) {
        g_peer2_connected = true;
        g_need_idr2 = true;
        ESP_LOGI(TAG, "=== CAM2 CONNECTED ===");
    } else if (state == ESP_PEER_STATE_DISCONNECTED || state == ESP_PEER_STATE_CONNECT_FAILED) {
        g_peer2_connected = false;
    }
    return 0;
}

static int peer2_on_msg(esp_peer_msg_t *msg, void *ctx)
{
    if (!msg || !g_sse2_connected) return 0;
    if (msg->type == ESP_PEER_MSG_TYPE_SDP) {
        char *s = strndup((char *)msg->data, msg->size);
        if (s) { sse2_push_sdp(s); free(s); }
    } else if (msg->type == ESP_PEER_MSG_TYPE_CANDIDATE) {
        sse2_push_candidate((char *)msg->data);
    }
    return 0;
}

static int peer2_on_video_info(esp_peer_video_stream_info_t *info, void *ctx) { return 0; }

// ?? CAM2 SSE handler: GET /cam2/signal ??
static esp_err_t cam2_sse_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/event-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    if (g_sse2_connected) {
        sse2_send("{\"error\":\"CAM2 busy\"}");
        httpd_resp_send_chunk(req, NULL, 0);
        return ESP_OK;
    }

    if (g_peer2 && g_peer2_connected) {
        sse2_send("{\"type\":\"status\",\"state\":\"connected\"}");
    } else {
        // Lazy init CAM2 peer on first browser connection
        if (!g_peer2 && cam2_peer_init() != ESP_OK) {
            sse2_send("{\"error\":\"cam2_init_fail\"}");
            httpd_resp_send_chunk(req, NULL, 0);
            return ESP_OK;
        }
        sse2_send("{\"type\":\"connected\"}");
    }
    g_sse2_connected = true;
    g_sse2_stopping = false;
    httpd_req_async_handler_begin(req, &g_sse2_req);
    if (g_sse2_req) xTaskCreate(sse2_send_task, "sse2_send", 4096, NULL, 5, NULL);

    if (g_peer2 && !g_peer2_connected) {
        esp_peer_new_connection(g_peer2);
        ESP_LOGI(TAG, "CAM2: new connection initiated");
    }
    return ESP_OK;
}

// ?? CAM2 POST handler: POST /cam2/signal ??
static esp_err_t cam2_signal_post_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    if (req->method == HTTP_OPTIONS) { httpd_resp_send(req, NULL, 0); return ESP_OK; }

    if (req->content_len > 16 * 1024) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Too long"); return ESP_OK; }
    char *buf = (char *)malloc(req->content_len + 1);
    if (!buf) return ESP_FAIL;
    int total = 0;
    while (total < req->content_len) {
        int r = httpd_req_recv(req, buf + total, req->content_len - total);
        if (r <= 0) { if (r == HTTPD_SOCK_ERR_TIMEOUT) continue; free(buf); return ESP_FAIL; }
        total += r;
    }
    buf[total] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) { free(buf); return ESP_OK; }
    cJSON *type = cJSON_GetObjectItem(root, "type");
    if (!type || !type->valuestring) { cJSON_Delete(root); free(buf); return ESP_OK; }

    if (strcmp(type->valuestring, "answer") == 0) {
        cJSON *sdp_j = cJSON_GetObjectItem(root, "sdp");
        if (sdp_j && sdp_j->valuestring) {
            esp_peer_msg_t m = { .type = ESP_PEER_MSG_TYPE_SDP, .data = (uint8_t *)sdp_j->valuestring, .size = (int)strlen(sdp_j->valuestring) };
            if (g_peer2) esp_peer_send_msg(g_peer2, &m);
        }
    } else if (strcmp(type->valuestring, "candidate") == 0) {
        cJSON *cand_j = cJSON_GetObjectItem(root, "candidate");
        if (cand_j && cand_j->valuestring) {
            esp_peer_msg_t m = { .type = ESP_PEER_MSG_TYPE_CANDIDATE, .data = (uint8_t *)cand_j->valuestring, .size = (int)strlen(cand_j->valuestring) };
            if (g_peer2) esp_peer_send_msg(g_peer2, &m);
        }
    }
    httpd_resp_sendstr(req, "OK");
    cJSON_Delete(root); free(buf);
    return ESP_OK;
}

// ?? CAM2 peer task ??
static void peer2_task(void *arg) {
    while (1) { if (g_peer2) esp_peer_main_loop(g_peer2); vTaskDelay(pdMS_TO_TICKS(10)); }
}

// ?? CAM2 peer init ??
static esp_err_t cam2_peer_init(void)
{
    esp_peer_default_cfg_t extra = {};
    extra.agent_recv_timeout = 1000;
    extra.rtp_cfg.send_pool_size = 64 * 1024;
    extra.rtp_cfg.send_queue_num = 32;
    extra.alive_binding_retries = 10;
    extra.ice_use_lite_mode = true;

    esp_peer_cfg_t cfg = {
        .role = ESP_PEER_ROLE_CONTROLLING,
        .audio_info = { .codec = ESP_PEER_AUDIO_CODEC_G711A, .sample_rate = 8000, .channel = 1 },
        .video_info = { .codec = ESP_PEER_VIDEO_CODEC_H264, .width = WEBRTC_VIDEO_WIDTH, .height = WEBRTC_VIDEO_HEIGHT, .fps = WEBRTC_VIDEO_FPS },
        .audio_dir = ESP_PEER_MEDIA_DIR_SEND_ONLY,
        .video_dir = ESP_PEER_MEDIA_DIR_SEND_ONLY,
        .no_auto_reconnect = true,
        .enable_data_channel = false,
        .extra_cfg = &extra, .extra_size = sizeof(extra), .ctx = NULL,
        .on_state = peer2_on_state, .on_msg = peer2_on_msg, .on_video_info = peer2_on_video_info,
    };
    xSemaphoreTake(g_peer_open_mutex, portMAX_DELAY);
    int ret = esp_peer_open(&cfg, esp_peer_get_default_impl(), &g_peer2);
    xSemaphoreGive(g_peer_open_mutex);
    if (ret != ESP_PEER_ERR_NONE) { ESP_LOGE(TAG, "CAM2 peer open: %d", ret); return ESP_FAIL; }
    if (xTaskCreatePinnedToCore(peer2_task, "cam2_peer", 8192, NULL, 12, &g_peer2_task, 0) != pdPASS) return ESP_FAIL;
    ESP_LOGI(TAG, "CAM2 peer initialized");
    return ESP_OK;
}

// ?? UDP relay task: receive H.264 from STA, send via CAM2 peer ??
static void udp_relay_task(void *arg)
{
    static uint32_t cam2_fc = 0;
    static int64_t  cam2_last = 0;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { ESP_LOGE(TAG, "UDP relay socket fail"); vTaskDelete(NULL); return; }

    struct sockaddr_in bind_addr = {};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(UDP_PUSH_PORT);
    bind_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "UDP relay bind fail"); close(sock); vTaskDelete(NULL); return;
    }
    ESP_LOGI(TAG, "UDP relay listening on :%d", UDP_PUSH_PORT);

    uint8_t *buf = (uint8_t *)heap_caps_aligned_alloc(128, 256 * 1024,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) { close(sock); vTaskDelete(NULL); return; }

    while (1) {
        struct sockaddr_in from;
        socklen_t fromlen = sizeof(from);
        int len = recvfrom(sock, buf, 256 * 1024, 0,
            (struct sockaddr *)&from, &fromlen);
        if (len <= 4) continue;  // need at least 4-byte header

        // Header: first 4 bytes = frame length
        uint32_t frame_len;
        memcpy(&frame_len, buf, 4);
        if (frame_len == 0 || frame_len > 256 * 1024 || 4 + frame_len > (uint32_t)len) continue;

        // Wait for IDR on new connection
        uint8_t *nal = buf + 4;
        if (g_need_idr2) {
            // Check for IDR: NAL type 5 (0x65 after start code)
            bool is_idr = false;
            for (uint32_t i = 0; i + 4 < frame_len; i++) {
                if (nal[i] == 0 && nal[i+1] == 0 && nal[i+2] == 0 && nal[i+3] == 1) {
                    uint8_t ntype = nal[i+4] & 0x1F;
                    if (ntype == 5) { is_idr = true; break; }
                }
            }
            if (!is_idr) continue;
            g_need_idr2 = false;
        }

        if (g_peer2_connected && frame_len > 0) {
            esp_peer_video_frame_t vf = { .pts = (uint32_t)(esp_timer_get_time() / 1000), .data = nal, .size = (int)frame_len };
            esp_peer_send_video(g_peer2, &vf);
        }

        cam2_fc++;
        int64_t now = esp_timer_get_time();
        if (cam2_fc % 100 == 0 && cam2_last > 0) {
            float fps = 100.f * 1000000.f / (float)(now - cam2_last);
            ESP_LOGI(TAG, "CAM2 relay: %.1f fps, frame=%" PRIu32 " size=%" PRIu32, fps, cam2_fc, frame_len);
        }
        if (cam2_fc % 100 == 0) cam2_last = now;
    }
    free(buf); close(sock);
    vTaskDelete(NULL);
}
