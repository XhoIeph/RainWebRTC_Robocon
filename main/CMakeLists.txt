idf_component_register(
    SRCS main.cpp webrtc_video.cpp
    EMBED_TXTFILES webrtc_test.html
    INCLUDE_DIRS .
    REQUIRES apps esp_http_server esp_wifi esp_netif esp_h264 esp_peer json)
