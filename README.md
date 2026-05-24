# RainWebRTC Robocon

ESP32-P4 + OV5647 → H.264 WebRTC 图传，通过 ESP32-C5 协处理器 SDIO 提供 5.8GHz WiFi。

## 性能

- **摄像头**: OV5647 MIPI-CSI, 1280×960 全幅 binning → ISP 裁切 **1280×720 16:9**
- **编码器**: 硬件 H.264, 720p@30fps, 默认 6Mbps (可在网页 2-15Mbps 运行时调节)
- **推流**: WebRTC via esp_peer, **30-45fps**, P 帧 ~7KB
- **WiFi**: 5.8GHz SoftAP Ch149 (CN 国家码解锁), 5GHz 单频

## 网页控制

连接 `ROBOT_CAM` WiFi 后打开 `http://192.168.4.1`

| 控制 | 说明 |
|------|------|
| 📷 曝光 | 1-150 (×100µs) |
| 🎬 码率 | 2-15 Mbps 运行时调节 |
| 🎬 GOP | 5-60 帧 |
| ⚡ 强制 IDR | 立即出关键帧 |

## 构建

```bash
idf.py build
idf.py flash monitor
```

## 硬件

- **主控**: ESP32-P4 (360MHz 双核 RISC-V, 32MB PSRAM)
- **协处理器**: ESP32-C5 (SDIO 4-bit 40MHz)
- **摄像头**: OV5647 (5MP MIPI-CSI)
- **天线**: 5.8GHz IPEX 棒棒糖

## 仓库

原 BSP: [WTDKP4C5-S1](https://github.com/wireless-tag-com/WTDKP4C5-S1)
