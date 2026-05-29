# RainWebRTC Robocon — 双路 H.264 WebRTC 图传

两台 ESP32-P4 + OV5647 独立推流，通过独立 C5 SoftAP 路由器实现同页面双画面。

## 架构

```
C5 (SoftAP 路由器, 192.168.4.1, 信道 149)
├── P4 #1 (STA, 192.168.4.10) — 相机A
├── P4 #2 (STA, 192.168.4.11) — 相机B
└── 手机/电脑 (DHCP, 192.168.4.2+)
```

- **P4 之间不通信**，各跑各的 WebRTC -> 无 SDIO 双 peer 压力
- **手机直连两个 P4**，页面同时渲染两路 H.264

## 性能（每路）

| 项 | 值 |
|------|------|
| 摄像头 | OV5647 MIPI-CSI, 1280×960 → ISP 裁切 **1280×720 16:9** |
| 编码器 | 硬件 H.264, 720p  **~45fps**, 6Mbps (网页可调 2-15Mbps) |
| 推流 | WebRTC via esp_peer, ICE-Lite + DTLS-SRTP |
| WiFi | 5.8GHz SoftAP Ch149 (CN 国家码) |
| 工作时长 | **已连续运行 20+ 分钟稳定** |

## 网页控制

手机/PC 连 C5 的 `ROBOT_CAM` WiFi，打开 `http://192.168.4.10`（或 `.11`）

- **两个视频框**：相机A（自身）+ 相机B（对端）
- **各自独立控制**：曝光、码率、GOP、强制 IDR
- 双路 `readyState` 追踪，卡住自动重刷

## 构建与烧录

### P4 固件（两个设备）

```bash
# 修改 main.cpp 中设备角色和 IP
#define DEVICE_ROLE_STA
// P4 #1: IP4_ADDR(&ip_info.ip, 192, 168, 4, 10)
// P4 #2: IP4_ADDR(&ip_info.ip, 192, 168, 4, 11)

idf.py build && idf.py -p COM8 flash    # P4 #1
idf.py build && idf.py -p COM12 flash   # P4 #2
```

### C5 路由器固件（独立开发板）

参见同仓库 `C5_Router/` 目录：

```bash
cd C5_Router
idf.py set-target esp32c5
idf.py build && idf.py -p COM?? flash
```

## 硬件清单

| 器件 | 数量 | 说明 |
|------|------|------|
| ESP32-P4 + C5 核心板 | ×2 | 主控 + 协处理器 (SDIO) |
| OV5647 MIPI-CSI 摄像头 | ×2 | 5MP |
| ESP32-C5 开发板 | ×1 | 独立路由器 |
| 5.8GHz IPEX 天线 | ×3 | 每块板一根 |

## 开发历史

- ✅ 单摄像头 AP 模式稳定
- ❌ 单 P4 双 peer （CAM2）→ DRAM 不足/DMA 冲突
- ❌ P4 AP + P4 STA 直连 → C5 STA-UDP 不通
- ✅ 独立 C5 路由器 + 双 P4 STA → 稳定双路

