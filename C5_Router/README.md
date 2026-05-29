# C5_Router — ESP32-C5 独立 SoftAP 路由器

## 概述

为 WebRTC 图传系统提供 5GHz SoftAP，使多台 P4 设备可通过 L2 桥接直接通信。

```
C5 SoftAP (192.168.4.1)
  ├── P4 #1 (192.168.4.2 固定)
  ├── P4 #2 (192.168.4.3 固定)
  └── 手机等设备 (DHCP 自动获取)
```

## 硬件

- 芯片：ESP32-C5
- 频段：5GHz（信道 149）
- 支持最多 4 个 STA 连接

## 构建

```bash
cd C5_Router
idf.py set-target esp32c5
idf.py build
idf.py -p PORT flash monitor
```

## 配置

| 参数 | 值 |
|------|-----|
| SSID | ROBOT_CAM |
| 密码 | 无（开放） |
| 信道 | 149 (5GHz) |
| AP IP | 192.168.4.1 |
| DHCP | 自动（192.168.4.4+） |
| 国家码 | CN |

P4 端需配置静态 IP：
- P4 #1：`192.168.4.2/24`
- P4 #2：`192.168.4.3/24`

## 验证

1. 串口日志显示 `SoftAP started`
2. 手机/电脑可搜到 `ROBOT_CAM` 无密码 WiFi
3. 连接后自动获取 192.168.4.x 地址
4. P4 之间可直接 ping 通
5. 浏览器访问 `http://192.168.4.2` 或 `http://192.168.4.3`
