# ESP32-C6 ZigBee Coordinator for Home Assistant

基于 ESP-IDF 框架的 ESP32-C6 ZigBee 协调器项目，通过 MQTT 与 Home Assistant OS (HAOS) 集成。

## 目录

- [项目概述](#项目概述)
- [系统架构](#系统架构)
- [硬件要求](#硬件要求)
- [软件依赖](#软件依赖)
- [项目配置](#项目配置)
- [代码架构](#代码架构)
- [功能说明](#功能说明)
- [编译与烧录](#编译与烧录)
- [HAOS 配置](#haos-配置)
- [实现过程](#实现过程)
- [后续开发](#后续开发)

## 项目概述

本项目实现了一个基于 ESP32-C6 的 ZigBee 协调器，具有以下特性：

- 基于 ZBOSS 协议栈（ESP-IDF 原生支持）
- WiFi 连接 + MQTT 通信
- Home Assistant MQTT Discovery 自动发现
- 断电离线检测（LWT 遗嘱消息）
- 支持最多 10 个 ZigBee 子设备入网

## 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        树莓派宿主机 (192.168.1.10)                │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │                  Mosquitto MQTT Broker                       │ │
│  │                      (端口 1883)                              │ │
│  │                    allow_anonymous: true                     │ │
│  └─────────────────────────────────────────────────────────────┘ │
│                              ▲                                   │
│                              │ MQTT                             │
│         ┌────────────────────┼────────────────────┐             │
│         │                    │                    │             │
│         ▼                    ▼                    ▼             │
│  ┌─────────────┐    ┌─────────────────┐   ┌──────────────────┐  │
│  │ ESP32 温湿度计 │    │ ESP32-C6 ZigBee │   │  HAOS 虚拟机     │  │
│  │  (传感器)     │    │    协调器        │   │ (192.168.1.201)  │  │
│  │              │    │                 │   │                  │  │
│  │ 温度/湿度/运动 │    │ ZigBee Channel 13│   │  Home Assistant │  │
│  │              │    │ PAN ID: 0xf6f5  │   │  MQTT 集成       │  │
│  └─────────────┘    └─────────────────┘   └──────────────────┘  │
│                                                   ▲              │
│                                                   │              │
│                                          连接到宿主机 Mosquitto   │
└─────────────────────────────────────────────────────────────────┘
```

### 网络拓扑

| 设备 | IP 地址 | 说明 |
|------|---------|------|
| 树莓派宿主机 | 192.168.1.10 | 运行 Mosquitto MQTT Broker |
| HAOS 虚拟机 | 192.168.1.201 | Home Assistant OS |
| ESP32 温湿度计 | 动态分配 | 温度/湿度/运动传感器 |
| ESP32-C6 协调器 | 动态分配 | ZigBee Coordinator |

### ZigBee 网络参数

| 参数 | 值 | 说明 |
|------|-----|------|
| Channel | 13 | ZigBee 通道 |
| PAN ID | 0xf6f5 | 网络标识符 |
| 最大子设备 | 10 | 最大入网设备数 |
| 入网窗口 | 180秒 | 启动时允许入网时间 |

## 硬件要求

- ESP32-C6 开发板
- USB 数据线
- （可选）ZigBee 设备用于测试

## 软件依赖

- ESP-IDF v5.x
- Python 3.8+
- ZBOSS 协议栈（ESP-IDF 组件）

## 项目配置

### WiFi 配置

编辑 `main/esp_zigbee_gateway.h`:

```c
#define WIFI_SSID                       "your_ssid"
#define WIFI_PASSWORD                   "your_password"
#define WIFI_MAXIMUM_RETRY              5
```

### MQTT 配置

```c
#define MQTT_BROKER_IP                  "192.168.1.10"  // 宿主机 IP
#define MQTT_BROKER_PORT                1883
#define MQTT_USERNAME                   NULL            // 匿名连接
#define MQTT_PASSWORD                   NULL
#define MQTT_TOPIC_PREFIX               "zigbee2mqtt"
```

### ZigBee 配置

```c
#define MAX_CHILDREN                    10
#define ESP_ZB_PRIMARY_CHANNEL_MASK     (1l << 13)      // 通道 13
#define ESP_ZB_GATEWAY_ENDPOINT         1
```

## 代码架构

```
esp32c6_zigbee/
├── CMakeLists.txt              # 项目构建配置
├── sdkconfig.defaults          # SDK 默认配置
├── partitions.csv              # 分区表
├── main/
│   ├── CMakeLists.txt          # 组件构建配置
│   ├── esp_zigbee_gateway.h    # 配置头文件
│   └── esp_zigbee_gateway.c    # 主程序实现
└── managed_components/         # ESP-IDF 管理的组件
```

### 主程序模块

#### 1. WiFi 模块

```c
// WiFi 事件处理
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);

// WiFi 初始化
static esp_err_t wifi_init_sta(void);
```

#### 2. MQTT 模块

```c
// MQTT 事件处理
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                int32_t event_id, void *event_data);

// MQTT 客户端启动
static void mqtt_app_start(void);

// 发布函数
static void mqtt_publish_ha_discovery(void);       // HA Discovery
static void mqtt_publish_coordinator_status(void); // 协调器状态
static void mqtt_publish_device_announce(...);     // 设备入网公告
static void mqtt_publish_bridge_state(...);        // 桥接状态
```

#### 3. ZigBee 模块

```c
// ZigBee 任务
static void esp_zb_task(void *pvParameters);

// 信号处理
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);
```

### 程序流程

```
app_main()
    │
    ├─→ esp_zb_platform_config()     // 配置平台
    ├─→ nvs_flash_init()             // 初始化 NVS
    ├─→ esp_netif_init()             // 初始化网络接口
    ├─→ esp_event_loop_create_default()
    └─→ xTaskCreate(esp_zb_task)     // 创建 ZigBee 任务
            │
            ├─→ esp_zb_init()        // 初始化 ZigBee 栈
            ├─→ esp_zb_device_register()
            └─→ esp_zb_start(false)  // 启动 ZigBee
                    │
                    └─→ esp_zb_app_signal_handler()
                            │
                            ├─→ SKIP_STARTUP:
                            │       ├─→ wifi_init_sta()     // 连接 WiFi
                            │       └─→ mqtt_app_start()     // 启动 MQTT
                            │
                            ├─→ DEVICE_REBOOT:
                            │       └─→ esp_zb_bdb_open_network(180)  // 开放入网
                            │
                            ├─→ PERMIT_JOIN_STATUS:
                            │       ├─→ mqtt_publish_ha_discovery()
                            │       └─→ mqtt_publish_coordinator_status()
                            │
                            └─→ DEVICE_ANNCE:
                                    └─→ mqtt_publish_device_announce()
```

## 功能说明

### 已实现功能

| 功能 | 状态 | 说明 |
|------|------|------|
| WiFi 连接 | ✅ | STA 模式，无限自动重连 |
| MQTT 连接 | ✅ | 匿名连接宿主机 Mosquitto，自动重连 |
| ZigBee 协调器 | ✅ | 通道 13，PAN ID 0xf6f5 |
| HA Discovery | ✅ | 自动在 HAOS 中发现设备 |
| 状态监控 | ✅ | 显示 online/offline 状态 |
| LWT 遗嘱消息 | ✅ | 断电后约 15 秒显示 offline |
| 设备入网公告 | ✅ | 新设备入网时发布 MQTT 消息 |
| 心跳机制 | ✅ | 每 60 秒发布状态更新 |
| 诊断信息 | ✅ | 重启原因、运行时间、剩余内存 |

### MQTT 话题

#### 发布话题

| 话题 | 方向 | 说明 |
|------|------|------|
| `homeassistant/sensor/zigbee_coordinator_{IEEE}/config` | 发布 | HA Discovery 配置 |
| `homeassistant/sensor/zigbee_coordinator_{IEEE}/state` | 发布 | 协调器状态 (online/offline) |
| `homeassistant/sensor/zigbee_coordinator_{IEEE}/attributes` | 发布 | 协调器属性 |
| `zigbee2mqtt/bridge/state` | 发布 | 桥接状态 |
| `zigbee2mqtt/bridge/info` | 发布 | 桥接信息 |
| `zigbee2mqtt/{short_addr}` | 发布 | 设备入网公告 |

#### 订阅话题

| 话题 | 方向 | 说明 |
|------|------|------|
| `zigbee2mqtt/bridge/#` | 订阅 | 接收控制命令（待实现） |

### HA Discovery 消息格式

```json
{
  "name": "ZigBee Coordinator",
  "unique_id": "zigbee_coordinator_{IEEE}",
  "state_topic": "homeassistant/sensor/zigbee_coordinator_{IEEE}/state",
  "json_attributes_topic": "homeassistant/sensor/zigbee_coordinator_{IEEE}/attributes",
  "device": {
    "identifiers": ["zigbee_coordinator_{IEEE}"],
    "name": "ZigBee Coordinator",
    "manufacturer": "Espressif",
    "model": "ESP32-C6"
  },
  "icon": "mdi:zigbee"
}
```

### 协调器属性

通过 MQTT 发布的属性信息：

| 属性 | 说明 | 示例值 |
|------|------|--------|
| `pan_id` | ZigBee 网络 PAN ID | "0xf6f5" |
| `channel` | ZigBee 通道 | 13 |
| `short_addr` | 短地址 | "0x0000" |
| `ieee` | IEEE 长地址 | "a1b2c3d4e5f6a7b8" |
| `reset_reason` | 重启原因 | "power_on", "wdt", "panic" |
| `uptime` | 运行时间 | "02:30:45" |
| `free_heap` | 剩余内存（字节）| 245760 |

### 重启原因说明

| 值 | 含义 |
|------|------|
| power_on | 正常上电启动 |
| software | 软件调用重启 |
| panic | 程序崩溃/异常 |
| wdt | 看门狗超时 |
| brownout | 电压过低重启 |
| int_wdt | 中断看门狗超时 |
| task_wdt | 任务看门狗超时 |
| deepsleep | 深度睡眠唤醒 |
| other | 其他原因 |

## 编译与烧录

### 环境准备

```bash
# 安装 ESP-IDF（如果未安装）
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
. ./export.sh
```

### 编译项目

```bash
cd d:/ESP32/esp32c6_zigbee
idf.py build
```

### 烧录

```bash
# 查找可用端口
idf.py -p COM? flash monitor

# 或指定端口
idf.py -p COM3 flash monitor
```

### 串口输出示例

```
I (2226) ESP_ZB_GATEWAY: Got IP:192.168.1.122
I (2226) ESP_ZB_GATEWAY: Connected to AP SSID:jiajia
I (2236) ESP_ZB_GATEWAY: MQTT client started, connecting to 192.168.1.10:1883
I (2246) ESP_ZB_GATEWAY: Initialize Zigbee stack
I (2246) ESP_ZB_GATEWAY: Device started up in non factory-reset mode
I (2256) ESP_ZB_GATEWAY: Device rebooted
I (2456) ESP_ZB_GATEWAY: MQTT Connected to broker
I (2956) ESP_ZB_GATEWAY: Network(0xf6f5) is open for 180 seconds
I (2956) ESP_ZB_GATEWAY: Published HA Discovery for coordinator
```

## HAOS 配置

### MQTT 集成

在 HAOS 中配置 MQTT 集成连接到宿主机 Mosquitto：

1. **设置** → **设备与服务** → **添加集成** → **MQTT**
2. 输入连接信息：
   - Broker: `192.168.1.10`
   - 端口: `1883`
   - 用户名: （留空，匿名连接）
   - 密码: （留空）

### 验证设备

在 HAOS 中查看协调器：

1. **设置** → **设备与服务** → **MQTT**
2. 点击进入，查看 **设备** 标签
3. 应该看到 "ZigBee Coordinator" 设备

### 宿主机 Mosquitto 配置

Mosquitto 配置文件 (`/etc/mosquitto/conf.d/local.conf`):

```
allow_anonymous true
listener 1883
persistence true
```

查看连接日志：
```bash
sudo tail -f /var/log/mosquitto/mosquitto.log
```

订阅协调器话题：
```bash
mosquitto_sub -h localhost -t 'homeassistant/sensor/zigbee_coordinator/#' -v
```

## 实现过程

### 阶段一：基础框架搭建

1. 基于 ESP-IDF ZigBee Gateway 示例创建项目
2. 配置 ESP32-C6 原生 ZigBee（不使用外部 RCP）
3. 移除 protocol_examples_common 依赖，硬编码 WiFi 配置

### 阶段二：MQTT 集成

1. 添加 WiFi STA 模式连接
2. 添加 MQTT 客户端组件
3. 实现连接宿主机 Mosquitto
4. 调试连接问题（匿名连接 vs 认证连接）

### 阶段三：HAOS 集成

1. 研究 Home Assistant MQTT Discovery 协议
2. 实现协调器的 Discovery 消息发布
3. 实现状态和属性发布
4. 实现 LWT 遗嘱消息（断电检测）

### 阶段四：问题修复

| 问题 | 解决方案 |
|------|---------|
| MQTT 连接成功但看不到客户端 | 确认连接的是正确的 broker 地址 |
| EMQX 和 Mosquitto 地址混淆 | 统一使用宿主机 Mosquitto (192.168.1.10) |
| Discovery 不触发 | 在网络就绪时发布 Discovery |
| 断电不显示 offline | 添加 LWT 遗嘱消息，设置 keepalive=10s |
| retained 消息残留 | MQTT 连接成功时立即发布 online 状态 |
| 长时间运行后离线 | 添加 WiFi/MQTT 无限自动重连 |
| 状态不更新 | 添加 60 秒心跳定时器 |
| 无法诊断问题原因 | 添加重启原因、运行时间、剩余内存属性 |

## 后续开发

### 待实现功能

| 功能 | 优先级 | 说明 |
|------|--------|------|
| ZigBee 设备属性解析 | 高 | 解析温度、湿度、开关等属性 |
| 设备 Discovery | 高 | 为入网设备发布 HA Discovery |
| 设备状态发布 | 高 | 发布设备传感器数据 |
| MQTT 命令控制 | 中 | 通过 MQTT 控制设备（开关等）|
| 入网窗口远程控制 | 中 | 通过 MQTT 命令开启入网窗口 |
| OTA 升级 | 低 | 支持 OTA 固件升级 |

### ZigBee 设备开发路线

1. **设备入网检测** - 已实现设备公告发布
2. **属性发现** - 读取设备的 cluster 和 attribute
3. **数据转换** - 将 ZigBee 数据转为 MQTT 消息
4. **HA Discovery** - 为每个设备生成 Discovery 配置
5. **命令处理** - 接收 MQTT 命令控制设备

## 许可证

本项目基于 ESP-IDF 示例代码，遵循 ESPRESSIF 许可证。

## 参考资料

- [ESP-IDF 编程指南](https://docs.espressif.com/projects/esp-idf/)
- [ESP-Zigbee-SDK](https://github.com/espressif/esp-zigbee-sdk)
- [Home Assistant MQTT Discovery](https://www.home-assistant.io/integrations/mqtt/#mqtt-discovery)
- [Zigbee Specification](https://zigbeealliance.org/)
