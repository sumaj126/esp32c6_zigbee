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
- [故障排除](#故障排除)
- [代码修改指南](#代码修改指南)
- [恢复步骤](#恢复步骤)
- [实现过程](#实现过程)
- [后续开发](#后续开发)

## 项目概述

本项目实现了一个基于 ESP32-C6 的 ZigBee 协调器，具有以下特性：

- 基于 ZBOSS 协议栈（ESP-IDF 原生支持）
- WiFi 连接 + MQTT 通信
- Home Assistant MQTT Discovery 自动发现
- 断电离线检测（LWT 遗嘱消息）
- 支持最多 10 个 ZigBee 子设备入网
- 支持人体感应传感器和开关设备（单键/双键）
- 实时状态监控和诊断信息
- **设备白名单机制** - 只允许已知设备
- **幽灵设备自动清理** - 心跳定时器自动删除无效设备

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
- （可选）ZigBee 设备用于测试（如人体感应开关）

## 软件依赖

- ESP-IDF v5.x
- Python 3.8+
- ZBOSS 协议栈（ESP-IDF 组件）
- Mosquitto MQTT Broker

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
#define MAX_CHILDREN                    10          // 协调器硬件支持的最大子设备数
#define MAX_ZIGBEE_DEVICES              20          // 代码可管理的设备信息数
#define ESP_ZB_PRIMARY_CHANNEL_MASK     (1l << 13)  // 通道 13
#define ESP_ZB_GATEWAY_ENDPOINT         1           // 网关端点标识符
```

### 设备白名单配置

在 `main/esp_zigbee_gateway.c` 中编辑 `is_device_in_whitelist()` 函数，添加或修改允许的设备 IEEE 地址：

```c
static bool is_device_in_whitelist(const zigbee_device_info_t *device)
{
    const uint8_t dual_switch_ieee[8] = { 0x8c, 0xf0, 0x6f, 0xd9, 0x59, 0x38, 0xc1, 0xa4 };
    const uint8_t single_switch_ieee[8] = { 0x61, 0x23, 0xa5, 0x36, 0x59, 0x38, 0xc1, 0xa4 };
    
    return (memcmp(device->ieee_addr, dual_switch_ieee, 8) == 0 ||
            memcmp(device->ieee_addr, single_switch_ieee, 8) == 0);
}
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
static void mqtt_publish_occupancy_event(...);     // 人体感应事件
```

#### 3. ZigBee 模块

```c
// ZigBee 任务
static void esp_zb_task(void *pvParameters);

// 信号处理
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);

// 核心动作处理
static esp_err_t zb_core_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);

// 设备管理
static zigbee_device_info_t* find_device_by_ieee(const esp_zb_ieee_addr_t ieee_addr);
static zigbee_device_info_t* find_device_by_short_addr(uint16_t short_addr);
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
                            │       └─→ esp_zb_bdb_open_network(180)  // 开放入网 180秒
                            │
                            ├─→ FORMATION:
                            │       ├─→ 网络形成成功
                            │       ├─→ 发布桥接状态和协调器信息
                            │       └─→ 启动网络引导
                            │
                            ├─→ DEVICE_ANNCE:
                            │       ├─→ 白名单检查
                            │       ├─→ 新设备入网
                            │       ├─→ 添加设备到列表
                            │       ├─→ 发布设备公告
                            │       └─→ 查询设备能力
                            │
                            └─→ PERMIT_JOIN_STATUS:
                                    └─→ 发布 HA Discovery 和协调器状态
```

## 功能说明

### 核心设计原则

**重要：只有 Device Annce (ZDO) 可以创建设备！**
- On/Off Report：只更新现有设备，**不创建新设备**
- IAS Zone：只更新现有设备，**不创建新设备**
- Device Annce：唯一可以创建设备的来源，通过白名单过滤

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
| 心跳机制 | ✅ | 每 60 秒发布状态更新 + 清理无效设备 |
| 诊断信息 | ✅ | 重启原因、运行时间、剩余内存 |
| 人体感应支持 | ✅ | 支持 IAS Zone 和 Occupancy Sensing 集群 |
| 开关控制 | ✅ | 支持 On/Off 集群控制（单键/双键） |
| MQTT 命令处理 | ✅ | 通过 MQTT 控制设备开关 |
| 设备白名单 | ✅ | 只允许已知设备加入 |
| 幽灵设备清理 | ✅ | 自动删除不在白名单的设备 |

### MQTT 话题

#### 发布话题

| 话题 | 方向 | 说明 |
|------|------|------|
| `homeassistant/sensor/zigbee_coordinator_{IEEE}/config` | 发布 | HA Discovery 配置 |
| `homeassistant/sensor/zigbee_coordinator_{IEEE}/state` | 发布 | 协调器状态 (online/offline) |
| `homeassistant/sensor/zigbee_coordinator_{IEEE}/attributes` | 发布 | 协调器属性 |
| `homeassistant/binary_sensor/zigbee_{IEEE}/occupancy/config` | 发布 | 人体感应传感器 Discovery |
| `homeassistant/switch/zigbee_{IEEE}/switch/config` | 发布 | 开关设备 Discovery |
| `zigbee2mqtt/bridge/state` | 发布 | 桥接状态 |
| `zigbee2mqtt/bridge/info` | 发布 | 桥接信息 |
| `zigbee2mqtt/bridge/device/{IEEE}` | 发布 | 设备入网公告 |
| `zigbee2mqtt/{IEEE}` | 发布 | 设备状态（如 {"state":"ON"} 或 {"occupancy":true}） |
| `zigbee2mqtt/{IEEE}/occupancy` | 发布 | 人体感应状态（ON/OFF） |

#### 订阅话题

| 话题 | 方向 | 说明 |
|------|------|------|
| `zigbee2mqtt/bridge/#` | 订阅 | 接收控制命令 |
| `zigbee2mqtt/{IEEE}/set` | 订阅 | 接收设备控制命令 |

### 设备信息结构

```c
typedef struct {
    uint16_t short_addr;          // 设备短地址
    esp_zb_ieee_addr_t ieee_addr; // 设备 IEEE 地址
    uint8_t endpoint;             // 设备端点
    uint16_t device_id;           // 设备 ID
    bool ha_discovery_done;       // HA Discovery 是否已完成
    bool pending_confirmation;    // 是否等待 Device Annce 确认
    int64_t created_time_ms;      // 设备创建时间
    bool has_onoff;               // 是否支持开关功能
    bool has_level;               // 是否支持亮度控制
    bool has_temperature;         // 是否支持温度传感器
    bool has_humidity;            // 是否支持湿度传感器
    bool has_occupancy;           // 是否支持人体感应
    int8_t rssi;                  // 设备 RSSI
} zigbee_device_info_t;
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
| `wifi_rssi` | WiFi 信号强度 | -58 |
| `zigbee_devices` | 已连接设备数 | 5 |

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
I (3106) ESP_ZB_GATEWAY: New device commissioned or rejoined (short: 0x4d2d)
I (3106) ESP_ZB_GATEWAY: Total ZigBee devices: 1
I (3116) ESP_ZB_GATEWAY: Querying simple descriptor for device 0x4d2d endpoint 1
I (3156) ESP_ZB_GATEWAY: Device 0x4d2d ep:1 device_id:0x0051
I (3156) ESP_ZB_GATEWAY:   Input clusters: 5, Output clusters: 1
I (3166) ESP_ZB_GATEWAY:   Input cluster[0]: 0x0000
I (3166) ESP_ZB_GATEWAY:   Input cluster[1]: 0x0003
I (3176) ESP_ZB_GATEWAY:   Input cluster[2]: 0x0006
I (3176) ESP_ZB_GATEWAY:   Input cluster[3]: 0x0008
I (3186) ESP_ZB_GATEWAY:   Input cluster[4]: 0x0500
I (3186) ESP_ZB_GATEWAY:   Output cluster[0]: 0x0019
I (3196) ESP_ZB_GATEWAY:   Device has occupancy sensing capability
I (3196) ESP_ZB_GATEWAY: Published HA Discovery for occupancy sensor device a4c1385936a52361
I (3206) ESP_ZB_GATEWAY: Published HA Discovery for switch device a4c1385936a52361
I (3206) ESP_ZB_GATEWAY: Subscribed to zigbee2mqtt/a4c1385936a52361/set
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
4. 当 ZigBee 设备入网后，会自动发现并添加对应的传感器和开关设备

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

## 故障排除

### 常见问题及解决方案

| 问题 | 症状 | 解决方案 |
|------|------|---------|
| WiFi 连接失败 | 串口显示 "WiFi disconnected, retrying..." | 检查 WiFi SSID 和密码是否正确 |
| MQTT 连接失败 | 串口显示 "MQTT Disconnected from broker" | 检查 MQTT Broker IP 是否正确，确保 Mosquitto 服务运行 |
| 设备无法入网 | 协调器不显示设备公告 | 确认入网窗口是否打开（启动后180秒内），检查设备是否处于配对模式 |
| 设备显示但无数据 | HA 中设备显示但无状态更新 | 检查设备是否正常工作，查看串口是否有属性报告 |
| 自定义集群错误 | 串口显示 "Received TO_CLI custom command, cannot find the custom client cluster (0xef00)" | 这是正常的，不影响核心功能，是设备发送的厂商自定义命令 |
| MQTT 命令无响应 | 发送命令后设备无反应 | 检查 IEEE 地址格式是否正确，确保设备已正确添加到列表 |
| 协调器离线 | HA 中显示协调器离线 | 检查电源连接，查看串口是否有重启信息 |
| **幽灵设备** | HA 中出现无效设备 | 确保设备在白名单中，心跳定时器会自动清理 |
| **设备被删除** | 真实设备被自动删除 | 检查白名单配置，确保设备 IEEE 地址在列表中 |

### 诊断命令

```bash
# 查看 MQTT 消息
mosquitto_sub -h 192.168.1.10 -t '#' -v

# 测试 MQTT 连接
mosquitto_pub -h 192.168.1.10 -t 'test' -m 'hello'

# 查看 ESP32 串口输出
idf.py -p COM3 monitor
```

## 代码修改指南

### 注意事项

1. **不要修改**以下核心配置：
   - `ESP_ZB_GATEWAY_ENDPOINT`：必须保持为 1
   - `ESP_ZB_PRIMARY_CHANNEL_MASK`：除非有特殊需求，否则保持为通道 13
   - `MAX_CHILDREN`：不要超过硬件限制（建议不超过 10）

2. **修改时需要注意**：
   - 所有修改前请备份原始文件
   - 保持代码风格一致
   - 确保所有函数调用都有错误处理
   - 测试修改后的功能

3. **推荐修改点**：
   - `MQTT_TOPIC_PREFIX`：可根据需要修改为自定义前缀
   - `HEARTBEAT_INTERVAL_SEC`：可调整心跳间隔
   - `PENDING_DEVICE_TIMEOUT_SEC`：可调整 PENDING 设备超时时间
   - 设备能力检测逻辑：可根据需要添加对其他集群的支持
   - `is_device_in_whitelist()`：修改设备白名单

### 代码结构说明

| 模块 | 主要功能 | 文件位置 | 注意事项 |
|------|---------|---------|----------|
| WiFi 模块 | 负责 WiFi 连接和重连 | esp_zigbee_gateway.c:89-441 | 确保 WiFi 配置正确 |
| MQTT 模块 | 负责 MQTT 连接和消息处理 | esp_zigbee_gateway.c:444-531 | 确保 MQTT Broker 配置正确 |
| ZigBee 核心 | 负责 ZigBee 网络管理 | esp_zigbee_gateway.c:1023-1125 | 不要修改核心网络参数 |
| 设备管理 | 负责设备的添加、查找和管理 | esp_zigbee_gateway.c:248-267 | 注意设备数量限制 |
| 命令处理 | 负责处理 MQTT 命令 | esp_zigbee_gateway.c:299-366 | 注意 IEEE 地址解析 |
| 事件处理 | 负责处理 ZigBee 事件 | esp_zigbee_gateway.c:736-980 | 注意事件类型处理 |
| 白名单检查 | 设备白名单验证 | esp_zigbee_gateway.c:276-283 | 修改时添加新设备 |

## 恢复步骤

如果代码修改后出现问题，可以按照以下步骤恢复：

1. **备份当前代码**（如果需要）
   ```bash
   cp -r esp32c6_zigbee esp32c6_zigbee_backup
   ```

2. **恢复原始代码**
   - 从版本控制系统中恢复（如 git）
   ```bash
   git checkout main/esp_zigbee_gateway.c
   git checkout main/esp_zigbee_gateway.h
   ```
   - 或从备份中恢复
   ```bash
   cp esp32c6_zigbee_backup/main/esp_zigbee_gateway.c main/
   cp esp32c6_zigbee_backup/main/esp_zigbee_gateway.h main/
   ```

3. **重新编译和烧录**
   ```bash
   idf.py clean
   idf.py build
   idf.py -p COM3 flash monitor
   ```

4. **验证恢复**
   - 检查串口输出是否正常
   - 确认 MQTT 连接成功
   - 测试设备入网功能

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

### 阶段四：设备支持

1. 实现设备入网检测和管理
2. 添加人体感应传感器支持（IAS Zone 和 Occupancy Sensing）
3. 添加开关控制支持（On/Off 集群）
4. 实现 MQTT 命令控制
5. 添加双键开关支持（按 IEEE 地址检测）

### 阶段五：问题修复

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
| 设备命令无响应 | 实现 IEEE 地址双向解析 |
| 幽灵设备过多 | 添加白名单机制 + 心跳清理 |
| 设备被误删 | 移除 On/Off Report/IAS Zone 的设备创建 |
| 人体感应一直显示 ON | 添加初始 OFF 状态发布 |

## 后续开发

### 待实现功能

| 功能 | 优先级 | 说明 |
|------|--------|------|
| 温度/湿度传感器支持 | 高 | 解析温度、湿度属性 |
| 亮度控制支持 | 中 | 支持 Level Control 集群 |
| 入网窗口远程控制 | 中 | 通过 MQTT 命令开启入网窗口 |
| OTA 升级 | 低 | 支持 OTA 固件升级 |
| 多通道支持 | 低 | 支持多个 ZigBee 通道 |

### ZigBee 设备开发路线

1. **设备入网检测** - 已实现设备公告发布
2. **属性发现** - 读取设备的 cluster 和 attribute
3. **数据转换** - 将 ZigBee 数据转为 MQTT 消息
4. **HA Discovery** - 为每个设备生成 Discovery 配置
5. **命令处理** - 接收 MQTT 命令控制设备
6. **状态同步** - 确保设备状态与 HA 同步

## 许可证

本项目基于 ESP-IDF 示例代码，遵循 ESPRESSIF 许可证。

## 参考资料

- [ESP-IDF 编程指南](https://docs.espressif.com/projects/esp-idf/)
- [ESP-Zigbee-SDK](https://github.com/espressif/esp-zigbee-sdk)
- [Home Assistant MQTT Discovery](https://www.home-assistant.io/integrations/mqtt/#mqtt-discovery)
- [Zigbee Specification](https://zigbeealliance.org/)
- [ZCL 规范](https://zigbeealliance.org/wp-content/uploads/2019/11/docs-05-3474-21-0csg-zigbee-cluster-library-specification.pdf)
