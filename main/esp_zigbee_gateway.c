/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: LicenseRef-Included
 *
 * Zigbee Gateway Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "driver/usb_serial_jtag.h"
#include "esp_coexist.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "esp_vfs_eventfd.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_zigbee_gateway.h"
#include "zb_config_platform.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include "zdo/esp_zigbee_zdo_command.h"

static const char *TAG = "ESP_ZB_GATEWAY";

/* WiFi Configuration */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;

/* MQTT Configuration */
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;
static bool ha_discovery_published = false;

/* ZigBee device tracking */
static uint8_t zigbee_device_count = 0;
#define MAX_ZIGBEE_DEVICES 20

/* Device info structure */
typedef struct {
    uint16_t short_addr;
    esp_zb_ieee_addr_t ieee_addr;
    uint8_t endpoint;
    uint16_t device_id;
    bool ha_discovery_done;
    bool has_onoff;
    bool has_level;
    bool has_temperature;
    bool has_humidity;
    bool has_occupancy;  // 人体感应
} zigbee_device_info_t;

static zigbee_device_info_t zigbee_devices[MAX_ZIGBEE_DEVICES];

/* Forward declarations */
static void mqtt_publish_coordinator_status(void);
static void heartbeat_timer_cb(uint8_t param);
static void handle_device_mqtt_command(char *ieee_str, char *data, int data_len);
static void send_onoff_command(zigbee_device_info_t *device, bool onoff);
static zigbee_device_info_t* find_device_by_ieee(const esp_zb_ieee_addr_t ieee_addr);
static zigbee_device_info_t* find_device_by_short_addr(uint16_t short_addr);
static void ieee_addr_to_string(esp_zb_ieee_addr_t ieee_addr, char *buf, size_t buf_len);
static void query_device_simple_desc(uint16_t short_addr, uint8_t endpoint);
static void simple_desc_resp_handler(esp_zb_zdp_status_t zdo_status, esp_zb_af_simple_desc_1_1_t *simple_desc, void *user_ctx);
static esp_err_t zb_core_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);
static void mqtt_publish_occupancy_event(zigbee_device_info_t *device, bool occupied);

/* Heartbeat interval in seconds */
#define HEARTBEAT_INTERVAL_SEC  60

/* Get WiFi RSSI */
static int8_t get_wifi_rssi(void)
{
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        return ap_info.rssi;
    }
    return 0;  // Return 0 if unable to get RSSI
}

/* Get device IEEE address as string */
static void ieee_addr_to_string(esp_zb_ieee_addr_t ieee_addr, char *buf, size_t buf_len)
{
    snprintf(buf, buf_len, "%02x%02x%02x%02x%02x%02x%02x%02x",
             ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4],
             ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0]);
}

/* User context for ZDO requests */
typedef struct {
    uint16_t short_addr;
} zdo_user_ctx_t;

static zdo_user_ctx_t zdo_ctx_pool[MAX_ZIGBEE_DEVICES];
static int zdo_ctx_index = 0;

/* ZDO Simple Descriptor callback */
static void mqtt_publish_ha_discovery_for_sensor(zigbee_device_info_t *device)
{
    if (!mqtt_connected || mqtt_client == NULL || device == NULL) {
        return;
    }
    
    char ieee_str[20];
    ieee_addr_to_string(device->ieee_addr, ieee_str, sizeof(ieee_str));
    
    char topic[128];
    char payload[512];
    
    // Publish HA Discovery for occupancy sensor
    snprintf(topic, sizeof(topic), "homeassistant/binary_sensor/zigbee_%s/occupancy/config", ieee_str);
    snprintf(payload, sizeof(payload),
             "{\"name\":\"人体感应传感器\",\"unique_id\":\"zigbee_%s_occupancy\",\"state_topic\":\"%s/%s/occupancy\",\"device_class\":\"occupancy\",\"device\":{\"identifiers\":[\"zigbee_%s\"],\"manufacturer\":\"Unknown\",\"model\":\"Occupancy Switch\"}}",
             ieee_str, MQTT_TOPIC_PREFIX, ieee_str, ieee_str);
    
    esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 1);
    ESP_LOGI(TAG, "Published HA Discovery for occupancy sensor device %s", ieee_str);
}

static void mqtt_publish_ha_discovery_for_switch(zigbee_device_info_t *device)
{
    if (!mqtt_connected || mqtt_client == NULL || device == NULL) {
        return;
    }
    
    char ieee_str[20];
    ieee_addr_to_string(device->ieee_addr, ieee_str, sizeof(ieee_str));
    
    char topic[128];
    char payload[512];
    
    // Publish HA Discovery for switch
    snprintf(topic, sizeof(topic), "homeassistant/switch/zigbee_%s/switch/config", ieee_str);
    snprintf(payload, sizeof(payload),
             "{\"name\":\"人体感应开关\",\"unique_id\":\"zigbee_%s_switch\",\"command_topic\":\"%s/%s/set\",\"state_topic\":\"%s/%s\",\"value_template\":\"{{ value_json.state }}\",\"device\":{\"identifiers\":[\"zigbee_%s\"],\"name\":\"人体感应开关\",\"manufacturer\":\"Unknown\",\"model\":\"Occupancy Switch\"}}",
             ieee_str, MQTT_TOPIC_PREFIX, ieee_str, MQTT_TOPIC_PREFIX, ieee_str, ieee_str);
    
    esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 1);
    ESP_LOGI(TAG, "Published HA Discovery for switch device %s", ieee_str);
}

static void simple_desc_resp_handler(esp_zb_zdp_status_t zdo_status, esp_zb_af_simple_desc_1_1_t *simple_desc, void *user_ctx)
{
    zdo_user_ctx_t *ctx = (zdo_user_ctx_t *)user_ctx;
    uint16_t short_addr = ctx->short_addr;
    
    if (zdo_status != ESP_ZB_ZDP_STATUS_SUCCESS || simple_desc == NULL) {
        ESP_LOGW(TAG, "Simple descriptor request failed for 0x%04x", short_addr);
        return;
    }

    zigbee_device_info_t *device = find_device_by_short_addr(short_addr);
    if (device == NULL) {
        ESP_LOGW(TAG, "Device 0x%04x not found in list", short_addr);
        return;
    }

    device->endpoint = simple_desc->endpoint;
    device->device_id = simple_desc->app_device_id;
    
    ESP_LOGI(TAG, "Device 0x%04x ep:%d device_id:0x%04x", short_addr, simple_desc->endpoint, simple_desc->app_device_id);
    ESP_LOGI(TAG, "  Input clusters: %d, Output clusters: %d", 
             simple_desc->app_input_cluster_count, simple_desc->app_output_cluster_count);

    /* Parse input clusters */
    device->has_onoff = false;
    device->has_level = false;
    device->has_temperature = false;
    device->has_humidity = false;
    device->has_occupancy = false;

    for (int i = 0; i < simple_desc->app_input_cluster_count; i++) {
        uint16_t cluster = simple_desc->app_cluster_list[i];
        ESP_LOGI(TAG, "  Input cluster[%d]: 0x%04x", i, cluster);
        
        switch (cluster) {
            case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
                device->has_onoff = true;
                break;
            case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
                device->has_level = true;
                break;
            case ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT:
                device->has_temperature = true;
                break;
            case ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT:
                device->has_humidity = true;
                break;
            case ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE:
            case ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING:
                device->has_occupancy = true;
                ESP_LOGI(TAG, "  Device has occupancy sensing capability");
                break;
        }
    }
    
    // Publish HA Discovery for sensor if it has occupancy capability
    if (device->has_occupancy) {
        mqtt_publish_ha_discovery_for_sensor(device);
    }
    // Also publish HA Discovery for switch if it has on/off capability
    if (device->has_onoff) {
        mqtt_publish_ha_discovery_for_switch(device);
        char ieee_str[20];
        ieee_addr_to_string(device->ieee_addr, ieee_str, sizeof(ieee_str));
        char topic[64];
        snprintf(topic, sizeof(topic), "%s/%s/set", MQTT_TOPIC_PREFIX, ieee_str);
        esp_mqtt_client_subscribe(mqtt_client, topic, 0);
        ESP_LOGI(TAG, "Subscribed to %s", topic);
    }
    // Mark discovery as done regardless of capabilities
    device->ha_discovery_done = true;
}

/* Query device simple descriptor */
static void query_device_simple_desc(uint16_t short_addr, uint8_t endpoint)
{
    static esp_zb_zdo_simple_desc_req_param_t req;
    req.addr_of_interest = short_addr;
    req.endpoint = endpoint;
    
    zdo_user_ctx_t *ctx = &zdo_ctx_pool[zdo_ctx_index % MAX_ZIGBEE_DEVICES];
    zdo_ctx_index++;
    ctx->short_addr = short_addr;
    
    esp_zb_zdo_simple_desc_req(&req, simple_desc_resp_handler, ctx);
}

/* Find device by short address */
static zigbee_device_info_t* find_device_by_short_addr(uint16_t short_addr)
{
    for (int i = 0; i < zigbee_device_count; i++) {
        if (zigbee_devices[i].short_addr == short_addr) {
            return &zigbee_devices[i];
        }
    }
    return NULL;
}

/* Find device by IEEE address */
static zigbee_device_info_t* find_device_by_ieee(const esp_zb_ieee_addr_t ieee_addr)
{
    for (int i = 0; i < zigbee_device_count; i++) {
        if (memcmp(zigbee_devices[i].ieee_addr, ieee_addr, sizeof(esp_zb_ieee_addr_t)) == 0) {
            return &zigbee_devices[i];
        }
    }
    return NULL;
}

/* Send On/Off command to device */
static void send_onoff_command(zigbee_device_info_t *device, bool onoff)
{
    if (device == NULL) {
        return;
    }

    esp_zb_zcl_on_off_cmd_t cmd;
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = device->short_addr;
    cmd.zcl_basic_cmd.dst_endpoint = device->endpoint;
    cmd.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.on_off_cmd_id = onoff ? ESP_ZB_ZCL_CMD_ON_OFF_ON_ID : ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID;

    esp_zb_zcl_on_off_cmd_req(&cmd);
    ESP_LOGI(TAG, "Sent %s command to device 0x%04x", onoff ? "ON" : "OFF", device->short_addr);
    
    // Publish switch state to MQTT
    if (mqtt_connected && mqtt_client != NULL) {
        char ieee_str[20];
        ieee_addr_to_string(device->ieee_addr, ieee_str, sizeof(ieee_str));
        char topic[64];
        char payload[64];
        snprintf(topic, sizeof(topic), "%s/%s", MQTT_TOPIC_PREFIX, ieee_str);
        snprintf(payload, sizeof(payload), "{\"state\":\"%s\"}", onoff ? "ON" : "OFF");
        int msg_id = esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
        ESP_LOGI(TAG, "Published switch state to MQTT: %s, msg_id=%d", onoff ? "ON" : "OFF", msg_id);
    }
}

/* Handle MQTT command for device */
static void handle_device_mqtt_command(char *ieee_str, char *data, int data_len)
{
    /* Find device by IEEE address */
    if (strlen(ieee_str) != 16) {
        ESP_LOGW(TAG, "Invalid IEEE address format: %s", ieee_str);
        return;
    }
    
    /* Parse IEEE address */
    // Try different byte orders to find the device
    esp_zb_ieee_addr_t ieee_addr1, ieee_addr2;
    
    // Order 1: direct order (as received)
    for (int i = 0; i < 8; i++) {
        char byte_str[3] = {ieee_str[i*2], ieee_str[i*2+1], 0};
        ieee_addr1[i] = (uint8_t)strtol(byte_str, NULL, 16);
    }
    
    // Order 2: reverse order (swap endian)
    for (int i = 0; i < 8; i++) {
        char byte_str[3] = {ieee_str[i*2], ieee_str[i*2+1], 0};
        ieee_addr2[7-i] = (uint8_t)strtol(byte_str, NULL, 16);
    }
    
    // Try to find device with both orders
    zigbee_device_info_t *device = find_device_by_ieee(ieee_addr1);
    if (device == NULL) {
        device = find_device_by_ieee(ieee_addr2);
    }
    
    // If still not found, try to find by short address (if available)
    if (device == NULL) {
        // Try to extract short address from IEEE address (last 2 bytes)
        uint16_t short_addr = 0;
        char short_addr_str[5] = {0};
        strncpy(short_addr_str, ieee_str + 12, 4);
        short_addr = (uint16_t)strtol(short_addr_str, NULL, 16);
        device = find_device_by_short_addr(short_addr);
        if (device != NULL) {
            ESP_LOGI(TAG, "Found device by short address: 0x%04x", short_addr);
        }
    }
    if (device == NULL) {
        ESP_LOGW(TAG, "Device not found: %s", ieee_str);
        // Log all devices for debugging
        ESP_LOGW(TAG, "Current devices in list:");
        for (int i = 0; i < zigbee_device_count; i++) {
            char device_ieee_str[20];
            ieee_addr_to_string(zigbee_devices[i].ieee_addr, device_ieee_str, sizeof(device_ieee_str));
            ESP_LOGW(TAG, "  Device %d: short=0x%04x, ieee=%s, endpoint=%d", 
                     i, zigbee_devices[i].short_addr, device_ieee_str, zigbee_devices[i].endpoint);
        }
        return;
    }
    
    ESP_LOGI(TAG, "Found device: short=0x%04x, ieee=%s, endpoint=%d", 
             device->short_addr, ieee_str, device->endpoint);

    /* Parse command */
    if (strcasecmp(data, "ON") == 0 || strstr(data, "ON")) {
        send_onoff_command(device, true);
    } else if (strcasecmp(data, "OFF") == 0 || strstr(data, "OFF")) {
        send_onoff_command(device, false);
    } else {
        ESP_LOGW(TAG, "Unknown command: %s", data);
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // Always reconnect WiFi (infinite retry)
        mqtt_connected = false;  // Mark MQTT as disconnected
        esp_wifi_connect();
        s_retry_num++;
        ESP_LOGW(TAG, "WiFi disconnected, retrying... (count: %d)", s_retry_num);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                         ESP_EVENT_ANY_ID,
                                                         &wifi_event_handler,
                                                         NULL,
                                                         &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                         IP_EVENT_STA_GOT_IP,
                                                         &wifi_event_handler,
                                                         NULL,
                                                         &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID:%s", WIFI_SSID);
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
        return ESP_FAIL;
    }

    return ESP_FAIL;
}

/* MQTT Event Handler */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    
    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT Connected to broker");
        mqtt_connected = true;
        // Subscribe to bridge topics
        esp_mqtt_client_subscribe(event->client, MQTT_TOPIC_PREFIX "/bridge/#", 0);
        // Subscribe to device command topics
        esp_mqtt_client_subscribe(event->client, MQTT_TOPIC_PREFIX "/+/set", 0);
        // Publish online status immediately to clear any retained offline
        mqtt_publish_coordinator_status();
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT Disconnected from broker, will auto-reconnect");
        mqtt_connected = false;
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT Subscribed, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        // Parse topic to check if it's a device command
        char topic[128];
        int topic_len = event->topic_len < sizeof(topic) - 1 ? event->topic_len : sizeof(topic) - 1;
        strncpy(topic, event->topic, topic_len);
        topic[topic_len] = '\0';
        
        // Check if it's a device set command: zigbee2mqtt/<ieee>/set
        if (strncmp(topic, MQTT_TOPIC_PREFIX "/", strlen(MQTT_TOPIC_PREFIX) + 1) == 0) {
            char *ieee_start = topic + strlen(MQTT_TOPIC_PREFIX) + 1;
            char *set_pos = strstr(ieee_start, "/set");
            if (set_pos != NULL) {
                *set_pos = '\0';  // Terminate IEEE address
                char *data = strndup(event->data, event->data_len);
                if (data != NULL) {
                    ESP_LOGI(TAG, "MQTT Data: topic=%s, data=%s", topic, data);
                    handle_device_mqtt_command(ieee_start, data, event->data_len);
                    free(data);
                }
            }
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT Error");
        break;
    default:
        break;
    }
}

static void mqtt_app_start(void)
{
    // Build LWT topic
    esp_zb_ieee_addr_t ieee_address;
    esp_zb_get_long_address(ieee_address);
    static char lwt_topic[128];
    snprintf(lwt_topic, sizeof(lwt_topic), 
             "homeassistant/sensor/zigbee_coordinator_%02x%02x%02x%02x%02x%02x%02x%02x/state",
             ieee_address[7], ieee_address[6], ieee_address[5], ieee_address[4],
             ieee_address[3], ieee_address[2], ieee_address[1], ieee_address[0]);
    
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.hostname = MQTT_BROKER_IP,
        .broker.address.port = MQTT_BROKER_PORT,
        .broker.address.transport = MQTT_TRANSPORT_OVER_TCP,
        .session.keepalive = 10,
        .session.last_will = {
            .topic = lwt_topic,
            .msg = "offline",
            .qos = 1,
            .retain = true,
        },
    };
    
    if (MQTT_USERNAME != NULL) {
        mqtt_cfg.credentials.username = MQTT_USERNAME;
    }
    if (MQTT_PASSWORD != NULL) {
        mqtt_cfg.credentials.authentication.password = MQTT_PASSWORD;
    }

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
    ESP_LOGI(TAG, "MQTT client started, connecting to %s:%d", MQTT_BROKER_IP, MQTT_BROKER_PORT);
}

static void mqtt_publish_device_announce(uint16_t short_addr, esp_zb_ieee_addr_t ieee_addr)
{
    if (!mqtt_connected || mqtt_client == NULL) {
        ESP_LOGW(TAG, "MQTT not connected, cannot publish device announce");
        return;
    }
    
    char ieee_str[20];
    ieee_addr_to_string(ieee_addr, ieee_str, sizeof(ieee_str));
    
    char topic[64];
    char payload[256];
    
    snprintf(topic, sizeof(topic), "%s/bridge/device/%s", MQTT_TOPIC_PREFIX, ieee_str);
    snprintf(payload, sizeof(payload), 
             "{\"ieeeAddr\":\"%s\",\"type\":\"Unknown\",\"networkAddress\":%d}",
             ieee_str, short_addr);
    
    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
    ESP_LOGI(TAG, "Published device announce to MQTT, msg_id=%d", msg_id);
}

static void mqtt_publish_bridge_state(const char *state)
{
    if (!mqtt_connected || mqtt_client == NULL) {
        return;
    }
    
    char topic[64];
    snprintf(topic, sizeof(topic), "%s/bridge/state", MQTT_TOPIC_PREFIX);
    esp_mqtt_client_publish(mqtt_client, topic, state, 0, 1, 1);
}

static void mqtt_publish_coordinator_info(void)
{
    if (!mqtt_connected || mqtt_client == NULL) {
        return;
    }
    
    char topic[128];
    char payload[512];
    esp_zb_ieee_addr_t ieee_address;
    esp_zb_get_long_address(ieee_address);
    
    snprintf(topic, sizeof(topic), "%s/bridge/info", MQTT_TOPIC_PREFIX);
    snprintf(payload, sizeof(payload),
             "{\"version\":\"1.0.0\",\"commit\":\"unknown\",\"coordinator\":{\"type\":\"zboss\",\"meta\":{\"version\":\"1.6.0\",\"ieeeAddr\":\"%02x%02x%02x%02x%02x%02x%02x%02x\",\"nwkAddr\":0,\"panId\":\"0x%04hx\",\"channel\":%d}}}",
             ieee_address[7], ieee_address[6], ieee_address[5], ieee_address[4],
             ieee_address[3], ieee_address[2], ieee_address[1], ieee_address[0],
             esp_zb_get_pan_id(), esp_zb_get_current_channel());
    
    esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
}

static void mqtt_publish_ha_discovery(void)
{
    if (!mqtt_connected || mqtt_client == NULL) {
        return;
    }
    
    esp_zb_ieee_addr_t ieee_address;
    esp_zb_get_long_address(ieee_address);
    
    char device_id[32];
    snprintf(device_id, sizeof(device_id), "%02x%02x%02x%02x%02x%02x%02x%02x",
             ieee_address[7], ieee_address[6], ieee_address[5], ieee_address[4],
             ieee_address[3], ieee_address[2], ieee_address[1], ieee_address[0]);
    
    // Device config sensor
    char topic[128];
    char payload[1024];
    
    snprintf(topic, sizeof(topic), "homeassistant/sensor/zigbee_coordinator_%s/config", device_id);
    snprintf(payload, sizeof(payload),
             "{\"name\":\"ZigBee Coordinator\",\"unique_id\":\"zigbee_coordinator_%s\",\"state_topic\":\"homeassistant/sensor/zigbee_coordinator_%s/state\",\"json_attributes_topic\":\"homeassistant/sensor/zigbee_coordinator_%s/attributes\",\"device\":{\"identifiers\":[\"zigbee_coordinator_%s\"],\"name\":\"ZigBee Coordinator\",\"manufacturer\":\"Espressif\",\"model\":\"ESP32-C6\"},\"icon\":\"mdi:zigbee\"}",
             device_id, device_id, device_id, device_id);
    esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 1);
    
    ESP_LOGI(TAG, "Published HA Discovery for coordinator");
}

static void publish_device_states(void)
{
    if (!mqtt_connected || mqtt_client == NULL) {
        return;
    }
    
    // Publish state for all devices
    for (int i = 0; i < zigbee_device_count; i++) {
        zigbee_device_info_t *device = &zigbee_devices[i];
        char ieee_str[20];
        ieee_addr_to_string(device->ieee_addr, ieee_str, sizeof(ieee_str));
        
        // Publish switch state (default to OFF if unknown)
        char topic[64];
        char payload[64];
        snprintf(topic, sizeof(topic), "%s/%s", MQTT_TOPIC_PREFIX, ieee_str);
        snprintf(payload, sizeof(payload), "{\"state\":\"OFF\",\"occupancy\":false}");
        esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
        ESP_LOGI(TAG, "Published initial state for device %s: OFF", ieee_str);
    }
}

static void mqtt_publish_coordinator_status(void)
{
    if (!mqtt_connected || mqtt_client == NULL) {
        return;
    }
    
    esp_zb_ieee_addr_t ieee_address;
    esp_zb_get_long_address(ieee_address);
    
    char device_id[32];
    snprintf(device_id, sizeof(device_id), "%02x%02x%02x%02x%02x%02x%02x%02x",
             ieee_address[7], ieee_address[6], ieee_address[5], ieee_address[4],
             ieee_address[3], ieee_address[2], ieee_address[1], ieee_address[0]);
    
    char topic[128];
    char payload[768];
    
    // State
    snprintf(topic, sizeof(topic), "homeassistant/sensor/zigbee_coordinator_%s/state", device_id);
    esp_mqtt_client_publish(mqtt_client, topic, "online", 0, 1, 1);
    
    // Get reset reason
    const char* reset_reason_str = "unknown";
    esp_reset_reason_t reset_reason = esp_reset_reason();
    switch (reset_reason) {
        case ESP_RST_POWERON:       reset_reason_str = "power_on"; break;
        case ESP_RST_EXT:           reset_reason_str = "external"; break;
        case ESP_RST_SW:            reset_reason_str = "software"; break;
        case ESP_RST_PANIC:         reset_reason_str = "panic"; break;
        case ESP_RST_INT_WDT:       reset_reason_str = "int_wdt"; break;
        case ESP_RST_TASK_WDT:      reset_reason_str = "task_wdt"; break;
        case ESP_RST_WDT:           reset_reason_str = "wdt"; break;
        case ESP_RST_DEEPSLEEP:     reset_reason_str = "deepsleep"; break;
        case ESP_RST_BROWNOUT:      reset_reason_str = "brownout"; break;
        case ESP_RST_SDIO:          reset_reason_str = "sdio"; break;
        default:                    reset_reason_str = "other"; break;
    }
    
    // Get uptime in seconds
    int uptime_sec = (int)(esp_timer_get_time() / 1000000);
    int uptime_hours = uptime_sec / 3600;
    int uptime_mins = (uptime_sec % 3600) / 60;
    int uptime_secs = uptime_sec % 60;

    // Get WiFi RSSI
    int8_t wifi_rssi = get_wifi_rssi();

    // Attributes with diagnostics
    snprintf(topic, sizeof(topic), "homeassistant/sensor/zigbee_coordinator_%s/attributes", device_id);

    // Build payload with WiFi RSSI and ZigBee device status
    if (zigbee_device_count == 0) {
        snprintf(payload, sizeof(payload),
                 "{\"pan_id\":\"0x%04hx\",\"channel\":%d,\"short_addr\":\"0x%04hx\",\"ieee\":\"%s\","
                 "\"reset_reason\":\"%s\",\"uptime\":\"%02d:%02d:%02d\",\"free_heap\":%lu,"
                 "\"wifi_rssi\":%d,\"zigbee_devices\":\"N/A\"}",
                 esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address(), device_id,
                 reset_reason_str, uptime_hours, uptime_mins, uptime_secs,
                 (unsigned long)esp_get_free_heap_size(), wifi_rssi);
    } else {
        snprintf(payload, sizeof(payload),
                 "{\"pan_id\":\"0x%04hx\",\"channel\":%d,\"short_addr\":\"0x%04hx\",\"ieee\":\"%s\","
                 "\"reset_reason\":\"%s\",\"uptime\":\"%02d:%02d:%02d\",\"free_heap\":%lu,"
                 "\"wifi_rssi\":%d,\"zigbee_devices\":%d}",
                 esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address(), device_id,
                 reset_reason_str, uptime_hours, uptime_mins, uptime_secs,
                 (unsigned long)esp_get_free_heap_size(), wifi_rssi, zigbee_device_count);
    }
    esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 1);
    
    // Publish initial states for all devices
    publish_device_states();
}

static void mqtt_publish_occupancy_event(zigbee_device_info_t *device, bool occupied)
{
    if (!mqtt_connected || mqtt_client == NULL || device == NULL) {
        return;
    }
    
    char ieee_str[20];
    ieee_addr_to_string(device->ieee_addr, ieee_str, sizeof(ieee_str));
    
    char topic[64];
    char payload[64];
    
    // Publish to the device's main topic
    snprintf(topic, sizeof(topic), "%s/%s", MQTT_TOPIC_PREFIX, ieee_str);
    snprintf(payload, sizeof(payload), "{\"occupancy\":%s,\"state\":\"%s\"}", 
             occupied ? "true" : "false", 
             occupied ? "ON" : "OFF");
    
    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
    ESP_LOGI(TAG, "Published occupancy event to MQTT: %s, msg_id=%d", occupied ? "occupied" : "not occupied", msg_id);
    
    // Also publish to a dedicated occupancy topic for sensor
    snprintf(topic, sizeof(topic), "%s/%s/occupancy", MQTT_TOPIC_PREFIX, ieee_str);
    snprintf(payload, sizeof(payload), "%s", occupied ? "ON" : "OFF");
    esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
}

static esp_err_t zb_core_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    switch (callback_id) {
        case ESP_ZB_CORE_CMD_IAS_ZONE_ZONE_STATUS_CHANGE_NOT_ID: {
            // Handle IAS Zone status change notification (human presence detection)
            const esp_zb_zcl_ias_zone_status_change_notification_message_t *zone_msg = (
                const esp_zb_zcl_ias_zone_status_change_notification_message_t *)message;
            
            ESP_LOGI(TAG, "IAS Zone status change notification received");
            ESP_LOGI(TAG, "  Source endpoint: %d", zone_msg->info.src_endpoint);
            ESP_LOGI(TAG, "  Zone status: 0x%04x", zone_msg->zone_status);
            ESP_LOGI(TAG, "  Zone ID: %d", zone_msg->zone_id);
            
            // Check if zone status indicates occupancy
            bool occupied = (zone_msg->zone_status & 0x0001) != 0; // Bit 0: Alarm1
            ESP_LOGI(TAG, "  Occupancy status: %s", occupied ? "occupied" : "not occupied");
            
            // Get source address based on address type
            uint16_t src_short_addr = 0;
            if (zone_msg->info.src_address.addr_type == ESP_ZB_ZCL_ADDR_TYPE_SHORT) {
                src_short_addr = zone_msg->info.src_address.u.short_addr;
                ESP_LOGI(TAG, "  Source address (short): 0x%04x", src_short_addr);
            } else if (zone_msg->info.src_address.addr_type == ESP_ZB_ZCL_ADDR_TYPE_IEEE) {
                ESP_LOGI(TAG, "  Source address (IEEE): %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                         zone_msg->info.src_address.u.ieee_addr[0], zone_msg->info.src_address.u.ieee_addr[1],
                         zone_msg->info.src_address.u.ieee_addr[2], zone_msg->info.src_address.u.ieee_addr[3],
                         zone_msg->info.src_address.u.ieee_addr[4], zone_msg->info.src_address.u.ieee_addr[5],
                         zone_msg->info.src_address.u.ieee_addr[6], zone_msg->info.src_address.u.ieee_addr[7]);
                // Try to find device by IEEE address
                zigbee_device_info_t *device = find_device_by_ieee(zone_msg->info.src_address.u.ieee_addr);
                if (device != NULL) {
                    ESP_LOGI(TAG, "  Device found by IEEE address: short=0x%04x", device->short_addr);
                    // Publish occupancy event to MQTT
                    mqtt_publish_occupancy_event(device, occupied);
                } else {
                    ESP_LOGW(TAG, "  Device not found by IEEE address");
                    // Log all devices for debugging
                    ESP_LOGW(TAG, "  Current devices in list:");
                    for (int i = 0; i < zigbee_device_count; i++) {
                        char device_ieee_str[20];
                        ieee_addr_to_string(zigbee_devices[i].ieee_addr, device_ieee_str, sizeof(device_ieee_str));
                        ESP_LOGW(TAG, "    Device %d: short=0x%04x, ieee=%s", 
                                 i, zigbee_devices[i].short_addr, device_ieee_str);
                    }
                }
                break;
            }
            
            // Find device by short address
            zigbee_device_info_t *device = find_device_by_short_addr(src_short_addr);
            if (device != NULL) {
                ESP_LOGI(TAG, "  Device found: short=0x%04x, ieee=", device->short_addr);
                char ieee_str[20];
                ieee_addr_to_string(device->ieee_addr, ieee_str, sizeof(ieee_str));
                ESP_LOGI(TAG, "  IEEE: %s", ieee_str);
                
                // Publish occupancy event to MQTT
                mqtt_publish_occupancy_event(device, occupied);
            } else {
                ESP_LOGW(TAG, "  Device not found for short address: 0x%04x", src_short_addr);
                // Try to find device by IEEE address if available
                if (zone_msg->info.src_address.addr_type == ESP_ZB_ZCL_ADDR_TYPE_IEEE) {
                    device = find_device_by_ieee(zone_msg->info.src_address.u.ieee_addr);
                    if (device != NULL) {
                        ESP_LOGI(TAG, "  Device found by IEEE address: short=0x%04x", device->short_addr);
                        // Update short address
                        device->short_addr = src_short_addr;
                        ESP_LOGI(TAG, "  Updated device short address to: 0x%04x", src_short_addr);
                        // Publish occupancy event to MQTT
                        mqtt_publish_occupancy_event(device, occupied);
                    } else {
                        // Log all devices for debugging
                        ESP_LOGW(TAG, "  Current devices in list:");
                        for (int i = 0; i < zigbee_device_count; i++) {
                            char device_ieee_str[20];
                            ieee_addr_to_string(zigbee_devices[i].ieee_addr, device_ieee_str, sizeof(device_ieee_str));
                            ESP_LOGW(TAG, "    Device %d: short=0x%04x, ieee=%s", 
                                     i, zigbee_devices[i].short_addr, device_ieee_str);
                        }
                    }
                } else {
                    // Log all devices for debugging
                    ESP_LOGW(TAG, "  Current devices in list:");
                    for (int i = 0; i < zigbee_device_count; i++) {
                        char device_ieee_str[20];
                        ieee_addr_to_string(zigbee_devices[i].ieee_addr, device_ieee_str, sizeof(device_ieee_str));
                        ESP_LOGW(TAG, "    Device %d: short=0x%04x, ieee=%s", 
                                 i, zigbee_devices[i].short_addr, device_ieee_str);
                    }
                }
            }
            break;
        }
        case ESP_ZB_CORE_CMD_IAS_ZONE_ZONE_ENROLL_REQUEST_ID: {
            // Handle IAS Zone enroll request
            const esp_zb_zcl_ias_zone_enroll_request_message_t *enroll_msg = (
                const esp_zb_zcl_ias_zone_enroll_request_message_t *)message;
            
            ESP_LOGI(TAG, "IAS Zone enroll request received");
            ESP_LOGI(TAG, "  Source endpoint: %d", enroll_msg->info.src_endpoint);
            ESP_LOGI(TAG, "  Zone type: 0x%04x", enroll_msg->zone_type);
            
            // Get source address based on address type
            uint16_t src_short_addr = 0;
            if (enroll_msg->info.src_address.addr_type == ESP_ZB_ZCL_ADDR_TYPE_SHORT) {
                src_short_addr = enroll_msg->info.src_address.u.short_addr;
                ESP_LOGI(TAG, "  Source address (short): 0x%04x", src_short_addr);
            } else if (enroll_msg->info.src_address.addr_type == ESP_ZB_ZCL_ADDR_TYPE_IEEE) {
                ESP_LOGI(TAG, "  Source address (IEEE): %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                         enroll_msg->info.src_address.u.ieee_addr[0], enroll_msg->info.src_address.u.ieee_addr[1],
                         enroll_msg->info.src_address.u.ieee_addr[2], enroll_msg->info.src_address.u.ieee_addr[3],
                         enroll_msg->info.src_address.u.ieee_addr[4], enroll_msg->info.src_address.u.ieee_addr[5],
                         enroll_msg->info.src_address.u.ieee_addr[6], enroll_msg->info.src_address.u.ieee_addr[7]);
                // Try to find device by IEEE address
                zigbee_device_info_t *device = find_device_by_ieee(enroll_msg->info.src_address.u.ieee_addr);
                if (device != NULL) {
                    src_short_addr = device->short_addr;
                    ESP_LOGI(TAG, "  Device found by IEEE address: short=0x%04x", src_short_addr);
                }
            }
            
            // Send enroll response
            esp_zb_zcl_ias_zone_enroll_response_cmd_t enroll_rsp;
            enroll_rsp.zcl_basic_cmd.dst_addr_u.addr_short = src_short_addr;
            enroll_rsp.zcl_basic_cmd.dst_endpoint = enroll_msg->info.src_endpoint;
            enroll_rsp.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
            enroll_rsp.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
            enroll_rsp.enroll_rsp_code = 0x00; // Success
            enroll_rsp.zone_id = 1; // Assign zone ID
            
            esp_zb_zcl_ias_zone_enroll_cmd_resp(&enroll_rsp);
            ESP_LOGI(TAG, "Sent IAS Zone enroll response with zone ID: %d", enroll_rsp.zone_id);
            break;
        }
        case ESP_ZB_CORE_REPORT_ATTR_CB_ID: {
            // Handle attribute report (for Occupancy Sensing cluster)
            const esp_zb_zcl_report_attr_message_t *report_msg = (
                const esp_zb_zcl_report_attr_message_t *)message;
            
            // Check if it's an Occupancy Sensing cluster report
            if (report_msg->cluster == ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING) {
                ESP_LOGI(TAG, "Occupancy Sensing attribute report received");
                ESP_LOGI(TAG, "  Source endpoint: %d", report_msg->src_endpoint);
                ESP_LOGI(TAG, "  Cluster: 0x%04x", report_msg->cluster);
                ESP_LOGI(TAG, "  Attribute ID: 0x%04x", report_msg->attribute.id);
                
                // Check if it's the Occupancy attribute
                if (report_msg->attribute.id == ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID) {
                    // Get occupancy value
                    bool occupied = false;
                    if (report_msg->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                        occupied = *(bool *)report_msg->attribute.data.value;
                    } else if (report_msg->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
                        occupied = *(uint8_t *)report_msg->attribute.data.value;
                    }
                    
                    ESP_LOGI(TAG, "  Occupancy: %s", occupied ? "occupied" : "not occupied");
                    
                    // Find device by IEEE address
                    zigbee_device_info_t *device = find_device_by_ieee(report_msg->src_address.u.ieee_addr);
                    if (device != NULL) {
                        ESP_LOGI(TAG, "  Device found: short=0x%04x", device->short_addr);
                        // Publish occupancy event to MQTT
                        mqtt_publish_occupancy_event(device, occupied);
                    } else {
                        ESP_LOGW(TAG, "  Device not found by IEEE address");
                    }
                }
            }
            break;
        }
        default:
            // Ignore other events
            break;
    }
    return ESP_OK;
}

static void heartbeat_timer_cb(uint8_t param)
{
    // Publish status heartbeat
    if (mqtt_connected) {
        mqtt_publish_coordinator_status();
        ESP_LOGD(TAG, "Heartbeat sent");
    }
    // Schedule next heartbeat
    esp_zb_scheduler_alarm(heartbeat_timer_cb, 0, HEARTBEAT_INTERVAL_SEC * 1000);
}

/* Note: Please select the correct console output port based on the development board in menuconfig */
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
esp_err_t esp_zb_gateway_console_init(void)
{
    esp_err_t ret = ESP_OK;
    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Enable non-blocking mode on stdin and stdout */
    fcntl(fileno(stdout), F_SETFL, O_NONBLOCK);
    fcntl(fileno(stdin), F_SETFL, O_NONBLOCK);

    usb_serial_jtag_driver_config_t usb_serial_jtag_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    usb_serial_jtag_vfs_use_driver();
    uart_vfs_dev_register();
    return ret;
}
#endif

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee bdb commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
#if CONFIG_ESP_COEX_SW_COEXIST_ENABLE
        esp_coex_wifi_i154_enable();
#endif /* CONFIG_ESP_COEX_SW_COEXIST_ENABLE */
        ESP_RETURN_ON_FALSE(wifi_init_sta() == ESP_OK, , TAG, "Failed to connect to Wi-Fi");
        ESP_RETURN_ON_FALSE(esp_wifi_set_ps(WIFI_PS_MIN_MODEM) == ESP_OK, , TAG, "Failed to set Wi-Fi minimum modem power save type");
        // Start MQTT after WiFi connected
        mqtt_app_start();
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network formation");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                esp_zb_bdb_open_network(180);
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t ieee_address;
            esp_zb_get_long_address(ieee_address);
            ESP_LOGI(TAG, "Formed network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     ieee_address[7], ieee_address[6], ieee_address[5], ieee_address[4],
                     ieee_address[3], ieee_address[2], ieee_address[1], ieee_address[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            // Publish bridge state and coordinator info to MQTT
            mqtt_publish_bridge_state("online");
            mqtt_publish_coordinator_info();
            mqtt_publish_ha_discovery();
            mqtt_publish_coordinator_status();
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGI(TAG, "Restart network formation (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Network steering started");
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
    case ESP_ZB_ZDO_SIGNAL_DEVICE_UPDATE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (dev_annce_params != NULL) {
            ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);
            
            // Check if device already exists
            zigbee_device_info_t *existing = find_device_by_ieee(dev_annce_params->ieee_addr);
            if (existing != NULL) {
                ESP_LOGI(TAG, "Device already known, updating short address");
                existing->short_addr = dev_annce_params->device_short_addr;
            } else if (zigbee_device_count < MAX_ZIGBEE_DEVICES) {
                // Add new device
                zigbee_device_info_t *new_device = &zigbee_devices[zigbee_device_count];
                new_device->short_addr = dev_annce_params->device_short_addr;
                memcpy(new_device->ieee_addr, dev_annce_params->ieee_addr, sizeof(esp_zb_ieee_addr_t));
                new_device->endpoint = 1;  // Default to endpoint 1
                new_device->device_id = 0;
                new_device->ha_discovery_done = false;
                new_device->has_onoff = true;  // Assume device has on/off capability
                new_device->has_level = false;
                new_device->has_temperature = false;
                new_device->has_humidity = false;
                new_device->has_occupancy = true;  // 假设设备具有人体感应功能
                zigbee_device_count++;
                
                ESP_LOGI(TAG, "Total ZigBee devices: %d", zigbee_device_count);
                mqtt_publish_device_announce(dev_annce_params->device_short_addr, dev_annce_params->ieee_addr);
                
                // Publish initial state for the new device
                publish_device_states();
                
                // Query device simple descriptor to detect capabilities
                // Only query endpoint 1 (most devices have only one endpoint)
                ESP_LOGI(TAG, "Querying simple descriptor for device 0x%04x endpoint 1", dev_annce_params->device_short_addr);
                query_device_simple_desc(dev_annce_params->device_short_addr, 1);
            } else {
                ESP_LOGW(TAG, "Max devices reached, cannot add more");
            }
        } else {
            ESP_LOGW(TAG, "Device announce parameters are NULL");
        }
        break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
                // Publish HA Discovery once when network is ready
                if (!ha_discovery_published && mqtt_connected) {
                    mqtt_publish_ha_discovery();
                    mqtt_publish_coordinator_status();
                    ha_discovery_published = true;
                    // Start heartbeat timer (every 60 seconds)
                    esp_zb_scheduler_alarm(heartbeat_timer_cb, 0, HEARTBEAT_INTERVAL_SEC * 1000);
                    ESP_LOGI(TAG, "Heartbeat timer started (%d sec interval)", HEARTBEAT_INTERVAL_SEC);
                }
            } else {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
        ESP_LOGI(TAG, "Production configuration is %s", err_status == ESP_OK ? "ready" : "not present");
        esp_zb_set_node_descriptor_manufacturer_code(ESP_MANUFACTURER_CODE);
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZC_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    
    // Register Zigbee core action handler for IAS Zone events
    esp_zb_core_action_handler_register(zb_core_action_handler);
    
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = ESP_ZB_GATEWAY_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_REMOTE_CONTROL_DEVICE_ID,
        .app_device_version = 0,
    };

    esp_zb_attribute_list_t *basic_cluser = esp_zb_basic_cluster_create(NULL);
    esp_zb_basic_cluster_add_attr(basic_cluser, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(basic_cluser, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ESP_MODEL_IDENTIFIER);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluser, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_ep_list_add_gateway_ep(ep_list, cluster_list, endpoint_config);
    esp_zb_device_register(ep_list);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
    vTaskDelete(NULL);
}

void app_main(void)
{
    // Set log level for custom cluster module to reduce error messages
    esp_log_level_set("ESP_ZIGBEE_ZCL_CUSTOM_CLUSTER", ESP_LOG_WARN);
    
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };

    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    ESP_ERROR_CHECK(esp_zb_gateway_console_init());
#endif
    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}
