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

/* Forward declarations */
static void mqtt_publish_coordinator_status(void);

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Connect to the AP failed");
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
        // Subscribe to command topic
        esp_mqtt_client_subscribe(event->client, MQTT_TOPIC_PREFIX "/bridge/#", 0);
        // Publish online status immediately to clear any retained offline
        mqtt_publish_coordinator_status();
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT Disconnected from broker");
        mqtt_connected = false;
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGD(TAG, "MQTT Subscribed, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT Data received: topic=%.*s, data=%.*s", 
                 event->topic_len, event->topic, event->data_len, event->data);
        // TODO: Handle incoming commands for Zigbee devices
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
    
    char topic[64];
    char payload[256];
    
    snprintf(topic, sizeof(topic), "%s/%04x", MQTT_TOPIC_PREFIX, short_addr);
    snprintf(payload, sizeof(payload), 
             "{\"ieeeAddr\":\"%02x%02x%02x%02x%02x%02x%02x%02x\",\"type\":\"Unknown\",\"networkAddress\":%d}",
             ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4],
             ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0],
             short_addr);
    
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
    char payload[512];
    
    // State
    snprintf(topic, sizeof(topic), "homeassistant/sensor/zigbee_coordinator_%s/state", device_id);
    esp_mqtt_client_publish(mqtt_client, topic, "online", 0, 1, 1);
    
    // Attributes
    snprintf(topic, sizeof(topic), "homeassistant/sensor/zigbee_coordinator_%s/attributes", device_id);
    snprintf(payload, sizeof(payload),
             "{\"pan_id\":\"0x%04hx\",\"channel\":%d,\"short_addr\":\"0x%04hx\",\"ieee\":\"%s\"}",
             esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address(), device_id);
    esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 1);
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
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);
        // Publish device announcement to MQTT
        mqtt_publish_device_announce(dev_annce_params->device_short_addr, dev_annce_params->ieee_addr);
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
