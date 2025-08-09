/**
 * @file main.c
 * @brief ROBUST: ESP32 NimBLE Observer with OLED Display for Multi-Node Network
 *
 * This device operates in a connectionless observer mode:
 * 1. It continuously scans for advertising packets.
 * 2. It filters for packets containing a specific Manufacturer ID.
 * 3. It parses sensor data and correctly handles error codes for individual sensors.
 * 4. It stores the latest data for up to 36 nodes.
 * 5. A dedicated task cyclically displays node data, using partial refresh to prevent flicker
 * and implementing a timeout to show offline nodes.
 * This is a highly scalable, power-efficient architecture for data collection.
 */
#include <stdio.h>
#include <string.h>
#include <limits.h> // For INT16_MAX etc.
#include <math.h>   // For isnan
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "time.h"

// NimBLE 协议栈
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "host/ble_hs_id.h"

// OLED 驱动
#include "driver/i2c_master.h"
#include "ssd1306.h"

static const char *TAG = "NIMBLE_OBSERVER_OLED";

// I2C 和 OLED 配置
#define I2C_PORT_ID          I2C_NUM_0
#define I2C_SCL_PIN          GPIO_NUM_4
#define I2C_SDA_PIN          GPIO_NUM_5
#define DISPLAY_CYCLE_TIME_S 3 // 每个节点数据显示的秒数
#define NODE_TIMEOUT_S       30 // 节点被认为离线的秒数

// 目标广播数据配置 (必须与传感器节点匹配)
#define CUSTOM_MANU_ID       0x02E5 // Espressif Inc. 的官方蓝牙公司ID
#define MAX_SENSOR_NODES     36     // 支持的最大节点数量


// 传感器节点广播的数据包结构
#pragma pack(push, 1)
typedef struct {
    uint16_t manu_id;
    uint8_t  node_id;
    int16_t  temperature;
    uint16_t humidity;
    uint16_t illuminance;
} adv_sensor_data_t;
#pragma pack(pop)

// 定义错误码 (必须与传感器节点一致)
#define TEMP_ERROR_VAL      INT16_MAX
#define HUMI_ERROR_VAL      UINT16_MAX
#define LUX_ERROR_VAL       UINT16_MAX

// 用于存储所有节点数据的结构
typedef struct {
    uint8_t  node_id;
    float    temperature; // 使用 NAN 表示错误
    float    humidity;    // 使用 NAN 表示错误
    uint16_t illuminance; // 使用 LUX_ERROR_VAL 表示错误
    time_t   last_seen;
} sensor_node_status_t;

// 全局变量
static sensor_node_status_t g_sensor_nodes[MAX_SENSOR_NODES];
static int g_active_node_count = 0;

// OLED 句柄
static i2c_master_bus_handle_t g_i2c_bus_handle = NULL;
static ssd1306_handle_t g_oled_handle = NULL;

// 函数声明
static void ble_central_scan(void);

/**
 * @brief 初始化OLED屏幕
 */
static void oled_init(void) {
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_ID,
        .scl_io_num = I2C_SCL_PIN,
        .sda_io_num = I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &g_i2c_bus_handle));

    ssd1306_config_t dev_cfg = I2C_SSD1306_128x64_CONFIG_DEFAULT;
    ESP_ERROR_CHECK(ssd1306_init(g_i2c_bus_handle, &dev_cfg, &g_oled_handle));

    ESP_LOGI(TAG, "OLED Initialized");
}

/**
 * @brief 主GAP事件回调，只处理广播包
 */
static int ble_central_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_hs_adv_fields fields;

    if (event->type == BLE_GAP_EVENT_DISC) {
        if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) == 0) {
            if (fields.mfg_data != NULL && fields.mfg_data_len == sizeof(adv_sensor_data_t)) {
                adv_sensor_data_t *data = (adv_sensor_data_t *)fields.mfg_data;

                if (data->manu_id == CUSTOM_MANU_ID) {
                    uint8_t node_id = data->node_id;
                    
                    int node_index = -1;
                    for (int i = 0; i < g_active_node_count; i++) {
                        if (g_sensor_nodes[i].node_id == node_id) {
                            node_index = i;
                            break;
                        }
                    }
                    if (node_index == -1 && g_active_node_count < MAX_SENSOR_NODES) {
                        node_index = g_active_node_count;
                        g_sensor_nodes[node_index].node_id = node_id;
                        g_active_node_count++;
                    }

                    if (node_index != -1) {
                        // 检查每个传感器值是否为错误码
                        if (data->temperature == TEMP_ERROR_VAL) {
                            g_sensor_nodes[node_index].temperature = NAN;
                        } else {
                            g_sensor_nodes[node_index].temperature = (float)data->temperature / 100.0f;
                        }

                        if (data->humidity == HUMI_ERROR_VAL) {
                            g_sensor_nodes[node_index].humidity = NAN;
                        } else {
                            g_sensor_nodes[node_index].humidity = (float)data->humidity / 100.0f;
                        }
                        
                        g_sensor_nodes[node_index].illuminance = data->illuminance;
                        g_sensor_nodes[node_index].last_seen = time(NULL);

                        ESP_LOGI(TAG, "Received data from Node ID: %d", node_id);
                    }
                }
            }
        }
    }
    return 0;
}

/**
 * @brief 开始BLE扫描
 */
static void ble_central_scan(void)
{
    ESP_LOGI(TAG, "Starting BLE scan (Observer Mode)...");
    struct ble_gap_disc_params disc_params = {0};
    disc_params.passive = 1;
    disc_params.filter_duplicates = 0;
    ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params, ble_central_gap_event, NULL);
}

/**
 * @brief BLE协议栈同步完成回调
 */
static void ble_central_on_sync(void)
{
    uint8_t addr_type;
    int rc = ble_hs_id_infer_auto(0, &addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error inferring address; rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE stack synced. Starting scan.");
    ble_central_scan();
}

/**
 * @brief NimBLE主机任务
 */
void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/**
 * @brief 专用于轮播显示OLED的独立任务 (防频闪版)
 */
static void display_task(void *pvParameters)
{
    int current_node_index = 0;
    bool is_displaying_data = false;

    while (1) {
        if (g_oled_handle == NULL) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (g_active_node_count == 0) {
            if (is_displaying_data) {
                ssd1306_clear_display(g_oled_handle, false);
                is_displaying_data = false;
            }
            ssd1306_display_text(g_oled_handle, 0, "Scanning...", false);
            ssd1306_display_text(g_oled_handle, 2, "No nodes found.", false);
        } else {
            if (!is_displaying_data) {
                ssd1306_clear_display(g_oled_handle, false);
                is_displaying_data = true;
            }
            
            if (current_node_index >= g_active_node_count) {
                current_node_index = 0;
            }
            
            sensor_node_status_t *node = &g_sensor_nodes[current_node_index];
            char line_buf[32];
            
            bool is_offline = (time(NULL) - node->last_seen) > NODE_TIMEOUT_S;

            if (is_offline) {
                snprintf(line_buf, sizeof(line_buf), "#%d/%d ID:%-3d OFF", current_node_index + 1, g_active_node_count, node->node_id);
                ssd1306_display_text(g_oled_handle, 0, line_buf, false);
                ssd1306_display_text(g_oled_handle, 2, "                ", false);
                ssd1306_display_text(g_oled_handle, 4, "    OFFLINE     ", false);
                ssd1306_display_text(g_oled_handle, 6, "                ", false);
            } else {
                snprintf(line_buf, sizeof(line_buf), "#%d/%d ID:%-3d ON ", current_node_index + 1, g_active_node_count, node->node_id);
                ssd1306_display_text(g_oled_handle, 0, line_buf, false);
                
                // *** 核心修改：检查每个值是否有效，无效则显示'error' ***
                if (isnan(node->temperature)) {
                    snprintf(line_buf, sizeof(line_buf), "Temp: error     ");
                } else {
                    snprintf(line_buf, sizeof(line_buf), "Temp: %.2f C   ", node->temperature);
                }
                ssd1306_display_text(g_oled_handle, 2, line_buf, false);

                if (isnan(node->humidity)) {
                    snprintf(line_buf, sizeof(line_buf), "Humi: error     ");
                } else {
                    snprintf(line_buf, sizeof(line_buf), "Humi: %.2f %%   ", node->humidity);
                }
                ssd1306_display_text(g_oled_handle, 4, line_buf, false);

                if (node->illuminance == LUX_ERROR_VAL) {
                    snprintf(line_buf, sizeof(line_buf), "Lux:  error     ");
                } else {
                    snprintf(line_buf, sizeof(line_buf), "Lux:  %u      ", node->illuminance);
                }
                ssd1306_display_text(g_oled_handle, 6, line_buf, false);
            }
            current_node_index++;
        }
        
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_CYCLE_TIME_S * 1000));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    memset(g_sensor_nodes, 0, sizeof(g_sensor_nodes));

    oled_init();

    nimble_port_init();
    ble_hs_cfg.sync_cb = ble_central_on_sync;
    nimble_port_freertos_init(ble_host_task);

    xTaskCreate(display_task, "display_task", 2048, NULL, 5, NULL);
}
