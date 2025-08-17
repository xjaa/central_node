/** * @file main.c 
 * @brief ROBUST & LOW-POWER: ESP32 NimBLE Observer as a Headless Gateway (Stage 2.4 - Final Robust Fix)
 *
 * This device operates in a power-efficient, headless gateway mode: 
 * 1. It uses a reliable periodic scanning mode ("scan 3s, rest 2s") to save power.
 * 2. It filters for packets containing a specific Manufacturer ID. 
 * 3. It now correctly handles bursts of packets where the first few might be invalid, ensuring a valid packet is found and processed.
 * 4. The time-based duplicate filter is now only updated AFTER a packet is confirmed to be valid.
 * 5. It dynamically generates JSON property names based on the Node ID for scalability.
 */ 
#include <stdio.h> 
#include <string.h> 
#include <limits.h>
#include <math.h>
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

// UART 驱动
#include "driver/uart.h"
#include "driver/gpio.h" 

static const char *TAG = "HEADLESS_GATEWAY_S2_4"; 

// 配置
#define NODE_TIMEOUT_S       (31 * 60) // 节点超时时间应大于外围节点的睡眠周期 (30分钟)

// UART 配置  
#define UART_PORT_NUM    UART_NUM_1 
#define UART_TXD_PIN     GPIO_NUM_17
#define UART_RXD_PIN     GPIO_NUM_18
#define UART_BAUD_RATE   115200 
#define UART_BUF_SIZE    1024 

// 目标广播数据配置
#define CUSTOM_MANU_ID       0x02E5 
#define MAX_SENSOR_NODES     36 

#pragma pack(push, 1) 
typedef struct { 
    uint16_t manu_id; 
    uint8_t  node_id; 
    int16_t  temperature; 
    uint16_t humidity; 
    uint16_t illuminance; 
} adv_sensor_data_t; 
#pragma pack(pop) 

#define TEMP_ERROR_VAL   INT16_MAX 
#define HUMI_ERROR_VAL   UINT16_MAX 
#define LUX_ERROR_VAL    UINT16_MAX 

typedef struct { 
    uint8_t  node_id; 
    float    temperature; 
    float    humidity;
    uint16_t illuminance; 
    time_t   last_seen; 
} sensor_node_status_t; 

static sensor_node_status_t g_sensor_nodes[MAX_SENSOR_NODES]; 
static int g_active_node_count = 0; 

// 函数声明
static void ble_central_scan(void); 
static void send_json_via_uart(const sensor_node_status_t *node); 

static void uart_init(void) { 
    uart_config_t uart_config = { 
        .baud_rate = UART_BAUD_RATE, 
        .data_bits = UART_DATA_8_BITS, 
        .parity    = UART_PARITY_DISABLE, 
        .stop_bits = UART_STOP_BITS_1, 
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, 
        .source_clk = UART_SCLK_DEFAULT, 
    }; 
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0)); 
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config)); 
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TXD_PIN, UART_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); 
    ESP_LOGI(TAG, "UART Initialized on port %d", UART_PORT_NUM); 
} 

/**
 * @brief 根据节点ID动态生成JSON并发送 (假设数据已预先验证)
 */
static void send_json_via_uart(const sensor_node_status_t *node) { 
    char json_buffer[256];
    int len = 0;

    if (node->node_id == 1) {
        len = snprintf(json_buffer, sizeof(json_buffer), 
            "{\"id\":\"%d\",\"version\":\"1.0\",\"params\":{\"humidity\":{\"value\":%.2f},\"temperature\":{\"value\":%.2f},\"illuminance\":{\"value\":%u}}}", 
            node->node_id, node->humidity, node->temperature, node->illuminance);
    } else {
        len = snprintf(json_buffer, sizeof(json_buffer), 
            "{\"id\":\"%d\",\"version\":\"1.0\",\"params\":{\"humidity%d\":{\"value\":%.2f},\"temperature%d\":{\"value\":%.2f},\"illuminance%d\":{\"value\":%u}}}", 
            node->node_id, 
            node->node_id, node->humidity, 
            node->node_id, node->temperature, 
            node->node_id, node->illuminance);
    }

    if (len > 0 && len < sizeof(json_buffer)) {
        uart_write_bytes(UART_PORT_NUM, json_buffer, len);
        ESP_LOGI(TAG, "Sent JSON via UART for Node ID %d: %s", node->node_id, json_buffer);
    } else if (len >= sizeof(json_buffer)) {
        ESP_LOGE(TAG, "JSON buffer overflow for Node ID %d.", node->node_id);
    }
}

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
                        sensor_node_status_t* current_node = &g_sensor_nodes[node_index]; 
                        
                        // *** 健壮的重复数据包过滤器 ***
                        time_t now = time(NULL);
                        if (current_node->last_seen != 0 && (now - current_node->last_seen) < 60) {
                            ESP_LOGD(TAG, "Received duplicate broadcast from Node ID: %d. Ignoring.", node_id);
                            return 0;
                        }
                        
                        // --- 解析数据到临时变量 ---
                        float temp_val = (data->temperature == TEMP_ERROR_VAL) ? NAN : ((float)data->temperature / 100.0f);
                        float humi_val = (data->humidity == HUMI_ERROR_VAL) ? NAN : ((float)data->humidity / 100.0f);
                        uint16_t lux_val = data->illuminance;

                        // *** 新的、更智能的数据有效性检查 ***
                        if (isnan(temp_val) || isnan(humi_val) || lux_val == LUX_ERROR_VAL) {
                            ESP_LOGW(TAG, "Node %d sent an invalid packet. Waiting for a valid one in this burst.", node_id);
                            // 关键：不更新 last_seen，也不上报，直接返回，给后续的包一个机会
                            return 0;
                        }

                        // --- 如果代码能执行到这里，说明数据是“新的”且“完全有效”的 ---
                        current_node->temperature = temp_val;
                        current_node->humidity = humi_val;
                        current_node->illuminance = lux_val;
                        current_node->last_seen = now; // 只有在此时，才更新时间戳！
                        
                        ESP_LOGI(TAG, "Received NEW VALID data from Node ID: %d. Forwarding to UART.", node_id); 
                        send_json_via_uart(current_node); 
                    } 
                } 
            } 
        } 
    } 
    return 0; 
} 

/**
 * @brief 开始BLE扫描 (可靠的低功耗周期性扫描模式)
 */
static void ble_central_scan(void)
{
    ESP_LOGI(TAG, "Starting BLE scan (Reliable Low-Power Mode)...");
    struct ble_gap_disc_params disc_params = {0};

    disc_params.passive = 1;
    disc_params.itvl = 8000; 
    disc_params.window = 4800; 
    disc_params.filter_duplicates = 0;
    
    ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params, ble_central_gap_event, NULL);
}

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

void ble_host_task(void *param) 
{ 
    nimble_port_run(); 
    nimble_port_freertos_deinit(); 
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

    uart_init();  

    nimble_port_init(); 
    ble_hs_cfg.sync_cb = ble_central_on_sync; 
    nimble_port_freertos_init(ble_host_task); 

    ESP_LOGI(TAG, "Headless gateway started. Listening for BLE advertisements...");
}
