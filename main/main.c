/** * @file main.c 
 * @brief ROBUST & LOW-POWER: ESP32 NimBLE Observer with OLED and UART
 *
 * This device operates in a power-efficient observer mode: 
 * 1. It periodically scans for advertising packets (not continuously).
 * 2. It filters for packets containing a specific Manufacturer ID. 
 * 3. It parses sensor data and correctly handles error codes for individual sensors. 
 * 4. It stores the latest data for up to 36 nodes. 
 * 5. A dedicated task cyclically displays node data, using partial refresh to prevent flicker 
 * and implementing a timeout to show offline nodes. 
 * 6. Upon receiving valid data from specific nodes (ID 1 or 2), it sends a formatted 
 * JSON string via UART, suitable for a 4G module and a cloud platform like OneNET. 
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

// OLED 驱动 
#include "driver/i2c_master.h" 
#include "ssd1306.h" 

#include "driver/uart.h" 

static const char *TAG = "LOW_POWER_OBSERVER"; 

// I2C 和 OLED 配置 
#define I2C_PORT_ID          I2C_NUM_0 
#define I2C_SCL_PIN          GPIO_NUM_4 
#define I2C_SDA_PIN          GPIO_NUM_5 
#define DISPLAY_CYCLE_TIME_S 3 
#define NODE_TIMEOUT_S       90 // 节点超时时间应大于外围节点的睡眠+广播时间

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

static i2c_master_bus_handle_t g_i2c_bus_handle = NULL; 
static ssd1306_handle_t g_oled_handle = NULL; 

static void ble_central_scan(void); 
static void send_json_via_uart(const sensor_node_status_t *node); 

// (uart_init, send_json_via_uart, oled_init, ble_central_gap_event, ble_central_on_sync, 
// ble_host_task, display_task 函数与您之前提供的代码完全相同，此处省略以保持简洁...
// ... 唯一的改动在 ble_central_scan 函数中。)

// 为了代码的完整性，这里将所有函数都列出
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

static void send_json_via_uart(const sensor_node_status_t *node) { 
    char json_buffer[256]; 
    int len = 0; 
    if (node->node_id == 1) { 
        if (!isnan(node->temperature) && !isnan(node->humidity) && node->illuminance != LUX_ERROR_VAL) { 
            len = snprintf(json_buffer, sizeof(json_buffer), 
                "{\"id\":\"%d\",\"version\":\"1.0\",\"params\":{\"humidity\":{\"value\":%.2f},\"temperature\":{\"value\":%.2f},\"illuminance\":{\"value\":%u}}}", 
                node->node_id, node->humidity, node->temperature, node->illuminance); 
        } else { 
            ESP_LOGW(TAG, "Node 1 has invalid sensor data. UART message skipped."); 
            return; 
        } 
    } 
    else if (node->node_id == 2) { 
        if (!isnan(node->temperature) && !isnan(node->humidity)) { 
            len = snprintf(json_buffer, sizeof(json_buffer), 
                "{\"id\":\"%d\",\"version\":\"1.0\",\"params\":{\"humidity2\":{\"value\":%.2f},\"temperature2\":{\"value\":%.2f}}}", 
                node->node_id, node->humidity, node->temperature); 
        } else { 
            ESP_LOGW(TAG, "Node 2 has invalid sensor data. UART message skipped."); 
            return; 
        } 
    } 
    else { 
        return; 
    } 

    if (len > 0) { 
        uart_write_bytes(UART_PORT_NUM, json_buffer, len); 
        ESP_LOGI(TAG, "Sent JSON via UART for Node ID %d: %s", node->node_id, json_buffer); 
    } 
} 

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
                        if (data->temperature == TEMP_ERROR_VAL) { 
                            current_node->temperature = NAN; 
                        } else { 
                            current_node->temperature = (float)data->temperature / 100.0f; 
                        } 
                        if (data->humidity == HUMI_ERROR_VAL) { 
                            current_node->humidity = NAN; 
                        } else { 
                            current_node->humidity = (float)data->humidity / 100.0f; 
                        } 
                        current_node->illuminance = data->illuminance; 
                        current_node->last_seen = time(NULL); 
                        ESP_LOGI(TAG, "Received data from Node ID: %d", node_id); 
                        send_json_via_uart(current_node); 
                    } 
                } 
            } 
        } 
    } 
    return 0; 
} 

/**
 * @brief 开始BLE扫描 (低功耗周期性扫描模式)
 */
static void ble_central_scan(void)
{
    ESP_LOGI(TAG, "Starting BLE scan (Periodic Low-Power Mode)...");
    struct ble_gap_disc_params disc_params = {0};

    // *** MODIFIED: Use periodic scanning to save power ***
    disc_params.passive = 1; // Passive scan does not send scan requests

    // Set scan interval to 1000ms. Unit is 0.625ms, so 1000ms = 1600 * 0.625ms
    disc_params.itvl = 1600; 

    // Set scan window to 300ms. Unit is 0.625ms, so 300ms = 480 * 0.625ms
    // This means the radio is active for 300ms every 1000ms.
    disc_params.window = 480; 
    
    disc_params.filter_duplicates = 0; // Don't filter, we want to see updates
    
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
                ssd1306_display_text(g_oled_handle, 4, "   OFFLINE      ", false); 
                ssd1306_display_text(g_oled_handle, 6, "                ", false); 
            } else { 
                snprintf(line_buf, sizeof(line_buf), "#%d/%d ID:%-3d ON ", current_node_index + 1, g_active_node_count, node->node_id); 
                ssd1306_display_text(g_oled_handle, 0, line_buf, false); 
                if (isnan(node->temperature)) { 
                    snprintf(line_buf, sizeof(line_buf), "Temp: error     "); 
                } else { 
                    snprintf(line_buf, sizeof(line_buf), "Temp: %.2f C  ", node->temperature); 
                } 
                ssd1306_display_text(g_oled_handle, 2, line_buf, false); 
                if (isnan(node->humidity)) { 
                    snprintf(line_buf, sizeof(line_buf), "Humi: error     "); 
                } else { 
                    snprintf(line_buf, sizeof(line_buf), "Humi: %.2f %%  ", node->humidity); 
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
    uart_init();  

    nimble_port_init(); 
    ble_hs_cfg.sync_cb = ble_central_on_sync; 
    nimble_port_freertos_init(ble_host_task); 

    xTaskCreate(display_task, "display_task", 2048, NULL, 5, NULL); 
}