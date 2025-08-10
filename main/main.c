/**
 * @file main.c
 * @brief FINAL STABLE: ESP32 NimBLE Observer with SD Card Logging and OLED Display
 *
 * This version re-enables all functionality (Wi-Fi, SD Card, BLE, OLED)
 * with a robust, ordered startup sequence and an enhanced status display on the OLED
 * to provide clear feedback on system initialization, including error codes.
 */
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include <time.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_sntp.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

// NimBLE
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "host/ble_hs_id.h"

// OLED
#include "driver/i2c_master.h"
#include "ssd1306.h"

static const char *TAG = "CENTRAL_LOGGER";

// --- Wi-Fi & SNTP Configuration ---
#define WIFI_SSID      "luckyp"
#define WIFI_PASSWORD  "lyp19990308"

// --- SD Card SPI Configuration ---
#define SD_CARD_MOUNT_POINT "/sdcard"
#define PIN_NUM_MISO 13
#define PIN_NUM_MOSI 11
#define PIN_NUM_CLK  12
#define PIN_NUM_CS   10

// --- I2C & OLED Configuration ---
#define I2C_PORT_ID          I2C_NUM_0
#define I2C_SCL_PIN          GPIO_NUM_4
#define I2C_SDA_PIN          GPIO_NUM_5
#define DISPLAY_CYCLE_TIME_S 3
#define NODE_TIMEOUT_S       30

// --- BLE Configuration ---
#define CUSTOM_MANU_ID       0x02E5
#define MAX_SENSOR_NODES     36

// --- Data Structures ---
#pragma pack(push, 1)
typedef struct {
    uint16_t manu_id;
    uint8_t  node_id;
    int16_t  temperature;
    uint16_t humidity;
    uint16_t illuminance;
} adv_sensor_data_t;
#pragma pack(pop)

#define TEMP_ERROR_VAL      INT16_MAX
#define HUMI_ERROR_VAL      UINT16_MAX
#define LUX_ERROR_VAL       UINT16_MAX

typedef struct {
    uint8_t  node_id;
    float    temperature;
    float    humidity;
    uint16_t illuminance;
    time_t   last_seen;
} sensor_node_status_t;

// --- Global Variables & Flags for startup synchronization ---
static sensor_node_status_t g_sensor_nodes[MAX_SENSOR_NODES];
static int g_active_node_count = 0;
static bool g_sntp_initialized = false;
static bool g_sd_card_mounted = false;
static esp_err_t g_sd_card_err = ESP_OK; // *** 新增：存储SD卡错误码 ***
static QueueHandle_t g_logging_queue;

static i2c_master_bus_handle_t g_i2c_bus_handle = NULL;
static ssd1306_handle_t g_oled_handle = NULL;

static bool g_wifi_connected = false;
static bool g_ble_synced = false;


// --- Function Prototypes ---
static void ble_central_scan(void);

// --- Time Sync ---
void time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI(TAG, "Time synchronized");
    g_sntp_initialized = true;
}

static void initialize_sntp(void) {
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
}

// --- Wi-Fi Connection ---
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "Connected to Wi-Fi. Initializing SNTP...");
        g_wifi_connected = true;
        initialize_sntp();
        if (g_ble_synced) {
            ble_central_scan();
        }
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected from Wi-Fi. Retrying...");
        g_wifi_connected = false;
        esp_wifi_connect();
    }
}

static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// --- SD Card ---
static void sd_card_init(void) {
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    // *** 核心修改：不再使用ESP_ERROR_CHECK，而是手动检查错误 ***
    g_sd_card_err = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (g_sd_card_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus. Error: %s", esp_err_to_name(g_sd_card_err));
        g_sd_card_mounted = false;
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    g_sd_card_err = esp_vfs_fat_sdspi_mount(SD_CARD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (g_sd_card_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card VFS. Error: %s", esp_err_to_name(g_sd_card_err));
        g_sd_card_mounted = false;
        spi_bus_free(host.slot); // 挂载失败，释放SPI总线
    } else {
        ESP_LOGI(TAG, "SD card mounted successfully.");
        sdmmc_card_print_info(stdout, card);
        g_sd_card_mounted = true;
    }
}

// --- OLED Display ---
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

// --- BLE Logic ---
static int ble_central_gap_event(struct ble_gap_event *event, void *arg) {
    if (event->type == BLE_GAP_EVENT_DISC) {
        struct ble_hs_adv_fields fields;
        if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) == 0) {
            if (fields.mfg_data != NULL && fields.mfg_data_len == sizeof(adv_sensor_data_t)) {
                adv_sensor_data_t *data = (adv_sensor_data_t *)fields.mfg_data;
                if (data->manu_id == CUSTOM_MANU_ID) {
                    if (g_logging_queue != NULL) {
                        xQueueSend(g_logging_queue, data, 0);
                    }
                }
            }
        }
    }
    return 0;
}

static void ble_central_scan(void) {
    ESP_LOGI(TAG, "Starting BLE scan (Observer Mode)...");
    struct ble_gap_disc_params disc_params = {0};
    disc_params.passive = 1;
    disc_params.filter_duplicates = 0;
    ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params, ble_central_gap_event, NULL);
}

static void ble_central_on_sync(void) {
    uint8_t addr_type;
    int rc = ble_hs_id_infer_auto(0, &addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error inferring address; rc=%d", rc);
        return;
    }
    g_ble_synced = true;
    if (g_wifi_connected) {
        ble_central_scan();
    }
}

void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// --- Tasks ---
static void display_task(void *pvParameters) {
    int current_node_index = 0;
    bool all_systems_go = false;

    while (1) {
        if (g_oled_handle == NULL) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        all_systems_go = g_sd_card_mounted && g_sntp_initialized && g_wifi_connected;

        ssd1306_clear_display(g_oled_handle, false);

        if (!all_systems_go) {
            // 显示系统自检状态
            ssd1306_display_text(g_oled_handle, 0, "System Status:", false);
            
            char status_buf[32];
            // *** 核心修改：显示具体的错误码 ***
            if (g_sd_card_mounted) {
                snprintf(status_buf, sizeof(status_buf), "SD Card: OK");
            } else {
                snprintf(status_buf, sizeof(status_buf), "SD Card: FAIL(%d)", g_sd_card_err);
            }
            ssd1306_display_text(g_oled_handle, 2, status_buf, false);

            snprintf(status_buf, sizeof(status_buf), "Wi-Fi:   %s", g_wifi_connected ? "OK" : "...");
            ssd1306_display_text(g_oled_handle, 4, status_buf, false);

            snprintf(status_buf, sizeof(status_buf), "Time:    %s", g_sntp_initialized ? "OK" : "...");
            ssd1306_display_text(g_oled_handle, 6, status_buf, false);
            
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // --- 所有系统就绪，显示节点数据 ---
        if (g_active_node_count == 0) {
            ssd1306_display_text(g_oled_handle, 0, "Scanning...", false);
            ssd1306_display_text(g_oled_handle, 2, "No nodes found.", false);
        } else {
            if (current_node_index >= g_active_node_count) current_node_index = 0;
            
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
                
                if (isnan(node->temperature)) snprintf(line_buf, sizeof(line_buf), "Temp: error     ");
                else snprintf(line_buf, sizeof(line_buf), "Temp: %.2f C   ", node->temperature);
                ssd1306_display_text(g_oled_handle, 2, line_buf, false);

                if (isnan(node->humidity)) snprintf(line_buf, sizeof(line_buf), "Humi: error     ");
                else snprintf(line_buf, sizeof(line_buf), "Humi: %.2f %%   ", node->humidity);
                ssd1306_display_text(g_oled_handle, 4, line_buf, false);

                if (node->illuminance == LUX_ERROR_VAL) snprintf(line_buf, sizeof(line_buf), "Lux:  error     ");
                else snprintf(line_buf, sizeof(line_buf), "Lux:  %u      ", node->illuminance);
                ssd1306_display_text(g_oled_handle, 6, line_buf, false);
            }
            current_node_index++;
        }
        
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_CYCLE_TIME_S * 1000));
    }
}

static void logging_task(void *pvParameters) {
    adv_sensor_data_t received_data;

    while (1) {
        if (xQueueReceive(g_logging_queue, &received_data, portMAX_DELAY)) {
            // Update the global state for OLED display
            uint8_t node_id = received_data.node_id;
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
                if (received_data.temperature == TEMP_ERROR_VAL) g_sensor_nodes[node_index].temperature = NAN;
                else g_sensor_nodes[node_index].temperature = (float)received_data.temperature / 100.0f;

                if (received_data.humidity == HUMI_ERROR_VAL) g_sensor_nodes[node_index].humidity = NAN;
                else g_sensor_nodes[node_index].humidity = (float)received_data.humidity / 100.0f;
                
                g_sensor_nodes[node_index].illuminance = received_data.illuminance;
                g_sensor_nodes[node_index].last_seen = time(NULL);
            }

            // --- Perform SD Card Logging ---
            if (!g_sd_card_mounted || !g_sntp_initialized) {
                ESP_LOGW(TAG, "Skipping log write: SD mounted: %d, Time synced: %d", g_sd_card_mounted, g_sntp_initialized);
                continue;
            }

            char filepath[32];
            snprintf(filepath, sizeof(filepath), SD_CARD_MOUNT_POINT "/node_%d.csv", received_data.node_id);

            struct stat st;
            bool file_exists = (stat(filepath, &st) == 0);

            FILE *f = fopen(filepath, "a");
            if (f == NULL) {
                ESP_LOGE(TAG, "Failed to open file for writing: %s", filepath);
                continue;
            }

            if (!file_exists) {
                fprintf(f, "Timestamp,Temperature,Humidity,Illuminance\n");
                ESP_LOGI(TAG, "Created new log file and wrote header: %s", filepath);
            }

            time_t now = time(NULL);
            struct tm timeinfo = {0};
            localtime_r(&now, &timeinfo);
            char time_buf[64];
            strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);

            fprintf(f, "%s,%.2f,%.2f,%u\n",
                    time_buf,
                    g_sensor_nodes[node_index].temperature,
                    g_sensor_nodes[node_index].humidity,
                    g_sensor_nodes[node_index].illuminance == LUX_ERROR_VAL ? 0 : g_sensor_nodes[node_index].illuminance);
            
            fclose(f);
        }
    }
}


// --- Main ---
void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    
    g_logging_queue = xQueueCreate(10, sizeof(adv_sensor_data_t));

    oled_init();
    sd_card_init();
    wifi_init();

    nimble_port_init();
    ble_hs_cfg.sync_cb = ble_central_on_sync;
    nimble_port_freertos_init(ble_host_task);

    xTaskCreate(display_task, "display_task", 4096, NULL, 5, NULL);
    xTaskCreate(logging_task, "logging_task", 4096, NULL, 4, NULL);
}
