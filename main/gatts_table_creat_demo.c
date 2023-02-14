/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
*
* This demo showcases creating a GATT database using a predefined attribute table.
* It acts as a GATT server and can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server_service_table demo.
* Client demo will enable GATT server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "gatts_table_creat_demo.h"
#include "esp_gatt_common_api.h"
#include "esp_mac.h"

#include <string.h>

#define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "S-PATCH3"
#define SVC_INST_ID_0               0
#define SVC_INST_ID_1               1

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
*/
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

static uint8_t adv_config_done       = 0;

uint16_t heart_rate_handle_table_1[HRS_IDX_NB_1];
uint16_t heart_rate_handle_table_2[HRS_IDX_NB_2];
uint16_t heart_rate_handle_table_3[HRS_IDX_NB_3];

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

static esp_gatt_if_t gatts_if_noti;
static uint16_t conn_id_noti;
static int prepare_send = 0;
static int notify_data = 0;

#define CONFIG_SET_RAW_ADV_DATA
#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        /* flags */
        0x02, 0x01, 0x06, 
        0x1A, 0xFF, 0x75, 
        0x00, 0x02, 0x15, 
        0x58, 0x5C, 0xDE, 
        0x93, 0x1B, 0x01, 
        0x42, 0xCC, 0x9A, 
        0x13, 0x25, 0x00, 
        0x9B, 0xED, 0xC6, 
        0x5E, 0x53, 0x48, 0xAF, 0x22, 0xC5, 0x09,
};
static uint8_t raw_scan_rsp_data[] = {
    0x09, 0x53,0x2D, 0x50, 0x41, 0x54, 0x43, 0x48, 0x33, 0x13, 0x16, 0x0A, 0x18, 0x09, 0xC5, 0xFF, 0x5C, 0x6A, 0x00, 0xAA, 0x00, 0x34, 0x12, 0xBC, 0x9A, 0x78, 0x01, 0x00, 0x00
};

#else
static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance          = 0x00,
    .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};


/*********************************************************************/
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
/*********************************************************************/


/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* Service A */
static const uint16_t GATTS_SERVICE_UUID_TEST      = 0x180A;
static const uint16_t GATTS_CHAR_UUID_TEST_A       = 0x2a29;
static const uint16_t GATTS_CHAR_UUID_TEST_B       = 0x2a24;
static const uint16_t GATTS_CHAR_UUID_TEST_C       = 0x2a27;
static const uint16_t GATTS_CHAR_UUID_TEST_D       = 0x2a26;
static const uint16_t GATTS_CHAR_UUID_TEST_E       = 0x2a28;

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t character_user_descrip_uuid = ESP_GATT_UUID_CHAR_DESCRIPTION;
static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
//static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_write_nr            = ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t char_prop_read_notify         = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write            = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_write_nr_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_notify              = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write_write_nr   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const char char_value_A[]    = "Samsung SDS";
static const char char_value_B[]    = "S-Patch3-Cardio";
static const char char_value_C[]    = "S-Patch3-Cardio";
static const char char_value_D[]    = "V1.00";
static const char char_value_E[]    = "1.2.0";

/* Service B */
static const uint8_t GATTS_SERVICE2_UUID_TEST[]  = {   
    0xB7, 0x5C, 0x49, 0xD2, 0x04, 0xA3, 
    0x40, 0x71, 0xA0, 0xB5, 0x35, 0x85, 
    0x3E, 0xB0, 0x83, 0x07  
};
static const uint8_t GATTS_CHAR_UUID_TEST_A_2[] = {   
    0xB8, 0x5C, 0x49, 0xD2, 0x04, 0xA3, 
    0x40, 0x71, 0xA0, 0xB5, 0x35, 0x85, 
    0x3E, 0xB0, 0x83, 0x07  
};
static const uint8_t GATTS_CHAR_UUID_TEST_B_2[] = {   
    0xBA, 0x5C, 0x49, 0xD2, 0x04, 0xA3, 
    0x40, 0x71, 0xA0, 0xB5, 0x35, 0x85, 
    0x3E, 0xB0, 0x83, 0x07  
};
static const uint8_t GATTS_CHAR_UUID_TEST_C_2[] = {   
    0xB9, 0x5C, 0x49, 0xD2, 0x04, 0xA3, 
    0x40, 0x71, 0xA0, 0xB5, 0x35, 0x85, 
    0x3E, 0xB0, 0x83, 0x07  
};

uint8_t client_descrip_2[] = {0x29, 0x02};
uint8_t user_descrip_2[] = {0x29, 0x01};
static const char char_value_user_A[]    = "Server TX Data";
static const char char_value_user_B[]    = "Server RX Data";
static const char char_value_user_C[]    = "Flow Control";
static const char char_value_A_2[]    = "";

/* Service C */
static const uint16_t GATTS_SERVICE3_UUID_TEST = 0xFEF5;
static const uint8_t GATTS_CHAR_UUID_TEST_A_3[] = {   
    0x34, 0xCC, 0x54, 0xB9, 0xF9, 0x56,
    0xC6, 0x91, 0x21, 0x40, 0xA6, 0x41,
    0xA8, 0xCA, 0x82, 0x80
};
static const uint8_t GATTS_CHAR_UUID_TEST_B_3[] = {   
    0x51, 0x86, 0xF0, 0x5A, 0x34, 0x42,
    0x04, 0x88, 0x5F, 0x4B, 0xC3, 0x5E,
    0xF0, 0x49, 0x42, 0x72  
};
static const uint8_t GATTS_CHAR_UUID_TEST_C_3[] = {   
    0xD4, 0x4F, 0x33, 0xFB, 0x92, 0x7C,
    0x22, 0xA0, 0xFE, 0x45, 0xA1, 0x47,
    0x25, 0xDB, 0x53, 0x6C  
};
static const uint8_t GATTS_CHAR_UUID_TEST_D_3[] = {   
    0x31, 0xDA, 0x3F, 0x67, 0x5B, 0x85,
    0x83, 0x91, 0xD8, 0x49, 0x0C, 0x00,
    0xA3, 0xB9, 0x84, 0x9D 
};
static const uint8_t GATTS_CHAR_UUID_TEST_E_3[] = {   
    0xB2, 0x9C, 0x7B, 0xB1, 0xD0, 0x57,
    0x16, 0x91, 0xA1, 0x4C, 0x16, 0xD5,
    0xE8, 0x71, 0x78, 0x45   
};
static const uint8_t GATTS_CHAR_UUID_TEST_F_3[] = {   
    0x88, 0x5C, 0x06, 0x6A, 0xEB, 0xB3,
    0x0A, 0x99, 0xF5, 0x46, 0x8C, 0x79,
    0x94, 0xDF, 0x78, 0x5F  
};

static const uint8_t VALUE_F_3[] = {0x00};
static const uint16_t client_descript_F = 0x2902;

static uint8_t key_send_data_1[] = {
    0x55, 0xaa, 0xff, 0xff, 0x0b, 0x04, 0x00, 0x10, 0x01, 0x00, 0x81, 0x44, 0x99, 0xee, 0xee
};
static uint8_t key_send_data_2[] = {
    0x55, 0xaa, 0xff, 0xff, 0x06, 0x03, 0x00, 0x00, 0x0a, 0x00, 0x44, 0x99, 0xee, 0xee
};

/* Full Database Description - Used to add attributes into the database */

void send_data_task(void *pvParameters){

    const uint8_t data[] = {   
            0x55, 0xAA, 0xFF, 0xFF, 0x05, 0x13, 0x01, 0x11, 0x00, 0x01, 
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
            0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x04, 0x00, 0x02, 0x00, 
            0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x3D, 0x01, 0x96, 0x03, 
            0x0A, 0x06, 0x48, 0x07, 0x60, 0x07, 0x57, 0x07, 0xA1, 0x07, 
            0xD9, 0x07, 0xA8, 0x07, 0x4B, 0x07, 0x3E, 0x07, 0x95, 0x07, 
            0xE0, 0x07, 0xBF, 0x07, 0x63, 0x07, 0x49, 0x07, 0x96, 0x07, 
            0xE3, 0x07, 0xC9, 0x07, 0x6E, 0x07, 0x4F, 0x07, 0x95, 0x07,
            0xDF, 0x07, 0xC9, 0x07, 0x77, 0x07, 0x5A, 0x07, 0x97, 0x07, 
            0xDA, 0x07, 0xCB, 0x07, 0x82, 0x07, 0x5E, 0x07, 0x8E, 0x07, 
            0xD5, 0x07, 0xDB, 0x07, 0x96, 0x07, 0x5E, 0x07, 0x7C, 0x07, 
            0xCD, 0x07, 0xE9, 0x07, 0xA9, 0x07, 0x61, 0x07, 0x72, 0x07, 
            0xC8, 0x07, 0xF0, 0x07, 0xB7, 0x07, 0x6D, 0x07, 0x7B, 0x07, 
            0xCC, 0x07, 0xF3, 0x07, 0xBB, 0x07, 0x70, 0x07, 0x75, 0x07, 
            0xC1, 0x07, 0xF1, 0x07, 0xCC, 0x07, 0x87, 0x07, 0x80, 0x07, 
            0xBB, 0x07, 0xE8, 0x07, 0xCA, 0x07, 0x87, 0x07, 0x78, 0x07, 
            0xB4, 0x07, 0xEE, 0x07, 0xD8, 0x07, 0x8E, 0x07, 0x71, 0x07, 
            0xA8, 0x07, 0xE8, 0x07, 0xDB, 0x07, 0x92, 0x07, 0x70, 0x07, 
            0x9F, 0x07, 0xDE, 0x07, 0xDA, 0x07, 0x9D, 0x07, 0x7B, 0x07, 
            0x9F, 0x07, 0xD5, 0x07, 0xD9, 0x07, 0xA7, 0x07, 0x80, 0x07, 
            0x93, 0x07, 0xCB, 0x07, 0xE2, 0x07, 0xBC, 0x07, 0x89, 0x07, 
            0x8E, 0x07, 0xC6, 0x07, 0xE9, 0x07, 0xC3, 0x07, 0x82, 0x07, 
            0x81, 0x07, 0xC8, 0x07, 0xFD, 0x07, 0xD4, 0x07, 0x7F, 0x07, 
            0x74, 0x07, 0xC6, 0x07, 0x09, 0x08, 0xE0, 0x07, 0x7F, 0x07, 
            0x67, 0x07, 0xBB, 0x07, 0x0D, 0x08, 0xF0, 0x07, 0x84, 0x07, 
            0x56, 0x07, 0xA5, 0x07, 0x10, 0x08, 0x09, 0x08, 0x8F, 0x07, 
            0x3D, 0x07, 0x85, 0x07, 0x13, 0x08, 0x22, 0x01, 0x00, 0x01, 
            0x23, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x31, 0x02, 0x00, 
            0x7B, 0x56, 0x44, 0x99, 0xEE, 0xEE  };
    const uint8_t data_1[] = {
        0x55, 0xAA, 0xFF, 0xFF, 0x05, 0x13, 0x01, 0x11, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    const uint8_t data_2[] = {
        0x8E, 0x03, 0xA1, 0x03, 0xA4, 0x03, 0x8F, 0x03, 0x7F, 0x03, 0x8D, 0x03, 0xAC, 0x03, 0xB4, 0x03, 0x9D, 0x03, 0x89, 0x03
    };
    const uint8_t data_3[] = {
        0x8F, 0x03, 0x9A, 0x03, 0xA3, 0x03, 0xA0, 0x03, 0x95, 0x03, 0x8E, 0x03, 0x94, 0x03, 0x9F, 0x03, 0xA2, 0x03, 0x98, 0x03
    };
    const uint8_t data_4[] = {
        0x93, 0x03, 0x90, 0x03, 0x9A, 0x03, 0xA5, 0x03, 0xA3, 0x03, 0x97, 0x03, 0x91, 0x03, 0x99, 0x03, 0xAA, 0x03, 0xB0, 0x03
    };
    const uint8_t data_5[] = {
        0x55, 0xAA, 0xFF, 0xFF, 0x05, 0x13, 0x01, 0x11, 0x00, 0x01, 0x9C, 0x03, 0x96, 0x03, 0xA4, 0x03, 0xB4, 0x03, 0xB1, 0x03
    };
    const uint8_t data_6[] = {
        0xCE, 0x03, 0xCE, 0x03, 0xD3, 0x03, 0xD5, 0x03, 0xD4, 0x03, 0xD4, 0x03, 0xD4, 0x03, 0xD0, 0x03, 0xCB, 0x03, 0xCC, 0x03
    };
    const uint8_t data_7[] = {
        0xCB, 0x03, 0xD2, 0x03, 0xD1, 0x03, 0xC8, 0x03, 0xC3, 0x03, 0xC8, 0x03, 0xD2, 0x03, 0xD3, 0x03, 0xC9, 0x03, 0xC0, 0x03
    };
    const uint8_t data_8[] = {
        0xCC, 0x03, 0xC0, 0x03, 0xBE, 0x03, 0xC7, 0x03, 0xCF, 0x03, 0xCC, 0x03, 0xC4, 0x03, 0xC1, 0x03, 0xC7, 0x03, 0xCF, 0x03
    };
    // const uint8_t data_9[] = {
    //     55-AA-FF-FF-05-13-01-11-00-01-C7-03-CC-03-C8-03-C2-03-C2-03
    // }
    // const uint8_t data_10[] = {
        
    // }
    bool start = true;
    uint16_t len;
    const uint8_t *rx_data;
    int j = 0;
    while (start){
        if (prepare_send == 1){
            if (notify_data == 1 ){
                uint8_t notify_data[20];
                for (int i = 0; i < sizeof(notify_data); ++i)
                {
                    notify_data[i] = (i+j) % 0xff;
                }
                // esp_ble_gatts_send_indicate(gatts_if_noti, conn_id_noti, heart_rate_handle_table_2[IDX_CHAR_VAL_A_2], sizeof(notify_data), notify_data, false);
                // esp_ble_gatts_send_indicate(gatts_if_noti, conn_id_noti, heart_rate_handle_table_2[IDX_CHAR_VAL_A_2], sizeof(data_1), data_1, false);
                // vTaskDelay(10 / portTICK_PERIOD_MS);
                // esp_ble_gatts_send_indicate(gatts_if_noti, conn_id_noti, heart_rate_handle_table_2[IDX_CHAR_VAL_A_2], sizeof(data_2), data_2, false);
                // vTaskDelay(10 / portTICK_PERIOD_MS);
                // esp_ble_gatts_send_indicate(gatts_if_noti, conn_id_noti, heart_rate_handle_table_2[IDX_CHAR_VAL_A_2], sizeof(data_3), data_3, false);
                // vTaskDelay(10 / portTICK_PERIOD_MS);
                // esp_ble_gatts_send_indicate(gatts_if_noti, conn_id_noti, heart_rate_handle_table_2[IDX_CHAR_VAL_A_2], sizeof(data_4), data_4, false);
                // vTaskDelay(10 / portTICK_PERIOD_MS);
                // esp_ble_gatts_send_indicate(gatts_if_noti, conn_id_noti, heart_rate_handle_table_2[IDX_CHAR_VAL_A_2], sizeof(data_5), data_5, false);
                // vTaskDelay(10 / portTICK_PERIOD_MS);
                // esp_ble_gatts_send_indicate(gatts_if_noti, conn_id_noti, heart_rate_handle_table_2[IDX_CHAR_VAL_A_2], sizeof(data_6), data_6, false);
                // vTaskDelay(10 / portTICK_PERIOD_MS);
                // esp_ble_gatts_send_indicate(gatts_if_noti, conn_id_noti, heart_rate_handle_table_2[IDX_CHAR_VAL_A_2], sizeof(data_7), data_7, false);
                // vTaskDelay(10 / portTICK_PERIOD_MS);
                // esp_ble_gatts_send_indicate(gatts_if_noti, conn_id_noti, heart_rate_handle_table_2[IDX_CHAR_VAL_A_2], sizeof(data_8), data_8, false);
                
                // esp_ble_gatts_send_indicate(gatts_if_noti, conn_id_noti, heart_rate_handle_table_2[IDX_CHAR_VAL_A_2],
                //                                  sizeof(data), data, false);
                // esp_err_t err = esp_ble_gatts_set_attr_value(heart_rate_handle_table_2[IDX_CHAR_VAL_A_2], sizeof(data), data);
                // if (err != ESP_OK){
                //     ESP_LOGE(GATTS_TABLE_TAG, "Set Value Error");
                // }
                // esp_gatt_status_t a = esp_ble_gatts_get_attr_value(heart_rate_handle_table_2[IDX_CHAR_VAL_A_2], &len, &rx_data);
                // esp_log_buffer_hex(GATTS_TABLE_TAG, rx_data, len);
                j++;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        } else {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

esp_err_t ble_comm_p2p_start(void)
{
    static bool is_comm_p2p_started = false;
    if (!is_comm_p2p_started) {
        is_comm_p2p_started = true;
        xTaskCreate(&send_data_task, "send_data_task", 4096, NULL, 5, NULL);
    }
    return ESP_OK;
}

static const esp_gatts_attr_db_t gatt_db_A[HRS_IDX_NB_1] = {
    // Service Declaration
    [IDX_SVC_1]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t *)&GATTS_SERVICE_UUID_TEST}},

    /* Characteristic Declaration */
    [IDX_CHAR_A_1]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_A_1] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_A, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_A), (uint8_t *)char_value_A}},

    /* Characteristic Declaration */
    [IDX_CHAR_B_1]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_B_1]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_B, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_B), (uint8_t *)char_value_B}},

    /* Characteristic Declaration */
    [IDX_CHAR_C_1]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_C_1]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_C, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_C), (uint8_t *)char_value_C}},

    /* Characteristic Declaration */
    [IDX_CHAR_D_1]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_D_1]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_D, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_D), (uint8_t *)char_value_D}},

    /* Characteristic Declaration */
    [IDX_CHAR_E_1]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_E_1]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_E, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_E), (uint8_t *)char_value_E}},

};

static const esp_gatts_attr_db_t gatt_db_B[HRS_IDX_NB_2] = {
    // Service Declaration
    [IDX_SVC_2]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE2_UUID_TEST), (uint8_t *)&GATTS_SERVICE2_UUID_TEST}},

    /* Characteristic Declaration */
    [IDX_CHAR_A_2]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_notify }},

    /* Characteristic Value */
    [IDX_CHAR_VAL_A_2] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_TEST_A_2, ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_A_2), (uint8_t *)char_value_A_2}},

    /* Client Characteristic Configuration Descriptor Declaration */
    [IDX_CHAR_CFG_A_2]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Client Characteristic Configuration Descriptor Value */ 
    [IDX_CHAR_CFG_VAL_A_2] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&client_descrip_2, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_A_2), (uint8_t *)char_value_A_2}},

    /* Characteristic User Configuration Descriptor Declaration */
    [IDX_CHAR_CFG_USER_A_2]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_user_descrip_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic User Configuration Descriptor Value */ 
    [IDX_CHAR_CFG_USER_VAL_A_2] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&user_descrip_2, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_user_A), (uint8_t *)char_value_user_A}},

    /* Characteristic Declaration */
    [IDX_CHAR_B_2]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write_nr}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_B_2]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_TEST_B_2, ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_A_2), (uint8_t *)char_value_A_2}},

    /* Client Characteristic Configuration Descriptor Declaration */
    [IDX_CHAR_CFG_B_2]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Client Characteristic Configuration Descriptor Value */ 
    [IDX_CHAR_CFG_VAL_B_2] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&client_descrip_2, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_A_2), (uint8_t *)char_value_A_2}},

    /* Characteristic User Configuration Descriptor Declaration */
    [IDX_CHAR_CFG_USER_B_2]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_user_descrip_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic User Configuration Descriptor Value */ 
    [IDX_CHAR_CFG_USER_VAL_B_2] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&user_descrip_2, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_user_B), (uint8_t *)char_value_user_B}},

    /* Characteristic Declaration */
    [IDX_CHAR_C_2]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_nr_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_C_2]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_TEST_C_2, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_A_2), (uint8_t *)char_value_A_2}},
    
    /* Client Characteristic Configuration Descriptor Declaration */
    [IDX_CHAR_CFG_C_2]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Client Characteristic Configuration Descriptor Value */ 
    [IDX_CHAR_CFG_VAL_C_2] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&client_descrip_2, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_A_2), (uint8_t *)char_value_A_2}},

    /* Characteristic User Configuration Descriptor Declaration */
    [IDX_CHAR_CFG_USER_C_2]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_user_descrip_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic User Configuration Descriptor Value */ 
    [IDX_CHAR_CFG_USER_VAL_C_2] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&user_descrip_2, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_user_C), (uint8_t *)char_value_user_C}},
    
};

static const esp_gatts_attr_db_t gatt_db_C[HRS_IDX_NB_3] = {
    // Service Declaration
    [IDX_SVC_3]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE3_UUID_TEST), (uint8_t *)&GATTS_SERVICE2_UUID_TEST}},

    /* Characteristic Declaration */
    [IDX_CHAR_A_3]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write }},

    /* Characteristic Value */
    [IDX_CHAR_VAL_A_3] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_TEST_A_3, ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_A_2), (uint8_t *)char_value_A_2}},

    /* Characteristic Declaration */
    [IDX_CHAR_B_3]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write }},

    /* Characteristic Value */
    [IDX_CHAR_VAL_B_3] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_TEST_B_3, ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_A_2), (uint8_t *)char_value_A_2}},

    /* Characteristic Declaration */
    [IDX_CHAR_C_3]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_CHAR_PROP_BIT_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read }},

    /* Characteristic Value */
    [IDX_CHAR_VAL_C_3] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_TEST_C_3, ESP_GATT_CHAR_PROP_BIT_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_A_2), (uint8_t *)char_value_A_2}},
    
    /* Characteristic Declaration */
    [IDX_CHAR_D_3]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write }},

    /* Characteristic Value */
    [IDX_CHAR_VAL_D_3] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_TEST_D_3, ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_A_2), (uint8_t *)char_value_A_2}},

    /* Characteristic Declaration */
    [IDX_CHAR_E_3]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE_NR,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_write_nr}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_E_3] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_TEST_E_3, ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE_NR,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_A_2), (uint8_t *)char_value_A_2}},

    /* Characteristic Declaration */
    [IDX_CHAR_F_3]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify }},

    /* Characteristic Value */
    [IDX_CHAR_VAL_F_3] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&GATTS_CHAR_UUID_TEST_F_3, ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(VALUE_F_3), (uint8_t *)VALUE_F_3}},

    /* Client Characteristic Configuration Descriptor Declaration */
    [IDX_CHAR_CFG_F_3]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Client Characteristic Configuration Descriptor Value */ 
    [IDX_CHAR_CFG_VAL_F_3] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&client_descript_F, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_A_2), (uint8_t *)char_value_A_2}},
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    #ifdef CONFIG_SET_RAW_ADV_DATA
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #else
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #endif
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    } else {
        if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp){
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }else{
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TABLE_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
    #ifdef CONFIG_SET_RAW_ADV_DATA
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %s ", esp_err_to_name(raw_adv_ret));
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #else
            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #endif
            esp_err_t create_attr_ret1 = esp_ble_gatts_create_attr_tab(gatt_db_A, gatts_if, HRS_IDX_NB_1, SVC_INST_ID_0);
            esp_err_t create_attr_ret2 = esp_ble_gatts_create_attr_tab(gatt_db_B, gatts_if, HRS_IDX_NB_2, SVC_INST_ID_0);
            esp_err_t create_attr_ret3 = esp_ble_gatts_create_attr_tab(gatt_db_C, gatts_if, HRS_IDX_NB_3, SVC_INST_ID_0);
            if (create_attr_ret1){
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret1);
            }
            if (create_attr_ret2){
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret2);
            }
            if (create_attr_ret3){
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret3);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
       	    break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep){
                // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                
                if(memcmp(key_send_data_1, param->write.value, 15) == 0 ||
                   memcmp(key_send_data_2, param->write.value, 14) == 0 ){
                    prepare_send = 1;
                }
                
                if (heart_rate_handle_table_2[IDX_CHAR_CFG_A_2] == param->write.handle && param->write.len == 2){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify enable");
                        notify_data = 1;
                        conn_id_noti = param->write.conn_id;
                        gatts_if_noti = gatts_if;
                        if(memcmp(key_send_data_1, param->write.value, 15) == 0 ||
                            memcmp(key_send_data_2, param->write.value, 14) == 0 ){
                            uint8_t notify_data[20];
                            for (int i = 0; i < sizeof(notify_data); ++i)
                            {
                                notify_data[i] = i % 0xff;
                                esp_log_buffer_hex(GATTS_TABLE_TAG, notify_data, sizeof(notify_data));
                                //the size of notify_data[] need less than MTU size
                                esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table_2[IDX_CHAR_VAL_A_2],
                                                    sizeof(notify_data), notify_data, false);
                            }
                        }
                        // uint8_t notify_data[20];
                        // for (int i = 0; i < sizeof(notify_data); ++i)
                        // {
                        //     notify_data[i] = i % 0xff;
                        // }
                        // esp_log_buffer_hex(GATTS_TABLE_TAG, notify_data, sizeof(notify_data));
                        // //the size of notify_data[] need less than MTU size
                        // esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table_2[IDX_CHAR_VAL_A_2],
                        //                         sizeof(notify_data), notify_data, false);

                    }else if (descr_value == 0x0002){
                        ESP_LOGI(GATTS_TABLE_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i % 0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table_2[IDX_CHAR_VAL_A_2],
                                            sizeof(indicate_data), indicate_data, true);
                    }
                    else if (descr_value == 0x0000){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify/indicate disable ");
                        notify_data = 0;
                    }else{
                        ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                        esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                    }
                }
                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }else{
                /* handle prepare write */
                example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
            }
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            example_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != HRS_IDX_NB_1 && param->add_attr_tab.num_handle != HRS_IDX_NB_2 && param->add_attr_tab.num_handle != HRS_IDX_NB_3){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB_1(%d)", param->add_attr_tab.num_handle, HRS_IDX_NB_1);
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB_2(%d)", param->add_attr_tab.num_handle, HRS_IDX_NB_2);
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB_3(%d)", param->add_attr_tab.num_handle, HRS_IDX_NB_3);
            }
            else if (param->add_attr_tab.num_handle == HRS_IDX_NB_1){
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d",param->add_attr_tab.num_handle);
                memcpy(heart_rate_handle_table_1, param->add_attr_tab.handles, sizeof(heart_rate_handle_table_1));
                esp_ble_gatts_start_service(heart_rate_handle_table_1[IDX_SVC_1]);
            }
            else if (param->add_attr_tab.num_handle == HRS_IDX_NB_2) {
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d",param->add_attr_tab.num_handle);
                memcpy(heart_rate_handle_table_2, param->add_attr_tab.handles, sizeof(heart_rate_handle_table_2));
                esp_ble_gatts_start_service(heart_rate_handle_table_2[IDX_SVC_2]);
            }
            else if (param->add_attr_tab.num_handle == HRS_IDX_NB_3) {
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d",param->add_attr_tab.num_handle);
                memcpy(heart_rate_handle_table_3, param->add_attr_tab.handles, sizeof(heart_rate_handle_table_3));
                esp_ble_gatts_start_service(heart_rate_handle_table_3[IDX_SVC_3]);
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            printf("event number: %d\n", event);
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if) {
                if (heart_rate_profile_tab[idx].gatts_cb) {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void app_main(void)
{
    esp_err_t ret;
    
    /* Initialize NVS. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    ble_comm_p2p_start();
}
