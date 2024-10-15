#include <stdio.h>
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif.h"
//#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_err.h"
#include "esp_bt.h"

#include "driver/uart.h"
#include <stdint.h>
#include <string.h>
#include "esp_gap_ble_api.h"  //
#include "esp_gatt_defs.h"  //
#include "esp_gap_bt_api.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
//#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"   //


//#include "sdkconfig.h"

//#define SSID "Cedelerium"
//#define PASS "kaffemongo"
//#define SSID "STI Student"
//#define PASS "STI1924stu"

#define TAG "wifi connection"

#define SERVER_IP "213.164.197.238" //PC HOST
//#define SERVER_IP "192.168.0.155" //MAC HOST hemma
//#define SERVER_IP "172.16.218.202" // MAC HOST på skolan
#define SERVER_PORT 1349
#define UDP_PORT 1256

#define TCP_TAG "TCP_CLIENT"
#define UDP_TAG "UDP_CLIENT"
#define BIND_OPERATION 1
#define BROADCAST_OPERATION 2
#define UDP_SOCKET 1
#define TCP_SOCKET 2
#define LED_GPIO_PIN 20

#define SERVICE_UUID     0x1234
#define CHAR_SSID_UUID    0x1235
#define CHAR_PASS_UUID    0x1236
#define CHAR_IP_UUID      0x1237

#define DEVICE_NAME "ESP32_BLE_PTRX"

#define TAG_C "GAP_EVENT"
#define TAG_G "GATT_EVENT"


static uint16_t service_handle = 0;
static uint16_t char_ssid_handle = 0;
static uint16_t char_pass_handle = 0;
static uint16_t char_ip_handle  = 0;



static char ssid_value[64] = "Cedelerium";
static char pass_value[64] = "DEFAULT_PASS";
static char ip_value[64] = "DEFAULT_IP";


static int credentials_received = 0;


typedef struct {
    esp_bt_uuid_t uuid;
    esp_attr_value_t attr_value;
} characteristic_t;

static characteristic_t char_ssid;

static characteristic_t char_pass;

static characteristic_t char_ip;
#define ESP_GATT_UUID_CHAR_DESCRIPTION 0x2901 


QueueHandle_t sensorData_queue;

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void initialize_characteristics() {
    // Initialize char_ssid
    char_ssid.uuid.len = ESP_UUID_LEN_16;
    char_ssid.uuid.uuid.uuid16 = CHAR_SSID_UUID;
    char_ssid.attr_value.attr_max_len = sizeof(ssid_value);
    char_ssid.attr_value.attr_len = strlen(ssid_value);
    char_ssid.attr_value.attr_value = (uint8_t *)ssid_value;

    // Initialize char_pass
    char_pass.uuid.len = ESP_UUID_LEN_16;
    char_pass.uuid.uuid.uuid16 = CHAR_PASS_UUID;
    char_pass.attr_value.attr_max_len = sizeof(pass_value);
    char_pass.attr_value.attr_len = strlen(pass_value);
    char_pass.attr_value.attr_value = (uint8_t *)pass_value;

    // Initialize char_ip
    char_ip.uuid.len = ESP_UUID_LEN_16;
    char_ip.uuid.uuid.uuid16 = CHAR_IP_UUID;
    char_ip.attr_value.attr_max_len = sizeof(ip_value);
    char_ip.attr_value.attr_len = strlen(ip_value);
    char_ip.attr_value.attr_value = (uint8_t *)ip_value;
} 

void create_ble_service(esp_gatt_if_t gatts_if) {
    esp_err_t status;
    
    // Create the service
    esp_gatt_srvc_id_t service_id = {
        .id = {
            .uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid.uuid16 = SERVICE_UUID
            },
            .inst_id = 0
        },
        .is_primary = true,
    };

    
    status = esp_ble_gatts_create_service(gatts_if, &service_id, 10);
    if (status != ESP_OK) {
        ESP_LOGE("BLE", "Create service failed, error code = %x", status);
        return;
    }

    
}
void add_characteristics(esp_gatt_if_t gatts_if, uint16_t service_handle, characteristic_t *characteristic) {
    // Add characteristics
  
    esp_err_t status;
    esp_gatt_char_prop_t char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;

     ESP_LOGI(TAG_G, "Adding characteristic with UUID: %x, attr_value length: %d", characteristic->uuid.uuid.uuid16, characteristic->attr_value.attr_len);

    // Check if the attribute value length is valid
    if (characteristic->attr_value.attr_len > characteristic->attr_value.attr_max_len) {
        ESP_LOGE(TAG_G, "Invalid attribute value length: %d, max length: %d", characteristic->attr_value.attr_len, characteristic->attr_value.attr_max_len);
        return;
    }

    status = esp_ble_gatts_add_char(service_handle, &characteristic->uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, char_property, &characteristic->attr_value, NULL);
     if (status != ESP_OK) {
        ESP_LOGE("BLE", "Add characteristic failed, error code = %x", status);
        return;
    }
   
    
}
void setup_ble_adv_data() {
    uint8_t service_uuid[16] = {0};
    service_uuid[0] = (uint8_t)(SERVICE_UUID & 0xFF);
    service_uuid[1] = (uint8_t)((SERVICE_UUID >> 8) & 0xFF);
    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = true,  // This ensures the device name is included
        .include_txpower = true,
        .min_interval = 0x20,
        .max_interval = 0x40,
        .appearance = 0x00,
        .manufacturer_len = 0,  // No manufacturer specific data
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = sizeof(service_uuid),
        .p_service_uuid = service_uuid,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
        
    };
   
    esp_ble_gap_config_adv_data(&adv_data);
}



void add_descriptor(esp_gatt_if_t gatts_if, uint16_t service_handle, const char *description) {
    
    esp_err_t status;
    esp_bt_uuid_t descr_uuid = {
        .len = ESP_UUID_LEN_16,
        .uuid.uuid16 = ESP_GATT_UUID_CHAR_DESCRIPTION,
    };
    esp_attr_value_t descr_value = {
        .attr_max_len = strlen(description) + 1, // +1 for null terminator
        .attr_len = strlen(description)+1,
        .attr_value = (uint8_t *)description,
    };
    ESP_LOGI(TAG_G, "Adding descriptor with description: %s", description);
    status = esp_ble_gatts_add_char_descr(service_handle, &descr_uuid, ESP_GATT_PERM_READ, &descr_value, NULL);
    if (status != ESP_OK) {
        ESP_LOGE(TAG_G, "Add descriptor failed, error code = %x", status);
    }
}



void ble_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    
    switch (event) {    

        case ESP_GATTS_REG_EVT:
            ESP_LOGI("BLE", "Register event");
            create_ble_service(gatts_if);
            
            break;

        case ESP_GATTS_READ_EVT:
              ESP_LOGI(TAG_G, "ESP_GATTS_READ_EVT, handle: %d", param->read.handle);

            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;

            if (param->read.handle == char_ssid_handle) {
                ESP_LOGI(TAG_G, "Reading SSID characteristic");
                rsp.attr_value.len = strlen(ssid_value);
                memcpy(rsp.attr_value.value, ssid_value, rsp.attr_value.len);
            } else if (param->read.handle == char_pass_handle) {
                ESP_LOGI(TAG_G, "Reading Password characteristic");
                rsp.attr_value.len = strlen(pass_value);
                memcpy(rsp.attr_value.value, pass_value, rsp.attr_value.len);
            } else if (param->read.handle == char_ip_handle) {
                ESP_LOGI(TAG_G, "Reading IP Address characteristic");
                rsp.attr_value.len = strlen(ip_value);
                memcpy(rsp.attr_value.value, ip_value, rsp.attr_value.len);
            } else {
                ESP_LOGW(TAG_G, "Unknown handle: %d", param->read.handle);
                rsp.attr_value.len = 0;
            }

            esp_err_t status = esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            if (status != ESP_OK) {
                ESP_LOGE(TAG_G, "Failed to send response, error code = %x", status);
            } else {
                ESP_LOGI(TAG_G, "Response sent successfully");
            }
            break;
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            ESP_LOGI("BLE", "Added char descr event");
            break;


        case ESP_GATTS_WRITE_EVT:
            if (param->write.handle == char_ssid_handle) {            
                memset(ssid_value, 0, sizeof(ssid_value));
                memcpy(ssid_value, param->write.value, param->write.len);
                ssid_value[param->write.len] = '\0';
                
             //   printf("SSID Received: %.*s\n", param->write.len, param->write.value);
                printf("SSID Received: %s\n", ssid_value);
            } else if (param->write.handle == char_pass_handle) {
                memset(pass_value, 0, sizeof(pass_value));
                memcpy(pass_value, param->write.value, param->write.len);
                printf("Password Received: %.*s\n", param->write.len, param->write.value);
                ////////////
                credentials_received = 1;  //NOTERA, HÅRDKODAD JÄTTEDUM CONDITION, BEHÅLL INTE
                ////////////
            } else if (param->write.handle == char_ip_handle) {       
                memset(ip_value, 0, sizeof(ip_value));
                memcpy(ip_value, param->write.value, param->write.len);
                printf("IP Address Received: %.*s\n", param->write.len, param->write.value);
            }

        break;

        case ESP_GATTS_START_EVT:
            ESP_LOGI("BLE", "Start event");       
            break

        case ESP_GATTS_CONNECT_EVT   :
            ESP_LOGI("BLE", "Connected ESP");
            break;

        case ESP_GATTS_ADD_CHAR_EVT:
             
            if (param->add_char.char_uuid.uuid.uuid16 == CHAR_SSID_UUID) {
                char_ssid_handle = param->add_char.attr_handle;       
                ESP_LOGI("BLE", "Char ssid handle: %d", char_ssid_handle);
            } else if (param->add_char.char_uuid.uuid.uuid16 == CHAR_PASS_UUID) {
                char_pass_handle = param->add_char.attr_handle;
                ESP_LOGI("BLE", "Char pass handle: %d", char_pass_handle);
            } else if (param->add_char.char_uuid.uuid.uuid16 == CHAR_IP_UUID) {
                char_ip_handle = param->add_char.attr_handle;             
                ESP_LOGI("BLE", "Char ip handle: %d", char_ip_handle);
            }
            break;
  
        case ESP_GATTS_SET_ATTR_VAL_EVT  :
            ESP_LOGI("BLE", "Set attr value event");
            break;

        case ESP_GATTS_DISCONNECT_EVT:

            esp_ble_gap_start_advertising(&adv_params);
            ESP_LOGE("GATT", "Disconnect event");
            break;


        case ESP_GATTS_CREATE_EVT:  //esp_ble_gatts_create_service function calls this event
            ESP_LOGI("BLE", "Create event");
            service_handle = param->create.service_handle;
             if (service_handle == 0) {
                ESP_LOGE("BLE", "Invalid service handle");
                return;
            }
            add_characteristics(gatts_if, service_handle, &char_ssid);
       
            add_characteristics(gatts_if, service_handle, &char_pass);
        
            add_characteristics(gatts_if, service_handle, &char_ip);
            
            ESP_LOGI("BLE", "Service created, status: %d, service_handle: %d", param->create.status, param->create.service_handle);           
            ESP_ERROR_CHECK(esp_ble_gatts_start_service(service_handle));
            break;
 
        default:
         ESP_LOGW(TAG_G, "Unhandled GATTS event %d",event);
            break;
    }

}
    ////////////////

  /*
E (555) GAP_EVENT: GATT ent 7    create event
E (565) GAP_EVENT: GATT ent 12   start event (lägg till)

E (575) GAP_EVENT: GATT ent 9  add char event
E (575) GAP_EVENT: GATT ent 9
E (585) GAP_EVENT: GATT ent 9

E (16025) GAP_EVENT: GATT ent 14 connect event (kanske disconnect?)


// ESP_GATTS_REG_EVT                 = 0,       !< When register application id, the event comes */
    // ESP_GATTS_READ_EVT                = 1,       /*!< When gatt client request read operation, the event comes */
    // ESP_GATTS_WRITE_EVT               = 2,       /*!< When gatt client request write operation, the event comes */
    // ESP_GATTS_EXEC_WRITE_EVT          = 3,       /*!< When gatt client request execute write, the event comes */
    // ESP_GATTS_MTU_EVT                 = 4,       /*!< When set mtu complete, the event comes */
    // ESP_GATTS_CONF_EVT                = 5,       /*!< When receive confirm, the event comes */
    // ESP_GATTS_UNREG_EVT               = 6,       /*!< When unregister application id, the event comes */
    // ESP_GATTS_CREATE_EVT              = 7,       /*!< When create service complete, the event comes */
    // ESP_GATTS_ADD_INCL_SRVC_EVT       = 8,       /*!< When add included service complete, the event comes */
    // ESP_GATTS_ADD_CHAR_EVT            = 9,       /*!< Whenca add characteristic complete, the event comes */
    // ESP_GATTS_ADD_CHAR_DESCR_EVT      = 10,      /*!< When add descriptor complete, the event comes */
    // ESP_GATTS_DELETE_EVT              = 11,      /*!< When delete service complete, the event comes */
    // ESP_GATTS_START_EVT               = 12,      /*!< When start service complete, the event comes */
    // ESP_GATTS_STOP_EVT                = 13,      /*!< When stop service complete, the event comes */
    // ESP_GATTS_CONNECT_EVT             = 14,      /*!< When gatt client connect, the event comes */
    // ESP_GATTS_DISCONNECT_EVT          = 15,      /*!< When gatt client disconnect, the event comes */
    // ESP_GATTS_OPEN_EVT                = 16,      /*!< When connect to peer, the event comes */
    // ESP_GATTS_CANCEL_OPEN_EVT         = 17,      /*!< When disconnect from peer, the event comes */
    // ESP_GATTS_CLOSE_EVT               = 18,      /*!< When gatt server close, the event comes */
    // ESP_GATTS_LISTEN_EVT              = 19,      /*!< When gatt listen to be connected the event comes */
    // ESP_GATTS_CONGEST_EVT             = 20,      /*!< When congest happen, the event comes */
    // ESP_GATTS_RESPONSE_EVT            = 21,      /*!< When gatt send response complete, the event comes */
    // ESP_GATTS_CREAT_ATTR_TAB_EVT      = 22,      /*!< When gatt create table complete, the event comes */
    // ESP_GATTS_SET_ATTR_VAL_EVT        = 23,      /*!< When gatt set attr value complete, the event comes */
    // ESP_GATTS_SEND_SERVICE_CHANGE_EVT = 24,  
  
  
  

    //////////////

void ble_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {

   // ESP_LOGE(TAG_C, "GAP event %d",event); 

    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            // Handle advertising data set complete event
            ESP_LOGI("GAP", "Advertising data set complete");
            break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            // Handle advertising start complete event
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI("GAP", "Advertising started successfully");
            } else {
                ESP_LOGE("GAP", "Advertising failed to start, error code = %x", param->adv_start_cmpl.status);
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            // Handle advertising stop complete event
            if (param->adv_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI("GAP", "Advertising stopped successfully");
            } else {
                ESP_LOGE("GAP", "Advertising failed to stop, error code = %x", param->adv_stop_cmpl.status);
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            // Handle connection parameters update event
            ESP_LOGI("GAP", "Connection parameters updated, status: %d", param->update_conn_params.status);
            break;

        default:
            
            ESP_LOGW(TAG_C, "Unhandled GAP event %d", event);
            break;
    }
}

void init_gpio() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED_GPIO_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}
void send_mac_address(int sock) {
    uint8_t macBytes[6];
    esp_err_t ret = esp_wifi_get_mac(ESP_IF_WIFI_STA, macBytes);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get MAC address: %s", esp_err_to_name(ret));
        return;
    }

    int err = send(sock, macBytes, sizeof(macBytes), 0);
    if (err < 0) {
        ESP_LOGE(TCP_TAG, "Error occurred during sending: errno %d", errno);
    } else {
        ESP_LOGI(TCP_TAG, "MAC address sent successfully");
    }

}

void blinker(const char *data){

        if (strcmp(data, "on") == 0) {
            gpio_set_level(LED_GPIO_PIN, 1);
        } else if (strcmp(data, "off") == 0) {
            gpio_set_level(LED_GPIO_PIN, 0);
        }
}

void createTCPClient(void *pvParameters)
{
    
    char addr_str[128];

    int addr_family;
    int ip_protocol;

    while(1) //create the first loop that continuously tries to create a new socket if the connection fails and has to restart
    {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(SERVER_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

        int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TCP_TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TCP_TAG, "Socket created, connecting to %s:%d", SERVER_IP, SERVER_PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TCP_TAG, "Socket unable to connect: errno %d", errno);
            close(sock);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TCP_TAG, "Successfully connected");

            send_mac_address(sock);

            while(1)
            {

                int sensor_value;
                if(xQueueReceive(sensorData_queue, &sensor_value, pdMS_TO_TICKS(1000)) == pdTRUE) // Timeout added
                {
                    char payload[100];
                             
                  // printf("ADC Value before sending: %d\n", sensor_value);
                    snprintf(payload, sizeof(payload), "ADC Value: %d\n", sensor_value);
                    

                    int err = send(sock, payload, strlen(payload), 0);
                    if (err < 0) {
                        ESP_LOGE(TCP_TAG, "Error occurred during sending: errno %d", errno);
                        break;
                    } else {
                       
                      ESP_LOGI(TCP_TAG, "Message sent: %s", payload);
                 
                    }
                }
                else
                {
                    ESP_LOGE(TCP_TAG, "Error: No data received from queue");
                }

            }

            ESP_LOGI(TCP_TAG, "Shutting down socket and restarting...");
            close(sock);
            vTaskDelay(5000 / portTICK_PERIOD_MS);

    }
}



void adc_reader_task(void *pvParameter){

    while(1){
        int sensor_value = adc1_get_raw(ADC1_CHANNEL_2);
        xQueueSend(sensorData_queue, &sensor_value, portMAX_DELAY);
    
       // printf("ADC Value: %d\n", sensor_value);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

}
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGW(TAG, "WiFi started without me wanting to");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected. Trying to reconnect...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        char ip_str[16];
        esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "Got IP: %s", ip_str);
    }
    
}
void wifi_init_sta()
{
    
  // Initialize the TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
 
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
   


  
}
void wifi_finish_init(){

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",
        },
    };
    strncpy((char *)wifi_config.sta.ssid, ssid_value, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, pass_value, sizeof(wifi_config.sta.password));

    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_LOGI(TAG, "Setting WiFi configuration PASS %s...", wifi_config.sta.password);
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_LOGI(TAG, "Starting wifi...");
    ESP_ERROR_CHECK(esp_wifi_start());

}

void credentials_monitor_task(void* pvParamter){

    printf("Credentials monitor task started\n");

    while(1){

        if(credentials_received ==1){
           wifi_finish_init();
           vTaskDelay(pdMS_TO_TICKS(1000));

           credentials_received = 0;        
        }
        else{
                 
        }
        vTaskDelay(pdMS_TO_TICKS(100));
  


    }
}



void noiseCheck(int sensor_value)
{
    if(sensor_value < 500)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 4000); // Set duty cycle to 100%
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
    else
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); // Set duty cycle to 0%
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
}

void configure_adc(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);  // 12-bit resolution
    adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_11);  // 11dB attenuation
}

void configure_ledc_timer(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE, // Use low-speed mode
        .timer_num        = LEDC_TIMER_0,        // Use timer 0
        .duty_resolution  = LEDC_TIMER_13_BIT,   // Set duty resolution to 13 bits
        .freq_hz          = 1000,                // Set frequency to 1 kHz (adjustable)
        .clk_cfg          = LEDC_AUTO_CLK        // Auto select the clock source
    };

    ledc_timer_config(&ledc_timer);
}

void configure_ledc_channel(void)
{
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,   // Use low-speed mode
        .channel        = LEDC_CHANNEL_0,        // Use channel 0
        .timer_sel      = LEDC_TIMER_0,          // Use timer 0
        .intr_type      = LEDC_INTR_DISABLE,     // Disable interrupts
        .gpio_num       = GPIO_NUM_19,            // Set the GPIO number (adjust to your pin)
        .duty           = 0,                     // Set duty cycle to 0%
        .hpoint         = 0                      // No offset
    };

    ledc_channel_config(&ledc_channel);
}
void createTCPReciever(void *pvParameters){
    
    char addr_str[128];
    char recv_buf[64];

    int addr_family;
    int ip_protocol;

    while(1) //create the first loop that continuously tries to create a new socket if the connection fails and has to restart
    {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(SERVER_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

        int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TCP_TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TCP_TAG, "Socket created, connecting to %s:%d", SERVER_IP, SERVER_PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TCP_TAG, "Socket unable to connect: errno %d", errno);
            close(sock);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TCP_TAG, "Successfully connected");


            while(1)
            {

                
                int len = recv(sock, recv_buf, sizeof(recv_buf) - 1, 0);
                if (len < 0) {
                    ESP_LOGE(TCP_TAG, "recv failed: errno %d", errno);
                    break;
                } else if (len == 0) {
                    ESP_LOGW(TCP_TAG, "Connection closed");
                    break;
                } else {
                    send(sock, "ACK", strlen("ACK"),0);
                    recv_buf[len] = 0; // Null-terminate whatever we received and treat like a string
                  //  ESP_LOGI(TCP_TAG, "Received: %s", recv_buf);
                    esp_log_timestamp();
                    blinker(recv_buf);
                }            
            }

    }
}

int socketCreator(int socket_type) {  //detta är för UDP eller TCP // bara ändra om jag nån gång behöver IpV6
    int sock = socket(AF_INET, socket_type == UDP_SOCKET ? SOCK_DGRAM : SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return -1;
    }
    ESP_LOGI(TAG, "Socket created");
    return sock;
}
void setup_address(struct sockaddr_in *addr, const char *ip, uint16_t port) {
    memset(addr, 0, sizeof(*addr));
    addr->sin_family = AF_INET;
   if (inet_pton(AF_INET, ip, &addr->sin_addr) != 1) {
        ESP_LOGE(TAG, "Invalid IP address: %s", ip);
        return;
    }
    addr->sin_port = htons(port);

}

void socketConfig(int sock, int operation, struct sockaddr_in *dest_addr) {
    if (operation == BIND_OPERATION && dest_addr != NULL) {
        int err = bind(sock, (struct sockaddr *)dest_addr, sizeof(*dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            close(sock);
            vTaskDelete(NULL);
            return;
        }
        ESP_LOGI(TAG, "Socket bound to port %d", UDP_PORT);
    } else if (operation == BROADCAST_OPERATION) {
        int broadcast_enable = 1;
        if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable)) < 0) {
            ESP_LOGE(TAG, "Unable to set broadcast option: errno %d", errno);
            close(sock);
            vTaskDelete(NULL);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void app_main(void)
{

    initialize_characteristics();


  esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize the Bluetooth stack
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(ble_gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(ble_gap_event_handler));
    esp_ble_gatts_app_register(0);



    // Set the device name
    esp_ble_gap_set_device_name(DEVICE_NAME);

    // Configure and start BLE advertising
    setup_ble_adv_data();
    esp_ble_gap_start_advertising(&adv_params);

    ESP_LOGI("BLE", "Advertising started with device name: %s", DEVICE_NAME);
  
    wifi_init_sta();

    init_gpio();

    configure_ledc_timer();
    
   
    //gpio_set_level(LED_GPIO_PIN, 1);

    sensorData_queue = xQueueCreate(20, sizeof(int));
 
    configure_adc();
    xTaskCreate(&credentials_monitor_task, "credentials_monitor_task", 2048, NULL, 5, NULL);

    
   // xTaskCreate(&adc_reader_task, "adc_reader_task", 2048, NULL, 5, NULL); 
    //xTaskCreate(&createTCPClient, "createTCPClient", 4096, NULL, 5, NULL);
  //  xTaskCreate(&createTCPReciever, "createTCPReciever", 2048, NULL, 5, NULL);
    
    
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////