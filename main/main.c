#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include <string.h>
#include <netdb.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include "sdkconfig.h"
#include "sys/socket.h"
//#include "esp_event.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include <inttypes.h>
#include "nvs_flash.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
//#include "esp_gatT_common_api.h"
#include "esp_gatt_defs.h" 
#include "esp_gatts_api.h"
#include "esp_gap_bt_api.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_defs.h"
#include "esp_err.h"
#include <inttypes.h>

#define DEVICE_NAME "ESP32_BLE"

#define TAG "Wifi Station"
#define WIFI_SSID "AuPx-Wifi-2.4"
#define WIFI_PASS "AuPx-HIF"
#define SERVER_IP "192.168.10.74"
#define PORT 1256


#define TCP_TAG "TCP_CLIENT"
#define BINDOPERATION 1
#define BROADCASTOPERATION 2
#define TCP_SOCKET 2

#define LED_PIN 22 //Måste ändras sen

#define SERVICE_UUID 0x1222
#define CHAR_SSID_UUID 0x1223
#define CHAR_PASS_UUID 0x1224
#define CHAR_IP_UUID 0x1225

#define GAP "GAP EVENT"
#define GATT "GATT EVENT"
#define payload "Message from ESP32"

#define MAX_CHAR_LEN 128

#define ESP_GATT_UUID_CHAR_DESCRIPTION 0x2901


static uint16_t service_handle = 0;
static uint16_t char_ssid_handle = 0;
static uint16_t char_pass_handle = 0;
static uint16_t char_ip_handle = 0;


static char ssid[64] = "Default SSID";
static char pass[64] = "Default Password";
static char ip[64] = "Default IP";

static int credentials_received = 0;


typedef struct 
{
    esp_bt_uuid_t uuid;
    esp_attr_value_t attr_value;
}characteristic_t;

static characteristic_t char_ssid;
static characteristic_t char_pass;
static characteristic_t char_ip;


static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/*
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
     .min_interval = 0x20,  
    .max_interval = 0x40,  
    .appearance = 0x00,
    .manufacturer_len = 0,  
    .p_manufacturer_data = NULL,  
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),

};
*/
void init_characteristics()
{

    char_ssid.uuid.len = ESP_UUID_LEN_16;
    char_ssid.uuid.uuid.uuid16 = CHAR_SSID_UUID;
    char_ssid.attr_value.attr_max_len = sizeof(ssid);
    char_ssid.attr_value.attr_len = strlen(ssid);
    char_ssid.attr_value.attr_value = (uint8_t *)ssid;

    char_pass.uuid.len = ESP_UUID_LEN_16;
    char_pass.uuid.uuid.uuid16 = CHAR_PASS_UUID;
    char_pass.attr_value.attr_max_len = sizeof(pass);
    char_pass.attr_value.attr_len = strlen(pass);
    char_pass.attr_value.attr_value = (uint8_t *)pass;

    char_ip.uuid.len = ESP_UUID_LEN_16; 
    char_ip.uuid.uuid.uuid16 = CHAR_IP_UUID;
    char_ip.attr_value.attr_max_len = sizeof(ip);  
    char_ip.attr_value.attr_len = strlen(ip);
    char_ip.attr_value.attr_value = (uint8_t *)ip;

}

void create_ble_service(esp_gatt_if_t gatts_if)
{

    esp_err_t res;

    esp_gatt_srvc_id_t service_id = {
        .id = {
            .uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {
                    .uuid16 = SERVICE_UUID,
                },
            },
            .inst_id = 0,
        },
        .is_primary = true,
    };

    res = esp_ble_gatts_create_service(gatts_if, &service_id, 10);
    if (res != ESP_OK)
    {
        ESP_LOGE(GATT, "Failed to create service, error code %d", res);
        return;
    }
}

void add_characteristics(esp_gatt_if_t gatts_if, uint16_t service_handle, characteristic_t *characteristic)
{

    esp_err_t res;
    esp_gatt_char_prop_t char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
    ESP_LOGI(GATT, "Adding characteristic with UUID %x, attr_value length: %d", characteristic->uuid.uuid.uuid16, characteristic->attr_value.attr_len);

    //Idk if implement this is necessary
    if (characteristic->attr_value.attr_len > characteristic->attr_value.attr_max_len)
    {
        ESP_LOGE(GATT, "Attribute length exceeds maximum length, MAX LENGTH: %d, ACTUAL LENGTH: %d", characteristic->attr_value.attr_max_len, characteristic->attr_value.attr_len);
        return;
    }
    
    res = esp_ble_gatts_add_char(service_handle, &characteristic->uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, char_property, &characteristic->attr_value, NULL);
    if (res != ESP_OK)
    {
        ESP_LOGE(DEVICE_NAME, "Failed to add characteristic, error code %x", res);
        return;
    }
    
}


void setup_ble_adv_data()
{

    uint8_t service_uuid [16] = {0};
    service_uuid[0] = (uint8_t) (SERVICE_UUID & 0xFF);
    service_uuid[1] = (uint8_t) ((SERVICE_UUID  >> 8) & 0xFF);

   
    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = false,
        .min_interval = 0x20,  
        .max_interval = 0x40,  
        .appearance = 0x00,
        .manufacturer_len = 0,  
        .p_manufacturer_data = NULL,  
        .service_data_len = 0,
         .p_service_data = NULL,
        .service_uuid_len = sizeof(service_uuid),
        .p_service_uuid = service_uuid,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),

    };
    
    esp_ble_gap_config_adv_data(&adv_data);
}

void add_descriptor(esp_gatt_if_t gatts_if, uint16_t service_handle, const char *description)
{

    esp_err_t res;
    esp_bt_uuid_t descr_uuid = {
        .len = ESP_UUID_LEN_16,
        .uuid = {
            .uuid16 = ESP_GATT_UUID_CHAR_DESCRIPTION,
        },
    };
    
    esp_attr_value_t descr_value = {
        .attr_max_len = strlen(description) + 1, // +1 for null terminator
        .attr_len = strlen(description)+ 1,
        .attr_value = (uint8_t *)description,
    };  

    ESP_LOGI(GATT, "Adding descriptor with description: %s", description);
    res = esp_ble_gatts_add_char_descr(service_handle, &descr_uuid, ESP_GATT_PERM_READ, &descr_value, NULL);
    if (res != ESP_OK)
    {
        ESP_LOGE(GATT, "Failed to add descriptor, error code %x", res);
    }
}

void ble_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    switch(event)
    {

        case ESP_GATTS_REG_EVT:
            ESP_LOGI(DEVICE_NAME, "Register event");
            create_ble_service(gatts_if);
            break;
            
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(DEVICE_NAME, "ESP_GATTS_READ_EVT, handle: %d", param->read.handle);

            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            
            if(param->read.handle == char_ssid_handle)
            {
                ESP_LOGI(DEVICE_NAME, "Reading SSID characteristic");
                rsp.attr_value.len = strlen(ssid);
                memcpy(rsp.attr_value.value, ssid, rsp.attr_value.len);
            }
            else if (param->read.handle == char_pass_handle)
            {
                ESP_LOGI(DEVICE_NAME, "Reading password characteristic");
                rsp.attr_value.len = strlen(pass);
                memcpy(rsp.attr_value.value, pass, rsp.attr_value.len);
            }
            else if (param->read.handle == char_ip_handle)
            {
                ESP_LOGI(DEVICE_NAME, "Reading IP characteristic");
                rsp.attr_value.len = strlen(ip);
                memcpy(rsp.attr_value.value, ip, rsp.attr_value.len);
            }
            else
            {
                ESP_LOGE(DEVICE_NAME, "Unknown handle: %d", param->read.handle);
                rsp.attr_value.len = 0;
            }

            esp_err_t res = esp_ble_gatts_send_response(gatts_if, 
                    param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            if (res != ESP_OK)
            {
                ESP_LOGE(DEVICE_NAME, "Failed to send response, error code %x", res);
            }
            else
            {
                ESP_LOGI(DEVICE_NAME, "Response sent successfully");
            }
            break;
        
        case ESP_GATTS_WRITE_EVT:

            ESP_LOGI(DEVICE_NAME, "Write event, handle: %d, value len: %d, value :", param->write.handle, param->write.len);
            esp_log_buffer_hex(DEVICE_NAME, param->write.value, param->write.len); 

            if(param->write.len > MAX_CHAR_LEN)
            {
                ESP_LOGE(DEVICE_NAME, "Data exceeds maximum length");
                return;
            }
            else
            {
                ESP_LOGI(DEVICE_NAME, "Value: %s", param->write.value);
            }

            if(param->write.handle == char_ssid_handle)
            {
                memset(ssid, 0, sizeof(ssid));
                memcpy(ssid, param->write.value, param->write.len);
                ssid[param->write.len] = '\0';
                
                //ESP_LOGI(DEVICE_NAME, "SSID: %s", ssid);
                printf("SSID recv: %s\n", ssid);
            }
            else if (param->write.handle == char_pass_handle)
            {
                memset(pass, 0, sizeof(pass));
                memcpy(pass, param->write.value, param->write.len);
                pass[param->write.len] = '\0';

                //ESP_LOGI(DEVICE_NAME, "Password: %s", pass);
                printf("Password recv: %.*s\n", param->write.len, param->write.value);

                credentials_received = 1;
            }
            else if (param->write.handle == char_ip_handle)
            {
                memset(ip, 0, sizeof(ip));
                memcpy(ip, param->write.value, param->write.len);
                ip[param->write.len] = '\0';
                //ESP_LOGI(DEVICE_NAME, "IP: %s", ip);
                printf("IP recv: %.*s\n", param->write.len, param->write.value);
            }
            
        break;

        case ESP_GATTS_ADD_CHAR_EVT:
            if(param->add_char.char_uuid.uuid.uuid16 == CHAR_SSID_UUID)
            {
                char_ssid_handle = param->add_char.attr_handle;
                ESP_LOGI(DEVICE_NAME, "Added SSID characteristic: %d", char_ssid_handle);
            }
            else if (param->add_char.char_uuid.uuid.uuid16 == CHAR_PASS_UUID)
            {
                char_pass_handle = param->add_char.attr_handle;
                ESP_LOGI(DEVICE_NAME, "Added password characteristic: %d", char_pass_handle);
            }
            else if (param->add_char.char_uuid.uuid.uuid16 == CHAR_IP_UUID)
            {
                char_ip_handle = param->add_char.attr_handle;
                ESP_LOGI(DEVICE_NAME, "Added IP characteristic: %d", char_ip_handle);
            }
        break;

        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            ESP_LOGI(DEVICE_NAME, "Added characteristic descriptor event");	
            break;

        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(DEVICE_NAME, "Create event");
            service_handle = param->create.service_handle;
            if(service_handle == 0)
            {
                ESP_LOGE(DEVICE_NAME, "Failed to create service");
                return;
            }
            
            add_characteristics(gatts_if, service_handle, &char_ssid);
            add_characteristics(gatts_if, service_handle, &char_pass);
            add_characteristics(gatts_if, service_handle, &char_ip);

            ESP_LOGI(DEVICE_NAME, "Service created with handle: %d and status: %d", param->create.service_handle, param->create.status);
            ESP_ERROR_CHECK(esp_ble_gatts_start_service(service_handle));    
        
        break;

        case ESP_GATTS_START_EVT:
            ESP_LOGI(DEVICE_NAME, "Start event");
            break;
        
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(DEVICE_NAME, "Connect event");
            break;
        
        case ESP_GATTS_DISCONNECT_EVT:
            
            ESP_LOGE(DEVICE_NAME, "Disconnect event");
            esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
            if (ret)
            {
                ESP_LOGE(DEVICE_NAME, "Failed to start advertising, error code %x", ret);
            }
            else
            {
                ESP_LOGI(DEVICE_NAME, "Advertising started");
            }
            break;
        
        case ESP_GATTS_RESPONSE_EVT:
            ESP_LOGI(DEVICE_NAME, "Response event");
            break;
        
        case ESP_GATTS_SET_ATTR_VAL_EVT:
            ESP_LOGI(DEVICE_NAME, "Set attribute value event");
            break;

        default:
            ESP_LOGI(DEVICE_NAME, "Events %d not handled", event);
            break;

    }
}

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch(event)
    {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(GAP, "Advertising data set complete");
            break;
        
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if(param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(GAP, "Advertising start failed, error code %d", param->adv_start_cmpl.status);
            }
            else
            {
                ESP_LOGI(GAP, "Advertising start success");
            }
         break;
        
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if(param->adv_stop_cmpl.status == ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGI(GAP, "Advertising stop success");
            }
            else
            {
                ESP_LOGE(GAP, "Advertising stop failed, error code %d", param->adv_stop_cmpl.status);
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GAP, "Update connection parameters event");
            break;

        default:
            ESP_LOGI(GAP, "Event %d not handled", event);
            break;
    }
}       

void credentials_received_task()
{   
    printf("Credentials received\n");

    while(1)
    {
        if (credentials_received == 1)
        {
            //wifi_finish_init();
            vTaskDelay(pdMS_TO_TICKS(1000));
            credentials_received = 0;
        }
        else
        {
            printf("Waiting for credentials\n");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
}

//oid init_ble();
void init_led();
void turn_on_led();
void turn_off_led();
void tcp_client(void *pvParameters);
int read_photo_sensor();
void wifi_connect();


void app_main(void)
{

    init_characteristics();

    init_led();

    // Initilize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_connect(); 
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(ble_gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    esp_ble_gatts_app_register(0);
    
    esp_ble_gap_set_device_name(DEVICE_NAME);

    setup_ble_adv_data();
    esp_ble_gap_start_advertising(&adv_params);
    
    ESP_LOGI(DEVICE_NAME, "BLE init complete");
    //init_ble();


    //xTaskCreate(tcp_client, "tcp_client", 4096, NULL, 5, NULL);

    

}

void tcp_client(void *pvParameters)
{

    char rx_buffer[20];
    char tx_buffer[100];

    uint8_t mac_addr[6];
    esp_read_mac(mac_addr, ESP_MAC_WIFI_STA);

    char mac_str[18];
    sprintf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    

    char host_ip[] = SERVER_IP;
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;


    struct sockaddr_in dest_addr;
    {
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = addr_family;
        dest_addr.sin_port = htons(PORT);
    };
    

    while (1)
    {

        // Skapa socket
        int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0){

            ESP_LOGE(TCP_TAG, "Unable to create socket: errno %d", errno);
            break;

        }

        ESP_LOGI(TCP_TAG, "Socket created, connecting to %s:%d", host_ip, PORT);

        // Anslut till server
        
        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if(err != 0)
        {
            ESP_LOGE(TCP_TAG, "Socket unable to connect: errno %d", errno);
            close(sock);
            break;
        }

        
        ESP_LOGI(TCP_TAG, "Successfully connected");

        
        while(1)
        {
            // Read sensor value
            int sensor_value = read_photo_sensor();
            sprintf(tx_buffer, "Sensor value: %d", sensor_value);

            // Skicka data
            int err = send(sock, payload, strlen(payload), 0);
            if (err < 0)
            {
                ESP_LOGE(TCP_TAG, "Error occurred during sending: errno %d", errno);
                break;
            }

            // Ta emot data
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len < 0) {

                ESP_LOGE(TCP_TAG, "recv failed: errno %d", errno);
                break;

            }

            else {
                
                rx_buffer[len] = 0; // Null-terminera mottaget meddelande
                ESP_LOGI(TCP_TAG, "Received %d bytes: %s", len, rx_buffer);
                
                //ESP_LOGI(TAG2, "%s", rx_buffer);
                
                if(strcmp(rx_buffer, "TURN_ON_LED") == 0)
                {
                  //Code to turn on LED   
                }

                if (strcmp(rx_buffer, "TURN_OFF_LED") == 0)
                {
                  //Code to turn off LED
                }

            }



        }

        if (sock != -1) {
            ESP_LOGE(TCP_TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
        
    }
    
    vTaskDelete(NULL);

}

void wifi_connect() 
{

    // Initiera nätverksgränssnitt
    ESP_ERROR_CHECK(esp_netif_init());
    // Skapa händelseloop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // Initialisera WiFi med standardkonfiguration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Skapa standard WiFi STA gränssnitt
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    if (netif == NULL) {
        ESP_LOGE(TAG, "Failed to create default WiFi STA interface");
        return;
    }

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());


    ESP_LOGI(TAG, "Connecting to WiFi...");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s", WIFI_SSID, WIFI_PASS);
    ESP_ERROR_CHECK(esp_wifi_connect());


    // Vänta på att få en IP-adress 
    // Osäker om jag behöver denna delen
    esp_netif_ip_info_t ip_info;
    while (true) {
        esp_netif_get_ip_info(netif, &ip_info);
        if (ip_info.ip.addr != 0) {
            ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&ip_info.ip));
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
/*
void init_ble()
{
    esp_err_t ret;

    //Check if the BT controller is already enabled and disable it if it is
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
    ESP_LOGI(DEVICE_NAME, "Bluetooth controller already enabled, disabling first");
    ret = esp_bt_controller_disable();
    if (ret) {
        ESP_LOGE(DEVICE_NAME, "Bluetooth controller disable failed: %s", esp_err_to_name(ret));
        return;
    }
    ret = esp_bt_controller_deinit();
    if (ret) {
        ESP_LOGE(DEVICE_NAME, "Bluetooth controller deinitialize failed: %s", esp_err_to_name(ret));
        return;
    }
    }   
    
    //Check if the bluedroid stack is already enabled and disable it if it is
    if(esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED){

        ESP_LOGI(DEVICE_NAME, "Bluedroid stack already enabled, disabling first");
        ret = esp_bluedroid_disable();
        if (ret) {
            ESP_LOGE(DEVICE_NAME, "Bluedroid stack disable failed: %s", esp_err_to_name(ret));
            return;
        }
    }
    //Check if the bluedroid stack is already initialized and deinitialize it if it is
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED){

        ESP_LOGI(DEVICE_NAME, "Bluedroid stack already initialized, deinitializing first");
        ret = esp_bluedroid_deinit();
        if (ret) {
            ESP_LOGE(DEVICE_NAME, "Bluedroid stack deinitialize failed: %s", esp_err_to_name(ret));
            return;
        }

    }

    // Initilize the BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(DEVICE_NAME, "Bluetooth controller initialize failed: %s", esp_err_to_name(ret));
        return;
    }

    // Enable the BT controller
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if(ret) {
        ESP_LOGE(DEVICE_NAME, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // Check if the bluedroid stack is already uninitialized and initialize it if it is
    if(esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED){
        ret = esp_bluedroid_init();

        if (ret) {
            ESP_LOGE(DEVICE_NAME, "Bluedroid stack initialize failed: %s", esp_err_to_name(ret));
            return;
        }
    } 
    
    //Check if the bluedroid stack is already enabled and enable it if it don't
    if(esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED){
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(DEVICE_NAME, "Bluedroid stack enable failed: %s", esp_err_to_name(ret));
        return;
        }
    } 
    

    // Register the gatt server
    ret = esp_ble_gatts_register_callback(ble_gatts_event_handler);
    if (ret){
        ESP_LOGE(DEVICE_NAME, "%s gatts register failed, error code = %x", __func__, ret);
        return;
    }

    // Register the GAP callback 
    ret = esp_ble_gap_register_callback(gap_event_handler);
    ESP_ERROR_CHECK(ret);
    if (ret){
        ESP_LOGE(DEVICE_NAME, "%s gap register failed, error code = %x", __func__, ret);
        return;
    }

    // Set the local MTU size
     //MTU = Maximum Transmission Unit alltså max storlek på en paket
    //Kan vara bra att minnas att MTU är 23 bytes som standard 
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(512);
    if (local_mtu_ret){
        ESP_LOGE(DEVICE_NAME, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
    
    
    ESP_LOGI(DEVICE_NAME, "BLE init complete");

}
*/

int read_photo_sensor()
{

    //Configure ADC
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_0,
    };

    adc_oneshot_config_channel(adc1_handle, ADC1_CHANNEL_0, &config);

    int adc_raw;
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw);

    adc_oneshot_del_unit(adc1_handle);

    return adc_raw;

}

void init_led()
{
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}

void turn_on_led()
{
    gpio_set_level(LED_PIN, 1);
}

void turn_off_led()
{
    gpio_set_level(LED_PIN, 0);
}






