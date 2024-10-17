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
#include "esp_gatT_common_api.h"
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
#define SERVER_IP "192.168.10.148"
#define PORT 1256

#define TCP_TAG "TCP_CLIENT"
#define BINDOPERATION 1
#define BROADCASTOPERATION 2
#define TCP_SOCKET 2
//#define TCP_PORT 8080

#define LED_PIN 3 //Måste ändras sen

#define SERVICE_UUID 0x1222
#define CHAR_SSID_UUID 0x1223
#define CHAR_PASS_UUID 0x1224
#define CHAR_IP_UUID 0x1225

#define GAP "GAP EVENT"
#define GATT "GATT EVENT"
#define ESP "Message from ESP32"

#define MAX_CHAR_LEN 128
#define MAX_RAW_VALUE 4095
#define MAX_SCALE_VALUE 100
#define ESP_GATT_UUID_CHAR_DESCRIPTION 0x2901

static uint16_t service_handle = 0;
static uint16_t char_ssid_handle = 0;
static uint16_t char_pass_handle = 0;
static uint16_t char_ip_handle = 0;

static char ssid[64] = "Default SSID";
static char pass[64] = "Default Password";
static char ip[64] = "Default IP";
static int password_received = 0;

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

void ble_service(esp_gatt_if_t gatts_if)
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

void characteristics(esp_gatt_if_t gatts_if, uint16_t service_handle, characteristic_t *characteristic)
{

    esp_err_t res;
    esp_gatt_char_prop_t char_property = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_READ;
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

void ble_adv_data()
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

void descriptor(esp_gatt_if_t gatts_if, uint16_t service_handle, const char *description)
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

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    switch(event)
    {

        case ESP_GATTS_REG_EVT:
            ESP_LOGI(DEVICE_NAME, "Register event");
            ble_service(gatts_if);
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

            if(param->write.handle == char_ssid_handle)
            {
                memset(ssid, 0, sizeof(ssid));
                memcpy(ssid, param->write.value, param->write.len);
                ssid[param->write.len] = '\0';
                printf("SSID recv: %.*s\n", param->write.len, param->write.value);
                //ESP_LOGI(DEVICE_NAME, "SSID: %s", ssid);
            }
            else if (param->write.handle == char_pass_handle)
            {
                memset(pass, 0, sizeof(pass));
                memcpy(pass, param->write.value, param->write.len);
                pass[param->write.len] = '\0';

                //ESP_LOGI(DEVICE_NAME, "Password: %s", pass);
                printf("Password recv: %.*s\n", param->write.len, param->write.value);

                password_received = 1;
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

        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(DEVICE_NAME, "Execute write event");
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
            
            characteristics(gatts_if, service_handle, &char_ssid);
            characteristics(gatts_if, service_handle, &char_pass);
            characteristics(gatts_if, service_handle, &char_ip);

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

            
            ESP_LOGE(DEVICE_NAME, "Disconnect event, reason: %d", param->disconnect.reason);

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
        
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(DEVICE_NAME, "ESP_GATTS_MTU_EVT, MTU size negotition: %d", param->mtu.mtu);
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

void mac_address(int sock)
{
    uint8_t mac_addr[6];
    esp_err_t ret = esp_wifi_get_mac(ESP_IF_WIFI_STA, mac_addr);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get MAC address: %s", esp_err_to_name(ret));
        return;
    }

    
    char mac_str[100];
    snprintf(mac_str, sizeof(mac_str), "MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    
    ESP_LOGI(TCP_TAG, "Formatted MAC address: %s", mac_str);
    

    int err = send(sock, mac_str, sizeof(mac_str), 0);
    if (err < 0)
    {
        ESP_LOGE(TCP_TAG, "Error occurred during sending: errno %d", errno);
    }
    else
    {
        ESP_LOGI(TCP_TAG, "MAC address sent: %s", mac_str);
    }

}

int read_photo_sensor();
void init_led();
void turn_on_led();
void turn_off_led();

void tcp_client(void *pvParameters)
{
    
    char adr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    //char host_ip[] = SERVER_IP;
    
    char rx_buffer[20];
    char tx_buffer[100];
    

    
    struct sockaddr_in dest_addr;
    {
        dest_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
        dest_addr.sin_family = addr_family;
        dest_addr.sin_port = htons(PORT);
    };
    
    while (1)
    {

        /*
            dest_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
            dest_addr.sin_family = addr_family;
            dest_addr.sin_port = htons(PORT);
        */

        inet_ntoa_r(dest_addr.sin_addr, adr_str, sizeof(adr_str) - 1);

        // Skapa socket
        int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0){

            ESP_LOGE(TCP_TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TCP_TAG, "Socket created, connecting to %s:%d", SERVER_IP, PORT);

        // Anslut till server
        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if(err != 0)
        {
            ESP_LOGE(TCP_TAG, "Socket unable to connect: errno %d", errno);
            close(sock);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TCP_TAG, "Successfully connected");
        mac_address(sock);
        
        

        // Här börjar loopen för sensor, LED och dess value
        while(1)
        {
            // Read sensor value
            int raw_value = read_photo_sensor();
            int sensor_value = (raw_value * MAX_SCALE_VALUE) / MAX_RAW_VALUE; //Scale value to 0-100

            char message[100];
            snprintf(message, sizeof(message), "Sensor value: %d\n", sensor_value);

            // Skicka data
            int err = send(sock, message, strlen(message), 0);
            if (err < 0)
            {
                ESP_LOGE(TCP_TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            else
            {
                ESP_LOGI(TCP_TAG, "Message sent: %s", message);
            }

            // Ta emot data
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len > 0) {

                rx_buffer[len] = 0;
                ESP_LOGI(TCP_TAG, "Received: %s", rx_buffer);
                if(strcmp(rx_buffer, "TURN_ON_LED") == 0)
                {
                    turn_on_led();
                }
                else if(strcmp(rx_buffer, "TURN_OFF_LED") == 0)
                {
                    turn_off_led();
                }
            }

            else if(len < 0)
            {
                ESP_LOGE(TCP_TAG, "Error occurred during receiving: errno %d", errno);
                break;
            }

        }

        ESP_LOGE(TCP_TAG, "Shutting down socket and restarting...");
        shutdown(sock, 0); //Osäker på om detta är rätt
        close(sock);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{

    if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Connecting to WiFi...");
    }
    else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGI(TAG, "Disconnected from WiFi");
        ESP_LOGI(TAG, "Reconnecting...");
        esp_wifi_connect();
    }
    else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        char ip_address[16];
        esp_ip4addr_ntoa(&event->ip_info.ip, ip_address, sizeof(ip_address));
        ESP_LOGI(TAG, "Got IP: %s", ip_address);
    }
}

void wifi_start() 
{

    // Initiera nätverksgränssnitt
    ESP_ERROR_CHECK(esp_netif_init());

    // Skapa händelseloop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Skapa standard WiFi STA-gränssnitt
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    if (netif == NULL) {
        ESP_LOGE(TAG, "Failed to create default WiFi STA interface");
        return;
    }
    // Initialisera WiFi med standardkonfiguration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Skapa händelsgrupp för att hantera olika händelser
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
}

void wifi_init()
{
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",
        },
    };
    strcpy((char *)wifi_config.sta.ssid, ssid);
    strcpy((char *)wifi_config.sta.password, pass);


    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_LOGI(TAG, "WiFi configuration set, now starting WiFi...");
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi started");
}

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

    adc_oneshot_config_channel(adc1_handle, ADC1_CHANNEL_2, &config);

    int adc_raw;
    adc_oneshot_read(adc1_handle, ADC1_CHANNEL_2, &adc_raw);

    adc_oneshot_del_unit(adc1_handle);

    return adc_raw;

}

void init_led()
{
    //gpio_pad_select_gpio(LED_PIN);
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

void wifi_login()
{   
    printf("Login info received\n");

    while(1)
    {
        if (password_received == 1)
        {
            wifi_init();
            vTaskDelay(pdMS_TO_TICKS(1000));
            password_received = 0;
        }
        else
        {
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
}

void app_main(void)
{


    init_characteristics();

    init_led();
    /*
    while(1)
    {   
        
        int raw_value = read_photo_sensor();
        int scale_value = (raw_value * MAX_SCALE_VALUE) / MAX_RAW_VALUE; //Scale value to 0-100
        printf("Raw value: %d, sensor value: %d\n", raw_value, scale_value);
        if (scale_value < 50)
        {
            turn_on_led();
        }
        else
        {
            turn_off_led();
        }
        //Implementera sätt och skicka value till go servern.
        vTaskDelay(pdMS_TO_TICKS(1000));


    }
   */

    // Initilize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    esp_ble_gatts_app_register(0);
    esp_ble_gatt_set_local_mtu(500);
    
    esp_ble_gap_set_device_name(DEVICE_NAME);

    ble_adv_data();
    esp_ble_gap_start_advertising(&adv_params);
    
    ESP_LOGI(DEVICE_NAME, "BLE init complete");

    wifi_start(); 
    
    

    xTaskCreate(&wifi_login, TAG , 4096, NULL, 5, NULL);

    xTaskCreate(&tcp_client, TCP_TAG, 4096, NULL, 5, NULL);
    
}







