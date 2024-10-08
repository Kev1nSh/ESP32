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
#include <lwip/netdb.h>
#include <string.h>
#include <netdb.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include "sdkconfig.h"
#include "sys/socket.h"
#include "esp_event.h"
#include "esp_log.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <nvs_flash.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gatT_common_api.h"
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_defs.h"
#include "esp_err.h"



#define WIFI_SSID "AuPx-Wifi-2.4"
#define WIFI_PASS "AuPx-HIF"
#define PORT 1256
#define SERVER_IP "192.168.10.74"

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
#define SVC_INST_ID 0
#define GATTS_NUM_HANDLE_TEST_A 4

#define LED_PIN GPIO_NUM_2 // Need to change this to the correct GPIO pin

static const char *TAG = "esp32_Kevin";
static const char *TAG2 = "TCP_Client";

static const char *payload = "Message from ESP32";

//Vet inte heller vad det här är, måste kolla upp
static const uint16_t GATTS_SERVICE_UUID_TEST_A = 0x00FF;
static const uint16_t GATTS_CHAR_UUID_TEST_A = 0xFF01;
static const uint16_t GATTS_DESCR_UUID_TEST_A = 0x3333;

static const uint16_t primary_service_uuid = GATTS_SERVICE_UUID_TEST_A;
static const uint16_t character_declaration_uuid = 0x2803;
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
static const uint8_t char_prop_read_write = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE;
static const uint16_t descr_value = GATTS_DESCR_UUID_TEST_A;


static const uint8_t char_value[4] = {0x11, 0x22, 0x33, 0x44};
static uint8_t service_uuid[16] = {
   
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};


static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
//void wifi_connect_with_credentials(const char *ssid, const char *password);
void init_ble();



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

static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

//vet inte vad det här, måste kolla upp
static esp_gatts_attr_db_t gatt_db[GATTS_NUM_HANDLE_TEST_A] = {

    [0] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
            sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST_A), (uint8_t *)&GATTS_SERVICE_UUID_TEST_A
        }
    },

    // Characteristic Declaration
    [1] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write
        }
    },

    // Characteristic Value
    [2] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_A, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof(char_value), sizeof(char_value), (uint8_t *)char_value
        }
    },

    // Client Characteristic Configuration Descriptor
    [3] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t *)&GATTS_DESCR_UUID_TEST_A, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&descr_value
        }
    },

};

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
    .service_uuid_len = 16,
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),

};
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch(event)
    {
        case ESP_GATTS_REG_EVT:
            esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, GATTS_NUM_HANDLE_TEST_A, SVC_INST_ID);
            break;
    
        case ESP_GATTS_WRITE_EVT:
            if(param->write.handle == gl_profile_tab[PROFILE_A_APP_ID].char_handle)
            {
            
                char ssid[32] = {0};
                char password[64] = {0};
                char received_data[96] = {0};

                //Copy the received data to a local buffer
                memcpy(received_data, param->write.value, param->write.len);

                //Find the delimiter
                char *delimiter = strchr(received_data, ':');
                if (delimiter != NULL)
                {
                    
                    //Extract the SSID
                    size_t ssid_len = delimiter - received_data;
                    strncpy(ssid, received_data, ssid_len);
                    ssid[ssid_len] = '\0'; 

                    //Extract the password
                    strncpy(password, delimiter + 1, param->write.len - ssid_len - 1);
                    password[param->write.len - ssid_len - 1] = '\0';

                    //Print the received data
                    ESP_LOGI(TAG, "Received SSID: %s", ssid);
                    ESP_LOGI(TAG, "Received password: %s", password);

                    //Connect to the Wi-Fi network

                    //wifi_connect_with_credentials(ssid, password);
                    
                    //Kanske jag kan importera credentials till min wifi_connect funktion
                    
                    /*wifi_config_t wifi_config = {
                        .sta = {
                            .ssid = ssid,
                            .password = password,
                        },
                    };  
                    */
                }

                else 
                {
                    ESP_LOGE(TAG, "Invalid data received");
                }

            }
            break;

        default:
            break;

        
    
    }


}

// GATT server event handler
// Finns lite mer kod som kan saknas men kan hittas på:
// https://github.com/espressif/esp-idf/blob/v5.2.3/examples/bluetooth/bluedroid/ble/gatt_server/tutorial/Gatt_Server_Example_Walkthrough.md
void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{


    if(event ==  ESP_GATTS_REG_EVT)
    {
        esp_ble_gap_set_device_name("ESP32_BLE");
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        ESP_ERROR_CHECK(ret);
    }

    if (gl_profile_tab[PROFILE_A_APP_ID].gatts_cb )
    {
        gl_profile_tab[PROFILE_A_APP_ID].gatts_cb(event, gatts_if, param);

    }


}

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch(event)
    {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertising data set complete");
            break;
        
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if(param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(TAG, "Advertising start failed");
            }
            else
            {
                ESP_LOGI(TAG, "Advertising start success");
            }
         break;
        
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if(param->adv_stop_cmpl.status == ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGI(TAG, "Advertising stop success");
            }
            else
            {
                ESP_LOGE(TAG, "Advertising stop failed");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "Update connection parameters");
            break;

        default:
            ESP_LOGI(TAG, "Event %d not handled", event);
            break;
    }
}       



/*
// Wi-Fi event handler
//OSäker om jag behöver denna delen
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: %s", ip4addr_ntoa(&event->ip_info.ip));
    }
}
*/

void init_led();
void turn_on_led();
void turn_off_led();
void tcp_client(void *pvParameters);
int read_photo_sensor();
void wifi_connect();

void app_main(void)
{

    init_led();

    // Initilize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initilize Bluetooth
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());


    // Register the GATT server 
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(PROFILE_A_APP_ID));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));  

    wifi_connect(); 

    init_ble();

   // xTaskCreate(tcp_client, "tcp_client", 4096, NULL, 5, NULL);

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

            ESP_LOGE(TAG2, "Unable to create socket: errno %d", errno);
            break;

        }

        ESP_LOGI(TAG2, "Socket created, connecting to %s:%d", host_ip, PORT);

        // Anslut till server
        
        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if(err != 0)
        {
            ESP_LOGE(TAG2, "Socket unable to connect: errno %d", errno);
            close(sock);
            break;
        }

        
        ESP_LOGI(TAG2, "Successfully connected");

        
        while(1)
        {
            // Read sensor value
            int sensor_value = read_photo_sensor();
            sprintf(tx_buffer, "Sensor value: %d", sensor_value);

            // Skicka data
            int err = send(sock, payload, strlen(payload), 0);
            if (err < 0)
            {
                ESP_LOGE(TAG2, "Error occurred during sending: errno %d", errno);
                break;
            }

            // Ta emot data
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len < 0) {

                ESP_LOGE(TAG2, "recv failed: errno %d", errno);
                break;

            }

            else {
                
                rx_buffer[len] = 0; // Null-terminera mottaget meddelande
                ESP_LOGI(TAG2, "Received %d bytes: %s", len, rx_buffer);
                
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
            ESP_LOGE(TAG2, "Shutting down socket and restarting...");
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

// Connect to a Wi-Fi network with the specified SSID and password
// Lite osäker på om jag behöver denna funktionen
/*
void wifi_connect_with_credentials(const char *ssid, const char *password)
{
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ""
            .password = "",
        },
    };

    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());  
    ESP_ERROR_CHECK(esp_wifi_connect());


}
*/
void init_ble()
{
    esp_err_t ret;

    // Initilize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initilize the BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if(ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed", __func__);
        return;
    }

    // Initilize bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed", __func__);
        return;
    }

    // Register the gatt server
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(TAG, "%s gatts register failed, error code = %x", __func__, ret);
        return;
    }

    // Register the GAP callback 
    ret = esp_ble_gap_register_callback(gap_event_handler);
    ESP_ERROR_CHECK(ret);
    if (ret){
        ESP_LOGE(TAG, "%s gap register failed, error code = %x", __func__, ret);
        return;
    }


    // Vet inte om det nöödvändigt att ha dessa 3 funktioner, skrev av från GATT server exemplet på ESP-IDF
    // https://github.com/espressif/esp-idf/blob/v5.2.3/examples/bluetooth/bluedroid/ble/gatt_server/tutorial/Gatt_Server_Example_Walkthrough.md
    
    // Register the application ID
    // This function is called to register application identifier.
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID); 
    if (ret){
        ESP_LOGE(TAG, "%s gatts app register failed, error code = %x", __func__, ret);
        return;
    }

    /* 
    Denna är om man har flera appar som ska registreras, kommenterar det för att den buildar inte annars

    ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
    if (ret){
        ESP_LOGE(TAG, "%s gatts app register failed, error code = %x", __func__, ret);
        return;
    }
    */

    // Set the local MTU size
    //MTU = Maximum Transmission Unit alltså max storlek på en paket
    //Kan vara bra att minnas att MTU är 23 bytes som standard 
    
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(512);
    if (local_mtu_ret){
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(PROFILE_A_APP_ID));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    //ESP_ERROR_CHECK(esp_ble_gap_register_callback);  //kommenterade eftersom vet inte om det krockar med nåt

}

//Komma tllbak till denna senare
//extern esp_err_t esp_ble_gap_config_adv_data(esp_ble_adv_data_t *adv_data);


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
