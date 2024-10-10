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
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <inttypes.h>
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
#include <inttypes.h>


//#include <simple_ble/SimpleBle.h>


#define WIFI_SSID "AuPx-Wifi-2.4"
#define WIFI_PASS "AuPx-HIF"
#define PORT 1256
#define SERVER_IP "192.168.10.74"

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
#define SVC_INST_ID 0
#define GATTS_NUM_HANDLE_TEST_A 4
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40
#define adv_config_flag      (1 << 0)
#define BUF_MAX_SIZE 1024

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

static uint8_t char_value[] = {0x11, 0x22, 0x33};
//static uint16_t char_value_len = sizeof(char_value);
static uint8_t service_uuid[16] = {
   
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static uint8_t adv_config_done;

typedef struct{
    uint8_t *prepare_buf;
    int prepare_len;
}prepare_type_env_t;

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void init_ble();



static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr = {0},
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
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
    esp_bd_addr_t remote_bda; // Add this line
};

static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static prepare_type_env_t a_prepare_write_env;

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



static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch(event)
    {
        case ESP_GATTS_REG_EVT:

            //Register the application
            ESP_LOGI(TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
            gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
            gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

        
            esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, GATTS_NUM_HANDLE_TEST_A, SVC_INST_ID);
            esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
            esp_ble_gap_set_device_name("ESP32_BLE");
    
            //Configure the adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= adv_config_flag;
            break;
            
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_READ_EVT, conn_id %u, trans_id %u, handle %u",
                    (unsigned int)param->read.conn_id,
                    (unsigned int)param->read.trans_id,
                    (unsigned int)param->read.handle);
            esp_gatt_rsp_t rsp;

            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = 4;
            rsp.attr_value.value[0] = 0xde;
            rsp.attr_value.value[1] = 0xed;
            rsp.attr_value.value[2] = 0xbe;
            rsp.attr_value.value[3] = 0xef;

            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            break;
            
        case ESP_GATTS_WRITE_EVT:{
            ESP_LOGI(TAG, "ESP_GATTS_WRITE_EVT, conn_id %u, trans_id %u, handle %u",
                    (unsigned int)param->write.conn_id,
                    (unsigned int)param->write.trans_id,
                    (unsigned int)param->write.handle); 

            //Check if the write is request or a command
            if(!param->write.is_prep){
                ESP_LOGI(TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
                esp_log_buffer_hex(TAG, param->write.value, param->write.len);
                if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        //Kan strulas här
                        if (gl_profile_tab[PROFILE_A_APP_ID].property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                            ESP_LOGI(TAG, "notify enable");
                            uint8_t notify_data[15];
                            for (int i = 0; i < sizeof(notify_data); ++i)
                            {
                                notify_data[i] = i%0xff;
                            }
                            // the size of notify_data[] need less than MTU size
                            esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, 
                                                        gl_profile_tab[PROFILE_A_APP_ID].char_handle, 
                                                        sizeof(notify_data), 
                                                        notify_data, false);
                        }
                
                    }else if(descr_value == 0x0002){
                        if (gl_profile_tab[PROFILE_A_APP_ID].property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                            ESP_LOGI(TAG, "indicate enable");
                            uint8_t indicate_data[15];
                            for (int i = 0; i < sizeof(indicate_data); ++i)
                            {
                                indicate_data[i] = i%0xff;
                            }
                            // the size of indicate_data[] need less than MTU size
                            esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, 
                                                        gl_profile_tab[PROFILE_A_APP_ID].char_handle, 
                                                        sizeof(indicate_data), 
                                                        indicate_data, true);
                        }
                    }
                    else if(descr_value == 0x0000)
                    {
                        ESP_LOGI(TAG, "notify/indicate disable ");
                    }
                    else
                    {
                        ESP_LOGE(TAG, "unknown value");
                    }
                }
            }
            example_write_event_env(gatts_if, &a_prepare_write_env, param);
            break;
        }

        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            example_write_event_env(gatts_if, &a_prepare_write_env, param);
            break;

        case ESP_GATTS_RESPONSE_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_RESPONSE_EVT, status =  %d, handle =  %d", param->rsp.status, param->rsp.handle);    
            
            //Check the status of the response
            if (param->rsp.status != ESP_GATT_OK)
            {
                ESP_LOGI(TAG, "Response failed with status %d", param->rsp.status);
                //Här kan jag skriva kod för att hantera om responsen misslyckas
            }
            else
            {
                ESP_LOGI(TAG, "Response successful for handle %d", param->rsp.handle);
                //Här kan jag skriva kod för att hantera om responsen lyckas
            }
            break;
            
        case ESP_GATTS_MTU_EVT:

        case ESP_GATTS_CONF_EVT:

        case ESP_GATTS_UNREG_EVT:

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote " ESP_BD_ADDR_STR, param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));

            
            //Store the connection id and remote bluetooth device address
            gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
            memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            
            //Update the connection parameters
            conn_params.latency = 0;
            conn_params.max_int = 0x30;    // max_int = 0x30*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms

            ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x", 
                    param->connect.conn_id, 
                    param->connect.remote_bda[0], 
                    param->connect.remote_bda[1], 
                    param->connect.remote_bda[2], 
                    param->connect.remote_bda[3], 
                    param->connect.remote_bda[4], 
                    param->connect.remote_bda[5]);
                gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
                break;

        case ESP_GATTS_DISCONNECT_EVT:  
            ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, reason %d", param->disconnect.reason);
            ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, conn_id %d, remote " ESP_BD_ADDR_STR, param->disconnect.conn_id, ESP_BD_ADDR_HEX(param->disconnect.remote_bda));
            
            //Clear the connection id and remote bluetooth device address
            gl_profile_tab[PROFILE_A_APP_ID].conn_id = 0;
            memset(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, 0, sizeof(esp_bd_addr_t));

            //Restart advertising
            esp_ble_gap_start_advertising(&adv_params); 
            
            break;

        case ESP_GATTS_CREATE_EVT: 
            ESP_LOGI(TAG, "ESP_GATTS_CREATE_EVT, status %d, service_handle %d\n", param->create.status, param->create.service_handle);
            if(param->create.status == ESP_GATT_OK)
            {
               
                gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
                gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
                gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

                esp_gatt_char_prop_t a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;

                //Logga om service skapades
                esp_err_t start_service_ret = esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
                if (start_service_ret != ESP_OK){
                    ESP_LOGE(TAG, "start service failed, error code = %x", start_service_ret);
                }
                
                a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;	
            
                esp_attr_value_t char_val = {
                .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
                .attr_len = sizeof(char_value),
                .attr_value = char_value,
                };
            

            
                esp_err_t add_char_ret =
                esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle,
                                        &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                        a_property,
                                        &char_val,
                                        NULL);
            
                if (add_char_ret != ESP_OK){
                    ESP_LOGE(TAG, "add char failed, error code =%x", add_char_ret);
                }
                
            }   
            else {
                ESP_LOGE(TAG, "Service creation failed, status %d", param->create.status); 
            }
            break;

        case ESP_GATTS_START_EVT:

        case ESP_GATTS_STOP_EVT:

        case ESP_GATTS_ADD_CHAR_EVT:
            uint16_t length = 0;
            const uint8_t *prf_char;

            ESP_LOGI(TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n", 
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
                gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
                gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
                gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
                esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
                if (get_attr_ret == ESP_FAIL){
                    ESP_LOGE(TAG, "ILEGAL HANDLE =%x", get_attr_ret);
                }
            
            ESP_LOGI(TAG, "the gatts demo char length = %x\n", length);
            for(int i = 0; i < length; i++){
                ESP_LOGI(TAG, "prf_char[%x] =%x\n", i, prf_char[i]);
            }
            esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(
                                    gl_profile_tab[PROFILE_A_APP_ID].service_handle,
                                    &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                    NULL, NULL);
            if (add_descr_ret){
                ESP_LOGE(TAG, "add char descr failed, error code =%x", add_descr_ret);
            }
            break;
        
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            ESP_LOGI(TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n", 
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            break;

        
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:

        case ESP_GATTS_SET_ATTR_VAL_EVT:

        case ESP_GATTS_ADD_INCL_SRVC_EVT:

        case ESP_GATTS_SEND_SERVICE_CHANGE_EVT:

    

        default:
         ESP_LOGI(TAG, "Event %d not handled", event);
          break;

    
    }

}

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if(event ==  ESP_GATTS_REG_EVT)
    {
        if(param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        }   
        else
        {
            ESP_LOGI(TAG, "Reg app failed, app_id %04x, status %d\n", param->reg.app_id, param->reg.status);
            return;
        }
    }
    
    do 
    {
        int idx;
        for(idx = 0; idx < PROFILE_NUM; idx++)
        {
            if(gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile_tab[idx].gatts_if)
            {
                if(gl_profile_tab[idx].gatts_cb)
                {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    }
    while(0);
}

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch(event)
    {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertising data set complete");
            adv_config_done &= (~adv_config_flag);
            if(adv_config_done == 0)
            {
                esp_ble_gap_start_advertising(&adv_params);
            }
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
            ESP_LOGI(TAG, "Update connection parameters = %d, conn_int = %d, latency = %d, timeout = %d", 
                param->update_conn_params.status, 
                param->update_conn_params.conn_int, 
                param->update_conn_params.latency, 
                param->update_conn_params.timeout);
            break;

        default:
            ESP_LOGI(TAG, "Event %d not handled", event);
            break;
    }
}       


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

void init_ble()
{
    esp_err_t ret;

    //Check if the BT controller is already enabled and disable it if it is
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
    ESP_LOGI(TAG, "Bluetooth controller already enabled, disabling first");
    ret = esp_bt_controller_disable();
    if (ret) {
        ESP_LOGE(TAG, "Bluetooth controller disable failed: %s", esp_err_to_name(ret));
        return;
    }
    ret = esp_bt_controller_deinit();
    if (ret) {
        ESP_LOGE(TAG, "Bluetooth controller deinitialize failed: %s", esp_err_to_name(ret));
        return;
    }
    }   
    
    //Check if the bluedroid stack is already enabled and disable it if it is
    if(esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED){

        ESP_LOGI(TAG, "Bluedroid stack already enabled, disabling first");
        ret = esp_bluedroid_disable();
        if (ret) {
            ESP_LOGE(TAG, "Bluedroid stack disable failed: %s", esp_err_to_name(ret));
            return;
        }
    }
    //Check if the bluedroid stack is already initialized and deinitialize it if it is
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED){

        ESP_LOGI(TAG, "Bluedroid stack already initialized, deinitializing first");
        ret = esp_bluedroid_deinit();
        if (ret) {
            ESP_LOGE(TAG, "Bluedroid stack deinitialize failed: %s", esp_err_to_name(ret));
            return;
        }

    }

    // Initilize the BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Bluetooth controller initialize failed: %s", esp_err_to_name(ret));
        return;
    }

    // Enable the BT controller
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if(ret) {
        ESP_LOGE(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // Check if the bluedroid stack is already uninitialized and initialize it if it is
    if(esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED){
        ret = esp_bluedroid_init();

        if (ret) {
            ESP_LOGE(TAG, "Bluedroid stack initialize failed: %s", esp_err_to_name(ret));
            return;
        }
    } 
    
    //Check if the bluedroid stack is already enabled and enable it if it don't
    if(esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED){
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid stack enable failed: %s", esp_err_to_name(ret));
        return;
        }
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

    //Register the application ID
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID); 
    if (ret){
        ESP_LOGE(TAG, "%s gatts app register failed, error code = %x", __func__, ret);
        return;
    }

    // Set the local MTU size
     //MTU = Maximum Transmission Unit alltså max storlek på en paket
    //Kan vara bra att minnas att MTU är 23 bytes som standard 
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(512);
    if (local_mtu_ret){
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
    
    ESP_LOGI(TAG, "BLE init complete");

    // https://github.com/espressif/esp-idf/blob/v5.2.3/examples/bluetooth/bluedroid/ble/gatt_server/tutorial/Gatt_Server_Example_Walkthrough.md
    //Denna är om man har flera appar som ska registreras, kommenterar det för att den buildar inte annars
    /*
    ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
    if (ret){
        ESP_LOGE(TAG, "%s gatts app register failed, error code = %x", __func__, ret);
        return;
    }
    */

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

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_status_t status = ESP_GATT_OK;
    if(param->write.need_rsp)
    {
        if(param->write.is_prep)
        {
            if(param->write.offset > BUF_MAX_SIZE)
            {
                status = ESP_GATT_INVALID_OFFSET;
            }
            else if((param->write.offset + param->write.len) > BUF_MAX_SIZE)
            {
                status = ESP_GATT_INVALID_ATTR_LEN;
            }
            if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL)
            {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(BUF_MAX_SIZE);
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL)
                {
                    status = ESP_GATT_NO_RESOURCES;
                }
            }
        
            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            if(gatt_rsp)
            {
                gatt_rsp->attr_value.len = param->write.len;
                gatt_rsp->attr_value.handle = param->write.handle;
                gatt_rsp->attr_value.offset = param->write.offset;
                gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
                memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
                esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
                if (response_err != ESP_OK)
                {
                    ESP_LOGE(TAG, "Send response error");
                }
                free(gatt_rsp);
            }
            else{
                ESP_LOGE(TAG, "malloc failed, no resource to send response\n");
                status = ESP_GATT_NO_RESOURCES;
            }
            if (status != ESP_GATT_OK)
            {
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                param->write.value, 
                param->write.len);
            prepare_write_env->prepare_len += param->write.len;
        }
        else
        {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC)
    {
        esp_log_buffer_hex(TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }
    else
    {
        ESP_LOGI(TAG, "ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf)
    {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}



//54-32-04-01-49-f0
/*
    EXTRA FUNCTIONS THAT I MAY WANT TO USE LATER OR IMPLEMENT

    Wi-Fi event handler
    OSäker om jag behöver denna delen
    static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) 
    {
        if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) 
        {
            ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
            esp_wifi_connect();
        }   
        else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
        {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "Got IP: %s", ip4addr_ntoa(&event->ip_info.ip));
        }
    }

    //Komma tllbak till denna senare
    //extern esp_err_t esp_ble_gap_config_adv_data(esp_ble_adv_data_t *adv_data);

    // Connect to a Wi-Fi network with the specified SSID and password
    // Lite osäker på om jag behöver denna funktionen

    //void wifi_connect_with_credentials(const char *ssid, const char *password);

    void wifi_connect_with_credentials(const char *ssid, const char *password)
    {
        wifi_config_t wifi_config = 
        {
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
                    
                    wifi_config_t wifi_config = {
                        .sta = {
                            .ssid = ssid,
                            .password = password,
                        },
                    };  
                    
                }
            

    */

