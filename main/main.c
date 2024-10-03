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
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
/*
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
*/


#define WIFI_SSID "grupp3"
#define WIFI_PASS "datorkomunkation"
#define PORT 1256
#define SERVER_IP "192.168.1.100"
#define LED_PIN GPIO_NUM_2 // Need to change this to the correct GPIO pin


static const char *TAG = "esp32_Kevin";
static const char *TAG2 = "TCP_Client";

static const char *payload = "Message from ESP32";

void tcp_client(void)
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

void wifi_connect() {

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


int read_photo_sensor()
{

    //Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0); // Asumption that the photo sensor is connected to GPIO 36/adc1_channel_0
    
    //Read ADC value
    int adc_value = adc1_get_raw(ADC1_CHANNEL_0);
    return adc_value;

}


void init_led()
{
    gpio_pad_select_gpio(LED_PIN);
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

void app_main(void)
{

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_led();

    wifi_connect(); 

    xTaskCreate(tcp_client, "tcp_client", 4096, NULL, 5, NULL);

}