/* 
   
	Set up to build this project
	idf.py --preview set-target esp32c5
	idf.py menuconfig
	idf.py build
	idf.py -p com4 flash monitor

*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <netdb.h>
#include "esp_eth.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>

#define EXAMPLE_ESP_WIFI_SSID "SSID"
#define EXAMPLE_ESP_WIFI_PASS "KEY"
#define EXAMPLE_ESP_MAXIMUM_RETRY 10
#define EXAMPLE_STATIC_IP_ADDR "192.168.1.200"
#define EXAMPLE_STATIC_NETMASK_ADDR "255.255.255.0"
#define EXAMPLE_STATIC_GW_ADDR "192.168.1.254"
#define EXAMPLE_MAIN_DNS_SERVER "192.168.1.253"
#define EXAMPLE_BACKUP_DNS_SERVER "192.168.1.253"
#define UDP_RX_PORT 9000

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;

static esp_err_t example_set_dns_server(esp_netif_t *netif, uint32_t addr, esp_netif_dns_type_t type)
{
    if (addr && (addr != IPADDR_NONE))
	{
        esp_netif_dns_info_t dns;
        dns.ip.u_addr.ip4.addr = addr;
        dns.ip.type = IPADDR_TYPE_V4;
        ESP_ERROR_CHECK(esp_netif_set_dns_info(netif, type, &dns));
    }
    return ESP_OK;
}

static void example_set_static_ip(esp_netif_t *netif)
{
    
	if (esp_netif_dhcpc_stop(netif) != ESP_OK) 
	{
        printf("\r\n[%s]\tFailed to stop dhcp client", TAG);
        return;
    }
	
    esp_netif_ip_info_t ip;
    memset(&ip, 0 , sizeof(esp_netif_ip_info_t));
    ip.ip.addr = ipaddr_addr(EXAMPLE_STATIC_IP_ADDR);
    ip.netmask.addr = ipaddr_addr(EXAMPLE_STATIC_NETMASK_ADDR);
    ip.gw.addr = ipaddr_addr(EXAMPLE_STATIC_GW_ADDR);
    if (esp_netif_set_ip_info(netif, &ip) != ESP_OK) 
	{
        printf("\r\n[%s]\tFailed to set ip info", TAG);
        return;
    }
     printf("\r\n[%s]\tSuccess to set static ip\t[%s]\tmask\t[%s]Gateway\t[%s]", TAG, EXAMPLE_STATIC_IP_ADDR, EXAMPLE_STATIC_NETMASK_ADDR, EXAMPLE_STATIC_GW_ADDR);    
	ESP_ERROR_CHECK(example_set_dns_server(netif, ipaddr_addr(EXAMPLE_MAIN_DNS_SERVER), ESP_NETIF_DNS_MAIN));
    ESP_ERROR_CHECK(example_set_dns_server(netif, ipaddr_addr(EXAMPLE_BACKUP_DNS_SERVER), ESP_NETIF_DNS_BACKUP));
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) 
	{
		esp_wifi_connect();
    }
	else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
	{
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
		{
            esp_wifi_connect();
            s_retry_num++;
            printf("\r\n[%s] Retry Connect to the AP", TAG);
        }
		else
		{
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
		printf("\r\n[%s] connect to the AP fail", TAG);
    }
	else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
	{
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		esp_ip4_addr_t* deviceIP = &event->ip_info.ip;
        printf("\r\n\t[%s]\tGot IP:[%d.%d.%d.%d]",TAG, esp_ip4_addr_get_byte(deviceIP, 0),esp_ip4_addr_get_byte(deviceIP, 1),esp_ip4_addr_get_byte(deviceIP, 2),esp_ip4_addr_get_byte(deviceIP, 3) );
		s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
	//set static IP
	example_set_static_ip(sta_netif);
	
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&event_handler,NULL,&instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,IP_EVENT_STA_GOT_IP,&event_handler, NULL,&instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = {EXAMPLE_ESP_WIFI_SSID},
            .password = {EXAMPLE_ESP_WIFI_PASS},
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

	printf("\r\n[%s]\twifi_init_sta finished.", TAG);
    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,pdFALSE, pdFALSE,portMAX_DELAY);
    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
	{
        printf("\r\n[%s]\tconnected to ap SSID:[%s]\tKey:[%s]", TAG, EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
	else if (bits & WIFI_FAIL_BIT)
	{
        printf("\r\n[%s]\tFAILED to connected to ap SSID:[%s]\tKey:[%s]", TAG, EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
	else 
	{
        printf("\r\n[%s]\tUNEXPECTED EVENT", TAG);
    }
}

static void udp_client_task(void *pvParameters)
{
    char rx_buffer[6];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
	struct sockaddr_in  self_ap_addr;
	unsigned char* unsignedShortIntToCharPointer;
	unsigned short int packetCount;
	int remoteRSSI;
	
	self_ap_addr.sin_family = addr_family;
    self_ap_addr.sin_port = htons(UDP_RX_PORT);
	
    while (1)
	{
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0)
		{
            printf("\r\n[%s]\tUnable to create socket: errno [%d]", TAG, errno);
            break;
        }
		if(bind(sock, (const struct sockaddr *)&self_ap_addr,  sizeof(struct sockaddr_in)) < 0 )
		{
		  printf("\r\n[%s]\tAP UDP socket not binded", TAG);
		  shutdown(sock, 0);
		  close(sock);
		  break;
		}

  
        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);
		printf("\r\n[%s]\tSocket created", TAG);
        
		while(1)
		{
            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);
            // Error occurred during receiving
            if (len < 0)
			{
				printf("\r\n[%s]\trecvfrom failed: errno %d", TAG, errno);               
                break;
            }
            // Data received
            else
			{
                //printf("\r\n[%s]\tReceived [%d] bytes from [%s]:", TAG, len, host_ip);
				//grab the packet number
				unsignedShortIntToCharPointer = (unsigned char*)&packetCount;
				unsignedShortIntToCharPointer[0] = rx_buffer[0];
				unsignedShortIntToCharPointer[1] = rx_buffer[1];
				//grab the remote RSSI
				unsignedShortIntToCharPointer = (unsigned char*)&remoteRSSI;
				unsignedShortIntToCharPointer[0] = rx_buffer[2];
				unsignedShortIntToCharPointer[1] = rx_buffer[3];
				unsignedShortIntToCharPointer[2] = rx_buffer[4];
				unsignedShortIntToCharPointer[3] = rx_buffer[5];
				printf("\r\nGot Packet\t[%d]\tRemote RSSI[%d]", packetCount, remoteRSSI);				
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        if (sock != -1)
		{
            printf("\r\n[%s]\tShutting down socket and restarting...", TAG);
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}
void app_main(void)
{	
	//Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    printf("\r\n\r\n\t\t\tNVS[%d]", ret);
	wifi_init_sta();
	xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
}