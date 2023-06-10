#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "esp_event.h"

#include "Sim7090G_module.h"

#define BAUDRATE 115200
#define Client_ID "da73a39c-ee99-44b6-aa4c-86cb553d712f"
#define PUBLISH_TOPIC "messages/6b88a402-9679-499b-83fc-9502d113df22/attributets"
//"messages/dc9d5717-2522-4f39-a899-cce286152284/attributets" "messages/6b88a402-9679-499b-83fc-9502d113df22/attributets"
#define U2RXD 16
#define U2TXD 17
const char * TAG = "Sim7090G";

void Output_init(gpio_num_t Output_pin){
	gpio_config_t io_conf;
	io_conf.pin_bit_mask = 1ULL<< Output_pin;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pull_up_en = 1;
	io_conf.pull_down_en = 0;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	gpio_config(&io_conf);
}

simcom simcom_7090G;

client client_MQTT=
{
	Client_ID,
	"mqtt.innoway.vn",
	"batky",
	"sLmYvvEAUrrfNjZKVX2xI7CPfiW2fojh",
	1883,
	};

void app_main(void)
{
	Output_init(GPIO_NUM_2); 	//Pin 2 là chân PWRKEY
	Output_init(GPIO_NUM_4);	//Pin 4 là chân LED
	Output_init(GPIO_NUM_5);	//Pin 5 là chân DTR
	init_simcom(UART_NUM_2,U2TXD,U2RXD,BAUDRATE);	//Thiết lập thông tin để ESP giao tiếp SIM7090G
	printf("Waiting for Module Ready");
	bool Module_ready = Power_on(GPIO_NUM_2); //Bật module lên thông qua PWRKEY

	bool MQTT_client_data_upload = false;
	if(Module_ready)
	{
		if(!MQTT_client_data_upload)
		{
			int retry=5;
			while(retry--){
			vTaskDelay(10000/portTICK_PERIOD_MS); 	//Đợi 10s cho module ổn định
			if(mqtt_start(client_MQTT, 10*60, 0, 0, 0, 5))	//Upload thông tin client lên thiết bị
				{
					MQTT_client_data_upload = true;
					break;
				}
			else
				ESP_LOGI(TAG,"CAN NOT UPLOAD CLIENT DATA THIS PLACE DON'T HAVE INTERNET");
			}
		}
	}

	bool first_time=true;
    while (true) {
    	if(first_time == false) vTaskDelay(10000/portTICK_PERIOD_MS); //chờ 10s để module hoạt động ổn định sau khi Sleep

    	gpio_set_level(GPIO_NUM_4, 1); 	//Bật led lên để theo dõi
    	uint64_t time_start = esp_timer_get_time() / 1000 ;

    	bool flag_first_data_ok = false;
    	if(MQTT_client_data_upload)
    	{
			if(get_RSRP_RSRQ_SINR_PCI_cellID(20)==true)	//Tiến hành lấy dữ liệu đầu tiên
			{
				ESP_LOGI(TAG,"RSRP=%s,RSRQ=%s,SINR=%s,PCI=%s,CELLID=%s",simcom_7090G.RSRP,simcom_7090G.RSRQ,simcom_7090G.SINR,simcom_7090G.PCI,simcom_7090G.cellID);
				flag_first_data_ok = true;
			}
			else
			{
				flag_first_data_ok = false;
				ESP_LOGI(TAG,"NO SERVICE AT THIS LOCATION");
			}
    	}
    	bool flag_second_data_ok = false;
    	if(flag_first_data_ok)
    	{
    		GNSS_Power_ON(2);	//Bật GNSS lên để lấy dữ liệu
    		if(first_time)
    		{
    			 vTaskDelay(90000/portTICK_PERIOD_MS); //chờ 1p30s để GNSS hoạt động ổn định
    			 first_time=false;
    		}
    		else vTaskDelay(5000/portTICK_PERIOD_MS);	//Chờ 5s

    		if(get_latitude_longtitude(20)==true)	//Tiến hành lấy dữ liệu
    		{
    			flag_second_data_ok = true;
    			ESP_LOGI(TAG,"LAT:%s , LONG:%s",simcom_7090G.Latitude,simcom_7090G.Longitude);
    		}

    		else
    		{
    			flag_second_data_ok = false;

    			ESP_LOGI(TAG,"CAN NOT TAKE LAT LONG AT THIS LOCATION");
    		}
    		GNSS_Power_OFF(2);	//Tắt GNSS

    	}

    	bool send_MQTT_success= false;
    	if(flag_second_data_ok)
    	{
    		vTaskDelay(90000/portTICK_PERIOD_MS);
    		char buff[200];
    		while(true)
    		{
				if(isConnected_network(5))	//check kết nối mạng
				{
					if(Connect_MQTT(2))		//Kết nối MQTT
						{
								while(esp_timer_get_time()/1000 - time_start < 3*60000){vTaskDelay(10/portTICK_PERIOD_MS);}
								sprintf(buff,"{\"Latitude\":%s,\"Longitude\":%s,\"RSRP\":%s,\"RSRQ\":%s,\"SINR\":%s,\"PCI\":%s,\"CELLID\":%s}",simcom_7090G.Latitude,simcom_7090G.Longitude,simcom_7090G.RSRP,simcom_7090G.RSRQ,simcom_7090G.SINR,simcom_7090G.PCI,simcom_7090G.cellID);
								MQTT_PUBLISH(PUBLISH_TOPIC, buff, 2, 0, 0);	//Publish data
								send_MQTT_success= true;
								Dis_Connect_MQTT(2);	//Ngắt kết nối MQTT
								gpio_set_level(GPIO_NUM_4, 0);	//Tắt LED
								break;
						}
					else
						{
							ESP_LOGI(TAG,"CAN NOT SEND DATA TO MQTT BECAUSE CAN'T CONNECT MQTT...... RECONNECTING......");
							vTaskDelay(1000/portTICK_PERIOD_MS);
						}
				}
				vTaskDelay(10000/portTICK_PERIOD_MS);
    		}
    	}
    	if(send_MQTT_success)Sleep_mode(1,GPIO_NUM_5, 2); //Sleep_mode ON
    	vTaskDelay(2*60000/portTICK_PERIOD_MS);
    	if(send_MQTT_success)Sleep_mode(0,GPIO_NUM_5, 2); //Sleep_mode OFF
	}
}
