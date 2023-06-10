#include "Sim7090G_module.h"
#include <stdio.h>
#include <esp_log.h>


AT_flag _readFeedback(uint32_t timeout, char *expect) {
  uint64_t timeCurrent = esp_timer_get_time() / 1000;
  while(esp_timer_get_time() / 1000 < (timeout + timeCurrent)) {
    if(simcom_7090G.AT_buff_avai) {
      if(strstr((char *)simcom_7090G.AT_buff, "ERROR"))
      {
    	  return AT_ERROR;
      }

      else if(strstr((char *)simcom_7090G.AT_buff, expect)) return AT_OK;
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
  return AT_TIMEOUT;
}
bool WaitRestPond(uint32_t time_out)
{
	uint64_t time_old = esp_timer_get_time() / 1000;
	while(!simcom_7090G.AT_buff_avai && !(esp_timer_get_time()/1000>time_old+time_out))
	{
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
	if(simcom_7090G.AT_buff_avai) return true;
	else return false;
}

static const char * TAG = "Sim7090G";
static const char * Module_send_command = "ESP32";
void UART_RX(void *pvParameters);
void init_simcom(uart_port_t uart_num, int tx_io_num, int rx_io_num, int baud_rate)
{
  uart_config_t uart_config =
  {
    .baud_rate = baud_rate,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	.source_clk = UART_SCLK_DEFAULT,
  };

  uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);
  uart_param_config(uart_num, &uart_config);
  uart_set_pin(uart_num, tx_io_num, rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  xTaskCreate(UART_RX, "uart_task1_receive_data", 4*4096, NULL, 10, NULL);
}
void UART_RX(void *pvParameters){
  uint8_t data[512];
  while (1) {
    int len = uart_read_bytes(UART_NUM_2, data,BUF_SIZE, 100 / portTICK_PERIOD_MS);
    // Write data back to the UART
    if (len) {
      data[len] = '\0';
      ESP_LOGI(TAG, "Receive: %s", (char*) data);
      if(strstr((char*)data, "+CMQPUB:"))
      {
        simcom_7090G.mqtt_CB((char*)data);
      }
      else if(strstr((char*)data, "+CMQPUB:")) {}
      else
      {
        memcpy(simcom_7090G.AT_buff, data, len);
        simcom_7090G.AT_buff_avai = true;
      }
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}

static int filter_comma(char *respond_data, int begin, int end, char *output)
{
    memset(output, 0, strlen(output));
    int count_filter = 0;
    int lim = 0;
    int start = 0;
    int finish = 0;
    int i = 0;
    for (i = 0; i < strlen(respond_data); i++)
    {
        if ( respond_data[i] == ',')
        {
            count_filter ++;
            if (count_filter == begin)          start = i+1;
            if (count_filter == end)            finish = i;
        }

    }
    lim = finish - start;
    for (i = 0; i < lim; i++){
        output[i] = respond_data[start];
        start ++;
    }
    output[i] = 0;
    return 0;
}

static void send_ATComand(char *ATcommand) {
  ESP_LOGI(Module_send_command, "Send: %s", ATcommand);
  simcom_7090G.AT_buff_avai = false;
  memset(simcom_7090G.AT_buff, 0, BUF_SIZE);
  uart_write_bytes(UART_NUM_2, (char *)ATcommand, strlen((char *) ATcommand));
  uart_write_bytes(UART_NUM_2, "\r\n", strlen("\r\n"));
  vTaskDelay(100/portTICK_PERIOD_MS);
}

bool Power_on(gpio_num_t Pin)
{
	gpio_set_level(Pin, 1);
	vTaskDelay(200/portTICK_PERIOD_MS);
	gpio_set_level(Pin, 0);
	vTaskDelay(3000/portTICK_PERIOD_MS);
	gpio_set_level(Pin, 1);
	int retry =3;
	AT_flag res;
	while (retry --)
	{
		res = _readFeedback(15000, "READY");
		if(res == AT_OK) return true;
		else if(res == AT_ERROR) return false;
	}
	return false;
}

bool Sleep_mode(bool Mode,gpio_num_t Pin_control,int retry)
{
	AT_flag res;
	if(Mode == 1)
	{
		gpio_set_level(Pin_control, 1);
		while(retry --)
		{
			send_ATComand("AT+CSCLK=1");
			res = _readFeedback(1000, "OK");
			    if(res == AT_OK) return true;
			    else if(res == AT_ERROR) return false;
		}
	}
	else
	{
		gpio_set_level(Pin_control, 0);
		vTaskDelay(100/portTICK_PERIOD_MS);
		while(retry --)
		{
			send_ATComand("AT+CSCLK=0");
			res = _readFeedback(1000, "OK");
				if(res == AT_OK)
					{
						ESP_LOGI(TAG,"I'M WAKE UP");
						return true;
					}
				else if(res == AT_ERROR) return false;
		}
	}
	return false;
}

bool isInit(int retry) {
  AT_flag res;
  while(retry--) {
    send_ATComand("AT");
    res = _readFeedback(1000, "OK");
    if(res == AT_OK) return true;
    else if(res == AT_ERROR) return false;
  }
  return false;
}

bool check_Registger_status(int retry)
{
	AT_flag res;
	char buff[10];
		while(retry --)
		{
			send_ATComand("AT+CENG?");
			res = _readFeedback(1000, "LTE NB-IOT");
			if(res == AT_OK)
			{
				filter_comma((char*)simcom_7090G.AT_buff, 2,3, buff);
				if(strstr(buff,"0")==false)
					return true;
			}
			else if(res == AT_ERROR)
				return false;
			vTaskDelay(1000/portTICK_PERIOD_MS);
		}
		return false;
}

bool isConnected_network(int retry)
{
	AT_flag res;
	if(!check_Registger_status(retry)) return false;
	while(retry--)
	{
		send_ATComand("AT+CNACT=0,1");
			res = _readFeedback(5000, "+APP PDP");
			if(res == AT_OK){
			if(strstr((char*)simcom_7090G.AT_buff,"DEACTIVE")) continue;
			else
				return true;
			}
			else if(res == AT_ERROR) return true;
	}
	return false;
}

bool mqtt_start(client clientMQTT,int keeptime,int cleanss, int Qos, int Retain, int retry)
{
	if(!isConnected_network(retry)) return false;
	AT_flag res;
	int count = 8;
	char buf[300];
	while(retry --)
	{
		while(count --)
		{
			if(count==7)
			{
				sprintf(buf,"AT+SMCONF=\"CLIENTID\",\"%s\"",clientMQTT.client_id);
				send_ATComand(buf);
				res = _readFeedback(1000, "OK");
				if(res == AT_ERROR) return false;
			}
			else if(count==6)
			{
				sprintf(buf,"AT+SMCONF=\"URL\",\"%s\",\"%d\"",clientMQTT.url,clientMQTT.port);
				send_ATComand(buf);
				res = _readFeedback(1000, "OK");
				if(res == AT_ERROR) return false;
			}
			else if(count==5)
			{
				sprintf(buf,"AT+SMCONF=\"USERNAME\",\"%s\"",clientMQTT.user_name);
				send_ATComand(buf);
				res = _readFeedback(1000, "OK");
				if(res == AT_ERROR ) return false;
			}
			else if(count==4)
			{
				sprintf(buf,"AT+SMCONF=\"PASSWORD\",\"%s\"",clientMQTT.password);
				send_ATComand(buf);
				res = _readFeedback(1000, "OK");
				if(res == AT_ERROR ) return false;
			}
			else if(count==3)
			{
				sprintf(buf,"AT+SMCONF=\"KEEPTIME\",\"%d\"",keeptime);
				send_ATComand(buf);
				res = _readFeedback(1000, "OK");
				if(res == AT_ERROR) return false;
			}
			else if(count==2)
			{
				sprintf(buf,"AT+SMCONF=\"CLEANSS\",\"%d\"",cleanss);
				send_ATComand(buf);
				res = _readFeedback(1000, "OK");
				if(res == AT_ERROR ) return false;
			}
			else if(count==1)
			{
				sprintf(buf,"AT+SMCONF=\"QOS\",\"%d\"",Qos);
				send_ATComand(buf);
				res = _readFeedback(1000, "OK");
				if(res == AT_ERROR ) return false;
			}
			else if(count==0)
			{
				sprintf(buf,"AT+SMCONF=\"RETAIN\",\"%d\"",Retain);
				send_ATComand(buf);
				res = _readFeedback(1000, "OK");
				if(res == AT_ERROR ) return false;
				else if(res == AT_OK) return true;
			}
		}

	}
	return false;
}
bool Connect_MQTT(int retry)
{
	AT_flag res;
		while(retry--)
		{
			send_ATComand("AT+SMCONN");
			res = _readFeedback(2*60000,"OK");
			if(res == AT_OK) return true;
			else if(res == AT_ERROR) return false;
		}
		return false;
}
bool Dis_Connect_MQTT(int retry)
{
	AT_flag res;
	while(retry--)
	{
		send_ATComand("AT+SMDISC");
		res = _readFeedback(1000,"OK");
		if(res == AT_OK) return true;
		else if(res == AT_ERROR) return false;
	}
	return false;
}
bool Is_Connect_MQTT(int retry)
{
		while(retry--)
		{
			send_ATComand("AT+SMSTATE?");
			if(WaitRestPond(1000) == true)
			{
				if(strstr((char*)simcom_7090G.AT_buff,"1")||strstr((char*)simcom_7090G.AT_buff,"2")) return true;
				else continue;
			}else continue;
		}
		return false;
}
bool MQTT_PUBLISH(char*topic,char* data,int retry,int retain,int Qos)
{
	AT_flag res;
	char buff[200];
	sprintf(buff,"AT+SMPUB=\"%s\",%d,%d,%d",topic,strlen(data)+1,Qos,retain);
	while(retry --)
	{
		send_ATComand(buff);
		send_ATComand(data);

		res = _readFeedback(1000,"OK");
		if(res== AT_OK) return true;
		else if(res == AT_ERROR) return false;
	}
	return false;
}


bool mqtt_subscribe(client clientMQTT, char *topic, int qos, int retry,  void (*mqttSubcribeCB)(char * data)) {
  AT_flag res;
  char buf[200];
  sprintf(buf, "AT+SMSUB=\"%s\",%d",topic, qos);
  while(retry--) {
    send_ATComand(buf);
    res = _readFeedback(3000, "OK");
    if(res == AT_OK)
    {
      simcom_7090G.mqtt_CB = mqttSubcribeCB;
      return true;
    }
    else if(res == AT_ERROR) return false;
  }
  return false;
}

bool get_RSRP_RSRQ_SINR_PCI_cellID(int retry)
{
	AT_flag res;
	while(retry --)
	{
		vTaskDelay(1000/portTICK_PERIOD_MS);
		send_ATComand("AT+CENG?");
		res = _readFeedback(1000, "LTE NB-IOT");
		char buff[10];
		if(res == AT_OK)
		{
			filter_comma((char*)simcom_7090G.AT_buff, 6,7, buff);
			if(buff[0]!=0)
			{
				filter_comma((char*)simcom_7090G.AT_buff, 6, 7, simcom_7090G.RSRP);
				filter_comma((char*)simcom_7090G.AT_buff, 8, 9, simcom_7090G.RSRQ);
				filter_comma((char*)simcom_7090G.AT_buff, 9, 10, simcom_7090G.SINR);
				filter_comma((char*)simcom_7090G.AT_buff, 5, 6, simcom_7090G.PCI);
				filter_comma((char*)simcom_7090G.AT_buff, 11, 12, simcom_7090G.cellID);
				return true;
			}
		}
		else if(res == AT_ERROR)
			return false;
	}
	return false;
}

bool GNSS_Power_ON(int retry)
{
	AT_flag res;
	while(retry --)
	{
		send_ATComand("AT+CGNSPWR=1");
		res = _readFeedback(1000, "OK");
		if(res == AT_OK)
		{
//			send_ATComand("AT+CGNSHOT");
//			res = _readFeedback(2000, "OK");
//			if(res == AT_OK){return true;}
//			else{}
			return true;
		}
		else if(res == AT_ERROR)
			return false;
	}
	return false;
}

bool GNSS_Power_OFF(int retry)
{
	AT_flag res;
	while(retry --)
	{
		send_ATComand("AT+CGNSPWR=0");
		res = _readFeedback(1000, "OK");
		if(res == AT_OK)
		{
//			send_ATComand("AT+CGNSHOT");
//			res = _readFeedback(2000, "OK");
//			if(res == AT_OK){return true;}
//			else{}
			return true;
		}
		else if(res == AT_ERROR)
			return false;
	}
	return false;
}

bool get_latitude_longtitude(int retry)
{
	AT_flag res;
		while(retry --)
			{
				send_ATComand("AT+CGNSINF");
				res = _readFeedback(5000, "+CGNSINF");
				char buff[60];
				if(res == AT_OK)
				{
					filter_comma((char*)simcom_7090G.AT_buff, 3, 4,buff);
					ESP_LOGI("CHECK","DATA_BUFF: %s",buff);
					if(strstr(buff,"0.000000") || buff[0]==0)
					{
						vTaskDelay(2000/portTICK_PERIOD_MS);
					}
					else
					{
						filter_comma((char*)simcom_7090G.AT_buff, 3, 4,simcom_7090G.Latitude);
						filter_comma((char*)simcom_7090G.AT_buff, 4, 5,simcom_7090G.Longitude);
						return true;
					}
				}
				else if(res == AT_ERROR)
					return false;
			}

	return false;
}
