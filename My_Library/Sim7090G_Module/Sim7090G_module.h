#ifndef SIM7090G_MODULE_H
#define SIM7090G_MODULE_H

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <math.h>
#include <sys/time.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"

#define BUF_SIZE (1024 * 2)

typedef struct simcom_t
{
  char RSRP[10];
  char RSRQ[10];
  char SINR[10];
  char PCI[10];
  char cellID[10];
  char Latitude[20];
  char Longitude[20];
  bool AT_buff_avai;
  uint8_t AT_buff[BUF_SIZE];
  void (*mqtt_CB)(char * data);
}simcom;
extern simcom simcom_7090G;

typedef struct client_t
{
   char client_id[50];
   char url[51];
   char user_name[50];
   char password[50];
   int port;

}client;

typedef enum
{
AT_OK,
 AT_ERROR,
 AT_TIMEOUT,
}AT_flag;
//Khởi tạo giao tiếp giữa ESP32 và SIM7090G
void init_simcom(uart_port_t uart_num, int tx_io_num, int rx_io_num, int baud_rate);
//Bật module Sim7090G
bool Power_on(gpio_num_t Pin);
//Đưa module Sim vào chế độ Sleep mode
bool Sleep_mode(bool Mode,gpio_num_t Pin_control,int retry);
//Kiểm tra giao tiếp AT command
bool isInit(int retry);
//Kiểm tra đăng kí mạng
bool check_Registger_status(int retry);
//Kiểm tra kết nối mạng
bool isConnected_network(int retry);
//Kiểm tra đã đẩy thông tin client lên thiết bị
bool mqtt_start(client clientMQTT,int keeptime,int cleanss, int Qos, int Retain, int retry);
//Kết nối MQTT
bool Connect_MQTT(int retry);
//Ngắt kết nối MQTT
bool Dis_Connect_MQTT(int retry);
//Kiểm tra đã kết nối MQTT hay chưa
bool Is_Connect_MQTT(int retry);
//Gửi bản tin vào topic
bool MQTT_PUBLISH(char*topic,char* data,int retry,int retain,int Qos);
//đăng kí topic
bool mqtt_subscribe(client clientMQTT, char *topic, int qos, int retry,  void (*mqttSubcribeCB)(char * data));
//lấy dữ liệu đầu tiên
bool get_RSRP_RSRQ_SINR_PCI_cellID(int retry);
bool GNSS_Power_ON(int retry); //bật GNSS
bool GNSS_Power_OFF(int retry); //tắt GNSS
//lấy dữ liệu thứ 2
bool get_latitude_longtitude(int retry);

#endif /* SIM7090G_MODULE_H */
