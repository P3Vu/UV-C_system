/**
  ******************************************************************************
  * @file       my_http.h
  * @author     Pawel Wojciechowski
  * @version    V.1.0.0
  * @date       9-December-2020
  ******************************************************************************
  */

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 16384

extern const char howsmyssl_com_root_cert_pem_start[] asm("_binary_howsmyssl_com_root_cert_pem_start");
extern const char howsmyssl_com_root_cert_pem_end[]   asm("_binary_howsmyssl_com_root_cert_pem_end");

const char *TAG_HTTP;

int read_database(char *buf, char *date, int building, int room);
esp_err_t ScheduleInterrupted(int room, int building);
esp_err_t CheckBlockStatus(char *buf, int building, int room);
esp_err_t CheckLightSwitch(char *buf, int building, int room);
