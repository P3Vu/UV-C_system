/**
  ******************************************************************************
  * @file       main.c
  * @author     Pawel Wojciechowski
  * @version    V.1.0.0
  * @date       29-November-2020
  ******************************************************************************
  */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_tls.h"
#include "esp_http_client.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "my_i2c.h"
#include "my_http.h"
#include "my_JSON.h"
#include "MAX44009.h"
#include "ML8511.h"
#include "my_SNTP.h"
#include "tcpip_adapter.h"

#define LAMP1                       GPIO_NUM_2                    /* RELAY IN1 on board */
#define LAMP2                       GPIO_NUM_0                    /* RELAY IN2 on board */
#define LAMP3                       GPIO_NUM_18                   /* RELAY IN3 on board */
#define LAMP4                       GPIO_NUM_19                   /* RELAY IN4 on board */
#define MOTION_SENSOR               GPIO_NUM_13                   /* Microwave Motion Sensor */

#define GPIO_OUTPUT_PIN_SEL         ((1ULL<<GPIO_NUM_2) |   \
                                    (1ULL<<GPIO_NUM_0) |    \
                                    (1ULL<<GPIO_NUM_18)|    \
                                    (1ULL<<GPIO_NUM_19))

#define GPIO_INPUT_PIN_SEL          1ULL<<MOTION_SENSOR
#define ESP_INTR_FLAG_DEFAULT       0

#define LAMP_ON      0
#define LAMP_OFF     1

/* WiFi Config */
#define EXAMPLE_ESP_WIFI_SSID               "UPC8716827"
#define EXAMPLE_ESP_WIFI_PASS               "bvyhjy3jbdbC"
#define EXAMPLE_ESP_MAXIMUM_RETRY           1

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static xQueueHandle gpio_evt_queue      = NULL;
static const char   *TAG_MODULE           = "Module";
static const char   *TAG_WIFI             = "WiFi";


char HTTP_BUFFER[1024] = {0};
static int s_retry_num = 0;
time_t now;
struct tm timeinfo;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

#define number_of_days  7
#define number_of_time_sections  24
#define ID 2

enum week {Mon, Tue, Wed, Thur, Fri, Sat, Sun};
bool Calendar[number_of_days][number_of_time_sections] = {};

/*
Example with 24 time sections
                                    Days
                 0       1       2       3      4       5       6
                Mon     Tue     Wed     Thu    Fri     Sat     Sun
Hours
00:00 - 00:59    0    ...
01:00 - 01:59    1    ...
 ...
22:00 - 22:59    1    ...
23:00 - 23:59    0    ...

*/

struct Time {
    int     week_day;
    int     date_day;
    int     date_month;
    int     date_year;
    int     hours;
    int     minutes;
    int     seconds;
};

/** *************************************************************** */

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

/** Configure GPIO's for relay switch */
static void configGPIO(void){

    gpio_config_t io_conf;

    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that want to set
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins that will be inputs
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "Motion_Task", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(MOTION_SENSOR, gpio_isr_handler, (void*) MOTION_SENSOR);
    // remove isr handler for gpio number
    gpio_isr_handler_remove(MOTION_SENSOR);
    // hook isr handler for specific gpio pin again
    gpio_isr_handler_add(MOTION_SENSOR, gpio_isr_handler, (void*) MOTION_SENSOR);

    ESP_LOGI(TAG_MODULE, "GPIO Config finished.");
}
/** Turn On the Lamp */
static void TurnOnLamp(int gpioNum){
    int err = 0;
    err = gpio_set_level(gpioNum, LAMP_ON);
    if(err != ESP_OK) ESP_LOGW(TAG_MODULE, "Fail to set Lamp On at GPIO: %d", gpioNum);

    return;
}

/** Turn Off the Lamp */
static void TurnOffLamp(int gpioNum){
    int err = 0;
    err = gpio_set_level(gpioNum, LAMP_OFF);
    if(err != ESP_OK) ESP_LOGW(TAG_MODULE, "Fail to set Lamp Off at GPIO: %d", gpioNum);

    return;
}

/** Configures the ADC peripheral for UV sensor */
static void configADC(void){
    esp_err_t err;

    err = adc1_config_width(ADC_WIDTH_BIT_12);
    if(err != ESP_OK){
        ESP_LOGW(TAG_MODULE, "Fail to config ADC Width");
        return;
    }

    err = adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    if(err != ESP_OK){
        ESP_LOGW(TAG_MODULE, "Fail to config ADC channel attenuation");
    }

    ESP_LOGI(TAG_MODULE, "ADC Config finished.");
    return;
}

/** **************************** WIFI **************************** */
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG_WIFI, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG_WIFI,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_WIFI, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init()
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies module will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG_WIFI, "wifi_init finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_WIFI, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG_WIFI, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG_WIFI, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}



/** *************************************************************** */

/** Print the database */
void printCalendar()
{
    int day = Mon, section = 0;

    printf(" Hours | Mon | Tue | Wed | Thu | Fri | Sat | Sun |\n");
    for(section = 0; section < number_of_time_sections; section++){
        bool buf[7] = {};
        for(day = Mon; day <= Sun; day++){
            if(Calendar[day][section] == 1) buf[day] = 1;
        }
        if(section < 9)
            printf("  %d:%d  |  %d  |  %d  |  %d  |  %d  |  %d  |  %d  |  %d  |\n", section, section+1,
                   buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

        else if(section == 9)
            printf("  %d:%d |  %d  |  %d  |  %d  |  %d  |  %d  |  %d  |  %d  |\n", section, section+1,
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

        else if(section > 9)
            printf(" %d:%d |  %d  |  %d  |  %d  |  %d  |  %d  |  %d  |  %d  |\n", section, section+1,
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);
    }

    printf("\n");
}


/** Parses time */
void parseTime(struct Time *ctime, char *strftime_buf){

    // strftime_buf is like Fri Jan 1 19:53:49 2021

    int cntr = 0;
    char signs[] = " :";
    char elements[7][5] = {};       // 7 elements with max 5 characters
    char *buf;
    buf = strtok(strftime_buf, signs);

    while(buf != NULL)
    {
        //printf("%s\n", buf);
        strcpy(elements[cntr], buf);
        //printf("Elements is %s\n", elements[cntr]);
        buf = strtok(NULL, signs);
        cntr++;
    }

    /* Parse the week day */
    if(!strcmp("Mon", elements[0])) ctime->week_day = Mon;
    else if(!strcmp("Tue", elements[0])) ctime->week_day = Tue;
    else if(!strcmp("Wed", elements[0])) ctime->week_day = Wed;
    else if(!strcmp("Thu", elements[0])) ctime->week_day = Thur;
    else if(!strcmp("Fri", elements[0])) ctime->week_day = Fri;
    else if(!strcmp("Sat", elements[0])) ctime->week_day = Sat;
    else if(!strcmp("Sun", elements[0])) ctime->week_day = Sun;
    else ctime->week_day = -1;

    /* Parse the month */
    if (!strcmp("Jan", elements[1])) ctime->date_month = 1;
    else if (!strcmp("Feb", elements[1])) ctime->date_month = 2;
    else if (!strcmp("Mar", elements[1])) ctime->date_month = 3;
    else if (!strcmp("Apr", elements[1])) ctime->date_month = 4;
    else if (!strcmp("May", elements[1])) ctime->date_month = 5;
    else if (!strcmp("Jun", elements[1])) ctime->date_month = 6;
    else if (!strcmp("Jal", elements[1])) ctime->date_month = 7;
    else if (!strcmp("Aug", elements[1])) ctime->date_month = 8;
    else if (!strcmp("Sep", elements[1])) ctime->date_month = 9;
    else if (!strcmp("Oct", elements[1])) ctime->date_month = 10;
    else if (!strcmp("Nov", elements[1])) ctime->date_month = 11;
    else if (!strcmp("Dec", elements[1])) ctime->date_month = 12;
    else ctime->date_month = -1;

    /* Parse the rest of parameters */
    ctime->date_day     = atoi(elements[2]);
    ctime->hours        = atoi(elements[3]);
    ctime->minutes      = atoi(elements[4]);
    ctime->seconds      = atoi(elements[5]);
    ctime->date_year    = atoi(elements[6]);

    printf("%d %d %d %d %d %d %d\n", ctime->week_day, ctime->date_month,
           ctime->date_day, ctime->hours, ctime->minutes, ctime->seconds, ctime->date_year);


    return;
}

/** Set whole calendar to OFF state */
void CleanCalendar(void){
    int day, ts;

    for(day = Mon; day <= Sun; day++){
        for(ts = 0; ts <= 23; ts++ ){
            Calendar[day][ts] = 0;
        }
    }
}

/** Changes time string from format from 'Tue Jan 5 9:57:33 2021 to YYYY-MM-DD' */
void ChangeTimeFormat(char *destination, struct Time ctime){

    char date[12];

    char buf1[15] = {};
    sprintf(buf1, "%d", ctime.date_year);
    strcpy(date, buf1);

    sprintf(date+4,"%s", "-");

    char buf2[15] = {};
    if(ctime.date_month < 10) sprintf(buf2, "0%d", ctime.date_month);
    else sprintf(buf2, "%d", ctime.date_month);
    strcpy(date+5, buf2);

    sprintf(date+7,"%s", "-");

    char buf3[15] = {};
    if(ctime.date_day < 10) sprintf(buf3, "0%d", ctime.date_day);
    else sprintf(buf3, "%d", ctime.date_day);
    strcpy(date+8, buf3);

    strcpy(destination, date);
    return;
}

void http_task(void *pvParameters)
{
    char strftime_buf[64];
    char dtbTimeFormat[12];
    struct Time ctime;

    ESP_LOGI(TAG_HTTP, "Reading from database..");

    /* Update database info - read to buffer in JSON format at first*/
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    parseTime(&ctime, strftime_buf);

    ChangeTimeFormat(dtbTimeFormat, ctime);

    read_database(HTTP_BUFFER, dtbTimeFormat, ID);
    /* Parse JSON buffer and update Calendar */
// The semaphore is necessary here probably
    CleanCalendar();
    UpdateCalendar(HTTP_BUFFER, Calendar, ID);
    printCalendar();
//

// Give a signal that Calendar was updated //

    /* Wait */
    //vTaskDelay(300*1000 / portTICK_PERIOD_MS);  // 300secs = 5 minutes

    ESP_LOGI(TAG_HTTP, "Finish http example");
    vTaskDelete(NULL);
}

/** *************************************************************** */

void app_main()
{
    esp_log_level_set("*", ESP_LOG_VERBOSE);

    /* Basic hardware init */
    configGPIO();
    configADC();
    i2c_init();

    esp_err_t err = nvs_flash_init();
    if(err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* WiFi initial */
    ESP_LOGI(TAG_WIFI, "ESP_WIFI_MODE_STA");
    wifi_init();

    /* Get actual time over SNTP */
    time(&now);
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (2020 - 1900).
    if (timeinfo.tm_year < (2020 - 1900)) {
        ESP_LOGI(SNTP_TAG, "Time is not set yet. Getting time over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }

    char strftime_buf[64];

    // Set time
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(SNTP_TAG, "The current date/time is: %s", strftime_buf);

    printf("Region time is %s \n", strftime_buf);

    struct Time ctime = {
        .week_day = -1,
        .hours = -1
    };

    parseTime(&ctime, strftime_buf);

    //printCalendar();
    if(ctime.week_day > -1 && ctime.hours > -1
       && Calendar[ctime.week_day][ctime.hours] == 1){
           printf("Zapalam Lampy\n");
           //TurnOnLamp();
       }
    else{
        printf("Zgaszam Lampy\n");
        //TurnOffLamp();
    }

    // Napisac task ktory bedzie odpowiadal za zmiane stanu lamp - obslugiwal flage z http
    // ktora bedzie informowac o updejcie kalendarza.
    // Luiza nie jest w stanie wyslac do mnie informacji kiedy nastapi zmiana wiec nie ma mechanizmu przerwan

    xTaskCreate(&http_task, "http_test_task", 8192, NULL, 5, NULL);
    vTaskDelay(3*1000 / portTICK_PERIOD_MS);

    //printf("HTTP_BUFFER = %s\n", HTTP_BUFFER);
    UpdateCalendar(HTTP_BUFFER, Calendar, ID);
    //printCalendar();


    if(ctime.week_day > -1 && ctime.hours > -1 && Calendar[ctime.week_day][ctime.hours] == 1){
           printf("Zapalam Lampy\n");
           //TurnOnLamp();
       }
    else{
        printf("Zgaszam Lampy\n");
        //TurnOffLamp();
    }

    while(1){
        time(&now);
        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        ESP_LOGI(SNTP_TAG, "The current date/time is: %s", strftime_buf);
        vTaskDelay(3*1000 / portTICK_PERIOD_MS);
    }
}



