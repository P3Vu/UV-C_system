/**
  ******************************************************************************
  * @file       my_http.h
  * @author     Pawel Wojciechowski
  * @version    V.1.0.0
  * @date       9-December-2020
  ******************************************************************************
  */

  #include "esp_err.h"
  #include "esp_http_client.h"
  #include "esp_log.h"
  #include "my_http.h"
  #include "esp_tls.h"
  #include "my_JSON.h"

  const char *TAG_HTTP = "HTTP_CLIENT";

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    static char *output_buffer;  // Buffer to store response of http request from event handler
    static int output_len;       // Stores number of bytes read
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG_HTTP, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG_HTTP, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG_HTTP, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG_HTTP, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG_HTTP, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            //printf("%s HTTP_EVENT_ON_DATA, len = %d, chunked = %d\n", TAG_HTTP, evt->data_len, esp_http_client_is_chunked_response(evt->client));
            /*
             *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
             *  However, event handler can also be used in case chunked encoding is used.
             */
            //if (!esp_http_client_is_chunked_response(evt->client)) {
                // If user_data buffer is configured, copy the response into the buffer
                if (evt->user_data) {
                    memcpy(evt->user_data + output_len, evt->data, evt->data_len);
                    //printf("%.*s", evt->data_len, (char*)evt->data);
                } else {
                    if (output_buffer == NULL) {
                        output_buffer = (char *) malloc(esp_http_client_get_content_length(evt->client));
                        output_len = 0;
                        if (output_buffer == NULL) {
                            ESP_LOGE(TAG_HTTP, "Failed to allocate memory for output buffer");
                            return ESP_FAIL;
                        }
                    }
                    memcpy(output_buffer + output_len, evt->data, evt->data_len);
                }

                output_len += evt->data_len;
            //}

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG_HTTP, "HTTP_EVENT_ON_FINISH");
            if (output_buffer != NULL) {
                // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
                //ESP_LOG_BUFFER_HEX(TAG_HTTP, output_buffer, output_len);
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_DISCONNECTED:
            if(output_buffer != NULL) printf("output_buffer = %s\n", output_buffer);
            ESP_LOGI(TAG_HTTP, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error(evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                if (output_buffer != NULL) {
                    free(output_buffer);
                    output_buffer = NULL;
                }
                output_len = 0;
                ESP_LOGI(TAG_HTTP, "Last esp error code: 0x%x", err);
                ESP_LOGI(TAG_HTTP, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            break;
    }
    return ESP_OK;
}

/** GET - read schedule from database */
int read_database(char *buf, char *date, int building, int room)
{
    //char url[200] = "http://192.168.0.129/json.php?";                 /* Example for local RPi */
    char url[200] = "http://sterowanieuv.000webhostapp.com/json.php?";
    char urlargs[100] = {};

    sprintf(urlargs, "room=%d&date=%s&building=%d", room, date, building);
    strcat(url, urlargs);

    char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};
    esp_http_client_config_t config = {
        .url = url,
        .event_handler = _http_event_handler,
        .user_data = local_response_buffer,         // Pass address of local buffer to get response
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    // GET
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        ESP_LOGI(TAG_HTTP, "HTTP GET Status = %d, content_length = %d, chunked = %d, transport_type = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client),
                esp_http_client_is_chunked_response(client),
                esp_http_client_get_transport_type(client));
    }
    else {
        //ESP_LOGE(TAG_HTTP, "HTTP GET request failed: %s", esp_err_to_name(err));
        ESP_LOGW(TAG_HTTP, "HTTP GET request failed: %s", esp_err_to_name(err));
        return err;
    }

    //printf("\nbuffer = %s\n", local_response_buffer);
    /* Parse the string, copy everything between <body> .. </body> without whitespaces*/
    if(err == ESP_OK && local_response_buffer != NULL){
        char *strPtr = strstr(local_response_buffer, "<body>") + 6;
        //while(isspace(*strPtr)) strPtr++;
        char *endPtr = strstr(local_response_buffer, "</body");
        //endPtr -= 4;
        strncpy(buf, strPtr, (endPtr - strPtr)/sizeof(char));
        //printf("buf = %s\n", buf);
    }

    //printf("buf = %s\n", buf);
    esp_http_client_cleanup(client);

    return err;
}

/** GET - Checks if user took down the block (which was cause by sensor) by clicking switch on web */
esp_err_t CheckBlockStatus(char *buf, int building, int room){

    char url[200] = "http://sterowanieuv.000webhostapp.com/info.php?";
    char urlargs[100] = {};

    sprintf(urlargs, "room_id=%d&building_id=%d", room, building);
    strcat(url, urlargs);

    char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};

    esp_http_client_config_t config = {
    .url = url,
    .event_handler = _http_event_handler,
    .user_data = local_response_buffer,         // Pass address of local buffer to get response
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    // GET
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        ESP_LOGI(TAG_HTTP, "HTTP GET Status = %d, content_length = %d, chunked = %d, transport_type = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client),
                esp_http_client_is_chunked_response(client),
                esp_http_client_get_transport_type(client));
    }
    else {
        //ESP_LOGE(TAG_HTTP, "HTTP GET request failed: %s", esp_err_to_name(err));
        ESP_LOGW(TAG_HTTP, "HTTP GET request failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Parse the string, copy everything between <body> .. </body> without whitespaces*/
    if(err == ESP_OK && local_response_buffer != NULL){
        char *strPtr = strstr(local_response_buffer, "<body>") + 6;
        while(isspace(*strPtr)) strPtr++;
        char *endPtr = strstr(local_response_buffer, "</body>");
        endPtr -= 4;
        strncpy(buf, strPtr, (endPtr - strPtr)/sizeof(char));
        //printf("buf = %s\n", buf);
    }

    //printf("buf = %s\n", buf);
    esp_http_client_cleanup(client);

    return err;
}

/** GET - Checks if user switched off the lights */
esp_err_t CheckLightSwitch(char *buf, int building, int room){

    char url[200] = "http://sterowanieuv.000webhostapp.com/przerwa.php?";
    char urlargs[100] = {};

    sprintf(urlargs, "room=%d&building=%d", room, building);
    strcat(url, urlargs);

    char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};

    esp_http_client_config_t config = {
    .url = url,
    .event_handler = _http_event_handler,
    .user_data = local_response_buffer,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    // GET
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        ESP_LOGI(TAG_HTTP, "HTTP GET Status = %d, content_length = %d, chunked = %d, transport_type = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client),
                esp_http_client_is_chunked_response(client),
                esp_http_client_get_transport_type(client));
    }
    else {
        //ESP_LOGE(TAG_HTTP, "HTTP GET request failed: %s", esp_err_to_name(err));
        ESP_LOGW(TAG_HTTP, "HTTP GET request failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Parse the string, copy everything between <body> .. </body> without whitespaces*/
    if(err == ESP_OK && local_response_buffer != NULL){
        char *strPtr = strstr(local_response_buffer, "<body>") + 6;
        while(isspace(*strPtr)) strPtr++;
        char *endPtr = strstr(local_response_buffer, "</body>");
        endPtr -= 4;
        strncpy(buf, strPtr, (endPtr - strPtr)/sizeof(char));
        //printf("buf = %s\n", buf);
    }

    //printf("buf = %s\n", buf);
    esp_http_client_cleanup(client);

    return err;
}

/** Makes POST to database to inform that UV schedule was interruted */
esp_err_t ScheduleInterrupted(int room, int building){

    char url[200] = "http://sterowanieuv.000webhostapp.com/stop.php?";
    char urlargs[100] = {};

    sprintf(urlargs, "room_id=%d&building_id=%d", room, building);
    strcat(url, urlargs);

    char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};

    esp_http_client_config_t config = {
        .url = url,
        .event_handler = _http_event_handler,
        .user_data = local_response_buffer,         // Pass address of local buffer to get response
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    const char *post_data = "{\"UV\": 0}";

    esp_http_client_set_url(client, url);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        ESP_LOGI(TAG_HTTP, "HTTP POST Status = %d, content_length = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG_HTTP, "HTTP POST request failed: %s", esp_err_to_name(err));
    }

    return err;
}



