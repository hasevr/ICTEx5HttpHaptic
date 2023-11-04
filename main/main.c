/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include "sdkconfig.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_chip_info.h"
#include "spi_flash_mmap.h"
#include "esp_flash.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include <esp_system.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <rom/uart.h>
#include <driver/uart.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/mcpwm_prelude.h>
#include "driver/gpio.h"
#include "bdc_motor.h"
#include "esp_timer.h"
#include <math.h>
#include <cJSON.h>

const char* TAG = "main";

/* Constants that aren't configurable in menuconfig */
#define WEB_SERVER "192.168.12.179"
#define WEB_PORT "50000"

int adValue=0;
struct timeval timeReadAdc;
double freqValue=200.0;

static void http_get_task(void *pvParameters)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[64];

    while(1) {
        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);
        static const char *REQ1 = "GET /getadc";
        static const char *REQ2 = " HTTP/1.0\r\n"
            "Host: "WEB_SERVER":"WEB_PORT"\r\n"
            "User-Agent: esp-idf/1.0 esp32\r\n\r\n";
        static char REQUEST[2048];
        strcpy(REQUEST, REQ1);
        double timeDouble = timeReadAdc.tv_sec + timeReadAdc.tv_usec * 0.000001;
        sprintf(REQUEST + strlen(REQUEST), "?ADC=%d&TIME=%lf", adValue, timeDouble);
        strcat(REQUEST, REQ2);
        if (write(s, REQUEST, strlen(REQUEST)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");

        /* Read HTTP response */
        char buf[1024];
        buf[0] = '\0';
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
            strcat(buf, recv_buf);
        } while(r > 0);
        cJSON *root = NULL;
        char* pHead;

        for (pHead=buf; *pHead != '\0'&& *pHead != '{'; pHead++) ;
        if (*pHead == '{') {
            //   printf("Buffer stripped %s\n", pHead);
            root=cJSON_Parse(pHead);
            if (root == NULL) {
               ESP_LOGE(TAG, "... received wrong json format");
               continue;
            }
            if (cJSON_GetObjectItem(root, "freq")==NULL) {
               ESP_LOGE(TAG, "... freq not found");
               continue;
            }
            //    printf("freq: %s\n", cJSON_Print(cJSON_GetObjectItem(root, "freq")));
            freqValue=atof(cJSON_Print(cJSON_GetObjectItem(root, "freq")));
            printf("received freqValue = %lf\n", freqValue);
            //  check the received freqValue
            if (freqValue < 100) freqValue=100;
            else if (freqValue > 500) freqValue=500;
            printf(" modified freqValue = %lf\n", freqValue);
        }

        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
        close(s);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


//#define USE_TIMER   //  Whther use the timer or not. Without this definition, the function is called from a normal task.

#ifdef USE_TIMER
# define DT 0.0001  //  In the case of the timer, the minimum period is 50 micro second.
#else
# define DT (1.0/configTICK_RATE_HZ)  
                    //  In the case of the task, the since period is the since slice of the OS specified in menuconfig,
                    //  which is set to 1 ms=1 kHz.  
#endif

//  ADC setting
static adc_oneshot_unit_handle_t adc1_handle;
//  PWM control for bdc_motor
bdc_motor_handle_t motor = NULL;
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks

struct WaveParam{
    const double damp[3];
    const int nDamp;
    const double amplitude;
} wave = {
    .damp = {-2, -5, -10},
    .nDamp = sizeof(wave.damp) / sizeof(wave.damp[0]),
    .amplitude = 2,
};  
int count = 0;
double since = -1;  //  time passed since click the pressure sensor. 


void hapticFunc(void* arg){
    const char* TAG = "H_FUNC";
    static int i;               //  An integer to select waveform. 
    static double omega = 0;    //  angular frequency
    static double B=0;          //  damping coefficient
    gettimeofday(&timeReadAdc, NULL);
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &adValue);
    if (adValue < 2100 && since > 0.3){
        since = -1;
        printf("\r\n");
    }
    if (adValue > 2400 && since == -1){   //  When the button is pushed after finishing to output an wave.
        //  set the since to 0 and update the waveform parameters.
        since = 0;
        omega = freqValue * M_PI * 2;
        B = wave.damp[i];
        printf("Wave: %3.1fHz, A=%2.2f, B=%3.1f ", omega/(M_PI*2), wave.amplitude, B);
        i++;
        if (i >= wave.nDamp) i = 0;
    }
    //  Output the wave
    double pwm = 0;
    if (since >= 0){
        pwm = wave.amplitude * cos (omega * since) * exp(B*since);
        since += DT;
    }else{
        pwm = 0;
    }
    //  Rotating direction
    if (pwm > 0){
        bdc_motor_forward(motor);
#       ifndef USE_TIMER
        if (since >= 0) printf("+");
#       endif
    }else{
        bdc_motor_reverse(motor);
        pwm = -pwm;
#       ifndef USE_TIMER
        if (since >= 0) printf("-");
#       endif
    }
    if (pwm > 1) pwm = 1;    
    //  Set pwm duty rate
    unsigned int speed = pwm * BDC_MCPWM_DUTY_TICK_MAX;
    bdc_motor_set_speed(motor, speed);
    count ++;
    if (count >= 1000 ){
        ESP_LOGI(TAG, "ADC:%d", adValue);
        count = 0;
    }
}

#ifndef USE_TIMER
void hapticTask(void* arg){
    while(1){
        hapticFunc(arg);
        vTaskDelay(1);
    }
}
#endif

void app_main(void)
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    //----------------------------------
    printf("!!! Active Haptic Feedback Start !!!\n");
    
    ESP_LOGI("main", "Initialize ADC");
    static adc_oneshot_unit_init_cfg_t adc_init_config1 = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config1, &adc1_handle));
    static adc_oneshot_chan_cfg_t adc1_chan6_cfg = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &adc1_chan6_cfg));

    ESP_LOGI(TAG, "Initialize GPIO");
    gpio_config_t gpio_conf = {
        .pin_bit_mask = 1 << 16,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&gpio_conf);
    gpio_set_level(GPIO_NUM_16, 1);

    ESP_LOGI(TAG, "Initialize PWM");
    bdc_motor_config_t motor_config = {
        .pwma_gpio_num = GPIO_NUM_5,
        .pwmb_gpio_num = GPIO_NUM_17,
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    ESP_ERROR_CHECK(bdc_motor_enable(motor));

#ifdef USE_TIMER
    esp_timer_create_args_t timerDesc={
        callback: hapticFunc,
        arg: NULL,
        dispatch_method: ESP_TIMER_TASK,        
        name: "haptic",
        skip_unhandled_events: true
    };
    esp_timer_handle_t timerHandle = NULL;
    esp_timer_create(&timerDesc, &timerHandle);
    esp_timer_start_periodic(timerHandle, (int)(1000*1000*DT));     // period in micro second (100uS=10kHz)
#else
    TaskHandle_t taskHandle = NULL;
    xTaskCreate(hapticTask, "Haptic", 1024 * 10, NULL, 6, &taskHandle);
#endif

    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    xTaskCreate(&http_get_task, "http_get_task", 4096, NULL, 5, NULL);

    uart_driver_install(UART_NUM_0, 1024, 1024, 10, NULL, 0);
    while(1){
        uint8_t ch;
        uart_read_bytes(UART_NUM_0, &ch, 1, portMAX_DELAY);
        printf("'%c' received.\r\n", ch);
        switch(ch){
            case 'a':
            //  do something
            break;
        }
    }
}
