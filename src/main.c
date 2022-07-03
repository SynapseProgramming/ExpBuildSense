/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#define V_REF 1700

// ADC1_CHANNEL_4 is pin 32

void app_main(void)
{

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);

    // Calculate ADC characteristics i.e. gain and offset factors
    esp_adc_cal_characteristics_t characteristics;
    esp_adc_cal_characterize(ADC1_CHANNEL_4,ADC_ATTEN_DB_11,ADC_WIDTH_BIT_12,V_REF,&characteristics);

    while (1)
    {

       // int val = adc1_get_raw(ADC1_CHANNEL_0);
        uint32_t  voltage = esp_adc_cal_raw_to_voltage(ADC1_CHANNEL_4, &characteristics);
        ESP_LOGI("LOL", "ADC value %d", voltage);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}