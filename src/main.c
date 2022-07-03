
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
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &characteristics);

    while (1)
    {

        uint32_t val = adc1_get_raw(ADC1_CHANNEL_4);
        uint32_t  voltage = esp_adc_cal_raw_to_voltage(val, &characteristics);
        ESP_LOGI("current", "ADC in mV %d", voltage);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}