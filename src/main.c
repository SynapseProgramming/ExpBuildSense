#include "sensors.h"

// main execution loop
void task_bme280_bma220()
{

    // acceleration values
    int8_t x_val = 0;
    int8_t y_val = 0;
    int8_t z_val = 0;
    double temp = 0;
    double humidity = 0;
    double pressure = 0;

    // get env values
    BME280_get(&temp, &pressure, &humidity);


    wake_BMA220();

    // update
    BMA220_getAcc(BMA220_SENSOR_GETX, &x_val);
    BMA220_getAcc(BMA220_SENSOR_GETY, &y_val);
    BMA220_getAcc(BMA220_SENSOR_GETZ, &z_val);

    ESP_LOGI("combined sensors", "%.2f degC / %.3f hPa / %.3f / %d x / %d y / %d z %%",
             temp,
             pressure, // Pa -> hPa
             humidity, x_val, y_val, z_val);

    // set both sensors to sleep
    sleep_BMA220();
    ESP_LOGI("BMA220:", "Set to suspend mode!");
}

void task_sound()
{

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
    // Calculate ADC characteristics i.e. gain and offset factors
    esp_adc_cal_characteristics_t characteristics;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &characteristics);

    uint32_t val = adc1_get_raw(ADC1_CHANNEL_4);
    uint32_t voltage = esp_adc_cal_raw_to_voltage(val, &characteristics);
    ESP_LOGI("current", "ADC in mV %d", voltage);
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(init_BMA220());
    ESP_LOGI(TAG, "I2C initialized successfully");
    ESP_LOGI("INFO:", "Just Woke Up!");

    esp_sleep_enable_timer_wakeup(5e6);
    task_sound();
    task_bme280_bma220();
    esp_deep_sleep_start();
}
