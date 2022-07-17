#include "sensors.h"

// main execution loop
void task_bme280_bma220()
{

    struct bme280_t bme280 = {
        .bus_write = BME280_I2C_bus_write,
        .bus_read = BME280_I2C_bus_read,
        .dev_addr = BME280_I2C_ADDRESS2,
        .delay_msec = BME280_delay_msek};

    s32 com_rslt;
    s32 v_uncomp_pressure_s32;
    s32 v_uncomp_temperature_s32;
    s32 v_uncomp_humidity_s32;

    // acceleration values
    int8_t x_val = 0;
    int8_t y_val = 0;
    int8_t z_val = 0;

    com_rslt = bme280_init(&bme280);

    com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
    com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
    com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

    com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
    com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);

    com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);
    if (com_rslt == SUCCESS)
    {

        wake_BMA220();
        int loop_cnt = 0;
        while (loop_cnt < 10)
        {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            loop_cnt++;
            com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
                &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

            if (com_rslt == SUCCESS)
            {
                // update
                BMA220_getAcc(BMA220_SENSOR_GETX, &x_val);
                BMA220_getAcc(BMA220_SENSOR_GETY, &y_val);
                BMA220_getAcc(BMA220_SENSOR_GETZ, &z_val);

                // ESP_LOGI(TAG_BME280, "%d x / %d y / % z",x_val,y_val,z_val);

                ESP_LOGI("combined sensors", "%.2f degC / %.3f hPa / %.3f / %d x / %d y / %d z %%",
                         bme280_compensate_temperature_double(v_uncomp_temperature_s32),
                         bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100, // Pa -> hPa
                         bme280_compensate_humidity_double(v_uncomp_humidity_s32), x_val, y_val, z_val);
            }
            else
            {
                ESP_LOGE(TAG_BME280, "measure error. code: %d", com_rslt);
            }
        }
    }
    else
    {
        ESP_LOGE(TAG_BME280, "init or setting error. code: %d", com_rslt);
    }

    // set both sensors to sleep
    ESP_LOGI("INFO:", "Setting BME280 to sleep!");
    s32 sleep_status;
    sleep_status = bme280_set_power_mode(BME280_SLEEP_MODE);
    if (sleep_status == SUCCESS)
        ESP_LOGI("INFO:", "BME280 Sleep Successful!");
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
