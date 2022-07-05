
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "bme280.h"

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO 26        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 27        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define BMA220_SENSOR_ADDR 0x0A /*!< Slave address of the BMA220 sensor */
#define BMA220_SENSOR_GETX 0x04
#define BMA220_SENSOR_GETY 0x06
#define BMA220_SENSOR_GETZ 0x08

#define SDA_PIN I2C_MASTER_SDA_IO
#define SCL_PIN I2C_MASTER_SCL_IO

#define TAG_BME280 "BME280"
#define BLINK_GPIO 14

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

// function to initialise the BMA220 accelerometer sensor
static esp_err_t init_BMA220()
{
    int err;

    uint8_t low_pass_filter = 0x20;
    uint8_t filter_config = 0x05;
    uint8_t write_buf[2] = {low_pass_filter, filter_config};

    err = i2c_master_write_to_device(I2C_MASTER_NUM, BMA220_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return err;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 iError = BME280_INIT_VALUE;

    esp_err_t espRc;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, cnt, true);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        iError = SUCCESS;
    }
    else
    {
        iError = FAIL;
    }
    i2c_cmd_link_delete(cmd);

    return (s8)iError;
}

s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 iError = BME280_INIT_VALUE;
    esp_err_t espRc;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    if (cnt > 1)
    {
        i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        iError = SUCCESS;
    }
    else
    {
        iError = FAIL;
    }

    i2c_cmd_link_delete(cmd);

    return (s8)iError;
}

void BME280_delay_msek(u32 msek)
{
    vTaskDelay(msek / portTICK_PERIOD_MS);
}

// function to get the acceleration value of the BMA220 sensor. Must be called after BMA220_init
// argument would contain acceleration value

static esp_err_t BMA220_getAcc(uint8_t direction, int8_t *acc_value)
{
    // shift bits to the right to account for offset
    uint8_t shifted_value = 0;
    esp_err_t err;
    err = i2c_master_write_read_device(I2C_MASTER_NUM, BMA220_SENSOR_ADDR, &direction, 1, &shifted_value, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    *acc_value = (int8_t)shifted_value >> 2;
    return err;
}

// main execution loop
void task_bme280_bma220(void *ignore)
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
        while (true)
        {
            vTaskDelay(40 / portTICK_PERIOD_MS);

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

    vTaskDelete(NULL);
}

static uint8_t s_led_state = 0;

void task_blink_led(void *ignore)
{
    while (1)
    {
        // ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        /* Set the GPIO level according to the state (LOW or HIGH)*/
        gpio_set_level(BLINK_GPIO, s_led_state);
        /* Toggle the LED state */
        s_led_state = !s_led_state;
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    ESP_ERROR_CHECK(init_BMA220());

    // pins may have multiple functions. use this function to set it as a gpio pin
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    xTaskCreate(&task_bme280_bma220, "i2c_sensors", 2048, NULL, 6, NULL);
    xTaskCreate(&task_blink_led, "blink_led", 2048, NULL, 7, NULL);
}
