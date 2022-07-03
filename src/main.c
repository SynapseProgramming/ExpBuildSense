
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/i2c.h"

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO 26        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 27        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define BMA220_SENSOR_ADDR 0x0A        /*!< Slave address of the BMA220 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR 0x75 /*!< Register addresses of the "who am I" register */

#define MPU9250_PWR_MGMT_1_REG_ADDR 0x6B /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT 7


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

// function to get the X value of the BMA220 sensor. Must be called after BMA220_init
// argument would contain new x value

static esp_err_t BMA220_getX(int8_t *x_value)
{
    uint8_t x_command = 0x04;
    // shift bits to the right to account for offset
    uint8_t shifted_x_value = 0;
    esp_err_t err;
    err = i2c_master_write_read_device(I2C_MASTER_NUM, BMA220_SENSOR_ADDR, &x_command, 1, &shifted_x_value, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    *x_value = (int8_t)shifted_x_value >> 2;
    return err;
}

void app_main(void)
{
    // uint8_t data[2];
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    ESP_ERROR_CHECK(init_BMA220());

    // get current x of the accelerometer
    int8_t x_val = 0;
    while (1)
    {

        BMA220_getX(&x_val);
        ESP_LOGI(TAG, "X value: %d", x_val);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}