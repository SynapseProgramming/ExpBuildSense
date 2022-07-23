#include "sensors.h"

// function to initialise the BMA220 accelerometer sensor
esp_err_t init_BMA220()
{
    int err;

    uint8_t low_pass_filter = 0x20;
    uint8_t filter_config = 0x05;
    uint8_t write_buf[2] = {low_pass_filter, filter_config};

    err = i2c_master_write_to_device(I2C_MASTER_NUM, BMA220_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return err;
}

esp_err_t i2c_master_init(void)
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

esp_err_t BMA220_getAcc(uint8_t direction, int8_t *acc_value)
{
    // shift bits to the right to account for offset
    uint8_t shifted_value = 0;
    esp_err_t err;
    err = i2c_master_write_read_device(I2C_MASTER_NUM, BMA220_SENSOR_ADDR, &direction, 1, &shifted_value, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    *acc_value = (int8_t)shifted_value >> 2;
    return err;
}

bool suspend_BMA220()
{

    uint8_t suspend_command = 0x30;
    uint8_t suspend_status = 0;
    i2c_master_write_read_device(I2C_MASTER_NUM, BMA220_SENSOR_ADDR, &suspend_command, 1, &suspend_status, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (suspend_status == 0xFF)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void wake_BMA220()
{
    bool status = suspend_BMA220();
    if (status == true)
    {
        // sensor is currently sleeping. wake it up!
        suspend_BMA220();
    }
    // otherwise, the sensor is currently awake!
}

// helper function which ensure that the bma220 is alseep
void sleep_BMA220()
{
    bool status = suspend_BMA220();
    if (status == false)
    {
        // sensor is sleeping! wake it up!
        suspend_BMA220();
    }
    // otherwise, the sensor is currently asleep!
}
