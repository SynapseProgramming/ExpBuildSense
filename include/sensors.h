#ifndef SENSORS_H
#define SENSORS_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_sleep.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
static const char *TAG = "sensors";

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

#define V_REF 1700
#define BATTERY_MAX 3200 // maximum voltage of battery in mV
#define BATTERY_MIN 2600 // minimum voltage of battery in mV
// ADC1_CHANNEL_4 is pin 32


// function to initialise the BMA220 accelerometer sensor
esp_err_t init_BMA220();

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void);

// function to get the acceleration value of the BMA220 sensor. Must be called after BMA220_init
// argument would contain acceleration value
esp_err_t BMA220_getAcc(uint8_t direction, int8_t *acc_value);

// function to set the BMA220 accelerometer sensor to suspend mode
// returns true if BMA220 is currently placed in sleep mode after this function call
// return false otherwise
bool suspend_BMA220();

// helper function which ensures that the bma220 is awake
void wake_BMA220();

// helper function which ensures that the bma220 is alseep
void sleep_BMA220();


// function which initializes the ADC for battery voltage monitoring
// ADC1_CHANNEL_4 is pin 32 (connected to battery)

void init_ADC(esp_adc_cal_characteristics_t *adchar);

// function which gets the current battery state. 0 (empty) 100 (full)
uint8_t battery_voltage(esp_adc_cal_characteristics_t *adchar);

#endif