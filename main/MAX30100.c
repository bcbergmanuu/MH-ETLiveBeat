/*
Arduino-MAX30100 oximetry / heart rate integrated sensor library
Copyright (C) 2016  OXullo Intersecans <x@brainrapers.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "MAX30100.h"

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

static const char *TAG = "MAX30100";

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t max30100_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MAX30100_I2C_ADDRESS, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}


/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t max30100_register_write_byte(uint8_t reg_addr, uint8_t data)
{    
    uint8_t write_buf[2] = {reg_addr, data};

    return i2c_master_write_to_device(I2C_MASTER_NUM, MAX30100_I2C_ADDRESS, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);    
}

static void writeRegister(uint8_t reg_addr, uint8_t data) {
    ESP_ERROR_CHECK(max30100_register_write_byte(reg_addr, data));
}

static void readRegister(uint8_t reg_addr, uint8_t *data, size_t len) {    
    esp_err_t t = max30100_register_read(reg_addr, data, len);
    ESP_ERROR_CHECK(t);
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
        .master.clk_speed =  I2C_BUS_SPEED,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void MAX30100_init()
{
    ESP_ERROR_CHECK(i2c_master_init());
    //Wire.setClock(I2C_BUS_SPEED);

    uint8_t partId = getPartId();
    if (partId != EXPECTED_PART_ID) {
        ESP_LOGE(TAG, "partnumber not correct, partId was %i", partId);
        return;
    }

    setMode(DEFAULT_MODE);
    setLedsPulseWidth(DEFAULT_PULSE_WIDTH);
    setSamplingRate(DEFAULT_SAMPLING_RATE);
    setLedsCurrent(DEFAULT_IR_LED_CURRENT, DEFAULT_RED_LED_CURRENT);
    setHighresModeEnabled(true);
}

void setMode(Mode mode)
{
    writeRegister(MAX30100_REG_MODE_CONFIGURATION, mode);
}

void setLedsPulseWidth(LEDPulseWidth ledPulseWidth)
{   uint8_t previous;
    readRegister(MAX30100_REG_SPO2_CONFIGURATION, &previous, 1);
    writeRegister(MAX30100_REG_SPO2_CONFIGURATION, (previous & 0xfc) | ledPulseWidth);
}

void setSamplingRate(SamplingRate samplingRate)
{
    uint8_t previous;
    readRegister(MAX30100_REG_SPO2_CONFIGURATION, &previous, 1);
    writeRegister(MAX30100_REG_SPO2_CONFIGURATION, (previous & 0xe3) | (samplingRate << 2));
}

void setLedsCurrent(LEDCurrent irLedCurrent, LEDCurrent redLedCurrent)
{
    writeRegister(MAX30100_REG_LED_CONFIGURATION, redLedCurrent << 4 | irLedCurrent);
}

void setHighresModeEnabled(bool enabled)
{
    uint8_t previous;
    readRegister(MAX30100_REG_SPO2_CONFIGURATION, &previous, 1);
    if (enabled) {
        writeRegister(MAX30100_REG_SPO2_CONFIGURATION, previous | MAX30100_SPC_SPO2_HI_RES_EN);
    } else {
        writeRegister(MAX30100_REG_SPO2_CONFIGURATION, previous & ~MAX30100_SPC_SPO2_HI_RES_EN);
    }
}

void resetFifo()
{
    writeRegister(MAX30100_REG_FIFO_WRITE_POINTER, 0);
    writeRegister(MAX30100_REG_FIFO_READ_POINTER, 0);
    writeRegister(MAX30100_REG_FIFO_OVERFLOW_COUNTER, 0);
}

void readFifoData(SensorReadout *sensorData)
{
    
    uint8_t buffer[MAX30100_FIFO_DEPTH*4];
    
    uint8_t reg1;
    uint8_t reg2;
    readRegister(MAX30100_REG_FIFO_WRITE_POINTER, &reg1, 1);
    readRegister(MAX30100_REG_FIFO_READ_POINTER, &reg2, 1);
    uint8_t toRead = (reg1-reg2) & (MAX30100_FIFO_DEPTH-1);
    //toRead = (readRegister(MAX30100_REG_FIFO_WRITE_POINTER) - readRegister(MAX30100_REG_FIFO_READ_POINTER)) & (MAX30100_FIFO_DEPTH-1);

    if (toRead) {
        readRegister(MAX30100_REG_FIFO_DATA, buffer, 4 * toRead);

        for (uint8_t i=0 ; i < toRead ; ++i) {
            // Warning: the values are always left-aligned
            SensorReadout d;
            
            d.ir=(uint16_t)((buffer[i*4] << 8) | buffer[i*4 + 1]);
            d.red=(uint16_t)((buffer[i*4 + 2] << 8) | buffer[i*4 + 3]);
            
            sensorData[i] = d;
             //todo: return those!
        }
    }
}

void startTemperatureSampling()
{
    uint8_t modeConfig;
    readRegister(MAX30100_REG_MODE_CONFIGURATION, &modeConfig, 1);
    (modeConfig) |= MAX30100_MC_TEMP_EN;

    writeRegister(MAX30100_REG_MODE_CONFIGURATION, modeConfig);
}

bool isTemperatureReady()
{
    uint8_t config;
    readRegister(MAX30100_REG_MODE_CONFIGURATION, &config, 1);
    return !(config & MAX30100_MC_TEMP_EN);
}

float retrieveTemperature()
{
    uint8_t tempInteger;
    readRegister(MAX30100_REG_TEMPERATURE_DATA_INT, &tempInteger,1);
    
    uint8_t tempFrac;
    readRegister(MAX30100_REG_TEMPERATURE_DATA_FRAC, &tempFrac, 1); 

    return tempFrac * 0.0625 + 1.0*tempInteger;
}

void shutdown()
{
    uint8_t modeConfig;
    readRegister(MAX30100_REG_MODE_CONFIGURATION, &modeConfig, 1);
    modeConfig |= MAX30100_MC_SHDN;

    writeRegister(MAX30100_REG_MODE_CONFIGURATION, modeConfig);
}

void resume()
{
    uint8_t modeConfig;
    readRegister(MAX30100_REG_MODE_CONFIGURATION, &modeConfig, 1);
    modeConfig &= ~MAX30100_MC_SHDN;

    writeRegister(MAX30100_REG_MODE_CONFIGURATION, modeConfig);
}

uint8_t getPartId()
{
    uint8_t partId;   
    readRegister(0xff, &partId, 1);
    return partId;
}