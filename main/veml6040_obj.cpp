#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "veml6040.h"

#define WRITE_BIT             I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT              I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN          0x1               /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS         0x0               /*!< I2C master will not check ack from slave */
#define ACK_VAL               0x0               /*!< I2C ack value */
#define NACK_VAL              0x1               /*!< I2C nack value */
//CMD
#define VEML6040_SET_CMD      0x00
#define VEML6040_READ_REG     0x08
#define VEML6040_READ_GREEN   0x09
#define VEML6040_READ_BLUE    0x0A
#define VEML6040_READ_WHITE   0x0B
// G SENSITIVITY
#define VEML6040_GSENS_40MS   0.25168
#define VEML6040_GSENS_80MS   0.12584
#define VEML6040_GSENS_160MS  0.06292
#define VEML6040_GSENS_320MS  0.03146
#define VEML6040_GSENS_640MS  0.01573
#define VEML6040_GSENS_1280MS 0.007865

#define DEBUG false

static const char *TAG = "veml6040";

typedef struct {
    i2c_bus_handle_t bus_t;
    uint16_t dev_addr;
    veml6040_config_t config;
} veml6040_dev_t;


CVeml6040::CVeml6040(CI2CBus *p_i2c_bus, DoubleBuffer *p_buffer, uint8_t addr)
:DataAcquirer(p_i2c_bus,p_buffer) {
    veml6040_dev_t* sensor = (veml6040_dev_t*) calloc(1, sizeof(veml6040_dev_t));
    sensor->bus_t = getI2CBus()->get_bus_handle();
    sensor->dev_addr = addr;
    sensor->config.integration_time = VEML6040_INTEGRATION_TIME_DEFAULT;
    sensor->config.mode = VEML6040_MODE_DEFAULT;
    sensor->config.trigger = VEML6040_TRIGGER_DEFAULT;
    sensor->config.switch_en = VEML6040_SWITCH_DEFAULT;
    set_mode(sensor, &sensor->config);
    m_sensor_handle =  (veml6040_handle_t) sensor;
    init();
}

CVeml6040::~CVeml6040() {
    veml6040_dev_t* device = (veml6040_dev_t*) m_sensor_handle;
    free(device);
    m_sensor_handle = NULL;
}

esp_err_t CVeml6040::write(veml6040_handle_t sensor, uint8_t config_val) {
    esp_err_t ret;
    veml6040_dev_t* device = (veml6040_dev_t*) sensor;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device->dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, VEML6040_SET_CMD, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, config_val, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = iot_i2c_bus_cmd_begin(device->bus_t, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t CVeml6040::read(veml6040_handle_t sensor, uint8_t cmd_code) {
    esp_err_t ret;
    uint8_t data_low = 0;
    uint8_t data_high = 0;
    veml6040_dev_t* device = (veml6040_dev_t*) sensor;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device->dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, cmd_code, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device->dev_addr << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data_low, (i2c_ack_type_t)ACK_VAL);
    i2c_master_read_byte(cmd, &data_high, (i2c_ack_type_t)ACK_VAL);
    i2c_master_stop(cmd);
    ret = iot_i2c_bus_cmd_begin(device->bus_t, cmd, 1000 / portTICK_RATE_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "READ ERROR,ERROR ID:%d", ret);
    }
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? (data_low | (data_high << 8)) : VEML6040_I2C_ERR_RES;
}

esp_err_t CVeml6040::set_mode(veml6040_handle_t sensor, veml6040_config_t * device_info) {
    uint8_t cmd_buf = 0;
    veml6040_dev_t* dev = (veml6040_dev_t*) sensor;
    cmd_buf = device_info->integration_time << 4
            | device_info->trigger << 2
            | device_info->mode << 1
            | device_info->switch_en;
    dev->config.integration_time = device_info->integration_time;
    dev->config.trigger          = device_info->trigger;
    dev->config.mode             = device_info->mode;
    dev->config.switch_en        = device_info->switch_en;
    return write(sensor, cmd_buf);
}

int CVeml6040::red() {
    return read(m_sensor_handle, VEML6040_READ_REG);
}

int CVeml6040::green() {
    return read(m_sensor_handle, VEML6040_READ_GREEN);
}

int CVeml6040::blue() {
    return read(m_sensor_handle, VEML6040_READ_BLUE);
}

int CVeml6040::white() {
    return read(m_sensor_handle, VEML6040_READ_WHITE);
}

float CVeml6040::lux() {
    uint16_t sensorValue;
    float ambientLightInLux;
    veml6040_dev_t* dev = (veml6040_dev_t*) m_sensor_handle;
    sensorValue = green();
    switch (dev->config.integration_time) {
        case VEML6040_INTEGRATION_TIME_40MS:
            ambientLightInLux = sensorValue * VEML6040_GSENS_40MS;
            break;
        case VEML6040_INTEGRATION_TIME_80MS:
            ambientLightInLux = sensorValue * VEML6040_GSENS_80MS;
            break;
        case VEML6040_INTEGRATION_TIME_160MS:
            ambientLightInLux = sensorValue * VEML6040_GSENS_160MS;
            break;
        case VEML6040_INTEGRATION_TIME_320MS:
            ambientLightInLux = sensorValue * VEML6040_GSENS_320MS;
            break;
        case VEML6040_INTEGRATION_TIME_640MS:
            ambientLightInLux = sensorValue * VEML6040_GSENS_640MS;
            break;
        case VEML6040_INTEGRATION_TIME_1280MS:
            ambientLightInLux = sensorValue * VEML6040_GSENS_1280MS;
            break;
        default:
            ambientLightInLux = VEML6040_I2C_ERR_RES;
            break;
    }
    return ambientLightInLux;
}

uint16_t CVeml6040::cct(float offset) {
    uint16_t r,g,b;
    float cct, ccti;
    r   = red();
    g = green();
    b  = blue();
    ccti  = ((float) r - (float) b) / (float) g;
    ccti  = ccti + offset;
    cct   = 4278.6 * pow(ccti, -1.2455);
    return cct;
}

void CVeml6040::init() {
    if(DEBUG) ESP_LOGI(TAG," init Light Sensor");
    veml6040_config_t veml6040_info;
    memset(&veml6040_info, 0, sizeof(veml6040_info));
    veml6040_info.integration_time = VEML6040_INTEGRATION_TIME_160MS;
    veml6040_info.mode = VEML6040_MODE_AUTO;
    veml6040_info.trigger = VEML6040_TRIGGER_DIS;
    veml6040_info.switch_en = VEML6040_SWITCH_EN;
    set_mode(m_sensor_handle, &veml6040_info);
    if(DEBUG) ESP_LOGI(TAG,"Set Mode");
}

void CVeml6040::readData() {
    data.red = red();
    data.green = green();
    data.blue = blue();
    data.white = white();
    data.lux = lux();
    data.cct = cct(0.5);
}

light_sensor_data_t CVeml6040::getData() {
    readData();
    vTaskDelay(160 / portTICK_RATE_MS);
    return data;
}

void CVeml6040::showData() {
    printf("\n");
}

void CVeml6040::sendDataToBuffer() {
    while(1){
        readData();
        getBuffer()->write(&data);
        vTaskDelay(160 / portTICK_RATE_MS);
    }
}
void CVeml6040::taskWrapper(void* pvParameters) {
    CVeml6040* pThis = static_cast<CVeml6040*>(pvParameters);
    pThis->sendDataToBuffer();
}
void CVeml6040::startAcquisition() {
    xTaskCreatePinnedToCore(taskWrapper, "sendDataToBuffer", 2048*2, (void *)this, 5, &taskHandle, 0);
}
void CVeml6040::stopAcquisition() {
    printf("\n\n====== Acquisition finished ======\n\n");
    vTaskDelete(taskHandle);
}