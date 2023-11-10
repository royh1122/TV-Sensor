#include <stdio.h>
#include "wd3153.h"

#define DEBUG false

#define WRITE_BIT             I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT              I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN          0x1               /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS         0x0               /*!< I2C master will not check ack from slave */
#define ACK_VAL               0x0               /*!< I2C ack value */
#define NACK_VAL              0x1               /*!< I2C nack value */

//CMD
#define REG_RST		0x00	//Chip ID and Software Reset Register RSTR 
#define REG_GC		0x01	//Global Control Register GCR
#define REG_IS		0x02
#define REG_FCT 	0x03
#define REG_LCT 	0x30
#define REG_LCFG0	0x31
#define REG_LCFG1	0x32
#define REG_LCFG2	0x33
#define REG_PWM0	0x34
#define REG_PWM1	0x35
#define REG_PWM2	0x36
#define REG_LED0T1	0x37
#define REG_LED0T3	0x38
#define REG_LED0T0	0x39
#define REG_LED1T1	0x3A
#define REG_LED1T3	0x3B
#define REG_LED1T0	0x3C
#define REG_LED2T1	0x3D
#define REG_LED2T3	0x3E
#define REG_LED2T0	0x3F
#define REG_LEDSYN	0x4A
#define REG_IAD		0x77

#define LED0 0b00000000
#define LED1 0b00000001
#define LED2 0b00000010

static const char *TAG = "wd3153";

esp_err_t iot_wd3153_write(CI2CBus *bus, uint8_t slave_addr, uint8_t reg_addr, uint8_t data) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, (slave_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    // write the data
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = iot_i2c_bus_cmd_begin(bus->get_bus_handle(), cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

uint8_t iot_wd3153_read(CI2CBus *bus, uint8_t slave_addr, uint8_t reg_addr) {
	esp_err_t ret;
	uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
	if(ret != ESP_OK)ESP_LOGE(TAG, "MASTER START ERROR,ERROR ID:%d", ret);
    ret = i2c_master_write_byte(cmd, (slave_addr<<1) | WRITE_BIT, ACK_CHECK_EN);
	if(ret != ESP_OK)ESP_LOGE(TAG, "1st master write error,ERROR ID:%d", ret);
	ret = i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
	if(ret != ESP_OK)ESP_LOGE(TAG, "2nd master write error,ERROR ID:%d", ret);
    ret = i2c_master_start(cmd);
	if(ret != ESP_OK)ESP_LOGE(TAG, "repeated start error,ERROR ID:%d", ret);
    ret = i2c_master_write_byte(cmd, (slave_addr<<1) | READ_BIT, ACK_CHECK_EN);
    if(ret != ESP_OK)ESP_LOGE(TAG, "3rd master write error,ERROR ID:%d", ret);
	ret = i2c_master_read_byte(cmd, &data, (i2c_ack_type_t)0x1);
    if(ret != ESP_OK)ESP_LOGE(TAG, "master read error,ERROR ID:%d", ret);
	ret = i2c_master_stop(cmd);
    if(ret != ESP_OK)ESP_LOGE(TAG, "stop error,ERROR ID:%d", ret);
	ret = iot_i2c_bus_cmd_begin(bus->get_bus_handle(), cmd, 1000 / portTICK_RATE_MS);
    if(ret != ESP_OK)ESP_LOGE(TAG, "cmd begin error,ERROR ID:%d", ret);
	i2c_cmd_link_delete(cmd);
    return data;
}

inline void WD3153::writeReg(uint8_t reg, uint8_t data) {
	esp_err_t ret;
	ret = iot_wd3153_write(getI2CBus(), address, reg, data);
	if(ret != ESP_OK)ESP_LOGE(TAG, "WRITE ERROR,ERROR ID:%d", ret);
}

inline uint8_t WD3153::readReg(uint8_t reg) {
	return iot_wd3153_read(getI2CBus(), address, reg);
}


WD3153::WD3153(CI2CBus *p_i2c_bus, DoubleBuffer *p_buffer, uint8_t addr)
:DataAnalyzer(p_i2c_bus, p_buffer) {
	address = addr;
	chipID = 0x33;
	init();
}

void WD3153::setChipID(uint8_t chipID) {
	writeReg(REG_RST, chipID);
}
uint8_t WD3153::getChipID() {
	return readReg(REG_RST);
}

void WD3153::reset() {
	writeReg(REG_RST, 0x55);
}

void WD3153::setChipEnabled(bool state) {
	uint8_t gc = readReg(REG_GC);
	writeReg(REG_GC, (gc&0b11111110) | state);
}
bool WD3153::isChipEnabled() {
	uint8_t gc = readReg(REG_GC);
	return gc&0x01;
}

void WD3153::setUVLOInterruptEnabled(bool state) {
	uint8_t gc = readReg(REG_GC);
	writeReg(REG_GC, (gc&0b11101111) | (state<<4));
}
bool WD3153::isUVLOInterruptEnabled() {
	uint8_t gc = readReg(REG_GC);
	return (gc>>4) & 0x01;
}

void WD3153::setOTPInterruptEnabled(bool state) {
	uint8_t gc = readReg(REG_GC);
	writeReg(REG_GC, (gc&0b11110111) | (state<<3));
}
bool WD3153::isOTPInterruptEnabled() {
	uint8_t gc = readReg(REG_GC);
	return (gc>>3)&0x01;
}
	
void WD3153::setLEDInterruptEnabled(uint8_t ledX,bool state) {
	if(ledX>2) return;
	uint8_t gc = readReg(REG_GC);
	writeReg(REG_GC, (gc&(~(1<<(ledX+5)))) | (state<<(ledX+5)));
}
bool WD3153::isLEDInterruptEnabled(uint8_t ledX,bool state) {
	if(ledX>2) return false;
	uint8_t gc = readReg(REG_GC);
	return (gc>>(ledX+5))&0x01;
}
//Interrupt Status
//Power On Reset Interrupt
void WD3153::setPORInterruptStatus(bool state) {
	uint8_t is = readReg(REG_IS);
	writeReg(REG_GC, (is&0b11101111) | (state<<4));
}
bool WD3153::getPORInterruptStatus() {
	uint8_t is = readReg(REG_IS);
	return (is>>4)&0x1;
}

//Under Voltage Lock Out Interrupt
bool WD3153::getUVLOInterruptStatus() {
	uint8_t is = readReg(REG_IS);
	return (is>>3)&0x1;
}

//Over Temperature Interrupt
bool WD3153::getOTPInterruptStatus() {
	uint8_t is = readReg(REG_IS);
	return (is>>2)&0x1;
}

void WD3153::setLEDInterruptStatus(uint8_t ledX,bool state) {
	if(ledX>2) return;
	uint8_t is = readReg(REG_IS);
	writeReg(REG_IS, (is&(~(1<<(ledX+5)))) | (state<<(ledX+5)));
}
bool WD3153::getLEDInterruptStatus(uint8_t ledX) { 
	if(ledX>2) return false;
	uint8_t is = readReg(REG_IS);
	return (is>>(ledX+5))&0x01;
}

//Function Control
void WD3153::setOTPEnabled(bool state) {
	uint8_t fc = readReg(REG_FCT);
	writeReg(REG_FCT, (fc&0b11101111) | (state<<4));
}
bool WD3153::isOTPEnabled() {
	uint8_t fc = readReg(REG_FCT);
	return (fc>>4)&0x01;
}

void WD3153::setUVLOEnabled(bool state) {
	uint8_t fc = readReg(REG_FCT);
	writeReg(REG_FCT, (fc&0b11110111) | (state<<3));
}
bool WD3153::isUVLOEnabled() {
	uint8_t fc = readReg(REG_FCT);
	return (fc>>3)&0x01;
}

void WD3153::setCHRGEnabled(bool state) {
	uint8_t fc = readReg(REG_FCT);
	writeReg(REG_FCT, (fc&0b11111011) | ((!state)<<2));
}
bool WD3153::isCHRGEnabled() {
	uint8_t fc = readReg(REG_FCT);
	return (~(fc>>2))&0x01;
}

void WD3153::setGlobalPWMFrequencyLevel(bool state) {
	uint8_t fc = readReg(REG_FCT);
	writeReg(REG_FCT, (fc&0b11111101) | (state<<1));
}
bool WD3153::getGlobalPWMFrequencyLevel() {
	uint8_t fc = readReg(REG_FCT);
	return (fc>>1)&0x01;
}

void WD3153::setGlobalPWMEasingFunction(bool state) {
	uint8_t fc = readReg(REG_FCT);
	writeReg(REG_FCT, (fc&0b11111110) | state);
}
bool WD3153::getGlobalPWMEasingFunction() {
	uint8_t fc = readReg(REG_FCT);
	return fc&0x01;
}

//LED Enabled
void WD3153::setLEDEnabled(uint8_t ledX, bool state) {
	if(ledX>2) return;
	uint8_t lct = readReg(REG_LCT);
	writeReg(REG_LCT, (lct&(~(1<<ledX))) | (state<<ledX));
}
bool WD3153::isLEDEnabled(uint8_t ledX) {
	if(ledX>2) return false;
	uint8_t lct = readReg(REG_LCT);
	return (lct>>ledX)&0x01;
}

//Fade In/Out Mode
//Fade Out uses T3
//Fade In uses T1
void WD3153::setLEDFadeOutEnabled(uint8_t ledX, bool state) {
	if(ledX>2) return;
	uint8_t lcfg = readReg(REG_LCFG0+ledX);
	writeReg(REG_LCFG0+ledX, (lcfg&0b10111111) | (state<<6));
}
bool WD3153::isLEDFadeOutEnabled(uint8_t ledX) {
	if(ledX>2) return false;
	uint8_t lcfg = readReg(REG_LCFG0+ledX);
	return (lcfg>>6)&0x01;
}

void WD3153::setLEDFadeInEnabled(uint8_t ledX, bool state) {
	if(ledX>2) return;
	uint8_t lcfg = readReg(REG_LCFG0+ledX);
	writeReg(REG_LCFG0+ledX, (lcfg&0b11011111) | (state<<5));
}
bool WD3153::isLEDFadeInEnabled(uint8_t ledX) {
	if(ledX>2) return false;
	uint8_t lcfg = readReg(REG_LCFG0+ledX);
	return (lcfg>>5)&0x01;
}

//LED Mode
void WD3153::setLEDMode(uint8_t ledX, bool mode) {
	if(ledX>2) return;
	uint8_t lcfg = readReg(REG_LCFG0+ledX);
	writeReg(REG_LCFG0+ledX, (lcfg&0b11101111) | (mode<<4));
}
bool WD3153::getLEDMode(uint8_t ledX) {
	if(ledX>2) return false;
	uint8_t lcfg = readReg(REG_LCFG0+ledX);
	return (lcfg>>4)&0x01;
}

//LED Current Limit
void WD3153::setLEDCurrentLimit(uint8_t ledX, uint8_t currentLimit) {
	if(ledX>2) return;
	uint8_t lcfg = readReg(REG_LCFG0+ledX);
	writeReg(REG_LCFG0+ledX, (lcfg&0b11111000) | currentLimit);
}
uint8_t WD3153::getLEDCurrentLimit(uint8_t ledX) {
	if(ledX>2) return false;
	uint8_t lcfg = readReg(REG_LCFG0+ledX);
	return lcfg&0b00000111;
}

//PWM
void WD3153::setLEDPWM(uint8_t ledX,uint8_t duty) {
	writeReg(REG_PWM0+ledX,duty);
}
uint8_t WD3153::getLEDPWM(uint8_t ledX) {
	return readReg(REG_PWM0+ledX);
}
void WD3153::setGroupPWM(uint8_t duty) {
	for(uint8_t i=0;i<3;i++){
		setLEDPWM(i,duty);
	}
}

//T0 T1 T2 T3 T4
void WD3153::setLEDTime(uint8_t ledX, uint8_t timeX, uint8_t timeValue) {
	if(ledX>2 || timeValue>0b00001000) return;
	uint8_t ledT;
	switch(timeX) {
		case T0: //Only T0 supports 0s
			ledT = readReg(REG_LED0T0+ledX*3);
			writeReg(REG_LED0T0+ledX*3, (ledT&0b00001111) | (timeValue<<4));
			break;
		case T1: //T1 16.64s max
			if(timeValue == T0s) return;
			ledT = readReg(REG_LED0T1+ledX*3);
			writeReg(REG_LED0T1+ledX*3, (ledT&0b10001111) | ((timeValue-1)<<4));
			break;
		case T2: //T2 4.15s max
			if(timeValue == T0s) return;
			ledT = readReg(REG_LED0T1+ledX*3);
			writeReg(REG_LED0T1+ledX*3, (ledT&0b11111000) | (timeValue-1));
			break;
		case T3: //T3 16.64s max
			if(timeValue == T0s) return;
			ledT = readReg(REG_LED0T3+ledX*3);
			writeReg(REG_LED0T3+ledX*3, (ledT&0b10001111) | ((timeValue-1)<<4));
			break;
		case T4: //T4 16.64s max
			if(timeValue == T0s) return;
			ledT = readReg(REG_LED0T3+ledX*3);
			writeReg(REG_LED0T3+ledX*3, (ledT&0b11111000) | (timeValue-1));
			break;
	}
}
uint8_t WD3153::getLEDTime(uint8_t ledX, uint8_t timeX) {
	if(ledX>2) return false;
	uint8_t ledT;
	switch(timeX) {
		case T0:
			ledT = readReg(REG_LED0T0+ledX*3);
			return (ledT>>4)&0b00001111;
		case T1:
			ledT = readReg(REG_LED0T1+ledX*3);
			return ((ledT>>4)&0b00000111) + 1;
		case T2:
			ledT = readReg(REG_LED0T1+ledX*3);
			return (ledT&0b00000111) + 1;
		case T3:
			ledT = readReg(REG_LED0T3+ledX*3);
			return ((ledT>>4)&0b00000111) + 1;
		case T4:
			ledT = readReg(REG_LED0T3+ledX*3);
			return (ledT&0b00000111) + 1;
	}
	return false;
}

//Blink repeat times 1~15, 0 For infinity
void WD3153::setLEDBlinkTimes(uint8_t ledX, uint8_t repeatTimes) {
	if(ledX>2 || repeatTimes >15) return;
	uint8_t ledT = readReg(REG_LED0T0+ledX*3);
	writeReg(REG_LED0T0+ledX*3, (ledT&0b00001111) | repeatTimes);
}
uint8_t WD3153::getLEDBlinkTimes(uint8_t ledX) {
	if(ledX>2) return false;
	uint8_t ledT = readReg(REG_LED0T0+ledX*3);
	return ledT&0b00001111;
}

//Sync
void WD3153::setSyncEnabled(bool state) {
	uint8_t ledSync = readReg(REG_LEDSYN);
	writeReg(REG_LEDSYN, (ledSync&0b00000100) | (state<<2));
}
bool WD3153::isSyncEnabled() {
	uint8_t ledSync = readReg(REG_LEDSYN);
	return (ledSync>>2)&0x01;
}

//I2C Address
void WD3153::setI2CAddress(uint8_t addr) {
	if(addr>0b01111111) return; //invalid
	writeReg(REG_IAD, (0b10000000) | addr);
	address = addr;
}
uint8_t WD3153::getI2CAddress() {
	return address;
}

//Initialize
bool WD3153::init() {
    bool result=true;

    if(DEBUG) ESP_LOGI(TAG," init LedDriver ");
    reset();                //to init the setting of LedDriver
    setChipEnabled(true);   // Enable the chip
    setCHRGEnabled(false);  //disable the CHRG (CHRG is not set in PCB in this case) refer to WD3153 datasheet

    result=isChipEnabled();
    if(result) {
        setGlobalPWMFrequencyLevel(PWM500HZ); //Global PWM FrequencyLevel

        if(DEBUG) ESP_LOGI(TAG," setLEDEnabled");
        setLEDEnabled(LED0, true);
        setLEDEnabled(LED1, true);
        setLEDEnabled(LED2, true);

        //set LED current. Something like to adjust the intensity
        if(DEBUG) ESP_LOGI(TAG," setLEDCurrentLimit");
        setLEDCurrentLimit(LED0, LED10mA);
        setLEDCurrentLimit(LED1, LED10mA);
        setLEDCurrentLimit(LED2, LED10mA);

        //enable fade in/out
        if(DEBUG) ESP_LOGI(TAG," enable Fade IN/OUT ");
        setLEDFadeInEnabled(LED0, true);
        setLEDFadeInEnabled(LED1, true);
        setLEDFadeInEnabled(LED2, true);
        setLEDFadeOutEnabled(LED0, true);
        setLEDFadeOutEnabled(LED1, true);
        setLEDFadeOutEnabled(LED2, true);
    }
    return result;
}

void WD3153::average(light_sensor_data_t data[],float *r,float *g,float *b,float *w,float *l,uint16_t *cct) { 
	*r = *g = *b = *w = *l = *cct = 0;
	for(int i=0; i<BUFFER_SIZE; i++) {
		*r += data[i].red;
		*g += data[i].green;
		*b += data[i].blue;
		*w += data[i].white;
		*l += data[i].lux;
		*cct += data[i].cct;
	}
	// printf("lux: %f\n", *l);
	*r /= BUFFER_SIZE;
	*g /= BUFFER_SIZE;
	*b /= BUFFER_SIZE;
	*w /= BUFFER_SIZE;
	*l /= BUFFER_SIZE;
	*cct /= BUFFER_SIZE;
}

void WD3153::readDataFromBuffer() {
	int count = 0;
    while(count < BUFFER_COUNT) {
		getBuffer()->read(r_data);
		if(r_data != NULL) {
			for(int i = 0; i < BUFFER_SIZE; i++) {
				// printf("%d %d\t",count,i);
				printf("%d, %d, %d, %d, %d,\n", r_data[i].red
										, r_data[i].green
										, r_data[i].blue
										, r_data[i].white
										, r_data[i].cct);
			}
		}
		else {
            ESP_LOGE(TAG, "No item");
        }
		count++;
	}
	stopAnalysis();
}
void WD3153::taskWrapper(void* pvParameters) {
	WD3153* pThis = static_cast<WD3153*>(pvParameters);
    pThis->readDataFromBuffer();
}
void WD3153::startAnalysis() {
	xTaskCreatePinnedToCore(taskWrapper, "readDataFromBuffer", 2048*2, (void *)this, 5, &taskHandle, 0);
}
void WD3153::stopAnalysis() {
	printf("=====END=====\n");
	esp_restart();
	vTaskDelete(taskHandle);
}

//RGB Cycle
void WD3153::rgbCycle(uint8_t dutyCycle) {
	while(1) {
		uint16_t pos;
		for (pos = 0; pos <= 255; pos++) {
			setLEDPWM(0, 255-pos);
			setLEDPWM(1, pos);
			vTaskDelay(dutyCycle / portTICK_RATE_MS);
		}
		for (pos = 0; pos <= 255; pos++) {
			setLEDPWM(1, 255-pos);
			setLEDPWM(2, pos);
			vTaskDelay(dutyCycle / portTICK_RATE_MS);
		}
		for (pos = 0; pos <= 255; pos++) {
			setLEDPWM(0, pos);
			setLEDPWM(2, 255-pos);
			vTaskDelay(dutyCycle / portTICK_RATE_MS);
		}
	}
}

void WD3153::blink() {
	for(int i = 0; i < 2; i++) {
		setLEDPWM(0, 255);
		vTaskDelay(100 / portTICK_RATE_MS);
		setLEDPWM(0, 0);
		vTaskDelay(100 / portTICK_RATE_MS);
	}
}