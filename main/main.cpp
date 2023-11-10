/**
 * @file main.cpp
 * @brief Main application for controlling a light control system using the VEML6040 light sensor and WD3153 LED driver.
 *
 * This application provides three modes of operation:
 * 1. DATA_COLLECTION - Collects data from the VEML6040 light sensor and prints it to the console
 * 2. DIRECT_PROCCESING - Processes the data from the VEML6040 light sensor and controls the LED driver
 * 3. LIGHT_TRIGGER - Triggers the LED driver based on the ambient light level
 */

#include <stdio.h>
#include <time.h>
#include "string.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "unity.h"

#include "veml6040.h"
#include "wd3153.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_SCL_IO (gpio_num_t)CONFIG_I2C_MASTER_SCL                 /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO (gpio_num_t)CONFIG_I2C_MASTER_SDA                 /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM)               /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY                      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                                         /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                                         /*!< I2C master doesn't need buffer */

#define WINDOW_SIZE CONFIG_WINDOW_SIZE
#define MODE CONFIG_DEVICE_MODE
#define TRIGGER_SENSITIVITY CONFIG_TRIGGER_SENSITIVITY
#define TERMINAL_OUTPUT CONFIG_TERMINAL_OUTPUT

extern "C" {
	void app_main(void);
}

// Device modes
enum {
    DATA_COLLECTION,
    DIRECT_PROCCESING,
    LIGHT_TRIGGER
};

// Function prototypes  
void add_data(light_sensor_data_t* window, light_sensor_data_t data);
double max(double a, double b);
double min(double a, double b);  
void ledTriggerByLux(CVeml6040 *data_acquirer, WD3153 *led);

/**
 * @brief This is the main function which runs the program
 */
void app_main(void) {
     // Initialize components
    DoubleBuffer *buffer = new DoubleBuffer();
    CI2CBus *i2c_bus = new CI2CBus(I2C_MASTER_NUM, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
    CVeml6040 *data_acquirer = new CVeml6040(i2c_bus, buffer);
    WD3153 *data_analyzer = new WD3153(i2c_bus, buffer);

    // Run selected mode 
    if(MODE == DATA_COLLECTION) {    
        vTaskDelay(30000 / portTICK_RATE_MS);
        printf("=====START=====\n");
        data_acquirer->startAcquisition();
        data_analyzer->startAnalysis();
    }
    else if(MODE == DIRECT_PROCCESING) {
        // vTaskDelay(30000 / portTICK_RATE_MS);
        // printf("=====START=====\n");

        // set variables
        light_sensor_data_t* window;
        window = new light_sensor_data_t[WINDOW_SIZE];
        double min_threshold[3] = {40, 20, 40};
        double threshold[3] = {60, 20, 60};
        double sum_red = 0, sum_blue = 0, sum_white = 0;
        uint16_t length = 0;
        light_sensor_data_t data = data_acquirer->getData();
        light_sensor_data_t prev_data;

        uint16_t counter[] = {0, 0, 0};
        uint16_t over_threshold[] = {0, 0, 0};
        uint8_t on[] = {0, 0, 0};

        uint16_t data_count = 0;
        bool last_on[] = {false, false, false};

        // main loop
        while(1) {

            // get light intensity difference
            prev_data = data;
            data = data_acquirer->getData();
            light_sensor_data_t diff;
            diff.red = abs(data.red - prev_data.red);
            diff.green = abs(data.green - prev_data.green);
            diff.blue = abs(data.blue - prev_data.blue);
            diff.white = abs(data.white - prev_data.white);
            diff.lux = abs(data.lux - prev_data.lux);
            diff.cct = abs(data.cct - prev_data.cct);
            
            // update rolling sum
            if(length<WINDOW_SIZE) {
                window[length] = diff;
                length++;
                sum_red += diff.red;
                sum_blue += diff.blue;
                sum_white += diff.white;
            }
            else {
                sum_red = sum_red - window[0].red + diff.red;
                sum_blue = sum_blue - window[1].blue + diff.blue;
                sum_white = sum_white - window[2].white + diff.white;
            }
                
            // update threshold
            threshold[0] = min(max(sum_red / length * 5, min_threshold[0]), 80);
            threshold[1] = min(max(sum_blue / length * 5, min_threshold[1]), 80);
            threshold[2] = min(max(sum_white / length * 5, min_threshold[2]), 80);

            // count number of times over threshold
            if(diff.red > threshold[0]) {
                if(last_on[0] == false) {
                    counter[0] = 0;
                    over_threshold[0] += 1;
                    last_on[0] = true;
                }
            }
            else if(diff.red < min_threshold[0]) {
                last_on[0] = false;
            }
            if(diff.blue > threshold[1]) {
                if(last_on[1] == false) {
                    counter[1] = 0;
                    over_threshold[1] += 1;
                    last_on[1] = true;
                }
            }
            else if(diff.blue < min_threshold[1]) {
                last_on[1] = false;
            }
            if(diff.white > threshold[2]) {
                if(last_on[2] == false){
                    counter[2] = 0;
                    over_threshold[2] += 1;
                    last_on[2] = true;
                }
            }
            else if(diff.white < min_threshold[2]) {
                last_on[2] = false;
            }

            // update on/off status
            for(int i = 0; i < 3; i++) {
                if(counter[i] < WINDOW_SIZE && over_threshold[i] > TRIGGER_SENSITIVITY) {
                    on[i] = 1;
                }
                else if(counter[i] >= WINDOW_SIZE) {
                    on[i] = 0;
                    over_threshold[i] = 0;
                }
                counter[i]++;
            }
            add_data(window,diff);
            
            uint8_t on_off=0;
            for(int i = 0; i < 3; i++) {
                if(on[i] > 0){ on_off++; }
            }

            // print data
            if(TERMINAL_OUTPUT) {
                printf("%d\t",data_count);
                printf("Diff:\tr:%d, b:%d, w:%d\n",diff.red,diff.blue,diff.white);
                printf("Thrshld:\tr:%f, b:%f, w:%f\n",threshold[0], threshold[1], threshold[2]);
                printf("Over:\tr:%d, b:%d, w:%d\n",over_threshold[0],over_threshold[1],over_threshold[2]);
                printf("%d, %d, %d, %d, %d, ",data.red,data.green,data.blue,data.white,data.cct);
            }

            // set LED for on/off status
            if(on_off >= 2) {
                data_analyzer->setLEDPWM(1,255);
                if(TERMINAL_OUTPUT) printf("on\n");
            }else {
                data_analyzer->setLEDPWM(1,0);
                if(TERMINAL_OUTPUT) printf("off\n");
            }
            data_count++;
        }
    }
    else if(MODE == LIGHT_TRIGGER) {
        ledTriggerByLux(data_acquirer, data_analyzer);
    }

    // Clean up
    printf("=====END=====\n");
    vTaskDelete(NULL);
    delete(data_analyzer);
    delete(data_acquirer);
    delete(i2c_bus);
}

/**
 * @brief This function updates the given window of light sensor data with the new data
 *
 * @param window Pointer to the array of light_sensor_data_t type which stores the light sensor data
 * @param data New light_sensor_data_t type data to be added to the window
 */
void add_data(light_sensor_data_t* window, light_sensor_data_t data){
    for(int i = 0; i < WINDOW_SIZE - 1; i++) {
        window[i]=window[i+1];
    }
    window[WINDOW_SIZE-1]=data;
}

/**
 * @brief This function returns the maximum of two given double values
 *
 * @param a Double value
 * @param b Double value
 * @return Double value which is the maximum of the two given values
 */
double max(double a,double b) {
    return a>b?a:b;
}

/**
 * @brief This function returns the minimum of two given double values
 *
 * @param a Double value
 * @param b Double value
 * @return Double value which is the minimum of the two given values
 */
double min(double a,double b) {
    return a<b?a:b;
}

/**
 * @brief This function triggers the LED based on the lux value obtained from the light sensor
 *
 * @param data_acquirer Pointer to the object of CVeml6040 class which is used to acquire the lux value
 * @param led Pointer to the object of WD3153 class which is used to control the LED
 */
void ledTriggerByLux(CVeml6040 *data_acquirer, WD3153 *led) {
    while(1){
        float lux = data_acquirer->lux();
        if(lux < 255){
            led->setGroupPWM(255 - lux);
        }
        else{
            led->setGroupPWM(0);
        }
    }
}