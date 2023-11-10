#ifndef _IOT_VEML6040_H_
#define _IOT_VEML6040_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_log.h"
#include "string.h"
#include "data_acquirer.h"

#define VEML6040_I2C_ADDRESS    (0x10)
#define VEML6040_I2C_ERR_RES    (-1)

#define VEML6040_INTEGRATION_TIME_DEFAULT  VEML6040_INTEGRATION_TIME_40MS
#define VEML6040_TRIGGER_DEFAULT           VEML6040_TRIGGER_DIS
#define VEML6040_MODE_DEFAULT              VEML6040_MODE_AUTO
#define VEML6040_SWITCH_DEFAULT            VEML6040_SWITCH_EN

typedef enum {
    VEML6040_INTEGRATION_TIME_40MS = 0, /*!< Command to set integration time 40ms*/
    VEML6040_INTEGRATION_TIME_80MS = 1, /*!< Command to set integration time 80ms*/
    VEML6040_INTEGRATION_TIME_160MS = 2, /*!< Command to set integration time 160ms*/
    VEML6040_INTEGRATION_TIME_320MS = 3, /*!< Command to set integration time 320ms*/
    VEML6040_INTEGRATION_TIME_640MS = 4, /*!< Command to set integration time 640ms*/
    VEML6040_INTEGRATION_TIME_1280MS = 5, /*!< Command to set integration time 1280ms*/
    VEML6040_INTEGRATION_TIME_MAX,
} veml6040_integration_time_t;

typedef enum {
    VEML6040_TRIGGER_DIS = 0, /*!< set not trigger 				*/
    VEML6040_TRIGGER_ONCE = 1, /*!< set trigger one time detect cycle*/
    VEML6040_TRIGGER_MAX,
} veml6040_trigger_t;

typedef enum {
    VEML6040_MODE_AUTO = 0, /*!< set auto mode  */
    VEML6040_MODE_FORCE = 1, /*!< set force mode */
    VEML6040_MODE_MAX,
} veml6040_mode_t;

typedef enum {
    VEML6040_SWITCH_EN = 0, /*!< set enable  */
    VEML6040_SWITCH_DIS = 1, /*!< set disable */
    VEML6040_SWITCH_MAX,
} veml6040_switch_t;

/**
 * @brief  VEML6040 Init structure definition.
 */
typedef struct {
    veml6040_integration_time_t integration_time; /*!< set integration time  */
    veml6040_trigger_t trigger; /*!< set  trigger  */
    veml6040_mode_t mode; /*!< set  mode  */
    veml6040_switch_t switch_en; /*!< set if enable  */
} veml6040_config_t;

typedef void* veml6040_handle_t;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * class of vmel6040 Rgbw  sensor
 * simple usage:
 * CVeml6040 *veml6040 = new CVeml6040(&i2c_bus);
 * veml6040.red();
 * ......
 * delete(veml6040);
 */
class CVeml6040: public DataAcquirer {
    private:
        veml6040_handle_t m_sensor_handle;
        light_sensor_data_t data;
        TaskHandle_t taskHandle;

        /**
         * prevent copy constructing
         */
        CVeml6040(const CVeml6040&);
        CVeml6040& operator =(const CVeml6040&);

    public:
        /**
         * @brief constructor of CVeml6040
         *
         * @param p_i2c_bus pointer to CI2CBus object
         * @param addr slave device address
         */
        CVeml6040(CI2CBus *p_i2c_bus, DoubleBuffer *p_buffer, uint8_t addr = VEML6040_I2C_ADDRESS);

        /**
         * @brief   delete veml6040 handle_t
         *
         */
        ~CVeml6040();

        esp_err_t write(veml6040_handle_t sensor, uint8_t config_val);
        esp_err_t read(veml6040_handle_t sensor, uint8_t cmd_code);

        /**
         * @brief  set veml6040 mode
         *
         * @param  device info
         *
         * @return
         *     - ESP_OK Success
         *     - ESP_FAIL Fail
         */
        esp_err_t set_mode(veml6040_handle_t sensor, veml6040_config_t * device_info);

        /**
         * @brief Get the current red light brightness value
         * @return
         *    - red value
         *    - -1 if fail to read sensor
         */
        int red();

        /**
         * @brief Get the current green light brightness value
         * @return
         *    - green valueveml6040_info
         */
        int green();

        /**
         * @brief Get the current blue light brightness value
         * @return
         *    - blue value
         *    - -1 if fail to read sensor
         */
        int blue();

        /**
         * @brief Get the current white light brightness value
         * @return
         *    - white value
         *    - -1 if fail to read sensor
         */
        int white();

        /*
        * @brief Get LUX value
        * @return
        *    - LUX value
        */
        float lux();

        /*
        * @brief Get color temperature
        * @param  set offset
        * @return
        *    - white value
        *
        * In open-air conditions the offset = 0.5. Depending on the optical conditions (e.g. cover glass) this offset may change.
        */
        uint16_t cct(float offset);

        void init();
        void readData();
        light_sensor_data_t getData();
        void showData();

        void sendDataToBuffer();
        static void taskWrapper(void* pvParameters);
        void startAcquisition();
        void stopAcquisition();
};
#endif

#endif

