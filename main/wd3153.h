/**
 * @file wd3153.h
 * @brief Header for WD3153 LED driver chip
*/

#ifdef __cplusplus 
extern "C" {
#endif

#include "esp_log.h"
#include "data_analyzer.h"

#define WD3153_I2C_ADDRESS    (0x45)
#define WD3153_I2C_ERR_RES    (-1) 
#define WD3153_CHIP_ID        (0x33)

#ifdef __cplusplus
}
#endif

/**
 * @brief WD3153 class, a driver for the WD3153 light sensor and LED controller.
 *        It inherits from the DataAnalyzer class, which provides a double buffer for
 *        storing the sensor readings.
 */
class WD3153 : public DataAnalyzer {
	private:
		uint8_t address;          // I2C address of the WD3153 chip
		uint8_t chipID;           // ID of the WD3153 chip
		TaskHandle_t taskHandle;  // Handle of the task that reads data from the chip
		light_sensor_data_t r_data[BUFFER_SIZE];  // Buffer for storing sensor readings

		/**
         * prevent copy constructing
         */
		WD3153(const WD3153&);
		WD3153& operator=(const WD3153&);

		/**
		 * @brief Read a register from the WD3153 chip.
		 * @param reg The register address to read.
		 * @return The value of the register.
		 */
		inline uint8_t readReg(uint8_t reg);

		/**
		 * @brief Write a value to a register of the WD3153 chip.
		 * @param reg The register address to write.
		 * @param data The value to write.
		 */
		inline void writeReg(uint8_t reg, uint8_t data);

	public:
		/**
		 * @brief Construct a new WD3153 object.
		 * @param p_i2c_bus Pointer to the I2C bus used to communicate with the chip.
		 * @param p_buffer Pointer to the double buffer used to store sensor readings.
		 * @param addr I2C address of the chip (default: WD3153_I2C_ADDRESS).
		 */
		WD3153(CI2CBus* p_i2c_bus, DoubleBuffer* p_buffer, uint8_t addr = WD3153_I2C_ADDRESS);

		/**
		 * @brief Set the chip ID of the WD3153 object.
		 * @param chipID The ID to set.
		 */
		void setChipID(uint8_t chipID);

		/**
		 * @brief Get the chip ID of the WD3153 object.
		 * @return The chip ID.
		 */
		uint8_t getChipID();

		/**
		 * @brief Reset the WD3153 chip.
		 */
		void reset();

		/**
		 * @brief Enable or disable the WD3153 chip.
		 * @param state The state to set.
		 */
		void setChipEnabled(bool state);

		/**
		 * @brief Get the enabled state of the WD3153 chip.
		 * @return True if enabled, false if disabled.
		 */
		bool isChipEnabled();

		/**
		 * @brief Enable or disable the UVLO interrupt of the WD3153 chip.
		 * @param state The state to set.
		 */
		void setUVLOInterruptEnabled(bool state);

		/**
		 * @brief Get the enabled state of the UVLO interrupt of the WD3153 chip.
		 * @return True if enabled, false if disabled.
		 */
		bool isUVLOInterruptEnabled();

		/**
		 * @brief Enable or disable the OTP interrupt of the WD3153 chip.
		 * @param state The state to set.
		 */
		void setOTPInterruptEnabled(bool state);

		/**
		 * @brief Get the enabled state of the OTP interrupt of the WD3153 chip.
		 * @return True if enabled, false if disabled.
		 */
		bool isOTPInterruptEnabled();

		/**
		 * @brief Enable or disable the interrupt for a LED of the WD3153 chip.
		 * @param ledX The LED to set.
		 * @param state The state to set.
		 */
		void setLEDInterruptEnabled(uint8_t ledX, bool state);

		/**
		 * @brief Get the enabled state of the interrupt for a LED of the WD3153 chip.
		 * @param ledX The LED to query.
		 * @return True if enabled, false if disabled.
		 */
		bool isLEDInterruptEnabled(uint8_t ledX,bool state);

		// Interrupt Status

		/**
		 * @brief Set the Power On Reset (POR) interrupt status of the WD3153 chip.
		 * @param state The state to set.
		 */
		void setPORInterruptStatus(bool state);

		/**
		 * @brief Get the Power On Reset (POR) interrupt status of the WD3153 chip.
		 * @return True if the interrupt is active, false otherwise.
		 */
		bool getPORInterruptStatus();

		/**
		 * @brief Get the Under Voltage Lock Out (UVLO) interrupt status of the WD3153 chip.
		 * @return True if the interrupt is active, false otherwise.
		 */
		bool getUVLOInterruptStatus();

		/**
		 * @brief Get the Over Temperature (OTP) interrupt status ofthe WD3153 chip.
		 * @return True if the interrupt is active, false otherwise.
		 */
		bool getOTPInterruptStatus();

		/**
		 * @brief Set the interrupt status for a LED of the WD3153 chip.
		 * @param ledX The LED to set.
		 * @param state The state to set.
		 */
		void setLEDInterruptStatus(uint8_t ledX, bool state);

		/**
		 * @brief Get the interrupt status for a LED of the WD3153 chip.
		 * @param ledX The LED to query.
		 * @return True if the interrupt is active, false otherwise.
		 */
		bool getLEDInterruptStatus(uint8_t ledX);

		// Function Control

		/**
		 * @brief Enable or disable the OTP function of the WD3153 chip.
		 * @param state The state to set.
		 */
		void setOTPEnabled(bool state);

		/**
		 * @brief Get the enabled state of the OTP function of the WD3153 chip.
		 * @return True if enabled, false if disabled.
		 */
		bool isOTPEnabled();

		/**
		 * @brief Enable or disable the UVLO function of the WD3153 chip.
		 * @param state The state to set.
		 */
		void setUVLOEnabled(bool state);

		/**
		 * @brief Get the enabled state of the UVLO function of the WD3153 chip.
		 * @return True if enabled, false if disabled.
		 */
		bool isUVLOEnabled();

		/**
		 * @brief Enable or disable the charging function of the WD3153 chip.
		 * @param state The state to set.
		 */
		void setCHRGEnabled(bool state);

		/**
		 * @brief Get the enabled state of the charging function of the WD3153 chip.
		 * @return True if enabled, false if disabled.
		 */
		bool isCHRGEnabled();


		#define PWM250HZ 0
		#define PWM500HZ 1
		/**
		 * @brief Set the global PWM frequency level of the WD3153 chip.
		 * @param state The state to set.
		 */
		void setGlobalPWMFrequencyLevel(bool state);

		/**
		 * @brief Get the global PWM frequency level of the WD3153 chip.
		 * @return True if the high frequency level is selected, false if the low frequency level is selected.
		 */
		bool getGlobalPWMFrequencyLevel();

		#define EasingLinear 0
		#define EasingLog 1
		/**
		 * @brief Set the global PWM easing function of the WD3153 chip.
		 * @param state The state to set.
		 */
		void setGlobalPWMEasingFunction(bool state);

		/**
		 * @brief Get the global PWM easing function of the WD3153 chip.
		 * @return True if the log easing function is selected, false if the linear easing function is selected.
		 */
		bool getGlobalPWMEasingFunction();

		// LED Enabled

		/**
		 * @brief Enable or disable a LED of the WD3153 chip.
		 * @param ledX The LED to set.
		 * @param state The state to set.
		 */
		void setLEDEnabled(uint8_t ledX, bool state);

		/**
		 * @brief Get the enabled state of a LED of the WD3153 chip.
		 * @param ledX The LED to query.
		 * @return True if enabled, false if disabled.
		 */
		bool isLEDEnabled(uint8_t ledX);

		// Fade In/Out Mode

		/**
		 * @brief Enable or disable the fade-out mode of a LED of the WD3153 chip.
		 * @param ledX The LED to set.
		 * @param state The state to set.
		 */
		void setLEDFadeOutEnabled(uint8_t ledX, bool state);

		/**
		 * @brief Get the enabled state of the fade-out mode of a LED of the WD3153 chip.
		 * @param ledX The LED to query.
		 * @return True if enabled, false if disabled.
		 */
		bool isLEDFadeOutEnabled(uint8_t ledX);

		/**
		 * @brief Enable or disable the fade-in mode of a LED of the WD3153 chip.
		 * @param ledX The LED to set.
		 * @param state The state to set.
		 */
		void setLEDFadeInEnabled(uint8_t ledX, bool state);

		/**
		 * @brief Get the enabled state of the fade-in mode of a LED of the WD3153 chip.
		 * @param ledX The LED to query.
		 * @return True if enabled, false if disabled.
		 */
		bool isLEDFadeInEnabled(uint8_t ledX);


		#define DirectControlMode 0
		#define ProgrammableLightingMode 1
		/**
		 * @brief Sets the mode of a specific LED.
		 * 
		 * @param ledX The index of the LED to set the mode for.
		 * @param mode The desired mode for the LED. Use DirectControlMode or ProgrammableLightingMode.
		 */
		void setLEDMode(uint8_t ledX, bool mode);

		/**
		 * @brief Gets the mode of a specific LED.
		 * 
		 * @param ledX The index of the LED to get the mode for.
		 * @return The current mode of the LED. Returns DirectControlMode or ProgrammableLightingMode.
		 */
		bool getLEDMode(uint8_t ledX);


		#define LED0mA 0b00000000
		#define LED5mA 0b00000001
		#define LED10mA 0b00000010
		#define LED15mA 0b00000011
		#define LED25mA 0b00000100
		/**
		 * @brief Sets the current limit of a specific LED.
		 * 
		 * @param ledX The index of the LED to set the current limit for.
		 * @param currentLimit The desired current limit for the LED. Use one of the LED*_mA constants.
		 */
		void setLEDCurrentLimit(uint8_t ledX, uint8_t currentLimit);

		/**
		 * @brief Gets the current limit of a specific LED.
		 * 
		 * @param ledX The index of the LED to get the current limit for.
		 * @return The current current limit of the LED. Returns one of the LED*_mA constants.
		 */
		uint8_t getLEDCurrentLimit(uint8_t ledX);

		/**
		 * @brief Sets the duty cycle of a specific LED.
		 * 
		 * @param ledX The index of the LED to set the duty cycle for.
		 * @param duty The desired duty cycle for the LED. Should be between 0 and 255.
		 */
		void setLEDPWM(uint8_t ledX, uint8_t duty);

		/**
		 * @brief Gets the duty cycle of a specific LED.
		 * 
		 * @param ledX The index of the LED to get the duty cycle for.
		 * @return The current duty cycle of the LED. Returns a value between 0 and 255.
		 */
		uint8_t getLEDPWM(uint8_t ledX);

		/**
		 * @brief Sets the duty cycle for all LEDs in a group.
		 * 
		 * @param duty The desired duty cycle for all LEDs in the group. Should be between 0 and 255.
		 */
		void setGroupPWM(uint8_t duty);

		#define T0 0
		#define T1 1
		#define T2 2
		#define T3 3
		#define T4 4
		
		#define T0s 0b00000000
		#define T0_13s 0b00000001
		#define T0_26s 0b00000010
		#define T0_52s 0b00000011
		#define T1_04s 0b00000100
		#define T2_08s 0b00000101
		#define T4_16s 0b00000110
		#define T8_32s 0b00000111
		#define T16_64s 0b00001000
		/**
		 * @brief Sets the time value for a specific time register of a specific LED.
		 * 
		 * @param ledX The index of the LED to set the time register for.
		 * @param timeX The index of the time register to set.
		 * @param timeValue The desired time value for the time register. Use one of the T*_s constants.
		 */
		void setLEDTime(uint8_t ledX, uint8_t timeX, uint8_t timeValue);

		/**
		 * @brief Gets the time value for a specific time register of a specific LED.
		 * 
		 * @param ledX The index of the LED to get the time register for.
		 * @param timeX Theindex of the time register to get.
		 * @return The current time value of the time register. Returns one of the T*_s constants.
		 */
		uint8_t getLEDTime(uint8_t ledX, uint8_t timeX);

		/**
		 * @brief Sets the number of times a specific LED should blink.
		 * 
		 * @param ledX The index of the LED to set the blink times for.
		 * @param repeatTimes The desired number of times the LED should blink. Use a value between 0 and 15, or 0 for infinite blinking.
		 */
		void setLEDBlinkTimes(uint8_t ledX, uint8_t repeatTimes);

		/**
		 * @brief Gets the number of times a specific LED should blink.
		 * 
		 * @param ledX The index of the LED to get the blink times for.
		 * @return The current number of times the LED should blink. Returns a value between 0 and 15, or 0 for infinite blinking.
		 */
		uint8_t getLEDBlinkTimes(uint8_t ledX);

		/**
		 * @brief Enables or disables synchronization between LEDs.
		 * 
		 * @param state Whether synchronization should be enabled (true) or disabled (false).
		 */
		void setSyncEnabled(bool state);

		/**
		 * @brief Gets the current synchronization state.
		 * 
		 * @return Whether synchronization is currently enabled (true) or disabled (false).
		 */
		bool isSyncEnabled();

		/**
		 * @brief Sets the I2C address of the LED driver.
		 * 
		 * @param addr The desired I2C address of the LED driver. Should be between 0 and 127.
		 */
		void setI2CAddress(uint8_t addr);

		/**
		 * @brief Gets the current I2C address of the LED driver.
		 * 
		 * @return The current I2C address of the LED driver. Returns a value between 0 and 127.
		 */
		uint8_t getI2CAddress();

		/**
		 * @brief Initializes the LED driver.
		 * 
		 * @return Whether initialization was successful (true) or not (false).
		 */
		bool init();

		/**
		 * @brief Reads data from the LED driver buffer.
		 */
		void readDataFromBuffer();

		/**
		 * @brief A wrapper function for the task that analyzes LED sensor data.
		 * 
		 * @param pvParameters Not used.
		 */
		static void taskWrapper(void* pvParameters);

		/**
		 * @brief Calculates the average values of various sensor data.
		 * 
		 * @param data An array of sensor data to calculate the averages for.
		 * @param r Pointer to the variable where the red channel average should be stored.
		 * @param g Pointer to the variable where the green channel average should be stored.
		 * @param b Pointer to the variable where the blue channel average should be stored.
		 * @param w Pointer to the variable where the white channel average should be stored.
		 * @param lux Pointer to the variable where the lux value average should be stored.
		 * @param cct Pointer to the variable where the color temperature average should be stored.
		 */
		void average(light_sensor_data_t data[],float *r,float *g,float *b,float *w,float *lux,uint16_t *cct);

		/**
		 * @brief Starts analysis of LED sensor data.
		 */
		void startAnalysis();

		/**
		 * @brief Stops analysis of LED sensor data.
		 */
		void stopAnalysis();

		/**
		 * @brief Sets the duty cycle of all LEDs to create a RGB cycling effect.
		 * 
		 * @param dutyCycle The desired duty cycle for all LEDs.
		 */
		void rgbCycle(uint8_t dutyCycle);

		/**
		 * @brief Blinks all LEDs once.
		 */
		void blink();
};