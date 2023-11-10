/**
 * @file data_acquirer.h
 * @brief This file contains the declaration of the DataAcquirer class, which is an abstract class
 *        that provides an interface for acquiring data and storing it in a DoubleBuffer object.
 */

#ifndef DATA_ACQUIRER_H_
#define DATA_ACQUIRER_H_

#include <stdlib.h>
#include <math.h>
#include "driver/i2c.h"
#include "i2c_bus.h"
#include "unity.h"
#include "freertos/task.h"
#include "double_buffer.h"

/**
 * @brief The DataAcquirer class is an abstract class that provides an interface for acquiring data
 *        and storing it in a DoubleBuffer object.
 */
class DataAcquirer {
    private:
        CI2CBus *i2c_bus; /**< Pointer to the CI2CBus object. */
        DoubleBuffer *buffer; /**< Pointer to the DoubleBuffer object. */
        
    public:
        /**
         * @brief Constructor for the DataAcquirer class.
         * @param p_i2c_bus Pointer to a CI2CBus object.
         * @param p_buffer Pointer to a DoubleBuffer object.
         */
        DataAcquirer(CI2CBus *p_i2c_bus, DoubleBuffer *p_buffer) : i2c_bus(p_i2c_bus), buffer(p_buffer) {};

        /**
         * @brief Destructor for the DataAcquirer class.
         */
        virtual ~DataAcquirer() {};

        /**
         * @brief Initializes the data acquirer object.
         */
        virtual void init() = 0;

        /**
         * @brief Reads data from the data source.
         */
        virtual void readData() = 0;

        /**
         * @brief Displays the acquired data.
         */
        virtual void showData() = 0;

        /**
         * @brief Sends the acquired data to the DoubleBuffer object.
         */
        virtual void sendDataToBuffer() = 0;

        /**
         * @brief Starts the data acquisition process.
         */
        virtual void startAcquisition() = 0;

        /**
         * @brief Stops the data acquisition process.
         */
        virtual void stopAcquisition() = 0;

        /**
         * @brief Getter method for the CI2CBus object.
         * @return A pointer to the CI2CBus object.
         */
        CI2CBus *getI2CBus() {
            return i2c_bus;
        }

        /**
         * @brief Getter method for the DoubleBuffer object.
         * @return A pointer to the DoubleBuffer object.
         */
        DoubleBuffer *getBuffer() {
            return buffer;
        }
};

#endif /* DATA_ACQUIRER_H_ */