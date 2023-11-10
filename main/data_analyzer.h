/**
 * @file data_analyzer.h
 * @brief This file contains the declaration of the DataAnalyzer class, which is an abstract class
 *        that provides an interface for analyzing data stored in a DoubleBuffer object.
 */

#ifndef DATA_ANALYZER_H_
#define DATA_ANALYZER_H_

#include <stdlib.h>
#include <math.h>
#include "driver/i2c.h"
#include "i2c_bus.h"
#include "unity.h"
#include "freertos/task.h"
#include "veml6040.h"

/**
 * @brief The DataAnalyzer class is an abstract class that provides an interface for analyzing
 *        data stored in a DoubleBuffer object.
 */
class DataAnalyzer {
    private:
        CI2CBus *i2c_bus; /**< Pointer to the CI2CBus object. */
        DoubleBuffer *buffer; /**< Pointer to the DoubleBuffer object. */

    public:
        /**
         * @brief Constructor for the DataAnalyzer class.
         * @param p_i2c_bus Pointer to a CI2CBus object.
         * @param p_buffer Pointer to a DoubleBuffer object.
         */
        DataAnalyzer(CI2CBus *p_i2c_bus, DoubleBuffer *p_buffer) : i2c_bus(p_i2c_bus), buffer(p_buffer) {};

        /**
         * @brief Destructor for the DataAnalyzer class.
         */
        virtual ~DataAnalyzer() {};

        /**
         * @brief Initializes the data analyzer.
         * @return True if initialization is successful, false otherwise.
         */
        virtual bool init() = 0;

        /**
         * @brief Reads data from the DoubleBuffer object.
         */
        virtual void readDataFromBuffer() = 0;

        /**
         * @brief Starts the data analysis process.
         */
        virtual void startAnalysis() = 0;

        /**
         * @brief Stops the data analysis process.
         */
        virtual void stopAnalysis() = 0;

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

#endif /* DATA_ANALYZER_H_ */