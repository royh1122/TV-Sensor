/**
 * @file double_buffer.h
 * @brief This file contains the declaration of the DoubleBuffer class, which is a class that
 *        provides a double buffer for storing data.
 */

#ifndef DOUBLE_BUFFER_H_
#define DOUBLE_BUFFER_H_

#include "light_sensor_data.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define BUFFER_SIZE 1024
#define BUFFER_COUNT 21

/**
 * @brief The DoubleBuffer class provides a double buffer for storing data.
 */
class DoubleBuffer {
    private:
        light_sensor_data_t buffer1[BUFFER_SIZE]; /**< First buffer. */
        light_sensor_data_t buffer2[BUFFER_SIZE]; /**< Second buffer. */
        light_sensor_data_t *write_buffer; /**< Pointer to the current write buffer. */
        light_sensor_data_t *read_buffer; /**< Pointer to the current read buffer. */
        size_t currentBufferIndex; /**< Current index of the write buffer. */
        SemaphoreHandle_t semaphore; /**< Binary semaphore for synchronization. */
        SemaphoreHandle_t readSemaphore; /**< Binary semaphore for read operations. */

    public:
        /**
         * @brief Constructor for the DoubleBuffer class.
         */
        DoubleBuffer() {
            write_buffer = buffer1;
            read_buffer = buffer2;
            currentBufferIndex = 0;

            // Create the binary semaphore for synchronization
            semaphore = xSemaphoreCreateBinary();
            readSemaphore = xSemaphoreCreateBinary();
        }

        /**
         * @brief Destructor for the DoubleBuffer class.
         */
        ~DoubleBuffer() {
            vSemaphoreDelete(semaphore);
        }

        /**
         * @brief Writes data to the current write buffer.
         * @param data Pointer to the data to be written.
         */
        void write(light_sensor_data_t *data) {
            memcpy((write_buffer + currentBufferIndex / sizeof(light_sensor_data_t)), data, sizeof(light_sensor_data_t));
            currentBufferIndex += sizeof(light_sensor_data_t);

            if (isFull()) {
                // The buffer is full, switch to the other buffer
                xSemaphoreGive(semaphore);
                switchBuffer();
                xSemaphoreTake(semaphore, portMAX_DELAY);
                xSemaphoreGive(readSemaphore);
            }
        }

        /**
         * @brief Reads the entire read buffer.
         * @param data Pointer to the buffer where the read data will be stored.
         */
        void read(light_sensor_data_t* data) {
            xSemaphoreTake(readSemaphore, portMAX_DELAY);
            memcpy(data, read_buffer, BUFFER_SIZE * sizeof(light_sensor_data_t));
            xSemaphoreGive(semaphore);
        }

        /**
         * @brief Switches to the other buffer.
         */
        void switchBuffer() {
            currentBufferIndex = 0;
            if (write_buffer == buffer1) {
                write_buffer = buffer2;
                read_buffer = buffer1;
            }else {
                write_buffer = buffer1;
                read_buffer = buffer2;
            }
        }

        /**
         * @brief Gets the write buffer.
         * @return Pointer to the current write buffer.
         */
        light_sensor_data_t* getWriteBuffer() { 
            return write_buffer + currentBufferIndex;
        }

        /**
         * @brief Gets the read buffer.
         * @return Pointer to the current read buffer.
         */
        light_sensor_data_t* getReadBuffer() {
            return read_buffer;
        }

        /**
         * @brief Gets the size of the buffer.
         * @return The size of the buffer.
         */
        size_t getSize() {
            return BUFFER_SIZE;
        }

        /**
         * @brief Waits for the buffer to be switched.
         */
        void waitSwitch() {
            xSemaphoreTake(semaphore, portMAX_DELAY);
            xSemaphoreGive(semaphore);
        }

        /**
         * @brief Checks if the buffer is full.
         * @return True if the buffer is full, false otherwise.
         */
        bool isFull() {
            return currentBufferIndex == BUFFER_SIZE * sizeof(light_sensor_data_t);
        }
};

#endif /* DOUBLE_BUFFER_H_ */