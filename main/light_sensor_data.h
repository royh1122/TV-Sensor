/**
 * @brief Struct representing data acquired from a light sensor.
 */
typedef struct {
    int red; /**< The red component of the light sensor data. */
    int green; /**< The green component of the light sensor data. */
    int blue; /**< The blue component of the light sensor data. */
    int white; /**< The white component of the light sensor data. */
    float lux; /**< The lux value of the light sensor data. */
    uint16_t cct; /**< The correlated color temperature (CCT) of the light sensor data. */
} light_sensor_data_t;