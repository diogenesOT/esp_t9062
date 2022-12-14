#ifndef ESP_T9062_H
#define ESP_T9062_H
/**
 * esp t9062
 * @file esp_t9062.h
 * @brief Collection of functions to support the Amphenol T9062/ChipCap 2 Humidity and Temperature Sensor.
 *
 * The T9062 is an relative humidity and temperature sensor that can be connected via c²c bus.
 * specifications can be found under
 * https://f.hubspotusercontent40.net/hubfs/9035299/Documents/AAS-916-127J-Telaire-ChipCap2-022118-web.pdf
 *
 * This library has the following functions:
 *  -read sensor values
 *  -change the address of a sensor (*)
 *  -read/write the sensors registers (*)
 *
 * (*) For these functions the sensor power must be controlled or supplied by an gpio pin. This is because the sensor needs to retart to
 * enter command mode.
 *
 */

#include <driver/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

#define T9062_REGISTERS_SIZE 10
#define T9062_REGISTER_ADDRESS_OFFSET 0x16

typedef struct {
  float humidity;
  float temperature;
  uint8_t raw_data[4];
  uint8_t address;
  int64_t time_of_measurement;
  uint8_t i2c_port;
  uint8_t sensor_power_pin;
  i2c_config_t *i2c_config;
} t9062_sensor_t;

enum t9062_register_index {
  T9062_REG_PDM_CLIP_HIGH = 0,
  T9062_REG_PDM_CLIP_LOW,
  T9062_REG_ALARM_HIGH_ON,
  T9062_REG_ALARM_HIGH_OFF,
  T9062_REG_ALARM_LOW_ON,
  T9062_REG_ALARM_LOW_OFF,
  T9062_REG_CUST_CONFIG,
  T9062_REG_RESERVED,
  T9062_REG_CUST_ID2,
  T9062_REG_CUST_ID3,
};

static const char *const t9062_register_name[] = {[T9062_REG_PDM_CLIP_HIGH] = "PDM_CLIP_HIGH", [T9062_REG_PDM_CLIP_LOW] = "PDM_CLIP_LOW",
                                                  [T9062_REG_ALARM_HIGH_ON] = "ALARM_HIGH_ON", [T9062_REG_ALARM_HIGH_OFF] "ALARM_HIGH_OFF",
                                                  [T9062_REG_ALARM_LOW_ON] = "ALARM_LOW_ON",   [T9062_REG_ALARM_LOW_OFF] = "ALARM_LOW_OFF",
                                                  [T9062_REG_CUST_CONFIG] = "CUST_CONFIG",     [T9062_REG_RESERVED] = "RESERVED",
                                                  [T9062_REG_CUST_ID2] = "CUST_ID2",           [T9062_REG_CUST_ID3] = "CUST_ID3"};

/**
 * @brief Read measurements from ChipCap 2 sensor. In case of a failed query, the function returns the last occurred error.
 * @param sensor: Struct representing the sensor.
 * @return  0: success
 *          4: sensor error
 *          #: ESP error codes
 */
esp_err_t t9062_read(t9062_sensor_t *sensor);

/**
 * @brief Get the value of the measurements status bits.
 * @param status_byte: is the first byte returned by the sensor and stored in t9062_sensor_t.raw_data[0].
 * @return  0: valid data
 *          1: stale data
 *          2: command mode
 *          3: not used
 */
uint8_t t9062_get_status(uint8_t status_byte);

/**
 * @brief Change I²C address (7bit) of ChipCap 2 sensor.
 * @param sensor: Struct representing the sensor.
 * @param new_address: I²C address to be set.
 * @return  0: success
 *         -1: error
 *          #: ESP_ERROR_CODE
 */
esp_err_t t9062_change_address(t9062_sensor_t *sensor, uint8_t new_address);

/**
 * @brief Reads a registers from ChipCap 2 sensor.
 * @param sensor: Struct representing the sensor.
 * @param register_id: index of the register to be read.
 * @param register_data: buffer to write the register data.
 * @return  0: success
 *         -1: error
 *          #: ESP_ERROR_CODE
 */
esp_err_t t9062_read_register(t9062_sensor_t *sensor, uint8_t register_id, uint16_t *register_data);

/**
 * @brief Writes a register of ChipCap 2 sensor.
 * @param sensor: Struct representing the sensor.
 * @param register_id: index of the register to be written.
 * @param register_data: data to be written.
 * @return  0: success
 *         -1: error
 *          #: ESP_ERROR_CODE
 */
esp_err_t t9062_write_register(t9062_sensor_t *sensor, uint8_t register_id, uint16_t register_data);

/**
 * @brief Set power supply for ChipCap 2 sensor.
 * @param sensor: Struct representing the sensor.
 * @param state: Weather sitching power on(true) or off(false);
 * @return  0: success
 *          #: ESP_ERROR_CODE
 */
esp_err_t t9062_set_sensor_power(t9062_sensor_t *sensor, bool state);

void t9062_print_sensor_information(t9062_sensor_t *sensor, uint16_t *t9062_registers);

#ifdef __cplusplus
}
#endif

#endif