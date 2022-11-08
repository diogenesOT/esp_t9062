#ifndef T9062_C
#define T9062_C

#include "esp_t9062.h"

#include <driver/gpio.h>
#include <esp_log.h>

#define COMMAND_MODE_RETURN_SIZE 2
#define COMMAND_MODE_DATA_RETURN_SIZE 3
#define I2C_TIMEOUT_MS 50
#define NORMAL_MODE_RETURN_SIZE 4
#define RH_HIGH_MASK 0x003F
#define READ_MAX_RETRY 5
#define STARTUP_DELAY_MS 70

enum sensor_mode { NORMAL_MODE = 0, COMMAND_MODE = 2 };

const char *TAG = "esp_t9062.h";
int8_t enter_command_mode(t9062_sensor_t *sensor);
int8_t enter_normal_mode(t9062_sensor_t *sensor);
int8_t check_response(uint8_t status_byte, uint8_t sensor_mode);
void print_status(uint8_t status);
void print_response(uint8_t response);
void print_diagnostics(uint8_t diagnostics_bits);
void print_custom_configuration(uint16_t config);

int8_t t9062_read(t9062_sensor_t *sensor) {
  esp_err_t ret;
  uint16_t humidity_raw = 0;
  uint16_t temperature_raw = 0;
  uint8_t i;
  for (i = 0; i < READ_MAX_RETRY; i++) {
    ret = i2c_master_read_from_device(sensor->i2c_port, sensor->address, &sensor->raw_data[0], NORMAL_MODE_RETURN_SIZE,
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret == ESP_OK) {
      ret = check_response(sensor->raw_data[0], NORMAL_MODE);
      if (ret == 4) {
        ESP_LOGW(TAG, "[t9062_read()] sensor not in normal mode: 0x%03X(adr:0x%02X)", ret, sensor->address);
        enter_normal_mode(sensor);
        vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));
        continue;
      } else if (ret != 0) {
        ESP_LOGW(TAG, "[t9062_read()] error check_response returned: 0x%03X(adr:0x%02X)", ret, sensor->address);
        continue;
      } else {
        humidity_raw = ((sensor->raw_data[0] & RH_HIGH_MASK) << 8) | sensor->raw_data[1];
        sensor->humidity = (float)(humidity_raw / 16384.0 * 100.0);
        temperature_raw = (sensor->raw_data[2] << 6) | (sensor->raw_data[1] >> 2);
        sensor->temperature = (float)((temperature_raw / 16384.0 * 165.0) - 40.0);
        sensor->time_of_measurement = esp_timer_get_time();
        return 0;
      }
    } else {
      ESP_LOGE(TAG, "[t9062_read()] fatal error : 0x%03X(adr:0x%02X)", ret, sensor->address);
    }
  }
  ESP_LOGE(TAG, "[t9062_read()] error: 0x%03X(adr:0x%02X)", ret, sensor->address);
  return 1;
}

int8_t t9062_read_register(t9062_sensor_t *sensor, uint8_t register_id, uint16_t *register_data) {
  esp_err_t ret;
  uint8_t cmd[] = {0x00, 0x00, 0x00};
  uint8_t ans[] = {9, 9, 9};
  uint8_t register_address = register_id + T9062_REGISTER_ADDRESS_OFFSET;
  cmd[0] = register_address;

  ret = enter_command_mode(sensor);
  if (ret != 0) {
    ESP_LOGE(TAG, "[t9062_read_register()] enter command mode failed");
    return 1;
  }
  ret = i2c_master_write_to_device(sensor->i2c_port, sensor->address, cmd, sizeof(cmd), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG,
             "[t9062_read_register()] error %i sending read register command. "
             "register: %s [0x%02X] command: [0x%02X] "
             "[0x%02X] [0x%02X]",
             ret, t9062_register_name[register_address], register_address, cmd[0], cmd[1], cmd[2]);
    return 1;
  }
  ret =
      i2c_master_read_from_device(sensor->i2c_port, sensor->address, &ans[0], COMMAND_MODE_DATA_RETURN_SIZE, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG,
             "[t9062_read_register()] read %s register returned %i. command: "
             "[0x%02X] [0x%02X] [0x%02X] data: "
             "[0x%02X] [0x%02X] [0x%02X]",
             t9062_register_name[register_address], ret, cmd[0], cmd[1], cmd[2], ans[0], ans[1], ans[2]);
    return 2;
  }
  ret = check_response(ans[0], COMMAND_MODE);
  if (ret != 0) {
    ESP_LOGE(TAG, "[t9062_read_register()] check_response error");
    return 3;
  }
  *register_data = (uint16_t)(ans[1] << 8) | (ans[2]);
  return 0;
}

int8_t t9062_write_register(t9062_sensor_t *sensor, uint8_t register_id, uint16_t register_data) {
  esp_err_t ret;
  uint8_t cmd[3];
  uint8_t ans[] = {9, 9, 9};
  uint8_t register_address = register_id + T9062_REGISTER_ADDRESS_OFFSET;

  ret = enter_command_mode(sensor);
  if (ret != 0) {
    ESP_LOGE(TAG, "[t9062_write_register()] error %i enter command mode", ret);
    return 1;
  }
  cmd[0] = register_address + 0x40;
  cmd[1] = (uint8_t)(register_data >> 8);
  cmd[2] = (uint8_t)(register_data & 0x00FF);
  ret = i2c_master_write_to_device(sensor->i2c_port, sensor->address, cmd, sizeof(cmd), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "[t9062_write_register()] error %i write register. register: %s [0x%02X]", ret, t9062_register_name[register_address],
             register_address);
    return 2;
  }
  vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));  // wait for command to execute (datasheet sais 12ms)
  ret = i2c_master_read_from_device(sensor->i2c_port, sensor->address, &ans[0], COMMAND_MODE_RETURN_SIZE, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "[t9062_write_register()] error %i reading register. register: %s [0x%02X]", ret, t9062_register_name[register_address],
             register_address);
    return 3;
  }
  ret = check_response(ans[0], COMMAND_MODE);
  if (ret != 0) {
    ESP_LOGE(TAG, "[t9062_write_register()] error %i check_response error", ret);
    return 4;
  }
  return 0;
}

int8_t t9062_change_address(t9062_sensor_t *sensor, uint8_t new_address) {
  esp_err_t ret;
  uint8_t cmd[] = {0x00, 0x00, 0x00};
  uint8_t ans[] = {9, 9, 9};
  uint8_t register_address = T9062_REGISTER_ADDRESS_OFFSET + T9062_REG_CUST_CONFIG;
  uint16_t register_data;

  if ((new_address < 0x08) | (new_address > 0x7F)) {
    ESP_LOGE(TAG, "[t9062_change_address()] error illegal new address [0x%02X].", new_address);
    return 1;
  }
  ret = enter_command_mode(sensor);
  if (ret != 0) {
    ESP_LOGE(TAG, "[t9062_change_address()] enter command mode failed");
    return 2;
  }
  ret = t9062_read_register(sensor, T9062_REG_CUST_CONFIG, &register_data);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "[t9062_change_address()] error %i read register command. register: %s", ret, t9062_register_name[T9062_REG_CUST_CONFIG]);
    return 3;
  }
  if ((register_data & 0x003F) != sensor->address) {
    ESP_LOGE(TAG,
             "[t9062_change_address()] error sensor address and "
             "register data not matching.");
    return 3;
  }
  // create write new address command
  cmd[0] = register_address + 0x40;
  cmd[1] = register_data >> 8;
  cmd[2] = (new_address & 0x7F) | (((uint8_t)register_data) & 0x80);
  ret = i2c_master_write_to_device(sensor->i2c_port, sensor->address, cmd, sizeof(cmd), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "[t9062_change_address()] error %i write to device. register: %s", ret, t9062_register_name[T9062_REG_CUST_CONFIG]);
    return 4;
  }
  vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));  // wait write commands execution time
  ret = i2c_master_read_from_device(sensor->i2c_port, sensor->address, &ans[0], COMMAND_MODE_RETURN_SIZE, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "[t9062_change_address()] error %i read from device. register: %s", ret, t9062_register_name[T9062_REG_CUST_CONFIG]);
    return 5;
  }
  ret = check_response(ans[0], COMMAND_MODE);
  if (ret != 0) {
    ESP_LOGE(TAG, "[t9062_change_address()] check_response failed. ");
    return 5;
  }
  // read register again to confirm changed address
  register_data = 0;
  ret = t9062_read_register(sensor, T9062_REG_CUST_CONFIG, &register_data);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "[t9062_change_address()] error %i read register command. register: %s", ret, t9062_register_name[T9062_REG_CUST_CONFIG]);
    return 6;
  }
  if ((register_data & 0x003F) != new_address) {
    ESP_LOGE(TAG,
             "[t9062_change_address()] error new address and "
             "register data not matching.");
    return 6;
  }
  // connect to new address
  ret = enter_normal_mode(sensor);
  if ((ret != 0) & (ret != 1)) {
    // since the final check of the enter_normal_mode() fails the function
    // returns 1 instead of 0 due to the new address in NORMAL_MODE
    ESP_LOGE(TAG, "[t9062_change_address()] enter_normal_mode() failed.");
    return 7;
  }
  sensor->address = new_address;  // update address in sensor struct - lets see
                                  // if thats a good idea
  vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));
  t9062_read(sensor);  // read in normal mode

  return 0;
}

esp_err_t t9062_set_sensor_power(t9062_sensor_t *sensor, bool state) {
  esp_err_t err;
  err = gpio_set_direction(sensor->sensor_power_pin, GPIO_MODE_OUTPUT);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "error gpio_set_direction");
    return err;
  }
  err = gpio_set_level(sensor->sensor_power_pin, state);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "error gpio_set_level");
    return err;
  }
  return err;
}

void t9062_print_sensor_information(t9062_sensor_t *sensor, uint16_t *t9062_registers) {
  char byte_as_binary[8];
  // print information
  ESP_LOGI(TAG, "SENSOR INFORMATION:");
  print_status(sensor->raw_data[0] >> 6);
  print_response(sensor->raw_data[0] & 0x03);
  itoa(sensor->raw_data[0], byte_as_binary, 2);
  ESP_LOGI(TAG, "byte 1: 0b%8s", byte_as_binary);
  itoa(sensor->raw_data[1], byte_as_binary, 2);
  ESP_LOGI(TAG, "byte 2: 0b%8s", byte_as_binary);
  itoa(sensor->raw_data[2], byte_as_binary, 2);
  ESP_LOGI(TAG, "byte 3: 0b%8s", byte_as_binary);
  itoa(sensor->raw_data[3], byte_as_binary, 2);
  ESP_LOGI(TAG, "byte 4: 0b%8s", byte_as_binary);
}

uint8_t t9062_get_status(uint8_t status_byte) { return status_byte >> 6; }

/**
 * @brief Restart the sensor and enter command mode.
 * @attention To enter the command mode we must send the enter command mode command within the first 10ms (3ms) after
 * startup. Since the pulled up data and clock line are sometimes enough to power the sensor we need to set the i2c bus
 * low too.
 * @param sensor: Struct representing the sensor.
 * @return  0: success
 *          1: fail
 */
int8_t enter_command_mode(t9062_sensor_t *sensor) {
  int8_t ret;
  uint8_t cmd[] = {0x00, 0x00, 0x00};
  uint8_t ans[] = {9, 9, 9};

  ret = i2c_master_read_from_device(sensor->i2c_port, sensor->address, &ans[0], COMMAND_MODE_RETURN_SIZE, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  if ((ret == ESP_OK) & (t9062_get_status(ans[0]) == 2)) {
    return 0;  // already in command mode
  }
  ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_driver_delete(sensor->i2c_port));
  ret = t9062_set_sensor_power(sensor, false);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "[enter_command_mode] error t9062_set_sensor_power");
  }
  gpio_set_direction(sensor->i2c_config->sda_io_num, GPIO_MODE_OUTPUT);
  gpio_set_direction(sensor->i2c_config->scl_io_num, GPIO_MODE_OUTPUT);
  gpio_set_level(sensor->i2c_config->sda_io_num, 0);
  gpio_set_level(sensor->i2c_config->scl_io_num, 0);
  vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));
  ret = t9062_set_sensor_power(sensor, true);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "[enter_command_mode] error t9062_set_sensor_power");
  }
  i2c_param_config(sensor->i2c_port, sensor->i2c_config);
  i2c_driver_install(sensor->i2c_port, sensor->i2c_config->mode, 0, 0, 0);
  cmd[0] = 0xA0;
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      i2c_master_write_to_device(sensor->i2c_port, sensor->address, cmd, sizeof(cmd), pdMS_TO_TICKS(I2C_TIMEOUT_MS)));
  esp_rom_delay_us(105);
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      i2c_master_read_from_device(sensor->i2c_port, sensor->address, &ans[0], COMMAND_MODE_RETURN_SIZE, pdMS_TO_TICKS(I2C_TIMEOUT_MS)));

  if ((ans[0] >> 6) == COMMAND_MODE) {
    return 0;
  } else {
    ESP_LOGE(TAG, "enter command mode failed");
    return 1;
  }
}

/**
 * @brief Enter normal mode.
 * @param sensor: Struct representing the sensor.
 * @return    0: success
 *            1: fail
 *           -1: fail
 */
int8_t enter_normal_mode(t9062_sensor_t *sensor) {
  int8_t ret;
  uint8_t cmd[] = {0x80, 0x00, 0x00};
  uint8_t ans[] = {9, 9, 9};
  ret = i2c_master_read_from_device(sensor->i2c_port, sensor->address, &ans[0], COMMAND_MODE_RETURN_SIZE, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  if ((ret == ESP_OK) & (t9062_get_status(ans[0]) < 2)) {
    return 0;  // already in normal mode
  } else if (ret != ESP_OK) {
    ESP_LOGE(TAG,
             "[enter_normal_mode()] error i2c_master_read_from_device returned "
             "%i. Data:  0x%02X",
             ret, ans[0]);
    return -1;
  }
  cmd[0] = 0x80;
  ret = i2c_master_write_to_device(sensor->i2c_port, sensor->address, cmd, sizeof(cmd), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "[enter_normal_mode()] i2c error");
    return -1;
  }
  vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));
  ret = i2c_master_read_from_device(sensor->i2c_port, sensor->address, &ans[0], COMMAND_MODE_RETURN_SIZE, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  if ((ret == ESP_OK) & (t9062_get_status(ans[0]) < 2)) {
    return 0;  // in normal mode
  } else if (ret != ESP_OK) {
    ESP_LOGW(TAG,
             "[enter_normal_mode()] error i2c_master_read_from_device returned "
             "%i. Data:  0x%02X",
             ret, ans[0]);
    return 1;
  }
  return 0;
}

/**
 * @brief Checks the status and response bits to verify a command or measurement.
 * @param status_byte: the first byte the sensor returns after a command
 * @param mode: (enum) command or normal mode
 * @return  0: success
 *          1: fail in command mode, stale data in normal mode
 *          4: fail in normal mode
 */
int8_t check_response(uint8_t status_byte, uint8_t mode) {
  int8_t err = 0;
  if (mode == NORMAL_MODE) {
    if (((status_byte >> 6) & 0x03) > 1) {
      // status error
      ESP_LOGW(TAG, "status bits check failed. 0x%02X", (status_byte >> 6));
      err += 4;
    }
    return err;
  } else if (mode == COMMAND_MODE) {
    /* return bool(status_byte == 0x81); */
    if (status_byte == 0x81) {
      return 0;
    } else {
      ESP_LOGW(TAG, "status bits check failed. 0x%02X", (status_byte));
      return 1;
    }
  }
  ESP_LOGE(TAG, "[check_response()] error. status_byte: 0x%02X mode: %i", status_byte, mode);
  return 1;
}

/**
 * @brief
 * @param var:
 * @return  0: success
 *          1:
 */
void print_status(uint8_t status) {
  if (status == 0) {
    ESP_LOGI(TAG, "chipcap2 status: valid_data");
  } else if (status == 1) {
    ESP_LOGI(TAG, "chipcap2 status: stale_data");
  } else if (status == 2) {
    ESP_LOGI(TAG, "chipcap2 status: command_mode");
  } else if (status == 3) {
    ESP_LOGI(TAG, "chipcap2 status: not used - this should not happen");
  } else {
    ESP_LOGI(TAG, "chipcap2 status: unknown");
  }
}

/**
 * @brief
 * @param var:
 * @return  0: success
 *          1:
 */
void print_response(uint8_t response) {
  if (response == 0) {
    ESP_LOGI(TAG, "chipcap2 response: busy");
  } else if (response == 1) {
    ESP_LOGI(TAG, "chipcap2 response: success");
  } else if (response == 2) {
    ESP_LOGI(TAG, "chipcap2 response: command_failed");
  } else {
    ESP_LOGI(TAG, "chipcap2 response: unknown");
  }
}

void print_diagnostics(uint8_t diagnostics_bits) {
  if ((diagnostics_bits & 0x02) != 0) {
    ESP_LOGI(TAG, "diagnostics bit: corrected eeprom error");
  }

  if ((diagnostics_bits & 0x04) != 0) {
    ESP_LOGI(TAG, "diagnostics bit: uncorrectable eeprom error");
  }
  if ((diagnostics_bits & 0x08) != 0) {
    ESP_LOGI(TAG, "diagnostics bit: RAM parity error");
  }
  if ((diagnostics_bits & 0x10) != 0) {
    ESP_LOGI(TAG, "diagnostics bit: configuration error");
  }
}

void print_custom_configuration(uint16_t config) {
  ESP_LOGI(TAG, "Custom Config: 0x%04X", (config));
  ESP_LOGI(TAG, "I2C address: 0x%02X", (config & 0x7F));
  if ((config >> 7) & 1) {
    ESP_LOGI(TAG, "Alarm_Low_Cfg Alarm Polarity: Active Low (1)");
  } else {
    ESP_LOGI(TAG, "Alarm_Low_Cfg Alarm Polarity: Active High (0)");
  }
  if ((config >> 8) & 1) {
    ESP_LOGI(TAG, "Alarm_Low_Cfg Output Configuration: Open Drain (1)");
  } else {
    ESP_LOGI(TAG, "Alarm_Low_Cfg Output Configuration: Full push-pull (0)");
  }
  if ((config >> 9) & 1) {
    ESP_LOGI(TAG, "Alarm_High_Cfg Alarm Polarity: Active Low (1)");
  } else {
    ESP_LOGI(TAG, "Alarm_High_Cfg Alarm Polarity: Active High (0)");
  }
  if ((config >> 10) & 1) {
    ESP_LOGI(TAG, "Alarm_High_Cfg Output Configuration: Open Drain (1)");
  } else {
    ESP_LOGI(TAG, "Alarm_High_Cfg Output Configuration: Full push-pull (0)");
  }
  if ((config >> 12) & 1) {
    ESP_LOGI(TAG, "Ready_Open_Drain Ready Pin: Open Drain (1)");
  } else {
    ESP_LOGI(TAG, "Ready_Open_Drain Ready Pin: Full push-pull (0)");
  }
  if ((config >> 13) & 1) {
    ESP_LOGI(TAG, "Fast Startup: Enabled (3ms)");
  } else {
    ESP_LOGI(TAG, "Fast Startup: Disabled (10ms)");
  }
}

#endif