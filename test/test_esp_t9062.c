/* test_esp_t9062.c:
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <limits.h>
#include <unity.h>

#include "esp_t9062.h"

#ifdef __cplusplus
extern "C" {
#endif


#define CONFIG_I2C_SDA_PIN 21
#define CONFIG_I2C_SCL_PIN 22
#define CONFIG_SENSOR_POWER_PIN 23
#define CONFIG_SENSOR_ADDRESS 0x28
#define CONFIG_SENSOR_NEW_ADDRESS 0x28
#define CONFIG_SENSOR_ADDRESS_FAIL 0x20

i2c_config_t i2c_config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = CONFIG_I2C_SDA_PIN,
    .scl_io_num = CONFIG_I2C_SCL_PIN,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000,
};

t9062_sensor_t t9062_sensor = {
    .i2c_port = 0,
    .address = CONFIG_SENSOR_ADDRESS,
    .time_of_measurement = 0,
    .sensor_power_pin = CONFIG_SENSOR_POWER_PIN,
    .i2c_config = &i2c_config,
};

TEST_CASE("set sensor power ON", "[set_sensor_power]") { TEST_ASSERT_EQUAL(0, t9062_set_sensor_power(&t9062_sensor, 1)); }

TEST_CASE("set sensor power OFF", "[set_sensor_power]") { TEST_ASSERT_EQUAL(0, t9062_set_sensor_power(&t9062_sensor, 0)); }

TEST_CASE("i2c driver installation", "[t9062]") {
  TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, i2c_param_config(t9062_sensor.i2c_port, &i2c_config), "i2c_param_config() failed");
  TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, i2c_driver_install(t9062_sensor.i2c_port, i2c_config.mode, 0, 0, 0), "i2c_driver_install() failed");
  TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, i2c_driver_delete(t9062_sensor.i2c_port), "i2c_driver_delete() failed");
}

TEST_CASE("read sensor", "[t9062]") {
  i2c_param_config(t9062_sensor.i2c_port, &i2c_config);
  i2c_driver_install(t9062_sensor.i2c_port, i2c_config.mode, 0, 0, 0);
  t9062_set_sensor_power(&t9062_sensor, 1);
  vTaskDelay(pdMS_TO_TICKS(70));  // wait for sensor to power up

  t9062_sensor.address = CONFIG_SENSOR_ADDRESS;
  TEST_ASSERT_EQUAL(0, t9062_read(&t9062_sensor));
  TEST_ASSERT_MESSAGE(t9062_sensor.temperature != -40.0, "the temperature value is -40");

  i2c_driver_delete(t9062_sensor.i2c_port);
}

TEST_CASE("read sensor - wrong address", "[t9062]") {
  i2c_param_config(t9062_sensor.i2c_port, &i2c_config);
  i2c_driver_install(t9062_sensor.i2c_port, i2c_config.mode, 0, 0, 0);
  t9062_set_sensor_power(&t9062_sensor, 1);
  vTaskDelay(pdMS_TO_TICKS(70));  // wait for sensor to power up

  t9062_sensor.address = CONFIG_SENSOR_ADDRESS_FAIL;
  TEST_ASSERT_EQUAL(1, t9062_read(&t9062_sensor));
  t9062_sensor.address = CONFIG_SENSOR_ADDRESS;

  i2c_driver_delete(t9062_sensor.i2c_port);
}

TEST_CASE("read register", "[t9062]") {
  int32_t ret;
  uint16_t register_data[T9062_REGISTERS_SIZE];
  uint16_t i = 0;

  i2c_param_config(t9062_sensor.i2c_port, &i2c_config);
  i2c_driver_install(t9062_sensor.i2c_port, i2c_config.mode, 0, 0, 0);
  t9062_set_sensor_power(&t9062_sensor, 1);
  vTaskDelay(pdMS_TO_TICKS(70));  // wait for sensor to power up

  for (i = 0; i < T9062_REGISTERS_SIZE; i++) {
    register_data[i] = 0xFFFF;
    ret = t9062_read_register(&t9062_sensor, i, &register_data[i]);
    TEST_ASSERT_EQUAL(0, ret);
    TEST_ASSERT_MESSAGE(register_data[i] != 0xFFFF, "no data read");
  }

  i2c_driver_delete(t9062_sensor.i2c_port);
}

#ifdef __cplusplus
}
#endif