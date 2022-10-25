/* test_t9062.c:
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <unity.h>
#include <limits.h>

#include "t9062.h"

#define CONFIG_I2C_SDA_PIN 21
#define CONFIG_I2C_SCL_PIN 22
#define CONFIG_SENSOR_POWER_PIN 23
#define CONFIG_SENSOR_ADDRESS 0x28
#define CONFIG_SENSOR_NEW_ADDRESS 0x28

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

TEST_CASE("read sensor", "[t9062_read]") {

  i2c_param_config(t9062_sensor.i2c_port, &i2c_config);
  i2c_driver_install(t9062_sensor.i2c_port, i2c_config.mode, 0, 0, 0);
  t9062_set_sensor_power(&t9062_sensor, 1);
  vTaskDelay(pdMS_TO_TICKS(200));
  TEST_ASSERT_EQUAL(0, t9062_read(&t9062_sensor));
  i2c_driver_delete(t9062_sensor.i2c_port);
}