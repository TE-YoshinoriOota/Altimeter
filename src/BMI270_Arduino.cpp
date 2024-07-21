/*
  This file is part of the Arduino_LSM9DS1 library.
  Copyright (c) 2019 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Arduino.h"
#include "Wire.h"
#include "BMI270_Arduino.h"

BMI270Class::BMI270Class(TwoWire& wire) {
  _wire = &wire;
}


int BMI270Class::begin(uint8_t chip_id) {

  _wire->begin();

  bmi2.chip_id = chip_id;
  bmi2.read = bmi2_i2c_read;
  bmi2.write = bmi2_i2c_write;
  bmi2.delay_us = bmi2_delay_us;
  bmi2.intf = BMI2_I2C_INTF;
  bmi2.intf_ptr = &accel_gyro_dev_info;
  bmi2.read_write_len = 30; // Limitation of the Wire library
  bmi2.config_file_ptr = NULL; // Use the default BMI270 config file

  accel_gyro_dev_info._wire = _wire;
  accel_gyro_dev_info.dev_addr = bmi2.chip_id;


  int8_t bmi270InitResult = bmi270_init(&bmi2);
  print_rslt(bmi270InitResult);

  int8_t bmi270ConfigResult = configure_sensor(&bmi2);
  print_rslt(bmi270ConfigResult);

  bool success = bmi270InitResult == BMI2_OK && bmi270ConfigResult == BMI2_OK;
  _initialized = success;

  return success;
}


void BMI270Class::setContinuousMode() {
  bmi2_set_fifo_config(BMI2_FIFO_GYR_EN | BMI2_FIFO_ACC_EN, 1, &bmi2);
  continuousMode = true;
}

void BMI270Class::oneShotMode() {
  bmi2_set_fifo_config(BMI2_FIFO_GYR_EN | BMI2_FIFO_ACC_EN, 0, &bmi2);
  continuousMode = false;
}

// default range is +-4G, so conversion factor is (((1 << 15)/4.0f))
#define INT16_to_G   (8192.0f)

// Accelerometer
int BMI270Class::readAcceleration(float& x, float& y, float& z) {
  struct bmi2_sens_data sensor_data;
  auto ret = bmi2_get_sensor_data(&sensor_data, &bmi2);
  x = sensor_data.acc.x / INT16_to_G;
  y = sensor_data.acc.y / INT16_to_G;
  z = sensor_data.acc.z / INT16_to_G;
  return (ret == 0);
}

int BMI270Class::accelerationAvailable() {
  uint16_t status;
  bmi2_get_int_status(&status, &bmi2);
  int ret = ((status | _int_status) & BMI2_ACC_DRDY_INT_MASK);
  _int_status = status;
  _int_status &= ~BMI2_ACC_DRDY_INT_MASK;
  return ret;
}

float BMI270Class::accelerationSampleRate() {
  struct bmi2_sens_config sens_cfg;
  sens_cfg.type = BMI2_ACCEL;
  bmi2_get_sensor_config(&sens_cfg, 1, &bmi2);
  return (1 << sens_cfg.cfg.acc.odr) * 0.39;
}

// default range is +-2000dps, so conversion factor is (((1 << 15)/4.0f))
#define INT16_to_DPS   (16.384f)

// Gyroscope
int BMI270Class::readGyroscope(float& x, float& y, float& z) {
  struct bmi2_sens_data sensor_data;
  auto ret = bmi2_get_sensor_data(&sensor_data, &bmi2);
  x = sensor_data.gyr.x / INT16_to_DPS;
  y = sensor_data.gyr.y / INT16_to_DPS;
  z = sensor_data.gyr.z / INT16_to_DPS;
  return (ret == 0);
}

int BMI270Class::gyroscopeAvailable() {
  uint16_t status;
  bmi2_get_int_status(&status, &bmi2);
  int ret = ((status | _int_status) & BMI2_GYR_DRDY_INT_MASK);
  _int_status = status;
  _int_status &= ~BMI2_GYR_DRDY_INT_MASK;
  return ret;
}

float BMI270Class::gyroscopeSampleRate() {
  struct bmi2_sens_config sens_cfg;
  sens_cfg.type = BMI2_GYRO;
  bmi2_get_sensor_config(&sens_cfg, 1, &bmi2);
  return (1 << sens_cfg.cfg.gyr.odr) * 0.39;
}

int8_t BMI270Class::configure_sensor(struct bmi2_dev *dev)
{
  int8_t rslt;
  uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

  struct bmi2_int_pin_config int_pin_cfg;
  int_pin_cfg.pin_type = BMI2_INT1;
  int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
  int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
  int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
  int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
  int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

  struct bmi2_sens_config sens_cfg[2];
  sens_cfg[0].type = BMI2_ACCEL;
  sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
  sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
  sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
  sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
  sens_cfg[1].type = BMI2_GYRO;
  sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
  sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
  sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
  sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
  sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

  rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_sensor_enable(sens_list, 2, dev);
  if (rslt != BMI2_OK)
    return rslt;

  return rslt;
}

int8_t BMI270Class::bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  if ((reg_data == NULL) || (len == 0) || (len > 32)) {
    return -1;
  }
  uint8_t bytes_received;

  struct dev_info* dev_info = (struct dev_info*)intf_ptr;
  uint8_t dev_id = dev_info->dev_addr;

  dev_info->_wire->beginTransmission(dev_id);
  dev_info->_wire->write(reg_addr);
  if (dev_info->_wire->endTransmission() == 0) {
    bytes_received = dev_info->_wire->requestFrom(dev_id, len);
    // Optionally, throw an error if bytes_received != len
    for (uint16_t i = 0; i < bytes_received; i++)
    {
      reg_data[i] = dev_info->_wire->read();
    }
  } else {
    return -1;
  }

  return 0;
}

int8_t BMI270Class::bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  if ((reg_data == NULL) || (len == 0) || (len > 32)) {
    return -1;
  }

  struct dev_info* dev_info = (struct dev_info*)intf_ptr;
  uint8_t dev_id = dev_info->dev_addr;
  dev_info->_wire->beginTransmission(dev_id);
  dev_info->_wire->write(reg_addr);
  for (uint16_t i = 0; i < len; i++) {
    dev_info->_wire->write(reg_data[i]);
  }
  if (dev_info->_wire->endTransmission() != 0) {
    return -1;
  }

  return 0;
}

void BMI270Class::bmi2_delay_us(uint32_t period, void *intf_ptr) {
  delayMicroseconds(period);
}

static void panic_led_trap(void) {
  pinMode(LED0, OUTPUT);
  while (1) {
    digitalWrite(LED0, LOW);
    delay(100);
    digitalWrite(LED0, HIGH);
    delay(100);
  }
}

void BMI270Class::print_rslt(int8_t rslt) {

  switch (rslt) {
    case BMI2_OK: return; /* Do nothing */ break;
    case BMI2_E_NULL_PTR:
      fprintf(stderr, "Error : Null pointer");
      panic_led_trap();
      break;
    case BMI2_E_COM_FAIL:
      fprintf(stderr, "Error : Communication failure");
      panic_led_trap();
      break;
    case BMI2_E_DEV_NOT_FOUND:
      fprintf(stderr, "Error : Device not found");
      panic_led_trap();
      break;
    case BMI2_E_OUT_OF_RANGE:
      fprintf(stderr, "Error : Out of range");
      panic_led_trap();
      break;
    case BMI2_E_ACC_INVALID_CFG:
      fprintf(stderr, "Error : Invalid accel configuration");
      panic_led_trap();
      break;
    case BMI2_E_GYRO_INVALID_CFG:
      fprintf(stderr, "Error : Invalid gyro configuration");
      panic_led_trap();
      break;
    case BMI2_E_ACC_GYR_INVALID_CFG:
      fprintf(stderr, "Error : Invalid accel/gyro configuration");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_SENSOR:
      fprintf(stderr, "Error : Invalid sensor");
      panic_led_trap();
      break;
    case BMI2_E_CONFIG_LOAD:
      fprintf(stderr, "Error : Configuration loading error");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_PAGE:
      fprintf(stderr, "Error : Invalid page ");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_FEAT_BIT:
      fprintf(stderr, "Error : Invalid feature bit");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_INT_PIN:
      fprintf(stderr, "Error : Invalid interrupt pin");
      panic_led_trap();
      break;
    case BMI2_E_SET_APS_FAIL:
      fprintf(stderr, "Error : Setting advanced power mode failed");
      panic_led_trap();
      break;
    case BMI2_E_AUX_INVALID_CFG:
      fprintf(stderr, "Error : Invalid auxiliary configuration");
      panic_led_trap();
      break;
    case BMI2_E_AUX_BUSY:
      fprintf(stderr, "Error : Auxiliary busy");
      panic_led_trap();
      break;
    case BMI2_E_SELF_TEST_FAIL:
      fprintf(stderr, "Error : Self test failed");
      panic_led_trap();
      break;
    case BMI2_E_REMAP_ERROR:
      fprintf(stderr, "Error : Remapping error");
      panic_led_trap();
      break;
    case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
      fprintf(stderr, "Error : Gyro user gain update failed");
      panic_led_trap();
      break;
    case BMI2_E_SELF_TEST_NOT_DONE:
      fprintf(stderr, "Error : Self test not done");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_INPUT:
      fprintf(stderr, "Error : Invalid input");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_STATUS:
      fprintf(stderr, "Error : Invalid status");
      panic_led_trap();
      break;
    case BMI2_E_CRT_ERROR:
      fprintf(stderr, "Error : CRT error");
      panic_led_trap();
      break;
    case BMI2_E_ST_ALREADY_RUNNING:
      fprintf(stderr, "Error : Self test already running");
      panic_led_trap();
      break;
    case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
      fprintf(stderr, "Error : CRT ready for DL fail abort");
      panic_led_trap();
      break;
    case BMI2_E_DL_ERROR:
      fprintf(stderr, "Error : DL error");
      panic_led_trap();
      break;
    case BMI2_E_PRECON_ERROR:
      fprintf(stderr, "Error : PRECON error");
      panic_led_trap();
      break;
    case BMI2_E_ABORT_ERROR:
      fprintf(stderr, "Error : Abort error");
      panic_led_trap();
      break;
    case BMI2_E_GYRO_SELF_TEST_ERROR:
      fprintf(stderr, "Error : Gyro self test error");
      panic_led_trap();
      break;
    case BMI2_E_GYRO_SELF_TEST_TIMEOUT:
      fprintf(stderr, "Error : Gyro self test timeout");
      panic_led_trap();
      break;
    case BMI2_E_WRITE_CYCLE_ONGOING:
      fprintf(stderr, "Error : Write cycle ongoing");
      panic_led_trap();
      break;
    case BMI2_E_WRITE_CYCLE_TIMEOUT:
      fprintf(stderr, "Error : Write cycle timeout");
      panic_led_trap();
      break;
    case BMI2_E_ST_NOT_RUNING:
      fprintf(stderr, "Error : Self test not running");
      panic_led_trap();
      break;
    case BMI2_E_DATA_RDY_INT_FAILED:
      fprintf(stderr, "Error : Data ready interrupt failed");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_FOC_POSITION:
      fprintf(stderr, "Error : Invalid FOC position");
      panic_led_trap();
      break;
    default:
      fprintf(stderr, "Error : Unknown error code");
      panic_led_trap();
      break;
  }
}

