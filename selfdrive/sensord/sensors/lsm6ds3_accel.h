#pragma once

#include "selfdrive/sensord/sensors/i2c_sensor.h"

// Address of the chip on the bus
#define LSM6DS3_ACCEL_I2C_ADDR       0x6A

// Registers of the chip
#define LSM6DS3_ACCEL_I2C_REG_DRDY_CFG  0x0B
#define LSM6DS3_ACCEL_I2C_REG_ID        0x0F
#define LSM6DS3_ACCEL_I2C_REG_INT1_CTRL 0x0D
#define LSM6DS3_ACCEL_I2C_REG_CTRL1_XL  0x10
#define LSM6DS3_ACCEL_I2C_REG_STAT_REG  0x1E
#define LSM6DS3_ACCEL_I2C_REG_OUTX_L_XL 0x28

// Constants
#define LSM6DS3_ACCEL_CHIP_ID         0x69
#define LSM6DS3TRC_ACCEL_CHIP_ID      0x6A
#define LSM6DS3_ACCEL_ODR_104HZ       (0b0100 << 4)
#define LSM6DS3_ACCEL_INT1_DRDY_XL    0b1
#define LSM6DS3_ACCEL_DRDY_XLDA       0b1
#define LSM6DS3_ACCEL_DRDY_PULSE_MODE (1 << 7)


class LSM6DS3_Accel : public I2CSensor {
  uint8_t get_device_address() {return LSM6DS3_ACCEL_I2C_ADDR;}
  cereal::SensorEventData::SensorSource source = cereal::SensorEventData::SensorSource::LSM6DS3;
public:
  LSM6DS3_Accel(I2CBus *bus, int gpio_nr = 0, bool shared_gpio = false);
  int init();
  bool get_event(MessageBuilder &msg, uint64_t ts = 0);
  int shutdown();
};
