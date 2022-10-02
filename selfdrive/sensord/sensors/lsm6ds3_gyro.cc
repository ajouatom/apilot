#include "lsm6ds3_gyro.h"

#include <cassert>
#include <cmath>

#include "common/swaglog.h"
#include "common/timing.h"

#define DEG2RAD(x) ((x) * M_PI / 180.0)

LSM6DS3_Gyro::LSM6DS3_Gyro(I2CBus *bus, int gpio_nr, bool shared_gpio) :
  I2CSensor(bus, gpio_nr, shared_gpio) {}

int LSM6DS3_Gyro::init() {
  int ret = 0;
  uint8_t buffer[1];
  uint8_t value = 0;

  ret = read_register(LSM6DS3_GYRO_I2C_REG_ID, buffer, 1);
  if(ret < 0) {
    LOGE("Reading chip ID failed: %d", ret);
    goto fail;
  }

  if(buffer[0] != LSM6DS3_GYRO_CHIP_ID && buffer[0] != LSM6DS3TRC_GYRO_CHIP_ID) {
    LOGE("Chip ID wrong. Got: %d, Expected %d", buffer[0], LSM6DS3_GYRO_CHIP_ID);
    ret = -1;
    goto fail;
  }

  if (buffer[0] == LSM6DS3TRC_GYRO_CHIP_ID) {
    source = cereal::SensorEventData::SensorSource::LSM6DS3TRC;
  }

  ret = init_gpio();
  if (ret < 0) {
    goto fail;
  }

  // TODO: set scale. Default is +- 250 deg/s
  ret = set_register(LSM6DS3_GYRO_I2C_REG_CTRL2_G, LSM6DS3_GYRO_ODR_104HZ);
  if (ret < 0) {
    goto fail;
  }

  ret = set_register(LSM6DS3_GYRO_I2C_REG_DRDY_CFG, LSM6DS3_GYRO_DRDY_PULSE_MODE);
  if (ret < 0) {
    goto fail;
  }

  // enable data ready interrupt for gyro on INT1
  // (without resetting existing interrupts)
  ret = read_register(LSM6DS3_GYRO_I2C_REG_INT1_CTRL, &value, 1);
  if (ret < 0) {
    goto fail;
  }

  value |= LSM6DS3_GYRO_INT1_DRDY_G;
  ret = set_register(LSM6DS3_GYRO_I2C_REG_INT1_CTRL, value);

fail:
  return ret;
}

int LSM6DS3_Gyro::shutdown() {
  int ret = 0;

  // disable data ready interrupt for accel on INT1
  uint8_t value = 0;
  ret = read_register(LSM6DS3_GYRO_I2C_REG_INT1_CTRL, &value, 1);
  if (ret < 0) {
    goto fail;
  }

  value &= ~(LSM6DS3_GYRO_INT1_DRDY_G);
  ret = set_register(LSM6DS3_GYRO_I2C_REG_INT1_CTRL, value);
  if (ret < 0) {
    LOGE("Could not disable lsm6ds3 gyroscope interrupt!")
    goto fail;
  }

  // enable power-down mode
  value = 0;
  ret = read_register(LSM6DS3_GYRO_I2C_REG_CTRL2_G, &value, 1);
  if (ret < 0) {
    goto fail;
  }

  value &= 0x0F;
  ret = set_register(LSM6DS3_GYRO_I2C_REG_CTRL2_G, value);
  if (ret < 0) {
    LOGE("Could not power-down lsm6ds3 gyroscope!")
    goto fail;
  }

fail:
  return ret;
}

bool LSM6DS3_Gyro::get_event(MessageBuilder &msg, uint64_t ts) {

  // INT1 shared with accel, check STATUS_REG who triggered
  uint8_t status_reg = 0;
  read_register(LSM6DS3_GYRO_I2C_REG_STAT_REG, &status_reg, sizeof(status_reg));
  if ((status_reg & LSM6DS3_GYRO_DRDY_GDA) == 0) {
    return false;
  }

  uint8_t buffer[6];
  int len = read_register(LSM6DS3_GYRO_I2C_REG_OUTX_L_G, buffer, sizeof(buffer));
  assert(len == sizeof(buffer));

  float scale = 8.75 / 1000.0;
  float x = DEG2RAD(read_16_bit(buffer[0], buffer[1]) * scale);
  float y = DEG2RAD(read_16_bit(buffer[2], buffer[3]) * scale);
  float z = DEG2RAD(read_16_bit(buffer[4], buffer[5]) * scale);

  auto event = msg.initEvent().initGyroscope();
  event.setSource(source);
  event.setVersion(2);
  event.setSensor(SENSOR_GYRO_UNCALIBRATED);
  event.setType(SENSOR_TYPE_GYROSCOPE_UNCALIBRATED);
  event.setTimestamp(ts);

  float xyz[] = {y, -x, z};
  auto svec = event.initGyroUncalibrated();
  svec.setV(xyz);
  svec.setStatus(true);

  return true;
}
