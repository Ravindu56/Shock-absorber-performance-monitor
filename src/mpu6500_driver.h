// mpu6500_driver.h — direct register access to the MPU6500 over I2C.
//
// Only the accelerometer is used. The gyroscope is put in standby to reduce
// power and noise coupling; the FIFO is configured to store accelerometer
// samples only (6 bytes per frame: XH XL YH YL ZH ZL, big-endian int16).
#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "config.h"

namespace mpu {

// Register map (MPU-6500 Register Map and Descriptions, rev 2.1)
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;
constexpr uint8_t REG_FIFO_EN = 0x23;
constexpr uint8_t REG_INT_PIN_CFG = 0x37;
constexpr uint8_t REG_INT_ENABLE = 0x38;
constexpr uint8_t REG_INT_STATUS = 0x3A;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_USER_CTRL = 0x6A;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;
constexpr uint8_t REG_FIFO_COUNTH = 0x72;
constexpr uint8_t REG_FIFO_R_W = 0x74;
constexpr uint8_t REG_WHO_AM_I = 0x75;

constexpr uint8_t WHO_AM_I_MPU6500 = 0x70;
constexpr uint8_t WHO_AM_I_MPU9250 = 0x71;  // register-compatible accel/FIFO
constexpr uint8_t WHO_AM_I_MPU9255 = 0x73;  // register-compatible accel/FIFO

constexpr uint8_t INT_STATUS_FIFO_OFLOW = 0x10;
constexpr uint16_t FIFO_SIZE_BYTES = 512;
constexpr uint8_t FIFO_FRAME_BYTES = 6;

}  // namespace mpu

enum class MpuError : uint8_t {
  OK = 0,
  NOT_FOUND,       // no ACK at 0x68 or 0x69
  BAD_WHO_AM_I,    // device answered with an unexpected identity
  I2C_WRITE,       // endTransmission() returned non-zero
  I2C_READ,        // requestFrom() returned fewer bytes than requested
  VERIFY_FAILED,   // register read-back did not match what was written
};

const char* mpuErrorName(MpuError e);

class Mpu6500 {
 public:
  // Initialise the I2C bus and locate the sensor. Does not configure it.
  MpuError begin(TwoWire& wire);

  // Reset the device and apply profile/range. Leaves the FIFO enabled and empty.
  MpuError configure(uint8_t profile, uint8_t range);

  MpuError resetFifo();
  MpuError readFifoCount(uint16_t& count);
  // Read `len` bytes from FIFO_R_W (len may exceed the Wire buffer; it is chunked).
  MpuError readFifo(uint8_t* dst, size_t len);
  MpuError readIntStatus(uint8_t& status);
  // Direct register read of the latest accel sample (used only for diagnostics).
  MpuError readAccelRaw(int16_t xyz[3]);

  uint8_t address() const { return addr_; }
  uint8_t whoAmI() const { return who_; }

 private:
  MpuError writeReg(uint8_t reg, uint8_t val);
  MpuError writeVerify(uint8_t reg, uint8_t val);
  MpuError readRegs(uint8_t reg, uint8_t* dst, size_t len);
  bool probe(uint8_t addr);

  TwoWire* wire_ = nullptr;
  uint8_t addr_ = 0;
  uint8_t who_ = 0;
};
