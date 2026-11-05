#include "mpu6500_driver.h"

const char* mpuErrorName(MpuError e) {
  switch (e) {
    case MpuError::OK: return "OK";
    case MpuError::NOT_FOUND: return "NOT_FOUND";
    case MpuError::BAD_WHO_AM_I: return "BAD_WHO_AM_I";
    case MpuError::I2C_WRITE: return "I2C_WRITE";
    case MpuError::I2C_READ: return "I2C_READ";
    case MpuError::VERIFY_FAILED: return "VERIFY_FAILED";
  }
  return "UNKNOWN";
}

bool Mpu6500::probe(uint8_t addr) {
  wire_->beginTransmission(addr);
  return wire_->endTransmission(true) == 0;
}

MpuError Mpu6500::begin(TwoWire& wire) {
  wire_ = &wire;
  wire_->end();  // harmless if not started; allows begin() to be reused for bus recovery
  if (!wire_->begin(cfg::PIN_SDA, cfg::PIN_SCL, cfg::I2C_FREQ_HZ)) return MpuError::I2C_WRITE;
  wire_->setTimeOut(cfg::I2C_TIMEOUT_MS);

  if (probe(cfg::MPU_ADDR_AD0_LOW)) {
    addr_ = cfg::MPU_ADDR_AD0_LOW;
  } else if (probe(cfg::MPU_ADDR_AD0_HIGH)) {
    addr_ = cfg::MPU_ADDR_AD0_HIGH;
  } else {
    addr_ = 0;
    return MpuError::NOT_FOUND;
  }

  MpuError e = readRegs(mpu::REG_WHO_AM_I, &who_, 1);
  if (e != MpuError::OK) return e;
  // 0x71/0x73 parts (MPU9250/9255) share the MPU6500 accel + FIFO register map and are
  // accepted; the identity is reported so the user knows which die is fitted.
  if (who_ != mpu::WHO_AM_I_MPU6500 && who_ != mpu::WHO_AM_I_MPU9250 &&
      who_ != mpu::WHO_AM_I_MPU9255) {
    return MpuError::BAD_WHO_AM_I;
  }
  return MpuError::OK;
}

MpuError Mpu6500::configure(uint8_t profile, uint8_t range) {
  if (profile >= PROFILE_COUNT) profile = PROFILE_STD_1K_218;
  range &= 0x03;
  const SensorProfile& p = SENSOR_PROFILES[profile];
  MpuError e;

  // PWR_MGMT_1: H_RESET (bit 7) restores all registers to defaults.
  if ((e = writeReg(mpu::REG_PWR_MGMT_1, 0x80)) != MpuError::OK) return e;
  delay(100);  // init/recovery path only — never called from the sampling loop
  // PWR_MGMT_1: SLEEP = 0 (wake), CLKSEL = 1 (PLL if ready, else internal oscillator).
  if ((e = writeVerify(mpu::REG_PWR_MGMT_1, 0x01)) != MpuError::OK) return e;
  delay(10);
  // PWR_MGMT_2: DIS_XG/YG/ZG = 1 (gyro standby), accelerometer axes enabled.
  if ((e = writeVerify(mpu::REG_PWR_MGMT_2, 0x07)) != MpuError::OK) return e;
  // SMPLRT_DIV: sample rate = internal 1 kHz / (1 + div). Applies because
  // FCHOICE_B = 00 and 0 < DLPF_CFG < 7 below.
  if ((e = writeVerify(mpu::REG_SMPLRT_DIV, p.smplrt_div)) != MpuError::OK) return e;
  // CONFIG: FIFO_MODE (bit 6) = 1 -> when the FIFO is full, new samples are dropped
  // instead of overwriting old ones (keeps frame alignment; we detect and reset).
  // DLPF_CFG = 1..6 selects the 1 kHz internal rate that drives the FIFO.
  if ((e = writeVerify(mpu::REG_CONFIG, 0x40 | (p.gyro_dlpf_cfg & 0x07))) != MpuError::OK) return e;
  // GYRO_CONFIG: FCHOICE_B = 00 so DLPF/SMPLRT_DIV are in effect.
  if ((e = writeVerify(mpu::REG_GYRO_CONFIG, 0x00)) != MpuError::OK) return e;
  // ACCEL_CONFIG: ACCEL_FS_SEL[4:3] = 0:+-2g 1:+-4g 2:+-8g 3:+-16g.
  if ((e = writeVerify(mpu::REG_ACCEL_CONFIG, range << 3)) != MpuError::OK) return e;
  // ACCEL_CONFIG2: ACCEL_FCHOICE_B (bit 3) = 0 -> accel DLPF enabled,
  // A_DLPF_CFG[2:0] selects bandwidth (1: 218.1 Hz, 2: 99 Hz, 3: 44.8 Hz, 7: 420 Hz).
  if ((e = writeVerify(mpu::REG_ACCEL_CONFIG2, p.a_dlpf_cfg & 0x07)) != MpuError::OK) return e;
  // INT_PIN_CFG default (INT pin not wired). INT_ENABLE: FIFO_OFLOW_EN so the
  // overflow flag is latched in INT_STATUS (cleared when INT_STATUS is read).
  if ((e = writeReg(mpu::REG_INT_PIN_CFG, 0x00)) != MpuError::OK) return e;
  if ((e = writeVerify(mpu::REG_INT_ENABLE, mpu::INT_STATUS_FIFO_OFLOW)) != MpuError::OK) return e;
  // FIFO_EN: ACCEL (bit 3) -> X, Y, Z accel written to FIFO each sample period.
  if ((e = writeVerify(mpu::REG_FIFO_EN, 0x08)) != MpuError::OK) return e;
  return resetFifo();
}

MpuError Mpu6500::resetFifo() {
  MpuError e;
  // USER_CTRL: FIFO_EN = 0, FIFO_RST (bit 2) = 1 -> clear FIFO; then FIFO_EN (bit 6) = 1.
  if ((e = writeReg(mpu::REG_USER_CTRL, 0x04)) != MpuError::OK) return e;
  delayMicroseconds(100);
  if ((e = writeReg(mpu::REG_USER_CTRL, 0x40)) != MpuError::OK) return e;
  uint8_t st;
  return readIntStatus(st);  // clear any latched overflow flag
}

MpuError Mpu6500::readFifoCount(uint16_t& count) {
  uint8_t b[2];
  MpuError e = readRegs(mpu::REG_FIFO_COUNTH, b, 2);
  if (e != MpuError::OK) return e;
  count = (uint16_t(b[0] & 0x1F) << 8) | b[1];  // FIFO_COUNT is 13 bits
  return MpuError::OK;
}

MpuError Mpu6500::readFifo(uint8_t* dst, size_t len) {
  // Chunks are a multiple of the 6-byte frame and below the 128-byte Wire buffer.
  constexpr size_t CHUNK = 120;
  while (len > 0) {
    size_t n = len > CHUNK ? CHUNK : len;
    MpuError e = readRegs(mpu::REG_FIFO_R_W, dst, n);
    if (e != MpuError::OK) return e;
    dst += n;
    len -= n;
  }
  return MpuError::OK;
}

MpuError Mpu6500::readIntStatus(uint8_t& status) {
  return readRegs(mpu::REG_INT_STATUS, &status, 1);
}

MpuError Mpu6500::readAccelRaw(int16_t xyz[3]) {
  uint8_t b[6];
  MpuError e = readRegs(mpu::REG_ACCEL_XOUT_H, b, 6);
  if (e != MpuError::OK) return e;
  for (int i = 0; i < 3; ++i) xyz[i] = int16_t((uint16_t(b[2 * i]) << 8) | b[2 * i + 1]);
  return MpuError::OK;
}

MpuError Mpu6500::writeReg(uint8_t reg, uint8_t val) {
  wire_->beginTransmission(addr_);
  wire_->write(reg);
  wire_->write(val);
  return wire_->endTransmission(true) == 0 ? MpuError::OK : MpuError::I2C_WRITE;
}

MpuError Mpu6500::writeVerify(uint8_t reg, uint8_t val) {
  MpuError e = writeReg(reg, val);
  if (e != MpuError::OK) return e;
  uint8_t rb = 0;
  if ((e = readRegs(reg, &rb, 1)) != MpuError::OK) return e;
  return rb == val ? MpuError::OK : MpuError::VERIFY_FAILED;
}

MpuError Mpu6500::readRegs(uint8_t reg, uint8_t* dst, size_t len) {
  wire_->beginTransmission(addr_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return MpuError::I2C_WRITE;  // repeated start
  size_t got = wire_->requestFrom(uint16_t(addr_), len, true);
  if (got != len) {
    while (wire_->available()) wire_->read();
    return MpuError::I2C_READ;
  }
  for (size_t i = 0; i < len; ++i) dst[i] = uint8_t(wire_->read());
  return MpuError::OK;
}
