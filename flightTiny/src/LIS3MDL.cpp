#include "LIS3MDL.h"
#include "TinyWireM.h"

#include "tiny.hh"



bool LIS3MDL::begin(uint8_t SA1pin) {
  slaveAddress = (SA1pin) ? 0b0011110 : 0b0011100;

  // Try to read device ID
  uint8_t id;
  if (!readByte(0x0F, &id)) return false;
  return id == 0b00111101;
}

bool LIS3MDL::readByte(uint8_t addr, uint8_t *value) {
  uint8_t rc;
  TinyWireM.beginTransmission(slaveAddress);
  TinyWireM.send(addr);
  rc = TinyWireM.endTransmission();
  if (rc != 0) return false;

  rc = TinyWireM.requestFrom(slaveAddress, 1); // Request 1 byte from slave
  if (rc != 0) return false;

  if (TinyWireM.available() == 0) return false;
  *value = TinyWireM.receive();
  return true;
}

bool LIS3MDL::writeByte(uint8_t addr, uint8_t value) {
  uint8_t rc;
  TinyWireM.beginTransmission(slaveAddress);
  TinyWireM.send(addr);
  TinyWireM.send(value);
  rc = TinyWireM.endTransmission();
  if (rc != 0) return false;
  return true;
}

bool LIS3MDL::readWord(uint8_t addr, uint16_t *value) {
  uint8_t rc;
  TinyWireM.beginTransmission(slaveAddress);
  TinyWireM.send(addr | 0x80);
  rc = TinyWireM.endTransmission();
  if (rc != 0) return false;

  rc = TinyWireM.requestFrom(slaveAddress, 2); // Request 1 byte from slave
  if (rc != 0) return false;

  if (TinyWireM.available() < 2) return false;

  uint8_t lo = TinyWireM.receive();
  uint8_t hi = TinyWireM.receive();
  *value = (hi << 8) | lo;
  return true;
}

void LIS3MDL::configure1(bool tempSensorOn) {
  uint8_t value = 0;
  if (tempSensorOn) value |= 0x80;
  writeByte(0x20, value);
}

void LIS3MDL::configure3(uint8_t opMode) {
  uint8_t value = opMode;
  writeByte(0x22, value);
}

bool LIS3MDL::readMagX(int16_t &value) {
  return readWord(0x28, (uint16_t *)&value);
}

bool LIS3MDL::readMagY(int16_t &value) {
  return readWord(0x2A, (uint16_t *)&value);
}

bool LIS3MDL::readMagZ(int16_t &value) {
  return readWord(0x2C, (uint16_t *)&value);
}

bool LIS3MDL::readMag(int16_t &xvalue, int16_t &yvalue, int16_t &zvalue) {
  if (!readMagX(xvalue)) return false;
  if (!readMagY(yvalue)) return false;
  if (!readMagZ(zvalue)) return false;
  return true;
}
