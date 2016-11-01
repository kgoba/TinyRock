#pragma once
#include <stdint.h>

//template<uint8_t sa1 = 0>
class LIS3MDL {
public:
  bool begin(uint8_t SA1pin = 0);

  void configure1(bool tempSensorOn);
  void configure3(uint8_t opMode);

  bool readMagX(int16_t &value);
  bool readMagY(int16_t &value);
  bool readMagZ(int16_t &value);
  bool readMag(int16_t &xvalue, int16_t &yvalue, int16_t &zvalue);

private:
  bool readByte(uint8_t addr, uint8_t *value);
  bool writeByte(uint8_t addr, uint8_t value);
  bool readWord(uint8_t addr, uint16_t *value);

  uint8_t   slaveAddress;  // 00111x0, where x = SA1 pin state
};
