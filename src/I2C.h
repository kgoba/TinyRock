#pragma once
#include <Arduino.h>

class I2C {
public:
  I2C();

  byte write(byte address, const char *buffer, byte bufSize);
  byte read(byte address, char *buffer, byte bufSize);

private:
  uint16_t rxTimeout;
};
