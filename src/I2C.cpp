#include "I2C.h"

#include <TinyWireM.h>

I2C::I2C() {
  rxTimeout = 200;
}

byte I2C::write(byte address, const char *buffer, byte bufSize) {
  TinyWireM.beginTransmission(address);
  TinyWireM.write((uint8_t *)buffer, bufSize);
  return TinyWireM.endTransmission();
}

byte I2C::read(byte address, char *buffer, byte bufSize) {
  TinyWireM.requestFrom(address, bufSize);

  long start = millis();
  while (bufSize > 0) {
    while (TinyWireM.available() > 0) {
      *buffer++ = TinyWireM.read();
      bufSize--;
      if (bufSize == 0) return 0;
    }
    uint16_t elapsed = millis() - start;
    if (elapsed > rxTimeout) return 1;
  }
  return 0;
}
