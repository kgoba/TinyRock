#include "fsk.h"


//#define rttyOff {DDRD|=B00010000; PORTD&=B11100111;}  //ON/OFF Output high, data maybe input
#define rttyLow {DDRD&=B11101111; DDRD|=B00001000; PORTD&=B11101111;} //ON/OFF pin input; data pin output LOW
#define rttyHigh {DDRD&=B11100111; PORTD&=B11100111;}  //Both pins input
//#define rttyHigh DDRD&=B11101111; DDRD|=B00001000;PORTD&=B11101111 //ON/OFF pin input, data pin output high



FSKTransmitter::FSKTransmitter() {
  autoShutdown = false;
  txLength = 0;
  bitIndex = 0;
  active = false;
}


void FSKTransmitter::mark() {
  rttyHigh;
}


void FSKTransmitter::space() {
  rttyLow;
}


void FSKTransmitter::enable() {
  //
}


void FSKTransmitter::disable() {
  //
}


void FSKTransmitter::transmit(const uint8_t *buffer, uint16_t length) {
  if (active) return;
  txBuffer = buffer;
  txLength = length;
  if (txLength > 0) {
    active = true;
    enable();
    shiftNew();
  }
}


bool FSKTransmitter::isBusy() {
  return active;
}


void FSKTransmitter::tick() {
  if (!active) return;

  if (bitIndex == 10) {
    if (txLength > 0)
      shiftNew();
    else {
      if (autoShutdown) {
        disable();
      }
      else {
        mark();
      }
      active = false;
      return;
    }
  }
  if (bitIndex == 0) {
    space();
  }
  else if (bitIndex >= 8) {
    mark();
  }
  else {
    if (txShift & 1) mark();
    else space();

    txShift >>= 1;
  }
  bitIndex++;
}


void FSKTransmitter::shiftNew() {
  txShift = *txBuffer++;
  txLength--;
  bitIndex = 0;
}
