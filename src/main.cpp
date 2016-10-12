//#include <inttypes.h>
//#include <avr/io.h>
//#include <avr/eeprom.h>
//#include <avr/interrupt.h>
//#include <util/delay.h>

/*************************************************************************
 Hardware description

 ATTiny85, magnetometer (LIS3MDL), eject output pin, buzzer, LED.
 The buzzer output pin also senses battery voltage via a resistor divider.
 The eject output serves also as a safe/flight pin.
 ATTiny85 is connected via USI bus to an I2C slave device (LIS3MDL).
*************************************************************************/

#include <EEPROM.h>
#include <TinyWireM.h>

#include <avr/sleep.h>

#include "I2C.h"
#include "LIS3MDL.h"
#include "MS5607.h"
#include "tiny.hh"

// 0 for normal operation, 1 for debug (serial output)
#define DEBUG   1

const uint8_t kMAG_ADDR = 0b0011110;

const int kLOG_SIZE = 160;      // 3 bytes per entry

const int kSERVO_PERIOD_MS  = 20;
const int kSERVO_ON_US      = 1250;
const int kSERVO_OFF_US     = 550;

const uint8_t kTIMER1_DIV512  = 0b1010;
const uint8_t kTIMER1_DIV1024 = 0b1011;
const uint8_t kTIMER1_PERIOD = F_CPU / 1024 * kSERVO_PERIOD_MS / 1000;

DigitalOut<PortB::pin4> led;
DigitalOut<PortB::pin3> pyro;
DigitalOut<PortB::pin1> buzzer;

#if DEBUG
  TextOutStream<SoftwareSerialOut<PortB::pin4, 9600> > dbg;
#else
  TextOutStream<NullOutput> dbg;
#endif

const char *endl = "\r\n";

I2C bus;
BarometerSimple baroSensor(bus);
LIS3MDL magSensor;

void testI2C();
void testLED();
void flashID(uint8_t id);

void initPyro();
void initSensors();
void resetCalibration();

void calibrate(int16_t mx, int16_t my, int16_t mz);
void calibrateBaro(uint16_t p);

bool checkFlightPin();
void errorHalt();
void clearLog();
void reportLog(uint16_t index);
void updateLog(uint16_t index);

uint8_t lookupMorse(char c);

void restoreState();
void saveState();

struct LogEntry {
  uint8_t   mag;
  uint16_t  altitude;
};


/*
   Magnetic field zero values: 3378 / -4132 / 4365
 */

enum {
  kSTATE_RESET,
  kSTATE_SAFE,
  kSTATE_FLIGHT
};

enum {
  kBUZZER_TEST,
  kBUZZER_SILENT,
  kBUZZER_DESCENT,
  kBUZZER_ASCENT
};

enum {
  kFLAG_MEASURE,
  kFLAG_CAL,
  kFLAG_SERVO
};

int16_t gMinY, gMaxY, gZeroY;
uint16_t gZeroPressure;

uint8_t gState;
uint8_t gIsDescent;
bool    gPyroOn;
uint8_t gBuzzerMode;
uint16_t gAltitude;
uint16_t gMaxAltitude;
uint8_t  gLogIndex;

uint8_t  gMessage[5];

volatile uint8_t gFlags;

void setup()
{
  TinyWireM.begin();
  dbg.begin();

  dbg << "Reset" << endl;

  //TCCR1 = _BV(CTC1) | kTIMER1_DIV512;
  TCCR1 = kTIMER1_DIV512;
  OCR1C = 124;
  OCR1A = 124;
  TIMSK |= _BV(OCIE1A);

  sei();

  gBuzzerMode = kBUZZER_TEST;
  _delay_ms(100);
  gBuzzerMode = kBUZZER_SILENT;

  initPyro();
  initSensors();

  resetCalibration();
  gIsDescent = false;

  restoreState();

  if (checkFlightPin()) gState = kSTATE_FLIGHT;
  else gState = kSTATE_SAFE;

  _delay_ms(100);
  gBuzzerMode = kBUZZER_TEST;
  _delay_ms(100);
  gBuzzerMode = kBUZZER_SILENT;
  _delay_ms(500);

  gMessage[0] = '1';
  gMessage[1] = '5';
  gMessage[2] = '0';
  gMessage[3] = '7';
  gMessage[4] = 0;
}

void loop()
{
  static uint8_t  pyroTimer;
  static bool     pyroPhase;

  if (bit_check(gFlags, kFLAG_MEASURE)) {
    bit_clear(gFlags, kFLAG_MEASURE);

    int16_t   mx, my, mz;
    uint16_t  pressure, pressureComp;
    bool magOK = magSensor.readMag(mx, my, mz);
    bool baroOK = baroSensor.update();

    if (baroOK) {
      pressure = baroSensor.getPressure();
      if (gZeroPressure != 0) {
        pressureComp = (uint32_t)pressure * 25325 / gZeroPressure;
      }
      else {
        pressureComp = pressure;
      }
      gAltitude = baroSensor.getAltitude(pressureComp);
    }

    dbg << pressureComp << " / " << gAltitude << " * " << mx << " / " << my << " / " << mz << " | " << gZeroY << endl;

    bool doEject = magOK && (my > gZeroY);

    if (gState == kSTATE_SAFE) {
      // SAFE state
      // * indicate eject event on the buzzer
      // * calibrate magnetic sensor range
      // * calibrate reference (ground) pressure value
      // * dump log data on serial port (LED)

      gBuzzerMode = doEject ? kBUZZER_TEST : kBUZZER_SILENT;

      if (magOK) calibrate(mx, my, mz);
      if (baroOK) calibrateBaro(pressure);

      if (gLogIndex >= kLOG_SIZE) {
        gLogIndex = 0;  // Start all over again
      }
      reportLog(gLogIndex++);

      // Check if state has changed
      if (checkFlightPin()) {
        gState = kSTATE_FLIGHT;
        gMaxAltitude = 0;
        //clearLog();
      }
    }
    else if (gState == kSTATE_FLIGHT) {
      // FLIGHT state
      // * manage eject (repeat 3 times)
      // * compute altitude
      // * update and store log data

      if (gAltitude > gMaxAltitude) gMaxAltitude = gAltitude;

      if (doEject) {
        //if (!gPyroOn)
        {
          gIsDescent = true;

          pyroPhase = !pyroPhase;   // alternate between on/off states
          gPyroOn = pyroPhase;
          pyroTimer = 25;
        }
      }
      else {
        pyroPhase = false;
        if (gPyroOn) {
          gPyroOn = false;
          pyroTimer = 25;
        }
      }

      gBuzzerMode = gIsDescent ? kBUZZER_DESCENT : kBUZZER_ASCENT;

      if (gLogIndex < kLOG_SIZE) {
        updateLog(gLogIndex++);
      }

      // Check if state has changed
      if (!checkFlightPin()) {
        gState = kSTATE_SAFE;
        resetCalibration();
        gIsDescent = false;
        gPyroOn = false;
        pyroTimer = 25;
      }
    }
  }

  if (bit_check(gFlags, kFLAG_SERVO)) {
    bit_clear(gFlags, kFLAG_SERVO);

    if (pyroTimer) {
      pyroTimer--;

      pyro.high();
      if (gPyroOn) _delay_us(kSERVO_ON_US);
      else _delay_us(kSERVO_OFF_US);
      pyro.low();
    }
    else {
      pyro.low();
    }
  }

  //delay(1000);
  sleep_mode();
}

ISR(TIMER1_COMPA_vect) {
  static uint8_t cnt1;
  static uint8_t cnt2;
  static uint8_t cnt3;

  static uint8_t msgIdx;
  static uint8_t ditIdx;
  static uint8_t ditTimer;
  static uint8_t morseCode;
  static uint8_t ditCount;
  static uint8_t ditState;

  if (++cnt1 >= 3) {
    cnt1 = 0;
    bit_set(gFlags, kFLAG_SERVO);
  }

  uint8_t measurePeriod = (gState == kSTATE_SAFE) ? 25 : 125;
  if (++cnt2 >= measurePeriod) {
    cnt2 = 0;
    bit_set(gFlags, kFLAG_MEASURE);
  }

  switch (gBuzzerMode) {
  case kBUZZER_SILENT:
    buzzer.low();
    break;
  case kBUZZER_TEST:
    buzzer.high();
    break;
  case kBUZZER_ASCENT:
    if (cnt2 < 10 || (cnt2 > 20 && cnt2 < 40)) buzzer.high();
    else buzzer.low();

    break;
  case kBUZZER_DESCENT:
    if (ditTimer) ditTimer--;
    else {
      if (ditState == 1) {
        // Intercharacter silence
        ditState = 0;
        ditTimer = (ditIdx == 0) ? 50 : 6;
        if (ditIdx == 0 && gMessage[msgIdx] == 0) {
          msgIdx = 0;
          ditTimer = 250;
        }
      }
      else {
        if (ditIdx > 0) {
          ditIdx--;
          morseCode >>= 1;
          ditTimer = (morseCode & 1) ? 20 : 6;
          ditState = 1;
        }
        else {
          // look up morse code
          morseCode = lookupMorse(gMessage[msgIdx]);
          ditCount = morseCode >> 5;
          //morseCode = 0b00000010;
          //ditCount = 5;

          if (ditCount > 0) {
            ditIdx = ditCount - 1;
            ditState = 1;
            ditTimer = (morseCode & 1) ? 20 : 6;
          }

          msgIdx++;
        }
      }
    }

    // Dit/deet
    if (ditState == 1) {
      buzzer.high();
    }
    else {
      buzzer.low();
    }
    break;
  }
}

void calibrate(int16_t mx, int16_t my, int16_t mz)
{
  /*
  static int16_t minx = 32000, miny = 32000, minz = 32000;
  static int16_t maxx = -32000, maxy = -32000, maxz = -32000;

  if (mx > maxx) maxx = mx;
  else if (mx < minx) minx = mx;
  if (my > maxy) maxy = my;
  else if (my < miny) miny = my;
  if (mz > maxz) maxz = mz;
  else if (mz < minz) minz = mz;

  int16_t cx = (minx + maxx) / 2;
  int16_t cy = (miny + maxy) / 2;
  int16_t cz = (minz + maxz) / 2;
  dbg << "Cal: [" << cx << "/" << cy << "/" << cz << "]" << endl;
  */
  if (my < gMinY) gMinY = my;
  if (my > gMaxY) gMaxY = my;
  gZeroY = (gMinY + gMaxY) / 2;
}

void calibrateBaro(uint16_t pressure)
{
  static uint32_t  sum;
  static uint16_t  count;

  sum += pressure;
  count++;

  gZeroPressure = sum / count;
}

void resetCalibration()
{
  gMinY = 32000;
  gMaxY = -32000;
  gZeroPressure = 0;
}

bool checkFlightPin() {
  bool result;
  ADCSRA = _BV(ADPS2) | _BV(ADEN);

  pyro.low();
  pyro.modeInput();
  _delay_ms(1);

  ADMUX = 3;
  bit_set(ADCSRA, ADSC);
  while (bit_check(ADCSRA, ADSC)) {}
  uint16_t adcValue = ADC;

  result = (adcValue > 100);

  //pyro.readAnalog();
  pyro.modeOutput();
  ADCSRA = 0;

  return result;
}

void errorHalt() {
  while (true) {
    led.high();
    _delay_ms(100);
    led.low();
    _delay_ms(100);
  }
}

void initSensors() {
  bool magReady = false;
  for (uint8_t nTry = 10; nTry > 0; nTry--) {
    dbg << "Initializing magfield sensor" << endl;
    magReady = magSensor.begin(1);
    if (magReady) {
      dbg << "Enabling continuous conversion mode" << endl;
      magSensor.configure3(0);  // continuous conversion mode
      break;
    }
    delay(500);
  }
  if (!magReady) {
    errorHalt();
  }

  bool baroReady = false;
  for (uint8_t nTry = 10; nTry > 0; nTry--) {
    dbg << "Initializing pressure sensor" << endl;
    baroReady = baroSensor.initialize();
    if (baroReady) {
      dbg << "Barometer ready" << endl;
      break;
    }
    delay(500);
  }
  if (!baroReady) {
    errorHalt();
  }
}

void initPyro() {
  for (uint8_t cnt = 0; cnt < 25; cnt++) {
    pyro.high();
    _delay_us(kSERVO_OFF_US);
    pyro.low();
    _delay_ms(kSERVO_PERIOD_MS);
  }
}


void testLED()
{
  led.high();
  _delay_ms(400);
  pyro.high();
  _delay_ms(100);
  pyro.low();

  led.low();
  _delay_ms(1000);
}

void flashID(uint8_t id)
{
  for (uint8_t mask = 0x80; mask > 0; mask >>= 1) {
    if (id & mask) {
      pyro.high();
    }
    else {
      led.high();
    }
    _delay_ms(400);
    led.low();
    pyro.low();
    _delay_ms(100);
  }
}


void testI2C() {
  //SoftwareI2C<PortB::pin0, PortB::pin2> bus;
  /*
  uint8_t slaveAddress = 0b0011110;
  uint8_t rc;
  bus.beginTransmission(slaveAddress);
  bus.send(0x0F);
  rc = bus.endTransmission();
  if (rc != 0) {
    dbg << "Failed to write to slave" << endl;
  }
  else {
    rc = bus.requestFrom(slaveAddress, 1); // Request 1 byte from slave
    if (rc != 0) {
      dbg << "Failed to read from slave" << endl;
    }
  }
  if (bus.available() > 0) {
    uint8_t id = bus.receive();
    dbg << "ID: " << id << endl;
  }
  */
}

void clearLog()
{
  for (uint16_t idx = 0; idx < 512; idx++) {
    EEPROM.write(idx, 0xFF);
  }
}

void updateLog(uint16_t idx)
{
  LogEntry entry;
  entry.mag = 0;
  entry.altitude = gAltitude;
  EEPROM.put(10 + 3 * idx, entry);
}

void reportLog(uint16_t idx)
{
  LogEntry entry;
  EEPROM.get(10 + 3 * idx, entry);
  dbg << idx << '\t' << entry.mag << '\t' << entry.altitude << endl;
}

void restoreState()
{
  // Find the last entry in EEPROM
  uint16_t idx = kLOG_SIZE;
  while (idx > 0) {
    idx--;

    LogEntry entry;
    EEPROM.get(10 + 3 * idx, entry);
    if (entry.altitude != 0xFFFF) break;
  }
  gLogIndex = idx;

  // restore gMaxAltitude
  EEPROM.get(0, gMaxAltitude);
  if (gMaxAltitude == 0xFFFF) gMaxAltitude = 0;
}

void saveState()
{

}

uint8_t lookupMorse(char c)
{
  if (c < '0' || c > '9') return 0;

  uint8_t idx = 0;
  uint8_t code = 0;
  uint8_t length = 5;
  uint8_t digit = c - '0';
  if (digit <= 5) {
    for (; idx < 5 - digit; idx++) {
      code <<= 1;
      code |= 1;
    }
    for (; idx < 5; idx++) {
      code <<= 1;
      code |= 0;
    }
  }
  else {
    for (; idx < 10 - digit; idx++) {
      code <<= 1;
      code |= 0;
    }
    for (; idx < 5; idx++) {
      code <<= 1;
      code |= 1;
    }
  }
  code |= (length << 5);
  return code;
}
