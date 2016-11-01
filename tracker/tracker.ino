#include "TinyGPS.h"
#include "fsk.h"

FSKTransmitter tx;

/*
 * Configuration
 */

const int kBaudRate = 300;
const int kADCBattery = 0;    // Battery voltage on ADC0
const int pinBeeper = 6;  // beeper

TinyGPS gps;

char line[80];

uint16_t gSeconds;

uint16_t readBattMillivolts() 
{
  // Select channel
  ADMUX = (ADMUX & 0xF8) | (kADCBattery & 0x07);
  
  // Start conversion
  ADCSRA |= _BV(ADEN) | _BV(ADSC);
  
  // Wait for the conversion to finish
  while (ADCSRA & _BV(ADSC)) {}
  
  return ADC * 103ul / 32; // convert result to mV
}

void setup()
{
  // Setup Timer2: fast PWM, prescaler 128, overflow 300 Hz
  TCCR2A = _BV(WGM20) | _BV(WGM21);
  TCCR2B = _BV(WGM22) | _BV(CS22) | _BV(CS20);
  OCR2A = (F_CPU / 128 / kBaudRate) - 1;
  TIMSK2 = _BV(TOIE2);

  // Enable ADC
  ADMUX = _BV(REFS0);  // AVCC (AREF with external cap)
  ADCSRA = _BV(ADEN);
  
  Serial.begin(4800);
}

void loop()
{
  static float flat;
  static float flon;
  static float falt;
  static unsigned long age;
  static unsigned char numSats;

  static uint16_t batVoltageAvg;

  // Read all characters from serial buffer and decode NMEA
  while (Serial.available()) {
    if (gps.encode(Serial.read())) {
      // Update position and altitude
      gps.f_get_position(&flat, &flon, &age);
      falt = gps.f_altitude();
      numSats = gps.satellites();
    }
  }  

  if (!tx.isBusy()) {
    // Create TX packet and transmit
    char buffLatitude[10];
    char buffLongitude[10];
    char buffAltitude[8];

    // Read battery voltage (reference 3.3V)
    uint16_t batVoltage = readBattMillivolts(); // result in mV

    if (batVoltageAvg > 0) {
      batVoltageAvg += batVoltage - (batVoltageAvg + 4) / 8;
      batVoltage = batVoltageAvg / 8;      
    }
    else {
      batVoltageAvg = batVoltage * 8;
    }
    
    // Convert to status 00-99
    uint8_t batStatus = 0;
    //if (batVoltage > 1200) batStatus = (batVoltage - 1200) / 4;
    //if (batStatus > 99) batStatus = 99;
    if (batVoltage > 1000) batStatus = (batVoltage - 1000) / 10;

    snprintf(line, sizeof(line), "%s %s %s %hhu %hhu\n",
      dtostrf(flat, 0, 5, buffLatitude),
      dtostrf(flon, 0, 5, buffLongitude),
      dtostrf(falt, 0, 0, buffAltitude),
      numSats,
      batStatus
    );
     
    tx.transmit(line, strlen(line));
  }
}

ISR(TIMER2_OVF_vect) 
{
  tx.tick();

  static uint16_t cnt;
  
  if (++cnt >= kBaudRate) {
    cnt = 0;
    gSeconds++;
  }
}

/*
// sends entire data packet
int sendDataPacket(char * data)
{
  char c;
 
  c = *data++;
 
  while ( c != '\0')
  {
    sendByte(c);
    c = *data++;
  }
  //rttyTXBit(1);
  //delay(2000);
}

void sendByte(char c)
{
  int i;  
  rttyTXBit(0); // Start bit  
  // Send bits for for char LSB first  
  for (i = 0; i < 7; i++) // Change this here 7 or 8 for ASCII-7 / ASCII-8
  {
    if (c & 1) rttyTXBit(1);    
    else rttyTXBit(0);    
    c = c >> 1;
  }  
  rttyTXBit(1); // Stop bit
  rttyTXBit(1); // Stop bit
}
*/
