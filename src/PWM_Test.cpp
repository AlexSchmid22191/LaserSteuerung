#include <Wire.h>
#include <Arduino.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <Adafruit_MAX31856.h>


void setup()
{
    Serial.begin(9600);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  // Phase and frequency correct pwm mode 

  TCCR1A = 0;
  TCCR1B = 0;

  noInterrupts();
  TCCR1B |= 1 << WGM13;
  // No prescaler
  TCCR1B |= 1 << CS10;
  // Output on Pin 9
  TCCR1A |= 1 << COM1A1;
  // TOP value of the counter
  ICR1 = 10000;
  OCR1A = 2500;

  interrupts();

    Serial.println(TCCR1A, BIN);
        Serial.println(TCCR1B, BIN);
            Serial.println(TCCR1C, BIN);
                Serial.println(ICR1, BIN);
    Serial.println(OCR1A, BIN);



}

void loop()
{
    
}