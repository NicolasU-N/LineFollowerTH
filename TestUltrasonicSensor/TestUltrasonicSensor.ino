#include <avr/io.h>
#include "ADC.h"

#define TRIG1BACKDER PORTC7 //30
#define ECHO1BACKDER 31  //PC6 Ultrasonic sensor 1

#define TRIG2BACKIZQ PORTC5
#define ECHO2BACKIZQ 33  //PC4 Ultrasonic sensor 1

#define TRIG3LATIZQ PORTC3
#define ECHO3LATIZQECHO 35 //PC2 Ultrasonic sensor 1

#define TRIG4FRONTLATIZQ PORTC1
#define ECHO4FRONTLATIZQ 37 //PC0 Ultrasonic sensor 1

#define TRIG5FRONTLATDER PORTD7
#define ECHO5FRONTLATDER 39 //PG2 Ultrasonic sensor 1

#define TRIG6LATDER PORTG1
#define ECHO6LATDER 41 //PG0 Ultrasonic sensor 1

//Pines Sensor distancia infrarojo
#define IRDISF 2 //PF2 //Input Analog CANAL 2
#define IRDISB 3 //PF3 //Input Analog CANAL 3

float  distUltra[6];

uint16_t disIrSenValue[2];

void setup() {
  Serial.begin(115200);

  ADCInit();

  DDRD |= (1 << DDD7); // TRIG ULTRASONIC SENSOR
  DDRG |= (1 << DDG1); // TRIG ULTRASONIC SENSOR
  DDRC |= (1 << DDC1) | (1 << DDC3) | (1 << DDC5) | (1 << DDC7); // TRIG ULTRASONIC SENSOR

  //----------- PINES PULL UP ULTRASONICOS
  DDRC &= ~((1 << DDC6) | (1 << DDC4) | (1 << DDC2) | (1 << DDC0));
  DDRG &= ~((1 << DDG2) | (1 << DDG0));
  PORTC |= (1 << PORTC6) | (1 << PORTC4) | (1 << PORTC2) | (1 << PORTC0);
  PORTG |= (1 << PORTG2) | (1 << PORTG0);
}

void loop() {

  readUltrasonic();

}

void readUltrasonic() {
  static unsigned long previousMillis1 = 0;
  if ((millis() - previousMillis1) > 1000)
  {
    //-------------------------------------------------------------------------------
    PORTC &= ~(1 << TRIG1BACKDER);
    //digitalWrite(TRIG, LOW); // Set the trigger pin to low for 2uS
    _delay_us(2);

    //digitalWrite(TRIG, HIGH); // Send a 10uS high to trigger ranging
    PORTC |= (1 << TRIG1BACKDER);
    _delay_us(20);

    //digitalWrite(TRIG, LOW);                     // Send pin low again
    PORTC &= ~(1 << TRIG1BACKDER);
    distUltra[0] = (float)pulseIn(ECHO1BACKDER, HIGH, 26000); // Read in times pulse

    distUltra[0] = distUltra[0] / 58; //13.3511 instead of 58 because the speed of a soundwave in water is far bigger than in air

    //-------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------
    PORTC &= ~(1 << TRIG2BACKIZQ);
    //digitalWrite(TRIG, LOW); // Set the trigger pin to low for 2uS
    _delay_us(2);

    //digitalWrite(TRIG, HIGH); // Send a 10uS high to trigger ranging
    PORTC |= (1 << TRIG2BACKIZQ);
    _delay_us(20);

    //digitalWrite(TRIG, LOW);                     // Send pin low again
    PORTC &= ~(1 << TRIG2BACKIZQ);
    distUltra[1] = (float)pulseIn(ECHO2BACKIZQ, HIGH, 26000); // Read in times pulse

    distUltra[1] = distUltra[1] / 58; //13.3511 instead of 58 because the speed of a soundwave in water is far bigger than in air
    //-------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------
    PORTC &= ~(1 << TRIG3LATIZQ);
    //digitalWrite(TRIG, LOW); // Set the trigger pin to low for 2uS
    _delay_us(2);

    //digitalWrite(TRIG, HIGH); // Send a 10uS high to trigger ranging
    PORTC |= (1 << TRIG3LATIZQ);
    _delay_us(20);

    //digitalWrite(TRIG, LOW);                     // Send pin low again
    PORTC &= ~(1 << TRIG3LATIZQ);
    distUltra[2] = (float)pulseIn(ECHO3LATIZQECHO, HIGH, 26000); // Read in times pulse

    distUltra[2] = distUltra[2] / 58; //13.3511 instead of 58 because the speed of a soundwave in water is far bigger than in air
    //-------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------
    PORTC &= ~(1 << TRIG4FRONTLATIZQ);
    //digitalWrite(TRIG, LOW); // Set the trigger pin to low for 2uS
    _delay_us(2);

    //digitalWrite(TRIG, HIGH); // Send a 10uS high to trigger ranging
    PORTC |= (1 << TRIG4FRONTLATIZQ);
    _delay_us(20);

    //digitalWrite(TRIG, LOW);                     // Send pin low again
    PORTC &= ~(1 << TRIG4FRONTLATIZQ);
    distUltra[3] = (float)pulseIn(ECHO4FRONTLATIZQ, HIGH, 26300); // Read in times pulse

    distUltra[3] = distUltra[3] / 58; //13.3511 instead of 58 because the speed of a soundwave in water is far bigger than in air
    //-------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------
    PORTD &= ~(1 << TRIG5FRONTLATDER);
    //digitalWrite(TRIG, LOW); // Set the trigger pin to low for 2uS
    _delay_us(2);

    //digitalWrite(TRIG, HIGH); // Send a 10uS high to trigger ranging
    PORTD |= (1 << TRIG5FRONTLATDER);
    _delay_us(20);

    //digitalWrite(TRIG, LOW);                     // Send pin low again
    PORTD &= ~(1 << TRIG5FRONTLATDER);
    distUltra[4] = (float)pulseIn(ECHO5FRONTLATDER, HIGH, 26000); // Read in times pulse

    distUltra[4] = distUltra[4] / 58; //13.3511 instead of 58 because the speed of a soundwave in water is far bigger than in air

    //-------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------
    PORTG &= ~(1 << TRIG6LATDER);
    //digitalWrite(TRIG, LOW); // Set the trigger pin to low for 2uS
    _delay_us(2);

    //digitalWrite(TRIG, HIGH); // Send a 10uS high to trigger ranging
    PORTG |= (1 << TRIG6LATDER);
    _delay_us(20);

    //digitalWrite(TRIG, LOW);                     // Send pin low again
    PORTG &= ~(1 << TRIG6LATDER);
    distUltra[5] = (float)pulseIn(ECHO6LATDER, HIGH, 26000); // Read in times pulse

    distUltra[5] = distUltra[5] / 58; //13.3511 instead of 58 because the speed of a soundwave in water is far bigger than in air

    //-------------------------------------------------------------------------------

    // PRINT ARRAY
    Serial.print("-- DIS ULTRASONIC: ");
    for (size_t i = 0; i < 6; i++)
    {
      Serial.print(distUltra[i]);
      Serial.print("---");
    }
    Serial.println("");

    readDisIrSen();
    previousMillis1 += 1000;
  }
}

/*
  @brief Leer sensores distancia infrarojo
  @param canal
*/
void readDisIrSen()
{
  disIrSenValue[0] = ADCGetData(IRDISF); // leer dis front
  disIrSenValue[1] = ADCGetData(IRDISB); // leer dis back
  Serial.print("-> DIS IR1: ");
  Serial.print(disIrSenValue[0]);
  Serial.print("---");
  Serial.print("-> DIS IR2: ");
  Serial.print(disIrSenValue[1]);
  Serial.println("---");

}
