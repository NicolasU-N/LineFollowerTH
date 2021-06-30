#include <avr/io.h>

#define TRIG1BACKIZQ PORTC7
#define ECHO1BACKIZQ 31 // PC6 Ultrasonic sensor 1

#define TRIG2LATIZQ PORTC5
#define ECHO2LATIZQECHO 33 //PC4 Ultrasonic sensor 1

#define TRIG3FRONTLATIZQ PORTC3
#define ECHO3FRONTLATIZQ 35 //PC2 Ultrasonic sensor 1

#define TRIG4FRONTLATDER PORTC1
#define ECHO4FRONTLATDER 37 //PC0 Ultrasonic sensor 1

#define TRIG5LATDER PORTD7
#define ECHO5LATDER 39 //PG2 Ultrasonic sensor 1

#define TRIG6BACKDER PORTG1
#define ECHO6BACKDER 41 //PG0 Ultrasonic sensor 1

float  distUltra[6];

void setup() {
  Serial.begin(115200);

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



}

void readUltrasonic() {
  static unsigned long previousMillis1 = 0;
  if ((millis() - previousMillis1) > 1000)
  {
    //-------------------------------------------------------------------------------
    PORTC &= ~(1 << TRIG1BACKIZQ);
    //digitalWrite(TRIG, LOW); // Set the trigger pin to low for 2uS
    _delay_us(2);

    //digitalWrite(TRIG, HIGH); // Send a 10uS high to trigger ranging
    PORTC |= (1 << TRIG1BACKIZQ);
    _delay_us(20);

    //digitalWrite(TRIG, LOW);                     // Send pin low again
    PORTC &= ~(1 << TRIG1BACKIZQ);
    distUltra[0] = (float)pulseIn(ECHO1BACKIZQ, HIGH, 26000); // Read in times pulse

    distUltra[0] = distUltra[0] / 58; //13.3511 instead of 58 because the speed of a soundwave in water is far bigger than in air

    //-------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------
    PORTC &= ~(1 << TRIG2LATIZQ);
    //digitalWrite(TRIG, LOW); // Set the trigger pin to low for 2uS
    _delay_us(2);

    //digitalWrite(TRIG, HIGH); // Send a 10uS high to trigger ranging
    PORTC |= (1 << TRIG2LATIZQ);
    _delay_us(20);

    //digitalWrite(TRIG, LOW);                     // Send pin low again
    PORTC &= ~(1 << TRIG2LATIZQ);
    distUltra[1] = (float)pulseIn(ECHO2LATIZQECHO, HIGH, 26000); // Read in times pulse

    distUltra[1] = distUltra[1] / 58; //13.3511 instead of 58 because the speed of a soundwave in water is far bigger than in air
    //-------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------
    PORTC &= ~(1 << TRIG3FRONTLATIZQ);
    //digitalWrite(TRIG, LOW); // Set the trigger pin to low for 2uS
    _delay_us(2);

    //digitalWrite(TRIG, HIGH); // Send a 10uS high to trigger ranging
    PORTC |= (1 << TRIG3FRONTLATIZQ);
    _delay_us(20);

    //digitalWrite(TRIG, LOW);                     // Send pin low again
    PORTC &= ~(1 << TRIG3FRONTLATIZQ);
    distUltra[2] = (float)pulseIn(ECHO3FRONTLATIZQ, HIGH, 26000); // Read in times pulse

    distUltra[2] = distUltra[2] / 58; //13.3511 instead of 58 because the speed of a soundwave in water is far bigger than in air
    //-------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------
    PORTC &= ~(1 << TRIG4FRONTLATDER);
    //digitalWrite(TRIG, LOW); // Set the trigger pin to low for 2uS
    _delay_us(2);

    //digitalWrite(TRIG, HIGH); // Send a 10uS high to trigger ranging
    PORTC |= (1 << TRIG4FRONTLATDER);
    _delay_us(20);

    //digitalWrite(TRIG, LOW);                     // Send pin low again
    PORTC &= ~(1 << TRIG4FRONTLATDER);
    distUltra[3] = (float)pulseIn(ECHO4FRONTLATDER, HIGH, 26300); // Read in times pulse

    distUltra[3] = distUltra[3] / 58; //13.3511 instead of 58 because the speed of a soundwave in water is far bigger than in air
    //-------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------
    PORTD &= ~(1 << TRIG5LATDER);
    //digitalWrite(TRIG, LOW); // Set the trigger pin to low for 2uS
    _delay_us(2);

    //digitalWrite(TRIG, HIGH); // Send a 10uS high to trigger ranging
    PORTD |= (1 << TRIG5LATDER);
    _delay_us(20);

    //digitalWrite(TRIG, LOW);                     // Send pin low again
    PORTD &= ~(1 << TRIG5LATDER);
    distUltra[4] = (float)pulseIn(ECHO5LATDER, HIGH, 26000); // Read in times pulse

    distUltra[4] = distUltra[4] / 58; //13.3511 instead of 58 because the speed of a soundwave in water is far bigger than in air

    //-------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------
    PORTG &= ~(1 << TRIG6BACKDER);
    //digitalWrite(TRIG, LOW); // Set the trigger pin to low for 2uS
    _delay_us(2);

    //digitalWrite(TRIG, HIGH); // Send a 10uS high to trigger ranging
    PORTG |= (1 << TRIG6BACKDER);
    _delay_us(20);

    //digitalWrite(TRIG, LOW);                     // Send pin low again
    PORTG &= ~(1 << TRIG6BACKDER);
    distUltra[5] = (float)pulseIn(ECHO6BACKDER, HIGH, 26000); // Read in times pulse

    distUltra[5] = distUltra[5] / 58; //13.3511 instead of 58 because the speed of a soundwave in water is far bigger than in air

    //-------------------------------------------------------------------------------

    // PRINT ARRAY
    Serial.print("-- DISTANCE ULTRASONIC SENSOR: ");
    for (size_t i = 0; i < 6; i++)
    {
      Serial.print(distUltra[i]);
      Serial.print("---");
    }
    Serial.println("");
    previousMillis1 += 1000;
  }
}
