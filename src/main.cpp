/*
LineFollowerTH

TODO:
* Agregar control PID
* Definir encoders
* Definir celda de carga
* Terminar lib PWM
* Arraque suave
----------------
Pin Mapping:
Physical Pin      Arduino Pin    Port Pin     Function
========================================================
22                D22            PA0          S0 Mux der
23                D23            PA1          S1 Mux der
24                D24            PA2          S2 Mux der
25                D25            PA3          S3 Mux der
54                A0             PF0          SIG Mux der
26                D26            PA4          S0 Mux izq
25                D27            PA5          S1 Mux izq
26                D28            PA6          S2 Mux izq
16                D29            PA7          S3 Mux izq
13                D10            PF1          SIG Mux izq             OC1B
56                A2             PF2          IRDISF (Frontal)         PCINT18
57                A3             PF3          IRDISB (Back)         PCINT18
30                D30            PC7          TRIG1 BACK IZQ
31                D31            PC6          ECHO1 BACK IZQ
32                D32            PC5          TRIG2 LAT IZQ 
33                D33            PC4          ECHO2 LAT IZQ           OC2BLAT DER             OC2A
34                D34            PC3          TRIG3 FRONT LAT IZQ
35                D35            PC2          ECHO3 FRONT LAT IZQ
36                D36            PC1          TRIG4 FRONT LAT DER
37                D37            PC0          ECHO4 FRONT LAT DER
38                D38            PD7          TRIG5 LAT DER
39                D39            PG2          ECHO5 LAT DER
40                D40            PG1          TRIG6 BACK DER
41                D41            PG0          ECHO6 BACK DER
42                D42            PL7          RPWMIZQ
43                D43            PL6          LPWMIZQ
02                D2             PE4          PWMIZQ0
03                D3             PE5          PWMIZQ1
44                D44            PL5          RPWMDER
45                D45            PL4          LPWMDER
04                D4             PG5          PWMDER0
05                D5             PE3          PWMDER1
46                D46            PL3          BUZZER
47                D47            PL2          BTNBACKF //Adelante
48                D48            PL1          BTNBACKB //Atras
49                D49            PL0          BTNFRONTF //Adelante
50                D50            PB3          BTNFRONTB //Atras
*/

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <ADC.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define COLUMS 20
#define ROWS 4

#define LCD_SPACE_SYMBOL 0x20 //space symbol from the LCD ROM, see p.9 of GDM2004D datasheet

#define S0D PORTA0 //22  // OUTPUT
#define S1D PORTA1 //23  // OUTPUT
#define S3D PORTA2 //24  // OUTPUT
#define S2D PORTA3 //25  // OUTPUT
#define SIGD 0     //A0 // INPUT CANAL 0

#define BATTLEVEL 1 //A1 // INPUT CANAL 1

//Pines Sensor distancia infrarojo
#define IRDISF 2 //Input Analog CANAL 2
#define IRDISB 3 //Input Analog CANAL 3

// Pines direccion puente h
#define RENIZQ 9       // 9 PORTH6
#define LENIZQ 10      // 10 PORTB4
#define PWMIZQ0 PORTH3 // 6 PORTH3
#define PWMIZQ1 PORTH4 // 7 PORTH4

#define RENDER 11      // 11 PORTB5
#define LENDER 12      // 12 PORTB6
#define PWMDER0 PORTG5 // 4 PORTG5
#define PWMDER1 PORTE3 // 5 PORTE3

#define BUZZER PORTL7 //42 Output

#define BTNFRONTF 18 // PORTD3 Input
#define BTNFRONTB 19 // PORTD2 Input
#define BTNLINE 15   // PJ0 Input

#define TRIG1BACKDER PORTC7 //30
#define ECHO1BACKDER 31     //PC6 Ultrasonic sensor 1

#define TRIG2BACKIZQ PORTC5
#define ECHO2BACKIZQ 33 //PC4 Ultrasonic sensor 1

#define TRIG3LATIZQ PORTC3
#define ECHO3LATIZQECHO 35 //PC2 Ultrasonic sensor 1

#define TRIG4FRONTLATIZQ PORTC1
#define ECHO4FRONTLATIZQ 37 //PC0 Ultrasonic sensor 1

#define TRIG5FRONTLATDER PORTD7
#define ECHO5FRONTLATDER 39 //PG2 Ultrasonic sensor 1

#define TRIG6LATDER PORTG1
#define ECHO6LATDER 41 //PG0 Ultrasonic sensor 1

// ------------------- ESTADOS
#define STOP 0
#define BACKF 1
#define BACKB 2
#define FINISH 3
#define CHARGE 4

uint8_t state = 0;

boolean chargeFlag = false;

// ------------------- Arranque y parada
uint16_t fadeValue;
uint8_t velPwm;

// ------------------- Sensores de linea
volatile uint16_t lineSenBack[6];
volatile uint16_t lineSenLatIzq[2];
volatile uint16_t lineSenLatDer[2];
volatile uint16_t lineSenFront[6];

// ------------------- Sensores ultrasonicos
volatile float distUltra[6];

// ------------------- Sensores IR distancia
volatile uint16_t disIrSenValue[2];

// ------------------- LCD INIT
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

// ------------------- INICIALIZAR FUNCIONES
uint8_t funcPwm(uint16_t &t);

void motor_stop();
void motor_CW();
void motor_CCW();
void changeState();
void readUltrasonicSen();
void readDisIrSen();
void readMuxIzq();
void initLCD();

ISR(INT2_vect) //
{
  if (chargeFlag) //Si necesita carga entonces
  {
    state = CHARGE; // Estado es CHARGE
  }
  else
  {
    state = BACKB;
  }
}

ISR(INT3_vect)
{
  if (chargeFlag) //Si necesita carga entonces
  {
    chargeFlag = false; // Estado es STOP restablecemos carga
    state = STOP;
  }
  else
  {
    state = BACKF;
  }
}

void setup()
{
  cli();
  Serial.begin(115200);
  Wire.begin();

  /*
  initLCD();
  */

  Serial.println("Start");

  //CONFIGURAR ADC (SENSORES IR)
  ADCInit();

  //-----------CONFIGURACION PINES DE SALIDA BTNS LEDS  Y MUX SIGNALS
  DDRA |= (1 << DDA0) | (1 << DDA1) | (1 << DDA2) | (1 << DDA3); // OUTPUT MUX SIGNALS

  DDRL |= (1 << DDL6) | (1 << DDL7) | (1 << DDL5) | (1 << DDL3); // LUZ , BUZZER ALARMA, COLOR LED EMERGENCY , COMUN LUZ LED BTNS

  //----------- CONFIGURACION PINES DE SALIDA PUENTE H Y ULTRASONICOS
  DDRC |= (1 << DDC1) | (1 << DDC3) | (1 << DDC5) | (1 << DDC7); // TRIG ULTRASONIC SENSOR
  DDRH |= (1 << DDH4) | (1 << DDH3) | (1 << DDH6);               // PWMIZQ1 PWMIZQ0 RENIZQ
  DDRB |= (1 << DDB5) | (1 << DDB6);                             // RENDER Y LENDER
  DDRG |= (1 << DDG5) | (1 << DDG1);                             // PWMDER0 Y TRIG ULTRASONIC SENSOR
  DDRD |= (1 << DDD7);                                           // TRIG ULTRASONIC SENSOR
  DDRE |= (1 << DDE3);                                           // PWMDER1

  //----------- CONFIGURACION PINES DE ENTRADA BTNS
  //DDRE &= ~((1 << DDE5) | (1 << DDE4)); // PINES BOTONES BACK
  DDRD &= ~((1 << DDD2) | (1 << DDD3)); // PINES BOTONERAS CONECTADAS EN PARALELO
  DDRJ &= ~((1 << DDJ0));               //ENTRADA BTN LINE PCINT9

  //---------------------- CONFIGUACION INTERRUPCIONES -------------------
  EICRA &= ~((1 << ISC20) | (1 << ISC30));
  EICRA |= (1 << ISC21) | (1 << ISC31); // FLANCO DE BAJADA INT2(FRONT ATRAS) E INT3 (FRONT ADELANTE)

  //EICRB &= ~((1 << ISC40) | (1 << ISC50));
  //EICRB |= ((1 << ISC41) | (1 << ISC51)); // FLANCO DE BAJADA INT4(BACK ATRAS) E INT5(BACK ADELANTE)

  EIMSK |= ((1 << INT2) | (1 << INT3)); // ACTIVAR INTERRUPCION | (1 << INT4) | (1 << INT5)

  //----------- PINES PULL UP ULTRASONICOS

  DDRC &= ~((1 << DDC6) | (1 << DDC4) | (1 << DDC2) | (1 << DDC0));
  DDRG &= ~((1 << DDG2) | (1 << DDG0));

  PORTC |= (1 << PORTC6) | (1 << PORTC4) | (1 << PORTC2) | (1 << PORTC0);
  PORTG |= (1 << PORTG2) | (1 << PORTG0);

  sei();
}

void loop()
{
  // put your main code here, to run repeatedly:
  //readLineSensor();
  changeState(); // Change state

  readUltrasonicSen();

  readMuxIzq();

  /*
  switch (state)
  {
  case CW:
    motor_CW();
    break;

  case CCW:
    motor_stop();
    break;

  case STOP:
    motor_CCW();
    break;
  }
  */
}

void initLCD()
{
  while (lcd.begin(COLUMS, ROWS) != 1) //colums - 20, rows - 4
  {
    Serial.println(F("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal."));
    delay(5000);
  }
  lcd.print(F("PCF8574 is OK...")); //(F()) saves string to flash & keeps dynamic memory free
  delay(500);

  lcd.clear();

  /* prints static text */
  lcd.setCursor(0, 1); //set 1-st colum & 2-nd row, 1-st colum & row started at zero
  lcd.print(F("Hello world!"));

  // print dynamic text
  lcd.write(LCD_SPACE_SYMBOL);
}

void motor_CW()
{
  PORTH &= ~(1 << PWMIZQ1); // LOW Y OTRO HIGH ES PARA ATRAS
  PORTH |= (1 << PWMIZQ0);

  PORTE |= (1 << PWMDER1);
  PORTG &= ~(1 << PWMDER0);

  if (fadeValue < 447)
  {
    fadeValue++; // Valor max para 252 pwm
  }

  velPwm = funcPwm(fadeValue);
  Serial.println(velPwm);

  analogWrite(RENIZQ, velPwm); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(LENIZQ, velPwm); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(LENDER, velPwm); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(RENDER, velPwm); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain

  delay(10);
}

void motor_CCW()
{
  PORTH &= ~(1 << PWMIZQ0); // LOW Y OTRO HIGH ES PARA ATRAS
  PORTH |= (1 << PWMIZQ1);

  PORTE &= ~(1 << PWMDER1);
  PORTG |= (1 << PWMDER0);

  if (fadeValue < 447)
  {
    fadeValue++; // Valor max para 252 pwm
  }

  velPwm = funcPwm(fadeValue);
  Serial.println(velPwm);

  analogWrite(RENIZQ, velPwm); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(LENIZQ, velPwm); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(LENDER, velPwm); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(RENDER, velPwm); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain

  delay(10);
}

//Function buat motor STOP
void motor_stop()
{
  //---------------------- FRENO SUAVE
  if (velPwm > 0)
  {
    if (velPwm <= 2)
    {
      velPwm = 0;
    }
    else
    {
      velPwm--;
    }
    //Escribir velocidad decreciente
    Serial.print("velPwm:  ");
    Serial.println(velPwm);
    analogWrite(RENDER, velPwm);
    analogWrite(LENDER, velPwm);
    analogWrite(RENIZQ, velPwm);
    analogWrite(LENIZQ, velPwm);

    if (velPwm > 100)
    {
      delay(3);
    }
    else
    {
      delay(100);
    }
  }

  analogWrite(RENDER, 0);
  analogWrite(LENDER, 0);
  analogWrite(RENIZQ, 0);
  analogWrite(LENIZQ, 0);

  Serial.println("STOP");
}

/*
  @brief Leer mux para sensoresde linea
*/
void readMuxIzq()
{
  for (size_t i = 0; i < 16; i++)
  {
    PORTA = (i & 0x01 << S0D) | (i & 0x02 << S1D) | (i & 0x04 << S2D) | (i & 0x08 << S3D);
    /*digitalWrite(S0, i & 0x01);
    digitalWrite(S1, i & 0x02);
    digitalWrite(S2, i & 0x04);
    digitalWrite(S3, i & 0x08);*/
    if (i < 6) // FRONT
    {
      lineSenFront[i] = ADCGetData(SIGD); // leer canal
      Serial.print(lineSenFront[i]);
      Serial.print("---");
    }
    else if (i > 5 && i < 12) // BACK
    {
      lineSenBack[i - 6] = ADCGetData(SIGD); // leer canal
      Serial.print(lineSenBack[i - 5]);
      Serial.print("---");
    }
    else if (i > 11 && i < 14) // LATERALES IZQ
    {
      lineSenLatIzq[i - 12] = ADCGetData(SIGD);
      Serial.print(lineSenLatIzq[i - 10]);
      Serial.print("---");
    }
    else if (i > 13 && i < 16) // LATERALES DER
    {
      lineSenLatDer[i - 14] = ADCGetData(SIGD);
      Serial.print(lineSenLatDer[i - 10]);
      Serial.print("---");
    }
  }
  Serial.println("");
}

/*
  @brief Leer sensores distancia infrarojo
*/
void readDisIrSen()
{
  disIrSenValue[0] = ADCGetData(IRDISF); // leer dis front
  disIrSenValue[1] = ADCGetData(IRDISB); // leer dis back
}

/*
  @brief funcion exponencial para generar arranque suave  
  @param tiempo
*/
uint8_t funcPwm(uint16_t &t)
{
  return (uint8_t)2 * exp(0.0108 * t);
}

void readUltrasonicSen()
{
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