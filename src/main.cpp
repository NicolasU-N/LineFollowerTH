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
#define SIGD 0     //A0 // INPUT

#define S0I PORTA4 //22  // OUTPUT
#define S1I PORTA5 //23  // OUTPUT
#define S3I PORTA6 //24  // OUTPUT
#define S2I PORTA7 //25  // OUTPUT
#define SIGI 1     //A0 // INPUT

//Pines Sensor distancia infrarojo
#define IRDISF 2 //PF2 //Input Analog
#define IRDISB 3 //PF3 //Input Analog

// Pines direccion puente h
#define RPWMIZQ 42     //PORTL7 // // Hubungkan pin D9 dengan pin RPWM BTS7960
#define LPWMIZQ 43     //PORTL6 // Hubungkan pin D10 dengan pin LPWM BTS7960
#define PWMIZQ0 PORTE4 //6  // Hubungkan pin D11 dengan pin R_EN and L_EN BTS7960. Harus menggunakan pin PWM dari MicroController
#define PWMIZQ1 PORTE5 //7  // Hubungkan pin D11 dengan pin R_EN and L_EN BTS7960. Harus menggunakan pin PWM dari MicroController

#define RPWMDER 44  //PL5  // Hubungkan pin D9 dengan pin RPWM BTS7960
#define LPWMDER 45  //PL4 // Hubungkan pin D10 dengan pin LPWM BTS7960
#define PWMDER0 PG5 // 4 // Hubungkan pin D11 dengan pin R_EN and L_EN BTS7960. Harus menggunakan pin PWM dari MicroController
#define PWMDER1 PE3 // 5 // Hubungkan pin D11 dengan pin R_EN and L_EN BTS7960. Harus menggunakan pin PWM dari MicroController

#define BUZZER PL3 // Output

#define BTNBACKF PE5  // Input
#define BTNBACKB PE4  // Input
#define BTNFRONTF PD3 // Input
#define BTNFRONTB PD2 // Input

#define TRIG1BACKIZQ PORTC7
#define ECHO1BACKIZQ 31 // Ultrasonic sensor 1

#define TRIG2LATIZQ PORTC5
#define ECHO2LATIZQECHO 33 // Ultrasonic sensor 1

#define TRIG3FRONTLATIZQ PORTC3
#define ECHO3FRONTLATIZQ 35 // Ultrasonic sensor 1

#define TRIG4FRONTLATDER PORTC1
#define ECHO4FRONTLATDER 37 // Ultrasonic sensor 1

#define TRIG5LATDER PORTD7
#define ECHO5LATDER 39 // Ultrasonic sensor 1

#define TRIG6BACKDER PORTG1
#define ECHO6BACKDER 41 // Ultrasonic sensor 1

#define CW 0
#define CCW 1
#define STOP 2

uint8_t state = 1;
uint8_t fadeValue;
uint8_t fadeValue1;
uint8_t fadeValue2;

// Sensores izquierdos
volatile uint16_t lineSenBackIzq[5];
volatile uint16_t lineSenLatIzq[5];
volatile uint16_t lineSenFrontIzq[5];

volatile uint16_t lineSenBackDer[5];
volatile uint16_t lineSenLatDer[5];
volatile uint16_t linesenFrontDer[5];

volatile float distUltra[6];

volatile uint16_t disIrSenValue[2];

// ------------------- LCD INIT
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

//void readLineSensor();
void motor_stop();
void motor_CW();
void motor_CCW();
void changeState();
void readUltraSen();
void readDisIrSen();
void readMuxIzq();
void readMuxDer();

void setup()
{
  cli();
  Serial.begin(115200);
  Wire.begin();

  /*
  while (lcd.begin(COLUMS, ROWS) != 1) //colums - 20, rows - 4
  {
    Serial.println(F("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal."));
    delay(5000);
  }
  lcd.print(F("PCF8574 is OK...")); //(F()) saves string to flash & keeps dynamic memory free
  delay(2000);

  
  lcd.clear();



*/

  /* prints static text */
  //lcd.setCursor(0, 1); //set 1-st colum & 2-nd row, 1-st colum & row started at zero
  //lcd.print(F("Hello world!"));

  /*
  // print dynamic text
  lcd.setCursor(14, 2);
  lcd.print(random(10, 1000));
  lcd.write(LCD_SPACE_SYMBOL);
  */

  // put your setup code here, to run once:
  Serial.println("Start");

  //CONFIGURAR ADC (SENSORES IR)
  ADCInit();

  //PORTA = 0x00; // OFF ALL

  //CONFIGURACION PINES DE SALIDA
  DDRA = 0xFF; // OUTPUT ALL pines del puerto A
  DDRC |= (1 << DDC7) | (1 << DDC3) | (1 << DDC5) | (1 << DDC1);
  DDRH |= (1 << DDH4) | (1 << DDH3);
  DDRD |= (1 << DDD7);
  DDRL |= (1 << DDL3) | (1 << DDL4) | (1 << DDL5) | (1 << DDL6) | (1 << DDL7); //Pin 3 del puerto D ,//Pin 4 del puerto D, Pin 5 del puerto D, Pin 6 del puerto D
  DDRE |= (1 << DDE3);                                                         //| (1 << DDE4) | (1 << DDE5)
  DDRG |= (1 << DDG5) | (1 << DDG1);

  //CONFIGURACION PINES DE ENTRADA
  DDRL &= ~((1 << DDL1) | (1 << DDL2) | (1 << DDL0)); //
  DDRE &= ~((1 << DDE5) | (1 << DDE4));               // PINES BOTONES BACK

  //PINES PULL UP ULTRASONICOS
  DDRC &= ~((1 << DDC6) | (1 << DDC4) | (1 << DDC2) | (1 << DDC0));
  DDRG &= ~((1 << DDG2) | (1 << DDG0));

  PORTC |= (1 << PORTC6) | (1 << PORTC4) | (1 << PORTC2) | (1 << PORTC0);
  PORTG |= (1 << PORTG2) | (1 << PORTG0);

  /*
  pinMode(RPWMIZQ, OUTPUT);
  pinMode(LPWMIZQ, OUTPUT);
  pinMode(PWMIZQ0, OUTPUT);
  pinMode(PWMIZQ1, OUTPUT);

  pinMode(RPWMDER, OUTPUT);
  pinMode(LPWMDER, OUTPUT);
  pinMode(PWMDER0, OUTPUT);
  pinMode(PWMDER1, OUTPUT);

  pinMode(S0I, OUTPUT);
  pinMode(S1I, OUTPUT);
  pinMode(S2I, OUTPUT);
  pinMode(S3I, OUTPUT);

  pinMode(S0D, OUTPUT);
  pinMode(S1D, OUTPUT);
  pinMode(S2D, OUTPUT);
  pinMode(S3D, OUTPUT);
  */

  // ultrasonic sensor
  //pinMode(TRIG, OUTPUT);
  //pinMode(ECHO, INPUT_PULLUP);

  sei();
}

void loop()
{
  // put your main code here, to run repeatedly:
  //readLineSensor();
  changeState(); // Change state

  readUltraSen();
  readDisIrSen();

  readMuxIzq();
  readMuxDer();

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
}

/*
void readLineSensor()
{
  static unsigned long previousMillis1 = 0;
  if ((millis() - previousMillis1) > 100)
  {

    Serial.print(ADCGetData(0));
    Serial.print(" -- ");
    Serial.print(ADCGetData(1));
    Serial.print(" -- ");
    Serial.print(ADCGetData(2));
    Serial.print(" -- ");
    Serial.print(ADCGetData(3));
    Serial.print(" -- ");
    Serial.println(ADCGetData(5));
    previousMillis1 += 100;
  }
}
*/

void readUltraSen()
{
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
    distUltra[0] = pulseIn(ECHO1BACKIZQ, HIGH, 26000); // Read in times pulse

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
    distUltra[1] = pulseIn(ECHO2LATIZQECHO, HIGH, 26000); // Read in times pulse

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
    distUltra[2] = pulseIn(ECHO3FRONTLATIZQ, HIGH, 26000); // Read in times pulse

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
    distUltra[3] = pulseIn(ECHO4FRONTLATDER, HIGH, 26300); // Read in times pulse

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
    distUltra[4] = pulseIn(ECHO5LATDER, HIGH, 26000); // Read in times pulse

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
    distUltra[5] = pulseIn(ECHO6BACKDER, HIGH, 26000); // Read in times pulse

    distUltra[5] = distUltra[5] / 58; //13.3511 instead of 58 because the speed of a soundwave in water is far bigger than in air

    //-------------------------------------------------------------------------------

    // PRINT ARRAY
    Serial.print("-- DISTANCE ULTRASONIC SENSOR: ");
    for (size_t i = 0; i < 6; i++)
    {
      Serial.println(distUltra[i]);
    }
    previousMillis1 += 1000;
  }
}

void changeState()
{
  static unsigned long previousMillis3 = 0;
  if ((millis() - previousMillis3) > 3000)
  {
    if (state > 2)
    {
      state = 0;
    }
    else
    {
      state = state + 1;
    }

    //fadeValue = 0;
    //fadeValue1 = 0;

    previousMillis3 += 3000;
  }
}

//Function buat motor muter ke CW
void motor_CW()
{

  //Serial.print((int)analogRead(A0)); // IZ
  //Serial.print(" --- ");
  //Serial.println((int)analogRead(A1)); // DER

  //Other code here

  digitalWrite(LPWMIZQ, LOW);
  digitalWrite(RPWMIZQ, HIGH);

  //-----------------------------------------------------------
  if (fadeValue < 255)
  {
    if (fadeValue >= 245)
    {
      fadeValue = 255;
    }
    else
    {
      fadeValue++;
    }
    analogWrite(6, fadeValue); //PWMIZQ0
    analogWrite(7, fadeValue); //PWMIZQ1
    delay(50);
  }

  //-----------------------------------------------------------
  analogWrite(6, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(7, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain

  digitalWrite(LPWMDER, LOW);
  digitalWrite(RPWMDER, HIGH);
  //-----------------------------------------------------------
  if (fadeValue1 < 255)
  {
    if (fadeValue1 >= 245)
    {
      fadeValue1 = 255;
    }
    else
    {
      fadeValue1++;
    }
    analogWrite(4, fadeValue1); //PWMDER0
    analogWrite(5, fadeValue1); //PWMDER1
    delay(50);
  }

  //-----------------------------------------------------------
  analogWrite(4, 500); //PWMDER0    Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(5, 500); // PWMDER1    Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain

  Serial.println("Muter Kanan");
}

//Function buat motor muter ke CCW
void motor_CCW()
{
  //Serial.print((int)analogRead(A0)); // IZ
  //Serial.print(" --- ");
  //Serial.println((int)analogRead(A1)); // DER
  digitalWrite(LPWMIZQ, HIGH);
  digitalWrite(RPWMIZQ, LOW);
  // ------------------------------------------------------
  if (fadeValue < 255)
  {
    if (fadeValue >= 245)
    {
      fadeValue = 255;
    }
    else
    {
      fadeValue++;
    }
    analogWrite(6, fadeValue); //PWMIZQ0
    analogWrite(7, fadeValue); //PWMIZQ1
    delay(50);
  }

  //----------------------------------------------------
  analogWrite(6, 500); //PWMIZQ0 //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(7, 500); //PWMIZQ1//Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain

  digitalWrite(LPWMDER, HIGH);
  digitalWrite(RPWMDER, LOW);

  if (fadeValue1 < 255)
  {
    if (fadeValue1 >= 245)
    {
      fadeValue1 = 255;
    }
    else
    {
      fadeValue1++;
    }
    analogWrite(4, fadeValue1); //PWMDER0
    analogWrite(5, fadeValue1); //PWMDER1
    delay(50);
  }
  analogWrite(4, 500); //PWMDER0 //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(5, 500); //PWMDER1 //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain

  //analogWrite(PWMDER0, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  //analogWrite(PWMDER1, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  Serial.println("Muter Kiri");
}

//Function buat motor STOP
void motor_stop()
{
  digitalWrite(LPWMDER, LOW);
  digitalWrite(RPWMDER, LOW);

  if (fadeValue > 0)
  {
    if (fadeValue <= 15)
    {
      fadeValue = 0;
    }
    else
    {
      fadeValue--;
    }
    analogWrite(PWMDER0, fadeValue);
    analogWrite(PWMDER1, fadeValue);
    delay(50);
  }

  analogWrite(PWMDER0, 0); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(PWMDER1, 0); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  //fadeValue = 0;

  digitalWrite(LPWMIZQ, LOW);
  digitalWrite(RPWMIZQ, LOW);

  if (fadeValue1 > 0)
  {
    if (fadeValue1 <= 15)
    {
      fadeValue1 = 0;
    }
    else
    {
      fadeValue1--;
    }
    analogWrite(PWMIZQ0, fadeValue1);
    analogWrite(PWMIZQ1, fadeValue1);
    delay(50);
  }

  analogWrite(PWMIZQ0, 0); //Value "0" harus tetap 0, karena motornya diperintahkan untuk STOP
  analogWrite(PWMIZQ1, 0); //Value "0" harus tetap 0, karena motornya diperintahkan untuk STOP

  analogWrite(PWMDER0, 0); //Value "0" harus tetap 0, karena motornya diperintahkan untuk STOP
  analogWrite(PWMDER1, 0); //Value "0" harus tetap 0, karena motornya diperintahkan untuk STOP
  Serial.println("STOP");
}

/*
  @brief Leer mux 1  
  @param canal
*/
void readMuxIzq()
{
  for (size_t i = 0; i < 16; i++)
  {
    PORTA = (i & 0x01 << S0I) | (i & 0x02 << S1I) | (i & 0x04 << S2I) | (i & 0x08 << S3I);
    /*digitalWrite(S0, i & 0x01);
    digitalWrite(S1, i & 0x02);
    digitalWrite(S2, i & 0x04);
    digitalWrite(S3, i & 0x08);*/
    if (i < 5)
    {
      lineSenBackIzq[i] = analogRead(SIGI); // leer canal
      Serial.print(lineSenBackIzq[i]);
      Serial.print("---");
    }
    else if (i > 5 && i < 10)
    {
      lineSenLatIzq[i - 5] = analogRead(SIGI); // leer canal
      Serial.print(lineSenLatIzq[i - 5]);
      Serial.print("---");
    }
    else if (i > 10 && i < 15)
    {
      lineSenFrontIzq[i - 10] = analogRead(SIGI);
      Serial.print(lineSenFrontIzq[i - 10]);
      Serial.print("---");
    }
  }
  Serial.println("");
}

/*
  @brief Leer mux 2  
  @param canal
*/
void readMuxDer()
{
  for (size_t i = 0; i < 16; i++)
  {
    PORTA = (i & 0x01 << S0D) | (i & 0x02 << S1D) | (i & 0x04 << S2D) | (i & 0x08 << S3D);
    /*digitalWrite(S0, i & 0x01);
    digitalWrite(S1, i & 0x02);
    digitalWrite(S2, i & 0x04);
    digitalWrite(S3, i & 0x08);*/
    if (i < 5)
    {
      lineSenBackDer[i] = ADCGetData(SIGD); // leer canal
    }
    else if (i > 5 && i < 10)
    {
      lineSenLatDer[i - 5] = ADCGetData(SIGD); // leer canal
    }
    else if (i > 10 && i < 15)
    {
      linesenFrontDer[i - 10] = ADCGetData(SIGD);
    }
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
}
