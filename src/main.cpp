/*
LineFollowerTH

TODO:
* Agregar control PID
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

//Pines Mux
#define S0D PA0  //22  // OUTPUT
#define S1D PA1  //23  // OUTPUT
#define S3D PA2  //24  // OUTPUT
#define S2D PA3  //25  // OUTPUT
#define SIGD PF0 //A0 // INPUT

#define S0I PA4  //22  // OUTPUT
#define S1I PA5  //23  // OUTPUT
#define S3I PA6  //24  // OUTPUT
#define S2I PA7  //25  // OUTPUT
#define SIGI PF1 //A0 // INPUT

//Pines Sensor distancia infrarojo
#define IRDISF PF2 //Input Analog
#define IRDISB PF3 //Input Analog

// Pines direccion puente h
#define RPWMIZQ PL7 //42 // Hubungkan pin D9 dengan pin RPWM BTS7960
#define LPWMIZQ PL6 //43 // Hubungkan pin D10 dengan pin LPWM BTS7960
#define PWMIZQ0 PE4 //2  // Hubungkan pin D11 dengan pin R_EN and L_EN BTS7960. Harus menggunakan pin PWM dari MicroController
#define PWMIZQ1 PE5 //3  // Hubungkan pin D11 dengan pin R_EN and L_EN BTS7960. Harus menggunakan pin PWM dari MicroController

#define RPWMDER PL5 // Hubungkan pin D9 dengan pin RPWM BTS7960
#define LPWMDER PL4 // Hubungkan pin D10 dengan pin LPWM BTS7960
#define PWMDER0 PG5 // Hubungkan pin D11 dengan pin R_EN and L_EN BTS7960. Harus menggunakan pin PWM dari MicroController
#define PWMDER1 PE3 // Hubungkan pin D11 dengan pin R_EN and L_EN BTS7960. Harus menggunakan pin PWM dari MicroController

#define BUZZER PL3 // Output

#define BTNBACKF PL2  // Input
#define BTNBACKB PL1  // Input
#define BTNFRONTF PL0 // Input
#define BTNFRONTB PB3 // Input

#define CW 0
#define CCW 1
#define STOP 2

#define TRIG 30
#define ECHO 31 // Ultrasonic sensor 1

uint8_t state = 1;
uint8_t fadeValue;
uint8_t fadeValue1;
uint8_t fadeValue2;

// Sensores
//volatile uint16_t sensores[5];

void readLineSensor();
void motor_stop();
void motor_CW();
void motor_CCW();
void changeState();
void readUltrasonicSensor();

void setup()
{
  cli();
  Serial.begin(115200);
  // put your setup code here, to run once:
  Serial.println("Start");

  //PORTA = 0x00; // OFF ALL

  //CONFIGURACION PINES DE SALIDA
  DDRA = 0xFF;                                                                 // OUTPUT ALL pines del puerto A
  DDRL |= (1 << DDL1) | (1 << DDL2) | (1 << DDL3) | (1 << DDL4) | (1 << DDL5); //Pin 3 del puerto D ,//Pin 4 del puerto D, Pin 5 del puerto D, Pin 6 del puerto D
  DDRE |= (1 << DDE3) | (1 << DDE4) | (1 << DDE5);
  DDRG |= (1 << DDG5);

  //CONFIGURACION PINES DE ENTRADA
  DDRL &= ~((1 << DDL1) | (1 << DDL2) | (1 << DDL0) | (1 << DDL0));

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

  pinMode(TRIG, OUTPUT); // ultrasonic sensor
  pinMode(ECHO, INPUT_PULLUP);
  sei();
}

void loop()
{
  // put your main code here, to run repeatedly:
  //readLineSensor();
  changeState(); // Change state
  //readUltrasonicSensor();
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
  if (state > 2)
  {
    state = 0;
  }
}

void readLineSensor()
{
  static unsigned long previousMillis1 = 0;
  if ((millis() - previousMillis1) > 100)
  {
    Serial.print(analogRead(A0));
    Serial.print(" -- ");
    Serial.print(analogRead(A1));
    Serial.print(" -- ");
    Serial.print(analogRead(A2));
    Serial.print(" -- ");
    Serial.print(analogRead(A3));
    Serial.print(" -- ");
    Serial.println(analogRead(A4));
    previousMillis1 += 100;
  }
}

void readUltrasonicSensor()
{
  static unsigned long previousMillis1 = 0;
  if ((millis() - previousMillis1) > 1000)
  {
    digitalWrite(TRIG, LOW); // Set the trigger pin to low for 2uS
    delayMicroseconds(2);

    digitalWrite(TRIG, HIGH); // Send a 10uS high to trigger ranging
    delayMicroseconds(20);

    digitalWrite(TRIG, LOW);                     // Send pin low again
    float distance = pulseIn(ECHO, HIGH, 26000); // Read in times pulse

    distance = distance / 58; //13.3511 instead of 58 because the speed of a soundwave in water is far bigger than in air
    previousMillis1 += 1000;
  }
}

void changeState()
{
  static unsigned long previousMillis3 = 0;
  if ((millis() - previousMillis3) > 3000)
  {
    state = state + 1;
    //fadeValue = 0;
    //fadeValue1 = 0;

    previousMillis3 += 3000;
  }
}

//Function buat motor muter ke CW
void motor_CW()
{

  Serial.print((int)analogRead(A0)); // IZ
  Serial.print(" --- ");
  Serial.println((int)analogRead(A1)); // DER

  //Other code here

  digitalWrite(LPWMIZQ, LOW);
  digitalWrite(RPWMIZQ, HIGH);
  /*
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
    analogWrite(PWMIZQ0, fadeValue);
    analogWrite(PWMIZQ1, fadeValue);
    delay(50);
  }
*/
  analogWrite(PWMIZQ0, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(PWMIZQ1, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain

  digitalWrite(LPWMDER, LOW);
  digitalWrite(RPWMDER, HIGH);
  /*
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
    analogWrite(PWMDER0, fadeValue1);
    analogWrite(PWMDER1, fadeValue1);
    delay(50);
  }
  */
  analogWrite(PWMDER0, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(PWMDER1, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain

  Serial.println("Muter Kanan");
}

//Function buat motor muter ke CCW
void motor_CCW()
{
  Serial.print((int)analogRead(A0)); // IZ
  Serial.print(" --- ");
  Serial.println((int)analogRead(A1)); // DER
  digitalWrite(LPWMIZQ, HIGH);
  digitalWrite(RPWMIZQ, LOW);
  /*
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
    analogWrite(PWMDER0, fadeValue);
    analogWrite(PWMDER1, fadeValue);
    delay(50);
  }
  */
  //analogWrite(PWMDER0, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  //analogWrite(PWMDER1, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain

  analogWrite(PWMIZQ0, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(PWMIZQ1, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain

  digitalWrite(LPWMDER, HIGH);
  digitalWrite(RPWMDER, LOW);
  /*
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
    analogWrite(PWMDER0, fadeValue1);
    analogWrite(PWMDER1, fadeValue1);
    delay(50);
  }*/
  analogWrite(PWMDER0, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(PWMDER1, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain

  //analogWrite(PWMDER0, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  //analogWrite(PWMDER1, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  Serial.println("Muter Kiri");
}

//Function buat motor STOP
void motor_stop()
{
  digitalWrite(LPWMDER, LOW);
  digitalWrite(RPWMDER, LOW);
  /*
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
  */
  //analogWrite(PWMDER0, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  //analogWrite(PWMDER1, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  //fadeValue = 0;

  digitalWrite(LPWMIZQ, LOW);
  digitalWrite(RPWMIZQ, LOW);
  /*
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
  }*/

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
void readMux()
{
  for (size_t i = 0; i < 16; i++)
  {
    digitalWrite(S0, i & 0x01);
    digitalWrite(S1, i & 0x02);
    digitalWrite(S2, i & 0x04);
    digitalWrite(S3, i & 0x08);
  }
  analogRead(SIG); // leer canal
}