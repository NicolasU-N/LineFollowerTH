#include <Arduino.h>

// Pines direccion puente h
#define RPWM 9  // Hubungkan pin D9 dengan pin RPWM BTS7960
#define LPWM 10 // Hubungkan pin D10 dengan pin LPWM BTS7960
#define PWM 11  // Hubungkan pin D11 dengan pin R_EN and L_EN BTS7960. Harus menggunakan pin PWM dari MicroController

#define S0 22  // Hubungkan pin D9 dengan pin RPWM BTS7960
#define S1 23  // Hubungkan pin D10 dengan pin LPWM BTS7960
#define S3 24  // Hubungkan pin D11 dengan pin R_EN and L_EN BTS7960. Harus menggunakan pin PWM dari MicroController
#define S2 25  // Hubungkan pin D9 dengan pin RPWM BTS7960
#define SIG A0 // Hubungkan pin D9 dengan pin RPWM BTS7960

#define CW 0
#define CCW 1
#define STOP 2

#define TRIG 30
#define ECHO 31  // Ultrasonic sensor 1

uint8_t state = 2;

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
  Serial.begin(115200);
  // put your setup code here, to run once:
  Serial.println("Start");
  pinMode(RPWM, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(LPWM, OUTPUT);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  pinMode(TRIG, OUTPUT); // ultrasonic sensor 
  pinMode(ECHO, INPUT_PULLUP);
}

void loop()
{
  // put your main code here, to run repeatedly:
  readLineSensor();
  changeState(); // Change state
  readUltrasonicSensor();
  switch (state)
  {
  case CW:
    motor_CW();
    break;

  case CCW:
    motor_CCW();
    break;

  case STOP:
    motor_stop();
    break;
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
  static unsigned long previousMillis2 = 0;
  if ((millis() - previousMillis2) > 1000)
  {
    state += 1;
    previousMillis2 += 1000;
  }
}

//Function buat motor muter ke CW
void motor_CW()
{
  Serial.print((int)analogRead(A0)); // IZ
  Serial.print(" --- ");
  Serial.println((int)analogRead(A1)); // DER
  digitalWrite(LPWM, LOW);
  digitalWrite(RPWM, HIGH);
  analogWrite(PWM, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  Serial.println("Muter Kanan");
}

//Function buat motor muter ke CCW
void motor_CCW()
{
  Serial.print((int)analogRead(A0)); // IZ
  Serial.print(" --- ");
  Serial.println((int)analogRead(A1)); // DER
  digitalWrite(LPWM, HIGH);
  digitalWrite(RPWM, LOW);
  analogWrite(PWM, 500); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  Serial.println("Muter Kiri");
}

//Function buat motor STOP
void motor_stop()
{
  digitalWrite(LPWM, LOW);
  digitalWrite(RPWM, LOW);
  analogWrite(PWM, 0); //Value "0" harus tetap 0, karena motornya diperintahkan untuk STOP
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