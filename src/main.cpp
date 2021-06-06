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

// Sensores
//volatile uint16_t sensores[5];

void readSensor();

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
}

void loop()
{
  // put your main code here, to run repeatedly:
  readSensor();
  /*
  motor_CW(); //Muter CW 1 detik
  delay(1000);

  motor_stop(); //STOP 1 detik
  delay(1000);

  motor_CCW(); //Muter CCW 1 detik
  delay(1000);

  motor_stop(); //STOP 1 detik
  delay(1000);
  */
}

void readSensor()
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
