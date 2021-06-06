#include <Arduino.h>
/*
// Pines direccion puente h
#define MOT_DER_DIR 22 // 1 CC // 0 CW (INV)
#define MOT_DER_PWM 4
#define MOT_IZQ_DIR 28 // 1 CC //0 CW (INV)
#define MOT_IZQ_PWM 5

// Sensores
//volatile uint16_t sensores[5];
void readSensor();

void setup()
{
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(MOT_DER_PWM, OUTPUT);
  pinMode(MOT_DER_DIR, OUTPUT);
  pinMode(MOT_IZQ_DIR, OUTPUT);
  pinMode(MOT_IZQ_PWM, OUTPUT);
}

void loop()
{
  // put your main code here, to run repeatedly:

  readSensor();

  digitalWrite(MOT_DER_DIR, HIGH); // Run CC
  digitalWrite(MOT_IZQ_DIR, LOW);
  
  analogWrite(MOT_IZQ_PWM, 0);
  for (int i = 0; i < 255; i++)
  {
    analogWrite(MOT_DER_PWM, i);
    delay(7);
  }
  delay(1000);

  digitalWrite(MOT_DER_DIR, LOW); // Run CW
  digitalWrite(MOT_IZQ_DIR, HIGH);

  analogWrite(MOT_DER_PWM, 0);
  for (int i = 0; i < 255; i++)
  {
    analogWrite(MOT_IZQ_PWM, i);
    delay(7);
  }
  delay(1000);
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

*/

/*
Coding ini dibuat untuk tutorial Youtube: 
Dibuat oleh : Yonatan K. A. Sarumaha untuk TechnoHance
Tanggal     : 5/12/2020
Follow IG TechnoHance     : @technohance
Subscribe YT TechnoHance  : @technohance
*/

//inisialisasi pin yang digunakan oleh Arduino Nano
#define RPWM 9  // Hubungkan pin D9 dengan pin RPWM BTS7960
#define LPWM 10 // Hubungkan pin D10 dengan pin LPWM BTS7960
#define PWM 11  // Hubungkan pin D11 dengan pin R_EN and L_EN BTS7960. Harus menggunakan pin PWM dari MicroController

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

//Inisialisasi program. Dijalankan hanya 1 kali
void setup()
{
  Serial.begin(115200); //Buka serial monitor untuk melihat output ini. (CTRL+SHIFT+M)
  Serial.println("Start");
  pinMode(RPWM, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
}

//Main program
void loop()
{
  Serial.print((int)analogRead(A0)); // IZ
  Serial.print(" --- ");
  Serial.println((int)analogRead(A1)); // DER
  motor_CW();                          //Muter CW 1 detik
  delay(1000);
  Serial.print((int)analogRead(A0)); // IZ
  Serial.print(" --- ");
  Serial.println((int)analogRead(A1)); // DER
  motor_stop();                        //STOP 1 detik
  delay(1000);
  Serial.print((int)analogRead(A0)); // IZ
  Serial.print(" --- ");
  Serial.println((int)analogRead(A1)); // DER
  motor_CCW();                         //Muter CCW 1 detik
  delay(1000);
  Serial.print((int)analogRead(A0)); // IZ
  Serial.print(" --- ");
  Serial.println((int)analogRead(A1)); // DER
  motor_stop();                        //STOP 1 detik
  delay(1000);

  //Loop terus
}