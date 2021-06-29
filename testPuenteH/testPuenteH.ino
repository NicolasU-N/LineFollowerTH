// Pines direccion puente h
#define RENIZQ 9  // PORTH6
#define LENIZQ 10 // PORTB4
#define PWMIZQ0 6 // PORTH3
#define PWMIZQ1 7 // PORTH4

#define RENDER 11 // PB5
#define LENDER 12 // PB6
#define PWMDER0 4 // PG5
#define PWMDER1 5 // PE3

// PINES BOTONES
#define BTNBACKF 3  // Input
#define BTNBACKB 2  // Input
#define BTNFRONTF 18 // Input
#define BTNFRONTB 19 // Input


int fadeValue = 0;
int fadeValue1 = 0;

#define CW 0
#define STOP 1
uint8_t state = 0;

int pwmder;
int pwmizq;

void setup()
{
  Serial.begin(115200);
  // SALIDA PWM PUENTE H
  DDRE |= (1 << DDE3);
  DDRG |= (1 << DDG5);
  DDRB |= (1 << DDB4) | (1 << DDB5) | (1 << DDB6); // ENABLE PWM PUENTE H
  DDRH |= (1 << DDH4) | (1 << DDH3) | (1 << DDH6); // DDH6 ENABLE PWM PUENTE H

  //SALIDAS PUENTE H ENABLE
  //DDRL |= (1 << DDL4) | (1 << DDL5) | (1 << DDL6) | (1 << DDL7);

  //ENTRADA BTN FRONT
  DDRE &= ~((1 << DDE5) | (1 << DDE4));               // PINES BOTONES FRONT
  DDRD &= ~((1 << DDD2) | (1 << DDD3));               // PINES BOTONES BACK
  attachInterrupt(digitalPinToInterrupt(BTNFRONTF), blink, RISING);
}

void loop()
{
  // put your main code here, to run repeatedly:
  changeState();
  switch (state)
  {
    case CW:
      motor_CW();
      break;

    case STOP:
      motor_stop();
      break;
  }
  //delay(3000);
  //motor_stop();
  //delay(3000);
}

//Function buat motor muter ke CW
void motor_CW()
{
  digitalWrite(PWMIZQ0, HIGH); // LENIZQ // LOW Y OTRO HIGH ES PARA ATRAS
  digitalWrite(PWMIZQ1, LOW);  //RENIZQ

  digitalWrite(PWMDER0, LOW);
  digitalWrite(PWMDER1, HIGH);

  //-----------------------------------------------------------
  /*
    if (fadeValue < 255)
    {
    if (fadeValue >= 250)
    {
      fadeValue = 255;
    }
    else
    {
      fadeValue++;
    }
    analogWrite(RENIZQ, fadeValue); //PWMIZQ0
    analogWrite(LENIZQ, fadeValue); //PWMIZQ1
    if (fadeValue > 140) {
      if (fadeValue > 220) {
        delay(80);
      } else {
        delay(60);
      }
    } else {
      if (fadeValue > 50) {
        delay(7);
      } else {
        delay(3);
      }

    }

    }
  */

  if (fadeValue < 447)
  {
    fadeValue++;  // IZQ
    fadeValue1++; // DER
    //fadeValue = 0;
    //fadeValue1 = 0;
  }
  //else
  //{

  //}

  pwmizq = funcPwm(fadeValue);
  pwmder = funcPwm(fadeValue1);

  Serial.println(pwmizq);
  Serial.println(pwmder);

  analogWrite(RENIZQ, pwmizq); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(LENIZQ, pwmizq); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(LENDER, pwmder); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(RENDER, pwmder); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain

  delay(10);

  //-----------------------------------------------------------
  //analogWrite(RENIZQ, 255); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  //analogWrite(LENIZQ, 255); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain

  //-----------------------------------------------------------
  /*
    if (fadeValue1 < 255)
    {
    if (fadeValue1 >= 250)
    {
      fadeValue1 = 255;
    }
    else
    {
      fadeValue1++;
    }
    Serial.print("fadevalue1:  ");
    Serial.println(fadeValue1);
    analogWrite(LENDER, fadeValue1); //PWMDER0
    analogWrite(RENDER, fadeValue1); //PWMDER1
    if (fadeValue1 > 140) {
      delay(60);
    } else {
      delay(3);
    }
    }
  */

  //-----------------------------------------------------------
  //analogWrite(LENDER, 255); //PWMDER0    Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  //analogWrite(RENDER, 255); // PWMDER1    Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain

  //Serial.println("SENTIDO HORARIO");
}

//Function buat motor STOP
void motor_stop()
{
  if (pwmizq > 0)
  {
    if (pwmizq <= 2)
    {
      pwmizq = 0;
      pwmder = 0;
    }
    else
    {
      pwmizq--;
      pwmder--;
    }

    Serial.print("fadevalue:  ");
    Serial.println(pwmizq);
    Serial.print("fadevalue1:  ");
    Serial.println(pwmder);
    analogWrite(RENDER, pwmizq);
    analogWrite(LENDER, pwmizq);
    analogWrite(RENIZQ, pwmder);
    analogWrite(LENIZQ, pwmder);

    if (pwmizq > 100)
    {
      delay(3);
    }
    else
    {
      delay(100);
    }
  }

  analogWrite(RENDER, 0); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(LENDER, 0); //Value "100" bisa diganti dengan speed yang diinginkan (0-1024), atau menggunakan input potensio, atau yang lain
  analogWrite(RENIZQ, 0);
  analogWrite(LENIZQ, 0);
}

void changeState()
{
  /*
    static unsigned long previousMillis3 = 0;
    if ((millis() - previousMillis3) > 10000)
    {
    if (state > 1)
    {
      state = 0;
    }
    else
    {
      state = state + 1;
    }

    //fadeValue = 0;
    //fadeValue1 = 0;

    previousMillis3 += 10000;
    }
  */
  if (Serial.available() > 0)
  {
    char incomingByte = Serial.read();
    Serial.println(incomingByte);
    if (incomingByte == '0')
    {
      state = 0;
    }
    else
    {
      state = 1;
    }
  }
}

int funcPwm(int &t)
{
  return (int)2 * exp(0.0108 * t);
}

void blink() {
  cli();
  delay(600);

  Serial.println("--------------------------------------------------------INTE");
  if (state == 0) {
    state = 1;
  } else {
    state = 0;
  }
  sei();
}
