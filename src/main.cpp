/*
LineFollowerTH

TODO:
* Agregar control PID
* Definir celda de carga
----------------
Pin Mapping:
Physical Pin      Arduino Pin    Port Pin     Function
========================================================
22                D22            PA0          S0 Mux der
23                D23            PA1          S1 Mux der
24                D24            PA2          S2 Mux der
25                D25            PA3          S3 Mux der
54                A0             PF0          SIG Mux der
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
*/

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <ADC.h>
#include <PWM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HX711.h>

// ------------- HX711
#define LOADCELL_DOUT_PIN 2
#define LOADCELL_SCK_PIN 3

// ------------- LCD ------------------
#define COLUMS 20
#define ROWS 4

#define LCD_SPACE_SYMBOL 0x20 //space symbol from the LCD ROM, see p.9 of GDM2004D datasheet
// ------------------------------------

//#define S0D PORTA0 //22  // OUTPUT
//#define S1D PORTA1 //23  // OUTPUT
//#define S3D PORTA2 //24  // OUTPUT
//#define S2D PORTA3 //25  // OUTPUT
//#define SIGD 1     //A0 // INPUT CANAL 0

#define BATTLEVEL 4 //A4 // INPUT CANAL 1
#define MOTLEVEL 5  //A5 // INPUT CANAL 1

//Pines Sensor distancia infrarojo
#define IRDISF 2 //Input Analog CANAL 2
#define IRDISB 3 //Input Analog CANAL 3

// Pines direccion puente h
#define ENIZQ 9        // 9 PORTH6
#define ENDER 10       // 10 PORTB4
#define PWMIZQ0 PORTH3 // 6 PORTH3
#define PWMIZQ1 PORTH4 // 7 PORTH4

//#define RENDER 11      // 11 PORTB5
//#define LENDER 12      // 12 PORTB6
#define PWMDER0 PORTG5 // 4 PORTG5
#define PWMDER1 PORTE3 // 5 PORTE3

#define BUZZER PORTL7 //42 Output
#define LUZ PORTL6    //43 Output

#define BTNFRONTF 18 // PORTD3 Input
#define BTNFRONTB 19 // PORTD2 Input
#define BTNLINE PK7  //PK7 // PJ0 Input

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

uint8_t linestatus = 1; //1 -> negro 0->blanco

boolean chargeFlag = false;

// ------------------- Arranque y parada
uint16_t fadeValue;
int velPwm;

// ------------------- Sensores de linea
volatile uint16_t lineSenBack[6];
volatile uint16_t lineSenLatIzq[2];
volatile uint16_t lineSenLatDer[2];
volatile uint16_t lineSenFront[6];

// ------------------- Sensores ultrasonicos
volatile float distUltra[6];

// ------------------- Sensores IR distancia
volatile uint16_t disIrSenValue[2];

// ------------------- HX711
HX711 hx711;
long loadcellvalue;

// ------------------- LCD INIT
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

// -------------------------------------- PID
float KP = 0.25;  //constante proporcional 0.25
float KD = 8.5;   //constante derivativa
float KI = 0.001; //constante integral

int vel = 50;       //VELOCIDAD MÁXIMA DEL ROBOT MÁXIMA 255
int velrecta = 255; //VELOCIDAD MÁXIMA DEL ROBOT MÁXIMA 255
int velcurva = 40;  //VELOCIDAD MÁXIMA DEL ROBOT MÁXIMA 255
// --------------------------------------

// -------------------------------------- SENSORES
//volatile uint16_t sensores[6];
uint8_t factor = 100; // factor multiplicativo sensor promedio ponderado (calculo de posicion)
int posicion;
int lastpos;
// --------------------------------------

// ---------- Variable PID ----------
int proporcional = 0;
int integral = 0;
int derivativo = 0;
int diferencial = 0;
int last_prop;
int setpoint = 350; // Mitad de sensores

// datos para la integral
int errors[6];

// ------------------- INICIALIZAR FUNCIONES
int funcPwm(uint16_t &t);

void motores(int izq, int der);
void motor_stop();
void motor_CW();
void motor_CCW();

void readVoltage();
void readLoadCell();
void readUltrasonicSen();
void readDisIrSen();
void readSensLinea();
void initLCD();

ISR(INT2_vect) //
{
  static unsigned long lastintetime = 0;
  unsigned long interruptiontime = millis();
  //si la interrupcion dura menos de 200ms entonces es un rebote (ignorar)
  if (interruptiontime - lastintetime > 200)
  {
    // ----------------- DO
    if (chargeFlag) //Si necesita carga entonces
    {
      PWM_on();
      state = CHARGE; // Estado es CHARGE
    }
    else
    {
      PWM_on();
      state = BACKB;
    }
  }
  lastintetime = interruptiontime;
}

ISR(INT3_vect)
{
  static unsigned long lastintetime = 0;
  unsigned long interruptiontime = millis();
  //si la interrupcion dura menos de 200ms entonces es un rebote (ignorar)
  if (interruptiontime - lastintetime > 200)
  {
    // ----------------- DO
    if (chargeFlag) //Si necesita carga entonces
    {
      chargeFlag = false; // Estado es STOP restablecemos carga
      PWM_off();
      state = STOP;
    }
    else
    {
      PWM_on();
      state = BACKF;
    }
  }
  lastintetime = interruptiontime;
}

ISR(PCINT2_vect)
{
  static unsigned long lastintetime = 0;
  unsigned long interruptiontime = millis();
  //si la interrupcion dura menos de 200ms entonces es un rebote (ignorar)
  if (interruptiontime - lastintetime > 200)
  {
    if (linestatus == 1) // si es negro
    {
      linestatus = 0; // blanco
      Serial.println("------------ PCINT");
      Serial.println(linestatus);
    }
    else
    {
      linestatus = 1; // blanco
      Serial.println("------------ PCINT");
      Serial.println(linestatus);
    }
  }
  lastintetime = interruptiontime;
}

void setup()
{
  cli();
  Serial.begin(115200);
  Wire.begin();

  /*
  initLCD();
  
  hx711.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  */

  Serial.println("Start");

  //CONFIGURAR ADC
  ADCInit();

  //CONFIGURAR PWM
  PWM_init();
  PWM_on();

  //-----------ENTRADA SENSORES DE LINEA
  DDRJ &= ~((1 << DDJ1) | (1 << DDJ0));
  DDRH &= ~((1 << DDH1) | (1 << DDH0));
  //DDRD &= ~((1 << DDD2) | (1 << DDD3)); // no se pueden usar por botoneras

  DDRA &= ~(0xFF); // INPUTS SENSORES DE LINEA

  DDRL &= ~((1 << DDL2) | (1 << DDL3) | (1 << DDL1) | (1 << DDL0)); // 47 46 48 49

  //-----------CONFIGURACION PINES DE SALIDA BTNS LEDS
  DDRL |= (1 << DDL7) | (1 << DDL6) | (1 << DDL5) | (1 << DDL4); // BUZZER , LUZ ,  ALARMA, COLOR LED EMERGENCY , COMUN LUZ LED BTNS

  //----------- CONFIGURACION PINES DE SALIDA PUENTE H Y ULTRASONICOS
  DDRC |= (1 << DDC1) | (1 << DDC3) | (1 << DDC5) | (1 << DDC7); // TRIG ULTRASONIC SENSOR
  DDRH |= (1 << DDH4) | (1 << DDH3) | (1 << DDH6);               // PWMIZQ1 PWMIZQ0 RENIZQ
  DDRB |= (1 << DDB4);                                           // RENDER     // LENDER | (1 << DDB6)
  DDRG |= (1 << DDG5) | (1 << DDG1);                             // PWMDER0 Y TRIG ULTRASONIC SENSOR
  DDRD |= (1 << DDD7);                                           // TRIG ULTRASONIC SENSOR
  DDRE |= (1 << DDE3);                                           // PWMDER1

  //----------- CONFIGURACION PINES DE ENTRADA BTNS
  //DDRE &= ~((1 << DDE5) | (1 << DDE4)); // PINES BOTONES BACK
  DDRD &= ~((1 << DDD2) | (1 << DDD3)); // PINES BOTONERAS CONECTADAS EN PARALELO
  DDRJ &= ~((1 << DDJ0));               //ENTRADA BTN LINE PCINT9

  //---------------------- CONFIGUACION INTERRUPCIONES BOTONERAS -------------------
  EICRA &= ~((1 << ISC20) | (1 << ISC30));
  EICRA |= (1 << ISC21) | (1 << ISC31); // FLANCO DE BAJADA INT2(FRONT ATRAS) E INT3 (FRONT ADELANTE)

  //EICRB &= ~((1 << ISC40) | (1 << ISC50));
  //EICRB |= ((1 << ISC41) | (1 << ISC51)); // FLANCO DE BAJADA INT4(BACK ATRAS) E INT5(BACK ADELANTE)

  EIMSK |= ((1 << INT2) | (1 << INT3)); // ACTIVAR INTERRUPCION | (1 << INT4) | (1 << INT5)

  //---------------------- CONFIGUACION INTERRUPCION BTNLINE -------------------
  PCICR |= (1 << PCIE2);    //ACTIVAR INTERRUPCION PCINT15:8 INTERES(PCINT9)
  PCMSK2 |= (1 << PCINT23); // ACTIVR MASK PCINT9

  //----------- PINES PULL UP ULTRASONICOS

  DDRC &= ~((1 << DDC6) | (1 << DDC4) | (1 << DDC2) | (1 << DDC0));
  DDRG &= ~((1 << DDG2) | (1 << DDG0));

  PORTC |= (1 << PORTC6) | (1 << PORTC4) | (1 << PORTC2) | (1 << PORTC0);
  PORTG |= (1 << PORTG2) | (1 << PORTG0);

  //----------- ESTADO INICIAL
  // COMUN RELE
  PORTL |= (1 << PORTL4);
  _delay_us(500000);
  PORTL &= ~(1 << PORTL4); // Activado
  _delay_us(500000);

  PORTL |= (1 << PORTL5);
  _delay_us(500000);
  PORTL &= ~(1 << PORTL5); // Activado
  _delay_us(500000);

  sei();
}

void loop()
{
  //readLineSensor();
  //changeState(); // Change state

  static unsigned long previousMillis1 = 0;
  if ((millis() - previousMillis1) > 1000)
  {
    //------------DO
    //readUltrasonicSen();
    //readLoadCell();
    readSensLinea();
    //readVoltage();

    previousMillis1 += 1000;
  }

  switch (state)
  {
  case BACKF:
    motor_CW();
    break;

  case BACKB:
    motor_CCW();

    //readSensLinea();
    //posicion = calcPosicion();
    //PID();
    //TODO: implementar funcion para girar cuando detecta marcas

    break;

  case STOP:
    motor_stop();
    break;
  }
}

void initLCD()
{
  while (lcd.begin(COLUMS, ROWS) != 1) //colums - 20, rows - 4
  {
    Serial.println(F("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal."));
    delay(2000);
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
  //Serial.println(velPwm);

  setDutyPWMIZQ(velPwm);
  setDutyPWMDER(velPwm);

  //analogWrite(RENIZQ, velPwm);
  //analogWrite(LENIZQ, velPwm);
  //analogWrite(ENDER, velPwm);
  //analogWrite(ENIZQ, velPwm);
  Serial.println("CW");
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
  //Serial.println(velPwm);

  //analogWrite(ENDER, velPwm);
  //analogWrite(ENIZQ, velPwm);
  setDutyPWMIZQ(velPwm);
  setDutyPWMDER(velPwm);
  //analogWrite(RENIZQ, velPwm);
  //analogWrite(LENIZQ, velPwm);
  //analogWrite(LENDER, velPwm);
  //analogWrite(RENDER, velPwm);
  Serial.println("CCW");
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
    //Serial.print("velPwm:  ");
    //Serial.println(velPwm);
    //analogWrite(RENDER, velPwm);
    //analogWrite(LENDER, velPwm);
    //analogWrite(RENIZQ, velPwm);
    //analogWrite(LENIZQ, velPwm);
    //setDutyPWMIZQ(velPwm);
    //setDutyPWMDER(velPwm);
    if (velPwm > 100)
    {
      delay(3);
    }
    else
    {
      delay(80);
    }
  }

  //analogWrite(ENDER, 0);
  //analogWrite(ENIZQ, 0);
  setDutyPWMIZQ(0); //STOP
  setDutyPWMDER(0);
  //analogWrite(RENDER, 0);
  //analogWrite(LENDER, 0);
  //analogWrite(RENIZQ, 0);
  //analogWrite(LENIZQ, 0);

  //Serial.println("STOP");
}

/*
  @brief Leer mux para sensoresde linea
*/
void readSensLinea()
{
  //PORTA = (((i & 0x01) > 0 ? 1 : 0) << S0D) | (((i & 0x02) > 0 ? 1 : 0) << S1D) | (((i & 0x04) > 0 ? 1 : 0) << S2D) | (((i & 0x08) > 0 ? 1 : 0) << S3D);

  /*
    Serial.print((i & 0x01) > 0 ? 1 : 0);
    Serial.print("----");
    Serial.print((i & 0x02) > 0 ? 1 : 0);
    Serial.print("----");
    Serial.print((i & 0x04) > 0 ? 1 : 0);
    Serial.print("----");
    Serial.print((i & 0x08) > 0 ? 1 : 0);
    Serial.println("");
    */
  /*digitalWrite(S0, i & 0x01);
    digitalWrite(S1, i & 0x02);
    digitalWrite(S2, i & 0x04);
    digitalWrite(S3, i & 0x08);*/
  // FRONT

  lineSenFront[0] = PINJ & (1 << PINJ1);
  lineSenFront[1] = PINJ & (1 << PINJ0);
  lineSenFront[2] = PINH & (1 << PINH1);
  lineSenFront[3] = PINH & (1 << PINH0);
  lineSenFront[4] = PINL & (1 << PINL1);
  lineSenFront[5] = PINL & (1 << PINL0);

  for (size_t i = 0; i < 6; i++)
  {
    Serial.print(lineSenFront[i]);
    Serial.print("--");
  }

  lineSenLatDer[0] = PINA & (1 << PINA0);
  lineSenLatDer[1] = PINA & (1 << PINA1);

  for (size_t i = 0; i < 2; i++)
  {
    Serial.print(lineSenLatDer[i]);
    Serial.print("--");
  }

  lineSenBack[0] = PINA & (1 << PINA2);
  lineSenBack[1] = PINA & (1 << PINA3);
  lineSenBack[2] = PINA & (1 << PINA4);
  lineSenBack[3] = PINA & (1 << PINA5);
  lineSenBack[4] = PINA & (1 << PINA6);
  lineSenBack[5] = PINA & (1 << PINA7);
  for (size_t i = 0; i < 6; i++)
  {
    Serial.print(lineSenBack[i]);
    Serial.print("--");
  }

  lineSenLatIzq[0] = PINL & (1 << PINL3);
  lineSenLatIzq[1] = PINL & (1 << PINL2);

  for (size_t i = 0; i < 2; i++)
  {
    Serial.print(lineSenLatIzq[i]);
    Serial.print("--");
  }

  Serial.println("");
}

/*
  @brief Leer mux para sensoresde linea
  @param diection TRUE es para seleccionar 
         barra delantera FALSE para seleccionar barra trasera
*/
int calcPosicion()
{
  unsigned long sumap = 0;
  int suma = 0;
  int pos = 0;
  for (uint8_t i = 0; i < 6; i++) // 6 sensores
  {
    sumap += state == BACKF ? lineSenFront[i] * (i + 1) * factor : lineSenBack[i] * (i + 1) * factor;
    suma += state == BACKF ? lineSenFront[i] : lineSenBack[i];
  }
  pos = (sumap / suma);

  if (lastpos <= 100 && pos == -1)
  {
    pos = 0;
  }
  else if (lastpos >= 500 && pos == -1)
  {
    pos = 600;
  }
  lastpos = pos;
  return pos;
}

/*
  @brief Calcular corecion para aplicar a los motores
*/
void PID()
{
  proporcional = posicion - setpoint;
  derivativo = proporcional - last_prop;

  for (uint8_t i = 0; i < 6; i++)
  {
    integral += errors[i];
  }

  last_prop = proporcional;

  errors[5] = errors[4];
  errors[4] = errors[3];
  errors[3] = errors[2];
  errors[2] = errors[1];
  errors[1] = errors[0];
  errors[0] = proporcional;

  diferencial = (proporcional * KP) + (derivativo * KD) + (integral * KI);

  if (diferencial > vel)
  {
    diferencial = vel;
  }
  else if (diferencial < -vel)
  {
    diferencial = -vel;
  }

  diferencial < 0 ? motores(vel, vel + diferencial) : motores(vel - diferencial, vel);
}

/*
 @brief Escribir en motores
 @param velocidad izquierda velocidad deecha
*/
void motores(int izq, int der)
{
  //----------- MOT IZQ -----------
  if (izq >= 0)
  {
    //ADELANTE
    //PORTD |= (1 << MOT_IZQ_ADELANTE); // MOTOR ON
    //PORTD &= ~(1 << MOT_IZQ_ATRAS);   // MOTOR OFF

    //TODO : CONDICIONAL CAMBIAR DE DIRECCION EGUN EL ESTADO
    if (state == BACKF)
    {
      // ADELANTE MOTOR IZQ
      PORTH &= ~(1 << PWMIZQ1);
      PORTH |= (1 << PWMIZQ0);
    }
    else // en reversa
    {
      PORTH |= (1 << PWMIZQ1);
      PORTH &= ~(1 << PWMIZQ0);
    }
  }
  else
  {
    //ATRAS
    //PORTD &= ~(1 << MOT_IZQ_ADELANTE); // MOTOR ON
    //PORTD |= (1 << MOT_IZQ_ATRAS);     // MOTOR OFF
    if (state == BACKF)
    {
      PORTH |= (1 << PWMIZQ1);
      PORTH &= ~(1 << PWMIZQ0);
    }
    else
    {
      // ADELANTE MOTOR IZQ
      PORTH &= ~(1 << PWMIZQ1);
      PORTH |= (1 << PWMIZQ0);
    }

    izq = abs(izq); // convert to positive value
  }
  //Escribir PWM MOT IZQ
  //izq = map(izq, 0, 255, 1, 100);
  setDutyPWMIZQ(izq); // 0...255 TO 0...100

  //Serial.print(izq);
  //Serial.print("\t");

  //----------- MOT DER -----------
  if (der >= 0)
  {
    //ADELANTE
    //PORTB |= (1 << MOT_DER_ADELANTE); // MOTOR ON
    //PORTD &= ~(1 << MOT_DER_ATRAS);   // MOTOR OFF

    if (state == BACKF)
    {
      PORTE |= (1 << PWMDER1);
      PORTG &= ~(1 << PWMDER0);
    }
    else
    {
      PORTE &= ~(1 << PWMDER1);
      PORTG |= (1 << PWMDER0);
    }
  }
  else
  {
    //ATRAS
    //PORTB &= ~(1 << MOT_DER_ADELANTE); // MOTOR ON
    //PORTD |= (1 << MOT_DER_ATRAS);     // MOTOR OFF
    if (state == BACKF)
    {
      PORTE &= ~(1 << PWMDER1);
      PORTG |= (1 << PWMDER0);
    }
    else
    {
      PORTE |= (1 << PWMDER1);
      PORTG &= ~(1 << PWMDER0);
    }

    der = abs(der); // convert to positive value
  }
  //Escribir PWM MOT DER
  //der = map(der, 0, 255, 1, 100);
  setDutyPWMDER(der); // 0...255 TO 0...100
  //Serial.println(der);
}

/*
  @brief funcion exponencial para generar arranque suave  
  @param tiempo
*/
int funcPwm(uint16_t &t)
{
  return (int)2 * exp(0.0108 * t);
}

void readLoadCell()
{
  if (hx711.is_ready())
  {
    loadcellvalue = hx711.read();
    Serial.print("HX711 reading: ");
    Serial.println(loadcellvalue);
  }
  else
  {
    Serial.println("HX711 not found.");
  }
}

/*
  @brief Medir nivel de bateria
*/
void readVoltage()
{
  uint16_t read = ADCGetData(BATTLEVEL);
  uint16_t level = map(read, 0, 1023, 0, 4700);

  if (level <= 3900)
  {
    chargeFlag = true;
  }
  else
  {
    chargeFlag = false;
  }

  Serial.print("-----> NIVEL DE BAT: ");
  Serial.println(level);
}

/*
  @brief Leer sensores distancia infrarojo
*/
void readDisIrSen()
{
  disIrSenValue[0] = ADCGetData(IRDISF); // leer dis front
  disIrSenValue[1] = ADCGetData(IRDISB); // leer dis back
  Serial.print("----> DIS IR: ");
  Serial.print(disIrSenValue[0]);
  Serial.print("---");
  Serial.print(disIrSenValue[1]);
}

void readUltrasonicSen()
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
}