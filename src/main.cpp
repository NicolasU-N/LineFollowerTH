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
//#include <ADC.h>
#include <PWM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//#include <HX711.h>
#include "SingleEMAFilterLib.h"

SingleEMAFilter<int> singleEMAFilter1(0.4); // 0.2
SingleEMAFilter<int> singleEMAFilter2(0.3); // 0.2

// ------------- HX711
//#define LOADCELL_DOUT_PIN 52
//#define LOADCELL_SCK_PIN 50

// ------------- LCD ------------------
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

// ------------------------------------

#define BATTLEVEL A4 // INPUT CANAL 1
#define MOTLEVEL A13 // INPUT CANAL 13

//Pines Sensor distancia infrarojo
#define IRDISF A2 //Input Analog CANAL 2
#define IRDISB A3 //Input Analog CANAL 3

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
#define BTNSTOP 19   // PORTD2 Input

#define BTNLINE PK7 //PK7 // PJ0 Input

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

uint8_t state = STOP;

boolean linestatus = true; // true->negra | false->blanca

boolean chargeFlag = false; //false -> no necesita carga true -> necesita caraga

// ------------------- Arranque y parada
int fadeValue;
int velPwm;

// ------------------- Sensores de linea
volatile byte lineSenBack[6];
volatile byte lineSenLatIzq[2];
volatile byte lineSenLatDer[2];
volatile byte lineSenFront[6];

// ------------------- Sensores ultrasonicos
volatile float distUltra[6];

// ------------------- Sensores IR distancia
volatile int disIrSenValue[2];

// ------------------- HX711
//HX711 hx711;
long loadcellvalue;

// -------------------------------------- PID
float KP = 0.99;  //constante proporcional 0.25     .83 con menos de 190      ||||||||||||||||.93 para 190
float KD = 8.2;   //constante derivativa // 8.5 con 195
float KI = 0.001; //constante integral

int vel = 200; //VELOCIDAD M??XIMA DEL ROBOT M??XIMA 255 // 0.95  se sale mucho
//int velrecta = 255; //VELOCIDAD M??XIMA DEL ROBOT M??XIMA 255
//int velcurva = 150; //VELOCIDAD M??XIMA DEL ROBOT M??XIMA 255

int veladelante = 235; //VELOCIDAD DEL FRENO DIRECCI??N ADELANTE
int velatras = 230;    //VELOCIDAD DEL FRENO DIRECCI??N ATR??S
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

// -------------------------------------- BANDERAS
boolean flagFuncArranque = true; // TRUE  cuando no corre el arranque  FALSE cuando ya lo corre
boolean flagArrStopAuto = true;  // TRUE  cuando arranque stop es auto FALSE Cuando se para por btn
boolean flagObstaculo = false;   // FALSE cuando no hay obstaculo      TRUE cuando hay obstaculo
boolean flagObstaculoIR = false;

boolean flagParadaObstaculo = false; // FALSE cuando no hay parada por obstaculo TRUE cuando hay parada por obstaculo

boolean flagArranqueconCarga = false; // TRUE cuando se necesita arrancar con m??s carga FALSE cuando no necesita espera de arranque

// --------------------------------------

//--------------------------------------- FLAG PESO
String str;
float pesoCelda;

//--------------------------------------- BATERIA
float level;

// ------------------- INICIALIZAR FUNCIONES
int funcPwm(int &t);

void detectarObstaculo();

void motores(int izq, int der);
void motor_stop();
void motor_CW();
//void motor_CCW();

void readVoltEmergency();
void readVoltage();
void readLoadCell();
void readUltrasonicSen();
void readDisIrSen();
void readSensLinea();
void initLCDi();

void printLcdRecorrido();
void printLcdStop();

void PID();
int calcPosicion();
float mapf(float x, float in_min, float in_max, float out_min, float out_max);

void compensacionVelocidad();

ISR(INT2_vect) //
{
  static unsigned long lastintetime = 0;
  unsigned long interruptiontime = millis();
  //si la interrupcion dura menos de 200ms entonces es un rebote (ignorar)
  if (interruptiontime - lastintetime > 200)
  {
    // ----------------- DO

    Serial.println("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$------------------------ STOPPPPPPPPP");

    state = STOP;

    //APAGAR LUZ SIRENA
    PORTL |= (1 << PORTL6);
    PORTL &= ~(1 << PORTL7); // PITO OFF

    flagArrStopAuto = false; // ESTADO DE ARRANQUE AUTO
    flagFuncArranque = true; // Arranque suave
    fadeValue = 0;
    velPwm = 0;

    motores(0, 0);
    PWM_off();
  }
  lastintetime = interruptiontime;
}

ISR(INT3_vect)
{

  static unsigned long lastintetime1 = 0;
  unsigned long interruptiontime1 = millis();
  //si la interrupcion dura menos de 200ms entonces es un rebote (ignorar)
  if (interruptiontime1 - lastintetime1 > 200)
  {
    // ----------------- DO

    if (chargeFlag) //Si necesita carga entonces
    {
      PWM_off();
      state = STOP; // Estado es CHARGE

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(" BATER??A BAJA ");
      lcd.setCursor(0, 1);
      lcd.print(" CARGAR BATER??AS ");
    }
    else
    {
      Serial.println("#######################################------------------------ BACKBBBBBBB");
      //lcd.clear();
      PWM_on();
      state = BACKB;
    }
  }
  lastintetime1 = interruptiontime1;
}

ISR(INT4_vect)
{
  static unsigned long lastintetime = 0;
  unsigned long interruptiontime = millis();
  //si la interrupcion dura menos de 200ms entonces es un rebote (ignorar)
  if (interruptiontime - lastintetime > 200)
  {
    if (chargeFlag) //Si necesita carga entonces
    {
      PWM_off();
      state = STOP; // Estado es CHARGE

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(" BATER??A BAJA ");
      lcd.setCursor(0, 1);
      lcd.print(" CARGAR BATER??AS ");
    }
    else
    {
      Serial.println("--------------------------------------------------------- BACKFFFFFF");

      PWM_on();
      state = BACKF;
    }
  }
  lastintetime = interruptiontime;
}

ISR(PCINT2_vect)
{

  //static unsigned long lastintetime = 0;
  //unsigned long interruptiontime = millis();
  //si la interrupcion dura menos de 200ms entonces es un rebote (ignorar)
  //if (interruptiontime - lastintetime > 200)
  //{

  if (linestatus) // si es negro
  {
    linestatus = false; // blanco
    Serial.println("------------ PCINT");
    Serial.println(linestatus);
  }
  else
  {
    linestatus = true; // blanco
    Serial.println("------------ PCINT");
    Serial.println(linestatus);
  }
  //}
  //lastintetime = interruptiontime;
}

void setup()
{
  initLCDi();

  cli();
  Serial.begin(115200);
  Serial2.begin(9600);

  Serial.println("Start");

  //CONFIGURAR ADC
  //ADCInit(); NO TIENE FILTRO, se usa funcion nativa del frame work

  //CONFIGURAR PWM
  PWM_init();
  PWM_on();

  //-----------ENTRADA SENSORES DE LINEA
  DDRJ &= ~((1 << DDJ1) | (1 << DDJ0));
  //DDRH &= ~((1 << DDH1) | (1 << DDH0)); 16 y 17
  DDRB &= ~((1 << DDB1) | (1 << DDB0)); // 52 Y 53

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
  DDRE &= ~((1 << DDE4));               // (1 << DDE5) | PINES BOTONES BACK
  DDRD &= ~((1 << DDD2) | (1 << DDD3)); // PINES BOTONERAS CONECTADAS EN PARALELO
  DDRJ &= ~((1 << DDJ0));               //ENTRADA BTN LINE PCINT9

  //---------------------- CONFIGUACION INTERRUPCIONES BOTONERAS -------------------
  EICRA &= ~((1 << ISC20) | (1 << ISC30)); // CAMBIAR OPERADOR |= POR &= ~ para flanco de bajada
  EICRA |= (1 << ISC21) | (1 << ISC31);    // FLANCO DE SUBIDA INT2(FRONT ATRAS) E INT3 (FRONT ADELANTE)

  EICRB &= ~((1 << ISC40)); //  CAMBIAR OPERADOR |= POR &= ~ para flanco de bajada                       | (1 << ISC50)
  EICRB |= ((1 << ISC41));  //       | (1 << ISC51) FLANCO DE SUBIDA INT4(BACK ATRAS) E INT5(BACK ADELANTE)

  EIMSK |= ((1 << INT2) | (1 << INT3) | (1 << INT4)); // ACTIVAR INTERRUPCION | (1 << INT4) | (1 << INT5)

  //---------------------- CONFIGUACION INTERRUPCION BTNLINE -------------------
  PCICR |= (1 << PCIE2);    //ACTIVAR INTERRUPCION PCINT15:8 INTERES(PCINT9)
  PCMSK2 |= (1 << PCINT23); // ACTIVR MASK PCINT23

  //----------- PINES PULL UP ULTRASONICOS

  DDRC &= ~((1 << DDC6) | (1 << DDC4) | (1 << DDC2) | (1 << DDC0));
  DDRG &= ~((1 << DDG2) | (1 << DDG0));

  PORTC |= (1 << PORTC6) | (1 << PORTC4) | (1 << PORTC2) | (1 << PORTC0);
  PORTG |= (1 << PORTG2) | (1 << PORTG0);

  //----------- VENTILADOR CAJA
  DDRB |= (1 << DDB2);
  PORTB |= (1 << PORTB2);

  //----------- ESTADO INICIAL
  // COMUN RELE
  PORTL |= (1 << PORTL4);
  _delay_ms(400);
  PORTL &= ~(1 << PORTL4); // Activado
  _delay_ms(400);

  //LUZ EMERGENCY BTN
  PORTL |= (1 << PORTL5);
  _delay_ms(400);
  PORTL &= ~(1 << PORTL5); // Activado
  _delay_ms(400);

  //BUZZER y LUZ LICUADORA

  PORTL &= ~(1 << PORTL6); // ENCENDID LICUADORA
  PORTL |= (1 << PORTL7);  // ENCENDIDO PITO // Activado
  _delay_ms(400);
  PORTL &= ~(1 << PORTL7);
  PORTL |= (1 << PORTL6);
  _delay_ms(400);

  //TERMINAR SECUENCIA
  PORTL |= (1 << PORTL5); // VERDE BTN EMERGENCY

  //ACTVAR VENTILADOR
  DDRB |= (1 << DDB2);
  PORTB |= (1 << PORTB2);

  sei();

  lcd.clear();
}

void loop()
{

  static unsigned long previousMillis0 = 0;
  if ((millis() - previousMillis0) > 1000)
  {
    //------------DO
    readLoadCell();
    //readDisIrSen();

    readVoltage(); // LEER VOLTAJE BATERIA

    previousMillis0 += 1000;
  }

  static unsigned long previousMillis1 = 0;
  if ((millis() - previousMillis1) > 300)
  {
    //------------DO
    //readUltrasonicSen();
    //readDisIrSen();

    //readLoadCell();

    //                   detectarObstaculo(); //////////////////////////////////////////// POR DEPURAR TEMA DE SENORES

    //Serial.println(disIrSenValue[0]); // ATRASSSS
    //Serial.print("   ");
    //Serial.println(disIrSenValue[1]); // ADELANTE

    /*
    Serial.print(distUltra[0]);
    Serial.print(" ------- ");
    Serial.print(distUltra[2]);
    Serial.print(" ------- ");
    Serial.println(distUltra[5]);
    */

    /*
    Serial.print(flagObstaculoIR);
    Serial.print("--------");
    Serial.println(flagObstaculo);
    */

    readVoltEmergency();

    previousMillis1 += 300;
  }

  /*
  static unsigned long previousMillis3 = 0;
  if ((millis() - previousMillis3) > 80)
  {
    //-------------------------------------------------------DEBUG

    //if (!flagArrStopAuto) // Si es false                //-------------------------- SE MODIFICA LA CONDICIONAL PORQUE NO NECESITAMOS RETORNO
    //{

    //}

    previousMillis3 += 80;
  }
  */

  switch (state)
  {
  case BACKF:
    //ESTADO CUANDO PRESIONA BOTONERA TRASERA

    if (flagFuncArranque)
    {
      PWM_on();
      motor_CW();

      static unsigned long previousMillis5 = 0;
      if ((millis() - previousMillis5) > 4000)
      {
        printLcdRecorrido();
        previousMillis5 += 4000;
      }
    }
    else
    {
      //Serial.println("::::::::::::::::::::::::::: FUERA DE WHILE");

      static unsigned long previousMillis4 = 0;
      if ((millis() - previousMillis4) > 5000) //SE MODIFICA TIEMPO DE IMPRESION BATERIA
      {
        printLcdRecorrido();
        previousMillis4 += 5000;
      }

      readSensLinea();
      posicion = calcPosicion();

      Serial.print("       | POS: ");
      Serial.println(posicion);
      Serial.println("");

      if (posicion <= 150)
      {
        motores(-velatras, veladelante);
      }
      else if (posicion >= 550)
      {
        motores(veladelante, -velatras);
      }
      else
      {
        PID();
      }
    }

    if (flagObstaculo == true or flagObstaculoIR == true)
    {
      Serial.println("DETECCION DE OBSTACULO");
      state = STOP;
      flagParadaObstaculo = true;
      motores(-150, -150);

      PORTL &= ~(1 << PORTL6); // ENCENDID LICUADORA
      PORTL |= (1 << PORTL7);  // ENCENDIDO PITO // Activado
      _delay_ms(1000);
      PORTL &= ~(1 << PORTL7);
      PORTL |= (1 << PORTL6);

      distUltra[0] = 0;
      distUltra[2] = 0;
      distUltra[5] = 0;
      flagObstaculo = false;
      flagObstaculoIR = false;

      lcd.clear();
    }

    // ------------------------------- FUNCTION STOP AUTOMATICO
    if (lineSenFront[0] == 1 and lineSenFront[1] == 1 and lineSenFront[2] == 1 and lineSenFront[3] == 1 and lineSenFront[4] == 1 and lineSenFront[5] == 1) //
    {
      for (size_t i = 0; i < 6; i++) // Limpiar valores de sensores
      {
        lineSenFront[i] = 0;
      }
      posicion = 350;
      motores(-150, -150);
      _delay_ms(900);
      motores(0, 0);
      state = STOP;
      //break;
      //Serial.println("STOP AUTOMATICO");
      //flagArrStopAuto = true;
    }

    break;

  case STOP:

    static unsigned long previousMillis6 = 0;
    if ((millis() - previousMillis6) > 800)
    {
      printLcdStop();
      previousMillis6 += 800;
    }

    motor_stop();
    motores(0, 0);
    PWM_off();

    PORTH &= ~(1 << PWMIZQ0); // LOW Y OTRO HIGH ES PARA ATRAS
    PORTH &= ~(1 << PWMIZQ1);

    PORTE &= ~(1 << PWMDER1);
    PORTG &= ~(1 << PWMDER0);

    //APAGAR LUZ SIRENA
    PORTL |= (1 << PORTL6);
    PORTL &= ~(1 << PORTL7); // PITO OFF

    // RESETEAR FLAG SECUENCIA DE ARRANQUE
    flagFuncArranque = true;
    flagObstaculo = false;
    flagObstaculoIR = false;

    fadeValue = 0;
    velPwm = 0; // SIN CONTROL

    /*
    // CONDICION PARA CAMBIAR DE ESTADO CUANDO PARA POR SENSORES

    static unsigned long previousMillis10 = 0;
    if ((millis() - previousMillis10) > 3500)
    {
      Serial.println("COMPROBANDO TEMPORIZADOR");
      if (flagParadaObstaculo == true)
      {
        Serial.println("ARRANQUEEEE");
        PWM_on();
        state = BACKF;
        flagParadaObstaculo = false;
      }
      previousMillis10 += 3500;
    }
    */
    //_delay_ms(3500); //--------------------------------------------Espera para ver si se retiro el obstaculo

    break;

  case BACKB:
    //motor_CCW();

    //ESTADO CUANDO PRESIONA BOTONERA DELANTERA

    if (flagObstaculo == true or flagObstaculoIR == true)
    {
      Serial.println("DETECCION DE OBSTACULO");
      //state = STOP;
      //motores(-180, -180);
      motores(0, 0);
      PORTL &= ~(1 << PORTL6); // ENCENDID LICUADORA
      PORTL |= (1 << PORTL7);  // ENCENDIDO PITO // Activado
      _delay_ms(200);
      PORTL &= ~(1 << PORTL7);
      PORTL |= (1 << PORTL6);

      _delay_ms(3500); //--------------------------------------------Espera para ver si se retiro el obstaculo

      lcd.clear();
    }
    else
    {

      static unsigned long previousMillis7 = 0;
      if ((millis() - previousMillis7) > 6000)
      {
        printLcdRecorrido();
        previousMillis7 += 6000;
      }

      readSensLinea();
      //posicion = calcPosicion();

      //Serial.print("       | POS: ");
      //Serial.print(posicion);
      //Serial.println("");

      if (lineSenFront[4] == 1 && lineSenFront[5] == 1) // Detecta linea PRIMEROS DE DERECHA
      {
        motores(250, -255);
        PORTL &= ~(1 << PORTL6); // ENCENDID LICUADORA
        PORTL |= (1 << PORTL7);  // ENCENDIDO PITO // Activado
        _delay_ms(1400);
        state = STOP;
        lcd.clear();
      }
      else
      {
        PORTL &= ~(1 << PORTL6);
        PORTL &= ~(1 << PORTL7); // PITO OFF // ENCENDID LICUADORA
        motores(-200, 200);
      }
    }

    break;
  }
}

void readVoltEmergency()
{
  uint16_t motlev = analogRead(MOTLEVEL);
  //Serial.println(motlev);

  // ------------------------------- FUNCTION MEDIR BTN EMERGENCY
  if (motlev > 770)
  {
    PORTL |= (1 << PORTL5); // EL BTN EMERGENCY SUELTO VERDE
  }
  else
  {
    PORTL &= ~(1 << PORTL5); // EL BTN PRESIONADO ROJO
  }
}

void initLCDi()
{
  lcd.init(); // initialize the lcd
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(7, 0);
  lcd.print("BAKER");
  lcd.setCursor(2, 1);
  lcd.print("Todohidraulicos");
  lcd.setCursor(4, 2);
  lcd.print("TH-AutoKart");
  lcd.setCursor(5, 3);
  lcd.print("SMART CART");
}

void printLcdRecorrido()
{
  lcd.setCursor(0, 0);
  lcd.print("Peso: ");
  lcd.setCursor(7, 0);
  lcd.print(pesoCelda);

  lcd.setCursor(17, 0);
  lcd.print("KG");

  lcd.setCursor(0, 1);
  lcd.print("Voltaje: ");
  lcd.setCursor(10, 1);
  lcd.print(level);

  lcd.setCursor(4, 3);
  lcd.print("EN RECORRIDO");
}

void printLcdStop()
{
  lcd.setCursor(0, 0);
  lcd.print("Peso: ");
  lcd.setCursor(7, 0);
  lcd.print(pesoCelda);

  lcd.setCursor(17, 0);
  lcd.print("KG");

  lcd.setCursor(0, 1);
  lcd.print("Voltaje: ");
  lcd.setCursor(10, 1);
  lcd.print(level);

  lcd.setCursor(4, 3);
  lcd.print("  DETENIDO  ");
}

void motor_CW()
{
  //Serial.println("::::::::::::::::::::::::::: DENTRO DE WHILE");
  //ENCENCER LUZ SIRENA
  PORTL &= ~(1 << PORTL6);
  PORTL &= ~(1 << PORTL7); // PITO OFF

  //PUENTE H
  //PORTH &= ~(1 << PWMIZQ1); // LOW Y OTRO HIGH ES PARA ATRAS
  //PORTH |= (1 << PWMIZQ0);

  //PORTE |= (1 << PWMDER1);
  //PORTG &= ~(1 << PWMDER0);

  velPwm = funcPwm(fadeValue);
  //Serial.println(velPwm);

  //setDutyPWMIZQ(velPwm);
  //setDutyPWMDER(velPwm);

  motores(velPwm, velPwm);

  if (fadeValue < 447)
  {
    fadeValue++; // Valor max para 252 pwm
  }
  else
  {
    ///////////////////////////////////////////////
    lcd.clear();
    flagFuncArranque = false;
    flagArrStopAuto = false;

    PORTL &= ~(1 << PORTL6); // ENCENDID LICUADORA
    PORTL &= ~(1 << PORTL7); // ENCENDIDO PITO // off

    // flag espera a maxima potencia arranque
    if (flagArranqueconCarga)
    {
      //Serial.println("----------ESPERA PARA ARRANQUE----------");
      _delay_ms(800);
    }
    // PROBAR BAJANDO VARIABLE A 0 Y MODIFICAR LAS ISR
  }

  //Serial.println("CW");
  //Serial.println("BANDERA DE ESPERA EN ARRANQUE");
  //Serial.println(flagArranqueconCarga);

  _delay_ms(2);
}

/*
void motor_CCW()
{
  //Serial.println("::::::::::::::::::::::::::: DENTRO DE WHILE");
  //ENCENCER LUZ SIRENA
  PORTL &= ~(1 << PORTL6);
  PORTL &= ~(1 << PORTL7); // PITO OFF

  //PUENTE H
  PORTH &= ~(1 << PWMIZQ0); // LOW Y OTRO HIGH ES PARA ATRAS
  PORTH |= (1 << PWMIZQ1);

  PORTE &= ~(1 << PWMDER1);
  PORTG |= (1 << PWMDER0);

  if (fadeValue < 447)
  {
    fadeValue++; // Valor max para 252 pwm
  }
  else
  { // CAMBIAR FLAG
    flagFuncArranque = false;
  }

  velPwm = funcPwm(fadeValue);
  //Serial.println(velPwm);

  setDutyPWMIZQ(velPwm);
  setDutyPWMDER(velPwm);

  //analogWrite(RENIZQ, velPwm);
  //analogWrite(LENIZQ, velPwm);
  //analogWrite(ENDER, velPwm);
  //analogWrite(ENIZQ, velPwm);
  Serial.println("CCW");
  delay(10);
}
*/

void motor_stop()
{
  /*
  //---------------------- FRENO SUAVE
  if (vel > 0)
  {
    if (vel <= 2)
    {
      vel = 0;
    }
    else
    {
      vel--;
    }
    //Escribir velocidad decreciente
    //Serial.print("velPwm:  ");
    //Serial.println(velPwm);
    //analogWrite(RENDER, velPwm);
    //analogWrite(LENDER, velPwm);
    //analogWrite(RENIZQ, velPwm);
    //analogWrite(LENIZQ, velPwm);
    setDutyPWMIZQ(vel);
    setDutyPWMDER(vel);
    if (vel > 100)
    {
      delay(3);
    }
    else
    {
      delay(100);
    }
  }
  else
  {
   
  }
*/
  setDutyPWMIZQ(0); //STOP
  setDutyPWMDER(0);
  //Serial.println("STOP");
}

/*
  @brief Leer sensores de linea
*/
void readSensLinea()
{
  // FRONT
  lineSenFront[0] = ((PINJ & (1 << PINJ1)) > 0 ? linestatus ? 1 : 0 : linestatus ? 0
                                                                                 : 1); // f0
  lineSenFront[1] = ((PINJ & (1 << PINJ0)) > 0 ? linestatus ? 1 : 0 : linestatus ? 0
                                                                                 : 1); // f1
  lineSenFront[2] = ((PINB & (1 << PINB1)) > 0 ? linestatus ? 1 : 0 : linestatus ? 0
                                                                                 : 1); // f2
  lineSenFront[3] = ((PINB & (1 << PINB0)) > 0 ? linestatus ? 1 : 0 : linestatus ? 0
                                                                                 : 1); // f3
  lineSenFront[4] = ((PINL & (1 << PINL1)) > 0 ? linestatus ? 1 : 0 : linestatus ? 0
                                                                                 : 1); // f4
  lineSenFront[5] = ((PINL & (1 << PINL0)) > 0 ? linestatus ? 1 : 0 : linestatus ? 0
                                                                                 : 1); // f5

  /*
    Serial.println((PINJ & (1 << PINJ1)) > 0 ? 1 : 0);
    Serial.println((PINJ & (1 << PINJ0)) > 0 ? 1 : 0);
    Serial.println((PINH & (1 << PINH1)) > 0 ? 1 : 0);
    Serial.println((PINH & (1 << PINH0)) > 0 ? 1 : 0);
    Serial.println((PINL & (1 << PINL1)) > 0 ? 1 : 0);
    Serial.println((PINL & (1 << PINL0)) > 0 ? 1 : 0);
  */

  //if (state == BACKF)
  //{

  for (size_t i = 0; i < 6; i++)
  {
    //Serial.print("f");
    //Serial.print(i);
    //Serial.print("--");

    Serial.print(lineSenFront[i]);
    Serial.print("--");
  }

  //}

  lineSenLatDer[0] = ((PINA & (1 << PINA0)) > 0 ? linestatus ? 1 : 0 : linestatus ? 0
                                                                                  : 1);
  lineSenLatDer[1] = ((PINA & (1 << PINA1)) > 0 ? linestatus ? 1 : 0 : linestatus ? 0
                                                                                  : 1);

  /*
    for (size_t i = 0; i < 2; i++)
    {
      Serial.print("f");
      Serial.print(i);
      Serial.print("--");
      Serial.print(lineSenLatDer[i]);
    }
  */

  /*
  lineSenBack[0] = ((PINA & (1 << PINA5)) > 0 ? linestatus ? 1 : 0 : linestatus ? 0
                                                                                : 1);
  lineSenBack[1] = ((PINA & (1 << PINA4)) > 0 ? linestatus ? 1 : 0 : linestatus ? 0
                                                                                : 1);
  lineSenBack[2] = ((PINA & (1 << PINA3)) > 0 ? linestatus ? 1 : 0 : linestatus ? 0
                                                                                : 1);
  lineSenBack[3] = ((PINA & (1 << PINA2)) > 0 ? linestatus ? 1 : 0 : linestatus ? 0
                                                                                : 1);
  lineSenBack[4] = ((PINA & (1 << PINA6)) > 0 ? linestatus ? 1 : 0 : linestatus ? 0
                                                                                : 1);
  lineSenBack[5] = ((PINA & (1 << PINA7)) > 0 ? linestatus ? 1 : 0 : linestatus ? 0
                                                                                : 1);
  */

  /*
  if (state == BACKB)
  {
    for (size_t i = 0; i < 6; i++)
    {
      //Serial.print("f");
      //Serial.print(i);
      //Serial.print("--");
      Serial.print(lineSenBack[i]);
      Serial.print("--");
    }
  }
  */

  lineSenLatIzq[0] = ((PINL & (1 << PINL3)) > 0 ? linestatus ? 1 : 0 : linestatus ? 0
                                                                                  : 1);
  lineSenLatIzq[1] = ((PINL & (1 << PINL2)) > 0 ? linestatus ? 1 : 0 : linestatus ? 0
                                                                                  : 1);
  /*
  for (size_t i = 0; i < 2; i++)
  {
    //Serial.print("f");
    //Serial.print(i);
    Serial.print(lineSenLatIzq[i]);
    Serial.print("--");
  }
  */
  Serial.println("");
}

/*
  @brief Calcular posicion en base a las lecturas de sensores
*/
int calcPosicion()
{
  unsigned long sumap = 0;
  int suma = 0;
  int pos = 0;
  for (uint8_t i = 0; i < 6; i++) // 6 sensores
  {
    sumap += lineSenFront[i] * (i + 1) * factor; // state == BACKF ? ||||||   : lineSenBack[i] * (i + 1) * factor
    suma += lineSenFront[i];                     // state == BACKF ? : lineSenBack[i]
  }

  pos = (sumap / suma);

  if (lastpos <= 100 && pos == -1)
  {
    pos = 0; // Sale por izquierda
  }
  else if (lastpos >= 500 && pos == -1)
  {
    pos = 600; // Sale por derecha
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

    switch (state)
    {
    case BACKF:
      // ADELANTE MOTOR IZQ
      PORTH &= ~(1 << PWMIZQ1);
      PORTH |= (1 << PWMIZQ0);
      break;
    case BACKB:
      PORTH |= (1 << PWMIZQ1);
      PORTH &= ~(1 << PWMIZQ0);
      break;
    }
  }
  else
  {
    //ATRAS
    //PORTD &= ~(1 << MOT_IZQ_ADELANTE); // MOTOR ON
    //PORTD |= (1 << MOT_IZQ_ATRAS);     // MOTOR OFF

    switch (state)
    {
    case BACKF:
      PORTH |= (1 << PWMIZQ1);
      PORTH &= ~(1 << PWMIZQ0);
      break;

    case BACKB:
      // ADELANTE MOTOR IZQ
      PORTH &= ~(1 << PWMIZQ1);
      PORTH |= (1 << PWMIZQ0);
      break;
    }

    izq = abs(izq); // convert to positive value
  }
  //Escribir PWM MOT IZQ
  //izq = map(izq, 0, 255, 1, 100);

  //setDutyPWMIZQ(izq); // 0...255 TO 0...100 ESTABA ANTES

  // --------------------------------------------------------------------------- DEBUG ----------
  setDutyPWMDER(izq); // 0...255

  /*
  switch (state)
  {
  case BACKF:
    setDutyPWMDER(izq); // 0...255
    break;

  case BACKB:
    setDutyPWMIZQ(izq); // 0...255
    break;
  }
  */

  //----------- MOT DER -----------
  if (der >= 0)
  {
    //ADELANTE
    //PORTB |= (1 << MOT_DER_ADELANTE); // MOTOR ON
    //PORTD &= ~(1 << MOT_DER_ATRAS);   // MOTOR OFF

    switch (state)
    {
    case BACKF:
      PORTE |= (1 << PWMDER1);
      PORTG &= ~(1 << PWMDER0);
      break;
    case BACKB:
      PORTE &= ~(1 << PWMDER1);
      PORTG |= (1 << PWMDER0);
      break;
    }
  }
  else
  {
    //ATRAS
    //PORTB &= ~(1 << MOT_DER_ADELANTE); // MOTOR ON
    //PORTD |= (1 << MOT_DER_ATRAS);     // MOTOR OFF

    switch (state)
    {
    case BACKF:
      PORTE &= ~(1 << PWMDER1);
      PORTG |= (1 << PWMDER0);
      break;

    case BACKB:
      PORTE |= (1 << PWMDER1);
      PORTG &= ~(1 << PWMDER0);
      break;
    }

    der = abs(der); // convert to positive value
  }
  //Escribir PWM MOT DER
  //der = map(der, 0, 255, 1, 100);

  //setDutyPWMDER(der); // 0...255 TO 0...100 ESTABA ANTES

  // --------------------------------------------------------------------------- DEBUG ----------
  setDutyPWMIZQ(der); // 0...255 TO 0...100

  /*
  switch (state)
  {
  case BACKF:
    setDutyPWMIZQ(der); // 0...255 TO 0...100
    break;

  case BACKB:
    setDutyPWMDER(der); // 0...255 TO 0...100
    break;
  }
  */

  /*
  Serial.print(izq);
  Serial.print("     |     ");
  Serial.print(der);
  Serial.println("");
*/
}

/*
  @brief funcion exponencial para generar arranque suave  
  @param tiempo
*/
int funcPwm(int &t)
{
  return (int)2 * exp(0.0108 * t);
}

void compensacionVelocidad()
{
  ////////// -------------------------------------------------------------------- VALIDACION DEL PESO
  if (pesoCelda > 250)
  {
    //ENCENCER LUZ SIRENA
    PORTL &= ~(1 << PORTL6);
    PORTL &= ~(1 << PORTL7); // PITO OFF

    lcd.setCursor(2, 2);
    lcd.print("LIMITE DE CARGA");
    delay(1000);
    state = STOP;
  }
  else
  {

    if (pesoCelda > 0 and pesoCelda < 50)
    {
      KP = 1.5;
      KD = 9.5;
      vel = 200;
      veladelante = 235;
      velatras = 230;
      flagArranqueconCarga = false;
      lcd.setCursor(2, 2);
      lcd.print("                ");
    }

    if (pesoCelda > 50 and pesoCelda < 100)
    {
      KP = 0.95;
      vel = 195;
      veladelante = 235;
      velatras = 230;
      flagArranqueconCarga = false;
      lcd.setCursor(2, 2);
      lcd.print("                ");
    }
    else if (pesoCelda > 100 and pesoCelda < 150)
    {
      KP = 1.1;
      KD = 8.9;
      vel = 205;
      veladelante = 247;
      velatras = 240;

      lcd.setCursor(2, 2);
      lcd.print("                ");

      flagArranqueconCarga = false;
    }
    else if (pesoCelda > 150 and pesoCelda < 250)
    {
      KP = 3.0;
      KD = 9.2;
      vel = 235;
      veladelante = 254;
      velatras = 252;

      lcd.setCursor(2, 2);
      lcd.print("                ");

      //Bandera para que arranque dure m??s
      flagArranqueconCarga = true;
    }
  }
  ////////// -------------------------------------------------------------------- VALIDACION DEL PESO
}

/*
  @brief funcion para leer puerto serial celda de carga
*/
void readLoadCell()
{
  if (Serial2.available())
  {
    str = Serial2.readStringUntil('\n');

    String igual = str.substring(0, 1); // Ignorar el igual

    if (igual.equalsIgnoreCase("="))
    {
      float pesoCelda1;
      pesoCelda1 = str.substring(1).toFloat();
      if (pesoCelda1 != 0)
      {
        pesoCelda = pesoCelda1 - 47; ///////////////////////////RESTAMOS PESO DE CANASTA!!
      }
    }

    //Serial.print("Valor String: ");
    //Serial.println(pesoCelda);
  }

  compensacionVelocidad();
}

/*
  @brief Medir nivel de bateria
*/
void readVoltage()
{
  float read = (float)analogRead(BATTLEVEL);
  level = mapf(read, 0, 855.0, 0, 12.93); //CAMBIAR NIVEL DE VOLTAJE

  if (level < 12)
  {
    chargeFlag = true;
  }
  else
  {
    chargeFlag = false;
  }

  //Serial.print("-----> NIVEL DE BAT: ");
  //Serial.println(level); // level
}

/*
  @brief Funcion map con punto flotante
*/
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void detectarObstaculo()
{
  readDisIrSen();
  readUltrasonicSen();

  //--------------------------------------------------------------------------- Detectar ir sensor
  /*
  if (((disIrSenValue[0] > 110) && (disIrSenValue[0] < 130))) //-------------------------------------------- CALIBRAR SENSORES || ((disIrSenValue[1] > 125) && (disIrSenValue[1] < 220))
  {
    flagObstaculoIR = true;
  }
  else
  {
    flagObstaculoIR = false;
  }
  */

  /*
  if (((disIrSenValue[1] > 125) && (disIrSenValue[1] < 220))) //-------------------------------------------- CALIBRAR SENSORES
  {
    flagObstaculoIR = true;
  }
  else
  {
    flagObstaculoIR = false;
  }
  */

  //--------------------------------------------------------------------------- Detectar ultrasonic sensor
  if (((distUltra[0] > 25) && (distUltra[0] < 100)) || ((distUltra[2] > 25) && (distUltra[2] < 45)) || ((distUltra[5] > 25) && (distUltra[5] < 45))) //
  {
    flagObstaculo = true;
  }
  else
  {
    flagObstaculo = false;
  }
}

/*
  @brief Leer sensores distancia infrarojo
*/
void readDisIrSen()
{
  // leer dis front
  //disIrSenValue[1] = analogRead(IRDISB); // leer dis back

  // Calcular distancia y filtro
  singleEMAFilter1.AddValue(((6787 / (analogRead(IRDISF) - 3)) - 4)); //analogRead(IRDISF)

  disIrSenValue[0] = singleEMAFilter1.GetLowPass();

  //singleEMAFilter2.AddValue(); //analogRead(IRDISB) // SENSOR TRASERO NO ES NECESARIO
  //disIrSenValue[1] = singleEMAFilter2.GetLowPass();

  /*
  Serial.print("----> DIS IR: ");
  Serial.print(disIrSenValue[0]); //adelante
  Serial.print("---");
  Serial.print(disIrSenValue[1]);
  Serial.println("");
  */
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
  /*
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
  */
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
  /*
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
  */
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
  /*
  // PRINT ARRAY
  Serial.print("-- DISTANCE ULTRASONIC SENSOR: ");
  for (size_t i = 0; i < 6; i++)
  {
    Serial.print(distUltra[i]);
    Serial.print("---");
  }
  Serial.println("");
  */
}