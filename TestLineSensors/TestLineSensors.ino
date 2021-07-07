#include <avr/io.h>

// ------------------- Sensores de linea
volatile byte lineSenBack[6];
volatile byte lineSenLatIzq[2];
volatile byte lineSenLatDer[2];
volatile byte lineSenFront[6];

// -------------------------------------- SENSORES
//volatile uint16_t sensores[6];
uint8_t factor = 100; // factor multiplicativo sensor promedio ponderado (calculo de posicion)
int posicion;
int lastpos;
// --------------------------------------

// ------------------- ESTADOS
#define STOP 0
#define BACKF 1
#define BACKB 2
#define FINISH 3
#define CHARGE 4

uint8_t state = BACKB;

// ------------------- COLOR LINEA
uint8_t linestatus = 1; //1 -> negro 0->blanco

void setup() {
  Serial.begin(115200);

  //-----------ENTRADA SENSORES DE LINEA
  DDRJ &= ~((1 << DDJ1) | (1 << DDJ0));
  DDRH &= ~((1 << DDH1) | (1 << DDH0));
  //DDRD &= ~((1 << DDD2) | (1 << DDD3)); // no se pueden usar por botoneras

  DDRA &= ~(0xFF); // INPUTS SENSORES DE LINEA

  DDRL &= ~((1 << DDL2) | (1 << DDL3) | (1 << DDL1) | (1 << DDL0)); // 47 46 48 49

}

void loop() {
  readSensLinea();
  posicion = calcPosicion();
  Serial.print("      POS: ");
  Serial.print(posicion);
  Serial.println("");
  delay(500);
}

/*
  @brief Leer mux para sensoresde linea
*/
void readSensLinea()
{
  // FRONT
  lineSenFront[0] = ((PINJ & (1 << PINJ1))  > 0 ? 1 : 0) ; // f0
  lineSenFront[1] = ((PINJ & (1 << PINJ0))  > 0 ? 1 : 0) ; // f1
  lineSenFront[2] = ((PINH & (1 << PINH1))  > 0 ? 1 : 0) ; // f2
  lineSenFront[3] = ((PINH & (1 << PINH0))  > 0 ? 1 : 0) ; // f3
  lineSenFront[4] = ((PINL & (1 << PINL1))  > 0 ? 1 : 0) ; // f4
  lineSenFront[5] = ((PINL & (1 << PINL0))  > 0 ? 1 : 0) ; // f5


  /*
    Serial.println((PINJ & (1 << PINJ1)) > 0 ? 1 : 0);
    Serial.println((PINJ & (1 << PINJ0)) > 0 ? 1 : 0);
    Serial.println((PINH & (1 << PINH1)) > 0 ? 1 : 0);
    Serial.println((PINH & (1 << PINH0)) > 0 ? 1 : 0);
    Serial.println((PINL & (1 << PINL1)) > 0 ? 1 : 0);
    Serial.println((PINL & (1 << PINL0)) > 0 ? 1 : 0);
  */

  for (size_t i = 0; i < 6; i++)
  {
    //Serial.print("f");
    //Serial.print(i);
    //Serial.print("--");
    Serial.print(lineSenFront[i]);
    Serial.print("--");
  }

  lineSenLatDer[0] = (PINA & (1 << PINA0))  > 0 ? 1 : 0 ;
  lineSenLatDer[1] = (PINA & (1 << PINA1))  > 0 ? 1 : 0 ;

  /*
    for (size_t i = 0; i < 2; i++)
    {
      Serial.print("f");
      Serial.print(i);
      Serial.print("--");
      Serial.print(lineSenLatDer[i]);
    }
  */
  lineSenBack[0] = (PINA & (1 << PINA2))  > 0 ? 1 : 0 ;
  lineSenBack[1] = (PINA & (1 << PINA3))  > 0 ? 1 : 0 ;
  lineSenBack[2] = (PINA & (1 << PINA4))  > 0 ? 1 : 0 ;
  lineSenBack[3] = (PINA & (1 << PINA5))  > 0 ? 1 : 0 ;
  lineSenBack[4] = (PINA & (1 << PINA6))  > 0 ? 1 : 0 ;
  lineSenBack[5] = (PINA & (1 << PINA7))  > 0 ? 1 : 0 ;

  for (size_t i = 0; i < 6; i++)
  {
    //Serial.print("f");
    //Serial.print(i);
    //Serial.print("--");
    Serial.print(lineSenBack[i]);
    Serial.print("--");
  }

  lineSenLatIzq[0] = (PINL & (1 << PINL3))  > 0 ? 1 : 0 ;
  lineSenLatIzq[1] = (PINL & (1 << PINL2))  > 0 ? 1 : 0 ;

  for (size_t i = 0; i < 2; i++)
  {
    //Serial.print("f");
    //Serial.print(i);
    Serial.print(lineSenLatIzq[i]);
    Serial.print("--");
  }
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
