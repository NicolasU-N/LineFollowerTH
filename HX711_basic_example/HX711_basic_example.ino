#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 10;
const int LOADCELL_SCK_PIN = 9;

HX711 scale;

void setup() {
  Serial.begin(115200);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  Serial.print("Lectura del valor del ADC: ");
  Serial.println(scale.read());
  Serial.println("No ponga ningún objeto sobre la balanza");
  Serial.println("Destarando...");
  scale.set_gain(128);
  scale.set_scale(); //La escala por defecto es 1
  scale.tare(20);  //El peso actual es considerado Tara.
  Serial.println("Coloque un peso conocido:");

}

void loop() {
  Serial.print();

  if (scale.is_ready()) {
    Serial.print("Valor de lectura: ");
    Serial.println(scale.get_value(10), 0);
    delay(100);
  } else {
    Serial.println("HX711 not found.");
  }

  delay(500);

}
