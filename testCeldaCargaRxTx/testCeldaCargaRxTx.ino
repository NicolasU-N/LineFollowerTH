String str;

float pesoCelda;

void setup()
{
  Serial2.begin(9600);
  Serial.begin(9600);
}

void loop()
{
  if (Serial2.available())
  {

    str = Serial2.readStringUntil('\n');

    String igual = str.substring(0, 1); // Ignorar el igual


    if (igual.equalsIgnoreCase("="))
    {
      pesoCelda = str.substring(1).toFloat();
    }


    Serial.print("Valor String: ");
    Serial.println(pesoCelda );

    //Serial.print(" ");
    //Serial.println(pesoCelda);
  }

}
