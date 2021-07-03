#define s0 9
#define s1 10
#define s2 11
#define s3 12

void setup() {
  // put your setup code here, to run once:
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  for (int i = 0; i < 16; i++)
  {
    digitalWrite(s0,i&0x01);
    digitalWrite(s1,i&0x02);
    digitalWrite(s2,i&0x04);
    digitalWrite(s3,i&0x08);
    Serial.print("-");
    Serial.print(analogRead(A0));
    Serial.print("--");
  }
  Serial.println("");
  delay(500);
}
