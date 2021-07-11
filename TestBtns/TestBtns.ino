#define BTNFRONTF 18 // PORTD3 Input
#define BTNSTOP 19 // PORTD2 Input
#define BTNFRONTB 2 // PORTD2 Input

int stateBtn1;
int stateBtn2;
int stateBtn3;

void setup() {

  pinMode(BTNFRONTF, INPUT);
  pinMode(BTNSTOP, INPUT);
  pinMode(BTNFRONTB, INPUT);

  Serial.begin(115200);
}

void loop() {

  stateBtn1 = digitalRead(BTNFRONTF);
  stateBtn2 = digitalRead(BTNSTOP);
  stateBtn3 = digitalRead(BTNFRONTB);
  Serial.print("BTN BACKF: ");
  Serial.print(stateBtn1 );
  Serial.print(" BTN STOP: ");
  Serial.print(stateBtn2);
  Serial.print(" BTN BACKB: ");
  Serial.println(stateBtn3);
  delay(500);
}
