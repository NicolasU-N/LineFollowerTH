#include <math.h>
#include <stdio.h>

int i = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (i > 447) {
    i = 0;
  } else {
    i++;
  }
  Serial.println(funcPwm(i));

  delay(20);
}

int funcPwm(int &t) {
  return (int)2 * exp(0.0108 * t);
}
