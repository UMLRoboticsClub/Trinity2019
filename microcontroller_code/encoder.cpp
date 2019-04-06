#include <Wire.h>

#define ENC1_A 14
#define ENC1_B 12

#define ENC2_A 13
#define ENC2_B 15

#define ENC3_A 0
#define ENC3_B 2

//32 bit counter or something
int[] counter = {0, 0, 0};


//same add
//diff subtract

void enc1A(){
  int A = digitalRead(ENC1_A);
	int B = digitalRead(ENC1_B);
	(A == B) ? counter[0]++ : counter[0]--;
}

void enc2A(){
  int A = digitalRead(ENC2_A);
	int B = digitalRead(ENC2_B);
	(A == B) ? counter[1]++ : counter[1]--;
}

void enc3A(){
  int A = digitalRead(ENC3_A);
  int B = digitalRead(ENC3_B);
  (A == B) ? counter[2]++ : counter[2]--;
}

void requestEvent(){
	Wire.write(counter, 12);
}

void setup() {
  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2A, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENC3_A), enc3A, CHANGE);
}

void loop() {
  //test
	Wire.begin(0xFF);
	Wire.onRequest(requestEvent);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
}

