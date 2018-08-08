#include <SparrowVsleep.h>

int ok = 0;
int counter = 0;

void setup() {
  counter++;
  Serial.begin(9600);Serial. println();

  Serial.print("counter: ");
  Serial.println(counter);
  Serial.print("OK: ");
  Serial.println(ok);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial. println("****************************");
  Serial.println("Start");
  Serial.flush();

  if (ok == 0) {
    SparrowV_SleepInit(35, true);
    ok = 1;
  }
  setup();
  Serial.println("Stop");
  Serial. println("****************************");
  Serial. println();
  Serial.flush();
  delay(5000);
}
