/* This is an example for SparrowVSleep library.
    You only need to call SparrowV_SleepInit function.
    The funciton has 2 params.
    1st param: integer > 1 second (represent time in secons, how long board will sleep)
    2nd param: bool (true represent with data retention, false with no data retention)
               In case of no data retention the board can sleep max 8 secons and it will
    reboot after time will end.
*/

#include <SparrowVsleep.h>

void setup() {
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial. println("****************************");
  Serial.println("Start");
  Serial.flush();

  //sleep for 25 seconds with data retention
  SparrowV_SleepInit(25, true);

  Serial.println("Stop");
  Serial. println("****************************");
  Serial. println();
  Serial.flush();
  delay(5000);
}
