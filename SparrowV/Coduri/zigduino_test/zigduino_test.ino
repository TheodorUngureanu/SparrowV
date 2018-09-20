#include <ZigduinoSleep.h>
 
int led = LED_BUILTIN;
int state = 0;
int counter = 0;

void setup() {                
 
  pinMode(led, OUTPUT); 
 
  Serial.begin(9600);
  Serial.println("Sleep test");
 
  ZigduinoSleepInit();
}
 
void loop() {
   ZigduinoSleepSet(30);
 
   //Code that is executed before sleep
   state ^= 1;
   counter++;
   if(state == 0) digitalWrite(led, HIGH);   
     else digitalWrite(led, LOW);
 
   Serial.begin(9600);
   Serial.println("Test");
   Serial.println(counter);
   Serial.flush();
   Serial.end();
 
   //End of code
 
   ZigduinoSleep();
 
}
