#include "SparrowTransfer.h"
#include <util/delay.h>

//create object
SparrowTransfer ST;

struct RECEIVE_DATA_STRUCTURE {
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  uint16_t data;
  char ana[3];

};
//give a name to the group of data
RECEIVE_DATA_STRUCTURE mydata;

uint16_t old_index, received_index, lost;

void setup() {
  Serial.begin(9600);

  //start the library, pass in the data details
  ST.begin(details(mydata));

  pinMode(11, OUTPUT);
  digitalWrite(11, HIGH);

}

void blinkLED()
{
  digitalWrite(11, LOW);
  delay(20);
  digitalWrite(11, HIGH);
}

void loop() {
  //check and see if a data packet has come in.
  if (ST.receiveData()) {

    blinkLED();

    received_index++;

    if (old_index != 0)
      lost += mydata.data - old_index - 1;

    Serial.print("Frame arrived: ");
    Serial.print(mydata.data);
    Serial.print(" ");
    Serial.print(mydata.ana);
    Serial.print(" ");
    Serial.print(", lost: ");
    Serial.print(lost);
    Serial.print(", loss: ");
    Serial.print(lost * 100.0 / (lost + received_index), 3);
    Serial.println("%");

    old_index = mydata.data;
  }

  //you should make this delay shorter than your transmit delay or else messages could be lost
  delay(250);
//  _delay_ms(250);
}
