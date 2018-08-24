#include "SparrowTransfer.h"
#include <util/delay.h>

//create object
SparrowTransfer ST;

struct RECEIVE_DATA_STRUCTURE {
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  uint16_t data;
  char ana[4];
  int temperature;       // temperature
  int pressure;          // air pressure
  int air_humidity;      // air humidity
  int gas;               // gas in air (air cuality)
  int altitude;          // altitude
  int soil_humidity;     // soil humidity
  int light;             // light intensity
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

    Serial.println("*******************");
    Serial.print("Frame arrived:  ");
    Serial.println(mydata.data);

    Serial.print("String test:    ");
    Serial.println(mydata.ana);
    Serial.print("Temperature:    ");
    Serial.print(mydata.temperature);
    Serial.println(" °C");

    Serial.print("Pressure:       ");
    Serial.print(mydata.pressure);
    Serial.println(" hPa");

    Serial.print("Air humidity:   ");
    Serial.print(mydata.air_humidity);
    Serial.println(" %");

    Serial.print("Gas:            ");
    Serial.print(mydata.gas);
    Serial.println(" KOhms");

    Serial.print("Altitude:       ");
    Serial.print(mydata.altitude);
    Serial.println(" m");

    // for cjmcu luminosity
    Serial.print("Light:          ");
    Serial.print(mydata.light);
    Serial.println(" lux");

    // for soil humidity
    Serial.print("Soil Humidity:      ");
    Serial.print(mydata.soil_humidity);
    Serial.println(" %");

    Serial.print("lost:           ");
    Serial.println(lost);
    Serial.print("loss:           ");
    Serial.print(lost * 100.0 / (lost + received_index), 3);
    Serial.println("%");
    Serial.println("*******************");
    Serial.println();

    old_index = mydata.data;
  }

  //you should make this delay shorter than your transmit delay or else messages could be lost
  delay(250);
  //  _delay_ms(250);
}
