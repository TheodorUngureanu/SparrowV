#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "SparrowTransfer.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Wire.h>
#include <SparrowVsleep.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define BME680_VCC 8
//#define BME680_GND 9

Adafruit_BME680 bme; // I2C

// global variables
int counter;
boolean colect_data;

//create object
SparrowTransfer ST;

struct SEND_DATA_STRUCTURE {
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER SPARROW
  uint16_t data;  // ID
  char ana[4];
  int temperature;       // temperature
  int pressure;          // air pressure
  int air_humidity;      // air humidity
  int gas;               // gas in air (air cuality)
  int altitude;          // altitude
};

//give a name to the group of data
SEND_DATA_STRUCTURE mydata;

void setup_bme680() {
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void setup() {
  counter = 0;
  colect_data = false;
  Serial.begin(9600);

  //  pinMode(LED_BUILTIN, OUTPUT);
  //  digitalWrite(LED_BUILTIN, LOW);

  // for bme690
  pinMode(BME680_VCC, OUTPUT);
  //  pinMode(BME680_GND, OUTPUT);
  //  digitalWrite(BME680_GND, LOW);

  //start the library, pass in the data details
  ST.begin(details(mydata));

  mydata.data = 0;
  strcpy(mydata.ana, "TEST");

}

void power_on_sensors() {
  digitalWrite(BME680_VCC, HIGH);
}


void power_off_sensors() {
  digitalWrite(BME680_VCC, LOW);

  // incercare
  TWCR &= ~(1 << TWEN);
  PRR0 &= ~(1 << PRTWI);
  DDRD &= ~(1 << PD0);
  DDRD &= ~(1 << PD1);

  //  pinMode(SDA, OUTPUT);
  //  digitalWrite(SDA, 0);
  //  pinMode(SCL, OUTPUT);
  //  digitalWrite(SCL, 0);


  //  DDRD |= (1 << PD0);
  //  DDRD |= (1 << PD1);
  //  digitalWrite(SDA, 1);
  //  digitalWrite(SCL, 1);
  //    pinMode(SDA, INPUT);  // remove internal pullup
  //    pinMode(SCL, INPUT);  // remove internal pullup
  //  DDRD &= ~(1 << PD0);
  //  DDRD &= ~(1 << PD1);
}

void print_sensor_data() {

  // for bme680
  Serial.println("- - - - - - - - - - - - - - - - -");
  Serial.print("Temperature:    ");
  Serial.print(bme.temperature);
  Serial.println(" °C");

  Serial.print("Pressure:       ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Air humidity:   ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas:            ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.print("Altitude:       ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println("- - - - - - - - - - - - - - - - -");
  //    Serial.println();
}

void colect_sensors_data() {
  // colect bme680 data
  setup_bme680();
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
}

void update_struct() {
  mydata.data++;
  mydata.temperature = bme.temperature;
  mydata.pressure = bme.pressure / 100.0;
  mydata.air_humidity = bme.humidity;
  mydata.gas = bme.gas_resistance / 1000.0;
  mydata.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
}

void loop() {

  Serial.println("************************************************");
  Serial.println("start");
  Serial.flush();

  //  digitalWrite(LED_BUILTIN, HIGH);

  // power on sensors
  power_on_sensors();

  // transciever initialization
  Serial.println("transciever initialization");
  Serial.flush();
  ST.begin(details(mydata));

  Serial.println("colect data from sensors");
  Serial.flush();
  _delay_ms(1);

  //colectam date
  colect_sensors_data();

  // print sensors data
  print_sensor_data();

  // update structure with new data from sensors
  update_struct();

  Serial.println("sending data...");
  Serial.flush();

  //send the data
  ST.sendData();
  _delay_ms(1000);

  colect_data = false;

  Serial.println("done! data sent");
  Serial.flush();

  //  digitalWrite(LED_BUILTIN, LOW);

  // power off sensors
  power_off_sensors();

  //sleep
  Serial.println("going to sleep");
  Serial.flush();
  SparrowV_SleepInit(70, true);

  Serial.println("finish");
  Serial.println("************************************************");
  Serial.println();
}
