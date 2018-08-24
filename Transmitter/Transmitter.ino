#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "SparrowTransfer.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Wire.h>
#include <ClosedCube_OPT3001.h>
#include <SparrowVsleep.h>

//bme 680
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme; // I2C

//cjmcu
#define OPT3001_ADDRESS 0x44
ClosedCube_OPT3001 opt3001;
OPT3001 result;
#define cjmcu_vcc A1

//soil humidity
int SoilHumidity_Pin = A0;
int Soil_Humidity;
#define soil_vcc 10

// global variables
int counter;
boolean colect_data;

//create object
SparrowTransfer ST;

struct SEND_DATA_STRUCTURE {
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER SPARROW
  uint16_t data;         // ID
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

  // for soil humidity senzor
  //  pinMode(SoilHumidity_Pin, INPUT);

  //CJMCU
  pinMode(cjmcu_vcc, OUTPUT);
  analogWrite(cjmcu_vcc, 1023);
  opt3001.begin(OPT3001_ADDRESS);
  configureSensor();

  //soil humidity
  pinMode(soil_vcc, OUTPUT);
  digitalWrite(soil_vcc, HIGH);

  //start the library, pass in the data details
  ST.begin(details(mydata));

  mydata.data = 0;
  strcpy(mydata.ana, "TEST");
}

void configureSensor() {
  OPT3001_Config newConfig;

  newConfig.RangeNumber = B1100;
  newConfig.ConvertionTime = B0;
  newConfig.Latch = B1;
  newConfig.ModeOfConversionOperation = B11;

  OPT3001_ErrorCode errorConfig = opt3001.writeConfig(newConfig);

  if (errorConfig != NO_ERROR)
    Serial.println("OPT3001 configuration");
  else {
    OPT3001_Config sensorConfig = opt3001.readConfig();
  }
}

void print_sensor_data() {
  Serial.println("- - - - - - - - - - - - - - - - -");
  
  Serial.print("ID:                 ");
  Serial.println(mydata.data);
  
  // for bme680
  Serial.print("Temperature:        ");
  Serial.print(mydata.temperature);
  Serial.println(" °C");

  Serial.print("Pressure:           ");
  Serial.print(mydata.pressure);
  Serial.println(" hPa");

  Serial.print("Air humidity:       ");
  Serial.print(mydata.air_humidity);
  Serial.println(" %");

  Serial.print("Gas:                ");
  Serial.print(mydata.gas);
  Serial.println(" KOhms");

  Serial.print("Altitude:           ");
  Serial.print(mydata.altitude);
  Serial.println(" m");

  // for cjmcu luminosity
  Serial.print("Light:              ");
  Serial.print(mydata.light);
  Serial.println(" lux");

  // for soil humidity
  Serial.print("Soil Humidity:      ");
  Serial.print(mydata.soil_humidity);
  Serial.print(" % ");
  Serial.print("   (");
  Serial.print(Soil_Humidity);
  Serial.println(")");

  Serial.println("- - - - - - - - - - - - - - - - -");
}

void colect_sensors_data() {
  //reenable ADC
  ADCSRA = 151;

  // colect bme680 data
  setup_bme680();
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  // collect cjmcu data
  result = opt3001.readResult();

  //collect soil humidity senzor
  Soil_Humidity = analogRead(SoilHumidity_Pin);
}

void update_struct() {
  mydata.data++;
  mydata.temperature = bme.temperature;
  mydata.pressure = bme.pressure / 100.0;
  mydata.air_humidity = bme.humidity;
  mydata.gas = bme.gas_resistance / 1000.0;
  Serial.print("gas:");
  Serial.println(bme.gas_resistance);
  mydata.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  mydata.light = result.lux;
  mydata.soil_humidity =  map(Soil_Humidity, 1023, 200, 0, 100);
}

void power_on_sensors() {
  analogWrite(cjmcu_vcc, 1023);
  digitalWrite(soil_vcc, HIGH);
}


void power_off_sensors() {
  analogWrite(cjmcu_vcc, 0);
  digitalWrite(soil_vcc, LOW);
}

void loop() {

  Serial.println("************************************************");
  Serial.println("start");
  Serial.flush();

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

  // update structure with new data from sensors
  update_struct();

  // print sensors data
  print_sensor_data();

  Serial.println("sending data...");
  Serial.flush();

  //send the data
  ST.sendData();
  _delay_ms(1000);

  colect_data = false;

  Serial.println("done! data sent");
  Serial.flush();

  // power off sensors
  power_off_sensors();

  //sleep
  Serial.println("going to sleep");
  Serial.flush();
  SparrowV_SleepInit(10, true);

  Serial.println("finish");
  Serial.println("************************************************");
  Serial.println();
}
