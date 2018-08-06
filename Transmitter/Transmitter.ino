#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "SparrowTransfer.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Wire.h>

#define SEALEVELPRESSURE_HPA (1013.25)

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

void setup_WDT() {
  Serial.println("setup WDT");
  Serial.flush();
  cli();

  wdt_reset(); // reset the WDT timer
  MCUSR &= ~(1 << WDRF); // Clear the reset flag. (because the data sheet said to)

  // Enter in configuration mode
  WDTCSR = (1 << WDCE) | (1 << WDE);

  // Setup Watchdog for interrupt and not reset, and a approximately 8s timeout
  WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);

  sei();
}

void setup_sleep() {
  // prepare Deep-sleep Mode
  Serial.println("setup sleep");
  Serial.flush();

  // transcieverul off
  PRR1 |= (1 << PRTRX24);

  //  Shut down ADC before invoking the PRR bit for it
  ADCSRA = 0;

  // Power reduction registers (usart0 TODO)
  PRR0 |= (1 << PRTWI) | (1 << PRTIM2) | (1 << PRTIM0) | (1 << PRPGA)
          | (1 << PRTIM1) | (1 << PRSPI) | (1 << PRADC)  | (1 << PRUSART0);

  PRR1 |= (1 << PRTIM5) | (1 << PRTIM4) | (1 << PRTIM3) | (1 << PRUSART1) | (1 << PRUSART1);

  // uncoment for no data renention
  //  PRR2 |= (1 << PRRAM3) | (1 << PRRAM2) | (1 << PRRAM1) | (1 << PRRAM0);

  // Sleep Mode Control Register (sleep enable & power down)
  SMCR = (1 << SE) | (0 << SM0) | (1 << SM1) | (0 << SM2);
  sleep_cpu();
}

ISR(WDT_vect) {
  Serial.print("Watchdog Interrupt ");
  Serial.println(counter, DEC);
  Serial.flush();

  wdt_disable();

  counter++;
  if (counter == 1) {
    counter = 0;
    colect_data = true;

    // reenable the transciever
    PRR1 &= ~(1 << PRTRX24);

    // reenable ADC
    ADCSRA = (1 << ADEN);

    //restor PRR
    PRR0 = 0;
    PRR1 = 0;

  }
}

void setup() {
  counter = 0;
  colect_data = false;
  Serial.begin(9600);

  //  pinMode(LED_BUILTIN, OUTPUT);
  //  digitalWrite(LED_BUILTIN, LOW);

  //start the library, pass in the data details
  ST.begin(details(mydata));

  mydata.data = 0;
  strcpy(mydata.ana, "TEST");

}

void loop() {

  Serial.println("****************");
  Serial.println("am inceput");

  //  digitalWrite(LED_BUILTIN, HIGH);

  if (colect_data == true) {

    Serial.println("initializare transciever");
    ST.begin(details(mydata));

    Serial.println("acum colectam date si le trimitem");
    Serial.flush();
    _delay_ms(1);

    //colectam date
    setup_bme680();
    if (! bme.performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
    }
    Serial.println("***");
    Serial.print("Temperature = ");
    Serial.print(bme.temperature);
    Serial.println(" °C");

    Serial.print("Pressure = ");
    Serial.print(bme.pressure / 100.0);
    Serial.println(" hPa");

    Serial.print("Humidity = ");
    Serial.print(bme.humidity);
    Serial.println(" %");

    Serial.print("Gas = ");
    Serial.print(bme.gas_resistance / 1000.0);
    Serial.println(" KOhms");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.println("***");
    //    Serial.println();

    mydata.data++;
    mydata.temperature = bme.temperature;
    mydata.pressure = bme.pressure / 100.0;
    mydata.air_humidity = bme.humidity;
    mydata.gas = bme.gas_resistance / 1000.0;
    mydata.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

    Serial.println("incerc");
    Serial.flush();

    //send the data
    ST.sendData();
    _delay_ms(1000);

    colect_data = false;

    Serial.println("am trimis datele");
    Serial.flush();
  }

  digitalWrite(LED_BUILTIN, LOW);

  // prepare watchdog
  setup_WDT();

  // prepare sleep
  setup_sleep();


  Serial.println("am terminat");
  Serial.println("****************");
  Serial.println();
}
