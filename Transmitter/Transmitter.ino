#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "SparrowTransfer.h"

// global variables
int counter;
boolean colect_data;

//create object
SparrowTransfer ST;

struct SEND_DATA_STRUCTURE {
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  uint16_t data;  // ID
  char ana[3];
  //  int16_t temp;       // temperature
  //  int soil_humidity;
};

//give a name to the group of data
SEND_DATA_STRUCTURE mydata;

void setup_WDT() {
  //  Serial.println("setup WDT");
  //  Serial.flush();
  cli();

  wdt_reset(); // reset the WDT timer
  MCUSR &= ~(1 << WDRF); // because the data sheet said to

  // Enter in configuration mode
  WDTCSR = (1 << WDCE) | (1 << WDE);

  // Setup Watchdog for interrupt and not reset, and a approximately 4s timeout
  WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);

  sei();
}

void setup_sleep() {
  // prepare Deep-sleep Mode
  //  Serial.println("setup sleep");
  //  Serial.flush();

  // transcieverul off
  PRR1 |= (1 << PRTRX24);

  //  Shut down ADC before invoking the PRR bit for it
  ADCSRA = 0;

  // Power reduction registers (usart0 TODO)
  PRR0 |= (1 << PRTWI) | (1 << PRTIM2) | (1 << PRTIM0) | (1 << PRPGA) |
          (1 << PRTIM1) | (1 << PRSPI) | (1 << PRADC);                        // (usart0 TODO)
  //          | (1 << PRUSART0);

  PRR1 |= (1 << PRTIM5) | (1 << PRTIM4) | (1 << PRTIM3) | (1 << PRUSART1);   //(usart1 TODO)
  //  | (1 << PRUSART1);

  // uncoment for no data renention
  //  PRR2 |= (1 << PRRAM3) | (1 << PRRAM2) | (1 << PRRAM1) | (1 << PRRAM0);

  // Sleep Mode Control Register (sleep enable & power down)
  SMCR = (1 << SE) | (0 << SM0) | (1 << SM1) | (0 << SM2);
  sleep_cpu();
}

ISR(WDT_vect) {
  //  Serial.print("Watchdog Interrupt ");
  //  Serial.println(counter, DEC);
  //  Serial.flush();

  counter++;
  if (counter == 2) {
    counter = 0;
    colect_data = true;

    // reenable the transciever
    PRR1 &= ~(1 << PRTRX24);

    // reenable ADC
    ADCSRA = (1 << ADEN);

  }
}

void setup() {
  counter = 0;
  colect_data = false;
  //  Serial.begin(9600);
  //  pinMode(LED_BUILTIN, OUTPUT);
  //  digitalWrite(LED_BUILTIN, LOW);

  //start the library, pass in the data details
  ST.begin(details(mydata));

  mydata.data = 0;
  mydata.ana[0] = 'a';
  mydata.ana[1] = 'n';
  mydata.ana[2] = 'a';
}

void loop() {

  //  Serial.println("am inceput");

  //  digitalWrite(LED_BUILTIN, HIGH);

  if (colect_data == true) {

    //    Serial.println("initializare transciever");
    ST.begin(details(mydata));

    //    Serial.println("acum colectam date si le trimitem");
    //    Serial.flush();
    _delay_ms(500);

    mydata.data++;

    //    Serial.println("incerc");
    //    Serial.flush();
    //send the data
    ST.sendData();
    _delay_ms(1000);

    colect_data = false;

    //    Serial.println("am trimis datele");
    //    Serial.flush();
  }

  //  digitalWrite(LED_BUILTIN, LOW);

  // prepare watchdog
  setup_WDT();

  // prepare sleep
  setup_sleep();


  //  Serial.println("am terminat");
  //  Serial.println();
}
