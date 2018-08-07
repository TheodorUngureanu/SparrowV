// backup just for sleeping
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

int counter;
boolean colect_data;

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
  /* prepare Deep-sleep Mode */
  //  Serial.println("setup sleep");
  //  Serial.flush();

  // transcieverul off
  PRR1 |= (1 << PRTRX24);

  //  Shut down ADC before invoking the PRR bit for it
  ADCSRA = 0;

  // Power reduction registers (usart0 TODO)
  PRR0 |= (1 << PRTWI) | (1 << PRTIM2) | (1 << PRTIM0) | (1 << PRPGA) |
          (1 << PRTIM1) | (1 << PRSPI) | (1 << PRADC) | (1 << PRUSART0);

  PRR1 |= (1 << PRTIM5) | (1 << PRTIM4) | (1 << PRTIM3) | (1 << PRUSART1);   //(usart1 TODO)
  //  | (1 << PRUSART1);

  //  PRR2 |= (1 << PRRAM3) | (1 << PRRAM2) | (1 << PRRAM1) | (1 << PRRAM0);

  // Sleep Mode Control Register (sleep enable & power down)
  SMCR = (1 << SE) | (0 << SM0) | (1 << SM1) | (0 << SM2);
  sleep_cpu();
}

ISR(WDT_vect) {
  counter++;
  //  Serial.print("Watchdog Interrupt ");
  //  Serial.println(counter, DEC);
  //  Serial.flush();

  counter++;
  if (counter == 10) {
    counter = 0;
    colect_data = true;
  }
  // ReEnable the watchdog interrupt, as this gets reset when entering this ISR and
  // automatically enables the WDE signal that resets the MCU the next time the
  // timer overflows
  //  WDTCR |= (1<<WDIE);
}

void setup() {
  counter = 0;
  colect_data = false;
  //  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  //  setup_WDT();
}

void loop() {

  //  Serial.println("am inceput");

  digitalWrite(LED_BUILTIN, HIGH);
  _delay_ms(1000);
  if (colect_data == true) {
    //    Serial.println("acum colectam date si le trimitem");
    colect_data = false;
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);

    // prepare watchdog
    setup_WDT();

    // prepare sleep
    setup_sleep();
  }

  //  Serial.println("am terminat");
}


