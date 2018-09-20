#include <avr/sleep.h>
#include <avr/wdt.h>

int counter;

void setup_WDT() {
  noInterrupts();  // disable all the interrupts
  MCUSR = 0;       // reset status register (sourse of interrupt)

  /* Disable and clear all Watchdog settings. */
  WDTCSR = (1 << WDCE) | (1 << WDE);

  /* Setup Watchdog for interrupt and not reset, and a approximately 4s timeout */
  WDTCSR = (1 << WDIE) | (1 << WDP3);

  interrupts();  // re-enable interrupts
}

void setup_sleep() {
  /* prepare Deep-sleep Mode */

  // transcieverul off
  PRR1 |= (1 << PRTRX24);

  //  Shut down ADC before invoking the PRR bit for it
  ADCSRA = 0;

  // Enable SRAM Data Retention
  DRTRAM0 |= (1 <<  ENDRT);
  DRTRAM1 |= (1 <<  ENDRT);
  DRTRAM2 |= (1 <<  ENDRT);
  DRTRAM3 |= (1 <<  ENDRT);

  // Power reduction registers
  PRR0 |= (1 << PRTWI) | (1 << PRTIM2) | (1 << PRTIM0) | (1 << PRPGA) |
          (1 << PRTIM1) | (1 << PRSPI) | (1 << PRUSART0) | (1 << PRADC);

  PRR1 |= (1 << PRTIM5) | (1 << PRTIM4) | (1 << PRTIM3) | (1 << PRUSART1);

  //  PRR2 |= (1 << PRRAM3) | (1 << PRRAM2) | (1 << PRRAM1) | (1 << PRRAM0);

  // Sleep Mode Control Register (sleep enable & power down)
  SMCR = (1 << SE) | (0 << SM0) | (1 << SM1) | (0 << SM2);
  sleep_cpu();
}

ISR(WDT_vect) {
  counter++;
  //  WDTCSR = (1 << WDIE);
  Serial.println("Watchdog Interrupt");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  setup_WDT();
  setup_sleep();
}

void setup() {
  counter = 0;
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  //  setup_WDT();
}

void loop() {

  Serial.println("am inceput");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(5000);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("going to sleep");
  // prepare watchdog
  setup_WDT();

  // prepare sleep
  setup_sleep();

  Serial.println("am terminat");
}


