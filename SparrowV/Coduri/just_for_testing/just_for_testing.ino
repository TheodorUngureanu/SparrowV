#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
//#include <ZigduinoSleep.h>

int counter = 0;
int state = 0;
uint32_t symbol_threshold = 0x00000000;

ISR(SCNT_CMP1_vect)
{

}

void ZigduinoSleepInit()
{
  sei();

  // transcieverul off
  //  PRR1 |= (1 << PRTRX24);

  ASSR = 1 << AS2; // enable asynchronous mode, with external oscillator (32.768kHz in our case)

  //  Shut down ADC before invoking the PRR bit for it
  ADCSRA = 0;

  // Power reduction registers
  //  PRR0 |= (1 << PRTWI) | (1 << PRTIM2) | (1 << PRTIM0) | (1 << PRPGA) |
  //          (1 << PRTIM1) | (1 << PRSPI) | (1 << PRUSART0) | (1 << PRADC);

  PRR1 |= (1 << PRTIM5) | (1 << PRTIM4) | (1 << PRTIM3) | (1 << PRUSART1);

  //  PRR2 |= (1 << PRRAM3) | (1 << PRRAM2) | (1 << PRRAM1) | (1 << PRRAM0);

  SCOCR1HH = (symbol_threshold >> 24);
  SCOCR1HL = (symbol_threshold & 0x00ff0000) >> 16;
  SCOCR1LH = (symbol_threshold & 0x0000ff00) >>  8;
  SCOCR1LL = (symbol_threshold & 0x000000ff);
  SCCR0 = _BV(SCEN) ;//| _BV(SCCKSEL);  // enable symbol counter, with TOSC1/2 clock (32.768kHz)
  SCCNTHH = 0x00;
  SCCNTHL = 0x00;
  SCCNTLH = 0x00;
  SCCNTLL = 0x00;

  while (SCSR & _BV(SCBSY));
  SCIRQM = _BV(IRQMCP1);      // enable compare match 1 IRQ*/
}

void ZigduinoSleepSet(uint8_t seconds)
{
  symbol_threshold += (((uint32_t)seconds) * 62500);

  SCOCR1HH = (symbol_threshold >> 24);
  SCOCR1HL = (symbol_threshold & 0x00ff0000) >> 16;
  SCOCR1LH = (symbol_threshold & 0x0000ff00) >>  8;
  SCOCR1LL = (symbol_threshold & 0x000000ff);

  while (SCSR & _BV(SCBSY));
}

void ZigduinoSleep()
{
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  sleep_enable();
  sleep_cpu();
  sleep_disable();
}


void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  ZigduinoSleepInit();
}

void loop() {

  Serial.println("am inceput");
  //  digitalWrite(LED_BUILTIN, HIGH);
  //  delay(5000);
  //  digitalWrite(LED_BUILTIN, LOW);

  ZigduinoSleepSet(3);

  state ^= 1;
  counter++;
  if (state == 0) digitalWrite(LED_BUILTIN, HIGH);
  else digitalWrite(LED_BUILTIN, LOW);
  // prepare sleep
  //  setup_sleep();

  Serial.begin(9600);
  Serial.println("Test");
  Serial.println(counter);
  Serial.flush();
  Serial.end();

  ZigduinoSleep();

  Serial.println("am terminat");
}


