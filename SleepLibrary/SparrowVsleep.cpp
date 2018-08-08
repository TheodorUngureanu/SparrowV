#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <Arduino.h>
#include <util/delay.h>
#include "SparrowVsleep.h"

int global_seconds;

ISR(WDT_vect)
{
	Serial.println("Watchdog Interrupt ");
	Serial.flush();

	Serial.print("global_seconds: ");
	Serial.println(global_seconds);
	Serial.flush();

	// // reenable the transciever
	// PRR1 &= ~(1 << PRTRX24);
	//
	// // reenable ADC
	// ADCSRA = (1 << ADEN);
	//
	// //restore PRR
	// PRR0 = 0;
	// PRR1 = 0;
	//
	// wdt_disable();

	if (global_seconds <= 0)
	{
		Serial.println();
		Serial.println("Ready sleep");
		// reenable the transciever
		PRR1 &= ~(1 << PRTRX24);

		// reenable ADC
		ADCSRA = (1 << ADEN);

		//restore PRR
		PRR0 = 0;
		PRR1 = 0;

		wdt_disable();
	}
	else
	{
		// going to sleep again
		Serial.println();
		Serial.println("going to sleep again");
		choose(global_seconds);
	}
}

void choose(int seconds)
{
	Serial.println("choose");
	Serial.flush();

	if(seconds % 8 == 0) {
		Serial.println("choose 8");
		Serial.flush();
		setup_WDT(8);
	}
	else if (seconds % 4 == 0) {
		Serial.println("choose 4");
		Serial.flush();
		setup_WDT(4);
	}
	else if (seconds % 2 == 0) {
		Serial.println("choose 2");
		Serial.flush();
		setup_WDT(2);
	}
	else if (seconds % 1 == 0) {
		Serial.println("choose 1");
		Serial.flush();
		setup_WDT(1);
	}
	else {
		//TODO for less than 1 second
		return;
	}
}

void setup_WDT(int situation)
{
	Serial.println("setup WDT");
	Serial.flush();

	cli();

	wdt_reset(); // reset the WDT timer
	MCUSR &= ~(1 << WDRF); // Clear the reset flag. (because the data sheet said to)

	// Setup Watchdog for interrupt and not reset, and a approximately x seconds timeout

	switch (situation)
	{
		// wake up after 8 seconds
		case 8: // Enter in configuration mode
				WDTCSR = (1 << WDCE) | (1 << WDE);
				WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
				global_seconds -= 8;
				Serial.println("WDT 8");
				Serial.flush();
				break;

		// wake up after 4 seconds
		case 4: // Enter in configuration mode
				WDTCSR = (1 << WDCE) | (1 << WDE);
				WDTCSR = (1 << WDIE) | (1 << WDP3);
				global_seconds -= 4;
				Serial.println("WDT 4");
				Serial.flush();
				break;

		// wake up after 2 seconds
		case 2: // Enter in configuration mode
				WDTCSR = (1 << WDCE) | (1 << WDE);
				WDTCSR = (1 << WDIE) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0);
				global_seconds -= 2;
				Serial.println("WDT 2");
				Serial.flush();
				break;

		// wake up after 1 second
		case 1: // Enter in configuration mode
				WDTCSR = (1 << WDCE) | (1 << WDE);
				WDTCSR = (1 << WDIE) | (1 << WDP2) | (1 << WDP1);
				global_seconds -= 1;
				Serial.println("WDT 1");
				Serial.flush();
				break;

		//TODO for less than 1 second
	}
	sei();
	SparrowV_SleepSet(true);
}

void SparrowV_SleepSet(bool data_retention)
{
	// prepare Deep-sleep Mode
	Serial.println("sleep setup");
	Serial.flush();

	// transcieverul off
	PRR1 |= (1 << PRTRX24);

	//  Shut down ADC before invoking the PRR bit for it
	ADCSRA = 0;

	// Power reduction registers
	PRR0 |= (1 << PRTWI) | (1 << PRTIM2) | (1 << PRTIM0) | (1 << PRPGA)
	    | (1 << PRTIM1) | (1 << PRSPI) | (1 << PRADC)  /*| (1 << PRUSART0)*/;

	PRR1 |= (1 << PRTIM5) | (1 << PRTIM4) | (1 << PRTIM3) | (1 << PRUSART1)
			| (1 << PRUSART1);

	// Data renention (if no data retention max 8s sleep and then restart)
	if (data_retention == false) {
	   PRR2 |= (1 << PRRAM3) | (1 << PRRAM2) | (1 << PRRAM1) | (1 << PRRAM0);
	}

	// Sleep Mode Control Register (sleep enable & power down)
	SMCR = (1 << SE) | (0 << SM0) | (1 << SM1) | (0 << SM2);

	// comand for sleep
	sleep_cpu();
}

void SparrowV_SleepInit(int seconds, bool data_retention)
{
	// 8 < seconds
	global_seconds = seconds;
	Serial.println("global_seconds: ");
	Serial.println(global_seconds);
	Serial.flush();

	choose(seconds);
	// setup_WDT(8);

	//TODO
	// 4 < second < 8
	// 2 < seconds < 4
	// 1 < secons < 2

	// SparrowV_SleepSet(data_retention);
}
