/*
 * denso_adc_logic.c
 *
 *  Created on: Dec 9, 2012
 *      Author: Andrey Belomutskiy
 */

#include "main.h" // optional
#include "denso_adc_logic.h"
#include "denso_adc_io.h"
#include <misc.h>
#include "my_lib.h" // this is needed for 'GetCurrentTime()' definition

#ifdef HAS_CONSOLE
#include "rficonsole.h"
#endif

AdcReader reader;

volatile int pendingAdcControlValue = 0;
volatile int adc_control_state = ADC_COMMAND_STATE_IDLE;

void resetAdcInputState(AdcReader *reader) {
	reader->readState = IDLE;
	reader->readStartTime = 0;
}

int readAdcValueR(AdcReader *reader) {
	if (reader->readState == IDLE)
		return -100;
	int totalTime = GetCurrentTime() - reader->readStartTime;
	if (totalTime > 5 || reader->readState == FINISHED)
		return reader->value;
	return -101;
}

int readAdcValue() {
	int value = readAdcValueR(&reader);
	if (value < 0)
		return value; // error code
	unsigned char withoutLeadingBit = value / 2;

	return 0xff ^ withoutLeadingBit; // invert bits
}

void processSignal(AdcReader *reader, int length) {
	int count = (length + reader->bitLength / 2) / reader->bitLength;

	if (reader->readState == FINISHED)
		return;

	if (reader->readState == IDLE) {
		reader->lineState = HIGH;
		reader->readState = READING;
		reader->currentBit = 0;
		reader->value = 0;

		reader->readStartTime = GetCurrentTime();
	} else {
		int totalTime = GetCurrentTime() - reader->readStartTime;

		if (totalTime > 5 || reader->currentBit > 9) {
			reader->readState = FINISHED;
			return;
		}

		/**
		 * idle line state is HIGH, meaning we would not get a level change if the highest bits are high
		 * considering this it's easiest to consider low line state as 1 and hi
		 */
		if (reader->lineState == HIGH) {
			reader->lineState = LOW;
			for (int i = 0; i < count; i++) {
				reader->value += (1 << reader->currentBit);
				reader->currentBit++;
			}
		} else {
			reader->lineState = HIGH;
			reader->currentBit += count;
		}
	}
}

void processInputSignal(int length) {
	processSignal(&reader, length);
}

int requestAdcReading(int adcChannelIndex) {
	if (adc_control_state != ADC_COMMAND_STATE_IDLE)
		return FALSE; // current command not finished yet
	resetAdcInputState(&reader);
	// this value is assigned before we update state
	pendingAdcControlValue = adcChannelIndex;
	// now we assign state - pending value was already assigned, this sequence is
	// important for multi-threading with timer IRQ handler
	adc_control_state = ADC_COMMAND_STATE_NEED_FIRST_BIT;
	return TRUE;
}

int nextAdcControlBit() {
	if (adc_control_state == ADC_COMMAND_STATE_IDLE)
		return 0;
	if (adc_control_state == ADC_COMMAND_STATE_NEED_FIRST_BIT) {
		adc_control_state++;
		return 1;
	}
	if (adc_control_state == ADC_COMMAND_STATE_NEED_SECOND_BIT) {
		adc_control_state++;
		return 0;
	}
	// at this point state has to be ADC_COMMAND_STATE_NEED_SENDING_COMMAND
	int lowestBit = pendingAdcControlValue % 2;
	pendingAdcControlValue /= 2;
	if (pendingAdcControlValue == 0)
		adc_control_state = ADC_COMMAND_STATE_IDLE; // command transmission is over
	return lowestBit;
}

#ifdef HAS_CONSOLE
void printAdcValue(int channel) {
	requestAdcReading(channel);
	Delay(10); // ms
	int value = readAdcValue();
	print("adc channel %d: value %d\r\n", channel, value);
}
#endif


void initAdcDriver() {
	reader.bitLength = SystemCoreClock / 2 / DENSO_SERIAL_CLOCK_FREQUENCY;
	resetAdcInputState(&reader);

	/* is there a reason why all these GPIO clocks are not all enabled by default? */
	RCC_AHB1PeriphClockCmd(
			RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC
					| RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);

	initDataInputTimer();
	initDataOutputTimer();
	init1mhzTimer();

#ifdef HAS_CONSOLE
	addConsoleAction1("r", &printAdcValue);
#endif
}
