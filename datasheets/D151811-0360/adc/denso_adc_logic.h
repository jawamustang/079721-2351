/*
 * denso_adc_logic.h
 *
 *  Created on: Dec 9, 2012
 *      Author: Andrey Belomutskiy
 */

#ifndef DENSO_ADC_LOGIC_H_
#define DENSO_ADC_LOGIC_H_

//typedef enum {

#define ADC_COMMAND_STATE_IDLE 0
#define ADC_COMMAND_STATE_NEED_FIRST_BIT 1
#define ADC_COMMAND_STATE_NEED_SECOND_BIT 2
#define ADC_COMMAND_STATE_NEED_SENDING_COMMAND 3

typedef enum {
	LOW = 0, HIGH = 1
} AdcLineState;

typedef enum {
	IDLE = 0, READING = 1, FINISHED = 2
} AdcReadState;

typedef struct {
	AdcReadState readState;
	AdcLineState lineState;
	int bitLength;
	int readStartTime;
	int currentBit;
	int value;
} AdcReader;


void initAdcDriver();
void processInputSignal(int length);
int readAdcValue();

int requestAdcReading(int adcChannelIndex);
int nextAdcControlBit();

#ifndef TRUE
 #define TRUE 1
#endif

#ifndef FALSE
 #define FALSE 0
#endif

#endif /* DENSO_ADC_LOGIC_H_ */
