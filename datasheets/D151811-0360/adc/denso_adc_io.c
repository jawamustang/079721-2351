/*
 * denso_adc_io.c
 *
 *  Created on: Dec 9, 2012
 *      Author: Andrey Belomutskiy
 *
 * STM32F4 Denso D151811-0360 ADC serial interface driver
 * while STM32 has build-in USART mechanism, it's clock only operates while transmission - so, a PWM timer is needed
 * at least to generate constant 62.5Khz clock on ADC pin 9. Same PWM is used to send commands to pin ADC pin 7.
 * I am not sure if same PWM could be used for both output and input, so for now separate PWM is used to read ADC responses
 *
 * another PWM is needed to generate 1MHz clock for ADC pin 10.
 *
 * TODO: how about SPI? how about I2S?
 */

#include <stm32f4xx_rcc.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_gpio.h>
#include <misc.h>

#include "denso_adc_io.h"
#include "denso_adc_logic.h"

void setupPwmOutputChannel(TIM_TypeDef* TIMx, int channel, int width,
		int polarity) {
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = width;
	TIM_OCInitStructure.TIM_OCPolarity = polarity;

	if (channel == 1) {
		TIM_OC1Init(TIMx, &TIM_OCInitStructure);
	} else if (channel == 2) {
		TIM_OC2Init(TIMx, &TIM_OCInitStructure);
	} else if (channel == 3) {
		TIM_OC3Init(TIMx, &TIM_OCInitStructure);
	} else if (channel == 4) {
		TIM_OC4Init(TIMx, &TIM_OCInitStructure);
	}
}

void configeInputPin(GPIO_TypeDef * PORT, int pin, int af) {
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = 1 << pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(PORT, pin, af);
}

void configeOutputPin(GPIO_TypeDef * PORT, int pin, int af) {
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = 1 << pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(PORT, pin, af);
}

#ifdef DENSO_1MHZ_CLOCK
void init1mhzTimer() {
	RCC_AHB1PeriphClockCmd(DENSO_PRIMARY_CLOCK_IO_PORT, ENABLE);
	RCC_APB1PeriphClockCmd(DENSO_PRIMARY_CLOCK_CLK, ENABLE);

	TIM_TimeBaseInitTypeDef base_timer;
	TIM_TimeBaseStructInit(&base_timer);

	base_timer.TIM_Prescaler = (SystemCoreClock / 2
			/ (2 * DENSO_PRIMARY_CLOCK_FREQUENCY)) - 1;
	base_timer.TIM_ClockDivision = 0;
	base_timer.TIM_Period = 2 - 1;
	TIM_TimeBaseInit(DENSO_PRIMARY_CLOCK_TIMER, &base_timer);

	// TIM2_CH2 - PB3
	// TIM4_CH2 - PD13

	TIM_OC2PreloadConfig(DENSO_PRIMARY_CLOCK_TIMER, TIM_OCPreload_Enable );
	setupPwmOutputChannel(DENSO_PRIMARY_CLOCK_TIMER, 2, 1,
			TIM_OCPolarity_High );

	TIM_ARRPreloadConfig(DENSO_PRIMARY_CLOCK_TIMER, ENABLE);

	configeOutputPin(DENSO_PRIMARY_CLOCK_IO_PORT, DENSO_PRIMARY_CLOCK_PIN,
			DENSO_PRIMARY_CLOCK_MODE_ALT); // TIM4_CH2

	TIM_Cmd(DENSO_PRIMARY_CLOCK_TIMER, ENABLE);
}
#endif

void configureInputSignal(TIM_TypeDef *TIMx, int channelCode) {
	TIM_ICInitTypeDef TIM_ICInitStructure;
	/* TIMx configuration: PWM Input mode ------------------------
	 The external signal is connected to TIMx CHx pin,
	 The Rising edge is used as active edge,
	 The TIMx CCR2 is used to compute the frequency value
	 The TIMx CCR1 is used to compute the duty cycle value
	 ------------------------------------------------------------ */

	TIM_ICInitStructure.TIM_Channel = channelCode;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

	TIM_PWMIConfig(TIMx, &TIM_ICInitStructure);
}

void initDataInputTimer() {
	/* TIMx clock enable */
	RCC_APB1PeriphClockCmd(DENSO_SERIAL_INPUT_CLK, ENABLE);

	configeInputPin(DENSO_SERIAL_INPUT_IO_PORT, DENSO_SERIAL_INPUT_PIN,
			DENSO_SERIAL_INPUT_MODE_ALT );

	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the TIMx global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DENSO_SERIAL_INPUT_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	configureInputSignal(DENSO_SERIAL_INPUT_TIMER, TIM_Channel_2 );

	/* Select the TIMx Input Trigger: TI2FP2 */
	TIM_SelectInputTrigger(DENSO_SERIAL_INPUT_TIMER, TIM_TS_TI2FP2 );

	/* Select the slave Mode: Reset Mode */
	TIM_SelectSlaveMode(DENSO_SERIAL_INPUT_TIMER, TIM_SlaveMode_Reset );
	TIM_SelectMasterSlaveMode(DENSO_SERIAL_INPUT_TIMER,
			TIM_MasterSlaveMode_Enable );

	/* TIMx enable counter */
	TIM_Cmd(DENSO_SERIAL_INPUT_TIMER, ENABLE);

	/* Enable the CC2 Interrupt Request */
	TIM_ITConfig(DENSO_SERIAL_INPUT_TIMER, TIM_IT_CC2, ENABLE);
}

void initDataOutputTimer() {
	RCC_APB1PeriphClockCmd(DENSO_SERIAL_OUTPUT_CLK, ENABLE);
	TIM_TimeBaseInitTypeDef base_timer;
	TIM_TimeBaseStructInit(&base_timer);

	// be sure your system_stm32 file has the right timing values...
	// 168M / 2 / (2 * 62500)
	base_timer.TIM_Prescaler = (SystemCoreClock / 2
			/ (2 * DENSO_SERIAL_CLOCK_FREQUENCY)) - 1;
	base_timer.TIM_ClockDivision = 0;
	base_timer.TIM_Period = 2 - 1;
	TIM_TimeBaseInit(DENSO_SERIAL_OUTPUT_TIMER, &base_timer);

	TIM_OC3PreloadConfig(DENSO_SERIAL_OUTPUT_TIMER, TIM_OCPreload_Enable );
	TIM_OC4PreloadConfig(DENSO_SERIAL_OUTPUT_TIMER, TIM_OCPreload_Enable );

	setupPwmOutputChannel(DENSO_SERIAL_OUTPUT_TIMER, 3, 1, TIM_OCPolarity_Low );
	setupPwmOutputChannel(DENSO_SERIAL_OUTPUT_TIMER, 4, 1, TIM_OCPolarity_Low );

	TIM_ARRPreloadConfig(DENSO_SERIAL_OUTPUT_TIMER, ENABLE);

	configeOutputPin(DENSO_SERIAL_CLOCK_IO_PORT, DENSO_SERIAL_CLOCK_PIN,
			DENSO_SERIAL_CLOCK_MODE_ALT ); // CH3 - clock
	configeOutputPin(DENSO_SERIAL_COMMAND_IO_PORT, DENSO_SERIAL_COMMAND_PIN,
			DENSO_SERIAL_COMMAND_MODE_ALT ); // CH4 - command

	TIM_ITConfig(DENSO_SERIAL_OUTPUT_TIMER, TIM_IT_Update, ENABLE);
	TIM_Cmd(DENSO_SERIAL_OUTPUT_TIMER, ENABLE);

	NVIC_EnableIRQ(DENSO_SERIAL_COMMAND_IRQ);
}

void DENSO_SERIAL_OUTPUT_IRQHANDLER() {
	if (TIM_GetITStatus(DENSO_SERIAL_OUTPUT_TIMER, TIM_IT_Update ) != RESET) {
		TIM_ClearITPendingBit(DENSO_SERIAL_OUTPUT_TIMER, TIM_IT_Update );
		int currentBit = nextAdcControlBit();
		DENSO_SERIAL_OUTPUT_TIMER ->CCR4 = currentBit ? 2 : 0;
	}
}

void DENSO_SERIAL_INPUT_IRQHANDLER() {
	if (TIM_GetITStatus(DENSO_SERIAL_INPUT_TIMER, TIM_IT_CC2 ) != RESET) {
		/* Clear TIM4 Capture compare interrupt pending bit */
		TIM_ClearITPendingBit(DENSO_SERIAL_INPUT_TIMER, TIM_IT_CC2 );
		/* Get the Input Capture value */
		int IC2Value = TIM_GetCapture2(TIM4 );

		processInputSignal(IC2Value);
	}
}
