/*
 * denso_adc_h.h
 *
 *  Created on: Nov 22, 2012
 *      Author: Andrey Belomutskiy
 *
 * this header contains timers/pins configuration for STM32F4 Denso D151811-0360 ADC serial interface driver
 * please note: channel assignment within a timer is hardcoded and could not be adjusted by changing this file only
 *
 */

#ifndef ADC_DENSO_H_
#define ADC_DENSO_H_

#include "main.h" // optional
#ifdef CHIBIOS
 #include "ch.h"
#endif

/**
 * comment this if you do not want the primary clock feature (for instance,
 * if you have external 1MHz clock and you do not want to waste a timer)
 */
#define DENSO_1MHZ_CLOCK TRUE

/*
 * stm32f4:
 * TIM1_CH1
 *   PA8, AF1
 * TIM1_CH2
 *   PA9, AF1
 *   PE11, AF1
 * TIM1_CH3
 *   PA10, AF1
 *   PE13, AF1
 * TIM1_CH4
 *   PA11, AF1
 *   PE14, AF1
 *
 * TIM2_CH1
 *   PA0, AF1
 *
 * TIM2_CH2
 *   PA1, AF1
 *   PB3, AF1
 *
 * TIM2_CH3
 *   PA2, AF1
 *   PB10, AF1
 *
 * TIM2_CH4
 *   PA3, AF1
 *   PB11, AF1
 *
 * TIM3_CH1
 *   PA6, AF2
 *   PB4, AF2
 *   PC6
 * TIM3_CH2
 *   PA7, AF2
 *   PB5, AF2
 *   PC7, AF2
 * TIM3_CH3
 *   PB0, AF2
 *   PC8
 * TIM3_CH4
 *   PB1, AF2
 *   PC9
 *
 * TIM4_CH1
 *   PD12, AF2
 *
 * TIM4_CH2
 *   PB7, AF2
 *   PD13, AF2
 *
 * TIM4_CH3
 *   PD14, AF2
 *
 * TIM4_CH4
 *   PD15, AF2
 */

#ifdef CBIBIOS
 #define DENSO_SERIAL_OUTPUT_TIMER PWMD3
#else
 #define DENSO_SERIAL_OUTPUT_TIMER			TIM3
 #define DENSO_SERIAL_OUTPUT_IRQHANDLER		TIM3_IRQHandler
 #define DENSO_SERIAL_OUTPUT_CLK			RCC_APB1Periph_TIM3
 #define DENSO_SERIAL_INPUT_TIMER			TIM4
 #define DENSO_SERIAL_INPUT_IO_PORT			GPIOB
 #define DENSO_SERIAL_INPUT_CLK				RCC_APB1Periph_TIM4
 #define DENSO_SERIAL_INPUT_PIN				7
 #define DENSO_SERIAL_INPUT_MODE_ALT		GPIO_AF_TIM4
 #define DENSO_SERIAL_INPUT_IRQ				TIM4_IRQn
 #define DENSO_SERIAL_INPUT_IRQHANDLER		TIM4_IRQHandler

#endif

#define DENSO_SERIAL_CLOCK_FREQUENCY 62500
#define DENSO_SERIAL_CLOCK_IO_PORT GPIOB
#define DENSO_SERIAL_CLOCK_PIN 0
#define DENSO_SERIAL_CLOCK_MODE_ALT GPIO_AF_TIM3
// CH3 in STM32 terms
#define DENSO_SERIAL_CLOCK_CHANNEL 2

#define DENSO_SERIAL_COMMAND_IO_PORT GPIOB
#define DENSO_SERIAL_COMMAND_PIN 1
#define DENSO_SERIAL_COMMAND_MODE_ALT GPIO_AF_TIM3
#define DENSO_SERIAL_COMMAND_IRQ TIM3_IRQn

// CH1 in STM32 terms
#define DENSO_SERIAL_COMMAND_CHANNEL 0


void initDensoAdc2();

void initDensoAdc();


#ifdef DENSO_1MHZ_CLOCK
 void init1mhzTimer();
 #define DENSO_PRIMARY_CLOCK_FREQUENCY 1000000

// PB0: TIM3_CH3, AF3
#ifdef CBIBIOS
 #define DENSO_PRIMARY_CLOCK_TIMER PWMD3
 #define DENSO_PRIMARY_CLOCK_IO_PORT GPIOB
 #define DENSO_PRIMARY_CLOCK_PIN 0
 #define DENSO_PRIMARY_CLOCK_MODE_ALT 2
#else
 #define DENSO_PRIMARY_CLOCK_CLK RCC_APB1Periph_TIM2
 #define DENSO_PRIMARY_CLOCK_IO_PORT GPIOB
 #define DENSO_PRIMARY_CLOCK_PIN 3
 #define DENSO_PRIMARY_CLOCK_MODE_ALT 1
 #define DENSO_PRIMARY_CLOCK_TIMER TIM2

// #define DENSO_PRIMARY_CLOCK_CLK RCC_APB1Periph_TIM4
// #define DENSO_PRIMARY_CLOCK_IO_PORT GPIOD
// #define DENSO_PRIMARY_CLOCK_PIN 13
// #define DENSO_PRIMARY_CLOCK_MODE_ALT 2
// #define DENSO_PRIMARY_CLOCK_TIMER TIM4

#endif
// CH3 goes as '2' if you cound from zero
#define DENSO_PRIMARY_CLOCK_CHANNEL 2

#endif

void initDataInputTimer();
void initDataOutputTimer();

 #ifndef CHIBIOS
  #include <stm32f4xx_gpio.h>
  void configeInputPin(GPIO_TypeDef * PORT, int pin, int af);
  void configeOutputPin(GPIO_TypeDef * PORT, int pin, int af);
  void setupPwmOutputChannel(TIM_TypeDef* TIMx, int channel, int width,
		int polarity);
 #endif
#endif /* ADC_DENSO_H_ */
