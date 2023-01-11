
/*
 * 	adcST.h
 *
 *  Created on:
 *      Author: belyaev
 */

#ifndef _gpio_ST_H
#define _gpio_ST_H
//*******************************************************************************************
//*******************************************************************************************
//#include "stm32f10x.h"
//#include "stm32f103xb.h"

#include "main.h"

//*******************************************************************************************
//*******************************************************************************************
#define GPIO_POLLING_DELAY  10 //50

#define GPIO_PIN_HIGH		1
#define GPIO_PIN_LOW		0

//************************************
//// MCU_EN - PB2
//#define MCU_EN_GPIO_PORT	GPIOB
//#define MCU_EN_GPIO_PIN		2
//#define MCU_POWER_ON()	   	(MCU_EN_GPIO_PORT->BSRR = GPIO_BSRR_BS2)
//#define MCU_POWER_OFF()    	(MCU_EN_GPIO_PORT->BSRR = GPIO_BSRR_BR2)
//#define MCU_EN_Toggel() 	(MCU_EN_GPIO_PORT->ODR ^= GPIO_ODR_ODR2)
//// LED_ACT - PA15
//#define LED_ACT_GPIO_PORT	GPIOA
//#define LED_ACT_GPIO_PIN	15
//#define LED_ACT_High()   	(LED_ACT_GPIO_PORT->BSRR = GPIO_BSRR_BS15)
//#define LED_ACT_Low()    	(LED_ACT_GPIO_PORT->BSRR = GPIO_BSRR_BR15)
//#define LED_ACT_Toggel() 	(LED_ACT_GPIO_PORT->ODR ^= GPIO_ODR_ODR15)
//// GPS_EN - PB13
//#define GPS_EN_High()   	(GPIOB->BSRR = GPIO_BSRR_BS13)
//#define GPS_EN_Low()    	(GPIOB->BSRR = GPIO_BSRR_BR13)
//#define GPS_EN_Toggel() 	(GPIOB->ODR ^= GPIO_ODR_ODR13)
//// LIDAR_EN - PB5
//#define LIDAR_EN_High()   	(GPIOB->BSRR = GPIO_BSRR_BS5)
//#define LIDAR_EN_Low()    	(GPIOB->BSRR = GPIO_BSRR_BR5)
//#define LIDAR_EN_Toggel() 	(GPIOB->ODR ^= GPIO_ODR_ODR5)
//// BB_PWR_BTN - PA8
//#define BB_PWR_BTN_High()   (GPIOA->BSRR = GPIO_BSRR_BS8)
//#define BB_PWR_BTN_Low()    (GPIOA->BSRR = GPIO_BSRR_BR8)
//#define BB_PWR_BTN_Toggel() (GPIOA->ODR ^= GPIO_ODR_ODR8)
//// FAN_EN - PB12
//#define FAN_EN_High()   	(GPIOB->BSRR = GPIO_BSRR_BS12)
//#define FAN_EN_Low()    	(GPIOB->BSRR = GPIO_BSRR_BR12)
//#define FAN_EN_Toggel() 	(GPIOB->ODR ^= GPIO_ODR_ODR12)
//// PWR_BTN_LED - PB9
//#define PWR_BTN_LED_High()   (GPIOB->BSRR = GPIO_BSRR_BS9)
//#define PWR_BTN_LED_Low()    (GPIOB->BSRR = GPIO_BSRR_BR9)
//#define PWR_BTN_LED_Toggel() (GPIOB->ODR ^= GPIO_ODR_ODR9)
//// LAMP_PWM - PB1
//#define LAMP_PWM_High()   	(GPIOB->BSRR = GPIO_BSRR_BS1)
//#define LAMP_PWM_Low()   	(GPIOB->BSRR = GPIO_BSRR_BR1)
//#define LAMP_PWM_Toggel() 	(GPIOB->ODR ^= GPIO_ODR_ODR1)
////************************************
//// Линия упраяления драйвером мотора DRV8825
//// DRV_MODE2 - PC13
//#define DRV_MODE2_High()   	(GPIOC->BSRR = GPIO_BSRR_BS13)
//#define DRV_MODE2_Low()    	(GPIOC->BSRR = GPIO_BSRR_BR13)
//#define DRV_MODE2_Toggel() 	(GPIOC->ODR ^= GPIO_ODR_ODR13)
//// DRV_MODE1 - PC14
//#define DRV_MODE1_High()   	(GPIOC->BSRR = GPIO_BSRR_BS14)
//#define DRV_MODE1_Low()    	(GPIOC->BSRR = GPIO_BSRR_BR14)
//#define DRV_MODE1_Toggel() 	(GPIOC->ODR ^= GPIO_ODR_ODR14)
//// DRV_MODE0 - PC15
//#define DRV_MODE0_High()   	(GPIOC->BSRR = GPIO_BSRR_BS15)
//#define DRV_MODE0_Low()    	(GPIOC->BSRR = GPIO_BSRR_BR15)
//#define DRV_MODE0_Toggel() 	(GPIOC->ODR ^= GPIO_ODR_ODR15)
//// DRV_STEP - PA0
//#define DRV_STEP_GPIO_PORT	GPIOA
//#define DRV_STEP_GPIO_PIN	0
//#define DRV_STEP_High()   	(GPIOA->BSRR = GPIO_BSRR_BS0)
//#define DRV_STEP_Low()    	(GPIOA->BSRR = GPIO_BSRR_BR0)
//#define DRV_STEP_Toggel() 	(GPIOA->ODR ^= GPIO_ODR_ODR0)
//// /DRV_EN - PA1
//#define DRV_EN_GPIO_PORT	GPIOA
//#define DRV_EN_GPIO_PIN		1
//#define DRV_EN_High()   	(GPIOA->BSRR = GPIO_BSRR_BS1)
//#define DRV_EN_Low()    	(GPIOA->BSRR = GPIO_BSRR_BR1)
//#define DRV_EN_Toggel() 	(GPIOA->ODR ^= GPIO_ODR_ODR1)
//// DRV_DIR - PA4
//#define DRV_DIR_GPIO_PORT	GPIOA
//#define DRV_DIR_GPIO_PIN	4
//#define DRV_DIR_High()   	(GPIOA->BSRR = GPIO_BSRR_BS4)
//#define DRV_DIR_Low()    	(GPIOA->BSRR = GPIO_BSRR_BR4)
//#define DRV_DIR_Toggel() 	(GPIOA->ODR ^= GPIO_ODR_ODR4)
//// /DRV_RESET - PA7
//#define DRV_RESET_GPIO_PORT	GPIOA
//#define DRV_RESET_GPIO_PIN	7
//#define DRV_RESET_High()   	(GPIOA->BSRR = GPIO_BSRR_BS7)
//#define DRV_RESET_Low()    	(GPIOA->BSRR = GPIO_BSRR_BR7)
//#define DRV_RESET_Toggel() 	(GPIOA->ODR ^= GPIO_ODR_ODR7)
//************************************
// Макросы
#define GPIO_PIN_High(gpio, pin) 	(gpio->BSRR = (0x00001<<pin))
#define GPIO_PIN_Low(gpio, pin)  	(gpio->BSRR = (0x10000<<pin))
#define GPIO_PIN_Toggel(gpio, pin)	(gpio->ODR ^= (0x00001<<pin))

//************************************
//#define LedPC13On()     (GPIOC->BSRR = GPIO_BSRR_BS13)
//#define LedPC13Off()    (GPIOC->BSRR = GPIO_BSRR_BR13)
//#define LedPC13Toggel() (GPIOC->ODR ^= GPIO_ODR_ODR13)

//#define LedPA6_On()     	(GPIOA->BSRR = GPIO_BSRR_BS6)
//#define LedPA6_Off()    	(GPIOA->BSRR = GPIO_BSRR_BR6)
//#define LedPA6_Toggel() 	(GPIOA->ODR ^= GPIO_ODR_ODR6)

//#define LedPA7_On()     	(GPIOA->BSRR = GPIO_BSRR_BS7)
//#define LedPA7_Off()    	(GPIOA->BSRR = GPIO_BSRR_BR7)
//#define LedPA7_Toggel() 	(GPIOA->ODR ^= GPIO_ODR_ODR7)

//*******************************************************************************************
//*******************************************************************************************
void STM32_GPIO_InitForOutputPushPull(GPIO_TypeDef *port, uint32_t pin);
void STM32_GPIO_InitForOutputOpenDrain(GPIO_TypeDef *port, uint32_t pin);
void STM32_GPIO_InitForInputPullUp(GPIO_TypeDef *port, uint32_t pin);
void STM32_GPIO_InitForInputPullDown(GPIO_TypeDef *port, uint32_t pin);
void STM32_GPIO_InitForFloatingInput(GPIO_TypeDef *port, uint32_t pin);
void STM32_GPIO_InitForAnalogInput(GPIO_TypeDef *port, uint32_t pin);

void     STM32_GPIO_Init     (void);
void     STM32_GPIO_CheckLoop(void);
uint32_t STM32_GPIO_GetPortState(GPIO_TypeDef *port);
uint32_t STM32_GPIO_GetPinState(GPIO_TypeDef *port, uint32_t pin);

//*******************************************************************************************
//*******************************************************************************************
#endif /*_gpio_ST_H*/


