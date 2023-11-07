/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l5xx_hal.h"
#include "../../Drivers/SSD1306/ssd1306.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern TIM_HandleTypeDef htim2;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZER_OUT_Pin GPIO_PIN_0
#define BUZZER_OUT_GPIO_Port GPIOA
#define LED7_Pin GPIO_PIN_5
#define LED7_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_0
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_1
#define LED4_GPIO_Port GPIOB
#define LED8_Pin GPIO_PIN_14
#define LED8_GPIO_Port GPIOB
#define LED9_Pin GPIO_PIN_15
#define LED9_GPIO_Port GPIOB
#define DETECT_INSERT_Pin GPIO_PIN_12
#define DETECT_INSERT_GPIO_Port GPIOA
#define DETECT_INSERT_EXTI_IRQn EXTI12_IRQn
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_6
#define LED5_GPIO_Port GPIOB
#define LED6_Pin GPIO_PIN_7
#define LED6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
inline void leds_off() {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED9_GPIO_Port, LED9_Pin, GPIO_PIN_SET);
}

inline void stop_sound() {
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

inline void start_sound(float frequency) {
    htim2.Instance->ARR = 4e6f / frequency;
    htim2.Instance->CNT = 0; // Sometimes timer may complain if counter is larger than ARR so it never gets to an equal
    // point
//    htim2.Instance->CCR1 = htim2.Instance->ARR / 2; // duty cycle
    htim2.Instance->CCR1 = 200;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

inline void sleep() {
    leds_off();

    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();

    stop_sound();

    HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
