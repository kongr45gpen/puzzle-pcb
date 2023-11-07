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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../Drivers/SSD1306/ssd1306.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern TIM_HandleTypeDef htim5;
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
#define DETECT_INSERT_Pin GPIO_PIN_12
#define DETECT_INSERT_GPIO_Port GPIOA
#define DETECT_INSERT_EXTI_IRQn EXTI12_IRQn

/* USER CODE BEGIN Private defines */

// LED definition: LED1 (B) TIM3_CH1
// LED definition: LED2 (R) TIM3_CH2
// LED definition: LED3 (G) TIM3_CH3

// LED definition: LED4 (B) TIM3_CH4
// LED definition: LED5 (R) TIM4_CH1
// LED definition: LED6 (G) TIM4_CH2

// LED definition: LED7 (B) TIM2_CH1
// LED definition: LED8 (R) TIM15_CH1
// LED definition: LED9 (G) TIM15_CH2

// All values are 12-bit (4095 max)

enum LED {
    LED1B,
    LED1R,
    LED1G,
    LED2B,
    LED2R,
    LED2G,
    LED3B,
    LED3R,
    LED3G,
    _LED_MAX
};

inline void set_led(enum LED led, int value) {
    if (value > 4095) value = 4095;
    if (value < 0) value = 0;
//    value = 4095 - value;

    switch (led) {
        case LED1B:
            TIM3->CCR1 = value;
            break;
        case LED1R:
            TIM3->CCR2 = value;
            break;
        case LED1G:
            TIM3->CCR3 = value;
            break;
        case LED2B:
            TIM3->CCR4 = value;
            break;
        case LED2R:
            TIM4->CCR1 = value;
            break;
        case LED2G:
            TIM4->CCR2 = value;
            break;
        case LED3B:
            TIM2->CCR1 = value;
            break;
        case LED3R:
            TIM15->CCR1 = value;
            break;
        case LED3G:
            TIM15->CCR2 = value;
            break;
    }
}

inline void leds_off() {
//    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(LED9_GPIO_Port, LED9_Pin, GPIO_PIN_SET);
    for (int i = 0; i < _LED_MAX; i++) {
        set_led(i, 0);
    }
}

inline void stop_sound() {
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
}

inline void start_sound(float frequency) {
    htim5.Instance->ARR = 4e6f / frequency;
    htim5.Instance->CNT = 0; // Sometimes timer may complain if counter is larger than ARR so it never gets to an equal
    // point
//    htim2.Instance->CCR1 = htim2.Instance->ARR / 2; // duty cycle
    // DUTY CYCLE OF PSEAKER PWM
    htim5.Instance->CCR1 = 30;
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
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
