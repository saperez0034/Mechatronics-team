#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "tim.h"

void vServoControl (void){
    uint32_t angle;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    for (;;)
    {
        xTaskNotifyWait(pdFALSE,    // Don't clear bits on entry
            0,  // Clear all bits on exit
            &angle, // Receives the notification value
            portMAX_DELAY); // Wait indefinitely
        uint32_t pulse = 1000 + ((angle * 1000)/180);
	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse);
    }
}

void vLinServoControl (void){
    uint32_t pctg;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    for (;;)
    {
        xTaskNotifyWait(pdFALSE,    // Don't clear bits on entry
            0,  // Clear all bits on exit
            &pctg, // Receives the notification value
            portMAX_DELAY); // Wait indefinitely
        uint32_t pulse = 1000 + ((pctg * 1000)/100);
	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
    }
}

void vLinActControl (void){
    uint32_t pctg;
    // uint32_t prev_pctg = 0;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 10000);
    for (;;)
    {
        xTaskNotifyWait(pdFALSE,    // Don't clear bits on entry
            0,  // Clear all bits on exit
            &pctg, // Receives the notification value
            portMAX_DELAY); // Wait indefinitely
        if (pctg){
            HAL_GPIO_WritePin(hbridge_l1_GPIO_Port, hbridge_l1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(hbridge_l2_GPIO_Port, hbridge_l2_Pin, GPIO_PIN_RESET);
        }
        else if (!pctg){
            HAL_GPIO_WritePin(hbridge_l1_GPIO_Port, hbridge_l1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(hbridge_l2_GPIO_Port, hbridge_l2_Pin, GPIO_PIN_SET);
        }
        // prev_pctg = pctg;
    }
}