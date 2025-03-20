#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"

void vStepperControlX (void){
    int32_t steps;
    for (;;)
    {
        xTaskNotifyWait(pdFALSE,    // Don't clear bits on entry
            0,  // Clear all bits on exit
            &steps, // Receives the notification value
            portMAX_DELAY); // Wait indefinitely
        if (steps < 0)
        {
            HAL_GPIO_WritePin(stp1_dir_GPIO_Port, stp1_dir_Pin, GPIO_PIN_RESET);
            steps = steps * -1;
        }
        else
            HAL_GPIO_WritePin(stp1_dir_GPIO_Port, stp1_dir_Pin, GPIO_PIN_SET);
        
        for (uint32_t pwm = 0; pwm < steps; pwm++){
            HAL_GPIO_WritePin(stp1_pul_GPIO_Port, stp1_pul_Pin, GPIO_PIN_SET);
            osDelay(1);
            HAL_GPIO_WritePin(stp1_pul_GPIO_Port, stp1_pul_Pin, GPIO_PIN_RESET);
            osDelay(1);
        }
    }
}

void vStepperControlY (void){
    int32_t steps;
    for (;;)
    {
        xTaskNotifyWait(pdFALSE,    // Don't clear bits on entry
            0,  // Clear all bits on exit
            &steps, // Receives the notification value
            portMAX_DELAY); // Wait indefinitely
        if (steps < 0)
        {
            HAL_GPIO_WritePin(stp2_dir_GPIO_Port, stp2_dir_Pin, GPIO_PIN_RESET);
            steps = steps * -1;
        }
        else
            HAL_GPIO_WritePin(stp2_dir_GPIO_Port, stp2_dir_Pin, GPIO_PIN_SET);
        
        for (uint32_t pwm = 0; pwm < steps; pwm++){
            HAL_GPIO_WritePin(stp2_pul_GPIO_Port, stp2_pul_Pin, GPIO_PIN_SET);
            osDelay(1);
            HAL_GPIO_WritePin(stp2_pul_GPIO_Port, stp2_pul_Pin, GPIO_PIN_RESET);
            osDelay(1);
        }
    }
}