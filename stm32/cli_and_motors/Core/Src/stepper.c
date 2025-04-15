#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"

void vStepperControlX (void){
    int32_t steps;
    for (;;)
    {
    	int8_t prev_dir = 0;
        xTaskNotifyWait(pdFALSE,    // Don't clear bits on entry
            0,  // Clear all bits on exit
            &steps, // Receives the notification value
            portMAX_DELAY); // Wait indefinitely
        if (steps < 0)
        {
            HAL_GPIO_WritePin(stp1_dir_GPIO_Port, stp1_dir_Pin, GPIO_PIN_RESET);
            steps = steps * -1;
            prev_dir = 0;
        }
        else {
            HAL_GPIO_WritePin(stp1_dir_GPIO_Port, stp1_dir_Pin, GPIO_PIN_SET);
        	prev_dir = 1;
        }

        for (uint32_t pwm = 0; pwm < steps; pwm++){
            if (stepper_x_stop){
                osDelay(100);
                if (prev_dir) {
					HAL_GPIO_WritePin(stp1_dir_GPIO_Port, stp1_dir_Pin, GPIO_PIN_RESET);
                }
                else {
                	HAL_GPIO_WritePin(stp1_dir_GPIO_Port, stp1_dir_Pin, GPIO_PIN_SET);
                }
                osDelay(50);
                for (uint32_t i = 0; i < 50; i++) {
                	HAL_GPIO_WritePin(stp1_pul_GPIO_Port, stp1_pul_Pin, GPIO_PIN_SET);
					osDelay(1);
					HAL_GPIO_WritePin(stp1_pul_GPIO_Port, stp1_pul_Pin, GPIO_PIN_RESET);
					osDelay(1);
                }
                stepper_x_stop = 0;
                break;
            }
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
    	int8_t prev_dir = 0;
        xTaskNotifyWait(pdFALSE,    // Don't clear bits on entry
            0,  // Clear all bits on exit
            &steps, // Receives the notification value
            portMAX_DELAY); // Wait indefinitely
        if (steps < 0)
        {
            HAL_GPIO_WritePin(stp2_dir_GPIO_Port, stp2_dir_Pin, GPIO_PIN_RESET);
            steps = steps * -1;
            prev_dir = 0;
        }
        else {
            HAL_GPIO_WritePin(stp2_dir_GPIO_Port, stp2_dir_Pin, GPIO_PIN_SET);
            prev_dir = 1;
        }

        for (uint32_t pwm = 0; pwm < steps; pwm++){
            if (stepper_y_stop){
                osDelay(100);
                if (prev_dir) {
                	HAL_GPIO_WritePin(stp2_dir_GPIO_Port, stp2_dir_Pin, GPIO_PIN_RESET);
				}
				else {
					HAL_GPIO_WritePin(stp2_dir_GPIO_Port, stp2_dir_Pin, GPIO_PIN_SET);
				}
                osDelay(50);
                for (uint32_t i = 0; i < 50; i++) {
                	HAL_GPIO_WritePin(stp2_pul_GPIO_Port, stp2_pul_Pin, GPIO_PIN_SET);
					osDelay(1);
					HAL_GPIO_WritePin(stp2_pul_GPIO_Port, stp2_pul_Pin, GPIO_PIN_RESET);
					osDelay(1);
				}
                stepper_y_stop = 0;
                break;
            }
            HAL_GPIO_WritePin(stp2_pul_GPIO_Port, stp2_pul_Pin, GPIO_PIN_SET);
            osDelay(1);
            HAL_GPIO_WritePin(stp2_pul_GPIO_Port, stp2_pul_Pin, GPIO_PIN_RESET);
            osDelay(1);
        }

    }
}
