

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "main.h"

/**
 * @brief  The application entry point.
 * @retval int
 */

//FLAGS
volatile uint8_t imu_lock_flag = 0;
uint8_t p_lock_flag = 0;
uint8_t A_lock_flag = 0;
uint8_t is_path_planning = 0;
uint8_t start_assembly_flag = 0;

//Allignment variables
float target_angle = 0.0;
float Allign_x = 0.0;
float Allign_y = 0.0;
float Allign_w = 0.0;

//char buffer[120];

//pwm variabls
typedef struct {
	int32_t BDC5_pwm;
	int32_t BDC6_pwm;
	int32_t BDC7_pwm;
	int32_t BDC8_pwm;
} MotorSpeeds_t;

MotorSpeeds_t motor_pwm = {0, 0, 0, 0};
SemaphoreHandle_t xMotorMutex = NULL;

float angle_wrapper(float angle) {
	while (angle > 180.0){
		angle -= 360.0;
	}
	while (angle < -180.0){
		angle += 360.0;
	}
	return angle;
}

void vMotorControlTask(void *vParameters) {//read global motor pwm varible and control via shiftreg
	MotorSpeeds_t local_pwm = {0, 0, 0, 0};
	for(;;) {
		if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
			                  local_pwm = motor_pwm;
			xSemaphoreGive( xMotorMutex );
		}
		WriteBDC(&BDC5,local_pwm.BDC5_pwm);
		WriteBDC(&BDC6,local_pwm.BDC6_pwm);
		WriteBDC(&BDC7,local_pwm.BDC7_pwm);
		WriteBDC(&BDC8,local_pwm.BDC8_pwm);
		SHIFTREGShift(&SR);
		vTaskDelay(pdMS_TO_TICKS(20));
	}
}

//void vBoxIntakeTask(void *vParameters) {
//	uint8_t closed_flag = 0;
//	uint8_t rotate_flag = 0;
//	uint8_t return_flag = 0;
//	for(;;){
//		while (!closed_flag) {
//			//control motor to close gripper
//			if(lsf1){
//				closed_flag = 1;
//				//stop motor
//				rotate_flag = 1;
//			}
//		}
//		while (rotate_flag) {
//			//control motor to rotate gripper
//			if(lsf2){
//				//stop rotate
//				return_flag = 1;
//				rotate_flag = 0;
//			}
//		}
//		while(return_flag){
//			if(lsf1){
//				//open gripper
//			}
//			else {
//				//rotate back
//				if(lsf3){
//					//stop motor
//					return_flag = 0;
//				}
//			}
//		}
//	}
//}

void vAllignmentTask(void *pvParameters) {
	for(;;) {
		if(A_lock_flag){

			if (Allign_x < -0.02 && Allign_x > -0.002) Allign_x = -0.02;
			if (Allign_w < -0.02 && Allign_w > -0.002) Allign_w = -0.02;
			if (Allign_y < -0.02 && Allign_y > -0.002) Allign_y = -0.02;
			const float L = 0.34f;
			const float COS_45 = 0.7071f;

			float linear_speed = sqrt(Allign_x*Allign_x + Allign_y*Allign_y);
			if (linear_speed < 0.002 && fabs(Allign_w) < 0.004) {
			    RNSStop(&rns);
//			    A_lock_flag = 0;
//			    target_angle = angle_wrapper(rns.RNS_data.common_buffer[0].data);
//			    imu_lock_flag = 1;
//			    start_assembly_flag = 1;

				}

			else{
				float FL = (COS_45 * Allign_y) + (COS_45 * Allign_x) - (L * Allign_w);
				float FR = (COS_45 * Allign_y) - (COS_45 * Allign_x) + (L * Allign_w);
				float BL = (COS_45 * Allign_y) - (COS_45 * Allign_x) - (L * Allign_w);
				float BR = (COS_45 * Allign_y) + (COS_45 * Allign_x) + (L * Allign_w);

			float final_scale = 5.0f;
			RNSVelocity(FL*final_scale,
					FR*final_scale,
					BL*final_scale,
					BR*final_scale,
					&rns);

		}
		}
		vTaskDelay(pdMS_TO_TICKS(20));
	}
}

//void vAssemblyTask(void *pvParameters){
//	for(;;) {
//		if(start_assembly_flag) {
//			vTaskDelay(pdms_TO_TICKS(1000));
//
//			start_assembly_flag = 0;
//		}
//		vTaskDelay(pdms_TO_TICKS(20));
//	}
//}

//void vTelemetryTask(void *pvParameters) {
//		for(;;) {
//
//		RNSEnquire(RNS_X_Y_IMU_LSA, &rns);
//		float x = rns.RNS_data.common_buffer[1].data;
//		float y = rns.RNS_data.common_buffer[2].data;
//		sprintf(buffer, "%.2f,%.2f\r\n", x,y);
//		UARTPrintString(&huart2, buffer);
//
//		vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100ms
//	}
//}

void vControllerTask(void *pvParameters) { //PS4 controls
	uint16_t prev_button = 0;

    for(;;) {
    	ps4.button = ps4.buf1 | (ps4.buf2 << 8) | (ps4.buf3 << 16);
//    			A_lock_flag = !A_lock_flag;
//    	    	imu_lock_flag = 0;

    			if (ps4.button & PS) {
    				RNSStop(&rns);
    				NVIC_SystemReset();
    			}
    			else if ((ps4.button & UP) && !(prev_button & UP)) {
    				//platform lift up/down (limit switch)
    				//toggling
    			}
    			else if ((ps4.button & DOWN) && !(prev_button & DOWN)) {

    			}

    			else if ((ps4.button & RIGHT) && !(prev_button & RIGHT)) {
    				target_angle = angle_wrapper(floor(target_angle / 90.0) * 90.0 + 90.0);
    				//uses imu lock to rotate
    			}

    			else if ((ps4.button & LEFT) && !(prev_button & LEFT)) {
    				target_angle = angle_wrapper(ceil(target_angle / 90.0) * 90.0 - 90.0);
    				//uses imu lock to rotate
    			}

    			else if ((ps4.button & CIRCLE) && !(prev_button & CIRCLE)){
    				//rotate motor to close/open gripper (read encoder value)
    				//toggling
    			}
    			else if ((ps4.button & TRIANGLE) && !(prev_button & TRIANGLE)) {
    				p_lock_flag = !p_lock_flag;
    			}

    			else if ((ps4.button & SQUARE) && !(prev_button & SQUARE)) {
    				//rotate motor to lift kfs to platform/return to original place (read encoder value)
    				//toggling
    			}
    			else if ((ps4.button & CROSS) && !(prev_button & CROSS)) {
    				imu_lock_flag = !imu_lock_flag;
    			}
    			else if ((ps4.button & TOUCH) && !(prev_button & TOUCH)) {
    				//roll kfs on platform when 1, stop motor when 0
    			}
    			else if ((ps4.button & R1) && !(prev_button & R1)) {
    				// toggle weapon rototion
    			}
    			else if ((ps4.button & L1) && !(prev_button & L1)) {
    				// toggle weapon gripper open/close
    			}

    			prev_button = ps4.button; //for debounce
    			vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void vDriveTask(void *pvParameters) {
	float x = 0.0, y = 0.0, w = 0.0;
	int set_once = 1;
	float cos_t, sin_t;

	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(10);

    for(;;) {

    	if (!is_path_planning && !A_lock_flag) {
			if (fabs(ps4.joyL_x) > 0.1 || fabs(ps4.joyL_y) > 0.1 || fabs(ps4.joyR_x) > 0.1) {
				x = ps4.joyL_x; y = ps4.joyL_y; w = ps4.joyR_x;
			}


			RNSEnquire(RNS_X_Y_IMU_LSA, &rns);
			float current_angle = angle_wrapper(rns.RNS_data.common_buffer[0].data);
			float rad  = current_angle * 3.1415926f/180.0f;
			sin_t = sinf(rad);
			cos_t = cosf(rad);

			if(set_once){
				target_angle = current_angle;
				set_once = 0;
			}

			if(p_lock_flag){
				float x_world = x;
				float y_world = y;
				x =  x_world * cos_t + y_world * sin_t;
				y = -x_world * sin_t + y_world * cos_t;
			}

			if (imu_lock_flag) {
				error_val = -1*angle_wrapper(target_angle - current_angle);

				if (fabs(ps4.joyR_x) > 0.1){

					w = ps4.joyR_x;
					target_angle = current_angle;


				} else {

					if(fabs(error_val) > 2.0) {
						w = w_pid_out;
					} else{
						w = 0.0;
					}
				}
        }

        float motorA = y - x - w;
        float motorB = y + x + w;
        float motorC = y + x - w;
        float motorD = y - x + w;


        float max_val = fabs(motorA);
        if (fabs(motorB) > max_val) max_val = fabs(motorB);
        if (fabs(motorC) > max_val) max_val = fabs(motorC);
        if (fabs(motorD) > max_val) max_val = fabs(motorD);


        if (max_val > 1.0) {
        	motorA /= max_val;
        	motorB /= max_val;
        	motorC /= max_val;
        	motorD /= max_val;
        }


        float final_scale = 1.75f;
        RNSVelocity(motorA * final_scale,
        		motorB * final_scale,
				motorC * final_scale,
				motorD * final_scale,
				&rns);
    }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}




int main(void)
{

	set();

//	xMotorMutex = xSemaphoreCreateMutex();
//	if (xMotorMutex != NULL) {
//
//
//		xTaskCreate(
//				vReadEncoderTask,
//				"ReadEncoderTask",
//				512,
//				NULL,
//				3,
//				NULL
//		);
//		xTaskCreate(
//				vMotorControlTask,
//				"MotorControlTask",
//				512,
//				NULL,
//				3,
//				NULL
//		    );
	//	xTaskCreate(
	//			vControllerTask,
	//			"ControlTask",
	//			512,
	//			NULL,
	//			1,
	//			NULL
	//	    );
	//
	//	xTaskCreate(
	//			vTelemetryTask,
	//			"TelemetryTask",
	//			1024,
	//			NULL,
	//			1,
	//			NULL
	//	);

	//	xTaskCreate(
	//			vDriveTask,
	//			"DriveTask",
	//			512,
	//			NULL,
	//			2,
	//			NULL
	//	);
	//

	//	xTaskCreate(
	//			vAllignmentTask,
	//			"AllignmentTask",
	//			512,
	//			NULL,
	//			2,
	//			NULL
	//		);
//	}
//
//	vTaskStartScheduler();

	while(1){}
}

void TIM6_DAC_IRQHandler(void)
{

	if (imu_lock_flag) {
		led1 = !led1;
		PID(&pid_rotate);
	}

	HAL_TIM_IRQHandler(&htim6);
}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == UART5)
//    {
//
//        if (uart5_rx == '\n' || uart5_rx == '\r') {
//            if (rx_idx > 0) {
//                rx_buf[rx_idx] = '\0';
//                sscanf(rx_buf, "%f,%f,%f", &Allign_x, &Allign_y,&Allign_w);
//
//                rx_idx = 0;
//            }
//        }
//        else {
//
//            if (rx_idx < 63) {
//                rx_buf[rx_idx++] = uart5_rx;
//            }
//        }
//
//
//        HAL_UART_Receive_IT(&huart5, &uart5_rx, 1);
//    }
//}


/**
 * @brief  This function is executed in case of error occurrence.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // If the UART crashes due to an Overrun Error (or any error),
    // force it to clear the error and restart the receive interrupt.
    if (huart->Instance == UART5) {
        // Clear the Overrun error flag (syntax might vary slightly based on STM32F4 family)
        __HAL_UART_CLEAR_OREFLAG(huart);

        // Restart the interrupt
        HAL_UART_Receive_IT(&huart5, &uart5_rx, 1);
    }
}

void Error_Handler(void)
{


}
#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %dw\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

