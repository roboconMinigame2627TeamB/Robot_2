/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "MAIN/main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */



//void buttons_pressed(){
//		ps4.button = ps4.buf1 | (ps4.buf2 << 8) | (ps4.buf3 << 16);
//
//		if (ps4.button & PS) {
//			RNSStop(&rns);
//			NVIC_SystemReset();
//		}
//		else if ((ps4.button & UP) && !(prev_button & UP)) {
//
//			target_angle = angle_wrapper(target_angle + 180.0);
//			PIDDelayInit(&pid_rotate);
//
//		}
//		else if ((ps4.button & RIGHT) && !(prev_button & RIGHT)) {
//
//			target_angle = angle_wrapper(target_angle + 90.0);
//			PIDDelayInit(&pid_rotate);
//		}
//		else if ((ps4.button & LEFT) && !(prev_button & LEFT)) {
//
//			target_angle = angle_wrapper(target_angle - 90.0);
//			PIDDelayInit(&pid_rotate);
//
//		}
//		else if ((ps4.button & CIRCLE) && !(prev_button & CIRCLE)){
//			float point[2][7]={
//					{0.5,0.0,1.0,0.0,0.0,1.0,0.0},
//					{0.5,0.0,0.0,0.0,0.0,1.0,0.0}
////					{0.5,1.0,0.0,0.0,1.5,1.0,0.0},
////					{0.5,0.0,0.0,0.0,1.5,1.0,0.0}
//
//			};
//			RNSPPstart(point,2,&rns);
//			while(rns.RNS_data.common_instruction == RNS_BUSY) {
//
//			}
//
//		}
//		else if ((ps4.button & TRIANGLE) && !(prev_button & TRIANGLE)) {
//			imu_lock_flag = !imu_lock_flag;
//		}
//
//		else if ((ps4.button & SQUARE) && !(prev_button & SQUARE)) {
//			p_lock_flag = !p_lock_flag;
//		}
//		else if (fabs(ps4.joyL_x) > 0.1 || fabs(ps4.joyL_y) > 0.1 || fabs(ps4.joyR_x) > 0.1) {
//			x = ps4.joyL_x;
//			y = ps4.joyL_y;
//			w = ps4.joyR_x;
//		}
//		else {
//			x = 0.0;
//			y = 0.0;
//			w = 0.0;
//		}
//		prev_button = ps4.button;
//
//		RNSEnquire(RNS_X_Y_IMU_LSA, &rns);
//		float current_angle = angle_wrapper(rns.RNS_data.common_buffer[0].data);
//		float rad  = current_angle * 3.1415926f/180.0f;
//		sin_t = sinf(rad);
//		cos_t = cosf(rad);
//
//		if(set_once){
//			target_angle = current_angle;
//			set_once = 0;
//		}
//
//		if(p_lock_flag){
//			float x_world = x;
//			float y_world = y;
//			x =  x_world * cos_t + y_world * sin_t;
//			y = -x_world * sin_t + y_world * cos_t;
//		}
//
//		if (imu_lock_flag) {
//			error_val = -1*angle_wrapper(target_angle - current_angle);
//
//			if (fabs(ps4.joyR_x) > 0.1){
//				w = ps4.joyR_x;
//				target_angle = current_angle;
//				PIDDelayInit(&pid_rotate);
//
//			} else {
//
//				if(fabs(error_val) > 2.0) {
//					w = w_pid_out;
//				} else{
//					w = 0.0;
//				}
//			}
//		}
//
//		float motorA = y - x - w;
//		float motorB = y + x + w;
//		float motorC = y + x - w;
//		float motorD = y - x + w;
//
//
//		float max_val = fabs(motorA);
//		if (fabs(motorB) > max_val) max_val = fabs(motorB);
//		if (fabs(motorC) > max_val) max_val = fabs(motorC);
//		if (fabs(motorD) > max_val) max_val = fabs(motorD);
//
//
//		if (max_val > 1.0) {
//			motorA /= max_val;
//			motorB /= max_val;
//			motorC /= max_val;
//			motorD /= max_val;
//		}
//
//
//		float final_scale = 1.75f;
//		RNSVelocity(motorA * final_scale,
//				motorB * final_scale,
//				motorC * final_scale,
//				motorD * final_scale,
//				&rns);
//
//}
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
