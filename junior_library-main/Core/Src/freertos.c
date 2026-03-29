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



//motor pid tuning
/*
volatile float aw = 0.0, bw = 0.0, cw = 0.0, dw = 0.0;
volatile float kp = 0.0, ki = 0.0, kd = 0.0;
volatile int mode = 2;
volatile int tune_finish = 0;

char buffer[64];
char rx_buf[64];
volatile uint8_t rx_idx = 0;
volatile uint8_t cmd_ready = 0;
uint8_t uart5_rx;
////

void process_command() {
    if (!cmd_ready) return;

    char cmd = rx_buf[0];
    float val = atof((char*)&rx_buf[1]); // Convert everything after the first char to float

    switch(cmd) {
        case 'v': case 'V':
        	aw = val;
        	bw = val;
        	cw = val;
        	dw = val;
            RNSVelocity(aw, bw, cw, dw, &rns);
            break;
        case 'p': case 'P':
            kp = val;


            RNSSet(&rns, RNS_B_RIGHT_VEL_PID, kp, ki, kd);

            break;
        case 'i': case 'I':
            ki = val;


            RNSSet(&rns, RNS_B_RIGHT_VEL_PID, kp, ki, kd);

            break;
        case 'd': case 'D':
            kd = val;


            RNSSet(&rns, RNS_B_RIGHT_VEL_PID, kp, ki, kd);

            break;
        case 'n': case 'N': // Send 'n' to advance to the Next motor
            mode++;
            aw = 0; bw = 0; cw = 0; dw = 0; // Stop previous motor
            RNSVelocity(aw, bw, cw, dw, &rns);

            if(mode > 3) {
                mode = 0;
                tune_finish = 1;
            }
            break;
    }
    rx_idx = 0;
    cmd_ready = 0;
}

float actual_V = 0.0;
float target_V = 0.0;

void tune_motor() {
    uint32_t last_call = 0;
    tune_finish = 0;
    mode = 0;
    HAL_UART_Receive_IT(&huart5, &uart5_rx, 1);

    aw = 0; bw = 0; cw = 0; dw = 0;
    RNSVelocity(0, 0, 0, 0, &rns);
    while(!tune_finish) {
        process_command();
        uint32_t now = HAL_GetTick();
        if (now - last_call >= 1) {

        	RNSEnquire(RNS_VEL_BOTH, &rns);

            if (mode == 0)      { actual_V = rns.RNS_data.common_buffer[0].data; target_V = aw; }
            else if (mode == 1) { actual_V = rns.RNS_data.common_buffer[1].data; target_V = bw; }
            else if (mode == 2) { actual_V = rns.RNS_data.common_buffer[2].data; target_V = cw; }
            else if (mode == 3) { actual_V = rns.RNS_data.common_buffer[3].data; target_V = dw; }

            sprintf(buffer, "%.2f,%.2f\r\n", actual_V, target_V);
//            sprintf(buffer, "%.2f,%.2f,%.2f,%.2f\r\n", rns.RNS_data.common_buffer[0].data,rns.RNS_data.common_buffer[1].data,rns.RNS_data.common_buffer[2].data,rns.RNS_data.common_buffer[3].data);
            UARTPrintString(&huart5, buffer);

            last_call = now;
        }
    }
    RNSStop(&rns);
}

// Place this in your main.c (or wherever your user callbacks are defined)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // Check if the interrupt was triggered by UART5
    if (huart->Instance == UART5) {

        // Look for end-of-line characters (Enter key: Carriage Return or Line Feed)
        if (uart5_rx == '\r' || uart5_rx == '\n') {

            // Make sure we actually received data (prevents double-triggering on \r\n)
            if (rx_idx > 0) {
                rx_buf[rx_idx] = '\0'; // Null-terminate the buffer so atof() works correctly
                cmd_ready = 1;         // Signal process_command() to run
            }

        } else {
            // Append the new character to the buffer
            // We check against 63 to leave room for the '\0' terminator (buffer is 64)
            if (rx_idx < 63) {
                rx_buf[rx_idx] = uart5_rx;
                rx_idx++;
            }
        }

        // Re-arm the interrupt to listen for the next single byte
        HAL_UART_Receive_IT(&huart5, &uart5_rx, 1);
    }
}
*/
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
