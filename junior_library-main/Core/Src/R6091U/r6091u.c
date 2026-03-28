/*
 * r6091u.c
 *
 *  Created on: Oct 10, 2021
 *      Author: amera
 */

#include "r6091u.h"
#include "../include.h"
extern volatile int pop;
extern volatile uint8_t buffer[15];
extern volatile uint8_t index1;
extern volatile uint8_t rx_byte;



void R6091U_Init(R6091U_t* IMU,UART_HandleTypeDef* huartx){

	IMU->huartx = huartx;
	IMU->State = PENDING_SYNC;
//	HAL_UART_Receive_IT(IMU->huartx, (uint8_t *)&rx_byte, 1);
	HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 1);


}

void R6091U_ParseByte(uint8_t byte) {

    buffer[index1] = byte;
    index1++;

    // Verify first header byte
    if (index1 == 1) {
        if (buffer[0] != 0xAA) {
            index1 = 0;
        }
    }
    //Verify second header byte
    else if (index1 == 2) {
        if (buffer[1] != 0x00) {
            index1 = 0;
        }
    }
    // Process the full packet
    else if (index1 == 15) {
        uint8_t checksum = 0;

        for(int i = 2; i < 14; i++) {
            checksum += buffer[i];
        }


        if (checksum == buffer[14]) {

            int16_t yaw = (int16_t)(buffer[3] | (buffer[4] << 8));
            fyaw = ((float)yaw / 100.0f) + 180.0f;
            pop = 1;
        }

        index1 = 0;
    }
}

void R6091U_Handler(R6091U_t* IMU){

//	uint8_t checksum;

	switch(IMU->State){


	case PENDING_SYNC:

		if(IMU->Buffer[0] == 0xAA){
			IMU->State = CONFIRMING_SYNC;
		}

		HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 1);
		break;

	case CONFIRMING_SYNC:

		if(IMU->Buffer[0] == 0x00){
			IMU->State = IN_SYNC;
			HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 13);

		}else{

			IMU->State = PENDING_SYNC;
			HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 1);
		}

		break;

	case IN_SYNC:
		IMU->checksum = 0;

		IMU->checksum = IMU->Buffer[0] + IMU->Buffer[1] + IMU->Buffer[2] + IMU->Buffer[3] + IMU->Buffer[4] + IMU->Buffer[5]
						+ IMU->Buffer[6] + IMU->Buffer[7] + IMU->Buffer[8] + IMU->Buffer[9] + IMU->Buffer[10] + IMU->Buffer[11];


		if( IMU->checksum == IMU->Buffer[12]){
			int16_t yaw = (int16_t)(IMU->Buffer[1] | (IMU->Buffer[2] << 8));
			int16_t yawRate  = (int16_t)(IMU->Buffer[3]  | (IMU->Buffer[4]  << 8));
			int16_t accX_raw = (int16_t)(IMU->Buffer[5]  | (IMU->Buffer[6]  << 8));
			int16_t accY_raw = (int16_t)(IMU->Buffer[7]  | (IMU->Buffer[8]  << 8));
			int16_t accZ_raw = (int16_t)(IMU->Buffer[9] | (IMU->Buffer[10] << 8));
			float ax = accX_raw / 16384.0f;
			float ay = accY_raw / 16384.0f;
			float az = accZ_raw / 16384.0f;
			froll  = atan2f(ay, az) * 57.2958f;
			fpitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2958f;
			fyaw = ((float)(yaw) / (float)100.0) + 180.0;
			turn_count = IMU->Buffer[11];
			pop = 1;

		}
		memset(IMU->Buffer, 0, 13);
		IMU->State = PENDING_SYNC;
		HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 1);

		break;
	}
}
