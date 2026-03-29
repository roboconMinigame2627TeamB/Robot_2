
/*********************************************/
/*          Include Header                   */
/*********************************************/

#include "common.h"
#include "math.h"

void set(void) {

	Initialize();
//	PSxInitDMA(&ps4,&hi2c1);
//	TIMxInit(&htim6, 20000, 84, 5, 0);
	TIMxInit(&htim2, 20000, 84, 5, 0);//20ms
	RNS_config(&hcan1);
	PIDSourceInit(&error_val, &w_pid_out, &pid_rotate);
	PIDGainInit(0.02, 1.0, 1.0/180.0, 1.0, 2.7, 0.0, 1.1, 100.0, &pid_rotate);
	PIDDelayInit(&pid_rotate);

	MODNRobotBaseInit(MODN_FWD_OMNI, 2.0, 0.0, &Modn);
//	MODNRobotVelInit(&xr, &yr, &wr, &Modn);
//	MODNWheelVelInit(&v1, &v2, &v3, &v4, &Modn);
	led2 = 1;
	led3 = 1;
}


void RNS_config(CAN_HandleTypeDef* hcanx) {
	RNSInit(hcanx, &rns);

	//Encoder dcba(0-swap, 1-keep)  BDC dcba(0-keep, 1-swap) //0x00 0x00 0x

	RNSSet(&rns, RNS_DEVICE_CONFIG, (float) 0b00010101, (float) fwd_omni, (float) roboconPID);
	RNSSet(&rns, RNS_X_Y_ENC_CONFIG, 0.051 / 4000.0 * 3.142, 2.0, 0.0505 / 4000.0 * 3.142, 1.0); //1.0 for nonswap , 2.0 for swap
	RNSSet(&rns, RNS_F_KCD_PTD, 203.20885/ 204.50492, (float)(0.125 * 3.142 / 203.20885));
	RNSSet(&rns, RNS_B_KCD_PTD, 203.56232/ 203.60160, (float)(0.125 * 3.142 / 203.56232));

//	RNSSet(&rns, RNS_F_LEFT_VEL_SATEU, 1.0, 1.0 / 17.9120, 19999.0); //bluebot
//	RNSSet(&rns, RNS_F_RIGHT_VEL_SATEU, 1.0, 1.0 / 20.7897, 19999.0);
//	RNSSet(&rns, RNS_B_LEFT_VEL_SATEU, 1.0, 1.0 / 18.3077, 19999.0);
//	RNSSet(&rns, RNS_B_RIGHT_VEL_SATEU, 1.0, 1.0 / 18.7605, 19999.0);
//
//	RNSSet(&rns, RNS_F_LEFT_VEL_PID,  2.25, 13.5, 0.0);
//	RNSSet(&rns, RNS_F_RIGHT_VEL_PID, 1.85, 7.38, 0.0);
//	RNSSet(&rns, RNS_B_LEFT_VEL_PID,  3.3, 21.0, 0.0);
//	RNSSet(&rns, RNS_B_RIGHT_VEL_PID, 2.5, 8.5, 0.0);
	RNSSet(&rns, RNS_F_LEFT_VEL_SATEU, 1.0, 1.0/10.33, 19999.0); //black bot
	RNSSet(&rns, RNS_F_RIGHT_VEL_SATEU, 1.0, 1.0 / 11.48, 19999.0);
	RNSSet(&rns, RNS_B_LEFT_VEL_SATEU, 1.0, 1.0 / 10.67, 19999.0);
	RNSSet(&rns, RNS_B_RIGHT_VEL_SATEU, 1.0, 1.0 / 10.44, 19999.0);

	RNSSet(&rns, RNS_F_LEFT_VEL_PID,  1.8, 18.0, 0.0);
	RNSSet(&rns, RNS_F_RIGHT_VEL_PID, 1.764, 17.775, 0.0);
	RNSSet(&rns, RNS_B_LEFT_VEL_PID,  1.9125, 16.3928, 0.0);
	RNSSet(&rns, RNS_B_RIGHT_VEL_PID, 1.44, 17.28, 0.0);

//	RNSSet(&rns, RNS_F_LEFT_VEL_FUZZY_PID_BASE, 0.2, 0.2, 0.2);
//	RNSSet(&rns, RNS_F_LEFT_VEL_FUZZY_PID_PARAM, 0.02, 0.02, 0.02);

	RNSSet(&rns, RNS_PPInit); //Path Planning
	RNSSet(&rns, RNS_PPPathPID, 0.8, 0.5, 0.5);
	RNSSet(&rns, RNS_PPEndPID, 1.2, 0.1, 1.0);
	RNSSet(&rns, RNS_PPZPID, 1.0, 0.05, 0.2, 5.5);
	RNSSet(&rns, RNS_PPSetCRV_PTS, 10.0);         // Change No. of Points in the Curved Path
}


