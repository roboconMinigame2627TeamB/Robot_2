

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "main.h"

/**
 * @brief  The application entry point.
 * @retval int
 */
SERVO_t Servo_SpearGrip, Servo_SpearPitch;

#define PLATFORM_SPEED          15000   // PWM for platform lift  (0–20000)
#define ROLLER_SPEED            18000   // PWM for KFS ejection   (0–20000)

#define SPEAR_GRIP_OPEN         30      // Servo 1: open angle  (degrees)
#define SPEAR_GRIP_CLOSE        150     // Servo 1: close angle (degrees)

#define SPEAR_PITCH_UP          90      // Servo 2: raised angle (degrees)
#define SPEAR_PITCH_FLAT        0       // Servo 2: flat angle   (degrees)

/* Function Prototypes for Tasks */
//void vReadEncoderTask(void *pvParameters);
void vMotorControlTask(void *vParameters);
//void vAllignmentTask(void *pvParameters);
//void vTelemetryTask(void *pvParameters);
void vControllerTask(void *pvParameters);
void vDriveTask(void *pvParameters);
void vKFSGripperTask(void *pvParameters);
void vBoxIntakeTask(void *vParameters);

//Function Prototypes
int32_t previousEncoderValue = 0;
int32_t totalAccumulatedSteps = 0;
uint8_t isInitialized = 0;
float calculateKFSGripperAngle(int32_t currentEncoderValue);
float angle_wrapper(float angle);

//Flags

volatile uint8_t imu_lock_flag = 1;
volatile uint8_t p_lock_flag = 0;
volatile uint8_t kfs_rotate_flag = 0; // 0 - nothing, 1 - move
volatile uint8_t kfs_grip_flag = 0; // 0 - nothing, 1 - move
volatile uint8_t KFS_angle_state = 0; // 0 - rotate up, 1 - rotate down
volatile uint8_t KFS_grip_state = 0; //0 open, 1 close
volatile uint8_t start_automation_flag = 0;
// Alignment variables
float target_angle = 0.0;
float Allign_x = 0.0;
float Allign_y = 0.0;
float Allign_w = 0.0;

//float a,b,c,d,x,y;
//char buffer[120];


//pwm variables
typedef struct {
	int32_t BDC5_pwm; //Platform rollers
	int32_t BDC6_pwm; //Platform lift
	int32_t BDC7_pwm; //KFS gripper open/close + open, - close
	int32_t BDC8_pwm; //KFS gripper rotation + down, - up
} MotorSpeeds_t;

MotorSpeeds_t motor_pwm = {0, 0, 0, 0};
SemaphoreHandle_t xMotorMutex = NULL;
SemaphoreHandle_t xRNSMutex = NULL;

float angle_wrapper(float angle) {
	while (angle > 180.0) angle -= 360.0;
	while (angle < -180.0) angle += 360.0;
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
//	/*
//	 * lsf1 is IR sensor at gripper
//	 * IP2 is limit switch for opening gripper
//	 */
//	uint8_t closed_flag = 0;
//	uint8_t rotate_flag = 0;
//	uint8_t return_flag = 0;
//	uint8_t start_opened = 0;
//	uint8_t start_0degree = 0;
//	uint8_t set_once = 0;
//	uint8_t set_once1 = 0;
//	uint8_t first_box = 1;
//	float gripper_angle= 0.0;
//
//	 //start at 180degree
//	for(;;) {
//		if (start_automation_flag){
//			gripper_angle = calculateKFSGripperAngle(QEIRead(QEI1));
//
//			if(IP2 && !start_opened) { // ensure gripper is opened
//				if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
//					motor_pwm.BDC7_pwm = 1000;
//					xSemaphoreGive(xMotorMutex);
//				}
//			} else {
//				if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
//					if (!set_once) {
//						motor_pwm.BDC7_pwm = 0;
//						set_once = 1;
//						start_opened = 1;
//						closed_flag = 0;
//					}
//					xSemaphoreGive(xMotorMutex);
//				}
//			}
//
//			if(gripper_angle > -180.0 && !start_0degree) { // ensure gripper is 0degree
//				if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
//					motor_pwm.BDC8_pwm = 500;
//					xSemaphoreGive(xMotorMutex);
//				}
//			} else {
//				if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
//					if (!set_once1) {
//						motor_pwm.BDC8_pwm = 0;
//						set_once1 = 1;
//						start_0degree = 1;
//					}
//					xSemaphoreGive(xMotorMutex);
//				}
//			}
//
//			if(lsf1 && start_opened && start_0degree) {  // IR detected
//
//				// STATE 1: Close Gripper
//				if (!closed_flag) {
//					if(lsfl2) {
//						if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
//							motor_pwm.BDC7_pwm = -1000;
//							xSemaphoreGive(xMotorMutex);
//						}
//					} else {
//						if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
//							motor_pwm.BDC7_pwm = 0;
//							xSemaphoreGive(xMotorMutex);
//						}
//						closed_flag = 1;
//						rotate_flag = 1;
//					}
//				}
//
//				// STATE 2: Rotate Gripper Upwards
//				else if (rotate_flag && first_box) {
//
//					if(gripper_angle < -40.0) { // Keep rotating up
//						if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
//							motor_pwm.BDC8_pwm = -500;
//							xSemaphoreGive(xMotorMutex);
//						}
//					} else {
//						if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
//							motor_pwm.BDC8_pwm = 0;
//							xSemaphoreGive(xMotorMutex);
//						}
//						return_flag = 1;
//						rotate_flag = 0;
//						start_opened = 0;
//						set_once = 0;
//					}
//				}
//
//				// STATE 3: Rotate Gripper Downwards
//				else if (return_flag && first_box) {
//					start_0degree = 0;
//					return_flag = 0;
//					first_box = 0;
//					set_once1 = 0;
//				}
//			}
//		}
//
//		vTaskDelay(pdMS_TO_TICKS(50));
//	}
//}


typedef enum {
    STATE_INIT_OPEN,
    STATE_INIT_ROTATE_DOWN,
    STATE_WAIT_FOR_BOX,
    STATE_CLOSE_GRIPPER,
    STATE_ROTATE_UP,
    STATE_DONE
} AutomationState_t;

void vBoxIntakeTask(void *vParameters) {
    AutomationState_t current_state = STATE_INIT_OPEN;
    float gripper_angle = 0.0;
    uint8_t first_box =0;

    for(;;) {
    	gripper_angle = calculateKFSGripperAngle(QEIRead(QEI1));
        if (start_automation_flag) {
            switch (current_state) {
                case STATE_INIT_OPEN:
                    if(IP2) {
                        if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
                            motor_pwm.BDC7_pwm = 1000;
                            xSemaphoreGive(xMotorMutex);
                        }
                    } else {
                        if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
                            motor_pwm.BDC7_pwm = 0;
                            xSemaphoreGive(xMotorMutex);
                        }
                        current_state = STATE_INIT_ROTATE_DOWN;
                    }
                    break;

                case STATE_INIT_ROTATE_DOWN:
                    if(gripper_angle > -170.0) {
                        if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
                            motor_pwm.BDC8_pwm = 200;
                            xSemaphoreGive(xMotorMutex);
                        }
                    } else {
                        if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
                            motor_pwm.BDC8_pwm = 0;
                            xSemaphoreGive(xMotorMutex);
                        }
                        current_state = STATE_WAIT_FOR_BOX; // Move to next step
                    }
                    break;


                case STATE_WAIT_FOR_BOX:
                    if(lsf1) {
                        current_state = STATE_CLOSE_GRIPPER;
                    }
                    break;

                case STATE_CLOSE_GRIPPER:
                    if(lsfl2) {
                        if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
                            motor_pwm.BDC7_pwm = -1000;
                            xSemaphoreGive(xMotorMutex);
                        }
                    } else {
                        if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
                            motor_pwm.BDC7_pwm = 0;
                            xSemaphoreGive(xMotorMutex);
                        }
                        if (!first_box) current_state = STATE_ROTATE_UP;
                        else current_state = STATE_DONE;
                    }
                    break;

                case STATE_ROTATE_UP:
                    if(gripper_angle < -40.0) {
                        if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
                            motor_pwm.BDC8_pwm = -500;
                            xSemaphoreGive(xMotorMutex);
                        }
                    } else {
                        if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
                            motor_pwm.BDC8_pwm = 0;
                            xSemaphoreGive(xMotorMutex);
                        }
                        first_box = 1;
                        current_state = STATE_INIT_OPEN;
                    }
                    break;

                case STATE_DONE:

                    break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
/*
void vAllignmentTask(void *pvParameters) { //for robot 1
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


void vTelemetryTask(void *pvParameters) {
	for(;;) {

		RNSEnquire(RNS_X_Y_IMU_LSA, &rns);
		float x = rns.RNS_data.common_buffer[1].data;
		float y = rns.RNS_data.common_buffer[2].data;
		sprintf(buffer, "%.2f,%.2f\r\n", x,y);
		UARTPrintString(&huart2, buffer);

		vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100ms
	}
}
 */



void vControllerTask(void *pvParameters) {

	uint16_t prev_button = 0;
	uint8_t spear_grip_state   = 1;  // 1 = grip is open, 0 = closed

	for(;;) {
		ps4.button = ps4.buf1 | (ps4.buf2 << 8) | (ps4.buf3 << 16);
		int32_t req_roller = 0;
		int32_t req_lift = 0;

		if (ps4.button & PS) { // Emergency Button
			RNSStop(&rns);
			NVIC_SystemReset(); }

		else if ((ps4.button & TRIANGLE) && !(prev_button & TRIANGLE)) p_lock_flag = !p_lock_flag;	// Toggle Per. Control

		else if ((ps4.button & CROSS) && !(prev_button & CROSS)) imu_lock_flag = !imu_lock_flag;	// Toggle IMU_LOCK

		else if ((ps4.button & SQUARE) && !(prev_button & SQUARE)) {	// Open and Close the Gripper SERVO 1
			kfs_rotate_flag = 1;
			KFS_angle_state = !KFS_angle_state;
		}

		else if ((ps4.button & CIRCLE) && !(prev_button & CIRCLE)) {	// Open and Close the Gripper SERVO 1
			kfs_grip_flag = 1;
			KFS_grip_state = !KFS_grip_state;
		}

		else if ((ps4.button & TOUCH) && !(prev_button & TOUCH)) {	// Open and Close the Gripper SERVO 1
			spear_grip_state = !spear_grip_state;
			if (spear_grip_state) ServoSetAngle(&Servo_SpearGrip, SPEAR_GRIP_CLOSE);
			else                  ServoSetAngle(&Servo_SpearGrip, SPEAR_GRIP_OPEN);
		}
		else if ((ps4.button & R3) && !(prev_button & R3)) {	// Open and Close the Gripper SERVO 1
			start_automation_flag = !start_automation_flag;
		}

		//SPEARHEAD SERVO 2 — PITCH  (Arrow UP / DOWN)
		else if ((ps4.button & UP) && !(prev_button & UP)) ServoSetAngle(&Servo_SpearPitch, SPEAR_PITCH_UP);

		else if ((ps4.button & DOWN) && !(prev_button & DOWN)) ServoSetAngle(&Servo_SpearPitch, SPEAR_PITCH_FLAT);

		else if ((ps4.button & RIGHT) && !(prev_button & RIGHT)) {
			if (!imu_lock_flag) {
				imu_lock_flag = 1;
				if( xSemaphoreTake( xRNSMutex, portMAX_DELAY ) == pdTRUE ) {
					RNSEnquire(RNS_ANGLE, &rns);
					xSemaphoreGive(xRNSMutex);
				}

				target_angle = angle_wrapper(rns.RNS_data.common_buffer[0].data);
			}
			target_angle = angle_wrapper(floor(target_angle / 90.0) * 90.0 + 90.0);
		}

		else if ((ps4.button & LEFT) && !(prev_button & LEFT)) {
			if (!imu_lock_flag) {
				imu_lock_flag = 1;
				if( xSemaphoreTake( xRNSMutex, portMAX_DELAY ) == pdTRUE ) {
					RNSEnquire(RNS_ANGLE, &rns);
					xSemaphoreGive(xRNSMutex);
				}
				target_angle = angle_wrapper(rns.RNS_data.common_buffer[0].data);
			}
			target_angle = angle_wrapper(ceil(target_angle / 90.0) * 90.0 - 90.0);
		}

		if (ps4.button & L1) req_roller = ROLLER_SPEED;
		else req_roller = 0;

		if (ps4.button & L2) req_roller = -ROLLER_SPEED;
		else req_roller = 0;

		if (ps4.joyR_y > 0.15f) {
			if (!IP9) req_lift = PLATFORM_SPEED; // Top Limit Switch
			else req_lift = 0;
		}
		else if (ps4.joyR_y < -0.15f) {
			//  IP10 is Bottom Limit Switch
			if (!IP10) req_lift = -PLATFORM_SPEED;

			else req_lift = 0;
		}

		if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
			motor_pwm.BDC5_pwm = req_roller;
			motor_pwm.BDC6_pwm = req_lift;
			xSemaphoreGive(xMotorMutex );
		}

		prev_button = ps4.button; //for debounce
		vTaskDelay(pdMS_TO_TICKS(20));
	}
}

void vKFSGripperTask(void *pvParameters) {
	float gripper_angle;
	for(;;){
		gripper_angle = calculateKFSGripperAngle(QEIRead(QEI1));
		if(kfs_rotate_flag) {
			if (!KFS_angle_state) {

				if(gripper_angle >= -40.0){
					if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) { // stop rotating gripper
						motor_pwm.BDC8_pwm = 0;
						xSemaphoreGive(xMotorMutex);
					}
					kfs_rotate_flag = 0;
				} else{
					if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) { // rotate gripper upwards
						motor_pwm.BDC8_pwm = -500;
						xSemaphoreGive(xMotorMutex);
					}
				}
			} else{
				if(gripper_angle <= -180.0 ){
					if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) { // stop rotating gripper
						motor_pwm.BDC8_pwm = 0;
						xSemaphoreGive(xMotorMutex);
					}
					kfs_rotate_flag = 0;
				} else{
					if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) { // rotate gripper downwards
						motor_pwm.BDC8_pwm = 500;
						xSemaphoreGive(xMotorMutex);
					}
				}
			}
		}

		if(kfs_grip_flag) {
			if (!KFS_grip_state) { //open grip

				if(IP2) {
					if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
						motor_pwm.BDC7_pwm = 1000;
						xSemaphoreGive(xMotorMutex);
					}
				} else {
					if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
						motor_pwm.BDC7_pwm = 0;
						xSemaphoreGive(xMotorMutex);
					}
					kfs_grip_flag = 0;
				}

			} else { //close grip

				if(!lsfl2){
					if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) { //stop motor
						motor_pwm.BDC7_pwm = 0;
						xSemaphoreGive(xMotorMutex);
					}
					kfs_grip_flag = 0;
				} else{
					if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) { // close gripper
						motor_pwm.BDC7_pwm = -1000;
						xSemaphoreGive(xMotorMutex);
					}
				}
			}
		}
		vTaskDelay(pdMS_TO_TICKS(30));
	}
}


void vDriveTask(void *pvParameters) {
	float x = 0.0, y = 0.0, w = 0.0;
	uint8_t set_once = 1;
	float cos_t, sin_t;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(10);

	for(;;) {

		if (fabs(ps4.joyL_x) > 0.1 || fabs(ps4.joyL_y) > 0.1 || fabs(ps4.joyR_x) > 0.1) {
			x = ps4.joyL_x; y = ps4.joyL_y; w = ps4.joyR_x;
		} else {
			x = 0.0; y = 0.0; w =0.0;
		}
		if( xSemaphoreTake( xRNSMutex, portMAX_DELAY ) == pdTRUE ) {
			RNSEnquire(RNS_ANGLE, &rns);
			xSemaphoreGive(xRNSMutex);
		}
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
			error_val = -1 * angle_wrapper(target_angle - current_angle);

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
		if( xSemaphoreTake( xRNSMutex, portMAX_DELAY ) == pdTRUE ) {
			RNSVelocity(motorA * final_scale,
							motorB * final_scale,
							motorC * final_scale,
							motorD * final_scale,
							&rns);
			xSemaphoreGive(xRNSMutex);
		}
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}


}


float calculateKFSGripperAngle(int32_t currentEncoderValue) {
    if (isInitialized == 0) {
        previousEncoderValue = currentEncoderValue;
        isInitialized = 1;
    }

    int32_t delta = (int32_t)currentEncoderValue - (int32_t)previousEncoderValue;

    if (delta > 32768) {
        delta -= 65536;
    } else if (delta < -32768) {
        delta += 65536;
    }
    totalAccumulatedSteps -= delta;
    previousEncoderValue = currentEncoderValue;
    float angle = totalAccumulatedSteps * (180.0f / (65535.0f - 6314.0f)); //only tune the negative value which is encoder val when 180 degree

    return angle;
}

void vTestTask(void *pvParameters) {
	for(;;){
		if(!PB1){
			if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
				if(IP2) motor_pwm.BDC7_pwm = 1000;
				else motor_pwm.BDC7_pwm = 1000;
				xSemaphoreGive(xMotorMutex);
			}
		}
		else if(!PB2){
			if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
				motor_pwm.BDC7_pwm = -1000;
				xSemaphoreGive(xMotorMutex);
			}
		}
		else if(!PB3){
			if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
				motor_pwm.BDC6_pwm = 15000;
				xSemaphoreGive(xMotorMutex);
			}
		}
		else{
			if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
			motor_pwm.BDC8_pwm = 0;
			motor_pwm.BDC7_pwm = 0;
			motor_pwm.BDC6_pwm = 0;
			xSemaphoreGive(xMotorMutex);
			}
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}



int main(void)
{
	set();
	xRNSMutex   = xSemaphoreCreateMutex();
	xMotorMutex = xSemaphoreCreateMutex();

	if (xMotorMutex != NULL && xRNSMutex != NULL) {
		xTaskCreate(
				vMotorControlTask,
				"MotorControlTask",
				256,
				NULL,
				3,
				NULL
		);
		xTaskCreate(
				vBoxIntakeTask,
				"MotorControlTask",
				1024,
				NULL,
				2,
				NULL
		);

		xTaskCreate(
				vKFSGripperTask,
				"KFSGripperTask",
				512,
				NULL,
				2,
				NULL
		);

		xTaskCreate(
				vControllerTask,
				"ControlTask",
				512,
				NULL,
				2,
				NULL
		);

		xTaskCreate(
				vDriveTask,
				"DriveTask",
				512,
				NULL,
				4,
				NULL
		);
//		xTaskCreate(
//				vTestTask,
//				"TestTask",
//				512,
//				NULL,
//				1,
//				NULL
//		);
	}

	vTaskStartScheduler();

	while(1){}
}

void TIM6_DAC_IRQHandler(void)
{

	if (imu_lock_flag) {
//		led1 = !led1;
		PID(&pid_rotate);
	}

	HAL_TIM_IRQHandler(&htim6);
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if (huart->Instance == UART5)
//	{
//
//		if (uart5_rx == '\n' || uart5_rx == '\r') {
//			if (rx_idx > 0) {
//				rx_buf[rx_idx] = '\0';
//				sscanf(rx_buf, "%f,%f,%f", &Allign_x, &Allign_y,&Allign_w);
//
//				rx_idx = 0;
//			}
//		}
//		else {
//
//			if (rx_idx < 63) {
//				rx_buf[rx_idx++] = uart5_rx;
//			}
//		}
//
//
//		HAL_UART_Receive_IT(&huart5, &uart5_rx, 1);
//	}
//}
/**
 * @brief  This function is executed in case of error occurrence.
 */

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
