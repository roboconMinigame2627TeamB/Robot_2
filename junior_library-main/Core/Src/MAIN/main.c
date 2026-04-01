/* =========================================================
 * Includes
 * ========================================================= */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "main.h"

/**
 * @brief  The application entry point.
 * @retval int
 */

/* =========================================================
 * Hardware Handles (defined by CubeMX)
 * ========================================================= */
extern TIM_HandleTypeDef  htim6;
extern UART_HandleTypeDef huart5;

/* =========================================================
 * Tunable Constants
 * ========================================================= */

// Platform
#define PLATFORM_SPEED      15000   // PWM for lift motor     (0–20000)
#define ROLLER_SPEED        15000   // PWM for platform roller (0–20000)

// Spearhead servos
#define SPEAR_GRIP_OPEN     30      // Servo 1: open angle
#define SPEAR_GRIP_CLOSE    150     // Servo 1: close angle
#define SPEAR_PITCH_UP      90      // Servo 2: raised
#define SPEAR_PITCH_FLAT    0       // Servo 2: flat

// KFS gripper motor PWM
#define KFS_GRIP_SPEED      1000    // PWM for open/close motor
#define KFS_ROTATE_SPEED    500     // PWM for rotation motor

// KFS rotation angle targets (degrees, from calculateKFSGripperAngle)
#define KFS_ANGLE_UP        -40.0f  // "raised" position
#define KFS_ANGLE_DOWN      -170.0f // "resting" position

// Drive
#define JOY_DEADZONE        0.10f   // Joystick dead-zone threshold
#define DRIVE_SCALE         1.75f   // Final velocity multiplier
#define IMU_DEADZONE_DEG    2.0f    // Min heading error to apply PID correction
#define SNAP_TURN_DEG       90.0f   // Degrees per d-pad snap turn

// Motor PWM Struct
typedef struct {
	int32_t BDC5_pwm;   // Platform rollers   (+fwd / -rev)
	int32_t BDC6_pwm;   // Platform lift       (+up  / -down)
	int32_t BDC7_pwm;   // KFS grip open/close (+open / -close)
	int32_t BDC8_pwm;   // KFS rotation        (+down / -up)
} MotorSpeeds_t;

typedef enum {
	STATE_INIT_OPEN,        // ensure gripper is open
	STATE_INIT_ROTATE_DOWN, // swing arm to intake position
	STATE_WAIT_FOR_BOX,     // wait for IR sensor
	STATE_CLOSE_GRIPPER,    // grip the box
	STATE_ROTATE_UP,        // lift arm with box
	STATE_DONE
} AutomationState_t;


//Servo Objects
SERVO_t Servo_SpearGrip;    // Servo 1: spearhead gripper
SERVO_t Servo_SpearPitch;   // Servo 2: spearhead pitch

/* =========================================================
 * PID Object
 * ========================================================= */
PID_t pid_rotate;   // IMU heading lock PID (runs in TIM6 ISR)

/* =========================================================
 * Variables
 * ========================================================= */

// Drive flags
volatile uint8_t imu_lock_flag = 0;    // 1 = heading held by PID
uint8_t p_lock_flag   = 0;    // 1 = perspective control mode
uint8_t A_lock_flag   = 0;    // 1 = camera alignment active
uint8_t is_path_planning = 0; // 1 = path planner has control

// IMU PID shared between vDriveTask and TIM6 ISR
volatile float error_val  = 0.0f;  // heading error  (input to PID)
volatile float w_pid_out  = 0.0f;  // PID output     (rotation correction)

// Heading target (written by vControllerTask, read by vDriveTask)
float target_angle = 0.0f;

// Camera alignment inputs (written by UART ISR, read by vAlignmentTask)
float Allign_x = 0.0f;
float Allign_y = 0.0f;
float Allign_w = 0.0f;

// KFS gripper flags (written by vControllerTask, read by vKFSGripperTask)
volatile uint8_t kfs_rotate_flag = 0;  // 1 = rotation move requested
volatile uint8_t kfs_grip_flag   = 0;  // 1 = grip move requested
volatile uint8_t KFS_angle_state = 0;  // 0 = rotate up,  1 = rotate down
volatile uint8_t KFS_grip_state  = 0;  // 0 = open,       1 = close
volatile float gripper_angle = 0.0f;

// Auto-intake flag
volatile uint8_t start_automation_flag = 0;

// Motor PWM struct + mutex
MotorSpeeds_t        motor_pwm   = {0, 0, 0, 0};
SemaphoreHandle_t    xMotorMutex = NULL;

// RNS bus mutex
SemaphoreHandle_t xRNSMutex = NULL;

// Debug print buffer
char buffer[128];

/* =========================================================
 * Encoder Tracking
 * ========================================================= */
static int32_t  previousEncoderValue  = 0;
static int32_t  totalAccumulatedSteps = 0;
static uint8_t  isInitialized         = 0;

/* =========================================================
 * Function Prototypes
 * ========================================================= */

// Helper functions
float angle_wrapper(float angle);
float calculateKFSGripperAngle(int32_t currentEncoderValue);

// Tasks
void vMotorControlTask (void *pvParameters);
void vControllerTask   (void *pvParameters);
void vDriveTask        (void *pvParameters);
void vKFSGripperTask   (void *pvParameters);
void vBoxIntakeTask    (void *pvParameters);

/* =========================================================
 * Helper Functions
 * ========================================================= */

// Folds any angle back into (-180, +180].
float angle_wrapper(float angle)
{
	while (angle >  180.0f) angle -= 360.0f;
	while (angle < -180.0f) angle += 360.0f;
	return angle;
}

// Converts raw QEI encoder counts into angle.
float calculateKFSGripperAngle(int32_t currentEncoderValue)
{
	if (!isInitialized) {
		previousEncoderValue = currentEncoderValue;
		isInitialized = 1;
		return 0.0f;
	}

	int32_t delta = currentEncoderValue - previousEncoderValue;
	if      (delta >  32768) delta -= 65536;    // wrapped forward
	else if (delta < -32768) delta += 65536;    // wrapped backward

	totalAccumulatedSteps -= delta;
	previousEncoderValue   = currentEncoderValue;

	return totalAccumulatedSteps * (180.0f / (65535.0f - 6314.0f));
}



//Priority high, ensures motors are always written promptly
void vMotorControlTask(void *pvParameters)
{
	MotorSpeeds_t local_pwm = {0, 0, 0, 0};

	for (;;) {
		if (xSemaphoreTake(xMotorMutex, portMAX_DELAY) == pdTRUE) {
			local_pwm = motor_pwm;
			xSemaphoreGive(xMotorMutex);
		}

		WriteBDC(&BDC5, local_pwm.BDC5_pwm);
		WriteBDC(&BDC6, local_pwm.BDC6_pwm);
		WriteBDC(&BDC7, local_pwm.BDC7_pwm);
		WriteBDC(&BDC8, local_pwm.BDC8_pwm);
		SHIFTREGShift(&SR);

		vTaskDelay(pdMS_TO_TICKS(20));
	}
}

// Priority medium Handles manual rotate and grip commands from vControllerTask.
void vKFSGripperTask(void *pvParameters)
{
	for (;;) {
		// Refresh shared angle
		gripper_angle = calculateKFSGripperAngle(QEIRead(QEI1));

		// Rotation control
		if (kfs_rotate_flag) {
			if (KFS_angle_state == 0) {         // rotate to "up" position
				if (gripper_angle >= KFS_ANGLE_UP) {
					if (xSemaphoreTake(xMotorMutex, portMAX_DELAY) == pdTRUE) {
						motor_pwm.BDC8_pwm = 0;
						xSemaphoreGive(xMotorMutex);
					}
					kfs_rotate_flag = 0;
				} else {
					if (xSemaphoreTake(xMotorMutex, portMAX_DELAY) == pdTRUE) {
						motor_pwm.BDC8_pwm = -KFS_ROTATE_SPEED;
						xSemaphoreGive(xMotorMutex);
					}
				}
			} else {                            // rotate to "down" position
				if (gripper_angle <= KFS_ANGLE_DOWN) {
					if (xSemaphoreTake(xMotorMutex, portMAX_DELAY) == pdTRUE) {
						motor_pwm.BDC8_pwm = 0;
						xSemaphoreGive(xMotorMutex);
					}
					kfs_rotate_flag = 0;
				} else {
					if (xSemaphoreTake(xMotorMutex, portMAX_DELAY) == pdTRUE) {
						motor_pwm.BDC8_pwm = KFS_ROTATE_SPEED;
						xSemaphoreGive(xMotorMutex);
					}
				}
			}
		}

		// Grip control
		if (kfs_grip_flag) {
			if (KFS_grip_state == 0) {          // open grip until limit switch
				if (IP2) {
					if (xSemaphoreTake(xMotorMutex, portMAX_DELAY) == pdTRUE) {
						motor_pwm.BDC7_pwm = KFS_GRIP_SPEED;
						xSemaphoreGive(xMotorMutex);
					}
				} else {
					if (xSemaphoreTake(xMotorMutex, portMAX_DELAY) == pdTRUE) {
						motor_pwm.BDC7_pwm = 0;
						xSemaphoreGive(xMotorMutex);
					}
					kfs_grip_flag = 0;
				}
			} else {                            // close grip until sensor
				if (lsfl2) {
					if (xSemaphoreTake(xMotorMutex, portMAX_DELAY) == pdTRUE) {
						motor_pwm.BDC7_pwm = -KFS_GRIP_SPEED;
						xSemaphoreGive(xMotorMutex);
					}
				} else {
					if (xSemaphoreTake(xMotorMutex, portMAX_DELAY) == pdTRUE) {
						motor_pwm.BDC7_pwm = 0;
						xSemaphoreGive(xMotorMutex);
					}
					kfs_grip_flag = 0;
				}
			}
		}

		vTaskDelay(pdMS_TO_TICKS(30));
	}
}


//Priority: medium - Automated box intake sequence, enabled by start_automation_flag flag.

void vBoxIntakeTask(void *pvParameters)
{
	AutomationState_t state = STATE_INIT_OPEN;
	uint8_t first_box = 0;

	for (;;) {
		if (start_automation_flag) {
			switch (state) {

			case STATE_INIT_OPEN:
				// Drive open until limit switch clears
				if (IP2) {
					if (xSemaphoreTake(xMotorMutex, portMAX_DELAY) == pdTRUE) {
						motor_pwm.BDC7_pwm = KFS_GRIP_SPEED;
						xSemaphoreGive(xMotorMutex);
					}
				} else {
					if (xSemaphoreTake(xMotorMutex, portMAX_DELAY) == pdTRUE) {
						motor_pwm.BDC7_pwm = 0;
						xSemaphoreGive(xMotorMutex);
					}
					state = STATE_INIT_ROTATE_DOWN;
				}
				break;

			case STATE_INIT_ROTATE_DOWN:
				// Swing arm down to intake angle
				if (gripper_angle > KFS_ANGLE_DOWN) {
					if (xSemaphoreTake(xMotorMutex, portMAX_DELAY) == pdTRUE) {
						motor_pwm.BDC8_pwm = KFS_ROTATE_SPEED;
						xSemaphoreGive(xMotorMutex);
					}
				} else {
					if (xSemaphoreTake(xMotorMutex, portMAX_DELAY) == pdTRUE) {
						motor_pwm.BDC8_pwm = 0;
						xSemaphoreGive(xMotorMutex);
					}
					state = STATE_WAIT_FOR_BOX;
				}
				break;

			case STATE_WAIT_FOR_BOX:
				if (lsf1) state = STATE_CLOSE_GRIPPER;
				break;

			case STATE_CLOSE_GRIPPER:
				if (lsfl2) {
					if (xSemaphoreTake(xMotorMutex, portMAX_DELAY) == pdTRUE) {
						motor_pwm.BDC7_pwm = -KFS_GRIP_SPEED;
						xSemaphoreGive(xMotorMutex);
					}
				} else {
					if (xSemaphoreTake(xMotorMutex, portMAX_DELAY) == pdTRUE) {
						motor_pwm.BDC7_pwm = 0;
						xSemaphoreGive(xMotorMutex);
					}
					state = first_box ? STATE_DONE : STATE_ROTATE_UP;
				}
				break;

			case STATE_ROTATE_UP:
				if (gripper_angle < KFS_ANGLE_UP) {
					if (xSemaphoreTake(xMotorMutex, portMAX_DELAY) == pdTRUE) {
						motor_pwm.BDC8_pwm = -KFS_ROTATE_SPEED;
						xSemaphoreGive(xMotorMutex);
					}
				} else {
					if (xSemaphoreTake(xMotorMutex, portMAX_DELAY) == pdTRUE) {
						motor_pwm.BDC8_pwm = 0;
						xSemaphoreGive(xMotorMutex);
					}
					first_box = 1;
					state = STATE_INIT_OPEN;
				}
				break;

			case STATE_DONE:
				break;
			}
		}

		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

// Priority: low, input polling, writes flags and motor_pwm
void vControllerTask(void *pvParameters)
{
	uint32_t prev_button    = 0;
	uint8_t  spear_grip_state = 1;   // 1 = closed, 0 = open

	for (;;) {
		ps4.button = ps4.buf1 | (ps4.buf2 << 8) | (ps4.buf3 << 16);

		int32_t req_roller = 0;
		int32_t req_lift   = 0;

		if (ps4.button & PS) {                                          // Emergency stop
			RNSStop(&rns);
			NVIC_SystemReset();
		}
		else if ((ps4.button & TRIANGLE) && !(prev_button & TRIANGLE)) // perspective control toggle
			p_lock_flag = !p_lock_flag;

		else if ((ps4.button & CROSS) && !(prev_button & CROSS))       // IMU lock toggle
			imu_lock_flag = !imu_lock_flag;

		else if ((ps4.button & SQUARE) && !(prev_button & SQUARE)) {   // KFS rotate
			kfs_rotate_flag  = 1;
			KFS_angle_state  = !KFS_angle_state;
		}
		else if ((ps4.button & CIRCLE) && !(prev_button & CIRCLE)) {   // KFS grip
			kfs_grip_flag  = 1;
			KFS_grip_state = !KFS_grip_state;
		}
		else if ((ps4.button & TOUCH) && !(prev_button & TOUCH)) {     // Spearhead grip
			spear_grip_state = !spear_grip_state;
			ServoSetAngle(&Servo_SpearGrip,
					spear_grip_state ? SPEAR_GRIP_CLOSE : SPEAR_GRIP_OPEN);
		}
		else if ((ps4.button & R3) && !(prev_button & R3))             // Auto-intake toggle
			start_automation_flag = !start_automation_flag;

		else if ((ps4.button & UP)   && !(prev_button & UP))           // Spear pitch up
			ServoSetAngle(&Servo_SpearPitch, SPEAR_PITCH_UP);

		else if ((ps4.button & DOWN) && !(prev_button & DOWN))         // Spear pitch flat
			ServoSetAngle(&Servo_SpearPitch, SPEAR_PITCH_FLAT);

		else if ((ps4.button & RIGHT) && !(prev_button & RIGHT)) {     // Snap-turn +90°
			if (!imu_lock_flag) {
				if (xSemaphoreTake(xRNSMutex, portMAX_DELAY) == pdTRUE) {
					RNSEnquire(RNS_ANGLE, &rns);
					xSemaphoreGive(xRNSMutex);
				}
				imu_lock_flag = 1;
				target_angle  = angle_wrapper(rns.RNS_data.common_buffer[0].data);
			}
			target_angle = angle_wrapper(floorf(target_angle / SNAP_TURN_DEG) * SNAP_TURN_DEG + SNAP_TURN_DEG);
		}
		else if ((ps4.button & LEFT) && !(prev_button & LEFT)) {       // Snap-turn -90°
			if (!imu_lock_flag) {
				if (xSemaphoreTake(xRNSMutex, portMAX_DELAY) == pdTRUE) {
					RNSEnquire(RNS_ANGLE, &rns);
					xSemaphoreGive(xRNSMutex);
				}
				imu_lock_flag = 1;
				target_angle  = angle_wrapper(rns.RNS_data.common_buffer[0].data);
			}
			target_angle = angle_wrapper(ceilf(target_angle / SNAP_TURN_DEG) * SNAP_TURN_DEG - SNAP_TURN_DEG);
		}

		//Level-triggered for roller
		if      (ps4.button & L1) req_roller =  ROLLER_SPEED;
		else if (ps4.button & L2) req_roller = -ROLLER_SPEED;
		else                      req_roller =  0;

		// Level-triggered for platform lift
		if (ps4.joyR_y > JOY_DEADZONE) {
			req_lift = IP9  ? 0 : PLATFORM_SPEED;   // IP9  = top limit switch
		} else if (ps4.joyR_y < -JOY_DEADZONE) {
			req_lift = IP10 ? 0 : -PLATFORM_SPEED;  // IP10 = bottom limit switch
		}

		if (xSemaphoreTake(xMotorMutex, portMAX_DELAY) == pdTRUE) {
			motor_pwm.BDC5_pwm = req_roller;
			motor_pwm.BDC6_pwm = req_lift;
			xSemaphoreGive(xMotorMutex);
		}

		prev_button = ps4.button;
		vTaskDelay(pdMS_TO_TICKS(20));
	}
}

//Priority: highest. omni wheels drive with optional IMU heading lock
void vDriveTask(void *pvParameters)
{
	float x = 0.0f, y = 0.0f, w = 0.0f;
	float cos_t = 1.0f, sin_t = 0.0f;
	uint8_t set_once = 1;

	TickType_t       xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency    = pdMS_TO_TICKS(10);

	for (;;) {
		if (xSemaphoreTake(xRNSMutex, portMAX_DELAY) == pdTRUE) {
			RNSEnquire(RNS_ANGLE, &rns);
			xSemaphoreGive(xRNSMutex);
		}
		float current_angle = angle_wrapper(rns.RNS_data.common_buffer[0].data);
		float rad = current_angle * (3.1415926f / 180.0f);
		sin_t = sinf(rad);
		cos_t = cosf(rad);

		if (set_once) {
			target_angle = current_angle;
			set_once = 0;
		}

		if (fabsf(ps4.joyL_x) > JOY_DEADZONE ||
				fabsf(ps4.joyL_y) > JOY_DEADZONE ||
				fabsf(ps4.joyR_x) > JOY_DEADZONE) {

			// perspective control rotation into robot frame
			float x_world = ps4.joyL_x;
			float y_world = ps4.joyL_y;
			x =  x_world * cos_t + y_world * sin_t;
			y = -x_world * sin_t + y_world * cos_t;
			w =  ps4.joyR_x;
		} else {
			x = 0.0f; y = 0.0f; w = 0.0f;
		}

		if (imu_lock_flag) {
			error_val = -1.0f * angle_wrapper(target_angle - current_angle);

			if (fabsf(ps4.joyR_x) > JOY_DEADZONE) {
				w = ps4.joyR_x;
				target_angle = current_angle;
			} else {
				w = (fabsf(error_val) > IMU_DEADZONE_DEG) ? w_pid_out : 0.0f;
			}
		}

		float motorA = y - x - w;
		float motorB = y + x + w;
		float motorC = y + x - w;
		float motorD = y - x + w;

		float max_val = fabsf(motorA);
		if (fabsf(motorB) > max_val) max_val = fabsf(motorB);
		if (fabsf(motorC) > max_val) max_val = fabsf(motorC);
		if (fabsf(motorD) > max_val) max_val = fabsf(motorD);

		if (max_val > 1.0f) {
			motorA /= max_val;
			motorB /= max_val;
			motorC /= max_val;
			motorD /= max_val;
		}

		if (xSemaphoreTake(xRNSMutex, portMAX_DELAY) == pdTRUE) {
			RNSVelocity(motorA * DRIVE_SCALE,
					motorB * DRIVE_SCALE,
					motorC * DRIVE_SCALE,
					motorD * DRIVE_SCALE,
					&rns);
			xSemaphoreGive(xRNSMutex);
		}

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

void vServoInitTask(void *pvParameters) {
	for(;;){

		if(!PB1){
			ServoSetAngle(&Servo_SpearGrip, 30);
		}
		else if(!PB2){
			ServoSetAngle(&Servo_SpearGrip, 70);
		}
		else if(!PB3){
			ServoSetAngle(&Servo_SpearPitch, 90);
		}
		else if(!IP2){
			ServoSetAngle(&Servo_SpearPitch, 0);
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

/* =========================================================
 * main
 * ========================================================= */
int main(void)
{
	set();
	PSxSlaveInit(&ps4, &hi2c1);

	xMotorMutex = xSemaphoreCreateMutex();
	xRNSMutex   = xSemaphoreCreateMutex();

	if (xMotorMutex == NULL || xRNSMutex == NULL) {
		Error_Handler();
	}

	xTaskCreate(vMotorControlTask, "MotorCtrl",   256, NULL, 4, NULL);
	xTaskCreate(vKFSGripperTask,   "KFSGripper",  512, NULL, 3, NULL);
	xTaskCreate(vBoxIntakeTask,    "BoxIntake",    512, NULL, 2, NULL);
	xTaskCreate(vControllerTask,   "Controller",  512, NULL, 2, NULL);
	xTaskCreate(vDriveTask,        "Drive",        512, NULL, 5, NULL);
	xTaskCreate(vTestTask,		   "TestTask",		512,NULL,1,NULL);
	xTaskCreate(vServoInitTask,    "ServoInitTask",512,NULL,1,NULL);

	vTaskStartScheduler();

	while (1) {}
}

void vTestTask(void *pvParameters) {
	for(;;){
		push1 = PB1;
		if(!PB1){
			if( xSemaphoreTake( xMotorMutex, portMAX_DELAY ) == pdTRUE ) {
				if(PB2) motor_pwm.BDC7_pwm = 1000;
				else motor_pwm.BDC7_pwm = 1000;
				xSemaphoreGive(xMotorMutex);
			}
		}
		else if(!IP2){
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


/* =========================================================
 * ISR: TIM6 — PID heading correction
 * Runs at fixed rate, updates w_pid_out used by vDriveTask
 * ========================================================= */
void TIM6_DAC_IRQHandler(void)
{
	if (imu_lock_flag) PID(&pid_rotate);
	HAL_TIM_IRQHandler(&htim6);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_Slave_Receive_IT(hi2c, (uint8_t *)ps4.ReceiveBuffer, 11);
}

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
