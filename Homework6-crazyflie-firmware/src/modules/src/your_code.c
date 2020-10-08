#include "your_code.h"

//including from "Counter assignment"
//#include <stdio.h>
//#include <stdlib.h>

//#include "FreeRTOS.h"
//#include "task.h"
#include "semphr.h"
/***
 *
 * This file is where you should add you tasks. You already know the structure
 * Required to do so from the work with the simulator.
 *
 * The function yourCodeInit() is set to automatically execute when the
 * quadrotor is started. This is where you need to create your tasks. The
 * scheduler that runs the tasks is already up and running so you should
 * NOT make a call to vTaskStartScheduler();.
 *
 * Below that you can find a few examples of useful function calls and code snippets.
 *
 * For further reference on how this is done. Look into the file stabilizer.c
 * which is usually handles the control of the crazyflie.
 *
 ***/
#define TASK1_PRI     3
#define TASK2_PRI     1
#define TASK3_PRI     2
#define TASK4_PRI     2 //Music
#define TASK1_STACKSIZE     (3 * configMINIMAL_STACK_SIZE)
#define TASK2_STACKSIZE     (3 * configMINIMAL_STACK_SIZE)
#define TASK3_STACKSIZE     (3 * configMINIMAL_STACK_SIZE)
#define TASK4_STACKSIZE     (3 * configMINIMAL_STACK_SIZE)
#define TASK1_NAME    "Task1Name"
#define TASK2_NAME    "Task2Name"
#define TASK3_NAME    "Task3Name"
#define TASK4_NAME    "Task4Name"

//xTaskHandle Task1handle;
//xTaskHandle Task2handle;
//xTaskHandle Task3handle;

SemaphoreHandle_t Sem1; 
SemaphoreHandle_t SemGyro;
SemaphoreHandle_t SemPR;


//xSemaphoreTake(Semx, portMAX_DELAY);
//xSemaphoreGive(Semx);

// Constants
const double PI = 3.14159265358979323846;
const double const_gamma = 0.98;
const double dt = 0.01;
const double thrust_conversion = 445368.671423717;
const double angle_conversion = 0.0174532925199433;
//const double angle_conversion = 0.1;


// Def something like pointers to input thingys
double Base_Thrust = 0;
double Ref_Roll = 0;
double Ref_Pitch = 0;
double Ref_YawRate = 0;


// Def something pointers something output thingys

double Motor_1_value = 0;
double Motor_2_value = 0;
double Motor_3_value = 0;
double Motor_4_value = 0;


/*double Log1 = 0;
double Log2 = 0;
double Log3 = 0;
double Log4 = 0;
double Log5 = 0;
double Log6 = 0;
*/

double roll_prev = 0;
double pitch_prev = 0;
double roll = 0;
double pitch = 0;
double Acc_x = 0;
double Acc_y = 0;
double Acc_z = 0;
double Gyro_x = 0;
double Gyro_y = 0;
double Gyro_z = 0;

double acc_rp_roll = 0;
double acc_rp_pitch = 0;

double x_n = 0;
double y_n = 0;
double z_n = 0;

double measured_roll = 0;
double measured_pitch = 0;

// unsigned 16 - bit integer
uint16_t u_optimal_M1 = 0;
uint16_t u_optimal_M2 = 0;
uint16_t u_optimal_M3 = 0;
uint16_t u_optimal_M4 = 0;


double LQR_K1 = 0.0248;
double LQR_K2 = 0.0054;
double LQR_K3 = 0.0263;


double LQR_11 = -LQR_K1;
double LQR_12 = -LQR_K1;
double LQR_13 = -LQR_K2;
double LQR_14 = -LQR_K2;
double LQR_15 = -LQR_K3;

double LQR_21 = -LQR_K1;
double LQR_22 = LQR_K1;
double LQR_23 = -LQR_K2;
double LQR_24 = LQR_K2;
double LQR_25 = LQR_K3;

double LQR_31 = LQR_K1;
double LQR_32 = LQR_K1;
double LQR_33 = LQR_K2;
double LQR_34 = LQR_K2;
double LQR_35 = -LQR_K3;

double LQR_41 = LQR_K1;
double LQR_42 = -LQR_K1;
double LQR_43 = LQR_K2;
double LQR_44 = -LQR_K2;
double LQR_45 = LQR_K3;

uint32_t tick;

/*************************************************
 * RETRIEVE THE MOST RECENT SENSOR DATA
 *
 * The code creates a variable called sensorData and then calls a function
 * that fills this variable with the latest data from the sensors.
 */
/*
 static struct { //sensorData_t 
      Axis3f acc;
      Axis3f gyro;
      Axis3f mag;
      baro_t baro;
      zDistance_t zrange;
      point_t position;
 }sensorData;*/
/*
typedef struct sensorData_s {
  Axis3f acc;               // Gs
  Axis3f gyro;              // deg/s
  Axis3f mag;               // gauss
  baro_t baro;
  zDistance_t zrange;
  point_t position;         // m
#ifdef LOG_SEC_IMU
  Axis3f accSec;            // Gs
  Axis3f gyroSec;           // deg/s
#endif
  uint64_t interruptTimestamp;
} sensorData_t;*/
 
static sensorData_t sensorData;
// sensorsAcquire(&sensorData);


/*************************************************
 * RETRIEVE THE SET POINT FROM ANY EXTERNAL COMMAND INTERFACE
 *
 * The code creates a variable called setpoint and then calls a function
 * that fills this variable with the latest command input.
 */
 /*static struct {//setpoint_t
      uint32_t timestamp;
 
      attitude_t attitude;      // deg
      attitude_t attitudeRate;  // deg/s
      quaternion_t attitudeQuaternion;
      double thrust;
      point_t position;         // m
      velocity_t velocity;      // m/s
      acc_t acceleration;       // m/s^2
      bool velocity_body;       // true if velocity is given in body frame; false if velocity is given in world frame
 
      struct {
          stab_mode_t x;
          stab_mode_t y;
          stab_mode_t z;
          stab_mode_t roll;
          stab_mode_t pitch;
          stab_mode_t yaw;
          stab_mode_t quat;
      } mode;
  }setpoint;
*/
/*
typedef struct setpoint_s {
  uint32_t timestamp;

  attitude_t attitude;      // deg
  attitude_t attitudeRate;  // deg/s
  quaternion_t attitudeQuaternion;
  double thrust;
  point_t position;         // m
  velocity_t velocity;      // m/s
  acc_t acceleration;       // m/s^2
  bool velocity_body;       // true if velocity is given in body frame; false if velocity is given in world frame

  struct {
    stab_mode_t x;
    stab_mode_t y;
    stab_mode_t z;
    stab_mode_t roll;
    stab_mode_t pitch;
    stab_mode_t yaw;
    stab_mode_t quat;
  } mode;
} setpoint_t;*/

 /*
 ************************************************/
static setpoint_t setpoint;
// commanderGetSetpoint(&setpoint);

static state_t state;


//for task 2
static void Task2(void){
const TickType_t xDelayTask2 = 10 / portTICK_PERIOD_MS;
	while(true){	
			xSemaphoreTake(Sem1, portMAX_DELAY);
				commanderGetSetpoint(&setpoint, &state);
			xSemaphoreGive(Sem1);
			vTaskDelay(xDelayTask2);
	}
}


//for task 4
/*
static void Task4(void){
const TickType_t xDelayTask4 = 10 / portTICK_PERIOD_MS;
	while(true){	
			uint32_t counter=0;
			
			melodyplayer(uint32_t counter, uint32_t * mi, Melody * m)
			vTaskDelay(xDelayTask2);
	}
}*/


static void Task3(void)
{
const TickType_t xDelayTask3 = 10 / portTICK_PERIOD_MS;
	// From setpoint
	double Base_Thrust_local = 0;
	double Ref_Roll_local = 0;
	double Ref_Pitch_local = 0;
	//double Ref_YawRate_local = 0;
	// From comp. filt
	//double YawRate_local = 0;
	double pitch_local = 0;
	double roll_local = 0;

	double Gyro_x_local = 0;
	double Gyro_y_local = 0;
	double Gyro_z_local = 0;


	while (1)
	{
		//semtake
		xSemaphoreTake(Sem1, portMAX_DELAY);
			Base_Thrust_local = setpoint.thrust;
			Ref_Roll_local = setpoint.attitude.roll;
			Ref_Pitch_local = setpoint.attitude.pitch;
		xSemaphoreGive(Sem1);

		//semtake
			xSemaphoreTake(SemPR, portMAX_DELAY);
		pitch_local = pitch;
		roll_local = roll;
			xSemaphoreGive(SemPR);
		
		xSemaphoreTake(SemGyro, portMAX_DELAY);
			Gyro_x_local = Gyro_x;
			Gyro_y_local = Gyro_y;
			Gyro_z_local = Gyro_z;
		xSemaphoreGive(SemGyro);

		// Ref_YawRate_local = setpoint.mode.yaw;
		// NOTICE DEFINITION OF u_optimal_MX
		// Compute input, Note that basethrust is added
		Motor_1_value = -angle_conversion * thrust_conversion * (Gyro_x_local * LQR_13 - LQR_12 * (Ref_Pitch_local - pitch_local) - LQR_11 * (Ref_Roll_local - roll_local) + Gyro_y_local * LQR_14 + Gyro_z_local * LQR_15);
		Motor_2_value = -angle_conversion * thrust_conversion * (Gyro_x_local * LQR_23 - LQR_22 * (Ref_Pitch_local - pitch_local) - LQR_21 * (Ref_Roll_local - roll_local) + Gyro_y_local * LQR_24 + Gyro_z_local * LQR_25);
		Motor_3_value = -angle_conversion * thrust_conversion * (Gyro_x_local * LQR_33 - LQR_32 * (Ref_Pitch_local - pitch_local) - LQR_31 * (Ref_Roll_local - roll_local) + Gyro_y_local * LQR_34 + Gyro_z_local * LQR_35);
		Motor_4_value = -angle_conversion * thrust_conversion * (Gyro_x_local * LQR_43 - LQR_42 * (Ref_Pitch_local - pitch_local) - LQR_41 * (Ref_Roll_local - roll_local) + Gyro_y_local * LQR_44 + Gyro_z_local * LQR_45);
		
		Motor_1_value = Motor_1_value + Base_Thrust_local;
		Motor_2_value = Motor_2_value + Base_Thrust_local;
		Motor_3_value = Motor_3_value + Base_Thrust_local;
		Motor_4_value = Motor_4_value + Base_Thrust_local;
		
		if (Motor_1_value>65535){
			Motor_1_value=65535;
		}
		if (Motor_1_value<0){
			Motor_1_value=0;
		}		
		if (Motor_2_value>65535){
			Motor_2_value=65535;
		}		
		if (Motor_2_value<0){
			Motor_2_value=0;
		}		
		if (Motor_3_value>65535){
			Motor_3_value=65535;
		}		
		if (Motor_3_value<0){
			Motor_3_value=0;
		}		
		if (Motor_3_value>65535){
			Motor_3_value=65535;
		}		
		if (Motor_3_value<0){
			Motor_3_value=0;
		}
		
		//(uint16_t) implicit conversion to uint_16
		u_optimal_M1=Motor_1_value;
		u_optimal_M2=Motor_2_value;
		u_optimal_M3=Motor_3_value;
		u_optimal_M4=Motor_4_value;
		
			
		/*************************************************
		* SENDING OUTPUT TO THE MOTORS
		*
		* The code sends an output to each motor. The output should have the be
		* of the typ unsigned 16-bit integer, i.e. use variables such as:
		* uint16_t value_i
		*
		************************************************/

		motorsSetRatio(MOTOR_M1, u_optimal_M1);
		motorsSetRatio(MOTOR_M2, u_optimal_M2);
		motorsSetRatio(MOTOR_M3, u_optimal_M3);
		motorsSetRatio(MOTOR_M4, u_optimal_M4);
		vTaskDelay(xDelayTask3);
	}
}


static void task1_input(void) {
  sensorsAcquire(&sensorData, tick);
  Acc_x = sensorData.acc.x; // NOT SURE IF ARRAY OR NOT
  Acc_y = sensorData.acc.y;
  Acc_z = sensorData.acc.z;
  xSemaphoreTake(SemGyro, portMAX_DELAY);
  	Gyro_x = sensorData.gyro.x;
  	Gyro_y = sensorData.gyro.y;
  	Gyro_z = sensorData.gyro.z;
  xSemaphoreGive(SemGyro);
}

static void accelerometer_To_Angle(void) {
	if (Acc_x == 0 && Acc_y == 0 && Acc_z == 0) {
		acc_rp_roll = 0;
		acc_rp_pitch = 0;
	}
	else {
		y_n = Acc_y/sqrt(pow(Acc_x,2) + pow(Acc_y,2) + pow(Acc_z,2));
		z_n = Acc_z/sqrt(pow(Acc_x,2) + pow(Acc_y,2) + pow(Acc_z,2));
		x_n = Acc_x/sqrt(pow(Acc_x,2) + pow(Acc_y,2) + pow(Acc_z,2));
		acc_rp_roll=atan2(y_n,z_n)*180/PI;
		acc_rp_pitch = atan2(-x_n,sqrt(pow(y_n,2)+pow(z_n,2)))*180/PI;
	}
}

static void complementary_Filter(void) {
xSemaphoreTake(SemGyro, portMAX_DELAY);
	measured_roll = (const_gamma)*(roll_prev+Gyro_x*dt)+(1-const_gamma)*acc_rp_roll;
	measured_pitch = (const_gamma)*(pitch_prev+Gyro_y*dt)+(1-const_gamma)*acc_rp_pitch;
xSemaphoreGive(SemGyro);
}

static void task1_output(void) {
xSemaphoreTake(SemPR, portMAX_DELAY);
  roll = measured_roll;
  pitch = measured_pitch;
xSemaphoreGive(SemPR);
}

static void Task1(void) {
	const TickType_t xDelayTask1 = 10 / portTICK_PERIOD_MS;
	tick=1;

	for (int m = 0; m < NBR_OF_MOTORS; m++)
      {
        motorsBeep(m, true, testsound[m], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20);
        vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
        motorsBeep(m, false, 0, 0);
        vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
      }

	uint32_t lastWakeTime;
	lastWakeTime = xTaskGetTickCount ();
	while(!sensorsAreCalibrated()) {
   		vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
	}

	for (int m = 0; m < NBR_OF_MOTORS; m++)
      {
        motorsBeep(m, true, testsound[m], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20);
        vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
        motorsBeep(m, false, 0, 0);
        vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
      }

  while (1) {
    // take SEMAPHORE
    task1_input();
    // give SEMAPHORE
    roll_prev = roll;
    pitch_prev = pitch;
    accelerometer_To_Angle();
    complementary_Filter();
    // take SEMAPHORE
    task1_output();
    // give SEMAPHORE
    vTaskDelay(xDelayTask1);
    tick++;
  }
}

//static bool isInit;

void yourCodeInit(void)
{
  /*
   * CREATE AND EXECUTE YOUR TASKS FROM HERE
   */

	//vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

	//sensorsInit();
	//Wait for the system to be fully started to start stabilization loop
  	//systemWaitStart();

	Sem1 = xSemaphoreCreateBinary();
	xSemaphoreGive(Sem1);
	SemGyro = xSemaphoreCreateBinary();
	xSemaphoreGive(SemGyro);
	SemPR = xSemaphoreCreateBinary();
	xSemaphoreGive(SemPR);

  //starting the three tasks with different priorities
	xTaskCreate(Task2, TASK2_NAME, TASK2_STACKSIZE, NULL, TASK2_PRI, NULL);
	xTaskCreate(Task3, TASK3_NAME, TASK3_STACKSIZE, NULL, TASK3_PRI, NULL);
	xTaskCreate(Task1, TASK1_NAME, TASK1_STACKSIZE, NULL, TASK1_PRI, NULL);
	
}


/*************************************************
 * LOGGING VALUES THAT CAN BE PLOTTEN IN PYTHON CLIENT
 *
 * We have already set up three log blocks to for the accelerometer data, the
 * gyro data and the setpoints, just uncomment the block to start logging. Use
 * them as reference if you want to add custom blocks.
 *
 ************************************************/


LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)


LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)


LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

/*
LOG_GROUP_START(motor_mycode)
LOG_ADD(LOG_FLOAT, u_optimal_M1, &u_optimal_M1)
LOG_ADD(LOG_FLOAT, u_optimal_M2, &u_optimal_M2)
LOG_ADD(LOG_FLOAT, u_optimal_M3, &u_optimal_M3)
LOG_ADD(LOG_FLOAT, u_optimal_M4, &u_optimal_M4)
LOG_GROUP_STOP(motor_mycode)
*/

