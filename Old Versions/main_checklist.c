/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ECHO_Port 	GPIOC
#define ECHO_Pin	GPIO_PIN_7
#define TRIG_Port	GPIOC
#define TRIG_Pin	GPIO_PIN_8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ShowTask */
osThreadId_t ShowTaskHandle;
const osThreadAttr_t ShowTask_attributes = {
  .name = "ShowTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ServoTask */
osThreadId_t ServoTaskHandle;
const osThreadAttr_t ServoTask_attributes = {
  .name = "ServoTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EncoderBTask */
osThreadId_t EncoderBTaskHandle;
const osThreadAttr_t EncoderBTask_attributes = {
  .name = "EncoderBTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UltrasonicSenso */
osThreadId_t UltrasonicSensoHandle;
const osThreadAttr_t UltrasonicSenso_attributes = {
  .name = "UltrasonicSenso",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IRDistTask */
osThreadId_t IRDistTaskHandle;
const osThreadAttr_t IRDistTask_attributes = {
  .name = "IRDistTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for transmitTask */
osThreadId_t transmitTaskHandle;
const osThreadAttr_t transmitTask_attributes = {
  .name = "transmitTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for encoderDiffTask */
osThreadId_t encoderDiffTaskHandle;
const osThreadAttr_t encoderDiffTask_attributes = {
  .name = "encoderDiffTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for turningTask */
osThreadId_t turningTaskHandle;
const osThreadAttr_t turningTask_attributes = {
  .name = "turningTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM14_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void show(void *argument);
void motors(void *argument);
void encoder(void *argument);
void servo(void *argument);
void encoder_B(void *argument);
void ultra_sensor(void *argument);
void sensor_reading(void *argument);
void ir_dist(void *argument);
void transmit(void *argument);
void encoder_diff(void *argument);
void turning_task(void *argument);

/* USER CODE BEGIN PFP */
void MotorDrive_enable(void){
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
}

void MotorStopA(void){
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
}

void Motor_directionA(uint8_t forward) {
	if (forward){// move forward
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0); // set IN1 to maximum PWM (7199) for '1'
	  }
	else { // reverse
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0); // set IN2 to maximum PWM (7199) for '1'
	}
}

void Motor_forwardA(int  pwmVal){
	 // rotate motor in clockwise forward send the values to serial port for display
	 Motor_directionA(1); //forward
	 __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,pwmVal); // output PWM waveform to drive motor A

}

void Motor_reverseA(int  pwmVal){
	 // move robot forward send the values to serial port for display
	 Motor_directionA(0); //reverse
	 __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,pwmVal); // output PWM waveform to drive motor A

}

//--------------------------------------------------------------------------------
void MotorStopB(void){
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, 0);
}

void Motor_directionB(uint8_t forward) {
	if (forward){// move forward
		__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1,0); // set IN1 to maximum PWM (7199) for '1'
	  }
	else { // reverse
		__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,0); // set IN2 to maximum PWM (7199) for '1'
	}
}

void Motor_forwardB(int  pwmVal){
	 // rotate motor in clockwise forward send the values to serial port for display
	 Motor_directionB(1); //forward
	 __HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,pwmVal); // output PWM waveform to drive motor A

}

void Motor_reverseB(int  pwmVal){
	 // move robot forward send the values to serial port for display
	 Motor_directionB(0); //reverse
	 __HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1,pwmVal); // output PWM waveform to drive motor A

}
//--------------------------------------------------------------------------------
void MotorStop_both(void){
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, 0);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
}

void Motor_direction_both(uint8_t forward) {
	if (forward){// move forward
		__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1,0); // set IN1 to maximum PWM (7199) for '1'
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0);
	  }
	else { // reverse
		__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,0); // set IN2 to maximum PWM (7199) for '1'
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0);
	}
}

void Motor_forward_both(int  pwmVal){
	 // rotate motor in clockwise forward send the values to serial port for display
	 Motor_direction_both(1); //forward
	 __HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,pwmVal); // output PWM waveform to drive motor A
	 __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,pwmVal);
}

void Motor_reverse_both(int  pwmVal){
	 // move robot forward send the values to serial port for display
	 Motor_direction_both(0); //reverse
	 __HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1,pwmVal); // output PWM waveform to drive motor A
	 __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,pwmVal);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//-------- IR sensor ----------
float ir_dist_left = 0;
float ir_dist_right = 0;
float ir_dist_left_prev = 0.0f;
float ir_dist_right_prev = 0.0f;
float ir_dist_left_lpf = 0.0f;
float ir_dist_right_lpf = 0.0f;
uint16_t ir_raw_left = 0;
uint16_t ir_raw_right = 0;
int ir_dist_right_int = 0;
int ir_dist_left_int = 0;
//----------------------------


//------ Gyrometer ------
int16_t gyro_z_raw = 0;
float gyro_z_dps = 0.0f;
int gyro_z_dps_int = 0;
float gyro_z_dps_prev = 0.0f;
float angle_direction = 0.0f;
int angle_direction_int = 0;
float prev_angle_direction = 0.0f;

float angle_direction_hpf = 0.0f;
float angle_direction_hpf_prev = 0.0f;
int angle_direction_hpf_int = 0;
//-----------------------

//--- Ultrasonic Sensor ---
int us_echo = 0;
float us_dist = 0.0f;
int tc1 = 0;
int tc2 = 0;
float us_dist_prev = 0.0f;
float us_dist_lpf = 0.0f;
int us_dist_int = 0;
//--------------------------


//------- Motor ------------
#define RPM_OFFSET 0.0f
int motor_left_encoder = 0;
int motor_right_encoder = 0;
int prev_left_encoder = 0;
int prev_right_encoder = 0;

float motor_left_encoder_lpf = 0;
float motor_right_encoder_lpf = 0;
float motor_left_encoder_lpf_prev = 0;
float motor_right_encoder_lpf_prev = 0;

uint32_t total_left_encoder = 0;
uint32_t total_right_encoder = 0;
int distance_travelled_int = 0;
float distance_travelled = 0.0f;
float total_revolution = 0.0f;
int total_revolution_int = 0;
int total_encoder = 0;

float motor_left_rpm = 0.0f;
float motor_right_rpm = 0.0f;
int motor_left_rpm_int = 0;
int motor_right_rpm_int = 0;

float motor_left_rpm_lpf = 0.0f;
float motor_right_rpm_lpf = 0.0f;
int motor_left_rpm_lpf_int = 0;
int motor_right_rpm_lpf_int = 0;

int encoder_difference = 0;

int motor_stop = 0;
//--------------------------------

//-------- UART ------------
uint8_t aRxBuffer[20];
uint8_t uart_byte;
uint8_t uart_rx_buffer[64];
uint8_t uart_index = 0;
char last_oled_msg[32] = "";

int have_instruction = 0;
int instruction_type = 0;
char buffer[64];
//--------------------------

//------------ PID Controller ----------
#define OUTPUT_MIN_MOTOR -2500
#define OUTPUT_MAX_MOTOR 2500
#define OUTPUT_MAX_PWM_MOTOR 2500
#define OUTPUT_MIN_PWM_MOTOR 0
#define INTEGRAL_MAX_MOTOR 200

#define OUTPUT_MIN_ANGLE -2500
#define OUTPUT_MAX_ANGLE 2500
#define OUTPUT_MAX_PWM_ANGLE 2500
#define OUTPUT_MIN_PWM_ANGLE0
#define INTEGRAL_MAX_ANGLE 200

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	int prevError;
	int integral;
	int pid_change;
	int pwm_output;
} PID_Controller;

void setPID(PID_Controller *pid, float Kp, float Ki, float Kd, int pwm_output){
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->pwm_output = pwm_output;
}
PID_Controller pid_left, pid_right, pid_angle;

//--------------------------------------

//---------------- Miscellaneous --------------------
#define SERVO_LEFT 47
#define SERVO_MIDDLE 76
#define SERVO_RIGHT 115
#define LEFT 1
#define RIGHT 0
#define FORWARD 1
#define REVERSE 0

int error_left_encoder = 0, error_right_encoder = 0;
float error_left_rpm = 0.0f, error_right_rpm = 0.0f;
float deadband_left = 250.0f, deadband_right = 250.0f;
int pwm_left = 0, pwm_right = 0;
float error_angle = 0.0f;
int error_angle_int = 0;
//----------------------------------------------------

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM12_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM14_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  HAL_UART_Receive_IT(&huart3, &uart_byte, 1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ShowTask */
  ShowTaskHandle = osThreadNew(show, NULL, &ShowTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(motors, NULL, &MotorTask_attributes);

  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(encoder, NULL, &EncoderTask_attributes);

  /* creation of ServoTask */
  ServoTaskHandle = osThreadNew(servo, NULL, &ServoTask_attributes);

  /* creation of EncoderBTask */
  EncoderBTaskHandle = osThreadNew(encoder_B, NULL, &EncoderBTask_attributes);

  /* creation of UltrasonicSenso */
  UltrasonicSensoHandle = osThreadNew(ultra_sensor, NULL, &UltrasonicSenso_attributes);

  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(sensor_reading, NULL, &sensorTask_attributes);

  /* creation of IRDistTask */
  IRDistTaskHandle = osThreadNew(ir_dist, NULL, &IRDistTask_attributes);

  /* creation of transmitTask */
  transmitTaskHandle = osThreadNew(transmit, NULL, &transmitTask_attributes);

  /* creation of encoderDiffTask */
  encoderDiffTaskHandle = osThreadNew(encoder_diff, NULL, &encoderDiffTask_attributes);

  /* creation of turningTask */
  turningTaskHandle = osThreadNew(turning_task, NULL, &turningTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */

  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 16-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 7199;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 320;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1000;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 16-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OLED_DC_Pin|OLED_RES_Pin|OLED_SDA_Pin|OLED_SCL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_DC_Pin OLED_RES_Pin OLED_SDA_Pin OLED_SCL_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin|OLED_RES_Pin|OLED_SDA_Pin|OLED_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void delay_us(uint16_t us){
	HAL_TIM_Base_Start(&htim14);
	__HAL_TIM_SET_COUNTER(&htim14, 0);

	while(__HAL_TIM_GET_COUNTER(&htim14) < us);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){

	if(htim==&htim8){
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_SET){	//If pin on high, means positive edge
			tc1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);	//Retrive value and store in tc1
		} else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_RESET){	//If pin on low means negative edge
			tc2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);	//Retrive val and store in tc2
			if (tc2 > tc1){
				us_echo = tc2-tc1;		//Calculate the differnce = width of pulse
			} else {
				us_echo = (65536-tc1)+tc2;
			}
		}

	}
}




void display_if_changed(const char* msg)
{
    if (strcmp(last_oled_msg, msg) != 0) {
//        OLED_Clear();
        OLED_ShowString(5, 45, msg); //last row
        strncpy(last_oled_msg, msg, sizeof(last_oled_msg));
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//------------------------ OLD -------------------------------
	//Prevent unused argument(s) compilation warning
//	UNUSED(huart);
//	HAL_UART_Transmit(&huart3, (uint8_t *)aRxBuffer, 10, 0xFFFF);
	//------------------------------------------------------------

	//------------------------- NEW ------------------------------
	if (huart->Instance == USART3){
	  switch (uart_byte){
		  case 'f':
			  have_instruction = 1;
			  instruction_type = 1;
			  display_if_changed("Forward");
			  sprintf(buffer, "ACK\r\n");
			  HAL_UART_Transmit(&huart3, buffer, strlen(buffer), 0xFFFF);
			  break;

		  case 'b':
			  have_instruction = 1;
			  instruction_type = 2;
			  display_if_changed("Backwards");
			  sprintf(buffer, "ACK\r\n");
			  HAL_UART_Transmit(&huart3, buffer, strlen(buffer), 0xFFFF);
			  break;

		  case 'l':
			  have_instruction = 1;
			  instruction_type = 3;
			  display_if_changed("Turn Left");
			  sprintf(buffer, "ACK\r\n");
			  HAL_UART_Transmit(&huart3, buffer, strlen(buffer), 0xFFFF);
			  break;

		  case 'r':
			  have_instruction = 1;
			  instruction_type = 4;
			  display_if_changed("Turn Right");
			  sprintf(buffer, "ACK\r\n");
			  HAL_UART_Transmit(&huart3, buffer, strlen(buffer), 0xFFFF);
			  break;

		  case 's':
			  have_instruction = 1;
			  instruction_type = 5;
			  display_if_changed("Stopped");
			  sprintf(buffer, "ACK\r\n");
			  HAL_UART_Transmit(&huart3, buffer, strlen(buffer), 0xFFFF);
			  break;

		  case 'y':
			  have_instruction = 1;
			  display_if_changed("Image    ");
			  break;

		  case 'n':
			  have_instruction = 1;
			  display_if_changed("Bulls Eye");
			  break;

		  default:
			  if (uart_index < sizeof(uart_rx_buffer) - 1) {
				  uart_rx_buffer[uart_index++] = uart_byte;
				  uart_rx_buffer[uart_index] = '\0';
				  display_if_changed((char*)uart_rx_buffer);
			  }
			  break;
	  }

	  // ✅ Moved here so it always re-enables receive interrupt
	  HAL_UART_Receive_IT(&huart3, &uart_byte, 1);
  }
	//------------------------------------------------------------

}

void I2C_Bus_Recovery(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. De-init I2C peripheral
    HAL_I2C_DeInit(&hi2c2);

    // 2. Configure SCL and SDA as GPIO outputs open-drain
    __HAL_RCC_GPIOB_CLK_ENABLE();  // Change port if your I2C2 pins are different

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = GPIO_PIN_10; // SCL (check your board)
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11; // SDA (check your board)
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. Toggle SCL about 10 times while SDA is high
    for (int i = 0; i < 10; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_Delay(1);
    }

    // 4. Generate a STOP condition manually
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); // SDA low
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   // SCL high
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);   // SDA high

    // 5. Re-init I2C peripheral
    MX_I2C2_Init();
}

void icm20948_init(void)
{
    uint8_t data;
    uint8_t who_am_i;

    I2C_Bus_Recovery();

    // Select bank 0 (just to be safe)
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, 0x68 << 1, 0x7F, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

	// Read WHO_AM_I (0x00) - should be 0xEA
	HAL_I2C_Mem_Read(&hi2c2, 0x68 << 1, 0x00, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 1000);
	if (HAL_I2C_Mem_Read(&hi2c2, 0x68<<1, 0x00, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 1000) == HAL_OK) {
	    if (who_am_i == 0xEA) {
	        OLED_ShowString(95,45,"ok");
	    } else {
	        OLED_ShowString(95,45,"bad");
	    }
	} else {
	    OLED_ShowString(95,45,"i2c");
	}

    // Wake up
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c2, 0x68 << 1, 0x06, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    // Enable accel & gyro
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, 0x68 << 1, 0x07, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    // Disable ICM internal I2C master (required for BYPASS)
	data = 0x00; // USER_CTRL (0x03)
	HAL_I2C_Mem_Write(&hi2c2, 0x68<<1, 0x03, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

	// Enable BYPASS so MCU can talk to AK09916 at 0x0C
	data = 0x02; // INT_PIN_CFG (0x0F): BYPASS_EN=1
	HAL_I2C_Mem_Write(&hi2c2, 0x68<<1, 0x0F, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

	// Put AK09916 into continuous mode (e.g., 100 Hz)
	data = 0x08; // CNTL2 (0x31): 100 Hz
	HAL_I2C_Mem_Write(&hi2c2, 0x0C<<1, 0x31, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
}

void read_gyro_z(int16_t *gyro_z){
	uint8_t reg_addr = 0x37;    // start from GYRO_ZOUT_H
	uint8_t rawData[2];         // to store MSB and LSB
	char buf[20];

	// Step 1: Tell sensor which register to read
	if(HAL_I2C_Master_Transmit(&hi2c2, 0x68 << 1, &reg_addr, 1, 1000)!= HAL_OK){
		OLED_ShowString(5, 5, "0");
	}

	// Step 2: Read 2 bytes (MSB + LSB)
	if (HAL_I2C_Master_Receive(&hi2c2, 0x68 << 1, rawData, 2, 1000)!=HAL_OK){
		OLED_ShowString(5, 5, "1");
	}
	// Combine into signed 16-bit value
	*gyro_z = (int16_t)((rawData[0] << 8) | rawData[1]);
	float gyro_z_dps = *gyro_z / 131.0f;
	int a = gyro_z_dps;

	sprintf(buf, "%3d", a);
	OLED_ShowString(5, 15, buf);
}

void read_mag_z(int16_t *mag_z_out)
{
    uint8_t st1, st2, reg, buf[2];
    char buf1[20];

    // 1) Wait for data-ready (ST1 bit0 = 1)
    reg = 0x10;
    HAL_I2C_Master_Transmit(&hi2c2, 0x0C<<1, &reg, 1, 1000);
    HAL_I2C_Master_Receive(&hi2c2, 0x0C<<1, &st1, 1, 1000);
    if ((st1 & 0x01) == 0) { // no new data
        OLED_ShowString(80, 5, "err");
    }

    // 2) Read Z low/high (0x15, 0x16)
    reg = 0x15; // HZL
    HAL_I2C_Master_Transmit(&hi2c2, 0x0C<<1, &reg, 1, 1000);
    HAL_I2C_Master_Receive(&hi2c2, 0x0C<<1, buf, 2, 1000);
    int16_t mag_z = (int16_t)((buf[1] << 8) | buf[0]); // little-endian

    // 3) Read ST2 (0x18) to complete the read sequence
    reg = 0x18;
    HAL_I2C_Master_Transmit(&hi2c2, 0x0C<<1, &reg, 1, 1000);
    HAL_I2C_Master_Receive(&hi2c2, 0x0C<<1, &st2, 1, 1000);

    *mag_z_out = mag_z;
//    sprintf(buf1, "%5d", mag_z);
//    OLED_ShowString(5, 15, buf1);
}


void PID_Compute(PID_Controller *pid, float error, float dt)
{
	// Integral term with anti-windup
	pid->integral += error * dt;
	if (pid->integral > INTEGRAL_MAX_MOTOR) pid->integral = INTEGRAL_MAX_MOTOR;
	if (pid->integral < -INTEGRAL_MAX_MOTOR) pid->integral = -INTEGRAL_MAX_MOTOR;

	// Derivative term
	float derivative = (error - pid->prevError) / dt;

	// PID output
	float output = pid->Kp * error
				 + pid->Ki * pid->integral
				 + pid->Kd * derivative;

	// Clamp to output range
	if (output > OUTPUT_MAX_MOTOR) output = OUTPUT_MAX_MOTOR;
	if (output < OUTPUT_MIN_MOTOR) output = OUTPUT_MIN_MOTOR;

	pid->pid_change = output;
	pid->prevError = error;
	pid->pwm_output += (int)output;
	if (pid->pwm_output > OUTPUT_MAX_MOTOR) pid->pwm_output = OUTPUT_MAX_MOTOR;
	if (pid->pwm_output < 0) pid->pwm_output = 0;

}

//void PID_Angle(PID_Controller *pid, float error, float dt){
//	// Integral term with anti-windup
//	pid->integral += error * dt;
//	if (pid->integral > INTEGRAL_MAX_ANGLE) pid->integral = INTEGRAL_MAX_ANGLE;
//	if (pid->integral < -INTEGRAL_MAX_ANGLE) pid->integral = -INTEGRAL_MAX_ANGLE;
//
//	// Derivative term
//	float derivative = (error - pid->prevError) / dt;
//
//	// PID output
//	float output = pid->Kp * error
//				 + pid->Ki * pid->integral
//				 + pid->Kd * derivative;
//
//	// Clamp to output range
//	if (output > OUTPUT_MAX_ANGLE) output = OUTPUT_MAX_ANGLE;
//	if (output < OUTPUT_MIN_ANGLE) output = OUTPUT_MIN_ANGLE;
//
//	pid->pid_change = output;
//	pid->prevError = error;
//	pid->pwm_output = (int)output;
//	if (pid->pwm_output > OUTPUT_MAX_ANGLE) pid->pwm_output = OUTPUT_MAX_ANGLE;
//	if (pid->pwm_output < 0) pid->pwm_output = 0;
//}



void forward(float target_rpm, float target_distance, int delay){
	//-------------------- MOTOR START -----------------------
	setPID(&pid_left, 1.5f, 0.0f, 0.0f, 0);
	setPID(&pid_right, 1.5f, 0.0f, 0.0f, 0);
	MotorDrive_enable();
	Motor_direction_both(1);
	//--------------------------------------------------------

	distance_travelled = 0.0f;
	total_left_encoder = 0.0f;
	rotate_wheel(SERVO_LEFT);
	osDelay(1000);
	rotate_wheel(SERVO_MIDDLE);
	osDelay(300);

	for (;;){
		//--------------- PID Controller -------------------------
		  error_left_rpm = (target_rpm) - motor_left_rpm;
		  error_right_rpm = (target_rpm + RPM_OFFSET) - motor_right_rpm;

		  PID_Compute(&pid_left, error_left_rpm, (float)(delay/1000.0f));
		  PID_Compute(&pid_right, error_right_rpm, (float)(delay/1000.0f));

		  __HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,pid_left.pwm_output);  //Motor B (left)
		  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,pid_right.pwm_output); //Motor A (right)
		//-------------------------------------------------------------

		//-------------- Distance Calculation ----------------------
		  total_revolution = total_left_encoder / 1525.0f;  //1320 OR 1405
		  distance_travelled = total_revolution * 20.41f;  // wheel circumference = 20.736 OR 20.41,

		  total_revolution_int = total_revolution;
		  distance_travelled_int = distance_travelled;
		//----------- End of Distance Calculation ------------------


		//-------------------- Check distance travelled ------------------
		  if ((distance_travelled >= target_distance) && (target_distance != 0.0f)){
			  MotorStop_both();
			  break;
		  }
		//-----------------------------------------------------------------

		//------------------- Check if forced to stop ---------------------
//		  if (motor_stop == 1){
//			  MotorStop_both();
//			  motor_stop = 0;
//			  break;
//		  }
		//-----------------------------------------------------------------

		  osDelay(delay);
	}
}

void forward_until(float target_rpm, float target_distance, int delay){
	//-------------------- MOTOR START -----------------------
	setPID(&pid_left, 1.5f, 0.0f, 0.0f, 0);
	setPID(&pid_right, 1.5f, 0.0f, 0.0f, 0);
	MotorDrive_enable();
	Motor_direction_both(1);
	//--------------------------------------------------------

	distance_travelled = 0.0f;
	total_left_encoder = 0.0f;
	rotate_wheel(SERVO_LEFT);
	osDelay(1000);
	rotate_wheel(SERVO_MIDDLE);
	osDelay(300);

	for (;;){
		//--------------- PID Controller -------------------------
		  error_left_rpm = (target_rpm + RPM_OFFSET) - motor_left_rpm;
		  error_right_rpm = (target_rpm) - motor_right_rpm;

		  PID_Compute(&pid_left, error_left_rpm, (float)(delay/1000.0f));
		  PID_Compute(&pid_right, error_right_rpm, (float)(delay/1000.0f));

		  __HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,pid_left.pwm_output);  //Motor B (left)
		  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,pid_right.pwm_output); //Motor A (right)
		//-------------------------------------------------------------

		//-------------- Distance Calculation ----------------------
		  total_revolution = total_left_encoder / 1525.0f;  //1320 OR 1405
		  distance_travelled = total_revolution * 20.41f;  // wheel circumference = 20.736 OR 20.41,

		  total_revolution_int = total_revolution;
		  distance_travelled_int = distance_travelled;
		//----------- End of Distance Calculation ------------------


		//-------------------- Check distance travelled ------------------
		  if (us_dist <= target_distance){
			  MotorStop_both();
			  break;
		  }
		//-----------------------------------------------------------------


		  osDelay(delay);
	}
}

void reverse(float target_rpm, float target_distance, int delay){
	//-------------------- MOTOR START -----------------------
	setPID(&pid_left, 1.5f, 0.0f, 0.0f, 0);
	setPID(&pid_right, 1.5f, 0.0f, 0.0f, 0);
	MotorDrive_enable();
	Motor_direction_both(0);
	//--------------------------------------------------------

	distance_travelled = 0.0f;
	total_left_encoder = 0;
	rotate_wheel(SERVO_LEFT);
	osDelay(1000);
	rotate_wheel(SERVO_MIDDLE);
	osDelay(300);

	for (;;){
		//--------------- PID Controller -------------------------
		  error_left_rpm = (target_rpm) - motor_left_rpm;
		  error_right_rpm = (target_rpm) - motor_right_rpm;

		  PID_Compute(&pid_left, error_left_rpm, (float)(delay/1000.0f));
		  PID_Compute(&pid_right, error_right_rpm, (float)(delay/1000.0f));

		  __HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1,pid_left.pwm_output);  //Motor B (left)
		  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,pid_right.pwm_output); //Motor A (right)
		//-------------------------------------------------------------

		//-------------- Distance Calculation ----------------------
		  total_revolution = total_left_encoder / 1525.0f;  //1320 OR 1405
		  distance_travelled = total_revolution * 20.41f;  // wheel circumference = 20.736 OR 20.41,

		  total_revolution_int = total_revolution;
		  distance_travelled_int = distance_travelled;
		//----------- End of Distance Calculation ------------------


		//-------------------- Check distance travelled ------------------
		  if ((distance_travelled >= target_distance) && (target_distance != 0.0f)){
			  MotorStop_both();
			  break;
		  }
		//-----------------------------------------------------------------

		//------------------- Check if forced to stop ---------------------
		  if (motor_stop == 1){
			  MotorStop_both();
			  motor_stop = 0;
			  break;
		  }
		//-----------------------------------------------------------------

		  osDelay(delay);
	}
}

void forwardTurn(int dir, float target_angle, float target_rpm, int delay){
	//-------------------- Servo Init -----------------------
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	if (dir == LEFT) htim12.Instance->CCR1 = SERVO_LEFT;
	else 		  htim12.Instance->CCR1 = SERVO_RIGHT;
	//--------------------------------------------------------

	//-------------------- MOTOR START -----------------------
	setPID(&pid_left, 1.0f, 0.0f, 0.0f, 0);
	setPID(&pid_right, 1.0f, 0.0f, 0.0f, 0);
	MotorDrive_enable();
	Motor_direction_both(1);
	//--------------------------------------------------------

	osDelay(500);
	prev_angle_direction = 0.0f;
	angle_direction = 0.0f;


	for (;;){
		//------------- Turning motion -------------------------
		  error_angle = target_angle - fabsf(angle_direction);
		  error_angle_int = error_angle;
		//------------------------------------------------------

		//--------------- PID Controller -------------------------
		  error_left_rpm = (target_rpm) - motor_left_rpm;
		  error_right_rpm = (target_rpm) - motor_right_rpm;

		  PID_Compute(&pid_left, error_left_rpm, (float)(delay/1000.0f));
		  PID_Compute(&pid_right, error_right_rpm, (float)(delay/1000.0f));

		  __HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,pid_left.pwm_output);  //Motor B (left)
		  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,pid_right.pwm_output); //Motor A (right)
		//-------------------------------------------------------------

		//------------------ Check angle turned --------------------------
//		  if (error_angle <= 15.0f){
//			  target_rpm = 30.0f;
//		  }

		  if ((error_angle) <= 0.0f){
			  MotorStop_both();
			  osDelay(500);
			  htim12.Instance->CCR1 = SERVO_MIDDLE;
			  break;
		  }
		//----------------------------------------------------------------

		  osDelay(delay);
	}

}

void reverseTurn(int dir, float target_angle, float target_rpm, int delay){
	//-------------------- Servo Init -----------------------
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	if (dir == LEFT) htim12.Instance->CCR1 = SERVO_LEFT;
	else 		  htim12.Instance->CCR1 = SERVO_RIGHT;
	//--------------------------------------------------------

	//-------------------- MOTOR START -----------------------
	setPID(&pid_left, 1.5f, 0.0f, 0.0f, 0);
	setPID(&pid_right, 1.5f, 0.0f, 0.0f, 0);
	MotorDrive_enable();
	Motor_direction_both(0);
	//--------------------------------------------------------

	osDelay(500);
	prev_angle_direction = 0.0f;
	angle_direction = 0.0f;

	for (;;){
		//------------- Turning motion -------------------------
		  error_angle = target_angle - fabsf(angle_direction);
		  error_angle_int = error_angle;
		//------------------------------------------------------

		//--------------- PID Controller -------------------------
		  error_left_rpm = (target_rpm) - motor_left_rpm;
		  error_right_rpm = (target_rpm) - motor_right_rpm;

		  PID_Compute(&pid_left, error_left_rpm, (float)(delay/1000.0f));
		  PID_Compute(&pid_right, error_right_rpm, (float)(delay/1000.0f));

		  __HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1,pid_left.pwm_output);  //Motor B (left)
		  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,pid_right.pwm_output); //Motor A (right)
		//-------------------------------------------------------------

		//------------------ Check angle turned --------------------------
//		  if (error_angle <= 15.0f){
//			  target_rpm = 30.0f;
//		  }

		  if ((error_angle) <= 0.0f){
			  MotorStop_both();
			  osDelay(500);
			  htim12.Instance->CCR1 = SERVO_MIDDLE;
			  break;
		  }
		//----------------------------------------------------------------

		  osDelay(delay);
	}

}

int request_RPI(void){
	sprintf(buffer, "IMG\r\n");
	HAL_UART_Transmit(&huart3, buffer, strlen(buffer), 0xFFFF);
	for (;;){
		osDelay(10);
		if (have_instruction){
			switch (uart_byte){
				case 'y': //image detected
					have_instruction = 0;
					return 1;
				case 'n':
					have_instruction = 0;
					return 0;
				default:
					have_instruction = 0;
					return 0;
			}
		}
	}
}

void buzzerOn(int delay){
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
	osDelay(delay);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
}

void rotate_wheel(int side){
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	htim12.Instance->CCR1 = side;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for(;;)
  {

	// Toggle LED every 1s to show that STM not stuck in loop and still running
	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	osDelay(1000);

	// Buzzer turns on/off when USER button pressed
//	if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == RESET){
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
//
//	}
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_show */
/**
* @brief Function implementing the ShowTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_show */
void show(void *argument)
{
  /* USER CODE BEGIN show */
	char buffer[20];

  /* Infinite loop */
  for(;;)
  {
	  sprintf(buffer, "Motor L:%2d R:%2d", motor_left_rpm_int, motor_right_rpm_int);
	  OLED_ShowString(5, 5, buffer);

	  sprintf(buffer, "Dist:%4d", distance_travelled_int);
	  OLED_ShowString(5, 15, buffer);

	  sprintf(buffer, "IR L:%2d R:%2d", ir_dist_left_int, ir_dist_right_int);
	  OLED_ShowString(5, 25, buffer);

	  sprintf(buffer, "US:%3d", us_dist_int);
	  OLED_ShowString(5, 35, buffer);

//	  sprintf(buffer, "Angle:%3d", gyro_z_dps_int);
//	  sprintf(buffer, "%04d-\0", pid_right.pwm_output);
//	  sprintf(buffer, "%04d--", encoder_difference);
//	  OLED_ShowString(5, 45, buffer);






	// Refreshes the OLED screen every 0.5s
	// This is the only refresh function call in all the tasks
	OLED_Refresh_Gram();

    osDelay(250);
  }
  /* USER CODE END show */
}

/* USER CODE BEGIN Header_motors */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motors */
void motors(void *argument)
{
  /* USER CODE BEGIN motors */
//	pid_left.Kp = 2.0f;
//	pid_left.Ki = 0.0f;
//	pid_left.Kd = 0.0f;
//	pid_left.pwm_output = 0;
//
//	pid_right.Kp = 2.0f;
//	pid_right.Ki = 0.0f;
//	pid_right.Kd = 0.0f;
//	pid_right.pwm_output = 0;


//	int delay = 40;
//
//	float target_rpm_left = 60.1f;
//	float target_rpm_right = 60.0f;
//	float target_rpm = 60.0f;
//	float dt = delay/1000.0f;
//
//	int flag = 0;
//	int left_flag = 0;
//	int right_flag = 0;


	//90 -> 79.25
	//180 -> 158.5
	//270 -> 237.75
	//360 -> 317
//	float target_angle = 317.0f;

//	__HAL_TIM_SET_COUNTER(&htim2, 0);
//	__HAL_TIM_SET_COUNTER(&htim3, 0);

	//*************** MOTOR START *****************
//	setPID(&pid_left, 1.5f, 0.0f, 0.0f, 0);
//	setPID(&pid_right, 1.5f, 0.0f, 0.0f, 0);
//	MotorDrive_enable();
//	__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1,0);  //Motor B (left)
//	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0);  //Motor A (right)
	//*********************************************

	//-------------------------- Servo Init -----------------------------
//	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
//	htim12.Instance->CCR1 = SERVO_MIDDLE; //middle
	//--------------------------------------------------------------------


//	osDelay(5000);  //starting buffer
//	forward(60.0f, 100.0f, 40);
//	osDelay(1000);
//	forward(60.0f, 100.0f, 40);
//	osDelay(1000);
//	reverse(60.0f, 100.0f, 40);
//	forwardTurn(RIGHT, 90.0f, 60.0f, 40);
//	osDelay(2000);
//	forwardTurn(LEFT, 90.0f, 60.0f, 40);
//	osDelay(2000);
//	reverseTurn(LEFT, 90.0f, 60.0f, 40);
//	osDelay(2000);
//	reverseTurn(RIGHT, 90.0f, 60.0f, 40);
	//-------------- Servo Init 2 ------------------
//	htim12.Instance->CCR1 = SERVO_LEFT; //left
//	osDelay(1000);
	//----------------------------------------------

//	angle_direction = 0.0f;
	int flag = 0;
	int count = 0;
	int image_check;
	int start = 0;

//	osDelay(5000);  //starting buffer
//	rotate_wheel(SERVO_LEFT);
//	osDelay(1000);
//	rotate_wheel(SERVO_MIDDLE);
//	osDelay(5000);
//	rotate_wheel(SERVO_RIGHT);
//	osDelay(1000);
//	rotate_wheel(SERVO_MIDDLE-2);
//    forward(60.0f, 150.0f, 40);


//	forward(60.0f, 25.0f, 40);
//	osDelay(1000);
//	forwardTurn(LEFT, 90.0f, 60.0f, 40);
//	osDelay(1000);
//	forwardTurn(RIGHT, 90.0f, 60.0f, 40);
//	osDelay(1000);
//	forward(60.0f, 25.0f, 40);



	//*******************************
	//---------- Checklist ----------
	for (;;){
		osDelay(10);
		if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == RESET){
			break;
		}
	}
	forward_until(60.0f, 15.0f, 40);
	//-------------------------------

  /* Infinite loop */
  for(;;)
  {

	  //***** Un-comment to stop *****
//	  osDelay(1000);
//	  continue;
	  //*****************************

	  //------------- Checklist (Movement around object) ------------------------
	  if (flag){
		  osDelay(1000);
		  continue;
	  }

	  osDelay(3000); //Time delay to recognise image
//	  buzzerOn(200);
//	  image_check = request_RPI();
//	  if (image_check){
//		  flag = 1;
//		  buzzerOn(200);
//		  continue;
//	  }

//	  forward_until(60.0f, 15.0f, 40); //for testing


	  reverse(60.0f, 20.0f, 40);
	  osDelay(1000);

	  forwardTurn(LEFT, 45.0f, 60.0f, 40);
	  osDelay(1000);
	  forwardTurn(RIGHT, 45.0f, 60.0f, 40);
	  osDelay(1000);

	  forward(60.0f, 36.0f, 40);
	  osDelay(1000);
	  reverseTurn(LEFT, 88.0f, 60.0f, 40);
	  osDelay(1000);

	  forward_until(60.0f, 15.0f, 40);
	  osDelay(1000);

	  count++;
	  if (count == 3){
		  flag = 1;
//		  buzzerOn(200);
		  osDelay(200);
	  }
	  //-------------------------------------------------------------------------


	  //---------------- Receiving Instruction from RPi ----------------------
//	  osDelay(10);
//	  if (!have_instruction){
//		  MotorStop_both();
//		  continue;
//	  }
//	  else{
//		  switch (uart_byte){
//		  	  case 'f':
//		  		  forward(60.0f, 200.0f, 40);
//		  		  break;
//
//		  	  case 'b':
//		  		  reverse(60.0f, 200.0f, 40);
//		  		  break;
//
//		  	  case 'r':
//		  		forwardTurn(RIGHT, 90.0f, 60.0f, 40);
//		  		break;
//
//		  	  case 'l':
//		  		forwardTurn(LEFT, 90.0f, 60.0f, 40);
//		  		break;
//
//		  	  case 's':
//		  		  MotorStop_both();
//		  		  break;
//
//		  }
//		  sprintf(buffer, "FIN\r\n");
//		  HAL_UART_Transmit(&huart3, buffer, strlen(buffer), 0xFFFF);
//		  have_instruction = 0;
//	  }
	  //----------------------------------------------------------------------





//	  if (flag) continue;

	//------------- Turning motion -------------------------
//	  error_angle = target_angle - angle_direction;
//	  error_angle_int = error_angle;
	//------------------------------------------------------

    //--------------- PID Controller -------------------------
//	  forward(&pid_left, &pid_right, target_rpm_left, target_rpm_right, dt);
//	  error_left_rpm = (target_rpm_left) - motor_left_rpm;
//	  error_right_rpm = (target_rpm_right) - motor_right_rpm;
//
//	  PID_Compute(&pid_left, error_left_rpm, dt);
//	  PID_Compute(&pid_right, error_right_rpm, dt);
//
//	  __HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,pid_left.pwm_output);  //Motor B (left)
//	  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,pid_right.pwm_output); //Motor A (right)
	//-------------------------------------------------------------

	  //---------------------- Drift Correction ---------------------------------
//	  if (right_flag){
//		  right_flag++;
//		  if (right_flag >= 25){
//			  target_rpm_right = target_rpm;
//			  right_flag = 0;
//		  }
//	  }
//
//	  if (left_flag){
//		  left_flag++;
//		  if (left_flag >= 25){
//			  target_rpm_left = target_rpm;
//			  left_flag = 0;
//		  }
//	  }
//
//	  if (encoder_difference > 50 && !right_flag){
//		  target_rpm_right += 1.0f;
//		  right_flag = 1;
//	  }
//
//	  else if (encoder_difference < -50 && !left_flag){
//		  target_rpm_left += 1.0f;
//		  left_flag = 1;
//	  }
	  //------------------------------------------------------------------------------



	//-------------- Distance Calculation ----------------------
//	  total_revolution = total_left_encoder / 1525.0f;  //1320 OR 1405
//	  distance_travelled = total_revolution * 20.41f;  // wheel circumference = 20.736 OR 20.41,
//
//	  total_revolution_int = total_revolution;
//	  distance_travelled_int = distance_travelled;
	//----------- End of Distance Calculation ------------------


	//-------------------- Check distance travelled ------------------
//	  if ((distance_travelled >= 80.0f)){
//		  MotorStop_both();
//		  flag = 1;
//		  osDelay(500);
//		  htim12.Instance->CCR1 = SERVO_MIDDLE;
//	  }
	//-----------------------------------------------------------------

	//------------------ Check angle turned --------------------------
//	  if ((error_angle) <= 0.2f){
//		  MotorStop_both();
//		  flag = 1;
//		  osDelay(500);
//		  htim12.Instance->CCR1 = SERVO_MIDDLE;
//	  }
	//----------------------------------------------------------------

//	osDelay(delay);
  }
  /* USER CODE END motors */
}

/* USER CODE BEGIN Header_encoder */
/**
* @brief Function implementing the EncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoder */
void encoder(void *argument)
{
  /* USER CODE BEGIN encoder */
  /* Infinite loop */
  // --------------- Right Motor (Motor A) --------------------------
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  int cnt1A, cnt2A;
  uint32_t tickA;

  cnt1A = __HAL_TIM_GET_COUNTER(&htim2);
  tickA = HAL_GetTick();
  uint8_t display1[20];
  uint8_t display2[20];
  uint16_t dirA;
  float a = 0.95f;

  for(;;)
  {
    if (HAL_GetTick() - tickA > 40L){
    	cnt2A = __HAL_TIM_GET_COUNTER(&htim2);
    	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
    		if (cnt2A < cnt1A)
    			motor_right_encoder = cnt1A - cnt2A;
    		else
    			motor_right_encoder = (65535 - cnt2A) + cnt1A;
    	}
    	else{
    		if (cnt2A > cnt1A)
    			motor_right_encoder = cnt2A - cnt1A;
    		else
    			motor_right_encoder = (65535 - cnt1A) + cnt2A;
    	}

    	//----------------------- without LPF -----------------------
//    	if (motor_right_encoder > 250) motor_right_encoder = 0;
//    	total_right_encoder += motor_right_encoder;
//		motor_right_encoder *= 20;
//		motor_right_rpm = motor_right_encoder / 22.0f; // 60/1320 = 1/22
    	//-----------------------------------------------------------

    	//------------------------------- with LPF ---------------------------------------
    	if (motor_right_encoder > 250){
    		motor_right_encoder = 0;
    		motor_right_encoder_lpf = 0;
    	}
    	total_right_encoder += motor_right_encoder;
    	motor_right_encoder *= 25;
//    	total_right_encoder = __HAL_TIM_GET_COUNTER(&htim2);
    	motor_right_rpm = motor_right_encoder / 22.0f; // 60/1320 = 1/22

    	motor_right_encoder_lpf = (motor_right_encoder * (1-a)) + (prev_right_encoder * a);
    	prev_right_encoder = motor_right_encoder_lpf;  // for LPF
    	motor_right_rpm_lpf = motor_right_encoder_lpf / 22.0f; // 60/1320 = 1/22
    	motor_right_rpm_lpf_int = motor_right_rpm_lpf;
    	//---------------------------------------------------------------------------------


    	motor_right_rpm_int = motor_right_rpm;
    	dirA = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);

    	//-------------------- OLED Display --------------------------
//    	sprintf(display1, "(R)S:%3d.%2d",motor_right_rpm_int, motor_right_rpm_frac);
//    	sprintf(display2, " %1d",dirA);

//    	OLED_ShowString(5, 25, display1);
//    	OLED_ShowString(90, 25, display2);
    	//-------------------------------------------------------------

    	//---------- Reset Procedure ------------
    	cnt1A = __HAL_TIM_GET_COUNTER(&htim2);
	    tickA = HAL_GetTick();
	    //---------------------------------------
    }
//    osDelay(1);
  }
  /* USER CODE END encoder */
}

/* USER CODE BEGIN Header_servo */
/**
* @brief Function implementing the ServoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servo */
void servo(void *argument)
{
  /* USER CODE BEGIN servo */
//	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);

//	osDelay(1500);
//    htim12.Instance->CCR1 = SERVO_MIDDLE; //middle
//    osDelay(1500);
//	htim12.Instance->CCR1 = SERVO_LEFT; //middle
//    htim12.Instance->CCR1 = SERVO_RIGHT; //right

  /* Infinite loop */
  for(;;)
  {
//    htim12.Instance->CCR1 = 74; //middle
//    osDelay(1500);
//    htim12.Instance->CCR1 = 115; //right
//	osDelay(1500);
//    htim12.Instance->CCR1 = 75; //middle
//	osDelay(1500);
//	htim12.Instance->CCR1 = 45; //left
//	osDelay(1500);
	  osDelay(1000);

  }
  /* USER CODE END servo */
}

/* USER CODE BEGIN Header_encoder_B */
/**
* @brief Function implementing the EncoderBTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoder_B */
void encoder_B(void *argument)
{
  /* USER CODE BEGIN encoder_B */
	// --------------- Left Motor (Motor B) --------------------------
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	int cnt1A, cnt2A;
	uint32_t tickA;

	cnt1A = __HAL_TIM_GET_COUNTER(&htim3);
	tickA = HAL_GetTick();
	uint8_t display1[20];
	uint8_t display2[20];
	uint16_t dirA;
	float a = 0.95f;

	char sbuf[64];

	for(;;)
	{
	if (HAL_GetTick() - tickA > 40L){
		cnt2A = __HAL_TIM_GET_COUNTER(&htim3);
		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)){
			if (cnt2A < cnt1A)
				motor_left_encoder = cnt1A - cnt2A;
			else
				motor_left_encoder = (65535 - cnt2A) + cnt1A;
		}
		else{
			if (cnt2A > cnt1A)
				motor_left_encoder = cnt2A - cnt1A;
			else
				motor_left_encoder = (65535 - cnt1A) + cnt2A;
		}


		//------------------- without LPF -------------------------
//		if (motor_left_encoder > 250) motor_left_encoder = 0;
//		total_left_encoder += motor_left_encoder;
//		motor_left_encoder *= 20;
//		motor_left_rpm = motor_left_encoder / 22.0f;  //  60/1320 = 1/22
		//---------------------------------------------------------

		//---------------------- with LPF --------------------------
		if (motor_left_encoder > 250){
			motor_left_encoder = 0;
			motor_left_encoder_lpf = 0;
		}
		total_left_encoder += motor_left_encoder;
		motor_left_encoder *= 25;
//		total_left_encoder = __HAL_TIM_GET_COUNTER(&htim3);
		motor_left_rpm = motor_left_encoder / 22.0f;  //  60/1320 = 1/22

		motor_left_encoder_lpf = (motor_left_encoder * (1-a)) + (prev_left_encoder * a);  //for LPF
		prev_left_encoder = motor_left_encoder_lpf;  //for LPF
		motor_left_rpm_lpf = motor_left_encoder_lpf / 22.0f;  //  60/1320 = 1/22
		motor_left_rpm_lpf_int = motor_left_rpm_lpf;
		//-----------------------------------------------------------


		motor_left_rpm_int = motor_left_rpm;
		dirA = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
		dirA = !(dirA);

		//------------------- OLED Display ----------------
//		sprintf(display1, "(L)S:%3d.%2d",motor_left_rpm_int, motor_left_rpm_frac);
//		sprintf(display2, " %1d",dirA);

//		OLED_ShowString(5, 35, display1);
//		OLED_ShowString(90, 35, display2);
		//-------------------------------------------------


		//------------ Reset Procedure -------------
		cnt1A = __HAL_TIM_GET_COUNTER(&htim3);
		tickA = HAL_GetTick();
		//------------------------------------------

		}
//	osDelay(1);
	}
  /* USER CODE END encoder_B */
}

/* USER CODE BEGIN Header_ultra_sensor */
/**
* @brief Function implementing the UltrasonicSenso thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ultra_sensor */
void ultra_sensor(void *argument)
{
  /* USER CODE BEGIN ultra_sensor */
	char buf[20];
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(TRIG_Port, TRIG_Pin, GPIO_PIN_RESET);
	  osDelay(50);

	  //Output 1us of Trig
	  HAL_GPIO_WritePin(TRIG_Port, TRIG_Pin, GPIO_PIN_SET);
	  delay_us(10);
	  HAL_GPIO_WritePin(TRIG_Port, TRIG_Pin, GPIO_PIN_RESET);
	  osDelay(50);

	  //wait for rising edge
	  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
	  HAL_TIM_IC_CaptureCallback(&htim8);


	  us_dist = us_echo * 0.01715;
	  if (us_dist > 400.0f) us_dist = 400.0f;
	  if (us_dist < 0.0f) us_dist = 0.0f;
	  us_dist_int = us_dist;


	  //Low pass filter
//	  us_dist_lpf = (us_dist * 0.3) + (us_dist_prev * 0.7);
//	  us_dist_prev = us_dist_lpf;
//	  us_dist_int = us_dist_lpf;



//	  sprintf(buf, "Dis:%3dcm", us_dist_int);
//	  OLED_ShowString(10,48, buf);

  }
  /* USER CODE END ultra_sensor */
}

/* USER CODE BEGIN Header_sensor_reading */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sensor_reading */
void sensor_reading(void *argument)
{
  /* USER CODE BEGIN sensor_reading */
	uint8_t reg_addr = 0x37;    // start from GYRO_ZOUT_H
	uint8_t rawData[2];         // to store MSB and LSB
	float B = 0.9f;

	int16_t drift_raw, drift_total=0;
	float drift_degrees;

	int delay = 10;
	float dt;
	double offset = 0;
	double trash = 0;
	int i = 0;
	uint32_t tick1, tick2;

	icm20948_init();
	osDelay(100); //delay to make sure ICM 20948 power up

	tick1 = HAL_GetTick();

	while (i < 1000){
		osDelay(1);
		HAL_I2C_Master_Transmit(&hi2c2, 0x68 << 1, &reg_addr, 1, 1000);
		HAL_I2C_Master_Receive(&hi2c2, 0x68 << 1, rawData, 2, 1000);
		drift_raw = (int16_t)((rawData[0] << 8) | rawData[1]);
		offset += drift_raw;
		tick1 = HAL_GetTick();
		i++;
	}
	offset = offset/i;


	tick1 = HAL_GetTick();
  /* Infinite loop */
  for(;;)
  {
	//------------------- Gyroscope Readings ---------------------------------------
	// Step 1: Tell sensor which register to read
	if(HAL_I2C_Master_Transmit(&hi2c2, 0x68 << 1, &reg_addr, 1, 1000)!= HAL_OK){
		OLED_ShowString(5, 5, "0");
	}

	// Step 2: Read 2 bytes (MSB + LSB)
	if (HAL_I2C_Master_Receive(&hi2c2, 0x68 << 1, rawData, 2, 1000)!=HAL_OK){
		OLED_ShowString(5, 5, "1");
	}
	// Combine into signed 16-bit value
	gyro_z_raw = (int16_t)((rawData[0] << 8) | rawData[1]);
//	angle_direction += (double) ((double)gyro_z_raw - offset) * ((HAL_GetTick() - tick1)/16400.0f);
//	angle_direction_int = angle_direction;
//	tick1 = HAL_GetTick();
//	if (gyro_z_raw < 150 && gyro_z_raw > 0) gyro_z_raw = 0;
	gyro_z_raw -= offset;
	gyro_z_dps = gyro_z_raw / 131.0f;
	gyro_z_dps_int = gyro_z_dps;
	// -----------------------------------------------------------------------------


	//-------------------- Angle Calculation -----------------------
	tick2 = HAL_GetTick();
	dt = (tick2-tick1) / 1000.0f;
	angle_direction = prev_angle_direction + (gyro_z_dps * dt);
	prev_angle_direction = angle_direction;
	angle_direction_int = angle_direction;
	//--------------------------------------------------------------

	//------------------- High-Pass Filter -------------------------
//	angle_direction_hpf = (B * angle_direction_hpf_prev) + (B * (gyro_z_dps - gyro_z_dps_prev));
//	angle_direction_hpf_prev = angle_direction_hpf;
//	angle_direction_hpf_int = angle_direction_hpf;
	//--------------------------------------------------------------

	tick1 = tick2;

	osDelay(1);
  }
  /* USER CODE END sensor_reading */
}

/* USER CODE BEGIN Header_ir_dist */
/**
* @brief Function implementing the IRDistTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ir_dist */
void ir_dist(void *argument)
{
  /* USER CODE BEGIN ir_dist */

  /* Infinite loop */
  for(;;)
  {
	// ---------------- IR Sensor Left Readings ------------------------
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 20);
	ir_raw_left = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	//  40483.4058f / (raw + 50.8020f) - 4.6692f;
	ir_dist_left = (40483.4058f / (ir_raw_left + 50.8020f)) - 4.6692f;
	if (ir_dist_left > 40.0f) ir_dist_left = 40.0;
	if (ir_dist_left < 0.0f) ir_dist_left = 0.0f;
	ir_dist_left_int = ir_dist_left;

	// Low-pass Filter
//	ir_dist_left_lpf = (ir_dist_left * 0.3f) + (ir_dist_left_prev * 0.7f);
//	ir_dist_left_prev = ir_dist_left_lpf;
//	ir_dist_left_int = ir_dist_left_lpf;

	// -------------------------------------------------------------------

	// ------------------ IR Sensor Right Readings  -----------------------
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 20);
	ir_raw_right = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);


	// 44430.9618f / (raw + 109.84988f) - 5.72134f;
	ir_dist_right = (44430.9618f / (ir_raw_right + 109.84988f)) - 5.72134f;
	if (ir_dist_right > 40.0f) ir_dist_right = 40.0;
 	if (ir_dist_right < 0.0f) ir_dist_right = 0.0f;
	ir_dist_right_int = ir_dist_right;

	//Low-pass filter
//	ir_dist_right_lpf = (ir_dist_right * 0.3f) + (ir_dist_right_prev * 0.7f);
//	ir_dist_right_prev = ir_dist_right_lpf;
//	ir_dist_right_int = ir_dist_right_lpf;
	// --------------------------------------------------------------------

    osDelay(200);
  }
  /* USER CODE END ir_dist */
}

/* USER CODE BEGIN Header_transmit */
/**
* @brief Function implementing the transmitTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_transmit */
void transmit(void *argument)
{
  /* USER CODE BEGIN transmit */
//  uint8_t ch = 'A';
  char sbuf[64];
  int target_rpm = 60;

  /* Infinite loop */
  for(;;)
  {

	  //--------------------------------------------- Motor ------------------------------------------
//	  sprintf(sbuf, "%2d,%2d,%3d,%3d,%2d,%7d,0\r\n", motor_left_rpm_int, motor_right_rpm_int,
//			  	  	  	  	  	  	  	  	  motor_left_rpm_lpf_int, motor_right_rpm_lpf_int, target_rpm,
//											  encoder_difference);

//	  sprintf(sbuf, "%5d,100\r\n", distance_travelled_int);

//	  sprintf(sbuf, "%2d,%2d,%3d,%3d,%2d,%4d,%4d,0\r\n", motor_left_rpm_int, motor_right_rpm_int,
//			  	  	  	  	  	  	  	  	  (int)error_left_rpm, (int)error_right_rpm, target_rpm,
//	  	  	  	  	  	  	  	  	  	  	  pid_left.pwm_output, pid_right.pwm_output);
	  //-----------------------------------------------------------------------------------------------

	  //------------------------------- Turning Motion ----------------------------------------------
//	  sprintf(sbuf, "%2d,%3d\r\n", error_angle, angle_direction_hpf_int);
	  //---------------------------------------------------------------------------------------------

	  //------------------------------ Gyroscope readings -------------------------------------
//	  sprintf(sbuf, "%5d,%3d,%3d,%3d,90\r\n", gyro_z_raw, gyro_z_dps_int, angle_direction_int, error_angle_int);
	  //---------------------------------------------------------------------------------------

	  //----------------------- IR Distance / UART w RPi -------------------------------
//	  sprintf(sbuf, "%5d,%5d,%3d\r\n", ir_dist_left_int, ir_dist_right_int, us_dist_int);
	  //--------------------------------------------------------------------------------

//	  HAL_UART_Transmit(&huart3, sbuf, strlen(sbuf), 0xFFFF);

//	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
//	if (ch < 'Z') ch++;
//	else ch = 'A';

    osDelay(200);
  }
  /* USER CODE END transmit */
}

/* USER CODE BEGIN Header_encoder_diff */
/**
* @brief Function implementing the encoderDiffTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoder_diff */
void encoder_diff(void *argument)
{
  /* USER CODE BEGIN encoder_diff */
	int left, right;
  /* Infinite loop */
  for(;;)
  {
    left = 65535 - __HAL_TIM_GET_COUNTER(&htim3);
	right = __HAL_TIM_GET_COUNTER(&htim2);
	encoder_difference = left - right;

	osDelay(20);
  }
  /* USER CODE END encoder_diff */
}

/* USER CODE BEGIN Header_turning_task */
/**
* @brief Function implementing the turningTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_turning_task */
void turning_task(void *argument)
{
  /* USER CODE BEGIN turning_task */
//	int delay = 40;
//
//	float target_rpm_left = 60.0f;
//	float target_rpm_right = 60.0f;
//	float target_angle = 90.0f;
//	float dt = delay/1000.0f;
//
//	int flag = 0;

	//-------------------------- Motor Init -----------------------------
//	setPID(&pid_left, 1.5f, 0.0f, 0.0f, 0);
//	setPID(&pid_right, 1.5f, 0.0f, 0.0f, 0);
//	MotorDrive_enable();
//	__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1,0);  //Motor B (left)
//	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0);  //Motor A (right)
	//--------------------------------------------------------------------

	//-------------------------- Servo Init -----------------------------
//	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
//	htim12.Instance->CCR1 = SERVO_MIDDLE; //middle
//	osDelay(5000);
//	htim12.Instance->CCR1 = SERVO_LEFT; //left
//	osDelay(1000);
	//--------------------------------------------------------------------

	//reset value caused by vibration
//	angle_direction = 0.0f;
	osDelay(5000);
	int i;
	int finish = 0;

  /* Infinite loop */
  for(;;)
  {
	  osDelay(1000);
	  continue;
	  //------------------- Checklist (Move around obstacle) -------------------
	  if (finish){
		  osDelay(100);
		  continue;
	  }

	  osDelay(10);
	  if (us_dist <= 15.0f){
		  motor_stop = 1;

		  for (i = 0; i < 2; i++){
			  osDelay(5000); //Time delay to recognise image
			  buzzerOn(500);

			  reverse(60.0f, 20.0f, 40);
			  osDelay(1000);

			  forwardTurn(LEFT, 45.0f, 60.0f, 40);
			  osDelay(1000);
			  forwardTurn(RIGHT, 45.0f, 60.0f, 40);
			  osDelay(1000);

			  forward(60.0f, 40.0f, 40);
			  osDelay(1000);
			  reverseTurn(LEFT, 90.0f, 60.0f, 40);
			  osDelay(1000);

			  forward(60.0f, 5.0f, 40);
			  osDelay(5000);
		  }
		  finish = 1;
		  buzzerOn(500);
		  osDelay(500);
		  buzzerOn(500);
	  }
	  //------------------------------------------------------------------------


//	osDelay(1000);
//	continue;
//
//	if (flag) continue;

//    error_angle = target_angle - angle_direction;
//    error_angle_int = error_angle;

//    PID_Angle(&pid_angle, error_angle, dt);
//    target_rpm_left = pid_angle.pwm_output;
//    target_rpm_right = pid_angle.pwm_output;
//
//    //----------- Motor Control -------------------------
//	  error_left_rpm = (target_rpm_left) - motor_left_rpm;
//	  error_right_rpm = (target_rpm_right) - motor_right_rpm;
//
//	  PID_Compute(&pid_left, error_left_rpm, dt);
//	  PID_Compute(&pid_right, error_right_rpm, dt);
//
//	  __HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,pid_left.pwm_output);  //Motor B (left)
//	  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,pid_right.pwm_output); //Motor A (right)
    //---------------------------------------------------


//	if (error_angle <= 0.2){
//		MotorStop_both();
//		osDelay(500);
//		htim12.Instance->CCR1 = SERVO_MIDDLE; //middle
//		flag = 1;
//	}

//	osDelay(delay);
  }
  /* USER CODE END turning_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
