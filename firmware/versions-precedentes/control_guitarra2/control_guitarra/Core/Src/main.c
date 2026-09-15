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
#include "stm32f4xx.h" // For NVIC_SystemReset

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdlib.h>
#define ARM_MATH_CM4
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_FRETS_SEQUENCE_LENGTH 500


// Motor driver control signals
typedef struct {
	uint8_t dir;       // Direction signal (0 = one direction, 1 = opposite)
	uint8_t enable;    // Motor enable state (0 = disabled, 1 = enabled)
	uint16_t step;     // Remaining steps to move
} controller_instance;

// Current kinematic state of the system
typedef struct {
	float position;    // Current position [mm]
	float velocity;    // Current velocity [mm/s]
	float acceleration;// Current acceleration [mm/s²]
} kinematic_instance;

// Timer configuration values
typedef struct {
	uint16_t arr;      // Auto-Reload Register value
	uint16_t psc;      // Prescaler value
} TimerValues;

// Solenoid control and state
typedef struct {
	uint8_t press_type;               // Type of pressing action (1: soft, 2: hard)
	uint8_t release_type;             // Type of release action (1: soft, 2: hard)
	uint8_t state;                    // 0: Disabled, 1: Active
	uint32_t hold_ticks;              // Number of ticks to hold the solenoid
	uint32_t time_accumulator_ticks;  // Accumulated time in ticks
	uint32_t start_count;             // Tick count when action started
	float duty;                       // PWM duty cycle for solenoid
	uint16_t arr;                     // ARR value associated with PWM
} solenoid_t;

// Structure for representing a song sequence
typedef struct {
	uint16_t bpm;                             			// Beats per minute
	uint8_t subdivision;                      			// Subdivision per beat
	uint8_t frets_sequence[MAX_FRETS_SEQUENCE_LENGTH]; // Frets stored as integers
	uint16_t index;                           			// Next free position to write
} Song;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// System clock frequency
#define F_CPU 100000000 // Hz

// Diameter of the pulley attached to the motor shaft
#define PULLEY_DIAMETER 13.3 // mm

// Number of motor steps per full revolution (without microstepping)
#define STEPS_PER_REVOLUTION 200 // steps

// Maximum linear velocity allowed
#define MAX_VELOCITY 550.0 // mm/s

// Maximum linear acceleration allowed
//#define MAX_ACCELERATION 5000.0 // mm/s²
#define MAX_ACCELERATION 5000.0 // mm/s²

// Maximum travel distance from the home position
#define MAX_POSITION 255.0 // mm

// Number of microsteps per full step
#define MICROSTEPS 32

// Number of motor steps per millimeter of linear movement
#define STEPS_PER_MM (STEPS_PER_REVOLUTION / (M_PI * PULLEY_DIAMETER))

// Total number of motor steps to reach maximum position
#define TOTAL_STEPS ((uint16_t)(MAX_POSITION * STEPS_PER_MM))

// Number of frets on the virtual guitar neck
#define NUM_FRETS 22

#define SAMPLING_FREQ_HZ 8000U
#define ADC_BUFFER_SIZE  8192

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
kinematic_instance kinematic = {0, 0, 0};    // Current kinematic state (position, velocity, acceleration)
controller_instance controller = {0, 0, 0};  // Motor control signals (dir, enable, steps)
solenoid_t solenoid = {0};                   // Solenoid control and state
Song current_song = {0};                     // Currently loaded song sequence
uint8_t micro_steps = 32;                    // Number of microsteps per full step
float homing_speed = MAX_VELOCITY * 0.5;      // Homing speed set to 50% of max velocity
volatile uint8_t solenoid_action_done_flag = 0; // Indicates if solenoid PWM action has completed

uint8_t status = 0; // System state machine
/*
   status = 0 -> Not calibrated
   status = 1 -> Motor enabled
   status = 2 -> Homing in progress
   status = 3 -> Position referenced (ready)
   status = 4 -> Moving to target position
 */

uint8_t playMode = 1; // Guitar neck mode
/*
   playMode = 0 -> String not tuned
   playMode = 1 -> Discrete guitar neck (virtual frets)
   playMode = 2 -> Continuous guitar neck (fretless)
 */
volatile uint8_t auto_mode = 0;
uint8_t servoAngle = 0; // Current servo angle
uint8_t posi =10;
// UART VARIABLES
// Receiving buffers
uint8_t temp[20];          // Temporary buffer for incoming bytes
uint8_t command[10000];    // Buffer for assembling user commands
int ind_Com = 0;            // Index for command buffer
volatile int target_position = 0; // Desired motor target position

// Variables for controlling song playback
uint16_t arr_array[TOTAL_STEPS];  // Precomputed ARR values for each step
uint16_t psc_array[TOTAL_STEPS];  // Precomputed PSC values for each step
float desired_velocity = 0;       // Velocity to reach during song playback

int note_time_passed = 1;   // Flag to track note duration timeout
int is_playing = 0;       // 1 if a song is being played
int is_tunning = 0;       // 1 if tuning mode is active

// Guitar-related measurements
float string_length = 440.0f;  // String total length [mm]
float fret_positions[NUM_FRETS] = {0}; // Calculated fret positions along the string

// --- Predefined songs ---
// Example songs (replace or add as needed)
Song predefined_songs[] = {
    // Happy Birthday
    {
        .bpm = 80,
        .subdivision = 2,
        .frets_sequence = {0, 0, 2, 0, 5, 4, 100,100, 0, 0, 2, 0, 7, 5, 100,100,0,0,12,9,5,4,2,100,100,10,10,9,5,7,5,100,100},
        .index = 32
    },
    // Smoke on the Water
    {
        .bpm = 100,
        .subdivision = 2,
        .frets_sequence = {0,100, 3,100, 5,100,100, 0,100, 3,100, 6, 5,100,100,0,100, 3,100, 5,100,100, 3,100,0,100,100},
        .index = 27
    },
    // Come As You Are
    {
        .bpm = 110,
        .subdivision = 2,
        .frets_sequence = {0, 0, 1, 2,100, 5, 2, 5, 2, 2, 1, 0, 7, 0, 0, 100,7,0,1,2,5,2,5,2,2,1,0,7,0,0,100},
        .index = 32
    },
    // TEST
    {
        .bpm = 90,
        .subdivision = 2,
        .frets_sequence = {1, 0, 1, 2, 3, 0,0,0,0, 1,0,0,0,0,0,5, 0, 1, 0, 7, 0, 1,0,100,100,0,7,0,100,0,1},
        .index = 31
    }
};
#define NUM_PREDEFINED_SONGS (sizeof(predefined_songs)/sizeof(predefined_songs[0]))

volatile uint32_t tim5_overflow_count = 0; // Counter for TIM5 overflows (for timing)

uint16_t adc_buffer[ADC_BUFFER_SIZE];
float32_t fft_input[ADC_BUFFER_SIZE / 2];
float32_t fft_output[ADC_BUFFER_SIZE / 2];
arm_rfft_fast_instance_f32 fft_instance;

float32_t frequency;

// Agregar variable global para control de envío de frecuencia
volatile uint8_t send_frequency = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
float calculate_velocity(float frequency);
float get_timer1_frequency();
float get_current_velocity();
float calculate_required_frequency(float velocity);


float calculate_position(float steps);
float calculate_acceleration(float initial_velocity, float final_velocity, float time);
int find_next_real_fret(uint8_t* sequence, uint16_t length, uint16_t start_index);


TimerValues calculate_arr_psc(float velocity);
void calculate_steps_speed(float target_position, uint16_t* arr_array);
void calculate_fret_positions();
void toggle_solenoid();
void play_song();
void play_note();
void homing();
void chariot();
void start_move_to(float target_position_mm);
void tuning();
void interpret_command();
void set_servo_position(uint8_t angle);
void configure_timer1_for_velocity(float velocity);
void process_half_buffer(const uint16_t *src);
float32_t calculate_frequency(const float32_t *samples, uint32_t fft_size, float32_t sample_rate);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	HAL_UART_Receive_IT(&huart2, temp, 1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	calculate_fret_positions();

	// habilitar el reloj del timer:
	__HAL_RCC_TIM3_CLK_ENABLE();
	// habilitar el reloj del ADC:
	__HAL_RCC_ADC1_CLK_ENABLE();
	// habilitar el reloj del DMA:
	__HAL_RCC_DMA2_CLK_ENABLE();

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	arm_rfft_fast_init_f32(&fft_instance, ADC_BUFFER_SIZE/2);

	while (1)
	{
		if (is_playing == 1){
			play_song();
		}
		if (is_tunning == 1) {
			tuning();
		}
		if (auto_mode) {
		    if (status == 3) {
		        chariot();
		        status = 4;
		    }
		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65285;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 32642;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
	uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
	// Si APB1 prescaler > 1, los timers de APB1 corren a doble frecuencia:
	if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
		pclk1 *= 2;
	}

	// 'target' = total de ticks de timer por cada periodo de muestreo
	uint32_t target = pclk1 / SAMPLING_FREQ_HZ;

	// para que (ARR+1) <= 0xFFFF, despejamos PSC:
	uint32_t psc = (target - 1) / 0xFFFF;

	// ahora el valor real de ARR:
	uint32_t arr = (target / (psc + 1)) - 1;

	// cargar en registros:
	__HAL_TIM_SET_PRESCALER(&htim3, psc);
	__HAL_TIM_SET_AUTORELOAD(&htim3, arr);

	// habilitar el timer:
	HAL_TIM_Base_Start(&htim3);

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
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
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
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 99;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
	// habilitar DMA para TX:
	__HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SOL_GPIO_Port, SOL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin DIR_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENA_Pin */
  GPIO_InitStruct.Pin = ENA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SOL_Pin */
  GPIO_InitStruct.Pin = SOL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SOL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FC_Pin */
  GPIO_InitStruct.Pin = FC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

volatile float current_velocity = 0; // Latest calculated linear velocity [mm/s]

/**
 * @brief Calculate linear velocity based on the timer output frequency.
 * @param frequency Output frequency of the timer [Hz]
 * @return Calculated linear velocity [mm/s]
 */
float calculate_velocity(float frequency) {
	float circumference = M_PI * PULLEY_DIAMETER; // Pulley circumference [mm]
	float velocity = (frequency * circumference) / (STEPS_PER_REVOLUTION * MICROSTEPS); // Linear velocity
	current_velocity = velocity; // Update global velocity
	return velocity;
}

/**
 * @brief Get the actual output frequency of Timer 1.
 * @return Timer 1 output frequency [Hz]
 */
float get_timer1_frequency() {
	uint32_t timer_clock = HAL_RCC_GetPCLK2Freq(); // Retrieve the peripheral clock frequency for Timer 1
	uint32_t prescaler = htim1.Init.Prescaler + 1; // Effective prescaler value
	uint32_t period = htim1.Init.Period + 1;       // Effective ARR value
	float frequency = (float)timer_clock / (prescaler * period);
	return frequency;
}

/**
 * @brief Get the current linear velocity of the system.
 * @return Current linear velocity [mm/s]
 */
float get_current_velocity() {
	float frequency = get_timer1_frequency(); // Measure current timer frequency
	return calculate_velocity(frequency);     // Convert to linear velocity
}

/**
 * @brief Calculate ARR and PSC values for a given linear velocity.
 * @param velocity Desired linear velocity [mm/s]
 * @return TimerValues structure with ARR and PSC values
 */
TimerValues calculate_arr_psc(float velocity) {
	float radius = PULLEY_DIAMETER / 2.0;                          // Radius of pulley [mm]
	float angular_velocity = velocity / radius;                   // Angular velocity [rad/s]
	float frequency = (angular_velocity * STEPS_PER_REVOLUTION) / (2 * M_PI); // Motor step frequency [steps/s]

	uint32_t tentative_arr = (uint32_t)(F_CPU / frequency) - 1;    // Initial ARR value
	uint16_t psc = 0;                                              // Initial prescaler

	// Adjust prescaler if ARR exceeds 16-bit timer limit
	if (tentative_arr > 0xFFFF) {
		psc = (tentative_arr / 0xFFFF) + 1;
		tentative_arr = (uint32_t)(F_CPU / ((psc + 1) * frequency)) - 1;
	}

	TimerValues values;
	values.arr = (uint16_t)tentative_arr;
	values.psc = psc;
	return values;
}

/**
 * @brief Set the desired linear velocity and configure system tick.
 * @param velocity Desired linear velocity [mm/s]
 */
void set_velocity(float velocity) {
	if (velocity < 0 || velocity > MAX_VELOCITY) {
		// Ignore invalid velocity values
		return;
	}

	desired_velocity = velocity; // Update desired velocity

	// Configure SysTick timer to trigger every 1 ms
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
}

/**
 * @brief Configure Timer 1 to generate PWM corresponding to the desired velocity.
 * @param velocity Desired linear velocity [mm/s]
 */
void configure_timer1_for_velocity(float velocity) {
	if (velocity < 0 || velocity > MAX_VELOCITY) {
		// Ignore invalid velocity values
		return;
	}

	TimerValues values = calculate_arr_psc(velocity);

	// Update Timer 1 settings
	htim1.Init.Prescaler = values.psc;
	htim1.Init.Period = values.arr;
	htim1.Instance->CCR1 = values.arr / 2; // Set 50% duty cycle

	// Reinitialize Timer 1
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}

	// Start Timer 1
	if (HAL_TIM_Base_Start(&htim1) != HAL_OK) {
		Error_Handler();
	}
}


/**
 * @brief Calculate linear position [mm] from number of motor steps.
 * @param steps Number of motor steps
 * @return Position in millimeters
 */
float calculate_position(float steps) {
	float circumference = M_PI * PULLEY_DIAMETER; // Pulley circumference [mm]
	float position = (steps * circumference) / STEPS_PER_REVOLUTION;
	return position;
}

/**
 * @brief Calculate linear acceleration given initial and final velocities over time.
 * @param initial_velocity Initial velocity [mm/s]
 * @param final_velocity Final velocity [mm/s]
 * @param time Time interval [s]
 * @return Acceleration [mm/s²]
 */
float calculate_acceleration(float initial_velocity, float final_velocity, float time) {
	float acceleration = (final_velocity - initial_velocity) / time;
	return acceleration;
}

/**
 * @brief Precompute timer ARR and PSC values to perform a motion to a target position
 *        with a trapezoidal velocity profile.
 * @param target_position1 Target position in millimeters
 * @param arr_array Output array to store ARR values step by step
 */
void calculate_steps_speed(float target_position1, uint16_t* arr_array) {
	if (target_position1 < 0 || target_position1 > MAX_POSITION) {
		return; // Ignore invalid target positions
	}
	target_position = target_position1;
	float distance;

	// Determine movement direction and set motor DIR pin
	if (kinematic.position < target_position) {
		distance = target_position - kinematic.position;
		controller.dir = 1; // Forward
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
	} else {
		distance = kinematic.position - target_position;
		controller.dir = 0; // Reverse
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
	}

	float steps_per_mm = STEPS_PER_MM;
	uint16_t total_steps = (uint16_t)(distance * steps_per_mm);

	// Save total steps to move
	controller.step = total_steps;

	// Compute time and distance to accelerate to max speed
	float acceleration_time = (float)MAX_VELOCITY / (float)MAX_ACCELERATION;
	float acceleration_distance = 0.5f * (float)MAX_ACCELERATION * powf(acceleration_time, 2);

	// Calculate cruising distance (distance traveled at max speed)
	float cruise_distance = distance - 2 * acceleration_distance;
	if (cruise_distance < 0) {
		// No cruising phase, triangular profile (only acceleration and deceleration)
		acceleration_distance = distance / 2;
		cruise_distance = 0;
	}

	float cruise_time = cruise_distance / MAX_VELOCITY;

	// Calculate total motion time
	float total_time = 2 * acceleration_time + cruise_time;

	// Reset kinematic state
	kinematic.velocity = 0;
	kinematic.acceleration = MAX_ACCELERATION;

	// Precompute timer values for each step
	for (uint16_t step = 0; step < total_steps; step++) {
		float t = (float)step / (float)total_steps * total_time;

		if (t < acceleration_time) {
			// Acceleration phase
			kinematic.velocity = (float)MAX_ACCELERATION * t;
		} else if (t < acceleration_time + cruise_time) {
			// Constant velocity phase
			kinematic.velocity = MAX_VELOCITY;
		} else {
			// Deceleration phase
			kinematic.velocity = MAX_VELOCITY - MAX_ACCELERATION * (t - acceleration_time - cruise_time);
		}

		// Calculate ARR/PSC values for the current velocity
		TimerValues values = calculate_arr_psc(kinematic.velocity * MICROSTEPS);
		arr_array[step] = values.arr;
		psc_array[step] = values.psc;
	}

	// Configure timer for the first step immediately
	TIM1->ARR = arr_array[0];
	TIM1->PSC = psc_array[0];
	TIM1->CCR1 = arr_array[0] / 2; // 50% duty cycle
}


/**
 * @brief Toggle the solenoid state (ON/OFF).
 *        If the solenoid is off, it turns it on; if it's on, it turns it off.
 */
void toggle_solenoid() {
	if (solenoid.state == 0) {
		// Solenoid is off, turn it on
		HAL_GPIO_WritePin(SOL_GPIO_Port, SOL_Pin, GPIO_PIN_SET);
		solenoid.state = 1;
	} else {
		// Solenoid is on, turn it off
		HAL_GPIO_WritePin(SOL_GPIO_Port, SOL_Pin, GPIO_PIN_RESET);
		solenoid.state = 0;
	}
}

/**
 * @brief Calculate fret positions based on the string length and musical scale.
 *        Positions are adjusted by a mechanical offset based on the cart size.
 */
void calculate_fret_positions() {
	float offset = 17.2f; // Offset corresponding to half the solenoid carriage length [mm]

	for (int i = 0; i < (sizeof(fret_positions) / sizeof(fret_positions[0])); i++) {
		// Calculate raw position for each fret based on 12-TET (Twelve-tone equal temperament)
		float raw_position = string_length * (1.0f - 1.0f / powf(2.0f, i / 12.0f));

		// Apply mechanical offset
		fret_positions[i] = raw_position - offset;

		// Clamp position to 0 if it becomes negative
		if (fret_positions[i] < 0.0f) {
			fret_positions[i] = 0.0f;
		}
	}
}

// UART FUNCTIONS

/**
 * @brief Transmit a null-terminated string over UART2.
 * @param str Pointer to the string to transmit
 */
void send_string(char *str)
{
	int len = strlen(str); // Calculate the length of the string
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)str, len);
}

/**
 * @brief UART receive complete interrupt callback.
 *        Handles command parsing from user input.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		char received_char = temp[0]; // Capture the received character

		// Handle special characters
		if (received_char == ':') {
			// Start of a new command: clear the command buffer
			ind_Com = 0;
			memset(command, 0, sizeof(command));
		}
		else if (received_char == 0x08 || received_char == 0x7F) {
			// Backspace or delete: remove last character from buffer
			if (ind_Com > 0) {
				ind_Com--;
				command[ind_Com] = '\0';
			}
		}
		else if (received_char == '\n' || received_char == '\r') {
			// Enter key pressed: finalize and process the command
			if (ind_Com > 0) {  // Solo procesar si hay caracteres en el buffer
				command[ind_Com] = '\0';  // Asegurar que el comando termine en null
				interpret_command(); // Interpret and execute the received command
				ind_Com = 0;
				memset(command, 0, sizeof(command));
			}
		}
		else if (ind_Com < sizeof(command) - 1) {
			// Regular printable character: store it
			command[ind_Com++] = received_char;
		}

		// Reactivate UART reception for next character
		HAL_UART_Receive_IT(&huart2, temp, 1);
	}
}


/**
 * @brief Timer period elapsed callback.
 *        Handles actions depending on which timer triggered the interrupt.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	static uint8_t accumulator = 0; // Accumulator for microstepping

	if (htim->Instance == TIM1) {
		// Timer 1: Motor motion control
		switch (status) {
		case 2: // Homing operation in progress
			// No action needed, waiting for external event (limit switch)
			break;

		case 4: // Normal movement to a target position
			if (controller.step > 0) {
				// Decrement step counter only after completing a microstep batch
				if (accumulator++ >= MICROSTEPS) {
					accumulator = 0;
					controller.step--;

					// Update timer parameters for new velocity
					TIM1->ARR = arr_array[controller.step];
					TIM1->PSC = psc_array[controller.step];
					TIM1->CCR1 = arr_array[controller.step] / 2; // 50% duty cycle
				}
			} else {
				// All steps completed: stop PWM and update system status
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
				status = 3; // Now "position referenced"

				// If motor is still enabled, update kinematic position
				if (HAL_GPIO_ReadPin(ENA_GPIO_Port, ENA_Pin) == GPIO_PIN_RESET) {
					kinematic.position = target_position;
				}
			}
			break;

		default:
			break;
		}
	}

	else if (htim->Instance == TIM2) {
		// Timer 2: Used to track note duration during song playback
		note_time_passed = 1;
		HAL_TIM_Base_Stop_IT(&htim2); // Stop the timer after timeout
	}

	else if (htim->Instance == TIM5) {
		// Timer 5: Overflow counter for high precision delays
		tim5_overflow_count++;
	}
}



/**
 * @brief External interrupt callback.
 *        Handles actions when the limit switch (endstop) is triggered.
 * @param GPIO_Pin Pin number that triggered the interrupt
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == FC_Pin) { // Check if the interrupt was generated by the limit switch
		switch (status) {
		case 2: // Homing in progress
			// Stop motor PWM to halt movement
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

			// Confirm that the limit switch is indeed activated (debounce check)
			if (HAL_GPIO_ReadPin(FC_GPIO_Port, FC_Pin) == GPIO_PIN_SET) {
				// Homing successful, update system status
				send_string(":HR\r\n"); // Send homing result message")
				status = 3; // Position referenced and ready
			} else {
				// False trigger or incomplete homing: continue moving slowly toward switch
				kinematic.position = 0; // Set reference position to 0 mm
				configure_timer1_for_velocity(homing_speed / 2); // Reduce speed

				// Change motor direction toward the limit switch
				HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);

				// Restart PWM indefinitely to continue homing
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			}
			break;

		case 4: // Movement in progress
			// Emergency stop: movement interrupted by limit switch
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			status = 0; // Set system status to "not calibrated"
			break;

		default:
			// No action for other statuses
			break;
		}
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	if (send_frequency) {
		process_half_buffer(&adc_buffer[0]);
		if (frequency > 0 && frequency <= 2400.0f) {
			char buffer[32];
			int len = sprintf(buffer, ":F%.2f\r\n", frequency);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buffer, len);
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (send_frequency) {
		process_half_buffer(&adc_buffer[ADC_BUFFER_SIZE / 2]);
		if (frequency > 0 && frequency <= 2400.0f) {
			char buffer[32];
			int len = sprintf(buffer, ":F%.2f\r\n", frequency);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buffer, len);
		}
	}
}


/**
 * @brief Initiate the homing procedure.
 *        Moves the mechanism toward the limit switch at homing speed.
 */
void homing() {
	// Set motor speed for homing
	configure_timer1_for_velocity(homing_speed);

	// Move toward the limit switch
	HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

	// Start PWM indefinitely; movement will be stopped by external interrupt
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void chariot() {

	 if (posi == 10) {

		 posi = 90;
		} else {

				posi = 10;
		}

	 calculate_steps_speed(posi, arr_array);
	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	 play_note();
}

void start_move_to(float target_position_mm) {

    calculate_steps_speed(target_position_mm, arr_array);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    status = 4; // "Moving to target position"

}
/**
 * @brief Find the next real fret in a sequence (skip pauses and special codes).
 * @param sequence Pointer to the fret sequence array
 * @param length Total number of elements in the sequence
 * @param start_index Index from which to start searching
 * @return Index of the next real fret, or -1 if none found
 */
int find_next_real_fret(uint8_t* sequence, uint16_t length, uint16_t start_index) {
	for (uint16_t i = start_index; i < length; i++) {
		if (sequence[i] > 0 && sequence[i] < 100) {
			return i;
		}
	}
	return -1;
}

/**
 * @brief Set the servo to a specific angle.
 * @param angle Target angle [0°–180°]
 */
void set_servo_position(uint8_t angle) {
	if (angle > 180) angle = 180; // Clamp maximum angle

	uint16_t pulse = 500 + ((angle * 2500) / 180); // Map angle to pulse width in microseconds
	uint16_t ccr_value = (pulse * 19999) / 20000;  // Map pulse width to CCR1 value

	TIM4->CCR1 = ccr_value; // Update PWM duty cycle to move servo
}

/**
 * @brief Wait for a specified time in microseconds using TIM5.
 * @param delay_us Desired delay in microseconds
 */
void wait_using_tim5_us(uint32_t delay_us) {
	uint32_t ticks_per_us = (F_CPU / (htim5.Init.Prescaler + 1)) / 1000000;
	uint64_t target_ticks = (uint64_t)delay_us * ticks_per_us;

	// --- Reset counters and flags ---
	tim5_overflow_count = 0;
	__HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	__HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);

	HAL_TIM_Base_Start(&htim5);

	uint64_t ticks_total = 0;

	// Busy-wait until the desired number of ticks have passed
	while (ticks_total < target_ticks) {
		uint32_t current_count = __HAL_TIM_GET_COUNTER(&htim5);
		ticks_total = ((uint64_t)tim5_overflow_count * (uint64_t)(htim5.Init.Period + 1)) + current_count;
	}

	HAL_TIM_Base_Stop(&htim5);
	__HAL_TIM_DISABLE_IT(&htim5, TIM_IT_UPDATE);
}

float32_t calculate_frequency(const float32_t *samples, uint32_t fft_size, float32_t sample_rate) {
	static float32_t max_value;
	static uint32_t max_index;

	// 1. Remover offset DC
	float32_t mean;
	arm_mean_f32(samples, fft_size, &mean);
	for(uint32_t i = 0; i < fft_size; i++) {
		fft_input[i] = samples[i] - mean;
	}

	// 2. Calcular FFT
	arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);

	// 3. Calcular magnitudes para primera mitad del espectro
	for(uint32_t i = 0; i < fft_size/2; i++) {
		float32_t real = fft_output[2*i];
		float32_t imag = fft_output[2*i+1];
		fft_input[i] = sqrtf(real*real + imag*imag); // Reutilizamos fft_input para magnitudes
	}

	// 4. Encontrar máximo ignorando DC (primer bin)
	arm_max_f32(&fft_input[1], (fft_size/2)-1, &max_value, &max_index);
	max_index++; // Compensar por haber ignorado el primer bin

	// 5. Calcular frecuencia
	return (float32_t)max_index * sample_rate / (float32_t)fft_size;
}

void process_half_buffer(const uint16_t *src) {
	// Convertir a float y centrar alrededor de 0
	for(uint32_t i = 0; i < ADC_BUFFER_SIZE/2; i++) {
		fft_input[i] = (float32_t)src[i] - 2048.0f;  // Asumiendo ADC de 12 bits
	}

	frequency = calculate_frequency(fft_input, ADC_BUFFER_SIZE/2, SAMPLING_FREQ_HZ);
}


/**
 * @brief Tuning mode function.
 *        Repeatedly triggers the servo to play a note at fixed intervals.
 */
void tuning() {
	play_note();                          // Simulate picking a note with the servo
	wait_using_tim5_us(1000000);          // Wait for 1 second (1,000,000 microseconds)
}

/**
 * @brief Simulates a pluck by toggling the servo between two angles.
 *        This creates a repetitive picking motion.
 */
void play_note() {
	// Alternate between two predefined angles to simulate a pluck
	if (servoAngle == 0) {
		servoAngle = 30;
	} else {
		servoAngle = 0;
	}

	// Move the servo to the updated angle
	set_servo_position(servoAngle);
}


/**
 * @brief Plays the currently loaded song by interpreting fret codes,
 *        controlling motor positioning, solenoid activation, and servo picking.
 */
void play_song() {
	int current_fret = -1; // Last fret played
	int i = 0;             // Current index in fret sequence
	uint8_t solenoid_is_active = 0;

	int is_moving = 0;
	int target_fret = -1;

	while (i < current_song.index) {
		uint8_t code = current_song.frets_sequence[i];

		// Validate code
		if (!(code <= 99 || code == 100 || code == 101)) {
			send_string(":X1\r\n"); // Error de valor inválido
			i++;
			continue;
		}

		// --- Anticipated positioning for non-real notes (rests or holds) ---
		if (!is_moving && (code == 0 || code == 100 || code == 101)) {
			int next_index = find_next_real_fret(current_song.frets_sequence, current_song.index, i + 1);
			if (next_index != -1) {
				target_fret = current_song.frets_sequence[next_index];
				if (target_fret > 0 && target_fret < NUM_FRETS && target_fret != current_fret) {
					float pos = fret_positions[target_fret];
					calculate_steps_speed(pos, arr_array);
					HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
					status = 4;
					is_moving = 1;
				}
			}
		}

		// --- Handle special codes: 0 (pluck), 100 (hold), 101 (pause) ---
		if (code == 0 || code == 100 || code == 101) {
			if (solenoid_is_active) {
				toggle_solenoid();
				solenoid_is_active = 0;
				wait_using_tim5_us(5000); // release delay
			}

			if (code == 0) {
				play_note();                 // pluck the string
				wait_using_tim5_us(40000);   // delay after pluck
			}

			// Wait for the note duration
			note_time_passed = 0;
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			HAL_TIM_Base_Start_IT(&htim2);
			while (!note_time_passed);

			i++;
			continue;
		}

		// --- Handle real note ---
		int fret = code;

		// Wait for movement to finish, if any
		if (status == 4) {
			while (status == 4);
			is_moving = 0;
			if (target_fret >= 0 && target_fret == fret) {
				current_fret = target_fret;
			}
		}

		// Move to the correct fret if not already there
		if (fret != current_fret) {
			if (fret >= NUM_FRETS) {
				send_string(":X2\r\n"); // Error de traste fuera de rango
				i++;
				continue;
			}

			float pos = fret_positions[fret];
			calculate_steps_speed(pos, arr_array);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			status = 4;
			while (status == 4);
			current_fret = fret;
			is_moving = 0;
		}

		// Count repeated notes or holds for this fret
		int press_count = 1;
		int j = i + 1;
		while (j < current_song.index) {
			uint8_t next = current_song.frets_sequence[j];
			if (next == 100 || next == fret) {
				press_count++;
				j++;
			} else break;
		}

		// Press the fret with solenoid if not already active
		if (!solenoid_is_active) {
			toggle_solenoid();
			solenoid_is_active = 1;
		}

		wait_using_tim5_us(8000);  // press descent delay
		play_note();               // simulate picking
		wait_using_tim5_us(16000); // delay after pluck

		// Wait for the note to finish
		note_time_passed = 0;
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		HAL_TIM_Base_Start_IT(&htim2);
		while (!note_time_passed);

		// Release the solenoid after the note
		if (solenoid_is_active) {
			toggle_solenoid();
			solenoid_is_active = 0;
		}

		i++;
	}

	// Ensure solenoid is off at the end
	if (solenoid_is_active) {
		toggle_solenoid();
	}
	send_string(":GR\r\n"); // Canción terminada
	is_playing = 0;
	status = 3;
}


/**
 * @brief Interprets and executes the command stored in the 'command' buffer.
 *        Commands are sent via UART in text form and start with a character identifier.
 */
void interpret_command()
{
	int cmd;
	switch (command[0]) {
	char confirm[64];

	case 'f':  // Set motor velocity. Format: :fXXX
		cmd = atoi((char *)&command[1]);
		set_velocity(cmd);
		break;

	case 'H':  // Start homing if status allows it
		switch (status) {
		case 1: // Motor enabled but not homed
			status = 2;
			homing();
			break;
		default:
			// Ignore if in invalid state for homing
			break;
		}
		break;

	case 'P':  // Move to absolute position. Format: :PXXX
		if (status == 3) {
			cmd = atoi((char *)&command[1]);
			calculate_steps_speed(cmd, arr_array);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			status = 4;
		}
		break;

	case 'E':  // Enable or disable motor. Format: :E1 to enable, :E0 to disable
		cmd = atoi((char *)&command[1]);
		if (status == 0 && cmd != 0) {
			// Enable motor
			HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_RESET);
			status = 1;
			controller.enable = 1;
			send_string(":EN\r\n"); // Enabled motor
		} else if (cmd == 0) {
			// Disable motor and reset reference
			HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);
			status = 0;
			controller.enable = 0;
			send_string(":ED\r\n"); // Disabled motor
		}
		break;
	case 'I':
			send_string(":I\r\n");
		    if (status == 3){   // permet le mouvement
			chariot();
		    status = 4;   // mouvement en cours
			}
			break;

	case 'J':
	    send_string(":J\r\n");
	    auto_mode = 1;   // Démarre le mouvement continu
	    break;

	case 'K':
	    send_string(":K\r\n");
	    auto_mode = 0;   // Arrête le mouvement
	    break;
	case 'S':  // Toggle solenoid state
		toggle_solenoid();
		break;

	case 'V':  // Move servo to specified angle. Format: :VXXX
		cmd = atoi((char *)&command[1]);
		set_servo_position(cmd);
		break;

	case 'N':  // Play a single note using the servo
		play_note();
		send_string(":N\r\n");
		break;

	case 'B':  // Set BPM and subdivision. Format: :B120_4
		if (strlen((char *)command) < 6 || command[4] != '_') {
			send_string(":X3\r\n"); // Error de formato BPM
			break;
		}
		int bpm = atoi((char *)&command[1]);
		int subdiv = atoi((char *)&command[5]);

		if (bpm <= 0 || bpm > 300 || subdiv <= 0 || subdiv > 16) {
			send_string(":X4\r\n"); // Error de valores BPM
			break;
		}

		current_song.bpm = bpm;
		current_song.subdivision = subdiv;

		// Configure TIM2 for note duration
		uint32_t note_duration_ms = 60000 / (bpm * subdiv);
		uint32_t tick_freq = 1000000; // Timer counts in microseconds
		uint32_t prescaler = F_CPU / tick_freq - 1;
		uint32_t arr_value = note_duration_ms * 1000;

		htim2.Init.Prescaler = prescaler;
		htim2.Init.Period = arr_value;
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		HAL_TIM_Base_Init(&htim2);
		HAL_TIM_OC_Init(&htim2);
		HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE);
		__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
		__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
		break;

	case 'A':  // Toggle tuning mode (looping play_note)
		is_tunning = !is_tunning;
		if (is_tunning) {
			send_frequency = 1;
			send_string(":A\r\n");
		} else {
			send_frequency = 0;
			send_string(":AR\r\n");
			if (solenoid.state == 1) {
				toggle_solenoid(); // Ensure solenoid is released
			}
		}
		break;

	case 'T':  // Add fret to sequence. Format: :Txx, :T-- or :T**
		if (strlen((char *)command) < 3) {
			send_string(":X5\r\n"); // Error de formato T
			break;
		}

		if (current_song.index >= MAX_FRETS_SEQUENCE_LENGTH) {
			send_string(":X6\r\n"); // Error secuencia llena
			break;
		}

		char c1 = command[1];
		char c2 = command[2];
		uint8_t value;

		if (c1 == '*' && c2 == '*') {
			value = 100; // Hold
		} else if (c1 == '-' && c2 == '-') {
			value = 101; // Pause
		} else if (isdigit(c1) && isdigit(c2)) {
			value = (c1 - '0') * 10 + (c2 - '0');
		} else {
			send_string(":X7\r\n"); // Error combinación
			break;
		}

		current_song.frets_sequence[current_song.index++] = value;
		send_string(":T\r\n"); // Traste agregado
		break;

	case 'G':  // Start song playback
		if (status != 3) {
			send_string(":X8\r\n"); // Error homing
			break;
		}
		if (current_song.index == 0) {
			send_string(":X9\r\n"); // Error no song
			break;
		}
		if (current_song.bpm == 0) {
			send_string(":X10\r\n"); // Error no bpm
			break;
		}
		is_playing = 1;
		send_string(":G\r\n"); // Canción iniciada
		break;

	case 'C':
		/*
		 * @brief Load a predefined song by index.
		 *        Command format: :C<n> where n is the song number (0-based).
		 *        This will clear the current_song struct and load the selected song.
		 */
		if (status != 3) {
			send_string(":X11\r\n"); // Error: must home before loading song
			break;
		}
		int song_idx = 0;
		if (strlen((char*)command) > 1 && isdigit(command[1])) {
			song_idx = atoi((char*)&command[1]);
		}
		if (song_idx < 0 || song_idx >= NUM_PREDEFINED_SONGS) {
			send_string(":X12\r\n"); // Error: invalid song number
			break;
		}
		// Copiar toda la estructura Song
		memcpy(&current_song, &predefined_songs[song_idx], sizeof(Song));
		// Configurar el timer como en 'B'
		{
			uint32_t note_duration_ms = 60000 / (current_song.bpm * current_song.subdivision);
			uint32_t tick_freq = 1000000;
			uint32_t prescaler = F_CPU / tick_freq - 1;
			uint32_t arr_value = note_duration_ms * 1000;

			htim2.Init.Prescaler = prescaler;
			htim2.Init.Period = arr_value;
			htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
			htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
			htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
			HAL_TIM_Base_Init(&htim2);
			HAL_TIM_OC_Init(&htim2);
			HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE);
			__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
			__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
		}
		send_string(":C\r\n"); // Song loaded
		break;

	case 'l':  // Set string length and update fret positions. Format: :lXXX
		int new_length = atoi((char *)&command[1]);
		string_length = (float)new_length;
		calculate_fret_positions();
		break;

	case 'R':
		/*
		 * @brief Software reset command handler.
		 *        When the command ':R' is received, the microcontroller will perform a full system reset
		 *        using the NVIC_SystemReset() function from the STM32 HAL.
		 *        This brings all peripherals and variables to their initial state as after a power-on reset.
		 */
		NVIC_SystemReset();
		break;

	default:
		// Ignore unknown command
		break;
	}
}


/* USER CODE END 4 */

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
