/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdlib.h>
#include<stdint.h>
#include "lut.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum pulse_type{
	Async,
	SyncMp,
	S3p,
	W3p,
	CHM,
	SHE,
	HO
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t alpha_sort[3][90];
uint32_t alpha_est[3][90];
uint16_t alpha_cur[3][90];
int alpha_num = 7;
int alpha_num_cur = 7;
float ratio = 0.0;
float AMP;
float OFFSET;
float dutyu;
float dutyv;
float dutyw;
float dutyun;
float dutyvn;
float dutywn;
float Asdutyu;
float Asdutyv;
float Asdutyw;
int dir;
int motorState = 0;
int ampINT = 0;
int transit = 1;
float frq;
float basfrq = 1;
float basfrq_Jerk = 0.0;
int JerkPole;
int tca_cnt = 0;
int tcb_cnt = 0;
int tcb2_cnt = 0;
int pulse_mode;
int pnum_PWM;
float PER;
float PER_PWM;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

float min(float a, float b);
float max(float a, float b);
void swap (uint32_t *x, uint32_t *y);
void shell_sort (void);
void makePER(const int alpha[][alpha_num], const int pole[], int amp);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim == &htim1){
    	if(pulse_mode == 0){
    		PER_PWM = 2500000 / frq;
    		dutyu = Asdutyu * AMP / 32767 + OFFSET;
    		dutyv = Asdutyv * AMP / 32767 + OFFSET;
    		dutyw = Asdutyw * AMP / 32767 + OFFSET;
    	}
    	else if(pulse_mode == 1){
    		PER_PWM = 2500000 / (basfrq*pnum_PWM);
    		if(TIM1->CNT > PER_PWM/2){
    			dutyu = SYNC[0][tca_cnt%(pnum_PWM*2)];
    		}
    		else{
    			dutyu = SYNC[0][tca_cnt%(pnum_PWM*2)];
    		}
    	}
    	TIM1->ARR = (uint16_t)PER_PWM;
    	TIM1->CCR1 = (uint16_t)(PER_PWM*dutyu);
    	TIM1->CCR2 = (uint16_t)(PER_PWM*dutyv);
    	TIM1->CCR3 = (uint16_t)(PER_PWM*dutyw);
    }
    if(htim == &htim6){
    	if(tcb_cnt == 0){
    		if(pulse_mode == 0 || pulse_mode == 1){
    			if(transit == 1){
    				GPIO_InitTypeDef GPIO_InitStruct;
    			    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    			    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    			    GPIO_InitStruct.Pull = GPIO_NOPULL;
    			    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    			    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    			    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    			    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    			    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    			    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    			    tcb2_cnt = 0;
    			    transit = 0;
    			}
    		}
    		alpha_num_cur = alpha_num;
    		for(int i = 0; i < alpha_num_cur*12+6; i++){
    			alpha_cur[0][i] = (uint16_t)(alpha_est[0][i]);
    			alpha_cur[1][i] = (uint16_t)(alpha_est[1][i]);
    			alpha_cur[2][i] = (uint16_t)(alpha_est[2][i]);
    		}
    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    	}
    	if(alpha_cur[1][tcb_cnt] == 1){
    		if(alpha_cur[2][tcb_cnt]==1){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);}
    		else if(alpha_cur[2][tcb_cnt]==2){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);}
    		else if(alpha_cur[2][tcb_cnt]==4){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);}
    		}
    	else if(alpha_cur[1][tcb_cnt] == 0){
    		if(alpha_cur[2][tcb_cnt]==1){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);}
    		else if(alpha_cur[2][tcb_cnt]==2){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);}
    		else if(alpha_cur[2][tcb_cnt]==4){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);}
    	}
    	TIM6->ARR = alpha_cur[0][tcb_cnt];
    	tcb_cnt += 1;
    	if(tcb_cnt > alpha_num_cur*12+6 - 1){tcb_cnt = 0;}
    }
    if(htim == &htim2){
    	if(tcb2_cnt == 0){
    		if(pulse_mode != 0 && pulse_mode != 1){
    			if(transit == 0){
    				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    				GPIO_InitTypeDef GPIO_InitStruct;
    				GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    				GPIO_InitStruct.Pull = GPIO_NOPULL;
    				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    				transit = 1;
    				tcb_cnt = 0;
    			}
    		}
    	}
    	TIM2->ARR = (uint32_t)(303030.30303 / basfrq);
    	if(pulse_mode == 0 || transit == 0){
    		Asdutyu = SYNC[0][tcb2_cnt%66];
    		Asdutyv = SYNC[0][(tcb2_cnt+22)%66];
    		Asdutyw = SYNC[0][(tcb2_cnt+44)%66];
    	}
    	tcb2_cnt++;
    	tcb2_cnt %= 66;
    }
    if(htim == &htim7){
    	basfrq += (0.0005 * dir);
    	if(JerkPole != 0){basfrq_Jerk += (0.008*JerkPole);}
    }
}
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  makePER(_7alpha, _7alpha_pole, ampINT);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  pulse_mode = Async;
  frq = 525;
  basfrq = 0.1;
  AMP = min(max(basfrq * 0.0103, 0), 1);
  OFFSET = min(max(0.5 - basfrq * 0.00515, 0), 0.5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if ((GPIOC->IDR & GPIO_PIN_0) == 0){
		  //acceleration setting
		  motorState = 1;
		  if(basfrq_Jerk < basfrq){
			  JerkPole = 1;
			  ratio = min(max(basfrq_Jerk * 8.35, 1), 501);
			  AMP = min(max(basfrq_Jerk * 0.0185, 0), 1);
	          OFFSET = min(max(0.5 - basfrq_Jerk * 0.00925, 0), 0.5);
		  }
		  else{
			  dir = 1;
			  JerkPole = 0;
			  basfrq_Jerk = basfrq + 1;
			  ratio = min(max(basfrq * 8.35, 1), 501);
			  AMP = min(max(basfrq * 0.0185, 0), 1);
			  OFFSET = min(max(0.5 - basfrq * 0.00925, 0), 0.5);
		  }
	  }
	  else if ((GPIOC->IDR & GPIO_PIN_1) == 0){
		  motorState = -1;
		  if(basfrq_Jerk < basfrq){
			  JerkPole = 1;
			  ratio = min(max(basfrq_Jerk * 6.863, 1), 501);
			  AMP = min(max(basfrq_Jerk * 0.0103, 0), 1);
	            OFFSET = min(max(0.5 - basfrq_Jerk * 0.00515, 0), 0.5);
	  		}
	  		else{
	  			dir = -1;
	  			JerkPole = 0;
	  			basfrq_Jerk = basfrq;
	  			ratio = min(max(basfrq * 6.863, 1), 501);
	  			AMP = min(max(basfrq * 0.0103, 0), 1);
	            OFFSET = min(max(0.5 - basfrq * 0.00515, 0), 0.5);
	  		}
	  	}
	  	else{
	  		dir = 0;
	  		if(basfrq_Jerk > 0){
	  			JerkPole = -1;
	  			ratio = min(max(basfrq_Jerk * 6.863, 10), 501);
	  			AMP = min(max(basfrq_Jerk * 0.0103, 0), 1);
	            OFFSET = min(max(0.5 - basfrq_Jerk * 0.00515, 0), 0.5);
	  		}
	  		else{
	  			JerkPole = 0;
	  			basfrq_Jerk = 0;
	  			motorState = 0;
	  		}
	  	}

	  	if(motorState == 1){
	  		/*if(basfrq >= 80 && dir == 1){alpha_num = 0;}
	  		else if(basfrq >= 59){pulse_mode = CHM;alpha_num = 1;}
	  		else if(basfrq >= 57){pulse_mode = CHM;alpha_num = 2;}
	  		else if(basfrq >= 43.5){pulse_mode = CHM;alpha_num = 3;}
	  		else if(basfrq >= 37){pulse_mode = CHM;alpha_num = 4;}
	  		else if(basfrq >= 30){pulse_mode = CHM;alpha_num = 5;}
	  		else if(basfrq >= 27){pulse_mode = CHM;alpha_num = 6;}
	      	else if(basfrq >= 24){pulse_mode = CHM;alpha_num = 7;}
	  		else if(basfrq >= 0){pulse_mode = Async;frq = 400;}*/
	  		if(basfrq >= 80){pulse_mode = CHM;alpha_num = 0;}
	  		else if(basfrq >= 65){pulse_mode = CHM;alpha_num = 2;}
	  		else if(basfrq >= 63){pulse_mode = CHM;alpha_num = 3;}
	  		else if(basfrq >= 45){pulse_mode = CHM;alpha_num = 4;}
	  		else if(basfrq >= 40){pulse_mode = CHM;alpha_num = 5;}
	  		else if(basfrq >= 0){pulse_mode = Async;frq = 1000;}
	  	}
	  	else if(motorState == -1){
	  		if(basfrq >= 80 && dir == -1){pulse_mode = CHM;alpha_num = 0;}
	  		else if(basfrq >= 70.7){pulse_mode = CHM;alpha_num = 1;}
	  		else if(basfrq >= 63){pulse_mode = CHM;alpha_num = 2;}
	  		else if(basfrq >= 41){pulse_mode = CHM;alpha_num = 3;}
	  		else if(basfrq >= 34.5){pulse_mode = CHM;alpha_num = 4;}
	  		else if(basfrq >= 29){pulse_mode = CHM;alpha_num = 5;}
	  		else if(basfrq >= 25){pulse_mode = CHM;alpha_num = 6;}
	  		else if(basfrq >= 22.5){pulse_mode = CHM;alpha_num = 7;}
	  		else if(basfrq >= 0){pulse_mode = Async;frq = 400;}
	  	}

	  	if(alpha_num == 7 && pulse_mode == 4){ampINT = (uint16_t)ratio; makePER(_7alpha, _7alpha_pole, ampINT);}
	  	else if(alpha_num == 6 && pulse_mode == 4){ampINT = (uint16_t)ratio; makePER(_6alpha, _6alpha_pole, ampINT);}
	  	else if(alpha_num == 5 && pulse_mode == 4){ampINT = (uint16_t)ratio; makePER(_5alpha, _5alpha_pole, ampINT);}
	  	else if(alpha_num == 4 && pulse_mode == 4){ampINT = (uint16_t)ratio; makePER(_4alpha, _4alpha_pole, ampINT);}
	  	else if(alpha_num == 3 && pulse_mode == 4){ampINT = (uint16_t)ratio; makePER(_3alpha, _3alpha_pole, ampINT);}
	  	else if(alpha_num == 2 && pulse_mode == 4){ampINT = (uint16_t)ratio; makePER(_2alpha, _2alpha_pole, ampINT);}
	  	else if(alpha_num == 1 && pulse_mode == 4){ampINT = (uint16_t)ratio; makePER(_1alpha, _1alpha_pole, ampINT);}
	  	else if(alpha_num == 0 && pulse_mode == 4){ampINT = (uint16_t)ratio; makePER(_1alpha, _1alpha_pole, 100);}
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 3;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim1.Init.Period = 1;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 2500;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
float min(float a, float b){
    return (a < b) ? a : b;
}
float max(float a, float b) {
    return (a > b) ? a : b;
}

void swap (uint32_t *x, uint32_t *y){
	uint32_t temp;
	temp = *x;
	*x = *y;
	*y = temp;
}
void shell_sort (void){
	int i, j, h, array_size;
	array_size = alpha_num * 12 + 6;
	for(h = 1; h <= array_size/9; h = 3*h + 1);
	for( ; h > 0; h /= 3){
		for (i = h; i < array_size; i++){
			j = i;
			while((j > h - 1) && (alpha_sort[0][j-h] > alpha_sort[0][j])) {
				swap(&alpha_sort[0][j-h], &alpha_sort[0][j]);
				swap(&alpha_sort[1][j-h], &alpha_sort[1][j]);
				swap(&alpha_sort[2][j-h], &alpha_sort[2][j]);
				j -= h;
			}
		}
	}
}
void makePER(const int alpha[][alpha_num], const int pole[], int amp){
    int array_cnt = 0;
	int array_cnt_sus = 0;
	PER = 2500000 / basfrq;
	while(array_cnt < alpha_num){
		alpha_sort[0][array_cnt_sus] = (uint32_t)((alpha[amp][array_cnt]) * PER/36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus] = (pole[amp]+array_cnt_sus)%2;
		alpha_sort[2][array_cnt_sus] = 1;
		alpha_sort[0][array_cnt_sus+alpha_num*4] = (uint32_t)((12000 + alpha[amp][array_cnt])*PER/36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus+alpha_num*4] = (pole[amp]+array_cnt_sus)%2;
		alpha_sort[2][array_cnt_sus+alpha_num*4] = 2;
		alpha_sort[0][array_cnt_sus+alpha_num*8] = (uint32_t)((24000 + alpha[amp][array_cnt])*PER/36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus+alpha_num*8] = (pole[amp]+array_cnt_sus)%2;
		alpha_sort[2][array_cnt_sus+alpha_num*8] = 4;
		array_cnt++;
		array_cnt_sus++;
	}
	while(array_cnt > 0){
		array_cnt--;
		alpha_sort[0][array_cnt_sus] = (uint32_t)((18000 - alpha[amp][array_cnt]) * PER / 36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus] = (pole[amp]+array_cnt_sus)%2;
		alpha_sort[2][array_cnt_sus] = 1;
		alpha_sort[0][array_cnt_sus+alpha_num*4] = (uint32_t)((30000 - alpha[amp][array_cnt])*PER/36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus+alpha_num*4] = (pole[amp]+array_cnt_sus)%2;
		alpha_sort[2][array_cnt_sus+alpha_num*4] = 2;
		alpha_sort[0][array_cnt_sus+alpha_num*8] = (uint32_t)((42000 - alpha[amp][array_cnt])*PER/36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus+alpha_num*8] = (pole[amp]+array_cnt_sus)%2;
		alpha_sort[2][array_cnt_sus+alpha_num*8] = 4;
		array_cnt_sus++;
	}
	while(array_cnt < alpha_num){
		alpha_sort[0][array_cnt_sus] = (uint32_t)((18000 + alpha[amp][array_cnt]) * PER / 36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus] = (pole[amp]+array_cnt_sus+1)%2;
		alpha_sort[2][array_cnt_sus] = 1;
		alpha_sort[0][array_cnt_sus+alpha_num*4] = (uint32_t)((30000 + alpha[amp][array_cnt])*PER/36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus+alpha_num*4] = (pole[amp]+array_cnt_sus+1)%2;
		alpha_sort[2][array_cnt_sus+alpha_num*4] = 2;
		alpha_sort[0][array_cnt_sus+alpha_num*8] = (uint32_t)((42000 + alpha[amp][array_cnt])*PER/36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus+alpha_num*8] = (pole[amp]+array_cnt_sus+1)%2;
		alpha_sort[2][array_cnt_sus+alpha_num*8] = 4;
		array_cnt++;
		array_cnt_sus++;
	}
	while(array_cnt > 0){
		array_cnt--;
		alpha_sort[0][array_cnt_sus] = (uint32_t)((36000 - alpha[amp][array_cnt]) * PER / 36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus] = (pole[amp]+array_cnt_sus+1)%2;
		alpha_sort[2][array_cnt_sus] = 1;
		alpha_sort[0][array_cnt_sus+alpha_num*4] = (uint32_t)((48000 - alpha[amp][array_cnt])*PER/36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus+alpha_num*4] = (pole[amp]+array_cnt_sus+1)%2;
		alpha_sort[2][array_cnt_sus+alpha_num*4] = 2;
		alpha_sort[0][array_cnt_sus+alpha_num*8] = (uint32_t)((60000 - alpha[amp][array_cnt])*PER/36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus+alpha_num*8] = (pole[amp]+array_cnt_sus+1)%2;
		alpha_sort[2][array_cnt_sus+alpha_num*8] = 4;
		array_cnt_sus++;
	}
	alpha_sort[2][array_cnt_sus+alpha_num*8] = 1;
	alpha_sort[2][array_cnt_sus+alpha_num*8+3] = 1;
	alpha_sort[2][array_cnt_sus+alpha_num*8+1] = 2;
	alpha_sort[2][array_cnt_sus+alpha_num*8+4] = 2;
	alpha_sort[2][array_cnt_sus+alpha_num*8+2] = 4;
	alpha_sort[2][array_cnt_sus+alpha_num*8+5] = 4;
	for(int i=0; i<3; i++){
		alpha_sort[0][array_cnt_sus+alpha_num*8] = (uint32_t)(i * PER / 3) % (uint32_t)PER;
		alpha_sort[0][array_cnt_sus+alpha_num*8+3] = (uint32_t)((3 + i*2) * PER / 6) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus+alpha_num*8] = (pole[amp]+1)%2;
		alpha_sort[1][array_cnt_sus+alpha_num*8+3] = (pole[amp]+2)%2;
		array_cnt_sus++;
	}
	array_cnt = 0;
	shell_sort();
	for(int i = 0; i < alpha_num*12+6 - 1; i++){
		alpha_sort[0][i] = alpha_sort[0][i+1] - alpha_sort[0][i];
		if(alpha_sort[0][i] < 100){alpha_sort[0][i] = 10;}
	}
	alpha_sort[0][alpha_num*12+6 - 1] = (uint32_t)PER - alpha_sort[0][alpha_num*12+6 - 1];
	if(alpha_sort[0][alpha_num*12+6 - 1] < 10){alpha_sort[0][alpha_num*12+6 - 1] = 10;}
	for(int i = 0; i < alpha_num*12+6; i++){
		alpha_est[0][i] = alpha_sort[0][i]*4;
		alpha_est[1][i] = alpha_sort[1][i];
		alpha_est[2][i] = alpha_sort[2][i];
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
