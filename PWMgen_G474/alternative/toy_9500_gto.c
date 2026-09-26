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
#include <stdlib.h>
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
	SAMI,
	HO
};
enum async_type{
	PWM,
	RPPWM,
	SSPWM,
	THI,
	THI_RP,
	THI_SS
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PER 25000.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
uint32_t alpha_sort[3][90];
uint32_t alpha_est[3][90];
uint32_t alpha_cur[3][90];
int randnum;
int rand_est;
int alpha_num = 7;
int alpha_num_cur = 7;
int alpha_num_temp = 7;
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
int transit = 0;//Async->0 PP-PWM->1 Sync->2
int tran_tim5 = 0;
float frq;
float basfrq = 1;
float basfrq_Jerk = 0.0;
int JerkPole;
int tca_cnt = 0;
int tcb_cnt = 0;
int tcb2_cnt = 0;
int pulse_mode;
int async_mode = 0;
int cur_pnum_PWM = 9;
int pnum_PWM;
int pnum_temp;
int THI_alpha = 2;
float THI_beta = 0.4;
int cnt_THI;
float sgn_THI = 1;
float PER_PWM;
float spectrum[12] = {-25,-19,-13,-11,-5,-1,1,5,11,13,19,25};
float spect5[5] = {-25,-12.5,0,12.5,25};
int ss_num = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

float min(float a, float b);
float max(float a, float b);
void modulation_acc(float a);
void modulation_dec(float a);
void swap (uint32_t *x, uint32_t *y);
void shell_sort (void);
void makePER(const int alpha[][alpha_num], const int pole[], int amp);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim == &htim1){
    	if(transit == 0){
    		if(async_mode == 5){
    			randnum = 255;
    			PER_PWM = 2500000 / (frq+spectrum[rand_est%2]);
    			cnt_THI ++;
    			cnt_THI %= THI_alpha;
    			if(cnt_THI == 0){sgn_THI *= -1;}
    			dutyu = Asdutyu * (AMP + sgn_THI * THI_beta*basfrq_Jerk/basfrq) / 32767 + OFFSET;
    			dutyv = Asdutyv * (AMP + sgn_THI * THI_beta*basfrq_Jerk/basfrq) / 32767 + OFFSET;
    			dutyw = Asdutyw * (AMP + sgn_THI * THI_beta*basfrq_Jerk/basfrq) / 32767 + OFFSET;
    		}
    		else if(async_mode == 4){
    			PER_PWM = 2500000 / (frq + rand_est);
    			cnt_THI ++;
    			cnt_THI %= THI_alpha;
    			if(cnt_THI == 0){sgn_THI *= -1;}
    			dutyu = Asdutyu * (AMP + sgn_THI * THI_beta*basfrq_Jerk/basfrq) / 32767 + OFFSET;
    			dutyv = Asdutyv * (AMP + sgn_THI * THI_beta*basfrq_Jerk/basfrq) / 32767 + OFFSET;
    			dutyw = Asdutyw * (AMP + sgn_THI * THI_beta*basfrq_Jerk/basfrq) / 32767 + OFFSET;
    		}
    		else if(async_mode == 3){
    			PER_PWM = 2500000 / frq;
    			cnt_THI ++;
    			cnt_THI %= THI_alpha;
    			if(cnt_THI == 0){sgn_THI *= -1;}
    			dutyu = Asdutyu * (AMP + sgn_THI * THI_beta*basfrq_Jerk/basfrq) / 32767 + OFFSET;
    			dutyv = Asdutyv * (AMP + sgn_THI * THI_beta*basfrq_Jerk/basfrq) / 32767 + OFFSET;
    			dutyw = Asdutyw * (AMP + sgn_THI * THI_beta*basfrq_Jerk/basfrq) / 32767 + OFFSET;
    		}
    		else if(async_mode == 2){
    			if(basfrq <= 8){
    				ss_num %= 12;
    				PER_PWM = 2500000 / (frq + 0.8*spectrum[ss_num]);
    				ss_num++;
    			}
    			else{
    				ss_num %= 5;
    				PER_PWM = 2500000 / (frq+min(max(-0.026316*basfrq + 1.184211, 0.5), 1.0)*spect5[ss_num]);
    				ss_num++;
    			}
    			dutyu = Asdutyu * AMP / 32767 + OFFSET;
    			dutyv = Asdutyv * AMP / 32767 + OFFSET;
    			dutyw = Asdutyw * AMP / 32767 + OFFSET;
    		}
    		else if(async_mode == 1){
    			PER_PWM = 2500000 / (frq + rand_est);
    			dutyu = Asdutyu * AMP / 32767 + OFFSET;
    			dutyv = Asdutyv * AMP / 32767 + OFFSET;
    			dutyw = Asdutyw * AMP / 32767 + OFFSET;
    		}
    		else{
    			PER_PWM = 2500000 / frq;
    			dutyu = Asdutyu * AMP / 32767 + OFFSET;
    			dutyv = Asdutyv * AMP / 32767 + OFFSET;
    			dutyw = Asdutyw * AMP / 32767 + OFFSET;
    		}
    	}
    	else if(transit == 2){
    		PER_PWM = 2500000 / (basfrq*cur_pnum_PWM);
    		pnum_temp = (cur_pnum_PWM*2)/3;
    		if(TIM1->CNT > (uint16_t)(PER_PWM/2)){
    			dutyu = SYNC[(cur_pnum_PWM-3)/6 - 1][tca_cnt%(cur_pnum_PWM*2)] * AMP / 32767 + OFFSET;
    		    dutyv = SYNC[(cur_pnum_PWM-3)/6 - 1][(tca_cnt+pnum_temp*2)%(cur_pnum_PWM*2)] * AMP / 32767 + OFFSET;
    		    dutyw = SYNC[(cur_pnum_PWM-3)/6 - 1][(tca_cnt+pnum_temp)%(cur_pnum_PWM*2)] * AMP / 32767 + OFFSET;
    		    tca_cnt++;
    		    if(tca_cnt == 2){
    		    	if(pulse_mode == 0 && transit == 2){
    		    		transit = 0;
    		        	tcb2_cnt = 0;
    		    	}
    		        else if(pulse_mode >= 2 && transit == 2){
    		        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    		        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    		        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    		        	GPIO_InitTypeDef GPIO_InitStruct;
    		        	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    		        	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    		        	GPIO_InitStruct.Pull = GPIO_NOPULL;
    		        	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    		        	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    		        	transit = 1;
    		        	tcb_cnt = 0;
    		        	tran_tim5 = 0;
    		        }
    		    }
    		    if(tca_cnt >= (cur_pnum_PWM*2)){
    		    	tca_cnt = 0;
    		    	cur_pnum_PWM = pnum_PWM;
    		    }
    		}
    		else{
    			dutyu = SYNC[(cur_pnum_PWM-3)/6 - 1][tca_cnt%(cur_pnum_PWM*2)] * AMP / 32767 + OFFSET;
    		    dutyv = SYNC[(cur_pnum_PWM-3)/6 - 1][(tca_cnt+pnum_temp*2)%(cur_pnum_PWM*2)] * AMP / 32767 + OFFSET;
    		    dutyw = SYNC[(cur_pnum_PWM-3)/6 - 1][(tca_cnt+pnum_temp)%(cur_pnum_PWM*2)] * AMP / 32767 + OFFSET;
    		    tca_cnt++;
    		}
    	}
    	TIM1->ARR = (uint16_t)PER_PWM;
    	TIM1->CCR1 = (uint16_t)(PER_PWM*dutyu);
    	TIM1->CCR2 = (uint16_t)(PER_PWM*dutyv);
    	TIM1->CCR3 = (uint16_t)(PER_PWM*dutyw);
    }
    if(htim == &htim5){
    	TIM5->CR1 &= ~TIM_CR1_CEN;
    	TIM5->CNT = 0;
    	if(tcb_cnt == 0){
    		if(pulse_mode == 0 && transit == 1){
    			GPIO_InitTypeDef GPIO_InitStruct;
    			GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    			GPIO_InitStruct.Pull = GPIO_NOPULL;
    			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    			GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    			HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    			tcb2_cnt = 4;
    			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    			transit = 0;

    		}
    		else if(pulse_mode == 1 && transit == 1){
    			GPIO_InitTypeDef GPIO_InitStruct;
    			GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    			GPIO_InitStruct.Pull = GPIO_NOPULL;
    			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    			GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    			HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    			tca_cnt = 2;
    			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    			transit = 2;
    		}
    		if(alpha_num_temp == alpha_num){
    			alpha_num_cur = alpha_num;
    			for(int i = 0; i < alpha_num_cur*12+6; i++){
    				alpha_cur[0][i] = alpha_est[0][i];
    			    alpha_cur[1][i] = alpha_est[1][i];
    			    alpha_cur[2][i] = alpha_est[2][i];
    			}
    		}
    		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
    		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    	}
    	GPIOC->BSRR = (1U << (alpha_cur[2][tcb_cnt] + 16*(1-alpha_cur[1][tcb_cnt])));
    	if(tcb_cnt < alpha_num_cur*12+5){
    		TIM5->ARR = (uint32_t)(alpha_cur[0][tcb_cnt+1]*800 / basfrq);
    	}
    	else{
			alpha_num_temp = alpha_num;
    		for(int i = 0; i < alpha_num*12+6; i++){
    			alpha_cur[0][i] = alpha_est[0][i];
    			alpha_cur[1][i] = alpha_est[1][i];
    		    alpha_cur[2][i] = alpha_est[2][i];
    		}
    		TIM5->ARR = (uint32_t)(alpha_cur[0][0]*800 / basfrq);
    	}
    	tcb_cnt += 1;
    	if(tran_tim5 == 0){
    		alpha_num_cur = alpha_num;
    		for(int i = 0; i < alpha_num_cur*12+6; i++){
    			alpha_cur[0][i] = alpha_est[0][i];
    		    alpha_cur[1][i] = alpha_est[1][i];
    		    alpha_cur[2][i] = alpha_est[2][i];
    		}
    		TIM5->ARR = (uint32_t)(alpha_cur[0][0]*800 / basfrq);
    		tcb_cnt = 0;
    		tran_tim5 = 1;
    	}
    	if(tcb_cnt > alpha_num_cur*12+6 - 1){tcb_cnt = 0;}
    	TIM5->CR1 |= TIM_CR1_CEN;
    }
    if(htim == &htim2){
    	if(tcb2_cnt == 0){
    		if(pulse_mode >= 2 && transit == 0){
    			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    			GPIO_InitTypeDef GPIO_InitStruct;
    			GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    			GPIO_InitStruct.Pull = GPIO_NOPULL;
    			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    			HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    			transit = 1;
    			tcb_cnt = 0;
    			tran_tim5 = 0;
    		}
    		else if(pulse_mode == 1 && transit == 0){
    			transit = 2;
    			tca_cnt = 0;
    		}
    	}
    	TIM2->ARR = (uint32_t)(303030.30303 * 4 / basfrq);
    	if(pulse_mode == 0 || transit == 0){
    		Asdutyu = SYNC[4][tcb2_cnt%66];
    		Asdutyv = SYNC[4][(tcb2_cnt+44)%66];
    		Asdutyw = SYNC[4][(tcb2_cnt+22)%66];
    	}
    	tcb2_cnt++;
    	tcb2_cnt %= 66;
    }
    if(htim == &htim7){
    	basfrq += (0.0005 * dir);
    	if(JerkPole != 0){basfrq_Jerk += (0.0002*JerkPole*basfrq);}
    	rand_est = rand() % randnum;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  makePER(_7alpha, _7alpha_pole, ampINT);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim5);
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
	  //if(TIM5->CNT > TIM5->ARR){TIM5->CNT = 0;}
	  if(basfrq < 0.1){basfrq = 0.1;}
	  if ((GPIOC->IDR & GPIO_PIN_4) == 0){
		  //acceleration setting
		  motorState = 1;
		  if(basfrq_Jerk < basfrq){
			  JerkPole = 1;
			  modulation_acc(basfrq_Jerk);
		  }
		  else{
			  dir = 1;
			  JerkPole = 0;
			  modulation_acc(basfrq);
			  basfrq_Jerk = basfrq + 0.1;
		  }
	  }
	  else if ((GPIOC->IDR & GPIO_PIN_5) == 0){
		  motorState = -1;
		  if(basfrq_Jerk < basfrq){
			  JerkPole = 1;
			  modulation_dec(basfrq_Jerk);
		  }
		  else{
			  dir = -1;
			  JerkPole = 0;
			  basfrq_Jerk = basfrq;
			  modulation_dec(basfrq);
		  }
	  }
	  else{
		  dir = 0;
		  if(basfrq_Jerk > 0 && motorState == 1){
			  JerkPole = -1;
			  modulation_acc(basfrq_Jerk);
		  }
		  else if(basfrq_Jerk > 0 && motorState == -1){
			  JerkPole = -1;
			  modulation_dec(basfrq_Jerk);
		  }
		  else{
			  JerkPole = 0;
			  basfrq_Jerk = 0;
			  motorState = 0;
		  }
	  }

	  if(motorState == 1){
	  		if(basfrq >= 62 && dir == 1){pulse_mode = CHM;alpha_num = 0;}
	  		else if(basfrq >= 56 && dir == 1){pulse_mode = W3p;}
	  		else if(basfrq >= 56){pulse_mode = CHM;alpha_num = 1;}
	  		else if(basfrq >= 51){pulse_mode = S3p;}
	  		else if(basfrq >= 43){pulse_mode = SAMI;alpha_num = 2;}
	        else if(basfrq >= 27){pulse_mode = SyncMp;pnum_PWM = 9;}
	        else if(basfrq >= 10){pulse_mode = Async;frq = 4.375*basfrq + 246.875;async_mode = SSPWM;}
	        else if(basfrq >= 7){pulse_mode = Async;frq = 4.375*basfrq + 246.875;async_mode = PWM;}
	        else if(basfrq >= 3){pulse_mode = Async;frq = 4.375*basfrq + 246.875;async_mode = SSPWM;}
	  		else if(basfrq >= 0){pulse_mode = Async;frq = 260;async_mode = SSPWM;}
	  	}

	  	else if(motorState == -1){
	  		if(basfrq >= 77 && dir == -1){pulse_mode = CHM;alpha_num = 0;}
	  		else if(basfrq >= 68 && dir == -1){pulse_mode = W3p;}
	  		else if(basfrq >= 68){pulse_mode = CHM;alpha_num = 1;}
	  		else if(basfrq >= 60){pulse_mode = S3p;}
	  		else if(basfrq >= 43){pulse_mode = SAMI;alpha_num = 2;}
	  		else if(basfrq >= 25){pulse_mode = SyncMp;pnum_PWM = 9;}
	  		else if(basfrq >= 10){pulse_mode = Async;frq = 3.875 * basfrq + 253.125;async_mode = SSPWM;}
	  		else if(basfrq >= 8){pulse_mode = Async;frq = 3.875 * basfrq + 253.125;async_mode = PWM;}
	  		else if(basfrq >= 0){pulse_mode = Async;frq = 3.875 * basfrq + 253.125;async_mode = SSPWM;}
	  	}

	  	if(alpha_num == 7 && pulse_mode == 4){ampINT = (uint16_t)ratio; makePER(_7alpha, _7alpha_pole, ampINT);}
	  	else if(alpha_num == 6 && pulse_mode == 4){ampINT = (uint16_t)ratio; makePER(_6alpha, _6alpha_pole, ampINT);}
	  	else if(alpha_num == 5 && pulse_mode == 4){ampINT = (uint16_t)ratio; makePER(_5alpha, _5alpha_pole, ampINT);}
	  	else if(alpha_num == 4 && pulse_mode == 4){ampINT = (uint16_t)ratio; makePER(_4alpha, _4alpha_pole, ampINT);}
	  	else if(alpha_num == 3 && pulse_mode == 4){ampINT = (uint16_t)ratio; makePER(_3alpha, _3alpha_pole, ampINT);}
	  	else if(alpha_num == 2 && pulse_mode == 4){ampINT = (uint16_t)ratio; makePER(_2alpha, _2alpha_pole, ampINT);}
	  	else if(alpha_num == 1 && pulse_mode == 4){ampINT = (uint16_t)ratio; makePER(_1alpha, _1alpha_pole, ampINT);}
	  	else if(alpha_num == 0 && pulse_mode == 4){ampINT = (uint16_t)ratio; makePER(_1alpha, _1alpha_pole, 100);}
	  	else if(pulse_mode == 2){alpha_num = 1;ampINT = (uint16_t)ratio; makePER(S3P, S3P_pole, ampINT);}
	  	else if(pulse_mode == 3){alpha_num = 1;ampINT = (uint16_t)ratio; makePER(W3P, W3P_pole, ampINT);}
	  	else if(alpha_num == 5 && pulse_mode == 5){ampINT = (uint16_t)ratio; makePER(SHE11, W3P_pole, ampINT);}
	  	else if(alpha_num == 3 && pulse_mode == 5){ampINT = (uint16_t)ratio; makePER(SHE7, W3P_pole, ampINT);}
	  	else if(alpha_num == 2 && pulse_mode == 5){ampINT = (uint16_t)ratio; makePER(SHE5, S3P_pole, ampINT);}
	  	else if(alpha_num == 5 && pulse_mode == 6){ampINT = (uint16_t)ratio; makePER(SAMI11, S3P_pole, ampINT);}
	  	else if(alpha_num == 3 && pulse_mode == 6){ampINT = (uint16_t)ratio; makePER(SAMI7, S3P_pole, ampINT);}
	  	else if(alpha_num == 2 && pulse_mode == 6){ampINT = (uint16_t)ratio; makePER(SAMI5, S3P_pole, ampINT);}
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  htim1.Init.Period = 2500;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  htim2.Init.Period = 2500;
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

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 2500;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  htim7.Init.Period = 10000;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LPUART1_TX_Pin LPUART1_RX_Pin */
  GPIO_InitStruct.Pin = LPUART1_TX_Pin|LPUART1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
void modulation_acc (float a){
	ratio = min(max(a * 8.075862, basfrq * 3.3933335), 502);
	AMP = min(max(a * 0.022222, basfrq * 0.009165), 1);
	OFFSET = min(max(0.5 - a * 0.011111, 0), 0.5 - basfrq * 0.0045825);
}
void modulation_dec (float a){
	ratio = min(max(a * 6.786667, basfrq * 3.3933335), 502);
	AMP = min(max(a * 0.01833, basfrq * 0.009165), 1);
	OFFSET = min(max(0.5 - a * 0.009165, 0), 0.5 - basfrq * 0.0045825);
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
	while(array_cnt < alpha_num){
		alpha_sort[0][array_cnt_sus] = (uint32_t)((alpha[amp][array_cnt]) * PER/36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus] = (pole[amp]+array_cnt_sus)%2;
		alpha_sort[2][array_cnt_sus] = 0;
		alpha_sort[0][array_cnt_sus+alpha_num*4] = (uint32_t)((12000 + alpha[amp][array_cnt])*PER/36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus+alpha_num*4] = (pole[amp]+array_cnt_sus)%2;
		alpha_sort[2][array_cnt_sus+alpha_num*4] = 1;
		alpha_sort[0][array_cnt_sus+alpha_num*8] = (uint32_t)((24000 + alpha[amp][array_cnt])*PER/36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus+alpha_num*8] = (pole[amp]+array_cnt_sus)%2;
		alpha_sort[2][array_cnt_sus+alpha_num*8] = 2;
		array_cnt++;
		array_cnt_sus++;
	}
	while(array_cnt > 0){
		array_cnt--;
		alpha_sort[0][array_cnt_sus] = (uint32_t)((18000 - alpha[amp][array_cnt]) * PER / 36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus] = (pole[amp]+array_cnt_sus)%2;
		alpha_sort[2][array_cnt_sus] = 0;
		alpha_sort[0][array_cnt_sus+alpha_num*4] = (uint32_t)((30000 - alpha[amp][array_cnt])*PER/36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus+alpha_num*4] = (pole[amp]+array_cnt_sus)%2;
		alpha_sort[2][array_cnt_sus+alpha_num*4] = 1;
		alpha_sort[0][array_cnt_sus+alpha_num*8] = (uint32_t)((42000 - alpha[amp][array_cnt])*PER/36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus+alpha_num*8] = (pole[amp]+array_cnt_sus)%2;
		alpha_sort[2][array_cnt_sus+alpha_num*8] = 2;
		array_cnt_sus++;
	}
	while(array_cnt < alpha_num){
		alpha_sort[0][array_cnt_sus] = (uint32_t)((18000 + alpha[amp][array_cnt]) * PER / 36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus] = (pole[amp]+array_cnt_sus+1)%2;
		alpha_sort[2][array_cnt_sus] = 0;
		alpha_sort[0][array_cnt_sus+alpha_num*4] = (uint32_t)((30000 + alpha[amp][array_cnt])*PER/36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus+alpha_num*4] = (pole[amp]+array_cnt_sus+1)%2;
		alpha_sort[2][array_cnt_sus+alpha_num*4] = 1;
		alpha_sort[0][array_cnt_sus+alpha_num*8] = (uint32_t)((42000 + alpha[amp][array_cnt])*PER/36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus+alpha_num*8] = (pole[amp]+array_cnt_sus+1)%2;
		alpha_sort[2][array_cnt_sus+alpha_num*8] = 2;
		array_cnt++;
		array_cnt_sus++;
	}
	while(array_cnt > 0){
		array_cnt--;
		alpha_sort[0][array_cnt_sus] = (uint32_t)((36000 - alpha[amp][array_cnt]) * PER / 36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus] = (pole[amp]+array_cnt_sus+1)%2;
		alpha_sort[2][array_cnt_sus] = 0;
		alpha_sort[0][array_cnt_sus+alpha_num*4] = (uint32_t)((48000 - alpha[amp][array_cnt])*PER/36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus+alpha_num*4] = (pole[amp]+array_cnt_sus+1)%2;
		alpha_sort[2][array_cnt_sus+alpha_num*4] = 1;
		alpha_sort[0][array_cnt_sus+alpha_num*8] = (uint32_t)((60000 - alpha[amp][array_cnt])*PER/36000) % (uint32_t)PER;
		alpha_sort[1][array_cnt_sus+alpha_num*8] = (pole[amp]+array_cnt_sus+1)%2;
		alpha_sort[2][array_cnt_sus+alpha_num*8] = 2;
		array_cnt_sus++;
	}
	alpha_sort[2][array_cnt_sus+alpha_num*8] = 0;
	alpha_sort[2][array_cnt_sus+alpha_num*8+3] = 0;
	alpha_sort[2][array_cnt_sus+alpha_num*8+1] = 1;
	alpha_sort[2][array_cnt_sus+alpha_num*8+4] = 1;
	alpha_sort[2][array_cnt_sus+alpha_num*8+2] = 2;
	alpha_sort[2][array_cnt_sus+alpha_num*8+5] = 2;
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
		if(alpha_sort[0][i] < 30){
			alpha_sort[0][i+1] -= (30 - alpha_sort[0][i]);
			alpha_sort[0][i] = 30;
		}
	}
	alpha_sort[0][alpha_num*12+6 - 1] = (uint32_t)PER - alpha_sort[0][alpha_num*12+6 - 1];
	if(alpha_sort[0][alpha_num*12+6 - 1] < 30){
		alpha_sort[0][0] -= (30 - alpha_sort[0][alpha_num*12+6 - 1]);
		alpha_sort[0][alpha_num*12+6 - 1] = 30;
	}
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
