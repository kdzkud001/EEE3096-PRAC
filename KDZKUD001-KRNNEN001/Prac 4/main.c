/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables
#define NS 200       // Number of samples in LUT
#define TIM2CLK 8000000  // STM Clock frequency
#define F_SIGNAL 69 // Frequency of output analog signal
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs

//200 values for the LUT

uint32_t Sin_LUT[NS] = {512, 528, 544, 560, 576, 592, 608, 624, 639, 655, 670, 686, 701, 716, 730, 745, 759, 773, 787, 800, 813, 826, 839, 851, 863, 875, 886, 897, 907, 917, 927, 936, 945, 953, 961, 969, 976, 982, 988, 994, 999, 1004, 1008, 1011, 1015, 1017, 1019, 1021, 1022, 1023, 1023, 1023, 1022, 1020, 1018, 1016, 1013, 1010, 1006, 1001, 996, 991, 985, 979, 972, 965, 957, 949, 940, 931, 922, 912, 902, 891, 880, 869, 857, 845, 833, 820, 807, 794, 780, 766, 752, 738, 723, 708, 693, 678, 663, 647, 631, 616, 600, 584, 568, 552, 536, 520, 503, 487, 471, 455, 439, 423, 407, 392, 376, 360, 345, 330, 315, 300, 285, 271, 257, 243, 229, 216, 203, 190, 178, 166, 154, 143, 132, 121, 111, 101, 92, 83, 74, 66, 58, 51, 44, 38, 32, 27, 22, 17, 13, 10, 7, 5, 3, 1, 0, 0, 0, 1, 2, 4, 6, 8, 12, 15, 19, 24, 29, 35, 41, 47, 54, 62, 70, 78, 87, 96, 106, 116, 126, 137, 148, 160, 172, 184, 197, 210, 223, 236, 250, 264, 278, 293, 307, 322, 337, 353, 368, 384, 399, 415, 431, 447, 463, 479, 495, 511};

uint32_t saw_LUT[NS] = {512, 517, 522, 527, 532, 537, 542, 547, 553, 558, 563, 568, 573, 578, 583, 589, 594, 599, 604, 609, 614, 619, 625, 630, 635, 640, 645, 650, 655, 661, 666, 671, 676, 681, 686, 691, 697, 702, 707, 712, 717, 722, 727, 733, 738, 743, 748, 753, 758, 763, 769, 774, 779, 784, 789, 794, 799, 805, 810, 815, 820, 825, 830, 835, 841, 846, 851, 856, 861, 866, 871, 876, 882, 887, 892, 897, 902, 907, 912, 918, 923, 928, 933, 938, 943, 948, 954, 959, 964, 969, 974, 979, 984, 990, 995, 1000, 1005, 1010, 1015, 1020, 3, 8, 13, 18, 23, 28, 33, 39, 44, 49, 54, 59, 64, 69, 75, 80, 85, 90, 95, 100, 105, 111, 116, 121, 126, 131, 136, 141, 147, 152, 157, 162, 167, 172, 177, 182, 188, 193, 198, 203, 208, 213, 218, 224, 229, 234, 239, 244, 249, 254, 260, 265, 270, 275, 280, 285, 290, 296, 301, 306, 311, 316, 321, 326, 332, 337, 342, 347, 352, 357, 362, 368, 373, 378, 383, 388, 393, 398, 404, 409, 414, 419, 424, 429, 434, 440, 445, 450, 455, 460, 465, 470, 476, 481, 486, 491, 496, 501, 506, 512};

uint32_t triangle_LUT[NS] = {512, 522, 532, 542, 553, 563, 573, 583, 594, 604, 614, 625, 635, 645, 655, 666, 676, 686, 697, 707, 717, 727, 738, 748, 758, 769, 779, 789, 799, 810, 820, 830, 841, 851, 861, 871, 882, 892, 902, 912, 923, 933, 943, 954, 964, 974, 984, 995, 1005, 1015, 1020, 1010, 1000, 990, 979, 969, 959, 948, 938, 928, 918, 907, 897, 887, 876, 866, 856, 846, 835, 825, 815, 805, 794, 784, 774, 763, 753, 743, 733, 722, 712, 702, 691, 681, 671, 661, 650, 640, 630, 619, 609, 599, 589, 578, 568, 558, 547, 537, 527, 517, 506, 496, 486, 476, 465, 455, 445, 434, 424, 414, 404, 393, 383, 373, 362, 352, 342, 332, 321, 311, 301, 290, 280, 270, 260, 249, 239, 229, 218, 208, 198, 188, 177, 167, 157, 147, 136, 126, 116, 105, 95, 85, 75, 64, 54, 44, 33, 23, 13, 3, 8, 18, 28, 39, 49, 59, 69, 80, 90, 100, 111, 121, 131, 141, 152, 162, 172, 182, 193, 203, 213, 224, 234, 244, 254, 265, 275, 285, 296, 306, 316, 326, 337, 347, 357, 368, 378, 388, 398, 409, 419, 429, 440, 450, 460, 470, 481, 491, 501, 512};
//Declaring the variables to be used when debouncing
uint32_t lastDebounce = 0;
uint32_t debounceDelay = 250;//Change till works well 
int choice = 0;//For switching the mode it is in 
// TODO: Equation to calculate TIM2_Ticks
uint32_t TIM2_Ticks = TIM2CLK/(F_SIGNAL*NS); // How often to write new LUT value
uint32_t DestAddress = (uint32_t) &(TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
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
  init_LCD();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // TODO: Start TIM3 in PWM mode on channel 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1.
  HAL_TIM_OC_Start(&htim2,TIM_CHANNEL_1);

  // TODO: Start DMA in IT mode on TIM2->CH1; Source is LUT and Dest is TIM3->CCR3; start with Sine LUT
  HAL_DMA_Start_IT(&hdma_tim2_ch1, Sin_LUT, DestAddress, NS);

  // TODO: Write current waveform to LCD ("Sine")
  //lcd_command(CLEAR);
  lcd_putstring("Sine ");
  delay(3000);

  // TODO: Enable DMA (start transfer from LUT to CCR)
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1) ;
  
  /* USER CODE END 2 */

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Period = TIM2_Ticks - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1023;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_GPIO_SetPinPull(Button0_GPIO_Port, Button0_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(Button0_GPIO_Port, Button0_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void)
{
	// TODO: Debounce using HAL_GetTick()
	//The debouncing delay
	if (HAL_GetTick()- lastDebounce > debounceDelay)
	{

	// TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with new LUT and re-enable transfer
	// HINT: Consider using C's "switch" function to handle LUT changes
		
	//Updating the variable for use with the switch (waveform)
	choice = choice + 1;
	if (choice > 2){choice = 0;}
	
	//Stopping the DMA transfer 
	__HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
	HAL_DMA_Abort_IT(&hdma_tim2_ch1);
	switch(choice)
	{
	case 0: //For the sin
		//Start DMA in IT mode on TIM2->CH1; Source is LUT and Dest is TIM3->CCR3; start with Sine LUT
		  HAL_DMA_Start_IT(&hdma_tim2_ch1, Sin_LUT, DestAddress, NS);

		  //Write current waveform to LCD ("Sine")
		  lcd_command(CLEAR);
		  lcd_putstring("Sine ");
		  // Enable DMA (start transfer from LUT to CCR)
		  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1) ;
		  break;
	case 1: //For the saw
		// Start DMA in IT mode on TIM2->CH1; Source is LUT and Dest is TIM3->CCR3; start with saw LUT
		  HAL_DMA_Start_IT(&hdma_tim2_ch1, saw_LUT, DestAddress, NS);

		  // Write current waveform to LCD ("Sine")
		  lcd_command(CLEAR);
		  lcd_putstring("SawTooth ");
		  // Enable DMA (start transfer from LUT to CCR)
		  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1) ;
		  
		  break;
	
	case 2: //For the triangle
		//Start DMA in IT mode on TIM2->CH1; Source is LUT and Dest is TIM3->CCR3; start with triangle LUT
		  HAL_DMA_Start_IT(&hdma_tim2_ch1, triangle_LUT, DestAddress, NS);
		  //Write current waveform to LCD ("Triangle")
		  lcd_command(CLEAR);
		  lcd_putstring("Triangle ");
		  //Enable DMA (start transfer from LUT to CCR)
		  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1) ;
		  break;
	}
	 //Update HAL_GetTick() value
	lastDebounce = HAL_GetTick();
	
	}

	HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
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
