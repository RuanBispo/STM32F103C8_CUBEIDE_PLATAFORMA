/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  *					EXEMPLO DE ENCODER DE QUADRATURA (stm32f103c8)
  ******************************************************************************
  * Aluno:        Ruan Robert Bispo dos Santos                            2019.2
  * Orientadores: Lucas Molina e Elyson Adan Nunes Carvalho
  *
  * CODIGO DA PLACA DE INTERFACE DO MATLAB COM O STM32F103C8
  *
  * @Resumo: Código teste para o encoder de quadratura com a resolução de 1440
  * 		 furos por revolução. Nesse código é implementada a detecção de
  * 		 posição e velocidade de dois encoders (TIMER1 e TIMER2),utilizando
  * 		 como base de tempo um contator definido pelo TIMER3.
  * 		 É possível visualizar as variáveis no modo debug, ou pode ser
  * 		 utilizado o exemplo de USB para enviar os dados e visualizar no PC
  *
  * 		 OBS1: É feita uma configuração manual para interrupção por borda de
  * 		 subida e descida, uma vez que a plataforma CUBEIDE é limitada a
  * 		 apenas uma das opções. Isso é feito no final da main.c, utilizando
  * 		 a parte editável das funções:
  *
  * 		 	static void MX_TIM2_Init(void)
  * 		 	static void MX_TIM2_Init(void)
  *
  * 		 OBS2: Os Timers de leitura do encoder precisam ser configurados com
  * 		 o prescaler igual a zero para a contagem ser correta. Do contrário,
  * 		 haverá uma divisão em torno da medida. (IMPORTANTE LEMBRAR)
  *
  *
  * @Configurações:
  *   		 Clock   - 72 MHz
  *   		 TIMER1	 - usado para ler o encoder 1
  *   		 TIMER2	 - usado para ler o encoder 2
  *   		 TIMER3	 - usado como base fixa de tempo para calculo de velocidade
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

uint16_t POSICAO_ENCODER1; //variavel de posição do encoder 1 (0 a 1440)
uint16_t POSICAO_ENCODER2; //variavel de posição do encoder 2 (0 a 1440)

float POSICAO_ENCODER1_GRAUS; //variavel de posição do encoder 1 (0 a 360 graus)
float POSICAO_ENCODER2_GRAUS; //variavel de posição do encoder 2 (0 a 360 graus)

float VELOCIDADE_ENCODER1; //variavel de velocidade
float VELOCIDADE_ENCODER2; //variael de velocidade

uint16_t POSICAO_ATUAL1; // variavel para calculo de velocidade encoder 1
uint16_t POSICAO_ATUAL2; // variavel para calculo de velocidade encoder 2

uint16_t POSICAO_ANTERIOR1; // variavel para calculo de velocidade encoder 1
uint16_t POSICAO_ANTERIOR2; // variavel para calculo de velocidade encoder 2

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ///////////////////////////////////////////////////////////////////////////////////////////
	  /////////////////////////// REGISTRO DE POSIÇÃO DO ENCODER ////////////////////////////////
	  ///////////////////////////////////////////////////////////////////////////////////////////
	  // O registrador em TIMx->CNT é responsável por contar cada vez que é completado um ciclo
	  // do encoder, ou seja, sempre que há borda de subida e descida de ambos os canais. Assim,
	  // como o encoder utilizado para esse exemplo possui uma resolução de 1440/revolução, temos
	  // que o contador pode ir de 0 a 1440, decrescendo caso o motor gire em um sentido e crescendo
	  // caso o motor gire em outro sentido.

	  POSICAO_ENCODER1 = TIM1->CNT; // variavel que recebe o registrador de posição do encoder
	  POSICAO_ENCODER2 = TIM2->CNT; // variavel que recebe o registrador de posição do encoder

	  POSICAO_ENCODER1_GRAUS = (360/1440)*POSICAO_ENCODER1; //converte saida para graus
	  POSICAO_ENCODER2_GRAUS = (360/1440)*POSICAO_ENCODER2; //converte saida para graus

	  ///////////////////////////////////////////////////////////////////////////////////////////
	  ///////////////////////////////////////////////////////////////////////////////////////////
	  //obs: lembrar de configurar manualmente as funções de Timer (Código no final da main)


	  ///////////////////////////////////////////////////////////////////////////////////////////
	  ///////////////////////// REGISTRO DE VELOCIDADE DO ENCODER ///////////////////////////////
	  ///////////////////////////////////////////////////////////////////////////////////////////
	  // Função disponível em Src/stm32f1xx.it.c
	  // Implementado na função:
	  //
	  //	  TIM3_IRQHandler(void) // Para seguir o caminho basta descomentar a função segurar Ctrl
	                                // e clicar com o botao esquerdo do mouse na função.

	  VELOCIDADE_ENCODER1; //variavel atualizada a cada interrupção de TIMER3
	  VELOCIDADE_ENCODER2; //variavel atualizada a cada interrupção de TIMER3

	  ///////////////////////////////////////////////////////////////////////////////////////////
	  ///////////////////////////////////////////////////////////////////////////////////////////


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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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

  ///////////////////////////////////////////////////////////////////////////////////////////
  //////////////////// AJUSTE DE INTERRUPÇÃO DO ENCODER DE QUADRATURA ///////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////
  // Insere manualmente a opção de interrupção por subida e descida, uma vez que a CUBEIDE não
  // possui a opção ainda.
  // Obs.: o Hardware permite isso, essa é uma limitação da interface de configuração.

  sConfig.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;
  sConfig.IC2Polarity = TIM_ICPOLARITY_BOTHEDGE;

  ///////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////

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
  htim2.Init.Period = 0xffff-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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

  ///////////////////////////////////////////////////////////////////////////////////////////
  //////////////////// AJUSTE DE INTERRUPÇÃO DO ENCODER DE QUADRATURA ///////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////
  // Insere manualmente a opção de interrupção por subida e descida, uma vez que a CUBEIDE não
  // possui a opção ainda.
  // Obs.: o Hardware permite isso, essa é uma limitação da interface de configuração.

  sConfig.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;
  sConfig.IC2Polarity = TIM_ICPOLARITY_BOTHEDGE;

  ///////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////

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
  htim3.Init.Prescaler = 100-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 720-1;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
