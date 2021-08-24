#include "main.h"
#include "ssd1306.h"
#include "MAX30102.h"
#include "font.h"
#include <stdio.h>

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE *f)
{
  uint8_t temp[1]={ch};
  HAL_UART_Transmit(&huart2,temp,1,1);
  return ch;
}


int HR_count;
int HR;
int count;
int sum_HR;

double R[40];
double I[40];

double del1_R[30];
double del2_R[20];
double fil_R[20];
double MAX_R;
double MIN_R;
double MAX_fil_R;


double del1_I[30];
double del2_I[20];
double fil_I[20];
double MAX_I;
double MIN_I;
double MAX_fil_I;

double Ratio;
double spo;

uint16_t col1;
uint8_t wait_flag=0;
int test_count=0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  MAX_R=0;
  MAX_I=0;
  MIN_R=999999;
  MIN_I=999999;
  col1=18;
  
 
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();

  MAX30102_setup(0x6F, 32, 2, 400, 411, 16384); //MAX30102 setting
  
  
  uint8_t data[6];      //MAX30102 raw data buffer
  
  ssd1306_Init();       //ssd1306 Init
  ssd1306_Clear();      //ssd1306 clear
   
 
  while (1)
  {
    ssd1306_W_On_Heart(2, 0);   //Heart On
    /* USER CODE END WHILE */
    for(int i=0; i<40; i++)
    {
      MAX30102_FIFOWrite(data);
      I[i]=(data[0]<<16|data[1]<<8|data[2])&0x03FFFF;
      R[i]=(data[3]<<16|data[4]<<8|data[5])&0x03FFFF;
      
      if(R[i]<7000 && I[i]<7000)        //prevent chattering
      {
         R[i]=4000;
         I[i]=4000;
      }
      
      if(R[i]<5000 && I[i]<5000)        //No On finger
      {
         test_flag=1;
         ssd1306_W_Off_Heart(2, 0);     //Heart Off
         HAL_Delay(100);                //prevent chattering
      }
      
      if(test_flag==1)
      {
         //printf("Waiting...\r\n");
         
         ssd1306_W_String("On finger!",0,0);    
         ssd1306_W_String("HR:  0bpm",2,col1);
         ssd1306_W_String("SPO2:  0%",4,col1);
         
         test_flag=0;
         i=0;
      }
      else if(test_flag==0)
      {
        if(count==0)
        {
          ssd1306_W_String("testing   ",0,0);
          test_count++;
        }
        else if(count==1)
        {
          ssd1306_W_String("testing.  ",0,0);
          test_count++;
        }
        else if(count==2)
        {
          ssd1306_W_String("testing.. ",0,0);
          test_count++;
        }
        else if(count==3)
        {
          ssd1306_W_String("testing...",0,0);
          test_count=0;
        }
        ssd1306_W_String("HR:",2,col1);
        ssd1306_W_String("SPO2:",4,col1);
      }
      
           
      HAL_Delay(40);
      ssd1306_W_Off_Heart(2, 0);        //Heart Off
     
    }
 
    
    for(int i=0; i<30; i++) //first averaging filter, decrease noise
    {
      del1_R[i]=(R[i]+R[i+1]+R[i+2]+R[i+3]+R[i+4]+R[i+5]+R[i+6]+R[i+7]+R[i+8]+R[i+9])/10;
      del1_I[i]=(I[i]+I[i+1]+I[i+2]+I[i+3]+I[i+4]+I[i+5]+I[i+6]+I[i+7]+I[i+8]+I[i+9])/10;
      //printf("Red: %f, IR: %f\n\r",del1_R[i], del1_I[i]);
    }
    
    
    for(int i=0; i<20; i++) //second averaging filter, decrease noise
    {
      del2_R[i]=(del1_R[i]+del1_R[i+1]+del1_R[i+2]+del1_R[i+3]+del1_R[i+4]+del1_R[i+5]+del1_R[i+6]+del1_R[i+7]+del1_R[i+8]+del1_R[i+9])/10;
      del2_I[i]=(del1_I[i]+del1_I[i+1]+del1_I[i+2]+del1_I[i+3]+del1_I[i+4]+del1_I[i+5]+del1_I[i+6]+del1_I[i+7]+del1_I[i+8]+del1_I[i+9])/10;
      if(del2_R[i]>MAX_R)
        MAX_R=del2_R[i];
      if(del2_R[i]<MIN_R)
        MIN_R=del2_R[i];
      if(del2_I[i]>MAX_I)
        MAX_I=del2_I[i];
      if(del2_I[i]<MIN_I)
        MIN_I=del2_I[i];
    }
      
    for(int i=0; i<20; i++) //Set the Y-axis center to 0
      fil_I[i]=del1_I[i]-del2_I[i];
    
     
    
    for(int i=0; i<19; i++) //HR_count increases when moving from negative to positive
    {
      if(fil_I[i]<0 && fil_I[i+1]>0)
        HR_count++;
    }
    //printf("tp2\n");     
    sum_HR+=HR_count*38;        //40ms * 40loop * 38=60800ms~=1s 
    count++;                    //HR checks 3times
    
    if(count == 3)
    {
       int HR = sum_HR/3;
       int SPO2 = (int)(110-25*((MAX_R-MIN_R)/MIN_R)/((MAX_I-MIN_I)/MIN_I));
              
       char HR_lcd[3]={0};
       if( HR >= 100)          
        HR_lcd[0]=((HR/10)/10)%10+0x30;
       else
        HR_lcd[0]=' ';
       HR_lcd[1]=(HR/10)%10+0x30;
       HR_lcd[2]=HR%10+0x30;
       
       for(int i=0; i<3; i++)    //show HR
       {
        ssd1306_W_Char(HR_lcd[i],2,col1+font_width*3+font_width*i);
       }
       
       char SPO2_lcd[3]={0};
       if( SPO2 >= 100)         //limited 100%
       {
         SPO2=100;
         SPO2_lcd[0]=((SPO2/10)/10)%10+0x30;
       }
       else
         SPO2_lcd[0]=' ';
       SPO2_lcd[1]=(SPO2/10)%10+0x30;
       SPO2_lcd[2]=SPO2%10+0x30;
       
       for(int i=0; i<3; i++)   //show SPO2
       {
         ssd1306_W_Char(SPO2_lcd[i],4,col1+font_width*5+font_width*i);
       }
             
    
       col1=18;
       sum_HR=0;
       count=0;
    }
    
    HR_count=0;
    MAX_R=0;
    MIN_R=999999;
    MAX_I=0;
    MIN_I=999999;
  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.Prescaler = 8400-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000-1;
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
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

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

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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


