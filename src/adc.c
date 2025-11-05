#include "adc.h"
#include "main.h"


/* --------------------------------------------------------- STM32 ADC & DMA variables ---------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* --------------------------------------------------------- Global variables ---------------------------------------------------*/
uint8_t dma_adc_complete;                               // It is externed in Interrupt function
uint16_t adc_converter_counter;
/* --------------------------------------------------------- Local variables ---------------------------------------------------*/
uint32_t adcValue[NUMBER_OF_ADC_CHANNELS];                                      // The DMA (RAW ADC) value is saved in here
uint32_t totalAdcValue[NUMBER_OF_ADC_CHANNELS][NUMBER_OF_ADC_AVERAGE];          // The defined number of conversion is added and saved in here
uint16_t adcAverageValue[NUMBER_OF_ADC_CHANNELS];                               // The average ADC value is save in here

/* --------------------------------------------------------- Local Functions ---------------------------------------------------*/
uint16_t getCurrentADCValue(uint8_t channel);
uint16_t getAverageADCValue(uint8_t channel);
uint8_t  getCurrentADCValues(uint16_t *channels_data);
uint8_t  getAverageADCValues(uint16_t *channels_data);

uint8_t startConversation()
{
  return HAL_ADC_Start_DMA(&hadc1, adcValue, NUMBER_OF_ADC_CHANNELS);
}

uint16_t getCurrentADCValue(uint8_t channel)
{
  if(channel >= NUMBER_OF_ADC_CHANNELS)
    return -1;
  
  return (uint16_t)adcValue[channel];
}

uint16_t getAverageADCValue(uint8_t channel)
{
  if(channel >= NUMBER_OF_ADC_CHANNELS)
    return -1;
  
  return (uint16_t)adcAverageValue[channel];
}

uint8_t getCurrentADCValues(uint16_t *channels_data)
{
  uint8_t i;
  for(i = 0; i < NUMBER_OF_ADC_CHANNELS; i++){
    channels_data[i] = adcValue[i];
  }
  return 1;
}

uint8_t getAverageADCValues(uint16_t *channels_data)
{
  uint8_t i;
  for(i = 0; i < NUMBER_OF_ADC_CHANNELS; i++){
    channels_data[i] = adcAverageValue[i];
  }
  return 1;
}

uint8_t updateADC()
{
  uint8_t i;
  if(dma_adc_complete)
  {
    dma_adc_complete = 0;
    for(i = 0; i < NUMBER_OF_ADC_CHANNELS; i++)
    {
      totalAdcValue[i][0] += adcValue[i];
    }
    adc_converter_counter++;
  }
  
  if(adc_converter_counter >= NUMBER_OF_ADC_AVERAGE)
  {
    for(i = 0; i < NUMBER_OF_ADC_CHANNELS; i++)
    {
      adcAverageValue[i] = (uint16_t)((float)totalAdcValue[i][0] / adc_converter_counter);
      totalAdcValue[i][0] = 0;
    }
    adc_converter_counter = 0;
    return 1;
  }
  return 0;
}

uint8_t updateLastAverageADC()
{
  uint8_t i;
  if(dma_adc_complete)
  {
    dma_adc_complete = 0;
    for(i = 0; i < NUMBER_OF_ADC_CHANNELS; i++)
    {
      totalAdcValue[i][adc_converter_counter] = adcValue[i];
    }
    adc_converter_counter++;
    if(adc_converter_counter >= 4) {
      adc_converter_counter = 0;
    }
    
    for(i = 0; i < NUMBER_OF_ADC_CHANNELS; i++)
    {
      adcAverageValue[i] = (uint16_t)((float)(totalAdcValue[i][0] + totalAdcValue[i][1] + totalAdcValue[i][2] + totalAdcValue[i][3]) / NUMBER_OF_ADC_AVERAGE);
    }
    return 1;
  }
  return 0;
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = NUMBER_OF_ADC_CHANNELS;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}


/**
* @brief ADC MSP Initialization
* This function configures the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration    
    PC0     ------> ADC1_IN10
    PC1     ------> ADC1_IN11
    PC2     ------> ADC1_IN12
    PC3     ------> ADC1_IN13
    PA0     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    */
    GPIO_InitStruct.Pin = ADC1_Pin|ADC2_Pin|ADC3_Pin|ADC4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ADC5_Pin|ADC6_Pin|ADC7_Pin|ADC8_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc1);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

/**
* @brief ADC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PC0     ------> ADC1_IN10
    PC1     ------> ADC1_IN11
    PC2     ------> ADC1_IN12
    PC3     ------> ADC1_IN13
    PA0     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    */
    HAL_GPIO_DeInit(GPIOC, ADC1_Pin|ADC2_Pin|ADC3_Pin|ADC4_Pin);

    HAL_GPIO_DeInit(GPIOA, ADC5_Pin|ADC6_Pin|ADC7_Pin|ADC8_Pin);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
  dma_adc_complete = 1;
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}