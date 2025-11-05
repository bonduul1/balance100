#include "usart.h"
#include "main.h"

/* --------------------------------------------------------- STM32 USART variables ----------------------------------------------*/
UART_HandleTypeDef huart1;
/* --------------------------------------------------------- Global variables ---------------------------------------------------*/
#ifdef __GNUC__

/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  uint8_t data = (uint8_t)ch;
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, &data, 1, 10);
  return ch;
}

/* --------------------------------------------------------- Local variables  ---------------------------------------------------*/
uint8_t dataReceived;
uint8_t rx_data[RX_PACKET_LENGTH];
uint8_t received_data[RX_PACKET_LENGTH];

uint8_t startUsart()
{
  uint8_t i;
  
  dataReceived = 0;
  for(i = 0; i < RX_PACKET_LENGTH; i++){
    rx_data[i] = 0;
    received_data[i] = 0;
  }
  return HAL_UART_Receive_IT(&huart1, rx_data, RX_PACKET_LENGTH);
}

uint8_t checkUsart()
{
  if(dataReceived){
    dataReceived = 0;
    HAL_UART_Receive_IT(&huart1, rx_data, RX_PACKET_LENGTH);
    HAL_UART_Transmit(&huart1, rx_data, 1, 10);
    return 1;
  }
  return 0;
}

uint8_t updateUsart()
{
  uint8_t i;
  if(dataReceived){
    dataReceived = 0;
    HAL_UART_Receive_IT(&huart1, rx_data, RX_PACKET_LENGTH);
    for( i = 0; i < RX_PACKET_LENGTH; i++)
    {
      received_data[i] = rx_data[i];
    }
    return 1;
  }
  return 0;
}

void usart_transmit(uint8_t *tx_data, uint8_t len)
{
  HAL_UART_Transmit(&huart1, tx_data, len, len*10);
}

void usart_transmit_one(uint8_t tx_data)
{
  HAL_UART_Transmit(&huart1, &tx_data, 1, 10);
}

void usart_receive(uint8_t *data)
{
  uint8_t i;
  for(i = 0; i < RX_PACKET_LENGTH; i++){
    data[i] = received_data[i];
  }  
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  dataReceived = 1;
  /* USER CODE END USART1_IRQn 1 */
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    dataReceived = 1;
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
// This function is disabled because we did not use the UART    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */
// Disabled on 2022.01.05    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
// Disabled on 2022.01.05    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE END USART1_MspInit 1 */
  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */
// Disabled on 2022.01.05  HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE END USART1_MspDeInit 1 */
  }

}
