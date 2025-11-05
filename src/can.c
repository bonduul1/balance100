#include "can.h"
#include "main.h"
#include "output.h"
#include "input.h"
#include "sensors.h"
#include "kiwonHardware.h"
#include "watchdog.h"
#include "balance.h"

/* --------------------------------------------------------- STM32 CAN variables ---------------------------------------------------*/
CAN_HandleTypeDef       hcan;

/* --------------------------------------------------------- CAN variables ---------------------------------------------------------*/
CAN_TxHeaderTypeDef     canTxHeader;
CAN_RxHeaderTypeDef     canRxHeader;
uint8_t                 canTxData[8];
uint8_t                 canRxData[8];
uint32_t                canTxMailbox;

canRxAngle_t            canRxAngle;
canRxAngle_t            canRxAngleTemp;
canRxAngle_t            canRxAnglePrevious;

canRxGyro_t             canRxGyro;
canRxGyro_t             canRxGyroTemp;
canRxGyro_t             canRxGyroPrevious;
/* --------------------------------------------------------- External Functions ----------------------------------------------------*/


/* --------------------------------------------------------- Local Functions ----------------------------------------------------*/
uint8_t can_transmit(uint32_t id, uint8_t _data[]);
void MX_CAN_FILTER_Init(uint8_t filter, uint32_t id, uint32_t mask);
void run_bootloader(void);
void can_interrupt_enable();

void can_variable_init()
{
}

void can_receive_process()
{
  uint8_t i;
  
  if(canRxAngleTemp.flag == TRUE)
  {
    canRxAngleTemp.flag = FALSE;
    for(i = 0; i < 8; i++)
    {
      canRxAnglePrevious.data[i] = canRxAngle.data[i];
      canRxAngle.data[i] = canRxAngleTemp.data[i];
    }
  }
  
  if(canRxGyroTemp.flag == TRUE)
  {
    canRxGyroTemp.flag = FALSE;
    for(i = 0; i < 8; i++)
    {
      canRxGyroPrevious.data[i] = canRxGyro.data[i];
      canRxGyro.data[i] = canRxGyroTemp.data[i];
    }
  }
}

uint8_t can_transmit_packet()
{
  uint8_t txCANData[8] = {0,0,0,0,0,0,0,0};
  
  if(flagInputStatus.externalUpButton == ON)            txCANData[0] |= 0x01;
  if(flagInputStatus.externalDownButton == ON)          txCANData[0] |= 0x02;
  if(flagInputStatus.modeButton == ON)                  txCANData[0] |= 0x04;
  if(flagInputStatus.internalUpButton == ON)            txCANData[0] |= 0x08;
  if(flagInputStatus.internalDownButton == ON)          txCANData[0] |= 0x10;
  
  if(flagOutput.ledManual == ON)                        txCANData[1] |= 0x01;
  if(flagOutput.ledFlat == ON)                          txCANData[1] |= 0x02;
  if(flagOutput.ledSlope == ON)                         txCANData[1] |= 0x04;
  
  if(flagOutput.balanceUp == ON)                        txCANData[1] |= 0x10;
  if(flagOutput.balanceDown == ON)                      txCANData[1] |= 0x20;
  if(flagError.balanceUpValve == ON)                    txCANData[1] |= 0x40;
  if(flagError.balanceDownValve == ON)                  txCANData[1] |= 0x80;
  
  if(flagOutput.balanceUp == ON)
  {
    txCANData[2] = get_balanceUpCurrent();
    txCANData[3] = get_balanceUpCurrent() >> 8;
  }

  if(flagOutput.balanceDown == ON)
  {
    txCANData[4] = get_balanceDownCurrent();
    txCANData[5] = get_balanceDownCurrent() >> 8;
  }
  
  txCANData[6] = VAC_PROGRAM_VERSION;
  txCANData[7] = flagBalance.balanceMode;
    
  return can_transmit(TRANSMISSION_PACKET_ID, txCANData);
}

uint8_t can_transmit_packet1()
{
  uint8_t txCANData[8] = {0,0,0,0,0,0,0,0};
  
  txCANData[0] = get_hitch_dial() >> 4;
  txCANData[1] = get_balance_sensitive_dial() >> 4;
  txCANData[2] = get_balance_configure_dial() >> 4;
  txCANData[3] = get_direction() >> 4;
  txCANData[4] = get_hitch_position_sensor() >> 4;
  txCANData[5] = get_stroke_sensor() >> 4;
  txCANData[6] = get_balance_sensor() >> 4;
  txCANData[7] = get_battery_voltage() >> 4;
  
  return can_transmit(TRANSMISSION_PACKET1_ID, txCANData);
}

uint8_t can_transmit_process()
{
  static uint16_t timerPacket = 0;
  static uint16_t timerPacket1 = 0;

  timerPacket += 2;
  timerPacket1 += 2;
  
  if(timerPacket >= TRANSMISSION_PACKET_TIME) {
    if(can_transmit_packet() == TRUE) {
      timerPacket = 0;
    }
  }
  else if(timerPacket1 >= TRANSMISSION_PACKET_TIME) {
    if(can_transmit_packet1() == TRUE) {
      timerPacket1 = 0;
    }
  }
  
  return TRUE;
}

void can_data_handler(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData)
{
  uint8_t i;
  
  switch(rxHeader->ExtId)
  {
    case CAN_RX_PACKET_ID:
      break;
      
    case CAN_RX_ANGLE_ID:
      canRxAngleTemp.flag = TRUE;
      for(i = 0; i < 8; i++)
        canRxAngleTemp.data[i] = rxData[i];
      break;
          
    case CAN_RX_GYRO_ID:
      canRxGyroTemp.flag = TRUE;
      for(i = 0; i < 8; i++)
        canRxGyroTemp.data[i] = rxData[i];
      break;
  }
}

uint8_t can_transmit(uint32_t id, uint8_t _data[])
{
  static uint16_t canErrorCounter = 0;
  
  canTxHeader.IDE = CAN_ID_EXT;
  canTxHeader.RTR = CAN_RTR_DATA;
  canTxHeader.ExtId = id;
  canTxHeader.DLC = 8;
  
  canTxData[0] = _data[0];
  canTxData[1] = _data[1];
  canTxData[2] = _data[2];
  canTxData[3] = _data[3];
  canTxData[4] = _data[4];
  canTxData[5] = _data[5];
  canTxData[6] = _data[6];
  canTxData[7] = _data[7];
  
  canTxHeader.TransmitGlobalTime = DISABLE;
  if (HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &canTxMailbox) != HAL_OK)
  {
    canErrorCounter++;
    return FALSE;
  }
  return TRUE;
}

/* CAN Interrupt function */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(hcan->Instance == CAN1){
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canRxHeader, canRxData) != HAL_OK)
    {
      /* Reception Error */
    }

    if(canRxHeader.ExtId == BOOTLOADER_COMMAND_ID) {
      if((canRxData[0] == TYM_BALANCE_CONTROLLER) && (canRxData[1] == 0x80)) {    // CAN ID MATCHED and upgrade flag is enabled so the program should be going to bootloader address.
        run_bootloader();
      }
    }
    else {
      can_data_handler(&canRxHeader, canRxData);
    }
    canRxHeader.DLC = 0;
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* USER CODE BEGIN CAN1_Init 2 */
  
  // Filter CAN ID is updated on 2021.12.28 for mass product
  MX_CAN_FILTER_Init(0, BOOTLOADER_COMMAND_ID, BOOTLOADER_COMMAND_ID);
  MX_CAN_FILTER_Init(1, CAN_RX_PACKET_ID, CAN_RX_PACKET_ID);
  MX_CAN_FILTER_Init(1, CAN_RX_ANGLE_ID, CAN_RX_ANGLE_ID);
  MX_CAN_FILTER_Init(1, CAN_RX_GYRO_ID, CAN_RX_GYRO_ID);
  

  if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    // Disabled on 2022.12.19   Error_Handler();
  }

  if(HAL_CAN_Start(&hcan) != HAL_OK)
  {
    // Disabled on 2022.12.19   Error_Handler();
  }
  /* USER CODE END CAN1_Init 2 */
  
  // Changed the interrupt position on 2022.01.05
  can_interrupt_enable();
}

/* CAN FILTER init function*/
void MX_CAN_FILTER_Init(uint8_t filter, uint32_t id, uint32_t mask)
{
  CAN_FilterTypeDef  sFilterConfig;

  if(filter > 27){
    filter = 27;
  }
  
  sFilterConfig.FilterBank = filter;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
    
  id = (id << 3) | (1 << 2);                                    // IDE should be 1        
  mask = (mask << 3) | (1 << 2);                                // IDE should be 1
  
  sFilterConfig.FilterIdLow = id;
  sFilterConfig.FilterIdHigh = id >> 16;
  sFilterConfig.FilterMaskIdLow = id & mask;
  sFilterConfig.FilterMaskIdHigh = (id & mask) >> 16;

  if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    // Filter configuration Error 
    // Disabled on 2022.12.19   Error_Handler();
  }
}

/**
* @brief CAN MSP Initialization
* This function configures the hardware resources used in this example
* @param hcan: CAN handle pointer
* @retval None
*/
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void can_interrupt_enable()
{
  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}


/**
* @brief CAN MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hcan: CAN handle pointer
* @retval None
*/
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/**
  * @brief This function handles CAN1 RX0 interrupt.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/*--------------------------------------------------------- Bootloader handler function on user side start -----------------------------------------------------*/
void run_bootloader(void)
{
  watchdog_enable();
  while(1);
}
/*--------------------------------------------------------- Bootloader handler function on user side end -----------------------------------------------------*/