#include "fram.h"
#include "main.h"

uint8_t manufacturerID;
uint8_t continuationCode;
uint16_t productID;

extern SPI_HandleTypeDef hspi2;

void fram_enable()
{
  HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, GPIO_PIN_RESET);              // active FRAM
}

void fram_disable()
{
  HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, GPIO_PIN_SET);                // deactive FRAM
}

uint8_t fram_init()
{
  uint8_t ret = 1;
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, GPIO_PIN_SET);                // Not active the FRAM

  /*Configure GPIO pin : MEM_CS_Pin */
  GPIO_InitStruct.Pin = MEM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MEM_CS_GPIO_Port, &GPIO_InitStruct);
  return ret;
}

#if defined(MB85RS64V) || defined(MB85RS16N)
/**
 * @brief check the IC
 */
uint8_t fram_check(void)
{
  uint8_t t_data[1] = {OP_CODE_RDID};                                           // command byte
  uint8_t r_data[4] = {0x00, 0x00, 0x00, 0x00};                                 // The received data bytes
  uint8_t ret = 0;
  
  fram_enable();
  
  HAL_SPI_Transmit(&hspi2, t_data, 1, 10);                                      // Write op code
  HAL_SPI_Receive(&hspi2, r_data, 4, 10);
  
  fram_disable();
  
  manufacturerID        = r_data[0];
  continuationCode      = r_data[1];
  productID             = (r_data[2] << 8) + r_data[3];
  
  if((manufacturerID == MANUFACTURER_ID) && (continuationCode == CONTINUATION_CODE) && (productID == PRODUCT_ID))
  {
    ret = 1;
  }
  return ret;
}
#endif

/**
 * @brief Fram read status
 */
uint8_t fram_read_status(uint8_t *status)
{
  uint8_t t_data[1] = {OP_CODE_RDSR};
  uint8_t ret;

  fram_enable();
  ret = HAL_SPI_Transmit(&hspi2, t_data, 1, 10);                                // Write op code
  ret = HAL_SPI_Receive(&hspi2, status, 1, 10);                                 // Read status
  fram_disable();

  return ret;
}

/**
 * @brief Fram write status
 */
uint8_t fram_write_status(uint8_t status)
{
  uint8_t t_data[2] = {OP_CODE_RDSR, status};
  uint8_t ret;

  fram_enable();
  ret = HAL_SPI_Transmit(&hspi2, t_data, 2, 10);                                // Write op code & status
  fram_disable();

  return ret;
}

/**
 * @brief Fram write enable
 */
uint8_t fram_write_enable()
{
  uint8_t t_data[1] = {OP_CODE_WREN};
  uint8_t ret;

  fram_enable();
  ret = HAL_SPI_Transmit(&hspi2, t_data, 1, 10);
  fram_disable();

  return ret;
}

/**
 * @brief Fram write disable
 */
uint8_t fram_write_disable()
{
  uint8_t t_data[1] = {OP_CODE_WRDI};
  uint8_t ret;

  fram_enable();
  ret = HAL_SPI_Transmit(&hspi2, t_data, 1, 10);
  fram_disable();

  return ret;
}

/**
 * @brief Fram write data whose size is 4 bytes = 32bit
 */
uint8_t fram_write(uint16_t addr, uint8_t* dt)
{
  uint8_t t_data[7] = {0};
  uint8_t ret;

  t_data[0] = OP_CODE_WRITE;                                                    //op code : write cmd
  t_data[1] = (uint8_t)((addr & 0xFF00)>>8);
  t_data[2] = (uint8_t)(addr & 0xFF);
  t_data[3] = dt[0];
  t_data[4] = dt[1];
  t_data[5] = dt[2];
  t_data[6] = dt[3];
  
  fram_write_enable();

  fram_enable();
//  watchdog_trigger();
  ret = HAL_SPI_Transmit(&hspi2, t_data, 7, 10);

  fram_disable();

  fram_write_disable();

  return ret;
}

/**
 * @brief Fram read data whose size is 4 bytes = 32bit
 */
uint8_t fram_read(uint16_t addr, uint8_t *rdt)
{
  uint8_t t_data[3] = {OP_CODE_READ, (uint8_t)((addr & 0xFF00)>>8), (uint8_t)(addr & 0xFF)};
  uint8_t ret;

  fram_enable();
//  watchdog_trigger();
  HAL_SPI_Transmit(&hspi2, t_data, 3, 10);                                // Write op command & address
  ret = HAL_SPI_Receive(&hspi2, rdt, 4, 10);                                    // Read data
  
  fram_disable();

  return ret;
}

/**
 * @brief Fram testing function
 */
uint8_t fram_test()
{
  uint8_t i;
  uint8_t ret = 1;
  uint8_t t_data[4] = {0x11, 0x22, 0x33, 0x44 };
  uint8_t r_data[4] = {0x00, 0x00, 0x00, 0x00 };
  fram_write(0, t_data);
  fram_read(0, r_data);
  
  for( i = 0; i < 4; i++)
  {
    if(r_data[i] != t_data[i])
    {
      ret = 0;
      break;
    }
  }
  return ret;
}

