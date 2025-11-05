#include "input.h"
#include "adc.h"
#include "main.h"
#include "usart.h"

flagInputStatus_t flagInputStatus;
flagInputStatus_t flagInputStatusPrevious;
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void gpio_input_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  
  /*Configure GPIO pins : SW_UP_EXT_Pin */
  GPIO_InitStruct.Pin = SW_UP_EXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_UP_EXT_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : SW_DOWN_EXT_Pin */
  GPIO_InitStruct.Pin = SW_DOWN_EXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_DOWN_EXT_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : SW_MODE_Pin */
  GPIO_InitStruct.Pin = SW_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_MODE_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : SW_UP_INT_Pin */
  GPIO_InitStruct.Pin = SW_UP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_UP_INT_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : SW_DOWN_INT_Pin */
  GPIO_InitStruct.Pin = SW_DOWN_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_DOWN_INT_GPIO_Port, &GPIO_InitStruct);
}

void read_digital_inputs()
{
  static uint8_t rawData[NUMBER_OF_INPUTS];
  uint8_t i;
    
  for(i = 0; i < NUMBER_OF_INPUTS; i++) {
    rawData[i] <<= 1;
  }
  
  rawData[0] = (HAL_GPIO_ReadPin(SW_UP_EXT_GPIO_Port, SW_UP_EXT_Pin) == GPIO_PIN_SET)           ? (rawData[0]+1) : rawData[0];
  rawData[1] = (HAL_GPIO_ReadPin(SW_DOWN_EXT_GPIO_Port, SW_DOWN_EXT_Pin) == GPIO_PIN_SET)       ? (rawData[1]+1) : rawData[1];
  rawData[2] = (HAL_GPIO_ReadPin(SW_MODE_GPIO_Port, SW_MODE_Pin) == GPIO_PIN_RESET)             ? (rawData[2]+1) : rawData[2];
  rawData[3] = (HAL_GPIO_ReadPin(SW_UP_INT_GPIO_Port, SW_UP_INT_Pin) == GPIO_PIN_RESET)         ? (rawData[3]+1) : rawData[3];
  rawData[4] = (HAL_GPIO_ReadPin(SW_DOWN_INT_GPIO_Port, SW_DOWN_INT_Pin) == GPIO_PIN_RESET)     ? (rawData[4]+1) : rawData[4];
  
  flagInputStatusPrevious.modeButton = flagInputStatus.modeButton;
  
  flagInputStatus.externalUpButton = (rawData[0] == 255) ? ON :  OFF;
  flagInputStatus.externalDownButton = (rawData[1] == 255) ? ON :  OFF;
  flagInputStatus.modeButton = (rawData[2] == 255) ? ON :  OFF;
  flagInputStatus.internalUpButton = (rawData[3] == 255) ? ON :  OFF;
  flagInputStatus.internalDownButton = (rawData[4] == 255) ? ON :  OFF;
}

uint16_t get_raw_hitch_dial()                   { return getAverageADCValue(ADC_HIGHSET);       }
uint16_t get_raw_balance_sensitive_dial()       { return getAverageADCValue(ADC_SENSTIVE);      }
uint16_t get_raw_balance_configure_dial()       { return getAverageADCValue(ADC_SET_BALL);      }
uint16_t get_raw_direction()                    { return getAverageADCValue(ADC_DIR);           }
uint16_t get_raw_hitch_position_sensor()        { return getAverageADCValue(ADC_LIFT);          }
uint16_t get_raw_stroke_sensor()                { return getAverageADCValue(ADC_STROKE);        }
uint16_t get_raw_balance_sensor()               { return getAverageADCValue(ADC_BAL);           }
uint16_t get_raw_battery()                      { return getAverageADCValue(ADC_PWR);           }