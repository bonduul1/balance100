
#include "watchdog.h"

void watchdog_init()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  
  HAL_GPIO_WritePin(WDT_CLK_GPIO_Port, WDT_CLK_Pin, GPIO_PIN_RESET);            // Clock output
  HAL_GPIO_WritePin(WDT_EN_GPIO_Port, WDT_EN_Pin, GPIO_PIN_RESET);              // watchdog is disabled
  
  /*Configure GPIO pins : WDT_CLK_Pin */
  GPIO_InitStruct.Pin = WDT_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WDT_CLK_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : WDT_EN_Pin */
  GPIO_InitStruct.Pin = WDT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WDT_EN_GPIO_Port, &GPIO_InitStruct);
}

void watchdog_enable()
{
  HAL_GPIO_WritePin(WDT_EN_GPIO_Port, WDT_EN_Pin, GPIO_PIN_SET);
}

void watchdog_disable()
{
  HAL_GPIO_WritePin(WDT_EN_GPIO_Port, WDT_EN_Pin, GPIO_PIN_RESET);
}

void watchdog_trigger()
{
  HAL_GPIO_TogglePin(WDT_CLK_GPIO_Port, WDT_CLK_Pin);
}