#include "output.h"
#include "main.h"
#include "can.h"
#include "input.h"
#include "tle92464.h"
#include "balance.h"
#include "sensors.h"

flagOutput_t flagOutput;
uint8_t tle92464_init;
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void gpio_output_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  HAL_GPIO_WritePin(FCO_LED1_GPIO_Port, FCO_LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(FCO_LED2_GPIO_Port, FCO_LED2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(FCO_LED3_GPIO_Port, FCO_LED3_Pin, GPIO_PIN_SET);
  
  /*Configure GPIO pins : FCO_LED1_Pin */
  GPIO_InitStruct.Pin = FCO_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FCO_LED1_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FCO_LED2_Pin */
  GPIO_InitStruct.Pin = FCO_LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FCO_LED2_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FCO_LED3_Pin */
  GPIO_InitStruct.Pin = FCO_LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FCO_LED3_GPIO_Port, &GPIO_InitStruct);
  
  tle9246x_init();
  tle92464_init = 0;
}

void output_clear()
{
  flagOutput.ledManual = OFF;
  flagOutput.ledFlat = OFF;
  flagOutput.ledSlope = OFF;
  flagOutput.balanceUp = OFF;
  flagOutput.balanceDown = OFF;
}

uint8_t output_init()
{
  uint8_t ret = FALSE;
  //uint16_t data[3] = { nvDitherFrequency, nvDitherCurrent, nvPwmFrequency };      
  uint16_t data[3] = { 100, 130, 1000 };
  
  if(tle92464_init == 0){
    tle9246x_disable_output();
    tle92464_init = (tle9246x_handler() == TRUE) ? 1 : 0;
  }
  else if(tle92464_init == 1) {
    // Now we should configure dither frequency
    tle92464_init = (tle9246x_set_settings(data) == TRUE) ? 2 : 1;
  }
  else if(tle92464_init == 2) {
    tle9246x_set_current(TLE9246x_CHANNEL_BALANCE_UP, 0);
    tle9246x_set_current(TLE9246x_CHANNEL_BALANCE_DOWN, 0);
    
    tle92464_init = 3;
  }
    
  if(tle92464_init == 3) {
    ret = TRUE;
    tle9246x_enable_output();
  }
  
  return ret;
}

void output_controller()
{
  static uint16_t balanceUpLastUpdatedCurrent = 0;
  static uint16_t balanceDownLastUpdatedCurrent = 0;
  static uint16_t timerTLE92464Update = 0;
  uint8_t flagTLE92464Update = FALSE;
  static uint8_t updateTimer = 0;
  
  
  if(tle92464_init == 0) {
    updateTimer += 2;
    if(updateTimer >= 10) {
      updateTimer = 0;
      output_init();                                                   // Every 10ms it is called
    }
  }
  else {
    
    if(flagTimer.tenMs == TRUE) {
      timerTLE92464Update += 10;
    }
    if(timerTLE92464Update >= 300) {
      flagTLE92464Update = TRUE;
      timerTLE92464Update = 0;
    }
    
    if((flagOutput.balanceUp == ON) && (flagOutput.balanceDown == ON)) {
      flagOutput.balanceUp = OFF;
      flagOutput.balanceDown = OFF;
      // For safety
      tle9246x_set_current(TLE9246x_CHANNEL_BALANCE_UP, 0);
      tle9246x_set_current(TLE9246x_CHANNEL_BALANCE_DOWN, 0);
    }
    else
    {
      if((balanceUpLastUpdatedCurrent != get_balanceUpCurrent()) || (flagTLE92464Update == TRUE)) {
        balanceUpLastUpdatedCurrent = get_balanceUpCurrent();
        if(flagOutput.balanceUp == ON)
        {
          tle9246x_set_current(TLE9246x_CHANNEL_BALANCE_UP, balanceUpLastUpdatedCurrent);
        }
        else
        {
          tle9246x_set_current(TLE9246x_CHANNEL_BALANCE_UP, 0);
          clear_balanceUpStall();
        }
      }
    
      if((balanceDownLastUpdatedCurrent != get_balanceDownCurrent()) || (flagTLE92464Update == TRUE)) {
        balanceDownLastUpdatedCurrent = get_balanceDownCurrent();
        if(flagOutput.balanceDown == ON)
        {
          tle9246x_set_current(TLE9246x_CHANNEL_BALANCE_DOWN, balanceDownLastUpdatedCurrent);
        }
        else
        {
          tle9246x_set_current(TLE9246x_CHANNEL_BALANCE_DOWN, 0);
          clear_balanceDownStall();
        }
      }
    }
    
    
    if(flagTimer.hundredMs == TRUE)
    {
      if((flagOutput.balanceUp == OFF) && (flagOutput.balanceDown == OFF)) {
        tle9246x_clear_errors(TRUE);
      }
      else {
        tle9246x_get_diagnostic();
        flagError.balanceUpValve = tle9246x_outputError[TLE9246x_CHANNEL_BALANCE_UP];
        flagError.balanceDownValve = tle9246x_outputError[TLE9246x_CHANNEL_BALANCE_DOWN];
        
        if((flagError.balanceUpValve == FALSE) && (flagError.balanceDownValve == FALSE))
        {
          tle9246x_clear_errors(FALSE);
        }
        else {
          if(flagOutput.balanceUp == OFF)                tle9246x_outputError_local[TLE9246x_CHANNEL_BALANCE_UP] = FALSE;
          if(flagOutput.balanceDown == OFF)              tle9246x_outputError_local[TLE9246x_CHANNEL_BALANCE_DOWN] = FALSE;
        }
      }
    }
  }
  
  if(flagOutput.ledSlope == ON)         HAL_GPIO_WritePin(FCO_LED1_GPIO_Port, FCO_LED1_Pin, GPIO_PIN_RESET);
  else                                  HAL_GPIO_WritePin(FCO_LED1_GPIO_Port, FCO_LED1_Pin, GPIO_PIN_SET);
  
  if(flagOutput.ledFlat == ON)          HAL_GPIO_WritePin(FCO_LED2_GPIO_Port, FCO_LED2_Pin, GPIO_PIN_RESET);
  else                                  HAL_GPIO_WritePin(FCO_LED2_GPIO_Port, FCO_LED2_Pin, GPIO_PIN_SET);
  
  if(flagOutput.ledManual == ON)        HAL_GPIO_WritePin(FCO_LED3_GPIO_Port, FCO_LED3_Pin, GPIO_PIN_RESET);
  else                                  HAL_GPIO_WritePin(FCO_LED3_GPIO_Port, FCO_LED3_Pin, GPIO_PIN_SET);
}
