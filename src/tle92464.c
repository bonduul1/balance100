 /*
  *     File:           TLE9246x.c
  *     Author:         Enkhbat Batbayar
  *     Date:           2022.06.13
  *     Version:        1.0
  *     Note:           TLE9246x IC driver and testing source code
  *                     We do not need DRVx pins for controlling
  */

#include "TLE92464.h"
#include "main.h"
#include "spi.h"
#include "defines.h"
#include "main.h"
#include "usart.h"
#include "output.h"

/*---------------------------- Variables -------------------------------------*/
#define FSYS                    (float)28000000
#define EXP                     4
#define POW2EXP                 (float)16                                       // (2^4 = 16)
#define FLAT                    0
#define STEPS                   243
#define LOW_FREQ_RANGE_EN       8
#define PERIOD_EXP              7
#define POW2PERIOD_EXP          (float)128                                      // (2^7 = 128)                  --> This data is chnaged on 2022.08.22 because of the minimum frequency, now the minimum frequency is 107Hz, maximum is 27KHz

uint8_t crcTable[256];
#ifdef  TLE92466_USED
#ifdef TLE9246x_PARALLEL_CHANNEL_USED
  const uint16_t channelBaseAddress[TLE9246x_CHANNELS] = {
    CHANNEL0_REGISTER_BASE_ADDRESS,
    CHANNEL1_REGISTER_BASE_ADDRESS,
    CHANNEL2_REGISTER_BASE_ADDRESS,
  };
#else
  const uint16_t channelBaseAddress[TLE9246x_CHANNELS] = {
    CHANNEL0_REGISTER_BASE_ADDRESS,
    CHANNEL1_REGISTER_BASE_ADDRESS,
    CHANNEL2_REGISTER_BASE_ADDRESS,
    CHANNEL3_REGISTER_BASE_ADDRESS,
    CHANNEL4_REGISTER_BASE_ADDRESS,
    CHANNEL5_REGISTER_BASE_ADDRESS
  };
#endif
#endif
#ifdef  TLE92464_USED
#ifdef TLE9246x_PARALLEL_CHANNEL_USED
  const uint16_t channelBaseAddress[TLE9246x_CHANNELS] = {
    CHANNEL0_REGISTER_BASE_ADDRESS,
    CHANNEL1_REGISTER_BASE_ADDRESS,
  };
#else
  const uint16_t channelBaseAddress[TLE9246x_CHANNELS] = {
    CHANNEL0_REGISTER_BASE_ADDRESS,
    CHANNEL1_REGISTER_BASE_ADDRESS,
    CHANNEL2_REGISTER_BASE_ADDRESS,
    CHANNEL3_REGISTER_BASE_ADDRESS,
  };
#endif
#endif

uint16_t tle9246x_outputCurrent[TLE9246x_CHANNELS]; 
uint8_t  tle9246x_outputError[TLE9246x_CHANNELS];
uint16_t tle9246x_outputError_local[TLE9246x_CHANNELS];
uint16_t tle9246x_outputFrequency;
uint16_t tle9246x_ditherCurrent;
uint16_t tle9246x_ditherFrequency;

uint16_t channelEnableConfig;
uint16_t timerDiagnostic = 0;
uint8_t  initFinished;
uint8_t  tle9246x_state = TLE9246x_INIT_STEP_0;
/*---------------------------- Variables -------------------------------------*/

/*---------------------------- Base Functions --------------------------------*/

uint8_t crc_init()
{
  uint8_t _crc;
  uint16_t i;
  uint8_t bit;
  
  for (i = 0; i < 256; i++) {
    _crc = i;
    for (bit = 0; bit < 8; bit++) {
      _crc = (_crc & 0x80) ? ((_crc << 1) ^ TLE9246x_CRC) : (_crc << 1);
    }
    crcTable[i] = _crc;
  }
  return TRUE;
}

uint8_t tle9246x_calculate_crc(uint32_t data)
{
  uint8_t len = 3;
  uint8_t buf[3];
  buf[0] = ((data >> 0) & 0xFF);
  buf[1] = ((data >> 8) & 0xFF);
  buf[2] = ((data >> 16) & 0xFF);
    
  uint8_t index = 0;
  uint8_t _crc = 0xFF;

  while(len--) {
    _crc = crcTable[_crc ^ buf[index++]];
  }
  return ~_crc;
}

uint8_t tle9246x_check_crc(uint32_t data)
{
  uint8_t ret;
  uint8_t calc;
  calc = tle9246x_calculate_crc(data);
  if(calc == (uint8_t)((data >> 24) & 0xFF)) {
    ret = TRUE;
  }
  else {
    ret = FALSE;
  }
  return ret;
}

void tle9246x_enable_output()
{
  HAL_GPIO_WritePin(SPI_OUT_EN_GPIO_Port, SPI_OUT_EN_Pin, GPIO_PIN_SET);
}

void tle9246x_disable_output()
{
  HAL_GPIO_WritePin(SPI_OUT_EN_GPIO_Port, SPI_OUT_EN_Pin, GPIO_PIN_RESET);
}

void tle9246x_enable_reset()
{
  HAL_GPIO_WritePin(FO1_RESN_GPIO_Port, FO1_RESN_Pin, GPIO_PIN_RESET);
}

void tle9246x_disable_reset()
{
  HAL_GPIO_WritePin(FO1_RESN_GPIO_Port, FO1_RESN_Pin, GPIO_PIN_SET);
}

int8_t tle9246x_getStatus()
{
  return (HAL_GPIO_ReadPin(FO1_FAULTN_GPIO_Port, FO1_FAULTN_Pin) == GPIO_PIN_RESET) ? 0 : 1;
}

void tle9246x_disableSPI()
{
  HAL_GPIO_WritePin(FO1_CS_GPIO_Port, FO1_CS_Pin, GPIO_PIN_SET);
}

void tle9246x_enableSPI()
{
  HAL_GPIO_WritePin(FO1_CS_GPIO_Port, FO1_CS_Pin, GPIO_PIN_RESET);
}

uint8_t tle9246x_readRegister(uint16_t address, uint32_t *rData)
{
  uint8_t ret;
  reply22_t reply22;
  readFrame_t readFrame;

  readFrame.rw = TLE9246x_READ;
  readFrame.reserved = 0;
  readFrame.address = address;
  readFrame.crc = tle9246x_calculate_crc(readFrame.reg);
  
  tle9246x_enableSPI();
  delay300ns();
  SPI_TransmitReceive(readFrame.reg);
  delay300ns();
  tle9246x_disableSPI();

//  delay300ns();
  delay300ns();
  
  tle9246x_enableSPI();
  delay300ns();  
  reply22.reg = SPI_TransmitReceive(readFrame.reg);
  delay300ns();
  tle9246x_disableSPI();

//  delay300ns();
//  delay300ns();
  
  if(tle9246x_check_crc(reply22.reg) == TRUE) {
    *rData = reply22.reg;
    ret = TRUE;
  }
  else {
    *rData = 0;
    ret = FALSE;
  }
  return ret;
}

uint8_t tle9246x_writeRegister(uint16_t address, uint16_t tData)
{
  writeFrame_t writeFrame;
  writeFrame.reg = 0;
  
  writeFrame.rw = TLE9246x_WRITE;
  writeFrame.address = (uint8_t)(address & 0x007F);                             // 7 bit address is used for writing
  writeFrame.data = tData;
  writeFrame.crc = tle9246x_calculate_crc(writeFrame.reg);
  
  tle9246x_enableSPI();
  delay300ns();
  
  SPI_TransmitReceive(writeFrame.reg);
  
  delay300ns();
  tle9246x_disableSPI();
  
//  delay300ns();
//  delay300ns();
  return TRUE;
}

/*---------------------------- General Functions -----------------------------*/
uint8_t set_data16(uint16_t address, uint16_t data)
{
  return tle9246x_writeRegister(address, data);
}

uint8_t get_data16(uint16_t address, uint16_t *data)
{
  uint8_t ret;
  reply16_t reply;
  ret = tle9246x_readRegister(address, &reply.reg);
  
  *data = reply.data;
  return ret;
}

uint8_t get_data22(uint16_t address, uint32_t *data)
{
  uint8_t ret;
  reply22_t reply;
  ret = tle9246x_readRegister(address, &reply.reg);
  
  *data = reply.data;

  return ret;
}

uint8_t check_ICVID()
{
 /*
  *     Manufacturer ID (15:8)  = 0xC1 (TLE92466 & TLE92464)
  *     Chip Version    (7:0)
  *             a. 0xFC B11 Design Step (TLE92464), 0xFC A11 Design Step (TLE92466)
  *             b. 0xFD B12 Design Step (TLE92464)
  *             c. 0xFE B13 Design Step (TLE92464), 0xFE A13 Design Step (TLE92466)
  *             d. 0xFF B15 Design Step (TLE92464)
  */
  uint32_t data;
  uint8_t ret;
  
  ret = get_data22(CENTRAL_REGISTER_BASE_ADDRESS + ICVID, &data);
  
  if(ret == TRUE) {
    reply16_t reply;
    reply.reg = data;
    if((uint8_t)(reply.data >> 8) == TLE9246x_MANUFACTURER_ID) {
      ret = TRUE;
    }
    else {
      ret = FALSE;
    }
  }
  else {
    ret = FALSE;
  }
  return ret;
}

/*---------------------------- General Functions -----------------------------*/

/*---------------------------- Global Functions ------------------------------*/
uint8_t tle9246x_init()
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  HAL_GPIO_WritePin(SPI_OUT_EN_GPIO_Port, SPI_OUT_EN_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pins : SPI_OUT_EN_Pin */
  GPIO_InitStruct.Pin = SPI_OUT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_OUT_EN_GPIO_Port, &GPIO_InitStruct);


  tle9246x_state = TLE9246x_INIT_STEP_0;
  
  HAL_GPIO_WritePin(FO1_RESN_GPIO_Port, FO1_RESN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FO1_CS_GPIO_Port, FO1_CS_Pin, GPIO_PIN_SET);
  
  /*Configure GPIO pins : FO1_RESN_Pin */
  GPIO_InitStruct.Pin = FO1_RESN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FO1_RESN_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FO1_CS_Pin */
  GPIO_InitStruct.Pin = FO1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FO1_CS_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FO1_FAULTN_Pin */
  GPIO_InitStruct.Pin = FO1_FAULTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FO1_FAULTN_GPIO_Port, &GPIO_InitStruct);
  
  tle9246x_state = TLE9246x_INIT_STEP_1;

  // The following variables are setted all 0xFFFF because the first condition for setting these variables from the main algorithm
  tle9246x_outputCurrent[TLE9246x_CHANNEL_BALANCE_UP] = 0xFFFF;
  tle9246x_outputCurrent[TLE9246x_CHANNEL_BALANCE_DOWN] = 0xFFFF;
  
  tle9246x_outputFrequency = 0xFFFF;
  tle9246x_ditherCurrent = 0xFFFF;
  tle9246x_ditherFrequency = 0xFFFF;
  
  crc_init();
    
  return TRUE;
}

uint8_t tle9246x_handler()
{
  uint8_t ret = FALSE;
  switch (tle9246x_state)
  {
    case TLE9246x_INIT_STEP_0:
      tle9246x_init();
      tle9246x_state = TLE9246x_INIT_STEP_1;
      break;
    case TLE9246x_INIT_STEP_1:
      tle9246x_disable_output();
      tle9246x_disable_reset();
      tle9246x_state = TLE9246x_INIT_STEP_2;
      break;
    case TLE9246x_INIT_STEP_2:
      if(check_ICVID() == TRUE) {
        tle9246x_state = TLE9246x_INIT_STEP_1;                      // Changed on 2022.08.22 because of reconfiguration
        
        initFinished = 0;
        ret = TRUE;
      }
      break;
    default:
      break;
  }
  return ret;
}

uint8_t tle9246x_get_diagnostic()
{
  uint8_t  ret = FALSE;
  uint8_t  i;
  uint16_t data16;
  uint32_t data32;
  
  timerDiagnostic += 100;
  
  if(timerDiagnostic >= 200) {
    timerDiagnostic = 0;                                                                                // Added on 2023.02.16

    get_data16(CENTRAL_REGISTER_BASE_ADDRESS + PIN_STAT, &data16);                          // printf("PIN_STAT: %04x\r\n", data16);
    get_data16(CENTRAL_REGISTER_BASE_ADDRESS + DIAG_ERR_CHGR0, &data16);                    // printf("DIAG_ERR_CHGR0: %04x\r\n", data16);

    ret = get_data16(CENTRAL_REGISTER_BASE_ADDRESS + DIAG_ERR_CHGR1, &data16);              // printf("DIAG_ERR_CHGR1: %04x\r\n", data16);
    if(flagOutput.balanceUp == ON) {
      if((data16 & 0x001F) == 0)  {}//tle9246x_outputError_local[TLE9246x_CHANNEL_RESERVED2] = 0;
      else                        tle9246x_outputError_local[TLE9246x_CHANNEL_BALANCE_UP]++;
    }

    if(flagOutput.balanceDown == ON) {
      if((data16 & 0x1F00) == 0)  {}//tle9246x_outputError_local[TLE9246x_CHANNEL_TOPLINK_UNLOAD] = 0;                // The toplink is back to 1F on 2023.10.19
      else                        tle9246x_outputError_local[TLE9246x_CHANNEL_BALANCE_DOWN]++;
    }
  
    for(i = 0; i < TLE9246x_CHANNELS; i++) {
      if(tle9246x_outputError_local[i] >= 5) {
        tle9246x_outputError_local[i] = 5;
        tle9246x_outputError[i] = TRUE;
      }
      else {
        tle9246x_outputError[i] = FALSE;
      }
    }
    
    ret = get_data16(CENTRAL_REGISTER_BASE_ADDRESS + DIAG_WARN_CHGR0, &data16);             // printf("Selected IC = %d, DIAG_WARN_CHGR0: %04x\r\n", selectedIC, data16);
    ret = get_data16(CENTRAL_REGISTER_BASE_ADDRESS + DIAG_WARN_CHGR1, &data16);             // printf("Selected IC = %d, DIAG_WARN_CHGR1: %04x\r\n", selectedIC, data16);
    ret = get_data22(CENTRAL_REGISTER_BASE_ADDRESS + FB_STAT, &data32);                     // printf("FB_STAT: %08x\r\n", data32);
    ret = get_data16(CENTRAL_REGISTER_BASE_ADDRESS + GLOBAL_DIAG0, &data16);                // printf("GLOBAL_DIAG0: %04x\r\n", data16);
    ret = get_data16(CENTRAL_REGISTER_BASE_ADDRESS + GLOBAL_DIAG1, &data16);                // printf("GLOBAL_DIAG1: %04x\r\n", data16);
    ret = get_data22(channelBaseAddress[0] + FB_PERIOD_MIN_MAX, &data32);                   // printf("FB_PERIOD_MIN_MAX: %08x\r\n", data32);
  }
  return ret;
}

uint8_t tle9246x_set_mode(uint8_t isConfigMode)
{
  uint16_t data;
  uint8_t ret = FALSE;
  if(isConfigMode == TRUE) {
        
    get_data16(CENTRAL_REGISTER_BASE_ADDRESS + CLK_DIV, &data);                             // printf("CLK_DIV: %04x\r\n", data);
    
    // CH_CTRL configuration 
    // Config mode is enabled & output channels are enabled
    channelEnableConfig = 0x6000;
    ret = set_data16(CENTRAL_REGISTER_BASE_ADDRESS + CH_CTRL, channelEnableConfig);
    ret = get_data16(CENTRAL_REGISTER_BASE_ADDRESS + CH_CTRL, &data);                       // printf("CH_CTRL: %04x\r\n", data);
    
    if(data == 0x6000) {
      ret = TRUE;
    }
    else {
      ret = FALSE;
    }
  }
  else {   
    // CH_CTRL configuration 
    // Mission mode is enabled & output channels are enabled
    channelEnableConfig = 0xE000;
    ret = set_data16(CENTRAL_REGISTER_BASE_ADDRESS + CH_CTRL, channelEnableConfig);
  }
  return ret;
}

uint8_t tle9246x_global_config()
{
  uint8_t  ret;
  uint16_t data16;
  // GLOBAL_CONFIG configuration, default configuration is 0x4005                               // VIO_SEL = 5.0V, CRC_EN, CLK_WD_EN
  data16 = 0x0004;                                                                              // VIO_SEL = 3.3V, CRC_EN
  ret = set_data16(CENTRAL_REGISTER_BASE_ADDRESS + GLOBAL_CONFIG, data16);

  get_data16(CENTRAL_REGISTER_BASE_ADDRESS + GLOBAL_CONFIG, &data16);               // The glabol configuration is correctly configured, then it should be contiuned
  if(data16 != 0x0004) {
    return FALSE;
  }
  ret = TRUE;
  
  return ret;
}

uint8_t tle9246x_clear_errors(uint8_t clearFlags)
{
  uint8_t  ret;
  uint16_t data16;
  
  data16 = 0x0000;
  
  ret = set_data16(CENTRAL_REGISTER_BASE_ADDRESS + FAULT_MASK0, data16);
  ret = set_data16(CENTRAL_REGISTER_BASE_ADDRESS + FAULT_MASK1, data16);
  ret = set_data16(CENTRAL_REGISTER_BASE_ADDRESS + FAULT_MASK2, data16);

  ret = set_data16(CENTRAL_REGISTER_BASE_ADDRESS + GLOBAL_DIAG0, data16);
  ret = set_data16(CENTRAL_REGISTER_BASE_ADDRESS + GLOBAL_DIAG1, data16);
  ret = set_data16(CENTRAL_REGISTER_BASE_ADDRESS + DIAG_ERR_CHGR0, data16);
  ret = set_data16(CENTRAL_REGISTER_BASE_ADDRESS + DIAG_ERR_CHGR1, data16);
  ret = set_data16(CENTRAL_REGISTER_BASE_ADDRESS + DIAG_WARN_CHGR0, data16);
  ret = set_data16(CENTRAL_REGISTER_BASE_ADDRESS + DIAG_WARN_CHGR1, data16);
    
  if(clearFlags == TRUE)
  {
    tle9246x_outputError_local[TLE9246x_CHANNEL_BALANCE_UP] = 0;
    tle9246x_outputError_local[TLE9246x_CHANNEL_BALANCE_DOWN] = 0;
  }
  
  return ret;
}

uint8_t tle9246x_set_channel_mode()
{
  uint8_t i;
  uint8_t ret;
  uint16_t data16;
  
  for(i = 0; i < TLE9246x_CHANNELS; i++) {
    
// Disabled 2023.02.16    data16 = 0x0803;                                                            // Fixed open load threshold is 125 mA
// Disabled 2023.02.16    data16 = 0x0403;                                                            // Fixed open load threshold is 62.5 mA
    data16 = 0x0013;                                                            // Fixed open load threshold is NOT used. Relative to Setpoint method is used (1/8 current setting)
    
    ret = set_data16(channelBaseAddress[i] + CH_CONFIG, data16);

    data16 = 0x0600;                                                            // 
    ret = set_data16(channelBaseAddress[i] + CTRL_REG, data16);
      
    // data16 = | 12bits=reserved | 4bits=CH_MODE
    // CH_MODE = 1 --> ICC Current Control
    data16 = 1;
    ret = set_data16(channelBaseAddress[i] + MODE, data16);
        
    // data16 = | 8bits=STEPS | 8bits=FLAT
    data16 = ((uint16_t)STEPS << 8) | (uint16_t)FLAT;
    ret = set_data16(channelBaseAddress[i] + DITHER_STEP, data16);
  }
  return ret;
}

uint8_t tle9246x_set_dither_frequency(uint16_t frequency)
{
  uint8_t  i;
  uint8_t  ret;
  uint16_t mant;
  uint16_t data16;
    
  // data16 = | 1bits=DITHER_SETPOINT_SYNC_EN | 1bit=DITHER_PWM_SYNC_EN| 4bits=EXP | 10bits=MANT |
  // Tref_clk = MANT * (2 ^ EXP) / Fsys
  // Tflat = <FLAT> * Tref_clk;
  // Tdither = [4*<STEPS> + 2 * <FLAT>] * Tref_clk;
  // Tdither = [4*<STEPS>] * Tref_clk = [4*<STEPS>] * MANT * (2 ^ EXP) / Fsys;                                // FLAT is 0
  // EXP = 4
  // (Fsys / (frequency * [4*<STEPS>] * 16)) =  MANT
  if(frequency != 0) {
    mant = (uint16_t)(FSYS / ((float)frequency * 4 * (float)STEPS * POW2EXP));
    if(mant >= 1023) {
      mant = 1023;                                                                                              // if mant is 1023, the minimum frequency is 1.8 Hz
    }
    else if(mant < 1) {
      mant = 1;                                                                                                 // if mant is 1, the maximum frequency is 1800 Hz
    }
  }
  else {
    mant = 0;                                                                                                   // if mant is 0, the dither is not used
  }
  data16 = 0xC000 | (EXP << 10) | mant;
  
  // printf("Dither frequency, DITHER_CLK_DIV: %04x\r\n", data16);
  
  for(i = 0; i < TLE9246x_CHANNELS; i++) {
    ret = set_data16(channelBaseAddress[i] + DITHER_CLK_DIV, data16);
  }
  return ret;
}

uint8_t tle9246x_set_dither_current(uint16_t current)
{
  uint8_t  i;
  uint8_t  ret;
  uint16_t stepSize;
  uint16_t data16;
  
  // data16 = | 2bits=FAST_MEAS | 1bit=DEEP_DITHER | 1bit=reserved | 12bits=STEP_SIZE
  // Idither = <STEPS> * <STEP_SIZE> * 2A / 32767
  // <STEP_SIZE> = Idither * 32767 / (2000 * <STEPS>)
  if(current != 0) {
    stepSize = (uint16_t)(((float)current * 32767.0 / (2000.0 * (float)STEPS)) + 0.5);                          // +0.5 is needed to improve accuracy, changed on 2022.08.22
    //stepSize = 50 * 32767 / (2000 * 243) + 0.5 = 3
    //stepSize = 0 * 32767 / (2000 * 243) + 0.5 = 0
    if(stepSize > 34) {
      stepSize = 34;
    }                                                                                                           // it is updated on 2022.08.22 because the maximum dither current is now 500mA
    else if(stepSize < 1) {
      stepSize = 1;                                                                                             // Minimum dither current is now 14.83mA
    }
  }
  else {
    stepSize = 0;                                                                                               // Dither current is 0
  }
  
  data16 = 0x0000 | stepSize;                                                                                   // Deep dither is disabled
  
  // printf("Dither current, DITHER_CTRL: %04x\r\n", data16);
  for(i = 0; i < TLE9246x_CHANNELS; i++) {
    ret = set_data16(channelBaseAddress[i] + DITHER_CTRL, data16);
  }
  return ret;
}

uint8_t tle9246x_set_frequency(uint16_t frequency)
{
  uint8_t  i;
  uint8_t  ret;
  int16_t  periodMant;
  uint16_t data16;
  
  // data16 = | 4bits=PWM_CTRL_PARAM (ki) | 1bit=LOW_FREQ_RANGE_EN | 3bits=PERIOD_EXP | 8bits=PERIOD_MANT |
  // Tpwm = PERIOD_MANT * (2 ^ PERIOD_EXP) / Fsys
  // 1/Fpwm = PERIOD_MANT * (2 ^ PERIOD_EXP) / Fsys
  // Fsys = PERIOD_MANT * (2 ^ PERIOD_EXP) * Fpwm
  // PERIOD_MANT = Fsys / (Fpwm * (2 ^ PERIOD_EXP))
  // PERIOD_MANT = 28,000,000 / (128 * 1000 * 8) = 
  if(frequency != 0) {
    periodMant = (uint16_t)(FSYS / ((float)frequency * POW2PERIOD_EXP * LOW_FREQ_RANGE_EN));
    if(periodMant >= 255) {                                                     // Minimum frequency is 107 Hz
      periodMant = 255;
    }
    else if(periodMant < 7) {                                                   // Maximum frequency is 3900 Hz
      periodMant = 7;
    }
    /*
    else if(periodMant < 1) {                                                   // Maximum frequency is 27000 Hz
      periodMant = 1;
    }
    */
  }
  else {
    periodMant = 255;                                                           // Minimum frequency is 107 Hz
  }

  data16 = 0x6800 | (PERIOD_EXP << 8) | periodMant;
    
  // printf("Frequency, PERIOD: %04x\r\n", data16);
  
  for(i = 0; i < TLE9246x_CHANNELS; i++) {
    /* Disabled on 2025.10.23
    if((selectedIC == TLE9246x_SECOND_IC) && (i == 3)) {
      data16 = 0x0000;                                                          // unload valve
    }
    */
    ret = set_data16(channelBaseAddress[i] + PERIOD, data16);
  }
  return ret;
}

uint8_t tle9246x_set_current(uint8_t channel, uint16_t outCurrent)
{
  uint8_t  ret;
  uint16_t target;
  uint16_t data16;
  
  if(channel >= TLE9246x_CHANNELS) {
    return FALSE;
  }
  
  tle9246x_outputCurrent[channel] = outCurrent;
  
  if(outCurrent != 0) {
    // data16 = | 1bit=AUTO_LIMIT_DIS | 15bits=TARGET;
    // Equation: Iset = 2A * data16 / (2^15 - 1);
    target = (uint16_t)((float)32767.0 * (float)outCurrent / 4000.0 );
    if(target > 22117) {                                                                        // The maximum current is 2700 mA
      target = 22117;
    }
    else if(target < 1) {                                                                       // The minimum current is 0.122 mA
      target = 1;
    }
  }
  else {
    target = 1;
  }
    
  data16 = 0x0000 | target;                                                                     // AutoLimit feature is disabled
    
//  printf("Current, SETPOINT: %04x               channel:%d \r\n", data16, channel);

  ret = set_data16(channelBaseAddress[channel] + SETPOINT, data16);
  /*
  if(target > 50) {
    if(channel == TLE9246x_CHANNEL_BALANCE_UP)          channelEnableConfig |= 0x2000;          // 0010
    else if(channel == TLE9246x_CHANNEL_BALANCE_DOWN)   channelEnableConfig |= 0x4000;          // 0100
    
    set_data16(CENTRAL_REGISTER_BASE_ADDRESS + CH_CTRL, channelEnableConfig);
  }
  else {
    if(channel == TLE9246x_CHANNEL_BALANCE_UP)          channelEnableConfig &= 0xDFFF;          // 1101
    else if(channel == TLE9246x_CHANNEL_BALANCE_DOWN)   channelEnableConfig &= 0xBFFF;          // 1011
    
    set_data16(CENTRAL_REGISTER_BASE_ADDRESS + CH_CTRL, channelEnableConfig);
  }
  */
  
  return ret;
}

uint8_t tle9246x_set_settings(uint16_t* data)
{
  uint8_t ret = FALSE;
    
  if(initFinished == 0) {
    if(tle9246x_global_config() == TRUE) {
      initFinished = 1;
    }
  }
  else if(initFinished == 1) {
    tle9246x_clear_errors(TRUE);
    tle9246x_set_channel_mode();
    tle9246x_set_dither_frequency(data[0]);
    tle9246x_set_dither_current(data[1]);
    tle9246x_set_frequency(data[2]);
    tle9246x_set_current(TLE9246x_CHANNEL_BALANCE_UP, 0);
    tle9246x_set_current(TLE9246x_CHANNEL_BALANCE_DOWN, 0);
    
    initFinished = 2;
  }
  else if(initFinished == 2)
  {
    if(tle9246x_set_mode(TRUE) == TRUE)
    {
      initFinished = 3;
    }
  }
  else {
    ret = tle9246x_set_mode(FALSE);
  }
  return ret;
}
/*---------------------------- Global Functions ------------------------------*/
