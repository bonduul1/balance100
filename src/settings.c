#include <stdlib.h>
#include "can.h"
#include "input.h"
#include "output.h"
#include "main.h"
#include "watchdog.h"
#include "sensors.h"
#include "algorithm.h"
#include "settings.h"

/*--------------------------------------------------------- MAP number typedef and Variables Start ---------------------------------------------------*/
transmissionMapDataState_t              mapDataInfo[NUMBER_OF_MAP_NUMBER][NUMBER_OF_MAP_POINT];

/*--------------------------------------------------------- MAP number typedef and Variables End -----------------------------------------------------*/
uint16_t *nvBalanceUpCurrent;
uint16_t *nvBalanceUpAngle;
uint16_t *nvBalanceDownCurrent;
uint16_t *nvBalanceDownAngle;
  
uint16_t nvPwmFrequency;
uint16_t nvDitherFrequency;
uint16_t nvDitherCurrent;

uint16_t nvStrokeUpLimit;
uint16_t nvStrokeDownLimit;
uint16_t nvStrokeCenter;
uint16_t nvRollingCenter;
uint16_t nvBalanceSamplingTimeHigh;
uint16_t nvBalanceSamplingTimeMid;
uint16_t nvBalanceSamplingTimeLow;
uint16_t nvBalanceDeadbandHigh;
uint16_t nvBalanceDeadbandMid;
uint16_t nvBalanceDeadbandLow;

uint16_t nvDialMinAngle;
uint16_t nvDialMaxAngle;
uint16_t nvStrokeUpLimitAngle;
uint16_t nvStrokeDownLimitAngle;
uint16_t nvBalanceStallStepCurrent;
uint16_t nvBalanceStallStepTime;
uint16_t nvBalanceManualUpCurrent;
uint16_t nvBalanceManualDownCurrent;

nvData_t controlSettingsData[NUMBER_OF_CONTROL_PACKET] = {
  // DID,       min,       max,    default, value
  
  { 4001,       100,        3500,   1000,   &nvPwmFrequency                     },              // 1    --> Hz
  { 4002,       50,         350,    100,    &nvDitherFrequency                  },              // 2    --> Hz
  { 4003,       0,          400,    130,    &nvDitherCurrent                    },              // 3    --> mA

  { 4201,       710,        920,    900,    &nvStrokeUpLimit                    },              // 1    --> 10Bit ADC (data * 4.88 = mV)
  { 4202,       100,        307,    115,    &nvStrokeDownLimit                  },              // 2    --> 10Bit ADC (data * 4.88 = mV)
  { 4203,       452,        572,    512,    &nvStrokeCenter                     },              // 3    --> 10Bit ADC (data * 4.88 = mV)
  { 4204,       8700,       9300,   9000,   &nvRollingCenter                    },              // 4    --> degree, scale = 0.01, offset = -90,
  { 4205,       10,         1000,   150,    &nvBalanceSamplingTimeHigh          },              // 5    --> ms
  { 4206,       10,         1000,   100,    &nvBalanceSamplingTimeMid           },              // 6    --> ms
  { 4207,       10,         1000,   75,     &nvBalanceSamplingTimeLow           },              // 7    --> ms
  { 4208,       9000,       9300,   9050,   &nvBalanceDeadbandHigh              },              // 8    --> degree, scale = 0.01, offset = -90,
  { 4209,       9000,       9300,   9040,   &nvBalanceDeadbandMid               },              // 9    --> degree, scale = 0.01, offset = -90,
  { 4210,       9000,       9300,   9030,   &nvBalanceDeadbandLow               },              // 10   --> degree, scale = 0.01, offset = -90,
  { 4211,       6000,       12000,  7500,   &nvDialMinAngle                     },              // 11   --> degree, scale = 0.01, offset = -90,
  { 4212,       6000,       12000,  10500,  &nvDialMaxAngle                     },              // 12   --> degree, scale = 0.01, offset = -90,
  { 4213,       6000,       12000,  10500,  &nvStrokeUpLimitAngle               },              // 13   --> degree, scale = 0.01, offset = -90,
  { 4214,       6000,       12000,  7500,   &nvStrokeDownLimitAngle             },              // 14   --> degree, scale = 0.01, offset = -90,
  
  { 4215,       0,          500,    2,      &nvBalanceStallStepCurrent          },              // 15   --> Stall step current
  { 4216,       2,          500,    10,     &nvBalanceStallStepTime             },              // 16   --> Stall step time
  { 4217,       0,          1500,   1500,   &nvBalanceManualUpCurrent           },              // 17   --> Manual up current
  { 4218,       0,          1500,   1500,   &nvBalanceManualDownCurrent         },              // 18   --> Manual down current
};

/*--------------------------------------------------------- Local functions Start ----------------------------------------------*/

/*--------------------------------------------------------- Local functions End -------------------------------------------------*/

void init_pointers()
{
  nvBalanceUpCurrent = &(mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][0].y);
  nvBalanceUpAngle = &(mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][0].x);
  nvBalanceDownCurrent = &(mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][0].y);
  nvBalanceDownAngle = &(mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][0].x);
}

void set_default_setting(uint8_t isForce)
{
  uint16_t i, j;
  uint8_t tempWriteData[4];
  uint8_t tempReadData[4];
  uint8_t flagCheck = FALSE;
  uint16_t framUpdateCounter;
  
// Disabled 2025.11.03  fram_read(NV_CONTROL_START_ADDRESS, tempReadData);
  framUpdateCounter = ((uint16_t)tempReadData[1] << 8) + ((uint16_t)tempReadData[0]);

  if(framUpdateCounter == 0) {
    flagCheck = TRUE;
  }
  else if(framUpdateCounter == 0xFFFF) {
    flagCheck = TRUE;
    framUpdateCounter = 0;
  }
  
  if(isForce == TRUE) {
    flagCheck = TRUE;
  }
  
  if(flagCheck == FALSE) {      // No force write, data is saved
    return;
  }
    
  framUpdateCounter += 1;
  tempWriteData[0] = (uint8_t)(framUpdateCounter);
  tempWriteData[1] = (uint8_t)(framUpdateCounter >> 8);
  tempWriteData[2] = 0;
  tempWriteData[3] = 0;
// Disabled 2025.11.03  fram_write(NV_CONTROL_START_ADDRESS, tempWriteData);
  
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][0].x = 100;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][1].x = 200;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][2].x = 300;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][3].x = 400;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][4].x = 500;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][5].x = 600;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][6].x = 700;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][7].x = 1000;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][0].y = 800;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][1].y = 900;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][2].y = 1000;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][3].y = 1100;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][4].y = 1200;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][5].y = 1300;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][6].y = 1400;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][7].y = 1500;
  
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][0].x = 100;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][1].x = 200;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][2].x = 300;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][3].x = 400;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][4].x = 500;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][5].x = 600;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][6].x = 700;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][7].x = 1000;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][0].y = 800;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][1].y = 900;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][2].y = 1000;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][3].y = 1100;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][4].y = 1200;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][5].y = 1300;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][6].y = 1400;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][7].y = 1500;
    
  watchdog_trigger();
  
  for( i = 0; i < NUMBER_OF_MAP_NUMBER; i++)
  {
    for( j = 0; j < NUMBER_OF_MAP_POINT; j++)
    {
      tempWriteData[0] = mapDataInfo[i][j].x;
      tempWriteData[1] = mapDataInfo[i][j].x >> 8;
      tempWriteData[2] = mapDataInfo[i][j].y;
      tempWriteData[3] = mapDataInfo[i][j].y >> 8;
// Disabled on 2025.11.03      fram_write(NV_MAP_POINT_START_ADDRESS + ((i * NUMBER_OF_MAP_POINT + j) * DIAGNOSTIC_DATA_SIZE), tempWriteData);
    }
    if((i % 10) == 0) {
      watchdog_trigger();
    }
  }
  
  for(i = 0; i < NUMBER_OF_CONTROL_PACKET; i++)
  {
    // The default setting is updated
    check_control_data(TRUE, i, controlSettingsData[i].defaultValue);
    if((i % 10) == 0) {
      watchdog_trigger();
    }
  }
  
  init_diagnostic_variables();
}

void init_diagnostic_variables()
{
  uint16_t i;
  /*
  uint16_t i, j;
  uint8_t tempReadData[4];
 
  set_default_setting(FALSE);
    
  // Updating map data information from FRAM
  for( i = 0; i < NUMBER_OF_MAP_NUMBER; i++)
  {
    for( j = 0; j < NUMBER_OF_MAP_POINT; j++)
    {
// Disabled on 2025.11.03      fram_read(NV_MAP_POINT_START_ADDRESS + ((i * NUMBER_OF_MAP_POINT + j) * DIAGNOSTIC_DATA_SIZE), tempReadData);
      mapDataInfo[i][j].x = (tempReadData[1] << 8) + tempReadData[0];
      mapDataInfo[i][j].y = (tempReadData[3] << 8) + tempReadData[2];
    }
    if((i % 10) == 0) {
      watchdog_trigger();
    }
  }
  
  for(i = 0; i < NUMBER_OF_CONTROL_PACKET; i++)
  {
// Disabled on 2025.11.03    fram_read(NV_CONTROL_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempReadData);
    
    *(controlSettingsData[i].value) = (tempReadData[1] << 8) + tempReadData[0];

    check_control_data(FALSE, i, *(controlSettingsData[i].value));
    if((i % 10) == 0) {
      watchdog_trigger();
    }
  }
  */

  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][0].x = 100;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][1].x = 200;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][2].x = 300;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][3].x = 400;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][4].x = 500;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][5].x = 600;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][6].x = 700;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][7].x = 1000;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][0].y = 800;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][1].y = 900;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][2].y = 1000;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][3].y = 1100;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][4].y = 1200;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][5].y = 1300;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][6].y = 1400;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][7].y = 1500;
  
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][0].x = 100;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][1].x = 200;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][2].x = 300;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][3].x = 400;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][4].x = 500;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][5].x = 600;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][6].x = 700;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][7].x = 1000;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][0].y = 800;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][1].y = 900;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][2].y = 1000;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][3].y = 1100;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][4].y = 1200;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][5].y = 1300;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][6].y = 1400;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][7].y = 1500;
    
  for(i = 0; i < NUMBER_OF_CONTROL_PACKET; i++)
  {   
    *(controlSettingsData[i].value) = controlSettingsData[i].defaultValue;
  }
  
  init_pointers();
 
  watchdog_trigger();
}

uint8_t save_nvStrokeUpLimit(uint8_t _isSave, uint16_t _nvStrokeUpLimit)
{
  return check_control_data(_isSave, NV_STROKE_UP_LIMIT, _nvStrokeUpLimit);
}

uint8_t save_nvStrokeDownLimit(uint8_t _isSave, uint16_t _nvStrokeDownLimit)
{
  return check_control_data(_isSave, NV_STROKE_DOWN_LIMIT, _nvStrokeDownLimit);
}

uint8_t save_nvStrokeCenter(uint8_t _isSave, uint16_t _nvStroke)
{
  return check_control_data(_isSave, NV_STROKE_CENTER, _nvStroke);
}

uint8_t save_nvBalanceCenter(uint8_t _isSave, uint16_t _nvBalance)
{
  return check_control_data(_isSave, NV_ROLLING_CENTER, _nvBalance);
}

uint16_t get_control_minimum_data(uint8_t index)
{
  if(index >= NUMBER_OF_CONTROL_PACKET)
    return 0xFFFF;
  return (controlSettingsData[index].minValue);
}

uint16_t get_control_maximum_data(uint8_t index)
{
  if(index >= NUMBER_OF_CONTROL_PACKET)
    return 0xFFFF;
  return (controlSettingsData[index].maxValue);
}

uint16_t get_control_default_data(uint8_t index)
{
  if(index >= NUMBER_OF_CONTROL_PACKET)
    return 0xFFFF;
  return (controlSettingsData[index].defaultValue);
}

uint8_t get_control_data(uint16_t did, uint16_t *data)
{
  uint16_t index;
  
  for(index = 0; index < NUMBER_OF_CONTROL_PACKET; index++)
  {
    if(did == controlSettingsData[index].didValue)
    {
      data[0] = *(controlSettingsData[index].value);
      return TRUE;
    }
  }
  
  return FALSE;
}

uint8_t check_control_data(uint8_t _isSave, uint8_t index, uint16_t data)
{      
  uint8_t isSave = FALSE;
  uint8_t tempWriteData[4];
  uint8_t i = index;
  
  if(_isSave == TRUE) {                          // Which means the data should be updated
    if(*(controlSettingsData[i].value) == data) {
      // there no update is needed because the saved data is NOT changed
      return TRUE;
    }
    
    isSave = TRUE;
    
    if((data <= controlSettingsData[i].maxValue) && (data >= controlSettingsData[i].minValue)) {
      *(controlSettingsData[i].value) = data;
    }
    else {
      isSave = FALSE;
    }
  }
  else {
    if((*(controlSettingsData[i].value) > controlSettingsData[i].maxValue) || (*(controlSettingsData[i].value) < controlSettingsData[i].minValue)) {
      *(controlSettingsData[i].value) = controlSettingsData[i].defaultValue;
      isSave = TRUE;
    }
  }
  
  if(isSave == TRUE) {
    tempWriteData[0] = (uint8_t)(*(controlSettingsData[i].value));
    tempWriteData[1] = (uint8_t)(*(controlSettingsData[i].value) >> 8);
    tempWriteData[2] = 0;
    tempWriteData[3] = 0;
// Disabled on 2025.11.03    fram_write(NV_CONTROL_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
    return TRUE;
  }
  
  return FALSE;
}

uint8_t write_control_data(uint16_t did, uint16_t data)
{
  uint8_t tempWriteData[4];
  uint8_t index;
  
  for(index = 0; index < NUMBER_OF_CONTROL_PACKET; index++)
  {
    if(did == controlSettingsData[index].didValue) {
      if(*(controlSettingsData[index].value) == data) {
        return TRUE;
      }
      if((data <= controlSettingsData[index].maxValue) && (data >= controlSettingsData[index].minValue)) {
        *(controlSettingsData[index].value) = data;
        tempWriteData[0] = (uint8_t)(*(controlSettingsData[index].value));
        tempWriteData[1] = (uint8_t)(*(controlSettingsData[index].value) >> 8);
        tempWriteData[2] = 0;
        tempWriteData[3] = 0;
        
// Disabled on 2025.11.03        fram_write(NV_CONTROL_START_ADDRESS + (index * DIAGNOSTIC_DATA_SIZE), tempWriteData);
        return TRUE;
      }
      else {
        return FALSE;
      }
    }
  }
  return FALSE;
}

uint8_t write_control_datas(uint16_t did, uint8_t *pData, uint16_t len)
{
  uint16_t temp;
  uint8_t i;
  uint8_t start;
  uint8_t end;
  /*
  if(did == PUDS_SVC_PARAM_DI_M_BLOCK_OF_BALANCE_SETTING_DATA) {
    start = NV_STROKE_UP_LIMIT;
    end = NV_BALANCE_DEADBAND_LOW;
  }
  else {
    return FALSE;
  }
  */
  return FALSE;
  if(len != (end - start))
  {
    return FALSE;
  }
   
  for(i = 0; i < len; i++)
  {
    temp = (pData[i * 2] << 8) + pData[i * 2 + 1];
    check_control_data(TRUE, start + i, temp);
    if((i % 5) == 0)
    {
        watchdog_trigger();
    }
  }
  return TRUE;
}

uint8_t get_control_datas(uint16_t did, uint8_t *pData, uint16_t *len)
{
  uint16_t temp;
  uint8_t i;
  uint8_t start;
  uint8_t end;
  
  /*
  if(did == PUDS_SVC_PARAM_DI_M_BLOCK_OF_BALANCE_SETTING_DATA) {
    start = NV_STROKE_UP_LIMIT;
    end = NV_BALANCE_DEADBAND_LOW;
  }
  else {
    return FALSE;
  }
  */
  
  return FALSE;
  
  *len = ((end - start) * 2);
  
  for(i = start; i <= end; i++)
  {
    temp = *(controlSettingsData[i].value);
    pData[i * 2] = (uint8_t)(temp >> 8);
    pData[i * 2 + 1] = (uint8_t)(temp);
    if((i % 5) == 0)
    {
      watchdog_trigger();
    }
  }
  return TRUE;
}

uint8_t write_map_data_to_fram(uint8_t mapIndex, uint8_t mapDataIndex)
{
  uint8_t tempWriteData[4];
  
  tempWriteData[0] = mapDataInfo[mapIndex][mapDataIndex].x;
  tempWriteData[1] = mapDataInfo[mapIndex][mapDataIndex].x >> 8;
  tempWriteData[2] = mapDataInfo[mapIndex][mapDataIndex].y;
  tempWriteData[3] = mapDataInfo[mapIndex][mapDataIndex].y >> 8;
// Disabled on 2025.11.03  fram_write(NV_MAP_POINT_START_ADDRESS + ((mapIndex * NUMBER_OF_MAP_POINT + mapDataIndex) * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  return TRUE;
}
