#ifndef __SETTINGS_H__
#define __SETTINGS_H__

#include "stm32f1xx_hal.h"


/*----------------------------------------------------------------------------
 * Calculating memory
 * NV_CONTROL_START_ADDRESS = 4000 ==> 55 * 4
 * 64Kbit = 8KBytes
 * 8 * 1024 Bytes = 8192 Bytes =>
 * 32Bit data ==> 8192 Bytes / (read/write length = 4) = 2048 (32bit data)
 * 16Kbit = 2KBytes
 *
 *
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
#define NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS        0                       // 0 + (24 * 4 ) = 0 + 96 = 96
#define NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS        100                     // 100 + (48 * 4 ) = 100 + 192 = 292
#define NV_MAP_NUMBER_TRANSMISSION_TRAIN_ADDRESS        300                     // 300 + (32 * 4 ) = 300 + 128 = 428
#define NV_MAP_POINT_START_ADDRESS                      500                     // 500 + (100 * 8 * 4) = 3700                           // Maximum 100 map data
#define NV_CONTROL_START_ADDRESS                        4000                    // 4000 + (200 * 4) = 4800                              // Maximum 200 data
#define NV_START_ADDRESS_OF_UDS_STANDARD_DID            5000                    // 5000 + (200 * 4) = 5800                              // Maximum 200 data
#define NV_START_ADDRESS_OF_DTC_STATE_COUNTER           6000                    // 6000 + (200 * 4) = 6800                              // Maximum 200 data
#define NV_START_ADDRESS_OF_DTC_TIME                    7000                    // 7000 + (200 * 4) = 7800                              // Maximum 200 data


#define DIAGNOSTIC_DATA_SIZE                            4
/*----------------------------------------------------------------------------*/

/*--------------------------------------------------------- MAP number typedef and Variables Start ---------------------------------------------------*/
typedef union {
  uint32_t data;
  struct {
    uint16_t x;
    uint16_t y;
  } ;
} transmissionMapDataState_t;

#define NUMBER_OF_MAP_NUMBER                            2
#define NUMBER_OF_MAP_POINT                             8

#define MAP_NUMBER_BALANCE_UP                           1
#define MAP_NUMBER_BALANCE_DOWN                         2

extern transmissionMapDataState_t                       mapDataInfo[NUMBER_OF_MAP_NUMBER][NUMBER_OF_MAP_POINT];

/*--------------------------------------------------------- MAP number typedef and Variables End -----------------------------------------------------*/
extern uint16_t *nvBalanceUpCurrent;
extern uint16_t *nvBalanceUpAngle;
extern uint16_t *nvBalanceDownCurrent;
extern uint16_t *nvBalanceDownAngle;

/*--------------------------------------------------------- Control command typedef and Variables Start ----------------------------------------------*/
enum {
  NV_SHUTTLE_PWM_FREQUENCY = 0,
  NV_SHUTTLE_DITHER_FREQUENCY,
  NV_SHUTTLE_DITHER_CURRENT,

  NV_STROKE_UP_LIMIT,
  NV_STROKE_DOWN_LIMIT,
  NV_STROKE_CENTER,
  NV_ROLLING_CENTER,
  NV_BALANCE_SAMPLING_TIME_HIGH,
  NV_BALANCE_SAMPLING_TIME_MID,
  NV_BALANCE_SAMPLING_TIME_LOW,
  NV_BALANCE_DEADBAND_HIGH,
  NV_BALANCE_DEADBAND_MID,
  NV_BALANCE_DEADBAND_LOW,
  NV_BALANCE_VR_MIN_ANGLE,
  NV_BALANCE_VR_MAX_ANGLE,
  NV_STROKE_UP_LIMIT_ANGLE,
  NV_STROKE_DOWN_LIMIT_ANGLE,
  NV_BALANCE_STALL_STEP_CURRENT,
  NV_BALANCE_STALL_STEP_TIME,
  NV_BALANCE_MANUAL_UP_CURRENT,
  NV_BALANCE_MANUAL_DOWN_CURRENT,
  
  NUMBER_OF_CONTROL_PACKET
};

// NV Control data
  // Shuttle & drive control
extern uint16_t nvPwmFrequency;
extern uint16_t nvDitherFrequency;
extern uint16_t nvDitherCurrent;

extern uint16_t nvStrokeUpLimit;
extern uint16_t nvStrokeDownLimit;
extern uint16_t nvStrokeCenter;
extern uint16_t nvRollingCenter;
extern uint16_t nvBalanceSamplingTimeHigh;
extern uint16_t nvBalanceSamplingTimeMid;
extern uint16_t nvBalanceSamplingTimeLow;
extern uint16_t nvBalanceDeadbandHigh;
extern uint16_t nvBalanceDeadbandMid;
extern uint16_t nvBalanceDeadbandLow;

extern uint16_t nvDialMinAngle;
extern uint16_t nvDialMaxAngle;
extern uint16_t nvStrokeUpLimitAngle;
extern uint16_t nvStrokeDownLimitAngle;
extern uint16_t nvBalanceStallStepCurrent;
extern uint16_t nvBalanceStallStepTime;
extern uint16_t nvBalanceManualUpCurrent;
extern uint16_t nvBalanceManualDownCurrent;

typedef struct  {
   const uint16_t didValue;
   const uint16_t minValue;
   const uint16_t maxValue;
   const uint16_t defaultValue;
   uint16_t       *value;
} nvData_t;

/*--------------------------------------------------------- Control command typedef and Variables End -------------------------------------------------*/

/*----------------------------------------------------------------------------*/
void init_diagnostic_variables();

uint8_t save_nvStrokeUpLimit(uint8_t _isSave, uint16_t _nvStrokeUpLimit);
uint8_t save_nvStrokeDownLimit(uint8_t _isSave, uint16_t _nvStrokeDownLimit);
uint8_t save_nvStrokeCenter(uint8_t _isSave, uint16_t _nvStroke);
uint8_t save_nvBalanceCenter(uint8_t _isSave, uint16_t _nvBalance);

uint16_t get_control_minimum_data(uint8_t index);
uint16_t get_control_maximum_data(uint8_t index);
uint16_t get_control_default_data(uint8_t index);

uint8_t get_control_data(uint16_t did, uint16_t *data);
uint8_t get_control_datas(uint16_t did, uint8_t *pData, uint16_t *len);
uint8_t write_control_data(uint16_t did, uint16_t data);
uint8_t write_control_datas(uint16_t did, uint8_t *pData, uint16_t len);
uint8_t check_control_data(uint8_t _isSave, uint8_t index, uint16_t data);

uint8_t write_map_data_to_fram(uint8_t mapIndex, uint8_t mapDataIndex);
uint8_t write_transmission_map_number_to_fram(uint8_t mapNumberIndex);
uint8_t write_starter_map_number_to_fram(uint8_t mapNumberIndex);

void set_default_setting(uint8_t isForce);

#endif

