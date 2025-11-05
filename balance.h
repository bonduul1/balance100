/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BALANCE_H
#define __BALANCE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
  
/* Defines -------------------------------------------------------------------*/
#define VAC_UP_DEADBAND_ANGLE                   (float)0.5
#define VAC_DOWN_DEADBAND_ANGLE                 (float)0.5
#define VAC_UP_DOWN_DEADBAND_ANGLE              (float)0.5                      // 
  
#define VAC_BALANCE_MAX_CURRENT                 2500
  
#define VAC_BALANCE_SETTING_TIME                2000
#define VAC_BALANCE_SETTING_OVER_TIME           15000

#define VAC_BALANCE_OUTPUT_LIMIT_TIME           15000
  
#define VA_BALANCE_DELAY                        500    
#define VA_BALANCE_UP_DELAY                     100
#define VA_BALANCE_DOWN_DELAY                   100

#define VAB_STOP_MODE                           0
#define VAB_DOWN_MODE                           1
#define VAB_UP_MODE                             2

#define VAB_BALANCE_MANUAL_MODE                 0
#define VAB_BALANCE_FLAT_MODE                   1
#define VAB_BALANCE_SLOPE_MODE                  2
  
typedef union {
  uint32_t data;
  struct {
    uint32_t settingMode        : 1;
    uint32_t settingStroke      : 1;
    uint32_t settingRoll        : 1;
    uint32_t volumeMoved        : 1;
    uint32_t ptoStopped         : 1;
    uint32_t autoFlat           : 1;
    uint32_t autoSlope          : 1;
    uint32_t autoCenterStop     : 1;
    
    uint32_t balanceMode        : 8;
    
    uint32_t settingSequence    : 3;
    uint32_t autoCenterRunning  : 2;
    uint32_t isSteeringON       : 1;
    uint32_t backUpRun          : 1;
    uint32_t turnUpRun          : 1;
    
    uint32_t oneUpRun           : 1;
    uint32_t oneDownRun         : 1;
    uint32_t res1               : 6;
  };
} flagBalance_t;

extern flagBalance_t flagBalance;
extern float targetAngle;

void balance_setting(void);
void balance_init();
void balance_process(void);
//uint8_t unload_valve_control();

uint16_t get_balanceUpCurrent();
uint16_t get_balanceDownCurrent();

void clear_balanceUpStall();
void clear_balanceDownStall();
uint16_t get_balanceUpStall();
uint16_t get_balanceDownStall();

#ifdef __cplusplus
}
#endif

#endif /* __BALANCE_H */
