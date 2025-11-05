
/*
 *
 * File Name    : sensors.h
 * Program      : sensors are calculated in here
 * Author       : Enkhbat
 * Company      : Kiwon Electronics
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSORS_H
#define __SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "adc.h"
  
/* Defines -------------------------------------------------------------------*/

/* Defines Analog inputs -----------------------------------------------------*/

/* Defines Digital inputs ----------------------------------------------------*/
typedef union {
  uint32_t data;
  struct {    
    uint32_t strokeSensor               : 1;
    uint32_t rollingSensor              : 1;
    uint32_t hitchPosition              : 1;
    uint32_t heightDial                 : 1;
    uint32_t sensitiveDial              : 1;
    uint32_t balanceDial                : 1;
    uint32_t internalButtons            : 1;
    uint32_t externalButtons            : 1;

    uint32_t balanceUpValve             : 1;
    uint32_t balanceDownValve           : 1;
    
    uint32_t res                        : 22;
  };
} flagError_t;

extern flagError_t flagError;

void analog_sensors();
void digital_sensors();

uint16_t get_hitch_dial();
uint16_t get_balance_sensitive_dial();
uint16_t get_balance_configure_dial();
uint16_t get_direction();
uint16_t get_hitch_position_sensor();
uint16_t get_stroke_sensor();
uint16_t get_balance_sensor();
uint16_t get_battery_voltage();

float get_stroke_sensor_angle();
float get_balance_dial_angle();

float get_roll_angle();
float get_pitch_angle();
float get_pitch_angle_speed();
float get_roll_angle_speed();
float get_yaw_angle_speed();


#ifdef __cplusplus
}
#endif

#endif /* __SENSORS_H */
