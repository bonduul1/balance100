#include "sensors.h"
#include "input.h"
#include "output.h"
#include "main.h"
#include "can.h"

flagError_t flagError;

uint16_t hitchDial;
uint16_t balanceSensitiveDial;
uint16_t balanceConfigureDial;
uint16_t direction;
uint16_t hitchPositionSensor;
uint16_t strokeSensor;
uint16_t balanceSensor;
uint16_t batteryVoltage;

float rawRollSensorAngle;
float rollSensorAngle;
float rawPitchSensorAngle;
float pitchSensorAngle;
float rawRollSensorAngleSpeed;
float rollSensorAngleSpeed;
float rawPitchSensorAngleSpeed;
float pitchSensorAngleSpeed;
float rawYawSensorAngleSpeed;
float yawSensorAngleSpeed;

float strokeSensorAngle;
float balanceDialAngle;


float get_stroke_sensor_angle()         { return strokeSensorAngle;     }
float get_balance_dial_angle()          { return balanceDialAngle;      }

float get_roll_angle()                  { return rollSensorAngle;       }
float get_pitch_angle()                 { return pitchSensorAngle;      }
float get_pitch_angle_speed()           { return pitchSensorAngleSpeed; }
float get_roll_angle_speed()            { return rollSensorAngleSpeed;  }
float get_yaw_angle_speed()             { return yawSensorAngleSpeed;   }


uint16_t get_hitch_dial()               { return hitchDial;             }
uint16_t get_balance_sensitive_dial()   { return balanceSensitiveDial;  }
uint16_t get_balance_configure_dial()   { return balanceConfigureDial;  }
uint16_t get_direction()                { return direction;             }
uint16_t get_hitch_position_sensor()    { return hitchPositionSensor;   }
uint16_t get_stroke_sensor()            { return strokeSensor;          }
uint16_t get_balance_sensor()           { return balanceSensor;         }
uint16_t get_battery_voltage()          { return batteryVoltage;        }


void analog_sensors()
{
  uint16_t rawHitchDial;
  uint16_t rawBalanceSensitiveDial;
  uint16_t rawBalanceConfigureDial;
  uint16_t rawDirection;
  uint16_t rawHitchPositionSensor;
  uint16_t rawStrokeSensor;
  uint16_t rawBalanceSensor;
  uint16_t rawBatteryVoltage;

  updateLastAverageADC();
  
  // Analog sensors should be handled in here
  
  rawHitchDial = get_raw_hitch_dial();
  rawBalanceSensitiveDial = get_raw_balance_sensitive_dial();
  rawBalanceConfigureDial = get_raw_balance_configure_dial();
  rawDirection = get_raw_direction();
  rawHitchPositionSensor = get_raw_hitch_position_sensor();
  rawStrokeSensor = get_raw_stroke_sensor();
  rawBalanceSensor = get_raw_balance_sensor();
  rawBatteryVoltage = get_raw_battery();
  
  hitchDial = rawHitchDial;
  balanceSensitiveDial = rawBalanceSensitiveDial;
  balanceConfigureDial = rawBalanceConfigureDial;
  direction = rawDirection;
  hitchPositionSensor = rawHitchPositionSensor;
  strokeSensor = rawStrokeSensor;
  balanceSensor = rawBalanceSensor;
  batteryVoltage = rawBatteryVoltage;
  
  
  // This above part disabled due to rolling sensor data is received from CAN bus
  rawRollSensorAngle            = (canRxAngle.rollAngleLSB + canRxAngle.rollAngleMID * 256 + canRxAngle.rollAngleMSB * 256 * 256);
  rollSensorAngle               = (rawRollSensorAngle / 32768) - 250;
  
  rawPitchSensorAngle           = (canRxAngle.pitchAngleLSB + canRxAngle.pitchAngleMID * 256 + canRxAngle.pitchAngleMSB * 256 * 256);
  pitchSensorAngle              = (rawPitchSensorAngle / 32768) - 250;
  
  rawRollSensorAngleSpeed       = (canRxGyro.rollAngularRateLSB + canRxGyro.rollAngularRateMSB * 256);
  rollSensorAngleSpeed          = (rawRollSensorAngleSpeed / 128) - 250;
  
  rawPitchSensorAngleSpeed      = (canRxGyro.pitchAngularRateLSB + canRxGyro.pitchAngularRateMSB * 256);
  pitchSensorAngleSpeed         = (rawPitchSensorAngleSpeed / 128) - 250;
  
  rawYawSensorAngleSpeed        = (canRxGyro.yawAngularRateLSB + canRxGyro.yawAngularRateMSB * 256);
  yawSensorAngleSpeed           = (rawYawSensorAngleSpeed / 128) - 250;
  
}

void digital_sensors()
{
  read_digital_inputs();
}