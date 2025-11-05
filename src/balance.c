#include "balance.h"
#include "can.h"
#include "input.h"
#include "output.h"
#include "sensors.h"
#include "main.h"
#include "algorithm.h"
#include "settings.h"

flagBalance_t flagBalance;

float rollAngle;
float rollAngleRaw;
float strokeAngle;
float targetAngle;
float slopeCenterAngle;
float deadBandAngle;

uint16_t rollingSamplingTime;

float strokeUpLimitAngle;
float strokeDownLimitAngle;

uint16_t timerFlat;
uint16_t timerSlope;

uint16_t timerBalanceSetting = 0;

uint16_t balanceUpCurrent = 0;
uint16_t balanceDownCurrent = 0;

uint16_t balanceStallUp;
uint16_t balanceStallDown;

float balance_vr_angle()
{
  const static float balanceSettingAngleZone = 1;                               // 1 degree
  static float balanceSettingAnglePrevious;
  float balanceSettingAngle = get_balance_dial_angle();                         // canRxDial.balanceSetting * 4;                       /* 250 * 4 = 1000 */
  
  if(((balanceSettingAnglePrevious + balanceSettingAngleZone) <= balanceSettingAngle) || 
     ((balanceSettingAnglePrevious - balanceSettingAngleZone) >= balanceSettingAngle))
  {
    balanceSettingAnglePrevious = balanceSettingAngle;
    flagBalance.volumeMoved = TRUE;
  }
  
  return balanceSettingAngle;
}

void calculate_output_current(uint8_t direction, float diffAngle)
{
  static uint16_t timerStallUp = 0;
  static uint16_t timerStallDown = 0;
  uint8_t i;
  uint16_t diff = (uint16_t)(diffAngle * 100);
  
  if(direction == VAB_UP_MODE)
  {
    for(i = 0; i < 8; i++)
    {
      if(diff < nvBalanceUpAngle[i]) {
        if(i == 0) {
          balanceUpCurrent = nvBalanceUpCurrent[0];
        }
        else {
          balanceUpCurrent = (uint16_t)(nvBalanceUpCurrent[i - 1] + 
            ((((float)nvBalanceUpCurrent[i] - (float)nvBalanceUpCurrent[i - 1]) / 
            ((float)nvBalanceUpAngle[i] - (float)nvBalanceUpAngle[i - 1])) *
            ((float)diff - (float)nvBalanceUpAngle[i - 1])));
        }
        break;
      }
    }
    if(i == 8)
    {
      balanceUpCurrent = nvBalanceUpCurrent[7];
    }
    
    balanceStallDown = 0;
    timerStallDown = 0;
    
    timerStallUp += 2;
    
    if(timerStallUp >= nvBalanceStallStepTime)
    {
      timerStallUp = 0;
      
      balanceStallUp += nvBalanceStallStepCurrent;
      if(balanceStallUp >= 500)
        balanceStallUp = 500;
    }
    balanceUpCurrent = balanceUpCurrent + balanceStallUp;
    
    if(balanceUpCurrent >= VAC_BALANCE_MAX_CURRENT)
    {
      balanceUpCurrent = VAC_BALANCE_MAX_CURRENT;
    }
  }
  else if(direction == VAB_DOWN_MODE)
  {
    for(i = 0; i < 8; i++)
    {
      if(diff < nvBalanceDownAngle[i]) {
        if(i == 0) {
          balanceDownCurrent = nvBalanceDownCurrent[0];
        }
        else {
          balanceDownCurrent = (uint16_t)(nvBalanceDownCurrent[i - 1] + 
            ((((float)nvBalanceDownCurrent[i] - (float)nvBalanceDownCurrent[i - 1]) / 
            ((float)nvBalanceDownAngle[i] - (float)nvBalanceDownAngle[i - 1])) *
            ((float)diff - (float)nvBalanceDownAngle[i - 1])));
        }
        break;
      }
    }
    if(i == 8)
    {
      balanceDownCurrent = nvBalanceDownCurrent[7];
    }

    balanceStallUp = 0;
    timerStallUp = 0;

    timerStallDown += 2;
    
    if(timerStallDown > nvBalanceStallStepTime)
    {
      timerStallDown = 0;
      
      balanceStallDown += nvBalanceStallStepCurrent;
      if(balanceStallDown >= 500)
        balanceStallDown = 500;
    }
    balanceDownCurrent = balanceDownCurrent + balanceStallDown;
    
    if(balanceDownCurrent >= VAC_BALANCE_MAX_CURRENT)
    {
      balanceDownCurrent = VAC_BALANCE_MAX_CURRENT;
    }
  }
  else
  {
    balanceUpCurrent = 0;
    balanceDownCurrent = 0;
    
    balanceStallDown = 0;
    balanceStallUp = 0;
    
    timerStallUp = 0;
    timerStallDown = 0;
  }
}

void balance_fuzzy(uint8_t init)
{
  static uint8_t balanceStatus = 0;
  static uint16_t timerOutUp = 0;
  static uint16_t timerOutDown = 0;
    
  float targetDiffAngle = 0;
  uint8_t balanceStatusPrevious, runMode;
    
  if(init == 0)
  {
    flagBalance.volumeMoved = FALSE;
    balanceStatus = 0;
    timerOutUp = 0;
    timerOutDown = 0;
  }
  balanceStatusPrevious = balanceStatus;
  
  if(strokeAngle + deadBandAngle < targetAngle) {
    balanceStatus = VAB_UP_MODE;
  }
  else if(strokeAngle - deadBandAngle > targetAngle) {
    balanceStatus = VAB_DOWN_MODE;
  }
  else {
    if(flagBalance.volumeMoved == TRUE)
    {
      if(strokeAngle < targetAngle)            balanceStatus = VAB_UP_MODE;  
      else if(strokeAngle > targetAngle)       balanceStatus = VAB_DOWN_MODE;  
    }
  }

  flagBalance.volumeMoved = FALSE;
  runMode = VAB_STOP_MODE;

  if(balanceStatusPrevious == VAB_DOWN_MODE) {
    if(balanceStatus == VAB_DOWN_MODE ) {
      if(strokeAngle <= targetAngle + VAC_DOWN_DEADBAND_ANGLE) {
        runMode = VAB_STOP_MODE;
        targetDiffAngle = 0;
      }
      else {
        runMode = VAB_DOWN_MODE;
        targetDiffAngle = -(targetAngle - strokeAngle - VAC_DOWN_DEADBAND_ANGLE);
      }
    }
  }
  else if(balanceStatusPrevious == VAB_UP_MODE) {
    if(balanceStatus == VAB_UP_MODE) {
      if(strokeAngle >= targetAngle - VAC_UP_DEADBAND_ANGLE) {
        runMode = VAB_STOP_MODE;
        targetDiffAngle = 0;
      }
      else {
        runMode = VAB_UP_MODE;
        targetDiffAngle = targetAngle - strokeAngle - VAC_UP_DEADBAND_ANGLE;
      }
    }
  }
  else {
    if(balanceStatus == VAB_DOWN_MODE) {
      runMode = VAB_DOWN_MODE;
      targetDiffAngle = -(targetAngle - strokeAngle - VAC_DOWN_DEADBAND_ANGLE);
    }
    else if(balanceStatus == VAB_UP_MODE) {
      runMode = VAB_UP_MODE;
      targetDiffAngle = targetAngle - strokeAngle - VAC_UP_DEADBAND_ANGLE;
    }
    else {
      runMode = VAB_STOP_MODE;
      targetDiffAngle = 0;
    }
  }
  
  if(runMode == VAB_DOWN_MODE)
  {
    timerOutUp = 0;
    balanceStatus = VAB_DOWN_MODE;
    
    if(flagTimer.tenMs == TRUE)
    {
      timerOutDown += 10;
    }
    
    if(timerOutDown >= VA_BALANCE_DOWN_DELAY)
    {
       timerOutDown = VA_BALANCE_DOWN_DELAY;
       flagOutput.balanceDown = ON;
       
       // calculate output current in here
       calculate_output_current(VAB_DOWN_MODE, targetDiffAngle);
    }
        
    if(strokeAngle < strokeDownLimitAngle) {
      flagOutput.balanceDown = OFF;
    }
    return;
  }
  else if(runMode == VAB_UP_MODE)
  {
    timerOutDown = 0;
    balanceStatus = VAB_UP_MODE;
        
    if(flagTimer.tenMs == TRUE)
    {
      timerOutUp += 10;
    }
    
    if(timerOutUp >= VA_BALANCE_UP_DELAY) {
      timerOutUp = VA_BALANCE_UP_DELAY;
      flagOutput.balanceUp = ON;
      
      // calculate output current in here
       calculate_output_current(VAB_UP_MODE, targetDiffAngle);
    }
    
    if(strokeAngle > strokeUpLimitAngle) {
      flagOutput.balanceUp = OFF;
    }
    return;
  }
  
  balanceStatus = VAB_STOP_MODE; 
  timerOutUp = 0;
  timerOutDown = 0;
  flagOutput.balanceUp = OFF;
  flagOutput.balanceDown = OFF;
  return;
}

void calculate_angle(uint8_t _isFlat)
{ 
  float calculatedAngle;
  strokeUpLimitAngle = ((float)nvStrokeUpLimitAngle - 9000) / 100;
  strokeDownLimitAngle = ((float)nvStrokeDownLimitAngle - 9000) / 100;
  
  if(_isFlat == TRUE)
    calculatedAngle = balance_vr_angle() + rollAngle;
  else
    calculatedAngle = balance_vr_angle() + rollAngle - slopeCenterAngle;
    
  if(calculatedAngle > strokeUpLimitAngle)              targetAngle = strokeUpLimitAngle;
  else if(calculatedAngle < strokeDownLimitAngle)       targetAngle = strokeDownLimitAngle;
  else                                                  targetAngle = calculatedAngle;
}

void auto_flat()
{
  if(flagBalance.autoFlat == FALSE)
  {
    if(timerFlat >= VA_BALANCE_DELAY)
    {
      flagBalance.autoFlat = TRUE;
      timerFlat = 0;
      balance_fuzzy(0);
    }
    return;
  }
  timerFlat = 0;
 
  calculate_angle(TRUE);
  balance_fuzzy(1);                                                             // Added on 2022.01.25

  return;
}

void auto_slope()
{
  if(flagBalance.autoSlope == FALSE)
  {
    if(timerSlope >= VA_BALANCE_DELAY)
    {
      flagBalance.autoSlope = TRUE;
      timerSlope = 0;
      balance_fuzzy(0);
    }
    return;
  }
  timerSlope = 0;
  
  calculate_angle(FALSE);
  balance_fuzzy(1);                                                             // Added on 2022.01.25

  return;
}

void auto_center()
{
  if(flagBalance.autoCenterStop == FALSE)
  {
    if((strokeAngle <= VAC_UP_DOWN_DEADBAND_ANGLE) && (strokeAngle >= -VAC_UP_DOWN_DEADBAND_ANGLE))
    {
      flagOutput.balanceUp = OFF;
      flagOutput.balanceDown = OFF;
    }
    else
    {
      if(strokeAngle < -VAC_UP_DOWN_DEADBAND_ANGLE)
      {
        if((flagBalance.autoCenterRunning == VAB_STOP_MODE) || (flagBalance.autoCenterRunning == VAB_UP_MODE))
        {
          flagBalance.autoCenterRunning = VAB_UP_MODE;
          flagOutput.balanceUp = ON;
          flagOutput.balanceDown = OFF;
          
          calculate_output_current(VAB_UP_MODE, -strokeAngle);
        }
      }
      else if(strokeAngle > VAC_UP_DOWN_DEADBAND_ANGLE)
      {
        if((flagBalance.autoCenterRunning == VAB_STOP_MODE) || (flagBalance.autoCenterRunning == VAB_DOWN_MODE))
        {
          flagBalance.autoCenterRunning = VAB_DOWN_MODE;

          flagOutput.balanceUp = OFF;
          flagOutput.balanceDown = ON;
          
          calculate_output_current(VAB_DOWN_MODE, strokeAngle);
        }
      }
    }
  }
  else
  {
    flagBalance.autoCenterRunning = VAB_STOP_MODE;
  }
}

void balance_output_limit()
{
  static uint16_t timerOutUpLimit = 0;
  static uint16_t timerOutDownLimit = 0;
 
  if(flagOutput.balanceUp == ON)
  {
    if(timerOutUpLimit >= VAC_BALANCE_OUTPUT_LIMIT_TIME)
    {
      flagOutput.balanceUp = OFF;
      timerOutUpLimit = VAC_BALANCE_OUTPUT_LIMIT_TIME + 300;
    }
    else
    {
      if(flagTimer.hundredMs == TRUE)
      {
        timerOutUpLimit += 100;
      }
    }
    
    if(timerOutDownLimit >= VAC_BALANCE_OUTPUT_LIMIT_TIME)
    {
      if(flagTimer.hundredMs == TRUE)
      {
        timerOutDownLimit -= 100;
      }
    }
    else
    {
      timerOutDownLimit = 0;
    }
    
    return;
  }
  
  if(flagOutput.balanceDown == ON)
  {
    if(timerOutDownLimit >= VAC_BALANCE_OUTPUT_LIMIT_TIME)
    {
      flagOutput.balanceDown = OFF;
      timerOutDownLimit = VAC_BALANCE_OUTPUT_LIMIT_TIME + 300;
    }
    else
    {
      if(flagTimer.hundredMs == TRUE)
      {
        timerOutDownLimit += 100;
      }
    }
    
    if(timerOutUpLimit >= VAC_BALANCE_OUTPUT_LIMIT_TIME)
    {
      if(flagTimer.hundredMs == TRUE)
      {
        timerOutUpLimit -= 100;
      }
    }
    else
    {
      timerOutUpLimit = 0;
    }
    return;
  }
  
  if(timerOutUpLimit >= VAC_BALANCE_OUTPUT_LIMIT_TIME)
  {
    if(flagTimer.hundredMs == TRUE)
    {
      timerOutUpLimit -= 100;
    }
  }
  else
  {
    timerOutUpLimit = 0;
  }
  
  if(timerOutDownLimit >= VAC_BALANCE_OUTPUT_LIMIT_TIME)
  {
    if(flagTimer.hundredMs == TRUE)
    {
      timerOutDownLimit -= 100;
    }
  }
  else
  {
    timerOutDownLimit = 0;
  }
}

void balance_process()
{
  static uint8_t flagSlopeMode = 0;
  static uint16_t timerChangeFlagSlopeMode = 0;
  static uint16_t timerRollingChange = 0;
  static uint16_t timerLedBlink = 0;

  uint8_t isManualButtonPressed = FALSE;
  uint16_t liftStop = 0;
  
  timerSlope += 2;   
  timerFlat += 2;
  timerRollingChange += 2;
  timerLedBlink += 2;
  
  if((flagInputStatus.modeButton == ON) && (flagInputStatusPrevious.modeButton == OFF))
  {    
    if(flagBalance.balanceMode == VAB_BALANCE_MANUAL_MODE)      flagBalance.balanceMode = VAB_BALANCE_FLAT_MODE;
    else if(flagBalance.balanceMode == VAB_BALANCE_FLAT_MODE)   flagBalance.balanceMode = VAB_BALANCE_SLOPE_MODE;
    else                                                        flagBalance.balanceMode = VAB_BALANCE_MANUAL_MODE;
  }
  
  
  deadBandAngle = ((float)nvBalanceDeadbandMid - 9000) / 100;                   // Should be calculated based on DIAL
  rollingSamplingTime = nvBalanceSamplingTimeMid; 
  
  strokeAngle = get_stroke_sensor_angle();

  if(timerRollingChange > rollingSamplingTime)
  {
    timerRollingChange = 0;
    rollAngleRaw = (rollAngleRaw/2) + (get_roll_angle()/2);
    rollAngle = rollAngleRaw - (((float)nvRollingCenter - 9000) / 100);
  }
  
  flagBalance.settingMode = FALSE;              // for test
  if(flagBalance.settingMode == TRUE)
  {
    flagBalance.balanceMode = VAB_BALANCE_MANUAL_MODE;
    slopeCenterAngle = rollAngle;
    
    if(flagBalance.settingSequence == 0)
    {
      timerLedBlink = 0;
    }
    else if(flagBalance.settingSequence == 1)
    {
      flagOutput.ledManual = ON;
      flagOutput.ledFlat = ON;
      flagOutput.ledSlope = ON;
      timerLedBlink = 0;
    }
    else if(flagBalance.settingSequence == 2)
    {
      if(timerLedBlink < 500)
      {
        flagOutput.ledManual = ON;
        flagOutput.ledSlope = ON;
      }
      else if(timerLedBlink > 1000)
      {
        timerLedBlink = 0;
      }
    }
    else if(flagBalance.settingSequence == 3)
    {
      if(timerLedBlink < 500)
      {
        flagOutput.ledFlat = ON;
      }
      else if(timerLedBlink > 1000)
      {
        timerLedBlink = 0;
      }
    }
  }
  else
  {
    // check sensor error
    
    if(flagBalance.balanceMode == VAB_BALANCE_MANUAL_MODE)
    {
      flagOutput.ledManual = ON;
      slopeCenterAngle = rollAngle;
    }
    else if(flagBalance.balanceMode == VAB_BALANCE_FLAT_MODE)
    {
      flagOutput.ledFlat = ON;
      slopeCenterAngle = rollAngle;  
    }
    else if(flagBalance.balanceMode == VAB_BALANCE_SLOPE_MODE)
    {
      flagOutput.ledSlope = ON;
    }
  }
  
  // This content is added on 2022.06.15 - Start
  if(flagBalance.balanceMode == VAB_BALANCE_SLOPE_MODE)
  {
    if(flagSlopeMode != 2)
    {
      //if((flag.steeringRight == TRUE) || (flag.steeringLeft == TRUE))
      if(flagBalance.isSteeringON == TRUE)
      {
        flagSlopeMode = 1;
      }
      else
      {
        // Steering OFF
        if(flagSlopeMode == 1)
        {
          flagSlopeMode = 2;
          timerChangeFlagSlopeMode = 0;
        }
        else
        {
          if((flagBalance.backUpRun == TRUE) || (flagBalance.turnUpRun == TRUE) || (flagBalance.oneUpRun == TRUE))
          {
            flagSlopeMode = 3;
          }
          else if(flagBalance.oneDownRun == TRUE)
          {
            if(flagSlopeMode == 3)
            {
              flagSlopeMode = 2;
              timerChangeFlagSlopeMode = 0;
            }
          }
        }
      }
    }
    
    if(flagSlopeMode == 2)
    {
      if(flagTimer.tenMs == TRUE)
      {
        timerChangeFlagSlopeMode += 10;
      }
      if(timerChangeFlagSlopeMode >= 1000)
      {
        slopeCenterAngle = rollAngle;
        timerChangeFlagSlopeMode = 0;
      }
    }
  }

  if((flagBalance.balanceMode == VAB_BALANCE_FLAT_MODE) || (flagBalance.balanceMode == VAB_BALANCE_SLOPE_MODE))
  {
//    liftStop = (uint16_t)((float)nvPtoLower3Pposition + 
// Add next time                         ((float)nvPtoHigher3Pposition - (float)nvPtoLower3Pposition) * (float)canRxDial.ptoStopPosition / 250.0);
    
    if(get_hitch_position_sensor() > liftStop)
    {
      // pto stopped but if this part is implemented first time, the output never go out
      flagBalance.ptoStopped = TRUE;
      slopeCenterAngle = rollAngle;
    }
    else
    {
      // Auto mode 
      flagBalance.autoCenterStop = FALSE;
      flagBalance.ptoStopped = FALSE;
      flagBalance.autoCenterRunning = VAB_STOP_MODE;
    }
  }
  else
  {
    // Manual mode lock the auto pall function
    flagBalance.autoCenterStop = TRUE;
    flagBalance.autoCenterRunning = VAB_STOP_MODE;
  }

  if((flagInputStatus.externalUpButton == ON) || (flagInputStatus.internalUpButton == ON))
  {
    timerBalanceSetting = 0;
    
    isManualButtonPressed = TRUE;
    flagOutput.balanceUp = ON;
    balanceUpCurrent = nvBalanceManualUpCurrent;
    
    if((flagInputStatus.externalDownButton == ON) || (flagInputStatus.internalDownButton == ON))
    {
      flagOutput.balanceUp = OFF;
      flagOutput.balanceDown = OFF;
    }
  }
  else if((flagInputStatus.externalDownButton == ON) || (flagInputStatus.internalDownButton == ON))
  {
    timerBalanceSetting = 0;
    
    isManualButtonPressed = TRUE;
    flagOutput.balanceDown = ON;
    balanceDownCurrent = nvBalanceManualDownCurrent;
    
    if((flagInputStatus.externalUpButton == ON) || (flagInputStatus.internalUpButton == ON))
    {
      flagOutput.balanceUp = OFF;
      flagOutput.balanceDown = OFF;
    }
  }
  else {
    isManualButtonPressed = FALSE;
  }

  if((flagBalance.balanceMode == VAB_BALANCE_FLAT_MODE) || (flagBalance.balanceMode == VAB_BALANCE_SLOPE_MODE))
  {
    // Auto mode but button is pressed
    if(isManualButtonPressed == TRUE)
    {
      // The button input is
      flagBalance.autoCenterStop = TRUE;
      flagBalance.autoFlat = FALSE;
      flagBalance.autoSlope = FALSE;
      timerFlat = 0;
      timerSlope = 0;
      balance_output_limit();
      return;
    }
    
    // 3P position is higher that pto stop position
    if(flagBalance.ptoStopped == TRUE)
    {
      auto_center();
      balance_output_limit();
      return;
    }
    
    if(flagBalance.balanceMode == VAB_BALANCE_FLAT_MODE)
    {
      // Flat mode
      flagBalance.autoSlope = FALSE;
      timerSlope = 0;
      auto_flat();
      balance_output_limit();
      return;
    }
    else
    {
      // Slope mode
      flagBalance.autoFlat = FALSE;
      timerFlat = 0;
      auto_slope();
      balance_output_limit();
      return;
    }
  }
  
  // Manual mode
  flagBalance.autoFlat = FALSE;
  flagBalance.autoSlope = FALSE;
  timerSlope = 0;
  timerFlat = 0;
  balance_output_limit();
  return;
}


void balance_setting(void)
{
  timerBalanceSetting += 2;
  
  if(flagBalance.settingMode == FALSE)
  {
    if(flagInputStatus.modeButton == ON)
    {
      if(timerBalanceSetting >= VAC_BALANCE_SETTING_TIME)
      {
        flagBalance.settingMode = TRUE;
        flagBalance.settingStroke = FALSE;
        flagBalance.settingRoll = TRUE;
        
        flagBalance.settingSequence = 0;
        
        timerBalanceSetting = 0;
      }
      return;
    }
    timerBalanceSetting = 0;
    return;
  }
 
  if(timerBalanceSetting >= VAC_BALANCE_SETTING_OVER_TIME)
  {
    timerBalanceSetting = 0;
    flagBalance.settingMode = FALSE;
    return;
  }
 
  
  if(flagBalance.settingRoll == TRUE)
  {
    flagBalance.settingSequence = 1;
    if((rollAngleRaw >= -1.0) && (rollAngleRaw <= 1.0))
    {
      flagBalance.settingSequence = 2;
       
      if((flagInputStatus.modeButton == ON) && (flagInputStatusPrevious.modeButton == OFF))
      {
        flagBalance.settingSequence = 3;
        save_nvBalanceCenter(TRUE, (uint16_t)((rollAngleRaw + 90) * 100));
        
        flagBalance.settingStroke = TRUE;
        flagBalance.settingRoll = FALSE;
      }
      return;
    }
    return;
  }
  
  if(flagBalance.settingStroke == TRUE)
  {
    flagBalance.settingSequence = 1;
    if((get_stroke_sensor() >= get_control_minimum_data(NV_STROKE_UP_LIMIT)) &&
       (get_stroke_sensor() <= get_control_maximum_data(NV_STROKE_UP_LIMIT)))
    {
      flagBalance.settingSequence = 2;

      if((flagInputStatus.modeButton == ON) && (flagInputStatusPrevious.modeButton == OFF))
      {
        flagBalance.settingSequence = 3;
        
        save_nvStrokeUpLimit(TRUE, get_stroke_sensor());
        save_nvStrokeCenter(TRUE, (nvStrokeDownLimit + nvStrokeUpLimit) / 2);
      }
      return;
    }
    else if((get_stroke_sensor() >= get_control_minimum_data(NV_STROKE_DOWN_LIMIT)) &&
            (get_stroke_sensor() <= get_control_maximum_data(NV_STROKE_DOWN_LIMIT)))
    {
      flagBalance.settingSequence = 2;

      if((flagInputStatus.modeButton == ON) && (flagInputStatusPrevious.modeButton == OFF))
      {
        flagBalance.settingSequence = 3;
        
        save_nvStrokeDownLimit(TRUE, get_stroke_sensor());
        save_nvStrokeCenter(TRUE, (nvStrokeDownLimit + nvStrokeUpLimit) / 2);
        
      }
      return;
    }
    return;
  }
}

void balance_init()
{ 
  flagBalance.settingMode = FALSE;
  flagBalance.settingRoll = FALSE;
  flagBalance.settingStroke = FALSE;
  flagBalance.balanceMode = VAB_BALANCE_MANUAL_MODE;
  
  strokeAngle = get_stroke_sensor_angle();
}

void clear_balanceUpStall()
{
  balanceStallUp = 0;
}

void clear_balanceDownStall()
{
  balanceStallDown = 0;
}

uint16_t get_balanceUpStall()
{
  return balanceStallUp;
}

uint16_t get_balanceDownStall()
{
  return balanceStallDown;
}

uint16_t get_balanceUpCurrent()
{
  if(balanceUpCurrent >= VAC_BALANCE_MAX_CURRENT)
    balanceUpCurrent = VAC_BALANCE_MAX_CURRENT;
  
  return balanceUpCurrent;
}

uint16_t get_balanceDownCurrent()
{
  if(balanceDownCurrent >= VAC_BALANCE_MAX_CURRENT)
    balanceDownCurrent = VAC_BALANCE_MAX_CURRENT;
  
  return balanceDownCurrent;
}
