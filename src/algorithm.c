#include "algorithm.h"
#include "input.h"
#include "can.h"
#include "output.h"
#include "main.h"
#include "sensors.h"
#include "balance.h"
#include "settings.h"

void control_init()
{
  init_diagnostic_variables();
}

void control_process()
{
  output_clear();
  
  analog_sensors();
  digital_sensors();
  can_receive_process();
  
  balance_setting();
  balance_process();
  
  output_controller();
}
