/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INPUT_H
#define __INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
  
/* Defines -------------------------------------------------------------------*/
#define SW_UP_EXT_Pin                   GPIO_PIN_8
#define SW_UP_EXT_GPIO_Port             GPIOB
#define SW_DOWN_EXT_Pin                 GPIO_PIN_9
#define SW_DOWN_EXT_GPIO_Port           GPIOB
#define SW_MODE_Pin                     GPIO_PIN_11
#define SW_MODE_GPIO_Port               GPIOB
#define SW_UP_INT_Pin                   GPIO_PIN_2
#define SW_UP_INT_GPIO_Port             GPIOB
#define SW_DOWN_INT_Pin                 GPIO_PIN_10
#define SW_DOWN_INT_GPIO_Port           GPIOB

#define NUMBER_OF_INPUTS                5
  
typedef union {
  uint32_t data;
  struct {    
    uint32_t externalUpButton           : 1;
    uint32_t externalDownButton         : 1;
    uint32_t modeButton                 : 1;
    uint32_t internalUpButton           : 1;
    uint32_t internalDownButton         : 1;
    
    uint32_t res                        : 27;
  };
} flagInputStatus_t;

extern flagInputStatus_t flagInputStatus;
extern flagInputStatus_t flagInputStatusPrevious;


void gpio_input_init(void);
void read_digital_inputs();

uint16_t get_raw_hitch_dial();
uint16_t get_raw_balance_sensitive_dial();
uint16_t get_raw_balance_configure_dial();
uint16_t get_raw_direction();
uint16_t get_raw_hitch_position_sensor();
uint16_t get_raw_stroke_sensor();
uint16_t get_raw_balance_sensor();
uint16_t get_raw_battery();

#ifdef __cplusplus
}
#endif

#endif /* __INPUT_H */
