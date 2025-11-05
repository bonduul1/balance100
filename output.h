/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OUTPUT_H
#define __OUTPUT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
  
/* Defines -------------------------------------------------------------------*/
#define FCO_LED1_Pin                    GPIO_PIN_11
#define FCO_LED1_GPIO_Port              GPIOC
#define FCO_LED2_Pin                    GPIO_PIN_12
#define FCO_LED2_GPIO_Port              GPIOC
#define FCO_LED3_Pin                    GPIO_PIN_2
#define FCO_LED3_GPIO_Port              GPIOD
  
/* Global variable -----------------------------------------------------------*/

typedef union {
  uint32_t data;
  struct {
    uint32_t ledManual          : 1;
    uint32_t ledFlat            : 1;
    uint32_t ledSlope           : 1;
    uint32_t balanceUp          : 1;
    uint32_t balanceDown        : 1;

    uint32_t res                : 27;
  } ;
} flagOutput_t;

extern flagOutput_t flagOutput;
void    gpio_output_init();
uint8_t output_init();

void    output_clear();
void    output_controller();

#ifdef __cplusplus
}
#endif

#endif /* __OUTPUT_H */
