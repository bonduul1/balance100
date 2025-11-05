/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
  
/* Defines -------------------------------------------------------------------*/
#define ADC1_Pin                        GPIO_PIN_0
#define ADC1_GPIO_Port                  GPIOC
#define ADC2_Pin                        GPIO_PIN_1
#define ADC2_GPIO_Port                  GPIOC
#define ADC3_Pin                        GPIO_PIN_2
#define ADC3_GPIO_Port                  GPIOC
#define ADC4_Pin                        GPIO_PIN_3
#define ADC4_GPIO_Port                  GPIOC
#define ADC5_Pin                        GPIO_PIN_0
#define ADC5_GPIO_Port                  GPIOA
#define ADC6_Pin                        GPIO_PIN_1
#define ADC6_GPIO_Port                  GPIOA
#define ADC7_Pin                        GPIO_PIN_2
#define ADC7_GPIO_Port                  GPIOA
#define ADC8_Pin                        GPIO_PIN_3
#define ADC8_GPIO_Port                  GPIOA
  
#define NUMBER_OF_ADC_AVERAGE           4

enum
{
  ADC_HIGHSET = 0,
  ADC_SENSTIVE,
  ADC_SET_BALL,
  ADC_DIR,
  ADC_LIFT,
  ADC_STROKE,
  ADC_BAL,
  ADC_PWR,
  NUMBER_OF_ADC_CHANNELS  
};

/* Functions -----------------------------------------------------------------*/
void MX_DMA_Init(void);
void MX_ADC1_Init(void);

uint8_t  startConversation();
uint8_t  updateADC();
uint8_t  updateLastAverageADC();

uint16_t getCurrentADCValue(uint8_t channel);
uint16_t getAverageADCValue(uint8_t channel);
uint8_t  getCurrentADCValues(uint16_t *channels_data);
uint8_t  getAverageADCValues(uint16_t *channels_data);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H */
