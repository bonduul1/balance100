/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H
#define __SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Defines  ------------------------------------------------------------------*/

  
/* Functions -----------------------------------------------------------------*/  
void MX_SPI1_Init(void);
uint32_t SPI_TransmitReceive(uint32_t txData);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H */
