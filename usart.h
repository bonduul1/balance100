/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>

/* Defines  ------------------------------------------------------------------*/
#define RX_PACKET_LENGTH       1
  
/* Variables------------------------------------------------------------------*/
  
void MX_USART1_UART_Init(void);
uint8_t startUsart();
uint8_t updateUsart();
void usart_transmit(uint8_t *tx_data, uint8_t len);
void usart_transmit_one(uint8_t tx_data);
void usart_receive(uint8_t *data);                              // The transmitted pointer should be allocated same size as "RX_PACKET_LENGTH"
uint8_t checkUsart();

#ifdef __cplusplus
}
#endif

#endif /* __USART_H */
