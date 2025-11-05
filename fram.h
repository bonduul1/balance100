#ifndef __FRAM_H__
#define __FRAM_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>


/* Defines -------------------------------------------------------------------*/

/*
 *      2021.07.27
 *              1. In this board we use "CY15B064Q" this type of FRAM
 *                 Hold and write protect pins are connected to VCC which means these pins are not used in this example
 *              2. MSB FIRST
 *              3. 
 */

#define MEM_CS_Pin                      GPIO_PIN_12
#define MEM_CS_GPIO_Port                GPIOB

//#define MB85RS64V                       1
//#define MB85RS16N                       1
#define CY15B064Q                       1                                       // This IC has no product ID register

#if defined(MB85RS64V) || defined(MB85RS16N)
  #define MANUFACTURER_ID                 0x04                                    // Fijitsu
  #define CONTINUATION_CODE               0x7F
  #if defined(MB85RS64V)
    #define PRODUCT_ID                    0x0302                                  // 03 is density - 64Kbit
  #elif defined(MB85RS16N)
    #define PRODUCT_ID                    0x0101                                  // 01 is density - 16Kbit
  #endif
#endif

#define OP_CODE_WREN                    0x06                                    // Set write enable latch
#define OP_CODE_WRDI                    0x04                                    // Reset write enable latch
#define OP_CODE_RDSR                    0x05                                    // Read status register
#define OP_CODE_WRSR                    0x01                                    // Write status register
#define OP_CODE_READ                    0x03                                    // Read memory code
#define OP_CODE_WRITE                   0x02                                    // Write memory code

#if defined(MB85RS64V) || defined(MB85RS16N)
  #define OP_CODE_RDID                  0x9F                                    // Read device ID
#endif 

#define WRITE_PROTECT_BIT               0x80

#define BLOCK_PROTECT_NONE              0x00
#define BLOCK_PROTECT_UPPER_1_4         0x01
#define BLOCK_PROTECT_UPPER_1_2         0x02
#define BLOCK_PROTECT_ALL               0x03


uint8_t fram_init();
uint8_t fram_check(void);
uint8_t fram_read_status(uint8_t *status);
uint8_t fram_write_status(uint8_t status);
uint8_t fram_write(uint16_t addr, uint8_t* dt);
uint8_t fram_read(uint16_t addr, uint8_t *rdt);
uint8_t fram_test();

void    fram_enable();
void    fram_disable();

#endif

