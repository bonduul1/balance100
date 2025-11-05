/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TLE92464_H
#define __TLE92464_H

/*
  *     File:           TLE9246x.h
  *     Author:         Enkhbat Batbayar
  *     Date:           2022.06.13
  *     Version:        1.0
  *     Note:           TLE9246x IC driver and testing source code
  *     
  */

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Defines  ------------------------------------------------------------------*/
//#define TLE92466_USED                   TRUE
#define TLE92464_USED                   TRUE
  
#define TLE9246x_INIT_STEP_0            0
#define TLE9246x_INIT_STEP_1            1
#define TLE9246x_INIT_STEP_2            2
#define TLE9246x_INIT_STEP_3            3
#define TLE9246x_INIT_STEP_4            4
#define TLE9246x_INIT_READY             5
  
#define TLE9246x_READ                   0
#define TLE9246x_WRITE                  1
  
#define TLE9246x_CRC                    0x1D
#define TLE9246x_MANUFACTURER_ID        0xC1
  
#ifdef  TLE92466_USED
#ifdef TLE9246x_PARALLEL_CHANNEL_USED
  #define TLE9246x_CHANNELS             3
#else
  #define TLE9246x_CHANNELS             6
#endif
#endif
#ifdef TLE92464_USED
#define TLE9246x_PARALLEL_CHANNEL_USED  1                                       // The parallel channel is used therefore the maximum output is 2
#ifdef TLE9246x_PARALLEL_CHANNEL_USED
  #define TLE9246x_CHANNELS             2
#else
  #define TLE9246x_CHANNELS             4
#endif
#endif
  
enum {                                                                          // This enum never higher than TLE9246x_CHANNELS
  TLE9246x_CHANNEL_BALANCE_UP = 0,
  TLE9246x_CHANNEL_BALANCE_DOWN
};
  
/* Defines  Pins -------------------------------------------------------------*/  
#define SPI_OUT_EN_Pin                  GPIO_PIN_1
#define SPI_OUT_EN_GPIO_Port            GPIOB

#define FO1_CS_Pin                      GPIO_PIN_4
#define FO1_CS_GPIO_Port                GPIOA
#define FO1_FAULTN_Pin                  GPIO_PIN_0
#define FO1_FAULTN_GPIO_Port            GPIOB
#define FO1_RESN_Pin                    GPIO_PIN_5
#define FO1_RESN_GPIO_Port              GPIOC
   
/* Defines  Registers --------------------------------------------------------*/
#define CENTRAL_REGISTER_BASE_ADDRESS           0x0000
#define CHANNEL0_REGISTER_BASE_ADDRESS          0x0040
#define CHANNEL1_REGISTER_BASE_ADDRESS          0x0050
#define CHANNEL2_REGISTER_BASE_ADDRESS          0x0060
#define CHANNEL3_REGISTER_BASE_ADDRESS          0x0070
  
#ifdef  TLE92466_USED
#define CHANNEL4_REGISTER_BASE_ADDRESS          0x0020
#define CHANNEL5_REGISTER_BASE_ADDRESS          0x0030
#endif
   
/*---------------------------- Central Registers -----------------------------*/
#define CH_CTRL                 0x0000          /* Channel Control Register                     */
#define GLOBAL_CONFIG           0x0002          /* Global Configuration Register                */
#define GLOBAL_DIAG0            0x0003          /* Global Diagnosis Register 0                  */
#define GLOBAL_DIAG1            0x0004          /* Global Diagnosis Register 1                  */
#define GLOBAL_DIAG2            0x0005          /* Global Diagnosis Register 2                  */
#define VBAT_TH                 0x0006          /* VBAT Threshold Register                      */
#define FB_FRZ                  0x0007          /* Feedback Freeze Register                     */
#define FB_UPD                  0x0008          /* Feedback Update Register                     */
#define WD_RELOAD               0x0009          /* SPI Watchdog Register                        */
#define DIAG_ERR_CHGR0          0x000A          /* Diagnosis Error Register 0                   */
#define DIAG_ERR_CHGR1          0x000B          /* Diagnosis Error Register 1                   */
#ifdef  TLE92466_USED
#define DIAG_ERR_CHGR2          0x000C          /* Diagnosis Error Register 2                   */
#endif
#define DIAG_WARN_CHGR0         0x0010          /* Diagnosis Warning Register 0                 */
#define DIAG_WARN_CHGR1         0x0011          /* Diagnosis Warning Register 1                 */
#ifdef  TLE92466_USED
#define DIAG_WARN_CHGR2         0x0012          /* Diagnosis Warning Register 2                 */
#endif
#define FAULT_MASK0             0x0016          /* Fault Mask Register 0                        */
#define FAULT_MASK1             0x0017          /* Fault Mask Register 1                        */
#define FAULT_MASK2             0x0018          /* Fault Mask Register 2                        */
#define CLK_DIV                 0x0019          /* Clock Control Register                       */
#define SFF_BIST                0x003F          /* BIST Register                                */
#define ICVID                   0x0200          /* Version Register                             */
#define PIN_STAT                0x0201          /* Pin Status Register                          */
#define FB_STAT                 0x0202          /* Feedback Status Register                     */
#define FB_VOLTAGE1             0x0203          /* Feedback Voltage Register 1                  */
#define FB_VOLTAGE2             0x0204          /* Feedback Voltage Register 2                  */
#ifdef  TLE92466_USED
#define CHIPID0                 0x0205          /* Unique Chip Identification Number Register 0 */
#define CHIPID1                 0x0206          /* Unique Chip Identification Number Register 1 */
#define CHIPID2                 0x0207          /* Unique Chip Identification Number Register 2 */
#endif

/*---------------------------- Channel Registers -----------------------------*/
#define SETPOINT                0x0000          /* Setpoint Register                            */
#define CTRL_REG                0x0001          /* Control Register                             */
#define PERIOD                  0x0002          /* ICC PWM Frequency Controller Register        */
#define INTEGRATOR_LIMIT        0x0003          /* ICC Integrator Limitation Register           */
#define DITHER_CLK_DIV          0x0004          /* Dither Clock Register                        */
#define DITHER_STEP             0x0005          /* Dither Step Register                         */
#define DITHER_CTRL             0x0006          /* Dither Control Register                      */
#define CH_CONFIG               0x0007          /* Channel Configuration Register               */
#define MODE                    0x000C          /* Channel Mode Register                        */
#define TON                     0x000D          /* On-Time Register                             */
#define CTRL_INT_THRESH         0x000E          /* ICC Integrator Threshold Control Register    */
#define FB_DC                   0x0200          /* Feedback Duty Cycle Register                 */
#define FB_VBAT                 0x0201          /* Feedback Average VBAT                        */
#define FB_I_AVG                0x0202          /* Feedback Average Current                     */
#define FB_IMIN_IMAX            0x0203          /* Feedback Min/Max Current                     */
#define FB_I_AVG_s16            0x0204          /* Feedback signed Current                      */
#define FB_INT_THRESH           0x0205          /* Feedback ICC Integrator Threshold            */
#define FB_PERIOD_MIN_MAX       0x0206          /* Feedback Min/Max PWM Period                  */
  
/*---------------------------- Typedefs for TLE9246x -------------------------*/
typedef union {
  uint32_t reg;
  uint8_t buf[4];
  struct {
    uint32_t data       : 16;
    uint32_t rw         : 1;
    uint32_t address    : 7;
    uint32_t crc        : 8;
  };
}writeFrame_t;

typedef union {
  uint32_t reg;
  uint8_t buf[4];
  struct {
    uint32_t address    : 16;
    uint32_t rw         : 1;
    uint32_t reserved   : 7;
    uint32_t crc        : 8;
  };
}readFrame_t;
   
typedef union {
  uint32_t reg;
  uint8_t buf[4];
  struct {
    uint32_t data       : 16;
    uint32_t rw         : 1;
    uint32_t status     : 5;
    uint32_t replyMode  : 2;
    uint32_t crc        : 8;
  };
}reply16_t;

typedef union {
  uint32_t reg;
  uint8_t buf[4];
  struct {
    uint32_t data       : 22;
    uint32_t replyMode  : 2;
    uint32_t crc        : 8;
  };
}reply22_t;

typedef union {
  uint32_t reg;
  uint8_t buf[4];
  struct {
    uint32_t wd_ref_clk         : 1;
    uint32_t dig_clk_fast       : 1;
    uint32_t dig_clk_slow       : 1;
    uint32_t clk_fast           : 1;
    uint32_t clk_slow           : 1;
    uint32_t bg                 : 1;
    uint32_t voltage2v5         : 1;
    uint32_t voltage1v5         : 1;
    uint32_t reserved1          : 14;
    uint32_t replyMode          : 2;
    uint32_t reserved           : 8;
  };
}criticalReply_t;

/*---------------------------- Global  Functions -----------------------------*/
uint8_t tle9246x_init();

uint8_t tle9246x_handler();

uint8_t tle9246x_set_settings(uint16_t* data);
uint8_t tle9246x_set_current(uint8_t channel, uint16_t outCurrent);
uint8_t tle9246x_get_diagnostic();

uint8_t tle9246x_clear_errors(uint8_t clearFlags);
void tle9246x_disable_output();
void tle9246x_enable_output();

extern uint8_t tle9246x_outputError[TLE9246x_CHANNELS];
extern uint16_t tle9246x_outputError_local[TLE9246x_CHANNELS];
#ifdef __cplusplus
}
#endif

#endif /* __TLE92464_H */
