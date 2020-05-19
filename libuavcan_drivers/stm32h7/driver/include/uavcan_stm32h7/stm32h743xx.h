/**
  ******************************************************************************
  * @file    stm32h743xx.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    31-August-2017
  * @brief   CMSIS STM32H743xx Device Peripheral Access Layer Header File.
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral's registers declarations and bits definition
  *           - Macros to access peripheral�s registers hardware
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#define __IO
#define __I

#ifndef __STM32H743xx_H
#define __STM32H743xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief STM32H7XX Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum
{
/******  Cortex-M Processor Exceptions Numbers *****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  HardFault_IRQn              = -13,    /*!< 4 Cortex-M Memory Management Interrupt                            */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M Memory Management Interrupt                            */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M Bus Fault Interrupt                                    */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M Usage Fault Interrupt                                  */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M SV Call Interrupt                                     */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M Debug Monitor Interrupt                               */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M Pend SV Interrupt                                     */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M System Tick Interrupt                                 */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_AVD_IRQn                = 1,      /*!< PVD/AVD through EXTI Line detection Interrupt                     */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1 and  ADC2 global Interrupts                                  */
  FDCAN1_IT0_IRQn             = 19,     /*!< FDCAN1 Interrupt line 0                                           */
  FDCAN2_IT0_IRQn             = 20,     /*!< FDCAN2 Interrupt line 0                                           */
  FDCAN1_IT1_IRQn             = 21,     /*!< FDCAN1 Interrupt line 1                                           */
  FDCAN2_IT1_IRQn             = 22,     /*!< FDCAN2 Interrupt line 1                                           */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_IRQn               = 24,     /*!< TIM1 Break Interrupt                                              */
  TIM1_UP_IRQn                = 25,     /*!< TIM1 Update Interrupt                                             */
  TIM1_TRG_COM_IRQn           = 26,     /*!< TIM1 Trigger and Commutation Interrupt                            */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDMMC1_IRQn                 = 49,     /*!< SDMMC1 global Interrupt                                           */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!<   DMA2 Stream 0 global Interrupt                                  */
  DMA2_Stream1_IRQn           = 57,     /*!<   DMA2 Stream 1 global Interrupt                                  */
  DMA2_Stream2_IRQn           = 58,     /*!<   DMA2 Stream 2 global Interrupt                                  */
  DMA2_Stream3_IRQn           = 59,     /*!<   DMA2 Stream 3 global Interrupt                                  */
  DMA2_Stream4_IRQn           = 60,     /*!<   DMA2 Stream 4 global Interrupt                                  */
  ETH_IRQn                    = 61,     /*!< Ethernet global Interrupt                                         */
  ETH_WKUP_IRQn               = 62,     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
  FDCAN_CAL_IRQn              = 63,     /*!< FDCAN Calibration unit Interrupt                                  */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  RNG_IRQn                    = 80,     /*!< RNG global interrupt                                              */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  UART7_IRQn                  = 82,     /*!< UART7 global interrupt                                            */
  UART8_IRQn                  = 83,     /*!< UART8 global interrupt                                            */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SPI5_IRQn                   = 85,     /*!< SPI5 global Interrupt                                             */
  SPI6_IRQn                   = 86,     /*!< SPI6 global Interrupt                                             */
  SAI1_IRQn                   = 87,     /*!< SAI1 global Interrupt                                             */
  LTDC_IRQn                   = 88,     /*!< LTDC global Interrupt                                             */
  LTDC_ER_IRQn                = 89,     /*!< LTDC Error global Interrupt                                       */
  DMA2D_IRQn                  = 90,     /*!< DMA2D global Interrupt                                            */
  SAI2_IRQn                   = 91,     /*!< SAI2 global Interrupt                                             */
  QUADSPI_IRQn                = 92,     /*!< Quad SPI global interrupt                                         */
  LPTIM1_IRQn                 = 93,     /*!< LP TIM1 interrupt                                                 */
  CEC_IRQn                    = 94,     /*!< HDMI-CEC global Interrupt                                         */
  I2C4_EV_IRQn                = 95,     /*!< I2C4 Event Interrupt                                              */
  I2C4_ER_IRQn                = 96,     /*!< I2C4 Error Interrupt                                              */
  SPDIF_RX_IRQn               = 97,     /*!< SPDIF-RX global Interrupt                                         */
  OTG_FS_EP1_OUT_IRQn         = 98,     /*!< USB OTG HS2 global interrupt                                      */
  OTG_FS_EP1_IN_IRQn          = 99,     /*!< USB OTG HS2 End Point 1 Out global interrupt                      */
  OTG_FS_WKUP_IRQn            = 100,    /*!< USB OTG HS2 End Point 1 In global interrupt                       */
  OTG_FS_IRQn                 = 101,    /*!< USB OTG HS2 Wakeup through EXTI interrupt                         */
  DMAMUX1_OVR_IRQn	          = 102,    /*!<DMAMUX1 Overrun interrupt                                          */
  HRTIM1_Master_IRQn          = 103,    /*!< HRTIM Master Timer global Interrupts                              */
  HRTIM1_TIMA_IRQn            = 104,    /*!< HRTIM Timer A global Interrupt                                    */
  HRTIM1_TIMB_IRQn            = 105,    /*!< HRTIM Timer B global Interrupt                                    */
  HRTIM1_TIMC_IRQn            = 106,    /*!< HRTIM Timer C global Interrupt                                    */
  HRTIM1_TIMD_IRQn            = 107,    /*!< HRTIM Timer D global Interrupt                                    */
  HRTIM1_TIME_IRQn            = 108,    /*!< HRTIM Timer E global Interrupt                                    */
  HRTIM1_FLT_IRQn             = 109,    /*!< HRTIM Fault global Interrupt                                      */
  DFSDM1_FLT0_IRQn            = 110,    /*!<DFSDM Filter1 Interrupt                                            */
  DFSDM1_FLT1_IRQn            = 111,    /*!<DFSDM Filter2 Interrupt                                            */
  DFSDM1_FLT2_IRQn            = 112,    /*!<DFSDM Filter3 Interrupt                                            */
  DFSDM1_FLT3_IRQn            = 113,    /*!<DFSDM Filter4 Interrupt                                            */
  SAI3_IRQn                   = 114,    /*!< SAI3 global Interrupt                                             */
  SWPMI1_IRQn                 = 115,    /*!< Serial Wire Interface 1 global interrupt                          */
  TIM15_IRQn                  = 116,    /*!< TIM15 global Interrupt                                            */
  TIM16_IRQn                  = 117,    /*!< TIM16 global Interrupt                                            */
  TIM17_IRQn                  = 118,    /*!< TIM17 global Interrupt                                            */
  MDIOS_WKUP_IRQn             = 119,    /*!< MDIOS Wakeup  Interrupt                                           */
  MDIOS_IRQn                  = 120,    /*!< MDIOS global Interrupt                                            */
  JPEG_IRQn                   = 121,    /*!< JPEG global Interrupt                                             */
  MDMA_IRQn                   = 122,    /*!< MDMA global Interrupt                                             */
  SDMMC2_IRQn                 = 124,    /*!< SDMMC2 global Interrupt                                           */
  HSEM1_IRQn                  = 125,    /*!< HSEM1 global Interrupt                                            */
  ADC3_IRQn                   = 127,    /*!< ADC3 global Interrupt                                             */
  DMAMUX2_OVR_IRQn            = 128,    /*!<DMAMUX2 Overrun interrupt                                          */
  BDMA_Channel0_IRQn          = 129,    /*!< BDMA Channel 0 global Interrupt                                   */
  BDMA_Channel1_IRQn          = 130,    /*!< BDMA Channel 1 global Interrupt                                   */
  BDMA_Channel2_IRQn          = 131,    /*!< BDMA Channel 2 global Interrupt                                   */
  BDMA_Channel3_IRQn          = 132,    /*!< BDMA Channel 3 global Interrupt                                   */
  BDMA_Channel4_IRQn          = 133,    /*!< BDMA Channel 4 global Interrupt                                   */
  BDMA_Channel5_IRQn          = 134,    /*!< BDMA Channel 5 global Interrupt                                   */
  BDMA_Channel6_IRQn          = 135,    /*!< BDMA Channel 6 global Interrupt                                   */
  BDMA_Channel7_IRQn          = 136,    /*!< BDMA Channel 7 global Interrupt                                   */
  COMP_IRQn                   = 137 ,   /*!< COMP global Interrupt                                             */
  LPTIM2_IRQn                 = 138,    /*!< LP TIM2 global interrupt                                          */
  LPTIM3_IRQn                 = 139,    /*!< LP TIM3 global interrupt                                          */
  LPTIM4_IRQn                 = 140,    /*!< LP TIM4 global interrupt                                          */
  LPTIM5_IRQn                 = 141,    /*!< LP TIM5 global interrupt                                          */
  LPUART1_IRQn                = 142,    /*!< LP UART1 interrupt                                                */
  CRS_IRQn                    = 144,    /*!< Clock Recovery Global Interrupt                                   */
  SAI4_IRQn                   = 146,    /*!< SAI4 global interrupt                                             */
  WAKEUP_PIN_IRQn             = 149,    /*!< Interrupt for all 6 wake-up pins                                  */
} IRQn_Type;

/**
  * @}
  */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */




/**
  * @brief Configuration of the Cortex-M7 Processor and Core Peripherals
   */
#define __CM7_REV               0x0100U   /*!< Cortex-M7 revision r1p0                       */
#define __MPU_PRESENT             1       /*!< CM7 provides an MPU                           */
#define __NVIC_PRIO_BITS          4       /*!< CM7 uses 4 Bits for the Priority Levels       */
#define __Vendor_SysTickConfig    0       /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT             1       /*!< FPU present                                   */
#define __ICACHE_PRESENT          1       /*!< CM7 instruction cache present                 */
#define __DCACHE_PRESENT          1       /*!< CM7 data cache present                        */
// #include "core_cm7.h"                     /*!< Cortex-M7 processor and core peripherals      */

/**
  * @}
  */




// #include "system_stm32h7xx.h"
#include <stdint.h>

/** @addtogroup Peripheral_registers_structures
  * @{
  */

/**
  * @brief Analog to Digital Converter
  */

typedef struct
{
  __IO uint32_t ISR;              /*!< ADC Interrupt and Status Register,                 Address offset: 0x00 */
  __IO uint32_t IER;              /*!< ADC Interrupt Enable Register,                     Address offset: 0x04 */
  __IO uint32_t CR;               /*!< ADC control register,                              Address offset: 0x08 */
  __IO uint32_t CFGR;             /*!< ADC Configuration register,                        Address offset: 0x0C */
  __IO uint32_t CFGR2;            /*!< ADC Configuration register 2,                      Address offset: 0x10 */
  __IO uint32_t SMPR1;            /*!< ADC sample time register 1,                        Address offset: 0x14 */
  __IO uint32_t SMPR2;            /*!< ADC sample time register 2,                        Address offset: 0x18 */
  __IO uint32_t PCSEL;            /*!< ADC pre-channel selection,                         Address offset: 0x1C */
  __IO uint32_t LTR1;             /*!< ADC watchdog Lower threshold register 1,           Address offset: 0x20 */
  __IO uint32_t HTR1;             /*!< ADC watchdog higher threshold register 1,          Address offset: 0x24 */
  uint32_t      RESERVED1;        /*!< Reserved, 0x028                                                         */
  uint32_t      RESERVED2;        /*!< Reserved, 0x02C                                                         */
  __IO uint32_t SQR1;             /*!< ADC regular sequence register 1,                   Address offset: 0x30 */
  __IO uint32_t SQR2;             /*!< ADC regular sequence register 2,                   Address offset: 0x34 */
  __IO uint32_t SQR3;             /*!< ADC regular sequence register 3,                   Address offset: 0x38 */
  __IO uint32_t SQR4;             /*!< ADC regular sequence register 4,                   Address offset: 0x3C */
  __IO uint32_t DR;               /*!< ADC regular data register,                         Address offset: 0x40 */
  uint32_t      RESERVED3;        /*!< Reserved, 0x044                                                         */
  uint32_t      RESERVED4;        /*!< Reserved, 0x048                                                         */
  __IO uint32_t JSQR;             /*!< ADC injected sequence register,                    Address offset: 0x4C */
  uint32_t      RESERVED5[4];     /*!< Reserved, 0x050 - 0x05C                                                 */
  __IO uint32_t OFR1;             /*!< ADC offset register 1,                             Address offset: 0x60 */
  __IO uint32_t OFR2;             /*!< ADC offset register 2,                             Address offset: 0x64 */
  __IO uint32_t OFR3;             /*!< ADC offset register 3,                             Address offset: 0x68 */
  __IO uint32_t OFR4;             /*!< ADC offset register 4,                             Address offset: 0x6C */
  uint32_t      RESERVED6[4];     /*!< Reserved, 0x070 - 0x07C                                                 */
  __IO uint32_t JDR1;             /*!< ADC injected data register 1,                      Address offset: 0x80 */
  __IO uint32_t JDR2;             /*!< ADC injected data register 2,                      Address offset: 0x84 */
  __IO uint32_t JDR3;             /*!< ADC injected data register 3,                      Address offset: 0x88 */
  __IO uint32_t JDR4;             /*!< ADC injected data register 4,                      Address offset: 0x8C */
  uint32_t      RESERVED7[4];     /*!< Reserved, 0x090 - 0x09C                                                 */
  __IO uint32_t AWD2CR;           /*!< ADC  Analog Watchdog 2 Configuration Register,     Address offset: 0xA0 */
  __IO uint32_t AWD3CR;           /*!< ADC  Analog Watchdog 3 Configuration Register,     Address offset: 0xA4 */
  uint32_t      RESERVED8;        /*!< Reserved, 0x0A8                                                         */
  uint32_t      RESERVED9;        /*!< Reserved, 0x0AC                                                         */
  __IO uint32_t LTR2;             /*!< ADC watchdog Lower threshold register 2,           Address offset: 0xB0 */
  __IO uint32_t HTR2;             /*!< ADC watchdog Higher threshold register 2,          Address offset: 0xB4 */
  __IO uint32_t LTR3;             /*!< ADC watchdog Lower threshold register 3,           Address offset: 0xB8 */
  __IO uint32_t HTR3;             /*!< ADC watchdog Higher threshold register 3,          Address offset: 0xBC */
  __IO uint32_t DIFSEL;           /*!< ADC  Differential Mode Selection Register,         Address offset: 0xC0 */
  __IO uint32_t CALFACT;          /*!< ADC  Calibration Factors,                          Address offset: 0xC4 */
  __IO uint32_t CALFACT2;         /*!< ADC  Linearity Calibration Factors,                Address offset: 0xC8 */
} ADC_TypeDef;


typedef struct
{
__IO uint32_t CSR; /*!< ADC Common status register, Address offset: ADC1/3 base address + 0x300 */
uint32_t RESERVED; /*!< Reserved, ADC1/3 base address + 0x304 */
__IO uint32_t CCR; /*!< ADC common control register, Address offset: ADC1/3 base address + 0x308 */
__IO uint32_t CDR; /*!< ADC common regular data register for dual Address offset: ADC1/3 base address + 0x30C */
__IO uint32_t CDR2; /*!< ADC common regular data register for 32-bit dual mode Address offset: ADC1/3 base address + 0x310 */

} ADC_Common_TypeDef;

/**
  * @brief VREFBUF
  */

typedef struct
{
  __IO uint32_t CSR;         /*!< VREFBUF control and status register,         Address offset: 0x00 */
  __IO uint32_t CCR;         /*!< VREFBUF calibration and control register,    Address offset: 0x04 */
} VREFBUF_TypeDef;


/**
  * @brief FD Controller Area Network
  */

typedef struct
{
  __IO uint32_t CREL;         /*!< FDCAN Core Release register,                                     Address offset: 0x000 */
  __IO uint32_t ENDN;         /*!< FDCAN Endian register,                                           Address offset: 0x004 */
  __IO uint32_t RESERVED1;    /*!< Reserved,                                                                        0x008 */
  __IO uint32_t DBTP;         /*!< FDCAN Data Bit Timing & Prescaler register,                      Address offset: 0x00C */
  __IO uint32_t TEST;         /*!< FDCAN Test register,                                             Address offset: 0x010 */
  __IO uint32_t RWD;          /*!< FDCAN RAM Watchdog register,                                     Address offset: 0x014 */
  __IO uint32_t CCCR;         /*!< FDCAN CC Control register,                                       Address offset: 0x018 */
  __IO uint32_t NBTP;         /*!< FDCAN Nominal Bit Timing & Prescaler register,                   Address offset: 0x01C */
  __IO uint32_t TSCC;         /*!< FDCAN Timestamp Counter Configuration register,                  Address offset: 0x020 */
  __IO uint32_t TSCV;         /*!< FDCAN Timestamp Counter Value register,                          Address offset: 0x024 */
  __IO uint32_t TOCC;         /*!< FDCAN Timeout Counter Configuration register,                    Address offset: 0x028 */
  __IO uint32_t TOCV;         /*!< FDCAN Timeout Counter Value register,                            Address offset: 0x02C */
  __IO uint32_t RESERVED2[4]; /*!< Reserved,                                                                0x030 - 0x03C */
  __IO uint32_t ECR;          /*!< FDCAN Error Counter register,                                    Address offset: 0x040 */
  __IO uint32_t PSR;          /*!< FDCAN Protocol Status register,                                  Address offset: 0x044 */
  __IO uint32_t TDCR;         /*!< FDCAN Transmitter Delay Compensation register,                   Address offset: 0x048 */
  __IO uint32_t RESERVED3;    /*!< Reserved,                                                                        0x04C */
  __IO uint32_t IR;           /*!< FDCAN Interrupt register,                                        Address offset: 0x050 */
  __IO uint32_t IE;           /*!< FDCAN Interrupt Enable register,                                 Address offset: 0x054 */
  __IO uint32_t ILS;          /*!< FDCAN Interrupt Line Select register,                            Address offset: 0x058 */
  __IO uint32_t ILE;          /*!< FDCAN Interrupt Line Enable register,                            Address offset: 0x05C */
  __IO uint32_t RESERVED4[8]; /*!< Reserved,                                                                0x060 - 0x07C */
  __IO uint32_t GFC;          /*!< FDCAN Global Filter Configuration register,                      Address offset: 0x080 */
  __IO uint32_t SIDFC;        /*!< FDCAN Standard ID Filter Configuration register,                 Address offset: 0x084 */
  __IO uint32_t XIDFC;        /*!< FDCAN Extended ID Filter Configuration register,                 Address offset: 0x088 */
  __IO uint32_t RESERVED5;    /*!< Reserved,                                                                        0x08C */
  __IO uint32_t XIDAM;        /*!< FDCAN Extended ID AND Mask register,                             Address offset: 0x090 */
  __IO uint32_t HPMS;         /*!< FDCAN High Priority Message Status register,                     Address offset: 0x094 */
  __IO uint32_t NDAT1;        /*!< FDCAN New Data 1 register,                                       Address offset: 0x098 */
  __IO uint32_t NDAT2;        /*!< FDCAN New Data 2 register,                                       Address offset: 0x09C */
  __IO uint32_t RXF0C;        /*!< FDCAN Rx FIFO 0 Configuration register,                          Address offset: 0x0A0 */
  __IO uint32_t RXF0S;        /*!< FDCAN Rx FIFO 0 Status register,                                 Address offset: 0x0A4 */
  __IO uint32_t RXF0A;        /*!< FDCAN Rx FIFO 0 Acknowledge register,                            Address offset: 0x0A8 */
  __IO uint32_t RXBC;         /*!< FDCAN Rx Buffer Configuration register,                          Address offset: 0x0AC */
  __IO uint32_t RXF1C;        /*!< FDCAN Rx FIFO 1 Configuration register,                          Address offset: 0x0B0 */
  __IO uint32_t RXF1S;        /*!< FDCAN Rx FIFO 1 Status register,                                 Address offset: 0x0B4 */
  __IO uint32_t RXF1A;        /*!< FDCAN Rx FIFO 1 Acknowledge register,                            Address offset: 0x0B8 */
  __IO uint32_t RXESC;        /*!< FDCAN Rx Buffer/FIFO Element Size Configuration register,        Address offset: 0x0BC */
  __IO uint32_t TXBC;         /*!< FDCAN Tx Buffer Configuration register,                          Address offset: 0x0C0 */
  __IO uint32_t TXFQS;        /*!< FDCAN Tx FIFO/Queue Status register,                             Address offset: 0x0C4 */
  __IO uint32_t TXESC;        /*!< FDCAN Tx Buffer Element Size Configuration register,             Address offset: 0x0C8 */
  __IO uint32_t TXBRP;        /*!< FDCAN Tx Buffer Request Pending register,                        Address offset: 0x0CC */
  __IO uint32_t TXBAR;        /*!< FDCAN Tx Buffer Add Request register,                            Address offset: 0x0D0 */
  __IO uint32_t TXBCR;        /*!< FDCAN Tx Buffer Cancellation Request register,                   Address offset: 0x0D4 */
  __IO uint32_t TXBTO;        /*!< FDCAN Tx Buffer Transmission Occurred register,                  Address offset: 0x0D8 */
  __IO uint32_t TXBCF;        /*!< FDCAN Tx Buffer Cancellation Finished register,                  Address offset: 0x0DC */
  __IO uint32_t TXBTIE;       /*!< FDCAN Tx Buffer Transmission Interrupt Enable register,          Address offset: 0x0E0 */
  __IO uint32_t TXBCIE;       /*!< FDCAN Tx Buffer Cancellation Finished Interrupt Enable register, Address offset: 0x0E4 */
  __IO uint32_t RESERVED6[2]; /*!< Reserved,                                                                0x0E8 - 0x0EC */
  __IO uint32_t TXEFC;        /*!< FDCAN Tx Event FIFO Configuration register,                      Address offset: 0x0F0 */
  __IO uint32_t TXEFS;        /*!< FDCAN Tx Event FIFO Status register,                             Address offset: 0x0F4 */
  __IO uint32_t TXEFA;        /*!< FDCAN Tx Event FIFO Acknowledge register,                        Address offset: 0x0F8 */
  __IO uint32_t RESERVED7;    /*!< Reserved,                                                                        0x0FC */
} FDCAN_GlobalTypeDef;

/**
  * @brief TTFD Controller Area Network
  */

typedef struct
{
  __IO uint32_t TTTMC;          /*!< TT Trigger Memory Configuration register,    Address offset: 0x100 */
  __IO uint32_t TTRMC;          /*!< TT Reference Message Configuration register, Address offset: 0x104 */
  __IO uint32_t TTOCF;          /*!< TT Operation Configuration register,         Address offset: 0x108 */
  __IO uint32_t TTMLM;          /*!< TT Matrix Limits register,                   Address offset: 0x10C */
  __IO uint32_t TURCF;          /*!< TUR Configuration register,                  Address offset: 0x110 */
  __IO uint32_t TTOCN;          /*!< TT Operation Control register,               Address offset: 0x114 */
  __IO uint32_t TTGTP;          /*!< TT Global Time Preset register,              Address offset: 0x118 */
  __IO uint32_t TTTMK;          /*!< TT Time Mark register,                       Address offset: 0x11C */
  __IO uint32_t TTIR;           /*!< TT Interrupt register,                       Address offset: 0x120 */
  __IO uint32_t TTIE;           /*!< TT Interrupt Enable register,                Address offset: 0x124 */
  __IO uint32_t TTILS;          /*!< TT Interrupt Line Select register,           Address offset: 0x128 */
  __IO uint32_t TTOST;          /*!< TT Operation Status register,                Address offset: 0x12C */
  __IO uint32_t TURNA;          /*!< TT TUR Numerator Actual register,            Address offset: 0x130 */
  __IO uint32_t TTLGT;          /*!< TT Local and Global Time register,           Address offset: 0x134 */
  __IO uint32_t TTCTC;          /*!< TT Cycle Time and Count register,            Address offset: 0x138 */
  __IO uint32_t TTCPT;          /*!< TT Capture Time register,                    Address offset: 0x13C */
  __IO uint32_t TTCSM;          /*!< TT Cycle Sync Mark register,                 Address offset: 0x140 */
  __IO uint32_t RESERVED1[111]; /*!< Reserved,                                            0x144 - 0x2FC */
  __IO uint32_t TTTS;           /*!< TT Trigger Select register,                  Address offset: 0x300 */
} TTCAN_TypeDef;

/**
  * @brief FD Controller Area Network
  */

typedef struct
{
  __IO uint32_t CREL;  /*!< Clock Calibration Unit Core Release register, Address offset: 0x00 */
  __IO uint32_t CCFG;  /*!< Calibration Configuration register,           Address offset: 0x04 */
  __IO uint32_t CSTAT; /*!< Calibration Status register,                  Address offset: 0x08 */
  __IO uint32_t CWD;   /*!< Calibration Watchdog register,                Address offset: 0x0C */
  __IO uint32_t IR;    /*!< CCU Interrupt register,                       Address offset: 0x10 */
  __IO uint32_t IE;    /*!< CCU Interrupt Enable register,                Address offset: 0x14 */
} FDCAN_ClockCalibrationUnit_TypeDef;


/**
  * @brief Consumer Electronics Control
  */

typedef struct
{
  __IO uint32_t CR;           /*!< CEC control register,              Address offset:0x00 */
  __IO uint32_t CFGR;         /*!< CEC configuration register,        Address offset:0x04 */
  __IO uint32_t TXDR;         /*!< CEC Tx data register ,             Address offset:0x08 */
  __IO uint32_t RXDR;         /*!< CEC Rx Data Register,              Address offset:0x0C */
  __IO uint32_t ISR;          /*!< CEC Interrupt and Status Register, Address offset:0x10 */
  __IO uint32_t IER;          /*!< CEC interrupt enable register,     Address offset:0x14 */
}CEC_TypeDef;

/**
  * @brief CRC calculation unit
  */

typedef struct
{
  __IO uint32_t DR;          /*!< CRC Data register,                           Address offset: 0x00 */
  __IO uint32_t IDR;         /*!< CRC Independent data register,               Address offset: 0x04 */
  __IO uint32_t CR;          /*!< CRC Control register,                        Address offset: 0x08 */
  uint32_t      RESERVED2;   /*!< Reserved,                                                    0x0C */
  __IO uint32_t INIT;        /*!< Initial CRC value register,                  Address offset: 0x10 */
  __IO uint32_t POL;         /*!< CRC polynomial register,                     Address offset: 0x14 */
} CRC_TypeDef;


/**
  * @brief Clock Recovery System
  */
typedef struct
{
__IO uint32_t CR;            /*!< CRS ccontrol register,              Address offset: 0x00 */
__IO uint32_t CFGR;          /*!< CRS configuration register,         Address offset: 0x04 */
__IO uint32_t ISR;           /*!< CRS interrupt and status register,  Address offset: 0x08 */
__IO uint32_t ICR;           /*!< CRS interrupt flag clear register,  Address offset: 0x0C */
} CRS_TypeDef;


/**
  * @brief Digital to Analog Converter
  */

typedef struct
{
  __IO uint32_t CR;       /*!< DAC control register,                                    Address offset: 0x00 */
  __IO uint32_t SWTRIGR;  /*!< DAC software trigger register,                           Address offset: 0x04 */
  __IO uint32_t DHR12R1;  /*!< DAC channel1 12-bit right-aligned data holding register, Address offset: 0x08 */
  __IO uint32_t DHR12L1;  /*!< DAC channel1 12-bit left aligned data holding register,  Address offset: 0x0C */
  __IO uint32_t DHR8R1;   /*!< DAC channel1 8-bit right aligned data holding register,  Address offset: 0x10 */
  __IO uint32_t DHR12R2;  /*!< DAC channel2 12-bit right aligned data holding register, Address offset: 0x14 */
  __IO uint32_t DHR12L2;  /*!< DAC channel2 12-bit left aligned data holding register,  Address offset: 0x18 */
  __IO uint32_t DHR8R2;   /*!< DAC channel2 8-bit right-aligned data holding register,  Address offset: 0x1C */
  __IO uint32_t DHR12RD;  /*!< Dual DAC 12-bit right-aligned data holding register,     Address offset: 0x20 */
  __IO uint32_t DHR12LD;  /*!< DUAL DAC 12-bit left aligned data holding register,      Address offset: 0x24 */
  __IO uint32_t DHR8RD;   /*!< DUAL DAC 8-bit right aligned data holding register,      Address offset: 0x28 */
  __IO uint32_t DOR1;     /*!< DAC channel1 data output register,                       Address offset: 0x2C */
  __IO uint32_t DOR2;     /*!< DAC channel2 data output register,                       Address offset: 0x30 */
  __IO uint32_t SR;       /*!< DAC status register,                                     Address offset: 0x34 */
  __IO uint32_t CCR;      /*!< DAC calibration control register,                        Address offset: 0x38 */
  __IO uint32_t MCR;      /*!< DAC mode control register,                               Address offset: 0x3C */
  __IO uint32_t SHSR1;    /*!< DAC Sample and Hold sample time register 1,              Address offset: 0x40 */
  __IO uint32_t SHSR2;    /*!< DAC Sample and Hold sample time register 2,              Address offset: 0x44 */
  __IO uint32_t SHHR;     /*!< DAC Sample and Hold hold time register,                  Address offset: 0x48 */
  __IO uint32_t SHRR;     /*!< DAC Sample and Hold refresh time register,               Address offset: 0x4C */
} DAC_TypeDef;

/**
  * @brief DFSDM module registers
  */
typedef struct
{
  __IO uint32_t FLTCR1;          /*!< DFSDM control register1,                          Address offset: 0x100 */
  __IO uint32_t FLTCR2;          /*!< DFSDM control register2,                          Address offset: 0x104 */
  __IO uint32_t FLTISR;          /*!< DFSDM interrupt and status register,              Address offset: 0x108 */
  __IO uint32_t FLTICR;          /*!< DFSDM interrupt flag clear register,              Address offset: 0x10C */
  __IO uint32_t FLTJCHGR;        /*!< DFSDM injected channel group selection register,  Address offset: 0x110 */
  __IO uint32_t FLTFCR;          /*!< DFSDM filter control register,                    Address offset: 0x114 */
  __IO uint32_t FLTJDATAR;       /*!< DFSDM data register for injected group,           Address offset: 0x118 */
  __IO uint32_t FLTRDATAR;       /*!< DFSDM data register for regular group,            Address offset: 0x11C */
  __IO uint32_t FLTAWHTR;        /*!< DFSDM analog watchdog high threshold register,    Address offset: 0x120 */
  __IO uint32_t FLTAWLTR;        /*!< DFSDM analog watchdog low threshold register,     Address offset: 0x124 */
  __IO uint32_t FLTAWSR;         /*!< DFSDM analog watchdog status register             Address offset: 0x128 */
  __IO uint32_t FLTAWCFR;        /*!< DFSDM analog watchdog clear flag register         Address offset: 0x12C */
  __IO uint32_t FLTEXMAX;        /*!< DFSDM extreme detector maximum register,          Address offset: 0x130 */
  __IO uint32_t FLTEXMIN;        /*!< DFSDM extreme detector minimum register           Address offset: 0x134 */
  __IO uint32_t FLTCNVTIMR;      /*!< DFSDM conversion timer,                           Address offset: 0x138 */
} DFSDM_Filter_TypeDef;

/**
  * @brief DFSDM channel configuration registers
  */
typedef struct
{
  __IO uint32_t CHCFGR1;      /*!< DFSDM channel configuration register1,            Address offset: 0x00 */
  __IO uint32_t CHCFGR2;      /*!< DFSDM channel configuration register2,            Address offset: 0x04 */
  __IO uint32_t CHAWSCDR;     /*!< DFSDM channel analog watchdog and
                                   short circuit detector register,                  Address offset: 0x08 */
  __IO uint32_t CHWDATAR;     /*!< DFSDM channel watchdog filter data register,      Address offset: 0x0C */
  __IO uint32_t CHDATINR;     /*!< DFSDM channel data input register,                Address offset: 0x10 */
} DFSDM_Channel_TypeDef;

/**
  * @brief Debug MCU
  */

typedef struct
{
  __IO uint32_t IDCODE;        /*!< MCU device ID code,                     Address offset: 0x00 */
  __IO uint32_t CR;            /*!< Debug MCU configuration register,       Address offset: 0x04 */
  uint32_t RESERVED4[11];      /*!< Reserved,                             Address offset: 0x08 */
  __IO uint32_t APB3FZ1;     /*!< Debug MCU APB3FZ1 freeze register,    Address offset: 0x34 */
  uint32_t RESERVED5;          /*!< Reserved,                             Address offset: 0x38 */
  __IO uint32_t APB1LFZ1;    /*!< Debug MCU APB1LFZ1 freeze register,   Address offset: 0x3C */
  uint32_t RESERVED6;          /*!< Reserved,                             Address offset: 0x40 */
  __IO uint32_t APB1HFZ1;    /*!< Debug MCU APB1LFZ1 freeze register,   Address offset: 0x44 */
  uint32_t RESERVED7;          /*!< Reserved,                             Address offset: 0x48 */
  __IO uint32_t APB2FZ1;     /*!< Debug MCU APB2FZ1 freeze register,    Address offset: 0x4C */
  uint32_t RESERVED8;          /*!< Reserved,                             Address offset: 0x50 */
  __IO uint32_t APB4FZ1;     /*!< Debug MCU APB4FZ1 freeze register,    Address offset: 0x54 */
}DBGMCU_TypeDef;

/**
  * @brief DCMI
  */

typedef struct
{
  __IO uint32_t CR;       /*!< DCMI control register 1,                       Address offset: 0x00 */
  __IO uint32_t SR;       /*!< DCMI status register,                          Address offset: 0x04 */
  __IO uint32_t RISR;     /*!< DCMI raw interrupt status register,            Address offset: 0x08 */
  __IO uint32_t IER;      /*!< DCMI interrupt enable register,                Address offset: 0x0C */
  __IO uint32_t MISR;     /*!< DCMI masked interrupt status register,         Address offset: 0x10 */
  __IO uint32_t ICR;      /*!< DCMI interrupt clear register,                 Address offset: 0x14 */
  __IO uint32_t ESCR;     /*!< DCMI embedded synchronization code register,   Address offset: 0x18 */
  __IO uint32_t ESUR;     /*!< DCMI embedded synchronization unmask register, Address offset: 0x1C */
  __IO uint32_t CWSTRTR;  /*!< DCMI crop window start,                        Address offset: 0x20 */
  __IO uint32_t CWSIZER;  /*!< DCMI crop window size,                         Address offset: 0x24 */
  __IO uint32_t DR;       /*!< DCMI data register,                            Address offset: 0x28 */
} DCMI_TypeDef;

/**
  * @brief DMA Controller
  */

typedef struct
{
  __IO uint32_t CR;     /*!< DMA stream x configuration register      */
  __IO uint32_t NDTR;   /*!< DMA stream x number of data register     */
  __IO uint32_t PAR;    /*!< DMA stream x peripheral address register */
  __IO uint32_t M0AR;   /*!< DMA stream x memory 0 address register   */
  __IO uint32_t M1AR;   /*!< DMA stream x memory 1 address register   */
  __IO uint32_t FCR;    /*!< DMA stream x FIFO control register       */
} DMA_Stream_TypeDef;

typedef struct
{
  __IO uint32_t LISR;   /*!< DMA low interrupt status register,      Address offset: 0x00 */
  __IO uint32_t HISR;   /*!< DMA high interrupt status register,     Address offset: 0x04 */
  __IO uint32_t LIFCR;  /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
  __IO uint32_t HIFCR;  /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
} DMA_TypeDef;

typedef struct
{
  __IO uint32_t CCR;          /*!< DMA channel x configuration register        */
  __IO uint32_t CNDTR;        /*!< DMA channel x number of data register       */
  __IO uint32_t CPAR;         /*!< DMA channel x peripheral address register   */
  __IO uint32_t CMAR;         /*!< DMA channel x memory address register       */
} BDMA_Channel_TypeDef;

typedef struct
{
  __IO uint32_t ISR;          /*!< DMA interrupt status register,               Address offset: 0x00 */
  __IO uint32_t IFCR;         /*!< DMA interrupt flag clear register,           Address offset: 0x04 */
} BDMA_TypeDef;

typedef struct
{
  __IO uint32_t  CCR;        /*!< DMA Multiplexer Channel x Control Register   */
}DMAMUX_Channel_TypeDef;

typedef struct
{
  __IO uint32_t  CSR;      /*!< DMA Channel Status Register     */
  __IO uint32_t  CFR;      /*!< DMA Channel Clear Flag Register */
}DMAMUX_ChannelStatus_TypeDef;

typedef struct
{
  __IO uint32_t  RGCR;        /*!< DMA Request Generator x Control Register   */
}DMAMUX_RequestGen_TypeDef;

typedef struct
{
  __IO uint32_t  RGSR;        /*!< DMA Request Generator Status Register       */
  __IO uint32_t  RGCFR;       /*!< DMA Request Generator Clear Flag Register   */
}DMAMUX_RequestGenStatus_TypeDef;

/**
  * @brief MDMA Controller
  */
typedef struct
{
  __IO uint32_t  GISR0;   /*!< MDMA Global Interrupt/Status Register 0,          Address offset: 0x00 */
}MDMA_TypeDef;

typedef struct
{
  __IO uint32_t  CISR;      /*!< MDMA channel x interrupt/status register,             Address offset: 0x40 */
  __IO uint32_t  CIFCR;     /*!< MDMA channel x interrupt flag clear register,         Address offset: 0x44 */
  __IO uint32_t  CESR;      /*!< MDMA Channel x error status register,                 Address offset: 0x48 */
  __IO uint32_t  CCR;       /*!< MDMA channel x control register,                      Address offset: 0x4C */
  __IO uint32_t  CTCR;      /*!< MDMA channel x Transfer Configuration register,       Address offset: 0x50 */
  __IO uint32_t  CBNDTR;    /*!< MDMA Channel x block number of data register,         Address offset: 0x54 */
  __IO uint32_t  CSAR;      /*!< MDMA channel x source address register,               Address offset: 0x58 */
  __IO uint32_t  CDAR;      /*!< MDMA channel x destination address register,          Address offset: 0x5C */
  __IO uint32_t  CBRUR;     /*!< MDMA channel x Block Repeat address Update register,  Address offset: 0x60 */
  __IO uint32_t  CLAR;      /*!< MDMA channel x Link Address register,                 Address offset: 0x64 */
  __IO uint32_t  CTBR;      /*!< MDMA channel x Trigger and Bus selection Register,    Address offset: 0x68 */
  uint32_t       RESERVED0; /*!< Reserved, 0x68                                                             */
 __IO uint32_t    CMAR;      /*!< MDMA channel x Mask address register,                Address offset: 0x70 */
 __IO uint32_t   CMDR;       /*!< MDMA channel x Mask Data register,                   Address offset: 0x74 */
}MDMA_Channel_TypeDef;
/**
  * @brief DMA2D Controller
  */

typedef struct
{
  __IO uint32_t CR;            /*!< DMA2D Control Register,                         Address offset: 0x00 */
  __IO uint32_t ISR;           /*!< DMA2D Interrupt Status Register,                Address offset: 0x04 */
  __IO uint32_t IFCR;          /*!< DMA2D Interrupt Flag Clear Register,            Address offset: 0x08 */
  __IO uint32_t FGMAR;         /*!< DMA2D Foreground Memory Address Register,       Address offset: 0x0C */
  __IO uint32_t FGOR;          /*!< DMA2D Foreground Offset Register,               Address offset: 0x10 */
  __IO uint32_t BGMAR;         /*!< DMA2D Background Memory Address Register,       Address offset: 0x14 */
  __IO uint32_t BGOR;          /*!< DMA2D Background Offset Register,               Address offset: 0x18 */
  __IO uint32_t FGPFCCR;       /*!< DMA2D Foreground PFC Control Register,          Address offset: 0x1C */
  __IO uint32_t FGCOLR;        /*!< DMA2D Foreground Color Register,                Address offset: 0x20 */
  __IO uint32_t BGPFCCR;       /*!< DMA2D Background PFC Control Register,          Address offset: 0x24 */
  __IO uint32_t BGCOLR;        /*!< DMA2D Background Color Register,                Address offset: 0x28 */
  __IO uint32_t FGCMAR;        /*!< DMA2D Foreground CLUT Memory Address Register,  Address offset: 0x2C */
  __IO uint32_t BGCMAR;        /*!< DMA2D Background CLUT Memory Address Register,  Address offset: 0x30 */
  __IO uint32_t OPFCCR;        /*!< DMA2D Output PFC Control Register,              Address offset: 0x34 */
  __IO uint32_t OCOLR;         /*!< DMA2D Output Color Register,                    Address offset: 0x38 */
  __IO uint32_t OMAR;          /*!< DMA2D Output Memory Address Register,           Address offset: 0x3C */
  __IO uint32_t OOR;           /*!< DMA2D Output Offset Register,                   Address offset: 0x40 */
  __IO uint32_t NLR;           /*!< DMA2D Number of Line Register,                  Address offset: 0x44 */
  __IO uint32_t LWR;           /*!< DMA2D Line Watermark Register,                  Address offset: 0x48 */
  __IO uint32_t AMTCR;         /*!< DMA2D AHB Master Timer Configuration Register,  Address offset: 0x4C */
  uint32_t      RESERVED[236]; /*!< Reserved, 0x50-0x3FF */
  __IO uint32_t FGCLUT[256];   /*!< DMA2D Foreground CLUT,                          Address offset:400-7FF */
  __IO uint32_t BGCLUT[256];   /*!< DMA2D Background CLUT,                          Address offset:800-BFF */
} DMA2D_TypeDef;

/**
  * @brief Ethernet MAC
  */
typedef struct
{
  __IO uint32_t MACCR;
  __IO uint32_t MACECR;
  __IO uint32_t MACPFR;
  __IO uint32_t MACWTR;
  __IO uint32_t MACHT0R;
  __IO uint32_t MACHT1R;
  uint32_t      RESERVED1[14];
  __IO uint32_t MACVTR;
  uint32_t      RESERVED2;
  __IO uint32_t MACVHTR;
  uint32_t      RESERVED3;
  __IO uint32_t MACVIR;
  __IO uint32_t MACIVIR;
  uint32_t      RESERVED4[2];
  __IO uint32_t MACTFCR;
  uint32_t      RESERVED5[7];
  __IO uint32_t MACRFCR;
  uint32_t      RESERVED6[7];
  __IO uint32_t MACISR;
  __IO uint32_t MACIER;
  __IO uint32_t MACRXTXSR;
  uint32_t      RESERVED7;
  __IO uint32_t MACPCSR;
  __IO uint32_t MACRWKPFR;
  uint32_t      RESERVED8[2];
  __IO uint32_t MACLCSR;
  __IO uint32_t MACLTCR;
  __IO uint32_t MACLETR;
  __IO uint32_t MAC1USTCR;
  uint32_t      RESERVED9[12];
  __IO uint32_t MACVR;
  __IO uint32_t MACDR;
  uint32_t      RESERVED10;
  __IO uint32_t MACHWF0R;
  __IO uint32_t MACHWF1R;
  __IO uint32_t MACHWF2R;
  uint32_t      RESERVED11[54];
  __IO uint32_t MACMDIOAR;
  __IO uint32_t MACMDIODR;
  uint32_t      RESERVED12[2];
  __IO uint32_t MACARPAR;
  uint32_t      RESERVED13[59];
  __IO uint32_t MACA0HR;
  __IO uint32_t MACA0LR;
  __IO uint32_t MACA1HR;
  __IO uint32_t MACA1LR;
  __IO uint32_t MACA2HR;
  __IO uint32_t MACA2LR;
  __IO uint32_t MACA3HR;
  __IO uint32_t MACA3LR;
  uint32_t      RESERVED14[248];
  __IO uint32_t MMCCR;
  __IO uint32_t MMCRIR;
  __IO uint32_t MMCTIR;
  __IO uint32_t MMCRIMR;
  __IO uint32_t MMCTIMR;
  uint32_t      RESERVED15[14];
  __IO uint32_t MMCTSCGPR;
  __IO uint32_t MMCTMCGPR;
  int32_t       RESERVED16[5];
  __IO uint32_t MMCTPCGR;
  uint32_t      RESERVED17[10];
  __IO uint32_t MMCRCRCEPR;
  __IO uint32_t MMCRAEPR;
  uint32_t      RESERVED18[10];
  __IO uint32_t MMCRUPGR;
  uint32_t      RESERVED19[9];
  __IO uint32_t MMCTLPIMSTR;
  __IO uint32_t MMCTLPITCR;
  __IO uint32_t MMCRLPIMSTR;
  __IO uint32_t MMCRLPITCR;
  uint32_t      RESERVED20[65];
  __IO uint32_t MACL3L4C0R;
  __IO uint32_t MACL4A0R;
  uint32_t      RESERVED21[2];
  __IO uint32_t MACL3A0R0R;
  __IO uint32_t MACL3A1R0R;
  __IO uint32_t MACL3A2R0R;
  __IO uint32_t MACL3A3R0R;
  uint32_t      RESERVED22[4];
  __IO uint32_t MACL3L4C1R;
  __IO uint32_t MACL4A1R;
  uint32_t      RESERVED23[2];
  __IO uint32_t MACL3A0R1R;
  __IO uint32_t MACL3A1R1R;
  __IO uint32_t MACL3A2R1R;
  __IO uint32_t MACL3A3R1R;
  uint32_t      RESERVED24[108];
  __IO uint32_t MACTSCR;
  __IO uint32_t MACSSIR;
  __IO uint32_t MACSTSR;
  __IO uint32_t MACSTNR;
  __IO uint32_t MACSTSUR;
  __IO uint32_t MACSTNUR;
  __IO uint32_t MACTSAR;
  uint32_t      RESERVED25;
  __IO uint32_t MACTSSR;
  uint32_t      RESERVED26[3];
  __IO uint32_t MACTTSSNR;
  __IO uint32_t MACTTSSSR;
  uint32_t      RESERVED27[2];
  __IO uint32_t MACACR;
  uint32_t      RESERVED28;
  __IO uint32_t MACATSNR;
  __IO uint32_t MACATSSR;
  __IO uint32_t MACTSIACR;
  __IO uint32_t MACTSEACR;
  __IO uint32_t MACTSICNR;
  __IO uint32_t MACTSECNR;
  uint32_t      RESERVED29[4];
  __IO uint32_t MACPPSCR;
  uint32_t      RESERVED30[3];
  __IO uint32_t MACPPSTTSR;
  __IO uint32_t MACPPSTTNR;
  __IO uint32_t MACPPSIR;
  __IO uint32_t MACPPSWR;
  uint32_t      RESERVED31[12];
  __IO uint32_t MACPOCR;
  __IO uint32_t MACSPI0R;
  __IO uint32_t MACSPI1R;
  __IO uint32_t MACSPI2R;
  __IO uint32_t MACLMIR;
  uint32_t      RESERVED32[11];
  __IO uint32_t MTLOMR;
  uint32_t      RESERVED33[7];
  __IO uint32_t MTLISR;
  uint32_t      RESERVED34[55];
  __IO uint32_t MTLTQOMR;
  __IO uint32_t MTLTQUR;
  __IO uint32_t MTLTQDR;
  uint32_t      RESERVED35[8];
  __IO uint32_t MTLQICSR;
  __IO uint32_t MTLRQOMR;
  __IO uint32_t MTLRQMPOCR;
  __IO uint32_t MTLRQDR;
  uint32_t      RESERVED36[177];
  __IO uint32_t DMAMR;
  __IO uint32_t DMASBMR;
  __IO uint32_t DMAISR;
  __IO uint32_t DMADSR;
  uint32_t      RESERVED37[60];
  __IO uint32_t DMACCR;
  __IO uint32_t DMACTCR;
  __IO uint32_t DMACRCR;
  uint32_t      RESERVED38[2];
  __IO uint32_t DMACTDLAR;
  uint32_t      RESERVED39;
  __IO uint32_t DMACRDLAR;
  __IO uint32_t DMACTDTPR;
  uint32_t      RESERVED40;
  __IO uint32_t DMACRDTPR;
  __IO uint32_t DMACTDRLR;
  __IO uint32_t DMACRDRLR;
  __IO uint32_t DMACIER;
  __IO uint32_t DMACRIWTR;
__IO uint32_t DMACSFCSR;
  uint32_t      RESERVED41;
  __IO uint32_t DMACCATDR;
  uint32_t      RESERVED42;
  __IO uint32_t DMACCARDR;
  uint32_t      RESERVED43;
  __IO uint32_t DMACCATBR;
  uint32_t      RESERVED44;
  __IO uint32_t DMACCARBR;
  __IO uint32_t DMACSR;
uint32_t      RESERVED45[2];
__IO uint32_t DMACMFCR;
}ETH_TypeDef;

/**
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
__IO uint32_t RTSR1;               /*!< EXTI Rising trigger selection register,       Address offset: 0x00 */
__IO uint32_t FTSR1;               /*!< EXTI Falling trigger selection register,      Address offset: 0x04 */
__IO uint32_t SWIER1;              /*!< EXTI Software interrupt event register,       Address offset: 0x08 */
__IO uint32_t D3PMR1;              /*!< EXTI D3 Pending mask register,                Address offset: 0x0C */
__IO uint32_t D3PCR1L;             /*!< EXTI D3 Pending clear selection register low, Address offset: 0x10 */
__IO uint32_t D3PCR1H;             /*!< EXTI D3 Pending clear selection register High,Address offset: 0x14 */
uint32_t      RESERVED1;           /*!< Reserved, 0x18                                                     */
uint32_t      RESERVED2;           /*!< Reserved, 0x1C                                                     */
__IO uint32_t RTSR2;               /*!< EXTI Rising trigger selection register,       Address offset: 0x20 */
__IO uint32_t FTSR2;               /*!< EXTI Falling trigger selection register,      Address offset: 0x24 */
__IO uint32_t SWIER2;              /*!< EXTI Software interrupt event register,       Address offset: 0x28 */
__IO uint32_t D3PMR2;              /*!< EXTI D3 Pending mask register,                Address offset: 0x2C */
__IO uint32_t D3PCR2L;             /*!< EXTI D3 Pending clear selection register low, Address offset: 0x30 */
__IO uint32_t D3PCR2H;             /*!< EXTI D3 Pending clear selection register High,Address offset: 0x34 */
uint32_t      RESERVED3;           /*!< Reserved, 0x38                                                     */
uint32_t      RESERVED4;           /*!< Reserved, 0x3C                                                     */
__IO uint32_t RTSR3;               /*!< EXTI Rising trigger selection register,       Address offset: 0x40 */
__IO uint32_t FTSR3;               /*!< EXTI Falling trigger selection register,      Address offset: 0x44 */
__IO uint32_t SWIER3;              /*!< EXTI Software interrupt event register,       Address offset: 0x48 */
__IO uint32_t D3PMR3;              /*!< EXTI D3 Pending mask register,                Address offset: 0x4C */
__IO uint32_t D3PCR3L;             /*!< EXTI D3 Pending clear selection register low, Address offset: 0x50 */
__IO uint32_t D3PCR3H;             /*!< EXTI D3 Pending clear selection register High,Address offset: 0x54 */
}EXTI_TypeDef;

typedef struct
{
__IO uint32_t IMR1;                /*!< EXTI Interrupt mask register,                Address offset: 0x00 */
__IO uint32_t EMR1;                /*!< EXTI Event mask register,                    Address offset: 0x04 */
__IO uint32_t PR1;                 /*!< EXTI Pending register,                       Address offset: 0x08 */
uint32_t      RESERVED1;           /*!< Reserved, 0x0C                                                    */
__IO uint32_t IMR2;                /*!< EXTI Interrupt mask register,                Address offset: 0x10 */
__IO uint32_t EMR2;                /*!< EXTI Event mask register,                    Address offset: 0x14 */
__IO uint32_t PR2;                 /*!< EXTI Pending register,                       Address offset: 0x18 */
uint32_t      RESERVED2;           /*!< Reserved, 0x1C                                                    */
__IO uint32_t IMR3;                /*!< EXTI Interrupt mask register,                Address offset: 0x20 */
__IO uint32_t EMR3;                /*!< EXTI Event mask register,                    Address offset: 0x24 */
__IO uint32_t PR3;                 /*!< EXTI Pending register,                       Address offset: 0x28 */
}EXTI_Core_TypeDef;


/**
  * @brief FLASH Registers
  */

typedef struct
{
  __IO uint32_t ACR;             /*!< FLASH access control register,                           Address offset: 0x00 */
  __IO uint32_t KEYR1;           /*!< Flash Key Register for bank1,                            Address offset: 0x04 */
  __IO uint32_t OPTKEYR;         /*!< Flash Option Key Register,                                Address offset: 0x08 */
  __IO uint32_t CR1;             /*!< Flash Control Register for bank1,                        Address offset: 0x0C */
  __IO uint32_t SR1;             /*!< Flash Status Register for bank1,                         Address offset: 0x10 */
  __IO uint32_t CCR1;            /*!< Flash Control Register for bank1,                        Address offset: 0x14 */
  __IO uint32_t OPTCR;           /*!< Flash Option Control Register,                            Address offset: 0x18 */
  __IO uint32_t OPTSR_CUR;       /*!< Flash Option Status Current Register,                     Address offset: 0x1C */
  __IO uint32_t OPTSR_PRG;       /*!< Flash Option Status Current Register,                     Address offset: 0x20 */
  __IO uint32_t OPTCCR;          /*!< Flash Option Clear Control Register,                      Address offset: 0x24 */
  __IO uint32_t PRAR_CUR1;       /*!< Flash Current Protection Address Register for bank1,     Address offset: 0x28 */
  __IO uint32_t PRAR_PRG1;       /*!< Flash Protection Address to Program Register for bank1,  Address offset: 0x2C */
  __IO uint32_t SCAR_CUR1;       /*!< Flash Current Secure Address Register for bank1,         Address offset: 0x30 */
  __IO uint32_t SCAR_PRG1;       /*!< Flash Secure Address Register for bank1,                 Address offset: 0x34 */
  __IO uint32_t WPSN_CUR1;       /*!< Flash Current Write Protection Register on bank1,        Address offset: 0x38 */
  __IO uint32_t WPSN_PRG1;       /*!< Flash Write Protection to Program Register on bank1,     Address offset: 0x3C */
  __IO uint32_t BOOT_CUR;        /*!< Flash Current Boot Address for Pelican Core Register,     Address offset: 0x40 */
  __IO uint32_t BOOT_PRG;        /*!< Flash Boot Address to Program for Pelican Core Register,  Address offset: 0x44 */
  uint32_t      RESERVED0[2];    /*!< Reserved, 0x48 to 0x4C                                                        */
  __IO uint32_t CRCCR1;          /*!< Flash CRC Control register For Bank1 Register ,          Address offset: 0x50 */
  __IO uint32_t CRCSADD1;        /*!< Flash CRC Start Address Register for Bank1 ,             Address offset: 0x54 */
  __IO uint32_t CRCEADD1;        /*!< Flash CRC End Address Register for Bank1 ,               Address offset: 0x58 */
  __IO uint32_t CRCDATA;         /*!< Flash CRC Data Register for Bank1 ,                      Address offset: 0x5C */
  __IO uint32_t ECC_FA1;         /*!< Flash ECC Fail Address For Bank1 Register ,              Address offset: 0x60 */
  uint32_t      RESERVED1[40];   /*!< Reserved, 0x64 to 0x100                                                       */
  __IO uint32_t KEYR2;           /*!< Flash Key Register for bank2,                           Address offset: 0x104 */
  uint32_t      RESERVED2;       /*!< Reserved, 0x108                                                               */
  __IO uint32_t CR2;             /*!< Flash Control Register for bank2,                       Address offset: 0x10C */
  __IO uint32_t SR2;             /*!< Flash Status Register for bank2,                        Address offset: 0x110 */
  __IO uint32_t CCR2;            /*!< Flash Status Register for bank2,                        Address offset: 0x114 */
  uint32_t      RESERVED3[4];    /*!< Reserved, 0x118 to 0x124                                                      */
  __IO uint32_t PRAR_CUR2;       /*!< Flash Current Protection Address Register for bank2,    Address offset: 0x128 */
  __IO uint32_t PRAR_PRG2;       /*!< Flash Protection Address to Program Register for bank2, Address offset: 0x12C */
  __IO uint32_t SCAR_CUR2;       /*!< Flash Current Secure Address Register for bank2,        Address offset: 0x130 */
  __IO uint32_t SCAR_PRG2;       /*!< Flash Secure Address Register for bank2,                Address offset: 0x134 */
  __IO uint32_t WPSN_CUR2;       /*!< Flash Current Write Protection Register on bank2,       Address offset: 0x138 */
  __IO uint32_t WPSN_PRG2;       /*!< Flash Write Protection to Program Register on bank2,    Address offset: 0x13C */
  uint32_t      RESERVED4[4];    /*!< Reserved, 0x140 to 0x14C                                                      */
  __IO uint32_t CRCCR2;          /*!< Flash CRC Control register For Bank2 Register ,         Address offset: 0x150 */
  __IO uint32_t CRCSADD2;        /*!< Flash CRC Start Address Register for Bank2 ,            Address offset: 0x154 */
  __IO uint32_t CRCEADD2;        /*!< Flash CRC End Address Register for Bank2 ,              Address offset: 0x158 */
  __IO uint32_t CRCDATA2;        /*!< Flash CRC Data Register for Bank2 ,                     Address offset: 0x15C */
  __IO uint32_t ECC_FA2;         /*!< Flash ECC Fail Address For Bank2 Register ,             Address offset: 0x160 */
} FLASH_TypeDef;

/**
  * @brief Flexible Memory Controller
  */

typedef struct
{
  __IO uint32_t BTCR[8];    /*!< NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C */
} FMC_Bank1_TypeDef;

/**
  * @brief Flexible Memory Controller Bank1E
  */

typedef struct
{
  __IO uint32_t BWTR[7];    /*!< NOR/PSRAM write timing registers, Address offset: 0x104-0x11C */
} FMC_Bank1E_TypeDef;

/**
  * @brief Flexible Memory Controller Bank2
  */

typedef struct
{
  __IO uint32_t PCR2;       /*!< NAND Flash control register 2,                       Address offset: 0x60 */
  __IO uint32_t SR2;        /*!< NAND Flash FIFO status and interrupt register 2,     Address offset: 0x64 */
  __IO uint32_t PMEM2;      /*!< NAND Flash Common memory space timing register 2,    Address offset: 0x68 */
  __IO uint32_t PATT2;      /*!< NAND Flash Attribute memory space timing register 2, Address offset: 0x6C */
  uint32_t      RESERVED0;  /*!< Reserved, 0x70                                                            */
  __IO uint32_t ECCR2;      /*!< NAND Flash ECC result registers 2,                   Address offset: 0x74 */
} FMC_Bank2_TypeDef;

/**
  * @brief Flexible Memory Controller Bank3
  */

typedef struct
{
  __IO uint32_t PCR;       /*!< NAND Flash control register 3,                       Address offset: 0x80 */
  __IO uint32_t SR;        /*!< NAND Flash FIFO status and interrupt register 3,     Address offset: 0x84 */
  __IO uint32_t PMEM;      /*!< NAND Flash Common memory space timing register 3,    Address offset: 0x88 */
  __IO uint32_t PATT;      /*!< NAND Flash Attribute memory space timing register 3, Address offset: 0x8C */
  uint32_t      RESERVED;  /*!< Reserved, 0x90                                                            */
  __IO uint32_t ECCR;      /*!< NAND Flash ECC result registers 3,                   Address offset: 0x94 */
} FMC_Bank3_TypeDef;

/**
  * @brief Flexible Memory Controller Bank5 and 6
  */


typedef struct
{
  __IO uint32_t SDCR[2];        /*!< SDRAM Control registers ,      Address offset: 0x140-0x144  */
  __IO uint32_t SDTR[2];        /*!< SDRAM Timing registers ,       Address offset: 0x148-0x14C  */
  __IO uint32_t SDCMR;       /*!< SDRAM Command Mode register,    Address offset: 0x150  */
  __IO uint32_t SDRTR;       /*!< SDRAM Refresh Timer register,   Address offset: 0x154  */
  __IO uint32_t SDSR;        /*!< SDRAM Status register,          Address offset: 0x158  */
} FMC_Bank5_6_TypeDef;

/**
  * @brief General Purpose I/O
  */

typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint16_t BSRRL;    /*!< GPIO port bit set/reset low register,  Address offset: 0x18      */
  __IO uint16_t BSRRH;    /*!< GPIO port bit set/reset high register, Address offset: 0x1A      */
  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

/**
  * @brief Operational Amplifier (OPAMP)
  */

typedef struct
{
  __IO uint32_t CSR;          /*!< OPAMP control/status register,                     Address offset: 0x00 */
  __IO uint32_t OTR;          /*!< OPAMP offset trimming register for normal mode,    Address offset: 0x04 */
  __IO uint32_t HSOTR;        /*!< OPAMP offset trimming register for high speed mode, Address offset: 0x08 */
} OPAMP_TypeDef;

/**
  * @brief System configuration controller
  */

typedef struct
{
 uint32_t RESERVED1;           /*!< Reserved,                                           Address offset: 0x00        */
 __IO uint32_t PMCR;           /*!< SYSCFG peripheral mode configuration register,      Address offset: 0x04        */
 __IO uint32_t EXTICR[4];      /*!< SYSCFG external interrupt configuration registers,  Address offset: 0x08-0x14   */
 uint32_t RESERVED2[2];        /*!< Reserved,                                            Address offset: 0x18-0x1C  */
 __IO uint32_t CCCSR;          /*!< SYSCFG compensation cell control/status register,   Address offset: 0x20        */
 __IO uint32_t CCVR;           /*!< SYSCFG compensation cell value register,            Address offset: 0x24        */
 __IO uint32_t CCCR;           /*!< SYSCFG compensation cell code register,             Address offset: 0x28        */
  uint32_t     RESERVED3[62];  /*!< Reserved, 0x2C-0x120                                                            */
 __IO uint32_t PKGR;           /*!< SYSCFG package register,                            Address offset: 0x124       */
  uint32_t     RESERVED4[118]; /*!< Reserved, 0x128-0x2FC                                                           */
 __IO uint32_t UR0;            /*!< SYSCFG user register 0,                             Address offset: 0x300       */
 __IO uint32_t UR1;            /*!< SYSCFG user register 1,                             Address offset: 0x304       */
 __IO uint32_t UR2;            /*!< SYSCFG user register 2,                             Address offset: 0x308       */
 __IO uint32_t UR3;            /*!< SYSCFG user register 3,                             Address offset: 0x30C       */
 __IO uint32_t UR4;            /*!< SYSCFG user register 4,                             Address offset: 0x310       */
 __IO uint32_t UR5;            /*!< SYSCFG user register 5,                             Address offset: 0x314       */
 __IO uint32_t UR6;            /*!< SYSCFG user register 6,                             Address offset: 0x318       */
 __IO uint32_t UR7;            /*!< SYSCFG user register 7,                             Address offset: 0x31C       */
 __IO uint32_t UR8;            /*!< SYSCFG user register 8,                             Address offset: 0x320       */
 __IO uint32_t UR9;            /*!< SYSCFG user register 9,                             Address offset: 0x324       */
 __IO uint32_t UR10;           /*!< SYSCFG user register 10,                            Address offset: 0x328       */
 __IO uint32_t UR11;           /*!< SYSCFG user register 11,                            Address offset: 0x32C       */
 __IO uint32_t UR12;           /*!< SYSCFG user register 12,                            Address offset: 0x330       */
 __IO uint32_t UR13;           /*!< SYSCFG user register 13,                            Address offset: 0x334       */
 __IO uint32_t UR14;           /*!< SYSCFG user register 14,                            Address offset: 0x338       */
 __IO uint32_t UR15;           /*!< SYSCFG user register 15,                            Address offset: 0x33C       */
 __IO uint32_t UR16;           /*!< SYSCFG user register 16,                            Address offset: 0x340       */
 __IO uint32_t UR17;           /*!< SYSCFG user register 17,                            Address offset: 0x344       */

} SYSCFG_TypeDef;

/**
  * @brief Inter-integrated Circuit Interface
  */

typedef struct
{
  __IO uint32_t CR1;      /*!< I2C Control register 1,            Address offset: 0x00 */
  __IO uint32_t CR2;      /*!< I2C Control register 2,            Address offset: 0x04 */
  __IO uint32_t OAR1;     /*!< I2C Own address 1 register,        Address offset: 0x08 */
  __IO uint32_t OAR2;     /*!< I2C Own address 2 register,        Address offset: 0x0C */
  __IO uint32_t TIMINGR;  /*!< I2C Timing register,               Address offset: 0x10 */
  __IO uint32_t TIMEOUTR; /*!< I2C Timeout register,              Address offset: 0x14 */
  __IO uint32_t ISR;      /*!< I2C Interrupt and status register, Address offset: 0x18 */
  __IO uint32_t ICR;      /*!< I2C Interrupt clear register,      Address offset: 0x1C */
  __IO uint32_t PECR;     /*!< I2C PEC register,                  Address offset: 0x20 */
  __IO uint32_t RXDR;     /*!< I2C Receive data register,         Address offset: 0x24 */
  __IO uint32_t TXDR;     /*!< I2C Transmit data register,        Address offset: 0x28 */
} I2C_TypeDef;

/**
  * @brief Independent WATCHDOG
  */

typedef struct
{
  __IO uint32_t KR;   /*!< IWDG Key register,       Address offset: 0x00 */
  __IO uint32_t PR;   /*!< IWDG Prescaler register, Address offset: 0x04 */
  __IO uint32_t RLR;  /*!< IWDG Reload register,    Address offset: 0x08 */
  __IO uint32_t SR;   /*!< IWDG Status register,    Address offset: 0x0C */
  __IO uint32_t WINR; /*!< IWDG Window register,    Address offset: 0x10 */
} IWDG_TypeDef;


/**
  * @brief JPEG Codec
  */
typedef struct
{
  __IO uint32_t CONFR0;          /*!< JPEG Codec Control Register (JPEG_CONFR0),        Address offset: 00h       */
  __IO uint32_t CONFR1;          /*!< JPEG Codec Control Register (JPEG_CONFR1),        Address offset: 04h       */
  __IO uint32_t CONFR2;          /*!< JPEG Codec Control Register (JPEG_CONFR2),        Address offset: 08h       */
  __IO uint32_t CONFR3;          /*!< JPEG Codec Control Register (JPEG_CONFR3),        Address offset: 0Ch       */
  __IO uint32_t CONFR4;          /*!< JPEG Codec Control Register (JPEG_CONFR4),        Address offset: 10h       */
  __IO uint32_t CONFR5;          /*!< JPEG Codec Control Register (JPEG_CONFR5),        Address offset: 14h       */
  __IO uint32_t CONFR6;          /*!< JPEG Codec Control Register (JPEG_CONFR6),        Address offset: 18h       */
  __IO uint32_t CONFR7;          /*!< JPEG Codec Control Register (JPEG_CONFR7),        Address offset: 1Ch       */
  uint32_t  Reserved20[4];       /* Reserved                                            Address offset: 20h-2Ch   */
  __IO uint32_t CR;              /*!< JPEG Control Register (JPEG_CR),                  Address offset: 30h       */
  __IO uint32_t SR;              /*!< JPEG Status Register (JPEG_SR),                   Address offset: 34h       */
  __IO uint32_t CFR;             /*!< JPEG Clear Flag Register (JPEG_CFR),              Address offset: 38h       */
  uint32_t  Reserved3c;          /* Reserved                                            Address offset: 3Ch       */
  __IO uint32_t DIR;             /*!< JPEG Data Input Register (JPEG_DIR),              Address offset: 40h       */
  __IO uint32_t DOR;             /*!< JPEG Data Output Register (JPEG_DOR),             Address offset: 44h       */
  uint32_t  Reserved48[2];       /* Reserved                                            Address offset: 48h-4Ch   */
  __IO uint32_t QMEM0[16];       /*!< JPEG quantization tables 0,                       Address offset: 50h-8Ch   */
  __IO uint32_t QMEM1[16];       /*!< JPEG quantization tables 1,                       Address offset: 90h-CCh   */
  __IO uint32_t QMEM2[16];       /*!< JPEG quantization tables 2,                       Address offset: D0h-10Ch  */
  __IO uint32_t QMEM3[16];       /*!< JPEG quantization tables 3,                       Address offset: 110h-14Ch */
  __IO uint32_t HUFFMIN[16];     /*!< JPEG HuffMin tables,                              Address offset: 150h-18Ch */
  __IO uint32_t HUFFBASE[32];    /*!< JPEG HuffSymb tables,                             Address offset: 190h-20Ch */
  __IO uint32_t HUFFSYMB[84];    /*!< JPEG HUFFSYMB tables,                             Address offset: 210h-35Ch */
  __IO uint32_t DHTMEM[103];     /*!< JPEG DHTMem tables,                               Address offset: 360h-4F8h */
  uint32_t  Reserved4FC;         /* Reserved                                            Address offset: 4FCh      */
  __IO uint32_t HUFFENC_AC0[88]; /*!< JPEG encodor, AC Huffman table 0,                 Address offset: 500h-65Ch */
  __IO uint32_t HUFFENC_AC1[88]; /*!< JPEG encodor, AC Huffman table 1,                 Address offset: 660h-7BCh */
  __IO uint32_t HUFFENC_DC0[8];  /*!< JPEG encodor, DC Huffman table 0,                 Address offset: 7C0h-7DCh */
  __IO uint32_t HUFFENC_DC1[8];  /*!< JPEG encodor, DC Huffman table 1,                 Address offset: 7E0h-7FCh */

} JPEG_TypeDef;


/**
  * @brief LCD-TFT Display Controller
  */

typedef struct
{
  uint32_t      RESERVED0[2];  /*!< Reserved, 0x00-0x04 */
  __IO uint32_t SSCR;          /*!< LTDC Synchronization Size Configuration Register,    Address offset: 0x08 */
  __IO uint32_t BPCR;          /*!< LTDC Back Porch Configuration Register,              Address offset: 0x0C */
  __IO uint32_t AWCR;          /*!< LTDC Active Width Configuration Register,            Address offset: 0x10 */
  __IO uint32_t TWCR;          /*!< LTDC Total Width Configuration Register,             Address offset: 0x14 */
  __IO uint32_t GCR;           /*!< LTDC Global Control Register,                        Address offset: 0x18 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x1C-0x20 */
  __IO uint32_t SRCR;          /*!< LTDC Shadow Reload Configuration Register,           Address offset: 0x24 */
  uint32_t      RESERVED2[1];  /*!< Reserved, 0x28 */
  __IO uint32_t BCCR;          /*!< LTDC Background Color Configuration Register,        Address offset: 0x2C */
  uint32_t      RESERVED3[1];  /*!< Reserved, 0x30 */
  __IO uint32_t IER;           /*!< LTDC Interrupt Enable Register,                      Address offset: 0x34 */
  __IO uint32_t ISR;           /*!< LTDC Interrupt Status Register,                      Address offset: 0x38 */
  __IO uint32_t ICR;           /*!< LTDC Interrupt Clear Register,                       Address offset: 0x3C */
  __IO uint32_t LIPCR;         /*!< LTDC Line Interrupt Position Configuration Register, Address offset: 0x40 */
  __IO uint32_t CPSR;          /*!< LTDC Current Position Status Register,               Address offset: 0x44 */
  __IO uint32_t CDSR;         /*!< LTDC Current Display Status Register,                       Address offset: 0x48 */
} LTDC_TypeDef;

/**
  * @brief LCD-TFT Display layer x Controller
  */

typedef struct
{
  __IO uint32_t CR;            /*!< LTDC Layerx Control Register                                  Address offset: 0x84 */
  __IO uint32_t WHPCR;         /*!< LTDC Layerx Window Horizontal Position Configuration Register Address offset: 0x88 */
  __IO uint32_t WVPCR;         /*!< LTDC Layerx Window Vertical Position Configuration Register   Address offset: 0x8C */
  __IO uint32_t CKCR;          /*!< LTDC Layerx Color Keying Configuration Register               Address offset: 0x90 */
  __IO uint32_t PFCR;          /*!< LTDC Layerx Pixel Format Configuration Register               Address offset: 0x94 */
  __IO uint32_t CACR;          /*!< LTDC Layerx Constant Alpha Configuration Register             Address offset: 0x98 */
  __IO uint32_t DCCR;          /*!< LTDC Layerx Default Color Configuration Register              Address offset: 0x9C */
  __IO uint32_t BFCR;          /*!< LTDC Layerx Blending Factors Configuration Register           Address offset: 0xA0 */
  uint32_t      RESERVED0[2];  /*!< Reserved */
  __IO uint32_t CFBAR;         /*!< LTDC Layerx Color Frame Buffer Address Register               Address offset: 0xAC */
  __IO uint32_t CFBLR;         /*!< LTDC Layerx Color Frame Buffer Length Register                Address offset: 0xB0 */
  __IO uint32_t CFBLNR;        /*!< LTDC Layerx ColorFrame Buffer Line Number Register            Address offset: 0xB4 */
  uint32_t      RESERVED1[3];  /*!< Reserved */
  __IO uint32_t CLUTWR;         /*!< LTDC Layerx CLUT Write Register                               Address offset: 0x144 */

} LTDC_Layer_TypeDef;


/**
  * @brief Power Control
  */

typedef struct
{
  __IO uint32_t CR1;          /*!< PWR power control register 1,        Address offset: 0x00 */
  __IO uint32_t CSR1;      /*!< PWR power control status register 1,     Address offset: 0x04 */
  __IO uint32_t CR2;       /*!< PWR power control register 2,            Address offset: 0x08 */
  __IO uint32_t CR3;       /*!< PWR power control register 3,            Address offset: 0x0C */
  __IO uint32_t CPUCR;     /*!< PWR CPU control register,                Address offset: 0x10 */
       uint32_t RESERVED0; /*!< Reserved,                                Address offset: 0x14 */
  __IO uint32_t D3CR;      /*!< PWR D3 domain control register,          Address offset: 0x18 */
       uint32_t RESERVED1; /*!< Reserved,                                Address offset: 0x1C */
  __IO uint32_t WKUPCR;    /*!< PWR wakeup clear register,               Address offset: 0x20 */
  __IO uint32_t WKUPFR;    /*!< PWR wakeup flag register,                Address offset: 0x24 */
  __IO uint32_t WKUPEPR;   /*!< PWR wakeup enable and polarity register, Address offset: 0x28 */
} PWR_TypeDef;

/**
  * @brief Reset and Clock Control
  */

typedef struct
{
 __IO uint32_t CR;             /*!< RCC clock control register,                                              Address offset: 0x00  */
 __IO uint32_t ICSCR;          /*!< RCC Internal Clock Sources Calibration Register,                         Address offset: 0x04  */
 __IO uint32_t CRRCR;          /*!< Clock Recovery RC  Register,                                             Address offset: 0x08  */
 uint32_t     RESERVED0;       /*!< Reserved,                                                                Address offset: 0x0C  */
 __IO uint32_t CFGR;           /*!< RCC clock configuration register,                                        Address offset: 0x10  */
 uint32_t     RESERVED1;       /*!< Reserved,                                                                Address offset: 0x14  */
 __IO uint32_t D1CFGR;         /*!< RCC Domain 1 configuration register,                                     Address offset: 0x18  */
 __IO uint32_t D2CFGR;         /*!< RCC Domain 2 configuration register,                                     Address offset: 0x1C  */
 __IO uint32_t D3CFGR;         /*!< RCC Domain 3 configuration register,                                     Address offset: 0x20  */
 uint32_t     RESERVED2;       /*!< Reserved,                                                                Address offset: 0x24  */
 __IO uint32_t PLLCKSELR;      /*!< RCC PLLs Clock Source Selection Register,                                Address offset: 0x28  */
 __IO uint32_t PLLCFGR;        /*!< RCC PLLs  Configuration Register,                                        Address offset: 0x2C  */
 __IO uint32_t PLL1DIVR;       /*!< RCC PLL1 Dividers Configuration Register,                                Address offset: 0x30  */
 __IO uint32_t PLL1FRACR;      /*!< RCC PLL1 Fractional Divider Configuration Register,                      Address offset: 0x34  */
 __IO uint32_t PLL2DIVR;       /*!< RCC PLL2 Dividers Configuration Register,                                Address offset: 0x38  */
 __IO uint32_t PLL2FRACR;      /*!< RCC PLL2 Fractional Divider Configuration Register,                      Address offset: 0x3C  */
 __IO uint32_t PLL3DIVR;       /*!< RCC PLL3 Dividers Configuration Register,                                Address offset: 0x40  */
 __IO uint32_t PLL3FRACR;      /*!< RCC PLL3 Fractional Divider Configuration Register,                      Address offset: 0x44  */
 uint32_t      RESERVED3;      /*!< Reserved,                                                                Address offset: 0x48  */
 __IO uint32_t  D1CCIPR;       /*!< RCC Domain 1 Kernel Clock Configuration Register                         Address offset: 0x4C  */
 __IO uint32_t  D2CCIP1R;      /*!< RCC Domain 2 Kernel Clock Configuration Register                         Address offset: 0x50  */
 __IO uint32_t  D2CCIP2R;      /*!< RCC Domain 2 Kernel Clock Configuration Register                         Address offset: 0x54  */
 __IO uint32_t  D3CCIPR;       /*!< RCC Domain 3 Kernel Clock Configuration Register                         Address offset: 0x58  */
 uint32_t      RESERVED4;      /*!< Reserved,                                                                Address offset: 0x5C  */
 __IO uint32_t  CIER;          /*!< RCC Clock Source Interrupt Enable Register                               Address offset: 0x60  */
 __IO uint32_t  CIFR;          /*!< RCC Clock Source Interrupt Flag Register                                 Address offset: 0x64  */
 __IO uint32_t  CICR;          /*!< RCC Clock Source Interrupt Clear Register                                Address offset: 0x68  */
 uint32_t     RESERVED5;       /*!< Reserved,                                                                Address offset: 0x6C  */
 __IO uint32_t  BDCR;          /*!< RCC Vswitch Backup Domain Control Register,                              Address offset: 0x70  */
 __IO uint32_t  CSR;           /*!< RCC clock control & status register,                                     Address offset: 0x74  */
 uint32_t     RESERVED6;       /*!< Reserved,                                                                Address offset: 0x78  */
 __IO uint32_t AHB3RSTR;       /*!< RCC AHB3 peripheral reset register,                                      Address offset: 0x7C  */
 __IO uint32_t AHB1RSTR;       /*!< RCC AHB1 peripheral reset register,                                      Address offset: 0x80  */
 __IO uint32_t AHB2RSTR;       /*!< RCC AHB2 peripheral reset register,                                      Address offset: 0x84  */
 __IO uint32_t AHB4RSTR;       /*!< RCC AHB4 peripheral reset register,                                      Address offset: 0x88  */
 __IO uint32_t APB3RSTR;       /*!< RCC APB3 peripheral reset register,                                      Address offset: 0x8C  */
 __IO uint32_t APB1LRSTR;      /*!< RCC APB1 peripheral reset Low Word register,                             Address offset: 0x90  */
 __IO uint32_t APB1HRSTR;      /*!< RCC APB1 peripheral reset High Word register,                            Address offset: 0x94  */
 __IO uint32_t APB2RSTR;       /*!< RCC APB2 peripheral reset register,                                      Address offset: 0x98  */
 __IO uint32_t APB4RSTR;       /*!< RCC APB4 peripheral reset register,                                      Address offset: 0x9C  */
 __IO uint32_t GCR;            /*!< RCC RCC Global Control  Register,                                        Address offset: 0xA0  */
 uint32_t     RESERVED7;       /*!< Reserved,                                                                Address offset: 0xA4  */
 __IO uint32_t D3AMR;          /*!< RCC Domain 3 Autonomous Mode Register,                                   Address offset: 0xA8  */
 uint32_t     RESERVED8[9];    /*!< Reserved, 0xAC-0xCC                                                      Address offset: 0xAC  */
 __IO uint32_t RSR;            /*!< RCC Reset status register,                                               Address offset: 0xD0  */
 __IO uint32_t AHB3ENR;        /*!< RCC AHB3 peripheral clock  register,                                     Address offset: 0xD4  */
 __IO uint32_t AHB1ENR;        /*!< RCC AHB1 peripheral clock  register,                                     Address offset: 0xD8  */
 __IO uint32_t AHB2ENR;        /*!< RCC AHB2 peripheral clock  register,                                     Address offset: 0xDC  */
 __IO uint32_t AHB4ENR;        /*!< RCC AHB4 peripheral clock  register,                                     Address offset: 0xE0  */
 __IO uint32_t APB3ENR;        /*!< RCC APB3 peripheral clock  register,                                     Address offset: 0xE4  */
 __IO uint32_t APB1LENR;       /*!< RCC APB1 peripheral clock  Low Word register,                            Address offset: 0xE8  */
 __IO uint32_t APB1HENR;       /*!< RCC APB1 peripheral clock  High Word register,                           Address offset: 0xEC  */
 __IO uint32_t APB2ENR;        /*!< RCC APB2 peripheral clock  register,                                     Address offset: 0xF0  */
 __IO uint32_t APB4ENR;        /*!< RCC APB4 peripheral clock  register,                                     Address offset: 0xF4  */
 uint32_t      RESERVED9;      /*!< Reserved,                                                                Address offset: 0xF8  */
 __IO uint32_t AHB3LPENR;      /*!< RCC AHB3 peripheral sleep clock  register,                               Address offset: 0xFC  */
 __IO uint32_t AHB1LPENR;      /*!< RCC AHB1 peripheral sleep clock  register,                               Address offset: 0x100 */
 __IO uint32_t AHB2LPENR;      /*!< RCC AHB2 peripheral sleep clock  register,                               Address offset: 0x104 */
 __IO uint32_t AHB4LPENR;      /*!< RCC AHB4 peripheral sleep clock  register,                               Address offset: 0x108 */
 __IO uint32_t APB3LPENR;      /*!< RCC APB3 peripheral sleep clock  register,                               Address offset: 0x10C */
 __IO uint32_t APB1LLPENR;     /*!< RCC APB1 peripheral sleep clock  Low Word register,                      Address offset: 0x110 */
 __IO uint32_t APB1HLPENR;     /*!< RCC APB1 peripheral sleep clock  High Word register,                     Address offset: 0x114 */
 __IO uint32_t APB2LPENR;      /*!< RCC APB2 peripheral sleep clock  register,                               Address offset: 0x118 */
 __IO uint32_t APB4LPENR;      /*!< RCC APB4 peripheral sleep clock  register,                               Address offset: 0x11C */
 uint32_t     RESERVED10[4];   /*!< Reserved, 0x120-0x12C                                                    Address offset: 0x120 */

} RCC_TypeDef;


/**
  * @brief Real-Time Clock
  */

typedef struct
{
  __IO uint32_t TR;         /*!< RTC time register,                                         Address offset: 0x00 */
  __IO uint32_t DR;         /*!< RTC date register,                                         Address offset: 0x04 */
  __IO uint32_t CR;         /*!< RTC control register,                                      Address offset: 0x08 */
  __IO uint32_t ISR;        /*!< RTC initialization and status register,                    Address offset: 0x0C */
  __IO uint32_t PRER;       /*!< RTC prescaler register,                                    Address offset: 0x10 */
  __IO uint32_t WUTR;       /*!< RTC wakeup timer register,                                 Address offset: 0x14 */
       uint32_t reserved;   /*!< Reserved  */
  __IO uint32_t ALRMAR;     /*!< RTC alarm A register,                                      Address offset: 0x1C */
  __IO uint32_t ALRMBR;     /*!< RTC alarm B register,                                      Address offset: 0x20 */
  __IO uint32_t WPR;        /*!< RTC write protection register,                             Address offset: 0x24 */
  __IO uint32_t SSR;        /*!< RTC sub second register,                                   Address offset: 0x28 */
  __IO uint32_t SHIFTR;     /*!< RTC shift control register,                                Address offset: 0x2C */
  __IO uint32_t TSTR;       /*!< RTC time stamp time register,                              Address offset: 0x30 */
  __IO uint32_t TSDR;       /*!< RTC time stamp date register,                              Address offset: 0x34 */
  __IO uint32_t TSSSR;      /*!< RTC time-stamp sub second register,                        Address offset: 0x38 */
  __IO uint32_t CALR;       /*!< RTC calibration register,                                  Address offset: 0x3C */
  __IO uint32_t TAMPCR;     /*!< RTC tamper and alternate function configuration register,  Address offset: 0x40 */
  __IO uint32_t ALRMASSR;   /*!< RTC alarm A sub second register,                           Address offset: 0x44 */
  __IO uint32_t ALRMBSSR;   /*!< RTC alarm B sub second register,                           Address offset: 0x48 */
  __IO uint32_t OR;         /*!< RTC option register,                                       Address offset: 0x4C */
  __IO uint32_t BKP0R;      /*!< RTC backup register 0,                                     Address offset: 0x50 */
  __IO uint32_t BKP1R;      /*!< RTC backup register 1,                                     Address offset: 0x54 */
  __IO uint32_t BKP2R;      /*!< RTC backup register 2,                                     Address offset: 0x58 */
  __IO uint32_t BKP3R;      /*!< RTC backup register 3,                                     Address offset: 0x5C */
  __IO uint32_t BKP4R;      /*!< RTC backup register 4,                                     Address offset: 0x60 */
  __IO uint32_t BKP5R;      /*!< RTC backup register 5,                                     Address offset: 0x64 */
  __IO uint32_t BKP6R;      /*!< RTC backup register 6,                                     Address offset: 0x68 */
  __IO uint32_t BKP7R;      /*!< RTC backup register 7,                                     Address offset: 0x6C */
  __IO uint32_t BKP8R;      /*!< RTC backup register 8,                                     Address offset: 0x70 */
  __IO uint32_t BKP9R;      /*!< RTC backup register 9,                                     Address offset: 0x74 */
  __IO uint32_t BKP10R;     /*!< RTC backup register 10,                                    Address offset: 0x78 */
  __IO uint32_t BKP11R;     /*!< RTC backup register 11,                                    Address offset: 0x7C */
  __IO uint32_t BKP12R;     /*!< RTC backup register 12,                                    Address offset: 0x80 */
  __IO uint32_t BKP13R;     /*!< RTC backup register 13,                                    Address offset: 0x84 */
  __IO uint32_t BKP14R;     /*!< RTC backup register 14,                                    Address offset: 0x88 */
  __IO uint32_t BKP15R;     /*!< RTC backup register 15,                                    Address offset: 0x8C */
  __IO uint32_t BKP16R;     /*!< RTC backup register 16,                                    Address offset: 0x90 */
  __IO uint32_t BKP17R;     /*!< RTC backup register 17,                                    Address offset: 0x94 */
  __IO uint32_t BKP18R;     /*!< RTC backup register 18,                                    Address offset: 0x98 */
  __IO uint32_t BKP19R;     /*!< RTC backup register 19,                                    Address offset: 0x9C */
  __IO uint32_t BKP20R;     /*!< RTC backup register 20,                                    Address offset: 0xA0 */
  __IO uint32_t BKP21R;     /*!< RTC backup register 21,                                    Address offset: 0xA4 */
  __IO uint32_t BKP22R;     /*!< RTC backup register 22,                                    Address offset: 0xA8 */
  __IO uint32_t BKP23R;     /*!< RTC backup register 23,                                    Address offset: 0xAC */
  __IO uint32_t BKP24R;     /*!< RTC backup register 24,                                    Address offset: 0xB0 */
  __IO uint32_t BKP25R;     /*!< RTC backup register 25,                                    Address offset: 0xB4 */
  __IO uint32_t BKP26R;     /*!< RTC backup register 26,                                    Address offset: 0xB8 */
  __IO uint32_t BKP27R;     /*!< RTC backup register 27,                                    Address offset: 0xBC */
  __IO uint32_t BKP28R;     /*!< RTC backup register 28,                                    Address offset: 0xC0 */
  __IO uint32_t BKP29R;     /*!< RTC backup register 29,                                    Address offset: 0xC4 */
  __IO uint32_t BKP30R;     /*!< RTC backup register 30,                                    Address offset: 0xC8 */
  __IO uint32_t BKP31R;     /*!< RTC backup register 31,                                    Address offset: 0xCC */
} RTC_TypeDef;


/**
  * @brief Serial Audio Interface
  */

typedef struct
{
  __IO uint32_t GCR;           /*!< SAI global configuration register, Address offset: 0x00 */
  uint32_t      RESERVED0[16]; /*!< Reserved, 0x04 - 0x43                                   */
  __IO uint32_t PDMCR;         /*!< SAI PDM control register,          Address offset: 0x44 */
  __IO uint32_t PDMDLY;        /*!< SAI PDM delay register,            Address offset: 0x48 */
} SAI_TypeDef;

typedef struct
{
  __IO uint32_t CR1;      /*!< SAI block x configuration register 1,     Address offset: 0x04 */
  __IO uint32_t CR2;      /*!< SAI block x configuration register 2,     Address offset: 0x08 */
  __IO uint32_t FRCR;     /*!< SAI block x frame configuration register, Address offset: 0x0C */
  __IO uint32_t SLOTR;    /*!< SAI block x slot register,                Address offset: 0x10 */
  __IO uint32_t IMR;      /*!< SAI block x interrupt mask register,      Address offset: 0x14 */
  __IO uint32_t SR;       /*!< SAI block x status register,              Address offset: 0x18 */
  __IO uint32_t CLRFR;    /*!< SAI block x clear flag register,          Address offset: 0x1C */
  __IO uint32_t DR;       /*!< SAI block x data register,                Address offset: 0x20 */
} SAI_Block_TypeDef;

/**
  * @brief SPDIF-RX Interface
  */

typedef struct
{
  __IO uint32_t   CR;           /*!< Control register,                   Address offset: 0x00 */
  __IO uint32_t   IMR;          /*!< Interrupt mask register,            Address offset: 0x04 */
  __IO uint32_t   SR;           /*!< Status register,                    Address offset: 0x08 */
  __IO uint32_t   IFCR;         /*!< Interrupt Flag Clear register,      Address offset: 0x0C */
  __IO uint32_t   DR;           /*!< Data input register,                Address offset: 0x10 */
  __IO uint32_t   CSR;          /*!< Channel Status register,            Address offset: 0x14 */
  __IO uint32_t   DIR;          /*!< Debug Information register,         Address offset: 0x18 */
  uint32_t        RESERVED2;    /*!< Reserved,  0x1A                                          */
} SPDIFRX_TypeDef;


/**
  * @brief Secure digital input/output Interface
  */

typedef struct
{
  __IO uint32_t POWER;          /*!< SDMMC power control register,             Address offset: 0x00 */
  __IO uint32_t CLKCR;          /*!< SDMMC clock control register,             Address offset: 0x04 */
  __IO uint32_t ARG;            /*!< SDMMC argument register,                  Address offset: 0x08 */
  __IO uint32_t CMD;            /*!< SDMMC command register,                   Address offset: 0x0C */
  __I uint32_t  RESPCMD;        /*!< SDMMC command response register,          Address offset: 0x10 */
  __I uint32_t  RESP1;          /*!< SDMMC response 1 register,                Address offset: 0x14 */
  __I uint32_t  RESP2;          /*!< SDMMC response 2 register,                Address offset: 0x18 */
  __I uint32_t  RESP3;          /*!< SDMMC response 3 register,                Address offset: 0x1C */
  __I uint32_t  RESP4;          /*!< SDMMC response 4 register,                Address offset: 0x20 */
  __IO uint32_t DTIMER;         /*!< SDMMC data timer register,                Address offset: 0x24 */
  __IO uint32_t DLEN;           /*!< SDMMC data length register,               Address offset: 0x28 */
  __IO uint32_t DCTRL;          /*!< SDMMC data control register,              Address offset: 0x2C */
  __I uint32_t  DCOUNT;         /*!< SDMMC data counter register,              Address offset: 0x30 */
  __I uint32_t  STA;            /*!< SDMMC status register,                    Address offset: 0x34 */
  __IO uint32_t ICR;            /*!< SDMMC interrupt clear register,           Address offset: 0x38 */
  __IO uint32_t MASK;           /*!< SDMMC mask register,                      Address offset: 0x3C */
  __IO uint32_t ACKTIME;        /*!< SDMMC Acknowledgement timer register,     Address offset: 0x40 */
  uint32_t      RESERVED0[3];   /*!< Reserved, 0x44 - 0x4C - 0x4C                                   */
  __IO uint32_t IDMACTRL;       /*!< SDMMC DMA control register,               Address offset: 0x50 */
  __IO uint32_t IDMABSIZE;      /*!< SDMMC DMA buffer size register,           Address offset: 0x54 */
  __IO uint32_t IDMABASE0;      /*!< SDMMC DMA buffer 0 base address register, Address offset: 0x58 */
  __IO uint32_t IDMABASE1;      /*!< SDMMC DMA buffer 1 base address register, Address offset: 0x5C */
  uint32_t      RESERVED1[8];   /*!< Reserved, 0x60-0x7C                                            */
  __IO uint32_t FIFO;           /*!< SDMMC data FIFO register,                 Address offset: 0x80 */
  uint32_t      RESERVED2[222]; /*!< Reserved, 0x84-0x3F8                                           */
  __IO uint32_t IPVR;           /*!< SDMMC data FIFO register,                Address offset: 0x3FC */
} SDMMC_TypeDef;


/**
  * @brief Delay Block DLYB
  */

typedef struct
{
  __IO uint32_t CR;          /*!< DELAY BLOCK control register,  Address offset: 0x00 */
  __IO uint32_t CFGR;        /*!< DELAY BLOCK configuration register,  Address offset: 0x04 */
} DLYB_TypeDef;

/**
  * @brief HW Semaphore HSEM
  */

typedef struct
{
  __IO uint32_t R[32];      /*!< 2-step write lock and read back registers,     Address offset: 00h-7Ch  */
  __IO uint32_t RLR[32];    /*!< 1-step read lock registers,                    Address offset: 80h-FCh  */
  __IO uint32_t IER;        /*!< HSEM Interrupt enable register ,             Address offset: 100h     */
  __IO uint32_t ICR;        /*!< HSEM Interrupt clear register ,              Address offset: 104h     */
  __IO uint32_t ISR;        /*!< HSEM Interrupt Status register ,             Address offset: 108h     */
  __IO uint32_t MISR;       /*!< HSEM Interrupt Masked Status register ,      Address offset: 10Ch     */
  uint32_t  Reserved[12];   /* Reserved                                       Address offset: 110h-13Ch*/
  __IO uint32_t CR;         /*!< HSEM Semaphore clear register ,               Address offset: 140h      */
  __IO uint32_t KEYR;       /*!< HSEM Semaphore clear key register ,           Address offset: 144h      */

} HSEM_TypeDef;

/**
  * @brief Serial Peripheral Interface
  */

typedef struct
{
  __IO uint32_t CR1;          /*!< SPI Control register 1,                             Address offset: 0x00 */
  __IO uint32_t CR2;          /*!< SPI Control register 2,                             Address offset: 0x04 */
  __IO uint32_t CFG1;         /*!< SPI Status register,                                Address offset: 0x08 */
  __IO uint32_t CFG2;         /*!< SPI Status register,                                Address offset: 0x0C */
  __IO uint32_t IER;          /*!< SPI data register,                                  Address offset: 0x10 */
  __IO uint32_t SR;           /*!< SPI data register,                                  Address offset: 0x14 */
  __IO uint32_t IFCR;         /*!< SPI data register,                                  Address offset: 0x18 */
  uint32_t      RESERVED0;    /*!< SPI data register,                                  Address offset: 0x1C */
  __IO uint32_t TXDR;         /*!< SPI data register,                                  Address offset: 0x20 */
  uint32_t      RESERVED1[3]; /*!< Reserved, 0x24-0x2C                                                      */
  __IO uint32_t RXDR;         /*!< SPI data register,                                  Address offset: 0x30 */
  uint32_t      RESERVED2[3]; /*!< Reserved, 0x34-0x3C                                                      */
  __IO uint32_t CRCPOLY;     /*!< SPI data register,                                   Address offset: 0x40 */
  __IO uint32_t TXCRC;       /*!< SPI data register,                                   Address offset: 0x44 */
  __IO uint32_t RXCRC;       /*!< SPI data register,                                   Address offset: 0x48 */
  __IO uint32_t UDRDR;       /*!< SPI data register,                                   Address offset: 0x4C */
  __IO uint32_t I2SCFGR;      /*!< SPI data register,                                  Address offset: 0x50 */

} SPI_TypeDef;

/**
  * @brief QUAD Serial Peripheral Interface
  */

typedef struct
{
  __IO uint32_t CR;       /*!< QUADSPI Control register,                           Address offset: 0x00 */
  __IO uint32_t DCR;      /*!< QUADSPI Device Configuration register,              Address offset: 0x04 */
  __IO uint32_t SR;       /*!< QUADSPI Status register,                            Address offset: 0x08 */
  __IO uint32_t FCR;      /*!< QUADSPI Flag Clear register,                        Address offset: 0x0C */
  __IO uint32_t DLR;      /*!< QUADSPI Data Length register,                       Address offset: 0x10 */
  __IO uint32_t CCR;      /*!< QUADSPI Communication Configuration register,       Address offset: 0x14 */
  __IO uint32_t AR;       /*!< QUADSPI Address register,                           Address offset: 0x18 */
  __IO uint32_t ABR;      /*!< QUADSPI Alternate Bytes register,                   Address offset: 0x1C */
  __IO uint32_t DR;       /*!< QUADSPI Data register,                              Address offset: 0x20 */
  __IO uint32_t PSMKR;    /*!< QUADSPI Polling Status Mask register,               Address offset: 0x24 */
  __IO uint32_t PSMAR;    /*!< QUADSPI Polling Status Match register,              Address offset: 0x28 */
  __IO uint32_t PIR;      /*!< QUADSPI Polling Interval register,                  Address offset: 0x2C */
  __IO uint32_t LPTR;     /*!< QUADSPI Low Power Timeout register,                 Address offset: 0x30 */
} QUADSPI_TypeDef;


/**
  * @brief TIM
  */

typedef struct
{
  __IO uint16_t CR1;         /*!< TIM control register 1,                   Address offset: 0x00 */
  uint16_t      RESERVED0;   /*!< Reserved, 0x02                                                 */
  __IO uint32_t CR2;         /*!< TIM control register 2,                   Address offset: 0x04 */
  __IO uint32_t SMCR;        /*!< TIM slave mode control register,          Address offset: 0x08 */
  __IO uint32_t DIER;        /*!< TIM DMA/interrupt enable register,        Address offset: 0x0C */
  __IO uint32_t SR;          /*!< TIM status register,                      Address offset: 0x10 */
  __IO uint32_t EGR;         /*!< TIM event generation register,            Address offset: 0x14 */
  __IO uint32_t CCMR1;       /*!< TIM capture/compare mode register 1,      Address offset: 0x18 */
  __IO uint32_t CCMR2;       /*!< TIM capture/compare mode register 2,      Address offset: 0x1C */
  __IO uint32_t CCER;        /*!< TIM capture/compare enable register,      Address offset: 0x20 */
  __IO uint32_t CNT;         /*!< TIM counter register,                     Address offset: 0x24 */
  __IO uint16_t PSC;         /*!< TIM prescaler,                            Address offset: 0x28 */
  uint16_t      RESERVED9;   /*!< Reserved, 0x2A                                                 */
  __IO uint32_t ARR;         /*!< TIM auto-reload register,                 Address offset: 0x2C */
  __IO uint16_t RCR;         /*!< TIM repetition counter register,          Address offset: 0x30 */
  uint16_t      RESERVED10;  /*!< Reserved, 0x32                                                 */
  __IO uint32_t CCR1;        /*!< TIM capture/compare register 1,           Address offset: 0x34 */
  __IO uint32_t CCR2;        /*!< TIM capture/compare register 2,           Address offset: 0x38 */
  __IO uint32_t CCR3;        /*!< TIM capture/compare register 3,           Address offset: 0x3C */
  __IO uint32_t CCR4;        /*!< TIM capture/compare register 4,           Address offset: 0x40 */
  __IO uint32_t BDTR;        /*!< TIM break and dead-time register,         Address offset: 0x44 */
  __IO uint16_t DCR;         /*!< TIM DMA control register,                 Address offset: 0x48 */
  uint16_t      RESERVED12;  /*!< Reserved, 0x4A                                                 */
  __IO uint16_t DMAR;        /*!< TIM DMA address for full transfer,        Address offset: 0x4C */
  uint16_t      RESERVED13;  /*!< Reserved, 0x4E                                                 */
  uint16_t      RESERVED14;  /*!< Reserved, 0x50                                                 */
  __IO uint32_t CCMR3;       /*!< TIM capture/compare mode register 3,      Address offset: 0x54 */
  __IO uint32_t CCR5;        /*!< TIM capture/compare register5,            Address offset: 0x58 */
  __IO uint32_t CCR6;        /*!< TIM capture/compare register6,            Address offset: 0x5C */
  __IO uint32_t AF1;         /*!< TIM alternate function option register 1, Address offset: 0x60 */
  __IO uint32_t AF2;         /*!< TIM alternate function option register 2, Address offset: 0x64 */
  __IO uint32_t TISEL;       /*!< TIM Input Selection register,             Address offset: 0x68 */
} TIM_TypeDef;

/**
  * @brief LPTIMIMER
  */
typedef struct
{
  __IO uint32_t ISR;      /*!< LPTIM Interrupt and Status register,                Address offset: 0x00 */
  __IO uint32_t ICR;      /*!< LPTIM Interrupt Clear register,                     Address offset: 0x04 */
  __IO uint32_t IER;      /*!< LPTIM Interrupt Enable register,                    Address offset: 0x08 */
  __IO uint32_t CFGR;     /*!< LPTIM Configuration register,                       Address offset: 0x0C */
  __IO uint32_t CR;       /*!< LPTIM Control register,                             Address offset: 0x10 */
  __IO uint32_t CMP;      /*!< LPTIM Compare register,                             Address offset: 0x14 */
  __IO uint32_t ARR;      /*!< LPTIM Autoreload register,                          Address offset: 0x18 */
  __IO uint32_t CNT;      /*!< LPTIM Counter register,                             Address offset: 0x1C */
  uint16_t  RESERVED1;    /*!< Reserved, 0x20                                                 */
  __IO uint32_t CFGR2;     /*!< LPTIM Option register,                              Address offset: 0x24 */
} LPTIM_TypeDef;

/**
  * @brief Comparator
  */
typedef struct
{
  __IO uint32_t SR;        /*!< Comparator status register,                    Address offset: 0x00 */
 __IO uint32_t ICFR;      /*!< Comparator interrupt clear flag register,      Address offset: 0x04 */
	__IO uint32_t OR;        /*!< Comparator option register,                    Address offset: 0x08 */
} COMPOPT_TypeDef;

typedef struct
{
	__IO uint32_t CFGR;      /*!< Comparator configuration register  ,           Address offset: 0x00 */
} COMP_TypeDef;

typedef struct
{
  __IO uint32_t CFGR;       /*!< COMP control and status register, used for bits common to several COMP instances, Address offset: 0x00 */
} COMP_Common_TypeDef;
/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */

typedef struct
{
  __IO uint32_t CR1;    /*!< USART Control register 1,                 Address offset: 0x00 */
  __IO uint32_t CR2;    /*!< USART Control register 2,                 Address offset: 0x04 */
  __IO uint32_t CR3;    /*!< USART Control register 3,                 Address offset: 0x08 */
  __IO uint32_t BRR;    /*!< USART Baud rate register,                 Address offset: 0x0C */
  __IO uint16_t GTPR;   /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
  uint16_t  RESERVED2;  /*!< Reserved, 0x12                                                 */
  __IO uint32_t RTOR;   /*!< USART Receiver Time Out register,         Address offset: 0x14 */
  __IO uint16_t RQR;    /*!< USART Request register,                   Address offset: 0x18 */
  uint16_t  RESERVED3;  /*!< Reserved, 0x1A                                                 */
  __IO uint32_t ISR;    /*!< USART Interrupt and status register,      Address offset: 0x1C */
  __IO uint32_t ICR;    /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
  __IO uint16_t RDR;    /*!< USART Receive Data register,              Address offset: 0x24 */
  uint16_t  RESERVED4;  /*!< Reserved, 0x26                                                 */
  __IO uint16_t TDR;    /*!< USART Transmit Data register,             Address offset: 0x28 */
  uint16_t  RESERVED5;  /*!< Reserved, 0x2A                                                 */
  __IO uint32_t PRESC;  /*!< USART clock Prescaler register,           Address offset: 0x2C */
} USART_TypeDef;

/**
  * @brief Single Wire Protocol Master Interface SPWMI
  */
typedef struct
{
  __IO uint32_t CR;          /*!< SWPMI Configuration/Control register,     Address offset: 0x00 */
  __IO uint32_t BRR;         /*!< SWPMI bitrate register,                   Address offset: 0x04 */
    uint32_t  RESERVED1;     /*!< Reserved, 0x08                                                 */
  __IO uint32_t ISR;         /*!< SWPMI Interrupt and Status register,      Address offset: 0x0C */
  __IO uint32_t ICR;         /*!< SWPMI Interrupt Flag Clear register,      Address offset: 0x10 */
  __IO uint32_t IER;         /*!< SWPMI Interrupt Enable register,          Address offset: 0x14 */
  __IO uint32_t RFL;         /*!< SWPMI Receive Frame Length register,      Address offset: 0x18 */
  __IO uint32_t TDR;         /*!< SWPMI Transmit data register,             Address offset: 0x1C */
  __IO uint32_t RDR;         /*!< SWPMI Receive data register,              Address offset: 0x20 */
  __IO uint32_t OR;          /*!< SWPMI Option register,                    Address offset: 0x24 */
} SWPMI_TypeDef;

/**
  * @brief Window WATCHDOG
  */

typedef struct
{
  __IO uint32_t CR;   /*!< WWDG Control register,       Address offset: 0x00 */
  __IO uint32_t CFR;  /*!< WWDG Configuration register, Address offset: 0x04 */
  __IO uint32_t SR;   /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;

/**
  * @brief High resolution Timer (HRTIM)
  */
/* HRTIM master registers definition */
typedef struct
{
  __IO uint32_t MCR;            /*!< HRTIM Master Timer control register,                     Address offset: 0x00 */
  __IO uint32_t MISR;           /*!< HRTIM Master Timer interrupt status register,            Address offset: 0x04 */
  __IO uint32_t MICR;           /*!< HRTIM Master Timer interupt clear register,              Address offset: 0x08 */
  __IO uint32_t MDIER;          /*!< HRTIM Master Timer DMA/interrupt enable register         Address offset: 0x0C */
  __IO uint32_t MCNTR;          /*!< HRTIM Master Timer counter register,                     Address offset: 0x10 */
  __IO uint32_t MPER;           /*!< HRTIM Master Timer period register,                      Address offset: 0x14 */
  __IO uint32_t MREP;           /*!< HRTIM Master Timer repetition register,                  Address offset: 0x18 */
  __IO uint32_t MCMP1R;         /*!< HRTIM Master Timer compare 1 register,                   Address offset: 0x1C */
  uint32_t      RESERVED0;     /*!< Reserved,                                                                0x20 */
  __IO uint32_t MCMP2R;         /*!< HRTIM Master Timer compare 2 register,                   Address offset: 0x24 */
  __IO uint32_t MCMP3R;         /*!< HRTIM Master Timer compare 3 register,                   Address offset: 0x28 */
  __IO uint32_t MCMP4R;         /*!< HRTIM Master Timer compare 4 register,                   Address offset: 0x2C */
  uint32_t      RESERVED1[20];  /*!< Reserved,                                                          0x30..0x7C */
}HRTIM_Master_TypeDef;

/* HRTIM Timer A to E registers definition */
typedef struct
{
  __IO uint32_t TIMxCR;     /*!< HRTIM Timerx control register,                              Address offset: 0x00  */
  __IO uint32_t TIMxISR;    /*!< HRTIM Timerx interrupt status register,                     Address offset: 0x04  */
  __IO uint32_t TIMxICR;    /*!< HRTIM Timerx interrupt clear register,                      Address offset: 0x08  */
  __IO uint32_t TIMxDIER;   /*!< HRTIM Timerx DMA/interrupt enable register,                 Address offset: 0x0C  */
  __IO uint32_t CNTxR;      /*!< HRTIM Timerx counter register,                              Address offset: 0x10  */
  __IO uint32_t PERxR;      /*!< HRTIM Timerx period register,                               Address offset: 0x14  */
  __IO uint32_t REPxR;      /*!< HRTIM Timerx repetition register,                           Address offset: 0x18  */
  __IO uint32_t CMP1xR;     /*!< HRTIM Timerx compare 1 register,                            Address offset: 0x1C  */
  __IO uint32_t CMP1CxR;    /*!< HRTIM Timerx compare 1 compound register,                   Address offset: 0x20  */
  __IO uint32_t CMP2xR;     /*!< HRTIM Timerx compare 2 register,                            Address offset: 0x24  */
  __IO uint32_t CMP3xR;     /*!< HRTIM Timerx compare 3 register,                            Address offset: 0x28  */
  __IO uint32_t CMP4xR;     /*!< HRTIM Timerx compare 4 register,                            Address offset: 0x2C  */
  __IO uint32_t CPT1xR;     /*!< HRTIM Timerx capture 1 register,                            Address offset: 0x30  */
  __IO uint32_t CPT2xR;     /*!< HRTIM Timerx capture 2 register,                            Address offset: 0x34 */
  __IO uint32_t DTxR;       /*!< HRTIM Timerx dead time register,                            Address offset: 0x38 */
  __IO uint32_t SETx1R;     /*!< HRTIM Timerx output 1 set register,                         Address offset: 0x3C */
  __IO uint32_t RSTx1R;     /*!< HRTIM Timerx output 1 reset register,                       Address offset: 0x40 */
  __IO uint32_t SETx2R;     /*!< HRTIM Timerx output 2 set register,                         Address offset: 0x44 */
  __IO uint32_t RSTx2R;     /*!< HRTIM Timerx output 2 reset register,                       Address offset: 0x48 */
  __IO uint32_t EEFxR1;     /*!< HRTIM Timerx external event filtering 1 register,           Address offset: 0x4C */
  __IO uint32_t EEFxR2;     /*!< HRTIM Timerx external event filtering 2 register,           Address offset: 0x50 */
  __IO uint32_t RSTxR;      /*!< HRTIM Timerx Reset register,                                Address offset: 0x54 */
  __IO uint32_t CHPxR;      /*!< HRTIM Timerx Chopper register,                              Address offset: 0x58 */
  __IO uint32_t CPT1xCR;    /*!< HRTIM Timerx Capture 1 register,                            Address offset: 0x5C */
  __IO uint32_t CPT2xCR;    /*!< HRTIM Timerx Capture 2 register,                            Address offset: 0x60 */
  __IO uint32_t OUTxR;      /*!< HRTIM Timerx Output register,                               Address offset: 0x64 */
  __IO uint32_t FLTxR;      /*!< HRTIM Timerx Fault register,                                Address offset: 0x68 */
  uint32_t      RESERVED0[5];  /*!< Reserved,                                                              0x6C..0x7C */
}HRTIM_Timerx_TypeDef;

/* HRTIM common register definition */
typedef struct
{
  __IO uint32_t CR1;        /*!< HRTIM control register1,                                    Address offset: 0x00 */
  __IO uint32_t CR2;        /*!< HRTIM control register2,                                    Address offset: 0x04 */
  __IO uint32_t ISR;        /*!< HRTIM interrupt status register,                            Address offset: 0x08 */
  __IO uint32_t ICR;        /*!< HRTIM interrupt clear register,                             Address offset: 0x0C */
  __IO uint32_t IER;        /*!< HRTIM interrupt enable register,                            Address offset: 0x10 */
  __IO uint32_t OENR;       /*!< HRTIM Output enable register,                               Address offset: 0x14 */
  __IO uint32_t ODISR;      /*!< HRTIM Output disable register,                              Address offset: 0x18 */
  __IO uint32_t ODSR;       /*!< HRTIM Output disable status register,                       Address offset: 0x1C */
  __IO uint32_t BMCR;       /*!< HRTIM Burst mode control register,                          Address offset: 0x20 */
  __IO uint32_t BMTRGR;     /*!< HRTIM Busrt mode trigger register,                          Address offset: 0x24 */
  __IO uint32_t BMCMPR;     /*!< HRTIM Burst mode compare register,                          Address offset: 0x28 */
  __IO uint32_t BMPER;      /*!< HRTIM Burst mode period register,                           Address offset: 0x2C */
  __IO uint32_t EECR1;      /*!< HRTIM Timer external event control register1,               Address offset: 0x30 */
  __IO uint32_t EECR2;      /*!< HRTIM Timer external event control register2,               Address offset: 0x34 */
  __IO uint32_t EECR3;      /*!< HRTIM Timer external event control register3,               Address offset: 0x38 */
  __IO uint32_t ADC1R;      /*!< HRTIM ADC Trigger 1 register,                               Address offset: 0x3C */
  __IO uint32_t ADC2R;      /*!< HRTIM ADC Trigger 2 register,                               Address offset: 0x40 */
  __IO uint32_t ADC3R;      /*!< HRTIM ADC Trigger 3 register,                               Address offset: 0x44 */
  __IO uint32_t ADC4R;      /*!< HRTIM ADC Trigger 4 register,                               Address offset: 0x48 */
  __IO uint32_t DLLCR;      /*!< HRTIM DLL control register,                                 Address offset: 0x4C */
  __IO uint32_t FLTINR1;    /*!< HRTIM Fault input register1,                                Address offset: 0x50 */
  __IO uint32_t FLTINR2;    /*!< HRTIM Fault input register2,                                Address offset: 0x54 */
  __IO uint32_t BDMUPR;     /*!< HRTIM Burst DMA Master Timer update register,               Address offset: 0x58 */
  __IO uint32_t BDTAUPR;    /*!< HRTIM Burst DMA Timerx update register,                     Address offset: 0x5C */
  __IO uint32_t BDTBUPR;    /*!< HRTIM Burst DMA Timerx update register,                     Address offset: 0x60 */
  __IO uint32_t BDTCUPR;    /*!< HRTIM Burst DMA Timerx update register,                     Address offset: 0x64 */
  __IO uint32_t BDTDUPR;    /*!< HRTIM Burst DMA Timerx update register,                     Address offset: 0x68 */
  __IO uint32_t BDTEUPR;    /*!< HRTIM Burst DMA Timerx update register,                     Address offset: 0x6C */
  __IO uint32_t BDMADR;     /*!< HRTIM Burst DMA Master Data register,                       Address offset: 0x70 */
}HRTIM_Common_TypeDef;

/* HRTIM  register definition */
typedef struct {
  HRTIM_Master_TypeDef sMasterRegs;
  HRTIM_Timerx_TypeDef sTimerxRegs[5];
  uint32_t             RESERVED0[32];
  HRTIM_Common_TypeDef sCommonRegs;
}HRTIM_TypeDef;

/**
  * @brief RNG
  */

typedef struct
{
  __IO uint32_t CR;  /*!< RNG control register, Address offset: 0x00 */
  __IO uint32_t SR;  /*!< RNG status register,  Address offset: 0x04 */
  __IO uint32_t DR;  /*!< RNG data register,    Address offset: 0x08 */
} RNG_TypeDef;

/**
  * @brief MDIOS
  */

typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t WRFR;
  __IO uint32_t CWRFR;
  __IO uint32_t RDFR;
  __IO uint32_t CRDFR;
  __IO uint32_t SR;
  __IO uint32_t CLRFR;
  uint32_t RESERVED[57];
  __IO uint32_t DINR0;
  __IO uint32_t DINR1;
  __IO uint32_t DINR2;
  __IO uint32_t DINR3;
  __IO uint32_t DINR4;
  __IO uint32_t DINR5;
  __IO uint32_t DINR6;
  __IO uint32_t DINR7;
  __IO uint32_t DINR8;
  __IO uint32_t DINR9;
  __IO uint32_t DINR10;
  __IO uint32_t DINR11;
  __IO uint32_t DINR12;
  __IO uint32_t DINR13;
  __IO uint32_t DINR14;
  __IO uint32_t DINR15;
  __IO uint32_t DINR16;
  __IO uint32_t DINR17;
  __IO uint32_t DINR18;
  __IO uint32_t DINR19;
  __IO uint32_t DINR20;
  __IO uint32_t DINR21;
  __IO uint32_t DINR22;
  __IO uint32_t DINR23;
  __IO uint32_t DINR24;
  __IO uint32_t DINR25;
  __IO uint32_t DINR26;
  __IO uint32_t DINR27;
  __IO uint32_t DINR28;
  __IO uint32_t DINR29;
  __IO uint32_t DINR30;
  __IO uint32_t DINR31;
  __IO uint32_t DOUTR0;
  __IO uint32_t DOUTR1;
  __IO uint32_t DOUTR2;
  __IO uint32_t DOUTR3;
  __IO uint32_t DOUTR4;
  __IO uint32_t DOUTR5;
  __IO uint32_t DOUTR6;
  __IO uint32_t DOUTR7;
  __IO uint32_t DOUTR8;
  __IO uint32_t DOUTR9;
  __IO uint32_t DOUTR10;
  __IO uint32_t DOUTR11;
  __IO uint32_t DOUTR12;
  __IO uint32_t DOUTR13;
  __IO uint32_t DOUTR14;
  __IO uint32_t DOUTR15;
  __IO uint32_t DOUTR16;
  __IO uint32_t DOUTR17;
  __IO uint32_t DOUTR18;
  __IO uint32_t DOUTR19;
  __IO uint32_t DOUTR20;
  __IO uint32_t DOUTR21;
  __IO uint32_t DOUTR22;
  __IO uint32_t DOUTR23;
  __IO uint32_t DOUTR24;
  __IO uint32_t DOUTR25;
  __IO uint32_t DOUTR26;
  __IO uint32_t DOUTR27;
  __IO uint32_t DOUTR28;
  __IO uint32_t DOUTR29;
  __IO uint32_t DOUTR30;
  __IO uint32_t DOUTR31;
} MDIOS_TypeDef;


/**
  * @brief USB_OTG_Core_Registers
  */
typedef struct
{
 __IO uint32_t GOTGCTL;               /*!< USB_OTG Control and Status Register          000h */
  __IO uint32_t GOTGINT;              /*!< USB_OTG Interrupt Register                   004h */
  __IO uint32_t GAHBCFG;              /*!< Core AHB Configuration Register              008h */
  __IO uint32_t GUSBCFG;              /*!< Core USB Configuration Register              00Ch */
  __IO uint32_t GRSTCTL;              /*!< Core Reset Register                          010h */
  __IO uint32_t GINTSTS;              /*!< Core Interrupt Register                      014h */
  __IO uint32_t GINTMSK;              /*!< Core Interrupt Mask Register                 018h */
  __IO uint32_t GRXSTSR;              /*!< Receive Sts Q Read Register                  01Ch */
  __IO uint32_t GRXSTSP;              /*!< Receive Sts Q Read & POP Register            020h */
  __IO uint32_t GRXFSIZ;              /*!< Receive FIFO Size Register                   024h */
  __IO uint32_t DIEPTXF0_HNPTXFSIZ;   /*!< EP0 / Non Periodic Tx FIFO Size Register     028h */
  __IO uint32_t HNPTXSTS;             /*!< Non Periodic Tx FIFO/Queue Sts reg           02Ch */
  uint32_t Reserved30[2];             /*!< Reserved                                     030h */
  __IO uint32_t GCCFG;                /*!< General Purpose IO Register                  038h */
  __IO uint32_t CID;                  /*!< User ID Register                             03Ch */
  __IO uint32_t GSNPSID;              /* USB_OTG core ID                                 040h*/
  __IO uint32_t GHWCFG1;              /* User HW config1                                 044h*/
  __IO uint32_t GHWCFG2;              /* User HW config2                                 048h*/
  __IO uint32_t GHWCFG3;              /*!< User HW config3                              04Ch */
  uint32_t  Reserved6;                /*!< Reserved                                     050h */
  __IO uint32_t GLPMCFG;              /*!< LPM Register                                 054h */
  __IO uint32_t GPWRDN;               /*!< Power Down Register                          058h */
  __IO uint32_t GDFIFOCFG;            /*!< DFIFO Software Config Register               05Ch */
   __IO uint32_t GADPCTL;             /*!< ADP Timer, Control and Status Register       60Ch */
    uint32_t  Reserved43[39];         /*!< Reserved                                058h-0FFh */
  __IO uint32_t HPTXFSIZ;             /*!< Host Periodic Tx FIFO Size Reg               100h */
  __IO uint32_t DIEPTXF[0x0F];        /*!< dev Periodic Transmit FIFO */
} USB_OTG_GlobalTypeDef;


/**
  * @brief USB_OTG_device_Registers
  */
typedef struct
{
  __IO uint32_t DCFG;            /*!< dev Configuration Register   800h */
  __IO uint32_t DCTL;            /*!< dev Control Register         804h */
  __IO uint32_t DSTS;            /*!< dev Status Register (RO)     808h */
  uint32_t Reserved0C;           /*!< Reserved                     80Ch */
  __IO uint32_t DIEPMSK;         /*!< dev IN Endpoint Mask         810h */
  __IO uint32_t DOEPMSK;         /*!< dev OUT Endpoint Mask        814h */
  __IO uint32_t DAINT;           /*!< dev All Endpoints Itr Reg    818h */
  __IO uint32_t DAINTMSK;        /*!< dev All Endpoints Itr Mask   81Ch */
  uint32_t  Reserved20;          /*!< Reserved                     820h */
  uint32_t Reserved9;            /*!< Reserved                     824h */
  __IO uint32_t DVBUSDIS;        /*!< dev VBUS discharge Register  828h */
  __IO uint32_t DVBUSPULSE;      /*!< dev VBUS Pulse Register      82Ch */
  __IO uint32_t DTHRCTL;         /*!< dev threshold                830h */
  __IO uint32_t DIEPEMPMSK;      /*!< dev empty msk                834h */
  __IO uint32_t DEACHINT;        /*!< dedicated EP interrupt       838h */
  __IO uint32_t DEACHMSK;        /*!< dedicated EP msk             83Ch */
  uint32_t Reserved40;           /*!< dedicated EP mask            840h */
  __IO uint32_t DINEP1MSK;       /*!< dedicated EP mask            844h */
  uint32_t  Reserved44[15];      /*!< Reserved                 844-87Ch */
  __IO uint32_t DOUTEP1MSK;      /*!< dedicated EP msk             884h */
} USB_OTG_DeviceTypeDef;


/**
  * @brief USB_OTG_IN_Endpoint-Specific_Register
  */
typedef struct
{
  __IO uint32_t DIEPCTL;           /*!< dev IN Endpoint Control Reg    900h + (ep_num * 20h) + 00h */
  uint32_t Reserved04;             /*!< Reserved                       900h + (ep_num * 20h) + 04h */
  __IO uint32_t DIEPINT;           /*!< dev IN Endpoint Itr Reg        900h + (ep_num * 20h) + 08h */
  uint32_t Reserved0C;             /*!< Reserved                       900h + (ep_num * 20h) + 0Ch */
  __IO uint32_t DIEPTSIZ;          /*!< IN Endpoint Txfer Size         900h + (ep_num * 20h) + 10h */
  __IO uint32_t DIEPDMA;           /*!< IN Endpoint DMA Address Reg    900h + (ep_num * 20h) + 14h */
  __IO uint32_t DTXFSTS;           /*!< IN Endpoint Tx FIFO Status Reg 900h + (ep_num * 20h) + 18h */
  uint32_t Reserved18;             /*!< Reserved  900h+(ep_num*20h)+1Ch-900h+ (ep_num * 20h) + 1Ch */
} USB_OTG_INEndpointTypeDef;


/**
  * @brief USB_OTG_OUT_Endpoint-Specific_Registers
  */
typedef struct
{
  __IO uint32_t DOEPCTL;       /*!< dev OUT Endpoint Control Reg           B00h + (ep_num * 20h) + 00h */
  uint32_t Reserved04;         /*!< Reserved                               B00h + (ep_num * 20h) + 04h */
  __IO uint32_t DOEPINT;       /*!< dev OUT Endpoint Itr Reg               B00h + (ep_num * 20h) + 08h */
  uint32_t Reserved0C;         /*!< Reserved                               B00h + (ep_num * 20h) + 0Ch */
  __IO uint32_t DOEPTSIZ;      /*!< dev OUT Endpoint Txfer Size            B00h + (ep_num * 20h) + 10h */
  __IO uint32_t DOEPDMA;       /*!< dev OUT Endpoint DMA Address           B00h + (ep_num * 20h) + 14h */
  uint32_t Reserved18[2];      /*!< Reserved B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch */
} USB_OTG_OUTEndpointTypeDef;


/**
  * @brief USB_OTG_Host_Mode_Register_Structures
  */
typedef struct
{
  __IO uint32_t HCFG;             /*!< Host Configuration Register          400h */
  __IO uint32_t HFIR;             /*!< Host Frame Interval Register         404h */
  __IO uint32_t HFNUM;            /*!< Host Frame Nbr/Frame Remaining       408h */
  uint32_t Reserved40C;           /*!< Reserved                             40Ch */
  __IO uint32_t HPTXSTS;          /*!< Host Periodic Tx FIFO/ Queue Status  410h */
  __IO uint32_t HAINT;            /*!< Host All Channels Interrupt Register 414h */
  __IO uint32_t HAINTMSK;         /*!< Host All Channels Interrupt Mask     418h */
} USB_OTG_HostTypeDef;

/**
  * @brief USB_OTG_Host_Channel_Specific_Registers
  */
typedef struct
{
  __IO uint32_t HCCHAR;           /*!< Host Channel Characteristics Register    500h */
  __IO uint32_t HCSPLT;           /*!< Host Channel Split Control Register      504h */
  __IO uint32_t HCINT;            /*!< Host Channel Interrupt Register          508h */
  __IO uint32_t HCINTMSK;         /*!< Host Channel Interrupt Mask Register     50Ch */
  __IO uint32_t HCTSIZ;           /*!< Host Channel Transfer Size Register      510h */
  __IO uint32_t HCDMA;            /*!< Host Channel DMA Address Register        514h */
  uint32_t Reserved[2];           /*!< Reserved                                      */
} USB_OTG_HostChannelTypeDef;
/**
  * @}
  */

/** @addtogroup Peripheral_memory_map
  * @{
  */
#define D1_ITCMRAM_BASE           ((uint32_t)0x00000000) /*!< Base address of : 64KB RAM reserved for CPU execution/instruction accessible over ITCM  */
#define D1_ITCMICP_BASE           ((uint32_t)0x00100000) /*!< Base address of : (up to 128KB) embedded Test FLASH memory accessible over ITCM         */
#define D1_DTCMRAM_BASE           ((uint32_t)0x20000000) /*!< Base address of : 128KB system data RAM accessible over DTCM                            */
#define D1_AXIFLASH_BASE          ((uint32_t)0x08000000) /*!< Base address of : (up to 2 MB) embedded FLASH memory accessible over AXI                */
#define D1_AXIICP_BASE            ((uint32_t)0x1FF00000) /*!< Base address of : (up to 128KB) embedded Test FLASH memory accessible over AXI          */
#define D1_AXISRAM_BASE           ((uint32_t)0x24000000) /*!< Base address of : (up to 512KB) system data RAM accessible over over AXI                */

#define D2_AXISRAM_BASE           ((uint32_t)0x10000000) /*!< Base address of : (up to 288KB) system data RAM accessible over over AXI                */
#define D2_AHBSRAM_BASE           ((uint32_t)0x30000000) /*!< Base address of : (up to 288KB) system data RAM accessible over over AXI->AHB Bridge    */

#define D3_BKPSRAM_BASE           ((uint32_t)0x38800000) /*!< Base address of : Backup SRAM(4 KB) over AXI->AHB Bridge                                */
#define D3_SRAM_BASE              ((uint32_t)0x38000000) /*!< Base address of : Backup SRAM(64 KB) over AXI->AHB Bridge                               */

#define PERIPH_BASE               ((uint32_t)0x40000000) /*!< Base address of : AHB/ABP Peripherals                                                   */
#define QSPI_BASE                 ((uint32_t)0x90000000) /*!< Base address of : QSPI memories accessible over AXI                                     */

#define FLASH_BANK1_BASE          ((uint32_t)0x08000000) /*!< Base address of : Flash Bank1 accessible over AXI                                       */
#define FLASH_BANK2_BASE          ((uint32_t)0x08100000) /*!< Base address of : Flash Bank2 accessible over AXI                                       */
#define FLASH_END                 ((uint32_t)0x081FFFFF) /*!< FLASH end address                                                                       */

/* Legacy define */
#define FLASH_BASE                FLASH_BANK1_BASE


/*!< Peripheral memory map */
#define D2_APB1PERIPH_BASE        PERIPH_BASE
#define D2_APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)
#define D2_AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
#define D2_AHB2PERIPH_BASE       (PERIPH_BASE + 0x08020000)

#define D1_APB1PERIPH_BASE       (PERIPH_BASE + 0x10000000)
#define D1_AHB1PERIPH_BASE       (PERIPH_BASE + 0x12000000)

#define D3_APB1PERIPH_BASE       (PERIPH_BASE + 0x18000000)
#define D3_AHB1PERIPH_BASE       (PERIPH_BASE + 0x18020000)

/*!< Legacy Peripheral memory map */
#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000)

/*!< D1_AHB1PERIPH peripherals */

#define MDMA_BASE             (D1_AHB1PERIPH_BASE + 0x0000)
#define DMA2D_BASE            (D1_AHB1PERIPH_BASE + 0x1000)
#define JPGDEC_BASE           (D1_AHB1PERIPH_BASE + 0x3000)
#define FLASH_R_BASE          (D1_AHB1PERIPH_BASE + 0x2000)
#define FMC_R_BASE            (D1_AHB1PERIPH_BASE + 0x4000)
#define QSPI_R_BASE           (D1_AHB1PERIPH_BASE + 0x5000)
#define DLYB_QSPI_BASE        (D1_AHB1PERIPH_BASE + 0x6000)
#define SDMMC1_BASE           (D1_AHB1PERIPH_BASE + 0x7000)
#define DLYB_SDMMC1_BASE      (D1_AHB1PERIPH_BASE + 0x8000)

/*!< D2_AHB1PERIPH peripherals */

#define DMA1_BASE               (D2_AHB1PERIPH_BASE + 0x0000)
#define DMA2_BASE               (D2_AHB1PERIPH_BASE + 0x0400)
#define DMAMUX1_BASE            (D2_AHB1PERIPH_BASE + 0x0800)
#define ADC1_BASE               (D2_AHB1PERIPH_BASE + 0x2000)
#define ADC2_BASE               (D2_AHB1PERIPH_BASE + 0x2100)
#define ADC12_COMMON_BASE       (D2_AHB1PERIPH_BASE + 0x2300)
#define ART_BASE                (D2_AHB1PERIPH_BASE + 0x4400)
#define ETH_BASE                (D2_AHB1PERIPH_BASE + 0x8000)
#define ETH_MAC_BASE            (ETH_BASE)

/*!< USB registers base address */
#define USB1_OTG_HS_PERIPH_BASE              ((uint32_t )0x40040000)
#define USB2_OTG_FS_PERIPH_BASE              ((uint32_t )0x40080000)
#define USB_OTG_GLOBAL_BASE                  ((uint32_t )0x000)
#define USB_OTG_DEVICE_BASE                  ((uint32_t )0x800)
#define USB_OTG_IN_ENDPOINT_BASE             ((uint32_t )0x900)
#define USB_OTG_OUT_ENDPOINT_BASE            ((uint32_t )0xB00)
#define USB_OTG_EP_REG_SIZE                  ((uint32_t )0x20)
#define USB_OTG_HOST_BASE                    ((uint32_t )0x400)
#define USB_OTG_HOST_PORT_BASE               ((uint32_t )0x440)
#define USB_OTG_HOST_CHANNEL_BASE            ((uint32_t )0x500)
#define USB_OTG_HOST_CHANNEL_SIZE            ((uint32_t )0x20)
#define USB_OTG_PCGCCTL_BASE                 ((uint32_t )0xE00)
#define USB_OTG_FIFO_BASE                    ((uint32_t )0x1000)
#define USB_OTG_FIFO_SIZE                    ((uint32_t )0x1000)

/*!< D2_AHB2PERIPH peripherals */

#define DCMI_BASE              (D2_AHB2PERIPH_BASE + 0x0000)
#define RNG_BASE               (D2_AHB2PERIPH_BASE + 0x1800)
#define SDMMC2_BASE            (D2_AHB2PERIPH_BASE + 0x2400)
#define DLYB_SDMMC2_BASE       (D2_AHB2PERIPH_BASE + 0x2800)


/*!< D3_AHB1PERIPH peripherals */
#define GPIOA_BASE            (D3_AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASE            (D3_AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE            (D3_AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASE            (D3_AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASE            (D3_AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASE            (D3_AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASE            (D3_AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASE            (D3_AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASE            (D3_AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASE            (D3_AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASE            (D3_AHB1PERIPH_BASE + 0x2800)
#define RCC_BASE              (D3_AHB1PERIPH_BASE + 0x4400)
#define RCC_C1_BASE           (RCC_BASE + 0x130)
#define PWR_BASE              (D3_AHB1PERIPH_BASE + 0x4800)
#define CRC_BASE              (D3_AHB1PERIPH_BASE + 0x4C00)
#define BDMA_BASE             (D3_AHB1PERIPH_BASE + 0x5400)
#define DMAMUX2_BASE          (D3_AHB1PERIPH_BASE + 0x5800)
#define ADC3_BASE             (D3_AHB1PERIPH_BASE + 0x6000)
#define ADC3_COMMON_BASE      (D3_AHB1PERIPH_BASE + 0x6300)
#define HSEM_BASE             (D3_AHB1PERIPH_BASE + 0x6400)

/*!< D1_APB1PERIPH peripherals */
#define LTDC_BASE             (D1_APB1PERIPH_BASE + 0x1000)
#define LTDC_Layer1_BASE      (LTDC_BASE + 0x84)
#define LTDC_Layer2_BASE      (LTDC_BASE + 0x104)
#define WWDG1_BASE            (D1_APB1PERIPH_BASE + 0x3000)

/*!< D2_APB1PERIPH peripherals */
#define TIM2_BASE             (D2_APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE             (D2_APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE             (D2_APB1PERIPH_BASE + 0x0800)
#define TIM5_BASE             (D2_APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASE             (D2_APB1PERIPH_BASE + 0x1000)
#define TIM7_BASE             (D2_APB1PERIPH_BASE + 0x1400)
#define TIM12_BASE            (D2_APB1PERIPH_BASE + 0x1800)
#define TIM13_BASE            (D2_APB1PERIPH_BASE + 0x1C00)
#define TIM14_BASE            (D2_APB1PERIPH_BASE + 0x2000)
#define LPTIM1_BASE           (D2_APB1PERIPH_BASE + 0x2400)


#define SPI2_BASE             (D2_APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE             (D2_APB1PERIPH_BASE + 0x3C00)
#define SPDIFRX_BASE          (D2_APB1PERIPH_BASE + 0x4000)
#define USART2_BASE           (D2_APB1PERIPH_BASE + 0x4400)
#define USART3_BASE           (D2_APB1PERIPH_BASE + 0x4800)
#define UART4_BASE            (D2_APB1PERIPH_BASE + 0x4C00)
#define UART5_BASE            (D2_APB1PERIPH_BASE + 0x5000)
#define I2C1_BASE             (D2_APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE             (D2_APB1PERIPH_BASE + 0x5800)
#define I2C3_BASE             (D2_APB1PERIPH_BASE + 0x5C00)
#define CEC_BASE              (D2_APB1PERIPH_BASE + 0x6C00)
#define DAC1_BASE             (D2_APB1PERIPH_BASE + 0x7400)
#define UART7_BASE            (D2_APB1PERIPH_BASE + 0x7800)
#define UART8_BASE            (D2_APB1PERIPH_BASE + 0x7C00)
#define CRS_BASE              (D2_APB1PERIPH_BASE + 0x8400)
#define SWPMI1_BASE           (D2_APB1PERIPH_BASE + 0x8800)
#define OPAMP_BASE            (D2_APB1PERIPH_BASE + 0x9000)
#define OPAMP1_BASE           (D2_APB1PERIPH_BASE + 0x9000)
#define OPAMP2_BASE           (D2_APB1PERIPH_BASE + 0x9010)
#define MDIOS_BASE            (D2_APB1PERIPH_BASE + 0x9400)
#define FDCAN1_BASE           (D2_APB1PERIPH_BASE + 0xA000)
#define FDCAN2_BASE           (D2_APB1PERIPH_BASE + 0xA400)
#define FDCAN_CCU_BASE        (D2_APB1PERIPH_BASE + 0xA800)
#define SRAMCAN_BASE          (D2_APB1PERIPH_BASE + 0xAC00)

/*!< D2_APB2PERIPH peripherals */

#define TIM1_BASE             (D2_APB2PERIPH_BASE + 0x0000)
#define TIM8_BASE             (D2_APB2PERIPH_BASE + 0x0400)
#define USART1_BASE           (D2_APB2PERIPH_BASE + 0x1000)
#define USART6_BASE           (D2_APB2PERIPH_BASE + 0x1400)
#define SPI1_BASE             (D2_APB2PERIPH_BASE + 0x3000)
#define SPI4_BASE             (D2_APB2PERIPH_BASE + 0x3400)
#define TIM15_BASE            (D2_APB2PERIPH_BASE + 0x4000)
#define TIM16_BASE            (D2_APB2PERIPH_BASE + 0x4400)
#define TIM17_BASE            (D2_APB2PERIPH_BASE + 0x4800)
#define SPI5_BASE             (D2_APB2PERIPH_BASE + 0x5000)
#define SAI1_BASE             (D2_APB2PERIPH_BASE + 0x5800)
#define SAI1_Block_A_BASE     (SAI1_BASE + 0x004)
#define SAI1_Block_B_BASE     (SAI1_BASE + 0x024)
#define SAI2_BASE             (D2_APB2PERIPH_BASE + 0x5C00)
#define SAI2_Block_A_BASE     (SAI2_BASE + 0x004)
#define SAI2_Block_B_BASE     (SAI2_BASE + 0x024)
#define SAI3_BASE             (D2_APB2PERIPH_BASE + 0x6000)
#define SAI3_Block_A_BASE     (SAI3_BASE + 0x004)
#define SAI3_Block_B_BASE     (SAI3_BASE + 0x024)
#define DFSDM1_BASE           (D2_APB2PERIPH_BASE + 0x7000)
#define DFSDM1_Channel0_BASE  (DFSDM1_BASE + 0x00)
#define DFSDM1_Channel1_BASE  (DFSDM1_BASE + 0x20)
#define DFSDM1_Channel2_BASE  (DFSDM1_BASE + 0x40)
#define DFSDM1_Channel3_BASE  (DFSDM1_BASE + 0x60)
#define DFSDM1_Channel4_BASE  (DFSDM1_BASE + 0x80)
#define DFSDM1_Channel5_BASE  (DFSDM1_BASE + 0xA0)
#define DFSDM1_Channel6_BASE  (DFSDM1_BASE + 0xC0)
#define DFSDM1_Channel7_BASE  (DFSDM1_BASE + 0xE0)
#define DFSDM1_Filter0_BASE   (DFSDM1_BASE + 0x100)
#define DFSDM1_Filter1_BASE   (DFSDM1_BASE + 0x180)
#define DFSDM1_Filter2_BASE   (DFSDM1_BASE + 0x200)
#define DFSDM1_Filter3_BASE   (DFSDM1_BASE + 0x280)
#define HRTIM1_BASE           (D2_APB2PERIPH_BASE + 0x7400)
#define HRTIM1_TIMA_BASE      (HRTIM1_BASE + 0x00000080)
#define HRTIM1_TIMB_BASE      (HRTIM1_BASE + 0x00000100)
#define HRTIM1_TIMC_BASE      (HRTIM1_BASE + 0x00000180)
#define HRTIM1_TIMD_BASE      (HRTIM1_BASE + 0x00000200)
#define HRTIM1_TIME_BASE      (HRTIM1_BASE + 0x00000280)
#define HRTIM1_COMMON_BASE    (HRTIM1_BASE + 0x00000380)


/*!< D3_APB1PERIPH peripherals */
#define EXTI_BASE             (D3_APB1PERIPH_BASE + 0x0000)
#define EXTI_D1_BASE          (EXTI_BASE + 0x0080)
#define EXTI_D2_BASE          (EXTI_BASE + 0x00C0)
#define SYSCFG_BASE           (D3_APB1PERIPH_BASE + 0x0400)
#define LPUART1_BASE          (D3_APB1PERIPH_BASE + 0x0C00)
#define SPI6_BASE             (D3_APB1PERIPH_BASE + 0x1400)
#define I2C4_BASE             (D3_APB1PERIPH_BASE + 0x1C00)
#define LPTIM2_BASE           (D3_APB1PERIPH_BASE + 0x2400)
#define LPTIM3_BASE           (D3_APB1PERIPH_BASE + 0x2800)
#define LPTIM4_BASE           (D3_APB1PERIPH_BASE + 0x2C00)
#define LPTIM5_BASE           (D3_APB1PERIPH_BASE + 0x3000)
#define COMP12_BASE           (D3_APB1PERIPH_BASE + 0x3800)
#define COMP1_BASE            (COMP12_BASE + 0x0C)
#define COMP2_BASE            (COMP12_BASE + 0x10)
#define VREFBUF_BASE          (D3_APB1PERIPH_BASE + 0x3C00)
#define RTC_BASE              (D3_APB1PERIPH_BASE + 0x4000)
#define IWDG1_BASE            (D3_APB1PERIPH_BASE + 0x4800)


#define SAI4_BASE             (D3_APB1PERIPH_BASE + 0x5400)
#define SAI4_Block_A_BASE     (SAI4_BASE + 0x004)
#define SAI4_Block_B_BASE     (SAI4_BASE + 0x024)


#define BDMA_Channel0_BASE    (BDMA_BASE + 0x0008)
#define BDMA_Channel1_BASE    (BDMA_BASE + 0x001C)
#define BDMA_Channel2_BASE    (BDMA_BASE + 0x0030)
#define BDMA_Channel3_BASE    (BDMA_BASE + 0x0044)
#define BDMA_Channel4_BASE    (BDMA_BASE + 0x0058)
#define BDMA_Channel5_BASE    (BDMA_BASE + 0x006C)
#define BDMA_Channel6_BASE    (BDMA_BASE + 0x0080)
#define BDMA_Channel7_BASE    (BDMA_BASE + 0x0094)

#define DMAMUX2_Channel0_BASE    (DMAMUX2_BASE)
#define DMAMUX2_Channel1_BASE    (DMAMUX2_BASE + 0x0004)
#define DMAMUX2_Channel2_BASE    (DMAMUX2_BASE + 0x0008)
#define DMAMUX2_Channel3_BASE    (DMAMUX2_BASE + 0x000C)
#define DMAMUX2_Channel4_BASE    (DMAMUX2_BASE + 0x0010)
#define DMAMUX2_Channel5_BASE    (DMAMUX2_BASE + 0x0014)
#define DMAMUX2_Channel6_BASE    (DMAMUX2_BASE + 0x0018)
#define DMAMUX2_Channel7_BASE    (DMAMUX2_BASE + 0x001C)

#define DMAMUX2_RequestGenerator0_BASE  (DMAMUX2_BASE + 0x0100)
#define DMAMUX2_RequestGenerator1_BASE  (DMAMUX2_BASE + 0x0104)
#define DMAMUX2_RequestGenerator2_BASE  (DMAMUX2_BASE + 0x0108)
#define DMAMUX2_RequestGenerator3_BASE  (DMAMUX2_BASE + 0x010C)
#define DMAMUX2_RequestGenerator4_BASE  (DMAMUX2_BASE + 0x0110)
#define DMAMUX2_RequestGenerator5_BASE  (DMAMUX2_BASE + 0x0114)
#define DMAMUX2_RequestGenerator6_BASE  (DMAMUX2_BASE + 0x0118)
#define DMAMUX2_RequestGenerator7_BASE  (DMAMUX2_BASE + 0x011C)

#define DMAMUX2_ChannelStatus_BASE      (DMAMUX2_BASE + 0x0080)
#define DMAMUX2_RequestGenStatus_BASE   (DMAMUX2_BASE + 0x0140)

#define DMA1_Stream0_BASE     (DMA1_BASE + 0x010)
#define DMA1_Stream1_BASE     (DMA1_BASE + 0x028)
#define DMA1_Stream2_BASE     (DMA1_BASE + 0x040)
#define DMA1_Stream3_BASE     (DMA1_BASE + 0x058)
#define DMA1_Stream4_BASE     (DMA1_BASE + 0x070)
#define DMA1_Stream5_BASE     (DMA1_BASE + 0x088)
#define DMA1_Stream6_BASE     (DMA1_BASE + 0x0A0)
#define DMA1_Stream7_BASE     (DMA1_BASE + 0x0B8)

#define DMA2_Stream0_BASE     (DMA2_BASE + 0x010)
#define DMA2_Stream1_BASE     (DMA2_BASE + 0x028)
#define DMA2_Stream2_BASE     (DMA2_BASE + 0x040)
#define DMA2_Stream3_BASE     (DMA2_BASE + 0x058)
#define DMA2_Stream4_BASE     (DMA2_BASE + 0x070)
#define DMA2_Stream5_BASE     (DMA2_BASE + 0x088)
#define DMA2_Stream6_BASE     (DMA2_BASE + 0x0A0)
#define DMA2_Stream7_BASE     (DMA2_BASE + 0x0B8)

#define DMAMUX1_Channel0_BASE    (DMAMUX1_BASE)
#define DMAMUX1_Channel1_BASE    (DMAMUX1_BASE + 0x0004)
#define DMAMUX1_Channel2_BASE    (DMAMUX1_BASE + 0x0008)
#define DMAMUX1_Channel3_BASE    (DMAMUX1_BASE + 0x000C)
#define DMAMUX1_Channel4_BASE    (DMAMUX1_BASE + 0x0010)
#define DMAMUX1_Channel5_BASE    (DMAMUX1_BASE + 0x0014)
#define DMAMUX1_Channel6_BASE    (DMAMUX1_BASE + 0x0018)
#define DMAMUX1_Channel7_BASE    (DMAMUX1_BASE + 0x001C)
#define DMAMUX1_Channel8_BASE    (DMAMUX1_BASE + 0x0020)
#define DMAMUX1_Channel9_BASE    (DMAMUX1_BASE + 0x0024)
#define DMAMUX1_Channel10_BASE   (DMAMUX1_BASE + 0x0028)
#define DMAMUX1_Channel11_BASE   (DMAMUX1_BASE + 0x002C)
#define DMAMUX1_Channel12_BASE   (DMAMUX1_BASE + 0x0030)
#define DMAMUX1_Channel13_BASE   (DMAMUX1_BASE + 0x0034)
#define DMAMUX1_Channel14_BASE   (DMAMUX1_BASE + 0x0038)
#define DMAMUX1_Channel15_BASE   (DMAMUX1_BASE + 0x003C)

#define DMAMUX1_RequestGenerator0_BASE  (DMAMUX1_BASE + 0x0100)
#define DMAMUX1_RequestGenerator1_BASE  (DMAMUX1_BASE + 0x0104)
#define DMAMUX1_RequestGenerator2_BASE  (DMAMUX1_BASE + 0x0108)
#define DMAMUX1_RequestGenerator3_BASE  (DMAMUX1_BASE + 0x010C)
#define DMAMUX1_RequestGenerator4_BASE  (DMAMUX1_BASE + 0x0110)
#define DMAMUX1_RequestGenerator5_BASE  (DMAMUX1_BASE + 0x0114)
#define DMAMUX1_RequestGenerator6_BASE  (DMAMUX1_BASE + 0x0118)
#define DMAMUX1_RequestGenerator7_BASE  (DMAMUX1_BASE + 0x011C)

#define DMAMUX1_ChannelStatus_BASE      (DMAMUX1_BASE + 0x0080)
#define DMAMUX1_RequestGenStatus_BASE   (DMAMUX1_BASE + 0x0140)



/*!< FMC Banks registers base  address */
#define FMC_Bank1_R_BASE      (FMC_R_BASE + 0x0000)
#define FMC_Bank1E_R_BASE     (FMC_R_BASE + 0x0104)
#define FMC_Bank2_R_BASE      (FMC_R_BASE + 0x0060)
#define FMC_Bank3_R_BASE      (FMC_R_BASE + 0x0080)
#define FMC_Bank5_6_R_BASE    (FMC_R_BASE + 0x0140)

/* Debug MCU registers base address */
#define DBGMCU_BASE           ((uint32_t )0x5C001000)

#define MDMA_Channel0_BASE    (MDMA_BASE + 0x00000040)
#define MDMA_Channel1_BASE    (MDMA_BASE + 0x00000080)
#define MDMA_Channel2_BASE    (MDMA_BASE + 0x000000C0)
#define MDMA_Channel3_BASE    (MDMA_BASE + 0x00000100)
#define MDMA_Channel4_BASE    (MDMA_BASE + 0x00000140)
#define MDMA_Channel5_BASE    (MDMA_BASE + 0x00000180)
#define MDMA_Channel6_BASE    (MDMA_BASE + 0x000001C0)
#define MDMA_Channel7_BASE    (MDMA_BASE + 0x00000200)
#define MDMA_Channel8_BASE    (MDMA_BASE + 0x00000240)
#define MDMA_Channel9_BASE    (MDMA_BASE + 0x00000280)
#define MDMA_Channel10_BASE   (MDMA_BASE + 0x000002C0)
#define MDMA_Channel11_BASE   (MDMA_BASE + 0x00000300)
#define MDMA_Channel12_BASE   (MDMA_BASE + 0x00000340)
#define MDMA_Channel13_BASE   (MDMA_BASE + 0x00000380)
#define MDMA_Channel14_BASE   (MDMA_BASE + 0x000003C0)
#define MDMA_Channel15_BASE   (MDMA_BASE + 0x00000400)


// #define FDCAN1              ((FDCAN_GlobalTypeDef *) FDCAN1_BASE)
// #define FDCAN2              ((FDCAN_GlobalTypeDef *) FDCAN2_BASE)
// #define FDCAN_CCU           ((FDCAN_ClockCalibrationUnit_TypeDef *) FDCAN_CCU_BASE)


/**
  * @}
  */

/** @addtogroup Exported_constants
  * @{
  */

  /** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */

/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/


/******************************************************************************/
/*                                                                            */
/*                 Flexible Datarate Controller Area Network                  */
/*                                                                            */
/******************************************************************************/
/*!<FDCAN control and status registers */
/*****************  Bit definition for FDCAN_CREL register  *******************/
#define FDCAN_CREL_DAY_Pos        (0U)
#define FDCAN_CREL_DAY_Msk        (0xFFU << FDCAN_CREL_DAY_Pos)                /*!< 0x000000FF */
#define FDCAN_CREL_DAY            FDCAN_CREL_DAY_Msk                           /*!<Timestamp Day                           */
#define FDCAN_CREL_MON_Pos        (8U)
#define FDCAN_CREL_MON_Msk        (0xFFU << FDCAN_CREL_MON_Pos)                /*!< 0x0000FF00 */
#define FDCAN_CREL_MON            FDCAN_CREL_MON_Msk                           /*!<Timestamp Month                         */
#define FDCAN_CREL_YEAR_Pos       (16U)
#define FDCAN_CREL_YEAR_Msk       (0xFU << FDCAN_CREL_YEAR_Pos)                /*!< 0x000F0000 */
#define FDCAN_CREL_YEAR           FDCAN_CREL_YEAR_Msk                          /*!<Timestamp Year                          */
#define FDCAN_CREL_SUBSTEP_Pos    (20U)
#define FDCAN_CREL_SUBSTEP_Msk    (0xFU << FDCAN_CREL_SUBSTEP_Pos)             /*!< 0x00F00000 */
#define FDCAN_CREL_SUBSTEP        FDCAN_CREL_SUBSTEP_Msk                       /*!<Sub-step of Core release                */
#define FDCAN_CREL_STEP_Pos       (24U)
#define FDCAN_CREL_STEP_Msk       (0xFU << FDCAN_CREL_STEP_Pos)                /*!< 0x0F000000 */
#define FDCAN_CREL_STEP           FDCAN_CREL_STEP_Msk                          /*!<Step of Core release                    */
#define FDCAN_CREL_REL_Pos        (28U)
#define FDCAN_CREL_REL_Msk        (0xFU << FDCAN_CREL_REL_Pos)                 /*!< 0xF0000000 */
#define FDCAN_CREL_REL            FDCAN_CREL_REL_Msk                           /*!<Core release                            */

/*****************  Bit definition for FDCAN_ENDN register  *******************/
#define FDCAN_ENDN_ETV_Pos        (0U)
#define FDCAN_ENDN_ETV_Msk        (0xFFFFFFFFU << FDCAN_ENDN_ETV_Pos)          /*!< 0xFFFFFFFF */
#define FDCAN_ENDN_ETV            FDCAN_ENDN_ETV_Msk                           /*!<Endiannes Test Value                    */

/*****************  Bit definition for FDCAN_DBTP register  *******************/
#define FDCAN_DBTP_DSJW_Pos       (0U)
#define FDCAN_DBTP_DSJW_Msk       (0xFU << FDCAN_DBTP_DSJW_Pos)                /*!< 0x0000000F */
#define FDCAN_DBTP_DSJW           FDCAN_DBTP_DSJW_Msk                          /*!<Synchronization Jump Width              */
#define FDCAN_DBTP_DTSEG2_Pos     (4U)
#define FDCAN_DBTP_DTSEG2_Msk     (0xFU << FDCAN_DBTP_DTSEG2_Pos)              /*!< 0x000000F0 */
#define FDCAN_DBTP_DTSEG2         FDCAN_DBTP_DTSEG2_Msk                        /*!<Data time segment after sample point    */
#define FDCAN_DBTP_DTSEG1_Pos     (8U)
#define FDCAN_DBTP_DTSEG1_Msk     (0xFU << FDCAN_DBTP_DTSEG1_Pos)              /*!< 0x00000F00 */
#define FDCAN_DBTP_DTSEG1         FDCAN_DBTP_DTSEG1_Msk                        /*!<Data time segment before sample point   */
#define FDCAN_DBTP_DBRP_Pos       (16U)
#define FDCAN_DBTP_DBRP_Msk       (0x1FU << FDCAN_DBTP_DBRP_Pos)               /*!< 0x001F0000 */
#define FDCAN_DBTP_DBRP           FDCAN_DBTP_DBRP_Msk                          /*!<Data BIt Rate Prescaler                 */
#define FDCAN_DBTP_TDC_Pos        (23U)
#define FDCAN_DBTP_TDC_Msk        (0x1U << FDCAN_DBTP_TDC_Pos)                 /*!< 0x00800000 */
#define FDCAN_DBTP_TDC            FDCAN_DBTP_TDC_Msk                           /*!<Transceiver Delay Compensation          */

/*****************  Bit definition for FDCAN_TEST register  *******************/
#define FDCAN_TEST_LBCK_Pos       (4U)
#define FDCAN_TEST_LBCK_Msk       (0x1U << FDCAN_TEST_LBCK_Pos)                /*!< 0x00000010 */
#define FDCAN_TEST_LBCK           FDCAN_TEST_LBCK_Msk                          /*!<Loop Back mode                           */
#define FDCAN_TEST_TX_Pos         (5U)
#define FDCAN_TEST_TX_Msk         (0x3U << FDCAN_TEST_TX_Pos)                  /*!< 0x00000060 */
#define FDCAN_TEST_TX             FDCAN_TEST_TX_Msk                            /*!<Control of Transmit Pin                  */
#define FDCAN_TEST_RX_Pos         (7U)
#define FDCAN_TEST_RX_Msk         (0x1U << FDCAN_TEST_RX_Pos)                  /*!< 0x00000080 */
#define FDCAN_TEST_RX             FDCAN_TEST_RX_Msk                            /*!<Receive Pin                              */

/*****************  Bit definition for FDCAN_RWD register  ********************/
#define FDCAN_RWD_WDC_Pos         (0U)
#define FDCAN_RWD_WDC_Msk         (0xFU << FDCAN_RWD_WDC_Pos)                  /*!< 0x0000000F */
#define FDCAN_RWD_WDC             FDCAN_RWD_WDC_Msk                            /*!<Watchdog configuration                   */
#define FDCAN_RWD_WDV_Pos         (4U)
#define FDCAN_RWD_WDV_Msk         (0xFU << FDCAN_RWD_WDV_Pos)                  /*!< 0x000000F0 */
#define FDCAN_RWD_WDV             FDCAN_RWD_WDV_Msk                            /*!<Watchdog value                           */

/*****************  Bit definition for FDCAN_CCCR register  ********************/
#define FDCAN_CCCR_INIT_Pos       (0U)
#define FDCAN_CCCR_INIT_Msk       (0x1U << FDCAN_CCCR_INIT_Pos)                /*!< 0x00000001 */
#define FDCAN_CCCR_INIT           FDCAN_CCCR_INIT_Msk                          /*!<Initialization                           */
#define FDCAN_CCCR_CCE_Pos        (1U)
#define FDCAN_CCCR_CCE_Msk        (0x1U << FDCAN_CCCR_CCE_Pos)                 /*!< 0x00000002 */
#define FDCAN_CCCR_CCE            FDCAN_CCCR_CCE_Msk                           /*!<Configuration Change Enable              */
#define FDCAN_CCCR_ASM_Pos        (2U)
#define FDCAN_CCCR_ASM_Msk        (0x1U << FDCAN_CCCR_ASM_Pos)                 /*!< 0x00000004 */
#define FDCAN_CCCR_ASM            FDCAN_CCCR_ASM_Msk                           /*!<ASM Restricted Operation Mode            */
#define FDCAN_CCCR_CSA_Pos        (3U)
#define FDCAN_CCCR_CSA_Msk        (0x1U << FDCAN_CCCR_CSA_Pos)                 /*!< 0x00000008 */
#define FDCAN_CCCR_CSA            FDCAN_CCCR_CSA_Msk                           /*!<Clock Stop Acknowledge                   */
#define FDCAN_CCCR_CSR_Pos        (4U)
#define FDCAN_CCCR_CSR_Msk        (0x1U << FDCAN_CCCR_CSR_Pos)                 /*!< 0x00000010 */
#define FDCAN_CCCR_CSR            FDCAN_CCCR_CSR_Msk                           /*!<Clock Stop Request                       */
#define FDCAN_CCCR_MON_Pos        (5U)
#define FDCAN_CCCR_MON_Msk        (0x1U << FDCAN_CCCR_MON_Pos)                 /*!< 0x00000020 */
#define FDCAN_CCCR_MON            FDCAN_CCCR_MON_Msk                           /*!<Bus Monitoring Mode                      */
#define FDCAN_CCCR_DAR_Pos        (6U)
#define FDCAN_CCCR_DAR_Msk        (0x1U << FDCAN_CCCR_DAR_Pos)                 /*!< 0x00000040 */
#define FDCAN_CCCR_DAR            FDCAN_CCCR_DAR_Msk                           /*!<Disable Automatic Retransmission         */
#define FDCAN_CCCR_TEST_Pos       (7U)
#define FDCAN_CCCR_TEST_Msk       (0x1U << FDCAN_CCCR_TEST_Pos)                /*!< 0x00000080 */
#define FDCAN_CCCR_TEST           FDCAN_CCCR_TEST_Msk                          /*!<Test Mode Enable                         */
#define FDCAN_CCCR_FDOE_Pos       (8U)
#define FDCAN_CCCR_FDOE_Msk       (0x1U << FDCAN_CCCR_FDOE_Pos)                /*!< 0x00000100 */
#define FDCAN_CCCR_FDOE           FDCAN_CCCR_FDOE_Msk                          /*!<FD Operation Enable                      */
#define FDCAN_CCCR_BRSE_Pos       (9U)
#define FDCAN_CCCR_BRSE_Msk       (0x1U << FDCAN_CCCR_BRSE_Pos)                /*!< 0x00000200 */
#define FDCAN_CCCR_BRSE           FDCAN_CCCR_BRSE_Msk                          /*!<FDCAN Bit Rate Switching                 */
#define FDCAN_CCCR_PXHD_Pos       (12U)
#define FDCAN_CCCR_PXHD_Msk       (0x1U << FDCAN_CCCR_PXHD_Pos)                /*!< 0x00001000 */
#define FDCAN_CCCR_PXHD           FDCAN_CCCR_PXHD_Msk                          /*!<Protocol Exception Handling Disable      */
#define FDCAN_CCCR_EFBI_Pos       (13U)
#define FDCAN_CCCR_EFBI_Msk       (0x1U << FDCAN_CCCR_EFBI_Pos)                /*!< 0x00002000 */
#define FDCAN_CCCR_EFBI           FDCAN_CCCR_EFBI_Msk                          /*!<Edge Filtering during Bus Integration    */
#define FDCAN_CCCR_TXP_Pos        (14U)
#define FDCAN_CCCR_TXP_Msk        (0x1U << FDCAN_CCCR_TXP_Pos)                 /*!< 0x00004000 */
#define FDCAN_CCCR_TXP            FDCAN_CCCR_TXP_Msk                           /*!<Two CAN bit times Pause                  */
#define FDCAN_CCCR_NISO_Pos       (15U)
#define FDCAN_CCCR_NISO_Msk       (0x1U << FDCAN_CCCR_NISO_Pos)                /*!< 0x00008000 */
#define FDCAN_CCCR_NISO           FDCAN_CCCR_NISO_Msk                          /*!<Non ISO Operation                        */

/*****************  Bit definition for FDCAN_NBTP register  ********************/
#define FDCAN_NBTP_TSEG2_Pos      (0U)
#define FDCAN_NBTP_TSEG2_Msk      (0x7FU << FDCAN_NBTP_TSEG2_Pos)              /*!< 0x0000007F */
#define FDCAN_NBTP_TSEG2          FDCAN_NBTP_TSEG2_Msk                         /*!<Nominal Time segment after sample point  */
#define FDCAN_NBTP_NTSEG1_Pos     (8U)
#define FDCAN_NBTP_NTSEG1_Msk     (0xFFU << FDCAN_NBTP_NTSEG1_Pos)             /*!< 0x0000FF00 */
#define FDCAN_NBTP_NTSEG1         FDCAN_NBTP_NTSEG1_Msk                        /*!<Nominal Time segment before sample point */
#define FDCAN_NBTP_NBRP_Pos       (16U)
#define FDCAN_NBTP_NBRP_Msk       (0x1FFU << FDCAN_NBTP_NBRP_Pos)              /*!< 0x01FF0000 */
#define FDCAN_NBTP_NBRP           FDCAN_NBTP_NBRP_Msk                          /*!<Bit Rate Prescaler                       */
#define FDCAN_NBTP_NSJW_Pos       (25U)
#define FDCAN_NBTP_NSJW_Msk       (0x7FU << FDCAN_NBTP_NSJW_Pos)               /*!< 0xFE000000 */
#define FDCAN_NBTP_NSJW           FDCAN_NBTP_NSJW_Msk                          /*!<Nominal (Re)Synchronization Jump Width   */

/*****************  Bit definition for FDCAN_TSCC register  ********************/
#define FDCAN_TSCC_TSS_Pos        (0U)
#define FDCAN_TSCC_TSS_Msk        (0x3U << FDCAN_TSCC_TSS_Pos)                 /*!< 0x00000003 */
#define FDCAN_TSCC_TSS            FDCAN_TSCC_TSS_Msk                           /*!<Timestamp Select                         */
#define FDCAN_TSCC_TCP_Pos        (16U)
#define FDCAN_TSCC_TCP_Msk        (0xFU << FDCAN_TSCC_TCP_Pos)                 /*!< 0x000F0000 */
#define FDCAN_TSCC_TCP            FDCAN_TSCC_TCP_Msk                           /*!<Timestamp Counter Prescaler              */

/*****************  Bit definition for FDCAN_TSCV register  ********************/
#define FDCAN_TSCV_TSC_Pos        (0U)
#define FDCAN_TSCV_TSC_Msk        (0xFFFFU << FDCAN_TSCV_TSC_Pos)              /*!< 0x0000FFFF */
#define FDCAN_TSCV_TSC            FDCAN_TSCV_TSC_Msk                           /*!<Timestamp Counter                        */

/*****************  Bit definition for FDCAN_TOCC register  ********************/
#define FDCAN_TOCC_ETOC_Pos       (0U)
#define FDCAN_TOCC_ETOC_Msk       (0x1U << FDCAN_TOCC_ETOC_Pos)                /*!< 0x00000001 */
#define FDCAN_TOCC_ETOC           FDCAN_TOCC_ETOC_Msk                          /*!<Enable Timeout Counter                   */
#define FDCAN_TOCC_TOS_Pos        (1U)
#define FDCAN_TOCC_TOS_Msk        (0x3U << FDCAN_TOCC_TOS_Pos)                 /*!< 0x00000006 */
#define FDCAN_TOCC_TOS            FDCAN_TOCC_TOS_Msk                           /*!<Timeout Select                           */
#define FDCAN_TOCC_TOP_Pos        (16U)
#define FDCAN_TOCC_TOP_Msk        (0xFFFFU << FDCAN_TOCC_TOP_Pos)              /*!< 0xFFFF0000 */
#define FDCAN_TOCC_TOP            FDCAN_TOCC_TOP_Msk                           /*!<Timeout Period                           */

/*****************  Bit definition for FDCAN_TOCV register  ********************/
#define FDCAN_TOCV_TOC_Pos        (0U)
#define FDCAN_TOCV_TOC_Msk        (0xFFFFU << FDCAN_TOCV_TOC_Pos)              /*!< 0x0000FFFF */
#define FDCAN_TOCV_TOC            FDCAN_TOCV_TOC_Msk                           /*!<Timeout Counter                          */

/*****************  Bit definition for FDCAN_ECR register  *********************/
#define FDCAN_ECR_TEC_Pos         (0U)
#define FDCAN_ECR_TEC_Msk         (0xFU << FDCAN_ECR_TEC_Pos)                  /*!< 0x0000000F */
#define FDCAN_ECR_TEC             FDCAN_ECR_TEC_Msk                            /*!<Transmit Error Counter                   */
#define FDCAN_ECR_TREC_Pos        (8U)
#define FDCAN_ECR_TREC_Msk        (0x7FU << FDCAN_ECR_TREC_Pos)                /*!< 0x00007F00 */
#define FDCAN_ECR_TREC            FDCAN_ECR_TREC_Msk                           /*!<Receive Error Counter                    */
#define FDCAN_ECR_RP_Pos          (15U)
#define FDCAN_ECR_RP_Msk          (0x1U << FDCAN_ECR_RP_Pos)                   /*!< 0x00008000 */
#define FDCAN_ECR_RP              FDCAN_ECR_RP_Msk                             /*!<Receive Error Passive                    */
#define FDCAN_ECR_CEL_Pos         (16U)
#define FDCAN_ECR_CEL_Msk         (0xFFU << FDCAN_ECR_CEL_Pos)                 /*!< 0x00FF0000 */
#define FDCAN_ECR_CEL             FDCAN_ECR_CEL_Msk                            /*!<CAN Error Logging                        */

/*****************  Bit definition for FDCAN_PSR register  *********************/
#define FDCAN_PSR_LEC_Pos         (0U)
#define FDCAN_PSR_LEC_Msk         (0x7U << FDCAN_PSR_LEC_Pos)                  /*!< 0x00000007 */
#define FDCAN_PSR_LEC             FDCAN_PSR_LEC_Msk                            /*!<Last Error Code                          */
#define FDCAN_PSR_ACT_Pos         (3U)
#define FDCAN_PSR_ACT_Msk         (0x3U << FDCAN_PSR_ACT_Pos)                  /*!< 0x00000018 */
#define FDCAN_PSR_ACT             FDCAN_PSR_ACT_Msk                            /*!<Activity                                 */
#define FDCAN_PSR_EP_Pos          (5U)
#define FDCAN_PSR_EP_Msk          (0x1U << FDCAN_PSR_EP_Pos)                   /*!< 0x00000020 */
#define FDCAN_PSR_EP              FDCAN_PSR_EP_Msk                             /*!<Error Passive                            */
#define FDCAN_PSR_EW_Pos          (6U)
#define FDCAN_PSR_EW_Msk          (0x1U << FDCAN_PSR_EW_Pos)                   /*!< 0x00000040 */
#define FDCAN_PSR_EW              FDCAN_PSR_EW_Msk                             /*!<Warning Status                           */
#define FDCAN_PSR_BO_Pos          (7U)
#define FDCAN_PSR_BO_Msk          (0x1U << FDCAN_PSR_BO_Pos)                   /*!< 0x00000080 */
#define FDCAN_PSR_BO              FDCAN_PSR_BO_Msk                             /*!<Bus_Off Status                           */
#define FDCAN_PSR_DLEC_Pos        (8U)
#define FDCAN_PSR_DLEC_Msk        (0x7U << FDCAN_PSR_DLEC_Pos)                 /*!< 0x00000700 */
#define FDCAN_PSR_DLEC            FDCAN_PSR_DLEC_Msk                           /*!<Data Last Error Code                     */
#define FDCAN_PSR_RESI_Pos        (11U)
#define FDCAN_PSR_RESI_Msk        (0x1U << FDCAN_PSR_RESI_Pos)                 /*!< 0x00000800 */
#define FDCAN_PSR_RESI            FDCAN_PSR_RESI_Msk                           /*!<ESI flag of last received FDCAN Message  */
#define FDCAN_PSR_RBRS_Pos        (12U)
#define FDCAN_PSR_RBRS_Msk        (0x1U << FDCAN_PSR_RBRS_Pos)                 /*!< 0x00001000 */
#define FDCAN_PSR_RBRS            FDCAN_PSR_RBRS_Msk                           /*!<BRS flag of last received FDCAN Message  */
#define FDCAN_PSR_REDL_Pos        (13U)
#define FDCAN_PSR_REDL_Msk        (0x1U << FDCAN_PSR_REDL_Pos)                 /*!< 0x00002000 */
#define FDCAN_PSR_REDL            FDCAN_PSR_REDL_Msk                           /*!<Received FDCAN Message                   */
#define FDCAN_PSR_PXE_Pos         (14U)
#define FDCAN_PSR_PXE_Msk         (0x1U << FDCAN_PSR_PXE_Pos)                  /*!< 0x00004000 */
#define FDCAN_PSR_PXE             FDCAN_PSR_PXE_Msk                            /*!<Protocol Exception Event                 */
#define FDCAN_PSR_TDCV_Pos        (16U)
#define FDCAN_PSR_TDCV_Msk        (0x7FU << FDCAN_PSR_TDCV_Pos)                /*!< 0x007F0000 */
#define FDCAN_PSR_TDCV            FDCAN_PSR_TDCV_Msk                           /*!<Transmitter Delay Compensation Value     */

/*****************  Bit definition for FDCAN_TDCR register  ********************/
#define FDCAN_TDCR_TDCF_Pos       (0U)
#define FDCAN_TDCR_TDCF_Msk       (0x7FU << FDCAN_TDCR_TDCF_Pos)               /*!< 0x0000007F */
#define FDCAN_TDCR_TDCF           FDCAN_TDCR_TDCF_Msk                          /*!<Transmitter Delay Compensation Filter    */
#define FDCAN_TDCR_TDCO_Pos       (8U)
#define FDCAN_TDCR_TDCO_Msk       (0x7FU << FDCAN_TDCR_TDCO_Pos)               /*!< 0x00007F00 */
#define FDCAN_TDCR_TDCO           FDCAN_TDCR_TDCO_Msk                          /*!<Transmitter Delay Compensation Offset    */

/*****************  Bit definition for FDCAN_IR register  **********************/
#define FDCAN_IR_RF0N_Pos         (0U)
#define FDCAN_IR_RF0N_Msk         (0x1U << FDCAN_IR_RF0N_Pos)                  /*!< 0x00000001 */
#define FDCAN_IR_RF0N             FDCAN_IR_RF0N_Msk                            /*!<Rx FIFO 0 New Message                    */
#define FDCAN_IR_RF0W_Pos         (1U)
#define FDCAN_IR_RF0W_Msk         (0x1U << FDCAN_IR_RF0W_Pos)                  /*!< 0x00000002 */
#define FDCAN_IR_RF0W             FDCAN_IR_RF0W_Msk                            /*!<Rx FIFO 0 Watermark Reached              */
#define FDCAN_IR_RF0F_Pos         (2U)
#define FDCAN_IR_RF0F_Msk         (0x1U << FDCAN_IR_RF0F_Pos)                  /*!< 0x00000004 */
#define FDCAN_IR_RF0F             FDCAN_IR_RF0F_Msk                            /*!<Rx FIFO 0 Full                           */
#define FDCAN_IR_RF0L_Pos         (3U)
#define FDCAN_IR_RF0L_Msk         (0x1U << FDCAN_IR_RF0L_Pos)                  /*!< 0x00000008 */
#define FDCAN_IR_RF0L             FDCAN_IR_RF0L_Msk                            /*!<Rx FIFO 0 Message Lost                   */
#define FDCAN_IR_RF1N_Pos         (4U)
#define FDCAN_IR_RF1N_Msk         (0x1U << FDCAN_IR_RF1N_Pos)                  /*!< 0x00000010 */
#define FDCAN_IR_RF1N             FDCAN_IR_RF1N_Msk                            /*!<Rx FIFO 1 New Message                    */
#define FDCAN_IR_RF1W_Pos         (5U)
#define FDCAN_IR_RF1W_Msk         (0x1U << FDCAN_IR_RF1W_Pos)                  /*!< 0x00000020 */
#define FDCAN_IR_RF1W             FDCAN_IR_RF1W_Msk                            /*!<Rx FIFO 1 Watermark Reached              */
#define FDCAN_IR_RF1F_Pos         (6U)
#define FDCAN_IR_RF1F_Msk         (0x1U << FDCAN_IR_RF1F_Pos)                  /*!< 0x00000040 */
#define FDCAN_IR_RF1F             FDCAN_IR_RF1F_Msk                            /*!<Rx FIFO 1 Full                           */
#define FDCAN_IR_RF1L_Pos         (7U)
#define FDCAN_IR_RF1L_Msk         (0x1U << FDCAN_IR_RF1L_Pos)                  /*!< 0x00000080 */
#define FDCAN_IR_RF1L             FDCAN_IR_RF1L_Msk                            /*!<Rx FIFO 1 Message Lost                   */
#define FDCAN_IR_HPM_Pos          (8U)
#define FDCAN_IR_HPM_Msk          (0x1U << FDCAN_IR_HPM_Pos)                   /*!< 0x00000100 */
#define FDCAN_IR_HPM              FDCAN_IR_HPM_Msk                             /*!<High Priority Message                    */
#define FDCAN_IR_TC_Pos           (9U)
#define FDCAN_IR_TC_Msk           (0x1U << FDCAN_IR_TC_Pos)                    /*!< 0x00000200 */
#define FDCAN_IR_TC               FDCAN_IR_TC_Msk                              /*!<Transmission Completed                   */
#define FDCAN_IR_TCF_Pos          (10U)
#define FDCAN_IR_TCF_Msk          (0x1U << FDCAN_IR_TCF_Pos)                   /*!< 0x00000400 */
#define FDCAN_IR_TCF              FDCAN_IR_TCF_Msk                             /*!<Transmission Cancellation Finished       */
#define FDCAN_IR_TFE_Pos          (11U)
#define FDCAN_IR_TFE_Msk          (0x1U << FDCAN_IR_TFE_Pos)                   /*!< 0x00000800 */
#define FDCAN_IR_TFE              FDCAN_IR_TFE_Msk                             /*!<Tx FIFO Empty                            */
#define FDCAN_IR_TEFN_Pos         (12U)
#define FDCAN_IR_TEFN_Msk         (0x1U << FDCAN_IR_TEFN_Pos)                  /*!< 0x00001000 */
#define FDCAN_IR_TEFN             FDCAN_IR_TEFN_Msk                            /*!<Tx Event FIFO New Entry                  */
#define FDCAN_IR_TEFW_Pos         (13U)
#define FDCAN_IR_TEFW_Msk         (0x1U << FDCAN_IR_TEFW_Pos)                  /*!< 0x00002000 */
#define FDCAN_IR_TEFW             FDCAN_IR_TEFW_Msk                            /*!<Tx Event FIFO Watermark Reached          */
#define FDCAN_IR_TEFF_Pos         (14U)
#define FDCAN_IR_TEFF_Msk         (0x1U << FDCAN_IR_TEFF_Pos)                  /*!< 0x00004000 */
#define FDCAN_IR_TEFF             FDCAN_IR_TEFF_Msk                            /*!<Tx Event FIFO Full                       */
#define FDCAN_IR_TEFL_Pos         (15U)
#define FDCAN_IR_TEFL_Msk         (0x1U << FDCAN_IR_TEFL_Pos)                  /*!< 0x00008000 */
#define FDCAN_IR_TEFL             FDCAN_IR_TEFL_Msk                            /*!<Tx Event FIFO Element Lost               */
#define FDCAN_IR_TSW_Pos          (16U)
#define FDCAN_IR_TSW_Msk          (0x1U << FDCAN_IR_TSW_Pos)                   /*!< 0x00010000 */
#define FDCAN_IR_TSW              FDCAN_IR_TSW_Msk                             /*!<Timestamp Wraparound                     */
#define FDCAN_IR_MRAF_Pos         (17U)
#define FDCAN_IR_MRAF_Msk         (0x1U << FDCAN_IR_MRAF_Pos)                  /*!< 0x00020000 */
#define FDCAN_IR_MRAF             FDCAN_IR_MRAF_Msk                            /*!<Message RAM Access Failure               */
#define FDCAN_IR_TOO_Pos          (18U)
#define FDCAN_IR_TOO_Msk          (0x1U << FDCAN_IR_TOO_Pos)                   /*!< 0x00040000 */
#define FDCAN_IR_TOO              FDCAN_IR_TOO_Msk                             /*!<Timeout Occurred                         */
#define FDCAN_IR_DRX_Pos          (19U)
#define FDCAN_IR_DRX_Msk          (0x1U << FDCAN_IR_DRX_Pos)                   /*!< 0x00080000 */
#define FDCAN_IR_DRX              FDCAN_IR_DRX_Msk                             /*!<Message stored to Dedicated Rx Buffer    */
#define FDCAN_IR_ELO_Pos          (22U)
#define FDCAN_IR_ELO_Msk          (0x1U << FDCAN_IR_ELO_Pos)                   /*!< 0x00400000 */
#define FDCAN_IR_ELO              FDCAN_IR_ELO_Msk                             /*!<Error Logging Overflow                   */
#define FDCAN_IR_EP_Pos           (23U)
#define FDCAN_IR_EP_Msk           (0x1U << FDCAN_IR_EP_Pos)                    /*!< 0x00800000 */
#define FDCAN_IR_EP               FDCAN_IR_EP_Msk                              /*!<Error Passive                            */
#define FDCAN_IR_EW_Pos           (24U)
#define FDCAN_IR_EW_Msk           (0x1U << FDCAN_IR_EW_Pos)                    /*!< 0x01000000 */
#define FDCAN_IR_EW               FDCAN_IR_EW_Msk                              /*!<Warning Status                           */
#define FDCAN_IR_BO_Pos           (25U)
#define FDCAN_IR_BO_Msk           (0x1U << FDCAN_IR_BO_Pos)                    /*!< 0x02000000 */
#define FDCAN_IR_BO               FDCAN_IR_BO_Msk                              /*!<Bus_Off Status                           */
#define FDCAN_IR_WDI_Pos          (26U)
#define FDCAN_IR_WDI_Msk          (0x1U << FDCAN_IR_WDI_Pos)                   /*!< 0x04000000 */
#define FDCAN_IR_WDI              FDCAN_IR_WDI_Msk                             /*!<Watchdog Interrupt                       */
#define FDCAN_IR_PEA_Pos          (27U)
#define FDCAN_IR_PEA_Msk          (0x1U << FDCAN_IR_PEA_Pos)                   /*!< 0x08000000 */
#define FDCAN_IR_PEA              FDCAN_IR_PEA_Msk                             /*!<Protocol Error in Arbitration Phase      */
#define FDCAN_IR_PED_Pos          (28U)
#define FDCAN_IR_PED_Msk          (0x1U << FDCAN_IR_PED_Pos)                   /*!< 0x10000000 */
#define FDCAN_IR_PED              FDCAN_IR_PED_Msk                             /*!<Protocol Error in Data Phase             */
#define FDCAN_IR_ARA_Pos          (29U)
#define FDCAN_IR_ARA_Msk          (0x1U << FDCAN_IR_ARA_Pos)                   /*!< 0x20000000 */
#define FDCAN_IR_ARA              FDCAN_IR_ARA_Msk                             /*!<Access to Reserved Address               */

/*****************  Bit definition for FDCAN_IE register  **********************/
#define FDCAN_IE_RF0NE_Pos        (0U)
#define FDCAN_IE_RF0NE_Msk        (0x1U << FDCAN_IE_RF0NE_Pos)                 /*!< 0x00000001 */
#define FDCAN_IE_RF0NE            FDCAN_IE_RF0NE_Msk                           /*!<Rx FIFO 0 New Message Enable                 */
#define FDCAN_IE_RF0WE_Pos        (1U)
#define FDCAN_IE_RF0WE_Msk        (0x1U << FDCAN_IE_RF0WE_Pos)                 /*!< 0x00000002 */
#define FDCAN_IE_RF0WE            FDCAN_IE_RF0WE_Msk                           /*!<Rx FIFO 0 Watermark Reached Enable           */
#define FDCAN_IE_RF0FE_Pos        (2U)
#define FDCAN_IE_RF0FE_Msk        (0x1U << FDCAN_IE_RF0FE_Pos)                 /*!< 0x00000004 */
#define FDCAN_IE_RF0FE            FDCAN_IE_RF0FE_Msk                           /*!<Rx FIFO 0 Full Enable                        */
#define FDCAN_IE_RF0LE_Pos        (3U)
#define FDCAN_IE_RF0LE_Msk        (0x1U << FDCAN_IE_RF0LE_Pos)                 /*!< 0x00000008 */
#define FDCAN_IE_RF0LE            FDCAN_IE_RF0LE_Msk                           /*!<Rx FIFO 0 Message Lost Enable                */
#define FDCAN_IE_RF1NE_Pos        (4U)
#define FDCAN_IE_RF1NE_Msk        (0x1U << FDCAN_IE_RF1NE_Pos)                 /*!< 0x00000010 */
#define FDCAN_IE_RF1NE            FDCAN_IE_RF1NE_Msk                           /*!<Rx FIFO 1 New Message Enable                 */
#define FDCAN_IE_RF1WE_Pos        (5U)
#define FDCAN_IE_RF1WE_Msk        (0x1U << FDCAN_IE_RF1WE_Pos)                 /*!< 0x00000020 */
#define FDCAN_IE_RF1WE            FDCAN_IE_RF1WE_Msk                           /*!<Rx FIFO 1 Watermark Reached Enable           */
#define FDCAN_IE_RF1FE_Pos        (6U)
#define FDCAN_IE_RF1FE_Msk        (0x1U << FDCAN_IE_RF1FE_Pos)                 /*!< 0x00000040 */
#define FDCAN_IE_RF1FE            FDCAN_IE_RF1FE_Msk                           /*!<Rx FIFO 1 Full Enable                        */
#define FDCAN_IE_RF1LE_Pos        (7U)
#define FDCAN_IE_RF1LE_Msk        (0x1U << FDCAN_IE_RF1LE_Pos)                 /*!< 0x00000080 */
#define FDCAN_IE_RF1LE            FDCAN_IE_RF1LE_Msk                           /*!<Rx FIFO 1 Message Lost Enable                */
#define FDCAN_IE_HPME_Pos         (8U)
#define FDCAN_IE_HPME_Msk         (0x1U << FDCAN_IE_HPME_Pos)                  /*!< 0x00000100 */
#define FDCAN_IE_HPME             FDCAN_IE_HPME_Msk                            /*!<High Priority Message Enable                 */
#define FDCAN_IE_TCE_Pos          (9U)
#define FDCAN_IE_TCE_Msk          (0x1U << FDCAN_IE_TCE_Pos)                   /*!< 0x00000200 */
#define FDCAN_IE_TCE              FDCAN_IE_TCE_Msk                             /*!<Transmission Completed Enable                */
#define FDCAN_IE_TCFE_Pos         (10U)
#define FDCAN_IE_TCFE_Msk         (0x1U << FDCAN_IE_TCFE_Pos)                  /*!< 0x00000400 */
#define FDCAN_IE_TCFE             FDCAN_IE_TCFE_Msk                            /*!<Transmission Cancellation Finished Enable    */
#define FDCAN_IE_TFEE_Pos         (11U)
#define FDCAN_IE_TFEE_Msk         (0x1U << FDCAN_IE_TFEE_Pos)                  /*!< 0x00000800 */
#define FDCAN_IE_TFEE             FDCAN_IE_TFEE_Msk                            /*!<Tx FIFO Empty Enable                         */
#define FDCAN_IE_TEFNE_Pos        (12U)
#define FDCAN_IE_TEFNE_Msk        (0x1U << FDCAN_IE_TEFNE_Pos)                 /*!< 0x00001000 */
#define FDCAN_IE_TEFNE            FDCAN_IE_TEFNE_Msk                           /*!<Tx Event FIFO New Entry Enable               */
#define FDCAN_IE_TEFWE_Pos        (13U)
#define FDCAN_IE_TEFWE_Msk        (0x1U << FDCAN_IE_TEFWE_Pos)                 /*!< 0x00002000 */
#define FDCAN_IE_TEFWE            FDCAN_IE_TEFWE_Msk                           /*!<Tx Event FIFO Watermark Reached Enable       */
#define FDCAN_IE_TEFFE_Pos        (14U)
#define FDCAN_IE_TEFFE_Msk        (0x1U << FDCAN_IE_TEFFE_Pos)                 /*!< 0x00004000 */
#define FDCAN_IE_TEFFE            FDCAN_IE_TEFFE_Msk                           /*!<Tx Event FIFO Full Enable                    */
#define FDCAN_IE_TEFLE_Pos        (15U)
#define FDCAN_IE_TEFLE_Msk        (0x1U << FDCAN_IE_TEFLE_Pos)                 /*!< 0x00008000 */
#define FDCAN_IE_TEFLE            FDCAN_IE_TEFLE_Msk                           /*!<Tx Event FIFO Element Lost Enable            */
#define FDCAN_IE_TSWE_Pos         (16U)
#define FDCAN_IE_TSWE_Msk         (0x1U << FDCAN_IE_TSWE_Pos)                  /*!< 0x00010000 */
#define FDCAN_IE_TSWE             FDCAN_IE_TSWE_Msk                            /*!<Timestamp Wraparound Enable                  */
#define FDCAN_IE_MRAFE_Pos        (17U)
#define FDCAN_IE_MRAFE_Msk        (0x1U << FDCAN_IE_MRAFE_Pos)                 /*!< 0x00020000 */
#define FDCAN_IE_MRAFE            FDCAN_IE_MRAFE_Msk                           /*!<Message RAM Access Failure Enable            */
#define FDCAN_IE_TOOE_Pos         (18U)
#define FDCAN_IE_TOOE_Msk         (0x1U << FDCAN_IE_TOOE_Pos)                  /*!< 0x00040000 */
#define FDCAN_IE_TOOE             FDCAN_IE_TOOE_Msk                            /*!<Timeout Occurred Enable                      */
#define FDCAN_IE_DRXE_Pos         (19U)
#define FDCAN_IE_DRXE_Msk         (0x1U << FDCAN_IE_DRXE_Pos)                  /*!< 0x00080000 */
#define FDCAN_IE_DRXE             FDCAN_IE_DRXE_Msk                            /*!<Message stored to Dedicated Rx Buffer Enable */
#define FDCAN_IE_BECE_Pos         (20U)
#define FDCAN_IE_BECE_Msk         (0x1U << FDCAN_IE_BECE_Pos)                  /*!< 0x00100000 */
#define FDCAN_IE_BECE             FDCAN_IE_BECE_Msk                            /*!<Bit Error Corrected Interrupt Enable         */
#define FDCAN_IE_BEUE_Pos         (21U)
#define FDCAN_IE_BEUE_Msk         (0x1U << FDCAN_IE_BEUE_Pos)                  /*!< 0x00200000 */
#define FDCAN_IE_BEUE             FDCAN_IE_BEUE_Msk                            /*!<Bit Error Uncorrected Interrupt Enable       */
#define FDCAN_IE_ELOE_Pos         (22U)
#define FDCAN_IE_ELOE_Msk         (0x1U << FDCAN_IE_ELOE_Pos)                  /*!< 0x00400000 */
#define FDCAN_IE_ELOE             FDCAN_IE_ELOE_Msk                            /*!<Error Logging Overflow Enable                */
#define FDCAN_IE_EPE_Pos          (23U)
#define FDCAN_IE_EPE_Msk          (0x1U << FDCAN_IE_EPE_Pos)                   /*!< 0x00800000 */
#define FDCAN_IE_EPE              FDCAN_IE_EPE_Msk                             /*!<Error Passive Enable                         */
#define FDCAN_IE_EWE_Pos          (24U)
#define FDCAN_IE_EWE_Msk          (0x1U << FDCAN_IE_EWE_Pos)                   /*!< 0x01000000 */
#define FDCAN_IE_EWE              FDCAN_IE_EWE_Msk                             /*!<Warning Status Enable                        */
#define FDCAN_IE_BOE_Pos          (25U)
#define FDCAN_IE_BOE_Msk          (0x1U << FDCAN_IE_BOE_Pos)                   /*!< 0x02000000 */
#define FDCAN_IE_BOE              FDCAN_IE_BOE_Msk                             /*!<Bus_Off Status Enable                        */
#define FDCAN_IE_WDIE_Pos         (26U)
#define FDCAN_IE_WDIE_Msk         (0x1U << FDCAN_IE_WDIE_Pos)                  /*!< 0x04000000 */
#define FDCAN_IE_WDIE             FDCAN_IE_WDIE_Msk                            /*!<Watchdog Interrupt Enable                    */
#define FDCAN_IE_PEAE_Pos         (27U)
#define FDCAN_IE_PEAE_Msk         (0x1U << FDCAN_IE_PEAE_Pos)                  /*!< 0x08000000 */
#define FDCAN_IE_PEAE             FDCAN_IE_PEAE_Msk                            /*!<Protocol Error in Arbitration Phase Enable   */
#define FDCAN_IE_PEDE_Pos         (28U)
#define FDCAN_IE_PEDE_Msk         (0x1U << FDCAN_IE_PEDE_Pos)                  /*!< 0x10000000 */
#define FDCAN_IE_PEDE             FDCAN_IE_PEDE_Msk                            /*!<Protocol Error in Data Phase Enable          */
#define FDCAN_IE_ARAE_Pos         (29U)
#define FDCAN_IE_ARAE_Msk         (0x1U << FDCAN_IE_ARAE_Pos)                  /*!< 0x20000000 */
#define FDCAN_IE_ARAE             FDCAN_IE_ARAE_Msk                            /*!<Access to Reserved Address Enable            */

/*****************  Bit definition for FDCAN_ILS register  **********************/
#define FDCAN_ILS_RF0NL_Pos       (0U)
#define FDCAN_ILS_RF0NL_Msk       (0x1U << FDCAN_ILS_RF0NL_Pos)                /*!< 0x00000001 */
#define FDCAN_ILS_RF0NL           FDCAN_ILS_RF0NL_Msk                          /*!<Rx FIFO 0 New Message Line                  */
#define FDCAN_ILS_RF0WL_Pos       (1U)
#define FDCAN_ILS_RF0WL_Msk       (0x1U << FDCAN_ILS_RF0WL_Pos)                /*!< 0x00000002 */
#define FDCAN_ILS_RF0WL           FDCAN_ILS_RF0WL_Msk                          /*!<Rx FIFO 0 Watermark Reached Line            */
#define FDCAN_ILS_RF0FL_Pos       (2U)
#define FDCAN_ILS_RF0FL_Msk       (0x1U << FDCAN_ILS_RF0FL_Pos)                /*!< 0x00000004 */
#define FDCAN_ILS_RF0FL           FDCAN_ILS_RF0FL_Msk                          /*!<Rx FIFO 0 Full Line                         */
#define FDCAN_ILS_RF0LL_Pos       (3U)
#define FDCAN_ILS_RF0LL_Msk       (0x1U << FDCAN_ILS_RF0LL_Pos)                /*!< 0x00000008 */
#define FDCAN_ILS_RF0LL           FDCAN_ILS_RF0LL_Msk                          /*!<Rx FIFO 0 Message Lost Line                 */
#define FDCAN_ILS_RF1NL_Pos       (4U)
#define FDCAN_ILS_RF1NL_Msk       (0x1U << FDCAN_ILS_RF1NL_Pos)                /*!< 0x00000010 */
#define FDCAN_ILS_RF1NL           FDCAN_ILS_RF1NL_Msk                          /*!<Rx FIFO 1 New Message Line                  */
#define FDCAN_ILS_RF1WL_Pos       (5U)
#define FDCAN_ILS_RF1WL_Msk       (0x1U << FDCAN_ILS_RF1WL_Pos)                /*!< 0x00000020 */
#define FDCAN_ILS_RF1WL           FDCAN_ILS_RF1WL_Msk                          /*!<Rx FIFO 1 Watermark Reached Line            */
#define FDCAN_ILS_RF1FL_Pos       (6U)
#define FDCAN_ILS_RF1FL_Msk       (0x1U << FDCAN_ILS_RF1FL_Pos)                /*!< 0x00000040 */
#define FDCAN_ILS_RF1FL           FDCAN_ILS_RF1FL_Msk                          /*!<Rx FIFO 1 Full Line                         */
#define FDCAN_ILS_RF1LL_Pos       (7U)
#define FDCAN_ILS_RF1LL_Msk       (0x1U << FDCAN_ILS_RF1LL_Pos)                /*!< 0x00000080 */
#define FDCAN_ILS_RF1LL           FDCAN_ILS_RF1LL_Msk                          /*!<Rx FIFO 1 Message Lost Line                 */
#define FDCAN_ILS_HPML_Pos        (8U)
#define FDCAN_ILS_HPML_Msk        (0x1U << FDCAN_ILS_HPML_Pos)                 /*!< 0x00000100 */
#define FDCAN_ILS_HPML            FDCAN_ILS_HPML_Msk                           /*!<High Priority Message Line                  */
#define FDCAN_ILS_TCL_Pos         (9U)
#define FDCAN_ILS_TCL_Msk         (0x1U << FDCAN_ILS_TCL_Pos)                  /*!< 0x00000200 */
#define FDCAN_ILS_TCL             FDCAN_ILS_TCL_Msk                            /*!<Transmission Completed Line                 */
#define FDCAN_ILS_TCFL_Pos        (10U)
#define FDCAN_ILS_TCFL_Msk        (0x1U << FDCAN_ILS_TCFL_Pos)                 /*!< 0x00000400 */
#define FDCAN_ILS_TCFL            FDCAN_ILS_TCFL_Msk                           /*!<Transmission Cancellation Finished Line     */
#define FDCAN_ILS_TFEL_Pos        (11U)
#define FDCAN_ILS_TFEL_Msk        (0x1U << FDCAN_ILS_TFEL_Pos)                 /*!< 0x00000800 */
#define FDCAN_ILS_TFEL            FDCAN_ILS_TFEL_Msk                           /*!<Tx FIFO Empty Line                          */
#define FDCAN_ILS_TEFNL_Pos       (12U)
#define FDCAN_ILS_TEFNL_Msk       (0x1U << FDCAN_ILS_TEFNL_Pos)                /*!< 0x00001000 */
#define FDCAN_ILS_TEFNL           FDCAN_ILS_TEFNL_Msk                          /*!<Tx Event FIFO New Entry Line                */
#define FDCAN_ILS_TEFWL_Pos       (13U)
#define FDCAN_ILS_TEFWL_Msk       (0x1U << FDCAN_ILS_TEFWL_Pos)                /*!< 0x00002000 */
#define FDCAN_ILS_TEFWL           FDCAN_ILS_TEFWL_Msk                          /*!<Tx Event FIFO Watermark Reached Line        */
#define FDCAN_ILS_TEFFL_Pos       (14U)
#define FDCAN_ILS_TEFFL_Msk       (0x1U << FDCAN_ILS_TEFFL_Pos)                /*!< 0x00004000 */
#define FDCAN_ILS_TEFFL           FDCAN_ILS_TEFFL_Msk                          /*!<Tx Event FIFO Full Line                     */
#define FDCAN_ILS_TEFLL_Pos       (15U)
#define FDCAN_ILS_TEFLL_Msk       (0x1U << FDCAN_ILS_TEFLL_Pos)                /*!< 0x00008000 */
#define FDCAN_ILS_TEFLL           FDCAN_ILS_TEFLL_Msk                          /*!<Tx Event FIFO Element Lost Line             */
#define FDCAN_ILS_TSWL_Pos        (16U)
#define FDCAN_ILS_TSWL_Msk        (0x1U << FDCAN_ILS_TSWL_Pos)                 /*!< 0x00010000 */
#define FDCAN_ILS_TSWL            FDCAN_ILS_TSWL_Msk                           /*!<Timestamp Wraparound Line                   */
#define FDCAN_ILS_MRAFE_Pos       (17U)
#define FDCAN_ILS_MRAFE_Msk       (0x1U << FDCAN_ILS_MRAFE_Pos)                /*!< 0x00020000 */
#define FDCAN_ILS_MRAFE           FDCAN_ILS_MRAFE_Msk                          /*!<Message RAM Access Failure Line             */
#define FDCAN_ILS_TOOE_Pos        (18U)
#define FDCAN_ILS_TOOE_Msk        (0x1U << FDCAN_ILS_TOOE_Pos)                 /*!< 0x00040000 */
#define FDCAN_ILS_TOOE            FDCAN_ILS_TOOE_Msk                           /*!<Timeout Occurred Line                       */
#define FDCAN_ILS_DRXE_Pos        (19U)
#define FDCAN_ILS_DRXE_Msk        (0x1U << FDCAN_ILS_DRXE_Pos)                 /*!< 0x00080000 */
#define FDCAN_ILS_DRXE            FDCAN_ILS_DRXE_Msk                           /*!<Message stored to Dedicated Rx Buffer Line  */
#define FDCAN_ILS_BECE_Pos        (20U)
#define FDCAN_ILS_BECE_Msk        (0x1U << FDCAN_ILS_BECE_Pos)                 /*!< 0x00100000 */
#define FDCAN_ILS_BECE            FDCAN_ILS_BECE_Msk                           /*!<Bit Error Corrected Interrupt Line          */
#define FDCAN_ILS_BEUE_Pos        (21U)
#define FDCAN_ILS_BEUE_Msk        (0x1U << FDCAN_ILS_BEUE_Pos)                 /*!< 0x00200000 */
#define FDCAN_ILS_BEUE            FDCAN_ILS_BEUE_Msk                           /*!<Bit Error Uncorrected Interrupt Line        */
#define FDCAN_ILS_ELOE_Pos        (22U)
#define FDCAN_ILS_ELOE_Msk        (0x1U << FDCAN_ILS_ELOE_Pos)                 /*!< 0x00400000 */
#define FDCAN_ILS_ELOE            FDCAN_ILS_ELOE_Msk                           /*!<Error Logging Overflow Line                 */
#define FDCAN_ILS_EPE_Pos         (23U)
#define FDCAN_ILS_EPE_Msk         (0x1U << FDCAN_ILS_EPE_Pos)                  /*!< 0x00800000 */
#define FDCAN_ILS_EPE             FDCAN_ILS_EPE_Msk                            /*!<Error Passive Line                          */
#define FDCAN_ILS_EWE_Pos         (24U)
#define FDCAN_ILS_EWE_Msk         (0x1U << FDCAN_ILS_EWE_Pos)                  /*!< 0x01000000 */
#define FDCAN_ILS_EWE             FDCAN_ILS_EWE_Msk                            /*!<Warning Status Line                         */
#define FDCAN_ILS_BOE_Pos         (25U)
#define FDCAN_ILS_BOE_Msk         (0x1U << FDCAN_ILS_BOE_Pos)                  /*!< 0x02000000 */
#define FDCAN_ILS_BOE             FDCAN_ILS_BOE_Msk                            /*!<Bus_Off Status Line                         */
#define FDCAN_ILS_WDIE_Pos        (26U)
#define FDCAN_ILS_WDIE_Msk        (0x1U << FDCAN_ILS_WDIE_Pos)                 /*!< 0x04000000 */
#define FDCAN_ILS_WDIE            FDCAN_ILS_WDIE_Msk                           /*!<Watchdog Interrupt Line                     */
#define FDCAN_ILS_PEAE_Pos        (27U)
#define FDCAN_ILS_PEAE_Msk        (0x1U << FDCAN_ILS_PEAE_Pos)                 /*!< 0x08000000 */
#define FDCAN_ILS_PEAE            FDCAN_ILS_PEAE_Msk                           /*!<Protocol Error in Arbitration Phase Line    */
#define FDCAN_ILS_PEDE_Pos        (28U)
#define FDCAN_ILS_PEDE_Msk        (0x1U << FDCAN_ILS_PEDE_Pos)                 /*!< 0x10000000 */
#define FDCAN_ILS_PEDE            FDCAN_ILS_PEDE_Msk                           /*!<Protocol Error in Data Phase Line           */
#define FDCAN_ILS_ARAE_Pos        (29U)
#define FDCAN_ILS_ARAE_Msk        (0x1U << FDCAN_ILS_ARAE_Pos)                 /*!< 0x20000000 */
#define FDCAN_ILS_ARAE            FDCAN_ILS_ARAE_Msk                           /*!<Access to Reserved Address Line             */

/*****************  Bit definition for FDCAN_ILE register  **********************/
#define FDCAN_ILE_EINT0_Pos       (0U)
#define FDCAN_ILE_EINT0_Msk       (0x1U << FDCAN_ILE_EINT0_Pos)                /*!< 0x00000001 */
#define FDCAN_ILE_EINT0           FDCAN_ILE_EINT0_Msk                          /*!<Enable Interrupt Line 0                   */
#define FDCAN_ILE_EINT1_Pos       (1U)
#define FDCAN_ILE_EINT1_Msk       (0x1U << FDCAN_ILE_EINT1_Pos)                /*!< 0x00000002 */
#define FDCAN_ILE_EINT1           FDCAN_ILE_EINT1_Msk                          /*!<Enable Interrupt Line 1                   */

/*****************  Bit definition for FDCAN_GFC register  **********************/
#define FDCAN_GFC_RRFE_Pos        (0U)
#define FDCAN_GFC_RRFE_Msk        (0x1U << FDCAN_GFC_RRFE_Pos)                 /*!< 0x00000001 */
#define FDCAN_GFC_RRFE            FDCAN_GFC_RRFE_Msk                           /*!<Reject Remote Frames Extended             */
#define FDCAN_GFC_RRFS_Pos        (1U)
#define FDCAN_GFC_RRFS_Msk        (0x1U << FDCAN_GFC_RRFS_Pos)                 /*!< 0x00000002 */
#define FDCAN_GFC_RRFS            FDCAN_GFC_RRFS_Msk                           /*!<Reject Remote Frames Standard             */
#define FDCAN_GFC_ANFE_Pos        (2U)
#define FDCAN_GFC_ANFE_Msk        (0x3U << FDCAN_GFC_ANFE_Pos)                 /*!< 0x0000000C */
#define FDCAN_GFC_ANFE            FDCAN_GFC_ANFE_Msk                           /*!<Accept Non-matching Frames Extended       */
#define FDCAN_GFC_ANFS_Pos        (4U)
#define FDCAN_GFC_ANFS_Msk        (0x3U << FDCAN_GFC_ANFS_Pos)                 /*!< 0x00000030 */
#define FDCAN_GFC_ANFS            FDCAN_GFC_ANFS_Msk                           /*!<Accept Non-matching Frames Standard       */

/*****************  Bit definition for FDCAN_SIDFC register  ********************/
#define FDCAN_SIDFC_FLSSA_Pos     (2U)
#define FDCAN_SIDFC_FLSSA_Msk     (0x3FFFU << FDCAN_SIDFC_FLSSA_Pos)           /*!< 0x0000FFFC */
#define FDCAN_SIDFC_FLSSA         FDCAN_SIDFC_FLSSA_Msk                        /*!<Filter List Standard Start Address        */
#define FDCAN_SIDFC_LSS_Pos       (16U)
#define FDCAN_SIDFC_LSS_Msk       (0xFFU << FDCAN_SIDFC_LSS_Pos)               /*!< 0x00FF0000 */
#define FDCAN_SIDFC_LSS           FDCAN_SIDFC_LSS_Msk                          /*!<List Size Standard                        */

/*****************  Bit definition for FDCAN_XIDFC register  ********************/
#define FDCAN_XIDFC_FLESA_Pos     (2U)
#define FDCAN_XIDFC_FLESA_Msk     (0x3FFFU << FDCAN_XIDFC_FLESA_Pos)           /*!< 0x0000FFFC */
#define FDCAN_XIDFC_FLESA         FDCAN_XIDFC_FLESA_Msk                        /*!<Filter List Standard Start Address        */
#define FDCAN_XIDFC_LSE_Pos       (16U)
#define FDCAN_XIDFC_LSE_Msk       (0xFFU << FDCAN_XIDFC_LSE_Pos)               /*!< 0x00FF0000 */
#define FDCAN_XIDFC_LSE           FDCAN_XIDFC_LSE_Msk                          /*!<List Size Extended                        */

/*****************  Bit definition for FDCAN_XIDAM register  ********************/
#define FDCAN_XIDAM_EIDM_Pos      (0U)
#define FDCAN_XIDAM_EIDM_Msk      (0x1FFFFFFFU << FDCAN_XIDAM_EIDM_Pos)        /*!< 0x1FFFFFFF */
#define FDCAN_XIDAM_EIDM          FDCAN_XIDAM_EIDM_Msk                         /*!<Extended ID Mask                          */

/*****************  Bit definition for FDCAN_HPMS register  *********************/
#define FDCAN_HPMS_BIDX_Pos       (0U)
#define FDCAN_HPMS_BIDX_Msk       (0x3FU << FDCAN_HPMS_BIDX_Pos)               /*!< 0x0000003F */
#define FDCAN_HPMS_BIDX           FDCAN_HPMS_BIDX_Msk                          /*!<Buffer Index                              */
#define FDCAN_HPMS_MSI_Pos        (6U)
#define FDCAN_HPMS_MSI_Msk        (0x3U << FDCAN_HPMS_MSI_Pos)                 /*!< 0x000000C0 */
#define FDCAN_HPMS_MSI            FDCAN_HPMS_MSI_Msk                           /*!<Message Storage Indicator                 */
#define FDCAN_HPMS_FIDX_Pos       (8U)
#define FDCAN_HPMS_FIDX_Msk       (0x7FU << FDCAN_HPMS_FIDX_Pos)               /*!< 0x00007F00 */
#define FDCAN_HPMS_FIDX           FDCAN_HPMS_FIDX_Msk                          /*!<Filter Index                              */
#define FDCAN_HPMS_FLST_Pos       (15U)
#define FDCAN_HPMS_FLST_Msk       (0x1U << FDCAN_HPMS_FLST_Pos)                /*!< 0x00008000 */
#define FDCAN_HPMS_FLST           FDCAN_HPMS_FLST_Msk                          /*!<Filter List                               */

/*****************  Bit definition for FDCAN_NDAT1 register  ********************/
#define FDCAN_NDAT1_ND0_Pos       (0U)
#define FDCAN_NDAT1_ND0_Msk       (0x1U << FDCAN_NDAT1_ND0_Pos)                /*!< 0x00000001 */
#define FDCAN_NDAT1_ND0           FDCAN_NDAT1_ND0_Msk                          /*!<New Data flag of Rx Buffer 0              */
#define FDCAN_NDAT1_ND1_Pos       (1U)
#define FDCAN_NDAT1_ND1_Msk       (0x1U << FDCAN_NDAT1_ND1_Pos)                /*!< 0x00000002 */
#define FDCAN_NDAT1_ND1           FDCAN_NDAT1_ND1_Msk                          /*!<New Data flag of Rx Buffer 1              */
#define FDCAN_NDAT1_ND2_Pos       (2U)
#define FDCAN_NDAT1_ND2_Msk       (0x1U << FDCAN_NDAT1_ND2_Pos)                /*!< 0x00000004 */
#define FDCAN_NDAT1_ND2           FDCAN_NDAT1_ND2_Msk                          /*!<New Data flag of Rx Buffer 2              */
#define FDCAN_NDAT1_ND3_Pos       (3U)
#define FDCAN_NDAT1_ND3_Msk       (0x1U << FDCAN_NDAT1_ND3_Pos)                /*!< 0x00000008 */
#define FDCAN_NDAT1_ND3           FDCAN_NDAT1_ND3_Msk                          /*!<New Data flag of Rx Buffer 3              */
#define FDCAN_NDAT1_ND4_Pos       (4U)
#define FDCAN_NDAT1_ND4_Msk       (0x1U << FDCAN_NDAT1_ND4_Pos)                /*!< 0x00000010 */
#define FDCAN_NDAT1_ND4           FDCAN_NDAT1_ND4_Msk                          /*!<New Data flag of Rx Buffer 4              */
#define FDCAN_NDAT1_ND5_Pos       (5U)
#define FDCAN_NDAT1_ND5_Msk       (0x1U << FDCAN_NDAT1_ND5_Pos)                /*!< 0x00000020 */
#define FDCAN_NDAT1_ND5           FDCAN_NDAT1_ND5_Msk                          /*!<New Data flag of Rx Buffer 5              */
#define FDCAN_NDAT1_ND6_Pos       (6U)
#define FDCAN_NDAT1_ND6_Msk       (0x1U << FDCAN_NDAT1_ND6_Pos)                /*!< 0x00000040 */
#define FDCAN_NDAT1_ND6           FDCAN_NDAT1_ND6_Msk                          /*!<New Data flag of Rx Buffer 6              */
#define FDCAN_NDAT1_ND7_Pos       (7U)
#define FDCAN_NDAT1_ND7_Msk       (0x1U << FDCAN_NDAT1_ND7_Pos)                /*!< 0x00000080 */
#define FDCAN_NDAT1_ND7           FDCAN_NDAT1_ND7_Msk                          /*!<New Data flag of Rx Buffer 7              */
#define FDCAN_NDAT1_ND8_Pos       (8U)
#define FDCAN_NDAT1_ND8_Msk       (0x1U << FDCAN_NDAT1_ND8_Pos)                /*!< 0x00000100 */
#define FDCAN_NDAT1_ND8           FDCAN_NDAT1_ND8_Msk                          /*!<New Data flag of Rx Buffer 8              */
#define FDCAN_NDAT1_ND9_Pos       (9U)
#define FDCAN_NDAT1_ND9_Msk       (0x1U << FDCAN_NDAT1_ND9_Pos)                /*!< 0x00000200 */
#define FDCAN_NDAT1_ND9           FDCAN_NDAT1_ND9_Msk                          /*!<New Data flag of Rx Buffer 9              */
#define FDCAN_NDAT1_ND10_Pos      (10U)
#define FDCAN_NDAT1_ND10_Msk      (0x1U << FDCAN_NDAT1_ND10_Pos)               /*!< 0x00000400 */
#define FDCAN_NDAT1_ND10          FDCAN_NDAT1_ND10_Msk                         /*!<New Data flag of Rx Buffer 10             */
#define FDCAN_NDAT1_ND11_Pos      (11U)
#define FDCAN_NDAT1_ND11_Msk      (0x1U << FDCAN_NDAT1_ND11_Pos)               /*!< 0x00000800 */
#define FDCAN_NDAT1_ND11          FDCAN_NDAT1_ND11_Msk                         /*!<New Data flag of Rx Buffer 11             */
#define FDCAN_NDAT1_ND12_Pos      (12U)
#define FDCAN_NDAT1_ND12_Msk      (0x1U << FDCAN_NDAT1_ND12_Pos)               /*!< 0x00001000 */
#define FDCAN_NDAT1_ND12          FDCAN_NDAT1_ND12_Msk                         /*!<New Data flag of Rx Buffer 12             */
#define FDCAN_NDAT1_ND13_Pos      (13U)
#define FDCAN_NDAT1_ND13_Msk      (0x1U << FDCAN_NDAT1_ND13_Pos)               /*!< 0x00002000 */
#define FDCAN_NDAT1_ND13          FDCAN_NDAT1_ND13_Msk                         /*!<New Data flag of Rx Buffer 13             */
#define FDCAN_NDAT1_ND14_Pos      (14U)
#define FDCAN_NDAT1_ND14_Msk      (0x1U << FDCAN_NDAT1_ND14_Pos)               /*!< 0x00004000 */
#define FDCAN_NDAT1_ND14          FDCAN_NDAT1_ND14_Msk                         /*!<New Data flag of Rx Buffer 14             */
#define FDCAN_NDAT1_ND15_Pos      (15U)
#define FDCAN_NDAT1_ND15_Msk      (0x1U << FDCAN_NDAT1_ND15_Pos)               /*!< 0x00008000 */
#define FDCAN_NDAT1_ND15          FDCAN_NDAT1_ND15_Msk                         /*!<New Data flag of Rx Buffer 15             */
#define FDCAN_NDAT1_ND16_Pos      (16U)
#define FDCAN_NDAT1_ND16_Msk      (0x1U << FDCAN_NDAT1_ND16_Pos)               /*!< 0x00010000 */
#define FDCAN_NDAT1_ND16          FDCAN_NDAT1_ND16_Msk                         /*!<New Data flag of Rx Buffer 16             */
#define FDCAN_NDAT1_ND17_Pos      (17U)
#define FDCAN_NDAT1_ND17_Msk      (0x1U << FDCAN_NDAT1_ND17_Pos)               /*!< 0x00020000 */
#define FDCAN_NDAT1_ND17          FDCAN_NDAT1_ND17_Msk                         /*!<New Data flag of Rx Buffer 17             */
#define FDCAN_NDAT1_ND18_Pos      (18U)
#define FDCAN_NDAT1_ND18_Msk      (0x1U << FDCAN_NDAT1_ND18_Pos)               /*!< 0x00040000 */
#define FDCAN_NDAT1_ND18          FDCAN_NDAT1_ND18_Msk                         /*!<New Data flag of Rx Buffer 18             */
#define FDCAN_NDAT1_ND19_Pos      (19U)
#define FDCAN_NDAT1_ND19_Msk      (0x1U << FDCAN_NDAT1_ND19_Pos)               /*!< 0x00080000 */
#define FDCAN_NDAT1_ND19          FDCAN_NDAT1_ND19_Msk                         /*!<New Data flag of Rx Buffer 19             */
#define FDCAN_NDAT1_ND20_Pos      (20U)
#define FDCAN_NDAT1_ND20_Msk      (0x1U << FDCAN_NDAT1_ND20_Pos)               /*!< 0x00100000 */
#define FDCAN_NDAT1_ND20          FDCAN_NDAT1_ND20_Msk                         /*!<New Data flag of Rx Buffer 20             */
#define FDCAN_NDAT1_ND21_Pos      (21U)
#define FDCAN_NDAT1_ND21_Msk      (0x1U << FDCAN_NDAT1_ND21_Pos)               /*!< 0x00200000 */
#define FDCAN_NDAT1_ND21          FDCAN_NDAT1_ND21_Msk                         /*!<New Data flag of Rx Buffer 21             */
#define FDCAN_NDAT1_ND22_Pos      (22U)
#define FDCAN_NDAT1_ND22_Msk      (0x1U << FDCAN_NDAT1_ND22_Pos)               /*!< 0x00400000 */
#define FDCAN_NDAT1_ND22          FDCAN_NDAT1_ND22_Msk                         /*!<New Data flag of Rx Buffer 22             */
#define FDCAN_NDAT1_ND23_Pos      (23U)
#define FDCAN_NDAT1_ND23_Msk      (0x1U << FDCAN_NDAT1_ND23_Pos)               /*!< 0x00800000 */
#define FDCAN_NDAT1_ND23          FDCAN_NDAT1_ND23_Msk                         /*!<New Data flag of Rx Buffer 23             */
#define FDCAN_NDAT1_ND24_Pos      (24U)
#define FDCAN_NDAT1_ND24_Msk      (0x1U << FDCAN_NDAT1_ND24_Pos)               /*!< 0x01000000 */
#define FDCAN_NDAT1_ND24          FDCAN_NDAT1_ND24_Msk                         /*!<New Data flag of Rx Buffer 24             */
#define FDCAN_NDAT1_ND25_Pos      (25U)
#define FDCAN_NDAT1_ND25_Msk      (0x1U << FDCAN_NDAT1_ND25_Pos)               /*!< 0x02000000 */
#define FDCAN_NDAT1_ND25          FDCAN_NDAT1_ND25_Msk                         /*!<New Data flag of Rx Buffer 25             */
#define FDCAN_NDAT1_ND26_Pos      (26U)
#define FDCAN_NDAT1_ND26_Msk      (0x1U << FDCAN_NDAT1_ND26_Pos)               /*!< 0x04000000 */
#define FDCAN_NDAT1_ND26          FDCAN_NDAT1_ND26_Msk                         /*!<New Data flag of Rx Buffer 26             */
#define FDCAN_NDAT1_ND27_Pos      (27U)
#define FDCAN_NDAT1_ND27_Msk      (0x1U << FDCAN_NDAT1_ND27_Pos)               /*!< 0x08000000 */
#define FDCAN_NDAT1_ND27          FDCAN_NDAT1_ND27_Msk                         /*!<New Data flag of Rx Buffer 27             */
#define FDCAN_NDAT1_ND28_Pos      (28U)
#define FDCAN_NDAT1_ND28_Msk      (0x1U << FDCAN_NDAT1_ND28_Pos)               /*!< 0x10000000 */
#define FDCAN_NDAT1_ND28          FDCAN_NDAT1_ND28_Msk                         /*!<New Data flag of Rx Buffer 28             */
#define FDCAN_NDAT1_ND29_Pos      (29U)
#define FDCAN_NDAT1_ND29_Msk      (0x1U << FDCAN_NDAT1_ND29_Pos)               /*!< 0x20000000 */
#define FDCAN_NDAT1_ND29          FDCAN_NDAT1_ND29_Msk                         /*!<New Data flag of Rx Buffer 29             */
#define FDCAN_NDAT1_ND30_Pos      (30U)
#define FDCAN_NDAT1_ND30_Msk      (0x1U << FDCAN_NDAT1_ND30_Pos)               /*!< 0x40000000 */
#define FDCAN_NDAT1_ND30          FDCAN_NDAT1_ND30_Msk                         /*!<New Data flag of Rx Buffer 30             */
#define FDCAN_NDAT1_ND31_Pos      (31U)
#define FDCAN_NDAT1_ND31_Msk      (0x1U << FDCAN_NDAT1_ND31_Pos)               /*!< 0x80000000 */
#define FDCAN_NDAT1_ND31          FDCAN_NDAT1_ND31_Msk                         /*!<New Data flag of Rx Buffer 31             */

/*****************  Bit definition for FDCAN_NDAT2 register  ********************/
#define FDCAN_NDAT2_ND32_Pos      (0U)
#define FDCAN_NDAT2_ND32_Msk      (0x1U << FDCAN_NDAT2_ND32_Pos)               /*!< 0x00000001 */
#define FDCAN_NDAT2_ND32          FDCAN_NDAT2_ND32_Msk                         /*!<New Data flag of Rx Buffer 32             */
#define FDCAN_NDAT2_ND33_Pos      (1U)
#define FDCAN_NDAT2_ND33_Msk      (0x1U << FDCAN_NDAT2_ND33_Pos)               /*!< 0x00000002 */
#define FDCAN_NDAT2_ND33          FDCAN_NDAT2_ND33_Msk                         /*!<New Data flag of Rx Buffer 33             */
#define FDCAN_NDAT2_ND34_Pos      (2U)
#define FDCAN_NDAT2_ND34_Msk      (0x1U << FDCAN_NDAT2_ND34_Pos)               /*!< 0x00000004 */
#define FDCAN_NDAT2_ND34          FDCAN_NDAT2_ND34_Msk                         /*!<New Data flag of Rx Buffer 34             */
#define FDCAN_NDAT2_ND35_Pos      (3U)
#define FDCAN_NDAT2_ND35_Msk      (0x1U << FDCAN_NDAT2_ND35_Pos)               /*!< 0x00000008 */
#define FDCAN_NDAT2_ND35          FDCAN_NDAT2_ND35_Msk                         /*!<New Data flag of Rx Buffer 35             */
#define FDCAN_NDAT2_ND36_Pos      (4U)
#define FDCAN_NDAT2_ND36_Msk      (0x1U << FDCAN_NDAT2_ND36_Pos)               /*!< 0x00000010 */
#define FDCAN_NDAT2_ND36          FDCAN_NDAT2_ND36_Msk                         /*!<New Data flag of Rx Buffer 36             */
#define FDCAN_NDAT2_ND37_Pos      (5U)
#define FDCAN_NDAT2_ND37_Msk      (0x1U << FDCAN_NDAT2_ND37_Pos)               /*!< 0x00000020 */
#define FDCAN_NDAT2_ND37          FDCAN_NDAT2_ND37_Msk                         /*!<New Data flag of Rx Buffer 37             */
#define FDCAN_NDAT2_ND38_Pos      (6U)
#define FDCAN_NDAT2_ND38_Msk      (0x1U << FDCAN_NDAT2_ND38_Pos)               /*!< 0x00000040 */
#define FDCAN_NDAT2_ND38          FDCAN_NDAT2_ND38_Msk                         /*!<New Data flag of Rx Buffer 38             */
#define FDCAN_NDAT2_ND39_Pos      (7U)
#define FDCAN_NDAT2_ND39_Msk      (0x1U << FDCAN_NDAT2_ND39_Pos)               /*!< 0x00000080 */
#define FDCAN_NDAT2_ND39          FDCAN_NDAT2_ND39_Msk                         /*!<New Data flag of Rx Buffer 39             */
#define FDCAN_NDAT2_ND40_Pos      (8U)
#define FDCAN_NDAT2_ND40_Msk      (0x1U << FDCAN_NDAT2_ND40_Pos)               /*!< 0x00000100 */
#define FDCAN_NDAT2_ND40          FDCAN_NDAT2_ND40_Msk                         /*!<New Data flag of Rx Buffer 40             */
#define FDCAN_NDAT2_ND41_Pos      (9U)
#define FDCAN_NDAT2_ND41_Msk      (0x1U << FDCAN_NDAT2_ND41_Pos)               /*!< 0x00000200 */
#define FDCAN_NDAT2_ND41          FDCAN_NDAT2_ND41_Msk                         /*!<New Data flag of Rx Buffer 41             */
#define FDCAN_NDAT2_ND42_Pos      (10U)
#define FDCAN_NDAT2_ND42_Msk      (0x1U << FDCAN_NDAT2_ND42_Pos)               /*!< 0x00000400 */
#define FDCAN_NDAT2_ND42          FDCAN_NDAT2_ND42_Msk                         /*!<New Data flag of Rx Buffer 42             */
#define FDCAN_NDAT2_ND43_Pos      (11U)
#define FDCAN_NDAT2_ND43_Msk      (0x1U << FDCAN_NDAT2_ND43_Pos)               /*!< 0x00000800 */
#define FDCAN_NDAT2_ND43          FDCAN_NDAT2_ND43_Msk                         /*!<New Data flag of Rx Buffer 43             */
#define FDCAN_NDAT2_ND44_Pos      (12U)
#define FDCAN_NDAT2_ND44_Msk      (0x1U << FDCAN_NDAT2_ND44_Pos)               /*!< 0x00001000 */
#define FDCAN_NDAT2_ND44          FDCAN_NDAT2_ND44_Msk                         /*!<New Data flag of Rx Buffer 44             */
#define FDCAN_NDAT2_ND45_Pos      (13U)
#define FDCAN_NDAT2_ND45_Msk      (0x1U << FDCAN_NDAT2_ND45_Pos)               /*!< 0x00002000 */
#define FDCAN_NDAT2_ND45          FDCAN_NDAT2_ND45_Msk                         /*!<New Data flag of Rx Buffer 45             */
#define FDCAN_NDAT2_ND46_Pos      (14U)
#define FDCAN_NDAT2_ND46_Msk      (0x1U << FDCAN_NDAT2_ND46_Pos)               /*!< 0x00004000 */
#define FDCAN_NDAT2_ND46          FDCAN_NDAT2_ND46_Msk                         /*!<New Data flag of Rx Buffer 46             */
#define FDCAN_NDAT2_ND47_Pos      (15U)
#define FDCAN_NDAT2_ND47_Msk      (0x1U << FDCAN_NDAT2_ND47_Pos)               /*!< 0x00008000 */
#define FDCAN_NDAT2_ND47          FDCAN_NDAT2_ND47_Msk                         /*!<New Data flag of Rx Buffer 47             */
#define FDCAN_NDAT2_ND48_Pos      (16U)
#define FDCAN_NDAT2_ND48_Msk      (0x1U << FDCAN_NDAT2_ND48_Pos)               /*!< 0x00010000 */
#define FDCAN_NDAT2_ND48          FDCAN_NDAT2_ND48_Msk                         /*!<New Data flag of Rx Buffer 48             */
#define FDCAN_NDAT2_ND49_Pos      (17U)
#define FDCAN_NDAT2_ND49_Msk      (0x1U << FDCAN_NDAT2_ND49_Pos)               /*!< 0x00020000 */
#define FDCAN_NDAT2_ND49          FDCAN_NDAT2_ND49_Msk                         /*!<New Data flag of Rx Buffer 49             */
#define FDCAN_NDAT2_ND50_Pos      (18U)
#define FDCAN_NDAT2_ND50_Msk      (0x1U << FDCAN_NDAT2_ND50_Pos)               /*!< 0x00040000 */
#define FDCAN_NDAT2_ND50          FDCAN_NDAT2_ND50_Msk                         /*!<New Data flag of Rx Buffer 50             */
#define FDCAN_NDAT2_ND51_Pos      (19U)
#define FDCAN_NDAT2_ND51_Msk      (0x1U << FDCAN_NDAT2_ND51_Pos)               /*!< 0x00080000 */
#define FDCAN_NDAT2_ND51          FDCAN_NDAT2_ND51_Msk                         /*!<New Data flag of Rx Buffer 51             */
#define FDCAN_NDAT2_ND52_Pos      (20U)
#define FDCAN_NDAT2_ND52_Msk      (0x1U << FDCAN_NDAT2_ND52_Pos)               /*!< 0x00100000 */
#define FDCAN_NDAT2_ND52          FDCAN_NDAT2_ND52_Msk                         /*!<New Data flag of Rx Buffer 52             */
#define FDCAN_NDAT2_ND53_Pos      (21U)
#define FDCAN_NDAT2_ND53_Msk      (0x1U << FDCAN_NDAT2_ND53_Pos)               /*!< 0x00200000 */
#define FDCAN_NDAT2_ND53          FDCAN_NDAT2_ND53_Msk                         /*!<New Data flag of Rx Buffer 53             */
#define FDCAN_NDAT2_ND54_Pos      (22U)
#define FDCAN_NDAT2_ND54_Msk      (0x1U << FDCAN_NDAT2_ND54_Pos)               /*!< 0x00400000 */
#define FDCAN_NDAT2_ND54          FDCAN_NDAT2_ND54_Msk                         /*!<New Data flag of Rx Buffer 54             */
#define FDCAN_NDAT2_ND55_Pos      (23U)
#define FDCAN_NDAT2_ND55_Msk      (0x1U << FDCAN_NDAT2_ND55_Pos)               /*!< 0x00800000 */
#define FDCAN_NDAT2_ND55          FDCAN_NDAT2_ND55_Msk                         /*!<New Data flag of Rx Buffer 55             */
#define FDCAN_NDAT2_ND56_Pos      (24U)
#define FDCAN_NDAT2_ND56_Msk      (0x1U << FDCAN_NDAT2_ND56_Pos)               /*!< 0x01000000 */
#define FDCAN_NDAT2_ND56          FDCAN_NDAT2_ND56_Msk                         /*!<New Data flag of Rx Buffer 56             */
#define FDCAN_NDAT2_ND57_Pos      (25U)
#define FDCAN_NDAT2_ND57_Msk      (0x1U << FDCAN_NDAT2_ND57_Pos)               /*!< 0x02000000 */
#define FDCAN_NDAT2_ND57          FDCAN_NDAT2_ND57_Msk                         /*!<New Data flag of Rx Buffer 57             */
#define FDCAN_NDAT2_ND58_Pos      (26U)
#define FDCAN_NDAT2_ND58_Msk      (0x1U << FDCAN_NDAT2_ND58_Pos)               /*!< 0x04000000 */
#define FDCAN_NDAT2_ND58          FDCAN_NDAT2_ND58_Msk                         /*!<New Data flag of Rx Buffer 58             */
#define FDCAN_NDAT2_ND59_Pos      (27U)
#define FDCAN_NDAT2_ND59_Msk      (0x1U << FDCAN_NDAT2_ND59_Pos)               /*!< 0x08000000 */
#define FDCAN_NDAT2_ND59          FDCAN_NDAT2_ND59_Msk                         /*!<New Data flag of Rx Buffer 59             */
#define FDCAN_NDAT2_ND60_Pos      (28U)
#define FDCAN_NDAT2_ND60_Msk      (0x1U << FDCAN_NDAT2_ND60_Pos)               /*!< 0x10000000 */
#define FDCAN_NDAT2_ND60          FDCAN_NDAT2_ND60_Msk                         /*!<New Data flag of Rx Buffer 60             */
#define FDCAN_NDAT2_ND61_Pos      (29U)
#define FDCAN_NDAT2_ND61_Msk      (0x1U << FDCAN_NDAT2_ND61_Pos)               /*!< 0x20000000 */
#define FDCAN_NDAT2_ND61          FDCAN_NDAT2_ND61_Msk                         /*!<New Data flag of Rx Buffer 61             */
#define FDCAN_NDAT2_ND62_Pos      (30U)
#define FDCAN_NDAT2_ND62_Msk      (0x1U << FDCAN_NDAT2_ND62_Pos)               /*!< 0x40000000 */
#define FDCAN_NDAT2_ND62          FDCAN_NDAT2_ND62_Msk                         /*!<New Data flag of Rx Buffer 62             */
#define FDCAN_NDAT2_ND63_Pos      (31U)
#define FDCAN_NDAT2_ND63_Msk      (0x1U << FDCAN_NDAT2_ND63_Pos)               /*!< 0x80000000 */
#define FDCAN_NDAT2_ND63          FDCAN_NDAT2_ND63_Msk                         /*!<New Data flag of Rx Buffer 63             */

/*****************  Bit definition for FDCAN_RXF0C register  ********************/
#define FDCAN_RXF0C_F0SA_Pos      (2U)
#define FDCAN_RXF0C_F0SA_Msk      (0x3FFFU << FDCAN_RXF0C_F0SA_Pos)            /*!< 0x0000FFFC */
#define FDCAN_RXF0C_F0SA          FDCAN_RXF0C_F0SA_Msk                         /*!<Rx FIFO 0 Start Address                   */
#define FDCAN_RXF0C_F0S_Pos       (16U)
#define FDCAN_RXF0C_F0S_Msk       (0x7FU << FDCAN_RXF0C_F0S_Pos)               /*!< 0x007F0000 */
#define FDCAN_RXF0C_F0S           FDCAN_RXF0C_F0S_Msk                          /*!<Number of Rx FIFO 0 elements              */
#define FDCAN_RXF0C_F0WM_Pos      (24U)
#define FDCAN_RXF0C_F0WM_Msk      (0x7FU << FDCAN_RXF0C_F0WM_Pos)              /*!< 0x7F000000 */
#define FDCAN_RXF0C_F0WM          FDCAN_RXF0C_F0WM_Msk                         /*!<FIFO 0 Watermark                          */
#define FDCAN_RXF0C_F0OM_Pos      (31U)
#define FDCAN_RXF0C_F0OM_Msk      (0x1U << FDCAN_RXF0C_F0OM_Pos)               /*!< 0x80000000 */
#define FDCAN_RXF0C_F0OM          FDCAN_RXF0C_F0OM_Msk                         /*!<FIFO 0 Operation Mode                     */

/*****************  Bit definition for FDCAN_RXF0S register  ********************/
#define FDCAN_RXF0S_F0FL_Pos      (0U)
#define FDCAN_RXF0S_F0FL_Msk      (0x7FU << FDCAN_RXF0S_F0FL_Pos)              /*!< 0x0000007F */
#define FDCAN_RXF0S_F0FL          FDCAN_RXF0S_F0FL_Msk                         /*!<Rx FIFO 0 Fill Level                      */
#define FDCAN_RXF0S_F0GI_Pos      (8U)
#define FDCAN_RXF0S_F0GI_Msk      (0x3FU << FDCAN_RXF0S_F0GI_Pos)              /*!< 0x00003F00 */
#define FDCAN_RXF0S_F0GI          FDCAN_RXF0S_F0GI_Msk                         /*!<Rx FIFO 0 Get Index                       */
#define FDCAN_RXF0S_F0PI_Pos      (16U)
#define FDCAN_RXF0S_F0PI_Msk      (0x3FU << FDCAN_RXF0S_F0PI_Pos)              /*!< 0x003F0000 */
#define FDCAN_RXF0S_F0PI          FDCAN_RXF0S_F0PI_Msk                         /*!<Rx FIFO 0 Put Index                       */
#define FDCAN_RXF0S_F0F_Pos       (24U)
#define FDCAN_RXF0S_F0F_Msk       (0x1U << FDCAN_RXF0S_F0F_Pos)                /*!< 0x01000000 */
#define FDCAN_RXF0S_F0F           FDCAN_RXF0S_F0F_Msk                          /*!<Rx FIFO 0 Full                            */
#define FDCAN_RXF0S_RF0L_Pos      (25U)
#define FDCAN_RXF0S_RF0L_Msk      (0x1U << FDCAN_RXF0S_RF0L_Pos)               /*!< 0x02000000 */
#define FDCAN_RXF0S_RF0L          FDCAN_RXF0S_RF0L_Msk                         /*!<Rx FIFO 0 Message Lost                    */

/*****************  Bit definition for FDCAN_RXF0A register  ********************/
#define FDCAN_RXF0A_F0AI_Pos      (0U)
#define FDCAN_RXF0A_F0AI_Msk      (0x3FU << FDCAN_RXF0A_F0AI_Pos)              /*!< 0x0000003F */
#define FDCAN_RXF0A_F0AI          FDCAN_RXF0A_F0AI_Msk                         /*!<Rx FIFO 0 Acknowledge Index               */

/*****************  Bit definition for FDCAN_RXBC register  ********************/
#define FDCAN_RXBC_RBSA_Pos       (2U)
#define FDCAN_RXBC_RBSA_Msk       (0x3FU << FDCAN_RXBC_RBSA_Pos)               /*!< 0x000000FC */
#define FDCAN_RXBC_RBSA           FDCAN_RXBC_RBSA_Msk                          /*!<Rx Buffer Start Address                   */

/*****************  Bit definition for FDCAN_RXF1C register  ********************/
#define FDCAN_RXF1C_F1SA_Pos      (2U)
#define FDCAN_RXF1C_F1SA_Msk      (0x3FU << FDCAN_RXF1C_F1SA_Pos)              /*!< 0x000000FC */
#define FDCAN_RXF1C_F1SA          FDCAN_RXF1C_F1SA_Msk                         /*!<Rx FIFO 1 Start Address                   */
#define FDCAN_RXF1C_F1S_Pos       (16U)
#define FDCAN_RXF1C_F1S_Msk       (0x7FU << FDCAN_RXF1C_F1S_Pos)               /*!< 0x007F0000 */
#define FDCAN_RXF1C_F1S           FDCAN_RXF1C_F1S_Msk                          /*!<Number of Rx FIFO 1 elements              */
#define FDCAN_RXF1C_F1WM_Pos      (24U)
#define FDCAN_RXF1C_F1WM_Msk      (0x7FU << FDCAN_RXF1C_F1WM_Pos)              /*!< 0x7F000000 */
#define FDCAN_RXF1C_F1WM          FDCAN_RXF1C_F1WM_Msk                         /*!<Rx FIFO 1 Watermark                       */
#define FDCAN_RXF1C_F1OM_Pos      (31U)
#define FDCAN_RXF1C_F1OM_Msk      (0x1U << FDCAN_RXF1C_F1OM_Pos)               /*!< 0x80000000 */
#define FDCAN_RXF1C_F1OM          FDCAN_RXF1C_F1OM_Msk                         /*!<FIFO 1 Operation Mode                     */

/*****************  Bit definition for FDCAN_RXF1S register  ********************/
#define FDCAN_RXF1S_F1FL_Pos      (0U)
#define FDCAN_RXF1S_F1FL_Msk      (0x7FU << FDCAN_RXF1S_F1FL_Pos)              /*!< 0x0000007F */
#define FDCAN_RXF1S_F1FL          FDCAN_RXF1S_F1FL_Msk                         /*!<Rx FIFO 1 Fill Level                      */
#define FDCAN_RXF1S_F1GI_Pos      (8U)
#define FDCAN_RXF1S_F1GI_Msk      (0x3FU << FDCAN_RXF1S_F1GI_Pos)              /*!< 0x00003F00 */
#define FDCAN_RXF1S_F1GI          FDCAN_RXF1S_F1GI_Msk                         /*!<Rx FIFO 1 Get Index                       */
#define FDCAN_RXF1S_F1PI_Pos      (16U)
#define FDCAN_RXF1S_F1PI_Msk      (0x3FU << FDCAN_RXF1S_F1PI_Pos)              /*!< 0x003F0000 */
#define FDCAN_RXF1S_F1PI          FDCAN_RXF1S_F1PI_Msk                         /*!<Rx FIFO 1 Put Index                       */
#define FDCAN_RXF1S_F1F_Pos       (24U)
#define FDCAN_RXF1S_F1F_Msk       (0x1U << FDCAN_RXF1S_F1F_Pos)                /*!< 0x01000000 */
#define FDCAN_RXF1S_F1F           FDCAN_RXF1S_F1F_Msk                          /*!<Rx FIFO 1 Full                            */
#define FDCAN_RXF1S_RF1L_Pos      (25U)
#define FDCAN_RXF1S_RF1L_Msk      (0x1U << FDCAN_RXF1S_RF1L_Pos)               /*!< 0x02000000 */
#define FDCAN_RXF1S_RF1L          FDCAN_RXF1S_RF1L_Msk                         /*!<Rx FIFO 1 Message Lost                    */

/*****************  Bit definition for FDCAN_RXF1A register  ********************/
#define FDCAN_RXF1A_F1AI_Pos      (0U)
#define FDCAN_RXF1A_F1AI_Msk      (0x3FU << FDCAN_RXF1A_F1AI_Pos)              /*!< 0x0000003F */
#define FDCAN_RXF1A_F1AI          FDCAN_RXF1A_F1AI_Msk                         /*!<Rx FIFO 1 Acknowledge Index               */

/*****************  Bit definition for FDCAN_RXESC register  ********************/
#define FDCAN_RXESC_F0DS_Pos      (0U)
#define FDCAN_RXESC_F0DS_Msk      (0x7U << FDCAN_RXESC_F0DS_Pos)               /*!< 0x00000007 */
#define FDCAN_RXESC_F0DS          FDCAN_RXESC_F0DS_Msk                         /*!<Rx FIFO 1 Data Field Size                 */
#define FDCAN_RXESC_F1DS_Pos      (4U)
#define FDCAN_RXESC_F1DS_Msk      (0x7U << FDCAN_RXESC_F1DS_Pos)               /*!< 0x00000070 */
#define FDCAN_RXESC_F1DS          FDCAN_RXESC_F1DS_Msk                         /*!<Rx FIFO 0 Data Field Size                 */
#define FDCAN_RXESC_RBDS_Pos      (8U)
#define FDCAN_RXESC_RBDS_Msk      (0x7U << FDCAN_RXESC_RBDS_Pos)               /*!< 0x00000700 */
#define FDCAN_RXESC_RBDS          FDCAN_RXESC_RBDS_Msk                         /*!<Rx Buffer Data Field Size                 */

/*****************  Bit definition for FDCAN_TXBC register  *********************/
#define FDCAN_TXBC_TBSA_Pos       (2U)
#define FDCAN_TXBC_TBSA_Msk       (0x3FU << FDCAN_TXBC_TBSA_Pos)               /*!< 0x000000FC */
#define FDCAN_TXBC_TBSA           FDCAN_TXBC_TBSA_Msk                          /*!<Tx Buffers Start Address                  */
#define FDCAN_TXBC_NDTB_Pos       (16U)
#define FDCAN_TXBC_NDTB_Msk       (0x3FU << FDCAN_TXBC_NDTB_Pos)               /*!< 0x003F0000 */
#define FDCAN_TXBC_NDTB           FDCAN_TXBC_NDTB_Msk                          /*!<Number of Dedicated Transmit Buffers      */
#define FDCAN_TXBC_TFQS_Pos       (24U)
#define FDCAN_TXBC_TFQS_Msk       (0x3FU << FDCAN_TXBC_TFQS_Pos)               /*!< 0x3F000000 */
#define FDCAN_TXBC_TFQS           FDCAN_TXBC_TFQS_Msk                          /*!<Transmit FIFO/Queue Size                  */
#define FDCAN_TXBC_TFQM_Pos       (30U)
#define FDCAN_TXBC_TFQM_Msk       (0x1U << FDCAN_TXBC_TFQM_Pos)                /*!< 0x40000000 */
#define FDCAN_TXBC_TFQM           FDCAN_TXBC_TFQM_Msk                          /*!<Tx FIFO/Queue Mode                        */

/*****************  Bit definition for FDCAN_TXFQS register  *********************/
#define FDCAN_TXFQS_TFFL_Pos      (0U)
#define FDCAN_TXFQS_TFFL_Msk      (0x3FU << FDCAN_TXFQS_TFFL_Pos)              /*!< 0x0000003F */
#define FDCAN_TXFQS_TFFL          FDCAN_TXFQS_TFFL_Msk                         /*!<Tx FIFO Free Level                        */
#define FDCAN_TXFQS_TFGI_Pos      (8U)
#define FDCAN_TXFQS_TFGI_Msk      (0x1FU << FDCAN_TXFQS_TFGI_Pos)              /*!< 0x00001F00 */
#define FDCAN_TXFQS_TFGI          FDCAN_TXFQS_TFGI_Msk                         /*!<Tx FIFO Get Index                         */
#define FDCAN_TXFQS_TFQPI_Pos     (16U)
#define FDCAN_TXFQS_TFQPI_Msk     (0x1FU << FDCAN_TXFQS_TFQPI_Pos)             /*!< 0x001F0000 */
#define FDCAN_TXFQS_TFQPI         FDCAN_TXFQS_TFQPI_Msk                        /*!<Tx FIFO/Queue Put Index                   */
#define FDCAN_TXFQS_TFQF_Pos      (21U)
#define FDCAN_TXFQS_TFQF_Msk      (0x1U << FDCAN_TXFQS_TFQF_Pos)               /*!< 0x00200000 */
#define FDCAN_TXFQS_TFQF          FDCAN_TXFQS_TFQF_Msk                         /*!<Tx FIFO/Queue Full                        */

/*****************  Bit definition for FDCAN_TXESC register  *********************/
#define FDCAN_TXESC_TBDS_Pos      (0U)
#define FDCAN_TXESC_TBDS_Msk      (0x7U << FDCAN_TXESC_TBDS_Pos)               /*!< 0x00000007 */
#define FDCAN_TXESC_TBDS          FDCAN_TXESC_TBDS_Msk                         /*!<Tx Buffer Data Field Size                 */

/*****************  Bit definition for FDCAN_TXBRP register  *********************/
#define FDCAN_TXBRP_TRP_Pos       (0U)
#define FDCAN_TXBRP_TRP_Msk       (0xFFFFFFFFU << FDCAN_TXBRP_TRP_Pos)         /*!< 0xFFFFFFFF */
#define FDCAN_TXBRP_TRP           FDCAN_TXBRP_TRP_Msk                          /*!<Transmission Request Pending              */

/*****************  Bit definition for FDCAN_TXBAR register  *********************/
#define FDCAN_TXBAR_AR_Pos        (0U)
#define FDCAN_TXBAR_AR_Msk        (0xFFFFFFFFU << FDCAN_TXBAR_AR_Pos)          /*!< 0xFFFFFFFF */
#define FDCAN_TXBAR_AR            FDCAN_TXBAR_AR_Msk                           /*!<Add Request                               */

/*****************  Bit definition for FDCAN_TXBCR register  *********************/
#define FDCAN_TXBCR_CR_Pos        (0U)
#define FDCAN_TXBCR_CR_Msk        (0xFFFFFFFFU << FDCAN_TXBCR_CR_Pos)          /*!< 0xFFFFFFFF */
#define FDCAN_TXBCR_CR            FDCAN_TXBCR_CR_Msk                           /*!<Cancellation Request                      */

/*****************  Bit definition for FDCAN_TXBTO register  *********************/
#define FDCAN_TXBTO_TO_Pos        (0U)
#define FDCAN_TXBTO_TO_Msk        (0xFFFFFFFFU << FDCAN_TXBTO_TO_Pos)          /*!< 0xFFFFFFFF */
#define FDCAN_TXBTO_TO            FDCAN_TXBTO_TO_Msk                           /*!<Transmission Occurred                     */

/*****************  Bit definition for FDCAN_TXBCF register  *********************/
#define FDCAN_TXBCF_CF_Pos        (0U)
#define FDCAN_TXBCF_CF_Msk        (0xFFFFFFFFU << FDCAN_TXBCF_CF_Pos)          /*!< 0xFFFFFFFF */
#define FDCAN_TXBCF_CF            FDCAN_TXBCF_CF_Msk                           /*!<Cancellation Finished                     */

/*****************  Bit definition for FDCAN_TXBTIE register  ********************/
#define FDCAN_TXBTIE_TIE_Pos      (0U)
#define FDCAN_TXBTIE_TIE_Msk      (0xFFFFFFFFU << FDCAN_TXBTIE_TIE_Pos)        /*!< 0xFFFFFFFF */
#define FDCAN_TXBTIE_TIE          FDCAN_TXBTIE_TIE_Msk                         /*!<Transmission Interrupt Enable             */

/*****************  Bit definition for FDCAN_ TXBCIE register  *******************/
#define FDCAN_TXBCIE_CF_Pos       (0U)
#define FDCAN_TXBCIE_CF_Msk       (0xFFFFFFFFU << FDCAN_TXBCIE_CF_Pos)         /*!< 0xFFFFFFFF */
#define FDCAN_TXBCIE_CF           FDCAN_TXBCIE_CF_Msk                          /*!<Cancellation Finished Interrupt Enable    */

/*****************  Bit definition for FDCAN_TXEFC register  *********************/
#define FDCAN_TXEFC_EFSA_Pos      (2U)
#define FDCAN_TXEFC_EFSA_Msk      (0x3FU << FDCAN_TXEFC_EFSA_Pos)              /*!< 0x000000FC */
#define FDCAN_TXEFC_EFSA          FDCAN_TXEFC_EFSA_Msk                         /*!<Event FIFO Start Address                  */
#define FDCAN_TXEFC_EFS_Pos       (8U)
#define FDCAN_TXEFC_EFS_Msk       (0x3FU << FDCAN_TXEFC_EFS_Pos)               /*!< 0x00003F00 */
#define FDCAN_TXEFC_EFS           FDCAN_TXEFC_EFS_Msk                          /*!<Event FIFO Size                           */
#define FDCAN_TXEFC_EFWM_Pos      (24U)
#define FDCAN_TXEFC_EFWM_Msk      (0x3FU << FDCAN_TXEFC_EFWM_Pos)              /*!< 0x3F000000 */
#define FDCAN_TXEFC_EFWM          FDCAN_TXEFC_EFWM_Msk                         /*!<Event FIFO Watermark                      */

/*****************  Bit definition for FDCAN_TXEFS register  *********************/
#define FDCAN_TXEFS_EFFL_Pos      (0U)
#define FDCAN_TXEFS_EFFL_Msk      (0x3FU << FDCAN_TXEFS_EFFL_Pos)              /*!< 0x0000003F */
#define FDCAN_TXEFS_EFFL          FDCAN_TXEFS_EFFL_Msk                         /*!<Event FIFO Fill Level                     */
#define FDCAN_TXEFS_EFGI_Pos      (8U)
#define FDCAN_TXEFS_EFGI_Msk      (0x1FU << FDCAN_TXEFS_EFGI_Pos)              /*!< 0x00001F00 */
#define FDCAN_TXEFS_EFGI          FDCAN_TXEFS_EFGI_Msk                         /*!<Event FIFO Get Index                      */
#define FDCAN_TXEFS_EFPI_Pos      (16U)
#define FDCAN_TXEFS_EFPI_Msk      (0x1FU << FDCAN_TXEFS_EFPI_Pos)              /*!< 0x001F0000 */
#define FDCAN_TXEFS_EFPI          FDCAN_TXEFS_EFPI_Msk                         /*!<Event FIFO Put Index                      */
#define FDCAN_TXEFS_EFF_Pos       (24U)
#define FDCAN_TXEFS_EFF_Msk       (0x1U << FDCAN_TXEFS_EFF_Pos)                /*!< 0x01000000 */
#define FDCAN_TXEFS_EFF           FDCAN_TXEFS_EFF_Msk                          /*!<Event FIFO Full                           */
#define FDCAN_TXEFS_TEFL_Pos      (25U)
#define FDCAN_TXEFS_TEFL_Msk      (0x1U << FDCAN_TXEFS_TEFL_Pos)               /*!< 0x02000000 */
#define FDCAN_TXEFS_TEFL          FDCAN_TXEFS_TEFL_Msk                         /*!<Tx Event FIFO Element Lost                */

/*****************  Bit definition for FDCAN_TXEFA register  *********************/
#define FDCAN_TXEFA_EFAI_Pos      (0U)
#define FDCAN_TXEFA_EFAI_Msk      (0x1FU << FDCAN_TXEFA_EFAI_Pos)              /*!< 0x0000001F */
#define FDCAN_TXEFA_EFAI          FDCAN_TXEFA_EFAI_Msk                         /*!<Event FIFO Acknowledge Index              */

/*****************  Bit definition for FDCAN_TTTMC register  *********************/
#define FDCAN_TTTMC_TMSA_Pos      (2U)
#define FDCAN_TTTMC_TMSA_Msk      (0x3FFFU << FDCAN_TTTMC_TMSA_Pos)            /*!< 0x0000FFFC */
#define FDCAN_TTTMC_TMSA          FDCAN_TTTMC_TMSA_Msk                         /*!<Trigger Memory Start Address              */
#define FDCAN_TTTMC_TME_Pos       (16U)
#define FDCAN_TTTMC_TME_Msk       (0x7FU << FDCAN_TTTMC_TME_Pos)               /*!< 0x007F0000 */
#define FDCAN_TTTMC_TME           FDCAN_TTTMC_TME_Msk                          /*!<Trigger Memory Elements                   */

/*****************  Bit definition for FDCAN_TTRMC register  *********************/
#define FDCAN_TTRMC_RID_Pos       (0U)
#define FDCAN_TTRMC_RID_Msk       (0x1FFFFFFFU << FDCAN_TTRMC_RID_Pos)         /*!< 0x1FFFFFFF */
#define FDCAN_TTRMC_RID           FDCAN_TTRMC_RID_Msk                          /*!<Reference Identifier                      */
#define FDCAN_TTRMC_XTD_Pos       (30U)
#define FDCAN_TTRMC_XTD_Msk       (0x1U << FDCAN_TTRMC_XTD_Pos)                /*!< 0x40000000 */
#define FDCAN_TTRMC_XTD           FDCAN_TTRMC_XTD_Msk                          /*!< Extended Identifier                      */
#define FDCAN_TTRMC_RMPS_Pos      (31U)
#define FDCAN_TTRMC_RMPS_Msk      (0x1U << FDCAN_TTRMC_RMPS_Pos)               /*!< 0x80000000 */
#define FDCAN_TTRMC_RMPS          FDCAN_TTRMC_RMPS_Msk                         /*!<Reference Message Payload Select          */

/*****************  Bit definition for FDCAN_TTOCF register  *********************/
#define FDCAN_TTOCF_OM_Pos        (0U)
#define FDCAN_TTOCF_OM_Msk        (0x3U << FDCAN_TTOCF_OM_Pos)                 /*!< 0x00000003 */
#define FDCAN_TTOCF_OM            FDCAN_TTOCF_OM_Msk                           /*!<Operation Mode                            */
#define FDCAN_TTOCF_GEN_Pos       (3U)
#define FDCAN_TTOCF_GEN_Msk       (0x1U << FDCAN_TTOCF_GEN_Pos)                /*!< 0x00000008 */
#define FDCAN_TTOCF_GEN           FDCAN_TTOCF_GEN_Msk                          /*!<Gap Enable                                */
#define FDCAN_TTOCF_TM_Pos        (4U)
#define FDCAN_TTOCF_TM_Msk        (0x1U << FDCAN_TTOCF_TM_Pos)                 /*!< 0x00000010 */
#define FDCAN_TTOCF_TM            FDCAN_TTOCF_TM_Msk                           /*!<Time Master                               */
#define FDCAN_TTOCF_LDSDL_Pos     (5U)
#define FDCAN_TTOCF_LDSDL_Msk     (0x7U << FDCAN_TTOCF_LDSDL_Pos)              /*!< 0x000000E0 */
#define FDCAN_TTOCF_LDSDL         FDCAN_TTOCF_LDSDL_Msk                        /*!<LD of Synchronization Deviation Limit     */
#define FDCAN_TTOCF_IRTO_Pos      (8U)
#define FDCAN_TTOCF_IRTO_Msk      (0x7FU << FDCAN_TTOCF_IRTO_Pos)              /*!< 0x00007F00 */
#define FDCAN_TTOCF_IRTO          FDCAN_TTOCF_IRTO_Msk                         /*!<Initial Reference Trigger Offset          */
#define FDCAN_TTOCF_EECS_Pos      (15U)
#define FDCAN_TTOCF_EECS_Msk      (0x1U << FDCAN_TTOCF_EECS_Pos)               /*!< 0x00008000 */
#define FDCAN_TTOCF_EECS          FDCAN_TTOCF_EECS_Msk                         /*!<Enable External Clock Synchronization     */
#define FDCAN_TTOCF_AWL_Pos       (16U)
#define FDCAN_TTOCF_AWL_Msk       (0xFFU << FDCAN_TTOCF_AWL_Pos)               /*!< 0x00FF0000 */
#define FDCAN_TTOCF_AWL           FDCAN_TTOCF_AWL_Msk                          /*!<Application Watchdog Limit                */
#define FDCAN_TTOCF_EGTF_Pos      (24U)
#define FDCAN_TTOCF_EGTF_Msk      (0x1U << FDCAN_TTOCF_EGTF_Pos)               /*!< 0x01000000 */
#define FDCAN_TTOCF_EGTF          FDCAN_TTOCF_EGTF_Msk                         /*!<Enable Global Time Filtering              */
#define FDCAN_TTOCF_ECC_Pos       (25U)
#define FDCAN_TTOCF_ECC_Msk       (0x1U << FDCAN_TTOCF_ECC_Pos)                /*!< 0x02000000 */
#define FDCAN_TTOCF_ECC           FDCAN_TTOCF_ECC_Msk                          /*!<Enable Clock Calibration                  */
#define FDCAN_TTOCF_EVTP_Pos      (26U)
#define FDCAN_TTOCF_EVTP_Msk      (0x1U << FDCAN_TTOCF_EVTP_Pos)               /*!< 0x04000000 */
#define FDCAN_TTOCF_EVTP          FDCAN_TTOCF_EVTP_Msk                         /*!<Event Trigger Polarity                    */

/*****************  Bit definition for FDCAN_TTMLM register  *********************/
#define FDCAN_TTMLM_CCM_Pos       (0U)
#define FDCAN_TTMLM_CCM_Msk       (0x3FU << FDCAN_TTMLM_CCM_Pos)               /*!< 0x0000003F */
#define FDCAN_TTMLM_CCM           FDCAN_TTMLM_CCM_Msk                          /*!<Cycle Count Max                           */
#define FDCAN_TTMLM_CSS_Pos       (6U)
#define FDCAN_TTMLM_CSS_Msk       (0x3U << FDCAN_TTMLM_CSS_Pos)                /*!< 0x000000C0 */
#define FDCAN_TTMLM_CSS           FDCAN_TTMLM_CSS_Msk                          /*!<Cycle Start Synchronization               */
#define FDCAN_TTMLM_TXEW_Pos      (8U)
#define FDCAN_TTMLM_TXEW_Msk      (0xFU << FDCAN_TTMLM_TXEW_Pos)               /*!< 0x00000F00 */
#define FDCAN_TTMLM_TXEW          FDCAN_TTMLM_TXEW_Msk                         /*!<Tx Enable Window                          */
#define FDCAN_TTMLM_ENTT_Pos      (16U)
#define FDCAN_TTMLM_ENTT_Msk      (0xFFFU << FDCAN_TTMLM_ENTT_Pos)             /*!< 0x0FFF0000 */
#define FDCAN_TTMLM_ENTT          FDCAN_TTMLM_ENTT_Msk                         /*!<Expected Number of Tx Triggers            */

/*****************  Bit definition for FDCAN_TURCF register  *********************/
#define FDCAN_TURCF_NCL_Pos       (0U)
#define FDCAN_TURCF_NCL_Msk       (0xFFFFU << FDCAN_TURCF_NCL_Pos)             /*!< 0x0000FFFF */
#define FDCAN_TURCF_NCL           FDCAN_TURCF_NCL_Msk                          /*!<Numerator Configuration Low               */
#define FDCAN_TURCF_DC_Pos        (16U)
#define FDCAN_TURCF_DC_Msk        (0x3FFFU << FDCAN_TURCF_DC_Pos)              /*!< 0x3FFF0000 */
#define FDCAN_TURCF_DC            FDCAN_TURCF_DC_Msk                           /*!<Denominator Configuration                 */
#define FDCAN_TURCF_ELT_Pos       (31U)
#define FDCAN_TURCF_ELT_Msk       (0x1U << FDCAN_TURCF_ELT_Pos)                /*!< 0x80000000 */
#define FDCAN_TURCF_ELT           FDCAN_TURCF_ELT_Msk                          /*!<Enable Local Time                         */

/*****************  Bit definition for FDCAN_TTOCN register  ********************/
#define FDCAN_TTOCN_SGT_Pos       (0U)
#define FDCAN_TTOCN_SGT_Msk       (0x1U << FDCAN_TTOCN_SGT_Pos)                /*!< 0x00000001 */
#define FDCAN_TTOCN_SGT           FDCAN_TTOCN_SGT_Msk                          /*!<Set Global time                           */
#define FDCAN_TTOCN_ECS_Pos       (1U)
#define FDCAN_TTOCN_ECS_Msk       (0x1U << FDCAN_TTOCN_ECS_Pos)                /*!< 0x00000002 */
#define FDCAN_TTOCN_ECS           FDCAN_TTOCN_ECS_Msk                          /*!<External Clock Synchronization            */
#define FDCAN_TTOCN_SWP_Pos       (2U)
#define FDCAN_TTOCN_SWP_Msk       (0x1U << FDCAN_TTOCN_SWP_Pos)                /*!< 0x00000004 */
#define FDCAN_TTOCN_SWP           FDCAN_TTOCN_SWP_Msk                          /*!<Stop Watch Polarity                       */
#define FDCAN_TTOCN_SWS_Pos       (3U)
#define FDCAN_TTOCN_SWS_Msk       (0x3U << FDCAN_TTOCN_SWS_Pos)                /*!< 0x00000018 */
#define FDCAN_TTOCN_SWS           FDCAN_TTOCN_SWS_Msk                          /*!<Stop Watch Source                         */
#define FDCAN_TTOCN_RTIE_Pos      (5U)
#define FDCAN_TTOCN_RTIE_Msk      (0x1U << FDCAN_TTOCN_RTIE_Pos)               /*!< 0x00000020 */
#define FDCAN_TTOCN_RTIE          FDCAN_TTOCN_RTIE_Msk                         /*!<Register Time Mark Interrupt Pulse Enable */
#define FDCAN_TTOCN_TMC_Pos       (6U)
#define FDCAN_TTOCN_TMC_Msk       (0x3U << FDCAN_TTOCN_TMC_Pos)                /*!< 0x000000C0 */
#define FDCAN_TTOCN_TMC           FDCAN_TTOCN_TMC_Msk                          /*!<Register Time Mark Compare                */
#define FDCAN_TTOCN_TTIE_Pos      (8U)
#define FDCAN_TTOCN_TTIE_Msk      (0x1U << FDCAN_TTOCN_TTIE_Pos)               /*!< 0x00000100 */
#define FDCAN_TTOCN_TTIE          FDCAN_TTOCN_TTIE_Msk                         /*!<Trigger Time Mark Interrupt Pulse Enable  */
#define FDCAN_TTOCN_GCS_Pos       (9U)
#define FDCAN_TTOCN_GCS_Msk       (0x1U << FDCAN_TTOCN_GCS_Pos)                /*!< 0x00000200 */
#define FDCAN_TTOCN_GCS           FDCAN_TTOCN_GCS_Msk                          /*!<Gap Control Select                        */
#define FDCAN_TTOCN_FGP_Pos       (10U)
#define FDCAN_TTOCN_FGP_Msk       (0x1U << FDCAN_TTOCN_FGP_Pos)                /*!< 0x00000400 */
#define FDCAN_TTOCN_FGP           FDCAN_TTOCN_FGP_Msk                          /*!<Finish Gap                                */
#define FDCAN_TTOCN_TMG_Pos       (11U)
#define FDCAN_TTOCN_TMG_Msk       (0x1U << FDCAN_TTOCN_TMG_Pos)                /*!< 0x00000800 */
#define FDCAN_TTOCN_TMG           FDCAN_TTOCN_TMG_Msk                          /*!<Time Mark Gap                             */
#define FDCAN_TTOCN_NIG_Pos       (12U)
#define FDCAN_TTOCN_NIG_Msk       (0x1U << FDCAN_TTOCN_NIG_Pos)                /*!< 0x00001000 */
#define FDCAN_TTOCN_NIG           FDCAN_TTOCN_NIG_Msk                          /*!<Next is Gap                               */
#define FDCAN_TTOCN_ESCN_Pos      (13U)
#define FDCAN_TTOCN_ESCN_Msk      (0x1U << FDCAN_TTOCN_ESCN_Pos)               /*!< 0x00002000 */
#define FDCAN_TTOCN_ESCN          FDCAN_TTOCN_ESCN_Msk                         /*!<External Synchronization Control          */
#define FDCAN_TTOCN_LCKC_Pos      (15U)
#define FDCAN_TTOCN_LCKC_Msk      (0x1U << FDCAN_TTOCN_LCKC_Pos)               /*!< 0x00008000 */
#define FDCAN_TTOCN_LCKC          FDCAN_TTOCN_LCKC_Msk                         /*!<TT Operation Control Register Locked      */

/*****************  Bit definition for FDCAN_TTGTP register  ********************/
#define FDCAN_TTGTP_TP_Pos        (0U)
#define FDCAN_TTGTP_TP_Msk        (0xFFFFU << FDCAN_TTGTP_TP_Pos)              /*!< 0x0000FFFF */
#define FDCAN_TTGTP_TP            FDCAN_TTGTP_TP_Msk                           /*!<Time Preset                               */
#define FDCAN_TTGTP_CTP_Pos       (16U)
#define FDCAN_TTGTP_CTP_Msk       (0xFFFFU << FDCAN_TTGTP_CTP_Pos)             /*!< 0xFFFF0000 */
#define FDCAN_TTGTP_CTP           FDCAN_TTGTP_CTP_Msk                          /*!<Cycle Time Target Phase                   */

/*****************  Bit definition for FDCAN_TTTMK register  ********************/
#define FDCAN_TTTMK_TM_Pos        (0U)
#define FDCAN_TTTMK_TM_Msk        (0xFFFFU << FDCAN_TTTMK_TM_Pos)              /*!< 0x0000FFFF */
#define FDCAN_TTTMK_TM            FDCAN_TTTMK_TM_Msk                           /*!<Time Mark                                 */
#define FDCAN_TTTMK_TICC_Pos      (16U)
#define FDCAN_TTTMK_TICC_Msk      (0x7FU << FDCAN_TTTMK_TICC_Pos)              /*!< 0x007F0000 */
#define FDCAN_TTTMK_TICC          FDCAN_TTTMK_TICC_Msk                         /*!<Time Mark Cycle Code                      */
#define FDCAN_TTTMK_LCKM_Pos      (31U)
#define FDCAN_TTTMK_LCKM_Msk      (0x1U << FDCAN_TTTMK_LCKM_Pos)               /*!< 0x80000000 */
#define FDCAN_TTTMK_LCKM          FDCAN_TTTMK_LCKM_Msk                         /*!<TT Time Mark Register Locked              */

/*****************  Bit definition for FDCAN_TTIR register  ********************/
#define FDCAN_TTIR_SBC_Pos        (0U)
#define FDCAN_TTIR_SBC_Msk        (0x1U << FDCAN_TTIR_SBC_Pos)                 /*!< 0x00000001 */
#define FDCAN_TTIR_SBC            FDCAN_TTIR_SBC_Msk                           /*!<Start of Basic Cycle                      */
#define FDCAN_TTIR_SMC_Pos        (1U)
#define FDCAN_TTIR_SMC_Msk        (0x1U << FDCAN_TTIR_SMC_Pos)                 /*!< 0x00000002 */
#define FDCAN_TTIR_SMC            FDCAN_TTIR_SMC_Msk                           /*!<Start of Matrix Cycle                     */
#define FDCAN_TTIR_CSM_Pos        (2U)
#define FDCAN_TTIR_CSM_Msk        (0x1U << FDCAN_TTIR_CSM_Pos)                 /*!< 0x00000004 */
#define FDCAN_TTIR_CSM            FDCAN_TTIR_CSM_Msk                           /*!<Change of Synchronization Mode            */
#define FDCAN_TTIR_SOG_Pos        (3U)
#define FDCAN_TTIR_SOG_Msk        (0x1U << FDCAN_TTIR_SOG_Pos)                 /*!< 0x00000008 */
#define FDCAN_TTIR_SOG            FDCAN_TTIR_SOG_Msk                           /*!<Start of Gap                              */
#define FDCAN_TTIR_RTMI_Pos       (4U)
#define FDCAN_TTIR_RTMI_Msk       (0x1U << FDCAN_TTIR_RTMI_Pos)                /*!< 0x00000010 */
#define FDCAN_TTIR_RTMI           FDCAN_TTIR_RTMI_Msk                          /*!<Register Time Mark Interrupt              */
#define FDCAN_TTIR_TTMI_Pos       (5U)
#define FDCAN_TTIR_TTMI_Msk       (0x1U << FDCAN_TTIR_TTMI_Pos)                /*!< 0x00000020 */
#define FDCAN_TTIR_TTMI           FDCAN_TTIR_TTMI_Msk                          /*!<Trigger Time Mark Event Internal          */
#define FDCAN_TTIR_SWE_Pos        (6U)
#define FDCAN_TTIR_SWE_Msk        (0x1U << FDCAN_TTIR_SWE_Pos)                 /*!< 0x00000040 */
#define FDCAN_TTIR_SWE            FDCAN_TTIR_SWE_Msk                           /*!<Stop Watch Event                          */
#define FDCAN_TTIR_GTW_Pos        (7U)
#define FDCAN_TTIR_GTW_Msk        (0x1U << FDCAN_TTIR_GTW_Pos)                 /*!< 0x00000080 */
#define FDCAN_TTIR_GTW            FDCAN_TTIR_GTW_Msk                           /*!<Global Time Wrap                          */
#define FDCAN_TTIR_GTD_Pos        (8U)
#define FDCAN_TTIR_GTD_Msk        (0x1U << FDCAN_TTIR_GTD_Pos)                 /*!< 0x00000100 */
#define FDCAN_TTIR_GTD            FDCAN_TTIR_GTD_Msk                           /*!<Global Time Discontinuity                 */
#define FDCAN_TTIR_GTE_Pos        (9U)
#define FDCAN_TTIR_GTE_Msk        (0x1U << FDCAN_TTIR_GTE_Pos)                 /*!< 0x00000200 */
#define FDCAN_TTIR_GTE            FDCAN_TTIR_GTE_Msk                           /*!<Global Time Error                         */
#define FDCAN_TTIR_TXU_Pos        (10U)
#define FDCAN_TTIR_TXU_Msk        (0x1U << FDCAN_TTIR_TXU_Pos)                 /*!< 0x00000400 */
#define FDCAN_TTIR_TXU            FDCAN_TTIR_TXU_Msk                           /*!<Tx Count Underflow                        */
#define FDCAN_TTIR_TXO_Pos        (11U)
#define FDCAN_TTIR_TXO_Msk        (0x1U << FDCAN_TTIR_TXO_Pos)                 /*!< 0x00000800 */
#define FDCAN_TTIR_TXO            FDCAN_TTIR_TXO_Msk                           /*!<Tx Count Overflow                         */
#define FDCAN_TTIR_SE1_Pos        (12U)
#define FDCAN_TTIR_SE1_Msk        (0x1U << FDCAN_TTIR_SE1_Pos)                 /*!< 0x00001000 */
#define FDCAN_TTIR_SE1            FDCAN_TTIR_SE1_Msk                           /*!<Scheduling Error 1                        */
#define FDCAN_TTIR_SE2_Pos        (13U)
#define FDCAN_TTIR_SE2_Msk        (0x1U << FDCAN_TTIR_SE2_Pos)                 /*!< 0x00002000 */
#define FDCAN_TTIR_SE2            FDCAN_TTIR_SE2_Msk                           /*!<Scheduling Error 2                        */
#define FDCAN_TTIR_ELC_Pos        (14U)
#define FDCAN_TTIR_ELC_Msk        (0x1U << FDCAN_TTIR_ELC_Pos)                 /*!< 0x00004000 */
#define FDCAN_TTIR_ELC            FDCAN_TTIR_ELC_Msk                           /*!<Error Level Changed                       */
#define FDCAN_TTIR_IWT_Pos        (15U)
#define FDCAN_TTIR_IWT_Msk        (0x1U << FDCAN_TTIR_IWT_Pos)                 /*!< 0x00008000 */
#define FDCAN_TTIR_IWT            FDCAN_TTIR_IWT_Msk                           /*!<Initialization Watch Trigger              */
#define FDCAN_TTIR_WT_Pos         (16U)
#define FDCAN_TTIR_WT_Msk         (0x1U << FDCAN_TTIR_WT_Pos)                  /*!< 0x00010000 */
#define FDCAN_TTIR_WT             FDCAN_TTIR_WT_Msk                            /*!<Watch Trigger                             */
#define FDCAN_TTIR_AW_Pos         (17U)
#define FDCAN_TTIR_AW_Msk         (0x1U << FDCAN_TTIR_AW_Pos)                  /*!< 0x00020000 */
#define FDCAN_TTIR_AW             FDCAN_TTIR_AW_Msk                            /*!<Application Watchdog                      */
#define FDCAN_TTIR_CER_Pos        (18U)
#define FDCAN_TTIR_CER_Msk        (0x1U << FDCAN_TTIR_CER_Pos)                 /*!< 0x00040000 */
#define FDCAN_TTIR_CER            FDCAN_TTIR_CER_Msk                           /*!<Configuration Error                       */

/*****************  Bit definition for FDCAN_TTIE register  ********************/
#define FDCAN_TTIE_SBCE_Pos       (0U)
#define FDCAN_TTIE_SBCE_Msk       (0x1U << FDCAN_TTIE_SBCE_Pos)                /*!< 0x00000001 */
#define FDCAN_TTIE_SBCE           FDCAN_TTIE_SBCE_Msk                          /*!<Start of Basic Cycle Interrupt Enable             */
#define FDCAN_TTIE_SMCE_Pos       (1U)
#define FDCAN_TTIE_SMCE_Msk       (0x1U << FDCAN_TTIE_SMCE_Pos)                /*!< 0x00000002 */
#define FDCAN_TTIE_SMCE           FDCAN_TTIE_SMCE_Msk                          /*!<Start of Matrix Cycle Interrupt Enable            */
#define FDCAN_TTIE_CSME_Pos       (2U)
#define FDCAN_TTIE_CSME_Msk       (0x1U << FDCAN_TTIE_CSME_Pos)                /*!< 0x00000004 */
#define FDCAN_TTIE_CSME           FDCAN_TTIE_CSME_Msk                          /*!<Change of Synchronization Mode Interrupt Enable   */
#define FDCAN_TTIE_SOGE_Pos       (3U)
#define FDCAN_TTIE_SOGE_Msk       (0x1U << FDCAN_TTIE_SOGE_Pos)                /*!< 0x00000008 */
#define FDCAN_TTIE_SOGE           FDCAN_TTIE_SOGE_Msk                          /*!<Start of Gap Interrupt Enable                     */
#define FDCAN_TTIE_RTMIE_Pos      (4U)
#define FDCAN_TTIE_RTMIE_Msk      (0x1U << FDCAN_TTIE_RTMIE_Pos)               /*!< 0x00000010 */
#define FDCAN_TTIE_RTMIE          FDCAN_TTIE_RTMIE_Msk                         /*!<Register Time Mark Interrupt Interrupt Enable     */
#define FDCAN_TTIE_TTMIE_Pos      (5U)
#define FDCAN_TTIE_TTMIE_Msk      (0x1U << FDCAN_TTIE_TTMIE_Pos)               /*!< 0x00000020 */
#define FDCAN_TTIE_TTMIE          FDCAN_TTIE_TTMIE_Msk                         /*!<Trigger Time Mark Event Internal Interrupt Enable */
#define FDCAN_TTIE_SWEE_Pos       (6U)
#define FDCAN_TTIE_SWEE_Msk       (0x1U << FDCAN_TTIE_SWEE_Pos)                /*!< 0x00000040 */
#define FDCAN_TTIE_SWEE           FDCAN_TTIE_SWEE_Msk                          /*!<Stop Watch Event Interrupt Enable                 */
#define FDCAN_TTIE_GTWE_Pos       (7U)
#define FDCAN_TTIE_GTWE_Msk       (0x1U << FDCAN_TTIE_GTWE_Pos)                /*!< 0x00000080 */
#define FDCAN_TTIE_GTWE           FDCAN_TTIE_GTWE_Msk                          /*!<Global Time Wrap Interrupt Enable                 */
#define FDCAN_TTIE_GTDE_Pos       (8U)
#define FDCAN_TTIE_GTDE_Msk       (0x1U << FDCAN_TTIE_GTDE_Pos)                /*!< 0x00000100 */
#define FDCAN_TTIE_GTDE           FDCAN_TTIE_GTDE_Msk                          /*!<Global Time Discontinuity Interrupt Enable        */
#define FDCAN_TTIE_GTEE_Pos       (9U)
#define FDCAN_TTIE_GTEE_Msk       (0x1U << FDCAN_TTIE_GTEE_Pos)                /*!< 0x00000200 */
#define FDCAN_TTIE_GTEE           FDCAN_TTIE_GTEE_Msk                          /*!<Global Time Error Interrupt Enable                */
#define FDCAN_TTIE_TXUE_Pos       (10U)
#define FDCAN_TTIE_TXUE_Msk       (0x1U << FDCAN_TTIE_TXUE_Pos)                /*!< 0x00000400 */
#define FDCAN_TTIE_TXUE           FDCAN_TTIE_TXUE_Msk                          /*!<Tx Count Underflow Interrupt Enable               */
#define FDCAN_TTIE_TXOE_Pos       (11U)
#define FDCAN_TTIE_TXOE_Msk       (0x1U << FDCAN_TTIE_TXOE_Pos)                /*!< 0x00000800 */
#define FDCAN_TTIE_TXOE           FDCAN_TTIE_TXOE_Msk                          /*!<Tx Count Overflow Interrupt Enable                */
#define FDCAN_TTIE_SE1E_Pos       (12U)
#define FDCAN_TTIE_SE1E_Msk       (0x1U << FDCAN_TTIE_SE1E_Pos)                /*!< 0x00001000 */
#define FDCAN_TTIE_SE1E           FDCAN_TTIE_SE1E_Msk                          /*!<Scheduling Error 1 Interrupt Enable               */
#define FDCAN_TTIE_SE2E_Pos       (13U)
#define FDCAN_TTIE_SE2E_Msk       (0x1U << FDCAN_TTIE_SE2E_Pos)                /*!< 0x00002000 */
#define FDCAN_TTIE_SE2E           FDCAN_TTIE_SE2E_Msk                          /*!<Scheduling Error 2 Interrupt Enable               */
#define FDCAN_TTIE_ELCE_Pos       (14U)
#define FDCAN_TTIE_ELCE_Msk       (0x1U << FDCAN_TTIE_ELCE_Pos)                /*!< 0x00004000 */
#define FDCAN_TTIE_ELCE           FDCAN_TTIE_ELCE_Msk                          /*!<Error Level Changed Interrupt Enable              */
#define FDCAN_TTIE_IWTE_Pos       (15U)
#define FDCAN_TTIE_IWTE_Msk       (0x1U << FDCAN_TTIE_IWTE_Pos)                /*!< 0x00008000 */
#define FDCAN_TTIE_IWTE           FDCAN_TTIE_IWTE_Msk                          /*!<Initialization Watch Trigger Interrupt Enable     */
#define FDCAN_TTIE_WTE_Pos        (16U)
#define FDCAN_TTIE_WTE_Msk        (0x1U << FDCAN_TTIE_WTE_Pos)                 /*!< 0x00010000 */
#define FDCAN_TTIE_WTE            FDCAN_TTIE_WTE_Msk                           /*!<Watch Trigger Interrupt Enable                    */
#define FDCAN_TTIE_AWE_Pos        (17U)
#define FDCAN_TTIE_AWE_Msk        (0x1U << FDCAN_TTIE_AWE_Pos)                 /*!< 0x00020000 */
#define FDCAN_TTIE_AWE            FDCAN_TTIE_AWE_Msk                           /*!<Application Watchdog Interrupt Enable             */
#define FDCAN_TTIE_CERE_Pos       (18U)
#define FDCAN_TTIE_CERE_Msk       (0x1U << FDCAN_TTIE_CERE_Pos)                /*!< 0x00040000 */
#define FDCAN_TTIE_CERE           FDCAN_TTIE_CERE_Msk                          /*!<Configuration Error Interrupt Enable              */

/*****************  Bit definition for FDCAN_TTILS register  ********************/
#define FDCAN_TTILS_SBCS_Pos      (0U)
#define FDCAN_TTILS_SBCS_Msk      (0x1U << FDCAN_TTILS_SBCS_Pos)               /*!< 0x00000001 */
#define FDCAN_TTILS_SBCS          FDCAN_TTILS_SBCS_Msk                         /*!<Start of Basic Cycle Interrupt Line               */
#define FDCAN_TTILS_SMCS_Pos      (1U)
#define FDCAN_TTILS_SMCS_Msk      (0x1U << FDCAN_TTILS_SMCS_Pos)               /*!< 0x00000002 */
#define FDCAN_TTILS_SMCS          FDCAN_TTILS_SMCS_Msk                         /*!<Start of Matrix Cycle Interrupt Line              */
#define FDCAN_TTILS_CSMS_Pos      (2U)
#define FDCAN_TTILS_CSMS_Msk      (0x1U << FDCAN_TTILS_CSMS_Pos)               /*!< 0x00000004 */
#define FDCAN_TTILS_CSMS          FDCAN_TTILS_CSMS_Msk                         /*!<Change of Synchronization Mode Interrupt Line     */
#define FDCAN_TTILS_SOGS_Pos      (3U)
#define FDCAN_TTILS_SOGS_Msk      (0x1U << FDCAN_TTILS_SOGS_Pos)               /*!< 0x00000008 */
#define FDCAN_TTILS_SOGS          FDCAN_TTILS_SOGS_Msk                         /*!<Start of Gap Interrupt Line                       */
#define FDCAN_TTILS_RTMIS_Pos     (4U)
#define FDCAN_TTILS_RTMIS_Msk     (0x1U << FDCAN_TTILS_RTMIS_Pos)              /*!< 0x00000010 */
#define FDCAN_TTILS_RTMIS         FDCAN_TTILS_RTMIS_Msk                        /*!<Register Time Mark Interrupt Interrupt Line       */
#define FDCAN_TTILS_TTMIS_Pos     (5U)
#define FDCAN_TTILS_TTMIS_Msk     (0x1U << FDCAN_TTILS_TTMIS_Pos)              /*!< 0x00000020 */
#define FDCAN_TTILS_TTMIS         FDCAN_TTILS_TTMIS_Msk                        /*!<Trigger Time Mark Event Internal Interrupt Line   */
#define FDCAN_TTILS_SWES_Pos      (6U)
#define FDCAN_TTILS_SWES_Msk      (0x1U << FDCAN_TTILS_SWES_Pos)               /*!< 0x00000040 */
#define FDCAN_TTILS_SWES          FDCAN_TTILS_SWES_Msk                         /*!<Stop Watch Event Interrupt Line                   */
#define FDCAN_TTILS_GTWS_Pos      (7U)
#define FDCAN_TTILS_GTWS_Msk      (0x1U << FDCAN_TTILS_GTWS_Pos)               /*!< 0x00000080 */
#define FDCAN_TTILS_GTWS          FDCAN_TTILS_GTWS_Msk                         /*!<Global Time Wrap Interrupt Line                   */
#define FDCAN_TTILS_GTDS_Pos      (8U)
#define FDCAN_TTILS_GTDS_Msk      (0x1U << FDCAN_TTILS_GTDS_Pos)               /*!< 0x00000100 */
#define FDCAN_TTILS_GTDS          FDCAN_TTILS_GTDS_Msk                         /*!<Global Time Discontinuity Interrupt Line          */
#define FDCAN_TTILS_GTES_Pos      (9U)
#define FDCAN_TTILS_GTES_Msk      (0x1U << FDCAN_TTILS_GTES_Pos)               /*!< 0x00000200 */
#define FDCAN_TTILS_GTES          FDCAN_TTILS_GTES_Msk                         /*!<Global Time Error Interrupt Line                  */
#define FDCAN_TTILS_TXUS_Pos      (10U)
#define FDCAN_TTILS_TXUS_Msk      (0x1U << FDCAN_TTILS_TXUS_Pos)               /*!< 0x00000400 */
#define FDCAN_TTILS_TXUS          FDCAN_TTILS_TXUS_Msk                         /*!<Tx Count Underflow Interrupt Line                 */
#define FDCAN_TTILS_TXOS_Pos      (11U)
#define FDCAN_TTILS_TXOS_Msk      (0x1U << FDCAN_TTILS_TXOS_Pos)               /*!< 0x00000800 */
#define FDCAN_TTILS_TXOS          FDCAN_TTILS_TXOS_Msk                         /*!<Tx Count Overflow Interrupt Line                  */
#define FDCAN_TTILS_SE1S_Pos      (12U)
#define FDCAN_TTILS_SE1S_Msk      (0x1U << FDCAN_TTILS_SE1S_Pos)               /*!< 0x00001000 */
#define FDCAN_TTILS_SE1S          FDCAN_TTILS_SE1S_Msk                         /*!<Scheduling Error 1 Interrupt Line                 */
#define FDCAN_TTILS_SE2S_Pos      (13U)
#define FDCAN_TTILS_SE2S_Msk      (0x1U << FDCAN_TTILS_SE2S_Pos)               /*!< 0x00002000 */
#define FDCAN_TTILS_SE2S          FDCAN_TTILS_SE2S_Msk                         /*!<Scheduling Error 2 Interrupt Line                 */
#define FDCAN_TTILS_ELCS_Pos      (14U)
#define FDCAN_TTILS_ELCS_Msk      (0x1U << FDCAN_TTILS_ELCS_Pos)               /*!< 0x00004000 */
#define FDCAN_TTILS_ELCS          FDCAN_TTILS_ELCS_Msk                         /*!<Error Level Changed Interrupt Line                */
#define FDCAN_TTILS_IWTS_Pos      (15U)
#define FDCAN_TTILS_IWTS_Msk      (0x1U << FDCAN_TTILS_IWTS_Pos)               /*!< 0x00008000 */
#define FDCAN_TTILS_IWTS          FDCAN_TTILS_IWTS_Msk                         /*!<Initialization Watch Trigger Interrupt Line       */
#define FDCAN_TTILS_WTS_Pos       (16U)
#define FDCAN_TTILS_WTS_Msk       (0x1U << FDCAN_TTILS_WTS_Pos)                /*!< 0x00010000 */
#define FDCAN_TTILS_WTS           FDCAN_TTILS_WTS_Msk                          /*!<Watch Trigger Interrupt Line                      */
#define FDCAN_TTILS_AWS_Pos       (17U)
#define FDCAN_TTILS_AWS_Msk       (0x1U << FDCAN_TTILS_AWS_Pos)                /*!< 0x00020000 */
#define FDCAN_TTILS_AWS           FDCAN_TTILS_AWS_Msk                          /*!<Application Watchdog Interrupt Line               */
#define FDCAN_TTILS_CERS_Pos      (18U)
#define FDCAN_TTILS_CERS_Msk      (0x1U << FDCAN_TTILS_CERS_Pos)               /*!< 0x00040000 */
#define FDCAN_TTILS_CERS          FDCAN_TTILS_CERS_Msk                         /*!<Configuration Error Interrupt Line                */

/*****************  Bit definition for FDCAN_TTOST register  ********************/
#define FDCAN_TTOST_EL_Pos        (0U)
#define FDCAN_TTOST_EL_Msk        (0x3U << FDCAN_TTOST_EL_Pos)                 /*!< 0x00000003 */
#define FDCAN_TTOST_EL            FDCAN_TTOST_EL_Msk                           /*!<Error Level                              */
#define FDCAN_TTOST_MS_Pos        (2U)
#define FDCAN_TTOST_MS_Msk        (0x3U << FDCAN_TTOST_MS_Pos)                 /*!< 0x0000000C */
#define FDCAN_TTOST_MS            FDCAN_TTOST_MS_Msk                           /*!<Master State                             */
#define FDCAN_TTOST_SYS_Pos       (4U)
#define FDCAN_TTOST_SYS_Msk       (0x3U << FDCAN_TTOST_SYS_Pos)                /*!< 0x00000030 */
#define FDCAN_TTOST_SYS           FDCAN_TTOST_SYS_Msk                          /*!<Synchronization State                    */
#define FDCAN_TTOST_QGTP_Pos      (6U)
#define FDCAN_TTOST_QGTP_Msk      (0x1U << FDCAN_TTOST_QGTP_Pos)               /*!< 0x00000040 */
#define FDCAN_TTOST_QGTP          FDCAN_TTOST_QGTP_Msk                         /*!<Quality of Global Time Phase             */
#define FDCAN_TTOST_QCS_Pos       (7U)
#define FDCAN_TTOST_QCS_Msk       (0x1U << FDCAN_TTOST_QCS_Pos)                /*!< 0x00000080 */
#define FDCAN_TTOST_QCS           FDCAN_TTOST_QCS_Msk                          /*!<Quality of Clock Speed                   */
#define FDCAN_TTOST_RTO_Pos       (8U)
#define FDCAN_TTOST_RTO_Msk       (0xFFU << FDCAN_TTOST_RTO_Pos)               /*!< 0x0000FF00 */
#define FDCAN_TTOST_RTO           FDCAN_TTOST_RTO_Msk                          /*!<Reference Trigger Offset                 */
#define FDCAN_TTOST_WGTD_Pos      (22U)
#define FDCAN_TTOST_WGTD_Msk      (0x1U << FDCAN_TTOST_WGTD_Pos)               /*!< 0x00400000 */
#define FDCAN_TTOST_WGTD          FDCAN_TTOST_WGTD_Msk                         /*!<Wait for Global Time Discontinuity       */
#define FDCAN_TTOST_GFI_Pos       (23U)
#define FDCAN_TTOST_GFI_Msk       (0x1U << FDCAN_TTOST_GFI_Pos)                /*!< 0x00800000 */
#define FDCAN_TTOST_GFI           FDCAN_TTOST_GFI_Msk                          /*!<Gap Finished Indicator                   */
#define FDCAN_TTOST_TMP_Pos       (24U)
#define FDCAN_TTOST_TMP_Msk       (0x7U << FDCAN_TTOST_TMP_Pos)                /*!< 0x07000000 */
#define FDCAN_TTOST_TMP           FDCAN_TTOST_TMP_Msk                          /*!<Time Master Priority                     */
#define FDCAN_TTOST_GSI_Pos       (27U)
#define FDCAN_TTOST_GSI_Msk       (0x1U << FDCAN_TTOST_GSI_Pos)                /*!< 0x08000000 */
#define FDCAN_TTOST_GSI           FDCAN_TTOST_GSI_Msk                          /*!<Gap Started Indicator                    */
#define FDCAN_TTOST_WFE_Pos       (28U)
#define FDCAN_TTOST_WFE_Msk       (0x1U << FDCAN_TTOST_WFE_Pos)                /*!< 0x10000000 */
#define FDCAN_TTOST_WFE           FDCAN_TTOST_WFE_Msk                          /*!<Wait for Event                           */
#define FDCAN_TTOST_AWE_Pos       (29U)
#define FDCAN_TTOST_AWE_Msk       (0x1U << FDCAN_TTOST_AWE_Pos)                /*!< 0x20000000 */
#define FDCAN_TTOST_AWE           FDCAN_TTOST_AWE_Msk                          /*!<Application Watchdog Event               */
#define FDCAN_TTOST_WECS_Pos      (30U)
#define FDCAN_TTOST_WECS_Msk      (0x1U << FDCAN_TTOST_WECS_Pos)               /*!< 0x40000000 */
#define FDCAN_TTOST_WECS          FDCAN_TTOST_WECS_Msk                         /*!<Wait for External Clock Synchronization  */
#define FDCAN_TTOST_SPL_Pos       (31U)
#define FDCAN_TTOST_SPL_Msk       (0x1U << FDCAN_TTOST_SPL_Pos)                /*!< 0x80000000 */
#define FDCAN_TTOST_SPL           FDCAN_TTOST_SPL_Msk                          /*!<Schedule Phase Lock                      */

/*****************  Bit definition for FDCAN_TURNA register  ********************/
#define FDCAN_TURNA_NAV_Pos       (0U)
#define FDCAN_TURNA_NAV_Msk       (0x3FFFFU << FDCAN_TURNA_NAV_Pos)            /*!< 0x0003FFFF */
#define FDCAN_TURNA_NAV           FDCAN_TURNA_NAV_Msk                          /*!<Numerator Actual Value                   */

/*****************  Bit definition for FDCAN_TTLGT register  ********************/
#define FDCAN_TTLGT_LT_Pos        (0U)
#define FDCAN_TTLGT_LT_Msk        (0xFFFFU << FDCAN_TTLGT_LT_Pos)              /*!< 0x0000FFFF */
#define FDCAN_TTLGT_LT            FDCAN_TTLGT_LT_Msk                           /*!<Local Time                               */
#define FDCAN_TTLGT_GT_Pos        (16U)
#define FDCAN_TTLGT_GT_Msk        (0xFFFFU << FDCAN_TTLGT_GT_Pos)              /*!< 0xFFFF0000 */
#define FDCAN_TTLGT_GT            FDCAN_TTLGT_GT_Msk                           /*!<Global Time                              */

/*****************  Bit definition for FDCAN_TTCTC register  ********************/
#define FDCAN_TTCTC_CT_Pos        (0U)
#define FDCAN_TTCTC_CT_Msk        (0xFFFFU << FDCAN_TTCTC_CT_Pos)              /*!< 0x0000FFFF */
#define FDCAN_TTCTC_CT            FDCAN_TTCTC_CT_Msk                           /*!<Cycle Time                               */
#define FDCAN_TTCTC_CC_Pos        (16U)
#define FDCAN_TTCTC_CC_Msk        (0x3FU << FDCAN_TTCTC_CC_Pos)                /*!< 0x003F0000 */
#define FDCAN_TTCTC_CC            FDCAN_TTCTC_CC_Msk                           /*!<Cycle Count                              */

/*****************  Bit definition for FDCAN_TTCPT register  ********************/
#define FDCAN_TTCPT_CCV_Pos       (0U)
#define FDCAN_TTCPT_CCV_Msk       (0x3FU << FDCAN_TTCPT_CCV_Pos)               /*!< 0x0000003F */
#define FDCAN_TTCPT_CCV           FDCAN_TTCPT_CCV_Msk                          /*!<Cycle Count Value                        */
#define FDCAN_TTCPT_SWV_Pos       (16U)
#define FDCAN_TTCPT_SWV_Msk       (0xFFFFU << FDCAN_TTCPT_SWV_Pos)             /*!< 0xFFFF0000 */
#define FDCAN_TTCPT_SWV           FDCAN_TTCPT_SWV_Msk                          /*!<Stop Watch Value                         */

/*****************  Bit definition for FDCAN_TTCSM register  ********************/
#define FDCAN_TTCSM_CSM_Pos       (0U)
#define FDCAN_TTCSM_CSM_Msk       (0xFFFFU << FDCAN_TTCSM_CSM_Pos)             /*!< 0x0000FFFF */
#define FDCAN_TTCSM_CSM           FDCAN_TTCSM_CSM_Msk                          /*!<Cycle Sync Mark                          */

/*****************  Bit definition for FDCAN_TTTS register  *********************/
#define FDCAN_TTTS_SWTSEL_Pos     (0U)
#define FDCAN_TTTS_SWTSEL_Msk     (0x3U << FDCAN_TTTS_SWTSEL_Pos)              /*!< 0x00000003 */
#define FDCAN_TTTS_SWTSEL         FDCAN_TTTS_SWTSEL_Msk                        /*!<Stop watch trigger input selection       */
#define FDCAN_TTTS_EVTSEL_Pos     (4U)
#define FDCAN_TTTS_EVTSEL_Msk     (0x3U << FDCAN_TTTS_EVTSEL_Pos)              /*!< 0x00000030 */
#define FDCAN_TTTS_EVTSEL         FDCAN_TTTS_EVTSEL_Msk                        /*!<Event trigger input selection            */

/********************************************************************************/
/*                                                                              */
/*                      FDCANCCU (Clock Calibration unit)                       */
/*                                                                              */
/********************************************************************************/

/*****************  Bit definition for FDCANCCU_CREL register  ******************/
#define FDCANCCU_CREL_DAY_Pos        (0U)
#define FDCANCCU_CREL_DAY_Msk        (0xFFU << FDCANCCU_CREL_DAY_Pos)          /*!< 0x000000FF */
#define FDCANCCU_CREL_DAY            FDCANCCU_CREL_DAY_Msk                     /*!<Timestamp Day                           */
#define FDCANCCU_CREL_MON_Pos        (8U)
#define FDCANCCU_CREL_MON_Msk        (0xFFU << FDCANCCU_CREL_MON_Pos)          /*!< 0x0000FF00 */
#define FDCANCCU_CREL_MON            FDCANCCU_CREL_MON_Msk                     /*!<Timestamp Month                         */
#define FDCANCCU_CREL_YEAR_Pos       (16U)
#define FDCANCCU_CREL_YEAR_Msk       (0xFU << FDCANCCU_CREL_YEAR_Pos)          /*!< 0x000F0000 */
#define FDCANCCU_CREL_YEAR           FDCANCCU_CREL_YEAR_Msk                    /*!<Timestamp Year                          */
#define FDCANCCU_CREL_SUBSTEP_Pos    (20U)
#define FDCANCCU_CREL_SUBSTEP_Msk    (0xFU << FDCANCCU_CREL_SUBSTEP_Pos)       /*!< 0x00F00000 */
#define FDCANCCU_CREL_SUBSTEP        FDCANCCU_CREL_SUBSTEP_Msk                 /*!<Sub-step of Core release                */
#define FDCANCCU_CREL_STEP_Pos       (24U)
#define FDCANCCU_CREL_STEP_Msk       (0xFU << FDCANCCU_CREL_STEP_Pos)          /*!< 0x0F000000 */
#define FDCANCCU_CREL_STEP           FDCANCCU_CREL_STEP_Msk                    /*!<Step of Core release                    */
#define FDCANCCU_CREL_REL_Pos        (28U)
#define FDCANCCU_CREL_REL_Msk        (0xFU << FDCANCCU_CREL_REL_Pos)           /*!< 0xF0000000 */
#define FDCANCCU_CREL_REL            FDCANCCU_CREL_REL_Msk                     /*!<Core release                            */

/*****************  Bit definition for FDCANCCU_CCFG register  ******************/
#define FDCANCCU_CCFG_TQBT_Pos       (0U)
#define FDCANCCU_CCFG_TQBT_Msk       (0x1FU << FDCANCCU_CCFG_TQBT_Pos)         /*!< 0x0000001F */
#define FDCANCCU_CCFG_TQBT           FDCANCCU_CCFG_TQBT_Msk                    /*!<Time Quanta per Bit Time                */
#define FDCANCCU_CCFG_BCC_Pos        (6U)
#define FDCANCCU_CCFG_BCC_Msk        (0x1U << FDCANCCU_CCFG_BCC_Pos)           /*!< 0x00000040 */
#define FDCANCCU_CCFG_BCC            FDCANCCU_CCFG_BCC_Msk                     /*!<Bypass Clock Calibration                */
#define FDCANCCU_CCFG_CFL_Pos        (7U)
#define FDCANCCU_CCFG_CFL_Msk        (0x1U << FDCANCCU_CCFG_CFL_Pos)           /*!< 0x00000080 */
#define FDCANCCU_CCFG_CFL            FDCANCCU_CCFG_CFL_Msk                     /*!<Calibration Field Length                */
#define FDCANCCU_CCFG_OCPM_Pos       (8U)
#define FDCANCCU_CCFG_OCPM_Msk       (0xFFU << FDCANCCU_CCFG_OCPM_Pos)         /*!< 0x0000FF00 */
#define FDCANCCU_CCFG_OCPM           FDCANCCU_CCFG_OCPM_Msk                    /*!<Oscillator Clock Periods Minimum        */
#define FDCANCCU_CCFG_CDIV_Pos       (16U)
#define FDCANCCU_CCFG_CDIV_Msk       (0xFU << FDCANCCU_CCFG_CDIV_Pos)          /*!< 0x000F0000 */
#define FDCANCCU_CCFG_CDIV           FDCANCCU_CCFG_CDIV_Msk                    /*!<Clock Divider                           */
#define FDCANCCU_CCFG_SWR_Pos        (31U)
#define FDCANCCU_CCFG_SWR_Msk        (0x1U << FDCANCCU_CCFG_SWR_Pos)           /*!< 0x80000000 */
#define FDCANCCU_CCFG_SWR            FDCANCCU_CCFG_SWR_Msk                     /*!<Software Reset                          */

/*****************  Bit definition for FDCANCCU_CSTAT register  *****************/
#define FDCANCCU_CSTAT_OCPC_Pos      (0U)
#define FDCANCCU_CSTAT_OCPC_Msk      (0x3FFFFU << FDCANCCU_CSTAT_OCPC_Pos)     /*!< 0x0003FFFF */
#define FDCANCCU_CSTAT_OCPC          FDCANCCU_CSTAT_OCPC_Msk                   /*!<Oscillator Clock Period Counter        */
#define FDCANCCU_CSTAT_TQC_Pos       (18U)
#define FDCANCCU_CSTAT_TQC_Msk       (0x7FFU << FDCANCCU_CSTAT_TQC_Pos)        /*!< 0x1FFC0000 */
#define FDCANCCU_CSTAT_TQC           FDCANCCU_CSTAT_TQC_Msk                    /*!<Time Quanta Counter                    */
#define FDCANCCU_CSTAT_CALS_Pos      (30U)
#define FDCANCCU_CSTAT_CALS_Msk      (0x3U << FDCANCCU_CSTAT_CALS_Pos)         /*!< 0xC0000000 */
#define FDCANCCU_CSTAT_CALS          FDCANCCU_CSTAT_CALS_Msk                   /*!<Calibration State                      */

/******************  Bit definition for FDCANCCU_CWD register  ******************/
#define FDCANCCU_CWD_WDC_Pos         (0U)
#define FDCANCCU_CWD_WDC_Msk         (0xFFFFU << FDCANCCU_CWD_WDC_Pos)         /*!< 0x0000FFFF */
#define FDCANCCU_CWD_WDC             FDCANCCU_CWD_WDC_Msk                      /*!<Watchdog Configuration                 */
#define FDCANCCU_CWD_WDV_Pos         (16U)
#define FDCANCCU_CWD_WDV_Msk         (0xFFFFU << FDCANCCU_CWD_WDV_Pos)         /*!< 0xFFFF0000 */
#define FDCANCCU_CWD_WDV             FDCANCCU_CWD_WDV_Msk                      /*!<Watchdog Value                         */

/******************  Bit definition for FDCANCCU_IR register  *******************/
#define FDCANCCU_IR_CWE_Pos          (0U)
#define FDCANCCU_IR_CWE_Msk          (0x1U << FDCANCCU_IR_CWE_Pos)             /*!< 0x00000001 */
#define FDCANCCU_IR_CWE              FDCANCCU_IR_CWE_Msk                       /*!<Calibration Watchdog Event             */
#define FDCANCCU_IR_CSC_Pos          (1U)
#define FDCANCCU_IR_CSC_Msk          (0x1U << FDCANCCU_IR_CSC_Pos)             /*!< 0x00000002 */
#define FDCANCCU_IR_CSC              FDCANCCU_IR_CSC_Msk                       /*!<Calibration State Changed              */

/******************  Bit definition for FDCANCCU_IE register  *******************/
#define FDCANCCU_IE_CWEE_Pos         (0U)
#define FDCANCCU_IE_CWEE_Msk         (0x1U << FDCANCCU_IE_CWEE_Pos)            /*!< 0x00000001 */
#define FDCANCCU_IE_CWEE             FDCANCCU_IE_CWEE_Msk                      /*!<Calibration Watchdog Event Enable      */
#define FDCANCCU_IE_CSCE_Pos         (1U)
#define FDCANCCU_IE_CSCE_Msk         (0x1U << FDCANCCU_IE_CSCE_Pos)            /*!< 0x00000002 */
#define FDCANCCU_IE_CSCE             FDCANCCU_IE_CSCE_Msk                      /*!<Calibration State Changed Enable       */



//#define RCC_D2CCIP1R_FDCANSEL_Pos              (28U)
//#define RCC_D2CCIP1R_FDCANSEL_Msk              (0x3U << RCC_D2CCIP1R_FDCANSEL_Pos) /*!< 0x30000000 */
//#define RCC_D2CCIP1R_FDCANSEL                  RCC_D2CCIP1R_FDCANSEL_Msk
//#define RCC_D2CCIP1R_FDCANSEL_0                (0x1U << RCC_D2CCIP1R_FDCANSEL_Pos) /*!< 0x10000000 */
//#define RCC_D2CCIP1R_FDCANSEL_1                (0x2U << RCC_D2CCIP1R_FDCANSEL_Pos) /*!< 0x20000000 */
//
//#define RCC_APB1HENR_FDCANEN_Pos               (8U)
//#define RCC_APB1HENR_FDCANEN_Msk               (0x1U << RCC_APB1HENR_FDCANEN_Pos) /*!< 0x00000100 */
//#define RCC_APB1HENR_FDCANEN                   RCC_APB1HENR_FDCANEN_Msk
//
//
//#define RCC_APB1HRSTR_FDCANRST_Pos             (8U)
//#define RCC_APB1HRSTR_FDCANRST_Msk             (0x1U << RCC_APB1HRSTR_FDCANRST_Pos) /*!< 0x00000100 */
//#define RCC_APB1HRSTR_FDCANRST                 RCC_APB1HRSTR_FDCANRST_Msk
//
///********************  Bit definition for RCC_APB1HLPENR register  ******************/
//#define RCC_APB1HLPENR_FDCANLPEN_Pos           (8U)
//#define RCC_APB1HLPENR_FDCANLPEN_Msk           (0x1U << RCC_APB1HLPENR_FDCANLPEN_Pos) /*!< 0x00000100 */
//#define RCC_APB1HLPENR_FDCANLPEN               RCC_APB1HLPENR_FDCANLPEN_Msk
//
//
//
///********************  Bit definition for APB1HFZ1 register  ************/
//#define DBGMCU_APB1HFZ1_DBG_FDCAN_Pos     (8U)
//#define DBGMCU_APB1HFZ1_DBG_FDCAN_Msk     (0x1U << DBGMCU_APB1HFZ1_DBG_FDCAN_Pos) /*!< 0x00000100 */
//#define DBGMCU_APB1HFZ1_DBG_FDCAN         DBGMCU_APB1HFZ1_DBG_FDCAN_Msk

/******************************* FDCAN Instances ******************************/
#define IS_FDCAN_ALL_INSTANCE(__INSTANCE__) (((__INSTANCE__) == FDCAN1) || \
                                             ((__INSTANCE__) == FDCAN2))

#define IS_FDCAN_TT_INSTANCE(__INSTANCE__) ((__INSTANCE__) == FDCAN1)

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32H7xx_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
