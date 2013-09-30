/************************************************************************************
 * configs/sa-1xx/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif
#include "stm32_rcc.h"
#include "stm32_sdio.h"
#include "stm32.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* The SA-1xx board features a external HSE 8MHz crystal.
 *
 * This is the canonical configuration:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 168000000    Determined by PLL configuration
 *   HCLK(Hz)                      : 168000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 8000000      (STM32_BOARD_XTAL)
 *   PLLM                          : 8            (STM32_PLLCFG_PLLM)
 *   PLLN                          : 336          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 7            (STM32_PLLCFG_PPQ)
 *   Main regulator output voltage : Scale1 mode  Needed for high speed SYSCLK
 *   Flash Latency(WS)             : 5
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Enabled
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 8MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSI
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (8,000,000 / 8) * 336
 *         = 336,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 336,000,000 / 2 = 168,000,000
 * USB OTG FS, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 48,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(336)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(7)

#define STM32_SYSCLK_FREQUENCY  168000000ul

/* AHB clock (HCLK) is SYSCLK (168MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (42MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

#ifdef CONFIG_SA1XX_ENABLE_SS
/* SSCGR (spread spectrum)
 *   fvco_out :  336 [MHz]
 *   fmod     : 1000 [Hz]
 *   md       :  1.5 [%]
 */
#define GUIDEPHONE_SSCGR_MODPER  250
#define GUIDEPHONE_SSCGR_INCSTEP 132
#endif

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (84MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define STM32_TIM18_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_TIM27_FREQUENCY   STM32_HCLK_FREQUENCY

/* SDIO dividers.  Note that slower clocking is required when DMA is disabled 
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(118+2)=400 KHz
 */

#define SDIO_INIT_CLKDIV        (118 << SDIO_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV    (1 << SDIO_CLKCR_CLKDIV_SHIFT) 
#else
#  define SDIO_MMCXFR_CLKDIV    (2 << SDIO_CLKCR_CLKDIV_SHIFT) 
#endif

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV     (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_SDXFR_CLKDIV     (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* Ethernet *************************************************************************/
/* We need to provide clocking to the MII PHY via MCO1 (PA8) */

#if defined(CONFIG_NET) && defined(CONFIG_STM32_ETHMAC)

#  if !defined(CONFIG_STM32_MII)
#    warning "CONFIG_STM32_MII required for Ethernet"
#  endif
#endif

/* LED definitions ******************************************************************/
/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any
 * way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with stm32_setled() */

#define BOARD_LED_G0      0
#define BOARD_LED_G1      1
#define BOARD_LED_G2      2
#define BOARD_LED_R0      3
#define BOARD_LED_R1      4
#define BOARD_LED_R2      5

#define BOARD_NLEDS       6

/* LED bits for use with stm32_setleds() */

#define BOARD_LED_G0_BIT    (1 << BOARD_LED_G0)
#define BOARD_LED_G1_BIT    (1 << BOARD_LED_G1)
#define BOARD_LED_G2_BIT    (1 << BOARD_LED_G2)
#define BOARD_LED_R0_BIT    (1 << BOARD_LED_R0)
#define BOARD_LED_R1_BIT    (1 << BOARD_LED_R1)
#define BOARD_LED_R2_BIT    (1 << BOARD_LED_R2)

/* Button definitions ***************************************************************/
/* The SA-1xx supports one button: */

#define BUTTON_INIT        0

#define NUM_BUTTONS        1

#define BUTTON_INIT_BIT  (1 << BUTTON_INIT)

/* DIPSW definitions ***************************************************************/
/* The SA-1xx supports one dipsw (4bit): */

#define DIPSW_INIT        0

#define NUM_DIPSW         4

/* DOUT/Relay definitions ***********************************************************/

#define NUM_RELAYS        2


/* Alternate function pin selections ************************************************/

/* UART1: console
 *
 */

#define GPIO_USART1_RX GPIO_USART1_RX_2	/* PB7 */
#define GPIO_USART1_TX GPIO_USART1_TX_2	/* PB6 */

/* UART2: XIO (XBee)
 *
 */

#define GPIO_USART2_RX  GPIO_USART2_RX_2    /* PD6 */
#define GPIO_USART2_TX  GPIO_USART2_TX_2    /* PD5 */
#define GPIO_USART2_CTS GPIO_USART2_CTS_2   /* PD3 */
#define GPIO_USART2_RTS GPIO_USART2_RTS_2   /* PD4 */


/* UART3: RS-232
 *
 */

#define GPIO_USART3_RX         GPIO_USART3_RX_3	/* PD9  */
#define GPIO_USART3_TX         GPIO_USART3_TX_3	/* PD8  */
#define GPIO_USART3_CTS        GPIO_USART3_CTS_2	/* PD11 */
#define GPIO_USART3_RTS        GPIO_USART3_RTS_2	/* PD12 */
#define GPIO_USART3_DTR        (GPIO_OUTPUT|GPIO_FLOAT|GPIO_PORTF|GPIO_PIN6)
#define GPIO_USART3_DSR        (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTF|GPIO_PIN7)
#define GPIO_USART3_CD         (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTF|GPIO_PIN8)
#define GPIO_USART3_RI         (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTF|GPIO_PIN9)
#define GPIO_USART3_RI_B       (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTF|GPIO_PIN10)
#define GPIO_USART3_FORCEON    (GPIO_OUTPUT|GPIO_FLOAT|GPIO_PORTF|GPIO_PIN4)
#define GPIO_USART3_FORCEOFF_N (GPIO_OUTPUT|GPIO_FLOAT|GPIO_PORTF|GPIO_PIN5)

/* UART6: RS-485
 *
 */

#define GPIO_USART6_RX GPIO_USART6_RX_2	/* PG9  */
#define GPIO_USART6_TX GPIO_USART6_TX_2	/* PG14 */
#define GPIO_USART6_RS485_RXEN (GPIO_OUTPUT|GPIO_FLOAT|GPIO_PORTG|GPIO_PIN10)
#define GPIO_USART6_RS485_TXEN (GPIO_OUTPUT|GPIO_FLOAT|GPIO_PORTG|GPIO_PIN11)

/* SPI */

#define GPIO_SPI2_NSS  (GPIO_OUTPUT|GPIO_FLOAT|GPIO_PORTI|GPIO_PIN0)
#define GPIO_SPI2_SCK  GPIO_SPI2_SCK_3   /* PI1 */
#define GPIO_SPI2_MISO GPIO_SPI2_MISO_3  /* PI2 */
#define GPIO_SPI2_MOSI GPIO_SPI2_MOSI_3  /* PI3 */

/* Ethernet:
 *
 * - PA2  is ETH_MDIO
 * - PC1  is ETH_MDC
 * - PB5  is ETH_PPS_OUT (not used)
 * - PH2  is ETH_MII_CRS
 * - PH3  is ETH_MII_COL
 * - PB10 is ETH_MII_RX_ER
 * - PB0  is ETH_MII_RXD2
 * - PB1  is ETH_MII_RXD3
 * - PC3  is ETH_MII_TX_CLK
 * - PC2  is ETH_MII_TXD2
 * - PE2  is ETH_MII_TXD3
 * - PA1  is ETH_MII_RX_CLK/ETH_RMII_REF_CLK
 * - PA7  is ETH_MII_RX_DV/ETH_RMII_CRS_DV
 * - PC4  is ETH_MII_RXD0/ETH_RMII_RXD0
 * - PC5  is ETH_MII_RXD1/ETH_RMII_RXD1
 * - PB11 is ETH_MII_TX_EN/ETH_RMII_TX_EN
 * - PB12 is ETH_MII_TXD0/ETH_RMII_TXD0
 * - PB13 is ETH_MII_TXD1/ETH_RMII_TXD1
 * [PHY]
 * - PH4  is ETH_RESET_N (PHY Reset)
 * - PH5  is ETH_INTRP_N (PHY interrupt request)
 * - PH6  is ETH_OSCENB  (PHY 25MHz OSC Enable)
 */

#define GPIO_ETH_PPS_OUT     GPIO_ETH_PPS_OUT_1
#define GPIO_ETH_MII_CRS     GPIO_ETH_MII_CRS_2   /* GPIO_PORTH|GPIO_PIN2  */
#define GPIO_ETH_MII_COL     GPIO_ETH_MII_COL_2   /* GPIO_PORTH|GPIO_PIN3  */
#define GPIO_ETH_MII_RX_ER   GPIO_ETH_MII_RX_ER_1 /* GPIO_PORTB|GPIO_PIN10 */
#define GPIO_ETH_MII_RXD2    GPIO_ETH_MII_RXD2_1  /* GPIO_PORTB|GPIO_PIN0  */
#define GPIO_ETH_MII_RXD3    GPIO_ETH_MII_RXD3_1  /* GPIO_PORTB|GPIO_PIN1  */
#define GPIO_ETH_MII_TXD3    GPIO_ETH_MII_TXD3_2  /* GPIO_PORTE|GPIO_PIN2  */
#define GPIO_ETH_MII_TX_EN   GPIO_ETH_MII_TX_EN_1 /* GPIO_PORTB|GPIO_PIN11 */
#define GPIO_ETH_MII_TXD0    GPIO_ETH_MII_TXD0_1  /* GPIO_PORTB|GPIO_PIN12 */
#define GPIO_ETH_MII_TXD1    GPIO_ETH_MII_TXD1_1  /* GPIO_PORTB|GPIO_PIN13 */
#define GPIO_ETH_PHY_RESET_N (GPIO_OUTPUT|GPIO_FLOAT|GPIO_PORTH|GPIO_PIN4)
#define GPIO_ETH_PHY_INTRP_N (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTH|GPIO_PIN5)
#define GPIO_ETH_PHY_OSCENB  (GPIO_OUTPUT|GPIO_FLOAT|GPIO_PORTH|GPIO_PIN6)

/* Timer Inputs/Outputs (see the README.txt file for options) */

#define GPIO_TIM2_CH1IN  GPIO_TIM2_CH1IN_2
#define GPIO_TIM2_CH2IN  GPIO_TIM2_CH2IN_1

#define GPIO_TIM8_CH1IN  GPIO_TIM8_CH1IN_1
#define GPIO_TIM8_CH2IN  GPIO_TIM8_CH2IN_1

/* CAN
 *
 * Mapping to STM32 GPIO pins:
 *
 *   PD0   = FSMC_D2 & CAN1_RX
 *   PD1   = FSMC_D3 & CAN1_TX
 */

#define GPIO_CAN1_RX        GPIO_CAN1_RX_3
#define GPIO_CAN1_TX        GPIO_CAN1_TX_3
#define GPIO_CAN1_STBY      (GPIO_OUTPUT|GPIO_FLOAT|GPIO_PORTG|GPIO_PIN13)

/* I2C.  Only I2C2 is available on the SA-1xx.  I2C2_SCL and I2C2_SDA are
 * available on the following pins:
 *
 * - PF1  is I2C2_SCL
 * - PF0  is I2C2_SDA
 */

#define GPIO_I2C2_SCL       GPIO_I2C2_SCL_2
#define GPIO_I2C2_SDA       GPIO_I2C2_SDA_2

/* SDIO */

#define GPIO_SDIO_CD        (GPIO_INPUT| GPIO_FLOAT|GPIO_PORTG|GPIO_PIN6)
#define GPIO_SDIO_WP        (GPIO_INPUT| GPIO_FLOAT|GPIO_PORTG|GPIO_PIN7)
#define GPIO_SDIO_VDD_ENB   (GPIO_OUTPUT|GPIO_FLOAT|GPIO_PORTG|GPIO_PIN8)

/* DIN */

#define GPIO_DIO_IN0        (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTI|GPIO_PIN4)
#define GPIO_DIO_IN1        (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTI|GPIO_PIN5)
#define GPIO_DIO_IN2        (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTI|GPIO_PIN6)
#define GPIO_DIO_IN3        (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTI|GPIO_PIN7)

/* DOUT */

#define GPIO_DIO_OUT0       (GPIO_OUTPUT|GPIO_FLOAT|GPIO_PORTE|GPIO_PIN3)
#define GPIO_DIO_OUT1       (GPIO_OUTPUT|GPIO_FLOAT|GPIO_PORTE|GPIO_PIN4)
#define GPIO_DIO_OUT2       (GPIO_OUTPUT|GPIO_FLOAT|GPIO_PORTE|GPIO_PIN5)
#define GPIO_DIO_OUT3       (GPIO_OUTPUT|GPIO_FLOAT|GPIO_PORTE|GPIO_PIN6)

/* DMA Channl/Stream Selections *****************************************************/
/* Stream selections are arbitrary for now but might become important in the future
 * is we set aside more DMA channels/streams.
 *
 * SDIO DMA
 *   DMAMAP_SDIO_1 = Channel 4, Stream 3
 *   DMAMAP_SDIO_2 = Channel 4, Stream 6
 */

#define DMAMAP_SDIO DMAMAP_SDIO_1

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

EXTERN void stm32_boardinitialize(void);

/************************************************************************************
 * Name:  stm32_ledinit, stm32_setled, and stm32_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board LEDs.  If
 *   CONFIG_ARCH_LEDS is not defined, then the following interfacesare available to
 *   control the LEDs from user applications.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_LEDS
EXTERN void stm32_ledinit(void);
EXTERN void stm32_setled(int led, bool ledon);
EXTERN void stm32_setleds(uint8_t ledset);
#endif

/************************************************************************************
 * Button support.
 *
 * Description:
 *   up_buttoninit() must be called to initialize button resources.  After
 *   that, up_buttons() may be called to collect the current state of all
 *   buttons or up_irqbutton() may be called to register button interrupt
 *   handlers.
 *
 *   After up_buttoninit() has been called, up_buttons() may be called to
 *   collect the state of all buttons.  up_buttons() returns an 8-bit bit set
 *   with each bit associated with a button.  See the BUTTON_*_BIT
 *   definitions in board.h for the meaning of each bit.
 *
 *   up_irqbutton() may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is a
 *   button enumeration value that uniquely identifies a button resource. See the
 *   BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.  The previous interrupt handler address is returned (so that it may
 *   restored, if so desired).
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
EXTERN void up_buttoninit(void);
EXTERN uint8_t up_buttons(void);
#endif

/************************************************************************************
 * Name: stm32_lm75initialize
 *
 * Description:
 *   Initialize and register the LM-75 Temperature Sensor driver.
 *
 * Input parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_I2C_LM75) && defined(CONFIG_STM32_I2C2)
EXTERN int stm32_lm75initialize(FAR const char *devpath);
#endif

/************************************************************************************
 * Relay control functions
 *
 * Description:
 *   Non-standard functions for relay/DOUT control from the SA-1xx board.
 *
 *   NOTE:  These must match the prototypes in include/nuttx/arch.h
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_RELAYS
EXTERN void up_relaysinit(void);
EXTERN void relays_setstat(int relays, bool stat);
EXTERN bool relays_getstat(int relays);
EXTERN void relays_setstats(uint32_t relays_stat);
EXTERN uint32_t relays_getstats(void);
EXTERN void relays_onoff(int relays, uint32_t mdelay);
EXTERN void relays_onoffs(uint32_t relays_stat, uint32_t mdelay);
EXTERN void relays_resetmode(int relays);
EXTERN void relays_powermode(int relays);
EXTERN void relays_resetmodes(uint32_t relays_stat);
EXTERN void relays_powermodes(uint32_t relays_stat);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_BOARD_BOARD_H */
