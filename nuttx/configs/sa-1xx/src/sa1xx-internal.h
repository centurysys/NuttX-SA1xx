/****************************************************************************************************
 * configs/sa-1xx/src/sa1xx-internal.h
 * arch/arm/src/board/sa1xx-internal.n
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __CONFIGS_SA1XX_SRC_SA1XX_INTERNAL_H
#define __CONFIGS_SA1XX_SRC_SA1XX_INTERNAL_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/
/* How many SPI modules does this chip support? */

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 3
#  undef CONFIG_STM32_SPI3
#endif

/* SA-1xx GPIOs **************************************************************************/
/* LEDs */

#define GPIO_LED_B1 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                     GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN12)
#define GPIO_LED_B2 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                     GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13)
#define GPIO_LED_L1 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                     GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)

/* BUTTONS -- NOTE that all have EXTI interrupts configured */

#define MIN_IRQBUTTON   BUTTON_INIT
#define MAX_IRQBUTTON   BUTTON_INIT
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_INIT   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTF|GPIO_PIN11)

/* DIPSW */
#define GPIO_DIPSW0     (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTF|GPIO_PIN12)
#define GPIO_DIPSW1     (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTF|GPIO_PIN13)
#define GPIO_DIPSW2     (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTF|GPIO_PIN14)
#define GPIO_DIPSW3     (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTF|GPIO_PIN15)

/* LM-75 Temperature Sensor: PI11 */

#define GPIO_LM75_OSINT (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTI|GPIO_PIN11)

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SA-1xx board.
 *
 ****************************************************************************************************/

void weak_function stm32_spiinitialize(void);

/****************************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in inialization to setup USB-related GPIO pins for
 *   the SA-1xx board.
 *
 ****************************************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void weak_function stm32_usbinitialize(void);
#endif

/****************************************************************************************************
 * Name: stm32_at24xxinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SA-1xx board.
 *
 ****************************************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_STM32_I2C2)
int stm32_at24xxinitialize(void);

int sa1xx_get_parameter(const char *paramname, char *buf, int size, int convert);
int sa1xx_set_parameter(const char *paramname, char *buf, int size);
void sa1xx_print_paramnames(void);
#endif

#ifdef CONFIG_STM32_SPI2
int sa1xx_spiflash_initialize(void);
#endif

#ifndef info
#define info(fmt, arg...) do {sched_lock(); lowsyslog(fmt, ##arg); sched_unlock();} while (0)
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SA1XX_SRC_SA1XX_INTERNAL_H */
