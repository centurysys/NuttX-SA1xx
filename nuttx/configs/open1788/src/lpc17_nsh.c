/****************************************************************************
 * config/open1788/src/lpc17_nsh.c
 * arch/arm/src/board/lpc17_nsh.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <unistd.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi.h>
#include <nuttx/mmcsd.h>
#include <nuttx/usb/usbhost.h>

#include "lpc17_gpio.h"
#include "open1788.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define NSH_HAVE_MMCSD    1
#define NSH_HAVE_USBHOST  1
#define NSH_HAVE_USBHDEV  1

/* MMC/SD support */

#if !defined(CONFIG_MMCSD) && !defined(CONFIG_MMCD_SPI)
#  undef NSH_HAVE_MMCSD
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef NSH_HAVE_MMCSD
#endif

/* MMC/SD support requires that an SPI support is enabled and an SPI port is selected */

#ifdef NSH_HAVE_MMCSD
#  if !defined(CONFIG_NSH_MMCSDSPIPORTNO)
#     error "No SSP port number is provided for MMC/SD support"
#     undef NSH_HAVE_MMCSD
#  elif CONFIG_NSH_MMCSDSPIPORTNO == 0 && !defined(CONFIG_LPC17_SSP0)
#     error "SSP port 0 is selected but SSP0 is not enabled"
#     undef NSH_HAVE_MMCSD
#  elif CONFIG_NSH_MMCSDSPIPORTNO == 1 && !defined(CONFIG_LPC17_SSP1)
#     error "SSP port 1 is selected but SSP1 is not enabled"
#     undef NSH_HAVE_MMCSD
#  elif CONFIG_NSH_MMCSDSPIPORTNO == 2 && !defined(CONFIG_LPC17_SSP2)
#     error "SSP port 2 is selected but SSP2 is not enabled"
#     undef NSH_HAVE_MMCSD
#  elif CONFIG_NSH_MMCSDSPIPORTNO > 2
#     error "SSP port number is out of range"
#     undef NSH_HAVE_MMCSD
#  endif
#endif

#ifdef NSH_HAVE_MMCSD
#  if !defined(CONFIG_NSH_MMCSDSLOTNO)
#    warning "Assuming slot MMC/SD slot 0"
#    define CONFIG_NSH_MMCSDSLOTNO 0
#  endif
#endif

#ifdef NSH_HAVE_MMCSD
#  if !defined(CONFIG_NSH_MMCSDSLOTNO)
#    warning "Assuming /dev/mmcsd0"
#    define CONFIG_NSH_MMCSDMINOR 0
#  endif
#endif

/* USB Host */

#ifndef CONFIG_USBHOST
#  undef NSH_HAVE_USBHOST
#endif

#ifndef CONFIG_LPC17_USBHOST
#  undef NSH_HAVE_USBHOST
#endif

#ifdef NSH_HAVE_USBHOST
#  ifndef CONFIG_USBHOST_DEFPRIO
#    define CONFIG_USBHOST_DEFPRIO 50
#  endif
#  ifndef CONFIG_USBHOST_STACKSIZE
#    define CONFIG_USBHOST_STACKSIZE 1024
#  endif
#endif

/* USB Device */

#ifndef CONFIG_USBDEV
#  undef NSH_HAVE_USBDEV
#endif

#ifndef CONFIG_LPC17_USBDEV
#  undef NSH_HAVE_USBDEV
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#  else
#    define message printf
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef NSH_HAVE_USBHOST
static struct usbhost_driver_s *g_drvr;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_waiter
 *
 * Description:
 *   Wait for USB devices to be connected.
 *
 ****************************************************************************/

#ifdef NSH_HAVE_USBHOST
static int nsh_waiter(int argc, char *argv[])
{
  bool connected = false;
  int ret;

  message("nsh_waiter: Running\n");
  for (;;)
    {
      /* Wait for the device to change state */

      ret = DRVR_WAIT(g_drvr, connected);
      DEBUGASSERT(ret == OK);

      connected = !connected;
      message("nsh_waiter: %s\n", connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (connected)
        {
          /* Yes.. enumerate the newly connected device */

          (void)DRVR_ENUMERATE(g_drvr);
        }
    }

  /* Keep the compiler from complaining */

  return 0;
}
#endif

/****************************************************************************
 * Name: nsh_sdinitialize
 *
 * Description:
 *   Initialize SPI-based microSD.
 *
 ****************************************************************************/

#ifdef NSH_HAVE_MMCSD
static int nsh_sdinitialize(void)
{
  FAR struct spi_dev_s *ssp;
  int ret;

  /* Enable power to the SD/MMC via a GPIO. LOW enables SD/MMC. */

  lpc17_gpiowrite(OPEN1788_MMC_PWR, false);

  /* Get the SSP port */

  ssp = up_spiinitialize(CONFIG_NSH_MMCSDSPIPORTNO);
  if (!ssp)
    {
      message("nsh_archinitialize: Failed to initialize SSP port %d\n",
              CONFIG_NSH_MMCSDSPIPORTNO);
      ret = -ENODEV;
      goto errout;
    }

  message("Successfully initialized SSP port %d\n",
          CONFIG_NSH_MMCSDSPIPORTNO);

  /* Bind the SSP port to the slot */

  ret = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR,
                               CONFIG_NSH_MMCSDSLOTNO, ssp);
  if (ret < 0)
    {
      message("nsh_sdinitialize: "
              "Failed to bind SSP port %d to MMC/SD slot %d: %d\n",
              CONFIG_NSH_MMCSDSPIPORTNO,
              CONFIG_NSH_MMCSDSLOTNO, ret);
      goto errout;
    }

  message("Successfuly bound SSP port %d to MMC/SD slot %d\n",
          CONFIG_NSH_MMCSDSPIPORTNO,
          CONFIG_NSH_MMCSDSLOTNO);
  return OK;

  /* Disable power to the SD/MMC via a GPIO. HIGH disables SD/MMC. */

errout:
  lpc17_gpiowrite(OPEN1788_MMC_PWR, true);
  return ret;
}
#else
#  define nsh_sdinitialize() (OK)
#endif

/****************************************************************************
 * Name: nsh_usbhostinitialize
 *
 * Description:
 *   Initialize SPI-based microSD.
 *
 ****************************************************************************/

#ifdef NSH_HAVE_USBHOST
static int nsh_usbhostinitialize(void)
{
  int pid;
  int ret;

  /* First, register all of the class drivers needed to support the drivers
   * that we care about:
   */

  message("nsh_usbhostinitialize: Register class drivers\n");
  ret = usbhost_storageinit();
  if (ret != OK)
    {
      message("nsh_usbhostinitialize: Failed to register the mass storage class\n");
    }

  /* Then get an instance of the USB host interface */

  message("nsh_usbhostinitialize: Initialize USB host\n");
  g_drvr = usbhost_initialize(0);
  if (g_drvr)
    {
      /* Start a thread to handle device connection. */

      message("nsh_usbhostinitialize: Start nsh_waiter\n");

#ifndef CONFIG_CUSTOM_STACK
      pid = task_create("usbhost", CONFIG_USBHOST_DEFPRIO,
                        CONFIG_USBHOST_STACKSIZE,
                        (main_t)nsh_waiter, (FAR char * const *)NULL);
#else
      pid = task_create("usbhost", CONFIG_USBHOST_DEFPRIO,
                        (main_t)nsh_waiter, (FAR char * const *)NULL);
#endif
      return pid < 0 ? -ENOEXEC : OK;
    }
  return -ENODEV;
}
#else
#  define nsh_usbhostinitialize() (OK)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int nsh_archinitialize(void)
{
  int ret;

  /* Initialize SPI-based microSD */

  ret = nsh_sdinitialize();
  if (ret == OK)
    {
      /* Initialize USB host */

      ret = nsh_usbhostinitialize();
    }

  return ret;
}