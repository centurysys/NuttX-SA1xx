/************************************************************************************
 * drivers/mtd/n25qx.c
 * Driver for SPI-based N25x16, x32, and x64 and N25q16, q32, q64, and q128 FLASH
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* Per the data sheet, the N25 parts can be driven with either SPI mode 0 (CPOL=0
 * and CPHA=0) or mode 3 (CPOL=1 and CPHA=1). But I have heard that other devices
 * can operate in mode 0 or 1.  So you may need to specify CONFIG_N25Q_SPIMODE to
 * select the best mode for your device.  If CONFIG_N25Q_SPIMODE is not defined,
 * mode 0 will be used.
 */

#ifndef CONFIG_N25Q_SPIMODE
#  define CONFIG_N25Q_SPIMODE SPIDEV_MODE0
#endif

/* SPI Frequency.  May be up to 42MHz. */

#ifndef CONFIG_N25Q_SPIFREQUENCY
#  define CONFIG_N25Q_SPIFREQUENCY 42000000
#endif

#ifndef CONFIG_N25Q_MANUFACTURER
#  define CONFIG_N25Q_MANUFACTURER 0x20
#endif

/* N25 Instructions *****************************************************************/
/*      Command                    Value      Description                           */
/*                                                                                  */
#define N25Q_WREN                   0x06    /* Write enable                          */
#define N25Q_WRDI                   0x04    /* Write Disable                         */
#define N25Q_RDSR                   0x05    /* Read status register                  */
#define N25Q_WRSR                   0x01    /* Write Status Register                 */
#define N25Q_RDDATA                 0x03    /* Read data bytes                       */
#define N25Q_FRD                    0x0b    /* Higher speed read                     */
#define N25Q_FRDD                   0x3b    /* Fast read, dual output                */
#define N25Q_PP                     0x02    /* Program page                          */
#define N25Q_BE                     0xd8    /* Block Erase (64KB)                    */
#define N25Q_SE                     0x20    /* Sector erase (4KB)                    */
#define N25Q_CE                     0xc7    /* Chip erase                            */
#define N25Q_PD                     0xb9    /* Power down                            */
#define N25Q_PURDID                 0xab    /* Release PD, Device ID                 */
#define N25Q_RDMFID                 0x90    /* Read Manufacturer / Device            */
#define N25Q_JEDEC_ID               0x9f    /* JEDEC ID read                         */

/* N25 Registers ********************************************************************/

/* JEDEC Read ID register values */

#define N25Q_JEDEC_MANUFACTURER    CONFIG_N25Q_MANUFACTURER
#define N25Q_JEDEC_MEMORY_TYPE     0xba  /* N25Q memory type */

#define N25Q_JEDEC_CAPACITY_16MBIT  0x15  /* 512x4096  = 16Mbit memory capacity */
#define N25Q_JEDEC_CAPACITY_32MBIT  0x16  /* 1024x4096 = 32Mbit memory capacity */
#define N25Q_JEDEC_CAPACITY_64MBIT  0x17  /* 2048x4096 = 64Mbit memory capacity */
#define N25Q_JEDEC_CAPACITY_128MBIT 0x18  /* 4096x4096 = 128Mbit memory capacity */

#define NSECTORS_16MBIT            512   /* 512 sectors x 4096 bytes/sector = 2Mb */
#define NSECTORS_32MBIT            1024  /* 1024 sectors x 4096 bytes/sector = 4Mb */
#define NSECTORS_64MBIT            2048  /* 2048 sectors x 4096 bytes/sector = 8Mb */
#define NSECTORS_128MBIT           4096  /* 4096 sectors x 4096 bytes/sector = 16Mb */

/* Status register bit definitions */

#define N25Q_SR_BUSY                (1 << 0)  /* Bit 0: Write in progress */
#define N25Q_SR_WEL                 (1 << 1)  /* Bit 1: Write enable latch bit */
#define N25Q_SR_BP_SHIFT            (2)       /* Bits 2-5: Block protect bits */
#define N25Q_SR_BP_MASK             (15 << N25Q_SR_BP_SHIFT)
#  define N25X16_SR_BP_NONE        (0 << N25Q_SR_BP_SHIFT)  /* Unprotected */
#  define N25X16_SR_BP_UPPER32nd   (1 << N25Q_SR_BP_SHIFT)  /* Upper 32nd */
#  define N25X16_SR_BP_UPPER16th   (2 << N25Q_SR_BP_SHIFT)  /* Upper 16th */
#  define N25X16_SR_BP_UPPER8th    (3 << N25Q_SR_BP_SHIFT)  /* Upper 8th */
#  define N25X16_SR_BP_UPPERQTR    (4 << N25Q_SR_BP_SHIFT)  /* Upper quarter */
#  define N25X16_SR_BP_UPPERHALF   (5 << N25Q_SR_BP_SHIFT)  /* Upper half */
#  define N25X16_SR_BP_ALL         (6 << N25Q_SR_BP_SHIFT)  /* All sectors */
#  define N25X16_SR_BP_LOWER32nd   (9 << N25Q_SR_BP_SHIFT)  /* Lower 32nd */
#  define N25X16_SR_BP_LOWER16th   (10 << N25Q_SR_BP_SHIFT) /* Lower 16th */
#  define N25X16_SR_BP_LOWER8th    (11 << N25Q_SR_BP_SHIFT) /* Lower 8th */
#  define N25X16_SR_BP_LOWERQTR    (12 << N25Q_SR_BP_SHIFT) /* Lower quarter */
#  define N25X16_SR_BP_LOWERHALF   (13 << N25Q_SR_BP_SHIFT) /* Lower half */

#  define N25X32_SR_BP_NONE        (0 << N25Q_SR_BP_SHIFT)  /* Unprotected */
#  define N25X32_SR_BP_UPPER64th   (1 << N25Q_SR_BP_SHIFT)  /* Upper 64th */
#  define N25X32_SR_BP_UPPER32nd   (2 << N25Q_SR_BP_SHIFT)  /* Upper 32nd */
#  define N25X32_SR_BP_UPPER16th   (3 << N25Q_SR_BP_SHIFT)  /* Upper 16th */
#  define N25X32_SR_BP_UPPER8th    (4 << N25Q_SR_BP_SHIFT)  /* Upper 8th */
#  define N25X32_SR_BP_UPPERQTR    (5 << N25Q_SR_BP_SHIFT)  /* Upper quarter */
#  define N25X32_SR_BP_UPPERHALF   (6 << N25Q_SR_BP_SHIFT)  /* Upper half */
#  define N25X32_SR_BP_ALL         (7 << N25Q_SR_BP_SHIFT)  /* All sectors */
#  define N25X32_SR_BP_LOWER64th   (9 << N25Q_SR_BP_SHIFT)  /* Lower 64th */
#  define N25X32_SR_BP_LOWER32nd   (10 << N25Q_SR_BP_SHIFT) /* Lower 32nd */
#  define N25X32_SR_BP_LOWER16th   (11 << N25Q_SR_BP_SHIFT) /* Lower 16th */
#  define N25X32_SR_BP_LOWER8th    (12 << N25Q_SR_BP_SHIFT) /* Lower 8th */
#  define N25X32_SR_BP_LOWERQTR    (13 << N25Q_SR_BP_SHIFT) /* Lower quarter */
#  define N25X32_SR_BP_LOWERHALF   (14 << N25Q_SR_BP_SHIFT) /* Lower half */

#  define N25X64_SR_BP_NONE        (0 << N25Q_SR_BP_SHIFT)  /* Unprotected */
#  define N25X64_SR_BP_UPPER64th   (1 << N25Q_SR_BP_SHIFT)  /* Upper 64th */
#  define N25X64_SR_BP_UPPER32nd   (2 << N25Q_SR_BP_SHIFT)  /* Upper 32nd */
#  define N25X64_SR_BP_UPPER16th   (3 << N25Q_SR_BP_SHIFT)  /* Upper 16th */
#  define N25X64_SR_BP_UPPER8th    (4 << N25Q_SR_BP_SHIFT)  /* Upper 8th */
#  define N25X64_SR_BP_UPPERQTR    (5 << N25Q_SR_BP_SHIFT)  /* Upper quarter */
#  define N25X64_SR_BP_UPPERHALF   (6 << N25Q_SR_BP_SHIFT)  /* Upper half */
#  define N25X46_SR_BP_ALL         (7 << N25Q_SR_BP_SHIFT)  /* All sectors */
#  define N25X64_SR_BP_LOWER64th   (9 << N25Q_SR_BP_SHIFT)  /* Lower 64th */
#  define N25X64_SR_BP_LOWER32nd   (10 << N25Q_SR_BP_SHIFT) /* Lower 32nd */
#  define N25X64_SR_BP_LOWER16th   (11 << N25Q_SR_BP_SHIFT) /* Lower 16th */
#  define N25X64_SR_BP_LOWER8th    (12 << N25Q_SR_BP_SHIFT) /* Lower 8th */
#  define N25X64_SR_BP_LOWERQTR    (13 << N25Q_SR_BP_SHIFT) /* Lower quarter */
#  define N25X64_SR_BP_LOWERHALF   (14 << N25Q_SR_BP_SHIFT) /* Lower half */
                                             /* Bit 6: Reserved */
#define N25Q_SR_SRP                 (1 << 7)  /* Bit 7: Status register write protect */

#define N25Q_DUMMY                  0xa5

/* Chip Geometries ******************************************************************/
/* All members of the family support uniform 4K-byte sectors and 256 byte pages */

#define N25Q_SECTOR_SHIFT           12        /* Sector size 1 << 12 = 4Kb */
#define N25Q_SECTOR_SIZE            (1 << 12) /* Sector size 1 << 12 = 4Kb */
#define N25Q_PAGE_SHIFT             8         /* Sector size 1 << 8 = 256b */
#define N25Q_PAGE_SIZE              (1 << 8)  /* Sector size 1 << 8 = 256b */

#ifdef CONFIG_N25Q_SECTOR512                  /* Simulate a 512 byte sector */
#  define N25Q_SECTOR512_SHIFT      9         /* Sector size 1 << 9 = 512 bytes */
#  define N25Q_SECTOR512_SIZE       (1 << 9)  /* Sector size 1 << 9 = 512 bytes */
#endif

#define N25Q_ERASED_STATE           0xff      /* State of FLASH when erased */

/* Cache flags */

#define N25Q_CACHE_VALID            (1 << 0)  /* 1=Cache has valid data */
#define N25Q_CACHE_DIRTY            (1 << 1)  /* 1=Cache is dirty */
#define N25Q_CACHE_ERASED           (1 << 2)  /* 1=Backing FLASH is erased */

#define IS_VALID(p)                ((((p)->flags) & N25Q_CACHE_VALID) != 0)
#define IS_DIRTY(p)                ((((p)->flags) & N25Q_CACHE_DIRTY) != 0)
#define IS_ERASED(p)               ((((p)->flags) & N25Q_CACHE_DIRTY) != 0)

#define SET_VALID(p)               do { (p)->flags |= N25Q_CACHE_VALID; } while (0)
#define SET_DIRTY(p)               do { (p)->flags |= N25Q_CACHE_DIRTY; } while (0)
#define SET_ERASED(p)              do { (p)->flags |= N25Q_CACHE_DIRTY; } while (0)

#define CLR_VALID(p)               do { (p)->flags &= ~N25Q_CACHE_VALID; } while (0)
#define CLR_DIRTY(p)               do { (p)->flags &= ~N25Q_CACHE_DIRTY; } while (0)
#define CLR_ERASED(p)              do { (p)->flags &= ~N25Q_CACHE_DIRTY; } while (0)

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s must
 * appear at the beginning of the definition so that you can freely cast between
 * pointers to struct mtd_dev_s and struct n25q_dev_s.
 */

struct n25q_dev_s
{
  struct mtd_dev_s      mtd;         /* MTD interface */
  FAR struct spi_dev_s *spi;         /* Saved SPI interface instance */
  uint16_t              nsectors;    /* Number of erase sectors */

#if defined(CONFIG_N25Q_SECTOR512) && !defined(CONFIG_N25Q_READONLY)
  uint8_t               flags;       /* Buffered sector flags */
  uint16_t              esectno;     /* Erase sector number in the cache*/
  FAR uint8_t          *sector;      /* Allocated sector data */
#endif
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Helpers */

static void n25q_lock(FAR struct spi_dev_s *spi);
static inline void n25q_unlock(FAR struct spi_dev_s *spi);
static inline int n25q_readid(FAR struct n25q_dev_s *priv);
#ifndef CONFIG_N25Q_READONLY
static void n25q_unprotect(FAR struct n25q_dev_s *priv);
#endif
static uint8_t n25q_waitwritecomplete(FAR struct n25q_dev_s *priv);
static inline void n25q_wren(FAR struct n25q_dev_s *priv);
static inline void n25q_wrdi(FAR struct n25q_dev_s *priv);
static void n25q_sectorerase(FAR struct n25q_dev_s *priv, off_t offset);
static inline int n25q_chiperase(FAR struct n25q_dev_s *priv);
static void n25q_byteread(FAR struct n25q_dev_s *priv, FAR uint8_t *buffer,
                           off_t address, size_t nbytes);
#ifndef CONFIG_N25Q_READONLY
static void n25q_pagewrite(FAR struct n25q_dev_s *priv, FAR const uint8_t *buffer,
                            off_t address, size_t nbytes);
#endif
#ifdef CONFIG_N25Q_SECTOR512
static void n25q_cacheflush(struct n25q_dev_s *priv);
static FAR uint8_t *n25q_cacheread(struct n25q_dev_s *priv, off_t sector);
static void n25q_cacheerase(struct n25q_dev_s *priv, off_t sector);
static void n25q_cachewrite(FAR struct n25q_dev_s *priv, FAR const uint8_t *buffer,
                             off_t sector, size_t nsectors);
#endif

/* MTD driver methods */

static int n25q_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);
static ssize_t n25q_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR uint8_t *buf);
static ssize_t n25q_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, FAR const uint8_t *buf);
static ssize_t n25q_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                          FAR uint8_t *buffer);
static int n25q_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: n25q_lock
 ************************************************************************************/

static void n25q_lock(FAR struct spi_dev_s *spi)
{
  /* On SPI busses where there are multiple devices, it will be necessary to
   * lock SPI to have exclusive access to the busses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusiv access to
   * the SPI buss.  We will retain that exclusive access until the bus is unlocked.
   */

  (void)SPI_LOCK(spi, true);

  /* After locking the SPI bus, the we also need call the setfrequency, setbits, and
   * setmode methods to make sure that the SPI is properly configured for the device.
   * If the SPI buss is being shared, then it may have been left in an incompatible
   * state.
   */

  SPI_SETMODE(spi, CONFIG_N25Q_SPIMODE);
  SPI_SETBITS(spi, 8);
  (void)SPI_SETFREQUENCY(spi, CONFIG_N25Q_SPIFREQUENCY);
}

/************************************************************************************
 * Name: n25q_unlock
 ************************************************************************************/

static inline void n25q_unlock(FAR struct spi_dev_s *spi)
{
  (void)SPI_LOCK(spi, false);
}

/************************************************************************************
 * Name: n25q_readid
 ************************************************************************************/

static inline int n25q_readid(struct n25q_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t memory;
  uint16_t capacity;

  fvdbg("priv: %p\n", priv);

  /* Lock the SPI bus, configure the bus, and select this FLASH part. */

  n25q_lock(priv->spi);
  SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

  /* Send the "Read ID (RDID)" command and read the first three ID bytes */

  (void)SPI_SEND(priv->spi, N25Q_JEDEC_ID);
  manufacturer = SPI_SEND(priv->spi, N25Q_DUMMY);
  memory       = SPI_SEND(priv->spi, N25Q_DUMMY);
  capacity     = SPI_SEND(priv->spi, N25Q_DUMMY);

  /* Deselect the FLASH and unlock the bus */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
  n25q_unlock(priv->spi);

  fvdbg("manufacturer: %02x memory: %02x capacity: %02x\n",
        manufacturer, memory, capacity);

  /* Check for a valid manufacturer and memory type */

  if ((manufacturer == N25Q_JEDEC_MANUFACTURER) &&
      (memory == N25Q_JEDEC_MEMORY_TYPE))
    {
      /* Okay.. is it a FLASH capacity that we understand? If so, save
       * the FLASH capacity.
       */

      /* 16M-bit / 2M-byte (2,097,152)
       *
       * W24X16, N25Q16BV, N25Q16CL, N25Q16CV, N25Q16DW
       */

      if (capacity == N25Q_JEDEC_CAPACITY_16MBIT)
        {
           priv->nsectors = NSECTORS_16MBIT;
        }

      /* 32M-bit / M-byte (4,194,304)
       *
       * N25X32, N25Q32BV, N25Q32DW
       */

      else if (capacity == N25Q_JEDEC_CAPACITY_32MBIT)
        {
           priv->nsectors = NSECTORS_32MBIT;
        }

      /* 64M-bit / 8M-byte (8,388,608)
       *
       * N25X64,  N25Q64BV, N25Q64CV, N25Q64DW
       */

      else if (capacity == N25Q_JEDEC_CAPACITY_64MBIT)
        {
           priv->nsectors = NSECTORS_64MBIT;
        }

      /* 128M-bit / 16M-byte (16,777,216)
       *
       * N25Q128BV
       */

      else if (capacity == N25Q_JEDEC_CAPACITY_128MBIT)
        {
           priv->nsectors = NSECTORS_128MBIT;
        }
      else
        {
          /* Nope.. we don't understand this capacity. */

          return -ENODEV;
        }

      return OK;
    }

  /* We don't understand the manufacturer or the memory type */

  return -ENODEV;
}

/************************************************************************************
 * Name: n25q_unprotect
 ************************************************************************************/

#ifndef CONFIG_N25Q_READONLY
static void n25q_unprotect(FAR struct n25q_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

  /* Send "Write enable (WREN)" */

  n25q_wren(priv);

  /* Re-select this FLASH part (This might not be necessary... but is it shown in
   * the SST25 timing diagrams from which this code was leveraged.)
   */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
  SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

  /* Send "Write enable status (EWSR)" */

  SPI_SEND(priv->spi, N25Q_WRSR);

  /* Following by the new status value */

  SPI_SEND(priv->spi, 0);
  SPI_SEND(priv->spi, 0);
}
#endif

/************************************************************************************
 * Name: n25q_waitwritecomplete
 ************************************************************************************/

static uint8_t n25q_waitwritecomplete(struct n25q_dev_s *priv)
{
  uint8_t status;

  /* Are we the only device on the bus? */

#ifdef CONFIG_SPI_OWNBUS

  /* Select this FLASH part */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

  /* Send "Read Status Register (RDSR)" command */

  (void)SPI_SEND(priv->spi, N25Q_RDSR);
  
  /* Loop as long as the memory is busy with a write cycle */

  do
    {
      /* Send a dummy byte to generate the clock needed to shift out the status */

      status = SPI_SEND(priv->spi, N25Q_DUMMY);
    }
  while ((status & N25Q_SR_BUSY) != 0);

  /* Deselect the FLASH */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, false);

#else

  /* Loop as long as the memory is busy with a write cycle */

  do
    {
      /* Select this FLASH part */

      SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

      /* Send "Read Status Register (RDSR)" command */

      (void)SPI_SEND(priv->spi, N25Q_RDSR);

      /* Send a dummy byte to generate the clock needed to shift out the status */

      status = SPI_SEND(priv->spi, N25Q_DUMMY);

      /* Deselect the FLASH */

      SPI_SELECT(priv->spi, SPIDEV_FLASH, false);

      /* Given that writing could take up to few tens of milliseconds, and erasing
       * could take more.  The following short delay in the "busy" case will allow
       * other peripherals to access the SPI bus.
       */

#if 0 /* Makes writes too slow */
      if ((status & N25Q_SR_BUSY) != 0)
        {
          n25q_unlock(priv->spi);
          usleep(1000);
          n25q_lock(priv->spi);
        }
#endif
    }
  while ((status & N25Q_SR_BUSY) != 0);
#endif

  return status;
}

/************************************************************************************
 * Name:  n25q_wren
 ************************************************************************************/

static inline void n25q_wren(struct n25q_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

  /* Send "Write Enable (WREN)" command */

  (void)SPI_SEND(priv->spi, N25Q_WREN);
  
  /* Deselect the FLASH */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
}

/************************************************************************************
 * Name:  n25q_wrdi
 ************************************************************************************/

static inline void n25q_wrdi(struct n25q_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

  /* Send "Write Disable (WRDI)" command */

  (void)SPI_SEND(priv->spi, N25Q_WRDI);
  
  /* Deselect the FLASH */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
}

/************************************************************************************
 * Name:  n25q_sectorerase
 ************************************************************************************/

static void n25q_sectorerase(struct n25q_dev_s *priv, off_t sector)
{
  off_t address = sector << N25Q_SECTOR_SHIFT;

  fvdbg("sector: %08lx\n", (long)sector);

  /* Wait for any preceding write or erase operation to complete. */

  (void)n25q_waitwritecomplete(priv);

  /* Send write enable instruction */

  n25q_wren(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

  /* Send the "Sector Erase (SE)" instruction */

  (void)SPI_SEND(priv->spi, N25Q_SE);

  /* Send the sector address high byte first. Only the most significant bits (those
   * corresponding to the sector) have any meaning.
   */

  (void)SPI_SEND(priv->spi, (address >> 16) & 0xff);
  (void)SPI_SEND(priv->spi, (address >> 8) & 0xff);
  (void)SPI_SEND(priv->spi, address & 0xff);

  /* Deselect the FLASH */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
}

/************************************************************************************
 * Name:  n25q_chiperase
 ************************************************************************************/

static inline int n25q_chiperase(struct n25q_dev_s *priv)
{
  fvdbg("priv: %p\n", priv);

  /* Wait for any preceding write or erase operation to complete. */

  (void)n25q_waitwritecomplete(priv);

  /* Send write enable instruction */

  n25q_wren(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

  /* Send the "Chip Erase (CE)" instruction */

  (void)SPI_SEND(priv->spi, N25Q_CE);

  /* Deselect the FLASH */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
  fvdbg("Return: OK\n");
  return OK;
}

/************************************************************************************
 * Name: n25q_byteread
 ************************************************************************************/

static void n25q_byteread(FAR struct n25q_dev_s *priv, FAR uint8_t *buffer,
                           off_t address, size_t nbytes)
{
  uint8_t status;

  fvdbg("address: %08lx nbytes: %d\n", (long)address, (int)nbytes);

  /* Wait for any preceding write or erase operation to complete. */

  status = n25q_waitwritecomplete(priv);
//DEBUGASSERT((status & (N25Q_SR_WEL|N25Q_SR_BP_MASK)) == 0);

  /* Make sure that writing is disabled */

  n25q_wrdi(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

  /* Send "Read from Memory " instruction */

  (void)SPI_SEND(priv->spi, N25Q_RDDATA);

  /* Send the address high byte first. */

  (void)SPI_SEND(priv->spi, (address >> 16) & 0xff);
  (void)SPI_SEND(priv->spi, (address >> 8) & 0xff);
  (void)SPI_SEND(priv->spi, address & 0xff);

  /* Then read all of the requested bytes */

  SPI_RECVBLOCK(priv->spi, buffer, nbytes);

  /* Deselect the FLASH */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
}

/************************************************************************************
 * Name:  n25q_pagewrite
 ************************************************************************************/

#ifndef CONFIG_N25Q_READONLY
static void n25q_pagewrite(struct n25q_dev_s *priv, FAR const uint8_t *buffer,
                          off_t address, size_t nbytes)
{
  uint8_t status;

  fvdbg("address: %08lx nwords: %d\n", (long)address, (int)nbytes);
  DEBUGASSERT(priv && buffer && (address & 0xff) == 0 &&
             (nbytes & 0xff) == 0);

  for (; nbytes > 0; nbytes -= N25Q_PAGE_SIZE)
    {
      /* Wait for any preceding write or erase operation to complete. */

      status = n25q_waitwritecomplete(priv);
//    DEBUGASSERT((status & (N25Q_SR_WEL|N25Q_SR_BP_MASK)) == 0);

      /* Enable write access to the FLASH */

      n25q_wren(priv);

      /* Select this FLASH part */

      SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

      /* Send the "Page Program (N25Q_PP)" Command */

      SPI_SEND(priv->spi, N25Q_PP);

      /* Send the address high byte first. */

      (void)SPI_SEND(priv->spi, (address >> 16) & 0xff);
      (void)SPI_SEND(priv->spi, (address >> 8) & 0xff);
      (void)SPI_SEND(priv->spi, address & 0xff);

      /* Then send the page of data */

      SPI_SNDBLOCK(priv->spi, buffer, N25Q_PAGE_SIZE);

      /* Deselect the FLASH and setup for the next pass through the loop */

      SPI_SELECT(priv->spi, SPIDEV_FLASH, false);

      /* Update addresses */

      address += N25Q_PAGE_SIZE;
      buffer  += N25Q_PAGE_SIZE;
    }

  /* Disable writing */

  n25q_wrdi(priv);
}
#endif

/************************************************************************************
 * Name: n25q_cacheflush
 ************************************************************************************/

#if defined(CONFIG_N25Q_SECTOR512) && !defined(CONFIG_N25Q_READONLY)
static void n25q_cacheflush(struct n25q_dev_s *priv)
{
  /* If the cached is dirty (meaning that it no longer matches the old FLASH contents)
   * or was erased (with the cache containing the correct FLASH contents), then write
   * the cached erase block to FLASH.
   */

  if (IS_DIRTY(priv) || IS_ERASED(priv))
    {
      /* Write entire erase block to FLASH */

      n25q_pagewrite(priv, priv->sector, (off_t)priv->esectno << N25Q_SECTOR_SHIFT,
                      N25Q_SECTOR_SIZE);

      /* The case is no long dirty and the FLASH is no longer erased */

      CLR_DIRTY(priv);
      CLR_ERASED(priv);
    }
}
#endif

/************************************************************************************
 * Name: n25q_cacheread
 ************************************************************************************/

#if defined(CONFIG_N25Q_SECTOR512) && !defined(CONFIG_N25Q_READONLY)
static FAR uint8_t *n25q_cacheread(struct n25q_dev_s *priv, off_t sector)
{
  off_t esectno;
  int   shift;
  int   index;
 
  /* Convert from the 512 byte sector to the erase sector size of the device.  For
   * exmample, if the actual erase sector size if 4Kb (1 << 12), then we first
   * shift to the right by 3 to get the sector number in 4096 increments.
   */

  shift    = N25Q_SECTOR_SHIFT - N25Q_SECTOR512_SHIFT;
  esectno  = sector >> shift;
  fvdbg("sector: %ld esectno: %d shift=%d\n", sector, esectno, shift);

  /* Check if the requested erase block is already in the cache */

  if (!IS_VALID(priv) || esectno != priv->esectno)
    {
      /* No.. Flush any dirty erase block currently in the cache */

      n25q_cacheflush(priv);

      /* Read the erase block into the cache */

      n25q_byteread(priv, priv->sector, (esectno << N25Q_SECTOR_SHIFT), N25Q_SECTOR_SIZE);

      /* Mark the sector as cached */

      priv->esectno = esectno;

      SET_VALID(priv);          /* The data in the cache is valid */
      CLR_DIRTY(priv);          /* It should match the FLASH contents */
      CLR_ERASED(priv);         /* The underlying FLASH has not been erased */
    }

  /* Get the index to the 512 sector in the erase block that holds the argument */

  index = sector & ((1 << shift) - 1);

  /* Return the address in the cache that holds this sector */

  return &priv->sector[index << N25Q_SECTOR512_SHIFT];
}
#endif

/************************************************************************************
 * Name: n25q_cacheerase
 ************************************************************************************/

#if defined(CONFIG_N25Q_SECTOR512) && !defined(CONFIG_N25Q_READONLY)
static void n25q_cacheerase(struct n25q_dev_s *priv, off_t sector)
{
  FAR uint8_t *dest;

  /* First, make sure that the erase block containing the 512 byte sector is in
   * the cache.
   */

  dest = n25q_cacheread(priv, sector);

  /* Erase the block containing this sector if it is not already erased.
   * The erased indicated will be cleared when the data from the erase sector
   * is read into the cache and set here when we erase the block.
   */

  if (!IS_ERASED(priv))
    {
      off_t esectno  = sector >> (N25Q_SECTOR_SHIFT - N25Q_SECTOR512_SHIFT);
      fvdbg("sector: %ld esectno: %d\n", sector, esectno);

      n25q_sectorerase(priv, esectno);
      SET_ERASED(priv);
    }

  /* Put the cached sector data into the erase state and mart the cache as dirty
   * (but don't update the FLASH yet.  The caller will do that at a more optimal
   * time).
   */

  memset(dest, N25Q_ERASED_STATE, N25Q_SECTOR512_SIZE);
  SET_DIRTY(priv);
}
#endif

/************************************************************************************
 * Name: n25q_cachewrite
 ************************************************************************************/

#if defined(CONFIG_N25Q_SECTOR512) && !defined(CONFIG_N25Q_READONLY)
static void n25q_cachewrite(FAR struct n25q_dev_s *priv, FAR const uint8_t *buffer,
                            off_t sector, size_t nsectors)
{
  FAR uint8_t *dest;

  for (; nsectors > 0; nsectors--)
    {
      /* First, make sure that the erase block containing 512 byte sector is in
       * memory.
       */

      dest = n25q_cacheread(priv, sector);

      /* Erase the block containing this sector if it is not already erased.
       * The erased indicated will be cleared when the data from the erase sector
       * is read into the cache and set here when we erase the sector.
       */

      if (!IS_ERASED(priv))
        {
          off_t esectno  = sector >> (N25Q_SECTOR_SHIFT - N25Q_SECTOR512_SHIFT);
          fvdbg("sector: %ld esectno: %d\n", sector, esectno);

          n25q_sectorerase(priv, esectno);
          SET_ERASED(priv);
        }

      /* Copy the new sector data into cached erase block */

      memcpy(dest, buffer, N25Q_SECTOR512_SIZE);
      SET_DIRTY(priv);

      /* Set up for the next 512 byte sector */

      buffer += N25Q_SECTOR512_SIZE;
      sector++;
    }

  /* Flush the last erase block left in the cache */

  n25q_cacheflush(priv);
}
#endif

/************************************************************************************
 * Name: n25q_erase
 ************************************************************************************/

static int n25q_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
#ifdef CONFIG_N25Q_READONLY
  return -EACESS
#else
  FAR struct n25q_dev_s *priv = (FAR struct n25q_dev_s *)dev;
  size_t blocksleft = nblocks;

  fvdbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  n25q_lock(priv->spi);

  while (blocksleft-- > 0)
    {
      /* Erase each sector */

#ifdef CONFIG_N25Q_SECTOR512
      n25q_cacheerase(priv, startblock);
#else
      n25q_sectorerase(priv, startblock);
#endif
      startblock++;
    }

#ifdef CONFIG_N25Q_SECTOR512
  /* Flush the last erase block left in the cache */

  n25q_cacheflush(priv);
#endif

  n25q_unlock(priv->spi);
  return (int)nblocks;
#endif
}

/************************************************************************************
 * Name: n25q_bread
 ************************************************************************************/

static ssize_t n25q_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                           FAR uint8_t *buffer)
{
  ssize_t nbytes;

  fvdbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented read */

#ifdef CONFIG_N25Q_SECTOR512
  nbytes = n25q_read(dev, startblock << N25Q_SECTOR512_SHIFT, nblocks << N25Q_SECTOR512_SHIFT, buffer);
  if (nbytes > 0)
    {
      nbytes >>= N25Q_SECTOR512_SHIFT;
    }
#else
  nbytes = n25q_read(dev, startblock << N25Q_SECTOR_SHIFT, nblocks << N25Q_SECTOR_SHIFT, buffer);
  if (nbytes > 0)
    {
      nbytes >>= N25Q_SECTOR_SHIFT;
    }
#endif

  return nbytes;
}

/************************************************************************************
 * Name: n25q_bwrite
 ************************************************************************************/

static ssize_t n25q_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                            FAR const uint8_t *buffer)
{
#ifdef CONFIG_N25Q_READONLY
  return -EACCESS;
#else
  FAR struct n25q_dev_s *priv = (FAR struct n25q_dev_s *)dev;

  fvdbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock the SPI bus and write all of the pages to FLASH */

  n25q_lock(priv->spi);

#if defined(CONFIG_N25Q_SECTOR512)
  n25q_cachewrite(priv, buffer, startblock, nblocks);
#else
  n25q_pagewrite(priv, buffer, startblock << N25Q_SECTOR_SHIFT,
                  nblocks << N25Q_SECTOR_SHIFT);
#endif
  n25q_unlock(priv->spi);

  return nblocks;
#endif
}

/************************************************************************************
 * Name: n25q_read
 ************************************************************************************/

static ssize_t n25q_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                          FAR uint8_t *buffer)
{
  FAR struct n25q_dev_s *priv = (FAR struct n25q_dev_s *)dev;

  fvdbg("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the SPI bus and select this FLASH part */

  n25q_lock(priv->spi);
  n25q_byteread(priv, buffer, offset, nbytes);
  n25q_unlock(priv->spi);

  fvdbg("return nbytes: %d\n", (int)nbytes);
  return nbytes;
}

/************************************************************************************
 * Name: n25q_ioctl
 ************************************************************************************/

static int n25q_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct n25q_dev_s *priv = (FAR struct n25q_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  fvdbg("cmd: %d \n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)((uintptr_t)arg);
          if (geo)
            {
              /* Populate the geometry structure with information need to know
               * the capacity and how to access the device.
               *
               * NOTE: that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but the client
               * will expect the device logic to do whatever is necessary to make it
               * appear so.
               */

#ifdef CONFIG_N25Q_SECTOR512
              geo->blocksize    = (1 << N25Q_SECTOR512_SHIFT);
              geo->erasesize    = (1 << N25Q_SECTOR512_SHIFT);
              geo->neraseblocks = priv->nsectors << (N25Q_SECTOR_SHIFT - N25Q_SECTOR512_SHIFT);
#else
              geo->blocksize    = N25Q_SECTOR_SIZE;
              geo->erasesize    = N25Q_SECTOR_SIZE;
              geo->neraseblocks = priv->nsectors;
#endif
              ret               = OK;

              fvdbg("blocksize: %d erasesize: %d neraseblocks: %d\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
            /* Erase the entire device */

            n25q_lock(priv->spi);
            ret = n25q_chiperase(priv);
            n25q_unlock(priv->spi);
        }
        break;
 
      case MTDIOC_XIPBASE:
      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  fvdbg("return %d\n", ret);
  return ret;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: n25q_initialize
 *
 * Description:
 *   Create an initialize MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ************************************************************************************/

FAR struct mtd_dev_s *n25q_initialize(FAR struct spi_dev_s *spi)
{
  FAR struct n25q_dev_s *priv;
  int ret;

  fvdbg("spi: %p\n", spi);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per SPI
   * device (only because of the SPIDEV_FLASH definition) and so would have
   * to be extended to handle multiple FLASH parts on the same SPI bus.
   */

  priv = (FAR struct n25q_dev_s *)kzalloc(sizeof(struct n25q_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure */

      priv->mtd.erase  = n25q_erase;
      priv->mtd.bread  = n25q_bread;
      priv->mtd.bwrite = n25q_bwrite;
      priv->mtd.read   = n25q_read;
      priv->mtd.ioctl  = n25q_ioctl;
      priv->spi        = spi;

      /* Deselect the FLASH */

      SPI_SELECT(spi, SPIDEV_FLASH, false);

      /* Identify the FLASH chip and get its capacity */

      ret = n25q_readid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and return NULL */

          fdbg("Unrecognized\n");
          kfree(priv);
          priv = NULL;
        }
      else
        {
          /* Make sure the the FLASH is unprotected so that we can write into it */

#ifndef CONFIG_N25Q_READONLY
          n25q_unprotect(priv);
#endif

#ifdef CONFIG_N25Q_SECTOR512        /* Simulate a 512 byte sector */
          /* Allocate a buffer for the erase block cache */

          priv->sector = (FAR uint8_t *)kmalloc(N25Q_SECTOR_SIZE);
          if (!priv->sector)
            {
              /* Allocation failed! Discard all of that work we just did and return NULL */

              fdbg("Allocation failed\n");
              kfree(priv);
              priv = NULL;
            }
#endif
        }
    }

  /* Return the implementation-specific state structure as the MTD device */

  fvdbg("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}
