/****************************************************************************
 * drivers/mtd/mtd_nand.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic was based largely on Atmel sample code with modifications for
 * better integration with NuttX.  The Atmel sample code has a BSD
 * compatibile license that requires this copyright notice:
 *
 *   Copyright (c) 2011, 2012, Atmel Corporation
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
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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
#include <nuttx/mtd/nand_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/nand.h>
#include <nuttx/mtd/onfi.h>
#include <nuttx/mtd/nand_scheme.h>
#include <nuttx/mtd/nand_model.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MTD driver methods */

static int     nand_erase(struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks);
static ssize_t nand_bread(struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks, uint8_t *buf);
static ssize_t nand_bwrite(struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks, const uint8_t *buf);
static int     nand_ioctl(struct mtd_dev_s *dev, int cmd,
                 unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nand_erase
 *
 * Description:
 *   Erase several blocks, each of the size previously reported.
 *
 ****************************************************************************/

static int nand_erase(struct mtd_dev_s *dev, off_t startblock,
                      size_t nblocks)
{
  struct nand_raw_s *priv = (struct nand_raw_s *)dev;

  /* The interface definition assumes that all erase blocks are the same size.
   * If that is not true for this particular device, then transform the
   * start block and nblocks as necessary.
   */
#warning Missing logic

  /* Erase the specified blocks and return status (OK or a negated errno) */

  return OK;
}

/****************************************************************************
 * Name: nand_bread
 *
 * Description:
 *   Read the specified number of blocks into the user provided buffer.
 *
 ****************************************************************************/

static ssize_t nand_bread(struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, uint8_t *buf)
{
  struct nand_raw_s *priv = (struct nand_raw_s *)dev;

  /* The interface definition assumes that all read/write blocks are the same size.
   * If that is not true for this particular device, then transform the
   * start block and nblocks as necessary.
   */

  /* Read the specified blocks into the provided user buffer and return status
   * (The positive, number of blocks actually read or a negated errno).
   */
#warning Missing logic

  return 0;
}

/****************************************************************************
 * Name: nand_bwrite
 *
 * Description:
 *   Write the specified number of blocks from the user provided buffer.
 *
 ****************************************************************************/

static ssize_t nand_bwrite(struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, const uint8_t *buf)
{
  struct nand_raw_s *priv = (struct nand_raw_s *)dev;

  /* The interface definition assumes that all read/write blocks are the same size.
   * If that is not true for this particular device, then transform the
   * start block and nblocks as necessary.
   */

  /* Write the specified blocks from the provided user buffer and return status
   * (The positive, number of blocks actually written or a negated errno)
   */
#warning Missing logic

  return 0;
}

/****************************************************************************
 * Name: nand_ioctl
 ****************************************************************************/

static int nand_ioctl(struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  struct nand_raw_s *priv = (struct nand_raw_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          struct mtd_geometry_s *geo = (struct mtd_geometry_s *)arg;
          if (geo)
            {
              /* Populate the geometry structure with information needed to know
               * the capacity and how to access the device.
               *
               * NOTE: that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but the client
               * will expect the device logic to do whatever is necessary to make it
               * appear so.
               */

              geo->blocksize    = 512;  /* Size of one read/write block */
              geo->erasesize    = 4096; /* Size of one erase block */
              geo->neraseblocks = 1024; /* Number of erase blocks */
              ret               = OK;
          }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          ret = OK;
        }
        break;

      case MTDIOC_XIPBASE:
      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nand_initialize
 *
 * Description:
 *   Probe and initialize NAND.
 *
 * Input parameters:
 *   raw      - Raw NAND FLASH MTD interface
 *   cmdaddr  - NAND command address base
 *   addraddr - NAND address address base
 *   dataaddr - NAND data address
 *   model    - A pointer to the model data (probably in the raw MTD
 *              driver instance.
 *
 * Returned value.
 *   A non-NULL MTD driver intstance is returned on success.  NULL is
 *   returned on any failaure.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *nand_initialize(FAR struct mtd_dev_s *raw,
                                      uintptr_t cmdaddr, uintptr_t addraddr,
                                      uintptr_t dataaddr,
                                      FAR struct nand_model_s *model)
{
  FAR struct nand_dev_s *priv;
  struct onfi_pgparam_s onfi;
  bool compatible;
  bool havemodel = false;
  int ret;

  fvdbg("cmdaddr=%p addraddr=%p dataaddr=%p\n",
        (FAR void *)cmdaddr, (FAR void *)addraddr, (FAR void *)dataaddr);

  /* Check if there is NAND connected on the EBI */

  if (!onfi_ebidetect(cmdaddr, addraddr, dataaddr))
    {
      fdbg("ERROR: No NAND device detected at: %p %p %p\n",
           (FAR void *)cmdaddr, (FAR void *)addraddr, (FAR void *)dataaddr);
      return NULL;
    }

  /* Read the ONFI page parameters from the NAND device */

  ret = onfi_read(cmdaddr, addraddr, dataaddr, &onfi);
  if (ret < 0)
    {
      fvdbg("ERROR: Failed to get ONFI page parameters: %d\n", ret);
      compatible = false;
    }
  else
    {
      uint64_t size;

      fvdbg("Found ONFI compliant NAND FLASH\n");
      compatible = true;

      /* Construct the NAND model structure */

      model->devid     = onfi.manufacturer;
      model->options   = onfi.buswidth ? NANDMODEL_DATAWIDTH16 : NANDMODEL_DATAWIDTH8;
      model->pagesize  = onfi.pagesize;
      model->sparesize = onfi.sparesize;

      size             = (uint64_t)onfi.pagesperblock *
                         (uint64_t)onfi.blocksperlun *
                         (uint64_t)onfi.pagesize;
      DEBUGASSERT(size < (uint64_t)(1 << 21));

      model->devsize   = (uint16_t)(size >> 20);

      size             = (uint64_t)onfi.pagesperblock *
                         (uint64_t)onfi.pagesize;
      DEBUGASSERT(size < (uint64_t)(1 << 11));

      model->blocksize = (uint16_t)(size >> 10);

      switch (onfi.pagesize)
        {
          case 256:
            model->scheme = &g_nand_sparescheme256;
            break;

          case 512:
            model->scheme = &g_nand_sparescheme512;
            break;

          case 2048:
            model->scheme = &g_nand_sparescheme2048;
            break;

          case 4096:
            model->scheme = &g_nand_sparescheme4096;
            break;
        }

      havemodel = true;

      /* Disable any internal, embedded ECC function */

      (void)onfi_embeddedecc(&onfi, cmdaddr, addraddr, dataaddr, false);
    }

  /* Allocate an NAND MTD device structure */

  priv = (FAR struct nand_dev_s *)kzalloc(sizeof(struct nand_dev_s));
  if (!priv)
    {
      fdbg("ERROR: Failed to allocate the NAND MTD device structure\n");
      return NULL;
    }

  /* Initialize the NAND MTD device structure */

  priv->mtd.erase  = nand_erase;
  priv->mtd.bread  = nand_bread;
  priv->mtd.bwrite = nand_bwrite;
  priv->mtd.ioctl  = nand_ioctl;
  priv->raw        = raw;
  priv->model      = model;

  #warning Missing logic

  /* Return the implementation-specific state structure as the MTD device */

  return OK;
}
