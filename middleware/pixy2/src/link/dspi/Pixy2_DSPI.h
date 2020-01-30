/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __PIXY2_DSPI_H
#define __PIXY2_DSPI_H

#include "fsl_dspi.h"
#include "Pixy2Cam.h"

#define PIXY2_SPI_CLOCKRATE 2000000

/* DSPI Handle structure
 * contains data used by the DSPI link module
 */
typedef struct _Pixy2_DSPI_Data
{
    SPI_Type *spi_base;
    uint32_t spi_clkFrq;
    IRQn_Type irq_handler;

    volatile bool isTransferCompleted;
    dspi_transfer_t masterXfer;
    dspi_master_handle_t g_m_handle;
    void (*callback)(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData);

} Pixy2_DSPI_Handle;

/* Function: Pixy2_BindDSPILink
 * ----------------------------
 * Bind the DSPI handle to the given camera handle.
 * This function must be called BEFORE camera initialization Pixy2Cam_Init!
 *
 * p: Pointer to pixy2 camera handle
 * handle: Pointer to DSPI handle
 * spi_base: Base SPI peripheral pointer eg SPI0
 * spi_clkFreq: SPI clock frequency (use CLOCK_GetFreq(DSPI0_CLK_SRC))
 * irq_handle: IRQ Type for the given SPI peripheral SPI0_IRQn
 *
 * returns: void
 */
void Pixy2_BindDSPILink(
    Pixy2Cam *p, Pixy2_DSPI_Handle *handle, SPI_Type *spi_base, uint32_t spi_clkFrq, IRQn_Type irq_handler);

#endif
