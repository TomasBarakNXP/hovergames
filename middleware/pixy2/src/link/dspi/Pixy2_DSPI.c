/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Pixy2_DSPI.h"
#include "Pixy2Cam.h"
#include "fsl_dspi.h"

static int DSPI_Init(void *comdata);
static int DSPI_Close(void *comdata);
static int DSPI_Send(void *comdata, uint8_t *rx_buffer, uint32_t len);
static int DSPI_Recv(void *comdata, uint8_t *tx_buffer, uint32_t len);

// The DSPI Transfer complete callback
static void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData);

void Pixy2_BindDSPILink(
    Pixy2Cam *p, Pixy2_DSPI_Handle *handle, SPI_Type *spi_base, uint32_t spi_clkFrq, IRQn_Type irq_handler)
{
    handle->spi_base            = spi_base;
    handle->spi_clkFrq          = spi_clkFrq;
    handle->irq_handler         = irq_handler;
    handle->isTransferCompleted = false;
    handle->callback            = DSPI_MasterUserCallback;

    // Bind the handle to the camera. This data is passed to the interface functions
    p->com.data = handle;

    p->com.init  = DSPI_Init;
    p->com.close = DSPI_Close;
    p->com.recv  = DSPI_Recv;
    p->com.send  = DSPI_Send;
}

static int DSPI_Init(void *comdata)
{
    Pixy2_DSPI_Handle *handle = (Pixy2_DSPI_Handle *)comdata;

    dspi_master_config_t masterConfig;
    /* Master config */
    masterConfig.whichCtar                                = kDSPI_Ctar0;
    masterConfig.ctarConfig.baudRate                      = PIXY2_SPI_CLOCKRATE;
    masterConfig.ctarConfig.bitsPerFrame                  = 8U;
    masterConfig.ctarConfig.cpol                          = kDSPI_ClockPolarityActiveHigh;
    masterConfig.ctarConfig.cpha                          = kDSPI_ClockPhaseSecondEdge;
    masterConfig.ctarConfig.direction                     = kDSPI_MsbFirst;
    masterConfig.ctarConfig.pcsToSckDelayInNanoSec        = 1000000000U / PIXY2_SPI_CLOCKRATE;
    masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec    = 1000000000U / PIXY2_SPI_CLOCKRATE;
    masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U / PIXY2_SPI_CLOCKRATE;
    masterConfig.whichPcs                                 = kDSPI_Pcs0;
    masterConfig.pcsActiveHighOrLow                       = kDSPI_PcsActiveLow;
    masterConfig.enableContinuousSCK                      = false;
    masterConfig.enableRxFifoOverWrite                    = false;
    masterConfig.enableModifiedTimingFormat               = false;
    masterConfig.samplePoint                              = kDSPI_SckToSin0Clock;

    DSPI_MasterInit(handle->spi_base, &masterConfig, handle->spi_clkFrq);
    NVIC_SetPriority(handle->irq_handler, 1U);

    /* Set up master transfer */
    DSPI_MasterTransferCreateHandle(handle->spi_base, &handle->g_m_handle, handle->callback, handle);

    return 0;
}

static int DSPI_Close(void *comdata)
{
    Pixy2_DSPI_Handle *handle = (Pixy2_DSPI_Handle *)comdata;

    DSPI_Deinit(handle->spi_base);
    return 0;
}

static int DSPI_Send(void *comdata, uint8_t *tx_buffer, uint32_t len)
{
    Pixy2_DSPI_Handle *handle = (Pixy2_DSPI_Handle *)comdata;

    handle->masterXfer.txData      = tx_buffer;
    handle->masterXfer.rxData      = NULL;
    handle->masterXfer.dataSize    = len;
    handle->masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

    handle->isTransferCompleted = false;
    status_t stat;
    stat = DSPI_MasterTransferNonBlocking(handle->spi_base, &handle->g_m_handle, &handle->masterXfer);
    assert(stat == kStatus_Success);

    // Wait for the transfer to finish (should be async)
    while (!handle->isTransferCompleted)
    {
    }

    return len;
}

static int DSPI_Recv(void *comdata, uint8_t *rx_buffer, uint32_t len)
{
    Pixy2_DSPI_Handle *handle = (Pixy2_DSPI_Handle *)comdata;

    if (len == 0)
        return -1;

    handle->masterXfer.txData      = NULL;
    handle->masterXfer.rxData      = rx_buffer;
    handle->masterXfer.dataSize    = len;
    handle->masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

    handle->isTransferCompleted = false;

    status_t stat;
    stat = DSPI_MasterTransferNonBlocking(handle->spi_base, &handle->g_m_handle, &handle->masterXfer);

    assert(stat == kStatus_Success);

    // Wait for the transfer to finish (should be async)
    while (!handle->isTransferCompleted)
    {
    }

    return len;
}

static void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData)
{
    Pixy2_DSPI_Handle *spi_data = (Pixy2_DSPI_Handle *)userData;
    if (status == kStatus_Success)
    {
        __NOP();
        spi_data->isTransferCompleted = true;
    }
}
