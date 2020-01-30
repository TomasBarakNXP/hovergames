/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "Pixy2Cam.h"
#include "Pixy2_DSPI.h"
#include "fsl_uart.h"
#include "fsl_dspi.h"
#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define EXAMPLE_DSPI_SLAVE_BASEADDR SPI1
#define SLAVE_SPI_IRQ SPI1_IRQn

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */

void Pixy_printBlock(Pixy2CCCBlock *b)
{
    PRINTF("sig: %d, x: %d, y: %d, width: %d, height: %d, ang: %d, index: %d, age: %d\n", b->signature, b->x, b->y,
           b->width, b->height, b->angle, b->index, b->age);
}

void Pixy_PrintVersion(Pixy2Cam *p)
{
    Pixy2Version *v = &p->version;

    PRINTF(
        "--Pixy Camera Version--\n"
        "Hardware Version:%d Firmware Version: %d.%d build %d \"%s\"\n\n",
        v->hwVersion, v->firmwareMajor, v->firmwareMinor, v->firmwareBuild, v->firmwareType);
}

int main(void)
{
    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    PRINTF("Hello world Pixy2.\r\n");

    Pixy2Cam cam;                  // This is the handle used by the Pixy2 Library
    Pixy2_DSPI_Handle dspi_handle; // This is internal SPI handle passed to the interface layer

    Pixy2_BindDSPILink(&cam, &dspi_handle, SPI0, CLOCK_GetFreq(DSPI0_CLK_SRC), SPI0_IRQn);

    /* get ccc data and print first ccc block (information about biggest object) */
    if (Pixy2Cam_Init(&cam) != PIXY2_OK)
    {
        PRINTF("Pixy2Cam_Init failed\n");
        while (1)
            ;
    }

    // Print the version of the Pixy Camera
    Pixy_PrintVersion(&cam);

    // Array for the Color blocks
    Pixy2CCCBlock blocks[10];

    while (1)
    {
        int32_t num = Pixy2Cam_CCCgetBlocks(&cam, CCC_SIG1, 10, blocks);

        if (num > 0)
        {
            PRINTF("We have %d blocks\n", num);
            for (int32_t i = 0; i < num; i++)
            {
                Pixy_printBlock(blocks + i);
            }
        }
        else
        {
            PRINTF("pixy2_CCC_getBlocks: %d\n", num);
        }

        volatile int i;
        for (i = 0; i < 1000000; i++)
        {
        }
    }
}
