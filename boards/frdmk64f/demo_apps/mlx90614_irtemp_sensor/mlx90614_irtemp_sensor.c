/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "mlx90614.h"

#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Main function
 */
int main()
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    MLX90614_init();
    PRINTF("Basic MLX90614 IR sensor demo. \r\n");

    while (1)
    {
        float temperature = MLX90614_getT();

        PRINTF("Temperature: %f\r\n", temperature);

        volatile int i;
        for (i = 0; i < 10000000; i++)
        {
        }
    }
}
