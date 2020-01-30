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
#include "fsl_uart.h"

#include "common/mavlink.h"

#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* UART instance and clock */
#define MAV_UART UART3
#define MAV_UART_CLKSRC UART3_CLK_SRC
#define MAV_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define MAV_UART_IRQn UART3_RX_TX_IRQn
#define MAV_UART_IRQHandler UART3_RX_TX_IRQHandler
/* Keeps track of number of loop iterations */
int32_t ticks = 0;

/* Keep track of the drone state */
enum DroneStates
{
    DroneStop,
    DroneTurnLeft,
    DroneTurnRight
} drone_state;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* Send a manual control MAVLink message with given yaw, pitch, roll, and throttle */
void sendManual(int16_t yaw, int16_t pitch, int16_t roll, int16_t thr)
{
    int16_t b = 0; // Buttons are no used, set to 0

    // Prepare message
    mavlink_message_t msg;
    /* System ID and component ID for FMU are both 1 */
    mavlink_msg_manual_control_pack(1, 1, &msg, 1, roll, pitch, thr, yaw, b);

    // Populate UART buffer
    uint8_t buf[512];
    uint16_t lenght = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the UART buffer to the FMU
    UART_WriteBlocking(MAV_UART, buf, lenght);
}

void controlLoop()
{
    switch (drone_state)
    {
        case DroneTurnLeft:
            sendManual(-1000, 0, 0, 500);
            PRINTF("Turn Left\n");
            break;

        case DroneTurnRight:
            sendManual(1000, 0, 0, 500);
            PRINTF("Turn Right\n");
            break;

        case DroneStop:
        default:
            sendManual(0, 0, 0, 0);
            PRINTF("Stop\n");
            break;
    }

    /* Every 500  loop iterations, cycle through the states */
    if (ticks > 500)
    {
        ticks = 0;
        drone_state += 1;
        if (drone_state > DroneTurnRight)
        { /* return back to the start */
            drone_state = DroneStop;
        }
    }

    ticks++;
}

/*******************************************************************************
 * Code
 ******************************************************************************/

int init()
{
    /* Initialize UART */
    uart_config_t config;
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200;
    config.enableTx     = true;
    config.enableRx     = true;

    UART_Init(MAV_UART, &config, MAV_UART_CLK_FREQ);

    drone_state = DroneStop;
    return 0;
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    init();

    while (1)
    {
        controlLoop();

        // Arbitrary delay
        volatile int i;
        for (i = 0; i < 10000; i++)
        {
        }
    }
}
