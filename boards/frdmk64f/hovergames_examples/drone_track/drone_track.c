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
#include "Pixy2Cam.h"
#include "Pixy2_DSPI.h"
#include "fsl_uart.h"
#include "fsl_i2c.h"
#include "fsl_dspi.h"

#include "common/mavlink.h"

#include <stdarg.h>
#include <stdlib.h>
#include "fsl_str.h"

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

#define BT_UART UART1
#define BT_UART_CLKSRC UART1_CLK_SRC
#define BT_UART_CLK_FREQ CLOCK_GetFreq(UART1_CLK_SRC)
#define BT_UART_IRQn UART1_RX_TX_IRQn
#define BT_UART_IRQHandler UART1_RX_TX_IRQHandler

///* UART instance and clock */

#define DEMO_RING_BUFFER_SIZE 512
uint8_t demoRingBuffer[DEMO_RING_BUFFER_SIZE];
volatile uint16_t txIndex; /* Index of the data to send out. */
volatile uint16_t rxIndex; /* Index of the memory to save new arrived data. */

#define MLX90614_ADDR_7BIT (0x5AU)
#define I2C_BAUDRATE 50000
#define I2C_DATA_LENGTH 32U

enum Direction
{
    Direction_Left,
    Direction_Right
};
enum State
{
    Drone_Idle,
    Drone_Tracking
};

struct drone_state
{
    uint32_t noObjTimeout;
    enum Direction lastDirection;
    enum State state;
    float temperature;
};

volatile struct drone_state DroneState;

Pixy2Cam cam;                  // This is the handle used by the Pixy2 Library
Pixy2_DSPI_Handle dspi_handle; // This is internal SPI handle passed to the interface layer

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void sendManual(int16_t yaw, int16_t pitch, int16_t roll, int16_t thr);
float getTemperature();
void controlLoop();
int init();

/*******************************************************************************
 * Code
 ******************************************************************************/

void sendManual(int16_t yaw, int16_t pitch, int16_t roll, int16_t thr)
{
    // Prepare manual control values to be sent
    int16_t b = 0;

    // Prepare message
    mavlink_message_t msg;
    mavlink_msg_manual_control_pack(1, 1, &msg, 1, roll, pitch, thr, yaw, b);

    // Populate UART buffer
    uint8_t buf[512];
    uint16_t lenght = mavlink_msg_to_send_buffer(buf, &msg);

    UART_WriteBlocking(MAV_UART, buf, lenght);
}

float getTemperature()
{
    uint8_t data_buff[3];
    data_buff[0] = 0x7; // read RAM address 0x7 - Tobj1

    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = MLX90614_ADDR_7BIT;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = data_buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferNoStopFlag;

    I2C_MasterTransferBlocking(I2C0, &masterXfer);

    masterXfer.slaveAddress   = MLX90614_ADDR_7BIT;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = data_buff;
    masterXfer.dataSize       = 3;
    masterXfer.flags          = kI2C_TransferRepeatedStartFlag; // kI2C_TransferDefaultFlag;

    I2C_MasterTransferBlocking(I2C0, &masterXfer);

    // TODO check CRC

    // Tobj1
    return ((((uint16_t)data_buff[1]) << 8) | data_buff[0]) * 0.02 - 273.15;
}

void controlLoop()
{
    /* Get blocks form pixy2 camera */

    Pixy2CCCBlock blocks[10];
    /* Get blocks form pixy2 camera */
    int block_num = Pixy2Cam_CCCgetBlocks(&cam, CCC_SIG1, 10, blocks);

    /*  If we can see a color block, start tracking */
    if (block_num > 0)
    {
        DroneState.noObjTimeout = 50;
        DroneState.state        = Drone_Tracking;

        int position             = ((int)blocks[0].x - (320 / 2)) * 6;
        DroneState.lastDirection = (position > 0) ? Direction_Left : Direction_Right;

        sendManual(position, 0, 0, 500);
    }
    else
    {
        if (DroneState.noObjTimeout <= 0)
        {
            /* Stop the drone */
            sendManual(0, 0, 0, 0);
            DroneState.state = Drone_Idle;
        }
        else
        {
            DroneState.noObjTimeout -= 1;
            switch (DroneState.lastDirection)
            {
                case Direction_Right:
                    sendManual(-160, 0, 0, 500);
                    break;
                case Direction_Left:
                    sendManual(160, 0, 0, 500);
                    break;
                default:
                    sendManual(0, 0, 0, 0);
            }
        }
    }
}

void updateTemperature()
{
    DroneState.temperature = getTemperature();
    char buffer[32];
    PRINTF("T: %3.1f*C\n", DroneState.temperature);
    snprintf(buffer, 32, "T: %3.1f*C\n", DroneState.temperature);
    UART_WriteBlocking(BT_UART, (uint8_t *)buffer, strlen(buffer));
}

int init()
{
    /* Initialize UART */
    uart_config_t config;
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200;
    config.enableTx     = true;
    config.enableRx     = true;

    UART_Init(MAV_UART, &config, MAV_UART_CLK_FREQ);

    UART_Init(BT_UART, &config, BT_UART_CLK_FREQ);

    /* Initialize I2C */
    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(I2C0_CLK_SRC));

    /* Initialize Pixy2 */
    Pixy2_BindDSPILink(&cam, &dspi_handle, SPI0, CLOCK_GetFreq(DSPI0_CLK_SRC), SPI0_IRQn);

    if (Pixy2Cam_Init(&cam) != PIXY2_OK)
    {
        PRINTF("Pixy2Cam_Init failed\n");
        while (1)
            ;
    }

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
        updateTemperature();

        // Arbitrary delay
        volatile int i;
        for (i = 0; i < 10000; i++)
        {
        }
    }
}
