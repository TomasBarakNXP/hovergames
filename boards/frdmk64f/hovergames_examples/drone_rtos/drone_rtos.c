/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include <string.h>

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "Pixy2Cam.h"
#include "Pixy2_DSPI.h"
#include "common/mavlink.h"

#include "fsl_uart_freertos.h"
#include "fsl_uart.h"
#include "fsl_i2c.h"
#include "fsl_dspi.h"
#include "fsl_gpio.h"

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


/* Temperature sensor I2C definitions */
#define MLX90614_ADDR_7BIT (0x5AU)
#define I2C_BAUDRATE 50000
#define I2C_DATA_LENGTH 32U

/* RTOS task priorities */
#define controlLoop_PRIORITY (configMAX_PRIORITIES - 2)
#define tempLoop_PRIORITY (configMAX_PRIORITIES - 2)

///* UART instance and clock */
uart_rtos_handle_t mav_handle; // Mavlink UART
struct _uart_handle mav_t_handle;
uint8_t mav_background_buffer[32];

uart_rtos_handle_t bt_handle; // Bluetooth UART
struct _uart_handle bt_t_handle;
uint8_t bt_background_buffer[32];

uart_rtos_config_t mav_uart_config = {
    .baudrate    = 115200,
    .base        = MAV_UART,
    .parity      = kUART_ParityDisabled,
    .stopbits    = kUART_OneStopBit,
    .buffer      = mav_background_buffer,
    .buffer_size = sizeof(mav_background_buffer),
};

uart_rtos_config_t bt_uart_config = {
    .baudrate    = 115200,
    .base        = BT_UART,
    .parity      = kUART_ParityDisabled,
    .stopbits    = kUART_OneStopBit,
    .buffer      = bt_background_buffer,
    .buffer_size = sizeof(bt_background_buffer),
};

/* SPI config for pixy2 camera
    - provides handles for SPI instance and IRQ handler
    - use SPI0
    - IRQ hander is initialized in main
*/

Pixy2Cam cam; // This is the handle used by the Pixy2 Library

Pixy2_DSPI_Handle dspi_handle; // This is internal SPI handle passed to the interface layer

TimerHandle_t powerTimeout;

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

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* contrainBounds
 * returns `value` limited between `min` and `max`
 */
int16_t constrainBounds(int16_t value, int16_t min, int16_t max);

/* sendManual
 * sends manual command to FMU via MAVLink
 */
void sendManual(int16_t yaw, int16_t pitch, int16_t roll, int16_t thr);

/* getTemperature
 * get a floating point conversion of temperature from MLX90614
 */
float getTemperature();

/* timer handler for power timeout */
void vTimerTimeoutCallback(TimerHandle_t xTimer);

/* main control loop task*/
static void controlLoop(void *pvParameters);

/* temperature update task */
static void updateTemperature(void *pvParameters);

/* init function */
int init();

/*******************************************************************************
 * Code
 ******************************************************************************/

int16_t constrainBounds(int16_t value, int16_t min, int16_t max)
{
    if (value < min)
    {
        value = min;
    }
    else if (value > max)
    {
        value = max;
    }

    return value;
}

/* send manual control values to the FMU */
void sendManual(int16_t yaw, int16_t pitch, int16_t roll, int16_t thr)
{
    /* Make sure the input is within bounds */
    yaw   = constrainBounds(yaw, -1000, 1000);
    pitch = constrainBounds(pitch, -1000, 1000);
    roll  = constrainBounds(roll, -1000, 1000);
    thr   = constrainBounds(thr, 0, 1000);

    // Prepare manual control values to be sent
    int16_t b = 0; // Buttons are no used, set to 0

    // Prepare message
    mavlink_message_t msg;
    /* System ID and component ID for FMU are both 1 */
    mavlink_msg_manual_control_pack(1, 1, &msg, 1, roll, pitch, thr, yaw, b);

    // Populate UART buffer
    uint8_t buf[512];
    uint16_t length = mavlink_msg_to_send_buffer(buf, &msg);

    UART_RTOS_Send(&mav_handle, buf, length); // Send using RTOS UART
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

void vTimerTimeoutCallback(TimerHandle_t xTimer)
{
    DroneState.state = Drone_Idle;
}

static void controlLoop(void *pvParameters)
{
    // used to generate accurate periodic pause
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        Pixy2CCCBlock blocks[10];
        /* Get blocks form pixy2 camera */
        int block_num = Pixy2Cam_CCCgetBlocks(&cam, CCC_SIG1, 10, blocks);

        /*  If we can see a color block, start tracking */
        if (block_num > 0)
        {
            DroneState.state = Drone_Tracking;

            int position             = ((int)blocks[0].x - (320 / 2)) * 6;
            DroneState.lastDirection = (position > 0) ? Direction_Left : Direction_Right;

            position *= 1.1; /* small proportional gain */

            // 0 pitch and roll/ 50% throttle
            sendManual(position, 0, 0, 500);

            if (xTimerReset(powerTimeout, 100) != pdPASS)
            {
                PRINTF("ERROR starting timer\n");
            }
        }
        else
        {
            if (DroneState.state == Drone_Idle)
            {
                /* Stop the drone */
                sendManual(0, 0, 0, 0);
                // DroneState.state = Drone_Idle;
            }
            else
            {
                switch (DroneState.lastDirection)
                {
                    case Direction_Right:
                        sendManual(-1000, 0, 0, 500);
                        break;
                    case Direction_Left:
                        sendManual(1000, 0, 0, 500);
                        break;
                    default:
                        sendManual(0, 0, 0, 0);
                }
            }
        }
        vTaskDelayUntil(&xLastWakeTime, 10);
    }
}

static void updateTemperature(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        DroneState.temperature = getTemperature();

        char buffer[32];
        snprintf(buffer, 32, "T: %3.1f*C\n", DroneState.temperature);
        UART_RTOS_Send(&bt_handle, (uint8_t *)buffer, strlen(buffer));

        vTaskDelayUntil(&xLastWakeTime, 1000); // Send every 1s
        GPIO_PortToggle(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_GPIO_PIN);
    }
}

int init()
{
    mav_uart_config.srcclk = MAV_UART_CLK_FREQ;
    bt_uart_config.srcclk  = BT_UART_CLK_FREQ;

    UART_RTOS_Init(&mav_handle, &mav_t_handle, &mav_uart_config);
    UART_RTOS_Init(&bt_handle, &bt_t_handle, &bt_uart_config);

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

    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        1,
    };

    GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PIN, &led_config);
    GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, &led_config);
    GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PIN, &led_config);

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
    NVIC_SetPriority(BT_UART_IRQn, 5);
    NVIC_SetPriority(MAV_UART_IRQn, 5);
    init();

    powerTimeout = xTimerCreate("powerTimeout", 1000, pdTRUE, 0, vTimerTimeoutCallback);
    // DroneState.xStateSemaphore = xSemaphoreCreateMutex();

    if (powerTimeout == NULL)
    {
        PRINTF("Timer creation failed! \n");
        while (1)
            ;
    }

    if (xTaskCreate(controlLoop, "controlLoop", configMINIMAL_STACK_SIZE + 300, NULL, controlLoop_PRIORITY, NULL) !=
        pdPASS)
    {
        PRINTF("Task controlLoop creation failed!.\r\n");
        while (1)
            ;
    }

    if (xTaskCreate(updateTemperature, "updateTemperature", configMINIMAL_STACK_SIZE + 200, NULL, tempLoop_PRIORITY,
                    NULL) != pdPASS)
    {
        PRINTF("Task updateTemperature creation failed!.\r\n");
        while (1)
            ;
    }

    vTaskStartScheduler();
    for (;;)
        ;
}
