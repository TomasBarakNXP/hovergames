/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
#define DEMO_UART UART3
#define DEMO_UART_CLKSRC UART3_CLK_SRC
#define DEMO_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define DEMO_UART_IRQn UART3_RX_TX_IRQn
#define DEMO_UART_IRQHandler UART3_RX_TX_IRQHandler
/*! @brief Ring buffer size (Unit: Byte). */
#define DEMO_RING_BUFFER_SIZE 512

//#define MAVLINK_COMM_NUM_BUFFERS 1
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*
  Ring buffer for data input and output, in this example, input data are saved
  to ring buffer in IRQ handler. The main function polls the ring buffer status,
  if there are new data, then send them out.
  Ring buffer full: (((rxIndex + 1) % DEMO_RING_BUFFER_SIZE) == txIndex)
  Ring buffer empty: (rxIndex == txIndex)
*/
uint8_t demoRingBuffer[DEMO_RING_BUFFER_SIZE];
volatile uint16_t txIndex; /* Index of the data to send out. */
volatile uint16_t rxIndex; /* Index of the memory to save new arrived data. */

/*******************************************************************************
 * Code
 ******************************************************************************/

void DEMO_UART_IRQHandler(void)
{
    uint8_t data;

    /* If new data arrived. */
    if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(DEMO_UART))
    {
        data = UART_ReadByte(DEMO_UART);

        /* If ring buffer is not full, add data to ring buffer. */
        if (((rxIndex + 1) % DEMO_RING_BUFFER_SIZE) != txIndex)
        {
            demoRingBuffer[rxIndex] = data;
            rxIndex++;
            rxIndex %= DEMO_RING_BUFFER_SIZE;
        }
    }
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitPins();
    uart_config_t config;

    mavlink_global_position_int_t global_position;
    mavlink_status_t status;
    mavlink_sys_status_t sys_status;
    mavlink_message_t msg;
    uint32_t msgReceived;
    uint8_t visible_sats;

    // BOARD_InitPins();
    // BOARD_BootClockRUN();

    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUART_ParityDisabled;
     * config.stopBitCount = kUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 1;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200;
    config.enableTx     = true;
    config.enableRx     = true;

    UART_Init(DEMO_UART, &config, DEMO_UART_CLK_FREQ);

    /* Enable RX interrupt. */
    UART_EnableInterrupts(DEMO_UART, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
    EnableIRQ(DEMO_UART_IRQn);
    

    while (1)
    {
        /* receiving a message via UART*/
        while ((kUART_TxDataRegEmptyFlag & UART_GetStatusFlags(DEMO_UART)) && (rxIndex != txIndex))
        {
            msgReceived = mavlink_parse_char(MAVLINK_COMM_0, demoRingBuffer[txIndex], &msg, &status);
            // PRINTF("%x",demoRingBuffer[txIndex]);
            txIndex++;
            txIndex %= DEMO_RING_BUFFER_SIZE;
        }

        if (msgReceived)
        {
            // if (msg.magic == MAVLINK_STX_MAVLINK1)
            //{
            //    PRINTF("This is a MAVLink 1 message\r\n");
            // PRINTF("Received message with ID %d, sequence: %d from component %d of system %d, status: %d \r\n",
            // msg.msgid, msg.seq, msg.compid, msg.sysid, status);

            //}
            // else
            //{
            //    PRINTF("This is a MAVLink 2 message\r\n");
            // PRINTF("Received message with ID %d, sequence: %d from component %d of system %d, status: %d \r\n",
            // msg.msgid, msg.seq, msg.compid, msg.sysid, status); mavlink_msg_sys_status_decode(&msg, &sys_status);
            // PRINTF("Voltage = %.2f V \r\n", (float)(sys_status.voltage_battery) / 1000);
            //}
            PRINTF("Received message with ID %d, sequence: %d from component %d of system %d, len: %d, status: %d \r\n",
                   msg.msgid, msg.seq, msg.compid, msg.sysid, msg.len, status.parse_error);
            // if (msg.msgid != 0 )
            //{
            // PRINTF("Received message with ID %d, sequence: %d from component %d of system %d, status: %d \r\n",
            // msg.msgid, msg.seq, msg.compid, msg.sysid, status);
            //}

            switch (msg.msgid)
            {
                case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    // Get all fields in payload (into global_position)
                    PRINTF("This is a MAVLINK_MSG_ID_HEARTBEAT\r\n");
                }
                break;
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: // ID for GLOBAL_POSITION_INT
                {
                    PRINTF("This is a MAVLINK_MSG_ID_GLOBAL_POSITION_INT\r\n");
                    // Get all fields in payload (into global_position)
                    mavlink_msg_global_position_int_decode(&msg, &global_position);
                    PRINTF("time: %d", global_position.time_boot_ms);
                }
                break;
                case MAVLINK_MSG_ID_GPS_STATUS:
                {
                    PRINTF("This is a MAVLINK_MSG_ID_GPS_STATUS\r\n");
                    // Get just one field from payload
                    visible_sats = mavlink_msg_gps_status_get_satellites_visible(&msg);
                }
                break;
                case MAVLINK_MSG_ID_SYS_STATUS: // #1: SYS_STATUS
                {
                    PRINTF("This is a MAVLINK_MSG_ID_SYS_STATUS\r\n");
                    mavlink_msg_sys_status_decode(&msg, &sys_status);
                    PRINTF("Voltage = %.2f V \r\n", (float)(sys_status.voltage_battery) / 1000);
                }
                break;
                case MAVLINK_MSG_ID_HIGHRES_IMU:
                {
                    PRINTF("This is a MAVLINK_MSG_ID_HIGHRES_IMU\r\n");
                }
                break;
                default:
                    break;
            }
        }
        msgReceived = 0;
    }
}
