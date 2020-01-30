/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*  SDK Included Files */
//#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"

#include "mlx90614.h"

#define EXAMPLE_I2C_MASTER_CLK_FREQ CLOCK_GetFreq(I2C0_CLK_SRC)
#define EXAMPLE_I2C_MASTER ((I2C_Type *)I2C0)

#define MLX90614_ADDR_7BIT (0x5AU)
#define I2C_BAUDRATE 50000
#define I2C_DATA_LENGTH 32U

int MLX90614_init()
{
    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    I2C_MasterInit(EXAMPLE_I2C_MASTER, &masterConfig, EXAMPLE_I2C_MASTER_CLK_FREQ);

    return 0;
}

float MLX90614_getT()
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

    I2C_MasterTransferBlocking(EXAMPLE_I2C_MASTER, &masterXfer);

    masterXfer.slaveAddress   = MLX90614_ADDR_7BIT;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = data_buff;
    masterXfer.dataSize       = 3;
    masterXfer.flags          = kI2C_TransferRepeatedStartFlag; // kI2C_TransferDefaultFlag;

    I2C_MasterTransferBlocking(EXAMPLE_I2C_MASTER, &masterXfer);

    uint16_t temp = ((((uint16_t)data_buff[1]) << 8) | data_buff[0]);

    return temp * 0.02 - 273.15;
}
