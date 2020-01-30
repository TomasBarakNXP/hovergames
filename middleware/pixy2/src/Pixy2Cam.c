/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Pixy2Cam.h"
#include "fsl_debug_console.h"

#define PIXY2_HEADER_LENGTH 4

/* The 16 bit start value sent at the beginning of a communication transmission
 * Since the values are sent in little endian order, the value is also exposed
 * split into high and low byte
 */

#define PIXY2_NO_CHECKSUM_SYNC 0xc1ae
#define PIXY2_NO_CHECKSUM_SYNC_B0 0xae
#define PIXY2_NO_CHECKSUM_SYNC_B1 0xc1

#define PIXY2_CHECKSUM_SYNC 0xc1af
#define PIXY2_CHECKSUM_SYNC_B0 0xaf
#define PIXY2_CHECKSUM_SYNC_B1 0xc1

/* When waiting for a sync, how many times should be try reading from the com
 * interface before giving up
 */
#define PIXY2_MAX_RETRIES 16

/* Request and response type codes */
#define PIXY2_TRES_RESULT 0x01
#define PIXY2_TRES_ERROR 0x03

#define PIXY2_TREQ_RESOLUTION 0x0c
#define PIXY2_TRES_RESOLUTION 0x0d

#define PIXY2_TREQ_VERSION 0x0e
#define PIXY2_TRES_VERSION 0x0f

#define PIXY2_TREQ_BRIGHTNESS 0x10
#define PIXY2_TREQ_SERVO 0x12
#define PIXY2_TREQ_LED 0x14
#define PIXY2_TREQ_LAMP 0x16
#define PIXY2_TREQ_FPS 0x18

#define PIXY2_TREQ_CCC 0x20
#define PIXY2_TRES_CCC 0x21

/* Local macro for setting the payload and buffer value by index */
#define SET_PAYLOAD(INDEX, VALUE) p->payload[INDEX] = VALUE
#define SET_BUFFER(INDEX, VALUE) p->buffer[INDEX] = VALUE

/* Joins two consecutive bytes in buffer to one 16 bit little endian value */
#define BUFF2UINT16(BUFFER, INDEX) (BUFFER[INDEX] | (BUFFER[INDEX + 1] << 8))

/* The actual function that retrieve the data from the camera are only called during Init */

/* Retreaves framerate from camera and saves it in internal variable */
static int Pixy2Cam_getFPS(Pixy2Cam *p);
/* Retreaves version from camera and saves it in internal variable */
static int Pixy2Cam_getVersion(Pixy2Cam *p);
/* Retreaves resolution from camera and saves it in internal variable */
static int Pixy2Cam_getResolution(Pixy2Cam *p);

/* Helper function used to calculate buffer checksum */
static uint16_t Pixy2Cam_calcCS(uint8_t *buf, uint8_t len);
/* Waits for the sync header value in response and returns if payload is checksummed or not */
static int Pixy2Cam_getSync(Pixy2Cam *p);
/* Receive a packet from camera */
static int Pixy2Cam_recvPacket(Pixy2Cam *p);
/* Send a packet to camera with given type and length. Payload must be set before using  SET_PAYLOAD*/
static int Pixy2Cam_sendPacket(Pixy2Cam *p, uint8_t type, uint8_t length);

int Pixy2Cam_Init(Pixy2Cam *p)
{
    int ret;

    // Initialize Communication interface
    p->com.init(p->com.data);

    // Setup the offset payload pointer
    p->payload = p->buffer + PIXY2_HEADER_LENGTH;

    // Get the actual values form the Camera
    ret = Pixy2Cam_getVersion(p);
    if (ret < 0)
    {
        return ret;
    }

    ret = Pixy2Cam_getResolution(p);
    if (ret < 0)
    {
        return ret;
    }

    ret = Pixy2Cam_getFPS(p);
    if (ret < 0)
    {
        return ret;
    }

    return PIXY2_OK;
}

int Pixy2Cam_SetCameraBrightness(Pixy2Cam *p, uint8_t brightness)
{
    int res;
    /* Payload SetCameraBrightness
     * Byte  | Description | Value(s)
     ********|*************|*********
     * 0     | Brightness  | 0-255
     */

    SET_PAYLOAD(0, brightness);

    res = Pixy2Cam_sendPacket(p, PIXY2_TREQ_BRIGHTNESS, 1);
    if (!res)
        return res;

    res = Pixy2Cam_recvPacket(p);
    if (res < 0)
        return res;

    switch (p->type)
    {
        case PIXY2_TRES_RESULT:

            return PIXY2_OK;

        case PIXY2_TRES_ERROR:
        default:
            return PIXY2_RESULT_ERROR;
    }
}

int Pixy2Cam_SetServos(Pixy2Cam *p, uint16_t s0, uint16_t s1)
{
    int res;
    /* Payload SetServos
     * Byte  | Description           | Value(s)
     ********|***********************|*********
     * 0-1   | 16-bit pan servo val  | 0-511
     * 2-3   | 16-bit tilt servo val | 0-511
     */

    SET_PAYLOAD(0, (s0 & 0xff));
    SET_PAYLOAD(1, ((s0 >> 8) & 0xff));

    SET_PAYLOAD(2, (s1 & 0xff));
    SET_PAYLOAD(3, ((s1 >> 8) & 0xff));

    res = Pixy2Cam_sendPacket(p, PIXY2_TREQ_SERVO, 4);
    if (!res)
        return res;

    res = Pixy2Cam_recvPacket(p);
    if (res < 0)
        return res;

    switch (p->type)
    {
        case PIXY2_TRES_RESULT:
            return PIXY2_OK;

        case PIXY2_TRES_ERROR:
        default:
            return PIXY2_RESULT_ERROR;
    }
}

int Pixy2Cam_SetLED(Pixy2Cam *p, uint8_t r, uint8_t g, uint8_t b)
{
    int res;
    /* Payload SetLED
     * Byte  | Description    | Value(s)
     ********|****************|*********
     * 0     | r - Red value  | 0-255
     * 1     | g - Red value  | 0-255
     * 2     | b - Red value  | 0-255
     */
    SET_PAYLOAD(0, r);
    SET_PAYLOAD(1, g);
    SET_PAYLOAD(2, b);

    res = Pixy2Cam_sendPacket(p, PIXY2_TREQ_LED, 3);
    if (!res)
        return res;

    res = Pixy2Cam_recvPacket(p);
    if (res < 0)
        return res;

    switch (p->type)
    {
        case PIXY2_TRES_RESULT:

            return PIXY2_OK;

        case PIXY2_TRES_ERROR:
        default:
            return PIXY2_RESULT_ERROR;
    }
}

int Pixy2Cam_SetLamp(Pixy2Cam *p, uint8_t upper, uint8_t lower)
{
    int res;

    /* Payload SetLamp
     * Byte  | Description                                    | Value(s)
     ********|************************************************|*****************
     * 0     | Upper - turn on two white LEDs along top edge  | 0(off) or 1 (on)
     * 1     | Lower - turn on all channels of lower RGB LED  | 0(off) or 1 (on)
     */
    SET_PAYLOAD(0, upper);
    SET_PAYLOAD(1, lower);

    res = Pixy2Cam_sendPacket(p, PIXY2_TREQ_LAMP, 2);
    if (!res)
        return res;

    res = Pixy2Cam_recvPacket(p);
    if (res < 0)
        return res;

    switch (p->type)
    {
        case PIXY2_TRES_RESULT:

            return PIXY2_OK;

        case PIXY2_TRES_ERROR:
        default:
            return PIXY2_RESULT_ERROR;
    }
}

int Pixy2Cam_CCCgetBlocks(Pixy2Cam *p, uint8_t sigmap, uint8_t maxBlocks, Pixy2CCCBlock *blocks)
{
    int res;
    int numberOfBlocks = 0;

    /* Payload CCCgetBlocks
     * Byte  | Description                           | Value(s)
     ********|***************************************|*******************
     * 0     | Sigmap - which signatures to receive  | 0(none) - 255(all)
     * 1     | Maximum number of blocks to return    | 0-255
     */
    SET_PAYLOAD(0, sigmap);
    SET_PAYLOAD(1, maxBlocks);

    res = Pixy2Cam_sendPacket(p, PIXY2_TREQ_CCC, 2);
    if (!res)
        return res;

    res = Pixy2Cam_recvPacket(p);
    if (res < 0)
        return res;

    switch (p->type)
    {
        case PIXY2_TRES_CCC:
            numberOfBlocks = p->length / sizeof(Pixy2CCCBlock);
            if (numberOfBlocks > maxBlocks)
            {
                return PIXY2_RESULT_ERROR;
            }
            for (int i = 0; i < numberOfBlocks; i++)
            {
                uint8_t *cBuf = p->buffer + i * sizeof(Pixy2CCCBlock);

                blocks[i].signature = BUFF2UINT16(cBuf, 0);
                blocks[i].x         = BUFF2UINT16(cBuf, 2);
                blocks[i].y         = BUFF2UINT16(cBuf, 4);
                blocks[i].width     = BUFF2UINT16(cBuf, 6);
                blocks[i].height    = BUFF2UINT16(cBuf, 8);
                blocks[i].angle     = BUFF2UINT16(cBuf, 10);
                ;
                blocks[i].index = cBuf[12];
                blocks[i].age   = cBuf[13];
            }

            return numberOfBlocks;

        case PIXY2_TRES_ERROR:
        default:
            return PIXY2_RESULT_ERROR;
    }
}

int Pixy2Cam_Close(Pixy2Cam *p)
{
    int ret;

    ret = p->com.close(p->com.data);
    if (!ret)
    {
        return ret;
    }
    return PIXY2_OK;
}

static int Pixy2Cam_getVersion(Pixy2Cam *p)
{
    int res;
    uint32_t typeLength;

    res = Pixy2Cam_sendPacket(p, PIXY2_TREQ_VERSION, 0);
    if (!res)
        return res;

    res = Pixy2Cam_recvPacket(p);
    if (res < 0)
        return res;

    switch (p->type)
    {
        case PIXY2_TRES_VERSION:
            p->version.hwVersion     = BUFF2UINT16(p->buffer, 0);
            p->version.firmwareMajor = p->buffer[2];
            p->version.firmwareMinor = p->buffer[3];
            p->version.firmwareBuild = BUFF2UINT16(p->buffer, 4);

            typeLength = p->length - 6;
            typeLength = (typeLength < 16) ? typeLength : 15; // Make sure we fit in our string buffer

            memcpy(p->version.firmwareType, p->buffer + 6, typeLength);
            p->version.firmwareType[typeLength] = '\0';

            return PIXY2_OK;
        case PIXY2_TRES_ERROR:
        default:
            return PIXY2_RESULT_ERROR;
    }
}

static int Pixy2Cam_getResolution(Pixy2Cam *p)
{
    int res;

    SET_PAYLOAD(0, 0);
    res = Pixy2Cam_sendPacket(p, PIXY2_TREQ_RESOLUTION, 1);
    if (!res)
        return res;

    res = Pixy2Cam_recvPacket(p);
    if (res < 0)
        return res;

    switch (p->type)
    {
        case PIXY2_TRES_RESOLUTION:

            p->width = BUFF2UINT16(p->buffer, 0);
            ;
            p->height = BUFF2UINT16(p->buffer, 2);
            ;
            return PIXY2_OK;

        case PIXY2_TRES_ERROR:
        default:
            return PIXY2_RESULT_ERROR;
    }
}

static int Pixy2Cam_getFPS(Pixy2Cam *p)
{
    int res;

    res = Pixy2Cam_sendPacket(p, PIXY2_TREQ_FPS, 0);
    if (!res)
        return res;

    res = Pixy2Cam_recvPacket(p);
    if (res < 0)
        return res;

    switch (p->type)
    {
        case PIXY2_TRES_RESULT:
            p->fps = (p->buffer[0]) | (p->buffer[1] >> 8) | (p->buffer[2] >> 16) | (p->buffer[3] >> 24);

            return p->fps;

        case PIXY2_TRES_ERROR:
        default:
            return PIXY2_RESULT_ERROR;
    }
}

static uint16_t Pixy2Cam_calcCS(uint8_t *buf, uint8_t len)
{
    uint16_t cs = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        cs += buf[i];
    }

    return cs;
}

static int Pixy2Cam_getSync(Pixy2Cam *p)
{
    int ret = 0;

    uint8_t read_buf[1] = {0}; // We read one byte at a time
    uint8_t lastRead, try_counter = 0;

    ret = p->com.recv(p->com.data, read_buf, 1); // Read one byte
    if (ret < 0)
        return ret;

    lastRead = read_buf[0]; // Get last read byte

    while (1)
    {
        ret = p->com.recv(p->com.data, read_buf, 1); // Read one byte
        if (ret < 0)
            return ret;

        // Since data is little endian, the most recent byte is more significant
        if (read_buf[0] == PIXY2_CHECKSUM_SYNC_B1)
        { // The more significant byte is same for both cases
            if (lastRead == PIXY2_NO_CHECKSUM_SYNC_B0)
            {
                return PIXY2_SYNC_OK_NOCS;
            }
            if (lastRead == PIXY2_CHECKSUM_SYNC_B0)
            {
                return PIXY2_SYNC_OK_CS;
            }
        }
        lastRead = read_buf[0];
        try_counter++;

        if (try_counter > PIXY2_MAX_RETRIES)
        {
            return PIXY2_SYNC_ERROR;
        }
    }
}

static int Pixy2Cam_recvPacket(Pixy2Cam *p)
{
    int res;

    res = Pixy2Cam_getSync(p);

    bool haveCS = false;
    int16_t cs;

    switch (res)
    {
        case PIXY2_SYNC_OK_CS:
            haveCS = true;
            break;
        case PIXY2_SYNC_OK_NOCS:
            haveCS = false;
            break;
        case PIXY2_SYNC_ERROR:
        default:
            return res;
    }

    /* If we have a checksum, header is 4 bytes long, otherwise only 2 bytes */
    res = p->com.recv(p->com.data, p->buffer, (haveCS) ? (4) : (2));
    if (res < 0)
        return res;

    p->type   = p->buffer[0];
    p->length = p->buffer[1];

    if (p->length == 0) // There is no payload
        return 0;

    if (haveCS) // We also have a checksum
        cs = BUFF2UINT16(p->buffer, 2);

    res = p->com.recv(p->com.data, p->buffer, p->length);
    if (res < 0)
        return res;

    if (haveCS && (Pixy2Cam_calcCS(p->buffer, p->length) != cs))
    {
        return PIXY2_CS_ERROR;
    }

    return PIXY2_OK;
}

static int Pixy2Cam_sendPacket(Pixy2Cam *p, uint8_t type, uint8_t length)
{
    SET_BUFFER(0, PIXY2_NO_CHECKSUM_SYNC_B0);
    SET_BUFFER(1, PIXY2_NO_CHECKSUM_SYNC_B1);
    SET_BUFFER(2, type);
    SET_BUFFER(3, length);

    return p->com.send(p->com.data, p->buffer, length + PIXY2_HEADER_LENGTH);
}
