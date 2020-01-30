/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __PIXY2CAM_H
#define __PIXY2CAM_H

#include <stdint.h>

#define PIXY2_BUFFERSIZE (256 + 4)

#define CCC_SIG1 0x01
#define CCC_SIG2 0x02
#define CCC_SIG3 0x04
#define CCC_SIG4 0x08

#define CCC_SIG5 0x10
#define CCC_SIG6 0x20
#define CCC_SIG7 0x40
#define CCC_COLOR_CODES 0x80

#define CCC_SIG_ALL 0xff

/* Most common values returned by functions
 * Zero or positive indicate success
 * Negative indicate failure
 */
typedef enum
{
    PIXY2_OK           = 0, // Asserts a successful operation
    PIXY2_SYNC_OK_NOCS = 1, // Asserts a successful SYNC detection with no checksum
    PIXY2_SYNC_OK_CS   = 2, // Asserts a successful SYNC with a checksum

    PIXY2_SYNC_ERROR   = -1, // No Sync was detected in the given tries
    PIXY2_CS_ERROR     = -2, // The frame checksum does not match
    PIXY2_RESULT_ERROR = -3  // The result frame returned an error
} Pixy2SyncReturn;

/* Communication interface
 *  This structure contains pointers to communication functions for a arbitrary serial interface
 *  Before initializing the Pixy2Cam handle, the interface should be bound to it
 */
typedef struct __Pixy2ComInterface
{
    void *data;               // Interface data, passed to the interface function when called
    int (*init)(void *data);  // Initialization function, called during Pixy2Cam initialization
    int (*close)(void *data); // Deinitialize interface
    int (*recv)(void *data, uint8_t *rx_buffer, uint32_t len); // Receive data
    int (*send)(void *data, uint8_t *tx_buffer, uint32_t len); // Send data
} Pixy2ComInterface;

/* Struct representing version information
 *
 */
typedef struct _Pixy2Version
{
    uint16_t hwVersion;     // Hardware version
    uint8_t firmwareMajor;  // firmware (Major)
    uint8_t firmwareMinor;  // firmware (Minor)
    uint16_t firmwareBuild; // firmware build
    char firmwareType[16];  // Firmware type (human readable)

} Pixy2Version;

/* Pixy2Cam Handle
 *
 */
typedef struct _Pixy2Cam
{
    uint16_t width;  // Camera resolution width
    uint16_t height; // Camera resolution height;
    uint32_t fps;    // Camera fps;
    Pixy2Version version;

    uint8_t type;                     // Type of the received packet
    uint8_t length;                   // Length of the received packet
    uint8_t buffer[PIXY2_BUFFERSIZE]; // Buffer used for sending/receiving
    uint8_t *payload;                 // Offset to the payload for sending

    Pixy2ComInterface com; // Communication interface
} Pixy2Cam;

/* Struct representing one CCC Block as defined in documentation:
 * https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:porting_guide#color-connected-components-packet-reference
 */
typedef struct _Pixy2CCCBlock
{
    uint16_t signature; // Block Signature
    uint16_t x;         // X position (center)
    uint16_t y;         // Y position (center)
    uint16_t width;     // Block width in pixels
    uint16_t height;    // Block height in pixels
    int16_t angle;      // Angle in degrees
    uint8_t index;      // Tracking index
    uint8_t age;        // Number of frames this block has been tracked
} Pixy2CCCBlock;

/* Function: Pixy2Cam_Init
 * -----------------------
 * Initialize the Pixy2Cam handle and communication interface
 * The communication interface MUST be bound to the link interface  before initialization!
 *
 * p: pointer to the camera handle
 *
 * returns: PIXY2_OK on success or error
 */
int Pixy2Cam_Init(Pixy2Cam *p);

/* Function: Pixy2Cam_SetCameraBrightness
 * -----------------------
 * Sets the camera brightness
 *
 * p: pointer to the camera handle
 * brightness: values 0-255 of the desired brightness
 *
 * returns: PIXY2_OK on success or error
 */
int Pixy2Cam_SetCameraBrightness(Pixy2Cam *p, uint8_t brightness);

/* Function: Pixy2Cam_SetServos
 * -----------------------
 * Sets the position values for the pan and tilt servos
 *
 * p: pointer to the camera handle
 * s0: Value for the pan servo  0-511
 * s1: Value for the tilt servo 0-511
 *
 * returns: PIXY2_OK on success or error
 */
int Pixy2Cam_SetServos(Pixy2Cam *p, uint16_t s0, uint16_t s1);

/* Function: Pixy2Cam_SetLED
 * -----------------------
 * Sets the RGB LED to a given color
 *
 * p: pointer to the camera handle
 * r: Red value 0-255
 * g: Green value 0-255
 * b: Blue value 0-255
 *
 * returns: PIXY2_OK on success or error
 */
int Pixy2Cam_SetLED(Pixy2Cam *p, uint8_t r, uint8_t g, uint8_t b);

/* Function: Pixy2Cam_SetLamp
 * -----------------------
 * Turns the LED lamps on the camera on and off
 *
 * p: pointer to the camera handle
 * upper: controls the two white LEDs at the top of the Pixel2 0 (off) 1 (on)
 * lower: controls the RGB LED setting it to white 0 (off) 1 (on)
 *
 * returns: PIXY2_OK on success or error
 */
int Pixy2Cam_SetLamp(Pixy2Cam *p, uint8_t upper, uint8_t lower);

/* Function: Pixy2Cam_CCCgetBlocks
 * -----------------------
 * Gets the detected blocks ordered by their size (larges first)
 *
 * p: pointer to the camera handle
 * sigmap: The map signature as configured in the camera eg. CCC_SIG1, CCC_SIG_ALL
 * maxBlocks: maximum number of blocks to retrieve
 * blocks: pointer to an array of Pixy2CCCBlock blocks at least maxBlocks long - the block data is saved here
 *
 * returns: 0-maxBlocks as the number of returned blocks or negative error
 */
int Pixy2Cam_CCCgetBlocks(Pixy2Cam *p, uint8_t sigmap, uint8_t maxBlocks, Pixy2CCCBlock *blocks);

/* Function: Pixy2Cam_Close
 * -----------------------
 * Closes the camera communication and frees any resources
 *
 * p: pointer to the camera handle
 *
 * returns: PIXY2_OK on success or error
 */
int Pixy2Cam_Close(Pixy2Cam *p);

#endif
