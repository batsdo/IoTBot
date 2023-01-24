#ifndef IOTBOT_SHIELD__UART_PROTOCOL_H_
#define IOTBOT_SHIELD__UART_PROTOCOL_H_

// TODO: Manuel Kreutz: change enums to enum classes
// TODO: Manuel Kreutz: place header in util folder
// TODO: Manuel Kreutz: separate headers and source files in separated folders
// TODO: Manuel Kreutz: place main functions outside of the source files in separated main.cpp


// ************************************* Commands *************************************

enum ESerialCommand
{
    // Operating commands
    CMD_ENABLE              = 0x01,
    CMD_DISABLE             = 0x02,
    CMD_SET_TIMEOUT         = 0x03,
    CMD_SET_PWM_MAX         = 0x04,
    CMD_SEND_RPM            = 0x05,
    CMD_SEND_POS            = 0x06,
    CMD_INVERT_ENC          = 0x07,
    CMD_LOW_VOLTAGE_CHECK   = 0x08,
    CMD_STALL_CHECK         = 0x09,
    CMD_IMU_RAW_DATA        = 0x0A,
    CMD_FUSE_IMU_WEIGHT     = 0x0B,
    CMD_UART_TIMEOUT        = 0x0C,
    CMD_IMU_CALIBRATE       = 0x0D,

    CMD_SET_PWM             = 0x10,
    CMD_SET_RPM             = 0x11,
    CMD_FREQ                = 0x12,
    CMD_SYNC                = 0x13,

    // Closed / Open loop controller parameters
    CMD_CTL_KP              = 0x20,
    CMD_CTL_KI              = 0x21,
    CMD_CTL_KD              = 0x22,
    CMD_CTL_ANTI_WINDUP     = 0x23,
    CMD_CTL_INPUT_FILTER    = 0x24,
    CMD_CTL_ENC_LOWPASS     = 0x25,

    // Platform parameters
    CMD_GEAR_RATIO          = 0x30, // Ratio of motor gears
    CMD_TICKS_PER_REV       = 0x31, // Ticks per motor revolution ((raising + falling edges) x 2 channels)

    CMD_AUX1                = 0x40, // Enable / Disable auxilary power output 1
    CMD_AUX2                = 0x41, // Enable / Disable auxilary power output 2

    CMD_LIGHTS_OFF          = 0x42, // Switch lights off
    CMD_LIGHTS_DIM_LIGHT    = 0x43, // Dimmed headlight
    CMD_LIGHTS_HIGH_BEAM    = 0x44, // High beam headlight
    CMD_LIGHTS_FLASH_ALL    = 0x45, // Flash lights
    CMD_LIGHTS_FLASH_LEFT   = 0x46, // Flash lights to the left
    CMD_LIGHTS_FLASH_RIGHT  = 0x47, // Flash lights to the right
    CMD_LIGHTS_PULSATION    = 0x48, // Pulsation
    CMD_LIGHTS_ROTATION     = 0x49, // Rotation light, e.g. police light in blue
    CMD_LIGHTS_RUNNING      = 0x4A  // Running light
};


// ************************************* Request (IOT to Shield) *************************************
/*
 *    1 Byte    |  1 Byte   |     8 Bytes     |  1 Byte
 *  Start Byte  |  Command  |     Payload     |   CRC
 *     0xFF                       e.g. RPM
 */

#define SERIAL_REQUEST_BUFFER_LEN   11
#define SERIAL_REQUEST_PAYLOAD_LEN  4

// Start byte for request and CRC byte
#define SERIAL_REQUEST_START_VALUE  0xFF
#define SERIAL_REQUEST_CRC_VALUE    0xEE

enum ESerialRequestOffset
{
    REQ_START   = 0,    // 1 byte
    REQ_COMMAND = 1,    // 1 byte
    REQ_PAYLOAD = 2,    // 8 bytes
    REQ_CRC     = 10    // 1 byte
};

typedef struct
{
    //int8_t  start;
    int8_t  command;
    int16_t payload[SERIAL_REQUEST_PAYLOAD_LEN];
    //int8_t  crc;

} SerialRequestData;


// ************************************* Response (Shield to IOT) *************************************
/*
 *  1 Byte   |        8 Bytes         |        12 Bytes       |   8 Bytes    |  2 Bytes  |    1 Byte     |  1 Byte
 *  Command  |          RPM           |      IMU raw data     |   ToF data   |  Voltage  |  Temperature  |   CRC
 *           |       2 per wheel      |   2 per acc x, y, z   |  2 per scan  |
 *           |     fl, fr, rl, rr     |   2 per gyr x, y, z   |
 */

#define SERIAL_RESPONSE_BUFFER_LEN  32
#define SERIAL_RESPONSE_RPM_LEN     4
#define SERIAL_RESPONSE_IMU_LEN     6
#define SERIAL_RESPONSE_TOF_LEN     4

enum ESerialResponseOffset
{
    RES_COMMAND     = 0,    // 1 byte
    RES_RPM         = 1,    // 8 bytes
    RES_IMU         = 9,    // 12 bytes
    RES_TOF         = 21,   // 8 bytes
    RES_VOLTAGE     = 29,   // 2 bytes
    RES_TEMPERATURE = 31,   // 1 byte
    RES_CRC         = 32    // 1 byte
};

typedef struct
{
    int8_t  command;
    int16_t rpm[SERIAL_RESPONSE_RPM_LEN];
    int16_t imu[SERIAL_RESPONSE_IMU_LEN];
    int16_t tof[SERIAL_RESPONSE_TOF_LEN];
    int16_t voltage;
    int8_t  temperature;
    //int8_t  crc;

} SerialResponseData;


#endif // IOTBOT_SHIELD__UART_PROTOCOL_H_
