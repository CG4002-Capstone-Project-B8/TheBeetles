#include <stdint.h>

/*
 * total size of a packet = 1 + 6 + 12 + 1 = 20 bytes
 */
typedef struct __attribute__((__packed__)) packet
{
    unsigned char details;
    int16_t gyro_data[3];
    float accel_data[3];
    unsigned char checksum;
} TPacket;
