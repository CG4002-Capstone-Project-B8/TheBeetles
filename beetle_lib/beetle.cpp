#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "beetle.h"

#define PACKET_SIZE 20
#define NUM_SENSOR_VALUES 3

#define PACKET_TYPE_SHIFT 6
#define SEQNUM_SHIFT 5
#define PLAYER_ID_SHIFT 4
#define DEVICE_ID_SHIFT 2
#define SEND_SHOT_SHIFT 1
#define RECEIVE_SHOT_SHIFT 0

#define PACKET_TYPE_MASK (0x03 << PACKET_TYPE_SHIFT)
#define SEQNUM_MASK (0x01 << SEQNUM_SHIFT)
#define PLAYER_ID_MASK (0x01 << PLAYER_ID_SHIFT)
#define DEVICE_ID_MASK (0x03 << DEVICE_ID_SHIFT)
#define SEND_SHOT_MASK (0x01 << SEND_SHOT_SHIFT)
#define RECEIVE_SHOT_MASK (0x01 << RECEIVE_SHOT_SHIFT)

/**
 * @brief Serializes a packet into a char array
 * 
 * @param buffer buffer which will contain serialized data of the packet
 * @param packet_to_send packet to be serialized
 */
void serialize(char *buffer, TPacket *packet_to_send)
{
    memcpy(buffer, packet_to_send, PACKET_SIZE);
}

/**
 * @brief Deserializes data in a char array to form a packet
 * 
 * @param packet_recv received packet in which data from the buffer is to be written into
 * @param buffer buffer containing data to be deserialized into the packet
 */
void deserialize(TPacket *packet_recv, char *buffer)
{
    memcpy(packet_recv, buffer, PACKET_SIZE);
}

/**
 * @brief Determines if a packet_type is valid, that is it is contained in the TPacketType enumeration
 * 
 * @param packet_type character representing the type of packet received
 * @return true if packet type is valid
 * @return false otherwise
 */
bool isValidType(unsigned char packet_type) {
    for(int i = 0; i < 4; i++) {
        if (VALID_PACKET_TYPES[i] == packet_type) { return true; }
    }
    return false;
}

/**
 * @brief Checks if a packet received is a valid packet. Verifies that the packet
 *        is not a NACK. Verifies that the checksum generated is correct and ensures
 *        the sequence number is as expected.
 * 
 * @param packet_recv The packet received by the Beetle
 * @param current_seq_num Current sequence number expected
 * @return true if packet is not a NACK, has a valid sequence number
 * @return false otherwise
 */
bool isValidPacket(TPacket *packet_recv, int current_seq_num)
{
    unsigned char details = packet_recv->details;
    unsigned char packet_type = details >> PACKET_TYPE_SHIFT;

    if(!isValidType(packet_type)) {
        return false;
    }
    
    if (packet_type == PACKET_TYPE_NACK) {
        return false;
    }

    unsigned char seqnum = (details & SEQNUM_MASK) >> SEQNUM_SHIFT;

    if (packet_type == PACKET_TYPE_ACK && seqnum != current_seq_num) {
        return false;
    }

    return true;
}

/**
 * @brief Generates a checksum from the contents of a packet by XORing each 
 *        byte.
 * 
 * @param packet packet whose checksum is to be obtained
 * @return the generated checksum
 */
char getChecksum(TPacket *packet)
{
    unsigned char checksum = 0;
    checksum ^= packet->details;

    unsigned char *ptr = (unsigned char *)packet->accel_data;
    for(int i = 0; i < sizeof(packet->accel_data); i++) {
        checksum ^= ptr[i];
    }

    ptr = (unsigned char *)packet->gyro_data;

    for(int i = 0; i < sizeof(packet->gyro_data); i++) 
    {
        checksum ^= ptr[i];
    }
    
    return checksum;
}
