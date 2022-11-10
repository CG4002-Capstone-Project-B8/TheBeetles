/*
 *  This file containts packet types
 *  (packet_type, seq num, player_id, device_id)
 */
 // Packet types
typedef enum packet_type
{
	PACKET_TYPE_HANDSHAKE = 0,
	PACKET_TYPE_ACK = 1,
	PACKET_TYPE_NACK = 2,
	PACKET_TYPE_DATA = 3
} TPacketType;

const TPacketType VALID_PACKET_TYPES[] = {
	PACKET_TYPE_HANDSHAKE,
	PACKET_TYPE_ACK,
	PACKET_TYPE_NACK,
	PACKET_TYPE_DATA
};

typedef enum beetle_state
{
	WAIT_FOR_HANDSHAKE = 0,
	WAIT_FOR_HANDSHAKE_ACK = 1,
	WAIT_FOR_ACK = 2,
	SEND_DATA = 3
} TBeetleState;
