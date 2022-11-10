#include "constants.h"
#include "packet.h"

void serialize(char *buffer, TPacket *packet_to_send);
void deserialize(TPacket *packet_recv, char *buffer);
bool isValidType(char packet_type);
bool isValidPacket(TPacket *packet_recv, int current_seq_num);
char getChecksum(TPacket *packet);