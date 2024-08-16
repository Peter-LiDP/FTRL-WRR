#ifndef BLEST_H
#define BLEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct PacketStreamOffsetMap {
    uint64_t packet_number;
    uint64_t path_id;
    uint64_t bytes; 
    bool processed;
    uint64_t stream_id; 
    uint64_t offset;
    struct PacketStreamOffsetMap* next;
} PacketStreamOffsetMap;

void initialize(); 
void passACK_BLEST(uint64_t path_id, uint64_t highestAckedPacket, int range);
extern PacketStreamOffsetMap* mapping_head;
void check_for_missing_offsets();

PacketStreamOffsetMap* find_mapping(uint64_t packet_number, uint64_t path_id);

#ifdef __cplusplus
}
#endif

#endif // BLEST_H

