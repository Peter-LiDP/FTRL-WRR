#ifndef WRAPPER_H
#define WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int value;
    int count;
    int t;
} ActionResult;
  
void FTRL_initialize(int numActions); 
void FTRL_drawAction(ActionResult* result);
void FTRL_passACK(int path_id, uint64_t highestAckedPacket, int range, uint64_t ack_received_time);
void mapPacketNumberToCurrentTimeStep(int path_id, uint64_t packetNumber, int current_timestep, double bytes, uint64_t current_wall_time);
void FTRL_second_update();
void updateTimestep(int current_timestep, uint64_t start_time);
void updateBandwidth(int current_timestep, int path_id, uint64_t bandwidth_high);
void ADWIN2_passBW(uint64_t bw0, uint64_t bw1);

#ifdef __cplusplus
}
#endif

#endif // WRAPPER_H

