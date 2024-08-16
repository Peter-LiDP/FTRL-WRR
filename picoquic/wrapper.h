#ifndef WRAPPER_H
#define WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

void LINUCB_initialize(int numActions); 
void LinUCB_passACK(int path_id, uint64_t highestAckedPacket, int range, uint64_t ack_received_time);
void pass_features(uint64_t cwnd0, uint64_t InP0, uint64_t RTT0, uint64_t cwnd1, uint64_t InP1, uint64_t RTT1, uint64_t swnd); 
void mapPacketNumberToCurrentTimeStep(int path_id, uint64_t packetNumber, int current_timestep, double bytes, uint64_t current_wall_time, double gamma_s);
int wait_or_transmit(uint64_t cwnd0, uint64_t InP0, uint64_t RTT0, uint64_t cwnd1, uint64_t InP1, uint64_t RTT1, uint64_t swnd); 
void updateTimestep(int current_timestep, uint64_t start_time);
void manual_update();

#ifdef __cplusplus
}
#endif

#endif // WRAPPER_H

