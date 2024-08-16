#include "linucb.h"
#include "wrapper.h"
#include <memory>
#include <vector> 
#include <thread>
#include <mutex>
#include <condition_variable> 
#include <queue>
#include <utility>
#include <atomic>
#include <queue>
#include <map>
#include <iostream>
#include <random>
#include "globals.h"
#include <fstream>
#include <algorithm>

static std::unique_ptr<LINUCB> LINUCB_instance; 
static std::queue<std::pair<double, int>> lossQueue;
static std::queue<std::pair<int, int>> actionQueue;
static std::queue<std::tuple<int, uint64_t, int, uint64_t>> ACKQueue;

std::map<int, std::vector<double>> feature_timestep_map;

const size_t NUM_PATHS = 2;
std::vector<std::queue<uint64_t>> path_updating_queues(NUM_PATHS);
std::vector<double> global_harmonic_means(NUM_PATHS, 0.0); 
std::vector<uint64_t> last_update_time(NUM_PATHS, 0); 

int current_processing_timestep0;
int current_processing_timestep1;
int starting_check;
double pt;
double pw;

static std::mutex UCBMutex;
static std::condition_variable ACKCondition;
static std::condition_variable queueCondition;
static std::condition_variable actionCondition;

static std::atomic<bool> processingACK{true};
static std::atomic<bool> processingLoss{true};
static std::thread lossProcessingThread;
static std::thread ACKProcessingThread;

void processLoss() {
    while (processingLoss) {
        std::unique_lock<std::mutex> lock(UCBMutex);
        queueCondition.wait(lock, []{ return !lossQueue.empty() || !processingLoss; });

        while (!lossQueue.empty() && LINUCB_instance) {
            auto loss_info = lossQueue.front();
            lossQueue.pop();
            double loss = std::get<0>(loss_info);
            int timestep = std::get<1>(loss_info);
            std::vector<double> feature_vector(6);
            
            auto it = feature_timestep_map.find(timestep);
            if (it != feature_timestep_map.end()) {
                feature_vector = it->second;
            }
            LINUCB_instance->update(loss, timestep, feature_vector);
            std::vector<double> q = LINUCB_instance->calculate_q();
            LINUCB_instance->q_t = q[0];
            LINUCB_instance->q_w = q[1];
            double average_e = LINUCB_instance->e/LINUCB_instance->count_t;
            double average_f = LINUCB_instance->f/LINUCB_instance->count_w;
            LINUCB_instance->p_indiff_t = average_e / (average_e + average_f);
            LINUCB_instance->p_indiff_w = average_f / (average_e + average_f);
            if (q[0] >= 0.7) {
                pt = 0.9;
            }
            else if (q[0] >= 0.3) {
                pt = LINUCB_instance->p_indiff_t;
            }
            else {
                pt = 0.1;
            }
                
            if (q[1] >= 0.7) {
                pw = 0.9;
            }
            else if (q[1] >= 0.3) {
                pw = LINUCB_instance->p_indiff_w;
            }
            else {
                pw = 0.1;
            }   

        }
    }
}


void processACK() {
    while (processingACK) {
        std::unique_lock<std::mutex> lock(UCBMutex);
        ACKCondition.wait(lock, []{ return !ACKQueue.empty() || !processingACK; });

        while (!ACKQueue.empty() && LINUCB_instance) {
            auto ack = ACKQueue.front();
            ACKQueue.pop();
            lock.unlock();

            int path_id = std::get<0>(ack);
            uint64_t highestAckedPacket = std::get<1>(ack);
            int range = std::get<2>(ack);
            uint64_t ack_received_time = std::get<3>(ack);

            std::map<uint64_t, std::tuple<int, double, bool, uint64_t, double>>& targetMap = (path_id == 0) ? LINUCB_instance->packetNumbertoTimestepMap0 : LINUCB_instance->packetNumbertoTimestepMap1;
            uint64_t startPacketNumber = highestAckedPacket - range + 1;

            for (uint64_t p = startPacketNumber; p <= highestAckedPacket; ++p) {
                auto it = targetMap.find(p);
                auto last_packet = targetMap.find(highestAckedPacket);
                if (it != targetMap.end() && last_packet != targetMap.end() && !std::get<2>(it->second)) {
                    int timestep = std::get<0>(it->second);
                    uint64_t current_packet_sent_time = std::get<3>(it->second);
                    uint64_t last_packet_sent_time = std::get<3>(last_packet->second);
                    if (path_id == 0) {
                        if(current_processing_timestep0 != timestep) {
                            for(int i = current_processing_timestep0; i < timestep; i++) {
                                if (LINUCB_instance->TimestepMap.find(i) != LINUCB_instance->TimestepMap.end()) {
                                    auto& timestepData0 = LINUCB_instance->TimestepMap[i];
                                    std::get<1>(timestepData0) = true;
                                    current_processing_timestep0 = i + 1;
                                }
                            }
                        }
                    }
                    if (path_id == 1) {
                        if(current_processing_timestep1 != timestep) {
                            for(int j = current_processing_timestep1; j < timestep; j++) {
                                if (LINUCB_instance->TimestepMap.find(j) != LINUCB_instance->TimestepMap.end()) {
                                    auto& timestepData1 = LINUCB_instance->TimestepMap[j];
                                    std::get<2>(timestepData1) = true;
                                    current_processing_timestep1 = j + 1;
                                }
                            }
                        }
                    }
                    if (LINUCB_instance->TimestepMap.find(starting_check) != LINUCB_instance->TimestepMap.end()) {
                        auto firstEntry = LINUCB_instance->TimestepMap[starting_check];
                        auto& firstValue = firstEntry;
                        bool firstBool = std::get<1>(firstValue);
                        bool secondBool = std::get<2>(firstValue);
                        if (firstBool && secondBool) {
                            double current_reward = std::get<3>(firstValue);
                            int finished_timestep = starting_check;
                            lossQueue.push(std::make_pair(current_reward, finished_timestep));
                            queueCondition.notify_one();
                            starting_check++; 
                        }  
                    }
                    double bytes = std::get<1>(it->second);
                    double rtt_for_packet = (double)(ack_received_time - current_packet_sent_time - (last_packet_sent_time - current_packet_sent_time)) / 1000.0;
                    double current_packet_throughput = bytes / rtt_for_packet;
                    double gamma_s = std::get<4>(it->second);
                    auto& timestepData = LINUCB_instance->TimestepMap[timestep];
                    std::get<3>(timestepData) += current_packet_throughput * gamma_s;
                    bool& ackedFlag = std::get<2>(it->second);
                    ackedFlag = true;
                }
            }
 
            lock.lock(); 
        }
    }
}

extern "C" {

void LINUCB_initialize(int numActions) {
    if (!LINUCB_instance) {
        LINUCB_instance.reset(new LINUCB(2));
        processingLoss = true;
        processingACK = true;
        lossProcessingThread = std::thread(processLoss);
        ACKProcessingThread = std::thread(processACK);
    }
}

void LinUCB_passACK(int path_id, uint64_t highestAckedPacket, int range, uint64_t ack_received_time) { 
    {
        std::lock_guard<std::mutex> lock(UCBMutex);
        ACKQueue.push(std::make_tuple(path_id, highestAckedPacket, range, ack_received_time));
    }
    ACKCondition.notify_one();
}

void pass_features(uint64_t cwnd0, uint64_t InP0, uint64_t RTT0, uint64_t cwnd1, uint64_t InP1, uint64_t RTT1, uint64_t swnd) { 
    if (LINUCB_instance) {
        int t = LINUCB_instance->get_timestep();
        if (feature_timestep_map.find(t) == feature_timestep_map.end()) {
            std::vector<double> x_t(6);
            x_t[0] = (double)cwnd0 / (double)RTT0;
            x_t[1] = (double)InP0 / (double)RTT0;
            x_t[2] = (double)swnd / (double)RTT0;
            x_t[3] = (double)cwnd1 / (double)RTT1;
            x_t[4] = (double)InP1 / (double)RTT1;
            x_t[5] = (double)swnd / (double)RTT1;
            feature_timestep_map[t] = x_t;
        }
    }
}

void manual_update() { 
    if (LINUCB_instance) {
        LINUCB_instance->manual_update();
    }
}
 
void mapPacketNumberToCurrentTimeStep(int path_id, uint64_t packetNumber, int current_timestep, double bytes, uint64_t current_wall_time, double gamma_s) {
    if (LINUCB_instance && (path_id == 0 || path_id == 1)) {
        std::map<uint64_t, std::tuple<int, double, bool, uint64_t, double>>& targetMap = (path_id == 0) ? 
            LINUCB_instance->packetNumbertoTimestepMap0 : 
            LINUCB_instance->packetNumbertoTimestepMap1;
        targetMap[packetNumber] = std::make_tuple(current_timestep, bytes, false, current_wall_time, gamma_s);
    }
}

void updateTimestep(int current_timestep, uint64_t start_time) {
    if (LINUCB_instance) {
        LINUCB_instance->TimestepMap[current_timestep] = std::make_tuple(start_time, false, false, 0.0);
    }
}

int wait_or_transmit(uint64_t cwnd0, uint64_t InP0, uint64_t RTT0, uint64_t cwnd1, uint64_t InP1, uint64_t RTT1, uint64_t swnd) {
    if (LINUCB_instance) {
        if (learning == true) {
            if (current_timestep%2 == 0) {
                return 0;
            }
            else {
                return 1;
            }
        }
        if (deployment == true) {
            std::vector<double> x_t(6);
            x_t[0] = (double)cwnd0 / (double)RTT0;
            x_t[1] = (double)InP0 / (double)RTT0;
            x_t[2] = (double)swnd / (double)RTT0;
            x_t[3] = (double)cwnd1 / (double)RTT1;
            x_t[4] = (double)InP1 / (double)RTT1;
            x_t[5] = (double)swnd / (double)RTT1;
            int action = LINUCB_instance->select_action(x_t); 
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0.0, 1.0);

            double random_value = dis(gen);
            
            if (action == 0) {
                if (random_value < pt) {
                    return 0;
                } else {
                    return 1;
                }
            }
            if (action == 1) {
                if (random_value < pw) {
                    return 1;
                } else {
                    return 0;
                }
            }
        }
    }
    else {
        return 0;
    }
}

}

