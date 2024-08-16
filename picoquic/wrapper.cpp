#include "ftrl.h"
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
#include "globals.h"
#include <fstream>
#include <algorithm>

static std::unique_ptr<FTRL> FTRL_instance;
static std::queue<std::pair<double, int>> lossQueue;
static std::queue<std::pair<int, int>> actionQueue;
static std::queue<std::tuple<int, uint64_t, int, uint64_t>> ACKQueue;

const size_t NUM_PATHS = 2;
std::vector<std::queue<uint64_t>> path_updating_queues(NUM_PATHS);
std::vector<uint64_t> last_update_time(NUM_PATHS, 0); 

int current_processing_timestep0 = 1;
int current_processing_timestep1 = 1;
int starting_check = 1;

static std::mutex FTRLMutex;
static std::condition_variable ACKCondition;
static std::condition_variable queueCondition;
static std::condition_variable actionCondition;

static std::atomic<bool> processingACK{true};
static std::atomic<bool> processingLoss{true};
static std::atomic<bool> processingAction{true};
static std::thread lossProcessingThread;
static std::thread actionProcessingThread;
static std::thread ACKProcessingThread;
static std::atomic<bool> resetModel{true};
static std::thread modelResetThread;

void processLoss() {
    while (processingLoss) {
        std::unique_lock<std::mutex> lock(FTRLMutex);
        queueCondition.wait(lock, []{ return !lossQueue.empty() || !processingLoss; });
        
        while (!lossQueue.empty() && FTRL_instance) {
            auto loss_info = lossQueue.front();
            lossQueue.pop();
            double loss = std::get<0>(loss_info);
            int timestep = std::get<1>(loss_info);
            FTRL_instance->update(loss, timestep);
        }
    }
}

void processAction() {
    while (processingAction) {
        std::unique_lock<std::mutex> lock(FTRLMutex);
        actionCondition.wait(lock, []{ return actionQueue.size() < 1000 || !processingAction; });

        if (clearQueue) {
            actionQueue = std::queue<std::pair<int, int>>();
            clearQueue = false;
        }

        while (actionQueue.size() < 1000 && FTRL_instance) {
            std::pair<int, int> action = FTRL_instance->drawAction();
            actionQueue.push(action);
        }
    }
}

void processACK() {
    while (processingACK) {
        std::unique_lock<std::mutex> lock(FTRLMutex);
        ACKCondition.wait(lock, []{ return !ACKQueue.empty() || !processingACK; });

        while (!ACKQueue.empty() && FTRL_instance) {
            auto ack = ACKQueue.front();
            ACKQueue.pop();
            lock.unlock();

            int path_id = std::get<0>(ack);
            uint64_t highestAckedPacket = std::get<1>(ack);
            int range = std::get<2>(ack);
            uint64_t ack_received_time = std::get<3>(ack);

            std::map<uint64_t, std::tuple<int, double, bool, uint64_t>>& targetMap = (path_id == 0) ? FTRL_instance->packetNumbertoTimestepMap0 : FTRL_instance->packetNumbertoTimestepMap1;
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
                            if (FTRL_instance->TimestepMap.find(current_processing_timestep0) != FTRL_instance->TimestepMap.end()) {
                                auto& timestepData0 = FTRL_instance->TimestepMap[current_processing_timestep0];
                                std::get<1>(timestepData0) = true;
                            }
                            current_processing_timestep0 = timestep;
                        }
                    }
                    if (path_id == 1) {
                        if(current_processing_timestep1 != timestep) {
                            if (FTRL_instance->TimestepMap.find(current_processing_timestep1) != FTRL_instance->TimestepMap.end()) {
                                auto& timestepData1 = FTRL_instance->TimestepMap[current_processing_timestep1];
                                std::get<2>(timestepData1) = true;
                            }
                            current_processing_timestep1 = timestep;
                        }
                    }
                    if (FTRL_instance->TimestepMap.find(starting_check) != FTRL_instance->TimestepMap.end()) {
                        auto firstEntry = FTRL_instance->TimestepMap[starting_check];
                        auto& firstValue = firstEntry;
                        bool firstBool = std::get<1>(firstValue);
                        bool secondBool = std::get<2>(firstValue);
                        if (firstBool && secondBool) {
                            uint64_t bandwidth_high0 = std::get<4>(firstValue);
                            uint64_t bandwidth_high1 = std::get<5>(firstValue);
                            double bandwidth_sum = (double)(bandwidth_high0 + bandwidth_high1) / (1000.0);
                            uint64_t sent_start_time = std::get<0>(firstValue); 
                            double acked_bytes = std::get<3>(firstValue);
                            double time_interval = (double)(ack_received_time - sent_start_time - last_packet_sent_time + current_packet_sent_time) / 1000.0;
                            double throughput = acked_bytes / time_interval;
                            double expected_distribution = FTRL_instance->XtAtTimeStep[starting_check];
                            double actual_distribution = std::get<6>(firstValue) / (std::get<6>(firstValue) + std::get<7>(firstValue));
                            if (expected_distribution >= actual_distribution) {
                                throughput = throughput*actual_distribution / expected_distribution;
                            }
                            else {
                                throughput = throughput*(1 - actual_distribution) / (1 - expected_distribution);
                            }
                            
                            double Lx = 1 - (throughput / bandwidth_sum);
                            if (throughput > bandwidth_sum) {
                                Lx = 0;
                            }
                            int finished_timestep = starting_check;
                            starting_check++; 
                        }  
                    }
                    double bytes = std::get<1>(it->second);
                    auto& timestepData = FTRL_instance->TimestepMap[timestep];
                    std::get<3>(timestepData) += bytes;
                    if (path_id == 0) {
                        std::get<6>(timestepData) += bytes;
                    }
                    if (path_id == 1) {
                        std::get<7>(timestepData) += bytes;
                    }
                    bool& ackedFlag = std::get<2>(it->second);
                    ackedFlag = true;
                }
            }

            lock.lock();
        }
    }
}

extern "C" {

void FTRL_initialize(int numActions) {
    if (!FTRL_instance) {
        FTRL_instance.reset(new FTRL(numActions));
        processingLoss = true;
        processingAction = true;
        processingACK = true;
        lossProcessingThread = std::thread(processLoss);
        actionProcessingThread = std::thread(processAction);
        ACKProcessingThread = std::thread(processACK);
        resetModel = true;
    }
}

void FTRL_passACK(int path_id, uint64_t highestAckedPacket, int range, uint64_t ack_received_time) { 
    {
        std::lock_guard<std::mutex> lock(FTRLMutex);
        ACKQueue.push(std::make_tuple(path_id, highestAckedPacket, range, ack_received_time));
    }
    ACKCondition.notify_one();
}

void FTRL_second_update() { 
    if (FTRL_instance) {
        FTRL_instance->second_timestep_update();
    }
}
 
void FTRL_drawAction(ActionResult* result) {
    std::lock_guard<std::mutex> lock(FTRLMutex);
    if (!actionQueue.empty()) {
        std::pair<int, int> firstAction = actionQueue.front();
        int actionValue = firstAction.first;
        int actionTime = firstAction.second;
        int count = 0;

        while (!actionQueue.empty() && actionQueue.front().first == actionValue && actionQueue.front().second == actionTime) {
            count++;
            actionQueue.pop();
        }
        actionCondition.notify_one();

        result->value = actionValue;
        result->count = count; 
        result->t = actionTime;
    }
    else {
        if (FTRL_instance) {
            std::pair<int, int> action = FTRL_instance->drawAction();
            result->value = action.first;
            result->count = 1;
            result->t = action.second;
        } else {
            result->value = -1;
            result->count = 0;
            result->t = 0;
        }
    }
}
 
void mapPacketNumberToCurrentTimeStep(int path_id, uint64_t packetNumber, int current_timestep, double bytes, uint64_t current_wall_time) {
    if (FTRL_instance && (path_id == 0 || path_id == 1)) {
        std::map<uint64_t, std::tuple<int, double, bool, uint64_t>>& targetMap = (path_id == 0) ? 
            FTRL_instance->packetNumbertoTimestepMap0 : 
            FTRL_instance->packetNumbertoTimestepMap1;
        targetMap[packetNumber] = std::make_tuple(current_timestep, bytes, false, current_wall_time);
    }
}

void updateTimestep(int current_timestep, uint64_t start_time) {
    if (FTRL_instance) {
        FTRL_instance->TimestepMap[current_timestep] = std::make_tuple(start_time, false, false, 0.0, 0, 0, 0.0, 0.0);
    }
}

void updateBandwidth(int current_timestep, int path_id, uint64_t bandwidth_high) {
    if (FTRL_instance) {
        auto it = FTRL_instance->TimestepMap.find(current_timestep);
        if (it != FTRL_instance->TimestepMap.end()) {
            if (path_id == 0) {
                std::get<4>(it->second) = bandwidth_high;
            }
            if (path_id == 1) {
                std::get<5>(it->second) = bandwidth_high;
            }
        }
    }
}

}

