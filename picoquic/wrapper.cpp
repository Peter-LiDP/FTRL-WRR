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
#include <deque>
#include <cmath>
 
struct Bucket {
    int capacity;
    double sum;
    double sum_of_squares;

    Bucket(double value) {
        capacity = 1;
        sum = value;
        sum_of_squares = value * value;
    }

    Bucket(int cap, double s, double s_squares) {
        capacity = cap;
        sum = s;
        sum_of_squares = s_squares;
    }
};

static std::unique_ptr<FTRL> FTRL_instance;
static std::queue<std::pair<double, int>> lossQueue;
static std::queue<std::pair<int, int>> actionQueue;
static std::queue<std::tuple<int, uint64_t, int, uint64_t>> ACKQueue;
static std::queue<std::pair<uint64_t, uint64_t>> BWQueue;
std::deque<Bucket> buckets;
double delta = 0.3;

const size_t NUM_PATHS = 2;
std::vector<std::queue<uint64_t>> path_updating_queues(NUM_PATHS);
std::vector<uint64_t> last_update_time(NUM_PATHS, 0); 

int current_processing_timestep0 = 1;
int current_processing_timestep1 = 1;
int starting_check = 1;

static std::mutex FTRLMutex;
static std::condition_variable ACKCondition;
static std::condition_variable BWCondition;
static std::condition_variable queueCondition;
static std::condition_variable actionCondition;

static std::atomic<bool> processingACK{true};
static std::atomic<bool> processingBW{true};
static std::atomic<bool> processingLoss{true};
static std::atomic<bool> processingAction{true};
static std::thread lossProcessingThread;
static std::thread actionProcessingThread;
static std::thread ACKProcessingThread;
static std::thread BWProcessingThread;
static std::atomic<bool> resetModel{true};
static std::thread modelResetThread;

void discardOldBuckets(int capacityToDiscard) {
    int discardedCapacity = 0;

    while (!buckets.empty() && discardedCapacity < capacityToDiscard) {
        discardedCapacity += buckets.front().capacity;
        buckets.pop_front();
    }
}

bool testForChange(double recentMean, double olderMean, int recentCapacity, int olderCapacity, double total_variance, int totalCapacity) {
    double m = (1.0 / recentCapacity) + (1.0 / olderCapacity);
    double delta_m = delta / totalCapacity;
    double epsilon = sqrt((1.0 / 2.0 * m) * log(4.0 / delta_m));

    return fabs(recentMean - olderMean) > epsilon;
}

void checkForChange() {
    double totalSum = 0.0;
    double totalSumSquare = 0.0;
    int totalCapacity = 0;

    for (const auto& bucket : buckets) {
        totalSum += bucket.sum;
        totalSumSquare += bucket.sum_of_squares;
        totalCapacity += bucket.capacity;
    }

    double recentSum = 0.0;
    int recentCapacity = 0;

    for (int i = buckets.size() - 1; i >= 0; --i) {
        recentSum += buckets[i].sum;
        recentCapacity += buckets[i].capacity;

        int olderCapacity = totalCapacity - recentCapacity;
        double olderSum = totalSum - recentSum;

        if (olderCapacity > 0) {
            double recentMean = recentSum / recentCapacity;
            double olderMean = olderSum / olderCapacity;
            double total_variance = (totalSumSquare / totalCapacity) - pow((totalSum / totalCapacity), 2);

            if (testForChange(recentMean, olderMean, recentCapacity, olderCapacity, total_variance, totalCapacity)) {
                discardOldBuckets(olderCapacity);
                FTRL_instance->reset_t = 1;
                if ((olderMean <= 0.5 && recentMean > 0.5) || (olderMean > 0.5 && recentMean <= 0.5)) {
                    FTRL_instance->sum_g = 0;
                    FTRL_instance->sum_g_without_lr = 0;
                }
                break;
            }
        }
    }
}

void mergeLastTwoBuckets(int currentIndex) {
    Bucket& last = buckets[currentIndex];
    Bucket& second_last = buckets[currentIndex - 1];

    int new_capacity = last.capacity + second_last.capacity;
    double new_sum = last.sum + second_last.sum;
    double new_sum_of_squares = last.sum_of_squares + second_last.sum_of_squares;

    buckets[currentIndex - 1] = Bucket(new_capacity, new_sum, new_sum_of_squares);

    buckets.pop_back();
}

void reorganizeBuckets() {
    int currentIndex = buckets.size() - 1;

    while (currentIndex > 0) {
        int currentCapacity = buckets[currentIndex].capacity;
        int previousCapacity = buckets[currentIndex - 1].capacity;

        if (currentCapacity == previousCapacity) {
            mergeLastTwoBuckets(currentIndex);
            currentIndex--;
        } else {
            break;
        }
    }
}

void addDataPoint(double value) {
    buckets.push_back(Bucket(value));
    reorganizeBuckets();
    checkForChange();
}

void processBW() {
    while (processingBW) {
        std::unique_lock<std::mutex> lock(FTRLMutex);
        BWCondition.wait(lock, []{ return !BWQueue.empty() || !processingBW; });
        
        while (!BWQueue.empty() && FTRL_instance) {
            auto BW_info = BWQueue.front();
            BWQueue.pop();
            double p0_ratio = static_cast<double>(std::get<0>(BW_info)) / (static_cast<double>(std::get<0>(BW_info)) + static_cast<double>(std::get<1>(BW_info)));
            
            addDataPoint(p0_ratio);
        }
    }
}

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
                            
                            double Lx = pow(1 - (throughput / bandwidth_sum), 2);
                            if (throughput > bandwidth_sum) {
                                Lx = 0;
                            }
                            int finished_timestep = starting_check;
                            lossQueue.push(std::make_pair(Lx, finished_timestep));
                            queueCondition.notify_one();
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
        processingBW = true;
        lossProcessingThread = std::thread(processLoss);
        actionProcessingThread = std::thread(processAction);
        ACKProcessingThread = std::thread(processACK);
        BWProcessingThread = std::thread(processBW);
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

void ADWIN2_passBW(uint64_t bw0, uint64_t bw1) { 
    {
        std::lock_guard<std::mutex> lock(FTRLMutex);
        BWQueue.push(std::make_pair(bw0, bw1));
    }
    BWCondition.notify_one();
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

