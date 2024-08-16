#include "blest.h"
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
#include <set>
#include "picoquic_internal.h" 

typedef std::pair<uint64_t, uint64_t> Interval;
std::set<Interval> offset_intervals;
int var_blest = 0;
bool path0_checked = false;
bool path1_checked = false;

static std::queue<std::tuple<uint64_t, uint64_t, int>> ACKQueue;

static std::mutex BGDMutex;
static std::condition_variable ACKCondition;

static std::atomic<bool> processingACK{false};
static std::thread ACKProcessingThread;

void processACK() {
    while (processingACK) {
        std::unique_lock<std::mutex> lock(BGDMutex);
        ACKCondition.wait(lock, []{ return !ACKQueue.empty() || !processingACK; });

        while (!ACKQueue.empty()) {
            auto ack = ACKQueue.front();
            ACKQueue.pop();
            lock.unlock();

            uint64_t path_id = std::get<0>(ack);
            if (path_id == 0) {
                path0_checked = true;
            }
            if (path_id == 1) {
                if (path0_checked == true)
                    path1_checked = true; 
            }
            uint64_t highestAckedPacket = std::get<1>(ack);
            int range = std::get<2>(ack);
            uint64_t startPacketNumber = highestAckedPacket - range + 1;

            for (uint64_t p = startPacketNumber; p <= highestAckedPacket; ++p) {
		    PacketStreamOffsetMap* mapping = find_mapping(p, path_id);
		    if (mapping != nullptr) {
			if (mapping->processed) {
			    continue;
			} else {
			    uint64_t start_offset = mapping->offset;
                           uint64_t end_offset = mapping->offset + mapping->bytes;
                           offset_intervals.insert({start_offset, end_offset});
			    mapping->processed = true;
			}
		    }
		}
		check_for_missing_offsets();


            lock.lock();
        }
    }
}



extern "C" {

PacketStreamOffsetMap* mapping_head = nullptr;

void initialize() {
    if (processingACK == false) {
        processingACK = true;
        ACKProcessingThread = std::thread(processACK);
        printf("init BLEST");
    }
}

void passACK_BLEST(uint64_t path_id, uint64_t highestAckedPacket, int range) { 
    {
        std::lock_guard<std::mutex> lock(BGDMutex);
        ACKQueue.push(std::make_tuple(path_id, highestAckedPacket, range));
    }
    ACKCondition.notify_one();
}

PacketStreamOffsetMap* find_mapping(uint64_t packet_number, uint64_t path_id) {
    PacketStreamOffsetMap* current = mapping_head;
    while (current != nullptr) {
        if (current->packet_number == packet_number && current->path_id == path_id) {
            return current;
        }
        current = current->next;
    }
    return nullptr;
}

void check_for_missing_offsets() {
    if (offset_intervals.empty()) {
        std::cout << "No gaps detected" << std::endl;
        var_blest--;
        if (var_blest < -1000) {
            var_blest = -1000;
        }
        return;
    }

    uint64_t highest_offset = 0;
    for (const auto& interval : offset_intervals) {
        if (interval.second > highest_offset) {
            highest_offset = interval.second;
        }
    }

    uint64_t threshold = highest_offset > 100000 ? highest_offset - 100000 : 0;

    uint64_t last_end = 0;
    bool gap_found = false;

    for (const auto& interval : offset_intervals) {
        if (interval.second < threshold) {
            continue;
        }
        if (last_end != 0 && interval.first > last_end) {
            gap_found = true;
        }
        last_end = interval.second;
    }

    if (!gap_found) {
        var_blest--;
        if (var_blest < -1000) {
            var_blest = -1000;
        }
    }
    else {
        var_blest++;
    }
}


}

