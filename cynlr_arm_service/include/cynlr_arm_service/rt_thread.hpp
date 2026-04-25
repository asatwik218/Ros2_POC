#pragma once
#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include "cynlr_arm_core/types.hpp"
#include "cynlr_arm_core/arm_interface.hpp"

namespace cynlr::arm_service {

struct RTConfig {
    int cycle_time_us{1000};     // 1ms → 1kHz
    int cpu_affinity{-1};        // -1 = no pinning
    std::string scheduler{"fifo"};
    int priority{90};
    int command_timeout_ms{100}; // zero command after this long without update
};

// Lock-free triple buffer for single-producer, single-consumer use.
//
// Encoding (state_ bits):
//   [1:0] = write slot index  (producer owns this)
//   [3:2] = staging slot index (last written; consumer swaps with read on "new" flag)
//   [4]   = dirty flag (new data available since last read)
//
// Read slot is implicit: 3 - write_slot - staging_slot
// Initial value 0b00110: write=2, staging=1, read=0, dirty=0.
template<typename T>
class TripleBuffer {
public:
    // Called by the single producer. Never blocks.
    void write(const T& value) {
        int s = state_.load(std::memory_order_relaxed);
        int write_slot = s & 3;
        slots_[write_slot] = value;
        // Swap write ↔ staging, mark dirty
        int staging_slot = (s >> 2) & 3;
        int new_s = staging_slot | (write_slot << 2) | (1 << 4);
        state_.store(new_s, std::memory_order_release);
    }

    // Called by the single consumer. Never blocks.
    // Returns the latest value — if no new data, returns the last read value.
    T read() const {
        int s = state_.load(std::memory_order_acquire);
        int write_slot = s & 3;
        int staging_slot = (s >> 2) & 3;
        int read_slot = 3 - write_slot - staging_slot;

        if (!(s & (1 << 4))) {
            return slots_[read_slot];
        }

        // New data: swap staging ↔ read, clear dirty
        int new_s = write_slot | (read_slot << 2); // dirty bit cleared
        while (!state_.compare_exchange_weak(s, new_s,
            std::memory_order_acq_rel, std::memory_order_acquire))
        {
            write_slot = s & 3;
            staging_slot = (s >> 2) & 3;
            read_slot = 3 - write_slot - staging_slot;
            if (!(s & (1 << 4))) {
                return slots_[read_slot];
            }
            new_s = write_slot | (read_slot << 2);
        }
        // CAS succeeded: old staging_slot now holds the value we promoted
        return slots_[staging_slot];
    }

    bool has_new() const {
        return (state_.load(std::memory_order_acquire) & (1 << 4)) != 0;
    }

private:
    mutable std::atomic<int> state_{0b00110};
    T slots_[3]{};
};

class RTThread {
public:
    explicit RTThread(RTConfig config);
    ~RTThread();

    // Non-copyable
    RTThread(const RTThread&) = delete;
    RTThread& operator=(const RTThread&) = delete;

    // Start the RT loop. `arm` must outlive this object.
    void start(cynlr::arm::ArmInterface* arm);
    void stop();

    // Called from NRT side to feed commands into the RT loop.
    void write_command(const cynlr::arm::StreamCommand& cmd);

    // Called from NRT side to read the latest arm state.
    cynlr::arm::ArmState read_state() const;

    bool is_running() const { return running_.load(std::memory_order_acquire); }

private:
    void run();
    void configure_scheduling();

    RTConfig config_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    cynlr::arm::ArmInterface* arm_{nullptr};

    TripleBuffer<cynlr::arm::StreamCommand> command_buffer_;
    TripleBuffer<cynlr::arm::ArmState>      state_buffer_;
};

} // namespace cynlr::arm_service
