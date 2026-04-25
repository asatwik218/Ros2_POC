#include "cynlr_arm_service/rt_thread.hpp"

#include <spdlog/spdlog.h>
#include <stdexcept>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif

namespace cynlr::arm_service {

RTThread::RTThread(RTConfig config)
    : config_(std::move(config))
{}

RTThread::~RTThread() {
    stop();
}

void RTThread::start(cynlr::arm::ArmInterface* arm) {
    if (running_.load(std::memory_order_acquire)) {
        return;
    }
    arm_ = arm;
    running_.store(true, std::memory_order_release);
    thread_ = std::thread(&RTThread::run, this);
}

void RTThread::stop() {
    if (!running_.load(std::memory_order_acquire)) {
        return;
    }
    running_.store(false, std::memory_order_release);
    if (thread_.joinable()) {
        thread_.join();
    }
    arm_ = nullptr;
}

void RTThread::write_command(const cynlr::arm::StreamCommand& cmd) {
    command_buffer_.write(cmd);
}

cynlr::arm::ArmState RTThread::read_state() const {
    return state_buffer_.read();
}

void RTThread::configure_scheduling() {
#ifdef __linux__
    if (config_.cpu_affinity >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(config_.cpu_affinity, &cpuset);
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
            spdlog::warn("RTThread: failed to set CPU affinity to {}", config_.cpu_affinity);
        }
    }

    if (config_.scheduler == "fifo") {
        sched_param param{};
        param.sched_priority = config_.priority;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
            spdlog::warn("RTThread: failed to set SCHED_FIFO priority {} "
                         "(run as root or with CAP_SYS_NICE)", config_.priority);
        }
    }
#else
    spdlog::debug("RTThread: RT scheduling not configured (non-Linux platform)");
#endif
}

void RTThread::run() {
    configure_scheduling();
    spdlog::info("RTThread: started (cycle {}us)", config_.cycle_time_us);

    const auto cycle = std::chrono::microseconds(config_.cycle_time_us);
    auto next_wake = std::chrono::steady_clock::now() + cycle;

    while (running_.load(std::memory_order_acquire)) {
        // --- Read latest command from NRT side ---
        cynlr::arm::StreamCommand cmd = command_buffer_.read();

        // --- Stream command to hardware ---
        if (arm_) {
            auto result = arm_->stream_command(cmd);
            if (!result) {
                spdlog::warn("RTThread: stream_command failed: {}", result.error().message);
            }

            // --- Read state from hardware ---
            auto state_result = arm_->get_state();
            if (state_result) {
                state_buffer_.write(*state_result);
            }
        }

        // --- Sleep until next cycle ---
        std::this_thread::sleep_until(next_wake);
        next_wake += cycle;
    }

    spdlog::info("RTThread: stopped");
}

} // namespace cynlr::arm_service
