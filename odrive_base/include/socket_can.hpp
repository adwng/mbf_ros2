#ifndef SOCKET_CAN_HPP
#define SOCKET_CAN_HPP

#include "epoll_event_loop.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>
#include <functional>
#include <atomic>
#include <cstdint>
#include <thread>

using FrameProcessor = std::function<void(const can_frame&)>;

class SocketCanIntf {
public:
    ~SocketCanIntf();
    bool init(const std::string& interface, EpollEventLoop* event_loop, FrameProcessor frame_processor);
    void deinit();
    bool send_can_frame(const can_frame& frame);

    // Start a dedicated thread that blocks on the CAN socket and drains inbound
    // frames as they arrive, decoupling RX from the ros2_control read() cycle.
    // Call once after init() succeeds. Returns false if the thread could not be
    // started (e.g. the stop eventfd failed to create).
    bool start_rx_thread();

    bool read_nonblocking();

    // Number of TX frames dropped because write() failed (typically ENOBUFS,
    // i.e. the netdev/MCP2515 TX queue could not be drained fast enough).
    uint64_t tx_drop_count() const {
        return tx_drop_count_.load(std::memory_order_relaxed);
    }

private:
    std::string interface_;
    int socket_id_ = -1;
    EpollEventLoop* event_loop_ = nullptr;
    EpollEventLoop::EvtId socket_evt_id_;
    FrameProcessor frame_processor_;
    bool broken_ = false;

    // Dedicated receive thread. RX is drained here (via poll() on the CAN
    // socket plus a stop eventfd) instead of synchronously inside read().
    std::thread rx_thread_;
    std::atomic<bool> rx_running_{false};
    int stop_fd_ = -1;

    std::atomic<uint64_t> tx_drop_count_{0};

    void rx_thread_func();
    void stop_rx_thread();
    void on_socket_event(uint32_t mask);
    void process_can_frame(const can_frame& frame) {
        frame_processor_(frame);
    }
};

#endif  // SOCKET_CAN_HPP