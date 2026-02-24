#pragma once

#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace autopilot {

using Clock = std::chrono::steady_clock;

struct StateData {
    double gimbal_faceing[3];
    bool autopilot_enabled;
};
struct PilotData {
    double chassis_vel[3]; // x, y, w
};

class Communication {
public:
    Communication()
        : sockfd_(-1)
        , send_sockfd_(-1)
        , running_(false) {}

    ~Communication() {
        stop();
        if (sockfd_ >= 0) {
            close(sockfd_);
        }
        if (send_sockfd_ >= 0) {
            close(send_sockfd_);
        }
    }

    bool startSending(std::string* error = nullptr) {
        if (send_sockfd_ >= 0) {
            return true;   // Already initialized
        }

        send_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (send_sockfd_ < 0) {
            if (error)
                *error = strerror(errno);

            return false;
        }
        return true;
    }

    bool startReceiving(
        const std::string& ip, uint16_t port,
        std::function<void(const PilotData&, std::string)> callback, std::string* error = nullptr) {
        if (running_) {
            return false;
        }

        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            if (error)
                *error = strerror(errno);
            return false;
        }

        struct sockaddr_in addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = inet_addr(ip.c_str());

        if (bind(sockfd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(sockfd_);
            sockfd_ = -1;
            if (error)
                *error = strerror(errno);
            return false;
        }

        running_ = true;
        callback_ = callback;

        recv_thread_ = std::thread(&Communication::receiveThread, this);
        return true;
    }

    void stop() {
        running_ = false;
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
    }

    bool sendStateData(
        const StateData& data, const std::string& ip, uint16_t port, std::string* error = nullptr) {
        if (send_sockfd_ < 0) {
            if (error) {
                *error = "Send socket not initialized. Call startSending() first.";
            }
            return false;
        }

        struct sockaddr_in dest_addr;
        std::memset(&dest_addr, 0, sizeof(dest_addr));
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(port);
        dest_addr.sin_addr.s_addr = inet_addr(ip.c_str());

        ssize_t sent = sendto(
            send_sockfd_, &data, sizeof(StateData), 0, (struct sockaddr*)&dest_addr,
            sizeof(dest_addr));

        if (error && sent < 0) {
            *error = strerror(errno);
        }

        return sent == sizeof(StateData);
    }

private:
    void receiveThread() {
        PilotData data;
        struct sockaddr_in sender_addr;
        socklen_t addr_len = sizeof(sender_addr);

        while (running_) {
            ssize_t recv_len = recvfrom(
                sockfd_, &data, sizeof(PilotData), 0, (struct sockaddr*)&sender_addr, &addr_len);

            if (recv_len == sizeof(PilotData) && callback_)
                callback_(data, "");
            else
                callback_(data, strerror(errno));
        }
    }

    int sockfd_;
    int send_sockfd_;
    std::atomic<bool> running_;
    std::thread recv_thread_;
    std::function<void(const PilotData&, std::string)> callback_;
};
} // namespace autopilot