/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_UIKA_HPP
#define RL_REAL_UIKA_HPP

// #define PLOT
// #define CSV_LOGGER

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_uika.hpp"

#include <csignal>
#include <thread>
#include <mutex>
#include <sys/socket.h>
#include <linux/can.h>
#include <net/if.h>
#include <unistd.h>

// #include "matplotlibcpp.h"
// namespace plt = matplotlibcpp;

class UICANInterface
{
public:
    UICANInterface() : can_socket_(-1) {}
    ~UICANInterface() { Close(); }

    bool Init(const std::string& can_channel)
    {
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0)
        {
            std::cerr << "Failed to create CAN socket" << std::endl;
            return false;
        }

        strncpy(ifr_.ifr_name, can_channel.c_str(), IFNAMSIZ - 1);
        ifr_.ifr_name[IFNAMSIZ - 1] = '\0';
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr_) < 0)
        {
            std::cerr << "Failed to get CAN interface index" << std::endl;
            return false;
        }

        addr_.can_family = AF_CAN;
        addr_.can_ifindex = ifr_.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0)
        {
            std::cerr << "Failed to bind CAN socket" << std::endl;
            return false;
        }

        return true;
    }

    void Close()
    {
        if (can_socket_ >= 0)
        {
            close(can_socket_);
            can_socket_ = -1;
        }
    }

    // Receive robot state from CAN
    // User needs to implement this based on UIKA's CAN protocol
    bool Receive(RobotState<float>* state)
    {
        if (can_socket_ < 0) return false;

        struct can_frame frame;
        int nbytes = read(can_socket_, &frame, sizeof(frame));

        if (nbytes < 0)
        {
            return false;
        }

        // User needs to implement UIKA CAN protocol parsing here
        // This is a placeholder that returns zeros
        // Format: expected UIKA CAN message format
        for (int i = 0; i < 12; ++i)
        {
            state->motor_state.q[i] = 0.0f;
            state->motor_state.dq[i] = 0.0f;
            state->motor_state.tau_est[i] = 0.0f;
        }
        for (int i = 0; i < 3; ++i)
        {
            state->imu.gyroscope[i] = 0.0f;
        }
        state->imu.quaternion[0] = 1.0f;
        state->imu.quaternion[1] = 0.0f;
        state->imu.quaternion[2] = 0.0f;
        state->imu.quaternion[3] = 0.0f;

        return true;
    }

    // Send command to motors via CAN
    // User needs to implement this based on UIKA's CAN protocol
    bool Send(const RobotCommand<float>* cmd)
    {
        if (can_socket_ < 0) return false;

        struct can_frame frame;
        frame.can_id = 0x01;  // UIKA command CAN ID (user needs to verify)
        frame.can_dlc = 8;     // Data length code (user needs to adjust)

        // User needs to implement UIKA CAN protocol formatting here
        // This is a placeholder
        for (int i = 0; i < 8; ++i)
        {
            frame.data[i] = 0;
        }

        int nbytes = write(can_socket_, &frame, sizeof(frame));
        return nbytes == sizeof(frame);
    }

private:
    int can_socket_;
    struct ifreq ifr_;
    struct sockaddr_can addr_;
};

class RL_Real : public RL
{
public:
    RL_Real(int argc, char **argv);
    ~RL_Real();

private:
    // rl functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    // CAN interface
    UICANInterface can_interface_;
    std::string can_channel_;

    // others
    std::vector<float> mapped_joint_positions;
    std::vector<float> mapped_joint_velocities;
};

#endif // RL_REAL_UIKA_HPP
