/**
 * Test of the base height of the Unitree G1, through the stand height of the locomotion controller
 * (unitree::robot::g1::LocoClient::SetStandHeight). It uses unitree_sdk2 directly, without ROS and without the
 * driver, and prints the return code of every call (0 means that the robot accepted it).
 *
 * The robot must already be walking (keeping the balance and following the joystick). This test only calls the
 * stand height and the getters: it does not change the FSM and does not send velocities, so the joystick keeps
 * working during the test.
 *
 * Build (from the root of this repository; unitree_sdk2 installed in /opt/unitree_robotics):
 *   cmake -S cpp_tests -B cpp_tests/build
 *   cmake --build cpp_tests/build
 *
 * Usage:
 *   ./cpp_tests/build/g1_stand_height_test <network_interface> [height_1 height_2 ...]
 *   ./cpp_tests/build/g1_stand_height_test enp6s0             # 0.0 (the same as LocoClient::LowStand()), then back
 *   ./cpp_tests/build/g1_stand_height_test enp6s0 0.7 0.6 0.5 # each target in turn, then back
 * The network interface is the one connected to the robot: e.g. enp6s0 on a desktop (with an address in
 * 192.168.123.x), or eth0 on the onboard computer.
 *
 * After the targets (or after Ctrl+C), the initial stand height is sent again. If the robot does not report it
 * (GetStandHeight fails), the targets are still sent, and the test ends with the highest stand height instead (the
 * value of LocoClient::HighStand()).
 *
 * Results on our G1 (3-DoF waist, 2026-10-21): GetBalanceMode and GetStandHeight return 7301 ("LocoState not
 * available") in FSM 801 and 501. In 501, SetStandHeight returns 0, but the robot does not move (as in
 * unitree_sdk2_python issue #138). FSM 500 (LocoClient::Start()) has not been tested yet.
 */
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

namespace
{

std::atomic_bool kill_this_process{false};

void sig_int_handler(int)
{
    kill_this_process = true;
}

constexpr double kWaitTimeSec = 3.0;  // Time given to the robot after each target.

/**
 * @brief Prints the FSM ID, the FSM mode, the balance mode, and the stand height, with their return codes.
 */
void _print_state(unitree::robot::g1::LocoClient& client)
{
    int fsm_id{};
    int fsm_mode{};
    int balance_mode{};
    float stand_height{};
    const int32_t fsm_id_code = client.GetFsmId(fsm_id);
    const int32_t fsm_mode_code = client.GetFsmMode(fsm_mode);
    const int32_t balance_mode_code = client.GetBalanceMode(balance_mode);
    const int32_t stand_height_code = client.GetStandHeight(stand_height);
    std::cout << "fsm_id " << fsm_id << " (code " << fsm_id_code << ")"
              << ", fsm_mode " << fsm_mode << " (code " << fsm_mode_code << ")"
              << ", balance_mode " << balance_mode << " (code " << balance_mode_code << ")"
              << ", stand_height " << stand_height << " (code " << stand_height_code << ")" << std::endl;
}

/**
 * @brief Sends the stand height, waits kWaitTimeSec (less if Ctrl+C is pressed), and prints the state.
 */
void _set_stand_height(unitree::robot::g1::LocoClient& client, const float& stand_height)
{
    const int32_t code = client.SetStandHeight(stand_height);
    std::cout << "SetStandHeight(" << stand_height << "): code " << code << std::endl;

    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(kWaitTimeSec);
    while (!kill_this_process && std::chrono::steady_clock::now() < end)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    _print_state(client);
}

}

int main(int argc, char** argv)
{
    const std::vector<std::string> args(argv, argv + argc);
    if (args.size() < 2)
    {
        std::cerr << "Usage: " << args.at(0) << " <network_interface> [height_1 height_2 ...]" << std::endl;
        return 1;
    }
    const std::string network_interface = args.at(1);
    std::vector<float> target_heights;
    for (std::size_t i = 2; i < args.size(); ++i)
        target_heights.push_back(std::stof(args.at(i)));
    if (target_heights.empty())
        target_heights.push_back(0.0f);

    if (signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");

    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    unitree::robot::g1::LocoClient client;
    client.Init();
    client.SetTimeout(10.0f);

    std::cout << "Initial state: ";
    _print_state(client);

    float initial_height{};
    const int32_t code = client.GetStandHeight(initial_height);
    if (code != 0)
    {
        // The same value as LocoClient::HighStand().
        initial_height = static_cast<float>(std::numeric_limits<uint32_t>::max());
        std::cerr << "GetStandHeight failed (code " << code << "). The test ends with the highest stand height "
                     "(LocoClient::HighStand()) instead of the initial one." << std::endl;
    }

    for (const auto& target_height : target_heights)
    {
        if (kill_this_process)
            break;
        _set_stand_height(client, target_height);
    }

    // Back to the initial height, also after Ctrl+C.
    kill_this_process = false;
    std::cout << "Back to the initial stand height (or the highest one)." << std::endl;
    _set_stand_height(client, initial_height);
    return 0;
}
