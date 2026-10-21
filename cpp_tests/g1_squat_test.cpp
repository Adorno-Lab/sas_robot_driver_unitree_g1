
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

namespace
{

constexpr int kStandUpSquatFsmId = 706;  // StandUp2Squat() and Squat2StandUp() of unitree_sdk2_python.
constexpr int kRegularModeFsmId = 501;   // Regular mode of the G1 with the 3-DoF waist.
constexpr double kWatchTimeSec = 5.0;    // Time during which the state is printed after each command.

/**
 * @brief Prints the FSM ID and the FSM mode, with their return codes.
 */
void _print_state(unitree::robot::g1::LocoClient& client)
{
    int fsm_id{};
    int fsm_mode{};
    const int32_t fsm_id_code = client.GetFsmId(fsm_id);
    const int32_t fsm_mode_code = client.GetFsmMode(fsm_mode);
    std::cout << "  fsm_id " << fsm_id << " (code " << fsm_id_code << ")"
              << ", fsm_mode " << fsm_mode << " (code " << fsm_mode_code << ")" << std::endl;
}

/**
 * @brief Sends the FSM ID, then prints the state every 0.5 s for kWatchTimeSec.
 */
void _set_fsm_id(unitree::robot::g1::LocoClient& client, const int& fsm_id)
{
    const int32_t code = client.SetFsmId(fsm_id);
    std::cout << "SetFsmId(" << fsm_id << "): code " << code << std::endl;

    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(kWatchTimeSec);
    while (std::chrono::steady_clock::now() < end)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        _print_state(client);
    }
}

/**
 * @brief Prints the message and waits for Enter.
 */
void _wait_for_enter(const std::string& message)
{
    std::cout << std::endl << message << " Press Enter to continue (Ctrl+C to stop)." << std::endl;
    std::string line;
    std::getline(std::cin, line);
}

/**
 * @brief Asks a yes/no question.
 * @return True only if the answer starts with y or Y.
 */
bool _ask(const std::string& question)
{
    std::cout << std::endl << question << " [y/N] " << std::flush;
    std::string answer;
    std::getline(std::cin, answer);
    return !answer.empty() && (answer.at(0) == 'y' || answer.at(0) == 'Y');
}

}

int main(int argc, char** argv)
{
    const std::vector<std::string> args(argv, argv + argc);
    if (args.size() < 2)
    {
        std::cerr << "Usage: " << args.at(0) << " <network_interface> [squat_fsm_id]" << std::endl;
        return 1;
    }
    const std::string network_interface = args.at(1);
    const int squat_fsm_id = args.size() > 2 ? std::stoi(args.at(2)) : kStandUpSquatFsmId;

    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    unitree::robot::g1::LocoClient client;
    client.Init();
    client.SetTimeout(10.0f);

    std::cout << "Initial state:" << std::endl;
    _print_state(client);

    _wait_for_enter("Step 1/3: squat with SetFsmId(" + std::to_string(squat_fsm_id) + ").");
    _set_fsm_id(client, squat_fsm_id);

    _wait_for_enter("Step 2/3: stand up with SetFsmId(" + std::to_string(kStandUpSquatFsmId) + ").");
    _set_fsm_id(client, kStandUpSquatFsmId);
    if (!_ask("Did the robot stand up?") &&
        _ask("Try the sequence of Squat2StandUp of unitree_sdk2_python: Damp() (the robot goes limp and sits down), "
             "then SetFsmId(" + std::to_string(kStandUpSquatFsmId) + ")?"))
    {
        std::cout << "Damp(): code " << client.Damp() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        _set_fsm_id(client, kStandUpSquatFsmId);
    }

    if (_ask("Step 3/3: back to the regular mode with SetFsmId(" + std::to_string(kRegularModeFsmId) + ")? "
             "(No: the robot stays as it is, e.g. to switch with the remote.)"))
        _set_fsm_id(client, kRegularModeFsmId);

    std::cout << "Final state:" << std::endl;
    _print_state(client);
    return 0;
}
