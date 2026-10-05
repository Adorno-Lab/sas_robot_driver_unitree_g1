/**
 * @file DriverUnitreeG1.cpp
 * @brief Implementation of DriverUnitreeG1: aggregates DriverUnitreeLocoClient,
 *        DriverUnitreeG1ArmSDK, and DriverUnitreeLowState behind a single class, with
 *        an Eigen::VectorXd-based interface for every joint-position getter/setter,
 *        and OPERATION_MODE-gated access -- see the header for the overall design.
 */
#include "DriverUnitreeG1.h"
#include <unitree/robot/channel/channel_factory.hpp>

#include <stdexcept>

namespace
{
/**
 * @brief Converts a fixed-size std::array<double, N> to an Eigen::VectorXd of the
 *        same size.
 * @tparam N Array size.
 * @param array Source array.
 * @return An Eigen::VectorXd of size N holding the same values, in order.
 */
template <std::size_t N>
Eigen::VectorXd array_to_vector(const std::array<double, N>& array)
{
    Eigen::VectorXd vector(static_cast<Eigen::Index>(N));
    for (std::size_t i = 0; i < N; ++i)
    {
        vector(static_cast<Eigen::Index>(i)) = array[i];
    }
    return vector;
}

/**
 * @brief Converts an Eigen::VectorXd to a fixed-size std::array<double, N>.
 * @tparam N Expected array size.
 * @param vector Source vector; must already have been size-checked by
 *        DriverUnitreeG1::validate_size() before calling this.
 * @return A std::array<double, N> holding the same values, in order.
 */
template <std::size_t N>
std::array<double, N> vector_to_array(const Eigen::VectorXd& vector)
{
    std::array<double, N> array{};
    for (std::size_t i = 0; i < N; ++i)
    {
        array[i] = vector(static_cast<Eigen::Index>(i));
    }
    return array;
}
/**
 * @brief Converts a std::vector<double> to an Eigen::VectorXd of the same size.
 * @details Unlike array_to_vector(), the size isn't known at compile time -- used for
 *          low_state_'s methods, whose joint count varies by robot/message set.
 * @param vec Source vector.
 * @return An Eigen::VectorXd holding the same values, in order, same size as @p vec.
 */
Eigen::VectorXd stdvector_to_eigen(const std::vector<double>& vec)
{
    Eigen::VectorXd out(static_cast<Eigen::Index>(vec.size()));
    for (std::size_t i = 0; i < vec.size(); ++i)
    {
        out(static_cast<Eigen::Index>(i)) = vec[i];
    }
    return out;
}
} // namespace

/**
 * @brief Validates that a joint-position vector has the expected size.
 * @param vector The vector to validate.
 * @param expected_size The required size for the joint group being set (7 for an
 *        arm, 3 for the waist).
 * @param method_name Name of the calling setter, used in the exception message.
 * @throws std::invalid_argument if vector.size() != expected_size.
 */
void DriverUnitreeG1::_validate_size(const Eigen::VectorXd& vector,
                                    const Eigen::Index& expected_size,
                                    const std::string& method_name)
{
    if (vector.size() != expected_size)
    {
        throw std::invalid_argument(
            method_name + ": expected a vector of size " + std::to_string(expected_size) +
            ", got size " + std::to_string(vector.size()));
    }
}

/**
 * @brief Guards a method that only makes sense in OPERATION_MODE::HIGH_LEVEL (i.e.
 *        one that delegates to loco_client_ or arm_sdk_).
 * @param method_name Name of the calling method, used in the exception message.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
void DriverUnitreeG1::_ensure_high_level(const std::string& method_name) const
{
    if (operation_mode_ != OPERATION_MODE::HIGH_LEVEL)
    {
        throw std::logic_error(
            method_name + ": only available in OPERATION_MODE::HIGH_LEVEL");
    }
}

/**
 * @brief Called by the joint-position setters when operation_mode_ ==
 *        OPERATION_MODE::LOW_LEVEL. Low-level joint commands are meant to go through
 *        a DriverUnitreeLowCmd member, which isn't available yet, so there is
 *        currently no way to honor these setters in LOW_LEVEL.
 * @param method_name Name of the calling setter, used in the exception message.
 * @throws std::logic_error unconditionally.
 */
void DriverUnitreeG1::_throw_low_level_unsupported(const std::string& method_name) const
{
    throw std::logic_error(
        method_name + ": low-level joint commands are not yet supported "
                      "(DriverUnitreeLowCmd is not available); in OPERATION_MODE::LOW_LEVEL, "
                      "only DriverUnitreeLowState reads are available");
}

/**
 * @brief Constructs the aggregated G1 driver. No hardware/DDS I/O happens here; see
 *        connect().
 * @param shutdown_signaler Shared sas::ShutdownSignaler (typically the same one a
 *        SIGINT handler calls shutdown() on), forwarded as-is to loco_client_,
 *        arm_sdk_, and low_state_ so all three sub-drivers poll the same
 *        should_shutdown() state.
 * @param op_mode Which control path this instance uses for its entire lifetime.
 * @param domain_id DDS domain ID used to initialize the process-wide
 *        unitree::robot::ChannelFactory in connect(), and shared by loco_client_,
 *        arm_sdk_, and low_state_.
 * @param network_interface Network interface name (e.g. "enp6s0") for the DDS
 *        channel factory.
 * @throws std::invalid_argument if shutdown_signaler is nullptr. Propagates from
 *         loco_client_'s, arm_sdk_'s, or low_state_'s own constructor, all of which
 *         validate this before DriverUnitreeG1's own constructor body runs.
 */
DriverUnitreeG1::DriverUnitreeG1(const std::shared_ptr<marinholab::sas::core::ShutdownSignaler> &shutdown_signaler, const OPERATION_MODE &op_mode,
                                 const int32_t& domain_id,
                                 const std::string& network_interface)
    : operation_mode_{op_mode},
    domain_id_{domain_id},
    network_interface_{network_interface},
    loco_client_{shutdown_signaler, DriverUnitreeLocoClient::ROBOT::G1, 0.01},
    arm_sdk_{shutdown_signaler, DriverUnitreeArmSDK::ROBOT::G1, 0.02},
    low_state_{shutdown_signaler, DriverUnitreeLowState::ROBOT::G1}  // Default to 500Hz (2ms) by the Unitree SDK2
{

}

/**
 * @brief Destructor: ensures a safe shutdown regardless of lifecycle state by calling
 *        deinitialize() then disconnect().
 */
DriverUnitreeG1::~DriverUnitreeG1()
{
    deinitialize();
    disconnect();
}

/**
 * @brief Initializes the process-wide DDS channel factory (a shared precondition for
 *        loco_client_, arm_sdk_, and low_state_), then connects the sub-drivers
 *        relevant to operation_mode_.
 * @details HIGH_LEVEL: connects loco_client_, arm_sdk_, then low_state_.
 *          LOW_LEVEL: connects only low_state_.
 */
void DriverUnitreeG1::connect()
{
    // Process-wide DDS channel factory init: a shared precondition for all
    // sub-drivers actually used in this operation mode, so it's called once here
    // rather than inside any of them.
    unitree::robot::ChannelFactory::Instance()->Init(domain_id_, network_interface_);

    if (operation_mode_ == OPERATION_MODE::HIGH_LEVEL)
    {
        loco_client_.connect();
        arm_sdk_.connect();
    }
    low_state_.connect();
}

/**
 * @brief Initializes the sub-drivers relevant to operation_mode_.
 * @details HIGH_LEVEL: initializes loco_client_, arm_sdk_, then low_state_.
 *          LOW_LEVEL: initializes only low_state_.
 */
void DriverUnitreeG1::initialize()
{
    if (operation_mode_ == OPERATION_MODE::HIGH_LEVEL)
    {
        loco_client_.initialize();
        arm_sdk_.initialize();
    }
    low_state_.initialize();
}

/**
 * @brief Deinitializes the sub-drivers relevant to operation_mode_, arm first in
 *        HIGH_LEVEL: arm_sdk_ brings the arms to a safe, blend-weight-zero state
 *        (holding last pose) before loco_client_ is deinitialized, matching the
 *        original class's shutdown order. low_state_ is always deinitialized last;
 *        it's a no-op beyond flipping a flag (it has no control loop or publisher to
 *        ramp down -- see its class docs), so its position in this order has no
 *        safety implication.
 * @details HIGH_LEVEL: arm_sdk_, loco_client_, then low_state_.
 *          LOW_LEVEL: only low_state_.
 */
void DriverUnitreeG1::deinitialize()
{
    if (operation_mode_ == OPERATION_MODE::HIGH_LEVEL)
    {
        // Arm first: brings the arms to a safe, blend-weight-zero state (holding
        // last pose) before touching locomotion, matching the original class's
        // shutdown order.
        arm_sdk_.deinitialize();
        loco_client_.deinitialize();
    }
    low_state_.deinitialize();
}

/**
 * @brief Disconnects the sub-drivers relevant to operation_mode_.
 * @details HIGH_LEVEL: arm_sdk_, loco_client_, then low_state_.
 *          LOW_LEVEL: only low_state_.
 */
void DriverUnitreeG1::disconnect()
{
    if (operation_mode_ == OPERATION_MODE::HIGH_LEVEL)
    {
        arm_sdk_.disconnect();
        loco_client_.disconnect();
    }
    low_state_.disconnect();
}

/**
 * @brief Switches between HIGH_MODE::LOCOMOTION and HIGH_MODE::ARM_CONTROL, performing whatever
 *        hand-off is required so the robot actually responds afterward, instead of
 *        leaving the caller to sequence enable/disable_arm_control(), set_balance_mode(),
 *        and a guessed wait between them by hand.
 * @details
 *  - HIGH_MODE::ARM_CONTROL: calls arm_sdk_.enable_arm_control(). Non-blocking -- the blend
 *    weight ramps 0 -> 1 in the background over the next few control-loop ticks.
 *  - HIGH_MODE::LOCOMOTION: calls arm_sdk_.deinitialize(), which synchronously ramps the
 *    blend weight 1 -> 0 -- holding the last commanded pose throughout -- and only
 *    returns once that ramp has actually completed, unlike disable_arm_control(),
 *    which only requests the ramp and returns immediately. Because deinitialize() also
 *    stops arm_sdk_'s background control thread, arm_sdk_.initialize() is called right
 *    after to restart it, so a later set_mode(HIGH_MODE::ARM_CONTROL) still works. Only then
 *    is loco_client_.set_balance_mode(1) called: the robot ignores velocity commands
 *    while the arm blend weight is above 0, so this restores that response once the
 *    weight has actually reached 0 -- not merely once disable_arm_control() was
 *    requested.
 * @param mode The mode to switch to.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 * @throws std::invalid_argument if @p mode doesn't match either enumerator.
 */
void DriverUnitreeG1::set_mode(const HIGH_MODE& mode)
{
    _ensure_high_level("DriverUnitreeG1::set_mode");

    switch (mode)
    {
    case HIGH_MODE::ARM_CONTROL:
        arm_sdk_.enable_arm_control();
        break;
    case HIGH_MODE::LOCOMOTION:
        // Blocks until the blend weight has fully ramped down to 0, holding the last
        // commanded pose throughout -- see DriverUnitreeG1ArmSDK::deinitialize().
        arm_sdk_.deinitialize();
        // deinitialize() stops arm_sdk_'s background control thread; restart it so a
        // later set_mode(MODE::ARM_CONTROL) still works.
        arm_sdk_.initialize();
        // Only now, with the weight confirmed at 0, restore the robot's response to
        // velocity commands.
        loco_client_.set_balance_mode(1);
        break;
    default:
        throw std::invalid_argument("DriverUnitreeG1::set_mode: unknown MODE");
    }
}

/**
 * @brief Returns the robot's current FSM (finite state machine) ID.
 * @return The FSM ID, as reported by loco_client_.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
int DriverUnitreeG1::get_fsm_id()
{
    _ensure_high_level("DriverUnitreeG1::get_fsm_id");
    return loco_client_.get_fsm_id();
}

/**
 * @brief Returns the robot's current FSM mode.
 * @return The FSM mode, as reported by loco_client_.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
int DriverUnitreeG1::get_fsm_mode()
{
    _ensure_high_level("DriverUnitreeG1::get_fsm_mode");
    return loco_client_.get_fsm_mode();
}

/**
 * @brief Returns the robot's current balance mode.
 * @return The balance mode, as reported by loco_client_.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
int DriverUnitreeG1::get_balance_mode()
{
    _ensure_high_level("DriverUnitreeG1::get_balance_mode");
    return loco_client_.get_balance_mode();
}

/**
 * @brief Returns the robot's current swing height.
 * @return The swing height, in meters, as reported by loco_client_.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
double DriverUnitreeG1::get_swing_height()
{
    _ensure_high_level("DriverUnitreeG1::get_swing_height");
    return loco_client_.get_swing_height();
}

/**
 * @brief Returns the robot's current stand height.
 * @return The stand height, in meters, as reported by loco_client_.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
double DriverUnitreeG1::get_stand_height()
{
    _ensure_high_level("DriverUnitreeG1::get_stand_height");
    return loco_client_.get_stand_height();
}

/**
 * @brief Returns the robot's current gait phase.
 * @return The gait phase values, as reported by loco_client_.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
std::vector<double> DriverUnitreeG1::get_phase()
{
    _ensure_high_level("DriverUnitreeG1::get_phase");
    return loco_client_.get_phase();
}

/**
 * @brief Sets the robot's FSM (finite state machine) ID.
 * @param fsm_id The FSM ID to request. See DriverUnitreeLocoClient's warning on
 *        firmware-dependent FSM IDs.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
void DriverUnitreeG1::set_fsm_id(const int& fsm_id)
{
    _ensure_high_level("DriverUnitreeG1::set_fsm_id");
    loco_client_.set_fsm_id(fsm_id);
}

/**
 * @brief Sets the robot's balance mode.
 * @param balance_mode The balance mode to request. See DriverUnitreeLocoClient's
 *        warning on firmware-dependent balance-mode values.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
void DriverUnitreeG1::set_balance_mode(const int& balance_mode)
{
    _ensure_high_level("DriverUnitreeG1::set_balance_mode");
    loco_client_.set_balance_mode(balance_mode);
}

/**
 * @brief Sets the robot's swing height.
 * @param swing_height The swing height to request, in meters.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
void DriverUnitreeG1::set_swing_height(const double& swing_height)
{
    _ensure_high_level("DriverUnitreeG1::set_swing_height");
    loco_client_.set_swing_height(swing_height);
}

/**
 * @brief Sets the robot's stand height.
 * @param stand_height The stand height to request, in meters.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
void DriverUnitreeG1::set_stand_height(const double& stand_height)
{
    _ensure_high_level("DriverUnitreeG1::set_stand_height");
    loco_client_.set_stand_height(stand_height);
}

/**
 * @brief Sets the robot's speed mode.
 * @param mode The speed mode to request.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
void DriverUnitreeG1::set_speed_mode(int mode)
{
    _ensure_high_level("DriverUnitreeG1::set_speed_mode");
    loco_client_.set_speed_mode(mode);
}

/**
 * @brief Sets the robot's target high-level velocities.
 * @details Not a joint position, so this is kept as std::array<double,3> rather than
 *          Eigen::VectorXd, per the VectorXd requirement applying specifically to
 *          joint positions.
 * @param target_high_level_velocities Target (vx, vy, yaw_rate).
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
void DriverUnitreeG1::set_target_high_level_velocities(const std::array<double,3>& target_high_level_velocities)
{
    _ensure_high_level("DriverUnitreeG1::set_target_high_level_velocities");
    loco_client_.set_target_high_level_velocities(target_high_level_velocities);
}

// --- Arm SDK control API ---

/**
 * @brief Requests the arm-control blend weight to ramp up toward 1.0 and starts
 *        target-position tracking. Non-blocking; see
 *        DriverUnitreeG1ArmSDK::enable_arm_control().
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
void DriverUnitreeG1::enable_arm_control()
{
    _ensure_high_level("DriverUnitreeG1::enable_arm_control");
    arm_sdk_.enable_arm_control();
}

/**
 * @brief Requests the arm-control blend weight to ramp down toward 0.0, holding the
 *        last commanded pose. Non-blocking; see
 *        DriverUnitreeG1ArmSDK::disable_arm_control().
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
void DriverUnitreeG1::disable_arm_control()
{
    _ensure_high_level("DriverUnitreeG1::disable_arm_control");
    arm_sdk_.disable_arm_control();
}

/**
 * @brief Whether arm control is currently requested to be engaged.
 * @return True if enable_arm_control() was called more recently than
 *         disable_arm_control()/deinitialize(). See
 *         DriverUnitreeG1ArmSDK::is_arm_control_enabled().
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
bool DriverUnitreeG1::is_arm_control_enabled() const
{
    _ensure_high_level("DriverUnitreeG1::is_arm_control_enabled");
    return arm_sdk_.is_arm_control_enabled();
}

/**
 * @brief Sets the left arm's 7 target joint positions, in radians.
 * @param target_positions Size-7 vector of target joint positions, in radians.
 * @throws std::invalid_argument if target_positions.size() != 7.
 * @throws std::logic_error if operation_mode_ == OPERATION_MODE::LOW_LEVEL (no
 *         low-level joint command path is available yet).
 */
void DriverUnitreeG1::set_left_arm_target_positions(const Eigen::VectorXd& target_positions)
{
    _validate_size(target_positions, 7, "DriverUnitreeG1::set_left_arm_target_positions");
    if (operation_mode_ == OPERATION_MODE::LOW_LEVEL)
    {
        _throw_low_level_unsupported("DriverUnitreeG1::set_left_arm_target_positions");
    }
    else
    {
        arm_sdk_.set_target_positions(
            DriverUnitreeArmSDK::LIMB::LEFT_ARM,
            std::vector<double>(target_positions.data(),
                                target_positions.data() + target_positions.size()));
    }
}

/**
 * @brief Sets the right arm's 7 target joint positions, in radians.
 * @param target_positions Size-7 vector of target joint positions, in radians.
 * @throws std::invalid_argument if target_positions.size() != 7.
 * @throws std::logic_error if operation_mode_ == OPERATION_MODE::LOW_LEVEL (no
 *         low-level joint command path is available yet).
 */
void DriverUnitreeG1::set_right_arm_target_positions(const Eigen::VectorXd& target_positions)
{
    _validate_size(target_positions, 7, "DriverUnitreeG1::set_right_arm_target_positions");
    if (operation_mode_ == OPERATION_MODE::LOW_LEVEL)
    {
        _throw_low_level_unsupported("DriverUnitreeG1::set_right_arm_target_positions");
    }else
    {
        arm_sdk_.set_target_positions(
            DriverUnitreeArmSDK::LIMB::RIGHT_ARM,
            std::vector<double>(target_positions.data(),
                                target_positions.data() + target_positions.size()));
    }
}

/**
 * @brief Sets the waist's 3 target joint positions, in radians.
 * @param target_positions Size-3 vector of target joint positions, in radians.
 * @throws std::invalid_argument if target_positions.size() != 3.
 * @throws std::logic_error if operation_mode_ == OPERATION_MODE::LOW_LEVEL (no
 *         low-level joint command path is available yet).
 */
void DriverUnitreeG1::set_waist_target_positions(const Eigen::VectorXd& target_positions)
{
    _validate_size(target_positions, 3, "DriverUnitreeG1::set_waist_target_positions");
    if (operation_mode_ == OPERATION_MODE::LOW_LEVEL)
    {
        _throw_low_level_unsupported("DriverUnitreeG1::set_waist_target_positions");
    }else{
        arm_sdk_.set_target_positions(
            DriverUnitreeArmSDK::LIMB::WAIST,
            std::vector<double>(target_positions.data(),
                                target_positions.data() + target_positions.size()));
    }
}

/**
 * @brief Returns the left arm's 7 measured joint positions.
 * @return Size-7 vector of measured joint positions, in radians.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
Eigen::VectorXd DriverUnitreeG1::get_left_arm_positions()
{
    _ensure_high_level("DriverUnitreeG1::get_left_arm_positions");
    return stdvector_to_eigen(arm_sdk_.get_positions(DriverUnitreeArmSDK::LIMB::LEFT_ARM));
}

/**
 * @brief Returns the right arm's 7 measured joint positions.
 * @return Size-7 vector of measured joint positions, in radians.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
Eigen::VectorXd DriverUnitreeG1::get_right_arm_positions()
{
    _ensure_high_level("DriverUnitreeG1::get_right_arm_positions");
    return stdvector_to_eigen(arm_sdk_.get_positions(DriverUnitreeArmSDK::LIMB::RIGHT_ARM));
}

/**
 * @brief Returns the waist's 3 measured joint positions.
 * @return Size-3 vector of measured joint positions, in radians.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
Eigen::VectorXd DriverUnitreeG1::get_waist_positions()
{
    _ensure_high_level("DriverUnitreeG1::get_waist_positions");
    return stdvector_to_eigen(arm_sdk_.get_positions(DriverUnitreeArmSDK::LIMB::WAIST));
}

/**
 * @brief Returns the left arm's 7 commanded trajectory-point positions.
 * @return Size-7 vector of commanded joint positions, in radians.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
Eigen::VectorXd DriverUnitreeG1::get_left_arm_desired_positions()
{
    _ensure_high_level("DriverUnitreeG1::get_left_arm_desired_positions");
    return stdvector_to_eigen(arm_sdk_.get_desired_positions(DriverUnitreeArmSDK::LIMB::LEFT_ARM));
}

/**
 * @brief Returns the right arm's 7 commanded trajectory-point positions.
 * @return Size-7 vector of commanded joint positions, in radians.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
Eigen::VectorXd DriverUnitreeG1::get_right_arm_desired_positions()
{
    _ensure_high_level("DriverUnitreeG1::get_right_arm_desired_positions");
    return stdvector_to_eigen(arm_sdk_.get_desired_positions(DriverUnitreeArmSDK::LIMB::RIGHT_ARM));
}

/**
 * @brief Returns the waist's 3 commanded trajectory-point positions.
 * @return Size-3 vector of commanded joint positions, in radians.
 * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
 */
Eigen::VectorXd DriverUnitreeG1::get_waist_desired_positions()
{
    _ensure_high_level("DriverUnitreeG1::get_waist_desired_positions");
    return stdvector_to_eigen(arm_sdk_.get_desired_positions(DriverUnitreeArmSDK::LIMB::WAIST));
}

// --- Low-level state telemetry API (available in both operation modes) ---

/**
 * @brief Returns the number of joint slots in the underlying rt/lowstate message.
 * @return 35 for G1 (0 if no rt/lowstate message has been received yet), as reported
 *         by low_state_.
 */
std::size_t DriverUnitreeG1::get_num_joints()
{
    return low_state_.num_joints();
}

/**
 * @brief Returns every joint's last measured position.
 * @return Vector of measured joint positions, in radians, sized to low_state_'s
 *         current joint count (empty if no rt/lowstate message has been received yet).
 */
Eigen::VectorXd DriverUnitreeG1::get_all_joint_positions()
{
    return stdvector_to_eigen(low_state_.get_joint_positions());
}

/**
 * @brief Returns every joint's last measured velocity.
 * @return Vector of measured joint velocities, in rad/s, sized to low_state_'s
 *         current joint count (empty if no rt/lowstate message has been received yet).
 */
Eigen::VectorXd DriverUnitreeG1::get_all_joint_velocities()
{
    return stdvector_to_eigen(low_state_.get_joint_velocities());
}

/**
 * @brief Returns every joint's last estimated torque.
 * @return Vector of estimated joint torques, in Nm, sized to low_state_'s current
 *         joint count (empty if no rt/lowstate message has been received yet).
 */
Eigen::VectorXd DriverUnitreeG1::get_all_joint_torques()
{
    return stdvector_to_eigen(low_state_.get_joint_torques());
}

/**
 * @brief Returns the last received IMU reading.
 * @return A DriverUnitreeLowState::IMUData; its `valid` field is false if no
 *         rt/lowstate message has been received yet.
 */
DriverUnitreeLowState::IMUData DriverUnitreeG1::get_imu_data()
{
    return low_state_.get_imu_data();
}

// --- Per-limb (and torso) telemetry API (available in both operation modes) ---

/**
 * @brief Returns the number of joints in the given limb (or torso).
 * @param limb Which limb (or torso) to query.
 * @return The joint count for @p limb, as reported by low_state_. A static property
 *         of the limb and G1's layout, so it doesn't require an rt/lowstate message
 *         to have been received yet.
 */
std::size_t DriverUnitreeG1::num_joints(const DriverUnitreeLowState::LIMB& limb) const
{
    return low_state_.num_joints(limb);
}

/**
 * @brief Returns the given limb's (or the torso's) last measured joint positions.
 * @param limb Which limb (or torso) to read.
 * @return An Eigen::VectorXd of joint positions, in radians, as reported by
 *         low_state_ (already Eigen::VectorXd there, so no conversion needed).
 *         Empty (size 0) if no rt/lowstate message has been received yet.
 */
Eigen::VectorXd DriverUnitreeG1::get_joint_positions(const DriverUnitreeLowState::LIMB& limb) const
{
    return low_state_.get_joint_positions(limb);
}

/**
 * @brief Returns the given limb's (or the torso's) last measured joint velocities.
 * @param limb Which limb (or torso) to read.
 * @return An Eigen::VectorXd of joint velocities, in rad/s, as reported by
 *         low_state_. Empty (size 0) if no rt/lowstate message has been received yet.
 */
Eigen::VectorXd DriverUnitreeG1::get_joint_velocities(const DriverUnitreeLowState::LIMB& limb) const
{
    return low_state_.get_joint_velocities(limb);
}

/**
 * @brief Returns the given limb's (or the torso's) last estimated joint torques.
 * @param limb Which limb (or torso) to read.
 * @return An Eigen::VectorXd of estimated joint torques, in Nm, as reported by
 *         low_state_. Empty (size 0) if no rt/lowstate message has been received yet.
 */
Eigen::VectorXd DriverUnitreeG1::get_joint_torques(const DriverUnitreeLowState::LIMB& limb) const
{
    return low_state_.get_joint_torques(limb);
}
