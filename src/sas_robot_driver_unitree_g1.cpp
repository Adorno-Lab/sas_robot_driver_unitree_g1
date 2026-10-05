#include <sas_robot_driver_unitree_g1/sas_robot_driver_unitree_g1.hpp>
#include "DriverUnitreeG1.h"
#include "RobotDriverUnitreeG1Limb.h"
#include <stdexcept>

namespace sas
{

namespace
{

using LIMB = DriverUnitreeLowState::LIMB;

/**
 * @brief One limb of the G1 (29 DoF). The joint names and their order are the ones of unitree_drivers'
 *        DriverUnitreeLowState (unitree_sdk2's G1JointIndex). The joint limits, in radians, are taken
 *        from unitree_ros/robots/g1_description/g1_29dof_rev_1_0.urdf (identical in g1_29dof.urdf).
 */
struct G1Limb
{
    std::string name;
    LIMB limb;
    bool commandable_in_standing;
    std::vector<std::string> joint_names;
    std::vector<double> lower_limits;
    std::vector<double> upper_limits;
};

const std::vector<G1Limb> kG1Limbs = {
    {"left_leg", LIMB::LEFT_LEG, false,
     {"LeftHipPitch", "LeftHipRoll", "LeftHipYaw", "LeftKnee", "LeftAnklePitch", "LeftAnkleRoll"},
     {-2.5307, -0.5236, -2.7576, -0.087267, -0.87267, -0.2618},
     { 2.8798,  2.9671,  2.7576,  2.8798,    0.5236,   0.2618}},
    {"right_leg", LIMB::RIGHT_LEG, false,
     {"RightHipPitch", "RightHipRoll", "RightHipYaw", "RightKnee", "RightAnklePitch", "RightAnkleRoll"},
     {-2.5307, -2.9671, -2.7576, -0.087267, -0.87267, -0.2618},
     { 2.8798,  0.5236,  2.7576,  2.8798,    0.5236,   0.2618}},
    {"waist", LIMB::WAIST, true,
     {"WaistYaw", "WaistRoll", "WaistPitch"},
     {-2.618, -0.52, -0.52},
     { 2.618,  0.52,  0.52}},
    {"left_arm", LIMB::LEFT_ARM, true,
     {"LeftShoulderPitch", "LeftShoulderRoll", "LeftShoulderYaw", "LeftElbow", "LeftWristRoll", "LeftWristPitch", "LeftWristYaw"},
     {-3.0892, -1.5882, -2.618, -1.0472, -1.972222054, -1.614429558, -1.614429558},
     { 2.6704,  2.2515,  2.618,  2.0944,  1.972222054,  1.614429558,  1.614429558}},
    {"right_arm", LIMB::RIGHT_ARM, true,
     {"RightShoulderPitch", "RightShoulderRoll", "RightShoulderYaw", "RightElbow", "RightWristRoll", "RightWristPitch", "RightWristYaw"},
     {-3.0892, -2.2515, -2.618, -1.0472, -1.972222054, -1.614429558, -1.614429558},
     { 2.6704,  1.5882,  2.618,  2.0944,  1.972222054,  1.614429558,  1.614429558}},
};

VectorXd _to_vectorxd(const std::vector<double>& values)
{
    return Eigen::Map<const VectorXd>(values.data(), static_cast<Eigen::Index>(values.size()));
}

}

class RobotDriverUnitreeG1::Impl
{
public:
    RobotDriverUnitreeG1Configuration::CONTROL_LEVEL control_level_;
    std::shared_ptr<DriverUnitreeG1> hardware_;
    std::vector<LimbEntry> limbs_;
    std::vector<bool> commandable_in_standing_;
};

RobotDriverUnitreeG1::~RobotDriverUnitreeG1()=default;

RobotDriverUnitreeG1::RobotDriverUnitreeG1(const RobotDriverUnitreeG1Configuration &configuration,
                                           const std::shared_ptr<ShutdownSignaler> &shutdown_signaler)
    :LeggedRobotDriver{shutdown_signaler},
    impl_{std::make_unique<RobotDriverUnitreeG1::Impl>()}
{
    impl_->control_level_ = configuration.control_level;
    if (impl_->control_level_ == RobotDriverUnitreeG1Configuration::CONTROL_LEVEL::LOW_LEVEL)
        throw std::runtime_error("RobotDriverUnitreeG1: the low-level control (commands through rt/lowcmd) "
                                 "is not implemented yet. Use the high-level control.");

    impl_->hardware_ = std::make_shared<DriverUnitreeG1>(shutdown_signaler,
                                                         DriverUnitreeG1::OPERATION_MODE::HIGH_LEVEL,
                                                         configuration.domain_id,
                                                         configuration.network_interface);
    for (const auto& g1_limb : kG1Limbs)
    {
        if (impl_->hardware_->num_joints(g1_limb.limb) != g1_limb.joint_names.size())
            throw std::logic_error("RobotDriverUnitreeG1: unitree_drivers reports " +
                                   std::to_string(impl_->hardware_->num_joints(g1_limb.limb)) + " joints for " +
                                   g1_limb.name + ", but this driver expects " +
                                   std::to_string(g1_limb.joint_names.size()) + ".");

        auto driver = std::make_shared<RobotDriverUnitreeG1Limb>(impl_->hardware_,
                                                                 g1_limb.limb,
                                                                 std::make_tuple(_to_vectorxd(g1_limb.lower_limits),
                                                                                 _to_vectorxd(g1_limb.upper_limits)),
                                                                 shutdown_signaler);
        impl_->limbs_.push_back({g1_limb.name, driver, g1_limb.joint_names});
        impl_->commandable_in_standing_.push_back(g1_limb.commandable_in_standing);
    }
}

void RobotDriverUnitreeG1::connect()
{
    impl_->hardware_->connect();
}

void RobotDriverUnitreeG1::disconnect()
{
    impl_->hardware_->disconnect();
}

void RobotDriverUnitreeG1::initialize()
{
    impl_->hardware_->initialize();
}

void RobotDriverUnitreeG1::deinitialize()
{
    impl_->hardware_->deinitialize();
}

void RobotDriverUnitreeG1::_set_high_level_mode(const HIGH_LEVEL_MODE &mode)
{
    // Stop before any mode change.
    impl_->hardware_->set_target_high_level_velocities({0.0, 0.0, 0.0});
    switch (mode)
    {
    case HIGH_LEVEL_MODE::IDLE:
        // Keep the current arm state: switching arm control off would move the arms back to
        // Unitree's default pose.
        return;
    case HIGH_LEVEL_MODE::STANDING:
        impl_->hardware_->set_mode(DriverUnitreeG1::HIGH_MODE::ARM_CONTROL);
        return;
    case HIGH_LEVEL_MODE::WALKING:
        impl_->hardware_->set_mode(DriverUnitreeG1::HIGH_MODE::LOCOMOTION);
        return;
    }
}

void RobotDriverUnitreeG1::set_target_twist(const DQ &twist)
{
    //    0  1  2  3  4  5
    //   wx wy wz vx vy vz
    const VectorXd twist_vec = twist.vec6();
    impl_->hardware_->set_target_high_level_velocities({twist_vec(3), twist_vec(4), twist_vec(2)});
}

void RobotDriverUnitreeG1::set_target_base_orientation([[maybe_unused]] const DQ &r)
{
    throw std::logic_error("RobotDriverUnitreeG1::set_target_base_orientation: not supported.");
}

void RobotDriverUnitreeG1::set_target_base_height([[maybe_unused]] const double &base_height)
{
    throw std::logic_error("RobotDriverUnitreeG1::set_target_base_height: not supported.");
}

DQ RobotDriverUnitreeG1::get_orientation()
{
    const auto imu_data = impl_->hardware_->get_imu_data();
    const DQ r(imu_data.quaternion.at(0),  // w
               imu_data.quaternion.at(1),  // x
               imu_data.quaternion.at(2),  // y
               imu_data.quaternion.at(3)); // z
    if (!imu_data.valid || r.vec4().norm() == 0.0)
        return DQ(0);
    return r.normalize();
}

DQ RobotDriverUnitreeG1::get_angular_velocity()
{
    const auto imu_data = impl_->hardware_->get_imu_data();
    if (!imu_data.valid)
        return DQ(0);
    return DQ(0, imu_data.gyroscope.at(0), imu_data.gyroscope.at(1), imu_data.gyroscope.at(2));
}

DQ RobotDriverUnitreeG1::get_linear_acceleration()
{
    const auto imu_data = impl_->hardware_->get_imu_data();
    if (!imu_data.valid)
        return DQ(0);
    return DQ(0, imu_data.accelerometer.at(0), imu_data.accelerometer.at(1), imu_data.accelerometer.at(2));
}

std::vector<LeggedRobotDriver::HIGH_LEVEL_MODE> RobotDriverUnitreeG1::get_supported_high_level_modes() const
{
    if (impl_->control_level_ == RobotDriverUnitreeG1Configuration::CONTROL_LEVEL::LOW_LEVEL)
        return {HIGH_LEVEL_MODE::IDLE, HIGH_LEVEL_MODE::STANDING};  // Nothing walks in low-level control.
    return {HIGH_LEVEL_MODE::IDLE, HIGH_LEVEL_MODE::STANDING, HIGH_LEVEL_MODE::WALKING};
}

bool RobotDriverUnitreeG1::is_supported(const LEGGED_FUNCTIONALITY &functionality) const
{
    if (impl_->control_level_ == RobotDriverUnitreeG1Configuration::CONTROL_LEVEL::LOW_LEVEL)
        return false;  // No locomotion controller: no twist, base height, or base orientation.
    return functionality == LEGGED_FUNCTIONALITY::TWIST;
}

std::vector<LeggedRobotDriver::LimbEntry> RobotDriverUnitreeG1::get_limbs() const
{
    return impl_->limbs_;
}

std::vector<bool> RobotDriverUnitreeG1::get_commandable_limbs() const
{
    if (current_mode_ != HIGH_LEVEL_MODE::STANDING)
        return std::vector<bool>(impl_->limbs_.size(), false);
    if (impl_->control_level_ == RobotDriverUnitreeG1Configuration::CONTROL_LEVEL::LOW_LEVEL)
        return std::vector<bool>(impl_->limbs_.size(), true);  // Every limb, the legs included.
    return impl_->commandable_in_standing_;
}

}
