#include "RobotDriverUnitreeG1Limb.h"
#include <stdexcept>

namespace sas
{

RobotDriverUnitreeG1Limb::RobotDriverUnitreeG1Limb(const std::shared_ptr<DriverUnitreeG1> &hardware,
                                                   const DriverUnitreeLowState::LIMB &limb,
                                                   const std::tuple<Eigen::VectorXd, Eigen::VectorXd> &joint_limits,
                                                   const std::shared_ptr<ShutdownSignaler> &shutdown_signaler):
    RobotDriver(shutdown_signaler),
    hardware_(hardware),
    limb_(limb)
{
    set_joint_limits(joint_limits);
}

Eigen::VectorXd RobotDriverUnitreeG1Limb::get_joint_positions()
{
    return hardware_->get_joint_positions(limb_);
}

Eigen::VectorXd RobotDriverUnitreeG1Limb::get_joint_velocities()
{
    return hardware_->get_joint_velocities(limb_);
}

Eigen::VectorXd RobotDriverUnitreeG1Limb::get_joint_torques()
{
    return hardware_->get_joint_torques(limb_);
}

void RobotDriverUnitreeG1Limb::set_target_joint_positions(const Eigen::VectorXd &target_joint_positions)
{
    switch (limb_)
    {
    case DriverUnitreeLowState::LIMB::LEFT_ARM:
        hardware_->set_left_arm_target_positions(target_joint_positions);
        return;
    case DriverUnitreeLowState::LIMB::RIGHT_ARM:
        hardware_->set_right_arm_target_positions(target_joint_positions);
        return;
    case DriverUnitreeLowState::LIMB::WAIST:
        hardware_->set_waist_target_positions(target_joint_positions);
        return;
    case DriverUnitreeLowState::LIMB::LEFT_LEG:
    case DriverUnitreeLowState::LIMB::RIGHT_LEG:
        break;
    }
    throw std::logic_error("RobotDriverUnitreeG1Limb::set_target_joint_positions: the legs are moved by the "
                           "locomotion controller and cannot be commanded in high-level control.");
}

// The lifecycle of the hardware belongs to RobotDriverUnitreeG1.
void RobotDriverUnitreeG1Limb::connect()
{

}

void RobotDriverUnitreeG1Limb::disconnect()
{

}

void RobotDriverUnitreeG1Limb::initialize()
{

}

void RobotDriverUnitreeG1Limb::deinitialize()
{

}

}
