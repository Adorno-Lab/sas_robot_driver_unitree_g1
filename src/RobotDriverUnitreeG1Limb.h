#pragma once
#include <memory>
#include <tuple>
#include <sas_core/sas_robot_driver.hpp>
#include "DriverUnitreeG1.h"

namespace sas
{

/**
 * @brief The RobotDriverUnitreeG1Limb class is a RobotDriver view of one limb of the Unitree G1
 *        (a leg, the waist, or an arm), on top of the DriverUnitreeG1 shared by every limb.
 *
 * The joint layout of each limb is the one of unitree_drivers' DriverUnitreeLowState::LIMB. The
 * lifecycle (connect(), initialize(), ...) belongs to RobotDriverUnitreeG1, so it is a no-op here.
 */
class RobotDriverUnitreeG1Limb: public RobotDriver
{
private:
    std::shared_ptr<DriverUnitreeG1> hardware_;
    DriverUnitreeLowState::LIMB limb_;

public:
    RobotDriverUnitreeG1Limb()=delete;
    RobotDriverUnitreeG1Limb(const RobotDriverUnitreeG1Limb&)=delete;

    /**
     * @brief RobotDriverUnitreeG1Limb
     * @param hardware The driver of the G1, shared by every limb.
     * @param limb The limb of this view.
     * @param joint_limits {min, max} joint positions of the limb, in radians.
     * @param shutdown_signaler Shared with RobotDriverUnitreeG1.
     */
    RobotDriverUnitreeG1Limb(const std::shared_ptr<DriverUnitreeG1>& hardware,
                             const DriverUnitreeLowState::LIMB& limb,
                             const std::tuple<Eigen::VectorXd, Eigen::VectorXd>& joint_limits,
                             const std::shared_ptr<ShutdownSignaler>& shutdown_signaler);

    /**
     * @brief get_joint_positions Returns the joint positions of the limb, in radians. Empty until the
     *        first rt/lowstate message arrives.
     */
    Eigen::VectorXd get_joint_positions() override;

    /**
     * @brief get_joint_velocities Returns the joint velocities of the limb, in rad/s. Empty until the
     *        first rt/lowstate message arrives.
     */
    Eigen::VectorXd get_joint_velocities() override;

    /**
     * @brief get_joint_torques Returns the estimated joint torques of the limb, in Nm. Empty until the
     *        first rt/lowstate message arrives.
     */
    Eigen::VectorXd get_joint_torques() override;

    /**
     * @brief set_target_joint_positions Sends the targets of the waist or an arm to rt/arm_sdk.
     *        LeggedRobotDriverROS only calls it while the limb is commandable (STANDING).
     * @param target_joint_positions One target per joint of the limb, in radians.
     * @throws std::logic_error for a leg (the legs are moved by the locomotion controller).
     * @throws std::invalid_argument if the size does not match the limb.
     */
    void set_target_joint_positions(const Eigen::VectorXd& target_joint_positions) override;

    void connect() override;
    void disconnect() override;
    void initialize() override;
    void deinitialize() override;
};

}
