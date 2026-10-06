#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <sas_core/sas_shutdown_signaler.hpp>
#include <sas_tools/LeggedRobotDriver.hpp>

namespace sas
{

struct RobotDriverUnitreeG1Configuration
{
    /**
     * @brief How the driver commands the robot. Both levels read the state of the robot (joints and
     *        IMU) from rt/lowstate; they only differ in how the commands are sent.
     *   - HIGH_LEVEL: through Unitree's locomotion controller (loco client, for the twist) and
     *     rt/arm_sdk (for the waist and the arms).
     *   - LOW_LEVEL: through rt/lowcmd, for every joint (no loco client and no rt/arm_sdk).
     *     Not implemented yet.
     */
    enum class CONTROL_LEVEL{
        HIGH_LEVEL=0,
        LOW_LEVEL,
    };

    int32_t domain_id;              ///< DDS domain: 0 for the real robot, 1 for the simulation.
    std::string network_interface;  ///< e.g. "eth0" on the onboard computer, or "enp6s0" on a desktop.
    CONTROL_LEVEL control_level{CONTROL_LEVEL::HIGH_LEVEL};
};

/**
 * @brief The RobotDriverUnitreeG1 class is the LeggedRobotDriver of the Unitree G1 (29 DoF). It does
 *        not use ROS; run it with LeggedRobotDriverROS.
 *
 * The state of the robot (joints and IMU) is always read from rt/lowstate. The capabilities depend on
 * RobotDriverUnitreeG1Configuration::CONTROL_LEVEL, which sets how the commands are sent. Only
 * HIGH_LEVEL is implemented: Unitree's locomotion controller moves the legs, and rt/arm_sdk moves the
 * arms and the waist. The rest of this description is about HIGH_LEVEL. (LOW_LEVEL is planned to command every
 * limb through rt/lowcmd, in IDLE and STANDING only, with no twist, base height, or base orientation.)
 *
 * Limbs (served on \<prefix\>/\<name\>), with the joint layout of unitree_drivers'
 * DriverUnitreeLowState::LIMB:
 *   - left_leg (6), right_leg (6): never commandable (moved by the locomotion controller).
 *   - waist (3), left_arm (7), right_arm (7): commandable in STANDING only.
 *
 * Modes:
 *   - IDLE: zero velocity; the arms keep their current state (arm control is not switched off,
 *     so that they do not move back to Unitree's default pose).
 *   - STANDING: arm control (DriverUnitreeG1::HIGH_MODE::ARM_CONTROL); the robot cannot walk.
 *   - WALKING: locomotion (DriverUnitreeG1::HIGH_MODE::LOCOMOTION); the twist is accepted.
 *
 * Every mode change first stops the robot (zero velocity). Only the twist is supported:
 * neither the base height nor the base orientation can be commanded.
 */
class RobotDriverUnitreeG1: public LeggedRobotDriver
{
private:
    class Impl;
    std::unique_ptr<Impl> impl_;

protected:
    void _set_high_level_mode(const HIGH_LEVEL_MODE& mode) override;

public:
    ~RobotDriverUnitreeG1();
    RobotDriverUnitreeG1()=delete;
    RobotDriverUnitreeG1(const RobotDriverUnitreeG1&) = delete;
    RobotDriverUnitreeG1& operator=(const RobotDriverUnitreeG1&) = delete;
    RobotDriverUnitreeG1(RobotDriverUnitreeG1&&) = delete;
    RobotDriverUnitreeG1& operator=(RobotDriverUnitreeG1&&) = delete;

    /**
     * @brief RobotDriverUnitreeG1 Creates the driver and its five limbs. It does not connect yet.
     * @param configuration The DDS domain, the network interface, and the control level.
     * @param shutdown_signaler Shared with LeggedRobotDriverROS.
     * @throws std::runtime_error if the control level is LOW_LEVEL, which is not implemented yet.
     * @throws std::logic_error if the joint layout of unitree_drivers does not match the limb table
     *         of this driver.
     */
    RobotDriverUnitreeG1(const RobotDriverUnitreeG1Configuration& configuration,
                         const std::shared_ptr<ShutdownSignaler>& shutdown_signaler);

    void connect() override;
    void disconnect() override;
    void initialize() override;
    void deinitialize() override;

    /**
     * @brief set_target_twist Sets the target velocities of the locomotion controller.
     * @param twist Only wz (yaw rate), vx and vy are used. Ignored by the robot while arm control
     *        is engaged (STANDING), so a zero twist is accepted in every mode.
     */
    void set_target_twist(const DQ& twist) override;

    /**
     * @brief set_target_base_orientation Not supported by the G1 driver.
     * @throws std::logic_error always (LeggedRobotDriverROS never calls it, since is_supported() is false).
     */
    void set_target_base_orientation(const DQ& r) override;

    /**
     * @brief set_target_base_height Not supported by the G1 driver.
     * @throws std::logic_error always (LeggedRobotDriverROS never calls it, since is_supported() is false).
     */
    void set_target_base_height(const double& base_height) override;

    /**
     * @brief get_orientation Returns the IMU orientation (rt/lowstate).
     * @return A unit quaternion, or DQ(0) until the first rt/lowstate message arrives.
     */
    DQ get_orientation() override;

    /**
     * @brief get_angular_velocity Returns the IMU angular velocity, in rad/s, in the body frame.
     * @return A pure quaternion, or DQ(0) until the first rt/lowstate message arrives.
     */
    DQ get_angular_velocity() override;

    /**
     * @brief get_linear_acceleration Returns the IMU linear acceleration (with gravity), in m/s^2,
     *        in the body frame.
     * @return A pure quaternion, or DQ(0) until the first rt/lowstate message arrives.
     */
    DQ get_linear_acceleration() override;

    /**
     * @brief get_supported_high_level_modes Returns IDLE, STANDING, and WALKING in HIGH_LEVEL, and IDLE
     *        and STANDING in LOW_LEVEL.
     */
    std::vector<HIGH_LEVEL_MODE> get_supported_high_level_modes() const override;

    /**
     * @brief is_supported Returns true only for LEGGED_FUNCTIONALITY::TWIST in HIGH_LEVEL, and false
     *        for every functionality in LOW_LEVEL.
     */
    bool is_supported(const LEGGED_FUNCTIONALITY& functionality) const override;

    /**
     * @brief get_limbs Returns left_leg, right_leg, waist, left_arm, and right_arm, in this order.
     */
    std::vector<LimbEntry> get_limbs() const override;

    /**
     * @brief get_commandable_limbs In STANDING, the waist and the arms in HIGH_LEVEL (the legs are moved
     *        by the locomotion controller), and every limb in LOW_LEVEL. None in the other modes.
     * @return One entry per limb of get_limbs().
     */
    std::vector<bool> get_commandable_limbs() const override;
};

}
