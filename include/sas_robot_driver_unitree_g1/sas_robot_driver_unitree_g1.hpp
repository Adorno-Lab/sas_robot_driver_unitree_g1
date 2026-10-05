#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <dqrobotics/DQ.h>
#include <sas_tools/LeggedRobotDriver.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

// The ones to keep
#include <marinholab/sas/core/sas_robot_driver.hpp>
#include <marinholab/sas/core/sas_shutdown_signaler.hpp>

// Trouxa headers
#include <sas_core/sas_robot_driver.hpp>
#include <sas_core/sas_shutdown_signaler.hpp>


using namespace DQ_robotics;

namespace sas
{

struct RobotDriverUnitreeG1Configuration
{
    int32_t domain_id;  // 0: real robot   1: simulation
    std::string network_interface; //"enp6s0"
    std::string topic_prefix;

};

class RobotDriverUnitreeG1: public LeggedRobotDriver
{
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
private:
    Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_left_arm_joint_states_;
    Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_right_arm_joint_states_;
    Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_left_leg_joint_states_;
    Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_right_leg_joint_states_;
    Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_torso_joint_states_;
    void _read_joint_states_and_publish();

    Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_IMU_state_;
    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_IMU_orientation_;
    void _read_imu_state_and_publish();
    void _set_target_velocities_from_subscriber();

    Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_target_twist_;
    std::array<double, 6> target_twist_{};
    void _callback_target_twist(const geometry_msgs::msg::TwistStamped& msg);
    bool new_target_twist_available_{false};

    Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscriber_mode_switch_;
    void _callback_mode_switch(const std_msgs::msg::Int32MultiArray& msg);
    LeggedRobotDriver::HIGH_LEVEL_MODE target_mode_;

protected:
    RobotDriverUnitreeG1Configuration configuration_;
    std::shared_ptr<rclcpp::Node> node_;
    double time_step_ = 0.002;  // Used for low-state

    std::string high_level_mode_to_string(const LeggedRobotDriver::HIGH_LEVEL_MODE& mode) const;

public:
    ~RobotDriverUnitreeG1();
    RobotDriverUnitreeG1()=delete;
    // Delete copy constructor and assignment (prevents double initialization)
    RobotDriverUnitreeG1(const RobotDriverUnitreeG1&) = delete;
    RobotDriverUnitreeG1& operator=(const RobotDriverUnitreeG1&) = delete;
    RobotDriverUnitreeG1(RobotDriverUnitreeG1&&) = delete;
    RobotDriverUnitreeG1& operator=(RobotDriverUnitreeG1&&) = delete;

    RobotDriverUnitreeG1(std::shared_ptr<rclcpp::Node> &node,
                         const RobotDriverUnitreeG1Configuration &configuration,
                         const std::shared_ptr<sas::ShutdownSignaler> &shutdown_signaler);

    VectorXd get_joint_positions() override;
    void set_target_joint_positions(const VectorXd& desired_joint_positions_rad) override;

    //void set_target_joint_velocities(const VectorXd& desired_joint_velocities_rad_s) override;

    VectorXd get_joint_velocities() override;
    VectorXd get_joint_torques() override;

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;

    // pure virtual methods from LeggedRobotDriver

    void set_target_twist(const DQ& twist) override;
    void set_target_base_orientation(const DQ& r) override;
    void set_target_base_height(const double& base_height) override;

    DQ get_orientation() override;
    DQ get_angular_velocity() override;
    DQ get_linear_acceleration() override;

};
}
