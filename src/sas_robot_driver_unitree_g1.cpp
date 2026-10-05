#include <sas_robot_driver_unitree_g1/sas_robot_driver_unitree_g1.hpp>
#include "DriverUnitreeG1.h"
#include <marinholab/sas/core/sas_thread_manager.hpp>
#include <sas_conversions/DQ_geometry_msgs_conversions.hpp>
#include <sas_core/eigen3_std_conversions.hpp>

namespace sas
{

class RobotDriverUnitreeG1::Impl
{

public:
    std::unique_ptr<DriverUnitreeG1> unitree_g1_driver_;
    Impl()
    {

    };
};




RobotDriverUnitreeG1::~RobotDriverUnitreeG1()
{

}

RobotDriverUnitreeG1::RobotDriverUnitreeG1(std::shared_ptr<rclcpp::Node> &node,
                                           const RobotDriverUnitreeG1Configuration &configuration,
                                           const std::shared_ptr<sas::ShutdownSignaler> &shutdown_signaler)
:LeggedRobotDriver{shutdown_signaler},
configuration_{configuration},
node_{node}
{
    impl_ = std::make_unique<RobotDriverUnitreeG1::Impl>();

    impl_->unitree_g1_driver_ = std::make_unique<DriverUnitreeG1>(shutdown_signaler,
                                                                  DriverUnitreeG1::OPERATION_MODE::HIGH_LEVEL,
                                                                  configuration_.domain_id,
                                                                  configuration_.network_interface);

    publisher_left_arm_joint_states_  = node_->create_publisher<sensor_msgs::msg::JointState>(configuration.topic_prefix + "/get/left_arm_joint_states",1);
    publisher_right_arm_joint_states_ = node_->create_publisher<sensor_msgs::msg::JointState>(configuration.topic_prefix + "/get/right_arm_joint_states",1);
    publisher_left_leg_joint_states_  = node_->create_publisher<sensor_msgs::msg::JointState>(configuration.topic_prefix + "/get/left_leg_joint_states",1);
    publisher_right_leg_joint_states_ = node_->create_publisher<sensor_msgs::msg::JointState>(configuration.topic_prefix + "/get/right_leg_joint_states",1);
    publisher_torso_joint_states_     = node_->create_publisher<sensor_msgs::msg::JointState>(configuration.topic_prefix + "/get/torso_joint_states",1);

    publisher_IMU_state_ = node_->create_publisher<sensor_msgs::msg::Imu>(configuration_.topic_prefix + "/get/imu_state", 1);
    publisher_IMU_orientation_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(configuration_.topic_prefix  + "/get/imu_orientation",1);

    subscriber_target_twist_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        configuration.topic_prefix + "/set/target_twist",
        1,
        std::bind(&RobotDriverUnitreeG1::_callback_target_twist, this, std::placeholders::_1)
        );


    subscriber_mode_switch_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        configuration.topic_prefix + "/set/mode",
        1,
        std::bind(&RobotDriverUnitreeG1::_callback_mode_switch, this, std::placeholders::_1)
        );


    // Define the callback for the sas::RobotDriverROS control loop
    set_control_loop_callback([this]() {
        try {
            _read_joint_states_and_publish();
            _read_imu_state_and_publish();
            _set_target_velocities_from_subscriber();
        } catch (...) {}
    });
    current_mode_ = LeggedRobotDriver::HIGH_LEVEL_MODE::WALKING;
    target_mode_ = LeggedRobotDriver::HIGH_LEVEL_MODE::WALKING;
}

VectorXd RobotDriverUnitreeG1::get_joint_positions()
{
    return impl_->unitree_g1_driver_->get_all_joint_positions();
}

void RobotDriverUnitreeG1::set_target_joint_positions(const VectorXd &desired_joint_positions_rad)
{
    // desired_joint_positions_rad = [q_left;
    //                                q_right;
    //                                q_torso];
/*
ros2 topic pub --once /sas_g1/g1_1/set/target_joint_positions std_msgs/msg/Float64MultiArray \
"{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0]}"
*/

    constexpr Eigen::Index kNumArmJoints = 7;
    constexpr Eigen::Index kNumWaistJoints = 3;
    constexpr Eigen::Index kExpectedSize = 2 * kNumArmJoints + kNumWaistJoints;

    if (desired_joint_positions_rad.size() != kExpectedSize)
    {
        throw std::invalid_argument(
            "RobotDriverUnitreeG1::set_target_joint_positions: expected a vector of size " +
            std::to_string(kExpectedSize) + ", got size " +
            std::to_string(desired_joint_positions_rad.size()));
    }

    // segment(start, length)
    const VectorXd q_left  = desired_joint_positions_rad.segment(0, kNumArmJoints);
    const VectorXd q_right = desired_joint_positions_rad.segment(kNumArmJoints, kNumArmJoints);
    const VectorXd q_torso = desired_joint_positions_rad.segment(2 * kNumArmJoints, kNumWaistJoints);

    if (impl_->unitree_g1_driver_->is_arm_control_enabled())
    {
        impl_->unitree_g1_driver_->set_left_arm_target_positions(q_left);
        impl_->unitree_g1_driver_->set_right_arm_target_positions(q_right);
        impl_->unitree_g1_driver_->set_waist_target_positions(q_torso);
    }
}

VectorXd RobotDriverUnitreeG1::get_joint_velocities()
{
    return impl_->unitree_g1_driver_->get_all_joint_velocities();
}

VectorXd RobotDriverUnitreeG1::get_joint_torques()
{
    return impl_->unitree_g1_driver_->get_all_joint_torques();
}

void RobotDriverUnitreeG1::connect()
{
    impl_->unitree_g1_driver_->connect();
    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Connected");
}

void RobotDriverUnitreeG1::disconnect()
{
    impl_->unitree_g1_driver_->disconnect();
    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Disconnected");
}

void RobotDriverUnitreeG1::initialize()
{
    impl_->unitree_g1_driver_->initialize();
    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Initialized");
}

void RobotDriverUnitreeG1::deinitialize()
{
    impl_->unitree_g1_driver_->deinitialize();
    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Deinitialized.");
}

void RobotDriverUnitreeG1::set_target_twist(const DQ &twist)
{
    /*
    ros2 topic pub --once /sas_g1/g1_1/set/target_twist geometry_msgs/msg/TwistStamped \
    "{header: {frame_id: ''}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
     */
    const VectorXd twist_vec = twist.vec6();
    //    0  1  2  3  4  5
    //   wx wy wz vx vy vz
    const double& vx = twist_vec(3);
    const double& vy = twist_vec(4);
    const double& wz = twist_vec(2);
    impl_->unitree_g1_driver_->set_target_high_level_velocities({vx,vy,wz});
}

void RobotDriverUnitreeG1::set_target_base_orientation([[maybe_unused]] const DQ &r)
{
    throw std::runtime_error("RobotDriverUnitreeG1::set_target_base_orientation not available!");
}

void RobotDriverUnitreeG1::set_target_base_height([[maybe_unused]] const double &base_height)
{
    throw std::runtime_error("RobotDriverUnitreeG1::set_target_base_height not available!");
}

DQ RobotDriverUnitreeG1::get_orientation()
{
    auto imu_data = impl_->unitree_g1_driver_->get_imu_data();
    return DQ(
    imu_data.quaternion.at(0), //w
    imu_data.quaternion.at(1), //x
    imu_data.quaternion.at(2), //y
    imu_data.quaternion.at(3)).normalize(); //z
}

DQ RobotDriverUnitreeG1::get_angular_velocity()
{
    auto imu_data = impl_->unitree_g1_driver_->get_imu_data();
    return DQ(0,
              imu_data.gyroscope.at(0),
              imu_data.gyroscope.at(1),
              imu_data.gyroscope.at(2));
}

DQ RobotDriverUnitreeG1::get_linear_acceleration()
{
    auto imu_data = impl_->unitree_g1_driver_->get_imu_data();
    return DQ(0,
              imu_data.accelerometer.at(0),
              imu_data.accelerometer.at(1),
              imu_data.accelerometer.at(2));
}

void RobotDriverUnitreeG1::_read_joint_states_and_publish()
{
    sensor_msgs::msg::JointState ros_msg_left_arm;
    sensor_msgs::msg::JointState ros_msg_right_arm;
    sensor_msgs::msg::JointState ros_msg_left_leg;
    sensor_msgs::msg::JointState ros_msg_right_leg;
    sensor_msgs::msg::JointState ros_msg_torso;

    const auto now = node_->get_clock()->now();
    ros_msg_left_arm.header.stamp  = now;
    ros_msg_right_arm.header.stamp = now;
    ros_msg_left_leg.header.stamp  = now;
    ros_msg_right_leg.header.stamp = now;
    ros_msg_torso.header.stamp     = now;

    VectorXd q_left_arm      = impl_->unitree_g1_driver_->get_joint_positions(DriverUnitreeLowState::LIMB::LEFT_ARM);
    VectorXd q_left_arm_dot  = impl_->unitree_g1_driver_->get_joint_velocities(DriverUnitreeLowState::LIMB::LEFT_ARM);
    VectorXd q_left_arm_tau  = impl_->unitree_g1_driver_->get_joint_torques(DriverUnitreeLowState::LIMB::LEFT_ARM);

    VectorXd q_right_arm     = impl_->unitree_g1_driver_->get_joint_positions(DriverUnitreeLowState::LIMB::RIGHT_ARM);
    VectorXd q_right_arm_dot = impl_->unitree_g1_driver_->get_joint_velocities(DriverUnitreeLowState::LIMB::RIGHT_ARM);
    VectorXd q_right_arm_tau = impl_->unitree_g1_driver_->get_joint_torques(DriverUnitreeLowState::LIMB::RIGHT_ARM);

    VectorXd q_left_leg      = impl_->unitree_g1_driver_->get_joint_positions(DriverUnitreeLowState::LIMB::LEFT_LEG);
    VectorXd q_left_leg_dot  = impl_->unitree_g1_driver_->get_joint_velocities(DriverUnitreeLowState::LIMB::LEFT_LEG);
    VectorXd q_left_leg_tau  = impl_->unitree_g1_driver_->get_joint_torques(DriverUnitreeLowState::LIMB::LEFT_LEG);

    VectorXd q_right_leg     = impl_->unitree_g1_driver_->get_joint_positions(DriverUnitreeLowState::LIMB::RIGHT_LEG);
    VectorXd q_right_leg_dot = impl_->unitree_g1_driver_->get_joint_velocities(DriverUnitreeLowState::LIMB::RIGHT_LEG);
    VectorXd q_right_leg_tau = impl_->unitree_g1_driver_->get_joint_torques(DriverUnitreeLowState::LIMB::RIGHT_LEG);

    VectorXd q_torso         = impl_->unitree_g1_driver_->get_joint_positions(DriverUnitreeLowState::LIMB::TORSO);
    VectorXd q_torso_dot     = impl_->unitree_g1_driver_->get_joint_velocities(DriverUnitreeLowState::LIMB::TORSO);
    VectorXd q_torso_tau     = impl_->unitree_g1_driver_->get_joint_torques(DriverUnitreeLowState::LIMB::TORSO);

    if (q_left_arm.size() > 0)
        ros_msg_left_arm.position = vectorxd_to_std_vector_double(q_left_arm);
    if (q_left_arm_dot.size() > 0)
        ros_msg_left_arm.velocity = vectorxd_to_std_vector_double(q_left_arm_dot);
    if (q_left_arm_tau.size() > 0)
        ros_msg_left_arm.effort = vectorxd_to_std_vector_double(q_left_arm_tau);

    if (q_right_arm.size() > 0)
        ros_msg_right_arm.position = vectorxd_to_std_vector_double(q_right_arm);
    if (q_right_arm_dot.size() > 0)
        ros_msg_right_arm.velocity = vectorxd_to_std_vector_double(q_right_arm_dot);
    if (q_right_arm_tau.size() > 0)
        ros_msg_right_arm.effort = vectorxd_to_std_vector_double(q_right_arm_tau);

    if (q_left_leg.size() > 0)
        ros_msg_left_leg.position = vectorxd_to_std_vector_double(q_left_leg);
    if (q_left_leg_dot.size() > 0)
        ros_msg_left_leg.velocity = vectorxd_to_std_vector_double(q_left_leg_dot);
    if (q_left_leg_tau.size() > 0)
        ros_msg_left_leg.effort = vectorxd_to_std_vector_double(q_left_leg_tau);

    if (q_right_leg.size() > 0)
        ros_msg_right_leg.position = vectorxd_to_std_vector_double(q_right_leg);
    if (q_right_leg_dot.size() > 0)
        ros_msg_right_leg.velocity = vectorxd_to_std_vector_double(q_right_leg_dot);
    if (q_right_leg_tau.size() > 0)
        ros_msg_right_leg.effort = vectorxd_to_std_vector_double(q_right_leg_tau);

    if (q_torso.size() > 0)
        ros_msg_torso.position = vectorxd_to_std_vector_double(q_torso);
    if (q_torso_dot.size() > 0)
        ros_msg_torso.velocity = vectorxd_to_std_vector_double(q_torso_dot);
    if (q_torso_tau.size() > 0)
        ros_msg_torso.effort = vectorxd_to_std_vector_double(q_torso_tau);

    publisher_left_arm_joint_states_->publish(ros_msg_left_arm);
    publisher_right_arm_joint_states_->publish(ros_msg_right_arm);
    publisher_left_leg_joint_states_->publish(ros_msg_left_leg);
    publisher_right_leg_joint_states_->publish(ros_msg_right_leg);
    publisher_torso_joint_states_->publish(ros_msg_torso);
}

void RobotDriverUnitreeG1::_read_imu_state_and_publish()
{
    sensor_msgs::msg::Imu ros_msg_imu;
    ros_msg_imu.header.stamp = node_->get_clock()->now();

    geometry_msgs::msg::PoseStamped ros_msg_pose;
    ros_msg_pose.header.stamp = node_->get_clock()->now();
    const DQ orientation = get_orientation();
    if (is_unit(orientation))
    {
        VectorXd vec_orientation = orientation.vec4();
        ros_msg_imu.orientation.w = vec_orientation(0);
        ros_msg_imu.orientation.x = vec_orientation(1);
        ros_msg_imu.orientation.y = vec_orientation(2);
        ros_msg_imu.orientation.z = vec_orientation(3);
        publisher_IMU_orientation_->publish(sas::dq_to_geometry_msgs_pose_stamped(orientation));

        VectorXd vec_angular_velocity = get_angular_velocity().vec3();
        ros_msg_imu.angular_velocity.x = vec_angular_velocity(0);
        ros_msg_imu.angular_velocity.y = vec_angular_velocity(1);
        ros_msg_imu.angular_velocity.z = vec_angular_velocity(2);

        VectorXd vec_acceleration = get_linear_acceleration().vec3();
        ros_msg_imu.linear_acceleration.x = vec_acceleration(0);
        ros_msg_imu.linear_acceleration.y = vec_acceleration(1);
        ros_msg_imu.linear_acceleration.z = vec_acceleration(2);

        publisher_IMU_state_->publish(ros_msg_imu);
    }

}

void RobotDriverUnitreeG1::_set_target_velocities_from_subscriber()
{
    if (new_target_twist_available_)
    {
        //    0  1  2  3  4  5
        //   wx wy wz vx vy vz
        const double& vx = target_twist_.at(3);
        const double& vy = target_twist_.at(4);
        const double& wz = target_twist_.at(2);
        impl_->unitree_g1_driver_->set_target_high_level_velocities({vx,vy,wz});
        new_target_twist_available_ = false;
    }
}


void RobotDriverUnitreeG1::_callback_target_twist(const geometry_msgs::msg::TwistStamped &msg)
{
    target_twist_ = {{msg.twist.angular.x,
                      msg.twist.angular.y,
                      msg.twist.angular.z,
                      msg.twist.linear.x,
                      msg.twist.linear.y,
                      msg.twist.linear.z}
                    };
    new_target_twist_available_ = true;
}

void RobotDriverUnitreeG1::_callback_mode_switch(const std_msgs::msg::Int32MultiArray &msg)
{
    /*
     *  IDLE=0,
        STANDING, // 1
        WALKING,  // 2
    */
    auto target_mode = static_cast<LeggedRobotDriver::HIGH_LEVEL_MODE>(msg.data[0]);
    // Only request if mode is different
    if (target_mode != current_mode_)
    {
        switch (target_mode)
        {
        case LeggedRobotDriver::HIGH_LEVEL_MODE::STANDING:
            impl_->unitree_g1_driver_->set_mode(DriverUnitreeG1::HIGH_MODE::ARM_CONTROL);
            break;
        case LeggedRobotDriver::HIGH_LEVEL_MODE::WALKING:
        case LeggedRobotDriver::HIGH_LEVEL_MODE::IDLE:
        default:
            impl_->unitree_g1_driver_->set_mode(DriverUnitreeG1::HIGH_MODE::LOCOMOTION);
            break;
        }

        RCLCPP_INFO(node_->get_logger(), "Mode switched to: %s",
                    high_level_mode_to_string(target_mode).c_str());
        current_mode_ = target_mode;
    }
    else
    {
        RCLCPP_DEBUG(node_->get_logger(), "Ignoring mode switch to same mode: %s",
                     high_level_mode_to_string(target_mode).c_str());
    }
}

std::string RobotDriverUnitreeG1::high_level_mode_to_string(const HIGH_LEVEL_MODE &mode) const
{
    switch (mode)
    {
    case HIGH_LEVEL_MODE::IDLE:     return "IDLE";
    case HIGH_LEVEL_MODE::STANDING: return "STANDING";
    case HIGH_LEVEL_MODE::WALKING:  return "WALKING";
    }
    return "UNKNOWN";
}


}
