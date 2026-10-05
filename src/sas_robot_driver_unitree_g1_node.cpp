#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/sas_shutdown_signaler.hpp>
#include <sas_tools/sas_legged_robot_driver_ros.hpp>
#include <sas_robot_driver_unitree_g1/sas_robot_driver_unitree_g1.hpp>

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::shared_ptr<sas::ShutdownSignaler> shutdown_signaler = std::make_shared<sas::ShutdownSignaler>();
void sig_int_handler(int)
{
    shutdown_signaler->shutdown();
}

/**
 * Parameters (set by the launch file):
 *   - domain_id (int, mandatory):            DDS domain: 0 for the real robot, 1 for unitree_mujoco.
 *   - network_interface (string, mandatory): e.g. "eth0" on the onboard computer, "lo" for unitree_mujoco.
 *   - thread_sampling_time_sec (double, mandatory): period of the control loop, e.g. 0.002.
 *   - twist_timeout_sec (double, optional, default 0.2): a zero twist is sent when no twist arrives in time.
 *
 * The topic prefix is the name of the node (set by the launch file), inside its namespace, e.g. the node
 * "g1_1" in the namespace "sas_g1" serves sas_g1/g1_1/... as in the other SAS robot drivers.
 */
int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");

    // Keep our SIGINT handler: it signals the shutdown so that the control loop ends and the driver
    // deinitializes the robot before ROS is shut down.
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_unitree_g1");

    try
    {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");

        int domain_id;
        sas::RobotDriverUnitreeG1Configuration robot_configuration;
        sas::get_ros_parameter(node, "domain_id", domain_id);
        sas::get_ros_parameter(node, "network_interface", robot_configuration.network_interface);
        robot_configuration.domain_id = static_cast<int32_t>(domain_id);

        sas::LeggedRobotDriverROSConfiguration configuration{};
        configuration.robot_driver_ros.robot_driver_provider_prefix = node->get_name();
        sas::get_ros_parameter(node, "thread_sampling_time_sec", configuration.robot_driver_ros.thread_sampling_time_sec);
        sas::get_ros_optional_parameter(node, "twist_timeout_sec", configuration.twist_timeout_sec, 0.2);

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Parameters OK: domain_id " << domain_id
                                                    << ", network_interface " << robot_configuration.network_interface
                                                    << ", prefix " << node->get_fully_qualified_name()
                                                    << ", thread_sampling_time_sec " << configuration.robot_driver_ros.thread_sampling_time_sec
                                                    << ", twist_timeout_sec " << configuration.twist_timeout_sec);

        auto robot_driver = std::make_shared<sas::RobotDriverUnitreeG1>(robot_configuration, shutdown_signaler);

        sas::LeggedRobotDriverROS legged_robot_driver_ros(node, robot_driver, configuration, shutdown_signaler);
        legged_robot_driver_ros.control_loop();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
        std::cerr << std::string("::Exception::") << e.what();
    }

    rclcpp::shutdown();
    return 0;
}
