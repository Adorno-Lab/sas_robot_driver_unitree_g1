#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>

#include <dqrobotics/utils/DQ_Math.h>
#include <sas_robot_driver/sas_robot_driver_ros.hpp>
#include <sas_robot_driver_unitree_g1/sas_robot_driver_unitree_g1.hpp>
#include <marinholab/sas/core/sas_robot_driver.hpp>
#include <marinholab/sas/core/sas_shutdown_signaler.hpp>
#include <marinholab/sas/core/eigen3_std_conversions.hpp>

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::shared_ptr<sas::ShutdownSignaler> shutdown_signaler = std::make_shared<sas::ShutdownSignaler>();
void sig_int_handler(int)
{
    shutdown_signaler->shutdown();
}

int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_unitree_g1");

    try
    {
        sas::RobotDriverUnitreeG1Configuration robot_configuration;
        robot_configuration.domain_id = 0; //   real robot: 0   simulation: 1
        robot_configuration.network_interface = "eth0";// Desktop with Ethernet cable: "enp6s0"; onboard PC: "eth0"  ("lo" doesn't work)
        robot_configuration.topic_prefix = "sas_g1/g1_1";

        auto robot_driver = std::make_shared<sas::RobotDriverUnitreeG1>(node,
                                                                        robot_configuration,
                                                                        shutdown_signaler);

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");

        sas::RobotDriverROSConfiguration robot_driver_ros_configuration;
        robot_driver_ros_configuration.thread_sampling_time_sec = 0.002;
        robot_driver_ros_configuration.robot_driver_provider_prefix = robot_configuration.topic_prefix;//node->get_name();

        sas::RobotDriverROS robot_driver_ros(node,
                                             robot_driver,
                                             robot_driver_ros_configuration,
                                             shutdown_signaler);
        robot_driver_ros.control_loop();

    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
        std::cerr << std::string("::Exception::") << e.what();
    }

    return 0;
}
