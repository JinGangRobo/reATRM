/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       rm_serial_driver.hpp/cpp
  * @brief      串口通信模块
  * @note       感谢@ChenJun创建本模块并开源，
  *             现内容为北极熊基于开源模块进行修改并适配自己的车车后的结果。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2022            ChenJun         1. done
  *  V1.0.1     2023-12-11      Penguin         1. 添加与rm_rune_dector_node模块连接的Client
  *  V1.0.2     2024-3-1        LihanChen       1. 添加导航数据包，并重命名packet和相关函数
  *  V1.0.3     2024-3-4        LihanChen       1. 添加裁判系统数据包
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  */

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <atrm_interfaces/msg/cmd_gimbal.hpp>
#include <atrm_interfaces/msg/supercap_info.hpp>
#include <auto_aim_interfaces/msg/enable_auto_aim.hpp>
#include <auto_aim_interfaces/msg/target.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rm_decision_interfaces/msg/all_robot_hp.hpp>
#include <rm_serial_driver/packet.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "std_msgs/msg/bool.hpp"

// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;

private:
  void getParams();

  void connectSerial();

  void receiveData();

  void publishData(ReceiveRoboInfo & data);

  void gimbalCmdCallback(const atrm_interfaces::msg::CmdGimbal::SharedPtr msg);

  void velCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void sendData();

  void reopenPort();

  std::string findFirstTtyACMDevice();

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::string urdf_type_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  // Send buffer
  SendRoboControl send_packet_buffer;

  rclcpp::Subscription<atrm_interfaces::msg::CmdGimbal>::SharedPtr gimbal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gimbal_pub_;
  rclcpp::Publisher<atrm_interfaces::msg::SupercapInfo>::SharedPtr supercap_pub_;

  rclcpp::Publisher<auto_aim_interfaces::msg::EnableAutoAim>::SharedPtr enable_auto_aim_pub_;

  sensor_msgs::msg::JointState gimbal_state_;
  atrm_interfaces::msg::SupercapInfo supercap_state_;
  auto_aim_interfaces::msg::EnableAutoAim enable_auto_aim_;

  std::thread receive_thread_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
