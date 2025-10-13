#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <dirent.h>

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  // TF broadcaster

  // Create Publisher
  gimbal_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("gimbal_state", 3);
  supercap_pub_ = this->create_publisher<atrm_interfaces::msg::SupercapInfo>("supercap", 3);
  enable_auto_aim_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::EnableAutoAim>("switch/auto_aim", 3);

  RCLCPP_INFO(get_logger(), "READY TO CONNECT");
  connectSerial();
  RCLCPP_INFO(get_logger(), "CONNECT OK");
  receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
  RCLCPP_INFO(get_logger(), "THREAD START");

  // Create Subscription
  gimbal_sub_ = this->create_subscription<atrm_interfaces::msg::CmdGimbal>(
    "cmd_gimbal", rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(&RMSerialDriver::gimbalCmdCallback, this, std::placeholders::_1));

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(&RMSerialDriver::velCmdCallback, this, std::placeholders::_1));

  // Init send packet buffer
  send_packet_buffer.frame_header.sof = PACKET_VERSION;
  send_packet_buffer.frame_header.id = 0x3A;
  send_packet_buffer.frame_header.len = sizeof(SendRoboControl);
  send_packet_buffer.pitch = -1;
  send_packet_buffer.yaw = -1;
  send_packet_buffer.fire = false;
  send_packet_buffer.vx = 0;
  send_packet_buffer.vy = 0;
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::connectSerial()
{
  rclcpp::WallRate reconnect_rate(1);
  while (rclcpp::ok()) {
    try {
      serial_driver_->init_port(findFirstTtyACMDevice(), *device_config_);
      RCLCPP_INFO(get_logger(), "PORT INIT DONE");
      if (!serial_driver_->port()->is_open()) {
        serial_driver_->port()->open();
      }
      RCLCPP_INFO(get_logger(), "PORT CHECK DONE");
      if (serial_driver_->port()->is_open()) {
        return;
      }
      RCLCPP_INFO(get_logger(), "PORT REOPEN DONE");

    } catch (const std::exception & ex) {
      RCLCPP_INFO(get_logger(), "Error creating serial port(%s). Retry", ex.what());
    }

    reconnect_rate.sleep();
  }
}

std::string RMSerialDriver::findFirstTtyACMDevice()
{
  const std::string basePath = "/dev/";
  const std::string prefix = "ttyACM";
  DIR * dir;
  struct dirent * ent;

  if ((dir = opendir(basePath.c_str())) != nullptr) {
    while ((ent = readdir(dir)) != nullptr) {
      std::string filename(ent->d_name);
      if (filename.find(prefix) == 0) {  // 检查是否以 ttyACM 开头
        // 返回完整路径
        return basePath + filename;
      }
    }
    closedir(dir);
  } else {
    std::cerr << "Could not open directory " << basePath << std::endl;
  }

  return "";  // 如果没有找到设备，返回空字符串
}

void RMSerialDriver::receiveData()
{
  RCLCPP_INFO(get_logger(), "Start receiveData!");

  std::vector<uint8_t> sof(1);
  std::vector<uint8_t> receive_data;

  int sof_count = 0;

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(sof);

      if (sof[0] != PACKET_VERSION) {
        sof_count++;
        RCLCPP_INFO(get_logger(), "Version ERR, cnt=%d", sof_count);
        continue;
      }

      // Reset sof_count when SOF_RECEIVE is found
      sof_count = 0;

      // sof[0] == SOF_RECEIVE 后读取剩余 header_frame 内容
      std::vector<uint8_t> header_frame_buf(3);  // sof 在读取完数据后添加

      serial_driver_->port()->receive(header_frame_buf);          // 读取除 sof 外剩下的数据
      header_frame_buf.insert(header_frame_buf.begin(), sof[0]);  // 添加 sof
      HeaderFrame header_frame = fromVector<HeaderFrame>(header_frame_buf);

      // HeaderFrame CRC8 check
      bool crc8_ok = crc8::verify_CRC8_check_sum(
        reinterpret_cast<uint8_t *>(&header_frame), sizeof(header_frame));
      if (!crc8_ok) {
        RCLCPP_ERROR(get_logger(), "Header frame CRC8 error!");
        continue;
      }

      // crc8_ok 校验正确后读取数据段
      // 根据数据段长度读取数据
      std::vector<uint8_t> data_buf(header_frame.len + 2);  // len + crc
      int received_len = serial_driver_->port()->receive(data_buf);
      int received_len_sum = received_len;
      // 考虑到一次性读取数据可能存在数据量过大，读取不完整的情况。需要检测是否读取完整
      // 计算剩余未读取的数据长度
      int remain_len = header_frame.len + 2 - received_len;
      while (remain_len > 0) {  // 读取剩余未读取的数据
        std::vector<uint8_t> remain_buf(remain_len);
        received_len = serial_driver_->port()->receive(remain_buf);
        data_buf.insert(data_buf.begin() + received_len_sum, remain_buf.begin(), remain_buf.end());
        received_len_sum += received_len;
        remain_len -= received_len;
      }

      // 数据段读取完成后添加 header_frame_buf 到 data_buf，得到完整数据包
      data_buf.insert(data_buf.begin(), header_frame_buf.begin(), header_frame_buf.end());

      // 整包数据校验
      bool crc16_ok = crc16::verify_CRC16_check_sum(data_buf);
      if (!crc16_ok) {
        RCLCPP_ERROR(get_logger(), "Data segment CRC16 error!");
        continue;
      }
      auto robo_info = fromVector<ReceiveRoboInfo>(data_buf);
      publishData(robo_info);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
      connectSerial();
    }
  }
}

void RMSerialDriver::publishData(ReceiveRoboInfo & data)
{
  // Publish gimbal state
  gimbal_state_.name = {
    "gimbal_main_yaw_odom_joint", "gimbal_sub_yaw_odom_joint", "gimbal_yaw_joint",
    "gimbal_pitch_joint"};
  gimbal_state_.position = {
    -data.encoder_up + data.yaw, data.encoder_up - data.yaw, data.yaw,
    -data.pitch + (M_PI / 4)};
  gimbal_state_.velocity = {0.0, 0.0, 0.0, 0.0};
  gimbal_state_.effort = {0.0, 0.0, 0.0, 0.0};
  gimbal_pub_->publish(gimbal_state_);

  // RCLCPP_INFO(get_logger(), "Gimbal RAW State: pitch=%.2f, yaw=%.2f", data.pitch, data.yaw);

  // Publish is enable auto aim
  enable_auto_aim_.enable_auto_aim = data.is_auto_aim;
  enable_auto_aim_pub_->publish(enable_auto_aim_);
}

void RMSerialDriver::gimbalCmdCallback(const atrm_interfaces::msg::CmdGimbal::SharedPtr msg)
{
  // packet.frame_header.sof = 0x23;
  // packet.frame_header.id = 0xA4;
  // packet.frame_header.len = (u_int8_t)(sizeof(SendPacketTwist) - 6);
  send_packet_buffer.frame_header.sof = PACKET_VERSION;
  send_packet_buffer.frame_header.id = 0x3A;
  send_packet_buffer.frame_header.len = (uint8_t)(sizeof(SendRoboControl) - 6);

  send_packet_buffer.pitch = msg->pitch;
  send_packet_buffer.yaw = msg->yaw;
  send_packet_buffer.fire = msg->fire;
  sendData();
}
void RMSerialDriver::velCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  send_packet_buffer.frame_header.sof = PACKET_VERSION;
  send_packet_buffer.frame_header.id = 0x3A;
  send_packet_buffer.frame_header.len = (uint8_t)(sizeof(SendRoboControl) - 6);

  send_packet_buffer.vx = msg->linear.x;
  send_packet_buffer.vy = msg->linear.y;
  sendData();
}

void RMSerialDriver::sendData()
{
  try {
    crc8::append_CRC8_check_sum(
      reinterpret_cast<uint8_t *>(&send_packet_buffer), sizeof(HeaderFrame));
    crc16::Append_CRC16_Check_Sum(
      reinterpret_cast<uint8_t *>(&send_packet_buffer), sizeof(SendRoboControl));
    std::vector<uint8_t> data = toVector(send_packet_buffer);
    serial_driver_->port()->send(data);

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
