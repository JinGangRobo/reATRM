#include <fast_tf/rcl.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/timer.hpp>
#include <thread>

#include <librmcs/client/cboard.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
// #include <rmcs_msgs/serial_interface.hpp>
// #include <rmcs_utility/fps_counter.hpp>
// #include <serial/serial.h>
#include <std_msgs/msg/int32.hpp>
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/supercap.hpp"
namespace rmcs_core::hardware {

class Engineer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Engineer()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<EngineerCommand>(get_component_name() + "_command", *this)) {
        arm_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/arm/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                arm_calibrate_subscription_callback(std::move(msg));
            });
        arm_board_ = std::make_unique<ArmBoard>(
            *this, *command_component_,
            static_cast<int>(get_parameter("usb_pid_arm_board").as_int()));
        using namespace rmcs_description;
    }
    ~Engineer() override = default;

    void update() override { arm_board_->update(); }

    void command_update() { arm_board_->command_update(); }

private:
    void arm_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            get_logger(), "[arm calibration] New joint1 offset: %lld",
            static_cast<long long>(arm_board_->arm_joint1_motor_.calibrate_zero_point()));
        RCLCPP_INFO(
            get_logger(), "[arm calibration] New joint2 offset: %lld",
            static_cast<long long>(arm_board_->arm_joint2_motor_.calibrate_zero_point()));
        RCLCPP_INFO(
            get_logger(), "[arm calibration] New joint3 offset: %lld",
            static_cast<long long>(arm_board_->arm_joint3_motor_.calibrate_zero_point()));
        RCLCPP_INFO(
            get_logger(), "[arm calibration] New joint4 offset: %lld",
            static_cast<long long>(arm_board_->arm_joint4_motor_.calibrate_zero_point()));
        RCLCPP_INFO(
            get_logger(), "[arm calibration] New joint5 offset: %lld",
            static_cast<long long>(arm_board_->arm_joint5_motor_.calibrate_zero_point()));
        RCLCPP_INFO(
            get_logger(), "[arm calibration] New joint6 offset: %lld",
            static_cast<long long>(arm_board_->arm_joint6_motor_.calibrate_zero_point()));
    }
    class EngineerCommand : public rmcs_executor::Component {
    public:
        explicit EngineerCommand(Engineer& engineer)
            : engineer_(engineer) {}
        void update() override { engineer_.command_update(); }
        Engineer& engineer_;
    };
    std::shared_ptr<EngineerCommand> command_component_;

    class ArmBoard final : private librmcs::client::CBoard {
    public:
        friend class Engineer;
        explicit ArmBoard(Engineer& engineer, EngineerCommand& engineer_command, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , dr16_(engineer)
            , arm_joint1_motor_(
                  engineer, engineer_command, "/arm/joint_1/motor",
                  device::DmMotor::Config{device::DmMotor::Type::J4310}
                  .set_reduction_ratio(9.0)
                  .set_encoder_zero_point(static_cast<int>(engineer.get_parameter("arm_joint1_motor_zero_point").as_int())))
            , arm_joint2_motor_(
                  engineer, engineer_command, "/arm/joint_2/motor",
                  device::LkMotor::Config{device::LkMotor::Type::MG4005E_I10}
                  .set_encoder_zero_point(static_cast<int>(engineer.get_parameter("arm_joint2_motor_zero_point").as_int())))
            , arm_joint3_motor_(
                  engineer, engineer_command, "/arm/joint_3/motor",
                  device::LkMotor::Config{device::LkMotor::Type::MG4005E_I10}
                  .set_encoder_zero_point(static_cast<int>(engineer.get_parameter("arm_joint3_motor_zero_point").as_int())))
            , arm_joint4_motor_(
                  engineer, engineer_command, "/arm/joint_4/motor",
                  device::DmMotor::Config{device::DmMotor::Type::J4310}
                  .set_encoder_zero_point(static_cast<int>(engineer.get_parameter("arm_joint4_motor_zero_point").as_int())))
            , arm_joint5_motor_(
                  engineer, engineer_command, "/arm/joint_5/motor",
                  device::DmMotor::Config{device::DmMotor::Type::J4310}
                  .set_encoder_zero_point(static_cast<int>(engineer.get_parameter("arm_joint1_motor_zero_point").as_int())))
            , arm_joint6_motor_(
                  engineer, engineer_command, "/arm/joint_6/motor",
                  device::DmMotor::Config{device::DmMotor::Type::J4310}
                  .set_encoder_zero_point(static_cast<int>(engineer.get_parameter("arm_joint1_motor_zero_point").as_int())))
            , supercap_(engineer, 28.5)
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {}

        ~ArmBoard() final {
            stop_handling_events();
            event_thread_.join();
        }
        void update() {
            arm_joint1_motor_.update_status();
            arm_joint2_motor_.update_status();
            arm_joint3_motor_.update_status();
            arm_joint4_motor_.update_status();
            arm_joint5_motor_.update_status();
            arm_joint6_motor_.update_status();
            dr16_.update_status();
        }
        void command_update() {
            static bool even_phase{true};
            if (even_phase) {
                transmit_buffer_.add_can1_transmission(
                    0x02, arm_joint1_motor_.generate_torque_command());
                transmit_buffer_.add_can2_transmission(
                    0x141, arm_joint2_motor_.generate_torque_command());
                transmit_buffer_.add_can2_transmission(
                    0x142, arm_joint3_motor_.generate_torque_command());
            } else {
                transmit_buffer_.add_can1_transmission(
                    0x03, arm_joint4_motor_.generate_torque_command());
                transmit_buffer_.add_can1_transmission(
                    0x04, arm_joint5_motor_.generate_torque_command());
                transmit_buffer_.add_can1_transmission(
                    0x05, arm_joint6_motor_.generate_torque_command());
            }
            transmit_buffer_.trigger_transmission();
            even_phase = !even_phase;
        }
        

    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x53) {
                arm_joint4_motor_.store_status(can_data);
            } else if (can_id == 0x54) {
                arm_joint5_motor_.store_status(can_data);
            } else if (can_id == 0x55) {
                arm_joint6_motor_.store_status(can_data);
            } else if (can_id == 0x212) {
                arm_joint1_motor_.store_status(can_data);
            }
        }
        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x141) {
                arm_joint2_motor_.store_status(can_data);
            } else if (can_id == 0x142) {
                arm_joint3_motor_.store_status(can_data);
            } 
        }
        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }
        device::Dr16 dr16_;
        device::DmMotor arm_joint1_motor_;
        device::LkMotor arm_joint2_motor_;
        device::LkMotor arm_joint3_motor_;
        device::DmMotor arm_joint4_motor_;
        device::DmMotor arm_joint5_motor_;
        device::DmMotor arm_joint6_motor_;
        device::Supercap supercap_;
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr arm_calibrate_subscription_;
    std::unique_ptr<ArmBoard> arm_board_;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Engineer, rmcs_executor::Component)
