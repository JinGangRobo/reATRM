#include <memory>
#include <thread>

#include <librmcs/client/cboard.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/fps_counter.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"

namespace rmcs_core::hardware {

class TestingInfantry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    TestingInfantry()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<TestingInfantryCommand>(
                  get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        register_output("/tf", tf_);
        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.16, 0.0, 0.15});

        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_,
            static_cast<int>(get_parameter("usb_pid_bottom_board").as_int()));
    }

    ~TestingInfantry() override = default;

    void update() override { bottom_board_->update(); }

    void command_update() { bottom_board_->command_update(); }

private:
    class TestingInfantryCommand : public rmcs_executor::Component {
    public:
        explicit TestingInfantryCommand(TestingInfantry& testing_infantry)
            : testing_infantry_(testing_infantry) {}

        void update() override { testing_infantry_.command_update(); }

        TestingInfantry& testing_infantry_;
    };
    std::shared_ptr<TestingInfantryCommand> command_component_;

    class BottomBoard final : private librmcs::client::CBoard {
    public:
        friend class TestingInfantry;
        explicit BottomBoard(
            TestingInfantry& testing_infantry, TestingInfantryCommand& testing_infantry_command,
            int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , imu_(1000, 0.2, 0.0)
            , tf_(testing_infantry.tf_)
            , dr16_(testing_infantry)
            , chassis_wheel_motors_(
                  {testing_infantry, testing_infantry_command, "/chassis/left_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                  {testing_infantry, testing_infantry_command, "/chassis/left_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                  {testing_infantry, testing_infantry_command, "/chassis/right_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                  {testing_infantry, testing_infantry_command, "/chassis/right_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}})
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {

            testing_infantry.register_output(
                "/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
        }

        ~BottomBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            imu_.update_status();
            dr16_.update_status();

            *chassis_yaw_velocity_imu_ = imu_.gz();

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
        }

        void command_update() {
            uint16_t batch_commands[4];

            for (int i = 0; i < 4; i++)
                batch_commands[i] = chassis_wheel_motors_[i].generate_command();
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(batch_commands));

            transmit_buffer_.trigger_transmission();
        }

    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x201) {
                chassis_wheel_motors_[0].store_status(can_data);
            } else if (can_id == 0x202) {
                chassis_wheel_motors_[1].store_status(can_data);
            } else if (can_id == 0x203) {
                chassis_wheel_motors_[2].store_status(can_data);
            } else if (can_id == 0x204) {
                chassis_wheel_motors_[3].store_status(can_data);
            }
        }

        void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            referee_ring_buffer_receive_.emplace_back_multi(
                [&uart_data](std::byte* storage) { *storage = *uart_data++; }, uart_data_length);
        }

        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }

        void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_accelerometer_status(x, y, z);
        }

        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_gyroscope_status(x, y, z);
        }

        device::Bmi088 imu_;
        OutputInterface<rmcs_description::Tf>& tf_;

        OutputInterface<double> chassis_yaw_velocity_imu_;

        device::Dr16 dr16_;

        device::DjiMotor chassis_wheel_motors_[4];

        librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };

    OutputInterface<rmcs_description::Tf> tf_;

    std::unique_ptr<BottomBoard> bottom_board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::TestingInfantry, rmcs_executor::Component)