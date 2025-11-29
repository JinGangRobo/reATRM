#include <fast_tf/rcl.hpp>
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
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"

namespace rmcs_core::hardware {

class WheelLegInfantry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLegInfantry()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<WheelLegInfantryCommand>(get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        register_output("/tf", tf_);
        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.16, 0.0, 0.15});

        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_,
            static_cast<int>(get_parameter("usb_pid_bottom_board").as_int()));
    }

    ~WheelLegInfantry() override = default;

    void update() override {
        bottom_board_->update();
    }

    void command_update() {
        bottom_board_->command_update();
    }

private:
    class WheelLegInfantryCommand : public rmcs_executor::Component {
    public:
        explicit WheelLegInfantryCommand(WheelLegInfantry& wheeleg_infantry)
            : wheeleg_infantry_(wheeleg_infantry) {}

        void update() override { wheeleg_infantry_.command_update(); }

        WheelLegInfantry& wheeleg_infantry_;
    };
    std::shared_ptr<WheelLegInfantryCommand> command_component_;

    class BottomBoard final : private librmcs::client::CBoard {
    public:
        friend class WheelLegInfantry;
        explicit BottomBoard(WheelLegInfantry& wheeleg_infantry, WheelLegInfantryCommand& wheeleg_infantry_command, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , imu_(10.0f, 0.001f, 1000000.0f)
            , tf_(wheeleg_infantry.tf_)

            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {

            imu_.set_coordinate_mapping([](double x, double y, double z) {
                // Get the mapping with the following code.
                // The rotation angle must be an exact multiple of 90 degrees, otherwise use a
                // matrix.

                // Eigen::AngleAxisd pitch_link_to_imu_link{
                //     std::numbers::pi, Eigen::Vector3d::UnitZ()};
                // Eigen::Vector3d mapping = pitch_link_to_imu_link * Eigen::Vector3d{1, 2, 3};
                // std::cout << mapping << std::endl;

                return std::make_tuple(x, z, y);
            });


        }

        ~BottomBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            imu_.update_status();
        }

        void command_update() {
            uint16_t batch_commands[4];


            transmit_buffer_.trigger_transmission();
        }

    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

        }

        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

        }

        void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_accelerometer_status(x, y, z);
        }

        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_gyroscope_status(x, y, z);
        }

        device::Bmi088 imu_;
        OutputInterface<rmcs_description::Tf>& tf_;

        librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };

    OutputInterface<rmcs_description::Tf> tf_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    std::unique_ptr<BottomBoard> bottom_board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::WheelLegInfantry, rmcs_executor::Component)