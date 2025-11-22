#include <fast_tf/rcl.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <thread>

#include <librmcs/client/cboard.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/fps_counter.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/int32.hpp>

#include "filter/low_pass_filter.hpp"
#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/gy614.hpp"
#include "hardware/device/supercap.hpp"

namespace rmcs_core::hardware {

class DualSentry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DualSentry()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<DualSentryCommand>(
                  get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        register_output("/tf", tf_);
        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.16, 0.0, 0.15});

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });

        top_board_ = std::make_unique<TopBoard>(
            *this, *command_component_,
            static_cast<int>(get_parameter("usb_pid_top_board").as_int()));
        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_,
            static_cast<int>(get_parameter("usb_pid_bottom_board").as_int()));
    }

    ~DualSentry() override = default;

    void update() override {
        top_board_->update();
        bottom_board_->update();
    }

    void command_update() {
        top_board_->command_update();
        bottom_board_->command_update();
    }

private:
    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New yaw offset: %d",
            bottom_board_->gimbal_bottom_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New yaw offset: %d",
            top_board_->gimbal_top_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New pitch offset: %d",
            top_board_->gimbal_pitch_motor_.calibrate_zero_point());
    }

    class DualSentryCommand : public rmcs_executor::Component {
    public:
        explicit DualSentryCommand(DualSentry& dual_sentry)
            : dual_sentry_(dual_sentry) {}

        void update() override { dual_sentry_.command_update(); }

        DualSentry& dual_sentry_;
    };
    std::shared_ptr<DualSentryCommand> command_component_;

    class TopBoard final : private librmcs::client::CBoard {
    public:
        friend class DualSentry;
        explicit TopBoard(
            DualSentry& dual_sentry, DualSentryCommand& dual_sentry_command, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , tf_(dual_sentry.tf_)
            , imu_(10.0f, 0.001f, 1000000.0f)
            , gy614_(dual_sentry, "/friction_wheels/temperature")
            , dr16_{dual_sentry}
            , imu_bias_x(dual_sentry.get_parameter("imu_bias_x").as_int())
            , imu_bias_y(dual_sentry.get_parameter("imu_bias_y").as_int())
            , imu_bias_z(dual_sentry.get_parameter("imu_bias_z").as_int())
            , gimbal_top_yaw_motor_(dual_sentry, dual_sentry_command, "/gimbal/top_yaw")
            , gimbal_pitch_motor_(dual_sentry, dual_sentry_command, "/gimbal/pitch")
            , gimbal_left_friction_(
                  dual_sentry, dual_sentry_command, "/gimbal/left_friction")
            , gimbal_right_friction_(
                  dual_sentry, dual_sentry_command, "/gimbal/right_friction")
            , gimbal_bullet_feeder_(
                  dual_sentry, dual_sentry_command, "/gimbal/bullet_feeder")
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {

            gimbal_top_yaw_motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::GM6020}.set_encoder_zero_point(
                    static_cast<int>(
                        dual_sentry.get_parameter("top_yaw_motor_zero_point").as_int())));

            gimbal_pitch_motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            dual_sentry.get_parameter("pitch_motor_zero_point").as_int()))
                    .set_reversed());

            gimbal_left_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(1.));
            gimbal_right_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                    .set_reduction_ratio(1.)
                    .set_reversed());

            gimbal_bullet_feeder_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                    .enable_multi_turn_angle()
                    .set_reversed()
                    .set_reduction_ratio(19 * 2));

            imu_.set_coordinate_mapping([](double x, double y, double z) {
                // Get the mapping with the following code.
                // The rotation angle must be an exact multiple of 90 degrees, otherwise use a
                // matrix.

                // Eigen::AngleAxisd pitch_link_to_imu_link{
                //     std::numbers::pi, Eigen::Vector3d::UnitZ()};
                // Eigen::Vector3d mapping = pitch_link_to_imu_link * Eigen::Vector3d{1, 2, 3};
                // std::cout << mapping << std::endl;

                return std::make_tuple(x, y, z);
            });

            dual_sentry.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
            dual_sentry.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);
        }

        ~TopBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            imu_.update_status();
            Eigen::Quaterniond gimbal_imu_pose{imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3()};

            tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                gimbal_imu_pose.conjugate());

            gy614_.update_status();
            dr16_.update_status();

            *gimbal_yaw_velocity_imu_ = imu_gz_velocity_filter_.update(imu_.gz());
            *gimbal_pitch_velocity_imu_ = imu_gy_velocity_filter_.update(imu_.gy());

            gimbal_top_yaw_motor_.update_status();
            gimbal_pitch_motor_.update_status();

            gimbal_left_friction_.update_status();
            gimbal_right_friction_.update_status();
            gimbal_bullet_feeder_.update_status();

            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                gimbal_pitch_motor_.angle());

            fast_tf::rcl::broadcast_all(*tf_);
        }

        void command_update() {
            uint16_t can_commands[4];

            can_commands[0] = 0;
            can_commands[1] = gimbal_top_yaw_motor_.generate_command();
            can_commands[2] = gimbal_pitch_motor_.generate_command();
            can_commands[3] = gimbal_bullet_feeder_.generate_command();
            transmit_buffer_.add_can1_transmission(0x1FF, std::bit_cast<uint64_t>(can_commands));

            can_commands[0] = gimbal_right_friction_.generate_command();
            can_commands[1] = gimbal_left_friction_.generate_command();
            can_commands[2] = 0;
            can_commands[3] = 0;
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

            transmit_buffer_.trigger_transmission();
        }

    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x206) {
                gimbal_top_yaw_motor_.store_status(can_data);
            } else if (can_id == 0x207) {
                gimbal_pitch_motor_.store_status(can_data);
            } else if (can_id == 0x202) {
                gimbal_left_friction_.store_status(can_data);
            } else if (can_id == 0x201) {
                gimbal_right_friction_.store_status(can_data);
            } else if (can_id == 0x208) {
                gimbal_bullet_feeder_.store_status(can_data);
            }
        }

        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }

        void uart2_receive_callback(const std::byte* data, uint8_t length) override {
            gy614_.store_status(data, length);
        }

        void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_accelerometer_status(x, y, z);
        }

        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_gyroscope_status(x - imu_bias_x, y - imu_bias_y, z - imu_bias_z);
        }

        OutputInterface<rmcs_description::Tf>& tf_;

        device::Bmi088 imu_;
        device::Gy614 gy614_;
        device::Dr16 dr16_;

        OutputInterface<double> gimbal_yaw_velocity_imu_;
        OutputInterface<double> gimbal_pitch_velocity_imu_;

        OutputInterface<double> debug_imu_g_x_;
        OutputInterface<double> debug_imu_g_y_;
        OutputInterface<double> debug_imu_g_z_;

        int16_t imu_bias_x, imu_bias_y, imu_bias_z = 0.0;

        rmcs_core::filter::LowPassFilter<> imu_gy_velocity_filter_{40.0f, 1000.0f};
        rmcs_core::filter::LowPassFilter<> imu_gz_velocity_filter_{60.0f, 1000.0f};

        device::DjiMotor gimbal_top_yaw_motor_;
        device::DjiMotor gimbal_pitch_motor_;

        device::DjiMotor gimbal_left_friction_;
        device::DjiMotor gimbal_right_friction_;

        device::DjiMotor gimbal_bullet_feeder_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };

    class BottomBoard final : private librmcs::client::CBoard {
    public:
        friend class DualSentry;
        explicit BottomBoard(
            DualSentry& dual_sentry, DualSentryCommand& dual_sentry_command, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , imu_(10.0f, 0.001f, 1000000.0f)
            , tf_(dual_sentry.tf_)
            , gimbal_bottom_yaw_motor_(dual_sentry, dual_sentry_command, "/gimbal/bottom_yaw")
            , chassis_wheel_motors_(
                  {dual_sentry, dual_sentry_command, "/chassis/left_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                  {dual_sentry, dual_sentry_command, "/chassis/right_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                  {dual_sentry, dual_sentry_command, "/chassis/right_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                  {dual_sentry, dual_sentry_command, "/chassis/left_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}})
            , supercap_(dual_sentry, dual_sentry_command)
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {

            gimbal_bottom_yaw_motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::GM6020}.set_encoder_zero_point(
                    static_cast<int>(
                        dual_sentry.get_parameter("bottom_yaw_motor_zero_point").as_int())));

            dual_sentry.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_multi(
                    [&buffer](std::byte byte) { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                transmit_buffer_.add_uart1_transmission(buffer, size);
                return size;
            };

            dual_sentry.register_output("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
        }

        ~BottomBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            imu_.update_status();

            *chassis_yaw_velocity_imu_ = imu_.gz();

            gimbal_bottom_yaw_motor_.update_status();
            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_bottom_yaw_motor_.angle());
            fast_tf::rcl::broadcast_all(*tf_);

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();

            supercap_.update_status();
        }

        void command_update() {
            uint16_t can_commands[4];

            for (int i = 0; i < 4; i++)
                can_commands[i] = chassis_wheel_motors_[i].generate_command();
            transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

            can_commands[0] = gimbal_bottom_yaw_motor_.generate_command();
            can_commands[1] = 0;
            can_commands[2] = 0;
            can_commands[3] = 0;
            transmit_buffer_.add_can1_transmission(0x1FF, std::bit_cast<uint64_t>(can_commands));

            transmit_buffer_.trigger_transmission();
        }

    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x205) {
                gimbal_bottom_yaw_motor_.store_status(can_data);
            }
        }

        void can2_receive_callback(
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

        void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_accelerometer_status(x, y, z);
        }

        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_gyroscope_status(x, y, z);
        }

        device::Bmi088 imu_;
        OutputInterface<rmcs_description::Tf>& tf_;

        OutputInterface<double> chassis_yaw_velocity_imu_;

        device::DjiMotor gimbal_bottom_yaw_motor_;
        device::DjiMotor chassis_wheel_motors_[4];
        device::Supercap supercap_;

        librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };

    OutputInterface<rmcs_description::Tf> tf_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    std::unique_ptr<TopBoard> top_board_;
    std::unique_ptr<BottomBoard> bottom_board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DualSentry, rmcs_executor::Component)