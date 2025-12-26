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
#include "hardware/device/dr16.hpp"
#include "hardware/device/supercap.hpp"

namespace rmcs_core::hardware {

class MiniInfantry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MiniInfantry()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<MiniInfantryCommand>(
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

    ~MiniInfantry() override = default;

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
            bottom_board_->gimbal_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New pitch offset: %d",
            top_board_->gimbal_pitch_motor_.calibrate_zero_point());
    }

    class MiniInfantryCommand : public rmcs_executor::Component {
    public:
        explicit MiniInfantryCommand(MiniInfantry& mini_infantry)
            : mini_infantry_(mini_infantry) {}

        void update() override { mini_infantry_.command_update(); }

        MiniInfantry& mini_infantry_;
    };
    std::shared_ptr<MiniInfantryCommand> command_component_;

    class TopBoard final : private librmcs::client::CBoard {
    public:
        friend class MiniInfantry;
        explicit TopBoard(
            MiniInfantry& mini_infantry, MiniInfantryCommand& mini_infantry_command,
            int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , tf_(mini_infantry.tf_)
            , imu_(10.0f, 0.001f, 1000000.0f)
            , dr16_(mini_infantry)
            , imu_bias_x(mini_infantry.get_parameter("imu_bias_x").as_int())
            , imu_bias_y(mini_infantry.get_parameter("imu_bias_y").as_int())
            , imu_bias_z(mini_infantry.get_parameter("imu_bias_z").as_int())
            , gimbal_pitch_motor_(
                  mini_infantry, mini_infantry_command, "/gimbal/pitch",
                  device::DjiMotor::Config{device::DjiMotor::Type::GM6020}.set_encoder_zero_point(
                      static_cast<int>(
                          mini_infantry.get_parameter("pitch_motor_zero_point").as_int())))
            , gimbal_left_friction_(mini_infantry, mini_infantry_command, "/gimbal/left_friction")
            , gimbal_right_friction_(mini_infantry, mini_infantry_command, "/gimbal/right_friction")
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

                return std::make_tuple(x, y, z);
            });

            mini_infantry.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
            mini_infantry.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);

            mini_infantry.register_output("/debug/pitch/raw_angle", debug_pitch_raw_angle_);
            mini_infantry.register_output("/debug/pitch/temp", debug_pitch_temp);
            mini_infantry.register_output("/debug/imu/gx_bais", debug_imu_gx_bais_);
            mini_infantry.register_output("/debug/imu/gy_bais", debug_imu_gy_bais_);
            mini_infantry.register_output("/debug/imu/gz_bais", debug_imu_gz_bais_);
        }

        ~TopBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            imu_.update_status();
            Eigen::Quaterniond gimbal_imu_pose{imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3()};

            *debug_imu_gx_bais_ = imu_.cali_gx_ref();
            *debug_imu_gy_bais_ = imu_.cali_gy_ref();
            *debug_imu_gz_bais_ = imu_.cali_gz_ref();

            tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                gimbal_imu_pose.conjugate());
            tf_->set_transform<rmcs_description::BaseLink, rmcs_description::RawImu>(
                gimbal_imu_pose);
            fast_tf::rcl::broadcast_all(*tf_);

            dr16_.update_status();

            *gimbal_yaw_velocity_imu_ = imu_gz_velocity_filter_.update(imu_.gz());
            *gimbal_pitch_velocity_imu_ = imu_gy_velocity_filter_.update(imu_.gy());

            *debug_pitch_raw_angle_ = gimbal_pitch_motor_.last_raw_angle();

            gimbal_pitch_motor_.update_status();
            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                gimbal_pitch_motor_.angle());

            fast_tf::rcl::broadcast_all(*tf_);

            gimbal_left_friction_.update_status();
            gimbal_right_friction_.update_status();
        }

        void command_update() {
            uint16_t batch_commands[4];

            batch_commands[0] = gimbal_left_friction_.generate_command();
            batch_commands[1] = gimbal_right_friction_.generate_command();
            batch_commands[2] = 0;
            batch_commands[3] = 0;
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(batch_commands));

            batch_commands[0] = gimbal_pitch_motor_.generate_command();
            batch_commands[1] = 0;
            batch_commands[2] = 0;
            batch_commands[3] = 0;
            transmit_buffer_.add_can2_transmission(0x1FF, std::bit_cast<uint64_t>(batch_commands));

            transmit_buffer_.trigger_transmission();
        }

    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x201) {
                gimbal_left_friction_.store_status(can_data);
            } else if (can_id == 0x202) {
                gimbal_right_friction_.store_status(can_data);
            }
        }

        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x205) {
                gimbal_pitch_motor_.store_status(can_data);
            }
        }

        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }

        void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_accelerometer_status(x, y, z);
        }

        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_gyroscope_status(x - imu_bias_x, y - imu_bias_y, z - imu_bias_z);
        }

        OutputInterface<rmcs_description::Tf>& tf_;

        device::Bmi088 imu_;
        device::Dr16 dr16_;

        int16_t imu_bias_x, imu_bias_y, imu_bias_z = 0.0;

        OutputInterface<double> gimbal_yaw_velocity_imu_;
        OutputInterface<double> gimbal_pitch_velocity_imu_;
        OutputInterface<double> debug_pitch_raw_angle_;
        OutputInterface<double> debug_pitch_temp;
        OutputInterface<double> debug_imu_gx_bais_;
        OutputInterface<double> debug_imu_gy_bais_;
        OutputInterface<double> debug_imu_gz_bais_;

        device::DjiMotor gimbal_pitch_motor_;

        device::DjiMotor gimbal_left_friction_;
        device::DjiMotor gimbal_right_friction_;

        rmcs_core::utility::LowPassFilter<> imu_gy_velocity_filter_{4.0f, 1000.0f};
        rmcs_core::utility::LowPassFilter<> imu_gz_velocity_filter_{8.0f, 1000.0f};

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };

    class BottomBoard final : private librmcs::client::CBoard {
    public:
        friend class MiniInfantry;
        explicit BottomBoard(
            MiniInfantry& mini_infantry, MiniInfantryCommand& mini_infantry_command,
            int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , imu_(10.0f, 0.001f, 1000000.0f)
            , tf_(mini_infantry.tf_)
            , gimbal_yaw_motor_(
                  mini_infantry, mini_infantry_command, "/gimbal/yaw",
                  device::DjiMotor::Config{device::DjiMotor::Type::GM6020}.set_encoder_zero_point(
                      static_cast<int>(
                          mini_infantry.get_parameter("yaw_motor_zero_point").as_int())))
            , gimbal_bullet_feeder_(
                  mini_infantry, mini_infantry_command, "/gimbal/bullet_feeder",
                  device::DjiMotor::Config{device::DjiMotor::Type::M2006}.enable_multi_turn_angle())
            , chassis_wheel_motors_(
                  {mini_infantry, mini_infantry_command, "/chassis/left_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                  {mini_infantry, mini_infantry_command, "/chassis/left_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                  {mini_infantry, mini_infantry_command, "/chassis/right_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                  {mini_infantry, mini_infantry_command, "/chassis/right_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}})
            , supercap_(mini_infantry)
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

                return std::make_tuple(x, y, z);
            });

            mini_infantry.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_multi(
                    [&buffer](std::byte byte) { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                transmit_buffer_.add_uart1_transmission(buffer, size);
                return size;
            };

            mini_infantry.register_output(
                "/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
            mini_infantry.register_output("/debug/yaw/raw_angle", debug_yaw_raw_angle_, 0);
        }

        ~BottomBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            imu_.update_status();

            *chassis_yaw_velocity_imu_ = imu_gz_velocity_filter_.update(imu_.gz());
            *debug_yaw_raw_angle_ = gimbal_yaw_motor_.last_raw_angle();

            gimbal_yaw_motor_.update_status();
            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_yaw_motor_.angle());

            fast_tf::rcl::broadcast_all(*tf_);

            gimbal_bullet_feeder_.update_status();

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();

            supercap_.update_status();
        }

        void command_update() {
            uint16_t batch_commands[4];

            for (int i = 0; i < 4; i++)
                batch_commands[i] = chassis_wheel_motors_[i].generate_command();
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(batch_commands));

            batch_commands[0] = 0;
            batch_commands[1] = gimbal_yaw_motor_.generate_command();
            batch_commands[2] = gimbal_bullet_feeder_.generate_command();
            batch_commands[3] = 0;
            transmit_buffer_.add_can1_transmission(0x1FF, std::bit_cast<uint64_t>(batch_commands));

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
            } else if (can_id == 0x207) {
                gimbal_bullet_feeder_.store_status(can_data);
            } else if (can_id == 0x206) {
                gimbal_yaw_motor_.store_status(can_data);
            }
        }

        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x20c) {
                supercap_.store_status(can_data);
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

        rmcs_core::utility::LowPassFilter<> imu_gz_velocity_filter_{60.0f, 1000.0f};

        OutputInterface<double> chassis_yaw_velocity_imu_;
        OutputInterface<double> debug_yaw_raw_angle_;

        device::DjiMotor gimbal_yaw_motor_;
        device::DjiMotor gimbal_bullet_feeder_;

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

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::MiniInfantry, rmcs_executor::Component)