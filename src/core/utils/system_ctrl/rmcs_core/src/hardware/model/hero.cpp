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

#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/supercap.hpp"

namespace rmcs_core::hardware {

class Hero
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Hero()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<HeroCommand>(get_component_name() + "_command", *this)) {
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

    ~Hero() override = default;

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

    class HeroCommand : public rmcs_executor::Component {
    public:
        explicit HeroCommand(Hero& hero)
            : hero_(hero) {}

        void update() override { hero_.command_update(); }

        Hero& hero_;
    };
    std::shared_ptr<HeroCommand> command_component_;

    class TopBoard final : private librmcs::client::CBoard {
    public:
        friend class Hero;
        explicit TopBoard(Hero& hero, HeroCommand& hero_command, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , tf_(hero.tf_)
            , imu_(10.0f, 0.001f, 1000000.0f)
            , dr16_(hero)
            , imu_bias_x(hero.get_parameter("imu_bias_x").as_int())
            , imu_bias_y(hero.get_parameter("imu_bias_y").as_int())
            , imu_bias_z(hero.get_parameter("imu_bias_z").as_int())
            , gimbal_pitch_motor_(
                  hero, hero_command, "/gimbal/pitch",
                  device::DmMotor::Config{device::DmMotor::Type::J4310}
                      .set_encoder_zero_point(
                          static_cast<int>(hero.get_parameter("pitch_motor_zero_point").as_int()))
                      .set_reversed())

            // TODO: Bad CAN ID sequence, needs to be adjusted.
            , gimbal_friction_wheels_(
                  {hero, hero_command, "/gimbal/first_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                       .set_reduction_ratio(1.)
                       .set_reversed()},
                  {hero, hero_command, "/gimbal/second_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(1.)},
                  {hero, hero_command, "/gimbal/third_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                       .set_reduction_ratio(1.)
                       .set_reversed()})
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

                return std::make_tuple(-y, x, z);
            });

            hero.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
            hero.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);

            hero.register_output("/debug/pitch/raw_angle", debug_pitch_raw_angle_);
            hero.register_output("/debug/pitch/temp", debug_pitch_temp);
            hero.register_output("/debug/imu/gx_bais", debug_imu_gx_bais_);
            hero.register_output("/debug/imu/gy_bais", debug_imu_gy_bais_);
            hero.register_output("/debug/imu/gz_bais", debug_imu_gz_bais_);
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

            for (auto& motor : gimbal_friction_wheels_)
                motor.update_status();
        }

        void command_update() {
            uint16_t batch_commands[4];

            for (int i = 0; i < 3; i++)
                batch_commands[i] = gimbal_friction_wheels_[i].generate_command();
            batch_commands[3] = 0;
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(batch_commands));

            transmit_buffer_.add_can2_transmission(
                0x9, gimbal_pitch_motor_.generate_torque_command());

            transmit_buffer_.trigger_transmission();
        }

    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x201) {
                gimbal_friction_wheels_[0].store_status(can_data);
            } else if (can_id == 0x202) {
                gimbal_friction_wheels_[1].store_status(can_data);
            } else if (can_id == 0x203) {
                gimbal_friction_wheels_[2].store_status(can_data);
            }
        }

        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x219) {
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

        device::DmMotor gimbal_pitch_motor_;

        device::DjiMotor gimbal_friction_wheels_[3];

        rmcs_core::utility::LowPassFilter<> imu_gy_velocity_filter_{4.0f, 1000.0f};
        rmcs_core::utility::LowPassFilter<> imu_gz_velocity_filter_{8.0f, 1000.0f};

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };

    class BottomBoard final : private librmcs::client::CBoard {
    public:
        friend class Hero;
        explicit BottomBoard(Hero& hero, HeroCommand& hero_command, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , imu_(10.0f, 0.001f, 1000000.0f)
            , tf_(hero.tf_)
            , chassis_wheel_motors_(
                  {hero, hero_command, "/chassis/left_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                  {hero, hero_command, "/chassis/left_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                  {hero, hero_command, "/chassis/right_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                  {hero, hero_command, "/chassis/right_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}})

            , supercap_(hero)
            , gimbal_yaw_motor_(
                  hero, hero_command, "/gimbal/yaw",
                  device::DmMotor::Config{device::DmMotor::Type::J4310}.set_encoder_zero_point(
                      static_cast<int>(hero.get_parameter("yaw_motor_zero_point").as_int())))
            , gimbal_bullet_feeder_(
                  hero, hero_command, "/gimbal/bullet_feeder",
                  device::DmMotor::Config{device::DmMotor::Type::J4310}.enable_multi_turn_angle())
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

            hero.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_multi(
                    [&buffer](std::byte byte) { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                transmit_buffer_.add_uart1_transmission(buffer, size);
                return size;
            };

            hero.register_output("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
            hero.register_output("/debug/yaw/raw_angle", debug_yaw_raw_angle_);
        }

        ~BottomBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            imu_.update_status();
            gimbal_yaw_motor_.update_status();
            *chassis_yaw_velocity_imu_ = imu_gz_velocity_filter_.update(imu_.gz());

            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_yaw_motor_.angle());

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
            gimbal_bullet_feeder_.update_status();

            supercap_.update_status();

            *debug_yaw_raw_angle_ = gimbal_yaw_motor_.last_raw_angle();
        }

        void command_update() {
            uint16_t batch_commands[4];

            for (int i = 0; i < 4; i++)
                batch_commands[i] = chassis_wheel_motors_[i].generate_command();
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(batch_commands));

            transmit_buffer_.add_can2_transmission(
                0x4, gimbal_bullet_feeder_.generate_torque_command());
            transmit_buffer_.add_can2_transmission(
                0x2, gimbal_yaw_motor_.generate_torque_command());

            transmit_buffer_.trigger_transmission();
            // RCLCPP_INFO(
            //     rclcpp::get_logger("MOTOR"), "send command: %d, %d, %d, %d", batch_commands[0],
            //     batch_commands[1], batch_commands[2], batch_commands[3]);
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

        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x214) {
                gimbal_bullet_feeder_.store_status(can_data);
            } else if (can_id == 0x212) {
                gimbal_yaw_motor_.store_status(can_data);
            } else if (can_id == 0x20c) {
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
        OutputInterface<double> debug_yaw_raw_angle_;

        rmcs_core::utility::LowPassFilter<> imu_gz_velocity_filter_{60.0f, 1000.0f};

        OutputInterface<double> chassis_yaw_velocity_imu_;

        device::DjiMotor chassis_wheel_motors_[4];
        device::Supercap supercap_;
        device::DmMotor gimbal_yaw_motor_;

        device::DmMotor gimbal_bullet_feeder_;

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

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Hero, rmcs_executor::Component)