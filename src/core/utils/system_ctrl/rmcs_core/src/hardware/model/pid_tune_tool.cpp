#include <cmath>
#include <cstdint>
#include <fast_tf/rcl.hpp>
#include <memory>
#include <rmcs_msgs/switch.hpp>
#include <sys/types.h>
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

class PidTuneTool
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PidTuneTool()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<PidTuneToolCommand>(
                  get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        register_output("/tf", tf_);
        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.16, 0.0, 0.15});

        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_,
            static_cast<int>(get_parameter("usb_pid_bottom_board").as_int()));
    }

    ~PidTuneTool() override = default;

    void update() override { bottom_board_->update(); }

    void command_update() { bottom_board_->command_update(); }

private:
    class PidTuneToolCommand : public rmcs_executor::Component {
    public:
        explicit PidTuneToolCommand(PidTuneTool& pid_tune_tool)
            : pid_tune_tool_(pid_tune_tool) {}

        void update() override { pid_tune_tool_.command_update(); }

        PidTuneTool& pid_tune_tool_;
    };
    std::shared_ptr<PidTuneToolCommand> command_component_;

    class BottomBoard final : private librmcs::client::CBoard {
    public:
        friend class PidTuneTool;
        explicit BottomBoard(
            PidTuneTool& pid_tune_tool, PidTuneToolCommand& pid_tune_tool_command, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , tf_(pid_tune_tool.tf_)
            , imu_(10.0f, 0.001f, 1000000.0f)
            , dr16_{pid_tune_tool}
            , velocity_step(pid_tune_tool.get_parameter("velocity_step").as_double())
            , step_time(pid_tune_tool.get_parameter("step_time").as_double())
            , imu_bias_x(pid_tune_tool.get_parameter("imu_bias_x").as_int())
            , imu_bias_y(pid_tune_tool.get_parameter("imu_bias_y").as_int())
            , imu_bias_z(pid_tune_tool.get_parameter("imu_bias_z").as_int())
            , motor_(pid_tune_tool, pid_tune_tool_command, "/motor")

            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {

            //@TODO: IF YOU NEED TO CHANGE MOTOR, CHANGE IT HERE
            motor_.configure(
                device::DmMotor::Config{device::DmMotor::Type::J4310}.set_encoder_zero_point(
                    static_cast<int>(pid_tune_tool.get_parameter("motor_zero_point").as_int())));
            // motor_.configure(
            //     device::DjiMotor::Config{device::DjiMotor::Type::GM6020}.set_encoder_zero_point(
            //         static_cast<int>(
            //             pid_tune_tool.get_parameter("motor_zero_point").as_int())));

            imu_.set_coordinate_mapping([](double x, double y, double z) {
                // Get the mapping with the following code.
                // The rotation angle must be an exact multiple of 90 degrees, otherwise use a
                // matrix.

                // Eigen::AngleAxisd pitch_link_to_imu_link{
                //     std::numbers::pi, Eigen::Vector3d::UnitZ()};
                // Eigen::Vector3d mapping = pitch_link_to_imu_link * Eigen::Vector3d{1, 2, 3};
                // std::cout << mapping << std::endl;

                return std::make_tuple(-y, -x, z);
            });

            pid_tune_tool.register_output("/imu/gz", imu_gz);
            pid_tune_tool.register_output("/imu/gy", imu_gy);

            pid_tune_tool.register_output("/motor/raw_angle", motor_raw_angle_);
            pid_tune_tool.register_output("/motor/control_velocity", velocity_control_);
        }

        ~BottomBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            imu_.update_status();
            Eigen::Quaterniond gimbal_imu_pose{imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3()};

            tf_->set_transform<rmcs_description::BaseLink, rmcs_description::RawImu>(
                gimbal_imu_pose);
            fast_tf::rcl::broadcast_all(*tf_);

            dr16_.update_status();

            // user action begin
            uint64_t current_time =
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                          std::chrono::steady_clock::now().time_since_epoch())
                                          .count());

            if (dr16_.switch_left() != last_left_switch_state_) {
                if (last_left_switch_state_ != rmcs_msgs::Switch::UNKNOWN) {
                    if (dr16_.switch_left() == rmcs_msgs::Switch::MIDDLE
                        && last_left_switch_state_ == rmcs_msgs::Switch::DOWN) {
                        if (dr16_.switch_right() == rmcs_msgs::Switch::DOWN)
                            action_start_time = current_time;
                    }
                }
                last_left_switch_state_ = dr16_.switch_left();
            }

            if (dr16_.switch_right() == rmcs_msgs::Switch::DOWN) {
                if ((current_time - action_start_time) < step_time) {
                    *velocity_control_ = velocity_step;
                } else {
                    *velocity_control_ = 0.0;
                }
            } else {
                *velocity_control_ = nan_;
            }

            // user action end

            motor_.update_status();

            *imu_gz = imu_gz_velocity_filter_.update(imu_.gz());
            *imu_gy = imu_gy_velocity_filter_.update(imu_.gy());

            *motor_raw_angle_ = motor_.last_raw_angle();
        }

        void command_update() {
            //@TODO: IF YOU NEED TO CHANGE MOTOR, CHANGE IT HERE

            // uint16_t can_commands[4];

            // can_commands[0] = 0;
            // can_commands[1] = motor_.generate_command();
            // can_commands[2] = 0;
            // can_commands[3] = 0;
            // transmit_buffer_.add_can1_transmission(0x1FF, std::bit_cast<uint64_t>(can_commands));

            transmit_buffer_.add_can2_transmission(0x9, motor_.generate_torque_command());

            transmit_buffer_.trigger_transmission();
        }

    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {

            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            //@TODO: IF YOU NEED TO CHANGE MOTOR, CHANGE IT HERE
            // if (can_id == 0x206) {
            //     motor_.store_status(can_data);
            // }
        }

        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            //@TODO: IF YOU NEED TO CHANGE MOTOR, CHANGE IT HERE
            if (can_id == 0x219) {
                motor_.store_status(can_data);
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

        static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

        rmcs_msgs::Switch last_left_switch_state_ = rmcs_msgs::Switch::UNKNOWN;
        uint64_t action_start_time = 0;

        double velocity_step;
        double step_time;

        OutputInterface<double> velocity_control_;

        OutputInterface<double> imu_gz;
        OutputInterface<double> imu_gy;
        OutputInterface<double> debug_imu_g_z_;
        OutputInterface<double> motor_raw_angle_;

        int16_t imu_bias_x, imu_bias_y, imu_bias_z = 0.0;

        rmcs_core::utility::LowPassFilter<> imu_gy_velocity_filter_{4.0f, 1000.0f};
        rmcs_core::utility::LowPassFilter<> imu_gz_velocity_filter_{60.0f, 1000.0f};

        //@TODO: IF YOU NEED TO CHANGE MOTOR, CHANGE IT HERE
        device::DmMotor motor_;
        // device::DjiMotor motor_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };

    OutputInterface<rmcs_description::Tf> tf_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    std::unique_ptr<BottomBoard> bottom_board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::PidTuneTool, rmcs_executor::Component)