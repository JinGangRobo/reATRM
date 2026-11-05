#include <bit>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <limits>
#include <fast_tf/rcl.hpp>
#include <memory>
#include <string>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <thread>

#include <librmcs/client/cboard.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/fps_counter.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/bmi088.hpp"
// #include "hardware/device/buzzer.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/supercap.hpp"
#include "hardware/device/servo_protocol.hpp"

namespace rmcs_core::hardware {

namespace {
} // namespace

class Dart_launcher
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Dart_launcher()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<Dart_launcherCommend>(
                  get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        register_output("/tf", tf_);

        trigger_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/trigger/calibrate", rclcpp::QoS{1}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                trigger_calibrate_subscription_callback(std::move(msg));
            });

            c_board_ = std::make_unique<CBoard>(
            *this, *command_component_,
            static_cast<int>(get_parameter("usb_pid_top_board").as_int()));
    }

    ~Dart_launcher() override = default;

    void update() override {
        if (c_board_)
            c_board_->update();
    }

    void command_update() {
        if (c_board_)
            c_board_->command_update();
    }
    // 这是云台的控制程序包括日志输出和状态更新
private:
    void trigger_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        if (!c_board_) {
            RCLCPP_WARN(get_logger(), "Trigger calibrate requested but top board is  unavailable.");
            return;
        }
        RCLCPP_INFO(
            // 0为左 1为右
            get_logger(), "[trigger calibrate] New left offset: %d",
            c_board_->trigger_motor_[0].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[trigger calibrate] New right offset: %d",
            c_board_->trigger_motor_[1].calibrate_zero_point());
    }
    class Dart_launcherCommend : public rmcs_executor::Component {
    public:
        explicit Dart_launcherCommend(Dart_launcher& dart_launcher)
            : dart_launcher(dart_launcher) {
            register_input("/trigger/left_motor/release", left_motor_release_, false);
            register_input("/trigger/right_motor/release", right_motor_release_, false);
            for (std::size_t index = 0; index < 4; ++index)
                register_input(
                    "/trigger/servo/servo_" + std::to_string(index + 1) + "/angle",
                    servo_angle_[index]);
            register_input(
                "/trigger/servo/fire_request_sequence", fire_request_sequence_);
        }

        void update() override { dart_launcher.command_update(); }

        bool left_motor_release() const {
            return left_motor_release_.ready() && *left_motor_release_;
        }

        bool right_motor_release() const {
            return right_motor_release_.ready() && *right_motor_release_;
        }

        double servo_angle(std::size_t index) const { return *servo_angle_[index]; }

        std::uint8_t fire_request_sequence() const {
            return fire_request_sequence_.ready() ? *fire_request_sequence_ : 0;
        }

        Dart_launcher& dart_launcher;

    private:
        InputInterface<bool> left_motor_release_;
        InputInterface<bool> right_motor_release_;
        InputInterface<double> servo_angle_[4];
        InputInterface<std::uint8_t> fire_request_sequence_;
    };
    std::shared_ptr<Dart_launcherCommend> command_component_;
    OutputInterface<rmcs_description::Tf> tf_;
    class CBoard final : private librmcs::client::CBoard {
    public:
        friend class Dart_launcher;
        explicit CBoard(
            Dart_launcher& dart_launcher, Dart_launcherCommend& dart_launchercommand,
            int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            // , tf_(dart_launcher.tf_)
            , command_component_(dart_launchercommand)
            , dr16_(dart_launcher)
            // , buzzer_(dart_launchercommand)
            , supercap_(dart_launcher, 28.5)
            , trigger_motor_(
                  {dart_launcher, dart_launchercommand, "/trigger/left_motor",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                       .set_encoder_zero_point(
                           static_cast<int>(
                               dart_launcher.get_parameter("left_motor_zero_point").as_int()))
                       .set_reduction_ratio(3591.0 / 187.0)
                       .enable_multi_turn_angle()},
                  {dart_launcher, dart_launchercommand, "/trigger/right_motor",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                       .set_encoder_zero_point(
                           static_cast<int>(
                               dart_launcher.get_parameter("right_motor_zero_point").as_int()))
                       .set_reduction_ratio(3591.0 / 187.0)
                       .enable_multi_turn_angle()

                  })
            , launcher_motor_(
                  {dart_launcher, dart_launchercommand, "/yaw_motor",
                   device::DjiMotor::Config{device::DjiMotor::Type::M2006}
                       .set_encoder_zero_point(
                           static_cast<int>(
                               dart_launcher.get_parameter("yaw_motor_zero_point").as_int()))
                       .set_reduction_ratio(36.0)
                       .enable_multi_turn_angle()},
                  {dart_launcher, dart_launchercommand, "/distance_motor",
                   device::DjiMotor::Config{device::DjiMotor::Type::M2006}
                       .set_encoder_zero_point(
                           static_cast<int>(
                               dart_launcher.get_parameter("distance_motor_zero_point").as_int()))
                       .set_reduction_ratio(36.0)
                       .enable_multi_turn_angle()})
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {
            for (std::size_t index = 0; index < 4; ++index) {
                debug_servo_angles_[index].store(std::numeric_limits<double>::quiet_NaN());
                debug_servo_subscriptions_[index] =
                    dart_launcher.create_subscription<std_msgs::msg::Float64>(
                        "/debug/trigger/servo/servo_" + std::to_string(index + 1) + "/angle",
                        rclcpp::QoS{1},
                        [this, index](std_msgs::msg::Float64::UniquePtr&& msg) {
                            if (!msg || !std::isfinite(msg->data)) {
                                RCLCPP_WARN(
                                    rclcpp::get_logger("Dart_launcher"),
                                    "Debug servo command rejected: id=%zu angle is not finite",
                                    index + 1);
                                return;
                            }
                            const auto angle = std::clamp(msg->data, 0.0, 180.0);
                            debug_servo_angles_[index].store(angle);
                            RCLCPP_DEBUG(
                                rclcpp::get_logger("Dart_launcher"),
                                "Debug servo command received: id=%zu angle=%.1f",
                                index + 1, angle);
                        });
            }
            for (std::size_t index = 0; index < 4; ++index)
                dart_launcher.register_output(
                    "/trigger/servo/servo_" + std::to_string(index + 1) + "/feedback_angle",
                    servo_feedback_angle_[index], 0.0);
            RCLCPP_INFO(
                rclcpp::get_logger("Dart_launcher"),
                "Dart launcher USB board initialized: pid=0x%04x",
                static_cast<unsigned>(usb_pid));
            dart_launcher.register_output("/referee/serial", referee_serial_);
            dart_launcher.register_output("/trigger/servo/response_sequence", servo_response_sequence_, 0.0);
            dart_launcher.register_output("/trigger/servo/response_command", servo_response_command_, 0.0);
            dart_launcher.register_output("/trigger/servo/response_status", servo_response_status_, 0.0);
            dart_launcher.register_output("/trigger/servo/response_id", servo_response_id_, 0.0);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_multi(
                    [&buffer](std::byte byte) { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                transmit_buffer_.add_uart1_transmission(buffer, size);
                return size;
            };
        }

        ~CBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            imu_.update_status();
            Eigen::Quaterniond gimbal_imu_pose{imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3()};

            *debug_imu_gx_bais_ = imu_.cali_gx_ref();
            *debug_imu_gy_bais_ = imu_.cali_gy_ref();
            *debug_imu_gz_bais_ = imu_.cali_gz_ref();

            // tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
            //     gimbal_imu_pose.conjugate());
            // tf_->set_transform<rmcs_description::BaseLink, rmcs_description::RawImu>(
            //     gimbal_imu_pose);

            for (auto& motor : trigger_motor_)
                motor.update_status();
            for (auto& motor : launcher_motor_)
                motor.update_status();
            supercap_.update_status();
            dr16_.update_status();
            // buzzer_.update_status();

            // fast_tf::rcl::broadcast_all(*tf_);
        }

        void command_update() {
            uint16_t batch_commands[4];
            batch_commands[0] = command_component_.left_motor_release()
                                  ? static_cast<uint16_t>(
                                      static_cast<librmcs::device::DjiMotor&>(
                                          trigger_motor_[0]).generate_command(0.0))
                                  : trigger_motor_[0].generate_command();
            batch_commands[1] = command_component_.right_motor_release()
                                  ? static_cast<uint16_t>(
                                      static_cast<librmcs::device::DjiMotor&>(
                                          trigger_motor_[1]).generate_command(0.0))
                                  : trigger_motor_[1].generate_command();
            batch_commands[2] = launcher_motor_[0].generate_command();
            batch_commands[3] = launcher_motor_[1].generate_command();
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(batch_commands));

            bool servo_command_added = false;
            const auto fire_request_sequence = command_component_.fire_request_sequence();
            const bool fire_request_pending =
                fire_request_sequence != last_fire_request_sequence_;
            for (std::size_t index = 0; index < 4; ++index) {
                const auto debug_angle = debug_servo_angles_[index].load();
                const auto servo_angle = normalize_servo_angle(
                    std::isfinite(debug_angle) ? debug_angle : command_component_.servo_angle(index));
                const bool force_fire_command = index == 0 && fire_request_pending;
                if (!force_fire_command && servo_angle == last_servo_angle_[index])
                    continue;
                std::byte servo_packet[device::ServoProtocol::command_packet_size];
                device::ServoProtocol::Config servo_config{static_cast<std::uint8_t>(index + 1)};
                device::ServoProtocol servo{servo_config};
                const auto servo_size = servo.generate_command(servo_angle, servo_sequence_, servo_packet);
                RCLCPP_DEBUG(
                    rclcpp::get_logger("Dart_launcher"),
                    "Servo frame check: USB=81 field=7d len=%zu data=%02x %02x %02x %02x %02x %02x %02x",
                    servo_size, std::to_integer<unsigned>(servo_packet[0]),
                    std::to_integer<unsigned>(servo_packet[1]),
                    std::to_integer<unsigned>(servo_packet[2]),
                    std::to_integer<unsigned>(servo_packet[3]),
                    std::to_integer<unsigned>(servo_packet[4]),
                    std::to_integer<unsigned>(servo_packet[5]),
                    std::to_integer<unsigned>(servo_packet[6]));
                if (transmit_buffer_.add_servo_transmission(servo_packet, servo_size)) {
                    servo_command_added = true;
                    last_servo_angle_[index] = servo_angle;
                    pending_servo_sequence_[index] = servo_sequence_;
                    pending_servo_command_[index] = true;
                    if (force_fire_command)
                        last_fire_request_sequence_ = fire_request_sequence;
                    RCLCPP_DEBUG(
                        rclcpp::get_logger("Dart_launcher"),
                        "Servo command queued: id=%u angle=%.1f",
                        static_cast<unsigned>(index + 1), servo_angle);
                    ++servo_sequence_;
                }
            }

            const auto transmission_started = transmit_buffer_.trigger_transmission();
            if (!transmission_started && servo_command_added) {
                RCLCPP_WARN(rclcpp::get_logger("Dart_launcher"),
                    "Servo command queued, but the current USB frame could not be started");
            }
        }

    private:
        static double normalize_servo_angle(double angle) {
            if (!std::isfinite(angle))
                return 0.0;
            return std::clamp(angle, 0.0, 180.0);
        }

        void servo_receive_callback(const std::byte* data, uint8_t length) override {
            const auto id = length >= 6 ? static_cast<std::uint8_t>(data[5]) : 0;
            if (id < 1 || id > 4) {
                RCLCPP_WARN(
                    rclcpp::get_logger("Dart_launcher"),
                    "Servo response rejected: length=%u id=%u", length, id);
                return;
            }
            if (!servo_devices_[id - 1].store_status(data, length)) {
                RCLCPP_WARN(
                    rclcpp::get_logger("Dart_launcher"),
                    "Servo response rejected: id=%u length=%u", id, length);
                return;
            }
            *servo_feedback_angle_[id - 1] = servo_devices_[id - 1].angle();
            if (!pending_servo_command_[id - 1]
                || !servo_devices_[id - 1].response_matches(
                    servo_devices_[id - 1].response_sequence(),
                    device::ServoProtocol::Command::SET_ANGLE)) {
                RCLCPP_WARN(
                    rclcpp::get_logger("Dart_launcher"),
                    "Servo response ignored: id=%u sequence=%u has no matching request",
                    id, servo_devices_[id - 1].response_sequence());
                return;
            }
            if (servo_devices_[id - 1].response_sequence() != pending_servo_sequence_[id - 1]) {
                RCLCPP_WARN(
                    rclcpp::get_logger("Dart_launcher"),
                    "Servo response ignored: id=%u sequence=%u expected=%u",
                    id, servo_devices_[id - 1].response_sequence(),
                    pending_servo_sequence_[id - 1]);
                return;
            }
            pending_servo_command_[id - 1] = false;
            *servo_response_command_ = servo_devices_[id - 1].response_command();
            *servo_response_sequence_ = servo_devices_[id - 1].response_sequence();
            *servo_response_status_ = servo_devices_[id - 1].response_status();
            *servo_response_id_ = id;
            RCLCPP_DEBUG(rclcpp::get_logger("Dart_launcher"),
                "Servo response received: id=%u command=0x%02x sequence=%u status=0x%02x",
                id, *servo_response_command_, *servo_response_sequence_, *servo_response_status_);
        }

        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x201) {
                log_can_feedback_once(can_id);
                trigger_motor_[0].store_status(can_data);

            } else if (can_id == 0x202) {
                log_can_feedback_once(can_id);
                trigger_motor_[1].store_status(can_data);
            } else if (can_id == 0x203) {
                log_can_feedback_once(can_id);
                launcher_motor_[0].store_status(can_data);
            } else if (can_id == 0x204) {
                log_can_feedback_once(can_id);
                launcher_motor_[1].store_status(can_data);
            } else if (can_id == 0x20c) {
                supercap_.store_status(can_data);
            }
        }

        void log_can_feedback_once(uint32_t can_id) {
            const std::size_t index =
                can_id == 0x201 ? 0 : can_id == 0x202 ? 1 : can_id == 0x203 ? 2
                    : 3;
            if (can_feedback_seen_[index])
                return;
            can_feedback_seen_[index] = true;
            RCLCPP_INFO(
                rclcpp::get_logger("Dart_launcher"),
                "CAN1 feedback detected: id=0x%03x",
                can_id);
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
        void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            const auto accepted = referee_ring_buffer_receive_.emplace_back_multi(
                [&uart_data](std::byte* storage) { *storage = *uart_data++; }, uart_data_length);
            if (accepted != uart_data_length) {
                RCLCPP_WARN(
                    rclcpp::get_logger("Dart_launcher"),
                    "Referee UART receive buffer overflow: accepted %zu/%u bytes",
                    accepted, uart_data_length);
            }
        }
        // OutputInterface<rmcs_description::Tf>& tf_;
        Dart_launcherCommend& command_component_;

        device::Bmi088 imu_;
        device::Dr16 dr16_;
        // device::Buzzer buzzer_;
        device::Supercap supercap_;
        int16_t imu_bias_x = 0;
        int16_t imu_bias_y = 0;
        int16_t imu_bias_z = 0;
        librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};

        OutputInterface<double> debug_imu_gx_bais_;
        OutputInterface<double> debug_imu_gy_bais_;
        OutputInterface<double> debug_imu_gz_bais_;

        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        device::DjiMotor trigger_motor_[2];
        device::DjiMotor launcher_motor_[2];
        OutputInterface<uint8_t> servo_response_sequence_;
        OutputInterface<uint8_t> servo_response_command_;
        OutputInterface<uint8_t> servo_response_status_;
        OutputInterface<uint8_t> servo_response_id_;
        OutputInterface<double> servo_feedback_angle_[4];
        double last_servo_angle_[4] = {0.0, 0.0, 0.0, 0.0};
        std::uint8_t servo_sequence_ = 0;
        std::uint8_t last_fire_request_sequence_ = 0;
        std::uint8_t pending_servo_sequence_[4] = {};
        bool pending_servo_command_[4] = {};
        bool can_feedback_seen_[4] = {};
        device::ServoProtocol servo_devices_[4] = {
            device::ServoProtocol{device::ServoProtocol::Config{1}},
            device::ServoProtocol{device::ServoProtocol::Config{2}},
            device::ServoProtocol{device::ServoProtocol::Config{3}},
            device::ServoProtocol{device::ServoProtocol::Config{4}}};

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
        std::array<std::atomic<double>, 4> debug_servo_angles_;
        std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr, 4>
            debug_servo_subscriptions_;
    };
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trigger_calibrate_subscription_;

    std::unique_ptr<CBoard> c_board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Dart_launcher, rmcs_executor::Component)
