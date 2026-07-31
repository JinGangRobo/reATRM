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
              create_partner_component<WheelLegInfantryCommand>(
                  get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        register_output("/tf", tf_);
        // tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.16, 0.0, 0.15});

        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_,
            static_cast<int>(get_parameter("usb_pid_bottom_board").as_int()));
    }

    ~WheelLegInfantry() override = default;

    void update() override { bottom_board_->update(); }

    void command_update() { bottom_board_->command_update(); }

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
        explicit BottomBoard(
            WheelLegInfantry& wheeleg_infantry, WheelLegInfantryCommand& wheeleg_infantry_command,
            int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , imu_(10.0f, 0.001f, 1000000.0f)
            , tf_(wheeleg_infantry.tf_)
            , imu_bias_x(wheeleg_infantry.get_parameter("imu_bias_x").as_int())
            , imu_bias_y(wheeleg_infantry.get_parameter("imu_bias_y").as_int())
            , imu_bias_z(wheeleg_infantry.get_parameter("imu_bias_z").as_int())

            , chassis_wheel_motors_(
                  {wheeleg_infantry, wheeleg_infantry_command, "/chassis/left_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(
                       268.0 / 17.0)},
                  {wheeleg_infantry, wheeleg_infantry_command, "/chassis/right_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                       .set_reversed()
                       .set_reduction_ratio(268.0 / 17.0)

                  })
            , left_front_hip_motors_(
                  wheeleg_infantry, wheeleg_infantry_command, "/chassis/left_front_hip",
                  device::DmMotor::Config{device::DmMotor::Type::J4310}
                      .set_reversed()
                      .set_encoder_zero_point(
                          static_cast<int>(
                              wheeleg_infantry.get_parameter("left_front_hip_motors_zero_point")
                                  .as_int())))
            , left_back_hip_motors_(
                  wheeleg_infantry, wheeleg_infantry_command, "/chassis/left_back_hip",
                  device::DmMotor::Config{device::DmMotor::Type::J4310}
                      .set_reversed()
                      .set_encoder_zero_point(
                          static_cast<int>(
                              wheeleg_infantry.get_parameter("left_back_hip_motors_zero_point")
                                  .as_int())))
            , right_front_hip_motors_(
                  wheeleg_infantry, wheeleg_infantry_command, "/chassis/right_front_hip",
                  device::DmMotor::Config{device::DmMotor::Type::J4310}

                      .set_encoder_zero_point(
                          static_cast<int>(
                              wheeleg_infantry.get_parameter("right_front_hip_motors_zero_point")
                                  .as_int())))
            , right_back_hip_motors_(
                  wheeleg_infantry, wheeleg_infantry_command, "/chassis/right_back_hip",
                  device::DmMotor::Config{device::DmMotor::Type::J4310}

                      .set_encoder_zero_point(
                          static_cast<int>(
                              wheeleg_infantry.get_parameter("right_back_hip_motors_zero_point")
                                  .as_int())))
            , dr16_{wheeleg_infantry}

            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {

            imu_.set_coordinate_mapping([](double x, double y, double z) {
                // Get the mapping with the following code.
                // The rotation angle must be an exact multiple of 90 degrees, otherwise use a
                // matrix.

                // Eigen::AngleAxisd pitch_link_to_imu_link{
                // Eigen::Vector3d mapping = pitch_link_to_imu_link * Eigen::Vector3d{1, 2, 3};
                // std::cout << mapping << std::endl;

                return std::make_tuple(-x, -y, z);
            });
            wheeleg_infantry.register_output("/debug/imu/gx_bais", debug_imu_gx_bais_);
            wheeleg_infantry.register_output("/debug/imu/gy_bais", debug_imu_gy_bais_);
            wheeleg_infantry.register_output("/debug/imu/gz_bais", debug_imu_gz_bais_);

            wheeleg_infantry.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_multi(
                    [&buffer](std::byte byte) { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                transmit_buffer_.add_uart1_transmission(buffer, size);
                return size;
            };
            wheeleg_infantry.register_output(
                "/chassis/imu/pitch_velocity", chassis_pitch_velocity_imu_);
            wheeleg_infantry.register_output(
                "/chassis/imu/yaw_velocity", chassis_yaw_velocity_imu_);
            wheeleg_infantry.register_output(
                "/chassis/imu/roll_velocity", chassis_roll_velocity_imu_);

            wheeleg_infantry.register_output("/chassis/imu/pitch", chassis_pitch_angle_imu_);
            wheeleg_infantry.register_output("/chassis/imu/roll", chassis_roll_angle_imu_);
            wheeleg_infantry.register_output("/chassis/imu/yaw", chassis_yaw_angle_imu_);

            wheeleg_infantry.register_output("/debug/imu/ax", imu_ax);
            wheeleg_infantry.register_output("/debug/imu/ay", imu_ay);
            wheeleg_infantry.register_output("/debug/imu/az", imu_az);
            wheeleg_infantry.register_output("/debug/imu/ddz", imu_ddz);
            wheeleg_infantry.register_output("/debug/imu/ddx", imu_ddx);

            wheeleg_infantry.register_output(
                "/debug/left_front_hip/raw_angle", debug_left_front_hip_raw_angle_);
            wheeleg_infantry.register_output(
                "/debug/left_back_hip/raw_angle", debug_left_back_hip_raw_angle_);
            wheeleg_infantry.register_output(
                "/debug/right_front_hip/raw_angle", debug_right_front_hip_raw_angle_);
            wheeleg_infantry.register_output(
                "/debug/right_back_hip/raw_angle", debug_right_back_hip_raw_angle_);
        }

        ~BottomBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            imu_.update_status();
            Eigen::Quaterniond chassis_imu_pose{imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3()};

            // 2. 将世界系向量投影到机体系（推荐方法，防止 Yaw 耦合）
            Eigen::Vector3d up_in_chassis = chassis_imu_pose.conjugate() * Eigen::Vector3d::UnitZ();
            *chassis_pitch_angle_imu_ = std::asin(up_in_chassis.x());
            double w = chassis_imu_pose.w();
            double x = chassis_imu_pose.x();
            double y = chassis_imu_pose.y();
            double z = chassis_imu_pose.z();

            // 1. 直接解算重力向量在车身三轴上的分量（单位：g）
            double gx = 2.0 * (x * z - w * y);
            double gy = 2.0 * (w * x + y * z);
            double gz = w * w - x * x - y * y + z * z;

            // 2. 利用重力分量直接计算出绝对不失真的物理倾角（不受另一个轴的影响）
            *chassis_roll_angle_imu_ = std::atan2(gy, gz);

            // 1. 获取机体的前向向量 (X轴) 在世界系下的方向
            // q 是你的四元数，UnitX 是 (1,0,0)
            Eigen::Vector3d forward_in_world = chassis_imu_pose * Eigen::Vector3d::UnitX();

            // 2. 将其投影到水平面 (即去掉 Z 分量)
            // 这样得到的 Yaw 就是相对于世界系 X 轴的偏航角
            *chassis_yaw_angle_imu_ = std::atan2(forward_in_world.y(), forward_in_world.x());

            *chassis_yaw_velocity_imu_ = imu_gz_velocity_filter_.update(imu_.gz());
            *chassis_pitch_velocity_imu_ = imu_gy_velocity_filter_.update(imu_.gy());
            *chassis_roll_velocity_imu_ = imu_gx_velocity_filter_.update(imu_.gx());

            tf_->set_transform<rmcs_description::BaseLink, rmcs_description::RawImu>(
                chassis_imu_pose);
            fast_tf::rcl::broadcast_all(*tf_);
            dr16_.update_status();
            chassis_wheel_motors_[0].update_status();
            chassis_wheel_motors_[1].update_status();
            left_front_hip_motors_.update_status();

            left_back_hip_motors_.update_status();
            right_front_hip_motors_.update_status();
            right_back_hip_motors_.update_status();

            *debug_left_front_hip_raw_angle_ = left_front_hip_motors_.last_raw_angle();
            *debug_left_back_hip_raw_angle_ = left_back_hip_motors_.last_raw_angle();
            *debug_right_front_hip_raw_angle_ = right_front_hip_motors_.last_raw_angle();
            *debug_right_back_hip_raw_angle_ = right_back_hip_motors_.last_raw_angle();

            *debug_imu_gx_bais_ = imu_.cali_gx_ref();
            *debug_imu_gy_bais_ = imu_.cali_gy_ref();
            *debug_imu_gz_bais_ = imu_.cali_gz_ref();

            // 由加速度计测量值结合姿态矩阵消去重力加速度
            double r11 = chassis_imu_pose.w() * chassis_imu_pose.w()
                       + chassis_imu_pose.x() * chassis_imu_pose.x()
                       - chassis_imu_pose.y() * chassis_imu_pose.y()
                       - chassis_imu_pose.z() * chassis_imu_pose.z();
            double r12 = 2.0
                       * (chassis_imu_pose.x() * chassis_imu_pose.y()
                          - chassis_imu_pose.w() * chassis_imu_pose.z());
            double r13 = 2.0
                       * (chassis_imu_pose.x() * chassis_imu_pose.z()
                          + chassis_imu_pose.w() * chassis_imu_pose.y());

            double r31 = 2.0
                       * (chassis_imu_pose.x() * chassis_imu_pose.z()
                          + chassis_imu_pose.w() * chassis_imu_pose.y());
            double r32 = 2.0
                       * (chassis_imu_pose.y() * chassis_imu_pose.z()
                          - chassis_imu_pose.w() * chassis_imu_pose.x());
            double r33 = chassis_imu_pose.w() * chassis_imu_pose.w()
                       - chassis_imu_pose.x() * chassis_imu_pose.x()
                       - chassis_imu_pose.y() * chassis_imu_pose.y()
                       + chassis_imu_pose.z() * chassis_imu_pose.z();

            // 2. 投影到 Z 轴、减 1g 消除重力、乘以 9.80665 转换单位
            *imu_ddx = (r11 * imu_.ax() + r12 * imu_.ay() + r13 * imu_.az()) * 9.80665;
            *imu_ddz = ((r31 * imu_.ax() + r32 * imu_.ay() + r33 * imu_.az()) - 1.0) * 9.80665;
            *imu_az = imu_.az();
            // *imu_gx = imu_.gx();
            // *imu_gy = imu_.gy();
            // transmit_buffer_.add_can1_transmission(0x01, motor_.generate_torque_command(10));
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

        void command_update() {
            uint16_t can_commands[4];

            can_commands[0] = chassis_wheel_motors_[0].generate_command();
            can_commands[1] = chassis_wheel_motors_[1].generate_command();
            can_commands[2] = 0;
            can_commands[3] = 0;
            transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

            transmit_buffer_.add_can1_transmission(
                0x01, left_front_hip_motors_.generate_torque_command());
            transmit_buffer_.add_can1_transmission(
                0x02, left_back_hip_motors_.generate_torque_command());
            transmit_buffer_.add_can1_transmission(
                0x03, right_front_hip_motors_.generate_torque_command());
            transmit_buffer_.add_can1_transmission(
                0x04, right_back_hip_motors_.generate_torque_command());

            transmit_buffer_.trigger_transmission();
        }

    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x211) {
                left_front_hip_motors_.store_status(can_data);
            } else if (can_id == 0x212) {
                left_back_hip_motors_.store_status(can_data);
            } else if (can_id == 0x213) {
                right_front_hip_motors_.store_status(can_data);
            } else if (can_id == 0x214) {
                right_back_hip_motors_.store_status(can_data);
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
            }
        }
        device::DjiMotor chassis_wheel_motors_[2];

        device::DmMotor left_front_hip_motors_;
        device::DmMotor left_back_hip_motors_;
        device::DmMotor right_front_hip_motors_;
        device::DmMotor right_back_hip_motors_;

        device::Dr16 dr16_;
        OutputInterface<double> imu_gx;
        OutputInterface<double> imu_gz;
        OutputInterface<double> imu_gy;

        OutputInterface<double> imu_ax;
        OutputInterface<double> imu_az;
        OutputInterface<double> imu_ay;

        OutputInterface<double> imu_ddz;
        OutputInterface<double> imu_ddx;

        OutputInterface<double> debug_imu_g_z_;

        int16_t imu_bias_x, imu_bias_y, imu_bias_z = 0.0;

        OutputInterface<rmcs_description::Tf>& tf_;
        OutputInterface<double> debug_imu_gx_bais_;
        OutputInterface<double> debug_imu_gy_bais_;
        OutputInterface<double> debug_imu_gz_bais_;

        OutputInterface<double> chassis_yaw_velocity_imu_;
        OutputInterface<double> chassis_pitch_velocity_imu_;
        OutputInterface<double> chassis_roll_velocity_imu_;

        OutputInterface<double> chassis_pitch_angle_imu_;
        OutputInterface<double> chassis_yaw_angle_imu_;
        OutputInterface<double> chassis_roll_angle_imu_;

        OutputInterface<double> debug_left_front_hip_raw_angle_;
        OutputInterface<double> debug_left_back_hip_raw_angle_;
        OutputInterface<double> debug_right_front_hip_raw_angle_;
        OutputInterface<double> debug_right_back_hip_raw_angle_;

        librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
        rmcs_core::utility::LowPassFilter<> imu_gx_velocity_filter_{4.0f, 1000.0f};
        rmcs_core::utility::LowPassFilter<> imu_gy_velocity_filter_{4.0f, 1000.0f};
        rmcs_core::utility::LowPassFilter<> imu_gz_velocity_filter_{8.0f, 1000.0f};
        device::Bmi088 imu_;
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