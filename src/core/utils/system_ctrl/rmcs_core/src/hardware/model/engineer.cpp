#include <fast_tf/rcl.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <thread>

#include <librmcs/client/cboard.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
// #include <rmcs_msgs/serial_interface.hpp>
// #include <rmcs_utility/fps_counter.hpp>
// #include <serial/serial.h>
// #include <std_msgs/msg/int32.hpp>


#include "hardware/device/dm_motor.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/supercap.hpp"
namespace rmcs_core::hardware {

class Engineer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Engineer()
        :Node{
            get_component_name(),
            rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        ,command_component_(create_partner_component<EngineerCommand>(get_component_name()+"_command",*this)){
            arm_board_ = std::make_unique<ArmBoard>(
                *this,*command_component_,
                static_cast<int>(get_parameter("usb_pid_arm_board").as_int()));
        using namespace rmcs_description;
        register_output("/arm_tf", tf_);
        register_output("/arm/tcp/position", tcp_position_);    
        register_output("/arm/tcp/transform", tcp_transform_);
    }
    ~Engineer() override = default;

    void update() override {
        arm_board_->update();
    }
    
    void command_update(){
        arm_board_->command_update();
    }
private: 
    class EngineerCommand : public rmcs_executor::Component {
    public:
        explicit EngineerCommand(Engineer& engineer)
        : engineer_(engineer) {}
        void update() override {engineer_.command_update(); }
        Engineer& engineer_;
    };
    std::shared_ptr<EngineerCommand> command_component_;
    
    class ArmBoard final : private librmcs::client::CBoard {
    public: 
        friend class Engineer;
        explicit ArmBoard(Engineer& engineer, EngineerCommand& engineer_command, int usb_pid = -1)
        : librmcs::client::CBoard(usb_pid)
        ,tf_(engineer.tf_)
        , tcp_position_(engineer.tcp_position_)   
        , tcp_transform_(engineer.tcp_transform_)
        ,arm_joint0_motor_(engineer,engineer_command,"/arm/joint0",
                device::DmMotor::Config{device::DmMotor::Type::J4310})
        ,arm_joint1_motor_(
            engineer,engineer_command,"/arm/joint1",
                device::LkMotor::Config{device::LkMotor::Type::MG4005E_I10})
        ,arm_joint2_motor_(
            engineer,engineer_command,"/arm/joint2",
                device::LkMotor::Config{device::LkMotor::Type::MG4005E_I10})
        ,arm_joint3_motor_(
            engineer,engineer_command,"/arm/joint3",
                device::DmMotor::Config{device::DmMotor::Type::J4310})        
        ,arm_joint4_motor_(
            engineer,engineer_command,"/arm/joint4",
                device::DmMotor::Config{device::DmMotor::Type::J4310})
        ,arm_joint5_motor_(
            engineer,engineer_command,"/arm/joint5",
                device::DmMotor::Config{device::DmMotor::Type::J4310})
        , supercap_(engineer, 28.5)
        ,transmit_buffer_(*this,32)
        ,event_thread_([this](){handle_events();}){
        }


        ~ArmBoard() final {
            stop_handling_events();
            event_thread_.join();
        }
        void update() {
            arm_joint0_motor_.update_status();
            arm_joint1_motor_.update_status();
            arm_joint2_motor_.update_status();
            arm_joint3_motor_.update_status();
            arm_joint4_motor_.update_status();
            arm_joint5_motor_.update_status();
            tf_->set_state<rmcs_description::ArmBaseLink, rmcs_description::ArmLink1>(arm_joint0_motor_.angle());  
            tf_->set_state<rmcs_description::ArmLink1, rmcs_description::ArmLink2>(arm_joint1_motor_.angle());  
            tf_->set_state<rmcs_description::ArmLink2, rmcs_description::ArmLink3>(arm_joint2_motor_.angle());  
            tf_->set_state<rmcs_description::ArmLink3, rmcs_description::ArmLink4>(arm_joint3_motor_.angle());  
            tf_->set_state<rmcs_description::ArmLink4, rmcs_description::ArmLink5>(arm_joint4_motor_.angle());  
            tf_->set_state<rmcs_description::ArmLink5, rmcs_description::ArmLink6>(arm_joint5_motor_.angle());  
            // fast_tf::rcl::broadcast_all(*tf_);

            auto TCP = fast_tf::lookup_transform<rmcs_description::ArmBaseLink, rmcs_description::ArmEndLink>(*tf_);  
            *tcp_position_ = TCP.translation();  
            *tcp_transform_ = TCP.rotation(); 
            
            // RCLCPP_INFO(rclcpp::get_logger("Engineer"), "TCP position: %.3f, %.3f, %.3f", TCP_T.x(), TCP_T.y(), TCP_T.z());
            // RCLCPP_INFO(rclcpp::get_logger("Engineer"), "TCP position: %.3f, %.3f, %.3f", TCP_R.x(), TCP_R.y(), TCP_R.z());
        }
void command_update() {          
        transmit_buffer_.add_can2_transmission(0x140, arm_joint0_motor_.generate_torque_command());
        transmit_buffer_.add_can2_transmission(0x141, arm_joint1_motor_.generate_torque_command()); 
        transmit_buffer_.add_can2_transmission(0x142, arm_joint2_motor_.generate_torque_command());    
        transmit_buffer_.add_can1_transmission(0x03,  arm_joint3_motor_.generate_torque_command());  
        transmit_buffer_.add_can1_transmission(0x04,  arm_joint4_motor_.generate_torque_command()); 
        transmit_buffer_.add_can1_transmission(0x05,  arm_joint5_motor_.generate_torque_command()); 
        transmit_buffer_.trigger_transmission();    
}
    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x53) {
                arm_joint3_motor_.store_status(can_data);
            } else if (can_id == 0x54) {
                arm_joint4_motor_.store_status(can_data);
            } else if (can_id == 0x55) {
                arm_joint5_motor_.store_status(can_data);
            } 
        }
        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x141) {
                arm_joint1_motor_.store_status(can_data);
            } else if (can_id == 0x142) {
                arm_joint2_motor_.store_status(can_data);
            } else if (can_id == 0x140) {
                arm_joint0_motor_.store_status(can_data);
            }
        }
 
        OutputInterface<rmcs_description::ArmTf> &tf_;
        OutputInterface<Eigen::Vector3d>  &tcp_position_;  
        OutputInterface<Eigen::Isometry3d> &tcp_transform_; 
        device::DmMotor arm_joint0_motor_;
        device::LkMotor arm_joint1_motor_;
        device::LkMotor arm_joint2_motor_;
        device::DmMotor arm_joint3_motor_;
        device::DmMotor arm_joint4_motor_;
        device::DmMotor arm_joint5_motor_;
        device::Supercap supercap_;
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };
    OutputInterface<Eigen::Vector3d>  tcp_position_;  
    OutputInterface<Eigen::Isometry3d> tcp_transform_;
    OutputInterface<rmcs_description::ArmTf> tf_;

    std::unique_ptr<ArmBoard> arm_board_;

};

}  // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Engineer, rmcs_executor::Component)
