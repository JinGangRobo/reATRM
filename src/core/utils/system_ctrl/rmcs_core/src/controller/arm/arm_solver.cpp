#include <cstddef>
#include <eigen3/Eigen/Dense>
#include <array>
#include <cmath>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>
#include <string>
#include <string_view>
#include <unordered_set>
#include <tuple>
#include <vector>
#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::arm {

class ArmSolver final
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using TorqueVec = Eigen::Array<double, 6, 1>;
public:
        explicit ArmSolver()
        : Node(
                get_component_name(),
                rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , joint_angle_pid_controller{
                pid::PidCalculator(200.0, 0.0, 0.0),
                pid::PidCalculator(1600.0, 0.0, 0.0),
                pid::PidCalculator(200.0, 0.0, 0.0),
                pid::PidCalculator(200.0, 0.0, 0.0),
                pid::PidCalculator(200.0, 0.0, 0.0),
                pid::PidCalculator(50.0, 0.0, 0.0)   }
        , joint_vel_pid_controller{
                pid::PidCalculator(1.0, 0.0, 0.0),
                pid::PidCalculator(1.0, 0.0, 0.0),
                pid::PidCalculator(1.0, 0.0, 0.0),
                pid::PidCalculator(0.2, 0.0, 0.0),
                pid::PidCalculator(0.2, 0.0, 0.0),
                pid::PidCalculator(0.1, 0.0, 0.0)   } 
        {
            for(std::size_t i = 0; i < 6; ++i){
                const std::string joint_prefix = "/arm/joint_" + std::to_string(i+1);
                register_input(joint_prefix + "/theta", joint_theta[i]);
                register_input(joint_prefix + "/target_theta", joint_target_theta[i]);
                register_input(joint_prefix + "/lower_limit", joint_lower_limit_[i]);
                register_input(joint_prefix + "/upper_limit", joint_upper_limit_[i]);
                register_input(joint_prefix + "/velocity", joint_velocity_[i]);
                register_input(joint_prefix + "/friction", joint_friction_[i]);

                register_output(joint_prefix + "/motor/control_torque", target_torque_[i], NAN);
            }
            for(std::size_t i = 0; i < 6; ++i){
                const std::string joint_prefix = "/arm/link_" + std::to_string(i+1);
                register_input(joint_prefix + "/mass", link_mass_[i]);
                register_input(joint_prefix + "/length", link_length_[i]);
                register_input(joint_prefix + "/com", link_com_[i]);
            }
            register_input("urdf_loaded", is_loaded);
            register_input("/arm/enable_flag", is_arm_enable);
            const auto list = this->get_parameter("controller_list").as_string_array();
            load_controller_list(list);
        }

        void update() override {
            //todo
            
            TorqueVec torque_cmd;
            if(!*is_loaded){
                RCLCPP_WARN(rclcpp::get_logger("[Arm_Solver]"), "URDF NOT Fatel");
                return;
            }
            torque_cmd.setZero();
            if(*is_arm_enable){
                if(!last_is_arm_enable){
                    for(auto& pid: joint_angle_pid_controller){
                        pid.reset();
                    }
                    for(auto& pid: joint_vel_pid_controller){
                        pid.reset();
                    }
                }
                for(auto fn : controller_list_){
                    torque_cmd += (this->*fn)();
                }
            } else {
                torque_cmd = zero_calculate();
            }
            last_is_arm_enable = *is_arm_enable;
            for(std::size_t i = 0; i < 6;++i ){
                *target_torque_[i] = torque_cmd[i];
            }
        }

private:
    using controller_type = TorqueVec (ArmSolver::*)();
    std::vector<controller_type> controller_list_;
    static double normalize_angle(double angle) {
        return std::remainder(angle, 2.0 * M_PI);
    }
    TorqueVec pid_calculate(){
        auto clamp_angle = [this](std::size_t idx, double target_theta) {
            const double lower_limit = *joint_lower_limit_[idx];
            const double upper_limit = *joint_upper_limit_[idx];
            if (target_theta < lower_limit) {
                return lower_limit;
            } else if (target_theta > upper_limit) {
                return upper_limit;
            }
            return target_theta;
        };
        TorqueVec torque_pid;
        torque_pid.setZero();
        for(std::size_t i = 0; i < 6; ++i){
            const double current_theta = *joint_theta[i];
            const double target_theta = clamp_angle(i, *joint_target_theta[i]);
            const double current_vel = *joint_velocity_[i];

            const double angle_error = normalize_angle(target_theta - current_theta);
            const double target_vel = joint_angle_pid_controller[i].update(angle_error);
            const double vel_error = target_vel - current_vel;
            torque_pid[i] = joint_vel_pid_controller[i].update(vel_error);
        }
        return torque_pid;
    }
    TorqueVec gravity_calculate(){
        //Todo
        TorqueVec torque_gravity;
        torque_gravity.setZero();
        return torque_gravity;
    }
    TorqueVec friction_calculate(){
        //Todo
        TorqueVec torque_friction;
        torque_friction.setZero();
        return torque_friction;
    }
    TorqueVec zero_calculate(){
        TorqueVec torque_zero;
        torque_zero.fill(NAN);
        return torque_zero;
    }
    static constexpr std::array<std::tuple<std::string_view, controller_type>, 4> term_table_{
        {{"gravity", &ArmSolver::gravity_calculate},
         {"pid", &ArmSolver::pid_calculate},
         {"friction", &ArmSolver::friction_calculate},
         {"zero_torque", &ArmSolver::zero_calculate}}
    };
    void load_controller_list(const std::vector<std::string>& list){
        //todo Queue->gravity + friction + control_torque
        controller_list_.clear();
        controller_list_.reserve(list.size());
        std::unordered_set<std::string_view> record;
        record.reserve(list.size());
        for(const auto& name: list){
            const std::string_view name_view(name);
            if(!record.insert(name_view).second){
                RCLCPP_WARN(rclcpp::get_logger("ArmSolver"), "Duplicate controller name: %s", name.c_str());
                continue;
            }
            bool find = false;
            for(const auto& [term_name, fn]: term_table_){
                if(name_view == term_name){
                    controller_list_.push_back(fn);
                    find = true;
                    break;
                }
            }
            if(!find){
                RCLCPP_ERROR(rclcpp::get_logger("ArmSolver"), "Unknown controller name: %s", name.c_str());
            }
        }
    }
    std::array<pid::PidCalculator, 6> joint_angle_pid_controller;
    std::array<pid::PidCalculator, 6> joint_vel_pid_controller;
    std::array<InputInterface<double>, 6>  joint_theta;
    std::array<InputInterface<double>, 6>  joint_target_theta;
    std::array<InputInterface<double>, 6>  joint_lower_limit_;
    std::array<InputInterface<double>, 6>  joint_upper_limit_;
    std::array<InputInterface<double>, 6>  joint_velocity_;
    std::array<InputInterface<double>, 6>  joint_friction_;
    std::array<OutputInterface<double>, 6>  target_torque_;
    std::array<InputInterface<double>, 6>  link_mass_;
    std::array<InputInterface<double>, 6>  link_length_;
    std::array<InputInterface<Eigen::Vector3d>, 6>  link_com_;
    InputInterface<bool> is_loaded;
    InputInterface<bool> is_arm_enable;
    bool last_is_arm_enable{false};
    };
} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmSolver, rmcs_executor::Component)
