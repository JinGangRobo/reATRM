#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/util/Meta.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <string>
#include <rmcs_executor/component.hpp>
#include "utility/low_pass_filter.hpp"
#include <rmcs_msgs/arm_mode.hpp>
#include <rmcs_msgs/vtswitch.hpp>
#include "controller/arm/Action_planner/action_step.hpp"
#include "controller/arm/Action_planner/arm_action_machine.hpp"

namespace rmcs_core::controller::arm {

class ArmController final
    : public rmcs_executor::Component
    , public rclcpp::Node {    
public:
        ArmController()
        : Node(
                get_component_name(),
                rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) 
        // , action_machine_()
        , custom_joint_filter_(0.2){
            register_input("/vt03/mode_switch", mode_switch_);
            register_input("/vt03/fn_1", fn_1_);
            register_input("/vt03/fn_2", fn_2_);
            register_input("/vt03/rotary_knob", rotary_knob_);
            register_input("/vt03/joystick/right", joystick_right_);
            register_input("/vt03/joystick/left", joystick_left_);
            register_output("/arm/mode", arm_mode_);
            for(std::size_t i = 0; i<6; ++i){
                const std::string joint_prefix = "/arm/joint_" + std::to_string(i + 1);
                register_input(joint_prefix + "/theta", theta[i]);
                register_input(joint_prefix + "/lower_limit", joint_lower_limit_[i]);
                register_input(joint_prefix + "/upper_limit", joint_upper_limit_[i]);
                register_output(joint_prefix + "/target_theta", target_theta[i], NAN);
            }
            register_output("/arm/enable_flag", is_arm_enable_, false);
                
        }
        ~ArmController() override = default;

        void update() override{
            using namespace rmcs_msgs;
            auto sw = *mode_switch_;
            static bool initial_check_done{false};

            if (!initial_check_done) {
                *is_arm_enable_ = false;
                if (sw != VtSwitch::LEFT && sw != VtSwitch::UNKNOWN) {
                    RCLCPP_INFO(rclcpp::get_logger("arm_controller"),"initial!");
                    initial_check_done = true;
                }
                reset();
                return;
            }
            if ((sw == VtSwitch::LEFT || sw == VtSwitch::UNKNOWN)) {
                *is_arm_enable_ = false;
                *arm_mode_ = ArmMode::None;
                reset();
                last_arm_mode_ = ArmMode::None;
                return;
            }
            *is_arm_enable_ = true;
            if(*fn_1_ && !last_fn_1_){
                fn1_toggle_ = !fn1_toggle_;
            }
            if(*fn_2_ && !last_fn_2_){
                fn2_toggle_ = !fn2_toggle_;
            }
            mode_selection();
            if (last_arm_mode_ != *arm_mode_) {
                switch (*arm_mode_) {
                    case ArmMode::Custome: {
                        // 切进角度映射模式时,以当前关节角作为目标起点
                        for (std::size_t i = 0; i < 6; ++i) {
                            if (theta[i].ready() && !std::isnan(*theta[i])) {
                                *target_theta[i] = *theta[i];
                            } else if (std::isnan(*target_theta[i])) {
                                *target_theta[i] = 0.0;
                            }
                        }
                        break;
                    }
                    case ArmMode::execute_vt03_position:
                        for (std::size_t i = 0; i < 6; ++i) {
                            if (theta[i].ready() && !std::isnan(*theta[i])) {
                                *target_theta[i] = *theta[i];
                            } else if (std::isnan(*target_theta[i])) {
                            *target_theta[i] = 0.0;
                            }
                        }
                        break;
                    case ArmMode::execute_vt03_orientation:
                        for (std::size_t i = 0; i < 6; ++i) {
                            if (theta[i].ready() && !std::isnan(*theta[i])) {
                                *target_theta[i] = *theta[i];
                            } else if (std::isnan(*target_theta[i])) {
                            *target_theta[i] = 0.0;
                            }
                        }
                        break;
                    case ArmMode::Gripper:
                        break;
                    case ArmMode::None:
                        reset();
                        break;
                    default:
                        break;
                }
            }
            arm_control();
            last_fn_1_ = *fn_1_;
            last_fn_2_ = *fn_2_;
            last_arm_mode_ = *arm_mode_;
        }

private:
    void mode_selection() {
        auto sw = *mode_switch_;
        using namespace rmcs_msgs;
        if (sw == VtSwitch::MIDDLE && fn1_toggle_ == false) {        
                *arm_mode_ = ArmMode::execute_vt03_position;
        }else if(sw == VtSwitch::MIDDLE && fn1_toggle_ == true) {
                *arm_mode_ = ArmMode::execute_vt03_orientation;
        }else if(sw == VtSwitch::RIGHT && fn2_toggle_ == false) {
                *arm_mode_ = ArmMode::Gripper;
        }else if(sw == VtSwitch::RIGHT && fn2_toggle_ == true) {
                *arm_mode_ = ArmMode::Custome;
        }else{
            *arm_mode_ = ArmMode::None;
        }
    }
    void arm_control(){
        switch(*arm_mode_){
            using namespace rmcs_msgs;
            case ArmMode::Custome: {
                execute_custome();
                break;
            }
            case ArmMode::execute_vt03_position:{
                Vt03_Position_Control();
                break;
            }
            case ArmMode::execute_vt03_orientation:{
                Vt03_Orientation_Control();
                break;
            }
            case ArmMode::Gripper: {
                Gripper_Control();
                break;
        }
            default: {
                break;
            }
        }
    };
    // void execute_plan_request_and_trajectory_step(){
    //     const auto result = action_machine_.get_trajectory();
    //     if (!result || !result->plan_success || result->step_position_map.empty()) {
    //         return;
    //     }

    //     // 收到新的规划 Request ID，重置内部索引
    //     if (result->request_id != last_executed_request_id_) {
    //         current_step_index_       = 0;
    //         step_position_index_      = 0;
    //         last_executed_request_id_ = result->request_id;
    //         current_step_positions_.clear();
    //     }

    //     // 轨迹全部 Step 执行完
    //     if (static_cast<size_t>(current_step_index_) >= result->step_position_map.size()) {
    //         return;
    //     }

    //     // 当前 Step 刚开始执行，加载该 Step 的轨迹点
    //     if (step_position_index_ == 0) {
    //         auto pos_it = result->step_position_map.find(current_step_index_);
    //         if (pos_it != result->step_position_map.end()) {
    //             current_step_positions_ = pos_it->second;
    //         } else {
    //             current_step_positions_.clear();
    //         }

    //         // 边界检查：若当前 Step 无运动轨迹点（如纯夹爪 Step），跳过并进入下一 Step
    //         if (current_step_positions_.empty()) {
    //             current_step_index_++;
    //             step_position_index_ = 0;
    //             return;
    //         }
    //     }

    //     // 防段错误保护
    //     if (step_position_index_ >= current_step_positions_.size()) {
    //         step_position_index_ = 0;
    //         current_step_index_++;
    //         return;
    //     }

    //     // 将轨迹点赋给 target_theta 输出接口
    //     const auto& current_point = current_step_positions_[step_position_index_];
    //     for (size_t i = 0; i < 6 && i < current_point.size(); ++i) {
    //         *target_theta[i] = current_point[i];
    //     }

    //     // 递增点索引
    //     step_position_index_++;

    //     // 推进 Step
    //     if (step_position_index_ >= current_step_positions_.size()) {
    //         step_position_index_ = 0;
    //         current_step_index_++;
    //     }
    // }
    void Gripper_Control(){
        RCLCPP_INFO(rclcpp::get_logger("ArmController"),"Gripper Control Mode Active");
    }
    void execute_custome(){
        RCLCPP_INFO(rclcpp::get_logger("ArmController"),"Custome Control Mode Active");
        // TODO: 角度映射方案待实现
    } 
    void Vt03_Position_Control(){
        constexpr double DEADZONE = 0.05; 
        constexpr double STEP = 0.003;
        for (std::size_t i = 0; i < 6; ++i) {
            if (std::isnan(*target_theta[i])) {
            *target_theta[i] = (theta[i].ready() && !std::isnan(*theta[i])) ? *theta[i] : 0.0;
            }
        }
        auto safe_clamp = [this](std::size_t idx, double target) {
            double lower = (joint_lower_limit_[idx].ready() && !std::isnan(*joint_lower_limit_[idx]))
                           ? *joint_lower_limit_[idx] : -M_PI;
            double upper = (joint_upper_limit_[idx].ready() && !std::isnan(*joint_upper_limit_[idx]))
                           ? *joint_upper_limit_[idx] : M_PI;
            return std::clamp(target, lower, upper);
        };
        if (std::fabs(joystick_left_->x()) > DEADZONE) {
            double next_target = *target_theta[2] - STEP * joystick_left_->x();
            *target_theta[2]   = safe_clamp(2, next_target);
        }
        if (std::fabs(joystick_right_->x()) > DEADZONE) {
            double next_target = *target_theta[1] + STEP * joystick_right_->x();
            *target_theta[1]   = safe_clamp(1, next_target);
        }
        if (std::fabs(joystick_left_->y()) > DEADZONE) {
            double next_target = *target_theta[0] + STEP * joystick_left_->y();
            *target_theta[0]   = safe_clamp(0, next_target);
        }
    }
    void Vt03_Orientation_Control(){
        // RCLCPP_INFO(rclcpp::get_logger("ArmController"),"Orientation Control Mode Active");
        constexpr double DEADZONE = 0.05; 
        constexpr double STEP = 0.003;
        for (std::size_t i = 0; i < 6; ++i) {
            if (std::isnan(*target_theta[i])) {
            *target_theta[i] = (theta[i].ready() && !std::isnan(*theta[i])) ? *theta[i] : 0.0;
            }
        }
        auto safe_clamp = [this](std::size_t idx, double target) {
            double lower = (joint_lower_limit_[idx].ready() && !std::isnan(*joint_lower_limit_[idx]))
                           ? *joint_lower_limit_[idx] : -M_PI;
            double upper = (joint_upper_limit_[idx].ready() && !std::isnan(*joint_upper_limit_[idx]))
                           ? *joint_upper_limit_[idx] : M_PI;
            return std::clamp(target, lower, upper);
        };
        if (std::fabs(joystick_left_->x()) > DEADZONE) {
            double next_target = *target_theta[5] - STEP * joystick_left_->x();
            *target_theta[5]   = safe_clamp(5, next_target);
        }
        if (std::fabs(joystick_right_->x()) > DEADZONE) {
            double next_target = *target_theta[4] + STEP * joystick_right_->x();
            *target_theta[4]   = safe_clamp(4, next_target);
        }
        if (std::fabs(joystick_left_->y()) > DEADZONE) {
            double next_target = *target_theta[3] - STEP * joystick_left_->y();
            *target_theta[3]   = safe_clamp(3, next_target);
        }
    }
    void reset() {
        Eigen::Matrix<double, 6, 1> filter_init_vec;
        for (std::size_t i = 0; i < 6; ++i) {
            *target_theta[i] = theta[i].ready() ? *theta[i] : 0.0;
        }
        custom_joint_filter_.reset();
    }
    // ActionMachine action_machine_;
    rmcs_msgs::ArmMode last_arm_mode_{rmcs_msgs::ArmMode::None};
    bool last_fn_1_{false};
    bool last_fn_2_{false};
    bool fn1_toggle_{false};
    bool fn2_toggle_{false};
    InputInterface<bool> fn_1_;
    InputInterface<bool> fn_2_;
    InputInterface<double> rotary_knob_;
    InputInterface<rmcs_msgs::VtSwitch> mode_switch_;
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    OutputInterface<rmcs_msgs::ArmMode> arm_mode_;
    OutputInterface<bool> is_arm_enable_;
    std::array<InputInterface<double>,6> joint_lower_limit_;
    std::array<InputInterface<double>,6> joint_upper_limit_;
    InputInterface<double> theta[6];
    OutputInterface<double> target_theta[6];
    utility::LowPassFilter<6> custom_joint_filter_;
    };
} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmController, rmcs_executor::Component)
