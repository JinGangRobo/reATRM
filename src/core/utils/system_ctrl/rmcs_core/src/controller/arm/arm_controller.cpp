#include <boost/mpl/assert.hpp>
#include <limits>
#include <rclcpp/logger.hpp>
#define IKFAST_HAS_LIBRARY
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/util/Meta.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <string>
#include <vector>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/crc/dji_crc.hpp>
#include <rmcs_utility/package_receive.hpp>
#include <rmcs_utility/tick_timer.hpp>
#include "utility/low_pass_filter.hpp"
#include <rmcs_msgs/arm_mode.hpp>
#include <rmcs_msgs/switch.hpp>
#include "controller/arm/IKfast/ikfast.h"

namespace rmcs_core::controller::arm {

class ArmController final
    : public rmcs_executor::Component
    , public rclcpp::Node {    
public:
        ArmController()
        : Node(
                get_component_name(),
                rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) 
        , custom_joint_filter_(0.2){
            register_input("/remote/switch/right", switch_right_);
            register_input("/remote/switch/left", switch_left_);
            register_input("/remote/joystick/right", joystick_right_);
            register_input("/remote/joystick/left", joystick_left_);
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
            auto switch_right = *switch_right_;
            auto switch_left  = *switch_left_;
            using namespace rmcs_msgs;
            static bool initial_check_done{false};

            if (!initial_check_done) {
                *is_arm_enable_ = false;
                if (switch_left == Switch::DOWN && switch_right == Switch::DOWN) {
                    RCLCPP_INFO(rclcpp::get_logger("test"),"initial!");
                    initial_check_done = true;
                }
                reset();
                return;
            }
            if ((switch_left == Switch::DOWN || switch_right == Switch::DOWN)|| switch_left == Switch::UNKNOWN) {
                *is_arm_enable_ = false;
                *arm_mode_ = ArmMode::None;
                reset();
                last_arm_mode_ = ArmMode::None;
                return;
            } 
            *is_arm_enable_ = true;
            mode_selection();
            if (last_arm_mode_ != *arm_mode_) {
                switch (*arm_mode_) {
                    case ArmMode::Custome: {
                        std::array<IkReal, 6> current_angles;
                        for (std::size_t i = 0; i < 6; ++i) {
                            if (theta[i].ready() && !std::isnan(*theta[i])) {
                                *target_theta[i] = *theta[i];
                                current_angles[i] = *theta[i];
                            } else if (std::isnan(*target_theta[i])) {
                            *target_theta[i] = 0.0;
                            }
                        }
                        ComputeFk(current_angles.data(), target_eetrans_.data(), target_eerot_.data());
                        RCLCPP_INFO(
                        rclcpp::get_logger("ArmController"),
                        "FK Initialized TCP target to: [%.3f, %.3f, %.3f]",
                        target_eetrans_[0], target_eetrans_[1], target_eetrans_[2]);
                        custom_joint_filter_.reset();
                        break;
                    }
                    case ArmMode::execute_dr16_position:
                        for (std::size_t i = 0; i < 6; ++i) {
                            if (theta[i].ready() && !std::isnan(*theta[i])) {
                                *target_theta[i] = *theta[i];
                            } else if (std::isnan(*target_theta[i])) {
                            *target_theta[i] = 0.0;
                            }
                        }
                        break;
                    case ArmMode::None:
                        reset();
                        break;
                    default:
                        break;
                }
            }
            arm_control();
            last_switch_left_ = switch_left;
            last_switch_right_ = switch_right;
            last_arm_mode_ = *arm_mode_;
        }

private:
    double NormAngle(double angle){
        double a = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (a < 0.0)
        {
            a += 2.0 * M_PI;
        }
        return a - M_PI;
        }
        bool isValidJoint(const std::vector<IkReal>& solution) {
        if (solution.size() < 6) return false;
        for (std::size_t i = 0; i < 6; ++i) {
            if (!joint_lower_limit_[i].ready() || !joint_upper_limit_[i].ready()){ 
                //RCLCPP_WARN(rclcpp::get_logger("IK"), "Joint Limit Interface NOT READY!");
                continue;}
            double lower = *joint_lower_limit_[i];
            double upper = *joint_upper_limit_[i];
        if (std::isnan(lower) || std::isnan(upper)) continue;
        double norm_sol = NormAngle(solution[i]); 
        if (norm_sol < lower || norm_sol > upper) {
            //RCLCPP_WARN(rclcpp::get_logger("IK"), "Joint %zu limit breached! Sol: %.2f(Norm: %.2f) , Limit: [%.2f, %.2f]",i + 1, solution[i], norm_sol, lower, upper);
            return false;
        }
        }
        return true;
    }
    void mode_selection() {
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        using namespace rmcs_msgs;
        if (switch_left == Switch::MIDDLE && switch_right == Switch::MIDDLE) {        
                *arm_mode_ = ArmMode::Custome;
        }else if(switch_left == Switch::UP && switch_right == Switch::UP) {
                *arm_mode_ = ArmMode::execute_dr16_position;
        }else if(switch_left == Switch::UP && switch_right == Switch::MIDDLE) {
                *arm_mode_ = ArmMode::execute_dr16_orientation;
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
            case ArmMode::execute_dr16_position:{
                Dr16_Position_Control();
                break;
            }
            case ArmMode::execute_dr16_orientation:{
                Dr16_Orientation_Control();
                break;
            }
            default: {
                break;
            }
        }
    };

    void execute_custome(){
        //todo: image_transformission Initialize the TCP position and orientation, 
        // and then calculate the IkFast to get the target joint angles, 
        // and then filter the joint angles and output them

        ikfast::IkSolutionList<IkReal> solutions;

        // for(auto& v:eetrans){
        //     if(v==0.0)
        //         v=0.01;
        // }
        bool IkSuccess = ComputeIk(target_eetrans_.data(), target_eerot_.data(), nullptr, solutions);
        // bool IkSuccess = ComputeIk(eetrans, eerot, nullptr, solutions);
        if(!IkSuccess){
            RCLCPP_WARN(rclcpp::get_logger("IK"),"IK Failed");
            for (std::size_t j = 0; j < 6; ++j) {
                *target_theta[j] = theta[j].ready() ? *theta[j] : 0.0;
            }
            return ;
        }else {
            std::vector<IkReal>sol_values(GetNumJoints());
            std::vector<IkReal>best_sol;
            double min_error = std::numeric_limits<double>::max();
            for(std::size_t i = 0;i<solutions.GetNumSolutions();++i ){
                const auto& sol =solutions.GetSolution(i);
                std::vector<IkReal> free(sol.GetFree().size());
                sol.GetSolution(sol_values.data(), free.empty() ? nullptr : free.data());
                double error = 0.0;
                for(std::size_t j = 0;j<sol_values.size();++j){
                    double current_urdf_angle = theta[j].ready() ? *theta[j] : 0.0;
                    double e = NormAngle(sol_values[j] - current_urdf_angle);
                    error += e*e;
                }
                if(error < min_error && isValidJoint(sol_values)) {
                    //RCLCPP_INFO(rclcpp::get_logger("IK"),"IK Success");
                    min_error = error;
                    best_sol = sol_values;
                }
            }
            if(!best_sol.empty()){
                Eigen::Matrix<double, 6, 1> raw_target_theta;
                for(std::size_t i = 0;i < 6;++i){
                double base_angle = !std::isnan(*target_theta[i])
                                    ? *target_theta[i]
                                    : (theta[i].ready() ? *theta[i] : 0.0);
                    double delta_urdf = NormAngle(best_sol[i] - base_angle);
                    raw_target_theta[i] = base_angle + delta_urdf;
                }
                Eigen::Matrix<double, 6, 1> filtered_target = custom_joint_filter_.update(raw_target_theta);
                for (std::size_t j = 0; j < 6; ++j) {
                    *target_theta[j] = filtered_target[j];
                }
            }
        }
    } 
    void Dr16_Position_Control(){
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
    void Dr16_Orientation_Control(){
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
        // RCLCPP_INFO(rclcpp::get_logger("DR16_Position"),"%lf ,%lf, %lf",joystick_left_->x(),joystick_right_->x(),joystick_left_->y());
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
    rmcs_msgs::Switch last_switch_left_{rmcs_msgs::Switch::UNKNOWN};
    rmcs_msgs::Switch last_switch_right_{rmcs_msgs::Switch::UNKNOWN};
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    rmcs_msgs::ArmMode last_arm_mode_{rmcs_msgs::ArmMode::None};
    OutputInterface<rmcs_msgs::ArmMode> arm_mode_;
    OutputInterface<bool> is_arm_enable_;
    std::array<InputInterface<double>,6> joint_lower_limit_;
    std::array<InputInterface<double>,6> joint_upper_limit_;
    InputInterface<double> theta[6];
    OutputInterface<double> target_theta[6];
    utility::LowPassFilter<6> custom_joint_filter_;
    std::array<IkReal, 3> target_eetrans_;
    std::array<IkReal, 9> target_eerot_;
    };
} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmController, rmcs_executor::Component)