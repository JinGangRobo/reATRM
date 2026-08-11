#pragma once

#include <fast_tf/fast_tf.hpp>

#include <fast_tf/impl/joint.hpp>
#include <fast_tf/impl/joint_collection.hpp>
#include <fast_tf/impl/link.hpp>

namespace rmcs_description {

struct BaseLink : fast_tf::Link<BaseLink> {
    static constexpr char name[] = "base_link";
};

struct YawLink : fast_tf::Link<YawLink> {
    static constexpr char name[] = "yaw_link";
};
struct PitchLink : fast_tf::Link<PitchLink> {
    static constexpr char name[] = "pitch_link";
};

struct MuzzleLink : fast_tf::Link<MuzzleLink> {
    static constexpr char name[] = "muzzle_link";
};

struct CameraLink : fast_tf::Link<CameraLink> {
    static constexpr char name[] = "camera_link";
};

struct ViewerLink : fast_tf::Link<ViewerLink> {
    static constexpr char name[] = "viewer_link";
};

struct TransmitterLink : fast_tf::Link<TransmitterLink> {
    static constexpr char name[] = "transmitter_link";
};

struct OdomImu : fast_tf::Link<OdomImu> {
    static constexpr char name[] = "odom_imu";
};

struct RawImu : fast_tf::Link<RawImu> {
    static constexpr char name[] = "raw_imu";
};

struct GimbalCenterLink : fast_tf::Link<GimbalCenterLink> {
    static constexpr char name[] = "gimbal_center_link";
};
struct LeftFrontWheelLink : fast_tf::Link<LeftFrontWheelLink> {
    static constexpr char name[] = "left_front_wheel_link";
};
struct LeftBackWheelLink : fast_tf::Link<LeftFrontWheelLink> {
    static constexpr char name[] = "left_back_wheel_link";
};
struct RightBackWheelLink : fast_tf::Link<LeftFrontWheelLink> {
    static constexpr char name[] = "right_back_wheel_link";
};
struct RightFrontWheelLink : fast_tf::Link<LeftFrontWheelLink> {
    static constexpr char name[] = "right_front_wheel_link";
};

struct OmniLinkLeftFront : fast_tf::Link<OmniLinkLeftFront> {
    static constexpr char name[] = "omni_link_left_front";
};

struct OmniLinkRightFront : fast_tf::Link<OmniLinkRightFront> {
    static constexpr char name[] = "omni_link_right_front";
};

struct OmniLinkLeft : fast_tf::Link<OmniLinkLeft> {
    static constexpr char name[] = "omni_link_left";
};

struct OmniLinkRight : fast_tf::Link<OmniLinkRight> {
    static constexpr char name[] = "omni_link_right";
};

struct ArmBaseLink : fast_tf::Link<ArmBaseLink> {  
    static constexpr char name[] = "arm_base_link";  
};  
struct ArmLink1 : fast_tf::Link<ArmLink1> {  
    static constexpr char name[] = "arm_link1";  
};  
struct ArmLink2 : fast_tf::Link<ArmLink2> {  
    static constexpr char name[] = "arm_link2";  
};  
struct ArmLink3 : fast_tf::Link<ArmLink3> {  
    static constexpr char name[] = "arm_link3";  
};  
struct ArmLink4 : fast_tf::Link<ArmLink4> {  
    static constexpr char name[] = "arm_link4";  
};  
struct ArmLink5 : fast_tf::Link<ArmLink5> {  
    static constexpr char name[] = "arm_link5";  
};  
struct ArmLink6 : fast_tf::Link<ArmLink6> {  
    static constexpr char name[] = "arm_link6";  
};  
struct ArmEndLink : fast_tf::Link<ArmEndLink> {  
    static constexpr char name[] = "arm_end_link";  
};
} // namespace rmcs_description

template <>
struct fast_tf::Joint<rmcs_description::GimbalCenterLink> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::BaseLink;
    Eigen::Translation3d transform = Eigen::Translation3d::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::RawImu> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::BaseLink;
    Eigen::Quaterniond transform = Eigen::Quaterniond::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::YawLink> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::GimbalCenterLink;

    void set_state(double angle) { angle_ = angle; }
    auto get_transform() const { return Eigen::AngleAxisd{angle_, Eigen::Vector3d::UnitZ()}; }

private:
    double angle_;
};

template <>
struct fast_tf::Joint<rmcs_description::PitchLink> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::YawLink;

    void set_state(double angle) { angle_ = angle; }
    auto get_transform() const { return Eigen::AngleAxisd{angle_, Eigen::Vector3d::UnitY()}; }

private:
    double angle_;
};

template <>
struct fast_tf::Joint<rmcs_description::MuzzleLink> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::PitchLink;
    Eigen::Translation3d transform = Eigen::Translation3d::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::TransmitterLink> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::PitchLink;
    Eigen::Translation3d transform = Eigen::Translation3d::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::CameraLink> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::PitchLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::OdomImu> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::PitchLink;
    Eigen::Quaterniond transform = Eigen::Quaterniond::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::ViewerLink> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::PitchLink;

    void set_state(double angle) { angle_ = angle; }
    auto get_transform() const { return Eigen::AngleAxisd{angle_, Eigen::Vector3d::UnitY()}; };

private:
    double angle_;
};
template <>
struct fast_tf::Joint<rmcs_description::LeftFrontWheelLink> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{std::numbers::pi / 4, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

template <>
struct fast_tf::Joint<rmcs_description::LeftBackWheelLink> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{std::numbers::pi / 4 * 3, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

template <>
struct fast_tf::Joint<rmcs_description::RightBackWheelLink> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{-std::numbers::pi / 4 * 3, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

template <>
struct fast_tf::Joint<rmcs_description::RightFrontWheelLink> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{-std::numbers::pi / 4, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

// J1: ArmBaseLink → ArmLink1  
template <>  
struct fast_tf::Joint<rmcs_description::ArmLink1> : fast_tf::ModificationTrackable {  
    using Parent = rmcs_description::ArmBaseLink;  
    Eigen::Isometry3d transform = []() {  
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();  
        t.translation() = Eigen::Vector3d{0.0, 0.0, 0.08465};  
        return t;  
    }();  
    void set_state(double angle) {  
        transform.linear() = Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitZ()}.matrix();  
    }  
};  
  
// J2: ArmLink1 → ArmLink2  
template <>  
struct fast_tf::Joint<rmcs_description::ArmLink2> : fast_tf::ModificationTrackable {  
    using Parent = rmcs_description::ArmLink1;  
    Eigen::Isometry3d transform = []() {  
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();  
        t.translation() = Eigen::Vector3d{0.020084, 0.031625, 0.05555};  
        return t;  
    }();  
    void set_state(double angle) {  
        static const Eigen::Matrix3d R_origin =  
            Eigen::AngleAxisd{-M_PI / 2, Eigen::Vector3d::UnitX()}.matrix();  
        transform.linear() = R_origin * Eigen::AngleAxisd{-angle, Eigen::Vector3d::UnitZ()}.matrix();  
    }  
};  
  
// J3: ArmLink2 → ArmLink3  
template <>  
struct fast_tf::Joint<rmcs_description::ArmLink3> : fast_tf::ModificationTrackable {  
    using Parent = rmcs_description::ArmLink2;  
    Eigen::Isometry3d transform = []() {  
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();  
        t.translation() = Eigen::Vector3d{-0.264, 0.0, 0.0};  
        return t;  
    }();  
    void set_state(double angle) {  
        transform.linear() = Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitZ()}.matrix();  
    }  
};  
  
// J4: ArmLink3 → ArmLink4  
template <>  
struct fast_tf::Joint<rmcs_description::ArmLink4> : fast_tf::ModificationTrackable {  
    using Parent = rmcs_description::ArmLink3;  
    Eigen::Isometry3d transform = []() {  
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();  
        t.translation() = Eigen::Vector3d{0.2426, -0.054, -0.001625};  
        return t;  
    }();  
    void set_state(double angle) {   
        transform.linear() = Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()}.matrix();  
    }  
};  
  
// J5: ArmLink4 → ArmLink5  
template <>  
struct fast_tf::Joint<rmcs_description::ArmLink5> : fast_tf::ModificationTrackable {  
    using Parent = rmcs_description::ArmLink4;  
    Eigen::Isometry3d transform = []() {  
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();  
        t.translation() = Eigen::Vector3d{0.078308, -0.0375, -0.03};  
        return t;  
    }();  
    void set_state(double angle) {  
        static const Eigen::Matrix3d R_origin =  
            Eigen::AngleAxisd{-M_PI / 2, Eigen::Vector3d::UnitX()}.matrix();  
        // TODO: 确认 axis 方向，若 URDF 是 "0 0 -1" 则改为 -angle  
        transform.linear() = R_origin * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitZ()}.matrix();  
    }  
};  
  
// J6: ArmLink5 → ArmLink6  
template <>  
struct fast_tf::Joint<rmcs_description::ArmLink6> : fast_tf::ModificationTrackable {  
    using Parent = rmcs_description::ArmLink5;  
    Eigen::Isometry3d transform = []() {  
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();  
        t.translation() = Eigen::Vector3d{0.028008, 0.0, 0.04};  
        return t;  
    }();  
    void set_state(double angle) {  
        static const Eigen::Matrix3d R_origin =  
            Eigen::AngleAxisd{M_PI / 2, Eigen::Vector3d::UnitY()}.matrix();  
        transform.linear() = R_origin * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitZ()}.matrix();  
    }  
};  
  
// End effector: ArmLink6 → ArmEndLink 
template <>  
struct fast_tf::Joint<rmcs_description::ArmEndLink> : fast_tf::ModificationTrackable {  
    using Parent = rmcs_description::ArmLink6;  
    Eigen::Isometry3d transform = []() {  
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();  
        t.translation() = Eigen::Vector3d{0.0, 0.0, 0.15539};  
        t.linear() = (Eigen::AngleAxisd{3.1415, Eigen::Vector3d::UnitZ()}  
                      * Eigen::AngleAxisd{-M_PI / 2, Eigen::Vector3d::UnitY()}).matrix();  
        return t;  
    }();  
};



namespace rmcs_description {

using Tf = fast_tf::JointCollection<
    GimbalCenterLink, YawLink, PitchLink, MuzzleLink, TransmitterLink, CameraLink, OdomImu, RawImu,
    LeftFrontWheelLink, LeftBackWheelLink, RightBackWheelLink, RightFrontWheelLink, ViewerLink>;

using InfantryTf = fast_tf::JointCollection<
    GimbalCenterLink, YawLink, PitchLink, MuzzleLink, TransmitterLink, CameraLink, OdomImu,
    LeftFrontWheelLink, LeftBackWheelLink, RightBackWheelLink, RightFrontWheelLink>;

using HeroTf = fast_tf::JointCollection<
    GimbalCenterLink, YawLink, PitchLink, MuzzleLink, TransmitterLink, CameraLink, OdomImu,
    LeftFrontWheelLink, LeftBackWheelLink, RightBackWheelLink, RightFrontWheelLink, ViewerLink>;

using AutoAimTf = fast_tf::JointCollection<MuzzleLink, TransmitterLink, CameraLink, OdomImu>;

using ArmTf = fast_tf::JointCollection<ArmLink1, ArmLink2, ArmLink3, ArmLink4, ArmLink5, ArmLink6, ArmEndLink>;
} // namespace rmcs_description
