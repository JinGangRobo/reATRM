#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <bit>
#include <tuple>

#include <rmcs_msgs/robot_color.hpp>

#include "referee/app/ui/shape/shape.hpp"

namespace rmcs_core::referee::app::ui {

class StatusRing {
public:
    StatusRing(
        double supercap_limit, double power_limit, double friction_limit,
        int16_t bullet_speed_limit)
        : supercap_limit_(supercap_limit)
        , power_limit_(power_limit)
        , friction_limit_(friction_limit)
        , bullet_speed_limit_(bullet_speed_limit) {
        supercap_status_.set_x(x_center);
        supercap_status_.set_y(y_center);
        supercap_status_.set_r(visible_radius - width_ring);
        supercap_status_.set_angle_start(275);
        supercap_status_.set_angle_end(275 + visible_angle);
        supercap_status_.set_width(width_ring);
        supercap_status_.set_color(Shape::Color::PINK);
        supercap_status_.set_visible(true);

        power_status_.set_x(x_center);
        power_status_.set_y(y_center);
        power_status_.set_r(visible_radius - width_ring);
        power_status_.set_angle_start(265 - visible_angle);
        power_status_.set_angle_end(265);
        power_status_.set_width(width_ring);
        power_status_.set_color(Shape::Color::PINK);
        power_status_.set_visible(true);

        friction_wheel_speed_.set_x(x_center);
        friction_wheel_speed_.set_y(y_center);
        friction_wheel_speed_.set_r(visible_radius - width_ring);
        friction_wheel_speed_.set_angle_start(85 - visible_angle);
        friction_wheel_speed_.set_angle_end(85);
        friction_wheel_speed_.set_width(width_ring);
        friction_wheel_speed_.set_color(Shape::Color::PINK);
        friction_wheel_speed_.set_visible(true);

        bullet_status_.set_x(x_center);
        bullet_status_.set_y(y_center);
        bullet_status_.set_r(visible_radius - width_ring);
        bullet_status_.set_angle_start(95);
        bullet_status_.set_angle_end(95 + visible_angle);
        bullet_status_.set_width(width_ring);
        bullet_status_.set_color(Shape::Color::PINK);
        bullet_status_.set_visible(true);

        // UI
        arc_left_center_.set_x(x_center);
        arc_left_center_.set_y(y_center);
        arc_left_center_.set_r(visible_radius - width_ring);
        arc_left_center_.set_angle_start(270 - 5);
        arc_left_center_.set_angle_end(270 + 5);
        arc_left_center_.set_width(40);
        arc_left_center_.set_color(Shape::Color::WHITE);
        arc_left_center_.set_visible(true);

        arc_right_center_.set_x(x_center);
        arc_right_center_.set_y(y_center);
        arc_right_center_.set_r(visible_radius - width_ring);
        arc_right_center_.set_angle_start(90 - 5);
        arc_right_center_.set_angle_end(90 + 5);
        arc_right_center_.set_width(40);
        arc_right_center_.set_color(Shape::Color::WHITE);
        arc_right_center_.set_visible(true);

        arc_bullet_safe_.set_x(x_center);
        arc_bullet_safe_.set_y(y_center);
        arc_bullet_safe_.set_r(visible_radius - width_ring);
        arc_bullet_safe_.set_angle_start(95 + calculate_angle(25, 0, bullet_speed_limit));
        arc_bullet_safe_.set_angle_end(95 + calculate_angle(25, 0, bullet_speed_limit) + 1);
        arc_bullet_safe_.set_width(40);
        arc_bullet_safe_.set_color(Shape::Color::WHITE);
        arc_bullet_safe_.set_visible(true);

        arc_power_safe_.set_x(x_center);
        arc_power_safe_.set_y(y_center);
        arc_power_safe_.set_r(visible_radius - width_ring);
        arc_power_safe_.set_width(40);
        arc_power_safe_.set_color(Shape::Color::WHITE);
        arc_power_safe_.set_visible(true);

        arc_left_up_.set_x(x_center);
        arc_left_up_.set_y(y_center);
        arc_left_up_.set_r(visible_radius - width_ring);
        arc_left_up_.set_angle_start(275 + visible_angle + 1);
        arc_left_up_.set_angle_end(275 + visible_angle + 3);
        arc_left_up_.set_width(width_ring + 50);
        arc_left_up_.set_color(Shape::Color::WHITE);
        arc_left_up_.set_visible(true);

        arc_left_down_.set_x(x_center);
        arc_left_down_.set_y(y_center);
        arc_left_down_.set_r(visible_radius - width_ring);
        arc_left_down_.set_angle_start(265 - visible_angle - 3);
        arc_left_down_.set_angle_end(265 - visible_angle - 1);
        arc_left_down_.set_width(width_ring + 50);
        arc_left_down_.set_color(Shape::Color::WHITE);
        arc_left_down_.set_visible(true);

        arc_right_up_.set_x(x_center);
        arc_right_up_.set_y(y_center);
        arc_right_up_.set_r(visible_radius - width_ring);
        arc_right_up_.set_angle_start(85 - visible_angle - 3);
        arc_right_up_.set_angle_end(85 - visible_angle - 1);
        arc_right_up_.set_width(width_ring + 50);
        arc_right_up_.set_color(Shape::Color::WHITE);
        arc_right_up_.set_visible(true);

        arc_right_down_.set_x(x_center);
        arc_right_down_.set_y(y_center);
        arc_right_down_.set_r(visible_radius - width_ring);
        arc_right_down_.set_angle_start(95 + visible_angle + 1);
        arc_right_down_.set_angle_end(95 + visible_angle + 3);
        arc_right_down_.set_width(width_ring + 50);
        arc_right_down_.set_color(Shape::Color::WHITE);
        arc_right_down_.set_visible(true);
    }

    void set_power_safe(double power_safe) {
        power_safe_ = power_safe;
        auto angle = 265 - calculate_angle(power_safe, 0.0, power_limit_) - 1;
        arc_power_safe_.set_angle_start(static_cast<uint16_t>(angle));
        arc_power_safe_.set_angle_end(static_cast<uint16_t>(angle) + 1);
    }

    void set_visible(bool value) {
        // Dynamic
        supercap_status_.set_visible(value);
        power_status_.set_visible(value);
        friction_wheel_speed_.set_visible(value);
        bullet_status_.set_visible(value);

        arc_power_safe_.set_visible(value);

        // Static
        arc_left_center_.set_visible(value);
        arc_right_center_.set_visible(value);
        arc_left_up_.set_visible(value);
        arc_left_down_.set_visible(value);
        arc_right_up_.set_visible(value);
        arc_right_down_.set_visible(value);
    }

    void update_static_parts(std::tuple<bool, bool, bool> enable) {
        auto& [auto_aim_enable, precise_enable, auto_aim_tracking] = enable;
        auto static_enable = auto_aim_enable || precise_enable || auto_aim_tracking;

        static auto color{Shape::Color::WHITE};

        if (auto_aim_enable) {
            color = Shape::Color::ORANGE;
        } else {
            if (precise_enable) {
                color = Shape::Color::CYAN;
            }
            if (auto_aim_tracking) {
                color = Shape::Color::PINK;
            }
        }
        if (!static_enable) {
            color = Shape::Color::WHITE;
        }

        update_static_enable(static_enable, color);
    }

    void update_static_enable(bool enable, Shape::Color color) {
        static bool enable_last_{false};

        arc_left_up_.set_color(color);
        arc_left_down_.set_color(color);
        arc_right_up_.set_color(color);
        arc_right_down_.set_color(color);
        if (enable == enable_last_)
            return;

        enable_last_ = enable;
    }

    void update_supercap(double value, bool enable) {
        if (!enable) {
            supercap_status_.set_angle_end(static_cast<uint16_t>(275 + visible_angle));
            supercap_status_.set_color(Shape::Color::WHITE);
            return;
        }

        auto angle = 275 + calculate_angle(value, 10.5, supercap_limit_) + 1;
        supercap_status_.set_angle_end(static_cast<uint16_t>(angle));

        if (value > 75) {
            supercap_status_.set_color(Shape::Color::GREEN);
        } else if (value > 35) {
            supercap_status_.set_color(Shape::Color::ORANGE);
        } else {
            supercap_status_.set_color(Shape::Color::PINK);
        }
    }

    void update_power(double value, bool enable) {
        if (!enable) {
            power_status_.set_angle_start(static_cast<uint16_t>(265 - visible_angle));
            power_status_.set_color(Shape::Color::WHITE);
            return;
        }
        auto angle = 265 - calculate_angle(value, 0.0, power_limit_) - 1;
        power_status_.set_angle_start(static_cast<uint16_t>(angle));

        if (value > power_safe_) {
            power_status_.set_color(Shape::Color::PINK);
        } else {
            power_status_.set_color(Shape::Color::GREEN);
        }
    }

    void update_friction_wheel_speed(double value, bool enable) {
        auto angle = 85 - calculate_angle(value, 0, friction_limit_) - 1;
        friction_wheel_speed_.set_angle_start(static_cast<uint16_t>(angle));

        if (enable) {
            friction_wheel_speed_.set_color(Shape::Color::GREEN);
        } else {
            friction_wheel_speed_.set_color(Shape::Color::PINK);
        }
    }

    void update_bullet_speed(uint16_t value) {
        auto bullet_speed = std::bit_cast<int16_t>(value);

        // limit ring
        auto angle = 95 + calculate_angle(bullet_speed, 0, bullet_speed_limit_) + 1;
        bullet_status_.set_angle_end(static_cast<uint16_t>(angle));

        if (bullet_speed > 25) {
            bullet_status_.set_color(Shape::Color::PINK);
        } else if (bullet_speed < 18) {
            bullet_status_.set_color(Shape::Color::ORANGE);
        } else {
            bullet_status_.set_color(Shape::Color::GREEN);
        }
    }

private:
    static constexpr double calculate_angle(double value, double min, double max) {
        return visible_angle * std::clamp(value - min, 0.0, max - min) / (max - min);
    }

    constexpr static uint16_t x_center = 960;
    constexpr static uint16_t y_center = 540;
    constexpr static uint16_t width_ring = 15;
    constexpr static uint16_t visible_radius = 400;
    constexpr static uint16_t visible_angle = 40;

    double supercap_limit_;
    double power_limit_;
    double friction_limit_;
    double bullet_speed_limit_;
    double power_safe_;

    // Dynamic part
    Arc supercap_status_;
    Arc power_status_;
    Arc friction_wheel_speed_;
    Arc bullet_status_;

    Arc arc_power_safe_;

    // Static part
    Arc arc_left_center_;
    Arc arc_right_center_;
    Arc arc_left_up_;
    Arc arc_left_down_;
    Arc arc_right_up_;
    Arc arc_right_down_;
    Arc arc_bullet_safe_;
};

} // namespace rmcs_core::referee::app::ui