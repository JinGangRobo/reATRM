#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <rmcs_msgs/robot_color.hpp>

#include "referee/app/ui/shape/shape.hpp"

namespace rmcs_core::referee::app::ui {
class StatusBar {
public:
    StatusBar(double cool_limit, double bullet_limit)
        : cool_limit_(cool_limit)
        , bullet_limit_(bullet_limit) {
        line_bar_cool_.set_x(x_center - 150);
        line_bar_cool_.set_y(y_center + 300);
        line_bar_cool_.set_x2(x_center - 150);
        line_bar_cool_.set_y2(y_center + 300);
        line_bar_cool_.set_width(10);
        line_bar_cool_.set_color(Shape::Color::WHITE);

        line_bar_bullet_.set_x(x_center - 150);
        line_bar_bullet_.set_y(y_center - 360);
        line_bar_bullet_.set_x2(x_center + 150);
        line_bar_bullet_.set_y2(y_center - 360);
        line_bar_bullet_.set_width(10);
        line_bar_bullet_.set_color(Shape::Color::CYAN);

        integer_bullet_count_ten_.set_x(x_center - 40);
        integer_bullet_count_ten_.set_y(y_center - 285);
        integer_bullet_count_ten_.set_color(Shape::Color::CYAN);
        integer_bullet_count_ten_.set_font_size(50);
        integer_bullet_count_ten_.set_width(4);

        integer_bullet_count_one_.set_x(x_center + 10);
        integer_bullet_count_one_.set_y(y_center - 285);
        integer_bullet_count_one_.set_color(Shape::Color::CYAN);
        integer_bullet_count_one_.set_font_size(50);
        integer_bullet_count_one_.set_width(4);

        for (int i = 0; i < 2; ++i) {
            line_bar_bullet_background_[i].set_x(x_center - 160 + i * 320);
            line_bar_bullet_background_[i].set_y(y_center - 360 - 5);
            line_bar_bullet_background_[i].set_x2(x_center - 160 + i * 320);
            line_bar_bullet_background_[i].set_y2(y_center - 360 + 5);
            line_bar_bullet_background_[i].set_width(10);
            line_bar_bullet_background_[i].set_color(Shape::Color::WHITE);
        }
        for (int i = 0; i < 2; ++i) {
            line_bar_cool_background_[i].set_x(x_center - 160 + i * 320);
            line_bar_cool_background_[i].set_y(y_center + 300 - 5);
            line_bar_cool_background_[i].set_x2(x_center - 160 + i * 320);
            line_bar_cool_background_[i].set_y2(y_center + 300 + 5);
            line_bar_cool_background_[i].set_width(10);
            line_bar_cool_background_[i].set_color(Shape::Color::WHITE);
        }

        text_desc_power_.set_x(x_center - 150);
        text_desc_power_.set_y(y_center - 380);
        text_desc_power_.set_color(Shape::Color::WHITE);
        text_desc_power_.set_font_size(16);
        text_desc_power_.set_width(2);
        text_desc_power_.set_is_text_shape(true);
        text_desc_power_.set_value("C_PWR          /");

        text_desc_center_status_.set_x(x_center - 155);
        text_desc_center_status_.set_y(y_center - 420);
        text_desc_center_status_.set_color(Shape::Color::WHITE);
        text_desc_center_status_.set_font_size(16);
        text_desc_center_status_.set_width(2);
        text_desc_center_status_.set_is_text_shape(true);
        text_desc_center_status_.set_value("CENTER_AREA_STATUS");

        text_desc_cool_.set_x(x_center - 50);
        text_desc_cool_.set_y(y_center + 335);
        text_desc_cool_.set_color(Shape::Color::WHITE);
        text_desc_cool_.set_font_size(16);
        text_desc_cool_.set_width(2);
        text_desc_cool_.set_is_text_shape(true);
        text_desc_cool_.set_value("COOLING");

        integer_chassis_power_.set_x(x_center + 40);
        integer_chassis_power_.set_y(y_center - 380);
        integer_chassis_power_.set_color(Shape::Color::WHITE);
        integer_chassis_power_.set_font_size(16);
        integer_chassis_power_.set_width(2);
        integer_chassis_power_.set_value(999);

        integer_power_limit_.set_x(x_center + 110);
        integer_power_limit_.set_y(y_center - 380);
        integer_power_limit_.set_color(Shape::Color::WHITE);
        integer_power_limit_.set_font_size(16);
        integer_power_limit_.set_width(2);
        integer_power_limit_.set_value(999);
    }

    void set_cool_limit(double cool_limit) { cool_limit_ = cool_limit; }

    void set_visible(bool visible) {
        // Dynamic
        line_bar_cool_visible_ = visible;
        line_bar_bullet_.set_visible(visible);
        integer_bullet_count_ten_.set_visible(visible);
        integer_bullet_count_one_.set_visible(visible);
        integer_chassis_power_.set_visible(visible);
        integer_power_limit_.set_visible(visible);
        text_desc_center_status_.set_visible(visible);

        // Static
        for (int i = 0; i < 2; ++i) {
            line_bar_bullet_background_[i].set_visible(visible);
        }
        text_desc_power_.set_visible(visible);
    }

    void update_dynamic_part(double cool, int16_t bullet, uint8_t center_status) {
        if (bullet < 0) {
            bullet = 0;
        }
        line_bar_cool_.set_x2(x_center - 150 + std::clamp(cool / cool_limit_, 0.0, 1.0) * 300);
        line_bar_bullet_.set_x2(
            x_center - 150 + std::clamp(bullet / bullet_limit_, 0.0, 1.0) * 300);

        if (cool >= cool_limit_) {
            line_bar_cool_.set_color(Shape::Color::WHITE);
            update_cool_visible(false);
        } else if (cool >= cool_limit_ * 0.8) {
            line_bar_cool_.set_color(Shape::Color::CYAN);
            update_cool_visible(true);
        } else if (cool >= cool_limit_ * 0.5) {
            line_bar_cool_.set_color(Shape::Color::YELLOW);
            update_cool_visible(true);
        } else {
            line_bar_cool_.set_color(Shape::Color::PINK);
            update_cool_visible(true);
        }

        integer_bullet_count_ten_.set_value(bullet > 99 ? 9 : ((bullet - (bullet % 10)) / 10));
        integer_bullet_count_one_.set_value(bullet > 99 ? 9 : bullet % 10);
        switch (center_status) {
        case 0: text_desc_center_status_.set_color(Shape::Color::WHITE); break;
        case 1: text_desc_center_status_.set_color(Shape::Color::GREEN); break;
        case 2: text_desc_center_status_.set_color(Shape::Color::PURPLE); break;
        default: break;
        }
    }

    void update_power_part(int16_t chassis_power, int16_t power_limit, bool assist) {
        if (chassis_power < 0) {
            chassis_power = 0;
        }
        integer_chassis_power_.set_value(chassis_power);
        integer_power_limit_.set_value(power_limit);
        integer_chassis_power_.set_color(assist ? Shape::Color::ORANGE : Shape::Color::WHITE);
        integer_power_limit_.set_color(assist ? Shape::Color::ORANGE : Shape::Color::WHITE);
    }

    void update_cool_visible(bool visible_required) {
        if (line_bar_cool_visible_) {
            line_bar_cool_.set_visible(visible_required);
            text_desc_cool_.set_visible(visible_required);
            for (int i = 0; i < 2; ++i) {
                line_bar_cool_background_[i].set_visible(visible_required);
            }
        } else {
            line_bar_cool_.set_visible(false);
            text_desc_cool_.set_visible(false);
            for (int i = 0; i < 2; ++i) {
                line_bar_cool_background_[i].set_visible(false);
            }
        }
    }

private:
    double cool_limit_;
    double bullet_limit_;

    bool line_bar_cool_visible_ = false;

    // Dynamic
    Line line_bar_cool_;
    Line line_bar_bullet_;
    Integer integer_bullet_count_ten_;
    Integer integer_bullet_count_one_;
    Integer integer_chassis_power_;
    Integer integer_power_limit_;
    Text text_desc_center_status_;

    // Static
    Line line_bar_bullet_background_[2];
    Line line_bar_cool_background_[2];
    Text text_desc_power_;
    Text text_desc_cool_;

    constexpr static uint16_t x_center = 960;
    constexpr static uint16_t y_center = 540;
};
}; // namespace rmcs_core::referee::app::ui