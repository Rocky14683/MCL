#pragma once

#include <vector>
#include "laser_bean.hpp"
#include "transform2d.hpp"


constexpr double tpi = 180;
constexpr double robot_size = 20;
constexpr double robot_half_size = robot_size / 2;

class Robot {
public:
    struct EncoderInput {
        double left, middle, right;
    };

    Robot(Pose2d pose, std::vector<Pose2d> laser_bean_offsets, std::pair<double, double> noises = {3.f, 3.f})
            : pose(
            std::move(pose)), noises(noises) {
        for (const Pose2d &offset: laser_bean_offsets) {
            this->laser_beans.emplace_back(std::make_shared<LaserModel>(offset, this->pose));
        }
    }


    EncoderInput gen_noisy_encoder(EncoderInput input) {
        input.left += this->gen_gauss_random(0.0, this->noises.first);
        input.middle += this->gen_gauss_random(0.0, this->noises.second);
        input.right += this->gen_gauss_random(0.0, this->noises.first);

        return input;
    }

    Pose2d get_translation_and_update(const EncoderInput& input, bool update = true) {

        static constexpr double left_right_dist = 18.0 / 2;
        static constexpr double middle_dist = 8.0;

        double delta_left, delta_right, delta_middle, delta_angle;

        delta_left = input.left / tpi;
        delta_middle = input.middle / tpi;
        delta_right = input.right / tpi;
        delta_angle = (delta_right - delta_left) / (2 * left_right_dist);

        double local_x;
        double local_y;

        constexpr static double eps = 1e-6;
        if (std::abs(delta_angle) > eps) {
            double i = sin(delta_angle / 2.0) * 2.0;
            local_x = (delta_right + delta_left) / (2 * delta_angle) * i;
            local_y = (delta_middle / delta_angle + middle_dist) * i;
        } else {
            local_x = delta_right;
            local_y = delta_middle;
        }

        double p = this->pose.z() - delta_angle / 2.0; // global angle

        double dx = cos(p) * local_x - sin(p) * local_y;
        double dy = sin(p) * local_x + cos(p) * local_y;



        Pose2d new_pose = {
                this->pose.x() + cos(p) * local_x - sin(p) * local_y,
                this->pose.y() + sin(p) * local_x + cos(p) * local_y,
                this->pose.z() + delta_angle
        };

        if(update) {
            this->update(new_pose);
        }

        return {dx, dy, delta_angle};
    }


    void update(const Pose2d &new_pose) {
        this->pose = new_pose;
        for (const auto &laser: laser_beans) {
            laser->update(pose);
        }
    }

    Pose2d get_pose() const {
        return this->pose;
    }

    void set_pose(const Pose2d &new_pose) {
        this->update(new_pose);
    }

    void draw(std::string_view log_path = "robot") {
        std::vector<rr::Position2D> cornors = {
                rr::Position2D(robot_half_size, robot_half_size),
                rr::Position2D(robot_half_size, -robot_half_size),
                rr::Position2D(-robot_half_size, robot_half_size),
                rr::Position2D(-robot_half_size, -robot_half_size),
        };


        if (DRAW) {
            rr::Position2D center = {static_cast<float>(this->pose.x()), static_cast<float>(this->pose.y())};
            Transform2d transform(this->pose);
            for (auto &cord: cornors) {
                cord = transform * cord;
            }

            rr::Position2D heading = transform * rr::Position2D(robot_half_size, 0);

            std::vector<std::vector<rr::Position2D>> bbox = {
                    {cornors[0], cornors[1]},
                    {cornors[1], cornors[3]},
                    {cornors[3], cornors[2]},
                    {cornors[2], cornors[0]},
            };

            std::vector<std::vector<rr::Position2D>> heading_line = {
                    {center, heading},
            };


            rec.log(std::format("{}/position", log_path),
                    rr::Points2D()
                            .with_positions({center})
                            .with_colors(rr::Color(255, 0, 0))
                            .with_radii(0.1));


            rec.log(std::format("{}/bbox", log_path),
                    rr::LineStrips2D(bbox)
                            .with_colors(rr::Color(0, 255, 0))
                            .with_radii(0.05)
                            .with_labels({std::format("[{:.2f}, {:.2f}]", this->pose.x(), this->pose.y())}));

            rec.log(std::format("{}/heading", log_path),
                    rr::LineStrips2D(heading_line)
                            .with_colors(rr::YELLOW)
                            .with_radii(0.2));


            for (const auto &laser: laser_beans) {
                laser->draw_bean();
            }

        }
    }


    std::vector<double> get_laser_values_clean() {
        std::vector<double> values;
        for (const auto &laser: laser_beans) {
            values.push_back(laser->get_sensor_out_clean());
        }
        return values;
    }

    std::vector<double> get_laser_values_dirty() {
        std::vector<double> values;
        for (const auto &laser: laser_beans) {
            values.push_back(laser->get_sensor_value_dirty());
        }
        return values;
    }


    std::vector<std::shared_ptr<LaserModel>> get_lasers() {
        return this->laser_beans;
    }

private:
    Pose2d pose;
    std::pair<double, double> noises;
    std::vector<std::shared_ptr<LaserModel>> laser_beans{};
    std::random_device rd;
    std::mt19937 gen{rd()};

    inline double gen_gauss_random(double mean, double variance) {
        std::normal_distribution<double> gauss_dist(mean, variance);
        return gauss_dist(gen);
    }
};