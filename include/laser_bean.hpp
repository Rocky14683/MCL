#pragma once

#include <random>
#include <utility>
#include <raylib.h>
#include "transform2d.hpp"


class LaserModel {
public:
    LaserModel(const Pose2d &offset, const Pose2d &robot_pose, double noise = 0.1) : offset(offset),
                                                                                     noise(noise) {
        this->transform(robot_pose);
        this->id = id_counter++;
    }


    void update(const Pose2d &robot_pose) {
        this->transform(robot_pose);
    }


    void transform(const Pose2d &robot_pose) {
        Transform2d transform(robot_pose);
        auto offset_transformed = transform * offset;
        this->global_pose.x() = offset_transformed.x();
        this->global_pose.y() = offset_transformed.y();
        this->global_pose.z() = robot_pose.z() + offset.z();
        this->global_pose.z() = norm(this->global_pose.z());
    }


    double get_sensor_value() const {
        return this->sensor_value;
    }

    double get_sensor_out_t() const {
        double heading = global_pose.z();
        double dx = std::cos(norm(heading));
        double dy = std::sin(norm(heading));
        std::cout << "Laser at (" << global_pose.x() << ", " << global_pose.y()
                  << ") heading: " << heading << " rad\n";

        double t_min = std::numeric_limits<double>::infinity();

        // check x = 0 wall
        if (std::abs(dx) > 1e-6) {
            double t = (x_min - global_pose.x()) / dx;
            double y_hit = global_pose.y() + t * dy;
            if (t > 0 && y_hit >= y_min && y_hit <= y_max) {
                t_min = std::min(t_min, t);
            }
        }

        // check x = 144 wall
        if (std::abs(dx) > 1e-6) {
            double t = (x_max - global_pose.x()) / dx;
            double y_hit = global_pose.y() + t * dy;
            if (t > 0 && y_hit >= y_min && y_hit <= y_max) {
                t_min = std::min(t_min, t);
            }
        }

        // check y = 0 wall
        if (std::abs(dy) > 1e-6) {
            double t = (y_min - global_pose.y()) / dy;
            double x_hit = global_pose.x() + t * dx;
            if (t > 0 && x_hit >= x_min && x_hit <= x_max) {
                t_min = std::min(t_min, t);
            }
        }

        // check y = 144 wall
        if (std::abs(dy) > 1e-6) {
            double t = (y_max - global_pose.y()) / dy;
            double x_hit = global_pose.x() + t * dx;
            if (t > 0 && x_hit >= x_min && x_hit <= x_max) {
                t_min = std::min(t_min, t);
            }
        }

        if (t_min == std::numeric_limits<double>::infinity()) {
            std::cerr << "[LaserModel] Warning: No valid wall hit detected.\n";
        }

        return t_min;
    }

    void draw_bean() {
        static constexpr double sensor_size = 2;
        static constexpr double sensor_half_size = sensor_size / 2;
        std::vector<rr::Position2D> cornors = {
                rr::Position2D(sensor_half_size, sensor_half_size),
                rr::Position2D(sensor_half_size, -sensor_half_size),
                rr::Position2D(-sensor_half_size, sensor_half_size),
                rr::Position2D(-sensor_half_size, -sensor_half_size),
        };


        if (DRAW) {
            std::string log_path = std::format("laser_bean_{}", this->id);

            auto sensor_out = this->update_sensor_value();

            std::vector<std::vector<rr::Position2D>> laser_line = {
                    {rr::Position2D(static_cast<float>(this->global_pose.x()),
                                    static_cast<float>(this->global_pose.y())),
                     rr::Position2D(static_cast<float>((global_pose.x() + cos(global_pose.z()) * sensor_out)),
                                    static_cast<float>((global_pose.y() + sin(global_pose.z()) * sensor_out)))}
            };

            rec.log(log_path + "/laser", rr::LineStrips2D(laser_line)
                    .with_colors({0xFF0000FF}));


            rr::Position2D center = {static_cast<float>(this->global_pose.x()),
                                     static_cast<float>(this->global_pose.y())};
            Transform2d transform(this->global_pose);
            for (auto &cord: cornors) {
                cord = transform * cord;
            }

            std::vector<std::vector<rr::Position2D>> sensor_bbox = {
                    {cornors[0], cornors[1]},
                    {cornors[1], cornors[3]},
                    {cornors[3], cornors[2]},
                    {cornors[2], cornors[0]},
            };


            rec.log(log_path + "/sensor",
                    rr::LineStrips2D(sensor_bbox)
                            .with_radii(0.05)
                            .with_colors({0xFAFAFAFF})
                            .with_labels(
                                    {std::format("[{:.2f}, {:.2f}]", this->global_pose.x(), this->global_pose.y())}));

        }
    }

private:
    // laser position relative to the world
    Pose2d global_pose;
    // laser pose relative to the robot
    Pose2d offset;
    double noise;
    double sensor_value;
    std::random_device rd;
    std::mt19937 gen{rd()};
    size_t id = 0;

    double update_sensor_value() {
        double expected_dist = get_sensor_out_t();
        double err = this->gen_gauss_random(0.0, this->noise);
        this->sensor_value = expected_dist + err;
        return sensor_value;
    }

public:
    static size_t id_counter;

    inline double gen_gauss_random(double mean, double variance) {
        std::normal_distribution<double> gauss_dist(mean, variance);
        return gauss_dist(gen);
    }
};

size_t LaserModel::id_counter = 0;