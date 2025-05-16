#pragma once

#include <vector>
#include "laser_bean.hpp"
#include "transform2d.hpp"


constexpr double robot_size = 20;

class Robot {
public:
    Robot(Pose2d pose, std::initializer_list<Pose2d> laser_bean_offsets) : pose(std::move(pose)) {
        for (const Pose2d &offset: laser_bean_offsets) {
            this->laser_beans.emplace_back(std::make_shared<LaserModel>(offset, this->pose));
        }
    }


    void update(const Pose2d& new_pose) {
        this->pose = new_pose;
        for (const auto &laser: laser_beans) {
            laser->update(pose);
        }
    }

    void draw() {
        static float robot_half_size = robot_size / 2;
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

            rr::Position2D heading = transform * rr::Position2D(0, robot_half_size);

            std::vector<std::vector<rr::Position2D>> bbox = {
                    {cornors[0], cornors[1]},
                    {cornors[1], cornors[3]},
                    {cornors[3], cornors[2]},
                    {cornors[2], cornors[0]},
            };

            std::vector<std::vector<rr::Position2D>> heading_line = {
                    {center, heading},
            };


            rec.log("robot/position",
                    rr::Points2D()
                            .with_positions({center})
                            .with_colors(rr::Color(255, 0, 0))
                            .with_radii(0.1));


            rec.log("robot/bbox",
                    rr::LineStrips2D(bbox)
                            .with_colors(rr::Color(0, 255, 0))
                            .with_radii(0.05)
                            .with_labels({std::format("[{:.2f}, {:.2f}]", this->pose.x(), this->pose.y())}));

            rec.log("robot/heading",
                    rr::LineStrips2D(heading_line)
                            .with_colors(rr::YELLOW)
                            .with_radii(0.2));


            for (const auto &laser: laser_beans) {
                laser->draw_bean();
            }

        }
    }

private:
    Pose2d pose;
    std::vector<std::shared_ptr<LaserModel>> laser_beans{};
};