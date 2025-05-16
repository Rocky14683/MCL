#pragma once

#include <vector>
#include <Eigen/Dense>
#include <raylib.h>
#include <rerun.hpp>

namespace rr = rerun;
inline const auto rec = rr::RecordingStream("MCL");



constinit const int DRAWING_SCALAR = 2;

constinit const bool DRAW = true;
constinit const int WORLD_OFFSET = 72;


using Pose2d = Eigen::Vector3d;
using Point2d = Eigen::Vector2d;

inline const double x_min = 0;
inline const double x_max =  144;
inline const double y_min = 0;
inline const double y_max =  144;


inline const std::array<rr::Position2D, 4> walls = {
        rr::Position2D(x_max, y_min),
        rr::Position2D(x_min, y_min),
        rr::Position2D(x_min, y_max),
        rr::Position2D(x_max, y_max),
};


void draw_wall_static() {
    static const std::vector<std::vector<rr::Position2D>> walls_lines = {
            {walls[0], walls[1]}, {walls[1], walls[2]}, {walls[2], walls[3]}, {walls[3], walls[0]}
    };
    rec.log_static("world/walls", rr::LineStrips2D(walls_lines));
}



