#pragma once

#include <Eigen/Dense>

struct Transform2d {
    Eigen::Vector3d pose;

    Transform2d(Eigen::Vector3d pose) : pose(std::move(pose)) {}

    Transform2d(double x, double y, double theta) : pose(x, y, theta) {}

    Transform2d(const Eigen::Vector2d &position, double theta) : pose(position.x(), position.y(), theta) {}

    double x() const { return pose.x(); }

    double y() const { return pose.y(); }

    double theta() const { return pose.z(); }

    double &x() { return pose.x(); }

    double &y() { return pose.y(); }

    double &theta() { return pose.z(); }

    Transform2d operator*(const Transform2d &other) const {
        Eigen::Vector3d traslated = {
                this->x() + other.x() * cos(this->theta()) - other.y() * sin(this->theta()),
                this->y() + other.x() * sin(this->theta()) + other.y() * cos(this->theta()),
                this->theta() + other.theta()
        };
        return traslated;
    }

    rr::Position2D operator*(const rr::Position2D &other) const {
        auto x = this->x() + other.x() * cos(this->theta()) - other.y() * sin(this->theta());
        auto y = this->y() + other.x() * sin(this->theta()) + other.y() * cos(this->theta());
        return {static_cast<float>(x), static_cast<float>(y)};
    }

    Point2d operator*(const Point2d &other) const {
        auto x = this->x() + other.x() * cos(this->theta()) - other.y() * sin(this->theta());
        auto y = this->y() + other.x() * sin(this->theta()) + other.y() * cos(this->theta());
        return {x, y};
    }

    Point2d operator*(const Pose2d &other) const {
        auto x = this->x() + other.x() * cos(this->theta()) - other.y() * sin(this->theta());
        auto y = this->y() + other.x() * sin(this->theta()) + other.y() * cos(this->theta());
        return {x, y};
    }

};