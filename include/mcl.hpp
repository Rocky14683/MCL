#pragma once

#include <cmath>
#include <vector>
#include <array>
#include "robot.hpp"


template<size_t N>
class MCL {
public:
    struct Particle {
        Pose2d pose;
        double weight = 1.0;
        std::shared_ptr<Robot> robot;
    };

    MCL(const Pose2d &init_pose, std::vector<Pose2d> laser_offsets, std::function<double()> heading_callback,
        std::array<double, 3> init_noise = {0.0, 0.0, 0.0})
            : laser_offsets(std::move(laser_offsets)), heading_callback(heading_callback) {
        init_particles(init_pose, init_noise);
    }

    void init_particles(const Pose2d &init_pose, const std::array<double, 3> &init_noise) {
        for (int i = 0; i < N; i++) {
            Pose2d new_pose = init_pose;
            particles[i] = Particle{
                    .pose = new_pose,
                    .weight = 1.0,
                    .robot = std::make_shared<Robot>(new_pose, laser_offsets)
            };
        }
    }

    void predict(const Robot::EncoderInput &noisy_input) {
        for (auto &p: particles) {
            auto translation = p.robot->get_translation_and_update(noisy_input, false);
            p.pose += translation;
            clamp_pose(p.pose);
            p.robot->set_pose(p.pose);
        }
    }


    void update(const std::vector<double> &dirty_laser_values) {
        constexpr double stddev = 1.5;
        constexpr double min_likelihood = 1e-6;
        constexpr double max_per_diff = 50.0;
        double total_weight = 0.0;

        for (auto &p: particles) {
            const auto &expected_laser = p.robot->get_laser_values_clean();
            double error = 0.0;

            for (size_t i = 0; i < dirty_laser_values.size(); ++i) {
                double observed = std::clamp(dirty_laser_values[i], 0.0, x_max);
                double expected = expected_laser[i];

                double diff = observed - expected;
                diff = std::clamp(diff, -max_per_diff, max_per_diff);
                error += diff * diff;
            }

            double avg_error = error / dirty_laser_values.size();
            double likelihood = std::exp(-0.5 * avg_error / (stddev * stddev));
            std::println("likelihood: {}", likelihood);
            p.weight = std::max(likelihood, min_likelihood);
            total_weight += p.weight;
        }

        constexpr double eps = 1e-9;
        if (total_weight < eps) {
            std::cerr << "[MCL] Warning: total weight too small, resetting.\n";
            for (auto &p: particles)
                p.weight = 1.0 / N;
        } else {
            for (auto &p: particles)
                p.weight /= total_weight;
        }

        double ess = 0.0;
        for (const auto &p: particles) {
            ess += p.weight * p.weight;
        }
        ess = 1.0 / ess;

        if (ess < N) {
            std::cerr << "call to resample\n";
            resample();
        }

        double x = 0, y = 0, sin_sum = 0, cos_sum = 0;
        for (const auto &p: particles) {
            x += p.weight * p.pose.x();
            y += p.weight * p.pose.y();
            sin_sum += p.weight * std::sin(p.pose.z());
            cos_sum += p.weight * std::cos(p.pose.z());
        }
        this->estimate_pose = Pose2d{x, y, std::atan2(sin_sum, cos_sum)};
    }

    void resample() {
        double linear_noises = 0.3;
        double angular_noises = 0.05;

        std::array<Particle, N> new_particles{};
        std::uniform_real_distribution<> dist(0.0, 1.0 / N);
        double r = dist(gen);
        double c = particles[0].weight;
        size_t i = 0;

        for (size_t m = 0; m < N; ++m) {
            double u = r + static_cast<double>(m) * (1.0 / N);
            while (u > c && i < N - 1) {
                ++i;
                c += particles[i].weight;
            }

            Particle p = particles[i];

            p.pose.x() += gen_gauss_random(0.0, linear_noises);
            p.pose.y() += gen_gauss_random(0.0, linear_noises);
            p.pose.z() = heading_callback();
            clamp_pose(p.pose);

            p.robot->set_pose(p.pose);

            new_particles[m] = p;
        }

        this->particles = new_particles;
    }


    Pose2d get_estimate_pose() const {
        return estimate_pose;
    }

    inline double gen_gauss_random(double mean, double stddev) {
        std::normal_distribution<double> gauss_dist(mean, stddev);
        return gauss_dist(gen);
    }

    void draw() const {
        /*
        for (size_t i = 0; i < N; ++i) {
            const auto &p = particles[i];
            rr::Position2D center = {static_cast<float>(p.pose.x()), static_cast<float>(p.pose.y())};
            Transform2d transform(p.pose);

            rr::Position2D heading = transform * rr::Position2D(1.5, 0);

            std::vector<std::vector<rr::Position2D>> heading_line = {
                    {center, heading},
            };

            rec.log(std::format("particles/{}/pose", i), rr::Points2D()
                    .with_positions({rr::Position2D(p.pose.x(), p.pose.y())})
                    .with_colors(rr::Color(0, 255, 0))
                    .with_radii(p.weight * 3.0));
//
//            rec.log(std::format("particles/{}/heading", i),
//                    rr::LineStrips2D(heading_line)
//                            .with_colors(rr::YELLOW)
//                            .with_radii(0.2));

//            p.robot->draw("particles/" + std::to_string(i));
        }*/

        rec.log("robot/estimate", rr::Points2D()
                .with_positions({{(float) estimate_pose.x(), (float) estimate_pose.y()}})
                .with_colors(rr::Color(255, 0, 255))
                .with_radii(3));
    }

private:

    void clamp_pose(Pose2d &pose) {
        pose.x() = std::clamp(pose.x(), 0.0 + robot_half_size, x_max - robot_half_size);
        pose.y() = std::clamp(pose.y(), 0.0 + robot_half_size, y_max - robot_half_size);
    }


    std::array<Particle, N> particles{};
    std::vector<Pose2d> laser_offsets;
    Pose2d estimate_pose;
    std::function<double()> heading_callback;
    std::random_device rd{};
    std::mt19937 gen{rd()};
};
