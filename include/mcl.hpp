#pragma once

#include <array>
#include <vector>
#include <memory>
#include <random>
#include <cmath>
#include <algorithm>
#include <format>
#include <functional>
#include "math.hpp"
#include "robot.hpp"

template<size_t N>
class MCL {
public:
    struct Particle {
        Pose2d pose;
        double weight = 1.0;
        std::shared_ptr<Robot> robot;
    };

    MCL(const Pose2d &init_pose,
        std::initializer_list<Pose2d> laser_offsets,
        std::function<double()> angle_func)
            : laser_offsets(laser_offsets), angle_func(std::move(angle_func)) {
        init(init_pose);
    }

    void init(const Pose2d &init_pose) {
        for (size_t i = 0; i < N; ++i) {
            Pose2d noisy = init_pose;
            noisy.x() += gen_gauss(0, 5);
            noisy.y() += gen_gauss(0, 5);
            noisy.z() = angle_func();  // external heading
            clamp_pose(noisy);

            Particle p;
            p.pose = noisy;
            p.robot = std::make_shared<Robot>(noisy, laser_offsets);
            particles[i] = p;
        }

        prediction = init_pose;
        prediction.z() = angle_func();
    }

    void predict(const Robot::EncoderInput &input) {
        for (auto &p : particles) {
            auto noisy_input = p.robot->gen_noisy_encoder(input);
            p.robot->update(noisy_input);
            p.pose = p.robot->get_pose();
            p.pose.z() = angle_func();  // fix heading
            clamp_pose(p.pose);
            p.robot->set_pose(p.pose);
        }
    }

    void update(const std::vector<double> &observed_lasers) {
        constexpr double max_per_beam_error = 100.0;
        constexpr double error_scale = 0.05;

        double total_weight = 0.0;

        for (auto &p : particles) {
            auto &robot = p.robot;
            std::vector<double> expected;

            bool valid = true;
            for (const auto &laser : robot->get_lasers()) {
                double sim = laser->get_sensor_value();
                if (!std::isfinite(sim) || sim <= 0.0 || sim > max_sensor_range) {
                    valid = false;
                    break;
                }
                expected.push_back(std::clamp(sim, 0.0, max_sensor_range));
            }

            if (!valid || expected.size() != observed_lasers.size()) {
                p.weight = 0.0;
                continue;
            }

            double error = 0.0;
            for (size_t i = 0; i < expected.size(); ++i) {
                double obs = std::clamp(observed_lasers[i], 0.0, max_sensor_range);
                double diff = obs - expected[i];
                error += std::min(diff * diff, max_per_beam_error);
            }

            double scaled = error / expected.size();
            p.weight = std::exp(-scaled * error_scale);
            total_weight += p.weight;
        }

        if (total_weight < 1e-9) {
            for (auto &p : particles) p.weight = 1.0 / N;
        } else {
            for (auto &p : particles) p.weight /= total_weight;
        }
    }

    void resample() {
        std::array<Particle, N> new_particles;
        std::uniform_real_distribution<> dist(0.0, 1.0 / N);
        double r = dist(gen);
        double c = particles[0].weight;
        size_t i = 0;

        float x_sum = 0, y_sum = 0;

        for (size_t m = 0; m < N; ++m) {
            double u = r + m * (1.0 / N);
            while (u > c && i < N - 1) {
                ++i;
                c += particles[i].weight;
            }

            new_particles[m] = particles[i];
            new_particles[m].pose.x() += gen_gauss(0.0, 0.2);
            new_particles[m].pose.y() += gen_gauss(0.0, 0.2);
            new_particles[m].pose.z() = angle_func();  // re-sync heading
            clamp_pose(new_particles[m].pose);
            new_particles[m].robot->set_pose(new_particles[m].pose);

            x_sum += new_particles[m].pose.x();
            y_sum += new_particles[m].pose.y();
        }

        particles = new_particles;
        prediction = Pose2d{x_sum / N, y_sum / N, angle_func()};
    }

    Pose2d get_estimate() const {
        return prediction;
    }

    void draw() const {
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

            rec.log(std::format("particles/{}/heading", i),
                    rr::LineStrips2D(heading_line)
                            .with_colors(rr::YELLOW)
                            .with_radii(0.2));
        }

        rec.log("robot/estimate", rr::Points2D()
                .with_positions({rr::Position2D(prediction.x(), prediction.y())})
                .with_colors(rr::Color(255, 0, 255))
                .with_radii(3));


    }

private:
    std::array<Particle, N> particles;
    std::initializer_list<Pose2d> laser_offsets;
    std::function<double()> angle_func;
    Pose2d prediction;

    double max_sensor_range = 144.0;
    std::mt19937 gen{std::random_device{}()};

    double gen_gauss(double mean, double stddev) {
        std::normal_distribution<> d(mean, stddev);
        return d(gen);
    }

    void clamp_pose(Pose2d &pose) {
        pose.x() = std::clamp(pose.x(), 0.0, max_sensor_range);
        pose.y() = std::clamp(pose.y(), 0.0, max_sensor_range);
    }
};
