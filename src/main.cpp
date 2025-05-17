#include <iostream>
#include <libc.h>
#include <chrono>
#include <thread>

#include "api.hpp"
#include "global.hpp"
#include "rlgl.h"
#include "raymath.h"
#include <rerun.hpp>
#include <unordered_set>
#include <mutex>
#include <fcntl.h>
#include <termios.h>
#include <map>
#include <GLFW/glfw3.h>
#include <ncurses.h>


int main() {
    rec.spawn().exit_on_failure();

    draw_wall_static();

    float i = 20;
    std::vector<Pose2d> laser_offsets = {{-10, 5, std::numbers::pi}, {10, 5, 0}, {-5, -10, std::numbers::pi / 2.0 * 3.0},
                                          {5, 10, -std::numbers::pi / 2.0 * 3.0}};
    Robot ground_truth({i, i, 0}, laser_offsets);
    MCL<500> mcl({i, i, 0}, laser_offsets, [&ground_truth]() {
        return ground_truth.get_pose().z();
    });
    Robot::EncoderInput input{.left = 0.3, .middle = 0, .right = 0.7};
    Robot odometry({i, i, 0}, {{}});
    while(true) {
        ground_truth.get_translation_and_update(input, true);
        ground_truth.draw();

        // dirty sensor datas
        auto dirty_laser = ground_truth.get_laser_values_dirty();
        auto clean_laser = ground_truth.get_laser_values_clean();
//        for(int c = 0; c < dirty_laser.size(); c++) {
//            std::println("Expected: {}, Actual: {}", clean_laser[c], dirty_laser[c]);
//        }

        auto noisy_encoder_value = ground_truth.gen_noisy_encoder(input);

        // update pure odom
        odometry.get_translation_and_update(noisy_encoder_value, true);
        odometry.draw("odom");

        // update MCL
        mcl.predict(noisy_encoder_value);
        mcl.update(dirty_laser);


        mcl.draw();

//
//
//        std::cout << "Noisy encoder value: " << noisy_encoder_value.left << ", " << noisy_encoder_value.middle << ", "
//                  << noisy_encoder_value.right << "\n";
    }
    return 0;
}
