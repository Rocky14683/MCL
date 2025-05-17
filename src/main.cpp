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

    LaserModel l({0, 0, 0}, {0, 0, 0});
    float i = 20;
    Robot r({i, i, 0}, {{-10, 5, std::numbers::pi}, {10, 5, std::numbers::pi / 2.0}});
    MCL<100> mcl({i, i, 0}, {{-10, 5, std::numbers::pi}, {10, 5, std::numbers::pi / 2.0}}, [&r] -> double{
        return r.get_pose().z();
    });
    Robot::EncoderInput input{.left = 0.0001, .middle = 0, .right = 0.0002};
    while(true) {
        auto lasers = r.get_laser_values();
        r.update(input);
        r.draw();

        mcl.predict(input);
        mcl.update(lasers);
        mcl.resample();

        mcl.draw();
//
//        l.draw_bean();
    }
    return 0;
}
