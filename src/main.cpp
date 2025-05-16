#include <iostream>
#include <libc.h>
#include <chrono>
#include <thread>

#include "api.hpp"
#include "global.hpp"
#include "rlgl.h"
#include "raymath.h"
#include <rerun.hpp>

const int screen_w = 1400;
const int screen_h = 800;



int main() {
    rec.spawn().exit_on_failure();

    draw_wall_static();

    LaserModel l({0, 0, 0}, {0, 0, 0});
    float i = 20;
    Robot r({i, i, M_PI_4}, {{-10, 5, std::numbers::pi}, {10, 5, std::numbers::pi / 2.0}});

    float t = M_PI_4;
    while(true) {
        r.update({i+= 0.0005, i, t += 0.0001});
        r.draw();

//
//        l.draw_bean();
    }
    return 0;
}
