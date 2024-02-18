//------------------------------------------------------------------------------
//
// main.cpp -- new flock experiments
//
//  Created by Craig Reynolds on January 6, 2024.
//  MIT License -- Copyright © 2024 Craig Reynolds
//
//------------------------------------------------------------------------------

#include "Agent.h"
#include "Boid.h"
#include "flock.h"
#include "LocalSpace.h"
#include "obstacle.h"
#include "shape.h"
#include "Utilities.h"
#include "Vec3.h"
#include <iostream>

void run_unit_tests()
{
    util::unit_test();
    Vec3::unit_test();
    LocalSpace::unit_test();
    shape::unit_test();
    Obstacle::unit_test();
    Agent::unit_test();
    Boid::unit_test();
    Flock::unit_test();
    std::cout << "All unit tests OK." << std::endl;
}

int main(int argc, const char * argv[])
{
    run_unit_tests();
    
//    util::executions_per_second(Vec3::unit_test);
    
    Flock flock;
    flock.set_fixed_time_step(true);
//    flock.set_fixed_fps(60);
    flock.set_fixed_fps(30);

//    flock.set_max_simulation_steps(1000);
//    flock.set_boid_count(200);
//    flock.set_boid_count(1000);
    
//    flock.set_max_simulation_steps(10);
//    flock.set_boid_count(10);

    flock.set_max_simulation_steps(1000);
    flock.set_boid_count(200);

//    flock.set_max_simulation_steps(1000);
//    flock.set_boid_count(10);
//    flock.set_max_simulation_steps(500);
//    flock.set_boid_count(8);
//    flock.set_boid_count(10);

    std::stringstream timer_caption;
    timer_caption << "Basic flock test: " << flock.boid_count() << " boids "
                  << flock.max_simulation_steps() << " frames";
    {
        util::Timer t(timer_caption.str());
        flock.run();
    }

    return 0;
}
