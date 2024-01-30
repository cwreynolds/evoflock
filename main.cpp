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
    std::cout << "All unit tests OK." << std::endl;
}

int main(int argc, const char * argv[])
{
    run_unit_tests();
//    util::executions_per_second(Vec3::unit_test);
    return 0;
}
