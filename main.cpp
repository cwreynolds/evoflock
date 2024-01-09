//------------------------------------------------------------------------------
//
// main.cpp -- new flock experiments
//
//  Created by Craig Reynolds on January 6, 2024.
//  MIT License -- Copyright © 2024 Craig Reynolds
//
//------------------------------------------------------------------------------

#include "Vec3.h"
#include "Utilities.h"
#include <iostream>

void run_unit_tests()
{
    Vec3::unit_test();
    std::cout << "All unit tests pass." << std::endl;
}

int main(int argc, const char * argv[])
{
    run_unit_tests();
//    executions_per_second(Vec3::unit_test);
    return 0;
}
