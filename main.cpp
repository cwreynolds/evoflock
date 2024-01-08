//------------------------------------------------------------------------------
//
// main.cpp -- new flock experiments
//
//  Created by Craig Reynolds on January 6, 2024.
//  MIT License -- Copyright © 2024 Craig Reynolds
//
//------------------------------------------------------------------------------

#include <iostream>
#include "Vec3.h"

void run_unit_tests()
{
    Vec3::unit_test();
    std::cout << "All unit tests pass." << std::endl;
}

int main(int argc, const char * argv[])
{
    run_unit_tests();
//    {
//        // this takes ~1 second to run in "release" mode.
//        Timer foo_timer("foo");
//        for (int i = 0; i < 460000; i++)
//        {
//            Vec3::unit_test();
//        }
//    }
    return 0;
}
