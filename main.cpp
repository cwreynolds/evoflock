//
//  main.cpp
//  evoflock
//
//  Created by Craig Reynolds on January 6, 2024.
//

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
    return 0;
}
