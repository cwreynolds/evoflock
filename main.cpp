//------------------------------------------------------------------------------
//
//  main.cpp -- new flock experiments
//
//  Created by Craig Reynolds on January 6, 2024.
//  MIT License -- Copyright © 2024 Craig Reynolds
//
//------------------------------------------------------------------------------

#define USE_OPEN3D

#include "EvoFlock.h"

int main(int argc, const char * argv[])
{
    EF::unit_test();

    EF::runOneFlockEvolution();
//    EF::runFlockEvolutionLoop();
//    best_fits_histogram();
    
    Draw::deleteInstance();
    return EXIT_SUCCESS;
}
