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

    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    // TODO 20260831 temp code to record data in SimsPerFit1vs4.csv
    // EF::runOneFlockEvolution();
    EF::runFlockEvolutionLoop();
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

    Draw::deleteInstance();
    return EXIT_SUCCESS;
}
