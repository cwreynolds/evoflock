//------------------------------------------------------------------------------
//
//  main.cpp -- new flock experiments
//
//  Created by Craig Reynolds on January 6, 2024.
//  MIT License -- Copyright © 2024 Craig Reynolds
//
//------------------------------------------------------------------------------

#define USE_OPEN3D

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240628 can we do an eval of a const tree?
//#define eval_const_20240628
#ifdef eval_const_20240628
#else  // eval_const_20240628
#endif // eval_const_20240628
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


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
