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

//    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
//    // TODO 20251210 test emphasizeHighScores(), unit test?, reuse for sep score
//    
//    for (double emphasis = 0; emphasis <= 1; emphasis += 0.1)
//    {
//        debugPrint(emphasis);
//        for (double score = 0; score <= 1; score += 0.1)
//        {
//            double s2 = Flock::emphasizeHighScores(score, emphasis);
//            std::cout << std::format("{:.2}→{:.2}, ", score, s2);
//        }
//        std::cout << std::endl << std::endl;
//    }
//    exit(EXIT_SUCCESS);
//    
//    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~

    EF::runOneFlockEvolution();
//    EF::runFlockEvolutionLoop();
//    best_fits_histogram();
    
    Draw::deleteInstance();
    return EXIT_SUCCESS;
}
