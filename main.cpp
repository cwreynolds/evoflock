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


    
int best_fits_histogram()
{
    std::vector<double> bf = {
        0.922738,
        0.898911,
        0.936639,
        0.90417,
        0.913077,
        0.908212,
        0.924382,
        0.897925,
        0.912716,
        0.61509,
        0.904096,
        0.905901,
        0.609566,
        0.916224,
        0.906028,
        0.896384,
        0.924659,
        0.75599,
        0.824636,
        0.844356,
        0.923861,
        0.936629,
        0.622445,
        0.883624,
        0.87372,
        0.901277,
        0.880275,
        0.908504,
        0.903157,
        0.869309,
        0.911308,
        0.731087,
        0.816817,
        0.911903,
        0.902742,
        0.838345,
        0.909979,
        0.664561,
        0.872564,
        0.857419,
        0.901737,
        0.889453,
        0.888716,
        0.829721,
        0.888786,
        0.90817,
        0.733538,
        0.892828,
        0.909466,
        0.838204,
        0.900107,
        0.842347,
        0.895116,
        0.757397,
        0.896084,
    };
    debugPrint(bf.size());
    std::ranges::sort(bf);
    
//        double step = 0.02;
//        double bot = 0.60;
//        double top = bot + step;
//        while (top < 0.92)

    double min = 0.60;
    double max = 0.94;

    double step = 0.02;
//        double bot = 0.60;
    double bot = min;
    double top = bot + step;
//        while (top < 0.92)
    while (top <= max)
    {
        std::cout << bot << ",";
        std::vector<double> here;
        for (auto x : bf) { if (util::between(x, bot, top)) { here.push_back(x); } }
//        std::cout << here.size() << "," << util::vec_to_string(here) << std::endl;
        std::cout << here.size() << std::endl;
        bot += step;
        top += step;
    }
//    return EXIT_SUCCESS;
    exit(EXIT_SUCCESS) ;
}

int main(int argc, const char * argv[])
{
//    best_fits_histogram();
    
    EF::unit_test();

    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    // TODO 20260831 temp code to record data in SimsPerFit1vs4.csv
    // EF::runOneFlockEvolution();
    EF::runFlockEvolutionLoop();
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

    Draw::deleteInstance();
    return EXIT_SUCCESS;
}
