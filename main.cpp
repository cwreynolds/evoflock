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
#include "GP.h"
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
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240224 very preliminary test
    LazyPredator::unit_test();
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    std::cout << "All unit tests OK." << std::endl;
}


int main(int argc, const char * argv[])
{
    run_unit_tests();

    //--------------------------------------------------------------------------
    
//        int individuals = 500;
//        int subpops = 20;
//    //    int max_evolution_steps = 20000;  // TODO 20240311 3000 may be enough
//    //                                      // based on last evening's run
//        int max_evolution_steps = 3000;
//        int min_tree_size = 2;
//        int max_tree_size = 20;
    
    int individuals = 500;
//    int subpops = 20;
//    int subpops = 10;
    int subpops = 20;
//    int max_evolution_steps = 3000;
    int max_evolution_steps = 20000;
    int min_tree_size = 2;
    int max_tree_size = 20;

    std::cout << "Create population." << std::endl;
    util::Timer t("Create population.");
    LazyPredator::Population population(individuals,
                                        subpops,
                                        max_tree_size,
                                        min_tree_size,
                                        max_tree_size,
                                        evoflock_gp_function_set);
    
    {
        std::cout << "Run evolution." << std::endl;
        util::Timer t("Run evolution.");
        for (int i = 0; i < max_evolution_steps; i++)
        {
            population.evolutionStep(evoflock_fitness_function);
            std::cout << std::endl;
        }
    }
    
    std::cout << std::endl;
    std::cout << std::endl;
    for (int i = 0; i < 10; i++)
    {
        const LP::Individual* individual = population.nthBestFitness(i);
        std::cout << individual->tree().to_string() << std::endl;
        double fitness = rerun_flock_simulation(individual);
        debugPrint(fitness);
    }

    //--------------------------------------------------------------------------
        
//    double fitness = run_hand_tuned_flock_simulation(true);
//    debugPrint(fitness);

    //--------------------------------------------------------------------------
    
    return 0;
}
