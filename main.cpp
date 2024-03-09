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




void my_new_crossover(const LP::GpTree& parent0,
                      const LP::GpTree& parent1,
                      LP::GpTree& offspring,
                      int min_size,
                      int max_size,
                      int fs_min_size)
{
    offspring = parent0;
    for (int i = 0; i < parent0.subtrees().size(); i++)
    {
        if (LP::LPRS().randomBool())
        {
//            std::cout << std::endl;
//            std::cout << "Index " << i
//                      << " replace " << parent0.getSubtree(i).to_string()
//                      << " with " << parent1.getSubtree(i).to_string()
//                      << std::endl;            
            offspring.getSubtree(i) = parent1.getSubtree(i);
        }
    }
}









int main(int argc, const char * argv[])
{
    run_unit_tests();
    
//    //    util::executions_per_second(Vec3::unit_test);
//
//        Flock flock;
//        flock.set_fixed_time_step(true);
//    //    flock.set_fixed_fps(60);
//        flock.set_fixed_fps(30);
//
//    //    flock.set_max_simulation_steps(1000);
//    //    flock.set_boid_count(200);
//    //    flock.set_boid_count(1000);
//
//    //    flock.set_max_simulation_steps(10);
//    //    flock.set_boid_count(10);
//
//        flock.set_max_simulation_steps(1000);
//        flock.set_boid_count(200);
//
//    //    flock.set_max_simulation_steps(1000);
//    //    flock.set_boid_count(10);
//    //    flock.set_max_simulation_steps(500);
//    //    flock.set_boid_count(8);
//    //    flock.set_boid_count(10);
//
//    //    std::stringstream timer_caption;
//    //    timer_caption << "Basic flock test: " << flock.boid_count() << " boids "
//    //                  << flock.max_simulation_steps() << " frames";
//    //    {
//    //        util::Timer t(timer_caption.str());
//    //        flock.run();
//    //    }

    //--------------------------------------------------------------------------

    
//    LazyPredator::Population population(5, 5, evo_flock_gp_function_set);
//    
//    // Get "an" individual
//    typedef LazyPredator::Individual LPI;
//
//    population.applyToAllIndividuals([&](LPI* i){individual = i;});
//    debugPrint(individual->tree().to_string())
//    debugPrint(individual->tree().isLeaf())
    
//    evoflock_gp_function_set.print();
    
    //--------------------------------------------------------------------------

//    //    int max_tree_size = 5;
//    //    int max_tree_size = 50;
//        int min_tree_size = 2;
//        int max_tree_size = 20;
//        std::string root_type = "Fitness_0_1";
//
//    //    // This seems to suggest LP::FunctionSet::randomFunctionOfTypeInSize() is
//    //    // not returning our one function even when we supply the return type:
//    //    // (This must be "that convention": the first defined type is the root type)
//    //    LP::GpTree tree;
//    //    evoflock_gp_function_set.makeRandomTree(max_tree_size, root_type, tree);
//    //    debugPrint(tree.to_string())
//
//        // This seems to suggest LP::FunctionSet::randomFunctionOfTypeInSize() is
//        // not returning our one function even when we supply the return type:
//        // (This must be "that convention": the first defined type is the root type)
//        LP::GpTree tree1;
//        LP::GpTree tree2;
//        LP::GpTree tree3;
//        LP::GpTree tree4;
//        evoflock_gp_function_set.makeRandomTree(max_tree_size, root_type, tree1);
//        debugPrint(tree1.to_string())
//        evoflock_gp_function_set.makeRandomTree(max_tree_size, root_type, tree2);
//        debugPrint(tree2.to_string())
//
//    //    LP::GpTree::crossover(tree1, tree2, tree3,
//    //                          min_tree_size, max_tree_size,
//    //                          evoflock_gp_function_set.getCrossoverMinSize());
//    //    std::cout << std::endl;
//    //    debugPrint(tree3.to_string())
//
//
//    //    my_new_crossover(tree1, tree2, tree4,
//    //                     min_tree_size, max_tree_size,
//    //                     evoflock_gp_function_set.getCrossoverMinSize());
//    //    std::cout << std::endl;
//    //    debugPrint(tree4.to_string())
//
//
//        evoflock_gp_function_set.setCrossoverFunction(my_new_crossover);
//        auto crossover = evoflock_gp_function_set.getCrossoverFunction();
//        crossover(tree1, tree2, tree4,
//                  min_tree_size, max_tree_size,
//                  evoflock_gp_function_set.getCrossoverMinSize());
//        std::cout << std::endl;
//        debugPrint(tree4.to_string())

    
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20240304
    // Maybe I want to add a new "crossover function hook" to FunctionSet.
    // Then Population::evolutionStep can normally use the default
    //     GpTree::crossover() or, if one is provided, a custom crossover
    //     function from the "hook".
    //
     eg:
    //    static inline std::function<void(const GpTree& parent0,
    //                                     const GpTree& parent1,
    //                                     GpTree& offspring,
    //                                     int min_size,
    //                                     int max_size,
    //                                     int fs_min_size)>
    //        crossover_function_ = nullptr;
    //
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

    
    
    
//    LP::Population population(400, max_tree_size, evoflock_gp_function_set);
//    population.applyToAllIndividuals([&](LP::Individual* i)
//    {
//        if (not i->tree().isLeaf())
//        {
//            std::cout << std::endl;
//            debugPrint(i->tree().to_string())
//        }
//    });
    
    //--------------------------------------------------------------------------
    
//        int individuals = 100;
//        int subpops = 5;
//        int max_evolution_steps = 100;
//        int min_tree_size = 2;
//        int max_tree_size = 20;
//        LazyPredator::Population population(individuals,
//                                            subpops,
//                                            max_tree_size,
//                                            min_tree_size,
//                                            max_tree_size,
//                                            evoflock_gp_function_set);
//        util::Timer t("Evolution run.");
//        for (int i = 0; i < max_evolution_steps; i++)
//        {
//    //        population.evolutionStep([&]
//    //                                 (LazyPredator::TournamentGroup tg)
//    //                                 { return tournamentFunction(tg); });
//            population.evolutionStep([&]
//                                     (LazyPredator::TournamentGroup tg)
//                                     { return tg; });
//        }
    
    //--------------------------------------------------------------------------
    // TODO 20240307 working on fitness function

    // use fitness_function based version of
    // LazyPredator::Population::evolutionStep(FitnessFunction fitness_function)
    
    // Make function with this type (seems like it ought to return a double?)
    // typedef std::function<float(Individual*)> FitnessFunction;

    std::cout << "Creating population" << std::endl;
    int individuals = 200;
    int subpops = 20;
    int max_evolution_steps = 10000;
    int min_tree_size = 2;
    int max_tree_size = 20;
    LazyPredator::Population population(individuals,
                                        subpops,
                                        max_tree_size,
                                        min_tree_size,
                                        max_tree_size,
                                        evoflock_gp_function_set);
    
    std::cout << "Run evolution." << std::endl;
    util::Timer t("Evolution run.");
    for (int i = 0; i < max_evolution_steps; i++)
    {
        population.evolutionStep(evoflock_fitness_function);
        std::cout << std::endl;
    }

    //--------------------------------------------------------------------------

    
    return 0;
}
