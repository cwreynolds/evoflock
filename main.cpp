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

#include "evoflock.h"

// Handler for GUI's B key command to visualize "best" Individual.
void visualizeBestIfRequested(LP::Population* population)
{
    Draw& draw = Draw::getInstance();
    if (draw.getVisBestMode())
    {
        bool previous_emt_state = EF::enable_multithreading;
        EF::enable_multithreading = false;
        LP::Individual* individual = population->bestFitness();
        LP::GpTree tree = individual->tree();
        FlockParameters fp = GP::fp_from_ga_tree(tree);
        GP::run_flock_simulation(fp, 1);
        EF::enable_multithreading = previous_emt_state;
        draw.clearVisBestMode();
    }
}

// WIP tool for visualizing a logged FlockParameters, eg:
//     Run_Flock(87.4777, 20, 20, 20, 84.9358, 98.4413, 26.6924, 1.90821,
//               53.2896, 94.8003, 2.77889, 37.8503, 42.3571, -0.787644,
//               -0.401251, 0.575284, 9.87024, 8.99761)
void visualizePreviouslyLoggedFlockParameters()
{
//    // To use the hand-tuned parameters:
//    // FlockParameters fp;
//    // To visualize a given set of FlockParameters. Cut/paste from log, compile.
//    // (These values from run 20250524_test_use_scores_in_flock_data)
//    FlockParameters fp(90.3889, 20, 20, 20, 75.3918, 41.8466, 27.8009, 0.685528,
//                       90.1786, 73.3035, 2.34728, 8.31566, 67.5218, -0.750053,
//                       -0.96459, 0.312321, 3.8674, 1.26649);
//    EF::enable_multithreading = false;
//    while (true) { GP::run_flock_simulation(fp, 1); }
}


int main(int argc, const char * argv[])
{
    EF::setRS(LP::LPRS());
    EF::unit_test();
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20250510 temp global switch for controlling speed with fitness.
    //
    // I added this global switch to help with switching over to a regime where
    // there are no kinematic constraints on boid speed. Instead a target speed
    // range becomes one of the input parameters to a flock evolution run. A new
    // fitness objective is added which gives a "point" for every boid-step when
    // its speed is in the desired range.
    
    EF::fitness_speed_control = true;
//    EF::fitness_speed_control = false;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    // TODO experimental_GP_stub
    Boid::GP_not_GA = false;

    // Enable multiprocessing (run 4 Flock simulations in parallel, process
    // Flock's boids in parallel).
    EF::enable_multithreading = true;

    // But first, here in the main thread, build (then delete) one Flock object
    // to set up static state, such as defining Obstacle sets, making one active,
    // then uploading it to GPU.
    //
    // 20250426 When I tried removing this, the Open3D window showed the default
    // Obstacle set plus "Set 0: sphere and right hand vertical cylinder."
    {
        Flock flock;
    }

    // WIP/HACK runs flock sim, with graphics, for the FlockParameters written
    // inline in this function's source code, above.
    visualizePreviouslyLoggedFlockParameters();
    
    int individuals = 500;
    int subpops = 25;
    int max_evolution_steps = Boid::GP_not_GA ? 20 : 30000;

    int ga_tree_size = 1 + FlockParameters::tunableParameterCount();
    debugPrint(ga_tree_size);
    
    int min_crossover_tree_size = Boid::GP_not_GA ? 20 :  2;
    int max_crossover_tree_size = Boid::GP_not_GA ? 60 : ga_tree_size;
    int max_initial_tree_size   = Boid::GP_not_GA ? 60 : ga_tree_size;

    LP::LPRS().setSeed(20240722);

    // TODO experimental_GP_stub
    auto fitness_function = (Boid::GP_not_GA ?
                             GP::evoflock_gp_fitness_function :
                             GP::evoflock_ga_fitness_function);
    
    LP::Population* population = nullptr;

    // TODO experimental_GP_stub
    LP::FunctionSet fs = (Boid::GP_not_GA ?
                          GP::test_gp_boid_function_set() :
                          GP::evoflock_ga_function_set());
    fs.print();

    {
        std::cout << "Create population." << std::endl;
        util::Timer t("Create population.");
        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
        // TODO 20240713 experiment with increasing initial tree size.
        LP::Individual::increasing_initial_tree_size = true;
        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
        population = new LazyPredator::Population(individuals,
                                                  subpops,
                                                  max_initial_tree_size,
                                                  min_crossover_tree_size,
                                                  max_crossover_tree_size,
                                                  fs);
        // TODO experimental_GP_stub
        if (Boid::GP_not_GA)
        {
            population->explicit_treeValue_in_evolutionStep = false;
        }
        else
        {
            fs.setCrossoverFunction(GP::evoflock_ga_crossover);
        }
    }

    {
        std::cout << "Run evolution." << std::endl;
        util::Timer t("Run evolution.");
        
        for (int i = 0; i < max_evolution_steps; i++)
        {
            // Exit if user interactively exits run.
            if (Draw::getInstance().exitFromRun()) { break; }
            GP::save_fitness_time_series(*population);
            population->evolutionStep(fitness_function, GP::scalarize_fitness);
            if ((population->getStepCount() % 100) == 0)
            {
                LP::Individual* individual = population->bestFitness();
                std::cout << individual->tree().to_string(true) << std::endl;
            }
            std::cout << std::endl;
            visualizeBestIfRequested(population);
        }
    }
    
    // Save end of run data.
    auto record_top_10 = [&]()
    {
        std::cout << std::endl;
        std::cout << std::endl;
        for (int i = 0; i < 10; i++)
        {
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20240703 fix end of run logging for GP
            // TODO experimental_GP_stub
//            const LP::Individual* individual = population->nthBestFitness(i);
            LP::Individual* individual = population->nthBestFitness(i);
//            std::cout << individual->tree().to_string() << std::endl;
            std::cout << individual->tree().to_string(true) << std::endl;
//            auto fitness = GP::rerun_flock_simulation(individual);
            LazyPredator::MultiObjectiveFitness fitness;
            if (Boid::GP_not_GA)
            {
                fitness = GP::evoflock_gp_fitness_function(individual);
            }
            else
            {
                fitness = GP::rerun_flock_simulation(individual);
            }
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            debugPrint(fitness);
        }
    };
    
    if (not Draw::getInstance().exitFromRun()) { record_top_10(); }
    delete population;
    LP::Individual::leakCheck();
    delete &Draw::getInstance();
    return EXIT_SUCCESS;
}
