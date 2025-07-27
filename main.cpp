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

// Tool for visualizing a logged set of FlockParameters.
// To visualize a given set of FlockParameters. Cut/paste from log, compile.
// Comment out body of this function for normal evolution run.
void visualizePreviouslyLoggedFlockParameters()
{
//    // To use the hand-tuned parameters:
//    // FlockParameters fp;
//    
//    // (These values from run 20250524_test_use_scores_in_flock_data)
//    //FlockParameters fp(90.3889, 20, 20, 20, 75.3918, 41.8466, 27.8009, 0.685528,
//    //                   90.1786, 73.3035, 2.34728, 8.31566, 67.5218, -0.750053,
//    //                   -0.96459, 0.312321, 3.8674, 1.26649);
//    
//    // These values from run 20250601, very nice motion.
//    //FlockParameters fp(55.7424, 20, 20, 20, 88.3095, 59.9705, 28.3211, 28.0211,
//    //                   91.9161, 69.7648, 2.95192, 77.1844, 23.2996, -0.353221,
//    //                   -0.999566, 0.364522, 2.76299, 1.33611);
//    
//    // 20250616b_dv_neighbors_behind_pop_750
//    //FlockParameters fp(83.6605, 20, 20, 20, 99.4266, 39.4691, 25.5936, 14.3034,
//    //                   99.0738, 80.6256, 2.27497, 22.9588, 24.356, -0.115794,
//    //                   0.254552, 0.625807, 2.11305, 0.889898);
//    
//    // from run 20250617_only_1_sim_per_individual
//    //FlockParameters fp(96.076, 20, 20, 20, 85.7334, 57.0328, 24.97, 38.3267,
//    //                   96.3787, 83.4721, 3.15953, 59.1314, 71.415, -0.641807,
//    //                   -0.870298, 0.197666, 2.68647, 1.69024);
//
//    // from run 20250622_test_fix_for_slow_sim
//    //FlockParameters fp(93.4907, 20, 20, 20, 96.1618, 48.9378, 35.8806, 19.816,
//    //                   84.7544, 76.4079, 3.01808, 79.4333, 53.4527, -0.543717,
//    //                   -0.462297, 0.132325, 3.80525, 0.949715);
//
//    // from (disappointing) run 20250704_open_space_flocking
//    //FlockParameters fp(93.9295, 20, 20, 20, 76.095, 21.176, 31.0822, 13.7764,
//    //                   71.8005, 48.7475, 2.90648, 31.4824, 97.1207, -0.600948,
//    //                   -0.94693, -0.998404, 61.8045, 3.5044);
//    
//    // from run 20250709_obs_avoid_linear
//    //FlockParameters fp(82.893, 20, 20, 20, 96.0433, 54.32, 65.2531, 4.05004,
//    //                   85.3227, 36.9236, 2.51169, 9.19635, 25.098, -0.552929,
//    //                   -0.868455, 0.0992426, 28.1231, 1.28869);
//    
//    // from run 20250709_obs_avoid_square
//    //FlockParameters fp(95.0626, 20, 20, 20, 75.7318, 38.9368, 41.1228, 7.78766,
//    //                   96.1189, 27.145, 2.70433, 27.9354, 7.56547, -0.785536,
//    //                   -0.67676, 0.52532, 28.3042, 1.16165);
//    
//    // from run 20250721c_old_falloff_more_log
//    FlockParameters fp(72.9053, 78.2308, 53.0839, 36.1152, 6.51767, 88.0149,
//                       71.9993, 2.38176, 12.0969, 88.8369, -0.823082, -0.675907,
//                       0.532154, 5.18991, 1.19003);
//
//    // from run 20250721c_old_falloff_more_log
//    FlockParameters fp(89.1471, 77.8088, 49.7681, 43.8368, 0.148081, 88.4284,
//                       66.634, 2.55657, 5.65042, 27.2281, -0.92727, -0.53234,
//                       0.378495, 3.94743, 1.04012);
//    
//    EF::enable_multithreading = false;
//    while (true) { GP::run_flock_simulation(fp, 1); }
}


int main(int argc, const char * argv[])
{
    EF::unit_test();
    EF::setRS(LP::LPRS());
    EF::RS().setSeedFromClock();
    std::cout << "RandomSequence seed = " << EF::RS().getSeed() << std::endl;
    
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
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20250724 try 2x longer run just to know what happens

    int individuals = 300;
    int subpops = 17;
    
    int max_evolution_steps = Boid::GP_not_GA ? 20 : 30000;

//    int individuals = 150;
//    int subpops = 12;
//    
//    int max_evolution_steps = Boid::GP_not_GA ? 20 : 30000;
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    int ga_tree_size = 1 + FlockParameters::tunableParameterCount();
    debugPrint(ga_tree_size);
    
    int min_crossover_tree_size = Boid::GP_not_GA ? 20 :  2;
    int max_crossover_tree_size = Boid::GP_not_GA ? 60 : ga_tree_size;
    int max_initial_tree_size   = Boid::GP_not_GA ? 60 : ga_tree_size;

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
        std::cout << "Create population, individuals = " << individuals;
        std::cout << ", subpops/demes = " << subpops << std::endl;
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
            
            if (i == 0)
            {
                LP::GpTree t = individual->tree();
                GP::fp_from_ga_tree(t).print();
                std::cout << std::endl;
                std::cout << std::endl;
            }
            
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
