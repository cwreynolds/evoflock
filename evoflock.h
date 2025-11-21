//------------------------------------------------------------------------------
//
//  EvoFlock.h -- new flock experiments
//
//  Top level header file for evoflock project
//
//  Main entry points:
//    EF::runOneFlockEvolution()
//    EF::runFlockEvolutionLoop()
//    EF::setUsingGP()
//    EF::setUsingGA()
//    EF::usingGP()
//    EF::usingGA()
//    EF::RS()
//
//  Created by Craig Reynolds on September 3, 2024.
//  MIT License -- Copyright © 2024 Craig Reynolds
//------------------------------------------------------------------------------

#pragma once
#include <string>
#include <fstream>

// Working around a semi-circular definition chain: include Utilities.h for the
// RandomSequence class, which is used globally within EvoFlock.
#include "Utilities.h"


// Then define accessors for EvoFlock's random number generator: EF::RS().
// Also define a handful of global EvoFlock state variables.
namespace EvoFlock
{
// Accessors for global RS (RandomSequence) for all of EvoFlock
inline static RandomSequence rs_default_;
inline static RandomSequence* rs_ = &rs_default_;
inline void setRS(RandomSequence& rs) { rs_ = &rs; }
inline RandomSequence& RS(){ return *rs_; }

// Global switch to enable/disable multithreading.
inline static bool enable_multithreading = true;

// Controls roll (rotation around forward axis) blend rate for Boid and Camera.
inline static double roll_rate = 0.99;

// Global switch (temp?) enables 4th objective component for boosting curvature.
inline static bool add_curvature_objective = false;

static inline bool using_GA_ = true;
inline bool usingGP() { return not using_GA_; }
inline bool usingGA() { return using_GA_; }
inline void setUsingGP() { using_GA_ = false; }
inline void setUsingGA() { using_GA_ = true; }

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20251105 why did GP steering force get so small?

// Is the steering value returned by a GP-mode tree expressed in local or global
// space? Originally it was global, as in GA-mode. Then I decided local might be
// "easier" for evolution so unconditionally changed the code. Now I have the
// mysterious "boid are attracted to one quadrant(/octant?)" bug. I'd like to
// see if going back to global mode makes that go away. Maybe its a bug in the
// LS::localize/globalizeDirection() functions?
//static inline bool gp_tree_returns_local = true;
static inline bool gp_tree_returns_local = false;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

}  // end of namespace EvoFlock
namespace EF = EvoFlock;


// Include all the other EvoFlock headers.
#include "Agent.h"
#include "Boid.h"
#include "dbscan.h"
#include "Draw.h"
#include "flock.h"
#include "GP.h"
#include "LazyPredator/LazyPredator.h"  // version 2.0 currently in subdirectory
#include "LocalSpace.h"
#include "obstacle.h"
#include "shape.h"
#include "Vec3.h"


// Define several top-level utilities used in various EvoFlock applications.
namespace EvoFlock
{
void visualizeBestIfRequested(LP::Population* population);
void visualizePreviouslyLoggedFlockParameters();

void runOneFlockEvolution()
{
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20251105 why did GP steering force get so small?

    // Does this run use GA (genetic algorithm) or GP (genetic programming)?
    //EF::setUsingGA();
    EF::setUsingGP();
    std::cout << "Evolution mode: " << (EF::usingGP()?"GP":"GA") << std::endl;
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Enable multiprocessing (run 4 Flock simulations in parallel, process
    // Flock's boids in parallel).
//    enable_multithreading = false;
    enable_multithreading = true;
    std::cout << "Use multithreading: " << std::boolalpha;
    std::cout << enable_multithreading << std::endl;
    
    // Merge LP and EF RandomSequence, init from clock for unique runs, and log.
    setRS(LP::LPRS());
    RS().setSeedFromClock();
    std::cout << "RandomSequence seed = " << RS().getSeed() << std::endl;
    
    // WIP/HACK runs flock sim, with graphics, for the FlockParameters written
    // inline in this function's source code, above.
    visualizePreviouslyLoggedFlockParameters();

    // The number of Individuals in a population for evolutionary optimization.
    // By default it is divided into sqrt(individuals) breeding sub-populations.
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20251118 reduce tree size, bigger evo pop, longer run
    
//    int individuals = 300;
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20251119 no fix from "extra large" runs.
//    int individuals = 600;
    int individuals = 300;
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    int subpops = std::round(std::sqrt(individuals));
    
    // Total number of Individual update steps. (Steady state update stepss. For
    // a generational GA, this corresponds to (max_evolution_steps / individuals)
    // generations. So 30000 / 300 = 100 "generation equivalents.")
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20251118 reduce tree size, bigger evo pop, longer run
//    int max_evolution_steps = 30000;
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20251119 no fix from "extra large" runs.
//    int max_evolution_steps = 60000;
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    // TODO 20251119 try 2x steps, smaller trees
//    int max_evolution_steps = 30000;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20251120 log function usage counts
//    int max_evolution_steps = 60000;
    int max_evolution_steps = 30000;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

    int ga_tree_size = 1 + FlockParameters::tunableParameterCount();

//    int min_crossover_tree_size = EF::usingGP() ? 10 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 50 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 50 : ga_tree_size;
//
//    min_crossover_tree_size = EF::usingGP() ?  5 : 2;
//    max_crossover_tree_size = EF::usingGP() ? 50 : ga_tree_size;
//    max_initial_tree_size   = EF::usingGP() ? 50 : ga_tree_size;

    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20251118 reduce tree size, bigger evo pop, longer run

//    int min_crossover_tree_size = EF::usingGP() ?  5 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 50 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 50 : ga_tree_size;

//    int min_crossover_tree_size = EF::usingGP() ? 10 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 30 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 30 : ga_tree_size;

    // TODO, no that run was disappointing, even though it was “extra large.”
    
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    // TODO 20251119 try 2x steps, smaller trees
    
//    int min_crossover_tree_size = EF::usingGP() ? 10 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 50 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 50 : ga_tree_size;

//    int min_crossover_tree_size = EF::usingGP() ? 10 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 30 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 30 : ga_tree_size;

    int min_crossover_tree_size = EF::usingGP() ? 10 : 2;
    int max_crossover_tree_size = EF::usingGP() ? 40 : ga_tree_size;
    int max_initial_tree_size   = EF::usingGP() ? 40 : ga_tree_size;

    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

    debugPrint(min_crossover_tree_size);
    debugPrint(max_crossover_tree_size);
    debugPrint(max_initial_tree_size);
    
    LP::Population* population = nullptr;
    
    LP::FunctionSet fs = (EF::usingGP() ?
                          GP::evoflock_gp_function_set() :
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
        if (EF::usingGP())
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
        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20251120 log function usage counts
        LP::CountFunctionUsage usage;
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        for (int i = 0; i < max_evolution_steps; i++)
        {
            // Exit if user interactively quits run.
            if (Draw::getInstance().exitFromRun()) { break; }
            GP::save_fitness_time_series(*population);
            population->evolutionStep(GP::fitnessFunction, GP::scalarize_fitness);
            if ((population->getStepCount() % 100) == 0)
            {
                LP::Individual* individual = population->bestFitness();
                std::cout << individual->tree().to_string(true) << std::endl;
                
                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // TODO 20251120 log function usage counts
                
                usage.zeroEachCounter();
                usage.count(*population);
                auto log_count = [&](std::string name, int c)
                {
//                    std::string s = std::to_string(c);
//                    s.insert(s.begin(), 4 - s.length(), ' ');
//                    std::cout << s << " " << name << std::endl;
                    std::string count = std::to_string(c);
                    count.insert(count.begin(), 4 - count.length(), ' ');
                    std::cout << count << " " << name << std::endl;
                };
                usage.applyToAllCounts(log_count);

                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
            LP::Individual* individual = population->nthBestFitness(i);

            std::cout << std::endl << "**** ";
            std::cout << util::capitalize_word(util::ordinal_word(i));
            std::cout << " best end-of-run individual:" << std::endl << std::endl;
                        
            // For pop best, print formatted version of FlockParameters
            if ((i == 0) and (EF::usingGA()))
            {
                LP::GpTree t = individual->tree(); // Copy for non-const call.
                GP::fp_from_ga_tree(t).print();
                std::cout << std::endl;
            }

            std::cout << individual->tree().to_string(true) << std::endl;
            auto fitness = GP::run_flock_simulation(individual);
            debugPrint(fitness);
        }
    };
    
    if (not Draw::getInstance().exitFromRun()) { record_top_10(); }
    delete population;
    LP::Individual::leakCheck();
}


void runFlockEvolutionLoop()
{
    while (true) { runOneFlockEvolution(); };
}

// Handler for GUI's B key command to visualize "best" Individual.
void visualizeBestIfRequested(LP::Population* population)
{
    Draw& draw = Draw::getInstance();
    if (draw.getVisBestMode())
    {
        bool previous_emt_state = enable_multithreading;
        enable_multithreading = false;
        LP::Individual* individual = population->bestFitness();
        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
        // TODO 20251011 return to debug speed control
        if (EF::usingGP())
        {
            auto tree_string = individual->tree().to_string(true, "    ");
            std::cout << "    Best individual's tree:" << std::endl;
            std::cout << tree_string << std::endl;
        }
        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
        
        bool previous_draw_enable_state = Draw::getInstance().enable();
        Draw::getInstance().setEnable(true);
        GP::run_flock_simulation(individual, 1);
        Draw::getInstance().setEnable(previous_draw_enable_state);
        enable_multithreading = previous_emt_state;
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
    //    // from run 20250724_pop_300_steps_60000
    //    FlockParameters fp(89.1471, 77.8088, 49.7681, 43.8368, 0.148081, 88.4284,
    //                       66.634, 2.55657, 5.65042, 27.2281, -0.92727, -0.53234,
    //                       0.378495, 3.94743, 1.04012);
    //
    //    // from run 20250728_curve_0_10pc_80pc_1
    //    FlockParameters fp(98.0539, 92.3707, 56.2517, 51.0097, 29.2962, 96.414,
    //                       94.4825, 2.74096, 24.305, 34.2467, -0.880103, -0.856635,
    //                       0.2645, 3.96972, 1.24912);
    
    //    EF::enable_multithreading = false;
    //    while (true) { GP::run_flock_simulation(fp, 1); }
}

// Run unit tests in all modules
void unit_test()
{
    util::unit_test();
    Vec3::unit_test();
    LocalSpace::unit_test();
    shape::unit_test();
    Obstacle::unit_test();
    Agent::unit_test();
    Boid::unit_test();
    Flock::unit_test();
    LazyPredator::unit_test();
    Draw::unit_test();
    std::cout << "All unit tests OK." << std::endl;
}

}  // end of namespace EvoFlock
