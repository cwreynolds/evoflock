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

// Global default target speed. Move to const section of FlockParameters?
inline static double default_target_speed = 0;

static inline bool using_GA_ = true;
inline bool usingGP() { return not using_GA_; }
inline bool usingGA() { return using_GA_; }
inline void setUsingGP() { using_GA_ = false; }
inline void setUsingGA() { using_GA_ = true; }

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


//~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
// TODO 20251207 inject hand-written code into population
//               very ad hoc, needs work

// Inject hand-written "approximate solution" code into population, for testing.
void injectHandWrittenCodeIntoPopulation(LP::FunctionSet& fs,
                                         LP::Population* population)
{
    std::string gp_source =
    "Add3(Scale3(Add3(Sub3(NearestNeighborVelocity(), Velocity()),             \
                      Sub3(NearestNeighbor2Velocity(), Velocity())),    10),   \
          Add3(Scale3(FirstObstacleTimeLimitNormal(1),                 100),   \
               Add3(LengthAdjust(NearestNeighborOffset(), 10,           80),   \
                    LengthAdjust(Velocity(), 20,                        40))))";
    LP::GpTree tree = fs.compile(gp_source);
    auto inject = [&](LP::Individual* individual)
    {
        if (EF::RS().randomBool(0.33))
        {
            tree.mutate();
            individual->setTree(tree);
            // std::cout << tree.to_string(true) << std::endl;
        }
    };
    population->applyToAllIndividuals(inject);
}


//~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

void runOneFlockEvolution()
{
    // Does this run use GA (genetic algorithm) or GP (genetic programming)?
    // EF::setUsingGA();
    EF::setUsingGP();
    std::cout << "Evolution mode: " << (EF::usingGP()?"GP":"GA") << std::endl;

    // Enable multiprocessing (run 4 Flock simulations in parallel, process
    // Flock's boids in parallel).
    // enable_multithreading = false;
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
    int individuals = 300;
    int subpops = std::round(std::sqrt(individuals));
    
    // Total number of Individual update steps. (Steady state update stepss. For
    // a generational GA, this corresponds to (max_evolution_steps / individuals)
    // generations. So 30000 / 300 = 100 "generation equivalents.")
    int max_evolution_steps = 30000;

    int ga_tree_size = 1 + FlockParameters::tunableParameterCount();
    
//    int min_crossover_tree_size = EF::usingGP() ? 10 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 50 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 50 : ga_tree_size;

//    int min_crossover_tree_size = EF::usingGP() ? 10 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 30 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 30 : ga_tree_size;

//    int min_crossover_tree_size = EF::usingGP() ? 10 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 40 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 40 : ga_tree_size;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20251125 I keep flip-flopping on max tree size

//    int min_crossover_tree_size = EF::usingGP() ? 10 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 60 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 60 : ga_tree_size;

//    int min_crossover_tree_size = EF::usingGP() ? 10 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 30 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 30 : ga_tree_size;

    int min_crossover_tree_size = EF::usingGP() ? 10 : 2;
    int max_crossover_tree_size = EF::usingGP() ? 40 : ga_tree_size;
    int max_initial_tree_size   = EF::usingGP() ? 40 : ga_tree_size;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
            
            //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
            // TODO 20251207 inject hand-written code into population

            injectHandWrittenCodeIntoPopulation(fs, population);

            //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
        }
        else
        {
            fs.setCrossoverFunction(GP::evoflock_ga_crossover);
        }
    }
    
    LP::CountFunctionUsage usage;
    auto log_usage_counts = [&]()
    {
        usage.zeroEachCounter();
        usage.count(*population);
        auto log_count = [&](std::string name, int c)
        {
            std::string count = std::to_string(c);
            count.insert(count.begin(), 4 - count.length(), ' ');
            std::cout << count << " " << name << std::endl;
        };
        usage.applyToAllCounts(log_count);
    };

    {
        std::cout << "Run evolution for " << max_evolution_steps;
        std::cout << " steps." << std::endl;
        util::Timer t("Run evolution.");
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
                log_usage_counts();
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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20251203 visualizePreviouslyLoggedFlockParameters() for GP as well as GA

// Tool for visualizing a logged set of FlockParameters.
// To visualize a given set of FlockParameters. Cut/paste from log, compile.
// Comment out body of this function for normal evolution run.
void visualizePreviouslyLoggedFlockParameters()
{
    //    // To use the hand-tuned parameters:
    //    // FlockParameters fp;
    //
    //    // Saved FP values from a previous run (20250728_curve_0_10pc_80pc_1)
    //    FlockParameters fp(98.0539, 92.3707, 56.2517, 51.0097, 29.2962, 96.414,
    //                       94.4825, 2.74096, 24.305, 34.2467, -0.880103,
    //                       -0.856635, 0.2645, 3.96972, 1.24912);
    
    //    EF::enable_multithreading = false;
    //    while (true) { GP::run_flock_simulation(fp, 1); }
    
    
//    std::string gp_source =
//         "Add3(Scale3(FirstObstacleNormal(), 5), \
//               Add3(LengthAdjust(Velocity(), 20, 4), \
//                    LengthAdjust(NearestNeighborOffset(), 3, 7)))";
//    std::string gp_source =
//         "Add3(Scale3(FirstObstacleNormal(), 50), \
//               Add3(LengthAdjust(Velocity(), 20, 40), \
//                    LengthAdjust(NearestNeighborOffset(), 3, 70)))";
//    std::string gp_source =
//         "Add3(Scale3(Forward(),                                  50), \
//               Add3(Scale3(FirstObstacleNormal(),                100), \
//                    Add3(LengthAdjust(Velocity(), 20,             30), \
//                         LengthAdjust(NearestNeighborOffset(), 3, 30))))";
    
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20251204 why does handmade program behave poorly?
    
//    std::string gp_source =
//         "Add3(Scale3(Forward(),                                  66), \
//               Add3(Scale3(FirstObstacleNormal(),                100), \
//                    Add3(LengthAdjust(Velocity(), 20,             66), \
//                         LengthAdjust(NearestNeighborOffset(), 3, 33))))";

    
//    std::string gp_source = "Scale3(Forward(), 66)";

//    std::string gp_source = "LengthAdjust(Velocity(), 19, 66)";

//    std::string gp_source = "Add3(Scale3(Forward(), 33), \
//                                  LengthAdjust(Velocity(), 19, 100))";

    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20251205 rebuild hand-written GP example

//    std::string gp_source = "LengthAdjust(Velocity(), 20, 100)";
  
//    std::string gp_source = "Add3(LengthAdjust(Velocity(), 20,   100), \
//                                  Scale3(FirstObstacleNormal(),  100))";

//    std::string gp_source =
//        "SideAndForward(Scale3(FirstObstacleNormal(),  66), \
//                        LengthAdjust(Velocity(), 20,  100))";

//    std::string gp_source =
//        "SideAndForward(Scale3(Normalize(FirstObstacleNormal()),  66), \
//                        LengthAdjust(Velocity(), 20,  100))";

//    std::string gp_source =
//        "Add3(LengthAdjust(Velocity(), 20,                      33),  \
//              Scale3(Normalize(ToSide(FirstObstacleNormal())), 100))";

//    std::string gp_source =
//        "Add3(LengthAdjust(Velocity(), 20,                           33), \
//              Add3(LengthAdjust(NearestNeighborOffset(), 3,          66), \
//                   Scale3(Normalize(ToSide(FirstObstacleNormal())), 100)))";

//    std::string gp_source =
//        "Add3(LengthAdjust(Velocity(), 20,                           10), \
//              Add3(LengthAdjust(NearestNeighborOffset(), 3,          20), \
//                   Scale3(Normalize(ToSide(FirstObstacleNormal())), 100)))";

//    std::string gp_source =
//        "Add3(LengthAdjust(Velocity(), 20,                           10), \
//              Add3(LengthAdjust(NearestNeighborOffset(), 3,          20), \
//                   Scale3(FirstObstacleNormal(),                    100)))";

//    std::string gp_source =
//        "Add3(LengthAdjust(Velocity(), 20,                           10), \
//              Add3(LengthAdjust(NearestNeighborOffset(), 3,          20), \
//                   Scale3(Normalize(ToSide(FirstObstacleNormal())), 200)))";

    
//    std::string gp_source =
//        "Add3(Scale3(Normalize(ToSide(FirstObstacleTimeLimitNormal(1))), 200), \
//              Add3(LengthAdjust(NearestNeighborOffset(), 3,               20), \
//                   LengthAdjust(Velocity(), 20,                           10)))";

    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20251206 try adding an alignment clause

//    std::string gp_source =
//        "Add3(Scale3(Normalize(ToSide(FirstObstacleTimeLimitNormal(1))), 200), \
//              Add3(LengthAdjust(NearestNeighborOffset(), 3,              100), \
//                   LengthAdjust(Velocity(), 20,                           10)))";

//    std::string gp_source =
//    "Add3(Scale3(Normalize(ToSide(FirstObstacleTimeLimitNormal(1))),   200), \
//         Add3(LengthAdjust(NearestNeighborOffset(), 3,                 100), \
//              add3(LengthAdjust(Velocity(), 20,                         10), \
//                   Scale3(Sub3(NearestNeighborVelocity(), \
//                                Velocity()),                            10))))";

//    std::string gp_source =
//    "Add3(Scale3(FirstObstacleTimeLimitNormal(1),                      100), \
//         Add3(LengthAdjust(NearestNeighborOffset(), 3,                  60), \
//              Add3(LengthAdjust(Velocity(), 20,                         20), \
//                   Scale3(Sub3(NearestNeighborVelocity(), Velocity()),  10))))";

//    std::string gp_source =
//    "Add3(Scale3(FirstObstacleTimeLimitNormal(1),                      100), \
//         Add3(LengthAdjust(NearestNeighborOffset(), 3,                  70), \
//              Add3(LengthAdjust(Velocity(), 20,                         40), \
//                   Scale3(Sub3(NearestNeighborVelocity(), Velocity()),  20))))";

//    std::string gp_source =
//    "Add3(Scale3(FirstObstacleTimeLimitNormal(1),                      100), \
//         Add3(LengthAdjust(NearestNeighborOffset(), 4,                  50), \
//              Add3(LengthAdjust(Velocity(), 20,                         40), \
//                   Scale3(Sub3(NearestNeighborVelocity(), Velocity()),  30))))";

//    std::string gp_source =
//    "Add3(Scale3(FirstObstacleTimeLimitNormal(1),                      100), \
//         Add3(LengthAdjust(NearestNeighborOffset(), 4,                  50), \
//              Add3(LengthAdjust(Velocity(), 20,                         40), \
//                   Scale3(Sub3(Velocity(), NearestNeighborVelocity()),  30))))";

//    std::string gp_source =
//    "Add3(Scale3(FirstObstacleTimeLimitNormal(1),                      100), \
//         Add3(LengthAdjust(NearestNeighborOffset(), 4,                  50), \
//              Add3(LengthAdjust(Velocity(), 20,                         40), \
//                   LengthAdjust(Sub3(NearestNeighborVelocity(), \
//                                     Velocity()), 0,                    30))))";

//    std::string gp_source =
//    "Add3(Scale3(FirstObstacleTimeLimitNormal(1),                      100), \
//         Add3(LengthAdjust(NearestNeighborOffset(), 4,                  50), \
//              Add3(LengthAdjust(Velocity(), 20,                         40), \
//                   LengthAdjust(Sub3(Velocity(), \
//                                     NearestNeighborVelocity()), 0,     10))))";

//    std::string gp_source =
//    "Add3(Scale3(FirstObstacleTimeLimitNormal(1),                      100), \
//         Add3(LengthAdjust(NearestNeighborOffset(), 4,                  50), \
//              Add3(LengthAdjust(Velocity(), 20,                         40), \
//                   Scale3(Sub3(NearestNeighborVelocity(), Velocity()),  20))))";

//    std::string gp_source =
//    "Add3(Scale3(FirstObstacleTimeLimitNormal(1),                      100), \
//         Add3(LengthAdjust(NearestNeighborOffset(), 4,                 150), \
//              Add3(LengthAdjust(Velocity(), 20,                         40), \
//                   Scale3(Sub3(NearestNeighborVelocity(), Velocity()),  20))))";

//    std::string gp_source =
//    "Add3(Scale3(FirstObstacleTimeLimitNormal(1),                      100), \
//         Add3(LengthAdjust(NearestNeighborOffset(), 10,                 80), \
//              Add3(LengthAdjust(Velocity(), 20,                         40), \
//                   Scale3(Sub3(NearestNeighborVelocity(), Velocity()),  20))))";

    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    // TODO 20251207 try adding 2nd nearest neighbor velocity.
    
//    std::string gp_source =
//    "Add3(Scale3(FirstObstacleTimeLimitNormal(1),                      100), \
//         Add3(LengthAdjust(NearestNeighborOffset(), 10,                 60), \
//              Add3(LengthAdjust(Velocity(), 20,                         40), \
//                   Scale3(Sub3(NearestNeighborVelocity(), Velocity()),  20))))";

//    std::string gp_source =
//    "Add3(Scale3(Add3(Sub3(NearestNeighborVelocity(), Velocity()),             \
//                     Sub3(NearestNeighbor2Velocity(), Velocity())),     20),   \
//         Add3(Scale3(FirstObstacleTimeLimitNormal(1),                  100),   \
//              Add3(LengthAdjust(NearestNeighborOffset(), 10,            60),   \
//                   LengthAdjust(Velocity(), 20,                         40))))";

    std::string gp_source =
    "Add3(Scale3(Add3(Sub3(NearestNeighborVelocity(), Velocity()),             \
                      Sub3(NearestNeighbor2Velocity(), Velocity())),    10),   \
          Add3(Scale3(FirstObstacleTimeLimitNormal(1),                 100),   \
               Add3(LengthAdjust(NearestNeighborOffset(), 10,           80),   \
                    LengthAdjust(Velocity(), 20,                        40))))";

    
//    LP::FunctionSet fs = GP::evoflock_gp_function_set();
//    LP::GpTree tree = fs.compile(gp_source);
//    LP::Individual individual(tree);
//
//    EF::enable_multithreading = false;
//    Draw::getInstance().setEnable(true);
//    while (true) { GP::run_flock_simulation(&individual, 1); }


    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
