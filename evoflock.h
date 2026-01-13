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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20251227 Wait, what?! Confusion with default INITIAL speed?!

//    // Global default target speed. Move to const section of FlockParameters?
//    inline static double default_target_speed = 0;

// Global default target speed. Move to const section of FlockParameters?
inline static double default_target_speed = 20;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
//~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
// TODO 20260111 fix ownership of "current fs"
//void visualizePreviouslyLoggedFlockParameters();
void visualizePreviouslyLoggedFlockParameters(const LP::FunctionSet& fs);
//~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~


//~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
// TODO 20251207 inject hand-written code into population
//               very ad hoc, needs work

// Inject hand-written "approximate solution" code into population, for testing.
void injectHandWrittenCodeIntoPopulation(LP::FunctionSet& fs,
                                         LP::Population* population)
{
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    // TODO 20251208 2 clauses for velocity match for 2 nearest neighbors.

//    std::string gp_source =
//    "Add3(Scale3(Add3(Sub3(NearestNeighborVelocity(), Velocity()),             \
//                      Sub3(NearestNeighbor2Velocity(), Velocity())),    10),   \
//          Add3(Scale3(FirstObstacleTimeLimitNormal(1),                 100),   \
//               Add3(LengthAdjust(NearestNeighborOffset(), 10,           80),   \
//                    LengthAdjust(Velocity(), 20,                        40))))";

    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20251211 change NearestNeighborOffset() r=target from 10(!) to 5.

//    std::string hand_written_gp_source =
//    "Add3(Scale3(Sub3(NearestNeighborVelocity(), Velocity()),          20),   \
//          Add3(Scale3(Sub3(NearestNeighbor2Velocity(), Velocity()),    20),   \
//               Add3(Scale3(FirstObstacleTimeLimitNormal(1),           100),   \
//                    Add3(LengthAdjust(NearestNeighborOffset(), 10,     80),   \
//                         LengthAdjust(Velocity(), 20,                  40)))))";

    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    // TODO 20251212 add NearestNeighborOffset2

//    std::string hand_written_gp_source =
//    "Add3(Scale3(Sub3(NearestNeighborVelocity(), Velocity()),          20),   \
//          Add3(Scale3(Sub3(NearestNeighbor2Velocity(), Velocity()),    20),   \
//               Add3(Scale3(FirstObstacleTimeLimitNormal(1),           100),   \
//                    Add3(LengthAdjust(NearestNeighborOffset(), 5,      80),   \
//                         LengthAdjust(Velocity(), 20,                  40)))))";

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20251212 fix target (AGAIN!) for center of separation range.

//    std::string hand_written_gp_source =
//    "Add3(Add3(Scale3(Sub3(NearestNeighborVelocity(),  Velocity()),  20),    \
//               Scale3(Sub3(NearestNeighborVelocity2(), Velocity()),  20)),   \
//          Add3(Add3(LengthAdjust(NearestNeighborOffset(),  5,        80),    \
//                    LengthAdjust(NearestNeighborOffset2(), 5,        80)),   \
//               Add3(Scale3(FirstObstacleTimeLimitNormal(1),         100),    \
//                    LengthAdjust(Velocity(), 20,                     40))))";

    //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~
    // TODO 20260108 try new handwritten tree using "neighborhood" GpFuncs

//    std::string hand_written_gp_source =
//    "Add3(Add3(Scale3(Sub3(NearestNeighborVelocity(),  Velocity()),  20),    \
//               Scale3(Sub3(NearestNeighborVelocity2(), Velocity()),  20)),   \
//          Add3(Add3(LengthAdjust(NearestNeighborOffset(),  3,        80),    \
//                    LengthAdjust(NearestNeighborOffset2(), 3,        80)),   \
//               Add3(Scale3(FirstObstacleTimeLimitNormal(1),         100),    \
//                    LengthAdjust(Velocity(), 20,                     40))))";

//    std::string hand_written_gp_source =
//    "Add3(LengthAdjust(NeighborhoodVelocityDiff(1.2), 0, 100),  \
//          Add3(LengthAdjust(NeighborhoodOffset(1.2), 6,   90),  \
//               Add3(Scale3(ObstacleCollisionNormal(1),    80),  \
//                    LengthAdjust(Velocity(), 20,          70))))";

//    std::string hand_written_gp_source =
//    "Add3(LengthAdjust(NeighborhoodVelocityDiff(1.2), 0,  80),  \
//          Add3(LengthAdjust(NeighborhoodOffset(1.2), 6,   70),  \
//               Add3(Scale3(ObstacleCollisionNormal(1),   100),  \
//                    LengthAdjust(Velocity(), 20,          90))))";

//    std::string hand_written_gp_source =
//    "Add3(LengthAdjust(NeighborhoodVelocityDiff(1.2), 0,  8),  \
//          Add3(LengthAdjust(NeighborhoodOffset(1.2), 6,   7),  \
//               Add3(Scale3(ObstacleCollisionNormal(1),   10),  \
//                    LengthAdjust(Velocity(), 20,          9))))";

    // 20260108
    std::string hand_written_gp_source =
    "Add3(LengthAdjust(NeighborhoodVelocityDiff(1.2), 0,  3),  \
          Add3(LengthAdjust(NeighborhoodOffset(1.2), 6,   2),  \
               Add3(Scale3(ObstacleCollisionNormal(1),   10),  \
                    LengthAdjust(Velocity(), 20,          1))))";

    //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

    
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20260111 fix ownership of "current fs"

    assert(&fs == LP::FunctionSet::xxx_current_fs);  // TEMP for debugging

    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

    LP::GpTree compiled_tree = fs.compile(hand_written_gp_source);
    auto inject = [&](LP::Individual* individual)
    {
        if (EF::RS().randomBool(0.33))
        {
            LP::GpTree tree = compiled_tree;
            tree.mutate();
            individual->setTree(tree);
        }
    };
    population->applyToAllIndividuals(inject);
}


//~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

void runOneFlockEvolution()
{
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20260111 fix ownership of "current fs"

    // Does this run use GA (genetic algorithm) or GP (genetic programming)?
    // EF::setUsingGA();
    EF::setUsingGP();
    std::cout << "Evolution mode: " << (EF::usingGP()?"GP":"GA") << std::endl;

    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

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
    
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20260111 fix ownership of "current fs"
//    // WIP/HACK runs flock sim, with graphics, for the FlockParameters written
//    // inline in this function's source code, above.
//    visualizePreviouslyLoggedFlockParameters();
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

    // The number of Individuals in a population for evolutionary optimization.
    // By default it is divided into sqrt(individuals) breeding sub-populations.
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20260101 try 2x run with neighborhood sensors.
    
//    int individuals = EF::usingGA() ? 300 : 300;
    
    // TODO 20260102_gp_300_pop_60000_steps
//    int individuals = EF::usingGA() ? 300 : 600;
    
    // TODO 20260102_gp_150_pop_30000_steps

//    int individuals = EF::usingGA() ? 300 : 300;
    
    // TODO 20260102_gp_200_pop_30000_steps
//    int individuals = EF::usingGA() ? 300 : 150;
//    int individuals = EF::usingGA() ? 300 : 200;

    int individuals = EF::usingGA() ? 300 : 300;
    
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    int subpops = std::round(std::sqrt(individuals));
    
    // Total number of Individual update steps. (Steady state update stepss. For
    // a generational GA, this corresponds to (max_evolution_steps / individuals)
    // generations. So 30000 / 300 = 100 "generation equivalents.")
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20260101 try 2x run with neighborhood sensors.
//    int max_evolution_steps = 30000;

    // TODO 20260102_gp_150_pop_30000_steps

//    int max_evolution_steps = 60000;
    
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    // TODO 20260110 stop HW inject, tree size from 20-50 to 5-60, 60000 steps.
//    int max_evolution_steps = 30000;
//    int max_evolution_steps = 60000;
    int max_evolution_steps = 30000;
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

    int ga_tree_size = 1 + FlockParameters::tunableParameterCount();
      
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    // TODO 20251221 change logging for population sensor API.

//    int min_crossover_tree_size = EF::usingGP() ? 20 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 60 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 60 : ga_tree_size;

//    int min_crossover_tree_size = EF::usingGP() ? 20 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 40 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 40 : ga_tree_size;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20251229 reduce max tree size from 50 to 40

//    int min_crossover_tree_size = EF::usingGP() ? 20 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 50 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 50 : ga_tree_size;

    //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~
    // TODO 20251230 increase max tree size from 40 to 60

//    int min_crossover_tree_size = EF::usingGP() ? 20 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 40 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 40 : ga_tree_size;

    // TODO 20260103_gp_try_smaller_trees (again)
    
//    int min_crossover_tree_size = EF::usingGP() ? 20 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 60 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 60 : ga_tree_size;

    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    // TODO 20260105 trying bigger trees again
    
//    int min_crossover_tree_size = EF::usingGP() ? 20 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 35 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 35 : ga_tree_size;

    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    // TODO 20260110 stop HW inject, tree size from 20-50 to 5-60, 60000 steps.
    
//    int min_crossover_tree_size = EF::usingGP() ? 20 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 50 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 50 : ga_tree_size;

    int min_crossover_tree_size = EF::usingGP() ?  5 : 2;
    int max_crossover_tree_size = EF::usingGP() ? 60 : ga_tree_size;
    int max_initial_tree_size   = EF::usingGP() ? 60 : ga_tree_size;

    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~

    //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

    debugPrint(min_crossover_tree_size);
    debugPrint(max_crossover_tree_size);
    debugPrint(max_initial_tree_size);
    
    LP::Population* population = nullptr;
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20251222 why no sensor check during create population?
    
//    const LP::FunctionSet& fs = (EF::usingGP() ?
//                          //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
//                          // TODO 20251218 WIP on general purpose "is this tree OK" predicate.
//                          //                          GP::evoflock_gp_function_set() :
//                          GP::evoflockGpFunctionSet() :
//                          //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
//                          GP::evoflock_ga_function_set());
    

//    LP::FunctionSet fs = (EF::usingGP() ?
//                          GP::evoflockGpFunctionSet() :
//                          GP::evoflock_ga_function_set());
  
//    LP::FunctionSet fs = (EF::usingGP() ?
//                          GP::evoflock_gp_function_set_cached_ : // !!!!!!!!!!!!!!!
//                          GP::evoflock_ga_function_set());

    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20251222 hack to fix sometimes-it-works/sometimes-it-doesn't problem
    // with GP::evoflockGpValidateTree(). I had been copying the FS so I could
    // modify its mode (separately for GA and GP). Here I changed it to save a
    // reference to the FS instead (first saving a copy of the GA FS). This
    // seems to work but I don't know why. Need to test GA for regression.
    LP::FunctionSet fs_ga = GP::evoflock_ga_function_set();
    LP::FunctionSet& fs = (EF::usingGP() ?
                           GP::evoflock_gp_function_set_cached_ : // !!!!!!!!!!!!!!!
                           fs_ga);
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

    LP::FunctionSet::xxx_current_fs = &fs;
    
    debugPrint(LP::FunctionSet::xxx_current_fs);


    
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20260111 fix ownership of "current fs"
    
    // Hack to optionally visualize a previously loggged result. Runs flock sim,
    // with graphics, for the GA FlockParameters or GP source code written
    // inline in this function's definition.
    visualizePreviouslyLoggedFlockParameters(fs);
    
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

    
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    fs.print();
    
    {
        std::cout << "Create population, individuals = " << individuals;
        std::cout << ", subpops/demes = " << subpops << std::endl;
        util::Timer t("Create population.");
        LP::Individual::increasing_initial_tree_size = true;
        if (EF::usingGP())
        {
            fs.setValidateTreeFunction(GP::evoflockGpValidateTree);
        }
        else
        {
            fs.setCrossoverFunction(GP::evoflock_ga_crossover);
        }
        population = new LazyPredator::Population(individuals,
                                                  subpops,
                                                  max_initial_tree_size,
                                                  min_crossover_tree_size,
                                                  max_crossover_tree_size,
                                                  fs);
        if (EF::usingGP())
        {
            population->explicit_treeValue_in_evolutionStep = false;
            // injectHandWrittenCodeIntoPopulation(fs, population);
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
            if (EF::usingGP()) { GP::logUsageSensorAPI(*population); }
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

//~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
// TODO 20260111 fix ownership of "current fs"

//    //    // Tool for visualizing a logged set of FlockParameters.
//    //    // To visualize a given set of FlockParameters. Cut/paste from log, compile.
//    //    // Comment out body of this function for normal evolution run.
//    //    void visualizePreviouslyLoggedFlockParameters()
//
//    // Tool to (optionally) visualize a previous logged result. For example, after
//    // an overnight evolution has completed, a result can be copied and pasted from
//    // the log into this function. Works for either GA FlockParameters or for GP
//    // tree/program's "source code." To visualize a given result: cut/paste from log
//    // into the body of this function (following the examples below) and comment out
//    // the "return" statement at the top. This function will then loop forever
//    // running the logged result.
//    void visualizePreviouslyLoggedFlockParameters(const LP::FunctionSet& fs)
//
//    {
//        // For normal run: return without doing anything. Comment this out to post-
//        // visualize a logged result from a previous run.
//        return;
//    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//        //    // To use the hand-tuned parameters:
//        //    // FlockParameters fp;
//        //
//        //    // Saved FP values from a previous run (20250728_curve_0_10pc_80pc_1)
//        //    FlockParameters fp(98.0539, 92.3707, 56.2517, 51.0097, 29.2962, 96.414,
//        //                       94.4825, 2.74096, 24.305, 34.2467, -0.880103,
//        //                       -0.856635, 0.2645, 3.96972, 1.24912);
//
//        //    EF::enable_multithreading = false;
//        //    while (true) { GP::run_flock_simulation(fp, 1); }
//
//        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
//        // TODO 20251231 for run 20251230_gp_same_full_range_align
//
//    //        // run 20251230_gp_same_full_range_align
//    //        std::string gp_source =
//    //        "Scale3(Scale3(Sub3(LengthAdjust(LengthAdjust(ObstacleCollisionNormal(0.967375), \
//    //                                                     98.3067, \
//    //                                                     2.86499), \
//    //                                        57.4317, \
//    //                                        4.46489), \
//    //                           Div3(Add3(Div3(Velocity(), \
//    //                                          4.68352), \
//    //                                     Sub3(Add3(Div3(Velocity(), \
//    //                                                    4.63576), \
//    //                                               Sub3(NeighborhoodOffset(1.9779), \
//    //                                                    LengthAdjust(Scale3(NeighborhoodVelocity(1.95049), \
//    //                                                                        0.67775), \
//    //                                                                 59.2661, \
//    //                                                                 7.29023))), \
//    //                                          LengthAdjust(LengthAdjust(Scale3(NeighborhoodVelocity(1.9436), \
//    //                                                                           0.345667), \
//    //                                                                    92.6927, \
//    //                                                                    6.64604), \
//    //                                                       82.4551, \
//    //                                                       1.81472))), \
//    //                                2.37041)), \
//    //                      8.84354), \
//    //               2.09311)";
//    //
//
//        // 20260110
//        std::string gp_source =
//        "Add3(LengthAdjust(Velocity(),  \
//                          18.4136,  \
//                          20.7732),  \
//             Add3(LengthAdjust(NeighborhoodOffset(-0.762564),  \
//                               19.7924,  \
//                               -0.688828),  \
//                  Add3(Scale3(Add3(LengthAdjust(NeighborhoodVelocityDiff(1.47306),  \
//                                                0.559172,  \
//                                                -2.23372),  \
//                                   Add3(LengthAdjust(NeighborhoodOffset(8.79137),  \
//                                                     2.62476,  \
//                                                     -0.899679),  \
//                                        Add3(Scale3(ObstacleCollisionNormal(0.408902),  \
//                                                    10.4109),  \
//                                             LengthAdjust(Velocity(),  \
//                                                          20.8701,  \
//                                                          4.28683)))),  \
//                              14.7764),  \
//                       LengthAdjust(NeighborhoodVelocityDiff(2.90797),  \
//                                    2.13027,  \
//                                    -1.23851))))";
//
//        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//        // TODO 20260111 fix ownership of "current fs"
//
//    //        // TODO 20251230 very temp needs cleanup
//    //    //    LP::FunctionSet fs = GP::evoflock_gp_function_set();
//    //        LP::FunctionSet fs = GP::evoflock_gp_function_set_cached_;
//    //        LP::FunctionSet::xxx_current_fs = &fs;
//    //
//    //        LP::GpTree tree = fs.compile(gp_source);
//    //        LP::Individual individual(tree);
//    //
//    //        EF::enable_multithreading = false;
//    //        Draw::getInstance().setEnable(true);
//    //        while (true) { GP::run_flock_simulation(&individual, 1); }
//
//
//    //        // TODO 20251230 very temp needs cleanup
//    //    //    LP::FunctionSet fs = GP::evoflock_gp_function_set();
//    //        LP::FunctionSet fs = GP::evoflock_gp_function_set_cached_;
//    //        LP::FunctionSet::xxx_current_fs = &fs;
//
//        LP::GpTree tree = fs.compile(gp_source);
//        LP::Individual individual(tree);
//
//        EF::enable_multithreading = false;
//        Draw::getInstance().setEnable(true);
//        while (true) { GP::run_flock_simulation(&individual, 1); }
//
//        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
//
//    }


//    // Tool to (optionally) visualize a previous logged result. For example, after
//    // an overnight evolution has completed, a result can be copied and pasted from
//    // the log into this function. Works for either GA FlockParameters or for GP
//    // tree/program's "source code." To visualize a given result: cut/paste from log
//    // into the body of this function (following the examples below) and comment out
//    // the "return" statement at the top. This function will then loop forever
//    // running the logged result.
//    void visualizePreviouslyLoggedFlockParameters(const LP::FunctionSet& fs)
//
//    {
//        // For normal run: return without doing anything. Comment this out to post-
//        // visualize a logged result from a previous run.
//    //    return;
//
//        // To visualize FlockParameters from a previous GA run
//
//        //    // Saved FP values from a previous run (20250728_curve_0_10pc_80pc_1)
//        //    FlockParameters fp(98.0539, 92.3707, 56.2517, 51.0097, 29.2962, 96.414,
//        //                       94.4825, 2.74096, 24.305, 34.2467, -0.880103,
//        //                       -0.856635, 0.2645, 3.96972, 1.24912);
//
//        //    EF::enable_multithreading = false;
//        //    while (true) { GP::run_flock_simulation(fp, 1); }
//
//        // To visualize a GpTree (source code) from a previous GP run.
//
//        // 20260110
//        std::string gp_source =
//        "Add3(LengthAdjust(Velocity(),  \
//                          18.4136,  \
//                          20.7732),  \
//             Add3(LengthAdjust(NeighborhoodOffset(-0.762564),  \
//                               19.7924,  \
//                               -0.688828),  \
//                  Add3(Scale3(Add3(LengthAdjust(NeighborhoodVelocityDiff(1.47306),  \
//                                                0.559172,  \
//                                                -2.23372),  \
//                                   Add3(LengthAdjust(NeighborhoodOffset(8.79137),  \
//                                                     2.62476,  \
//                                                     -0.899679),  \
//                                        Add3(Scale3(ObstacleCollisionNormal(0.408902),  \
//                                                    10.4109),  \
//                                             LengthAdjust(Velocity(),  \
//                                                          20.8701,  \
//                                                          4.28683)))),  \
//                              14.7764),  \
//                       LengthAdjust(NeighborhoodVelocityDiff(2.90797),  \
//                                    2.13027,  \
//                                    -1.23851))))";
//        LP::GpTree tree = fs.compile(gp_source);
//        LP::Individual individual(tree);
//        EF::enable_multithreading = false;
//        Draw::getInstance().setEnable(true);
//        while (true) { GP::run_flock_simulation(&individual, 1); }
//    }


// Tool to (optionally) visualize a previous logged result. For example, after
// an overnight evolution has completed, a result can be copied and pasted from
// the log into this function. Works for either GA FlockParameters or for GP
// tree/program's "source code." To visualize a given result: cut/paste from log
// into the body of this function (following the examples below) and comment out
// the "return" statement at the top. This function will then loop forever
// running the logged result.
void visualizePreviouslyLoggedFlockParameters(const LP::FunctionSet& fs)

{
    return;  // For normal EF run: return without doing anything. Comment this
             // out to post-visualize a logged result from a previous run.

    if (EF::usingGA())
    {
        // To visualize FlockParameters from a previous GA run
        
        //// Saved FP values from run (20250728_curve_0_10pc_80pc_1)
        //FlockParameters fp(98.0539, 92.3707, 56.2517, 51.0097, 29.2962,
        //                   96.414, 94.4825, 2.74096, 24.305, 34.2467,
        //                   -0.880103, -0.856635, 0.2645, 3.96972, 1.24912);
        
        // Saved FP values from run 20260111_ga_regress_test (best fit 0.83)
        FlockParameters fp(63.744, 81.2466, 43.4222, 28.0775, 16.4975,
                           91.6614, 81.1959, 2.53281, 6.47123, 10.1784,
                           -0.980982, -0.878616, 0.238952, 5.23155, 1.70729);
        
        EF::enable_multithreading = false;
        Draw::getInstance().setEnable(true);
        //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~
        // TODO 20260112 fix ownership of "current fs"
//        LP::Individual individual(GP::gaTreeFromFP(fp, fs));
        LP::Individual individual(GP::gaTreeFromFP(fp, fs), fs);
        //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~
        while (true) { GP::run_flock_simulation(&individual, 1); }
    }
    else
    {
        // To visualize a GpTree (source code) from a previous GP run.
        
        // 20260110
        std::string gp_source =
        "Add3(LengthAdjust(Velocity(),  \
                          18.4136,  \
                          20.7732),  \
             Add3(LengthAdjust(NeighborhoodOffset(-0.762564),  \
                               19.7924,  \
                               -0.688828),  \
                  Add3(Scale3(Add3(LengthAdjust(NeighborhoodVelocityDiff(1.47306),  \
                                                0.559172,  \
                                                -2.23372),  \
                                   Add3(LengthAdjust(NeighborhoodOffset(8.79137),  \
                                                     2.62476,  \
                                                     -0.899679),  \
                                        Add3(Scale3(ObstacleCollisionNormal(0.408902),  \
                                                    10.4109),  \
                                             LengthAdjust(Velocity(),  \
                                                          20.8701,  \
                                                          4.28683)))),  \
                              14.7764),  \
                       LengthAdjust(NeighborhoodVelocityDiff(2.90797),  \
                                    2.13027,  \
                                    -1.23851))))";

        LP::GpTree tree = fs.compile(gp_source);
        //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~
        // TODO 20260112 fix ownership of "current fs"
//        LP::Individual individual(tree);
        LP::Individual individual(tree, fs);
        //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~
        EF::enable_multithreading = false;
        Draw::getInstance().setEnable(true);
        while (true) { GP::run_flock_simulation(&individual, 1); }
    }
}

//~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

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
