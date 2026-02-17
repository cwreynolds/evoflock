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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20260213 use EF::add_curvature_objective for no obstacles run

// Global switch (temp?) enables 4th objective component for boosting curvature.
//inline static bool add_curvature_objective = false;

// ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~
// TODO 20260215 make radius smaller for viewing convenience

//~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
// TODO 20260216 add EF::no_obstacles_mode

//inline static bool add_curvature_objective = true;
inline static bool add_curvature_objective = false;

// experimental / temp?
inline static int override_boids_per_flock = -1;

//~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

// ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
// TODO 20260216 add EF::no_obstacles_mode

//inline static bool no_obstacles_mode = false;
inline static bool no_obstacles_mode = true;

//~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~


// Global default target speed. Move to const section of FlockParameters?
inline static double default_target_speed = 20;

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

// Forward references
void visualizeBestIfRequested(LP::Population* population);
void visualizePreviouslyLoggedFlockParameters(const LP::FunctionSet& fs);
void injectHandWrittenCodeIntoPopulation(LP::FunctionSet& fs, LP::Population* p);


void runOneFlockEvolution()
{
    // Does this run use GA (genetic algorithm) or GP (genetic programming)?
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    EF::setUsingGA();
    // EF::setUsingGP();
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    std::cout << "Evolution mode: " << (EF::usingGP()?"GP":"GA") << std::endl;

    // Set likelihood of crossover versus hoist (on a randomly selected parent).
    LP::GpTree::likelihood_of_crossover_ = EF::usingGP() ? 0.9 : 1.0;
    debugPrint(LP::GpTree::likelihood_of_crossover_);

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
    

    // The number of Individuals in a population for evolutionary optimization.
    // By default it is divided into sqrt(individuals) breeding sub-populations.
    int individuals = EF::usingGA() ? 300 : 300;
    int subpops = std::round(std::sqrt(individuals));
    
    // Total number of Individual update steps. (Steady state update stepss. For
    // a generational GA, this corresponds to (max_evolution_steps / individuals)
    // generations. So 30000 / 300 = 100 "generation equivalents.")
    int max_evolution_steps = 30000;

    int ga_tree_size = 1 + FlockParameters::tunableParameterCount();

    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20260117 very experimental, until right size OR "valid"

//    int min_crossover_tree_size = EF::usingGP() ? 20 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 60 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 60 : ga_tree_size;

    //~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~
    // TODO 20260122 start-end max tree size
    
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20260124 change "style" retry loop to respect size constraints

    // Oh, this is for crossover, which does not yet use FS::genericTreeMaker()
//    double start_max_tree_size = 30;
//    double end_max_tree_size = 60;

    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    // TODO 20260124 trying to time travel back to 20260121?
    
//    double start_max_tree_size = 40;
//    double end_max_tree_size = 80;

    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    // TODO 20260126 change start/end max tree size from 60/60 to 20/60

//    double start_max_tree_size = 60;
//    double end_max_tree_size = 60;

//    // 20260126_gp_mts_60_60_to_20_60
//    double start_max_tree_size = 20;
//    double end_max_tree_size = 60;
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20260127 fiddle with min/max tree size

//    // 20260126_gp_mts_20_60_to_30_60
//    double start_max_tree_size = 30;
//    double end_max_tree_size = 60;
  
//    // 20260127_gp_mts_30_60_to_40_60
//    double start_max_tree_size = 40;
//    double end_max_tree_size = 60;
    
    // 20260205_gp_mts_40_60_to_60_60
    double start_max_tree_size = 60;
    double end_max_tree_size = 60;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~

    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    // TODO 20260126 change start/end max tree size from 60/60 to 20/60
    
//    int min_crossover_tree_size = EF::usingGP() ? 10 : 2;
//    int max_crossover_tree_size = EF::usingGP() ? 60 : ga_tree_size;
//    int max_initial_tree_size   = EF::usingGP() ? 60 : ga_tree_size;

    double smts = start_max_tree_size;
    int min_crossover_tree_size = EF::usingGP() ?   10 : 2;
    int max_crossover_tree_size = EF::usingGP() ? smts : ga_tree_size;
    int max_initial_tree_size   = EF::usingGP() ? smts : ga_tree_size;

    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

    //~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~

    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

    debugPrint(min_crossover_tree_size);
    debugPrint(max_crossover_tree_size);
    debugPrint(max_initial_tree_size);
    
    LP::Population* population = nullptr;

    // I would have just copied the FunctionSet objects, but that caused a
    // mysterious bug where GP::evoflockGpValidateTree() seemed to always
    // classify as not valid. Ended up changing the GA/GP side to return a
    // reference to ab FS object with modes pre-set.
    LP::FunctionSet& fs = (EF::usingGP() ?
                           GP::evoflockGpFunctionSet() :
                           GP::evoflockGaFunctionSet());
    //~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~
    // TODO 20260122 new generic tree maker with size and style constraints
    fs.setMinTreeSize(min_crossover_tree_size);
    fs.setMaxTreeSize(max_crossover_tree_size);
    //~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~
    LP::FunctionSet::reset_smallest_init_tree_xxx();
    fs.print();

    // Hack to optionally visualize a previously logged result. Runs flock sim,
    // with graphics, for the GA FlockParameters or GP source code written
    // inline in this function's definition.
    visualizePreviouslyLoggedFlockParameters(fs);

    {
        std::cout << "Create population, individuals = " << individuals;
        std::cout << ", subpops/demes = " << subpops << std::endl;
        util::Timer t("Create population.");
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
            //~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~
            // TODO 20260122 start-end max tree size

            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20260127 fiddle with min/max tree size

            double fraction = float(i) / max_evolution_steps;
            double max_tree_size = util::interpolate(fraction,
                                                     float(start_max_tree_size),
                                                     float(end_max_tree_size));
//            fs.setMaxTreeSize(max_tree_size);
//            population->setMaxCrossoverTreeSize(max_tree_size);
            fs.setMaxTreeSize(std::round(max_tree_size));
            population->setMaxCrossoverTreeSize(std::round(max_tree_size));

            std::cout << "    ";  // "log_prefix"
            debugPrint(max_tree_size);
            std::cout << std::endl;
            
            //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
            // TODO 20260212 log GpTree::likelihood_of_crossover_
//            std::cout << "    ";  // "log_prefix"
//            debugPrint(LP::GpTree::likelihood_of_crossover_);
//            std::cout << std::endl;
            //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~

            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            //~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~
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
        
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
        // TODO 20260216 add EF::no_obstacles_mode
        int previous_obpf = override_boids_per_flock;
        override_boids_per_flock = 1200;
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~

        GP::run_flock_simulation(individual, 1);
        
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
        // TODO 20260216 add EF::no_obstacles_mode
        override_boids_per_flock = previous_obpf;
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~

        Draw::getInstance().setEnable(previous_draw_enable_state);
        enable_multithreading = previous_emt_state;
        draw.clearVisBestMode();
    }
}


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
        
        // // Saved FP values from run 20260111_ga_regress_test (best fit 0.83)
        // FlockParameters fp(63.744, 81.2466, 43.4222, 28.0775, 16.4975,
        //                    91.6614, 81.1959, 2.53281, 6.47123, 10.1784,
        //                    -0.980982, -0.878616, 0.238952, 5.23155, 1.70729);
        
        // // Saved FP values from run 20260214_ga_no_obs_tweak_curvature_2
        // FlockParameters fp(96.1906, 93.11, 63.5963, 99.1355, 30.6651, 68.6572,
        //                    54.5162, 3.27533, 53.5294, 39.3137, -0.579767,
        //                 0.00972851, -0.825164, 27.2047, 1.90221);
        
        // Saved FP values from run 20260215_ga_no_obs_more_wip
        FlockParameters fp(91.1072, 98.6429, 51.8598, 94.8959, 21.8851, 22.2565,
                           5.87154, 2.96989, 92.1022, 12.3065, -0.805401,
                           -0.112003, -0.74847, 63.9714, 3.51265);
        
        EF::enable_multithreading = false;
        Draw::getInstance().setEnable(true);
        LP::Individual individual(GP::gaTreeFromFP(fp, fs), fs);
        while (true) { GP::run_flock_simulation(&individual, 1); }
    }
    else
    {
        // To visualize a GpTree (source code) from a previous GP run.
        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20260118 add today's best.
        
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

//        // 20260117_gp_re-enable_alignment -- 20260118_gp_fewer_retry_tree_init
//        std::string gp_source =
//        "Add3(Div3(Add3(LengthAdjust(Div3(Scale3(Div3(LengthAdjust(NeighborhoodOffset(1.14937), \
//                                                                  13.0763, \
//                                                                  88.5217), \
//                                                     2.45265), \
//                                                1.91974), \
//                                         2.41891), \
//                                    26.7903, \
//                                    36.0278), \
//                       Velocity()), \
//                  3.37266), \
//             Add3(Sub3(Sub3(Sub3(NeighborhoodVelocityDiff(1.36538), \
//                                 LengthAdjust(Sub3(NeighborhoodVelocityDiff(1.28684), \
//                                                   LengthAdjust(Scale3(Div3(LengthAdjust(Velocity(), \
//                                                                                         20.1294, \
//                                                                                         51.9071), \
//                                                                            3.1816), \
//                                                                       5.22176), \
//                                                                11.1512, \
//                                                                29.2247)), \
//                                              2.54187, \
//                                              41.6189)), \
//                            Div3(Velocity(), \
//                                 5.7574)), \
//                       LengthAdjust(ObstacleCollisionNormal(0.903173), \
//                                    85.3268, \
//                                    -60.5663)), \
//                  LengthAdjust(Div3(LengthAdjust(ObstacleCollisionNormal(0.358852), \
//                                                 67.4465, \
//                                                 33.6657), \
//                                    4.3167), \
//                               18.6311, \
//                               74.6501)))";
  
//        // 20260118_gp_exponentiate
//        std::string gp_source = "Sub3(Sub3(Scale3(LengthAdjust(Scale3(Div3(ObstacleCollisionNormal(0.50754), 2.2431), 9.86361), 55.8529, 72.8472), 4.11306), LengthAdjust(Scale3(Sub3(Scale3(Scale3(Div3(Sub3(NeighborhoodVelocityDiff(1.29208), LengthAdjust(LengthAdjust(Velocity(), 20.1216, 83.6724), 60.3436, 21.1387)), 1.80084), 5.56177), 7.40495), LengthAdjust(Div3(Div3(NeighborhoodOffset(1.98051), 6.95959), 9.88759), 12.129, 58.0658)), 2.63974), 39.4009, 87.1055)), Div3(NeighborhoodOffset(1.64434), 9.09468))";

//        // 20260119_gp_only_expt_separation_2
//        std::string gp_source =
//        "Add3(Sub3(Add3(Scale3(NeighborhoodVelocityDiff(0.981347), \
//                              8.81377), \
//                       Add3(Sub3(Sub3(Add3(LengthAdjust(LengthAdjust(Velocity(), \
//                                                                     8.11177, \
//                                                                     -11.4987), \
//                                                        24.0162, \
//                                                        -49.1329), \
//                                           Add3(Sub3(LengthAdjust(LengthAdjust(Velocity(), \
//                                                                               20.9619, \
//                                                                               -17.6903), \
//                                                                  26.7431, \
//                                                                  -92.4291), \
//                                                     Scale3(LengthAdjust(ObstacleCollisionNormal(0.617278), \
//                                                                         7.00654, \
//                                                                         -43.2003), \
//                                                            9.15498)), \
//                                                NeighborhoodVelocityDiff(1.11207))), \
//                                      LengthAdjust(NeighborhoodOffset(1.34865), \
//                                                   29.0255, \
//                                                   0.912394)), \
//                                 LengthAdjust(NeighborhoodOffset(1.26283), \
//                                              40.5006, \
//                                              24.6023)), \
//                            NeighborhoodVelocityDiff(1.05209))), \
//                  LengthAdjust(NeighborhoodOffset(0.957626), \
//                               13.7668, \
//                               -3.53544)), \
//             Scale3(NeighborhoodOffset(1.00308), \
//                    4.01942))";

//        // 20260122_gp_genericTreeMaker
//        std::string gp_source =
//        "Add3(Add3(Add3(NeighborhoodOffset(1.47095), \
//                       Sub3(Add3(Add3(NeighborhoodOffset(1.33325), \
//                                      Sub3(Add3(NeighborhoodOffset(1.09238), \
//                                                Sub3(Add3(Add3(NeighborhoodVelocityDiff(1.33661), \
//                                                               Sub3(Velocity(), \
//                                                                    LengthAdjust(Velocity(), \
//                                                                                 19.4879, \
//                                                                                 -59.8654))), \
//                                                          Div3(Add3(Add3(NeighborhoodOffset(1.73836), \
//                                                                         NeighborhoodOffset(1.6515)), \
//                                                                    ObstacleCollisionNormal(0.442421)), \
//                                                               5.64864)), \
//                                                     LengthAdjust(Velocity(), \
//                                                                  16.7502, \
//                                                                  -70.9157))), \
//                                           LengthAdjust(Velocity(), \
//                                                        28.3767, \
//                                                        -52.804))), \
//                                 NeighborhoodVelocityDiff(1.20695)), \
//                            LengthAdjust(NeighborhoodOffset(1.68827), \
//                                         28.4728, \
//                                         14.4578))), \
//                  NeighborhoodVelocityDiff(1.27099)), \
//             Scale3(LengthAdjust(ObstacleCollisionNormal(0.640166), \
//                                 77.5608, \
//                                 27.6224), \
//                    9.66235))";
        
//        // 20260128_gp_sep_score_exp_1.5_to_1
//        std::string gp_source =
//        "Add3(Add3(Div3(NeighborhoodVelocityDiff(0.518534), \
//                        0.526287), \
//                   Sub3(Add3(Div3(Sub3(NeighborhoodVelocityDiff(0.648968), \
//                                       LengthAdjust(Div3(Scale3(Velocity(), \
//                                                                9.62188), \
//                                                         2.45326), \
//                                                    81.183, \
//                                                    -79.8588)), \
//                                  1.99148), \
//                             LengthAdjust(ObstacleCollisionNormal(0.552556), \
//                                          49.129, \
//                                          77.7599)), \
//                        LengthAdjust(NeighborhoodOffset(1.83964), \
//                                     9.50483, \
//                                     20.1819))), \
//              Add3(NeighborhoodVelocityDiff(1.54523), \
//                   Add3(Add3(Add3(NeighborhoodVelocityDiff(1.48169), \
//                                  Add3(Add3(Add3(LengthAdjust(ObstacleCollisionNormal(0.708356), \
//                                                              84.1532, \
//                                                              79.3612), \
//                                                 NeighborhoodOffset(1.44192)), \
//                                            NeighborhoodOffset(1.93117)), \
//                                       NeighborhoodOffset(0.628169))), \
//                             NeighborhoodOffset(1.84771)), \
//                        NeighborhoodOffset(0.615739))))";

        // 20260130_gp_wip_hoist_2
        std::string gp_source =
        "Add3(LengthAdjust(ObstacleCollisionNormal(0.760695), \
                           5.93495, \
                           76.8379), \
              Div3(Add3(LengthAdjust(ObstacleCollisionNormal(0.834561), \
                                     11.8691, \
                                     97.3668), \
                        Div3(Sub3(NeighborhoodOffset(0.737614), \
                                  Add3(ObstacleCollisionNormal(0.867631), \
                                       Sub3(ObstacleCollisionNormal(0.747136), \
                                            Add3(Sub3(Scale3(NeighborhoodVelocityDiff(1.90785), \
                                                             5.1093), \
                                                      LengthAdjust(Div3(Div3(NeighborhoodOffset(1.74604), \
                                                                             2.63401), \
                                                                        7.60497), \
                                                                   92.7019, \
                                                                   13.9075)), \
                                                 LengthAdjust(Sub3(ObstacleCollisionNormal(0.752927), \
                                                                   Velocity()), \
                                                              19.7722, \
                                                              -58.0037))))), \
                             1.52315)), \
                   0.767777))";
        
        LP::GpTree tree = fs.compile(gp_source);
        LP::Individual individual(tree, fs);
        EF::enable_multithreading = false;
        Draw::getInstance().setEnable(true);
        while (true) { GP::run_flock_simulation(&individual, 1); }
    }
}


// Inject hand-written "approximate solution" code into population, for testing.
void injectHandWrittenCodeIntoPopulation(LP::FunctionSet& fs,
                                         LP::Population* population)
{

    // std::string hand_written_gp_source =
    // "Add3(LengthAdjust(NeighborhoodVelocityDiff(1.2), 0,  8),  \
    //       Add3(LengthAdjust(NeighborhoodOffset(1.2), 6,   7),  \
    //            Add3(Scale3(ObstacleCollisionNormal(1),   10),  \
    //                 LengthAdjust(Velocity(), 20,          9))))";

    // 20260108
    std::string hand_written_gp_source =
    "Add3(LengthAdjust(NeighborhoodVelocityDiff(1.2), 0,  3),  \
          Add3(LengthAdjust(NeighborhoodOffset(1.2), 6,   2),  \
               Add3(Scale3(ObstacleCollisionNormal(1),   10),  \
                    LengthAdjust(Velocity(), 20,          1))))";

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
