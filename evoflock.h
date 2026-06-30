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

// Select which objectives to enable.
inline static bool use_avoid_objective     = true;
inline static bool use_separate_objective  = true;
inline static bool use_speed_objective     = true;
inline static bool use_curvature_objective = true;
inline static bool use_alignment_objective = true;
inline static bool use_cluster_objective   = true;
//~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
// TODO 20260413 current flock centroid, and velocity
inline static bool use_centroid_objective  = true;
//~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

//~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
// TODO 20260428 move temp murmuration parameters to EF
double center_max_dist = 40;
double center_min_dist = center_max_dist * 0.4;
double centering_strength = 90;
//~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~

// experimental / temp?
inline static int override_boids_per_flock = -1;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20260421 backing up to test README.md

// Special mode for simulating murmurations.
//inline static bool murmuration_mode = false;
inline static bool murmuration_mode = true;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Added for debugging BoxObstacle, maybe remove later?
inline static bool current_boid_is_selected = true;

// No evo. Replay previous results in visualizePreviouslyLoggedFlockParameters.
inline static bool visualize_previous_results_mode = false;
//inline static bool visualize_previous_results_mode = true;

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
    EF::setUsingGA();
    // EF::setUsingGP();
    std::cout << "Evolution mode: " << (EF::usingGP()?"GP":"GA") << std::endl;
    
    // Select which objectives to enable, based on other mode flags.
    if (murmuration_mode)
    {
        use_avoid_objective     = true;
        use_separate_objective  = true;
        use_speed_objective     = true;
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20260515 try turning off curvature_objective
        //               (was seeing too much tornado)
        
        //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~
        // TODO 20260524 bring back curvature

//        use_curvature_objective = true;
//        use_curvature_objective = false;
        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20260618 change initial position to be throughout centroid

//        use_curvature_objective = true;
        use_curvature_objective = false;

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        //~   ~   ~   ~   ~   ~   ~   ~   ~   ~   ~   ~   ~   ~   ~   ~   ~   ~
        // TODO 20260508 try adding in alignment objective for murmuration_mode.
//        use_alignment_objective = false;
//        use_alignment_objective = true;
        // TODO 20260509 why did that get so slow? reverting to retest speed.
        use_alignment_objective = false;
        //~   ~   ~   ~   ~   ~   ~   ~   ~   ~   ~   ~   ~   ~   ~   ~   ~   ~
        
        //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~
        // TODO 20260524 maybe use_cluster_objective is more of a distraction?
//        use_cluster_objective   = true;
//        use_cluster_objective   = false;
        // or not...
        use_cluster_objective   = true;
        //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~

        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
        // TODO 20260413 current flock centroid, and velocity
        use_centroid_objective  = true;
        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    }
    else
    {
        use_avoid_objective     = true;
        use_separate_objective  = true;
        use_speed_objective     = true;
        use_curvature_objective = false;
        use_alignment_objective = false;
        use_cluster_objective   = false;
        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
        // TODO 20260413 current flock centroid, and velocity
        use_centroid_objective  = false;
        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    }

    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    // TODO 20260413 current flock centroid, and velocity
    debugPrint(murmuration_mode);
    debugPrint(use_avoid_objective);
    debugPrint(use_separate_objective);
    debugPrint(use_speed_objective);
    debugPrint(use_curvature_objective);
    debugPrint(use_alignment_objective);
    debugPrint(use_cluster_objective);
    debugPrint(use_centroid_objective);
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    
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
    
    // 20260205_gp_mts_40_60_to_60_60
    double start_max_tree_size = 60;
    double end_max_tree_size = 60;
    
    double smts = start_max_tree_size;
    int min_crossover_tree_size = EF::usingGP() ?   10 : 2;
    int max_crossover_tree_size = EF::usingGP() ? smts : ga_tree_size;
    int max_initial_tree_size   = EF::usingGP() ? smts : ga_tree_size;

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

            // Ramp min/max tree size over duration of evo run.
            double fraction = float(i) / max_evolution_steps;
            double max_tree_size = util::interpolate(fraction,
                                                     float(start_max_tree_size),
                                                     float(end_max_tree_size));
            fs.setMaxTreeSize(std::round(max_tree_size));
            population->setMaxCrossoverTreeSize(std::round(max_tree_size));
            std::cout << "    ";  // "log_prefix"
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20260608 adjust centerMaxDist() to maintain constant boid density
//            debugPrint(max_tree_size);
//            if (start_max_tree_size != end_max_tree_size)
//            {
//                debugPrint(max_tree_size);
//            }
            if (EF::usingGP()) { debugPrint(max_tree_size); }
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            std::cout << std::endl;
            
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
        int previous_obpf = override_boids_per_flock;
        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
        // TODO 20260417 combine computing centroid and setting it in all boids
//        if (murmuration_mode) { override_boids_per_flock = 500; }
        if (murmuration_mode) { override_boids_per_flock = 300; }
        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
        GP::run_flock_simulation(individual, 1);
        override_boids_per_flock = previous_obpf;
        Draw::getInstance().setEnable(previous_draw_enable_state);
        enable_multithreading = previous_emt_state;
        draw.clearVisBestMode();
    }
}

//~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
// TODO 20260625 annotation selected boid's steerTowardCentroid()
//bool pauseBeforeVisualizePreviousFS = false;
bool pauseBeforeVisualizePreviousFS = true;
//~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~

// Tool to (optionally) visualize a previous logged result. For example, after
// an overnight evolution has completed, a result can be copied and pasted from
// the log into this function. Works for either GA FlockParameters or for GP
// tree/program's "source code." To visualize a given result: cut/paste from log
// into the body of this function (following the examples below) and comment out
// the "return" statement at the top. This function will then loop forever
// running the logged result.
void visualizePreviouslyLoggedFlockParameters(const LP::FunctionSet& fs)
{
    // Do nothing unless in visualize_previous_results_mode.
    if (not visualize_previous_results_mode) { return; }
    
    Draw& draw = Draw::getInstance();
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20260601 test 20260601_ga_murm_no_out_vel with bigger flock
    std::cout<<"In EF::visualizePreviouslyLoggedFlockParameters()"<<std::endl;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
        // FlockParameters fp(91.1072, 98.6429, 51.8598, 94.8959, 21.8851, 22.2565,
        //                    5.87154, 2.96989, 92.1022, 12.3065, -0.805401,
        //                    -0.112003, -0.74847, 63.9714, 3.51265);
        
        // Saved FP values from run 20260216_ga_no_obs_no_curve
        // FlockParameters fp(94.7771, 94.2979, 42.4511, 71.7522, 23.1819, 8.37409,
        //                    82.8308, 3.31166, 98.9976, 9.43775, -0.932242,
        //                    -0.702709, -0.867646, 68.5945, 3.52568);
        
        // Saved FP values from run 20260219_ga_no_obs_rand_init_vel_3
        // FlockParameters fp(80.8611, 96.1782, 43.7249, 88.7047, 13.5773,
        //                    43.4836,25.946, 2.92637, 93.1734, 18.2696,
        //                    -0.919836, -0.757193, -0.193814, 77.0225, 3.97704);

        // Saved FP values from run 20260221_ga_new_ret_to_center
        // FlockParameters fp(93.6905, 96.6739, 50.5408, 79.4269, 5.37308,
        //                    31.3867, 5.28867, 3.95184, 77.7457, 94.3708,
        //                    -0.799625, -0.381311, 0.459778, 89.8753, 7.80046);
        
        // Saved FP values from run 20260222_ga_new_sep_interval
        // FlockParameters fp(94.7861, 97.4638, 80.4171, 98.9001, 4.41346,
        //                    4.37725, 3.68332, 5.16365, 41.8653, 72.6479,
        //                    -0.959596, -0.644678, -0.922466, 23.399, 9.32508);
        
        // Saved FP values from run 20260223_ga_no_obs_with_clusters
        // FlockParameters fp(97.8651, 85.5172, 53.1979, 35.4656, 18.1132,
        //                    70.676, 37.4797, 4.60951, 9.27746, 11.7462,
        //                    -0.802592, -0.763088, -0.319223, 23.1518, 3.91158);
        
        // Saved FP values from run 20260223_ga_no_obs_with_clusters_2
        // FlockParameters fp(57.9653, 97.7476, 45.4349, 46.2576, 26.0211,
        //                    83.7814, 67.2956, 2.29827, 7.20329, 11.3791,
        //                    -0.768295, -0.859752, -0.775641, 85.0299, 3.03248);
        
        // Saved FP values from run 20260302_ga_no_obs_fix_PO
        // FlockParameters fp(93.7918, 97.5165, 83.4214, 1.16525, 41.807,
        //                    96.9971, 72.9859, 3.73492, 69.2881, 52.8236,
        //                    -0.37416, 0.813266, 0.448999, 56.8067, 1.5802);
        
        // Saved FP values from run 20260305_ga_murm+curve+cluster
        // FlockParameters fp(97.3895, 94.8641, 67.1557, 10.691, 78.6854,
        //                    92.6751, 84.6933, 36.453, 26.2615, 70.0312,
        //                    0.479107, 0.93305, -0.770313, 8.49056, 0.721249);
        
        // Saved FP values from run 20260306_ga_murm_40x_vol+curve+cluster
        // FlockParameters fp(70.4853, 91.1533, 60.5059, 45.6154, 34.6533,
        //                    91.0774, 96.0671, 2.73925, 4.86185, 20.5608,
        //                    -0.62327, 0.2073, 0.340471, 6.76677, 1.19798);

        // Saved FP values from run 20260418_ga_murm_param_tweak
        // FlockParameters fp(14.3525, 98.2618, 67.4173, 84.6732, 15.0438,
        //                    71.0199, 21.2444, 2.2782, 62.0336, 96.8648,
        //                    -0.905143, -0.906197, -0.318942, 67.6338, 1.27933);

        // Saved FP values from run 20260505_ga_murm_evo_param
        // FlockParameters fp({94.6537, 97.1757, 76.4246, 89.4007, 11.7499,
        //                     6.80415, 92.7606, 2.43355, 14.4502, 5.96147,
        //                     -0.424488, -0.125247, -0.886847, 69.5286,
        //                     3.75423, 49.5624, 36.2599, 92.8005});
        
        // Saved best FP values from run 20260511_ga_murm_min_frac
        // FlockParameters fp({80.6664, 84.5755, 80.7222, 68.0803, 26.7053,
        //                     36.1821, 9.62825, 2.05975, 89.7731, 90.1345,
        //                     -0.965851, 0.356241, -0.184357, 91.0539,
        //                     9.86365, 46.5262, 0.700153, 85.2107});
        
        // Saved best FP values from run 20260512_ga_murm_use_expt
        // FlockParameters fp({94.7641, 91.9363, 63.7496, 98.1078, 9.6187,
        //                     77.7354, 96.0103, 2.1662, 89.3659, 12.7148,
        //                     -0.883542, 0.0379527, -0.68233, 89.7384,
        //                     4.59969, 48.6678, 4.91045, 69.8889});

        // Saved best FP values from run 20260514_ga_murm_dyna_center
        // FlockParameters fp({72.5795, 92.6647, 74.488, 92.9831, 21.9784,
        //                     18.5211, 1.67698, 2.73893, 74.3763, 13.6239,
        //                     -0.507361, 0.116378, -0.796355, 23.0692,
        //                     2.25646, 49.6829, 2.40116, 41.5759});

        // Saved best FP values from run 20260515_ga_murm_no_curve_obj
        // FlockParameters fp({44.675, 94.5187, 39.4083, 39.8856, 6.08479,
        //                     83.4592, 86.7651, 2.31746, 5.12713, 55.4686,
        //                     -0.97556, -0.272121, -0.167653, 34.4743,
        //                     7.74094, 47.9288, 2.70664, 49.1599});

        // Saved best FP values from run 20260516_ga_murm_tweak_param_range
        // FlockParameters fp({88.9612, 71.7194, 42.074, 0.84605, 66.7022,
        //                     3.57806, 50.4041, 49.1, 5.60767, 58.5394,
        //                     0.615671, 0.976181, 0.466433, 93.8883,
        //                     3.59881, 29.9343, 1.96593, 16.1432});
        
        // Saved best FP values from run 20260517_ga_murm_tweak_param_range
        // FlockParameters fp({97.8688, 99.0719, 59.3638, 85.193, 10.8989,
        //                     3.10101, 13.1836, 2.26472, 77.4632, 92.831,
        //                     -0.863476, 0.0546438, -0.807519, 48.5438,
        //                     6.48802, 49.7779, 1.83679, 39.5206});

        // Saved best FP values from run 20260518_ga_murm_expt
        // FlockParameters fp({80.8952, 88.0454, 75.3132, 1.71313, 98.4244,
        //                     92.2101, 95.0413, 22.9778, 0.636451, 71.2487,
        //                     -0.769009, 0.898582, -0.136664, 90.4239,
        //                     3.38967, 47.2217, 2.83461, 56.3936});

        // Saved best FP values from run 20260520_ga_murm_no_random_centroid
        // FlockParameters fp({76.6822, 95.7527, 50.3573, 75.9461, 10.9027,
        //                     23.1812, 13.6745, 2.36192, 6.5394, 85.6075,
        //                     -0.936932, 0.0371708, -0.532074, 3.49794,
        //                     1.3531, 48.2294, 1.90271, 30.9173});

        // Saved best FP values from run 20260521_ga_murm_centroid_velocity
        // FlockParameters fp({81.8399, 93.0806, 36.0119, 43.931, 15.0976,
        //                     24.1014, 17.2065, 2.48293, 5.442, 84.9158,
        //                     -0.876558, -0.988123, 0.204049, 75.3648,
        //                     6.07002, 48.2284, 1.52215, 20.0814});

        // Saved best FP values from run 20260522_ga_murm_new_score
        // FlockParameters fp({80.5681, 93.185, 51.197, 64.3521, 10.1647,
        //                     30.1383, 37.6609, 2.21258, 35.4355, 72.4732,
        //                     -0.997584, -0.786088, 0.979533, 61.0075,
        //                     4.65929, 47.2678, 1.43407, 62.5916});

        // Saved best FP values from run 20260523_ga_murm_refactored
        // FlockParameters fp({86.9914, 98.9878, 45.7494, 66.1938, 7.91513,
        //                     69.1745, 8.13903, 2.16534, 73.3584, 94.5009,
        //                     -0.908106, -0.939498, 0.934986, 57.5063,
        //                     9.40786, 47.935, 1.40237, 91.5868});

        // Saved best FP values from run 20260523_ga_murm_no_velocity
        // FlockParameters fp({67.6349, 97.0842, 51.0182, 59.9569, 5.80385,
        //                     5.10904, 21.0246, 2.01136, 5.57006, 51.3922,
        //                     -0.931831, -0.916835, 0.269348, 74.2693,
        //                     7.19527, 48.1632, 1.9225, 69.3859});
        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20260526 test 20260526_ga_murm_adj_escape_heading with bigger flock
        
        // Saved best FP values from run 20260523_ga_murm_curvature_on
        // FlockParameters fp({64.8864, 94.1601, 43.1356, 42.3645, 8.80777,
        //                     49.7421, 14.0622, 2.25498, 6.28338, 66.4416,
        //                     -0.863101, -0.67368, 0.275353, 88.4203,
        //                     5.66381, 49.9535, 1.93114, 69.1544});

        // Saved best FP values from run 20260525_ga_murm_no_escape_vel
        // FlockParameters fp({61.3092, 88.0454, 32.771, 25.5647, 18.3933,
        //                     55.9217, 84.2697, 2.6064, 81.129, 22.2795,
        //                     -0.973852, -0.711805, 0.175394, 75.6662,
        //                     5.98547, 49.7585, 1.97701, 8.46627});
        
        // Saved best FP values from run 20260526_ga_murm_adj_escape_heading
        // FlockParameters fp({61.3092, 88.0454, 32.771, 25.5647, 18.3933,
        //                     55.9217, 84.2697, 2.6064, 81.129, 22.2795,
        //                     -0.973852, -0.711805, 0.175394, 75.6662,
        //                     5.98547, 49.7585, 1.97701, 8.46627});
        
        // Saved best FP values from run 20260527_ga_murm_more_centering
        // FlockParameters fp({31.0392, 92.6025, 59.7564, 61.9005, 27.1931,
        //                     42.9833, 21.6326, 2.54932, 10.0939, 60.52,
        //                     -0.979254, 0.0482417, -0.460304, 70.5717,
        //                     8.16347, 47.1778, 1.91867, 2.91618});

        // Saved best FP values from run 20260530_ga_murm_new_objective
        // FlockParameters fp({27.9975, 97.5234, 82.8278, 67.8816, 18.136,
        //                     24.2784, 31.5087, 2.78594, 14.7445, 37.0759,
        //                     -0.328644, -0.328231, -0.0754814, 36.6902,
        //                     6.70107, 48.3511, 1.82985, 0.599145});

        // Saved best FP values from run 20260601_ga_murm_no_out_vel
        // FlockParameters fp({29.777, 88.1738, 67.666, 38.9277, 18.0599,
        //                     97.265, 91.8457, 2.39932, 28.8533, 18.2052,
        //                     -0.782398, -0.379199, 0.0668237, 75.1048,
        //                     7.68098, 47.8333, 1.9006, 0.88659});

        // Saved best FP values from run 20260602_ga_murm_3_new_fp
        // FlockParameters fp({23.5875, 73.2938, 38.4922, 67.4696, 7.50207,
        //                     80.6678, 96.5334, 2.41975, 52.9937, 76.9248,
        //                     -0.887382, -0.085909, -0.829808, 65.6678,
        //                     2.73662, 44.9387, 1.90384, 4.75753, 0.913459,
        //                     0.251085, 0.501427});

        // Saved best FP values from run 20260603_ga_murm_reverse_slowing
        // FlockParameters fp({43.1343, 94.9653, 43.8608, 43.7723, 13.9062,
        //                     63.1034, 75.5174, 2.39903, 5.65152, 27.9135,
        //                     -0.823712, -0.588954, 0.228872, 24.2207,
        //                     1.00104, 44.7399, 1.92882, 6.27248, 0.311584,
        //                     0.0351969, 0.157518});

        // Saved best FP values from run 20260607_ga_murm_adjust_density
        // FlockParameters fp({29.1696, 74.6058, 32.9445, 40.1742, 5.07739,
        //                     45.4729, 21.9687, 2.5749, 14.2125, 66.3719,
        //                     -0.797268, -0.777954, 0.504381, 88.6072,
        //                     6.565, 39.9455, 1.50628, 2.74551, -0.557402,
        //                     0.0494514, 0.364301});

        // Saved best FP values from run 20260607_ga_murm_adjust_density
        // FlockParameters fp({41.3796, 92.344, 42.9878, 54.2326, 17.2815,
        //                     81.733, 96.2759, 2.2627, 97.0137, 62.6568,
        //                     -0.965195, -0.873262, 0.331639, 7.36768,
        //                     4.63413, 14.4471, 1.84435, 7.26177, -0.924752,
        //                     0.0165574, 0.329412});
        
        // Saved best FP values from run 20260609_ga_murm_ignore_unaligned
        // FlockParameters fp({37.8103, 92.9423, 50.351, 71.7843, 5.39694,
        //                     38.5116, 92.7048, 2.08736, 25.3785, 52.5458,
        //                     -0.506184, -0.854214, 0.869957, 69.5605, 6.83093,
        //                     23.9512, 1.89026, 9.93311, -0.734261, 0.0753744,
        //                     0.178261});

        // Saved best FP values from run 20260611_ga_murm_fix_for_sep
        // FlockParameters fp({33.1629, 99.6163, 0.0665019, 34.3899, 55.5842,
        //                     1.6567, 14.6102, 68.4548, 40.9983, 1.48856,
        //                     0.30279, -0.0640597, -0.0371724, 11.8859,
        //                     1.78334, 9.46372, 1.11051, 5.63369, -0.270475,
        //                     0.0150335, 0.0135678});


        // Saved best FP values from run 20260611_ga_murm_no_sac_fix
        // FlockParameters fp({43.9872, 86.9619, 67.0914, 71.2227, 17.2915,
        //                     81.0617, 73.2342, 2.02347, 33.498, 59.5138,
        //                     -0.996257, -0.578367, 0.254471, 86.6538,
        //                     9.3115, 26.8014, 1.63077, 5.62702, -0.931885,
        //                     0.0811591, 0.247335});

        // Saved best FP values from run 20260612_ga_murm_donut_hole_axis
        // FlockParameters fp({34.1201, 94.6847, 68.1119, 55.6504, 14.3364,
        //                     91.7654, 75.7028, 2.08918, 33.2772, 83.0976,
        //                     -0.990558, -0.283315, -0.440062, 39.5169,
        //                     2.28927, 15.0282, 1.55601, 5.24907, -0.458992,
        //                     0.150057, 0.466543});
        
        // Saved best FP values from run 20260616_ga_murm_anti_donut
        // FlockParameters fp({42.9423, 98.8431, 87.4705, 77.2623, 10.8257,
        //                     34.5899, 34.3558, 2.4724, 31.2697, 53.6106,
        //                     -0.787378, 0.0609405, 0.590016, 56.743,
        //                     8.51073, 23.6691, 1.91501, 5.32095, -0.753097,
        //                     0.0329083, 0.204359});
        
        // Saved best FP values from run 20260616_ga_murm_anti_donut_0_2
        // FlockParameters fp({39.7655, 77.3154, 65.3954, 25.0801, 43.0818,
        //                     7.67208, 81.9496, 2.4321, 81.9055, 94.994,
        //                     -0.225381, -0.859306, -0.376802, 94.6896,
        //                     8.20399, 47.5827, 1.76824, 6.1069, -0.87459,
        //                     0.0357764, 0.261688});

        // Saved best FP values from run 20260617_ga_murm_InNess_non_neg
        // FlockParameters fp({75.5789, 97.6831, 38.4632, 79.9093, 7.44685,
        //                     32.2224, 38.8311, 3.05728, 15.041, 47.1982,
        //                     -0.811191, -0.33146, -0.37649, 85.6968,
        //                     1.53713, 36.4044, 1.87102, 2.97235, 0.295929,
        //                     0.139953, 0.247997});

        // Saved best FP values from run 20260618_ga_murm_no_curve_boost
        // FlockParameters fp({49.6018, 79.3125, 43.4539, 21.25, 39.2122,
        //                     57.8057, 45.8854, 2.27324, 16.4968, 1.70522,
        //                     -0.950944, -0.187334, -0.14328, 93.4128,
        //                     0.365626, 48.1672, 1.77244, 1.0194, 0.272146,
        //                     0.104974, 0.0928162});

        // Saved best FP values from run 20260619_ga_murm_stop_density_adj
        // FlockParameters fp({18.8782, 94.4679, 85.1808, 16.9288, 65.2134,
        //                     30.3057, 91.3098, 2.71957, 16.5376, 35.9854,
        //                     0.252984, -0.217378, 0.129258, 57.634, 9.93525,
        //                     48.2666, 1.89866, 14.9117, 0.811644, 0.0764724,
        //                     0.195077});
        
        // Saved best FP values from run 20260621_ga_murm_fix_initPose
        // FlockParameters fp({25.8798, 91.128, 66.7921, 73.1955, 19.0353,
        //                     5.58319, 26.3711, 2.58225, 33.3171, 51.912,
        //                     -0.777807, -0.921582, -0.840182, 29.7588,
        //                     8.6607, 37.3203, 1.95872, 12.5221, 0.297009,
        //                     0.142382, 0.206578});

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20260601 test 20260624_ga_murm_face_outward with bigger flock
        
        // Saved best FP values from run 20260624_ga_murm_face_outward
        FlockParameters fp({38.2494, 96.4766, 32.6476, 40.5122, 12.836,
                            59.8341, 27.8042, 2.10652, 76.5876, 66.4315,
                            -0.90675, 0.285309, -0.0509794, 75.4455,
                            1.03952, 16.0169, 1.82491, 6.10348, 0.517219,
                            0.172052, 0.155188});

        // TODO visualize_previous_results_mode
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        EF::enable_multithreading = false;
        draw.setEnable(true);
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
        draw.setEnable(true);

        //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
        // TODO 20260625 annotation selected boid's steerTowardCentroid()
        while (true)
        {
            if (pauseBeforeVisualizePreviousFS) { draw.simPause() = true; }
            GP::run_flock_simulation(&individual, 1);
        }
        //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
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
