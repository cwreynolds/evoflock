//------------------------------------------------------------------------------
//
//  GP.h -- new flock experiments
//
//  GP class, specialization of LazyPredator classes to use Genetic Programming
//  for evolutionary optimization of flocking parameters.
//
//  Created by Craig Reynolds on February 23, 2024.
//  (Borrows from GP.h in TexSyn.)
//  MIT License -- Copyright © 2024 Craig Reynolds
//------------------------------------------------------------------------------

#pragma once

#include "Draw.h"
#include "flock.h"
#include "shape.h"
// TODO 20240226 For now, a modified copy of LazyPredator is in a subdirectory.
#include "LazyPredator/LazyPredator.h"
namespace LP = LazyPredator;

namespace GP
{

// Abbreviated name for this overly-long class name.
typedef LazyPredator::MultiObjectiveFitness MOF;

//    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
//    // TODO 20250912 is GpFunc "Run_Flock" return value used?
//
//    //    // Fitness function, simply returns Individual's tree's value (computing it and
//    //    // caching it on first call).
//    //    inline MOF evoflock_ga_fitness_function(LP::Individual* individual)
//    //    {
//    //        return std::any_cast<MOF>(individual->tree().getRootValue());
//    //    }
//
//    // Fitness function, simply returns Individual's tree's value (computing it and
//    // caching it on first call).
//    inline MOF evoflock_ga_fitness_function(LP::Individual* individual)
//    {
//
//        std::cout << "in GP::evoflock_ga_fitness_function()" << std::endl;
//
//        return run_ga_gp_flock_simulation(individual);
//
//    }
//
//    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~


// Take the minimum element of a MultiObjectiveFitness ("Shangian scalarizer").
inline double scalarize_fitness_min(MOF mof) { return mof.min(); }


// Take the product of MultiObjectiveFitness elements.
inline double scalarize_fitness_prod(MOF mof)
{
    return std::reduce(mof.begin(), mof.end(), 1.0, std::multiplies());
}

// Take the product of MultiObjectiveFitness elements.
inline double scalarize_fitness_length(MOF mof)
{
    auto sum_of_sq = [](double sum, double val){ return sum + val * val; };
    double length_sq = std::accumulate(mof.begin(), mof.end(), 0.0, sum_of_sq);
    return (std::sqrt(length_sq) / std::sqrt(mof.size()));
}

// Take hypervolume (basically the product) of MultiObjectiveFitness elements.
inline double scalarize_fitness_hyperVolume(MOF mof) {return mof.hyperVolume();}

// Map a MultiObjectiveFitness to a scalar. Used as the FitnessScalarizeFunction
// for Population::evolutionStep(). Usually one of scalarize_fitness_min(),
// scalarize_fitness_prod(), or scalarize_fitness_length();
//inline std::function<double(MOF)> scalarize_fitness = scalarize_fitness_min;
inline std::function<double(MOF)> scalarize_fitness = scalarize_fitness_hyperVolume;


//~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
// TODO 20250902 Assertion failed: (mof.size() == mof_names().size())

//    inline std::vector<std::string> mof_names()
//    {
//        return (EF::usingGP() ?
//                std::vector<std::string>(
//                                         {
//                                             "avoid",
//                                             "separate",
//                                             "cohere",
//                                             "cluster",
//                                             "occupied",
//
//                                         }) :
//                (EF::add_curvature_objective ?
//                 std::vector<std::string>(
//                                          {
//                                              "avoid",
//                                              "separate",
//                                              "speed",
//                                              "curvature"
//                                          }) :
//                 std::vector<std::string>(
//                                          {
//                                              "avoid",
//                                              "separate",
//                                              "speed"
//                                          }))
//                );
//    }

inline std::vector<std::string> mof_names()
{
    return (EF::add_curvature_objective ?
             std::vector<std::string>(
                                      {
                                          "avoid",
                                          "separate",
                                          "speed",
                                          "curvature"
                                      }) :
             std::vector<std::string>(
                                      {
                                          "avoid",
                                          "separate",
                                          "speed"
                                      }));
}

//~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~


// After a Flock's simulation has been run, it is passed here to build its multi
// objective fitness object from metrics saved inside the Flock object.
inline MOF multiObjectiveFitnessOfFlock(const Flock& flock)
{
    return (EF::add_curvature_objective ?
            MOF(
               {
                   flock.obstacleCollisionsScore(),
                   flock.separationScore(),
                   flock.speedScore(),
                   flock.curvatureScore()
               }) :
            MOF(
                {
                    flock.obstacleCollisionsScore(),
                    flock.separationScore(),
                    flock.speedScore()
                })
            );
}

// Initialize basic run parameters of Flock object
inline void init_flock(Flock& flock)
{
    flock.set_fixed_time_step(true);
    flock.set_fixed_fps(flock.fp().getFPS());
    flock.set_boid_count(flock.fp().boidsPerFlock());
    flock.set_max_simulation_steps(flock.fp().maxSimulationSteps());
    flock.log_prefix = "    ";
}


// Adjust weight of one objective's fitness for "product of objective fitnesses".
// Each objective fitness is on [0, 1] and serves to modulate (decrease) other
// objective fitnesses. This reduces the range of decrease by mapping the input
// fitness from [0, 1] to [1-w, 1], that is, restricting it to the top of the
// range, hence limiting its contribution for smaller weights.
double fitness_product_weight_01(double fitness, double weight)
{
    return util::remap_interval_clip(fitness, 0, 1, 1 - weight, 1);
};

inline void fitness_logger(const MOF& mof)
{
    // Save current settings for formatting float/double values
    std::ios::fmtflags old_settings = std::cout.flags();
    size_t old_precision = std::cout.precision();
    // Format labels.
    std::string sc = "scalar composite";
    size_t cw = sc.size();  // Column width.
    std::vector<std::string> labels;
    for (auto& s : mof_names()) { labels.push_back(s + " fitness"); }
    for (auto& s : labels){ size_t ss = s.size(); if (cw < ss) { cw = ss; } }
    // Print one row of a table with a named mof fitness.
    auto print = [&](double fitness, std::string name)
    {
        std::cout << "    " << (name + std::string(cw,' ')).substr(0, cw);
        std::cout << std::setprecision(6) << std::setw(10) << std::fixed;
        std::cout << fitness << std::endl;
    };
    for (int i = 0; i < mof.size(); i++) { print(mof.at(i), labels.at(i)); }
    print(scalarize_fitness(mof), sc);
    std::cout << std::endl;
    // restore output format flags and precision
    std::cout.flags(old_settings);
    std::cout.precision(old_precision);
}

//~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
// TODO 20250913 change GpType for GpFunc "Run_Flock"


//    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//    // TODO 20250911 refactor run_flock_simulation() to include GP in addition to GA
//
//    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//    // TODO 20240628 can we do an eval of a const tree?
//    #ifdef eval_const_20240628
//    FlockParameters fp_from_ga_tree(const LazyPredator::GpTree& tree)
//    #else  // eval_const_20240628
//    FlockParameters fp_from_ga_tree(LazyPredator::GpTree& tree)
//    #endif // eval_const_20240628
//    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//    {
//        assert(EF::usingGA());
//        std::vector<double> parameters;
//        for (int i = 0; i < FlockParameters::tunableParameterCount(); i++)
//        {
//            parameters.push_back(tree.evalSubtree<double>(i));
//        }
//        return FlockParameters(parameters);
//    }
//
//
//    FlockParameters fp_from_ga_individual(LP::Individual* individual)
//    {
//        LP::GpTree tree = individual->tree();
//        return fp_from_ga_tree(tree);
//    }
//
//    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20250911 refactor run_flock_simulation() to include GP in addition to GA

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240628 can we do an eval of a const tree?
#ifdef eval_const_20240628
FlockParameters fp_from_ga_tree(const LazyPredator::GpTree& tree)
#else  // eval_const_20240628
FlockParameters fp_from_ga_tree(LazyPredator::GpTree& tree)
#endif // eval_const_20240628
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
{
    assert(EF::usingGA());
    std::vector<double> parameters;
    for (int i = 0; i < FlockParameters::tunableParameterCount(); i++)
    {
        parameters.push_back(tree.evalSubtree<double>(i));
    }
    return FlockParameters(parameters);
}


FlockParameters fp_from_ga_individual(LP::Individual* individual)
{
    LP::GpTree tree = individual->tree();
    return fp_from_ga_tree(tree);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




//~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~





//~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
// TODO 20250918 back to 30000, smaller trees, GpFunc To_Forward() To_Side()

//    double clean_num(double x)
//    {
//        bool unclean = (std::isnan(x) or
//                        std::isinf(x) or
//                        x < std::numeric_limits<double>::min());
//        return (unclean ? 0 : x);
//    }


//    double clean_num(double x)
//    {
//        bool unclean = (std::isnan(x) or
//                        std::isinf(x) or
//                        x < std::numeric_limits<double>::min()
//                        or
//                        std::abs(x) < 0.00000000001);
//        return (unclean ? 0 : x);
//    }

//    auto vlimiter = [](Vec3& v)
//    {
//        auto limiter = [](double x)
//        {
//            bool questionable = ((std::abs(x) < 0.00000000001) or
//                                 (x < -10000000000) or
//                                 (x > 10000000000));
//            return questionable ? 0 : x;
//        };
//
//        return Vec3(limiter(v.x()), limiter(v.y()), limiter(v.z()));
//    };


// These "cleaners" are to prevent ludicrous values from evolved trees.
double clean_num(double x)
{
    bool unclean = (std::isnan(x) or
                    std::isinf(x) or
                    (x < std::numeric_limits<double>::min()) or
                    (std::abs(x) < 0.00000000001) or
                    (x < -10000000000) or
                    (x > +10000000000));
    return (unclean ? 0 : x);
}

// These "cleaners" are to prevent ludicrous values from evolved trees.
Vec3 clean_vec3(Vec3 v)
{
    return { clean_num(v.x()), clean_num(v.y()), clean_num(v.z()) };
}

//~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~







//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20250913 change GpType for GpFunc "Run_Flock"

//~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
// TODO 20250911 refactor run_flock_simulation() to include GP in addition to GA


// TODO rewrite:
// Run flock simulation with given parameters "runs" times and returns the MOF
// with the LEAST scalar fitness score.

//~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
// TODO 20250922 WIP switch to "evolved program returns local vec


//    //inline MOF run_flock_simulation(const FlockParameters& fp, int runs = 4)
//    //inline MOF run_ga_gp_flock_simulation(LP::Individual* individual)
//    inline MOF run_ga_gp_flock_simulation(LP::Individual* individual, int runs = 4)
//    {
//    //    int runs = 4;
//    //    FlockParameters fp;
//    //    if (EF::usingGA()) { fp = fp_from_ga_individual(individual); }
//
//        MOF least_mof;
//        double least_scalar_fitness = std::numeric_limits<double>::infinity();
//        std::vector<double> scalar_fits;
//        std::mutex save_mof_mutex;
//
//        // Perform one simulation run, and record results.
//        auto do_1_run = [&]()
//        {
//            // These steps can happen in parallel threads:
//            Flock flock;
//            init_flock(flock);
//
//
//    //    //        FlockParameters fp;
//    //            if (EF::usingGA())
//    //            {
//    //                flock.fp() = fp_from_ga_individual(individual);
//    //            }
//    //    //        flock.fp() = fp;
//
//            if (EF::usingGA())
//            {
//                // For GA, set Flock's FlockParameters.
//                flock.fp() = fp_from_ga_individual(individual);
//            }
//            else
//            {
//                // For GP, set Flock's override_steer_function_.
//                flock.override_steer_function_ = [&]()
//                {
//                    LP::GpTree gp_tree = individual->tree();
//                    Vec3 steering = std::any_cast<Vec3>(gp_tree.eval());
//                    if (not steering.is_valid()) { steering = Vec3(); }
//
//                    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
//                    // TODO 20250918 back to 30000, smaller trees, GpFunc To_Forward() To_Side()
//
//                    // TODO in FlockParameters this ranges up to 100, so this clip may be too aggressive
//
//                    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
//                    // TODO 20250919 move all "data cleaning" to GP module
//
//                    // Ran a test in GA mode. Max steering force length was 1000.
//                    double max_steering_length = 1000;
//                    steering = steering.truncate(max_steering_length);
//
//                    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
//
//                    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
//
//                    double min_steering_length = 0.00001;
//                    if (steering.length() < min_steering_length) { steering = Vec3(); }
//
//                    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
//                    // TODO 20250918 back to 30000, smaller trees, GpFunc To_Forward() To_Side()
//
//                    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
//                    // TODO 20250919 move all "data cleaning" to GP module
//
//
//
//                    // Just give a 10% push in the forward direction, to prevent them
//                    // from sitting at their initial position at zero speed.
//    //                if (speed() < 15)
//                    Boid* b = Boid::getGpPerThread();
//                    if (b->speed() < 15) // TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//                    {
//    //                    steering += forward() * max_force() * 0.1;
//                        steering += b->forward() * b->max_force() * 0.1;
//                    }
//
//
//
//    //                return steering;
//                    return clean_vec3(steering);
//
//
//                    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
//
//                    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
//
//                };
//            }
//
//
//
//            flock.run();
//            MOF mof = multiObjectiveFitnessOfFlock(flock);
//            // These steps happen in the single thread with lock on save_mof_mutex.
//            {
//                std::lock_guard<std::mutex> smm(save_mof_mutex);
//                assert(mof.size() == mof_names().size());
//                scalar_fits.push_back(scalarize_fitness(mof));
//                if (least_scalar_fitness > scalar_fits.back())
//                {
//                    least_scalar_fitness = scalar_fits.back();
//                    least_mof = mof;
//                }
//                // Store these stats on the "current individual"
//                LP::Individual* i = LP::Population::evolution_step_individual;
//                if (i)
//                {
//                    std::vector<double>& udfp = i->user_data_for_plotting;
//                    udfp.clear();
//                    udfp.push_back(flock.obstacleCollisionsScore());
//                    udfp.push_back(flock.separationScore());
//                    udfp.push_back(flock.speedScore());
//                }
//            }
//        };
//
//        // Occasionally poll the Draw GUI to check for events esp the "B" command.
//        Draw::getInstance().pollEvents();
//
//    //    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//    //    // TODO 20250909 does turning of multithreading help?
//    //
//    //    // TODO 20250909 AHA?! in GP-but-not-multithreading mode this is not reached
//    //    {
//    //        grabPrintLock_evoflock();
//    //        std::cout << std::endl;
//    //        std::cout << "-----------------------------------------------" << std::endl;
//    //        debugPrint(EF::unify_GA_GP);
//    //        debugPrint(EF::usingGP());
//    //        debugPrint(EF::enable_multithreading);
//    //        debugPrint(Draw::getInstance().enable());
//    //        std::cout << "-----------------------------------------------" << std::endl;
//    //        std::cout << std::endl;
//    //
//    //    }
//    //
//    //    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//        if (EF::enable_multithreading)
//        {
//            bool previous_enable_state = Draw::getInstance().enable();
//            Draw::getInstance().setEnable(false);
//
//            // Do each simulation run in a parallel thread.
//            std::vector<std::thread> threads;
//            for (int r = 0; r < runs; r++) { threads.push_back(std::thread(do_1_run)); }
//            // Wait for helper threads to finish, join them with this thread.
//            for (auto& t : threads) { t.join(); }
//
//            Draw::getInstance().setEnable(previous_enable_state);
//        }
//        else
//        {
//            // Do each simulation run sequentially.
//            for (int r = 0; r < runs; r++) { do_1_run(); }
//        }
//
//        assert(scalar_fits.size() == runs);
//        fitness_logger(least_mof);
//        std::cout << "    min composite "<< least_scalar_fitness;
//        std::cout << "  {" << LP::vec_to_string(scalar_fits) << "}";
//        std::cout << std::endl << std::endl;
//        return least_mof;
//    }

//~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
// TODO 20251001 investigate low speed score with ONLY Speed_Control GpFunc

//    inline MOF run_ga_gp_flock_simulation(LP::Individual* individual, int runs = 4)
//    {
//        MOF least_mof;
//        double least_scalar_fitness = std::numeric_limits<double>::infinity();
//        std::vector<double> scalar_fits;
//        std::mutex save_mof_mutex;
//
//        // Perform one simulation run, and record results.
//        auto do_1_run = [&]()
//        {
//            // These steps can happen in parallel threads:
//            Flock flock;
//            init_flock(flock);
//            if (EF::usingGA())
//            {
//                // For GA, set Flock's FlockParameters from evolved "tree".
//                flock.fp() = fp_from_ga_individual(individual);
//            }
//            else
//            {
//                // For GP, set Flock's override_steer_function_.
//                flock.override_steer_function_ = [&]()
//                {
//                    Boid* boid = Boid::getGpPerThread();
//                    LP::GpTree gp_tree = individual->tree();
//
//                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                    // TODO 20250925 turn off curriculum learning experiment
//
//    //                if (boid->isSelected())
//    //                if (boid->isSelected() and (flock.clock().frameCounter() == 1))
//    //                {
//    //                    grabPrintLock_evoflock();
//    //                    std::cout << "====> ";
//    //                    debugPrint(boid->speed());
//    //                }
//
//                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//
//    //                Vec3 steering = std::any_cast<Vec3>(gp_tree.eval());
//                    // TEMP: here we are assuming GpTree returns a local steer vec
//                    Vec3 local_steering = std::any_cast<Vec3>(gp_tree.eval());
//                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                    // TODO 20250924 add LocalSpace::globalizeDirection()
//
//    //                Vec3 steering = boid->ls().globalizePosition(local_steering);
//                    Vec3 steering = boid->ls().globalizeDirection(local_steering);
//
//                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//                    // KEEP? added while tracking down "cleaning" issues
//                    if (not steering.is_valid()) { steering = Vec3(); }
//
//                    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
//                    // TODO 20250926 lower limit on max steering force 100 -> 10
//
//                    // Ran a test in GA mode. Max steering force length was 1000.
//    //                double max_steering_length = 1000;
//                    // WIP reduce by an order of magnitude, close to max_force()
//    //                double max_steering_length = 100;
//                    double max_steering_length = 10;
//                    steering = steering.truncate(max_steering_length);
//
//                    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
//                    // KEEP? added while tracking down "cleaning" issues
//                    double min_steering_length = 0.00001;
//                    if (steering.length() < min_steering_length) { steering = Vec3(); }
//
//                    // Just give a 10% push in the forward direction, to prevent them
//                    // from sitting at their initial position at zero speed.
//    //                Boid* b = Boid::getGpPerThread();
//    //                if (b->speed() < 15) // TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//    //                {
//    //                    steering += b->forward() * b->max_force() * 0.1;
//    //                }
//    //                if (boid->speed() < 15) // TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//    //                {
//    //                    steering += boid->forward() * boid->max_force() * 0.1;
//    //                }
//                    return clean_vec3(steering);
//                };
//            }
//
//            flock.run();
//            MOF mof = multiObjectiveFitnessOfFlock(flock);
//            // These steps happen in the single thread with lock on save_mof_mutex.
//            {
//                std::lock_guard<std::mutex> smm(save_mof_mutex);
//                assert(mof.size() == mof_names().size());
//                scalar_fits.push_back(scalarize_fitness(mof));
//                if (least_scalar_fitness > scalar_fits.back())
//                {
//                    least_scalar_fitness = scalar_fits.back();
//                    least_mof = mof;
//                }
//                // Store these stats on the "current individual"
//                LP::Individual* i = LP::Population::evolution_step_individual;
//                if (i)
//                {
//                    std::vector<double>& udfp = i->user_data_for_plotting;
//                    udfp.clear();
//                    udfp.push_back(flock.obstacleCollisionsScore());
//                    udfp.push_back(flock.separationScore());
//                    udfp.push_back(flock.speedScore());
//                }
//            }
//        };
//
//        // Occasionally poll the Draw GUI to check for events esp the "B" command.
//        Draw::getInstance().pollEvents();
//
//        if (EF::enable_multithreading)
//        {
//            bool previous_enable_state = Draw::getInstance().enable();
//            Draw::getInstance().setEnable(false);
//
//            // Do each simulation run in a parallel thread.
//            std::vector<std::thread> threads;
//            for (int r = 0; r < runs; r++) { threads.push_back(std::thread(do_1_run)); }
//            // Wait for helper threads to finish, join them with this thread.
//            for (auto& t : threads) { t.join(); }
//
//            Draw::getInstance().setEnable(previous_enable_state);
//        }
//        else
//        {
//            // Do each simulation run sequentially.
//            for (int r = 0; r < runs; r++) { do_1_run(); }
//        }
//
//        assert(scalar_fits.size() == runs);
//        fitness_logger(least_mof);
//        std::cout << "    min composite "<< least_scalar_fitness;
//        std::cout << "  {" << LP::vec_to_string(scalar_fits) << "}";
//        std::cout << std::endl << std::endl;
//        return least_mof;
//    }


inline MOF run_ga_gp_flock_simulation(LP::Individual* individual, int runs = 4)
{
    MOF least_mof;
    double least_scalar_fitness = std::numeric_limits<double>::infinity();
    std::vector<double> scalar_fits;
    std::mutex save_mof_mutex;
    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    // TODO 20251001 investigate low speed score with ONLY Speed_Control GpFunc
    Flock* log_flock = nullptr;
    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~

    // Perform one simulation run, and record results.
    auto do_1_run = [&]()
    {
        // These steps can happen in parallel threads:
        Flock flock;
        init_flock(flock);
        if (EF::usingGA())
        {
            // For GA, set Flock's FlockParameters from evolved "tree".
            flock.fp() = fp_from_ga_individual(individual);
        }
        else
        {
            // For GP, set Flock's override_steer_function_.
            flock.override_steer_function_ = [&]()
            {
                Boid* boid = Boid::getGpPerThread();
                LP::GpTree gp_tree = individual->tree();
                
                
//                    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
//                    // TODO 20251001 investigate low speed score with
//                    //               ONLY Speed_Control GpFunc
//                    if (boid->isSelected() and
//    //                    ((flock.clock().frameCounter() % 100) == 0))
//    //                    ((flock.clock().frameCounter() % 100) == 1))
//                        ((flock.clock().frameCounter() % 25) == 1))
//                    {
//                        if (log_flock == nullptr)
//                        {
//                            log_flock = &flock;
//                        }
//                        if (log_flock == &flock)
//                        {
//                            grabPrintLock_evoflock();
//                            std::cout << "====> ";
//                            std::cout << &flock;
//                            std::cout << ", selected boid speed: ";
//                            std::cout << boid->speed();
//                            std::cout << std::endl;
//                        }
//                    }
//                    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~

                // TEMP: here we are assuming GpTree returns a local steer vec
                Vec3 local_steering = std::any_cast<Vec3>(gp_tree.eval());
                Vec3 steering = boid->ls().globalizeDirection(local_steering);

                //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
                // TODO 20251001 investigate low speed score with
                //               ONLY Speed_Control GpFunc

                // KEEP? added while tracking down "cleaning" issues
                // (or should it be: steering = clean_vec3(steering);
//                if (not steering.is_valid()) { steering = Vec3(); }
                steering = clean_vec3(steering);
                
                // Ran a test in GA mode. Max steering force length was 1000.
                // WIP reduce by an order of magnitude, close to max_force()
//                double max_steering_length = 10;
                double max_steering_length = 100;
                steering = steering.truncate(max_steering_length);

                //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~

                // KEEP? added while tracking down "cleaning" issues
                double min_steering_length = 0.00001;
                if (steering.length() < min_steering_length) { steering = Vec3(); }
                
                
                //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
                // TODO 20251001 investigate low speed score with
                //               ONLY Speed_Control GpFunc
                if (boid->isSelected() and
//                    ((flock.clock().frameCounter() % 25) == 1))
                    ((flock.clock().frameCounter() % 10) == 1))
                {
                    if (log_flock == nullptr)
                    {
                        log_flock = &flock;
                    }
                    if (log_flock == &flock)
                    {
                        grabPrintLock_evoflock();
                        std::cout << "====> ";
                        std::cout << &flock;
                        std::cout << ", selected boid speed: ";
                        std::cout << boid->speed();
                        std::cout << ", local steer: " << local_steering;
                        std::cout << std::endl;
                    }
                }
                //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~

                
                return clean_vec3(steering);
            };
        }

        flock.run();
        MOF mof = multiObjectiveFitnessOfFlock(flock);
        // These steps happen in the single thread with lock on save_mof_mutex.
        {
            std::lock_guard<std::mutex> smm(save_mof_mutex);
            assert(mof.size() == mof_names().size());
            scalar_fits.push_back(scalarize_fitness(mof));
            if (least_scalar_fitness > scalar_fits.back())
            {
                least_scalar_fitness = scalar_fits.back();
                least_mof = mof;
            }
            // Store these stats on the "current individual"
            LP::Individual* i = LP::Population::evolution_step_individual;
            if (i)
            {
                std::vector<double>& udfp = i->user_data_for_plotting;
                udfp.clear();
                udfp.push_back(flock.obstacleCollisionsScore());
                udfp.push_back(flock.separationScore());
                udfp.push_back(flock.speedScore());
            }
        }
    };
    
    // Occasionally poll the Draw GUI to check for events esp the "B" command.
    Draw::getInstance().pollEvents();

    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    // TODO 20251002 investigate low speed score with ONLY Speed_Control GpFunc
    //               VERY TEMP, does it work better without multithreading?

    bool saved_ef_em = EF::enable_multithreading;
    EF::enable_multithreading = false;
    
    bool previous_enable_state = Draw::getInstance().enable();
    Draw::getInstance().setEnable(false);

    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~

    if (EF::enable_multithreading)
    {
        bool previous_enable_state = Draw::getInstance().enable();
        Draw::getInstance().setEnable(false);
        
        // Do each simulation run in a parallel thread.
        std::vector<std::thread> threads;
        for (int r = 0; r < runs; r++) { threads.push_back(std::thread(do_1_run)); }
        // Wait for helper threads to finish, join them with this thread.
        for (auto& t : threads) { t.join(); }
        
        Draw::getInstance().setEnable(previous_enable_state);
    }
    else
    {
        // Do each simulation run sequentially.
        for (int r = 0; r < runs; r++) { do_1_run(); }
    }
    
    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    // TODO 20251002 investigate low speed score with ONLY Speed_Control GpFunc
    //               VERY TEMP, does it work better without multithreading?
    
    Draw::getInstance().setEnable(previous_enable_state);

    EF::enable_multithreading = saved_ef_em;
    
    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    
    assert(scalar_fits.size() == runs);
    fitness_logger(least_mof);
    std::cout << "    min composite "<< least_scalar_fitness;
    std::cout << "  {" << LP::vec_to_string(scalar_fits) << "}";
    std::cout << std::endl << std::endl;
    return least_mof;
}

//~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

//~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

//~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// Run flock simulation with given parameters "runs" times and returns the MOF
// with the LEAST scalar fitness score.

inline MOF run_flock_simulation(const FlockParameters& fp, int runs = 4)
{
    MOF least_mof;
    double least_scalar_fitness = std::numeric_limits<double>::infinity();
    std::vector<double> scalar_fits;
    std::mutex save_mof_mutex;

    // Perform one simulation run, and record results.
    auto do_1_run = [&]()
    {
        // These steps can happen in parallel threads:
        Flock flock;
        init_flock(flock);
        flock.fp() = fp;
        flock.run();
        MOF mof = multiObjectiveFitnessOfFlock(flock);
        // These steps happen in the single thread with lock on save_mof_mutex.
        {
            std::lock_guard<std::mutex> smm(save_mof_mutex);
            assert(mof.size() == mof_names().size());
            scalar_fits.push_back(scalarize_fitness(mof));
            if (least_scalar_fitness > scalar_fits.back())
            {
                least_scalar_fitness = scalar_fits.back();
                least_mof = mof;
            }
            // Store these stats on the "current individual"
            LP::Individual* i = LP::Population::evolution_step_individual;
            if (i)
            {
                std::vector<double>& udfp = i->user_data_for_plotting;
                udfp.clear();
                udfp.push_back(flock.obstacleCollisionsScore());
                udfp.push_back(flock.separationScore());
                udfp.push_back(flock.speedScore());
            }
        }
    };
    
    // Occasionally poll the Draw GUI to check for events esp the "B" command.
    Draw::getInstance().pollEvents();
    
//    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//    // TODO 20250909 does turning of multithreading help?
//    // TODO 20250909 AHA?! in GP-but-not-multithreading mode this is not reached
//    {
//        grabPrintLock_evoflock();
//        std::cout << std::endl;
//        std::cout << "-----------------------------------------------" << std::endl;
//        debugPrint(EF::unify_GA_GP);
//        debugPrint(EF::usingGP());
//        debugPrint(EF::enable_multithreading);
//        debugPrint(Draw::getInstance().enable());
//        std::cout << "-----------------------------------------------" << std::endl;
//        std::cout << std::endl;
//        
//    }
//    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    if (EF::enable_multithreading)
    {
        bool previous_enable_state = Draw::getInstance().enable();
        Draw::getInstance().setEnable(false);
        
        // Do each simulation run in a parallel thread.
        std::vector<std::thread> threads;
        for (int r = 0; r < runs; r++) { threads.push_back(std::thread(do_1_run)); }
        // Wait for helper threads to finish, join them with this thread.
        for (auto& t : threads) { t.join(); }
        
        Draw::getInstance().setEnable(previous_enable_state);
    }
    else
    {
        // Do each simulation run sequentially.
        for (int r = 0; r < runs; r++) { do_1_run(); }
    }
    
    assert(scalar_fits.size() == runs);
    fitness_logger(least_mof);
    std::cout << "    min composite "<< least_scalar_fitness;
    std::cout << "  {" << LP::vec_to_string(scalar_fits) << "}";
    std::cout << std::endl << std::endl;
    return least_mof;
}

// TODO 20240622 just temporary for debugging
std::map<LP::Individual*, Vec3> values_of_individuals;
std::map<LP::Individual*, LP::GpTree> trees_of_individuals;

inline MOF run_gp_flock_simulation(LP::Individual* individual)
{
    assert(EF::usingGP());
    int runs = 4;
    MOF least_mof;
    double least_scalar_fitness = std::numeric_limits<double>::infinity();
    std::vector<double> scalar_fits;
    std::mutex save_mof_mutex;
    // Perform one simulation run, and record results.
    auto do_1_run = [&]()
    {
        // These steps can happen in parallel threads:
        Flock flock;
        init_flock(flock);
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240627 it would be nice to not have to duplicate tree for each
        //               boid on every simulation step.
        //
        // LP::GpTree gp_tree;
        // gp_tree = individual->tree();
        
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
        // TODO 20250909 does turning of multithreading help?

//        flock.override_steer_function = [&]()
        flock.override_steer_function_ = [&]()

        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
        {
            LP::GpTree gp_tree = individual->tree();
            Vec3 steering = std::any_cast<Vec3>(gp_tree.eval());
            if (not steering.is_valid()) { steering = Vec3(); }
            double max_steering_length = 10;
            steering = steering.truncate(max_steering_length);
            double min_steering_length = 0.00001;
            if (steering.length() < min_steering_length) { steering = Vec3(); }
            return steering;
        };
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        flock.run();
        MOF mof = multiObjectiveFitnessOfFlock(flock);
        // These steps happen in the single thread with lock on save_mof_mutex.
        {
            std::lock_guard<std::mutex> smm(save_mof_mutex);
            assert(mof.size() == mof_names().size());
            scalar_fits.push_back(scalarize_fitness(mof));
            if (least_scalar_fitness > scalar_fits.back())
            {
                least_scalar_fitness = scalar_fits.back();
                least_mof = mof;
            }
        }
    };
    
//    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//    // TODO 20250909 does turning of multithreading help?
//    // TODO 20250909 AHA?! in GP-but-not-multithreading mode this is not reached
//    {
//        grabPrintLock_evoflock();
//        std::cout << std::endl;
//        std::cout << "-----------------------------------------------" << std::endl;
//        debugPrint(EF::unify_GA_GP);
//        debugPrint(EF::usingGP());
//        debugPrint(EF::enable_multithreading);
//        debugPrint(Draw::getInstance().enable());
//        std::cout << "-----------------------------------------------" << std::endl;
//        std::cout << std::endl;
//    }
//    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    
    if (EF::enable_multithreading)
    {
        // Do each simulation run in a parallel thread.
        std::vector<std::thread> threads;
        for (int r = 0; r < runs; r++) { threads.push_back(std::thread(do_1_run)); }
        // Wait for helper threads to finish, join them with this thread.
        for (auto& t : threads) { t.join(); }
    }
    else
    {
        // Do simulation runs sequentially.
        for (int r = 0; r < runs; r++) { do_1_run(); }
    }
    
    assert(scalar_fits.size() == runs);
    fitness_logger(least_mof);
    std::cout << "    min composite "<< least_scalar_fitness;
    std::cout << "  {" << LP::vec_to_string(scalar_fits) << "}";
    std::cout << std::endl << std::endl;
    return least_mof;
    
}


//~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
// TODO 20250912 is GpFunc "Run_Flock" return value used?

//    // Fitness function, simply returns Individual's tree's value (computing it and
//    // caching it on first call).
//    inline MOF evoflock_ga_fitness_function(LP::Individual* individual)
//    {
//        return std::any_cast<MOF>(individual->tree().getRootValue());
//    }

// Fitness function, simply returns Individual's tree's value (computing it and
// caching it on first call).
inline MOF evoflock_ga_fitness_function(LP::Individual* individual)
{
    
//    std::cout << "in GP::evoflock_ga_fitness_function()" << std::endl;
    
    return run_ga_gp_flock_simulation(individual);
    
}

//~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~





//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20250913 change GpType for GpFunc "Run_Flock"

//~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
// TODO 20250912 is GpFunc "Run_Flock" return value used?


// This should presumable be calling run_ga_gp_flock_simulation()

//    // Fitness function, runs a flock simulation using evolved tree for steering
//    inline MOF evoflock_gp_fitness_function(LP::Individual* individual)
//    {
//        return run_gp_flock_simulation(individual);
//    }



//~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~

// Fitness function, runs a flock simulation using evolved tree for steering
inline MOF evoflock_gp_fitness_function(LP::Individual* individual)
{
    return run_ga_gp_flock_simulation(individual);
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~







//~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
// TODO 20250912 is GpFunc "Run_Flock" return value used?

// Maybe this should be calling run_ga_gp_flock_simulation() unconditionally?


    // Generic EvoFlock fitness function handling both GA and GP. In the future this
    // may want to be one of many fitness function but leaving it simple for now.
    inline MOF fitnessFunction(LP::Individual* individual)
    {
        return (EF::usingGP() ?
                evoflock_gp_fitness_function(individual) :
                evoflock_ga_fitness_function(individual));
    }

//    // Generic EvoFlock fitness function handling both GA and GP. In the future this
//    // may want to be one of many fitness function but leaving it simple for now.
//    inline MOF fitnessFunction(LP::Individual* individual)
//    {
//        return multiObjectiveFitnessOfFlock(<#const Flock &flock#>);
//    }

//~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~

// Run flock simulation with given parameters, return a MultiObjectiveFitness.
inline MOF run_hand_tuned_flock_simulation()
{
    return run_flock_simulation(FlockParameters());
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20250911 refactor run_flock_simulation() to include GP in addition to GA

//    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//    // TODO 20240628 can we do an eval of a const tree?
//    #ifdef eval_const_20240628
//    FlockParameters fp_from_ga_tree(const LazyPredator::GpTree& tree)
//    #else  // eval_const_20240628
//    FlockParameters fp_from_ga_tree(LazyPredator::GpTree& tree)
//    #endif // eval_const_20240628
//    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//    {
//        assert(EF::usingGA());
//        std::vector<double> parameters;
//        for (int i = 0; i < FlockParameters::tunableParameterCount(); i++)
//        {
//            parameters.push_back(tree.evalSubtree<double>(i));
//        }
//        return FlockParameters(parameters);
//    }

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Wrote this to run at the end of evolution on the top-10 fitness individuals
// of population in order to record flock data for playback
inline MOF rerun_flock_simulation(const LazyPredator::Individual* individual)
{
    // Is this tree copy needed to avoid using the previous cached tree root?
    LazyPredator::GpTree t = individual->tree();
    // Run simulation.
    return run_flock_simulation(fp_from_ga_tree(t));
}


// A custom “GA style” crossover for this degenerate GP function set.
inline void evoflock_ga_crossover(const LP::GpTree& parent0,
                                  const LP::GpTree& parent1,
                                  LP::GpTree& offspring,
                                  int min_size,
                                  int max_size,
                                  int fs_min_size)
{
    offspring = parent0;
    for (int i = 0; i < parent0.subtrees().size(); i++)
    {
        if (EF::RS().randomBool())
        {
            offspring.getSubtree(i) = parent1.getSubtree(i);
        }
    }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241203 use default FlockParameters for testing
    //std::cout << "parent0:   " << parent0.to_string()   << std::endl;
    //std::cout << "parent1:   " << parent1.to_string()   << std::endl;
    //std::cout << "offspring: " << offspring.to_string() << std::endl;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}


// An experiment to compare min and prod for MOF components.
void replace_scalar_fitness_metric(LP::Population& population,
                                   std::function<double(MOF)> scalarizer_func)
{
    auto replace = [&](LP::Individual* individual)
    {
        if (individual->hasMultiObjectiveFitness())
        {
            MOF mof = individual->getMultiObjectiveFitness();
            individual->setFitness(scalarizer_func(mof));
        }
    };
    population.applyToAllIndividuals(replace);
    scalarize_fitness = scalarizer_func;
}

// The default (in GpType::defaultJiggleScale()) is 0.05
double jiggle_scale = 0.05;

// This is a degenerate GP function set, for what is essentially a GA problem:
// selecting a set of real number parameters for a flock simulation, via an
// absolute and fixed fitness metric. There is only one function, all GpTrees
// are exactly one function deep, differing only in their parameter values.
LazyPredator::FunctionSet evoflock_ga_function_set_normal()
{
    return
    {
        {
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20250913 change GpType for GpFunc "Run_Flock"
//            { "Multi_Objective_Fitness" },
            { "Flock_Parameters" },
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            { "Real_0_1",    0.0,   1.0, jiggle_scale },
            { "Real_0_10",   0.0,  10.0, jiggle_scale },
            { "Real_0_100",  0.0, 100.0, jiggle_scale },
            { "Real_m1_p1", -1.0,  +1.0, jiggle_scale },
        },
        {
            {
                // GP function name:
                "Run_Flock",
                
                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // TODO 20250913 change GpType for GpFunc "Run_Flock"
                
//                // Return type (in this case it returns a MultiObjectiveFitness):
//                "Multi_Objective_Fitness",

                // Return type: a FlockParameters object.
                "Flock_Parameters",

                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

                // Function parameter type list, cf FlockParameters for details
                {
                    "Real_0_100",  // max_force

                    "Real_0_100",  // weight_forward
                    "Real_0_100",  // weight_separate
                    "Real_0_100",  // weight_align
                    "Real_0_100",  // weight_cohere

                    "Real_0_100",  // weightAvoidPredict
                    "Real_0_100",  // weightAvoidStatic

                    "Real_0_100",  // max_dist_separate
                    "Real_0_100",  // max_dist_align
                    "Real_0_100",  // max_dist_cohere

                    // Cosine of threshold angle (max angle from forward to be seen)
                    "Real_m1_p1",  // angle_separate
                    "Real_m1_p1",  // angle_align
                    "Real_m1_p1",  // angle_cohere
                    
                    "Real_0_100", // fly_away_max_dist
                    "Real_0_10",  // min_time_to_collide
                },
                
                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // TODO 20240628 can we do an eval of a const tree?
                
                // Evaluation function, which runs a flock simulation with the given
                // parameters and returns the fitness.
#ifdef eval_const_20240628
                [](const LazyPredator::GpTree& t)
#else  // eval_const_20240628
                [](LazyPredator::GpTree& t)
#endif // eval_const_20240628
                {
                    
                    
                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    // TODO 20250913 change GpType for GpFunc "Run_Flock"

                    
//                        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//                        // TODO 20250911 refactor run_flock_simulation() to include GP in addition to GA
//
//                        //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
//                        // TODO 20250912 is GpFunc "Run_Flock" return value used?
//
//    //                    auto fitness = run_flock_simulation(fp_from_ga_tree(t));
//
//    //                    auto fitness = run_flock_simulation(fp_from_ga_tree(t));
//
//                        MOF fitness;
//
//                        //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
//
//                        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//                        return std::any(fitness);
                    
                    
                    // TODO should the body of fp_from_ga_tree() be written
                    // inline here? or is it used elsewhere?
                    
                    return std::any(fp_from_ga_tree(t));
                    
                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

                }
                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            }
        }
    };
}

// This is a degenerate GP function set, for what is essentially a GA problem:
// selecting a set of real number parameters for a flock simulation, via an
// absolute and fixed fitness metric. There is only one GpFunc, all GpTrees
// are exactly one function deep, differing only in their parameter values.
LP::FunctionSet evoflock_ga_function_set()
{
    return evoflock_ga_function_set_normal();
}

// The five functions below are "accessors" to retrieve fitness component time
// series to be used for plotting evolution run performance. The data is stored
// on on Individual::user_data_for_plotting.
double getUserData(int index, LP::Individual* individual)
{
    double data = 0;
    std::vector<double>& udfp = individual->user_data_for_plotting;
    if (udfp.size() > index) { data = udfp.at(index); }
    return data;
}
double averageNonObsCol(LP::Population& population)
{
    float total = 0;
    auto f = [&](LP::Individual* i) { total += getUserData(0, i); };
    population.applyToAllIndividuals(f);
    return total / population.getIndividualCount();
}
double averageGoodNnDist(LP::Population& population)
{
    float total = 0;
    auto f = [&](LP::Individual* i) { total += getUserData(1, i); };
    population.applyToAllIndividuals(f);
    return total / population.getIndividualCount();
}
double averageSpeedScore(LP::Population& population)
{
    float total = 0;
    auto f = [&](LP::Individual* i) { total += getUserData(2, i); };
    population.applyToAllIndividuals(f);
    return total / population.getIndividualCount();
}
double bestNonObsCol(LP::Population& population)
{
    double best = - std::numeric_limits<double>::infinity();
    auto f = [&](LP::Individual* i) { best = std::max(best, getUserData(0, i)); };
    population.applyToAllIndividuals(f);
    return best;
}
double bestGoodNnDist(LP::Population& population)
{
    double best = - std::numeric_limits<double>::infinity();
    auto f = [&](LP::Individual* i) { best = std::max(best, getUserData(1, i)); };
    population.applyToAllIndividuals(f);
    return best;
}
double bestSpeedScore(LP::Population& population)
{
    double best = - std::numeric_limits<double>::infinity();
    auto f = [&](LP::Individual* i) { best = std::max(best, getUserData(2, i)); };
    population.applyToAllIndividuals(f);
    return best;
}

// Called each step to handle writing log file with fitness over time data.
void save_fitness_time_series(LP::Population& population)
{
    int step_frequency = 300;
    static std::string pathname;
    int count = population.getStepCount();
    if ((0 == count) or (0 == (count + 1) % step_frequency))
    {
        if (pathname.empty())
        {
            pathname = "/Users/cwr/Desktop/flock_data/fitness_data.csv";
            std::ofstream stream(pathname);
            std::string labels = ("step,average,best,"
                                  "ave_avoid_score,best_avoid_score,"
                                  "ave_sep_score,best_sep_score,"
                                  "ave_speed_score,best_speed_score");
            stream << labels << std::endl;
            stream.close();
        }
        std::ofstream stream(pathname, std::ios_base::app);
        stream << (count == 0 ? 0 : count + 1) << ",";
        stream << population.averageFitness() << ",";
        stream << population.bestFitness()->getFitness() << ",";
        stream << averageNonObsCol(population) << ",";
        stream << bestNonObsCol(population) << ",";
        stream << averageGoodNnDist(population) << ",";
        stream << bestGoodNnDist(population);
        stream << ",";
        stream << averageSpeedScore(population) << ",";
        stream << bestSpeedScore(population);
        stream << std::endl;
        stream.close();
    }
}

Boid* getGpBoidNeighbor(int n)
{
    assert(Boid::getGpPerThread());
    assert(Boid::getGpPerThread()->cached_nearest_neighbors().size() == 7);
    BoidPtrList neighbors = Boid::getGpPerThread()->cached_nearest_neighbors();
    assert(neighbors.size() >= n);
    return neighbors.at(n - 1);
}

//~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
// TODO 20250918 back to 30000, smaller trees, GpFunc To_Forward() To_Side()

//    double clean_num(double x)
//    {
//        bool unclean = (std::isnan(x) or
//                        std::isinf(x) or
//                        x < std::numeric_limits<double>::min());
//        return (unclean ? 0 : x);
//    }
//
//    Vec3 clean_vec3(Vec3 v)
//    {
//        return { clean_num(v.x()), clean_num(v.y()), clean_num(v.z()) };
//    }


//    Vec3 ensure_unit_length(Vec3 v)
//    {
//        return (v.is_unit_length() ? v : Vec3(1, 0, 0));
//    }

Vec3 ensure_unit_length(Vec3 v)
{
    Vec3 cv = clean_vec3(v);
    return (cv.is_unit_length() ? cv : Vec3(1, 0, 0));
}

//~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~

// FuntionSet for the GP version of EvoFlock.

#ifdef eval_const_20240628
LP::FunctionSet evoflock_gp_function_set() { return evoflock_ga_function_set;}
#else  // eval_const_20240628

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240712 experiment: get rid of GpType "Scalar_5" (replace with
//               "Scalar_100") to promote more "mixability" in the DSL.

//~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
// TODO 20250930 try version with ONLY GpFunc Speed_Control.

#define USE_ONLY_SPEED_CONTROL

LP::FunctionSet evoflock_gp_function_set()
{
#ifdef USE_ONLY_SPEED_CONTROL
    
    return
    {
        // GpTypes
        {
            { "Vec3" },
            { "Scalar_100", -100.0, 100.0 },
        },
        // GpFunctions
        {
            {
                "Speed_Control", "Vec3", {},
                [](LP::GpTree& tree)
                {
                    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
                    // TODO 20251002 investigate low speed score with ONLY Speed_Control GpFunc

//                    double target_speed = 20;
//                    Boid& b = *Boid::getGpPerThread();
//                        int sign =  (b.speed() < target_speed) ? 1 : -1;
//                        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//                        // TODO 20251001 investigate low speed score with ONLY Speed_Control GpFunc
//
//    //                    double max_force = 10;
//                        double max_force = 100;
//
//                        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//                        Vec3 thrust = Vec3(0, 0, 1) * max_force * sign;
                    
                    
//                    Boid& b = *Boid::getGpPerThread();
//                    Vec3 thrust(0, 0, b.steerForSpeedControl().length());
//                    double max_force = 10;
//                    return std::any(thrust * max_force);

                    Boid& b = *Boid::getGpPerThread();
                    Vec3 thrust(0, 0, b.steerForSpeedControl().dot(b.forward()));
//                    double max_force = 10;
//                    double max_force = 1;
                    double max_force = 5;
                    return std::any(thrust * max_force);

                    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
                }
            },
        }
        
    };
    
#else  // USE_ONLY_SPEED_CONTROL
    
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    
    return
    {
        // GpTypes
        {
            { "Vec3" },
            { "Scalar_100", -100.0, 100.0 },
        },
        
        // GpFunctions
        {
            // Scalar functions: add, multiply,
            {
                "Add", "Scalar_100", {"Scalar_100", "Scalar_100"},
                [](LP::GpTree& tree)
                {
                    return std::any(clean_num(tree.evalSubtree<double>(0) +
                                              tree.evalSubtree<double>(1)));
                }
            },
            {
                "Sub", "Scalar_100", {"Scalar_100", "Scalar_100"},
                [](LP::GpTree& tree)
                {
                    return std::any(clean_num(tree.evalSubtree<double>(0) -
                                              tree.evalSubtree<double>(1)));
                }
            },
            {
                "Mult", "Scalar_100", {"Scalar_100", "Scalar_100"},
                [](LP::GpTree& tree)
                {
                    return std::any(clean_num(tree.evalSubtree<double>(0) *
                                              tree.evalSubtree<double>(1)));
                }
            },
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20250920 experimental curriculum learning
            //            {
            //                "Power", "Scalar_100", {"Scalar_100", "Scalar_100"},
            //                [](LP::GpTree& tree)
            //                {
            //                    double base = tree.evalSubtree<double>(0);
            //                    double expt = tree.evalSubtree<double>(1);
            //                    return std::any(clean_num(std::pow(base, expt)));
            //                }
            //            },
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            {
                "Abs", "Scalar_100", {"Scalar_100"},
                [](LP::GpTree& tree)
                {
                    double x = tree.evalSubtree<double>(0);
                    return std::any(clean_num(std::abs(x)));
                }
            },
            
            // Vector functions:
            
            //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
            // TODO 20250929 I keep seeing bad designs with this at the top
            //               Not sure how useful it is. Turn it off for now.
            //            {
            //                "Vec3", "Vec3", {"Scalar_100", "Scalar_100", "Scalar_100"},
            //                [](LP::GpTree& tree)
            //                {
            //                    Vec3 v(tree.evalSubtree<double>(0),
            //                           tree.evalSubtree<double>(1),
            //                           tree.evalSubtree<double>(2));
            //                    return std::any(clean_vec3(v));
            //                }
            //            },
            //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
            {
                "Add_v3", "Vec3", {"Vec3", "Vec3"},
                [](LP::GpTree& tree)
                {
                    return std::any(tree.evalSubtree<Vec3>(0) +
                                    tree.evalSubtree<Vec3>(1));
                }
            },
            {
                "Sub_v3", "Vec3", {"Vec3", "Vec3"},
                [](LP::GpTree& tree)
                {
                    return std::any(tree.evalSubtree<Vec3>(0) -
                                    tree.evalSubtree<Vec3>(1));
                }
            },
            {
                "Scale_v3", "Vec3", {"Vec3", "Scalar_100"},
                [](LP::GpTree& tree)
                {
                    return std::any(tree.evalSubtree<Vec3>(0) *
                                    tree.evalSubtree<double>(1));
                }
            },
            {
                "Length", "Scalar_100", {"Vec3"},
                [](LP::GpTree& tree)
                {
                    return std::any(tree.evalSubtree<Vec3>(0).length());
                }
            },
            {
                "Normalize", "Vec3", {"Vec3"},
                [](LP::GpTree& tree)
                {
                    Vec3 v = tree.evalSubtree<Vec3>(0);
                    Vec3 n = v.normalize_or_0();
                    return std::any(clean_vec3(n));
                }
            },
            //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
            // TODO 20250930 fewer GpFunc that may be "attractive nuisances"
            //            {
            //                "Cross", "Vec3", {"Vec3", "Vec3"},
            //                [](LP::GpTree& tree)
            //                {
            //                    return std::any(Vec3::cross(tree.evalSubtree<Vec3>(0),
            //                                                tree.evalSubtree<Vec3>(1)));
            //                }
            //            },
            //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
            {
                "Dot", "Scalar_100", {"Vec3", "Vec3"},
                [](LP::GpTree& tree)
                {
                    return std::any(Vec3::dot(tree.evalSubtree<Vec3>(0),
                                              tree.evalSubtree<Vec3>(1)));
                }
            },
            
            //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
            // TODO 20250918 back to 30000, smaller trees, GpFunc To_Forward() To_Side()
            
            //            {
            //                "Parallel_Component", "Vec3", {"Vec3", "Vec3"},
            //                [](LP::GpTree& tree)
            //                {
            //                    Vec3 value = tree.evalSubtree<Vec3>(0);
            //                    Vec3 basis = tree.evalSubtree<Vec3>(1).normalize_or_0();
            //                    Vec3 unit_basis = ensure_unit_length(basis);
            //                    return std::any(value.parallel_component(unit_basis));
            //                }
            //            },
            //            {
            //                "Perpendicular_Component", "Vec3", {"Vec3", "Vec3"},
            //                [](LP::GpTree& tree)
            //                {
            //                    Vec3 value = tree.evalSubtree<Vec3>(0);
            //                    Vec3 basis = tree.evalSubtree<Vec3>(1).normalize_or_0();
            //                    Vec3 unit_basis = ensure_unit_length(basis);
            //                    return std::any(value.perpendicular_component(unit_basis));
            //                }
            //            },
            
            //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
            
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20250920 experimental curriculum learning
            
            //                {
            //                    "Interpolate", "Vec3", {"Scalar_100", "Vec3", "Vec3"},
            //                    [](LP::GpTree& tree)
            //                    {
            //                        double i = tree.evalSubtree<double>(0);
            //                        Vec3 a = tree.evalSubtree<Vec3>(1);
            //                        Vec3 b = tree.evalSubtree<Vec3>(2);
            //    //                    return std::any(util::interpolate(util::clip01(i), a, b));
            //                        return std::any(util::interpolate(i, a, b));
            //                    }
            //                },
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            
            
            
            //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~
            // TODO 20250915 replace Ifle with If_Pos
            //            {
            //                "Ifle", "Vec3", {"Scalar_100", "Scalar_100", "Vec3", "Vec3"},
            //                [](LP::GpTree& tree)
            //                {
            //                    double i = tree.evalSubtree<double>(0);
            //                    double j = tree.evalSubtree<double>(1);
            //                    Vec3 a = tree.evalSubtree<Vec3>(2);
            //                    Vec3 b = tree.evalSubtree<Vec3>(3);
            //                    return std::any(i <= j ? a : b);
            //                }
            //            },
            //            {
            //                "If_Pos", "Vec3", {"Scalar_100", "Vec3", "Vec3"},
            //                [](LP::GpTree& tree)
            //                {
            //                    double i = tree.evalSubtree<double>(0);
            //                    Vec3 a = tree.evalSubtree<Vec3>(1);
            //                    Vec3 b = tree.evalSubtree<Vec3>(2);
            //                    return std::any(i > 0 ? a : b);
            //                }
            //            },
            
            //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
            // TODO 20250930 fewer GpFunc that may be "attractive nuisances"
            
            //            {
            //                "If_Pos", "Vec3", {"Scalar_100", "Vec3", "Vec3"},
            //                [](LP::GpTree& tree)
            //                {
            //                    return std::any(0 < tree.evalSubtree<double>(0) ?
            //                                    tree.evalSubtree<Vec3>(1) :
            //                                    tree.evalSubtree<Vec3>(2));
            //                }
            //            },
            
            //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
            
            //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~
            
            // Boid API:
            //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~
            // TODO 20250915 add speed() GpFunc
            {
                "Speed", "Scalar_100", {},
                [](LP::GpTree& t)
                {
                    return std::any(Boid::getGpPerThread()->speed());
                }
            },
            //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~
            {
                "Velocity", "Vec3", {},
                [](LP::GpTree& t)
                {
                    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
                    // TODO 20250922 WIP switch to "evolved program returns local vec
                    
                    //                    return std::any(Boid::getGpPerThread()->velocity());
                    
                    //                    Boid* b = Boid::getGpPerThread();
                    //                    return std::any(b->ls().localize(b->velocity()));
                    
                    Boid& boid = *Boid::getGpPerThread();
                    return std::any(boid.ls().localizeDirection(boid.velocity()));
                    
                    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
                }
            },
            {
                "Acceleration", "Vec3", {},
                [](LP::GpTree& t)
                {
                    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
                    // TODO 20250922 WIP switch to "evolved program returns local vec
                    
                    //                    return std::any(Boid::getGpPerThread()->getAcceleration());
                    
                    //                    Boid* b = Boid::getGpPerThread();
                    //                    return std::any(b->ls().localize(b->getAcceleration()));
                    
                    //                    Boid& boid = *Boid::getGpPerThread();
                    //                    return std::any(boid.ls().localizeDirection(boid.getAcceleration()));
                    
                    Boid& boid = *Boid::getGpPerThread();
                    Vec3 acceleration = boid.getAcceleration();
                    return std::any(boid.ls().localizeDirection(acceleration));
                    
                    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
                }
            },
            
            //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
            // TODO 20250930 fewer GpFunc that may be "attractive nuisances"
            
            //                {
            //                    "Neighbor_1_Velocity", "Vec3", {},
            //                    [](LP::GpTree& t)
            //                    {
            //                        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
            //                        // TODO 20250922 WIP switch to "evolved program returns local vec
            //
            //    //                    return std::any(getGpBoidNeighbor(1)->velocity());
            //
            //    //                    Boid* b = Boid::getGpPerThread();
            //    //                    return std::any(b->ls().localize(getGpBoidNeighbor(1)->velocity()));
            //
            //                        Boid& boid = *Boid::getGpPerThread();
            //                        Vec3 nv = getGpBoidNeighbor(1)->velocity();
            //                        return std::any(boid.ls().localizeDirection(nv));
            //
            //                        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
            //                    }
            //                },
            //                {
            //                    "Neighbor_1_Offset", "Vec3", {},
            //                    [](LP::GpTree& t)
            //                    {
            //                        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
            //                        // TODO 20250922 WIP switch to "evolved program returns local vec
            //
            //    //                    return std::any(getGpBoidNeighbor(1)->position() -
            //    //                                    Boid::getGpPerThread()->position());
            //
            //    //                    Boid* b = Boid::getGpPerThread();
            //    //                    return std::any(b->ls().localize(getGpBoidNeighbor(1)->position()));
            //
            //                        Boid& b = *Boid::getGpPerThread();
            //                        Vec3 no = getGpBoidNeighbor(1)->position() - b.position();
            //                        return std::any(b.ls().localizeDirection(no));
            //
            //                        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
            //                    }
            //                },
            
            //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
            
            
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20250920 experimental curriculum learning
            
            //            {
            //                "Neighbor_2_Velocity", "Vec3", {},
            //                [](LP::GpTree& t)
            //                {
            //                    return std::any(getGpBoidNeighbor(2)->velocity());
            //
            //                }
            //            },
            //            {
            //                "Neighbor_2_Offset", "Vec3", {},
            //                [](LP::GpTree& t)
            //                {
            //                    return std::any(getGpBoidNeighbor(2)->position() -
            //                                    Boid::getGpPerThread()->position());
            //                }
            //            },
            //            {
            //                "Neighbor_3_Velocity", "Vec3", {},
            //                [](LP::GpTree& t)
            //                {
            //                    return std::any(getGpBoidNeighbor(3)->velocity());
            //
            //                }
            //            },
            //            {
            //                "Neighbor_3_Offset", "Vec3", {},
            //                [](LP::GpTree& t)
            //                {
            //                    return std::any(getGpBoidNeighbor(3)->position() -
            //                                    Boid::getGpPerThread()->position());
            //                }
            //            },
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            
            
            //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
            // TODO 20250930 fewer GpFunc that may be "attractive nuisances"
            
            //                {
            //                    "First_Obs_Dist", "Scalar_100", {},
            //                    [](LP::GpTree& t)
            //                    {
            //                        Boid& boid = *Boid::getGpPerThread();
            //                        double distance = std::numeric_limits<double>::infinity();
            //                        auto collisions = boid.get_predicted_obstacle_collisions();
            //                        if (collisions.size() > 0)
            //                        {
            //                            const Collision& first_collision = collisions.front();
            //                            Vec3 poi = first_collision.point_of_impact;
            //                            distance = (poi - boid.position()).length();
            //                        }
            //                        return std::any(distance);
            //                    }
            //                },
            //                {
            //                    "First_Obs_Normal", "Vec3", {},
            //                    [](LP::GpTree& t)
            //                    {
            //                        Boid& boid = *Boid::getGpPerThread();
            //                        Vec3 normal;
            //                        auto collisions = boid.get_predicted_obstacle_collisions();
            //                        if (collisions.size() > 0)
            //                        {
            //                            const Collision& first_collision = collisions.front();
            //                            normal = first_collision.normal_at_poi;
            //
            //                            //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
            //                            // TODO 20250922 WIP switch to "evolved program returns local vec
            //
            //                            // Convert normal to local space.
            //    //                        Vec3 norm_plus_pos = normal + boid.position();
            //    //                        normal = boid.ls().localize(norm_plus_pos);
            //
            //    //                        normal = boid.ls().localize(normal + boid.position());
            //
            //
            //                            normal = boid.ls().localizeDirection(normal);
            //
            //                            //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
            //                        }
            //                        return std::any(normal);
            //                    }
            //                },
            
            //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
            
            
            //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
            // TODO 20250918 back to 30000, smaller trees, GpFunc To_Forward() To_Side()
            
            //            {
            //                "Parallel_Component", "Vec3", {"Vec3", "Vec3"},
            //                [](LP::GpTree& tree)
            //                {
            //                    Vec3 value = tree.evalSubtree<Vec3>(0);
            //                    Vec3 basis = tree.evalSubtree<Vec3>(1).normalize_or_0();
            //                    Vec3 unit_basis = ensure_unit_length(basis);
            //                    return std::any(value.parallel_component(unit_basis));
            //                }
            //            },
            //            {
            //                "Perpendicular_Component", "Vec3", {"Vec3", "Vec3"},
            //                [](LP::GpTree& tree)
            //                {
            //                    Vec3 value = tree.evalSubtree<Vec3>(0);
            //                    Vec3 basis = tree.evalSubtree<Vec3>(1).normalize_or_0();
            //                    Vec3 unit_basis = ensure_unit_length(basis);
            //                    return std::any(value.perpendicular_component(unit_basis));
            //                }
            //            },
            
            
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20250929 not obvious these are helpful, try without
            //            {
            //                "To_Forward", "Vec3", {"Vec3"},
            //                [](LP::GpTree& tree)
            //                {
            //                    Vec3 value = clean_vec3(tree.evalSubtree<Vec3>(0));
            //                    Boid& b = *Boid::getGpPerThread();
            //                    // Take component of "value" which is parallel to "forward".
            //                    Vec3 parallel = value.parallel_component(b.forward());
            //                    return std::any(b.ls().localizeDirection(parallel));
            //                }
            //            },
            //            {
            //                "To_Side", "Vec3", {"Vec3"},
            //                [](LP::GpTree& tree)
            //                {
            //                    Vec3 value = clean_vec3(tree.evalSubtree<Vec3>(0));
            //                    Boid& b = *Boid::getGpPerThread();
            //                    // Take component of "value" perpendicular to "forward".
            //                    Vec3 perp = value.perpendicular_component(b.forward());
            //                    return std::any(b.ls().localizeDirection(perp));
            //                }
            //            },
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            
            
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20250929 experimental "higher level" GpFunc for boids
            
            //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
            // TODO 20250930 since we are taking such baby steps, remove arg
            
            
            {
                //                "Speed_Control", "Vec3", {"Scalar_100"},
                "Speed_Control", "Vec3", {},
                [](LP::GpTree& tree)
                {
                    //                    double target_speed = clean_num(tree.evalSubtree<double>(0));
                    double target_speed = 20;
                    Boid& b = *Boid::getGpPerThread();
                    int sign =  (b.speed() < target_speed) ? 1 : -1;
                    double max_force = 10;
                    
                    // TODO oh this looks like a real bug:
                    //                    Vec3 thrust = b.forward() * max_force * sign;
                    Vec3 thrust = Vec3(0, 0, 1) * max_force * sign;
                    
                    return std::any(thrust);
                }
            },
            
            //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
            
            
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            
            
            
            
            
            
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20250929 experimental "higher level" GpFunc for boids
            
            {
                "Avoid_Obstacle", "Vec3", {},
                [](LP::GpTree& tree)
                {
                    double min_dist = 25;
                    Boid& b = *Boid::getGpPerThread();
                    Vec3 avoidance;
                    auto collisions = b.get_predicted_obstacle_collisions();
                    if (collisions.size() > 0)
                    {
                        const Collision& first_collision = collisions.front();
                        Vec3 poi = first_collision.point_of_impact;
                        double distance = (poi - b.position()).length();
                        if (distance > min_dist)
                        {
                            Vec3 normal = first_collision.normal_at_poi;
                            avoidance = normal.parallel_component(b.forward());
                            
                            avoidance = b.ls().localizeDirection(avoidance);
                        }
                    }
                    return std::any(avoidance);
                }
            },
            
            {
                "Adjust_Neighbor_Dist", "Vec3", {},
                [](LP::GpTree& tree)
                {
                    Vec3 steering;
                    Boid& b = *Boid::getGpPerThread();
                    Vec3 neighbor_offset = (getGpBoidNeighbor(1)->position() -
                                            Boid::getGpPerThread()->position());
                    double neighbor_dist = neighbor_offset.length();
                    Vec3 neighbor_direction = neighbor_offset / neighbor_dist;
                    if (neighbor_dist < 2) { steering = neighbor_direction; }
                    if (neighbor_dist > 9) { steering = -neighbor_direction; }
                    
                    steering = b.ls().localizeDirection(steering);
                    
                    return std::any(steering * 10);
                }
            },
            
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            
            
            
            
            
            //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
            // TODO 20240801 very experimental, add high level "hint" to
            // simulate "seeding" population with handwritten steering program.
            
            
            //            {
            //                "Avoid_Obstacle", "Vec3", {},
            //                [](LP::GpTree& tree)
            //                {
            //                    double min_dist = 25;
            //                    Boid& boid = *Boid::getGpPerThread();
            //                    Vec3 avoidance;
            //                    auto collisions = boid.get_predicted_obstacle_collisions();
            //                    if (collisions.size() > 0)
            //                    {
            //                        const Collision& first_collision = collisions.front();
            //                        Vec3 poi = first_collision.point_of_impact;
            //                        double distance = (poi - boid.position()).length();
            //                        if (distance > min_dist)
            //                        {
            //                            Vec3 normal = first_collision.normal_at_poi;
            //                            avoidance = normal.parallel_component(boid.forward());
            //                        }
            //                    }
            //                    return std::any(avoidance);
            //                }
            //            },
            //            {
            //                "Adjust_Neighbor_Dist", "Vec3", {},
            //                [](LP::GpTree& tree)
            //                {
            //                    Vec3 steering;
            //                    Vec3 neighbor_offset = (getGpBoidNeighbor(1)->position() -
            //                                            Boid::getGpPerThread()->position());
            //                    double neighbor_dist = neighbor_offset.length();
            //                    Vec3 neighbor_direction = neighbor_offset / neighbor_dist;
            //                    if (neighbor_dist < 2) { steering = neighbor_direction; }
            //                    if (neighbor_dist > 9) { steering = -neighbor_direction; }
            //                    return std::any(steering * 10);
            //                }
            //            },
            
            //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
        }
    };
    
    
//~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
// TODO 20250930 try version with ONLY GpFunc Speed_Control.

#endif  // USE_ONLY_SPEED_CONTROL

//~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

}



//    LP::FunctionSet test_gp_boid_function_set()
//    {
//        return
//        {
//            // GpTypes
//            {
//                { "Vec3" },
//                { "Scalar_100", -100.0, 100.0 },
//            },
//            // GpFunctions
//            {
//
//                // TODO just laying out a hand-tuned boid behavior
//                {
//                    "Be_The_Boid", "Vec3", {},
//                    [](LP::GpTree& tree)
//                    {
//                        Vec3 avoidance;
//                        //                    Vec3 neighbor_dist_adjust;
//                        Boid& boid = *Boid::getGpPerThread();
//
//
//                        double min_dist = 25;
//                        auto collisions = boid.get_predicted_obstacle_collisions();
//                        if (collisions.size() > 0)
//                        {
//                            const Collision& first_collision = collisions.front();
//                            Vec3 poi = first_collision.point_of_impact;
//                            double distance = (poi - boid.position()).length();
//                            if (distance > min_dist)
//                            {
//                                Vec3 normal = first_collision.normal_at_poi;
//                                avoidance = normal.parallel_component(boid.forward());
//                            }
//                        }
//
//                        Vec3 neighbor_offset = (getGpBoidNeighbor(1)->position() -
//                                                Boid::getGpPerThread()->position());
//                        double neighbor_dist = neighbor_offset.length();
//                        Vec3 neighbor_direction = neighbor_offset / neighbor_dist;
//
//    //                    Vec3 neighbor_dist_adjust = (neighbor_direction *
//    //                                                 ((neighbor_dist < 2) ?
//    //                                                  1 : ((neighbor_dist > 9) ?
//    //                                                       -1 : 0)));
//                        Vec3 neighbor_dist_adjust = (neighbor_direction *
//                                                     ((neighbor_dist < 2) ?
//                                                      -1 :
//    //                                                  ((neighbor_dist > 9) ?
//                                                      ((neighbor_dist > 5) ?
//                                                       1 :
//                                                       0)));
//
//                        // TODO later try hard selection
//                        return std::any((avoidance + neighbor_dist_adjust) * 10);
//
//                    }
//                },
//
//                //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
//            }
//        };
//    }


//    LP::FunctionSet test_gp_boid_function_set()
//    {
//        return
//        {
//            // GpTypes
//            {
//                { "Vec3" },
//            },
//            // GpFunctions
//            {
//
//                // TODO just laying out a hand-tuned boid behavior
//                {
//                    "Be_The_Boid", "Vec3", {},
//                    [](LP::GpTree& tree)
//                    {
//                        Vec3 avoidance;
//                        //                    Vec3 neighbor_dist_adjust;
//                        Boid& boid = *Boid::getGpPerThread();
//
//
//                        double min_dist = 25;
//                        auto collisions = boid.get_predicted_obstacle_collisions();
//                        if (collisions.size() > 0)
//                        {
//                            const Collision& first_collision = collisions.front();
//                            Vec3 poi = first_collision.point_of_impact;
//                            double distance = (poi - boid.position()).length();
//                            if (distance > min_dist)
//                            {
//    //                            Vec3 normal = first_collision.normal_at_poi;
//    //                            avoidance = normal.parallel_component(boid.forward());
//                                avoidance = first_collision.normal_at_poi;
//                            }
//                        }
//
//                        Vec3 neighbor_offset = (getGpBoidNeighbor(1)->position() -
//                                                Boid::getGpPerThread()->position());
//                        double neighbor_dist = neighbor_offset.length();
//                        Vec3 neighbor_direction = neighbor_offset / neighbor_dist;
//
//                        double weight = 0;
//                        if (neighbor_dist < 2) { weight = -1; }
//                        if (neighbor_dist > 5) { weight = +1; }
//    //                    Vec3 neighbor_dist_adjust = (neighbor_direction *
//    //                                                 ((neighbor_dist < 2) ?
//    //                                                  -1 :
//    //                                                  ((neighbor_dist > 5) ?
//    //                                                   1 :
//    //                                                   0)));
//                        Vec3 neighbor_dist_adjust = neighbor_direction * weight;
//
//                        // TODO later try hard selection
//    //                    return std::any((avoidance + neighbor_dist_adjust) * 10);
//                        Vec3 steer = avoidance + neighbor_dist_adjust;
//                        return std::any(steer.perpendicular_component(boid.forward()) * 10);
//                    }
//                },
//
//                //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
//            }
//        };
//    }


//    // TODO just laying out a hand-tuned boid behavior
//    LP::FunctionSet test_gp_boid_function_set()
//    {
//        return
//        {
//            // GpTypes
//            {
//                { "Vec3" },
//            },
//            // GpFunctions
//            {
//                {
//                    "Be_The_Boid", "Vec3", {},
//                    [](LP::GpTree& tree)
//                    {
//                        Vec3 avoidance;
//                        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
//                        // TOODO 20240809 why is obstacle avoidance broken?
//
//    //                    double min_dist = 25;
//    //                    Boid& boid = *Boid::getGpPerThread();
//                        Boid& boid = *Boid::getGpPerThread();
//
//                        double min_dist = boid.speed() * boid.fp().min_time_to_collide;
//
//                        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
//                        auto collisions = boid.get_predicted_obstacle_collisions();
//                        if (collisions.size() > 0)
//                        {
//                            const Collision& first_collision = collisions.front();
//                            //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
//                            // TOODO 20240809 why is obstacle avoidance broken?
//    //                        Vec3 poi = first_collision.point_of_impact;
//    //                        double distance = (poi - boid.position()).length();
//    //                        if (distance > min_dist)
//                            if (first_collision.dist_to_collision > min_dist)
//                            //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
//                            {
//                                avoidance = first_collision.normal_at_poi;
//                            }
//                            //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
//                            // TOODO 20240809 why is obstacle avoidance broken?
//
//                            //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//                            // TODO 20240810 weird time issue in GP mode?
//                            if (boid.is_first_boid() and
//                                ((boid.draw().frameCounter() % 20) == 0))
//                            {
//                                Vec3 f = boid.forward();
//                                Vec3 lat_avoid = avoidance.perpendicular_component(f);
//
//
//    //                            if (not (lat_avoid.is_zero_length() or
//    //                                     lat_avoid.is_perpendicular(f)))
//    //                            {
//    //                                debugPrint(f)
//    //                                debugPrint(lat_avoid)
//    //                            }
//                                debugPrint(f)
//    //                            debugPrint(f.length())
//                                debugPrint(lat_avoid)
//    //                            debugPrint(lat_avoid.length())
//
//                                assert(lat_avoid.is_zero_length() or
//                                       lat_avoid.normalize().is_perpendicular(f, 0.0001));
//
//    //                            Vec3 local_lat_avoid = boid.ls().localize(lat_avoid);
//                                debugPrint(first_collision)
//                                std::cout << boid.draw().frameCounter() << ": ";
//    //                            std::cout << local_lat_avoid.normalize_or_0();
//    //                            std::cout << std::endl;
//    //                            debugPrint(local_lat_avoid.normalize_or_0())
//
//                                Vec3 local_lat_avoid = boid.ls().localize(lat_avoid + boid.position());
//                                debugPrint(local_lat_avoid)
//                            }
//                            //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//
//                            //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
//                        }
//                        Vec3 neighbor_offset = (getGpBoidNeighbor(1)->position() -
//                                                Boid::getGpPerThread()->position());
//                        double neighbor_dist = neighbor_offset.length();
//                        Vec3 neighbor_direction = neighbor_offset / neighbor_dist;
//
//                        double weight = 0;
//                        if (neighbor_dist < 2) { weight = -1; }
//                        if (neighbor_dist > 5) { weight = +1; }
//                        Vec3 neighbor_dist_adjust = neighbor_direction * weight;
//
//                        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                        // TODO 20240811 use had selection when avoidance needed
//
//    //                    // TODO later try hard selection
//    //                    Vec3 steer = avoidance + neighbor_dist_adjust;
//
//                        Vec3 steer = (avoidance.is_zero_length() ?
//                                      neighbor_dist_adjust :
//                                      avoidance);
//
//                        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                        Vec3 lateral = steer.perpendicular_component(boid.forward());
//                        return std::any(lateral * 10);
//                    }
//                },
//            }
//        };
//    }

// TODO just laying out a hand-tuned boid behavior
LP::FunctionSet test_gp_boid_function_set()
{
    return
    {
        // GpTypes
        { { "Vec3" }, },
        // GpFunctions
        {
            {
                "Be_The_Boid", "Vec3", {},
                [](LP::GpTree& tree)
                {
                    Vec3 avoidance;
                    Boid& boid = *Boid::getGpPerThread();
                    
                    // Steer to avoid obstacles.
                    double min_dist = boid.speed() * boid.fp().minTimeToCollide();
                    auto collisions = boid.get_predicted_obstacle_collisions();
                    if (collisions.size() > 0)
                    {
                        const Collision& first_collision = collisions.front();
                        if (min_dist > first_collision.dist_to_collision)
                        {
                            avoidance = first_collision.normal_at_poi;
                        }
                    }
                                        
//                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                    // VERY temp experiment
//                    avoidance = boid.steer_to_avoid();
//                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    
                    // Steer to adjust neighbor offset.
                    Vec3 neighbor_offset = (getGpBoidNeighbor(1)->position() -
                                            Boid::getGpPerThread()->position());
                    double neighbor_dist = neighbor_offset.length();
                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    // VERY temp experiment
//                    Vec3 neighbor_direction = neighbor_offset / neighbor_dist;
                    Vec3 neighbor_direction;
                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    double weight = 0;
                    if (neighbor_dist < 2) { weight = -1; }
                    if (neighbor_dist > 5) { weight = +1; }
                    Vec3 neighbor_dist_adjust = neighbor_direction * weight;
                    
                    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
                    // TODO 20240814 adjust speed
                    Vec3 speed_adjust;
                    Vec3 forward_weighted = boid.forward() * 0.5;
//                    if (boid.speed() < 18) { speed_adjust = forward_weighted; }
//                    if (boid.speed() > 22) { speed_adjust = -forward_weighted; }
                    if (boid.speed() < 16) { speed_adjust = forward_weighted; }
                    if (boid.speed() > 24) { speed_adjust = -forward_weighted; }


//                    Vec3 steer = (avoidance.is_zero_length() ?
//                                  neighbor_dist_adjust :
//                                  avoidance);
                    
//                    steer += speed_adjust;

                    Vec3 steer = (avoidance.is_zero_length() ?
                                  (neighbor_dist_adjust + speed_adjust) :
                                  avoidance);

                    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

                    Vec3 f = boid.forward();
                    Vec3 lateral = steer.perpendicular_component(boid.forward());
                    lateral = lateral.normalize_or_0();

                    // temp:
                    double e = 0.0001;
                    if (not lateral.is_zero_length())
                    {
                        assert(lateral.is_perpendicular(f, e));
                    }
//                        // temp:
//                        if (boid.is_first_boid() and
//                            ((boid.draw().frameCounter() % 20) == 0))
//                        {
//                            if (collisions.size() > 0)
//                            {
//                                std::cout << boid.draw().frameCounter() << ": ";
//                                debugPrint(collisions.front())
//                            }
//                            std::cout << boid.draw().frameCounter() << ": ";
//    //                        debugPrint(lateral)
//                            debugPrint(steer)
//                        }

//                    return std::any(lateral * 10);
//                    return std::any(lateral * 100);
//                    return std::any(steer * 100);
                    
                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    // TODO 20240816 VERY temp experiment
                    return std::any(boid.pre_GP_steer_to_flock());
                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

                }
            },
        }
    };
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#endif // eval_const_20240628

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240707 WIP on a prototype unit test for this GP module

void test_First_Obs_GpFuncs()
{
    const LP::FunctionSet fs = evoflock_gp_function_set();
    const LP::GpFunction* fod = fs.lookupGpFunctionByName("First_Obs_Dist");
    const LP::GpFunction* fon = fs.lookupGpFunctionByName("First_Obs_Normal");

    LP::GpTree gp_tree_fod;
    gp_tree_fod.setRootFunction(*fod);
    double distance = std::any_cast<double>(gp_tree_fod.eval());
    debugPrint(distance)
    
    LP::GpTree gp_tree_fon;
    gp_tree_fon.setRootFunction(*fon);
    Vec3 normal = std::any_cast<Vec3>(gp_tree_fon.eval());
    debugPrint(normal)
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

}  // end of namespace GP
