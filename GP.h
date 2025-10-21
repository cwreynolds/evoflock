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

// TODO 20250930 try version with ONLY GpFunc Speed_Control.
#define USE_ONLY_SPEED_CONTROL


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


FlockParameters fp_from_ga_tree(LazyPredator::GpTree& tree)
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


// These "cleaners" are to avoid ludicrous values in evolved trees.
double clean(double x)
{
    bool unclean = (std::isnan(x) or
                    std::isinf(x) or
                    (x < std::numeric_limits<double>::min()) or
                    (std::abs(x) < 0.00000000001) or
                    (x < -10000000000) or
                    (x > +10000000000));
    return (unclean ? 0 : x);
}

// These "cleaners" are to avoid returning ludicrous values from evolved trees.
Vec3 clean(Vec3 v)
{
    return { clean(v.x()), clean(v.y()), clean(v.z()) };
}


// Run flock simulation(s) described by the given evolutionary LP::Individual.
// Makes "runs" simulations, in parallel if EF::enable_multithreading is set to
// true. Returns the MOF with the LEAST scalar fitness score.
inline MOF run_flock_simulation(LP::Individual* individual, int runs = 4)
{
    MOF least_mof;
    double least_scalar_fitness = std::numeric_limits<double>::infinity();
    std::vector<double> scalar_fits;
    std::mutex save_mof_mutex;
    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    // TODO 20251001 investigate low speed score with ONLY Speed_Control GpFunc
    Flock* log_flock = nullptr;
    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20251018 lock to set log_flock ptr in GP::run_flock_simulation
    std::mutex log_flock_mutex;
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20251016 verify GP steering is not constant
    Vec3 prev_local_steering;
    Vec3 prev_steering;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20251017 make sure we are tracking the expected number of boid-steps
    int log_flock_selected_boid_steps = 0;
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

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
            //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
            // TODO 20251018 more logging
            {
                grabPrintLock_evoflock();
                std::cout << "  do_1_run(), flock = " << &flock << std::endl;
            }
            //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

            // For GP, set Flock's override_steer_function_.
            flock.override_steer_function_ = [&]()
            {
                Boid& b = *Boid::getGpPerThread();
                LP::GpTree gp_tree = individual->tree();
                
                // TEMP: here we are assuming GpTree returns a local steer vec
                Vec3 local_steering = std::any_cast<Vec3>(gp_tree.eval());
                Vec3 steering = b.globalizeDirection(local_steering);

                //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
                // TODO 20251001 investigate low speed score with
                //               ONLY Speed_Control GpFunc

                steering = clean(steering);

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
                
                //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
                // TODO 20251012 back to only Speed_Control: use more max force
//                if (boid->isSelected() and
//                    ((flock.clock().frameCounter() % 10) == 1))
                if (b.isSelected())
                //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

                
                {
                    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
                    // TODO 20251018 lock to set log_flock ptr in GP::run_flock_simulation

//                    if (log_flock == nullptr)
//                    {
//                        log_flock = &flock;
//                    }

                    {
                        std::lock_guard<std::mutex> lfm(log_flock_mutex);
                        if (log_flock == nullptr)
                        {
                            log_flock = &flock;
                            
                            //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
                            // TODO 20251018 more logging
                            {
                                grabPrintLock_evoflock();
                                std::cout << "  do_1_run(), log_flock = ";
                                std::cout << log_flock << std::endl;
                            }
                            //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

                        }
                    }
                    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
                    if (log_flock == &flock)
                    {
                        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
                        // TODO 20251003 re-enable multithreading, it was not the problem
                        
                        // set a temp variable on the boid for logging
                        b.log_flock = log_flock;
                        
                        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
                        
                        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
                        // TODO 20251017 make sure we are tracking the expected
                        //               number of boid-steps
                        log_flock_selected_boid_steps++;
                        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

                        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~
                        // TODO 20251011 return to debug speed control
#ifdef USE_ONLY_SPEED_CONTROL
                        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~
                        grabPrintLock_evoflock();
                        std::cout << std::endl;
                        std::cout << "  ====> ";
                        std::cout << &flock;
                        std::cout << ", selected boid speed: ";
                        std::cout << b.speed();
                        std::cout << ", local steer: " << local_steering;
                        std::cout << std::endl;
                        
                        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
                        // TODO 20251012 back to only Speed_Control: use more max force
                        std::cout << "  ";
                        Vec3 sfsc = b.steerForSpeedControl();
                        debugPrint(sfsc.dot(b.forward()));
                        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
                        
                        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                        // TODO 20251015 near zero speed
                        
                        if (b.new_speed_memory_ <= 0)
                        {
                            std::cout << "  new_speed=" << b.new_speed_memory_;
                            std::cout << ", old speed=" << b.old_speed_memory_;
                            double a = ((b.acceleration_memory_ *
                                         b.time_step_memory_)
                                        .dot(b.forward_memory_));
                            std::cout << ", local accel * dt=" << a;
                            std::cout << std::endl;
                        }
                        
                        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

                        
                        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                        // TODO 20251016 verify GP steering is not constant
                        
                        std::cout << "  ";
                        std::cout << ((prev_local_steering == local_steering) ?
                                      "same" : "diff" );
                        std::cout << "  ";
                        debugPrint(local_steering);

                        std::cout << "  ";
                        std::cout << ((prev_steering == steering) ?
                                      "same" : "diff" );
                        std::cout << "  ";
                        debugPrint(steering);
                        
                        
                        prev_local_steering = local_steering;
                        prev_steering = steering ;
                        
                        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

                        
                        //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
                        // TODO 20251020 back to too few boid steps
                        
                        std::cout << std::endl;
                        std::cout << "  steps counted in do_1_run:    ";
                        std::cout << log_flock_selected_boid_steps;
                        std::cout << std::endl;
                        std::cout << "  flock.clock().frameCounter(): ";
                        std::cout << flock.clock().frameCounter();
                        std::cout << std::endl;
                        
                        //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~

                        
                        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~
                        // TODO 20251011 return to debug speed control
#endif  // USE_ONLY_SPEED_CONTROL
                        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~
                    }
                }
                //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
                

                return clean(steering);
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
    
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20251017 make sure we are tracking the expected number of boid-steps
    debugPrint(log_flock_selected_boid_steps);
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

    assert(scalar_fits.size() == runs);
    fitness_logger(least_mof);
    std::cout << "    min composite "<< least_scalar_fitness;
    std::cout << "  {" << LP::vec_to_string(scalar_fits) << "}";
    std::cout << std::endl << std::endl;
    return least_mof;
}


// Generic EvoFlock fitness function handling both GA and GP. In the future this
// may want to be one of many fitness function but leaving it simple for now.
inline MOF fitnessFunction(LP::Individual* individual)
{
    return run_flock_simulation(individual);
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
//LazyPredator::FunctionSet evoflock_ga_function_set_normal()
LazyPredator::FunctionSet evoflock_ga_function_set()
{
    return
    {
        {
            { "Flock_Parameters" },
            { "Real_0_1",    0.0,   1.0, jiggle_scale },
            { "Real_0_10",   0.0,  10.0, jiggle_scale },
            { "Real_0_100",  0.0, 100.0, jiggle_scale },
            { "Real_m1_p1", -1.0,  +1.0, jiggle_scale },
        },
        {
            {
                // GP function name:
                "Run_Flock",

                // Return type: a FlockParameters object.
                "Flock_Parameters",

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
                                
                // Evaluation function, which runs a flock simulation with the given
                // parameters and returns the fitness.
                [](LazyPredator::GpTree& t)
                {
                    // TODO should the body of fp_from_ga_tree() be written
                    // inline here? or is it used elsewhere?
                    return std::any(fp_from_ga_tree(t));
                }
            }
        }
    };
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


Vec3 ensure_unit_length(Vec3 v)
{
    Vec3 cv = clean(v);
    return (cv.is_unit_length() ? cv : Vec3(1, 0, 0));
}


//------------------------------------------------------------------------------
// Scalar functions: abs, add, subtract, multiply, exponentiation.

inline LP::GpFunction Abs
 ("Abs",
  "Scalar",
  {"Scalar"},
  [](LP::GpTree& tree)
  {
     return std::any(clean(std::abs(clean(tree.evalSubtree<double>(0)))));
 });

inline LP::GpFunction Add
 ("Add",
  "Scalar",
  {"Scalar", "Scalar"},
  [](LP::GpTree& tree)
  {
     return std::any(clean(clean(tree.evalSubtree<double>(0)) +
                           clean(tree.evalSubtree<double>(1))));
 });

inline LP::GpFunction Sub
 ("Sub",
  "Scalar",
  {"Scalar", "Scalar"},
  [](LP::GpTree& tree)
  {
     return std::any(clean(clean(tree.evalSubtree<double>(0)) -
                           clean(tree.evalSubtree<double>(1))));
 });

inline LP::GpFunction Mult
("Mult",
 "Scalar",
 {"Scalar", "Scalar"},
 [](LP::GpTree& tree)
 {
    return std::any(clean(clean(tree.evalSubtree<double>(0)) *
                          clean(tree.evalSubtree<double>(1))));
});

inline LP::GpFunction Power
 (
  "Power",
  "Scalar",
  {"Scalar", "Scalar"},
  [](LP::GpTree& tree)
  {
      double base = clean(tree.evalSubtree<double>(0));
      double expt = clean(tree.evalSubtree<double>(1));
      return std::any(clean(std::pow(base, expt)));
  });

//------------------------------------------------------------------------------
// Vector functions: V3, Add_v3, Sub_v3, Scale_v3, Length, Normalize, Cross,
// Dot, Parallel_Component, Perpendicular_Component, Interpolate, If_Pos

inline LP::GpFunction V3
 (
  "Vec3",
  "Vec3",
  {"Scalar", "Scalar", "Scalar"},
  [](LP::GpTree& tree)
  {
      Vec3 v3(clean(tree.evalSubtree<double>(0)),
              clean(tree.evalSubtree<double>(1)),
              clean(tree.evalSubtree<double>(2)));
      return std::any(clean(v3));
  }
  );

inline LP::GpFunction Add_v3
(
    "Add_v3",
    "Vec3",
    {"Vec3", "Vec3"},
    [](LP::GpTree& tree)
    {
        return std::any(tree.evalSubtree<Vec3>(0) +
                        tree.evalSubtree<Vec3>(1));
    });

inline LP::GpFunction Sub_v3
 (
  "Sub_v3",
  "Vec3",
  {"Vec3", "Vec3"},
  [](LP::GpTree& tree)
  {
      return std::any(tree.evalSubtree<Vec3>(0) -
                      tree.evalSubtree<Vec3>(1));
  });

inline LP::GpFunction Scale_v3
 (
  "Scale_v3",
  "Vec3",
  {"Vec3", "Scalar"},
  [](LP::GpTree& tree)
  {
      return std::any(clean(tree.evalSubtree<Vec3>(0)) *
                      clean(tree.evalSubtree<double>(1)));
  });

inline LP::GpFunction Length
 (
  "Length",
  "Scalar",
  {"Vec3"},
  [](LP::GpTree& tree)
  {
      return std::any(tree.evalSubtree<Vec3>(0).length());
  });

inline LP::GpFunction Normalize
 (
  "Normalize",
  "Vec3",
  {"Vec3"},
  [](LP::GpTree& tree)
  {
      Vec3 v = clean(tree.evalSubtree<Vec3>(0));
      return std::any(v.normalize_or_0());
  });

inline LP::GpFunction Cross
 (
  "Cross",
  "Vec3",
  {"Vec3", "Vec3"},
  [](LP::GpTree& tree)
  {
      return std::any(Vec3::cross(tree.evalSubtree<Vec3>(0),
                                  tree.evalSubtree<Vec3>(1)));
  });

inline LP::GpFunction Dot
 (
  "Dot",
  "Scalar",
  {"Vec3", "Vec3"},
  [](LP::GpTree& tree)
  {
      return std::any(Vec3::dot(tree.evalSubtree<Vec3>(0),
                                tree.evalSubtree<Vec3>(1)));
  });

inline LP::GpFunction Parallel_Component
 (
  "Parallel_Component",
  "Vec3",
  {"Vec3", "Vec3"},
  [](LP::GpTree& tree)
  {
      Vec3 value = tree.evalSubtree<Vec3>(0);
      Vec3 basis = tree.evalSubtree<Vec3>(1).normalize_or_0();
      Vec3 unit_basis = ensure_unit_length(basis);
      return std::any(value.parallel_component(unit_basis));
  });

inline LP::GpFunction Perpendicular_Component
 (
  "Perpendicular_Component",
  "Vec3",
  {"Vec3", "Vec3"},
  [](LP::GpTree& tree)
  {
      Vec3 value = tree.evalSubtree<Vec3>(0);
      Vec3 basis = tree.evalSubtree<Vec3>(1).normalize_or_0();
      Vec3 unit_basis = ensure_unit_length(basis);
      return std::any(value.perpendicular_component(unit_basis));
  });

inline LP::GpFunction Interpolate
 (
  "Interpolate",
  "Vec3",
  {"Scalar", "Vec3", "Vec3"},
  [](LP::GpTree& tree)
  {
      double i = tree.evalSubtree<double>(0);
      Vec3 a = tree.evalSubtree<Vec3>(1);
      Vec3 b = tree.evalSubtree<Vec3>(2);
      //return std::any(util::interpolate(util::clip01(i), a, b));
      return std::any(util::interpolate(i, a, b));
  });

inline LP::GpFunction If_Pos
 (
  "If_Pos",
  "Vec3",
  {"Scalar", "Vec3", "Vec3"},
  [](LP::GpTree& tree)
  {
      return std::any(0 < tree.evalSubtree<double>(0) ?
                      tree.evalSubtree<Vec3>(1) :
                      tree.evalSubtree<Vec3>(2));
  });

//------------------------------------------------------------------------------
// Boid API: Speed, Velocity, Acceleration, Forward,
//           Neighbor_1_Velocity, Neighbor_1_Offset,
//           First_Obs_Dist, First_Obs_Normal, To_Forward, To_Side

inline LP::GpFunction Speed
 (
  "Speed",
  "Scalar",
  {},
  [](LP::GpTree& tree)
  {
      return std::any(Boid::getGpPerThread()->speed());
  });


inline LP::GpFunction Velocity
 (
  "Velocity",
  "Vec3",
  {},
  [](LP::GpTree& t)
  {
      Boid& boid = *Boid::getGpPerThread();
      return std::any(boid.localizeDirection(boid.velocity()));
  });

inline LP::GpFunction Acceleration
 (
  "Acceleration",
  "Vec3",
  {},
  [](LP::GpTree& t)
  {
      Boid& boid = *Boid::getGpPerThread();
      Vec3 acceleration = boid.getAcceleration();
      return std::any(boid.localizeDirection(acceleration));
  });

inline LP::GpFunction Forward
 (
  "Forward",
  "Vec3",
  {},
  [](LP::GpTree& t)
  {
      Boid& boid = *Boid::getGpPerThread();
      return std::any(boid.forward());
  });

inline LP::GpFunction Neighbor_1_Velocity
 (
  "Neighbor_1_Velocity",
  "Vec3",
  {},
  [](LP::GpTree& t)
  {
      Boid& boid = *Boid::getGpPerThread();
      Vec3 nv = getGpBoidNeighbor(1)->velocity();
      return std::any(boid.localizeDirection(nv));
  });

inline LP::GpFunction Neighbor_1_Offset
 (
  "Neighbor_1_Offset",
  "Vec3",
  {},
  [](LP::GpTree& t)
  {
      Boid& b = *Boid::getGpPerThread();
      Vec3 no = getGpBoidNeighbor(1)->position() - b.position();
      return std::any(b.localizeDirection(no));
  });

inline LP::GpFunction First_Obs_Dist
 (
  "First_Obs_Dist", "Scalar", {},
  [](LP::GpTree& t)
  {
      Boid& boid = *Boid::getGpPerThread();
      double distance = std::numeric_limits<double>::infinity();
      auto collisions = boid.get_predicted_obstacle_collisions();
      if (collisions.size() > 0)
      {
          const Collision& first_collision = collisions.front();
          Vec3 poi = first_collision.point_of_impact;
          distance = (poi - boid.position()).length();
      }
      return std::any(distance);
  });

inline LP::GpFunction First_Obs_Normal
 (
  "First_Obs_Normal", "Vec3", {},
  [](LP::GpTree& t)
  {
      Boid& boid = *Boid::getGpPerThread();
      Vec3 normal;
      auto collisions = boid.get_predicted_obstacle_collisions();
      if (collisions.size() > 0)
      {
          const Collision& first_collision = collisions.front();
          normal = first_collision.normal_at_poi;
          normal = boid.localizeDirection(normal);
      }
      return std::any(normal);
  });


inline LP::GpFunction To_Forward
 (
  "To_Forward",
  "Vec3",
  {"Vec3"},
  [](LP::GpTree& tree)
  {
      Vec3 value = clean(tree.evalSubtree<Vec3>(0));
      Boid& b = *Boid::getGpPerThread();
      // Take component of "value" which is parallel to "forward".
      Vec3 parallel = value.parallel_component(b.forward());
      return std::any(b.localizeDirection(parallel));
  });

inline LP::GpFunction To_Side
 (
  "To_Side",
  "Vec3",
  {"Vec3"},
  [](LP::GpTree& tree)
  {
      Vec3 value = clean(tree.evalSubtree<Vec3>(0));
      Boid& b = *Boid::getGpPerThread();
      // Take component of "value" perpendicular to "forward".
      Vec3 perp = value.perpendicular_component(b.forward());
      return std::any(b.localizeDirection(perp));
  });

//------------------------------------------------------------------------------
// Cartoonishly high level Boid API for debugging:
// Speed_Control, Avoid_Obstacle, Adjust_Neighbor_Dist

inline LP::GpFunction Speed_Control
 ("Speed_Control",
  "Vec3",
  {},
  [](LP::GpTree& tree)
  {
     Boid& b = *Boid::getGpPerThread();
     Vec3 sfsc = b.steerForSpeedControl();
     double max_force = 25; // TODO WARNING inline constant!!
     Vec3 local_speed_control = b.localizeDirection(sfsc);
     if (b.speed() <= 0) { local_speed_control = Vec3(0, 0, 1); }
     return std::any(local_speed_control * max_force);
 });

inline LP::GpFunction Avoid_Obstacle
 (
  "Avoid_Obstacle",
  "Vec3",
  {},
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
              avoidance = b.localizeDirection(avoidance);
          }
      }
      return std::any(avoidance);
  });

inline LP::GpFunction Adjust_Neighbor_Dist
 (
  "Adjust_Neighbor_Dist",
  "Vec3",
  {},
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
      
      steering = b.localizeDirection(steering);
      
      return std::any(steering * 10);
  });


// FunctionSet for the GP version of EvoFlock.
LP::FunctionSet evoflock_gp_function_set()
{
    return
    {
        // GpTypes
        {
            { "Vec3" },
            //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
            // TODO 20251015 verify Scalar type does not clip
            { "Scalar", -10.0, 10.0 },
            //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
        },

        // GpFunctions
        {
#ifdef USE_ONLY_SPEED_CONTROL

            Speed_Control,

#else  // USE_ONLY_SPEED_CONTROL

            // Scalar functions:
            Abs, Add, Sub, Mult, Power,

            // Vector functions:
            V3, Add_v3, Sub_v3, Scale_v3,
            Length, Normalize, Cross, Dot,
            Parallel_Component, Perpendicular_Component,
            Interpolate, If_Pos,
            
            // Boid API:
            Speed, Velocity, Acceleration, Forward,
            Neighbor_1_Velocity, Neighbor_1_Offset,
            First_Obs_Dist, First_Obs_Normal,
            To_Forward, To_Side,

            // Cartoonishly high level Boid API for debugging:
            Speed_Control, Avoid_Obstacle, Adjust_Neighbor_Dist,

#endif  // USE_ONLY_SPEED_CONTROL
        }
    };
}


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
                    if (boid.speed() < 16) { speed_adjust = forward_weighted; }
                    if (boid.speed() > 24) { speed_adjust = -forward_weighted; }

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
                    
                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    // TODO 20240816 VERY temp experiment
                    
                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    // TODO 20251013 clean up Boid::pre_GP_steer_to_flock() etc.

//                    return std::any(boid.pre_GP_steer_to_flock());
                    return std::any(boid.steerToFlockForGA());

                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    
                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

                }
            },
        }
    };
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



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
