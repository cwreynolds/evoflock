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

// FunctionSet for the GP version of EvoFlock. (forward reference)
LP::FunctionSet& evoflockGpFunctionSet();


// Component names for MOF (multi-objective fitness) values (by mode settings).
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
            
            (EF::usingGA() ?
             std::vector<std::string>(
                                      {
                                          "avoid",
                                          "separate",
                                          "speed"
                                      }) :
             
             std::vector<std::string>(
                                      {
                                          "avoid",
                                          "separate",
                                          "speed",
                                          "alignment"
                                      })
             ));
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
            //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
            // TODO 20251208 score for boid alignment.
//            MOF(
//                {
//                    flock.obstacleCollisionsScore(),
//                    flock.separationScore(),
//                    flock.speedScore()
//                })
//            );
            
            (EF::usingGA() ?
             MOF(
                 {
                     flock.obstacleCollisionsScore(),
                     flock.separationScore(),
                     flock.speedScore()
                 }) :
             MOF(
                 {
                     flock.obstacleCollisionsScore(),
                     flock.separationScore(),
                     flock.speedScore(),
                     flock.alignmentScore()
                 })
             ));
            //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
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


// New experiment, move a bunch of debugging crap out of do_1_run()
void doOneRunDebugLogging(Boid& boid,
                          Flock& flock,
                          Vec3 steering_from_tree,
                          Flock* log_flock,
                          std::mutex& log_flock_mutex,
                          int& log_flock_selected_boid_steps)
{
    if (boid.isSelected())
    {
        // Define first flock seen as the log_flock, guard for multithreading
        {
            std::lock_guard<std::mutex> lfm(log_flock_mutex);
            if (log_flock == nullptr) { log_flock = &flock; }
        }
        // Log only for the log_flock.
        if (log_flock == &flock)
        {
            // set a temp variable on the boid for logging
            boid.log_flock = log_flock;
            assert(boid.belongsToFlock(flock));
            //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
            // TODO 20251211 refactor boid steering annotation for GP
            
            // draw yellow line from selected boid in log_flock to origin,
            // to see if first obstacle is centered there.
//            Vec3 a = boid.position();
//            auto& d = Draw::getInstance();
            
//            d.addThickLineToAnimatedFrame(a, Vec3(), Color::magenta(), 0.02);
//            // Cyan line from boid to predicted obstacle avoidance
//            CollisionList cl = boid.get_predicted_obstacle_collisions();
//            Collision first_collision = cl.at(0);
//            Vec3 poi = first_collision.point_of_impact;
//            d.addThickLineToAnimatedFrame(a, poi, Color::cyan(), 0.02);
            //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
        }
    }
}


// Constraint to keep the "sensor" API in population, by requiring each tree to
// have >=1 call to each of the sensor GpFuncs. This tests for valid trees.
inline bool evoflockGpValidateTree(const LP::GpTree& tree,
                                   const LP::FunctionSet& fs)
{
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20251227 Wait, what?! Confusion with default INITIAL speed?!
    if (not (&fs == LP::FunctionSet::xxx_current_fs))
    {
        debugPrint(&fs);
        debugPrint(LP::FunctionSet::xxx_current_fs);
        assert(&fs == LP::FunctionSet::xxx_current_fs);  // TEMP for debugging
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    bool ok = true;
    std::vector<std::string> required_gp_funcs =
    {
        "Velocity",
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20260103 replace NeighborhoodVelocity with NeighborhoodVelocityDiff.
//        "NeighborhoodVelocity",
        "NeighborhoodVelocityDiff",
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        "NeighborhoodOffset",
        "ObstacleCollisionNormal",
    };
    for (auto& name : required_gp_funcs)
    {
        if (not fs.isFunctionInTree(name, tree)) { ok = false; }
    }
    return ok;
}


double measureUsageSensorAPI(const LP::Population& population)
{
    double trees_using_full_sensor_api = 0;
    auto measure_sensor_api_usage = [&](LP::Individual* individual)
    {
        const LP::GpTree& tree = individual->tree();
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20251222 why no sensor check during create population?
        const LP::FunctionSet& fs = *LP::FunctionSet::xxx_current_fs;
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        bool full_usage = evoflockGpValidateTree(tree, fs);
        if (full_usage) { trees_using_full_sensor_api++; }
    };
    population.applyToAllIndividuals(measure_sensor_api_usage);
    return trees_using_full_sensor_api / population.getIndividualCount();
}

void logUsageSensorAPI(const LP::Population& population)
{
    double usage = measureUsageSensorAPI(population);
    double count = population.getIndividualCount();
    std::cout << std::endl;
    std::cout << "    ";  // "log_prefix"
    std::cout << std::round(usage * 100) << "% (";
    std::cout << std::round(usage * count);
    std::cout << " of " << count << " individuals)";
    std::cout << " use full sensor API.";
    std::cout << std::endl;
}


// If evolved steering seems numerically odd, substitute zero.
inline Vec3 sanitizeEvolvedSteeringForce(Vec3 s)
{
    auto bad = [](double x)
    {
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20251120 more checks in sanitizeEvolvedSteeringForce
        if (std::isnan(x)) { debugPrint(std::isnan(x))}
        if (std::isinf(x)) { debugPrint(std::isinf(x))}
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO very ad hoc
        return not (x == 0 or util::between(std::abs(x), 0.0001, 1000));
    };
    if (bad(s.x()) or bad(s.y()) or bad(s.z())) { s = {};}
    return s;
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
            // For GP, set Flock's override_steer_function_.
            flock.override_steer_function_ = [&]()
            {
                Boid& boid = *Boid::getGpPerThread();
                LP::GpTree gp_tree = individual->tree();
                // Eval tree to get steering, optionally convert local to global
                Vec3 steering_from_tree = std::any_cast<Vec3>(gp_tree.eval());
                // Since the magnitude of steering force returned by the evolved
                // program is unbounded, use an ad hoc kinematic limit.
                double max_steer_force = 80;  // make some API for setting this.
                steering_from_tree = steering_from_tree.truncate(max_steer_force);
                doOneRunDebugLogging(boid,
                                     flock,
                                     steering_from_tree,
                                     log_flock,
                                     log_flock_mutex,
                                     log_flock_selected_boid_steps);
                return sanitizeEvolvedSteeringForce(steering_from_tree);
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
        // Do each simulation run in a parallel thread.
        std::vector<std::thread> threads;
        for (int r = 0; r < runs; r++) { threads.push_back(std::thread(do_1_run)); }
        // Wait for helper threads to finish, join them with this thread.
        for (auto& t : threads) { t.join(); }
    }
    else
    {
        // Do each simulation run sequentially.
        
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
        // TODO 20251023 align counters / no drawing for multithreading.
//        for (int r = 0; r < runs; r++) { do_1_run(); }
        for (int r = 0; r < runs; r++)
        {
            do_1_run();
            log_flock_selected_boid_steps = 0;
        }
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
    }
        

    if (EF::usingGP() and
        not evoflockGpValidateTree(individual->tree(),
                                   *LP::FunctionSet::xxx_current_fs))
    {
        least_mof *= std::pow(0.5, 1.0 / least_mof.size());
    }

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
    return (v.is_unit_length() ? v : Vec3(1, 0, 0));
}


//------------------------------------------------------------------------------
// Scalar functions: abs, add, subtract, multiply, exponentiation.

inline LP::GpFunction Abs
 ("Abs",
  "Scalar",
  {"Scalar"},
  [](LP::GpTree& tree)
  {
     return std::any(std::abs(tree.evalSubtree<double>(0)));
 });

inline LP::GpFunction Add
 ("Add",
  "Scalar",
  {"Scalar", "Scalar"},
  [](LP::GpTree& tree)
  {
     return std::any(tree.evalSubtree<double>(0) + tree.evalSubtree<double>(1));
 });

inline LP::GpFunction Sub
 ("Sub",
  "Scalar",
  {"Scalar", "Scalar"},
  [](LP::GpTree& tree)
  {
     return std::any(tree.evalSubtree<double>(0) - tree.evalSubtree<double>(1));
 });

inline LP::GpFunction Mult
 ("Mult",
  "Scalar",
  {"Scalar", "Scalar"},
  [](LP::GpTree& tree)
  {
     return std::any(tree.evalSubtree<double>(0) * tree.evalSubtree<double>(1));
 });

inline LP::GpFunction Div
 ("Div",
  "Scalar",
  {"Scalar", "Scalar"},
  [](LP::GpTree& tree)
  {
     double d = tree.evalSubtree<double>(1);
     return std::any(tree.evalSubtree<double>(0) / (d == 0 ? 1 : d));
 });

inline LP::GpFunction Power
 (
  "Power",
  "Scalar",
  {"Scalar", "Scalar"},
  [](LP::GpTree& tree)
  {
      double base = tree.evalSubtree<double>(0);
      double expt = tree.evalSubtree<double>(1);
      return std::any(std::pow(base, expt));
  });

//------------------------------------------------------------------------------
//    // Vector functions: V3, Add_v3, Sub_v3, Scale_v3, Length, Normalize, Cross,
// Vector functions: V3, Add3, Sub3, Scale3, Length, Normalize, Cross,
// Dot, Parallel_Component, Perpendicular_Component, Interpolate, If_Pos

inline LP::GpFunction V3
 (
  "Vec3",
  "Vec3",
  {"Scalar", "Scalar", "Scalar"},
  [](LP::GpTree& tree)
  {
      return std::any(Vec3(tree.evalSubtree<double>(0),
                           tree.evalSubtree<double>(1),
                           tree.evalSubtree<double>(2)));
  });

inline LP::GpFunction Add3
(
    "Add3",
    "Vec3",
    {"Vec3", "Vec3"},
    [](LP::GpTree& tree)
    {
        return std::any(tree.evalSubtree<Vec3>(0) + tree.evalSubtree<Vec3>(1));
    });

inline LP::GpFunction Sub3
 (
  "Sub3",
  "Vec3",
  {"Vec3", "Vec3"},
  [](LP::GpTree& tree)
  {
      return std::any(tree.evalSubtree<Vec3>(0) - tree.evalSubtree<Vec3>(1));
  });

inline LP::GpFunction Scale3
 (
  "Scale3",
  "Vec3",
  {"Vec3", "Scalar_0_10"},
  [](LP::GpTree& tree)
  {
      return std::any(tree.evalSubtree<Vec3>(0) * tree.evalSubtree<double>(1));
  });

inline LP::GpFunction Div3
 (
  "Div3",
  "Vec3",
  {"Vec3", "Scalar_0_10"},
  [](LP::GpTree& tree)
  {
      double d = tree.evalSubtree<double>(1);
      return std::any(tree.evalSubtree<Vec3>(0) / (d == 0 ? 1 : d));
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
      Vec3 v = tree.evalSubtree<Vec3>(0);
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

inline LP::GpFunction LengthAdjust
 (
  "LengthAdjust",
  "Vec3",
  {"Vec3", "Scalar_0_100", "Scalar_0_10"},  // ref vec, target length, strength
  [](LP::GpTree& tree)
  {
      Vec3 ref_vector = tree.evalSubtree<Vec3>(0);
      
      // TODO temporarily fix with an abs(), later with special Scalar def?
//      double target_length = std::abs(tree.evalSubtree<double>(1));
//      double strength = std::abs(tree.evalSubtree<double>(2));
      double target_length = tree.evalSubtree<double>(1);
      double strength = tree.evalSubtree<double>(2);

      double adjust = strength * (ref_vector.length() < target_length ? 1 : -1);
      Vec3 result = ref_vector.normalize_or_0() * adjust;
      return std::any(result);
  });


//------------------------------------------------------------------------------
// Boid API: Speed, Velocity, Acceleration, Forward,
//           Neighbor_1_Velocity, Neighbor_1_Offset,
//           First_Obs_Dist, First_Obs_Normal, ToForward, ToSide


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
    double min_speed = 0.01;  // cf target speed usually 20.
    Vec3 v = boid.velocity();
    if (boid.speed() < min_speed) { v = boid.forward() * min_speed; }
    return std::any(v);
});


inline LP::GpFunction Acceleration
 (
  "Acceleration",
  "Vec3",
  {},
  [](LP::GpTree& t)
  {
      Boid& boid = *Boid::getGpPerThread();
      return std::any(boid.getAcceleration());
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

inline LP::GpFunction NearestNeighborVelocity
 (
  "NearestNeighborVelocity",
  "Vec3",
  {},
  [](LP::GpTree& t)
  {
      return std::any(getGpBoidNeighbor(1)->velocity());
  });

inline LP::GpFunction NearestNeighborVelocity2  // 2nd nearest neighbor
 (
  "NearestNeighborVelocity2",
  "Vec3",
  {},
  [](LP::GpTree& t)
  {
      return std::any(getGpBoidNeighbor(2)->velocity());
  });

inline LP::GpFunction NearestNeighborOffset
 (
  "NearestNeighborOffset",
  "Vec3",
  {},
  [](LP::GpTree& t)
  {
      Boid& boid = *Boid::getGpPerThread();
      Vec3 no = getGpBoidNeighbor(1)->position() - boid.position();
      return std::any(no);
  });


inline LP::GpFunction NearestNeighborOffset2  // 2nd nearest neighbor
 (
  "NearestNeighborOffset2",
  "Vec3",
  {},
  [](LP::GpTree& t)
  {
      Boid& boid = *Boid::getGpPerThread();
      Vec3 no = getGpBoidNeighbor(2)->position() - boid.position();
      return std::any(no);
  });

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20260103 replace NeighborhoodVelocity with NeighborhoodVelocityDiff.
//               also normalize by weight.

//    inline LP::GpFunction NeighborhoodVelocity
//     (
//      "NeighborhoodVelocity",
//      "Vec3",
//      {"Scalar_0.5_2"},  // falloff exponent
//      [](LP::GpTree& tree)
//      {
//          Vec3 sum;
//          Boid& me = *Boid::getGpPerThread();
//          double exponent = tree.evalSubtree<double>(0);
//          for (int i = 1; i < 7; i++)
//          {
//              Boid& b = *getGpBoidNeighbor(i);
//              Vec3 offset = b.position() - me.position();
//              double distance = offset.length();
//              sum += b.velocity() * (1.0 / std::pow(distance, exponent));
//          }
//          return std::any(sum);;
//      });

inline LP::GpFunction NeighborhoodVelocity
 (
  "NeighborhoodVelocity",
  "Vec3",
  {"Scalar_0.5_2"},  // falloff exponent
  [](LP::GpTree& tree)
  {
//      Vec3 sum;
      Vec3 velocity_sum;
      double weight_sum = 0;

      Boid& me = *Boid::getGpPerThread();
      double exponent = tree.evalSubtree<double>(0);
      for (int i = 1; i < 7; i++)
      {
          Boid& b = *getGpBoidNeighbor(i);
          Vec3 offset = b.position() - me.position();
          double distance = offset.length();
//          sum += b.velocity() * (1.0 / std::pow(distance, exponent));
          
          double weight = 1.0 / std::pow(distance, exponent);
          velocity_sum += b.velocity() * weight;
          weight_sum += weight;
      }
//      return std::any(sum);;
      return std::any(velocity_sum / weight_sum);
  });


// "Pre subtracts" from this boid's velocity. We want to drive this to zero.
inline LP::GpFunction NeighborhoodVelocityDiff
 (
  "NeighborhoodVelocityDiff",
  "Vec3",
  {"Scalar_0.5_2"},  // falloff exponent
  [](LP::GpTree& tree)
  {
      Vec3 velocity_sum;
      double weight_sum = 0;
      Boid& me = *Boid::getGpPerThread();
      double exponent = tree.evalSubtree<double>(0);
      for (int i = 1; i < 7; i++)
      {
          Boid& b = *getGpBoidNeighbor(i);
          Vec3 offset = b.position() - me.position();
          double distance = offset.length();
          double weight = 1.0 / std::pow(distance, exponent);
          velocity_sum += b.velocity() * weight;
          weight_sum += weight;
      }
      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // TODO 20260105 sanity check, is this backwards? Should I allow negative weights?
//      return std::any((velocity_sum / weight_sum) - me.velocity());
      return std::any(me.velocity() - (velocity_sum / weight_sum));
      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  });



//    inline LP::GpFunction NeighborhoodOffset
//     (
//      "NeighborhoodOffset",
//      "Vec3",
//      {"Scalar_0.5_2"},  // falloff exponent
//      [](LP::GpTree& tree)
//      {
//          Vec3 sum;
//          Boid& me = *Boid::getGpPerThread();
//          double exponent = tree.evalSubtree<double>(0);
//          for (int i = 1; i < 7; i++)
//          {
//              Boid& b = *getGpBoidNeighbor(i);
//              Vec3 offset = b.position() - me.position();
//              double distance = offset.length();
//              sum += offset * (1.0 / std::pow(distance, exponent));
//          }
//          return std::any(sum);;
//      });

inline LP::GpFunction NeighborhoodOffset
 (
  "NeighborhoodOffset",
  "Vec3",
  {"Scalar_0.5_2"},  // falloff exponent
  [](LP::GpTree& tree)
  {
//      Vec3 sum;
      Vec3 offset_sum;
      double weight_sum = 0;
      Boid& me = *Boid::getGpPerThread();
      double exponent = tree.evalSubtree<double>(0);
      for (int i = 1; i < 7; i++)
      {
          Boid& b = *getGpBoidNeighbor(i);
          Vec3 offset = b.position() - me.position();
          double distance = offset.length();
//          sum += offset * (1.0 / std::pow(distance, exponent));
          
          double weight = 1.0 / std::pow(distance, exponent);
          offset_sum += offset * weight;
          weight_sum += weight;

      }
//      return std::any(sum);;
      return std::any(offset_sum / weight_sum);
  });

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

inline LP::GpFunction First_Obs_Dist
 (
  "First_Obs_Dist",
  "Scalar",
  {},
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

inline LP::GpFunction FirstObstacleOffset
(
 "FirstObstacleOffset",
 "Vec3",
 {},
 [](LP::GpTree& t)
 {
    Boid& boid = *Boid::getGpPerThread();
    Vec3 offset;
    auto collisions = boid.get_predicted_obstacle_collisions();
    if (collisions.size() > 0)
    {
        const Collision& first_collision = collisions.front();
        Vec3 poi = first_collision.point_of_impact;
        offset = poi - boid.position();
    }
    return std::any(offset);
});

inline LP::GpFunction FirstObstacleNormal
 (
  "FirstObstacleNormal",
  "Vec3",
  {},
  [](LP::GpTree& t)
  {
      Boid& boid = *Boid::getGpPerThread();
      Vec3 normal;
      auto collisions = boid.get_predicted_obstacle_collisions();
      if (collisions.size() > 0)
      {
          const Collision& first_collision = collisions.front();
          normal = first_collision.normal_at_poi;
      }
      return std::any(normal);
  });


inline LP::GpFunction ObstacleCollisionNormal
 (
  "ObstacleCollisionNormal",
  "Vec3",
  {"Scalar_0_1"}, // time_to_collision_threshold
  [](LP::GpTree& tree)
  {
      double time_to_collision_threshold = tree.evalSubtree<double>(0);
      Boid& boid = *Boid::getGpPerThread();
      Vec3 normal;
      auto collisions = boid.get_predicted_obstacle_collisions();
      if (collisions.size() > 0)
      {
          const Collision& first_collision = collisions.front();
          if (first_collision.time_to_collision < time_to_collision_threshold)
          {
              normal = first_collision.normal_at_poi;
          }
      }
      return std::any(normal);
  });


//~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

inline LP::GpFunction ToForward
 (
  "ToForward",
  "Vec3",
  {"Vec3"},
  [](LP::GpTree& tree)
  {
      Vec3 value = tree.evalSubtree<Vec3>(0);
      Boid& boid = *Boid::getGpPerThread();
      // Take component of "value" which is parallel to "forward".
      Vec3 parallel = value.parallel_component(boid.forward());
      return std::any(parallel);
  });


inline LP::GpFunction ToSide
 (
  "ToSide",
  "Vec3",
  {"Vec3"},
  [](LP::GpTree& tree)
  {
      Vec3 value = tree.evalSubtree<Vec3>(0);
      Boid& boid = *Boid::getGpPerThread();
      // Take component of "value" perpendicular to "forward".
      Vec3 perpendicular = value.perpendicular_component(boid.forward());
      return std::any(perpendicular);
  });


// Given two scalars and a Vec3, "stretch" the vector along the Boids' local
// forward and lateral directions.
inline LP::GpFunction LocalScale
 (
  "LocalScale",
  "Vec3",
  {"Scalar", "Scalar", "Vec3"},
  [](LP::GpTree& tree)
  {
      double forward = tree.evalSubtree<double>(0);
      double side = tree.evalSubtree<double>(1);
      Vec3 vector = tree.evalSubtree<Vec3>(2);
      Boid& boid = *Boid::getGpPerThread();
      return std::any(Vec3(vector.dot(boid.forward()) * forward,
                           vector.dot(boid.side()) * side,
                           vector.dot(boid.up()) * side));
  });


inline LP::GpFunction SideAndForward
 (
  "SideAndForward",
  "Vec3",
  {"Vec3", "Vec3"},  // side, forward
  [](LP::GpTree& tree)
  {
      Vec3 side = tree.evalSubtree<Vec3>(0);
      Vec3 forward = tree.evalSubtree<Vec3>(1);
      Boid& boid = *Boid::getGpPerThread();
      // Add perpendicular component of "side" to parallel of "forward".
      return std::any(side.perpendicular_component(boid.forward()) +
                      forward.parallel_component(boid.forward()));
  });


//------------------------------------------------------------------------------
// Cartoonishly high level Boid API for debugging:
// SpeedControl, AvoidObstacle, AdjustSeparation, and BeTheBoid

// New SpeedControl(target_speed, strength). Only medium cartoonish?
inline LP::GpFunction SpeedControl
 ("SpeedControl",
  "Vec3",
  {"Scalar", "Scalar"},
  [](LP::GpTree& tree)
  {
     // TODO try forcing all parameters to be positive? Seems very ad hoc.
     double target_speed = std::abs(tree.evalSubtree<double>(0));
     double strength = std::abs(tree.evalSubtree<double>(1));

     Boid& boid = *Boid::getGpPerThread();
     // Returns a global vector, parallel to forward(), magnitude on [-1,+1].
     Vec3 sfsc = boid.steerForSpeedControl(target_speed);
     return std::any(sfsc * strength);
 });


inline LP::GpFunction AvoidObstacle
 (
  "Avoid_Obstacle",
  "Vec3",
  {},
  [](LP::GpTree& tree)
  {
      double min_dist = 25;
      Boid& boid = *Boid::getGpPerThread();
      Vec3 avoidance;
      auto collisions = boid.get_predicted_obstacle_collisions();
      if (collisions.size() > 0)
      {
          const Collision& first_collision = collisions.front();
          Vec3 poi = first_collision.point_of_impact;
          double distance = (poi - boid.position()).length();
          if (distance > min_dist)
          {
              Vec3 normal = first_collision.normal_at_poi;
              avoidance = normal.parallel_component(boid.forward());
          }
      }
      return std::any(avoidance);
  });


inline LP::GpFunction AdjustSeparation
 (
  "AdjustSeparation",
  "Vec3",
  {"Scalar", "Scalar", "Scalar"},  // min_distance, max_distance, strength
  [](LP::GpTree& tree)
  {
      // TODO try forcing all parameters to be positive? Seems very ad hoc.
      double a = std::abs(tree.evalSubtree<double>(0));
      double b = std::abs(tree.evalSubtree<double>(1));
      double min_distance = std::min(a, b);
      double max_distance = std::max(a, b);
      double strength = std::abs(tree.evalSubtree<double>(2));
      
      Boid& boid = *Boid::getGpPerThread();
      Vec3 neighbor_offset = getGpBoidNeighbor(1)->position() - boid.position();
      double neighbor_dist = neighbor_offset.length();
      Vec3 neighbor_direction = neighbor_offset / neighbor_dist;
      Vec3 steering;
      if (neighbor_dist < min_distance) { steering = neighbor_direction; }
      if (neighbor_dist > max_distance) { steering = -neighbor_direction; }
      return std::any(steering * strength);
  });


inline LP::GpFunction BeTheBoid
 ("BeTheBoid",
  "Vec3",
  {},
  [](LP::GpTree& tree)
  {
     Boid& boid = *Boid::getGpPerThread();
     FlockParameters fp(91.491, 55.5244, 27.0869, 24.3598, 14.2042,
                        91.187, 44.2451, 2.60944, 11.2493, 50.384,
                        -0.834407, -0.961312, 0.39291, 3.85284, 0.936081);
     boid.set_fp(&fp);
     return std::any(boid.steerToFlockForGA());
 });


LP::FunctionSet evoflock_gp_function_set_cached_ =
{
    // GpTypes
    {
        { "Vec3" },
        { "Scalar", -100.0, 100.0, 0.005 },  // min, max, jiggle scale
        {"Scalar_0_1", 0.0, 1.0, 0.01},
        {"Scalar_0_10", 0.0, 10.0, 0.1},
        {"Scalar_0_100", 0.0, 100.0, 1.0},
        // To see plot of falloff curves: https://tinyurl.com/3p4jv7dx
        {"Scalar_0.5_2", 0.5, 2.0, 0.02},  // for inverse exponential falloff.
    },
    // GpFunctions
    {
        // Vector functions:
        // V3,
        Add3,
        Sub3,
        Scale3,
        Div3,
        // Length,
        // Normalize,
        // Dot,
        // Cross,
        // Parallel_Component,
        // Perpendicular_Component,
        // Interpolate,
        // If_Pos,
        LengthAdjust,
        
        // Boid API:
        // Speed,
        Velocity,
        // Acceleration,
        // Forward,
        // NearestNeighborVelocity,
        // NearestNeighborOffset,
        // NearestNeighborVelocity2,
        // NearestNeighborOffset2,
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20260103 replace NeighborhoodVelocity with NeighborhoodVelocityDiff.
//        NeighborhoodVelocity,
        NeighborhoodVelocityDiff,
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        NeighborhoodOffset,
        // First_Obs_Dist,
        // FirstObstacleNormal,
        // FirstObstacleOffset,
        
        // FirstObstacleTimeLimitNormal,
        ObstacleCollisionNormal,

        // ToForward,
        // ToSide,
        // SideAndForward,
        
        // Cartoonishly high level Boid API for debugging:
        // SpeedControl,
        // AvoidObstacle,
        // AdjustSeparation,
        // BeTheBoid,
    }
};

// FunctionSet for the GP version of EvoFlock.
LP::FunctionSet& evoflockGpFunctionSet()
{
    return evoflock_gp_function_set_cached_;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240707 WIP on a prototype unit test for this GP module

//    void test_First_Obs_GpFuncs()
//    {
//    //    const LP::FunctionSet fs = evoflock_gp_function_set();
//        const LP::FunctionSet& fs = evoflockGpFunctionSet();
//        const LP::GpFunction* fod = fs.lookupGpFunctionByName("First_Obs_Dist");
//        const LP::GpFunction* fon = fs.lookupGpFunctionByName("First_Obs_Normal");
//
//        LP::GpTree gp_tree_fod;
//        gp_tree_fod.setRootFunction(*fod);
//        double distance = std::any_cast<double>(gp_tree_fod.eval());
//        debugPrint(distance)
//
//        LP::GpTree gp_tree_fon;
//        gp_tree_fon.setRootFunction(*fon);
//        Vec3 normal = std::any_cast<Vec3>(gp_tree_fon.eval());
//        debugPrint(normal)
//    }

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

}  // end of namespace GP
