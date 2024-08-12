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

#include "flock.h"
#include "shape.h"
// TODO 20240226 For now, a modified copy of LazyPredator is in a subdirectory.
#include "LazyPredator/LazyPredator.h"
namespace LP = LazyPredator;

namespace GP
{

// Abbreviated name for this overly-long class name.
typedef LazyPredator::MultiObjectiveFitness MOF;

// Fitness function, simply returns Individual's tree's value (computing it and
// caching it on first call).
inline MOF evoflock_ga_fitness_function(LP::Individual* individual)
{
    return std::any_cast<MOF>(individual->tree().getRootValue());
}

inline MOF run_gp_flock_simulation(LP::Individual* individual,
                                   bool write_file);

inline MOF run_gp_flock_simulation(LP::Individual* individual)
{
    return run_gp_flock_simulation(individual, false);
}

// Fitness function, runs a flock simulation using evolved tree for steering
inline MOF evoflock_gp_fitness_function(LP::Individual* individual)
{
//    std::cout << "#################################" << std::endl;
//    std::cout << "in evoflock_gp_fitness_function()" << std::endl;
    return run_gp_flock_simulation(individual);
}


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

// Take the hypervolume of MultiObjectiveFitness elements (basically product).
inline double scalarize_fitness_hyperVolume(MOF mof)
{
    return mof.hyperVolume();
}

// Map a MultiObjectiveFitness to a scalar. Used as the FitnessScalarizeFunction
// for Population::evolutionStep(). Usually one of scalarize_fitness_min(),
// scalarize_fitness_prod(), or scalarize_fitness_length();

inline std::function<double(MOF)> scalarize_fitness = scalarize_fitness_min;

//inline std::function<double(MOF)> scalarize_fitness = scalarize_fitness_hyperVolume;

// 20240802
inline std::vector<std::string> mof_names()
{
    return (Boid::GP_not_GA ?
            std::vector<std::string>(
                                     {
                                         "avoid",
                                         "separate",
                                         "cohere",
                                         "cluster",
                                         "occupied"

                                     }) :
            std::vector<std::string>(
                                     {
                                         "separate",
                                         "avoid",
                                         "cohere",
                                     }));
}

inline MOF multiObjectiveFitnessOfFlock(const Flock& flock)
{
    double separate = flock.get_separation_score();
    double avoid = flock.get_avoid_obstacle_score();
    double cohere = flock.get_cohere_score();
    double cluster = flock.get_cluster_score();
    // double curvature = flock.get_curvature_score();
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240710 change speed fitness back to fraction of speed-ok chunks.
    // double speed = flock.get_gp_speed_score();
    // double speed = flock.get_speed_score();
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    auto ignore_function = [](Vec3 p) { return p.length() > 50;};
    double occupy = flock.occupancy_map.fractionOccupied(ignore_function);

    return (Boid::GP_not_GA ?
            MOF(
                {
                    // 20240802
                    avoid,
                    separate,
                    cohere,
                    cluster,
                    occupy,

                }) :
            MOF(
                {
                    // 20240723
                    separate,
                    avoid,
                    cohere,
                    // cluster,
                    // curvature,
                    // occupy,
                })
            );
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// Return a FlockParameters object with all given parameter values
inline FlockParameters init_fp(double max_force,
                               double max_speed,
                               double min_speed,
                               double speed,
                               
                               double weight_forward,
                               double weight_separate,
                               double weight_align,
                               double weight_cohere,
                               double weight_avoid,
                               
                               double max_dist_separate_in_body_radii,
                               
                               double exponent_separate,
                               double exponent_align,
                               double exponent_cohere,
                               
                               double angle_separate,
                               double angle_align,
                               double angle_cohere,
                               
                               double fly_away_max_dist_in_br,
                               double min_time_to_collide)
{
    FlockParameters fp;
    fp.max_force = max_force;
    fp.max_speed = std::max(min_speed, max_speed);
    fp.min_speed = std::min(min_speed, max_speed);
    fp.speed = util::clip(speed, min_speed, max_speed);
    fp.weight_forward = weight_forward;
    fp.weight_separate = weight_separate;
    fp.weight_align = weight_align;
    fp.weight_cohere = weight_cohere;
    fp.weight_avoid = weight_avoid;
    fp.max_dist_separate = max_dist_separate_in_body_radii * fp.body_radius;
    //fp.exponent_separate = exponent_separate;  // stay at 1 like in hand-tuned
    //fp.exponent_align = exponent_align;        // stay at 1 like in hand-tuned
    //fp.exponent_cohere = exponent_cohere;      // stay at 1 like in hand-tuned
    fp.angle_separate = angle_separate;
    fp.angle_align = angle_align;
    fp.angle_cohere = angle_cohere;
    fp.fly_away_max_dist_in_br = fly_away_max_dist_in_br;
    fp.min_time_to_collide = min_time_to_collide;
    return fp;
}


// Initialize basic run parameters of Flock object
inline void init_flock(Flock& flock)
{
    flock.set_boid_count(200);
    flock.set_fixed_fps(30);
    flock.set_fixed_time_step(true);
    flock.set_max_simulation_steps(500);  // 20240513: was 1000 before.
    flock.setLogStatInterval(flock.max_simulation_steps());
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


inline bool print_occupancy_map = false;  // Just for debugging.


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


// Run flock simulation with given parameters "runs" times and returns the MOF
// with the LEAST scalar fitness score.
// TODO 20240515 maybe the name should be measure_fitness_of_fp() ?
inline MOF run_flock_simulation(const FlockParameters& fp, bool write_file = false)
{
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
        flock.setSaveBoidCenters(write_file);
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
        }
    };
        
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

// TODO 20240622 just temporary for debugging
std::map<LP::Individual*, Vec3> values_of_individuals;
std::map<LP::Individual*, LP::GpTree> trees_of_individuals;

inline MOF run_gp_flock_simulation(LP::Individual* individual, bool write_file)
{
    assert(Boid::GP_not_GA);

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

        flock.override_steer_function = [&]()
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

// Run flock simulation with given parameters, return a MultiObjectiveFitness.
inline MOF run_hand_tuned_flock_simulation(bool write_flock_data_file = false)
{
    return run_flock_simulation(FlockParameters(), write_flock_data_file);
}


// Wrote this to run at the end of evolution on the top-10 fitness individuals
// of population in order to record flock data for playback
inline MOF rerun_flock_simulation(const LazyPredator::Individual* individual)
{
    // Is this tree copy needed to avoid using the previous cached tree root?
    LazyPredator::GpTree t = individual->tree();
    return run_flock_simulation(init_fp(t.evalSubtree<double>(0),
                                        t.evalSubtree<double>(1),
                                        t.evalSubtree<double>(2),
                                        t.evalSubtree<double>(3),
                                        t.evalSubtree<double>(4),
                                        t.evalSubtree<double>(5),
                                        t.evalSubtree<double>(6),
                                        t.evalSubtree<double>(7),
                                        t.evalSubtree<double>(8),
                                        t.evalSubtree<double>(9),
                                        t.evalSubtree<double>(10),
                                        t.evalSubtree<double>(11),
                                        t.evalSubtree<double>(12),
                                        t.evalSubtree<double>(13),
                                        t.evalSubtree<double>(14),
                                        t.evalSubtree<double>(15),
                                        t.evalSubtree<double>(16),
                                        t.evalSubtree<double>(17)),
                                true);  // write flock data file
}



// So for example, here is the first random tree from:
//
//     int max_tree_size = 20;
//     std::string root_type = "Fitness_0_1";
//     FunctionSet::makeRandomTree(max_tree_size, root_type, tree):
//
//     Run_Flock(57.4155, 35.5876, 95.5118, 92.8331,           // speeds
//               78.8442, 13.9993, 97.2554, 37.0498, 39.6934,  // weights
//               90.704,                                       // max dist sep
//               5.53134, 9.07756, 1.08763,                    // exponents
//               0.534839, 0.0996812, 0.405398)                // angle


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
    //std::cout << "parent0:   " << parent0.to_string()   << std::endl;
    //std::cout << "parent1:   " << parent1.to_string()   << std::endl;
    //std::cout << "offspring: " << offspring.to_string() << std::endl;
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
double jiggle_scale = 0.1;

// This is a degenerate GP function set, for what is essentially a GA problem:
// selecting a set of real number parameters for a flock simulation, via an
// absolute and fixed fitness metric. There is only one function, all GpTrees
// are exactly one function deep, differing only in their parameter values.
LazyPredator::FunctionSet evoflock_ga_function_set =
{
    {
        { "Multi_Objective_Fitness" },
        { "Real_0_1",    0.0,   1.0, jiggle_scale },
        { "Real_0_10",   0.0,  10.0, jiggle_scale },
        { "Real_0_100",  0.0, 100.0, jiggle_scale },
        { "Real_0_200",  0.0, 200.0, jiggle_scale },  // TODO keep?
        { "Real_m1_p1", -1.0,  +1.0, jiggle_scale },
        { "Real_20_20",  20.0,  20.0, 0 },            // for boid speed values
    },
    {
        {
            // GP function name:
            "Run_Flock",
            
            // Return type (in this case it returns a MultiObjectiveFitness):
            "Multi_Objective_Fitness",
            
            // Function parameter type list:
            //     TODO cf "FlockParameters" for details
            //     TODO note that I slightly reordering / reparameterizing
            //     TODO should body_radius be held constant at 0.5?
            {
                "Real_0_100",  // max_force
                // 20240427 Policy change: specify rather than optimize speed:
                "Real_20_20",  // max_speed
                "Real_20_20",  // min_speed
                "Real_20_20",  // speed
                
                "Real_0_100",  // weight_forward
                "Real_0_100",  // weight_separate
                "Real_0_100",  // weight_align
                "Real_0_100",  // weight_cohere
                "Real_0_100",  // weight_avoid
                
                "Real_0_200",  // max_dist_separate_in_body_radii
                // TODO set to 100, essentially infinity, in the FlockParameters
                // class. Keep them that way for now but needs to be revisited.
                //"Real_0_200",  // max_dist_align_in_body_radii
                //"Real_0_200",  // max_dist_cohere_in_body_radii
                
                "Real_0_10",   // exponent_separate
                "Real_0_10",   // exponent_align
                "Real_0_10",   // exponent_cohere
                
                // Cosine of threshold angle (max angle from forward to be seen)
                "Real_m1_p1",  // angle_separate
                "Real_m1_p1",  // angle_align
                "Real_m1_p1",  // angle_cohere
                
                "Real_0_100", // fly_away_max_dist_in_br
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
                FlockParameters fp = init_fp(t.evalSubtree<double>(0),
                                             t.evalSubtree<double>(1),
                                             t.evalSubtree<double>(2),
                                             t.evalSubtree<double>(3),
                                             t.evalSubtree<double>(4),
                                             t.evalSubtree<double>(5),
                                             t.evalSubtree<double>(6),
                                             t.evalSubtree<double>(7),
                                             t.evalSubtree<double>(8),
                                             t.evalSubtree<double>(9),
                                             t.evalSubtree<double>(10),
                                             t.evalSubtree<double>(11),
                                             t.evalSubtree<double>(12),
                                             t.evalSubtree<double>(13),
                                             t.evalSubtree<double>(14),
                                             t.evalSubtree<double>(15),
                                             t.evalSubtree<double>(16),
                                             t.evalSubtree<double>(17));
                auto fitness = run_flock_simulation(fp);
                return std::any(fitness);
            }
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        }
    }
};


// Called each step to handle writing log file with fitness over time data.
void save_fitness_time_series(LP::Population& population)
{
    int step_frequency = 300;
    static std::string pathname;
    if (0 == population.getStepCount() % step_frequency)
    {
        if (pathname.empty())
        {
            pathname = "/Users/cwr/Desktop/flock_data/fitness_data.csv";
            std::ofstream stream(pathname);
            stream << "step,average,best" << std::endl;
            stream.close();
        }
        std::ofstream stream(pathname, std::ios_base::app);
        stream << population.getStepCount() << ",";
        stream << population.averageFitness() << ",";
        stream << population.bestFitness()->getFitness() << std::endl;
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

double clean_num(double x)
{
    bool unclean = (std::isnan(x) or
                    std::isinf(x) or
                    x < std::numeric_limits<double>::min());
    return (unclean ? 0 : x);
}

Vec3 clean_vec3(Vec3 v)
{
    return { clean_num(v.x()), clean_num(v.y()), clean_num(v.z()) };
}

Vec3 ensure_unit_length(Vec3 v)
{
    return (v.is_unit_length() ? v : Vec3(1, 0, 0));
}

// FuntionSet for the GP version of EvoFlock.

#ifdef eval_const_20240628
LP::FunctionSet evoflock_gp_function_set() { return evoflock_ga_function_set;}
#else  // eval_const_20240628

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240712 experiment: get rid of GpType "Scalar_5" (replace with
//               "Scalar_100") to promote more "mixability" in the DSL.

LP::FunctionSet evoflock_gp_function_set()
{
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
            {
                "Power", "Scalar_100", {"Scalar_100", "Scalar_100"},
                [](LP::GpTree& tree)
                {
                    double base = tree.evalSubtree<double>(0);
                    double expt = tree.evalSubtree<double>(1);
                    return std::any(clean_num(std::pow(base, expt)));
                }
            },
            {
                "Abs", "Scalar_100", {"Scalar_100"},
                [](LP::GpTree& tree)
                {
                    double x = tree.evalSubtree<double>(0);
                    return std::any(clean_num(std::abs(x)));
                }
            },

            // Vector functions:
            {
                "Vec3", "Vec3", {"Scalar_100", "Scalar_100", "Scalar_100"},
                [](LP::GpTree& tree)
                {
                    Vec3 v(tree.evalSubtree<double>(0),
                           tree.evalSubtree<double>(1),
                           tree.evalSubtree<double>(2));
                    return std::any(clean_vec3(v));
                }
            },
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
            {
                "Cross", "Vec3", {"Vec3", "Vec3"},
                [](LP::GpTree& tree)
                {
                    return std::any(Vec3::cross(tree.evalSubtree<Vec3>(0),
                                                tree.evalSubtree<Vec3>(1)));
                }
            },
            {
                "Dot", "Scalar_100", {"Vec3", "Vec3"},
                [](LP::GpTree& tree)
                {
                    return std::any(Vec3::dot(tree.evalSubtree<Vec3>(0),
                                              tree.evalSubtree<Vec3>(1)));
                }
            },
            {
                "Parallel_Component", "Vec3", {"Vec3", "Vec3"},
                [](LP::GpTree& tree)
                {
                    Vec3 value = tree.evalSubtree<Vec3>(0);
                    Vec3 basis = tree.evalSubtree<Vec3>(1).normalize_or_0();
                    Vec3 unit_basis = ensure_unit_length(basis);
                    return std::any(value.parallel_component(unit_basis));
                }
            },
            {
                "Perpendicular_Component", "Vec3", {"Vec3", "Vec3"},
                [](LP::GpTree& tree)
                {
                    Vec3 value = tree.evalSubtree<Vec3>(0);
                    Vec3 basis = tree.evalSubtree<Vec3>(1).normalize_or_0();
                    Vec3 unit_basis = ensure_unit_length(basis);
                    return std::any(value.perpendicular_component(unit_basis));
                }
            },

            {
                "Interpolate", "Vec3", {"Scalar_100", "Vec3", "Vec3"},
                [](LP::GpTree& tree)
                {
                    double i = tree.evalSubtree<double>(0);
                    Vec3 a = tree.evalSubtree<Vec3>(1);
                    Vec3 b = tree.evalSubtree<Vec3>(2);
//                    return std::any(util::interpolate(util::clip01(i), a, b));
                    return std::any(util::interpolate(i, a, b));
                }
            },
            {
                "Ifle", "Vec3", {"Scalar_100", "Scalar_100", "Vec3", "Vec3"},
                [](LP::GpTree& tree)
                {
                    double i = tree.evalSubtree<double>(0);
                    double j = tree.evalSubtree<double>(1);
                    Vec3 a = tree.evalSubtree<Vec3>(2);
                    Vec3 b = tree.evalSubtree<Vec3>(3);
                    return std::any(i <= j ? a : b);
                }
            },

            // Boid API:
            {
                "Velocity", "Vec3", {},
                [](LP::GpTree& t)
                {
                    return std::any(Boid::getGpPerThread()->velocity());
                }
            },
            {
                "Acceleration", "Vec3", {},
                [](LP::GpTree& t)
                {
                    return std::any(Boid::getGpPerThread()->getAcceleration());
                }
            },
            
            {
                "Neighbor_1_Velocity", "Vec3", {},
                [](LP::GpTree& t)
                {
                    return std::any(getGpBoidNeighbor(1)->velocity());
                    
                }
            },
            {
                "Neighbor_1_Offset", "Vec3", {},
                [](LP::GpTree& t)
                {
                    return std::any(getGpBoidNeighbor(1)->position() -
                                    Boid::getGpPerThread()->position());
                }
            },

            {
                "Neighbor_2_Velocity", "Vec3", {},
                [](LP::GpTree& t)
                {
                    return std::any(getGpBoidNeighbor(2)->velocity());
                    
                }
            },
            {
                "Neighbor_2_Offset", "Vec3", {},
                [](LP::GpTree& t)
                {
                    return std::any(getGpBoidNeighbor(2)->position() -
                                    Boid::getGpPerThread()->position());
                }
            },
            {
                "Neighbor_3_Velocity", "Vec3", {},
                [](LP::GpTree& t)
                {
                    return std::any(getGpBoidNeighbor(3)->velocity());
                    
                }
            },
            {
                "Neighbor_3_Offset", "Vec3", {},
                [](LP::GpTree& t)
                {
                    return std::any(getGpBoidNeighbor(3)->position() -
                                    Boid::getGpPerThread()->position());
                }
            },
            {
                "First_Obs_Dist", "Scalar_100", {},
                [](LP::GpTree& t)
                {
                    Boid& boid = *Boid::getGpPerThread();
                    double distance = std::numeric_limits<double>::infinity();
                    auto collisions = boid.get_predicted_obstacle_collisions();
                    
                    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
                    // TOODO 20240809 why is obstacle avoidance broken?
                    assert(not collisions.empty());
                    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
                    
                    if (collisions.size() > 0)
                    {
                        const Collision& first_collision = collisions.front();
                        
                        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
                        // TOODO 20240809 why is obstacle avoidance broken?
                        debugPrint(first_collision)
                        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
                        
                        Vec3 poi = first_collision.point_of_impact;
                        distance = (poi - boid.position()).length();
                    }
                    return std::any(distance);
                }
            },
            {
                "First_Obs_Normal", "Vec3", {},
                [](LP::GpTree& t)
                {
                    Boid& boid = *Boid::getGpPerThread();
                    Vec3 normal;
                    auto collisions = boid.get_predicted_obstacle_collisions();
                    
                    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
                    // TOODO 20240809 why is obstacle avoidance broken?
                    assert(not collisions.empty());
                    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
                    
                    if (collisions.size() > 0)
                    {
                        const Collision& first_collision = collisions.front();
                        
                        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
                        // TOODO 20240809 why is obstacle avoidance broken?
//                        debugPrint(first_collision)
                        
                        debugPrint(Draw().frame_counter())
                        bool log = (boid.is_first_boid() and
                                    ((Draw().frame_counter() % 100) == 0));
                        if (log) { debugPrint(first_collision) }
                        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
                        
                        normal = first_collision.normal_at_poi;
                }
                    return std::any(normal);
                }
            },
            
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
//                                ((boid.draw().frame_counter() % 20) == 0))
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
//                                std::cout << boid.draw().frame_counter() << ": ";
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
                    double min_dist = boid.speed() * boid.fp().min_time_to_collide;
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
                    Vec3 neighbor_direction = neighbor_offset / neighbor_dist;
                    double weight = 0;
                    if (neighbor_dist < 2) { weight = -1; }
                    if (neighbor_dist > 5) { weight = +1; }
                    Vec3 neighbor_dist_adjust = neighbor_direction * weight;

                    Vec3 steer = (avoidance.is_zero_length() ?
                                  neighbor_dist_adjust :
                                  avoidance);

                    Vec3 f = boid.forward();
                    Vec3 lateral = steer.perpendicular_component(boid.forward());
                    lateral = lateral.normalize_or_0();

                    // temp:
                    double e = 0.0001;
                    if (not lateral.is_zero_length())
                    {
                        assert(lateral.is_perpendicular(f, e));
                    }
                    // temp:
                    if (boid.is_first_boid() and
                        ((boid.draw().frame_counter() % 20) == 0))
                    {
                        if (collisions.size() > 0)
                        {
                            std::cout << boid.draw().frame_counter() << ": ";
                            debugPrint(collisions.front())
                        }
                        std::cout << boid.draw().frame_counter() << ": ";
                        debugPrint(lateral)
                    }

//                    return std::any(lateral * 10);
                    return std::any(lateral * 100);
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
