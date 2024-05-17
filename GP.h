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
#include <algorithm>  // For std::sort.
// TODO 20240226 For now, a modified copy of LazyPredator is in a subdirectory.
#include "LazyPredator/LazyPredator.h"
namespace LP = LazyPredator;

// Abbreviated name for this overly-long class name.
typedef LazyPredator::MultiObjectiveFitness MOF;

// Fitness function, simply returns Individual's tree's value (computing it and
// caching it on first call).
inline MOF evoflock_fitness_function(LP::Individual* individual)
{
    return std::any_cast<MOF>(individual->tree().getRootValue());
}


// Take the minimum element of a MultiObjectiveFitness ("Shangian scalarizer").
inline double scalarize_fitness_min(MOF mof) { return mof.min(); }


// Take the product of MultiObjectiveFitness elements.
inline double scalarize_fitness_prod(MOF mof)
{
    return std::reduce(mof.begin(), mof.end(), 1.0, std::multiplies());
}


// Map a MultiObjectiveFitness to a scalar, here the minimum value.
// Used as the FitnessScalarizeFunction for Population::evolutionStep().
inline std::function<double(MOF)> scalarize_fitness = scalarize_fitness_min;


inline const std::vector<std::string> mof_names =
{
    "separate",
    "avoid",
    "cohere",
    "cluster",
    "curvature",
    "occupied"
};


inline MOF multiObjectiveFitnessOfFlock(const Flock& flock)
{
    double separate = flock.get_separation_score();
    double avoid = flock.get_avoid_obstacle_score();
    double cohere = flock.get_cohere_score();
    double cluster = flock.get_cluster_score();
    double curvature = flock.get_curvature_score();
    auto ignore_function = [](Vec3 p) { return p.length() > 50;};
    double occupy = flock.occupancy_map.fractionOccupied(ignore_function);
    return
    {{
        separate,
        avoid,
        cohere,
        cluster,
        curvature,
        occupy
    }};
}


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


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240513 run two flock sims for half as long, keep lower fitness

//    // Initialize basic run parameters of Flock object
//    inline void init_flock(Flock& flock)
//    {
//        flock.set_boid_count(200);
//        flock.set_fixed_fps(30);
//        flock.set_fixed_time_step(true);
//        flock.set_max_simulation_steps(1000);
//        flock.setLogStatInterval(flock.max_simulation_steps());
//        flock.setSaveBoidCenters(false);
//        flock.log_prefix = "    ";
//    }

// Initialize basic run parameters of Flock object
inline void init_flock(Flock& flock)
{
    flock.set_boid_count(200);
    flock.set_fixed_fps(30);
    flock.set_fixed_time_step(true);
    
//    flock.set_max_simulation_steps(1000);
    flock.set_max_simulation_steps(500);
    
    flock.setLogStatInterval(flock.max_simulation_steps());
    flock.setSaveBoidCenters(false);
    flock.log_prefix = "    ";
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


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
    for (auto& s : mof_names) { labels.push_back(s + " fitness"); }
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


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240513 run two flock sims for half as long, keep lower fitness

//    // Run flock simulation with given parameters, return a MultiObjectiveFitness.
//    inline MOF run_flock_simulation(const FlockParameters& fp, bool write_file = false)
//    {
//        Flock flock;
//        init_flock(flock);
//        flock.setSaveBoidCenters(write_file);
//        flock.fp() = fp;
//        flock.run();
//        MOF mof = multiObjectiveFitnessOfFlock(flock);
//        assert(mof.size() == mof_names.size());
//        fitness_logger(mof);
//        return mof;
//    }

//    // Run flock simulation with given parameters, return a MultiObjectiveFitness.
//    // Makes given number of runs, returning the MOF with least scalar fitness.
//    inline MOF run_flock_simulation(const FlockParameters& fp, bool write_file = false)
//    {
//    //    int runs = 2;
//        int runs = 4;
//        std::vector<MOF> mofs;
//        for (int r = 0; r < runs; r++)
//        {
//            Flock flock;
//            init_flock(flock);
//            flock.setSaveBoidCenters(write_file);
//            flock.fp() = fp;
//            flock.run();
//            MOF mof = multiObjectiveFitnessOfFlock(flock);
//            assert(mof.size() == mof_names.size());
//            fitness_logger(mof);
//
//            mofs.push_back(mof);
//        }
//        std::sort(mofs.begin(), mofs.end(),
//                  [](const MOF& a, const MOF& b){ return (scalarize_fitness(a) <
//                                                          scalarize_fitness(b)); });
//        MOF min_mof = mofs.at(0);
//        double min_scalar = scalarize_fitness(min_mof);
//        std::cout << "    min composite " << min_scalar << std::endl << std::endl;
//        return min_mof;
//    }


//    // Run flock simulation with given parameters, return a MultiObjectiveFitness.
//    // Makes given number of runs, returning the MOF with least scalar fitness.
//    inline MOF run_flock_simulation(const FlockParameters& fp, bool write_file = false)
//    {
//    //    int runs = 2;
//        int runs = 4;
//    //    std::vector<MOF> mofs;
//        MOF least_mof;
//        double least_scalar_fitness = std::numeric_limits<double>::infinity();
//        for (int r = 0; r < runs; r++)
//        {
//            Flock flock;
//            init_flock(flock);
//            flock.setSaveBoidCenters(write_file);
//            flock.fp() = fp;
//            flock.run();
//            MOF mof = multiObjectiveFitnessOfFlock(flock);
//            assert(mof.size() == mof_names.size());
//            fitness_logger(mof);
//
//    //        mofs.push_back(mof);
//
//    //        double sf = scalarize_fitness(mof);
//            if (least_scalar_fitness > scalarize_fitness(mof))
//            {
//                least_scalar_fitness = scalarize_fitness(mof);
//                least_mof = mof;
//            }
//        }
//    //    std::sort(mofs.begin(), mofs.end(),
//    //              [](const MOF& a, const MOF& b){ return (scalarize_fitness(a) <
//    //                                                      scalarize_fitness(b)); });
//    //    MOF min_mof = mofs.at(0);
//    //    double min_scalar = scalarize_fitness(min_mof);
//    //    std::cout << "    min composite " << min_scalar << std::endl << std::endl;
//    //    return min_mof;
//        std::cout<<"    min composite "<<least_scalar_fitness<<std::endl<<std::endl;
//        return least_mof;
//    }

//    // Run flock simulation with given parameters, return a MultiObjectiveFitness.
//    // Makes given number of runs, returning the MOF with least scalar fitness.
//    inline MOF run_flock_simulation(const FlockParameters& fp, bool write_file = false)
//    {
//        int runs = 4;
//        MOF least_mof;
//        double least_scalar_fitness = std::numeric_limits<double>::infinity();
//        std::vector<double> scalar_fits;
//        for (int r = 0; r < runs; r++)
//        {
//            Flock flock;
//            init_flock(flock);
//            flock.setSaveBoidCenters(write_file);
//            flock.fp() = fp;
//            flock.run();
//            MOF mof = multiObjectiveFitnessOfFlock(flock);
//            assert(mof.size() == mof_names.size());
//
//    //        fitness_logger(mof);
//
//    //        if (least_scalar_fitness > scalarize_fitness(mof))
//    //        {
//    //            least_scalar_fitness = scalarize_fitness(mof);
//    //            least_mof = mof;
//    //        }
//
//
//            scalar_fits.push_back(scalarize_fitness(mof));
//
//            if (least_scalar_fitness > scalar_fits.back())
//            {
//                least_scalar_fitness = scalar_fits.back();
//                least_mof = mof;
//            }
//
//        }
//
//        fitness_logger(least_mof);
//
//    //    std::cout<<"    min composite "<<least_scalar_fitness<<std::endl<<std::endl;
//
//        std::cout << "    min composite "<< least_scalar_fitness;
//        std::cout << "  {" << LP::vec_to_string(scalar_fits) << "}";
//        std::cout << std::endl << std::endl;
//
//        return least_mof;
//    }


//    // Run flock simulation with given parameters, return a MultiObjectiveFitness.
//    // Makes given number of runs, returning the MOF with least scalar fitness.
//    // TODO 20240515 maybe the name should be measure_fitness_of_fp() ?
//    inline MOF run_flock_simulation(const FlockParameters& fp, bool write_file = false)
//    {
//        int runs = 4;
//        MOF least_mof;
//        double least_scalar_fitness = std::numeric_limits<double>::infinity();
//        std::vector<double> scalar_fits;
//        for (int r = 0; r < runs; r++)
//        {
//            Flock flock;
//            init_flock(flock);
//            flock.setSaveBoidCenters(write_file);
//            flock.fp() = fp;
//            flock.run();
//            MOF mof = multiObjectiveFitnessOfFlock(flock);
//            assert(mof.size() == mof_names.size());
//            scalar_fits.push_back(scalarize_fitness(mof));
//            if (least_scalar_fitness > scalar_fits.back())
//            {
//                least_scalar_fitness = scalar_fits.back();
//                least_mof = mof;
//            }
//        }
//        fitness_logger(least_mof);
//        std::cout << "    min composite "<< least_scalar_fitness;
//        std::cout << "  {" << LP::vec_to_string(scalar_fits) << "}";
//        std::cout << std::endl << std::endl;
//        return least_mof;
//    }


// TODO 20240516 experiment with parallel simulations

//inline run_and_eval_one_flock_simulation ()
//{
//    Flock flock;
//    init_flock(flock);
//    flock.setSaveBoidCenters(write_file);
//    flock.fp() = fp;
//    flock.run();
//    MOF mof = multiObjectiveFitnessOfFlock(flock);
//    assert(mof.size() == mof_names.size());
//    scalar_fits.push_back(scalarize_fitness(mof));
//    if (least_scalar_fitness > scalar_fits.back())
//    {
//        least_scalar_fitness = scalar_fits.back();
//        least_mof = mof;
//    }
//}


// TODO 20240516 experiment with parallel simulations

// Run flock simulation with given parameters, return a MultiObjectiveFitness.
// (Now does given number of runs, returning the MOF with least scalar fitness.)
// TODO 20240515 maybe the name should be measure_fitness_of_fp() ?
inline MOF run_flock_simulation(const FlockParameters& fp, bool write_file = false)
{
    int runs = 4;
    MOF least_mof;
    double least_scalar_fitness = std::numeric_limits<double>::infinity();
    std::vector<double> scalar_fits;

//    std::recursive_mutex save_mof_mutex;
    std::mutex save_mof_mutex;

    auto do_1_run = [&]()
    {
        // These steps can happen in parallel threads:
        Flock flock;
        init_flock(flock);
        flock.setSaveBoidCenters(write_file);
        flock.fp() = fp;
        flock.run();
        MOF mof = multiObjectiveFitnessOfFlock(flock);
        // These steps happen in single thread with lock on save_mof_mutex.
        {
//            std::lock_guard<std::recursive_mutex> smm(save_mof_mutex);
            std::lock_guard<std::mutex> smm(save_mof_mutex);
            assert(mof.size() == mof_names.size());
            scalar_fits.push_back(scalarize_fitness(mof));
            if (least_scalar_fitness > scalar_fits.back())
            {
                least_scalar_fitness = scalar_fits.back();
                least_mof = mof;
            }
        }
    };
    
    
//    for (int r = 0; r < runs; r++)
//    {
//        Flock flock;
//        init_flock(flock);
//        flock.setSaveBoidCenters(write_file);
//        flock.fp() = fp;
//        flock.run();
//        MOF mof = multiObjectiveFitnessOfFlock(flock);
//        assert(mof.size() == mof_names.size());
//        scalar_fits.push_back(scalarize_fitness(mof));
//        if (least_scalar_fitness > scalar_fits.back())
//        {
//            least_scalar_fitness = scalar_fits.back();
//            least_mof = mof;
//        }
//    }

    // Start thread for each simulation run.
    std::vector<std::thread> threads;
    
#if 1
    for (int r = 0; r < runs; r++) { do_1_run(); }
#else
    for (int r = 0; r < runs; r++) { threads.push_back(std::thread(do_1_run)); }
    // Wait for helper threads to finish, join them with this thread.
    for (auto& t : threads) { t.join(); }
#endif
    
    assert(scalar_fits.size() == runs);

    fitness_logger(least_mof);
    std::cout << "    min composite "<< least_scalar_fitness;
    std::cout << "  {" << LP::vec_to_string(scalar_fits) << "}";
    std::cout << std::endl << std::endl;
    return least_mof;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


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
        if (LP::LPRS().randomBool())
        {
            offspring.getSubtree(i) = parent1.getSubtree(i);
        }
    }
    //std::cout << "parent0:   " << parent0.to_string()   << std::endl;
    //std::cout << "parent1:   " << parent1.to_string()   << std::endl;
    //std::cout << "offspring: " << offspring.to_string() << std::endl;
}

// An experiment to compare min and prod for MOF components.
void replace_scalar_fitness_with_product(LP::Population& population)
{
    auto replace = [](LP::Individual* individual)
    {
        MOF mof = individual->getMultiObjectiveFitness();
        individual->setFitness(scalarize_fitness_prod(mof));
    };
    population.applyToAllIndividuals(replace);
    scalarize_fitness = scalarize_fitness_prod;
}


// The default (in GpType::defaultJiggleScale()) is 0.05
double jiggle_scale = 0.1;

// This is a degenerate GP function set, for what is essentially a GA problem:
// selecting a set of real number parameters for a flock simulation, via an
// absolute and fixed fitness metric. There is only one function, all GpTrees
// are exactly one function deep, differing only in their parameter values.
LazyPredator::FunctionSet evoflock_gp_function_set =
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
            
            // Evaluation function, which runs a flock simulation with the given
            // parameters and returns the fitness.
            [](LazyPredator::GpTree& t)
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
        }
    }
};
