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

// TODO 20240226 For now, a modified copy of LazyPredator is in a subdirectory.
#include "LazyPredator/LazyPredator.h"

namespace LP = LazyPredator;


// Fitness function, simply returns Individual's tree's value (computing it and
// caching it on first call).
inline float evoflock_fitness_function(LazyPredator::Individual* individual)
{
    return std::any_cast<double>(individual->tree().getRootValue());
}

// TODO 20240309 temp?
void fitness_logger(double minsep_fitness, double minsep,
                    double maxsep_fitness, double maxsep,
                    double collide_fitness, double collide,
                    double ave_speed_fitness, double ave_speed,
                    double fitness)
{
    // Save current settings for formatting float/double values
    std::ios::fmtflags old_settings = std::cout.flags();
    size_t old_precision = std::cout.precision();
    
    auto print = [](std::string label, double fitness, double raw = -1)
    {
        std::cout << label;
        std::cout << std::setprecision(6) << std::setw(10) << std::fixed;
        std::cout << fitness;
        if (raw > -1) { std::cout << " (" << raw <<  ")"; }
        std::cout << std::endl;
    };
    print("    minsep    ", minsep_fitness,    minsep);
    print("    maxsep    ", maxsep_fitness,    maxsep);
    print("    collide   ", collide_fitness,   collide);
    print("    ave_speed ", ave_speed_fitness, ave_speed);
    print("    fitness   ", fitness);
    std::cout << std::endl;
    
    // restore output format flags and precision
    std::cout.flags(old_settings);
    std::cout.precision(old_precision);
}

// Run flock simulation with given parameters, return a scalar fitness on [0,1].
inline double run_flock_simulation(double max_force,
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
                                   
                                   bool write_flock_data_file = false)
{
    // Initialize Flock object and run simulation.
    Flock flock;
    flock.set_boid_count(200);
    flock.set_fixed_fps(30);
    flock.set_fixed_time_step(true);
    flock.set_max_simulation_steps(1000);
    flock.setLogStatInterval(1000);
    flock.setSaveBoidCenters(false);
    flock.log_prefix = "    ";
    flock.setSaveBoidCenters(write_flock_data_file);
    
    flock.fp().max_force = max_force;
    flock.fp().max_speed = std::max(min_speed, max_speed);
    flock.fp().min_speed = std::min(min_speed, max_speed);
    flock.fp().speed = util::clip(speed, min_speed, max_speed);
    flock.fp().weight_forward = weight_forward;
    flock.fp().weight_separate = weight_separate;
    flock.fp().weight_align = weight_align;
    flock.fp().weight_cohere = weight_cohere;
    flock.fp().weight_avoid = weight_avoid;
    flock.fp().max_dist_separate = (max_dist_separate_in_body_radii *
                                    flock.fp().body_radius);
    flock.fp().exponent_separate = exponent_separate;
    flock.fp().exponent_align = exponent_align;
    flock.fp().exponent_cohere = exponent_cohere;
    flock.fp().angle_separate = angle_separate;
    flock.fp().angle_align = angle_align;
    flock.fp().angle_cohere = angle_cohere;
    
    flock.run();

    // Compute fitness and return it as value of GpTree.
    double tiny = 0.1;
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240309 reconsider minsep
//    double minsep = flock.min_sep_dist_whole_sim;
    double minsep = flock.count_minsep_violations_whole_sim;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    double maxsep = flock.max_nn_dist_whole_sim;
    double collide = flock.total_avoid_fail_whole_sim;
    
    double minsep_limit = flock.fp().body_radius * 6; // TODO ad hoc
    double maxsep_limit = flock.fp().body_radius * 30; // TODO ad hoc

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240309 reconsider minsep

//    double minsep_fitness = util::remap_interval_clip(minsep,
//                                                      0, minsep_limit,
//                                                      tiny, 1);

    auto inv_count_01 = [](int count){ return  1.0 / (count + 1); };

    double minsep_fitness = util::remap_interval_clip(inv_count_01(minsep),
                                                      0, 1,
                                                      tiny, 1);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    
    
    
    double maxsep_fitness = util::remap_interval_clip(maxsep,
                                                      minsep_limit, maxsep_limit,
                                                      1, tiny);
    
    
    // For plot of this see: https://bitly.ws/3fqEu
//    double collide_invert = 1.0 / (collide + 1);
//    double collide_fitness = util::remap_interval_clip(collide_invert,
//                                                       0, 1,
//                                                       tiny, 1);
//    double collide_invert = 1.0 / (collide + 1);
    double collide_fitness = util::remap_interval_clip(inv_count_01(collide),
                                                       0, 1,
                                                       tiny, 1);

    double ave_speed = (flock.sum_all_speed_whole_sim /
                        (flock.max_simulation_steps() * flock.boid_count()));
    // TODO 20 is roughly the target speed.
    double ave_speed_fitness = util::remap_interval_clip(ave_speed, 0, 20, tiny, 1);

    double fitness = (minsep_fitness *
                      maxsep_fitness *
                      collide_fitness *
                      ave_speed_fitness);
    fitness_logger(minsep_fitness, minsep, maxsep_fitness, maxsep,
                   collide_fitness, collide, ave_speed_fitness, ave_speed,
                   fitness);
    return fitness;
}


// Wrote this to run at the end of evolution on the top-10 fitness individuals
// of population in order to record flock data for playback
inline float rerun_flock_simulation(const LazyPredator::Individual* individual)
{
    LazyPredator::GpTree t = individual->tree(); // copy tree
    return run_flock_simulation(t.evalSubtree<double>(0),
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
                                true);  // write flock data file
}



// TODO 20240304 so for example, here is the first random tree from:
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


// This is a degenerate GP function set, for what is essentially a GA problem:
// selecting a set of real number parameters for a flock simulation, via an
// absolute and fixed fitness metric. There is only one function, all GpTrees
// are exactly one function deep, differing only in their parameter values.
LazyPredator::FunctionSet evoflock_gp_function_set =
{
    {
        { "Fitness_0_1", 0.0,   1.0 },
        { "Real_0_1",    0.0,   1.0 },
        { "Real_0_10",   0.0,  10.0 },
        { "Real_0_100",  0.0, 100.0 },
        { "Real_0_200",  0.0, 200.0 },  // TODO keep?
        { "Real_m1_p1", -1.0,  +1.0 },
    },
    {
        {
            // GP function name:
            "Run_Flock",
            
            // Return type (in this case it returns a fitness on [0,1]):
            "Fitness_0_1",
            
            // Function parameter type list:
            //     TODO cf "FlockParameters" for details
            //     TODO note that I slightly reordering / reparameterizing
            //     TODO should body_radius be held constant at 0.5?
            {
                "Real_0_100",  // max_force
                "Real_0_100",  // max_speed
                "Real_0_100",  // min_speed
                "Real_0_100",  // speed
                
                "Real_0_100",  // weight_forward
                "Real_0_100",  // weight_separate
                "Real_0_100",  // weight_align
                "Real_0_100",  // weight_cohere
                "Real_0_100",  // weight_avoid

                "Real_0_200",  // max_dist_separate_in_body_radii
                //"Real_0_200",  // max_dist_align_in_body_radii
                //"Real_0_200",  // max_dist_cohere_in_body_radii

                "Real_0_10",   // exponent_separate
                "Real_0_10",   // exponent_align
                "Real_0_10",   // exponent_cohere

                // Cosine of threshold angle (max angle from forward to be seen)
                "Real_m1_p1",  // angle_separate
                "Real_m1_p1",  // angle_align
                "Real_m1_p1",  // angle_cohere
            },
            
            // Evaluation function, which runs a flock simulation with the given
            // parameters and returns the fitness.
            [](LazyPredator::GpTree& t)
            {
                auto fitness = run_flock_simulation(t.evalSubtree<double>(0),
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
                                                    t.evalSubtree<double>(15));
                return std::any(fitness);
            }
        }
    }
};
