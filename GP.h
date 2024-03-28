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


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240327 WIP for multi-objective fitness

// Fitness function, simply returns Individual's tree's value (computing it and
// caching it on first call).
inline float evoflock_fitness_function(LazyPredator::Individual* individual)
{
    return std::any_cast<double>(individual->tree().getRootValue());
}

//    // Fitness function, simply returns Individual's tree's value (computing it and
//    // caching it on first call).
//    inline std::vector<double> evoflock_fitness_function(LazyPredator::Individual* individual)
//    {
//        // TODO 20240327 MOCK!!
//        std::vector<double> multi_objective_fitness = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0};
//        return multi_objective_fitness;
//    }

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


void fitness_logger(double nn_sep_fitness, double nn_sep_err,
                    double collide_fitness, double collide,
                    double speed_err_fitness, double speed_err,
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
    print("    nn_sep_err", nn_sep_fitness,    nn_sep_err);
    print("    collide   ", collide_fitness,   collide);
    print("    speed_err ", speed_err_fitness, speed_err);
    print("    fitness   ", fitness);
    std::cout << std::endl;
    
    // restore output format flags and precision
    std::cout.flags(old_settings);
    std::cout.precision(old_precision);
}


// Return a FlockParameters object with all given parameter values
inline FlockParameters init_flock_parameters(double max_force,
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
    flock.set_max_simulation_steps(1000);
    flock.setLogStatInterval(flock.max_simulation_steps());
    flock.setSaveBoidCenters(false);
    flock.log_prefix = "    ";
}


//    // Move this to util, add unit tests
//    // For plot of this see: https://bitly.ws/3fqEu
//    //    auto inv_count_01 = [](int count){ return  1.0 / (count + 1); };


//    // Experimental non-linearity
//    // (Wolfram Alpha plot: https://tinyurl.com/bdew7a92)
//    auto nonlin = [](double x) { return (x + pow(x, 10)) / 2; };


// Flip the 0-to-1 range, but remap so [0,1] goes to [1,tiny]
inline double flip_and_keep_above_zero(double fitness_01, double tiny = 0.1)
{
    return util::remap_interval_clip(fitness_01, 0, 1, 1, tiny);
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


// Compute fitness and return it as value of GpTree.
inline double measure_fitness_after_flock_simulation(const Flock& flock)
{
    double steps = flock.max_simulation_steps();
    double nn_sep_err = flock.any_seperation_violation_per_step / steps;
    double collide    = flock.any_obstacle_violation_per_step / steps;
    double speed_err  = flock.any_speed_violation_per_step  / steps;
    auto ignore_function = [](Vec3 p) { return p.length() > 50;};
    double fraction_occupied = flock.occupancy_map.fractionOccupied(ignore_function);

    debugPrint(flock.any_seperation_violation_per_step)
    debugPrint(flock.any_obstacle_violation_per_step)
    debugPrint(flock.any_speed_violation_per_step)
    debugPrint(flock.any_seperation_violation_per_step / steps)
    debugPrint(flock.any_obstacle_violation_per_step   / steps)
    debugPrint(flock.any_speed_violation_per_step      / steps)

    double nn_sep_fitness    = flip_and_keep_above_zero(nn_sep_err);
    double collide_fitness   = flip_and_keep_above_zero(collide);
    double speed_err_fitness = flip_and_keep_above_zero(speed_err);
    double occupied_fitness = util::remap_interval_clip(fraction_occupied, 0, 1, 0.1, 1);

    nn_sep_fitness =    fitness_product_weight_01(nn_sep_fitness,    0.6);
    collide_fitness =   fitness_product_weight_01(collide_fitness,   1.0);
    speed_err_fitness = fitness_product_weight_01(speed_err_fitness, 0.2);
    occupied_fitness  = fitness_product_weight_01(fraction_occupied, 0.6);

    debugPrint(fraction_occupied)
    debugPrint(occupied_fitness)

    double fitness = (nn_sep_fitness * collide_fitness *
                      speed_err_fitness * occupied_fitness);

    fitness_logger(nn_sep_fitness, nn_sep_err,
                   collide_fitness, collide,
                   speed_err_fitness, speed_err,
                   fitness);

    // Just for testing
    if (print_occupancy_map) { flock.occupancy_map.print(true); }
    return fitness;
}


// Run flock simulation with given parameters, return a scalar fitness on [0,1].
inline double run_flock_simulation(const FlockParameters& fp,
                                   bool write_flock_data_file = false)
{
    Flock flock;
    init_flock(flock);
    flock.setSaveBoidCenters(write_flock_data_file);
    flock.fp() = fp;
    flock.run();
    return measure_fitness_after_flock_simulation(flock);
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

                                   double fly_away_max_dist_in_br,
                                   double min_time_to_collide,
                                   bool write_flock_data_file = false)
{
    FlockParameters fp = init_flock_parameters(max_force,
                                               max_speed,
                                               min_speed,
                                               speed,
                                               
                                               weight_forward,
                                               weight_separate,
                                               weight_align,
                                               weight_cohere,
                                               weight_avoid,

                                               max_dist_separate_in_body_radii,
                                               
                                               exponent_separate,
                                               exponent_align,
                                               exponent_cohere,
                                               
                                               angle_separate,
                                               angle_align,
                                               angle_cohere,

                                               fly_away_max_dist_in_br,
                                               min_time_to_collide);
    return run_flock_simulation(fp, write_flock_data_file);
}


// Run flock simulation with given parameters, return a scalar fitness on [0,1].
inline double run_hand_tuned_flock_simulation(bool write_flock_data_file = false)
{
    return run_flock_simulation(FlockParameters(), write_flock_data_file);
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
                                t.evalSubtree<double>(16),
                                t.evalSubtree<double>(17),
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
        // TODO 20240321 pre-ranging for speed values (is this "cheating"?)
        { "Real_15_30",  15.0,  30.0 },  // for boid speed values
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
                "Real_15_30",  // max_speed
                "Real_15_30",  // min_speed
                "Real_15_30",  // speed

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
                                                    t.evalSubtree<double>(15),
                                                    t.evalSubtree<double>(16),
                                                    t.evalSubtree<double>(17));
                return std::any(fitness);
            }
        }
    }
};
