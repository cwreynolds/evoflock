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



//    fitness_logger(nn_sep_fitness, nn_sep_err,
//                   collide_fitness, collide,
//                   ave_speed_fitness, ave_speed,
//                   fitness);


// TODO 20240309 temp?
//void fitness_logger(double nn_sep_fitness, double nn_sep_err,
//                    double collide_fitness, double collide,
//                    double ave_speed_fitness, double ave_speed,
//                    double fitness)
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
//    print("    minsep    ", minsep_fitness,    minsep);
//    print("    maxsep    ", maxsep_fitness,    maxsep);
    print("    nn_sep_err", nn_sep_fitness,    nn_sep_err);
    print("    collide   ", collide_fitness,   collide);
//    print("    ave_speed ", ave_speed_fitness, ave_speed);
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
                                             double angle_cohere)
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
    fp.max_dist_separate = (max_dist_separate_in_body_radii *
                                    fp.body_radius);
    fp.exponent_separate = exponent_separate;
    fp.exponent_align = exponent_align;
    fp.exponent_cohere = exponent_cohere;
    fp.angle_separate = angle_separate;
    fp.angle_align = angle_align;
    fp.angle_cohere = angle_cohere;
    return fp;
}


// Initialize basic run parameters of Flock object
inline void init_flock(Flock& flock)
{
    flock.set_boid_count(200);
    flock.set_fixed_fps(30);
    flock.set_fixed_time_step(true);
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240316 why avoiding sphere but collide with cylinder?
//    flock.set_max_simulation_steps(1000);
//    flock.setLogStatInterval(1000);
//    flock.set_max_simulation_steps(10);
//    flock.set_max_simulation_steps(200);
    flock.set_max_simulation_steps(1000);
    flock.setLogStatInterval(flock.max_simulation_steps());
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    flock.setSaveBoidCenters(false);
    flock.log_prefix = "    ";
}


//    inline double measure_fitness_after_flock_simulation(const Flock& flock)
//    {
//        // Compute fitness and return it as value of GpTree.
//        double tiny = 0.1;
//
//        double boid_count = flock.boid_count();
//
//    //    double minsep = (flock.count_minsep_violations_whole_sim / boid_count);
//    //    double maxsep = flock.max_nn_dist_whole_sim;
//
//        double nn_sep_err = (flock.count_nn_sep_violations_whole_sim / boid_count);
//
//        double collide = (flock.total_avoid_fail_whole_sim / boid_count);
//
//        double speed_err = (flock.count_speed_violations_whole_sim / boid_count);
//
//
//    //    double minsep_limit = flock.fp().body_radius * 6; // TODO ad hoc
//    //    double maxsep_limit = flock.fp().body_radius * 30; // TODO ad hoc
//
//        // For plot of this see: https://bitly.ws/3fqEu
//        auto inv_count_01 = [](int count){ return  1.0 / (count + 1); };
//
//    //    double minsep_fitness = util::remap_interval_clip(inv_count_01(minsep),
//    //                                                      0, 1,
//    //                                                      tiny, 1);
//    //    double maxsep_fitness = util::remap_interval_clip(maxsep,
//    //                                                      minsep_limit, maxsep_limit,
//    //                                                      1, tiny);
//    //    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//    //    // TODO 20240310 disable maxsep
//    //    maxsep_fitness = 1;
//    //    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//
//        double nn_sep_fitness = util::remap_interval_clip(inv_count_01(nn_sep_err),
//                                                          0, 1,
//                                                          tiny, 1);
//
//
//
//        double collide_fitness = util::remap_interval_clip(inv_count_01(collide),
//                                                           0, 1,
//                                                           tiny, 1);
//
//    //    double ave_speed = (flock.sum_all_speed_whole_sim /
//    //                        (flock.max_simulation_steps() * flock.boid_count()));
//
//
//
//
//        // TODO 20 is roughly the target speed.
//    //    double ave_speed_fitness = util::remap_interval_clip(ave_speed, 0, 20, tiny, 1);
//        double speed_err_fitness = util::remap_interval_clip(inv_count_01(speed_err),
//                                                             0, 1,
//                                                             tiny, 1);
//
//
//    //    double fitness = (minsep_fitness *
//    //                      maxsep_fitness *
//    //                      collide_fitness *
//    //                      ave_speed_fitness);
//    //    double fitness = (nn_sep_fitness *
//    //                      collide_fitness *
//    //                      ave_speed_fitness);
//        double fitness = (nn_sep_fitness *
//                          collide_fitness *
//                          speed_err_fitness);
//    //    fitness_logger(minsep_fitness, minsep,
//    //                   maxsep_fitness, maxsep,
//    //                   collide_fitness, collide,
//    //                   ave_speed_fitness, ave_speed,
//    //                   fitness);
//    //    fitness_logger(nn_sep_fitness, nn_sep_err,
//    //                   collide_fitness, collide,
//    //                   ave_speed_fitness, ave_speed,
//    //                   fitness);
//        fitness_logger(nn_sep_fitness, nn_sep_err,
//                       collide_fitness, collide,
//                       speed_err_fitness, speed_err,
//                       fitness);
//
//        return fitness;
//
//    }


//    inline double measure_fitness_after_flock_simulation(const Flock& flock)
//    {
//        // Compute fitness and return it as value of GpTree.
//        double tiny = 0.1;
//    //    double boid_count = flock.boid_count();
//
//        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//        // TODO 20240313 these values seem to be so large that the fitness values
//        // are all very close to 0. Trying to find some principled way to scale them
//        // down.
//        // Had been dividing by boid_count.
//        // What about 1 / (boid count * step_count)?
//
//        // Normalize down to "count per boid per step".
//        double norm = flock.boid_count() * flock.max_simulation_steps();
//
//    //    double nn_sep_err = (flock.count_nn_sep_violations_whole_sim / boid_count);
//    //    double collide = (flock.total_avoid_fail_whole_sim / boid_count);
//    //    double speed_err = (flock.count_speed_violations_whole_sim / boid_count);
//
//        double nn_sep_err = flock.count_nn_sep_violations_whole_sim / norm;
//        double collide    = flock.total_avoid_fail_whole_sim        / norm;
//        double speed_err  = flock.count_speed_violations_whole_sim  / norm;
//
//
//        // TODO 20240313 oh! but maybe the "hyperbolic" 1/count is not needed since
//        //               we now already scale it down to "bad behavior per boid per
//        //               step" which is already on [0, 1]
//        //
//        //               For a multi-agent simulation we can count the good (or bad)
//        //               outcomes each step for each agent. Then “normalize” that
//        //               count by dividing it by agent count times step count. This
//        //               is then a number on [0,1] averaging the good/bad events.
//        //
//        // Wait, I saw this go by (20240313):
//        //     speed_err   0.550000 (1.000000)
//        // Why isn't that 0.5?
//        // Oh, its from the "tiny" isn't it?
//        //
//        // Move this to util, add unit tests
//        // For plot of this see: https://bitly.ws/3fqEu
//    //    auto inv_count_01 = [](int count){ return  1.0 / (count + 1); };
//
//        auto inv_count_01 = [](double count)
//        {
//            assert (count >= 0);
//            return  1.0 / (count + 1);
//        };
//
//        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//        double nn_sep_fitness = util::remap_interval_clip(inv_count_01(nn_sep_err),
//                                                          0, 1,
//                                                          tiny, 1);
//        double collide_fitness = util::remap_interval_clip(inv_count_01(collide),
//                                                           0, 1,
//                                                           tiny, 1);
//        double speed_err_fitness = util::remap_interval_clip(inv_count_01(speed_err),
//                                                             0, 1,
//                                                             tiny, 1);
//        double fitness = (nn_sep_fitness *
//                          collide_fitness *
//                          speed_err_fitness);
//        fitness_logger(nn_sep_fitness, nn_sep_err,
//                       collide_fitness, collide,
//                       speed_err_fitness, speed_err,
//                       fitness);
//        return fitness;
//
//    }

inline double measure_fitness_after_flock_simulation(const Flock& flock)
{
    // Compute fitness and return it as value of GpTree.
    double tiny = 0.1;
    
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20240313 these values seem to be so large that the fitness values
    // are all very close to 0. Trying to find some principled way to scale them
    // down.
    // Had been dividing by boid_count.
    // What about 1 / (boid count * step_count)?
    
    // Normalize down to "count per boid per step".
    double norm = flock.boid_count() * flock.max_simulation_steps();
    
//    //    double nn_sep_err = (flock.count_nn_sep_violations_whole_sim / boid_count);
//    //    double collide = (flock.total_avoid_fail_whole_sim / boid_count);
//    //    double speed_err = (flock.count_speed_violations_whole_sim / boid_count);
    
    double nn_sep_err = flock.count_nn_sep_violations_whole_sim / norm;
    double collide    = flock.total_avoid_fail_whole_sim        / norm;
    double speed_err  = flock.count_speed_violations_whole_sim  / norm;
    
    
//    // TODO 20240313 oh! but maybe the "hyperbolic" 1/count is not needed since
//    //               we now already scale it down to "bad behavior per boid per
//    //               step" which is already on [0, 1]
//    //
//    //               For a multi-agent simulation we can count the good (or bad)
//    //               outcomes each step for each agent. Then “normalize” that
//    //               count by dividing it by agent count times step count. This
//    //               is then a number on [0,1] averaging the good/bad events.
//    //
//    // Wait, I saw this go by (20240313):
//    //     speed_err   0.550000 (1.000000)
//    // Why isn't that 0.5?
//    // Oh, its from the "tiny" isn't it?
//    //
//    // Move this to util, add unit tests
//    // For plot of this see: https://bitly.ws/3fqEu
//    //    auto inv_count_01 = [](int count){ return  1.0 / (count + 1); };
//    
//    auto inv_count_01 = [](double count)
//    {
//        assert (count >= 0);
//        return  1.0 / (count + 1);
//    };
    
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    
//    double nn_sep_fitness = util::remap_interval_clip(inv_count_01(nn_sep_err),
//                                                      0, 1,
//                                                      tiny, 1);
//    double collide_fitness = util::remap_interval_clip(inv_count_01(collide),
//                                                       0, 1,
//                                                       tiny, 1);
//    double speed_err_fitness = util::remap_interval_clip(inv_count_01(speed_err),
//                                                         0, 1,
//                                                         tiny, 1);

    double nn_sep_fitness = util::remap_interval_clip(nn_sep_err, 0, 1, 1, tiny);
    double collide_fitness = util::remap_interval_clip(collide, 0, 1, 1, tiny);
    double speed_err_fitness = util::remap_interval_clip(speed_err, 0, 1, 1, tiny);
    
    
    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    // TODO 20240314 try per-fitness-factor weightings
    
//    nn_sep_fitness =    util::remap_interval_clip(nn_sep_fitness,    0, 1, 0.5,  1);
//    collide_fitness =   util::remap_interval_clip(collide_fitness,   0, 1, 0,    1);
//    speed_err_fitness = util::remap_interval_clip(speed_err_fitness, 0, 1, 0.75, 1);
    
    // TODO 20240315 more extreme emphasis on obstacle avoidance

    nn_sep_fitness =    util::remap_interval_clip(nn_sep_fitness,    0, 1, 0.8, 1);
    collide_fitness =   util::remap_interval_clip(collide_fitness,   0, 1, 0,   1);
    speed_err_fitness = util::remap_interval_clip(speed_err_fitness, 0, 1, 0.9, 1);
    
    // TODO 20240315 experimental non-linearity
    // (Wolfram Alpha plot: https://tinyurl.com/bdew7a92)
    auto nonlin = [](double x) { return (x + pow(x, 10)) / 2; };

//    nn_sep_fitness =    nonlin(nn_sep_fitness);
//    collide_fitness =   nonlin(collide_fitness);
//    speed_err_fitness = nonlin(speed_err_fitness);
//
//    double fitness = (nn_sep_fitness * collide_fitness * speed_err_fitness);
    
    double fitness = nonlin(nn_sep_fitness * collide_fitness * speed_err_fitness);

    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    fitness_logger(nn_sep_fitness, nn_sep_err,
                   collide_fitness, collide,
                   speed_err_fitness, speed_err,
                   fitness);
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
                                   
                                   bool write_flock_data_file = false)
{
    FlockParameters fp = init_flock_parameters(max_force,
                                               //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                                               // TODO 20240316 why avoiding sphere
                                               // but collide with cylinder?
//                                               max_speed,
//                                               min_speed,
                                               20,
                                               20,
                                               //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                                               speed,
                                               
                                               weight_forward,
                                               weight_separate,
                                               weight_align,
                                               weight_cohere,
                                               //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                                               // TODO 20240316 why avoiding sphere
                                               // but collide with cylinder?
                                               weight_avoid,
//                                               100,  // QQQQQQQQQQQQQQQQQQQQQQQQ
                                               //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

                                               max_dist_separate_in_body_radii,
                                               
                                               exponent_separate,
                                               exponent_align,
                                               exponent_cohere,
                                               
                                               angle_separate,
                                               angle_align,
                                               angle_cohere);
    
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
