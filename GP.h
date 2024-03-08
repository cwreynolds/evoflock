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





//    //    Flock flock;
//    //    flock.set_fixed_time_step(true);
//    //    flock.set_fixed_fps(30);
//    //    flock.set_max_simulation_steps(1000);
//    //    flock.set_boid_count(200);
//    //
//    //    flock.run();
//
//
//    // TODO 20230303 mock
//    // Run flock simulation with given parameters, return a scalar fitness on [0,1].
//    inline
//    double run_flock_simulation (double p0,  double p1,  double p2,  double p3,
//                                 double p4,  double p5,  double p6,  double p7,
//                                 double p8,  double p9,  double p10, double p11,
//                                 double p12, double p13, double p14, double p15)
//    {
//        return 0;
//    }







//    {
//        "Real_0_100",
//        "Real_0_100",
//        "Real_0_100",
//        "Real_0_100",
//
//        "Real_0_100",
//        "Real_0_100",
//        "Real_0_100",
//        "Real_0_100",
//        "Real_0_100",
//
//        "Real_0_200",
//
//        "Real_0_10",
//        "Real_0_10",
//        "Real_0_10",
//
//        // Cosine of threshold angle (max angle from forward to be seen)
//        "Real_m1_p1",
//        "Real_m1_p1",
//        "Real_m1_p1",
//    }

//    // Run flock simulation with given parameters, return a scalar fitness on [0,1].
//    inline
//    double run_flock_simulation (double p0,  // max_force
//                                 double p1,  // max_speed (enforce min < max by sorting?)
//                                 double p2,  // min_speed (enforce min < max by sorting?)
//                                 double p3,  // speed
//
//                                 double p4,  // weight_forward
//                                 double p5,  // weight_separate
//                                 double p6,  // weight_align
//                                 double p7,  // weight_cohere
//                                 double p8,  // weight_avoid
//
//                                 double p9,  // max_dist_separate_in_body_radii
//
//                                 double p10,   // exponent_separate
//                                 double p11,   // exponent_align
//                                 double p12,   // exponent_cohere
//
//                                 double p13,  // angle_separate
//                                 double p14,  // angle_align
//                                 double p15)  // angle_cohere


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240307 working on fitness function

// TODO 20240307 OH! how do I get a pointer to the flock?

//Flock flock;
//
//flock
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240307 working on fitness function

// TODO 20240307 OH! how do I get a pointer to the flock?


// Make function with this type (seems like it ought to return a double?)
// typedef std::function<float(Individual*)> FitnessFunction;

//    inline float evoflock_fitness_function(LazyPredator::Individual* individual)
//    {
//        double fitness = 1;
//
//        // The least separation over all boids on all simulation steps.
//        //    double min_sep_dist_whole_sim = std::numeric_limits<double>::infinity();
//
//        // Largest separation over all boids on all simulation steps.
//        //    double max_nn_dist_whole_sim = 0;
//
//
//
//        debugPrint(individual->tree().to_string())
//        debugPrint(std::any_cast<double>(individual->tree().getRootValue()) )
//
//        return fitness;
//    }

inline float evoflock_fitness_function(LazyPredator::Individual* individual)
{
//    double fitness = 1;
    
    // The least separation over all boids on all simulation steps.
    //    double min_sep_dist_whole_sim = std::numeric_limits<double>::infinity();
    
    // Largest separation over all boids on all simulation steps.
    //    double max_nn_dist_whole_sim = 0;
    
    
    
//    debugPrint(individual->tree().to_string())
//    debugPrint(std::any_cast<double>(individual->tree().getRootValue()))
    
    return std::any_cast<double>(individual->tree().getRootValue());
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// Run flock simulation with given parameters, return a scalar fitness on [0,1].
inline double run_flock_simulation (double max_force,
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
//    std::cout << "Inside run_flock_simulation()" << std::endl;
//    
//    debugPrint(max_force)
//    debugPrint(max_speed)
//    debugPrint(min_speed)
//    debugPrint(speed)
//
//    debugPrint(weight_forward)
//    debugPrint(weight_separate)
//    debugPrint(weight_align)
//    debugPrint(weight_cohere)
//    debugPrint(weight_avoid)
//
//    debugPrint(max_dist_separate_in_body_radii)
//
//    debugPrint(exponent_separate)
//    debugPrint(exponent_align)
//    debugPrint(exponent_cohere)
//
//    debugPrint(angle_separate)
//    debugPrint(angle_align)
//    debugPrint(angle_cohere)


    

    Flock flock;
    flock.set_boid_count(200);
    flock.set_fixed_fps(30);
    flock.set_fixed_time_step(true);
    flock.set_max_simulation_steps(1000);
    flock.setLogStatInterval(1000);
    flock.setSaveBoidCenters(false);
    
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

    // The least separation over all boids on all simulation steps.
    //    double min_sep_dist_whole_sim = std::numeric_limits<double>::infinity();
    
    // Largest separation over all boids on all simulation steps.
    //    double max_nn_dist_whole_sim = 0;

    
//    // TODO total mock:
//    double fitness = (flock.min_sep_dist_whole_sim *
//                      flock.max_nn_dist_whole_sim *
//                      flock.total_avoid_fail_whole_sim);

    auto quadratic_01_clip = [](double x, double min, double max)
    {
        double f = util::remap_interval_clip(x, min, max, 0, 1);
        return f * f;
    };
    
    double minsep = flock.min_sep_dist_whole_sim;
    double maxsep = flock.max_nn_dist_whole_sim;
    double collide = flock.total_avoid_fail_whole_sim;
    
    double minsep_limit = flock.fp().body_radius * 6; // TODO ad hoc
    double maxsep_limit = flock.fp().body_radius * 30; // TODO ad hoc

    double minsep_fitness = 1 - quadratic_01_clip(minsep, minsep_limit, 0);
    double maxsep_fitness = 1 - (quadratic_01_clip(maxsep, 0, maxsep_limit) * 0.5);
    double collide_fitness = 1.0 / (collide + 1);
    
    
    double fitness = minsep_fitness * maxsep_fitness * collide_fitness;

    debugPrint(minsep_fitness)
    debugPrint(maxsep_fitness)
    debugPrint(collide_fitness)
    debugPrint(fitness)

    return fitness;
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
