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

// TODO 20240226 For now, a modified copy of LazyPredator is in a subdirectory.
#include "LazyPredator/LazyPredator.h"

namespace LP = LazyPredator;


// TODO 20230303 mock
// Run flock simulation with given parameters, return a scalar fitness on [0,1].
inline
double run_flock_simulation (double p0,  double p1,  double p2,  double p3,
                             double p4,  double p5,  double p6,  double p7,
                             double p8,  double p9,  double p10, double p11,
                             double p12, double p13, double p14, double p15)
{
    return 0;
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
                "Real_0_100",  // max_speed (enforce min < max by sorting?)
                "Real_0_100",  // min_speed (enforce min < max by sorting?)
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
                "Real_m1_p1",  // angle_separate
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
