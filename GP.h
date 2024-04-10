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
// TODO 20240330 WIP for multi-objective fitness
#ifdef MULTI_OBJECTIVE_FITNESS
#else
#endif
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240330 WIP for multi-objective fitness
#define MULTI_OBJECTIVE_FITNESS


typedef std::vector<double> MultiObjectiveFitness;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//    // TODO 20240327 WIP for multi-objective fitness
//
//    // Fitness function, simply returns Individual's tree's value (computing it and
//    // caching it on first call).
//    inline float evoflock_fitness_function(LazyPredator::Individual* individual)
//    {
//        return std::any_cast<double>(individual->tree().getRootValue());
//    }
//
//    //    // Fitness function, simply returns Individual's tree's value (computing it and
//    //    // caching it on first call).
//    //    inline std::vector<double> evoflock_fitness_function(LazyPredator::Individual* individual)
//    //    {
//    //        // TODO 20240327 MOCK!!
//    //        std::vector<double> multi_objective_fitness = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0};
//    //        return multi_objective_fitness;
//    //    }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240330 WIP for multi-objective fitness
#ifdef MULTI_OBJECTIVE_FITNESS
// Fitness function, simply returns Individual's tree's value (computing it and
// caching it on first call).
inline MultiObjectiveFitness evoflock_fitness_function(LazyPredator::Individual* individual)
{
    return std::any_cast<MultiObjectiveFitness>(individual->tree().getRootValue());
}
#else
// Fitness function, simply returns Individual's tree's value (computing it and
// caching it on first call).
inline float evoflock_fitness_function(LazyPredator::Individual* individual)
{
    return std::any_cast<double>(individual->tree().getRootValue());
}
#endif
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//void fitness_logger(double nn_sep_fitness, double nn_sep_err,
//                    double collide_fitness, double collide,
//                    double speed_err_fitness, double speed_err,
//                    double fitness)
void fitness_logger(double separation_fitness, double good_separation_fraction,
                    double avoid_fitness,      double good_avoid_fraction,
                    double speed_fitness,      double good_speed_fraction,
                    double occupied_fitness,   double occupied_fraction,
                    double fitness)

{
    // Save current settings for formatting float/double values
    std::ios::fmtflags old_settings = std::cout.flags();
    size_t old_precision = std::cout.precision();
    
    auto print = [](std::string label, double fitness, double raw = -1)
    {
        std::cout << "    " << label;
        std::cout << std::setprecision(6) << std::setw(10) << std::fixed;
        std::cout << fitness;
        if (raw > -1) { std::cout << " (" << raw <<  ")"; }
        std::cout << std::endl;
    };
    print("separation_fitness", separation_fitness, good_separation_fraction);
    print("avoid_fitness     ", avoid_fitness,      good_avoid_fraction);
    print("speed_fitness     ", speed_fitness,      good_speed_fraction);
    print("occupied_fitness  ", occupied_fitness,   occupied_fraction);
    print("fitness           ", fitness);
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


// Remap so [0,1] goes to [1,tiny]
inline double keep_above_zero(double fitness_01, double tiny = 0.1)
{
    return util::remap_interval_clip(fitness_01, 0, 1, tiny, 1);
}


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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240330 WIP for multi-objective fitness
// split multiObjectiveFitnessOfFloc from measure_fitness_after_flock_simulation

//inline std::vector<double> multiObjectiveFitnessOfFlock(const Flock& flock)
inline MultiObjectiveFitness multiObjectiveFitnessOfFlock(const Flock& flock)
{
    double steps = flock.max_simulation_steps();
    double good_separation_fraction = flock.count_steps_good_separation / steps;
    double good_avoid_fraction    = flock.count_steps_avoid_obstacle / steps;
    double good_speed_fraction  = flock.count_steps_good_speed  / steps;
    auto ignore_function = [](Vec3 p) { return p.length() > 50;};
    double occupied_fraction = flock.occupancy_map.fractionOccupied(ignore_function);
    return {good_separation_fraction,
            good_avoid_fraction,
            good_speed_fraction,
            occupied_fraction};
}

// Compute fitness from metrics maintained by flock during simulation.
inline double measure_fitness_after_flock_simulation(const Flock& flock)
{
//    double steps = flock.max_simulation_steps();
//    double good_separation_fraction = flock.count_steps_good_separation / steps;
//    double good_avoid_fraction    = flock.count_steps_avoid_obstacle / steps;
//    double good_speed_fraction  = flock.count_steps_good_speed  / steps;
//    auto ignore_function = [](Vec3 p) { return p.length() > 50;};
//    double occupied_fraction = flock.occupancy_map.fractionOccupied(ignore_function);

    MultiObjectiveFitness mo_fitness = multiObjectiveFitnessOfFlock(flock);
    double good_separation_fraction = mo_fitness.at(0);
    double good_avoid_fraction      = mo_fitness.at(1);
    double good_speed_fraction      = mo_fitness.at(2);
    double occupied_fraction        = mo_fitness.at(3);

    double separation_fitness = keep_above_zero(good_separation_fraction);
    double avoid_fitness      = keep_above_zero(good_avoid_fraction);
    double speed_fitness      = keep_above_zero(good_speed_fraction);
    double occupied_fitness   = keep_above_zero(occupied_fraction);

    separation_fitness = fitness_product_weight_01(separation_fitness, 0.6);
    avoid_fitness      = fitness_product_weight_01(avoid_fitness,      1.0);
    speed_fitness      = fitness_product_weight_01(speed_fitness,      0.2);
    occupied_fitness   = fitness_product_weight_01(occupied_fitness,   0.6);

    double fitness = (separation_fitness * avoid_fitness *
                      speed_fitness * occupied_fitness);
    fitness_logger(separation_fitness, good_separation_fraction,
                   avoid_fitness,      good_avoid_fraction,
                   speed_fitness,      good_speed_fraction,
                   occupied_fitness,   occupied_fraction,
                   fitness);
    // Just for testing
    if (print_occupancy_map) { flock.occupancy_map.print(true); }
    return fitness;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// Map a MultiObjectiveFitness to a scalar, here a normalized 4d magnitude.
// Used as the FitnessScalarizeFunction for Population::evolutionStep().
double scalarize_fitness(MultiObjectiveFitness mof)
{
    double sum_of_objectives_squared = 0;
    for (auto objective : mof) { sum_of_objectives_squared += sq(objective); }
    return std::sqrt(sum_of_objectives_squared) / std::sqrt(double(mof.size()));
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240330 WIP for multi-objective fitness
#ifdef MULTI_OBJECTIVE_FITNESS
// Run flock simulation with given parameters, return a MultiObjectiveFitness.
inline MultiObjectiveFitness run_flock_simulation(const FlockParameters& fp,
                                                  bool write_flock_data_file = false)
#else
// Run flock simulation with given parameters, return a scalar fitness on [0,1].
inline double run_flock_simulation(const FlockParameters& fp,
                                   bool write_flock_data_file = false)
#endif
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
{
    Flock flock;
    init_flock(flock);
    flock.setSaveBoidCenters(write_flock_data_file);
    flock.fp() = fp;
    flock.run();
//    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//    // TODO 20240330 WIP for multi-objective fitness
//#ifdef MULTI_OBJECTIVE_FITNESS
//    return multiObjectiveFitnessOfFlock(flock);
//#else
//    return measure_fitness_after_flock_simulation(flock);
//#endif
//    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240330 WIP for multi-objective fitness
#ifdef MULTI_OBJECTIVE_FITNESS
    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    // TODO 20240402 add per-objective logging for MultiObjectiveFitness case.
    // TODO          really ugly, needs polish.
    auto mofof = multiObjectiveFitnessOfFlock(flock);
//        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
//        // TODO 20240405 try sum to scalarize fitness
//        fitness_logger(mofof[0], -1, mofof[1], -1, mofof[2], -1, mofof[3], -1,
//    //                   (mofof[0] * mofof[1] * mofof[2] * mofof[3]));
//                       (mofof[0] + mofof[1] + mofof[2] + mofof[3]) / 4);
//        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20240405 try sum to scalarize fitness
    fitness_logger(mofof[0], -1, mofof[1], -1,
                   mofof[2], -1, mofof[3], -1,
                   scalarize_fitness(mofof));
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    return mofof;
    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
#else
    return measure_fitness_after_flock_simulation(flock);
#endif
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240330 WIP for multi-objective fitness
#ifdef MULTI_OBJECTIVE_FITNESS
// Run flock simulation with given parameters, return a MultiObjectiveFitness.
inline MultiObjectiveFitness run_flock_simulation(double max_force,
#else
// Run flock simulation with given parameters, return a scalar fitness on [0,1].
inline double run_flock_simulation(double max_force,
#endif
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240330 WIP for multi-objective fitness
#ifdef MULTI_OBJECTIVE_FITNESS
// Run flock simulation with given parameters, return a MultiObjectiveFitness.
inline MultiObjectiveFitness run_hand_tuned_flock_simulation(bool write_flock_data_file = false)
{
    return run_flock_simulation(FlockParameters(), write_flock_data_file);
}
#else
// Run flock simulation with given parameters, return a scalar fitness on [0,1].
inline double run_hand_tuned_flock_simulation(bool write_flock_data_file = false)
{
    return run_flock_simulation(FlockParameters(), write_flock_data_file);
}
#endif
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240330 WIP for multi-objective fitness
#ifdef MULTI_OBJECTIVE_FITNESS
// Wrote this to run at the end of evolution on the top-10 fitness individuals
// of population in order to record flock data for playback
inline MultiObjectiveFitness rerun_flock_simulation(const LazyPredator::Individual* individual)
#else
// Wrote this to run at the end of evolution on the top-10 fitness individuals
// of population in order to record flock data for playback
inline float rerun_flock_simulation(const LazyPredator::Individual* individual)
#endif
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
{
    // Is this tree copy needed to avoid using the previous cached tree root?
    LazyPredator::GpTree t = individual->tree();
    
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

// The default (in GpType::defaultJiggleScale()) is 0.05
double jiggle_scale = 0.1;

// This is a degenerate GP function set, for what is essentially a GA problem:
// selecting a set of real number parameters for a flock simulation, via an
// absolute and fixed fitness metric. There is only one function, all GpTrees
// are exactly one function deep, differing only in their parameter values.
LazyPredator::FunctionSet evoflock_gp_function_set =
{
    {
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240330 WIP for multi-objective fitness
#ifdef MULTI_OBJECTIVE_FITNESS
        { "Multi_Objective_Fitness" },
        
        // TODO needs work, like Texture in TexSyn (see constructors for GpType):
//        {
//            "Texture",
//            [](std::any a)
//            {
//                if (a.has_value())
//                {
//                    Texture* t = std::any_cast<Texture*>(a);
//                    if (t && t->valid()) delete t;
//                }
//            }
//        },
#else
        { "Fitness_0_1", 0.0,   1.0 },
#endif
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
        // TODO 20240409 trying increasing the jiggle scale (default is 0.05)

//        { "Real_0_1",    0.0,   1.0 },
//        { "Real_0_10",   0.0,  10.0 },
//        { "Real_0_100",  0.0, 100.0 },
//        { "Real_0_10_bigger_jiggle",   0.0,  10.0, 0.1 },
//        { "Real_0_100_bigger_jiggle",  0.0, 100.0, 0.1 },
//        { "Real_0_200",  0.0, 200.0 },  // TODO keep?
//        { "Real_m1_p1", -1.0,  +1.0 },
//        // TODO 20240321 pre-ranging for speed values (is this "cheating"?)
//        { "Real_15_30",  15.0,  30.0 },  // for boid speed values
        
        { "Real_0_1",    0.0,   1.0, jiggle_scale },
        { "Real_0_10",   0.0,  10.0, jiggle_scale },
        { "Real_0_100",  0.0, 100.0, jiggle_scale },
//        { "Real_0_10_bigger_jiggle",   0.0,  10.0, 0.1 },
//        { "Real_0_100_bigger_jiggle",  0.0, 100.0, 0.1 },
        { "Real_0_200",  0.0, 200.0, jiggle_scale },  // TODO keep?
        { "Real_m1_p1", -1.0,  +1.0, jiggle_scale },
        // TODO 20240321 pre-ranging for speed values (is this "cheating"?)
        { "Real_15_30",  15.0,  30.0, jiggle_scale },  // for boid speed values

        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    },
    {
        {
            // GP function name:
            "Run_Flock",
            
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20240330 WIP for multi-objective fitness
#ifdef MULTI_OBJECTIVE_FITNESS
            // Return type (in this case it returns a MultiObjectiveFitness):
            "Multi_Objective_Fitness",
#else
            // Return type (in this case it returns a fitness on [0,1]):
            "Fitness_0_1",
#endif
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            
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
                //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
                // TODO 20240409 trying increasing the jiggle scale (default is 0.05)
                "Real_0_100",  // weight_avoid
//                "Real_0_100_bigger_jiggle",  // weight_avoid
                //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

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
                //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
                // TODO 20240409 trying increasing the jiggle scale (default is 0.05)
                "Real_0_10",  // min_time_to_collide
//                "Real_0_10_bigger_jiggle",  // min_time_to_collide
                //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
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
