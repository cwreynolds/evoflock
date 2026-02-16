//------------------------------------------------------------------------------
//
//  FlockParameters.h -- new flock experiments
//
//  A container for all boid flocking parameters relevant to optimization.
//
//  A Flock and its Boids each point to a shared instance of FlockParameters.
//  Evolution adjusts this "genome" of parameters trying to find better fitness.
//
//  Distances and body_diameter are meant to be in meters (Although a spherical
//  boid 1 meter in diameter is quite large: in the eagle/vulture/turkey range).
//  Because all these parameters interact, it will probably not be meaningful to
//  share or compare parameters between flocks. Boids that are bigger/smaller,
//  faster/slower, or whatever, will need to be optimized separately for each
//  set of constraints.
//
//  Created by Craig Reynolds on 3/23/25.
//  MIT License -- Copyright © 2025 Craig Reynolds
//
//------------------------------------------------------------------------------

#pragma once
#include "Vec3.h"

//  A container for all boid flocking parameters relevant to optimization.
class FlockParameters
{
public:
    // Shared const/input parameters. These are not subject to optimization and
    // remain fixed during evolution run. MUST update constParameterCount() when
    // const parameters are added or removed.
    static int constParameterCount() { return 7; }
    // The count(/size) of tunable parameters in this class. For consistency
    // check, especially when changing the number of parameters.
    static int tunableParameterCount() { return 15; }
    // "assume a spherical boid" unit diameter
    double bodyDiameter() const { return body_diameter_; }
    // Should this be called "world radius"?
    const double& sphereRadius() const { return sphere_radius_; }
    // Should this be called "world center"?
    Vec3 sphereCenter() const { return sphere_center_; }
    // A flock simulation will run for this many steps (if no exception occurs).
    int maxSimulationSteps() const { return max_simulation_steps_; }
    // Number of Boids in a flock.
    int boidsPerFlock() const { return boids_per_flock_; }
    // Name of Obstacle set to use.
    std::string useObstacleSet() const { return use_obstacle_set; }
    // Simulation step frequency -- frames per second.
    int getFPS() const { return fps_; }
    
    // Hand-tuned parameters used as default.
    const static inline std::vector<double> hand_tuned_parameters =
    {
        100,  // max_force = ;          // Max acceleration (m/s²)

        // Weights for component steering behaviors.
        4,    // weight_forward
        40,   // weight_separate
        45,   // weight_align
        35,   // weight_cohere
        85,   // weightAvoidPredict
        55,   // weightAvoidStatic

        // Max distance at which a neighbor Boid influences a steering behavior.
        // (TODO would prefer parameterization with infinite support (1/d^n ?))
        15,   // max_dist_separate
        40,   // max_dist_align
        40,   // max_dist_cohere

        // Cosine of threshold angle (max angle from forward to be seen)
        -1,    // angle_separate = ~°
        +0.6,  // angle_align    = ~°
        -0.4,  // angle_cohere   = ~°

        // Max distance at which a boid flies away from an obstacle's surface.
        10,    // fly_away_max_dist = ;
                
        // Ignore obstacle until predicted impact is less than this many seconds.
        // Predict obstacle collisions only...
        // ...should this be renamed?
        1.4,   // min_time_to_collide
    };
    
    // Accessors for tuning parameters:
    const double& maxForce()        const { return tuning_parameters.at(0); }
    
    const double& weightForward()   const { return tuning_parameters.at(1); }
    const double& weightSeparate()  const { return tuning_parameters.at(2); }
    const double& weightAlign()     const { return tuning_parameters.at(3); }
    const double& weightCohere()    const { return tuning_parameters.at(4); }
    const double& weightAvoidPredict()const { return tuning_parameters.at(5); }
    const double& weightAvoidStatic() const { return tuning_parameters.at(6); }
    
    const double& maxDistSeparate() const { return tuning_parameters.at(7); }
    const double& maxDistAlign()    const { return tuning_parameters.at(8); }
    const double& maxDistCohere()   const { return tuning_parameters.at(9); }
    
    // Cosine of threshold angle (max angle from forward to be seen)
    const double& angleSeparate()   const { return tuning_parameters.at(10); }
    const double& angleAlign()      const { return tuning_parameters.at(11); }
    const double& angleCohere()     const { return tuning_parameters.at(12); }
    
    const double& flyAwayMaxDist()  const { return tuning_parameters.at(13); }
    
    // ignore obstacle until predicted impact is in less than this many seconds.
    const double& minTimeToCollide() const { return tuning_parameters.at(14); }

    // Default constructor
    FlockParameters() : FlockParameters(hand_tuned_parameters) {}
    
    // Constructor based on a vector of numeric parameters.
    FlockParameters(const std::vector<double>& vector_of_parameters_)
    {
        assert(vector_of_parameters_.size() == tunableParameterCount());
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20250908 assert FlockParameters only used in GA mode.
//        assert(EF::usingGA());
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        tuning_parameters = vector_of_parameters_;
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20250908 assert FlockParameters only used in GA mode.

//    // Constructor for individual tunable parameters.
//    FlockParameters(double max_force,
//                    double weight_forward,
//                    double weight_separate,
//                    double weight_align,
//                    double weight_cohere,
//                    double weight_avoid_predict,
//                    double weight_avoid_static,
//                    double max_dist_separate,
//                    double max_dist_align,
//                    double max_dist_cohere,
//                    double angle_separate,
//                    double angle_align,
//                    double angle_cohere,
//                    double fly_away_max_dist,
//                    double min_time_to_collide)
//    {
//
//        // Set tuning parameter vector to given values.
//        tuning_parameters =
//        {
//            max_force,
//            weight_forward,
//            weight_separate,
//            weight_align,
//            weight_cohere,
//            weight_avoid_predict,
//            weight_avoid_static,
//            max_dist_separate,
//            max_dist_align,
//            max_dist_cohere,
//            angle_separate,
//            angle_align,
//            angle_cohere,
//            fly_away_max_dist,
//            min_time_to_collide
//        };
//    }

    // Constructor for individual tunable parameters.
    FlockParameters(double max_force,
                    double weight_forward,
                    double weight_separate,
                    double weight_align,
                    double weight_cohere,
                    double weight_avoid_predict,
                    double weight_avoid_static,
                    double max_dist_separate,
                    double max_dist_align,
                    double max_dist_cohere,
                    double angle_separate,
                    double angle_align,
                    double angle_cohere,
                    double fly_away_max_dist,
                    double min_time_to_collide)
      : FlockParameters
    (std::vector<double>({
        max_force,
        weight_forward,
        weight_separate,
        weight_align,
        weight_cohere,
        weight_avoid_predict,
        weight_avoid_static,
        max_dist_separate,
        max_dist_align,
        max_dist_cohere,
        angle_separate,
        angle_align,
        angle_cohere,
        fly_away_max_dist,
        min_time_to_collide
    })) {}

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // The count(/size) of ALL parameters in this class.
    static int parameterCount()
    {
        return constParameterCount() + tunableParameterCount();
    }

    void print() const
    {
        assert(tuning_parameters.size() == tunableParameterCount());
        auto indent = [](){ std::cout << "    "; };
        std::cout << "FlockParameters object containing these values:";
        std::cout << std::endl << "  " << constParameterCount();
        std::cout << " constant parameters:" << std::endl;
        indent(); debugPrint(bodyDiameter());
        indent(); debugPrint(sphereRadius());
        indent(); debugPrint(sphereCenter());
        indent(); debugPrint(maxSimulationSteps());
        indent(); debugPrint(boidsPerFlock());
        indent(); debugPrint(useObstacleSet());
        indent(); debugPrint(getFPS());
        std::cout << "  " << tunableParameterCount();
        std::cout << " parameters to be optimized:" << std::endl;
        indent(); debugPrint(maxForce());
        indent(); debugPrint(weightForward());
        indent(); debugPrint(weightSeparate());
        indent(); debugPrint(weightAlign());
        indent(); debugPrint(weightCohere());
        indent(); debugPrint(weightAvoidPredict());
        indent(); debugPrint(weightAvoidStatic());
        indent(); debugPrint(maxDistSeparate());
        indent(); debugPrint(maxDistAlign());
        indent(); debugPrint(maxDistCohere());
        indent(); debugPrint(angleSeparate());
        indent(); debugPrint(angleAlign());
        indent(); debugPrint(angleCohere());
        indent(); debugPrint(flyAwayMaxDist());
        indent(); debugPrint(minTimeToCollide());
    }
    
    // Get i-th tuning parameter
    // TODO experimental reconsider design.
    double getTuningParameter(int i) const { return tuning_parameters.at(i); }

private:
    // Parameters for tuning:
    std::vector<double> tuning_parameters = hand_tuned_parameters;

    // const/input parameters:
    double body_diameter_ = 1;
    double sphere_radius_ = 50;
    Vec3 sphere_center_;
    
    int max_simulation_steps_ = 500;    // ~17 seconds: for evolution run
    //int max_simulation_steps_ = 2000;   // ~66 seconds: for obs collision test
    //int max_simulation_steps_ = 9000;  // 5 minutes for demo mode
    
    int boids_per_flock_ = 200;  // Normal, for evolution runs.
    //int boids_per_flock_ = 400;  // More.
    //int boids_per_flock_ = 800;  // Even more.
    //int boids_per_flock_ = 1000;  // Kiloboid.
    //int boids_per_flock_ = 2500;  // for NoObstacles

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20260213 try flocking trained on open space, no obstacles.
//    std::string use_obstacle_set = "SmallSpheresInBigSphere";
    std::string use_obstacle_set = "NoObstacles";
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    int fps_ = 30;
};
