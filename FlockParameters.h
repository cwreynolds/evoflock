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
    // Shared const/input parameters. Perhaps eventually include max sim steps,
    // obstacle set, etc.? Be sure to update constParameterCount() when const
    // parameters are added or removed.
    static size_t constParameterCount() { return 6; }
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

    
    // Hand-tuned parameters used as default.
    const static inline std::vector<double> hand_tuned_parameters =
    {
        100,  // max_force = ;          // Max acceleration (m/s²)
        20,   // min_speed = ;           // Speed lower limit (m/s)
        0,    //speed = 0;                // Initial speed (m/s)
        20,   // max_speed = ;           // Speed upper limit (m/s)
        
        4,    // weight_forward  = ;
        23,   // weight_separate = ;
        12,   // weight_align    = ;
        18,   // weight_cohere   = ;
        25,   //weight_avoid    = ;
        
        10,   // max_dist_separate = ;
        100,  // max_dist_align    = ;  // TODO 20231017 should this be ∞ or
        100,  // max_dist_cohere   = ;  //      should the behavior just ignore it?
        
        // Cosine of threshold angle (max angle from forward to be seen)
        0,    // angle_separate = 0;  // 90°
        0,    // angle_align    = 0;  // 90°
        0,    // angle_cohere   = 0;  // 90°
        
        10,   // fly_away_max_dist = ;   // max fly-away dist from obstacle surface
        
        // ignore obstacle until predicted impact is in less than this many seconds.
        0.8,  // min_time_to_collide = ;
    };
        
    // Acessors for tuning parameters:
    const double& maxForce()        const { return tuning_parameters.at(0); }
    const double& minSpeed()        const { return tuning_parameters.at(1); }
    const double& initSpeed()       const { return tuning_parameters.at(2); }
    const double& maxSpeed()        const { return tuning_parameters.at(3); }
    
    void setMinSpeed(double s)      { tuning_parameters.at(1) = s; }
    void setSpeed(double s)         { tuning_parameters.at(2) = s; }
    void setMaxSpeed(double s)      { tuning_parameters.at(3) = s; }
    
    const double& weightForward()   const { return tuning_parameters.at(4); }
    const double& weightSeparate()  const { return tuning_parameters.at(5); }
    const double& weightAlign()     const { return tuning_parameters.at(6); }
    const double& weightCohere()    const { return tuning_parameters.at(7); }
    const double& weightAvoid()     const { return tuning_parameters.at(8); }
    
    const double& maxDistSeparate() const { return tuning_parameters.at(9); }
    const double& maxDistAlign()    const { return tuning_parameters.at(10); }
    const double& maxDistCohere()   const { return tuning_parameters.at(11); }
    
    // Cosine of threshold angle (max angle from forward to be seen)
    const double& angleSeparate()   const { return tuning_parameters.at(12); }
    const double& angleAlign()      const { return tuning_parameters.at(13); }
    const double& angleCohere()     const { return tuning_parameters.at(14); }
    
    const double& flyAwayMaxDist()  const { return tuning_parameters.at(15); }
    
    // ignore obstacle until predicted impact is in less than this many seconds.
    const double& minTimeToCollide() const { return tuning_parameters.at(16); }
    
    // Default constructor
    FlockParameters() : FlockParameters(hand_tuned_parameters) {}
    
    // Constructor based on a vector of numeric parameters.
    FlockParameters(const std::vector<double>& vector_of_parameters_)
    {
        assert(vector_of_parameters_.size() == tunableParameterCount());
        tuning_parameters = vector_of_parameters_;
        enforceSpeedConstraints();
    }
    
    // Constructor for individual tunable parameters.
    FlockParameters(double max_force,
                    double min_speed,
                    double speed,
                    double max_speed,
                    double weight_forward,
                    double weight_separate,
                    double weight_align,
                    double weight_cohere,
                    double weight_avoid,
                    double max_dist_separate,
                    double max_dist_align,
                    double max_dist_cohere,
                    double angle_separate,
                    double angle_align,
                    double angle_cohere,
                    double fly_away_max_dist,
                    double min_time_to_collide)
    {
        // Set tuning parameter vector to given values.
        tuning_parameters =
        {
            max_force,
            min_speed,
            speed,
            max_speed,
            weight_forward,
            weight_separate,
            weight_align,
            weight_cohere,
            weight_avoid,
            max_dist_separate,
            max_dist_align,
            max_dist_cohere,
            angle_separate,
            angle_align,
            angle_cohere,
            fly_away_max_dist,
            min_time_to_collide
        };
        enforceSpeedConstraints();
    }
    
    // Enforce some constraints since values get randomized by evolutions.
    void enforceSpeedConstraints()
    {
        double a = minSpeed();
        double b = maxSpeed();
        setMinSpeed(std::min(a, b));
        setMinSpeed(std::max(a, b));
        setSpeed(util::clip(initSpeed(), minSpeed(), maxSpeed()));
    }
    
    // The count(/size) of tunable parameters in this class.
    static size_t tunableParameterCount() {return hand_tuned_parameters.size();}

    // The count(/size) of ALL parameters in this class.
    static size_t parameterCount()
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
        std::cout << "  " << tunableParameterCount();
        std::cout << " parameters to be optimized:" << std::endl;
        indent(); debugPrint(maxForce());
        indent(); debugPrint(minSpeed());
        indent(); debugPrint(initSpeed());
        indent(); debugPrint(maxSpeed());
        indent(); debugPrint(weightForward());
        indent(); debugPrint(weightSeparate());
        indent(); debugPrint(weightAlign());
        indent(); debugPrint(weightCohere());
        indent(); debugPrint(weightAvoid());
        indent(); debugPrint(maxDistSeparate());
        indent(); debugPrint(maxDistAlign());
        indent(); debugPrint(maxDistCohere());
        indent(); debugPrint(angleSeparate());
        indent(); debugPrint(angleAlign());
        indent(); debugPrint(angleCohere());
        indent(); debugPrint(flyAwayMaxDist());
        indent(); debugPrint(minTimeToCollide());
    }
    
private:
    // Parameters for tuning:
    std::vector<double> tuning_parameters = hand_tuned_parameters;

    // const/input parameters:
    double body_diameter_ = 1;
    double sphere_radius_ = 50;
    Vec3 sphere_center_;
    int max_simulation_steps_ = 500;
    int boids_per_flock_ = 200;
    std::string use_obstacle_set = "Sphere";
};
