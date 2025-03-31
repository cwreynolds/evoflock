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
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20250330 try switching to storing parameters in vectors
    
    
//    // Shared read-only "input" parameters:
//    double body_diameter = 1;        // "assume a spherical boid" unit diameter
//    double sphere_radius = 50;       // Should this be called "world radius"?
//    Vec3 sphere_center;              // Should this be called "world center"?

    
    // Shared read-only "input" parameters:
    std::vector<double> const_parameters = {1, 50};
    // "assume a spherical boid" unit diameter
    const double& body_diameter() const { return const_parameters[0]; }
    // Should this be called "world radius"?
    const double& sphere_radius() const { return const_parameters[1]; }
    // Should this be called "world center"?
    Vec3 sphere_center_;
    const Vec3& sphere_center() const { return sphere_center_; }

    // copy assignment operator
    FlockParameters& operator=(const FlockParameters& other)
    {
        const_parameters = other.const_parameters;
        sphere_center_ = other.sphere_center_;

        // TODO copy all tuning parameters
        
        return *this;
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Parameters for tuning:
    double max_force = 100;          // Max acceleration (m/s²)
    double min_speed = 20;           // Speed lower limit (m/s)
    double speed = 0;                // Initial speed (m/s)
    double max_speed = 20;           // Speed upper limit (m/s)
    
    double weight_forward  = 4;
    double weight_separate = 23;
    double weight_align    = 12;
    double weight_cohere   = 18;
    double weight_avoid    = 25;
    
    double max_dist_separate = 10;
    double max_dist_align    = 100;  // TODO 20231017 should this be ∞ or
    double max_dist_cohere   = 100;  //      should the behavior just ignore it?
    
    // Cosine of threshold angle (max angle from forward to be seen)
    double angle_separate = 0;  // 90°
    double angle_align    = 0;  // 90°
    double angle_cohere   = 0;  // 90°

    double fly_away_max_dist = 10;   // max fly-away dist from obstacle surface
    
    // ignore obstacle until predicted impact is in less than this many seconds.
    double min_time_to_collide = 0.8;
    
    // Default constructor
    FlockParameters() {}

    // Constructor for overwriting all of the tunable parameters.
    FlockParameters(double max_force_,
                    double min_speed_,
                    double speed_,
                    double max_speed_,
                    double weight_forward_,
                    double weight_separate_,
                    double weight_align_,
                    double weight_cohere_,
                    double weight_avoid_,
                    double max_dist_separate_,
                    double max_dist_align_,
                    double max_dist_cohere_,
                    double angle_separate_,
                    double angle_align_,
                    double angle_cohere_,
                    double fly_away_max_dist_,
                    double min_time_to_collide_)
      : max_force(max_force_),
        min_speed(min_speed_),
        speed(speed_),
        max_speed(max_speed_),
        weight_forward(weight_forward_),
        weight_separate(weight_separate_),
        weight_align(weight_align_),
        weight_cohere(weight_cohere_),
        weight_avoid(weight_avoid_),
        max_dist_separate(max_dist_separate_),
        max_dist_align(max_dist_align_),
        max_dist_cohere(max_dist_cohere_),
        angle_separate(angle_separate_),
        angle_align(angle_align_),
        angle_cohere(angle_cohere_),
        fly_away_max_dist(fly_away_max_dist_),
        min_time_to_collide(min_time_to_collide_)
    {
        // Enforce some constraints since values get randomized by evolutions.
        double a = min_speed;
        double b = max_speed;
        max_speed = std::max(a, b);
        min_speed = std::min(a, b);
        speed = util::clip(speed, min_speed, max_speed);
    }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20250329 move mess from GP to FlockParameters constructor.
    
    
    FlockParameters(const std::vector<double>& tunable_parameters)
      : FlockParameters(tunable_parameters.at(0),
                        tunable_parameters.at(1),
                        tunable_parameters.at(2),
                        tunable_parameters.at(3),
                        tunable_parameters.at(4),
                        tunable_parameters.at(5),
                        tunable_parameters.at(6),
                        tunable_parameters.at(7),
                        tunable_parameters.at(8),
                        tunable_parameters.at(9),
                        tunable_parameters.at(10),
                        tunable_parameters.at(11),
                        tunable_parameters.at(12),
                        tunable_parameters.at(13),
                        tunable_parameters.at(14),
                        tunable_parameters.at(15),
                        tunable_parameters.at(16))
    {
        assert(tunable_parameters.size() == tunableParameterCount());
    }
    
    // The count(/size) of tunable parameters in this class.
    static size_t tunableParameterCount() { return 17; }
    
    // The count(/size) of ALL parameters in this class.
    static size_t parameterCount() { return 17 + 3; }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    bool operator==(const FlockParameters&) const = default;
    
    void print() const
    {
        auto indent = [](){ std::cout << "    "; };
        std::cout << "FlockParameters object contains these fields:";
        std::cout << std::endl << "  constant:" << std::endl;
        indent(); debugPrint(body_diameter());
        indent(); debugPrint(sphere_radius());
        indent(); debugPrint(sphere_center());

        std::cout << "  parameters to be optimized:" << std::endl;
        indent(); debugPrint(max_force);
        indent(); debugPrint(min_speed);
        indent(); debugPrint(speed);
        indent(); debugPrint(max_speed);
        indent(); debugPrint(weight_forward);
        indent(); debugPrint(weight_separate);
        indent(); debugPrint(weight_align);
        indent(); debugPrint(weight_cohere);
        indent(); debugPrint(weight_avoid);
        indent(); debugPrint(max_dist_separate);
        indent(); debugPrint(max_dist_align);
        indent(); debugPrint(max_dist_cohere);
        indent(); debugPrint(angle_separate);
        indent(); debugPrint(angle_align);
        indent(); debugPrint(angle_cohere);
        indent(); debugPrint(fly_away_max_dist);
        indent(); debugPrint(min_time_to_collide);
    }
};
