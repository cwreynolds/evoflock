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
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20250326 refactor GpTypes in evoflock_ga_function_set_handmade()

public:
//        // Shared constant parameters:
//        double body_diameter = 1;      // "assume a spherical boid" -- unit diameter
//        double sphere_radius = 50;
//        Vec3 sphere_center;
//    //    double min_speed_factor = 0.3;  // (const?)
//
//
//        double max_speed = 20.0;                          // Speed upper limit (m/s)
//        double max_force = 100.0;                         // Max acceleration (m/s²)
//    //    double min_speed = max_speed * min_speed_factor;  // Speed lower limit (m/s)
//        double min_speed = 6;                     // Speed lower limit (m/s)
//    //    double speed = min_speed;                         // init speed is min
//        double speed = 0;                         // init speed is min
//
//    //    double body_diameter = 1;      // "assume a spherical boid" -- unit diameter
//
//    //    double sphere_radius = 50;
//    //    Vec3 sphere_center;
//
//        // Parameters for tuning:
//        double weight_forward  = 4;
//        double weight_separate = 23;
//        double weight_align    = 12;
//        double weight_cohere   = 18;
//        double weight_avoid    = 40;
//
//        double max_dist_separate = 10;
//        double max_dist_align    = 100;  // TODO 20231017 should this be ∞ or
//        double max_dist_cohere   = 100;  // should the behavior just ignore it?
//
//        // Cosine of threshold angle (max angle from forward to be seen)
//        double angle_separate = -0.707;  // 135°
//        double angle_align    =  0.940;  // 20°
//        double angle_cohere   =  0;      // 90°
//
//        double fly_away_max_dist = 10;   // max fly-away dist from obstacle surface
//
//        // ignore obstacle until predicted impact is in less than this many seconds.
//        double min_time_to_collide = 0.8;
    
//        void print() const
//        {
//            std::cout << "FlockParameters object contains these fields:";
//            std::cout << std::endl << "constant:" << std::endl;
//            debugPrint(body_diameter);
//            debugPrint(sphere_radius);
//            debugPrint(sphere_center);
//    //        debugPrint(min_speed_factor); // ??
//
//            std::cout << "parameters to be optimized:" << std::endl;
//            debugPrint(max_speed);
//            debugPrint(max_force);
//    //        debugPrint(min_speed_factor);
//            debugPrint(min_speed);
//            debugPrint(speed);
//    //        debugPrint(body_diameter);
//    //        debugPrint(sphere_radius);
//    //        debugPrint(sphere_center);
//            debugPrint(weight_forward);
//            debugPrint(weight_separate);
//            debugPrint(weight_align);
//            debugPrint(weight_cohere);
//            debugPrint(weight_avoid);
//            debugPrint(max_dist_separate);
//            debugPrint(max_dist_align);
//            debugPrint(max_dist_cohere);
//            debugPrint(angle_separate);
//            debugPrint(angle_align);
//            debugPrint(angle_cohere);
//            debugPrint(fly_away_max_dist);
//            debugPrint(min_time_to_collide);
//        }

    
    // Shared read-only parameters:
//    const double body_diameter = 1;  // "assume a spherical boid" unit diameter
//    const double sphere_radius = 50; // Should this be called "world radius"?
//    const Vec3 sphere_center;        // Should this be called "world center"?
    double body_diameter = 1;        // "assume a spherical boid" unit diameter
    double sphere_radius = 50;       // Should this be called "world radius"?
    Vec3 sphere_center;              // Should this be called "world center"?

    // Parameters for tuning:
    double max_force = 100.0;        // Max acceleration (m/s²)
    double max_speed = 20.0;         // Speed upper limit (m/s)
    double min_speed = 6;            // Speed lower limit (m/s)
    double speed = 0;                // Initial speed.
    
    double weight_forward  = 4;
    double weight_separate = 23;
    double weight_align    = 12;
    double weight_cohere   = 18;
    double weight_avoid    = 40;
    
    double max_dist_separate = 10;
    double max_dist_align    = 100;  // TODO 20231017 should this be ∞ or
    double max_dist_cohere   = 100;  //      should the behavior just ignore it?
    
    // Cosine of threshold angle (max angle from forward to be seen)
    double angle_separate = -0.707;  // 135°
    double angle_align    =  0.940;  // 20°
    double angle_cohere   =  0;      // 90°

    double fly_away_max_dist = 10;   // max fly-away dist from obstacle surface

    // ignore obstacle until predicted impact is in less than this many seconds.
    double min_time_to_collide = 0.8;


    void print() const
    {
        auto indent = [](){ std::cout << "    "; };
        std::cout << "FlockParameters object contains these fields:";
        std::cout << std::endl << "  constant:" << std::endl;
        indent(); debugPrint(body_diameter);
        indent(); debugPrint(sphere_radius);
        indent(); debugPrint(sphere_center);

        std::cout << "  parameters to be optimized:" << std::endl;
        indent(); debugPrint(max_force);
        indent(); debugPrint(max_speed);
//        indent(); debugPrint(max_force);
        indent(); debugPrint(min_speed);
        indent(); debugPrint(speed);
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
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
};
