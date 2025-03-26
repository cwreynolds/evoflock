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
    double max_speed = 20.0;                          // Speed upper limit (m/s)
    double max_force = 100.0;                         // Max acceleration (m/s²)
    double min_speed_factor = 0.3;  // (const?)
    double min_speed = max_speed * min_speed_factor;  // Speed lower limit (m/s)
    double speed = min_speed;                         // init speed is min
    
    double body_diameter = 1;      // "assume a spherical boid" -- unit diameter

    double sphere_radius = 50;
    Vec3 sphere_center;
    
    // Tuning parameters
    double weight_forward  = 4;
    double weight_separate = 23;
    double weight_align    = 12;
    double weight_cohere   = 18;
    double weight_avoid    = 40;
    
    double max_dist_separate = 10;
    double max_dist_align    = 100;  // TODO 20231017 should this be ∞ or
    double max_dist_cohere   = 100;  // should the behavior just ignore it?
    
    // Cosine of threshold angle (max angle from forward to be seen)
    double angle_separate = -0.707;  // 135°
    double angle_align    =  0.940;  // 20°
    double angle_cohere   =  0;      // 90°

    double fly_away_max_dist = 10;   // max fly-away dist from obstacle surface

    // ignore obstacle until predicted impact is in less than this many seconds.
    double min_time_to_collide = 0.8;
};
