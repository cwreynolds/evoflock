//------------------------------------------------------------------------------
//
//  FlockParameters.h -- new flock experiments
//
//  A container for all boid flocking parameters relevant to optimization.
//
//  A Flock and its Boids each point to a shared instance of FlockParameters.
//  Evolution adjusts this "genome" of parameters trying to find better fitness.
//
//  Note that all distances are expressed in units of body_diameter which "just
//  happen" to be 1
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
//    double body_radius = 0.5;      // "assume a spherical boid" -- unit diameter
    double body_diameter = 1;      // "assume a spherical boid" -- unit diameter

    double sphere_radius = 50;
    Vec3 sphere_center;
    
    // Tuning parameters
    double weight_forward  = 4;
    double weight_separate = 23;
    double weight_align    = 12;
    double weight_cohere   = 18;
    double weight_avoid    = 40;
    // TODO 20240318 should this (or all 3) be "_in_br" ?
//    double max_dist_separate = 15 * body_radius;
    double max_dist_separate = 8 * body_diameter;
    double max_dist_align    = 100;
    double max_dist_cohere   = 100;  // TODO 20231017 should this be ∞ or
    // should the behavior just ignore it?
    
    double exponent_separate = 1;  // TODO 20231019 are these useful? Or should
    double exponent_align    = 1;  // it just assume 1/dist is used to weight
    double exponent_cohere   = 1;  // all neighbors in all three behaviors?
    // Cosine of threshold angle (max angle from forward to be seen)
    double angle_separate = -0.707;  // 135°
    double angle_align    =  0.940;  // 20°
    double angle_cohere   =  0;      // 90°
//    double fly_away_max_dist_in_br = 20;  // max fly-away dist from obs surface
    double fly_away_max_dist = 10 * body_diameter;  // max fly-away dist from obs surface
    double min_time_to_collide = 0.8;     // react to predicted impact (seconds)
};
