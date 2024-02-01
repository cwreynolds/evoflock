//------------------------------------------------------------------------------
//
//  Boid.h -- new flock experiments
//
//  Boid class, specialization of Agent.
//
//  A Flock is a collection of Boid instances. Boid.fly_with_flock() is its main
//  entry point. Boids are normally created by a Flock. Each Boid is created
//  with a link back to its Flock, for finding neighbors, etc.
//
//  Created by Craig Reynolds on January 27, 2024.
//  (Based on earlier C++ and Python versions.)
//  MIT License -- Copyright © 2024 Craig Reynolds
//------------------------------------------------------------------------------


#pragma once
#include "Vec3.h"
#include "Utilities.h"
#include "Agent.h"
#include "obstacle.h"
#include <algorithm>  // For sort.


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240201 prototype FlockParameters.

// Basically a container for all flocking parameters relevant to optimization.
class FlockParameters
{
public:
};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


class Boid;
typedef std::vector<Boid*> BoidPtrList;

class Flock;

class Draw
{
public:
    static const bool enable = false;
};

class Boid : public Agent
{
private:  // move to bottom of class later
    
    double max_speed_ = 20.0;    // Speed upper limit (m/s)
    double max_force_ = 100.0;     // Acceleration upper limit (m/s²)
    double min_speed_ = max_speed_ * 0.3;  // TODO 20231225 ad hoc factor.
    double speed_ = min_speed_;
    double body_radius_ = 0.5;   // "assume a spherical boid" -- unit diameter
    
    // TODO 20240129 should be wrapped in accessor.
    Flock* flock_ = nullptr;
    
    double sphere_radius_ = 0;
    Vec3 sphere_center_;
    // Set during sense/plan phase, saved for steer phase.
    Vec3 next_steer_;
    // Low pass filter for steering vector.
    util::Blender steer_memory_;
    // Low pass filter for roll control ("up" target).
    util::Blender up_memory_;
    // Cache of nearest neighbors, updating "occasionally".
    //    std::vector<Boid*> cached_nearest_neighbors_;
    BoidPtrList cached_nearest_neighbors_;
    // Seconds between neighbor refresh (Set to zero to turn off caching.)
    double neighbor_refresh_rate_ = 0.5;
    double time_since_last_neighbor_refresh_ = 0;

    // Vec3 wander_state_;     // For wander_steer()
    Vec3 color_;
    Vec3 annote_avoid_poi_ = Vec3();  // This might be too elaborate: two vals
    double annote_avoid_weight_ = 0;    // per boid just for avoid annotation.
    
    // Tuning parameters
    double weight_forward_  = 4;
    double weight_separate_ = 23;
    double weight_align_    = 12;
    double weight_cohere_   = 18;
    double weight_avoid_    = 40;
    double max_dist_separate_ = 15 * body_radius_;
    double max_dist_align_    = 100;
    double max_dist_cohere_   = 100;  // TODO 20231017 should this be ∞ or
    // should the behavior just ignore it?
    
    double exponent_separate_ = 1;  // TODO 20231019 are these useful? Or should
    double exponent_align_    = 1;  // it just assume 1/dist is used to weight
    double exponent_cohere_   = 1;  // all neighbors in all three behaviors?
    // Cosine of threshold angle (max angle from forward to be seen)
    double angle_separate_ = -0.707;  // 135°
    double angle_align_    =  0.940;  // 20°
    double angle_cohere_   =  0;      // 90°
    
    inline static RandomSequence rs_;
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240131 temp work-around to avoid Flock/Boid definition cycle
    bool flock_wrap_vs_avoid = false;
    double flock_min_time_to_collide = 0.8;  // react to predicted impact (seconds)
    bool flock_avoid_blend_mode = true; // obstacle avoid: blend vs hard switch
    ObstaclePtrList flock_obstacles;  // Flock's current list of obstacles.
    BoidPtrList flock_boids;  // List of boids in flock.
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

public:
    // Constructor
    Boid() : Agent()
    {
        // Temp? Pick a random midrange boid color.
        auto mrbc = [](){ return rs_.frandom2(0.5, 0.8); };
        color_ = Vec3(mrbc(), mrbc(), mrbc());
    }

    // Determine and store desired steering for this simulation step
    void plan_next_steer(double time_step)
    {
        next_steer_ = steer_to_flock(time_step);
    }

    // Apply desired steering for this simulation step
    void apply_next_steer(double time_step)
    {
        steer(next_steer_, time_step);
    }

    // Basic flocking behavior. Computes steering force for one simulation step
    // (an animation frame) for one boid in a flock.
    Vec3 steer_to_flock(double time_step)
    {
        BoidPtrList neighbors = nearest_neighbors(time_step);
        Vec3 f = forward() * weight_forward_;
        Vec3 s = steer_to_separate(neighbors) * weight_separate_;
        Vec3 a = steer_to_align(neighbors) * weight_align_;
        Vec3 c = steer_to_cohere(neighbors) * weight_cohere_;
        Vec3 o = steer_to_avoid() * weight_avoid_;
        Vec3 combined_steering = smoothed_steering(f + s + a + c + o);
        combined_steering = anti_stall_adjustment(combined_steering);
        annotation(s, a, c, o, combined_steering);
        return combined_steering;
    }
    
    // Steering force component to move away from neighbors.
    Vec3 steer_to_separate(const BoidPtrList& neighbors)
    {
        Vec3 direction;
        for (Boid* neighbor : neighbors)
        {
            Vec3 offset = position() - neighbor->position();
            double dist = offset.length();
            double weight = 1 / std::pow(dist, exponent_separate_);
            weight *= 1 - util::unit_sigmoid_on_01(dist / max_dist_separate_);
            weight *= angle_weight(neighbor, angle_separate_);
            direction += offset * weight;
        }
        return direction.normalize_or_0();
    }

    // Steering force component to align path with neighbors.
    Vec3 steer_to_align(const BoidPtrList& neighbors)
    {
        Vec3 direction;
        for (Boid* neighbor : neighbors)
        {
            Vec3 heading_offset = neighbor->forward() - forward();
            double dist = (neighbor->position() - position()).length();
            double weight = 1 / pow(dist, exponent_align_);
            weight *= 1 - util::unit_sigmoid_on_01(dist / max_dist_align_);
            weight *= angle_weight(neighbor, angle_align_);
            direction += heading_offset.normalize() * weight;
        }
        return direction.normalize_or_0();
    }

    Vec3 steer_to_cohere(const BoidPtrList& neighbors)
    {
        Vec3 direction;
        Vec3 neighbor_center;
        double total_weight = 0;
        for (Boid* neighbor : neighbors)
        {
            double dist = (neighbor->position() - position()).length();
            double weight = 1 / pow(dist, exponent_cohere_);
            weight *= 1 - util::unit_sigmoid_on_01(dist / max_dist_cohere_);
            weight *= angle_weight(neighbor, angle_cohere_);
            neighbor_center += neighbor->position() * weight;
            total_weight += weight;
        }
        if (total_weight > 0) { neighbor_center /= total_weight; }
        direction = neighbor_center - position();
        return direction.normalize_or_0();
    }

    // Steering force to avoid obstacles. Takes the max of "predictive" avoidance
    // (I will collide with obstacle within Flock.min_time_to_collide seconds)
    // and "static" avoidance (I should fly away from this obstacle, for everted
    // containment obstacles).
    Vec3 steer_to_avoid()
    {
        Vec3 avoid;
        avoid_obstacle_annotation(0, Vec3::none(), 0);
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240131 temp work-around to avoid Flock/Boid definition cycle
//        if (not flock_->wrap_vs_avoid())
        if (not flock_wrap_vs_avoid)
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        {
            Vec3 predict_avoid = steer_for_predictive_avoidance();
            Vec3 static_avoid = fly_away_from_obstacles();
            avoid = Vec3::max({static_avoid, predict_avoid});
        }
        avoid_obstacle_annotation(3, Vec3::none(), 0);
        return avoid;
    }
    
    // Steering force component for predictive obstacles avoidance.
    Vec3 steer_for_predictive_avoidance()
    {
        double weight = 0;
        Vec3 avoidance;
        CollisionList collisions = predict_future_collisions();
        if (not collisions.empty())
        {
            const Collision& first_collision = collisions.front();
            Vec3 poi = first_collision.point_of_impact;
            Vec3 normal = first_collision.normal_at_poi;
            Vec3 pure_steering = pure_lateral_steering(normal);
            avoidance = pure_steering.normalize();
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20240131 temp work-around to avoid Flock/Boid definition cycle
//            double min_dist = speed() * flock_->min_time_to_collide();
            double min_dist = speed() * flock_min_time_to_collide;
            // Near enough to require avoidance steering?
            bool near = min_dist > first_collision.dist_to_collision;
//            if (flock_->avoid_blend_mode())
            if (flock_avoid_blend_mode)
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            {
                // Smooth weight transition from 80% to 120% of min dist.
                double d = util::remap_interval(first_collision.dist_to_collision,
                                                min_dist * 0.8, min_dist * 1.2,
                                                1, 0);
                weight = util::unit_sigmoid_on_01(d);
            }
            else
            {
                weight = near ? 1 : 0;
            }
            avoid_obstacle_annotation(1, poi, weight);
        }
        return avoidance * weight;
    }

    // Computes static obstacle avoidance: steering AWAY from nearby obstacle.
    // Non-predictive "repulsion" from "large" obstacles like walls.
    Vec3 fly_away_from_obstacles()
    {
        Vec3 avoidance;
        Vec3 p = position();
        Vec3 f = forward();
        double max_distance = body_radius_ * 20;  // TODO tuning parameter?
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240131 temp work-around to avoid Flock/Boid definition cycle
//        for (Obstacle* obstacle : flock_->obstacles())
        for (Obstacle* obstacle : flock_obstacles)
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        {
            Vec3 oa = obstacle->fly_away(p, f, max_distance, body_radius_);
            double weight = oa.length();
            avoid_obstacle_annotation(2, obstacle->nearest_point(p), weight);
            avoidance += oa;
        }
        return avoidance;
    }

    // Draw a ray from Boid to point of impact, or nearest point for fly-away.
    // Magenta for strong avoidance, shades to background gray (85%) for gentle
    // avoidance. "Phase" used to show strongest avodiance: predictive vs static.
    void avoid_obstacle_annotation(int phase, Vec3 poi, double weight)
    {
        //    if phase == 0:
        //        self.annote_avoid_poi = Vec3()  # This might be too elaborate: two vals
        //        self.annote_avoid_weight = 0    # per boid just for avoid annotation.
        //    # For predictive avoidance (phase 0) just store poi and weight.
        //    if phase == 1:
        //        self.annote_avoid_poi = poi
        //        self.annote_avoid_weight = weight
        //    # For static avoidance (phase 1) use values for max weight.
        //    if phase == 2:
        //        if weight > self.annote_avoid_weight:
        //            self.annote_avoid_poi = poi
        //            self.annote_avoid_weight = weight
        //    if phase == 3:
        //        if self.should_annotate() and self.annote_avoid_weight > 0.01:
        //            Draw.add_line_segment(self.position,
        //                                  self.annote_avoid_poi,
        //                                  # Interp color between gray and magenta.
        //                                  util.interpolate(self.annote_avoid_weight,
        //                                                   Vec3(0.85, 0.85, 0.85),
        //                                                   Vec3(1, 0, 1)))
    }
    
    
    
    
    // Prevent "stalls" -- when a Boid's speed drops so low that it looks like
    // it is floating rather than flying. Tries to keep the boid's speed above
    // min_speed() (currently (20231227) self.max_speed * 0.3). It starts to
    // adjust when self.speed get within 1.5 times the min speed (which is about
    // self.max_speed * 0.5 now). This is done by extracting the lateral turning
    // component of steering and adding that to a moderate forward acceleration.
    Vec3 anti_stall_adjustment(const Vec3& raw_steering)
    {
        Vec3 adjusted = raw_steering;
        double prevention_margin = 1.5;
        if (speed() < (min_speed_ * prevention_margin))
        {
            if (raw_steering.dot(forward()) < 0)
            {
                Vec3 ahead = forward() * max_force_ * 0.9;
                Vec3 side = raw_steering.perpendicular_component(forward());
                adjusted = ahead + side;
            }
        }
        return adjusted;
    }

    // Wander aimlessly via slowly varying steering force. Remove unused method.

    // Return weight related to neighbor's position relative to my forward axis.
    double angle_weight(Boid* neighbor, double cos_angle_threshold)
    {
        Vec3 offset = neighbor->position() - position();
        double projection_onto_forward = offset.normalize().dot(forward());
        return (projection_onto_forward > cos_angle_threshold) ? 1 : 0;
    }

    // Returns a list of the N Boids nearest this one.
    // (n=3 increased frame rate from ~30 to ~50 fps. No other obvious changes.)
    BoidPtrList nearest_neighbors(double time_step, int n = 7)
    {
        time_since_last_neighbor_refresh_ += time_step;
        if (time_since_last_neighbor_refresh_ > neighbor_refresh_rate_)
        {
            recompute_nearest_neighbors(n);
        }
        return cached_nearest_neighbors_;
    }

    // Recomputes a cached list of the N Boids nearest this one.
    void recompute_nearest_neighbors(int n=7)
    {
        auto distance_squared_from_me = [&](const Boid* boid){
            return (boid->position() - position()).length_squared(); };
        auto sorted = [&](const Boid* a, const Boid* b){
            return distance_squared_from_me(a) < distance_squared_from_me(b); };
        // TODO 20240129 look also at std::partial_sort() and std::stable_sort()
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240131 temp work-around to avoid Flock/Boid definition cycle
//        BoidPtrList all_boids = flock_->boids();
        BoidPtrList all_boids = flock_boids;
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        std::sort(all_boids.begin(), all_boids.end(), sorted);
        
//        // TODO 20240129 probably a better way to do this:
//        cached_nearest_neighbors_.clear();
//        for (int i = 0; i < n; i++)
//        {
//            cached_nearest_neighbors_.push_back(all_boids[i]);
//        }
        
        
//            cached_nearest_neighbors_.resize(n);
//    //        std::copy(cached_nearest_neighbors_.begin(),
//    //                  cached_nearest_neighbors_.begin() + n,
//    //                  all_boids.begin());
//            auto cnnb = cached_nearest_neighbors_.begin();
//            std::copy(cnnb, cnnb + n, all_boids.begin());

//        std::vector<int> count = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
//        std::vector<int> seven(7);
//        std::copy(count.begin(), count.begin() + 7, seven.begin());
//        show(count);
//        show(seven);
        
//        // Set cached_nearest_neighbors_ to nearest "n" in all_boids.
//        cached_nearest_neighbors_.resize(n);
//        auto cnnb = cached_nearest_neighbors_.begin();
//        std::copy(cnnb, cnnb + n, all_boids.begin());

        // Set "cached_nearest_neighbors_" to nearest "n" in "all_boids".
        cached_nearest_neighbors_.resize(n);
        auto abb = all_boids.begin();
        std::copy(abb, abb + n, cached_nearest_neighbors_.begin());

        
        time_since_last_neighbor_refresh_ = 0;
    }
    
    //    # Ad hoc low-pass filtering of steering force. Blends this step's newly
    //    # determined "raw" steering into a per-boid accumulator, then returns that
    //    # smoothed value to use for actually steering the boid this simulation step.
    //    def smoothed_steering(self, steer):
    //        return self.steer_memory.blend(steer, 0.8) # Ad hoc smoothness param.
    
    // Ad hoc low-pass filtering of steering force. Blends this step's newly
    // determined "raw" steering into a per-boid accumulator, then returns that
    // smoothed value to use for actually steering the boid this simulation step.
    Vec3 smoothed_steering(Vec3 steer)
    {
        // TODO 20240128 for now, skip smoothing
        // return steer_memory_.blend(steer, 0.8);  // Ad hoc smoothness param.
        return steer;
    }
    
    //    # Draw this Boid's “body” -- currently an irregular tetrahedron.
    //    def draw(self, color=None):
    //        if Draw.enable:
    //            center = self.position
    //            nose = center + self.forward * self.body_radius
    //            tail = center - self.forward * self.body_radius
    //            bd = self.body_radius * 2  # body diameter (defaults to 1)
    //            apex = tail + self.up * 0.25 * bd + self.forward * 0.1 * bd
    //            wingtip0 = tail + self.side * 0.3 * bd
    //            wingtip1 = tail - self.side * 0.3 * bd
    //            # Draw the 4 triangles of a boid's body.
    //            def draw_tri(a, b, c, color):
    //                Draw.add_colored_triangle(a, b, c, color)
    //            if color is None:
    //                color = self.color
    //            draw_tri(nose, apex,     wingtip1, color * 1.00)
    //            draw_tri(nose, wingtip0, apex,     color * 0.95)
    //            draw_tri(apex, wingtip0, wingtip1, color * 0.90)
    //            draw_tri(nose, wingtip1, wingtip0, color * 0.70)
    //
    //    # Should this Boid be annotated? (At most its those near selected boid.)
    //    def should_annotate(self):
    //        return (Draw.enable and
    //                self.flock.enable_annotation and
    //                self.flock.tracking_camera and
    //                ((self == self.flock.selected_boid()) or
    //                 (self.flock.selected_boid().is_neighbor(self))))

    // Draw optional annotation of this Boid's current steering forces
    void annotation(const Vec3& separation,
                    const Vec3& alignment, 
                    const Vec3& cohesion, 
                    const Vec3& avoidance,
                    const Vec3& combined)
    {
        if (Draw::enable)
        {
            //    scale = 0.05
            //    center = self.position
            //    def relative_force_annotation(offset, color):
            //        Draw.add_line_segment(center, center + offset * scale, color)
            //    if self.should_annotate():
            //        relative_force_annotation(separation, Vec3(1, 0, 0))
            //        relative_force_annotation(alignment,  Vec3(0, 1, 0))
            //        relative_force_annotation(cohesion,   Vec3(0, 0, 1))
            //        relative_force_annotation(avoidance,  Vec3(1, 0, 1))
            //        relative_force_annotation(combined,   Vec3(0.5, 0.5, 0.5))
        }
    }

    
    //    def is_neighbor(self, other_boid):
    //        return other_boid in self.flock.selected_boid().cached_nearest_neighbors

    //    # Bird-like roll control: blends vector toward path curvature center with
    //    # global up. Overrides method in base class Agent
    //    def up_reference(self, acceleration):
    //        new_up = acceleration + Vec3(0, 0.01, 0)  # slight bias toward global up
    //        self.up_memory.blend(new_up, 0.999)
    //        self.up_memory.value = self.up_memory.value.normalize()
    //        return self.up_memory.value
    
    // Bird-like roll control: blends vector toward path curvature center with
    //global up. Overrides method in base class Agent
    Vec3 up_reference(const Vec3& acceleration)
    {
        //    new_up = acceleration + Vec3(0, 0.01, 0)  # slight bias toward global up
        //    self.up_memory.blend(new_up, 0.999)
        //    self.up_memory.value = self.up_memory.value.normalize()
        //    return self.up_memory.value
        return acceleration; // TODO 20240130 MOCK FIX!
    }

    // Returns a list of future collisions sorted by time, with soonest first.
    CollisionList predict_future_collisions() const
    {
        CollisionList collisions;
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240131 temp work-around to avoid Flock/Boid definition cycle
//        for (Obstacle* obstacle : flock_->obstacles())
        for (Obstacle* obstacle : flock_obstacles)
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        {
            Vec3 point_of_impact = obstacle->ray_intersection(position(),
                                                              forward(),
                                                              body_radius_);
            if (not point_of_impact.is_none())
            {
                double dist_to_collision = (point_of_impact - position()).length();
                double time_to_collision = dist_to_collision / speed();
                Vec3 normal_at_poi = obstacle->normal_at_poi(point_of_impact,
                                                             position());
                collisions.push_back(Collision(*obstacle,
                                               time_to_collision,
                                               dist_to_collision,
                                               point_of_impact,
                                               normal_at_poi));
            }
        }
        auto sorted = [&](const Collision& a, const Collision& b)
            { return a.time_to_collision < b.time_to_collision; };
        std::sort(collisions.begin(), collisions.end(), sorted);
        return collisions;
    }

    
    static void unit_test()
    {
        Boid b;
        
        
//            auto show = [](std::vector<int> v)
//            {
//                for (int i : v)
//                {
//                    std::cout << i << " ";
//                }
//                std::cout << std::endl;
//            };
//    //
//    //
//    ////        std::vector<int> zeros = {0, 0, 0, 0, 0};
//    ////        std::vector<int> count = {1, 2, 3, 4, 5};
//    ////        std::vector<int> mixed;
//    ////        mixed.resize(5);
//    ////        std::copy(zeros.begin(), zeros.end(), mixed.begin());
//    ////        show(mixed);
//    ////        std::copy(count.begin() + 1, count.begin() + 4, mixed.begin() + 1);
//    ////        show(mixed);
//    //
//    //
//    //        std::vector<int> count = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
//    //        std::vector<int> seven(7);
//    //        std::copy(count.begin(), count.begin() + 7, seven.begin());
//    //        show(count);
//    //        show(seven);
//
//            int n = 7;
//            std::vector<int> cached_nearest_neighbors_;
//            std::vector<int> all_boids = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
//            cached_nearest_neighbors_.resize(n);
//    //        auto cnnb = cached_nearest_neighbors_.begin();
//    //        std::copy(cnnb, cnnb + n, all_boids.begin());
//            auto abb = all_boids.begin();
//            std::copy(abb, abb + n, cached_nearest_neighbors_.begin());
//
//            show(all_boids);
//            show(cached_nearest_neighbors_);

    }
};
