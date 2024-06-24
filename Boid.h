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
#include <algorithm>  // For sorting cached_nearest_neighbors_.

class Boid;
typedef std::vector<Boid*> BoidPtrList;
typedef std::vector<Boid> BoidInstanceList;

// TODO 20240203 mock of Draw class for prototyping
class Draw
{
public:
    bool enable() { return false; }
    double frame_duration() const { return frame_duration_; }
    bool poll_events() const { return true; }
    int frame_counter() const { return frame_counter_; }

    // Measure how much wall clock time has elapsed for this simulation step.
    void measure_frame_duration()
    {
        util::TimePoint frame_end_time = util::TimeClock::now();
        frame_duration_ = util::time_diff_in_seconds(frame_end_time,
                                                     frame_start_time);
        frame_start_time = frame_end_time;
        frame_counter_ += 1;
    }
    
    util::TimePoint frame_start_time;
    double frame_duration_ = 0; // measured in seconds
    int frame_counter_ = 0;
};

// Basically a container for all flocking parameters relevant to optimization.
class FlockParameters
{
public:
    double max_speed = 20.0;                          // Speed upper limit (m/s)
    double max_force = 100.0;                         // Max acceleration (m/s²)
//    double min_speed = max_speed * 0.3;  // TODO 20231225 ad hoc factor.
    double min_speed_factor = 0.3;
    double min_speed = max_speed * min_speed_factor;  // Speed lower limit (m/s)
    double speed = min_speed;                         // init speed is min
    double body_radius = 0.5;      // "assume a spherical boid" -- unit diameter
    
    double sphere_radius = 50;
    Vec3 sphere_center;
    
    // Tuning parameters
    double weight_forward  = 4;
    double weight_separate = 23;
    double weight_align    = 12;
    double weight_cohere   = 18;
    double weight_avoid    = 40;
    // TODO 20240318 should this (or all 3) be "_in_br" ?
    double max_dist_separate = 15 * body_radius;
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
    double fly_away_max_dist_in_br = 20;  // max fly-away dist from obs surface
    double min_time_to_collide = 0.8;     // react to predicted impact (seconds)

    // Historical holdovers, no longer used.
    bool wrap_vs_avoid = false;   // always false
    bool avoid_blend_mode = true; // obstacle avoid: blend vs hard switch
};

class Flock;

class Boid : public Agent
{
private:  // move to bottom of class later
    Flock* flock_ = nullptr;
    Draw* draw_ = nullptr;
    FlockParameters* fp_ = nullptr;
    ObstaclePtrList* flock_obstacles_;  // Flock's current list of obstacles.
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240617 revisit incremental_sort()
//    BoidPtrList* flock_boids_;  // List of boids in flock.
    BoidPtrList flock_boids_;  // List of boids in flock.
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Set during sense/plan phase, saved for steer phase.
    Vec3 next_steer_;

    Vec3 color_;
    Vec3 annote_avoid_poi_ = Vec3();  // This might be too elaborate: two vals
    double annote_avoid_weight_ = 0;  // per boid just for avoid annotation.
    
    // Cumulative count: how many avoidance failures (collisions) has it had?
    int avoidance_failure_counter_ = 0;

    // Low pass filter for steering vector.
    util::Blender<Vec3> steer_memory_;
    // Low pass filter for roll control ("up" target).
    util::Blender<Vec3> up_memory_;
    
    // Cache of nearest neighbors.
    BoidPtrList cached_nearest_neighbors_;
    int neighbors_count = 7;

    // Used to detect agent crossing Obstacle surface.
    Vec3 previous_position_ = Vec3::none();
    
    // Per step cache of predicted obstacle collisions.
    CollisionList predicted_obstacle_collisions_;
    bool predicted_obstacle_collisions_cached_this_step_ = false;

    // Used to generate unique string names for Boid instances
    std::string name_;
    static inline int name_counter_ = 0;
    
public:
    // Accessors
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240617 revisit incremental_sort()
    
//    BoidPtrList& flock_boids() { return *flock_boids_; }
//    const BoidPtrList& flock_boids() const { return *flock_boids_; }
//    void set_flock_boids(BoidPtrList* bpl) { flock_boids_ = bpl; }
    
    
    // TODO each boid keeps its own "approximjately sorted" list of flockmates.
    
    BoidPtrList& flock_boids() { return flock_boids_; }
    const BoidPtrList& flock_boids() const { return flock_boids_; }
    void set_flock_boids(BoidPtrList* bpl) { flock_boids_ = *bpl; }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    ObstaclePtrList& flock_obstacles() { return *flock_obstacles_; }
    const ObstaclePtrList& flock_obstacles() const { return *flock_obstacles_; }
    void set_flock_obstacles(ObstaclePtrList* opl) { flock_obstacles_ = opl; }
    
    FlockParameters& fp() { return *fp_; }
    const FlockParameters& fp() const { return *fp_; }
    
    void set_fp(FlockParameters* fp)
    {
        fp_ = fp;
        setSpeed(fp->speed);
        setMaxSpeed(fp->max_speed);
        setMaxForce(fp->max_force);
    }
    
    Draw& draw() { return *draw_; }
    const Draw& draw() const { return *draw_; }
    void set_draw(Draw* draw) { draw_ = draw; }

    // Cache of nearest neighbors, updating "occasionally".
    const BoidPtrList& cached_nearest_neighbors() const
    {
        return cached_nearest_neighbors_;
    }

    int avoidance_failure_counter() const {return avoidance_failure_counter_;};

    std::string name() const { return name_; }
    
    // Constructor
    Boid() : Agent()
    {
        // Temp? Pick a random midrange boid color.
        auto mrbc = [](){ return EF::RS().frandom2(0.5, 0.8); };
        color_ = Vec3(mrbc(), mrbc(), mrbc());
        name_ = "boid_" + std::to_string(name_counter_++);
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
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240619 WIP first GP_not_GA run
    //               VERY PROTOTYPE!!
    
//    // Basic flocking behavior. Computes steering force for one simulation step
//    // (an animation frame) for one boid in a flock.
//    Vec3 steer_to_flock(double time_step)
//    {
//        BoidPtrList neighbors = nearest_neighbors();
//        flush_cache_of_predicted_obstacle_collisions();
//        Vec3 f = forward() * fp().weight_forward;
//        Vec3 s = steer_to_separate(neighbors) * fp().weight_separate;
//        Vec3 a = steer_to_align(neighbors) * fp().weight_align;
//        Vec3 c = steer_to_cohere(neighbors) * fp().weight_cohere;
//        Vec3 o = steer_to_avoid() * fp().weight_avoid;
//        Vec3 combined_steering = smoothed_steering(f + s + a + c + o);
//        combined_steering = anti_stall_adjustment(combined_steering);
//        annotation(s, a, c, o, combined_steering);
//        return combined_steering;
//    }
  
    static inline bool GP_not_GA = false;
    
    std::function<Vec3()> override_steer_function = nullptr;
    
    // In the GP (vs GA) version, the evolved code is a per-frame steering function
    // for each Boid. This API supplies a "per thread global" which points to the
    // current Boid.
    static inline thread_local Boid* gp_boid_per_thread_ = nullptr;
    static void setGpPerThread(Boid* boid) { gp_boid_per_thread_ = boid; }
    static Boid* getGpPerThread()
    {
        assert(gp_boid_per_thread_ && "invalid Boid::gp_boid_per_thread_");
        return gp_boid_per_thread_;
    }
    
    static inline int qqq_counter = 0;

    // Basic flocking behavior. Computes steering force for one simulation step
    // (an animation frame) for one boid in a flock.
    Vec3 steer_to_flock(double time_step)
    {
        BoidPtrList neighbors = nearest_neighbors();
        flush_cache_of_predicted_obstacle_collisions();

        if (GP_not_GA and override_steer_function)
        {
            qqq_counter++;
            debugPrint(Boid::qqq_counter)
            
            assert(override_steer_function);
            setGpPerThread(this);
            Vec3 steering_from_evolved_function = override_steer_function();
            setGpPerThread(nullptr);
            return steering_from_evolved_function;
        }
        else
        {
            Vec3 f = forward() * fp().weight_forward;
            Vec3 s = steer_to_separate(neighbors) * fp().weight_separate;
            Vec3 a = steer_to_align(neighbors) * fp().weight_align;
            Vec3 c = steer_to_cohere(neighbors) * fp().weight_cohere;
            Vec3 o = steer_to_avoid() * fp().weight_avoid;
            Vec3 combined_steering = smoothed_steering(f + s + a + c + o);
            combined_steering = anti_stall_adjustment(combined_steering);
            annotation(s, a, c, o, combined_steering);
            return combined_steering;
        }
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Steering force component to move away from neighbors.
    Vec3 steer_to_separate(const BoidPtrList& neighbors)
    {
        Vec3 direction;
        for (Boid* neighbor : neighbors)
        {
            assert(neighbor);
            
            Vec3 offset = position() - neighbor->position();
            double dist = offset.length();
            double weight = 1 / std::pow(dist, fp().exponent_separate);
            weight *= 1 - util::unit_sigmoid_on_01(dist / fp().max_dist_separate);
            weight *= angle_weight(neighbor, fp().angle_separate);
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
            double weight = 1 / pow(dist, fp().exponent_align);
            weight *= 1 - util::unit_sigmoid_on_01(dist / fp().max_dist_align);
            weight *= angle_weight(neighbor, fp().angle_align);
            direction += heading_offset.normalize_or_0() * weight;
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
            double weight = 1 / pow(dist, fp().exponent_cohere);
            weight *= 1 - util::unit_sigmoid_on_01(dist / fp().max_dist_cohere);
            weight *= angle_weight(neighbor, fp().angle_cohere);
            neighbor_center += neighbor->position() * weight;
            total_weight += weight;
        }
        if (total_weight > 0) { neighbor_center /= total_weight; }
        direction = neighbor_center - position();
        return direction.normalize_or_0();
    }

    // Steering force to avoid obstacles. Takes the max of "predictive" avoidance
    // (I will collide with obstacle within min_time_to_collide seconds)
    // and "static" avoidance (I should fly away from this obstacle, for everted
    // containment obstacles).
    Vec3 steer_to_avoid()
    {
        Vec3 avoid;
        avoid_obstacle_annotation(0, Vec3::none(), 0);
        if (not fp().wrap_vs_avoid)
        {
            Vec3 predict_avoid = steer_for_predictive_avoidance();
            Vec3 static_avoid = fly_away_from_obstacles();
            avoid = static_avoid + predict_avoid;
        }
        avoid_obstacle_annotation(3, Vec3::none(), 0);
        return avoid;
    }
    
    // Steering force component for predictive obstacles avoidance.
    Vec3 steer_for_predictive_avoidance()
    {
        double weight = 0;
        Vec3 avoidance;
        CollisionList collisions = get_predicted_obstacle_collisions();
        if (not collisions.empty())
        {
            const Collision& first_collision = collisions.front();
            Vec3 poi = first_collision.point_of_impact;
            Vec3 normal = first_collision.normal_at_poi;
            Vec3 pure_steering = pure_lateral_steering(normal);
            avoidance = pure_steering.normalize();
            double min_dist = speed() * fp().min_time_to_collide;
            // Near enough to require avoidance steering?
            bool near = min_dist > first_collision.dist_to_collision;
            if (fp().avoid_blend_mode)
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
        double max_distance = fp().body_radius * fp().fly_away_max_dist_in_br;
        for (Obstacle* obstacle : flock_obstacles())
        {
            Vec3 oa = obstacle->fly_away(p, f, max_distance, fp().body_radius);
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
        if (speed() < (fp().min_speed * prevention_margin))
        {
            if (raw_steering.dot(forward()) < 0)
            {
                Vec3 ahead = forward() * fp().max_force * 0.9;
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

    // Returns a list of the "neighbors_count" Boids nearest this one.
    BoidPtrList nearest_neighbors() {return nearest_neighbors(neighbors_count);}
    BoidPtrList nearest_neighbors(int n){return recompute_nearest_neighbors(n);}

    // Recomputes a cached list of the "neighbors_count" Boids nearest this one.
    BoidPtrList recompute_nearest_neighbors()
    {
        return recompute_nearest_neighbors(neighbors_count);
    }
    BoidPtrList recompute_nearest_neighbors(int n)
    {
        BoidPtrList& fb = flock_boids();
        // How far is given boid from "this" boid? Returns infinity for itself.
        auto distance_squared_from_me = [&](const Boid* boid)
        {
            double d2 = (boid->position() - position()).length_squared();
            return (d2 > 0) ? d2 : std::numeric_limits<double>::infinity();
        };
        // Are boids a and b sorted by least distance from me?
        auto sorted = [&](const Boid* a, const Boid* b)
        {
            return distance_squared_from_me(a) < distance_squared_from_me(b);
        };
        // Maybe neighbor list size should be min(n,flock.size())? But for now:
        assert((fb.size() > n) && "neighborhood > flock size");
        // Sort all boids in flock by nearest distance (squared) from me.
        std::partial_sort(fb.begin(), fb.begin() + n, fb.end(), sorted);
        // Set "cached_nearest_neighbors_" to nearest "n" of flock_boids.
        cached_nearest_neighbors_.resize(n);
        std::copy(fb.begin(), fb.begin() + n, cached_nearest_neighbors_.begin());
        // Verify neighbor distances.
        assert(distance_squared_from_me(cached_nearest_neighbors_[0]) > 0);
        return cached_nearest_neighbors_;
    }
    
    // Ad hoc low-pass filtering of steering force. Blends this step's newly
    // determined "raw" steering into a per-boid accumulator, then returns that
    // smoothed value to use for actually steering the boid this simulation step.
    Vec3 smoothed_steering(Vec3 steer)
    {
        return steer_memory_.blend(steer, 0.8);  // Ad hoc smoothness param.
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
        if (draw().enable())
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

    // Bird-like roll control: blends vector toward path curvature center with
    // global up. Overrides method in base class Agent
    Vec3 up_reference(const Vec3& acceleration) override
    {
        Vec3 global_up_scaled = Vec3(0, acceleration.length(), 0);
        Vec3 new_up = acceleration + global_up_scaled;
        up_memory_.blend(new_up, 0.95);
        return up_memory_.value.normalize();
    }

    void flush_cache_of_predicted_obstacle_collisions()
    {
        predicted_obstacle_collisions_cached_this_step_ = false;
    }

    CollisionList get_predicted_obstacle_collisions()
    {
        if (not predicted_obstacle_collisions_cached_this_step_)
        {
            cache_predicted_obstacle_collisions();
            predicted_obstacle_collisions_cached_this_step_ = true;
        }
        return predicted_obstacle_collisions_;
    }

    // Build a list of future collisions sorted by time, with soonest first.
    void cache_predicted_obstacle_collisions()
    {
        predicted_obstacle_collisions_.clear();
        for (Obstacle* obstacle : flock_obstacles())
        {
            // Compute predicted point of impact, if any.
            Vec3 poi = obstacle->ray_intersection(position(),
                                                  forward(),
                                                  fp().body_radius);
            if (not poi.is_none())
            {
                double dist_to_collision = (poi - position()).length();
                double time_to_collision = dist_to_collision / speed();
                Vec3 normal_at_poi = obstacle->normal_at_poi(poi, position());
                predicted_obstacle_collisions_.push_back({*obstacle,
                                                          time_to_collision,
                                                          dist_to_collision,
                                                          poi,
                                                          normal_at_poi});
            }
        }
        // This would have caught the bug I spent days tracking down (20240617):
        assert(predicted_obstacle_collisions_.size()<=flock_obstacles().size());
        auto sorted = [&](const Collision& a, const Collision& b)
        { 
            return a.time_to_collision < b.time_to_collision;
        };
        std::ranges::sort(predicted_obstacle_collisions_, sorted);
    }

    bool detectObstacleViolations()
    {
        bool violation = false;
        for (Obstacle* obstacle : flock_obstacles())
        {
            if (obstacle->constraintViolation(position(), previous_position_))
            {
                avoidance_failure_counter_ += 1;
                violation = true;
            }
        }
        previous_position_ = position();
        return violation;
    }
    
    // Only used for debugging, to to pick out one boid to track/log/whatever.
    bool is_first_boid() const { return this == flock_boids().at(0); }
    
    // For debugging: does this boid instance appear to be valid?
    // TODO 20230204 if needed again, should check other invariants.
    bool is_valid() const
    {
        return ((speed() == 0 or std::isnormal(speed())) and
                true);
    }
    void assert_valid() const { assert(is_valid()); }

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
