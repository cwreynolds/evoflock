//------------------------------------------------------------------------------
//
//  Boid.h -- new flock experiments
//
//  Boid class, specialization of Agent.
//
//  A Flock is a collection of Boid instances. Boid.steer_to_flock() is its main
//  entry point (accessed through plan_next_steer() and apply_next_steer() for
//  deterministic behavior). Boids are normally created by a Flock. Each Boid is
//  created with a link back to its Flock.
//
//  Created by Craig Reynolds on January 27, 2024.
//  (Based on earlier C++ and Python versions.)
//  MIT License -- Copyright © 2024 Craig Reynolds
//------------------------------------------------------------------------------

#pragma once
#include "Agent.h"
#include "Draw.h"
#include "FlockParameters.h"
#include "obstacle.h"
#include "Utilities.h"
#include "Vec3.h"

class Boid;
typedef std::vector<Boid*> BoidPtrList;
typedef std::vector<Boid> BoidInstanceList;
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
    
    Color color_;

    // Save this Boid's steering forces for drawing annotation later.
    Vec3 annote_separation_;
    Vec3 annote_alignment_;
    Vec3 annote_cohesion_;
    Vec3 annote_avoidance_;
    Vec3 annote_combined_;
    Vec3 annote_avoid_poi_ = Vec3();
    double annote_avoid_weight_ = 0;

public:
    
    // Accessors
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240617 revisit incremental_sort()
    
//    BoidPtrList& flock_boids() { return *flock_boids_; }
//    const BoidPtrList& flock_boids() const { return *flock_boids_; }
//    void set_flock_boids(BoidPtrList* bpl) { flock_boids_ = bpl; }
    
    
    // TODO each boid keeps its own "approximately sorted" list of flockmates.
    
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

    const Color& color() const { return color_; }
    void setColor(Color c) { color_ = c; }

    std::string name() const { return name_; }
    
    Vec3 getPreviousPosition() const { return previous_position_; }
    void setPreviousPosition(Vec3 prev_pos) { previous_position_ = prev_pos; }


    // Constructor
    Boid() : Agent()
    {
        color_ = Color::randomInRgbBox(Color(0.5), Color(0.8));
        name_ = "boid_" + std::to_string(name_counter_++);
    }

    // Determine and store desired steering for this simulation step
    void plan_next_steer()
    {
        next_steer_ = steer_to_flock();
    }
  
    // Apply the "steering force" -- previously computed in plan_next_steer()
    // during a separate pass -- to this Boid's geometric state.
    void apply_next_steer(double time_step)
    {
        setPreviousPosition(position());
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
    
//    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
//    // TODO 20240724 try ignoring fitness of boid after it hits obstacle.
//    // TODO 20240728 retracted.
//    bool dead = false;
//    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    
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

//        // Basic flocking behavior. Computes steering force for one simulation step
//        // (an animation frame) for one boid in a flock.
//        Vec3 steer_to_flock(double time_step)
//        {
//            BoidPtrList neighbors = nearest_neighbors();
//            flush_cache_of_predicted_obstacle_collisions();
//
//            if (GP_not_GA and override_steer_function)
//            {
//                qqq_counter++;
//    //            debugPrint(Boid::qqq_counter)
//
//                assert(override_steer_function);
//                setGpPerThread(this);
//                Vec3 steering_from_evolved_function = override_steer_function();
//                setGpPerThread(nullptr);
//    //            assert(steering_from_evolved_function.length_squared() > 0);
//                return steering_from_evolved_function;
//            }
//            else
//            {
//                Vec3 f = forward() * fp().weight_forward;
//                Vec3 s = steer_to_separate(neighbors) * fp().weight_separate;
//                Vec3 a = steer_to_align(neighbors) * fp().weight_align;
//                Vec3 c = steer_to_cohere(neighbors) * fp().weight_cohere;
//                Vec3 o = steer_to_avoid() * fp().weight_avoid;
//                Vec3 combined_steering = smoothed_steering(f + s + a + c + o);
//                combined_steering = anti_stall_adjustment(combined_steering);
//                annotation(s, a, c, o, combined_steering);
//                return combined_steering;
//            }
//        }
    
    
//    // very temp debugging utility
//    void print_first_10_flock_boids()
//    {
//        for (int i = 0; i < 10; i++)
//        {
//            std::cout << flock_boids_[i] << " ";
//        }
//        std::cout << std::endl;
//    }

//    // Basic flocking behavior. Computes steering force for one simulation step
//    // (an animation frame) for one boid in a flock.
//    Vec3 steer_to_flock(double time_step)
//    {
//        BoidPtrList neighbors = nearest_neighbors();
//        flush_cache_of_predicted_obstacle_collisions();
//
//        if (GP_not_GA and override_steer_function)
//        {
//            assert(override_steer_function);
//            setGpPerThread(this);
//            Vec3 steering_from_evolved_function = override_steer_function();
//            setGpPerThread(nullptr);
//            return steering_from_evolved_function;
//        }
//        else
//        {
//            Vec3 f = forward() * fp().weight_forward;
//            Vec3 s = steer_to_separate(neighbors) * fp().weight_separate;
//            Vec3 a = steer_to_align(neighbors) * fp().weight_align;
//            Vec3 c = steer_to_cohere(neighbors) * fp().weight_cohere;
//            Vec3 o = steer_to_avoid() * fp().weight_avoid;
//            Vec3 combined_steering = smoothed_steering(f + s + a + c + o);
//            combined_steering = anti_stall_adjustment(combined_steering);
//            annotation(s, a, c, o, combined_steering);
//            return combined_steering;
//        }
//    }

    // Basic flocking behavior. Computes steering force for one simulation step
    // (an animation frame) for one boid in a flock.
    Vec3 steer_to_flock()
    {
        BoidPtrList neighbors = nearest_neighbors();
        flush_cache_of_predicted_obstacle_collisions();

        if (GP_not_GA and override_steer_function)
        {
            // TODO seems redundant, both here and in the IF
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
            saveAnnotation(s, a, c, o, combined_steering);
            return combined_steering;
        }
    }

    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    // TODO 20240816 VERY temp experiment
    
    
    Vec3 pre_GP_steer_to_flock()
    {
        BoidPtrList neighbors = nearest_neighbors();
        flush_cache_of_predicted_obstacle_collisions();

        Vec3 f = forward() * fp().weight_forward;
        Vec3 s = steer_to_separate(neighbors) * fp().weight_separate;
        Vec3 a = steer_to_align(neighbors) * fp().weight_align;
        Vec3 c = steer_to_cohere(neighbors) * fp().weight_cohere;
        Vec3 o = steer_to_avoid() * fp().weight_avoid;
        Vec3 combined_steering = smoothed_steering(f + s + a + c + o);
        combined_steering = anti_stall_adjustment(combined_steering);
        saveAnnotation(s, a, c, o, combined_steering);
        return combined_steering;
    }

    
    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Steering force component to move away from neighbors.
    Vec3 steer_to_separate(const BoidPtrList& neighbors)
    {
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20250320 back to: why aren't boids flocking
        if (isSelected()) { std::cout << std::endl << "steer_to_separate" << std::endl; }
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Vec3 direction;
        for (Boid* neighbor : neighbors)
        {
            assert(neighbor);
            Vec3 offset = position() - neighbor->position();
            double weight = neighborWeight(neighbor,
                                           fp().max_dist_separate,
                                           fp().angle_separate);
            direction += offset * weight;
        }
        return direction.normalize_or_0();
    }

    // Steering force component to align path with neighbors.
    Vec3 steer_to_align(const BoidPtrList& neighbors)
    {
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20250320 back to: why aren't boids flocking
        if (isSelected()) { std::cout << "steer_to_align" << std::endl; }
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Vec3 direction;
        for (Boid* neighbor : neighbors)
        {
            Vec3 heading_offset = neighbor->forward() - forward();
            double weight = neighborWeight(neighbor,
                                           fp().max_dist_align,
                                           fp().angle_align);
            direction += heading_offset.normalize_or_0() * weight;
        }
        return direction.normalize_or_0();
    }

    Vec3 steer_to_cohere(const BoidPtrList& neighbors)
    {
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20250320 back to: why aren't boids flocking
        if (isSelected()) { std::cout << "steer_to_cohere" << std::endl; }
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Vec3 direction;
        Vec3 neighbor_center;
        double total_weight = 0;
        for (Boid* neighbor : neighbors)
        {
            double weight = neighborWeight(neighbor,
                                           fp().max_dist_cohere,
                                           fp().angle_cohere);
            neighbor_center += neighbor->position() * weight;
            total_weight += weight;
        }
        if (total_weight > 0) { neighbor_center /= total_weight; }
        direction = neighbor_center - position();
        Vec3 direction_normalized = direction.normalize_or_0();
        return direction_normalized;
    }

    // Steering force to avoid obstacles. Takes the max of "predictive" avoidance
    // (I will collide with obstacle within min_time_to_collide seconds)
    // and "static" avoidance (I should fly away from this obstacle to add
    // clearance, to avoid scraping along everted containment obstacles).
    Vec3 steer_to_avoid()
    {
        Vec3 avoid;
        avoid_obstacle_annotation(0, Vec3::none(), 0);
        if (not draw().avoidingObstaclesMode()) { return {}; }

        Vec3 predict_avoid = steer_for_predictive_avoidance();
        Vec3 static_avoid = fly_away_from_obstacles();

        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//        avoid = static_avoid + (predict_avoid * 3);
//        avoid = static_avoid + (predict_avoid * 1.5);
//        avoid = static_avoid + (predict_avoid * 3);
//        avoid = (static_avoid * 2) + (predict_avoid * 3);
//        avoid = (static_avoid + predict_avoid) * 2;
//        avoid = (static_avoid + predict_avoid) * 3;
        avoid = (static_avoid * 2) + (predict_avoid * 3);
        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
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
            // Smooth weight transition
            double dtc = first_collision.dist_to_collision;
            weight = util::remap_interval_clip(dtc,  0, min_dist,  1, 0.3);
            avoid_obstacle_annotation(1, poi, weight);
        }
        // TODO should that constant (-0.5) be moved to FlockParameters?
        Vec3 braking = forward() * (avoidance.length() * -0.5);
        return (avoidance + braking) * weight;
    }

    // Computes static obstacle avoidance: steering AWAY from nearby obstacle.
    // Non-predictive "repulsion" to avoid scraping "large" obstacles like walls.
    Vec3 fly_away_from_obstacles()
    {
        Vec3 avoidance;
        Vec3 f = forward();
        Vec3 p = position();
        double br = fp().body_diameter / 2;
        double max_distance = fp().fly_away_max_dist;
        for (Obstacle* obstacle : flock_obstacles())
        {
            Vec3 oa = obstacle->fly_away(p, f, max_distance, br);
            double weight = oa.length();
            avoid_obstacle_annotation(2, obstacle->nearest_point(p), weight);
            avoidance += oa;
        }
        return avoidance;
    }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241219 reconsider avoid_blend_mode

//    // Draw a ray from Boid to point of impact, or nearest point for fly-away.
//    // Magenta for strong avoidance, shades to background gray (85%) for gentle
//    // avoidance. "Phase" used to show strongest avodiance: predictive vs static.
//    void avoid_obstacle_annotation(int phase, Vec3 poi, double weight)
//    {
//        //    if phase == 0:
//        //        self.annote_avoid_poi = Vec3()  # This might be too elaborate: two vals
//        //        self.annote_avoid_weight = 0    # per boid just for avoid annotation.
//        //    # For predictive avoidance (phase 0) just store poi and weight.
//        //    if phase == 1:
//        //        self.annote_avoid_poi = poi
//        //        self.annote_avoid_weight = weight
//        //    # For static avoidance (phase 1) use values for max weight.
//        //    if phase == 2:
//        //        if weight > self.annote_avoid_weight:
//        //            self.annote_avoid_poi = poi
//        //            self.annote_avoid_weight = weight
//        //    if phase == 3:
//        //        if self.should_annotate() and self.annote_avoid_weight > 0.01:
//        //            Draw.add_line_segment(self.position,
//        //                                  self.annote_avoid_poi,
//        //                                  # Interp color between gray and magenta.
//        //                                  util.interpolate(self.annote_avoid_weight,
//        //                                                   Vec3(0.85, 0.85, 0.85),
//        //                                                   Vec3(1, 0, 1)))
//    }

    // Draw a ray from Boid to point of impact, or nearest point for fly-away.
    // Magenta for strong avoidance, shades to background gray (85%) for gentle
    // avoidance. "Phase" used to show strongest avoidance: predictive vs static.
    void avoid_obstacle_annotation(int phase, Vec3 poi, double weight)
    {
        //    if phase == 0:
        //        self.annote_avoid_poi = Vec3()  # This might be too elaborate: two vals
        //        self.annote_avoid_weight = 0    # per boid just for avoid annotation.
        
        // TODO Phase 0 and 1 are identical?
        
        if (phase == 0)
        {
            annote_avoid_poi_ = Vec3();
            annote_avoid_weight_ = 0;
        }
        
        //    # For predictive avoidance (phase 0) just store poi and weight.
        //    if phase == 1:
        //        self.annote_avoid_poi = Vec3()  # This might be too elaborate: two vals
        //        self.annote_avoid_weight = 0    # per boid just for avoid annotation.

        if (phase == 1)
        {
//            annote_avoid_poi_ = Vec3();
//            annote_avoid_weight_ = 0;
            annote_avoid_poi_ = poi;
            annote_avoid_weight_ = weight;
        }
        
        //    # For static avoidance (phase 1) use values for max weight.
        //    if phase == 2:
        //        if weight > self.annote_avoid_weight:
        //            self.annote_avoid_poi = poi
        //            self.annote_avoid_weight = weight
        
        if (phase == 2)
        {
            if (weight > annote_avoid_weight_)
            {
                annote_avoid_poi_ = poi;
                annote_avoid_weight_ = weight;
            }
        }
        
        //    if phase == 3:
        //        if self.should_annotate() and self.annote_avoid_weight > 0.01:
        //            Draw.add_line_segment(self.position,
        //                                  self.annote_avoid_poi,
        //                                  # Interp color between gray and magenta.
        //                                  util.interpolate(self.annote_avoid_weight,
        //                                                   Vec3(0.85, 0.85, 0.85),
        //                                                   Vec3(1, 0, 1)))
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
    
    // Compute a behavioral weight (on [0, 1]) for a neighbor of this Boid.
    // Shared by separate, align, and cohere steering behaviors
    double neighborWeight(Boid* neighbor,
                          double max_dist,
                          double cos_angle_threshold)
    {
        double dist = (neighbor->position() - position()).length();
        double unit_nearness = 1 - util::clip01(dist / max_dist);
        double angular_cutoff = angle_weight(neighbor, cos_angle_threshold);
        double weight = unit_nearness * angular_cutoff;
        if (isSelected())
        {
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20250322 just a FYI (FMI?) I think boids start at speed 0.
            std::cout << std::setw(5) << std::fixed;
            std::cout << " distance = " << dist;
            std::cout << ", weight = " << weight;
            std::cout << " (unit_nearness = " << unit_nearness;
            std::cout << ", angular_cutoff = " << angular_cutoff;
            std::cout << ", max_dist = " << max_dist;
            std::cout << ")" << std::endl;
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        }
        return weight;
    }

    // Binary weight: 1 if neighbor's angular offset from my forward axis is
    // smaller than cos_angle_threshold, and 0 otherwise (say, if behind me).
    double angle_weight(Boid* neighbor, double cos_angle_threshold)
    {
        // Offset from my position to neighbor's position, and unit direction.
        Vec3 offset = neighbor->position() - position();
        Vec3 unit_offset = offset.normalize();
        // Project unit offset onto forward axis.
        double projection_onto_forward = unit_offset.dot(forward());
        // Weight is 1 if projection is bigger than threshold. (So more parallel
        // with forward axis.)
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
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240626 Oh, this is why is_first_boid() stopped working...
//        BoidPtrList& fb = flock_boids();
//        BoidPtrList fb = flock_boids();
        // Copy list of pointers to flock-mates, to sorted in-place.
        BoidPtrList fb = flock_boids();
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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

    // Use Draw api to draw this Boid's “body” -- an irregular tetrahedron.
    void draw_body()
    {
        double bd = fp().body_diameter;  // body diameter (defaults to 1)
        double br = bd / 2;
        Vec3 center = position();
        Vec3 nose = center + forward() * br;
        Vec3 tail = center - forward() * br;
        Vec3 apex = tail + (up() * 0.25 * bd) + (forward() * 0.1 * bd);
        Vec3 wingtip0 = tail + (side() * 0.3 * bd);
        Vec3 wingtip1 = tail - (side() * 0.3 * bd);
        draw().addTriMeshToAnimatedFrame(// vertices
                                         {nose, apex, wingtip0, wingtip1},
                                         // triangles
                                         {1,2,3, 3,2,0, 0,1,3, 2,1,0},
                                         // color
                                         color());
    }
        
    // Save this Boid's steering forces for drawing annotation later.
    void saveAnnotation(const Vec3& separation,
                        const Vec3& alignment,
                        const Vec3& cohesion,
                        const Vec3& avoidance,
                        const Vec3& combined)
    {
        annote_separation_ = separation;
        annote_alignment_ = alignment;
        annote_cohesion_ = cohesion;
        annote_avoidance_ = avoidance;
        annote_combined_ = combined;
    }

    // Draw optional annotation of this Boid's current steering forces
    void drawAnnotation()
    {
        double scale = 0.05;
        auto relative_force_annotation = [&](const Vec3& offset,
                                             const Color& color)
        {
            Vec3 ep = position() + offset * scale;
            draw().addThickLineToAnimatedFrame(position(), ep, color);
        };
        relative_force_annotation(annote_separation_, Color::red());
        relative_force_annotation(annote_alignment_,  Color::green());
        relative_force_annotation(annote_cohesion_,   Color::blue());
        relative_force_annotation(annote_avoidance_,  Color::magenta());
        relative_force_annotation(annote_combined_,   Color::yellow());
        if (annote_avoid_weight_ > 0.1)
        {
            double a = annote_avoid_weight_;
            Color c = util::interpolate(a, Color(0.5), Color(0.8, 0.5, 0.8));
            draw().addThickLineToAnimatedFrame(position(), annote_avoid_poi_, c);
        }
    }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20250320 back to: why aren't boids flocking
    
//    // Called from Flock to draw annotation for selected Boid and its neighbors.
//    void drawAnnotationForBoidAndNeighbors()
//    {
//        drawAnnotation();
//        for (Boid* b : cached_nearest_neighbors()) { b->drawAnnotation(); }
//    }
    
    // Called from Flock to draw annotation for selected Boid and its neighbors.
    void drawAnnotationForBoidAndNeighbors()
    {
        drawAnnotation();
//        for (Boid* b : cached_nearest_neighbors()) { b->drawAnnotation(); }
        for (Boid* b : cached_nearest_neighbors())
        {
            draw().addThickLineToAnimatedFrame(position(), b->position(), Vec3());
        }
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Bird-like roll control: blends vector toward path curvature center with
    // global up. Overrides method in base class Agent
    Vec3 up_reference(const Vec3& acceleration) override
    {
        Vec3 global_up_scaled = Vec3(0, acceleration.length(), 0);
        Vec3 new_up = acceleration + global_up_scaled;
        up_memory_.blend(new_up, 0.95);
        Vec3 up_ref = up_memory_.value.normalize();
        // Make REALLY sure this always returns a unit length vector.
        return (up_ref.is_unit_length() ? up_ref : Vec3(0, 1, 0));

    }

    // This ad hoc global value is provided only for temporary debugging.
    // Do not use this in "real" production code. Probably not thread safe.
    static inline Boid* selected_boid_ = nullptr;
    bool isSelected() const { return selected_boid_ == this; }
    static void setSelected(Boid* b) { selected_boid_ = b; }

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

    // Build list of future Obstacle collisions, sorted with soonest first.
    void cache_predicted_obstacle_collisions()
    {
        predicted_obstacle_collisions_.clear();
        for (Obstacle* o : flock_obstacles())
        {
            // Compute predicted point of impact, if any.
            Vec3 poi = o->rayIntersection(position(), forward(), fp().body_diameter / 2);
            if (not poi.is_none())
            {
                // Make a Collision object, add it to collection of collisions.
                double dist = (poi - position()).length(); // dist_to_collision
                Vec3 normal = o->normalTowardAgent(poi, position());
                Collision c(*o, dist / speed(), dist, poi, normal);
                predicted_obstacle_collisions_.push_back(c);
            }
        }
        // Sort collisions by time_to_collision.
        auto sorted = [&](const Collision& a, const Collision& b)
                      { return a.time_to_collision < b.time_to_collision; };
        std::ranges::sort(predicted_obstacle_collisions_, sorted);
    }

    // TODO / TEMP? / rename?
    // Total count of all obstacle collisions during the lifetime of this Boid.
    int temp_obs_collision_count = 0;
    
    // Test this Boid against each Obstacle in the scene. If it has violated the
    // Obstacle's constraint -- for example crashed through the surface into the
    // interior of an Obstacle with an ExcludeFrom of "inside" -- then it is
    // moved outside, its speed is set to zero, and it is oriented to point away
    // from the obstacle surface.
    void enforceObstacleConstraint()
    {
        for (auto& o : flock_obstacles())
        {
            Vec3 prev_position = getPreviousPosition();
            if (prev_position.is_none()) { prev_position = position(); }
            Vec3 ec = o->enforceConstraint(position(), prev_position);
            if (ec != position())
            {
                // Count collision, set speed to zero, clear smoothing history.
                temp_obs_collision_count++;
                setSpeed(0);
                resetSteerUpMemories();
                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // TODO 20250319 maybe do not point away after obs collisions?
                //               or maybe point away, but start "stunned" clock?

                // Orient boid to point directly away from obstacle.
                Vec3 normal = o->normalTowardAllowedSide(ec, prev_position);
                Vec3 to = ec + (normal * fp().body_diameter);
                set_ls(ls().fromTo(ec, to));

//                setPosition(ec);
                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            }
        }
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
    
    void resetSteerUpMemories()
    {
        steer_memory_.clear();
        up_memory_.clear();
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
