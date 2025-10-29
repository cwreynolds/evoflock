// -----------------------------------------------------------------------------
// 
//  flock.h -- new flock experiments
//
//  Flock class.
//
//  Contains a collection of Boids and manages a simulation run.
//
//  (Note: aspects of this class related to collecting metrics to be used for
//  optimization during a flock simulation run assume that a given Flock object
//  is used for only one run. Reusing a Flock object requires adding a "reset"
//  method.)
//
//  MIT License -- Copyright © 2023 Craig Reynolds
// 
//  Created by Craig Reynolds on February 1, 2024.
//  (Based on earlier C++ and Python versions.)
//  MIT License -- Copyright © 2024 Craig Reynolds
// -----------------------------------------------------------------------------

#pragma once
#include "Boid.h"
#include "dbscan.h"
#include "Draw.h"
#include "obstacle.h"
#include "Utilities.h"
#include "Vec3.h"

class Flock
{
private:
    // TODO move to bottom of class definition.
    FlockParameters fp_;
    BoidPtrList boids_;

    BoidInstanceList boid_instance_list_;
    util::AnimationClock animation_clock_;

    // TODO Parameters that may (or may not?) be better kept separate from FP.
    int boid_count_ = 200;
    double max_simulation_steps_ = std::numeric_limits<double>::infinity();
    bool fixed_time_step_ = false;
    int fixed_fps_ = 30;  // Should always be overwritten, should this be NaN?
    int seed_ = 1234567890;

    int total_stalls_ = 0;
    int cumulative_sep_fail_ = 0;   // separation fail: a pair of boids touch.

    util::Blender<double> fps_;
        
    // Pointers to all Obstacles in currently selected ObstacleSet.
    static inline ObstaclePtrList obstacles_;

    // Static collection of all known ObstacleSets. "O" cmd cycles through these.
    static inline std::vector<ObstacleSet> obstacle_sets_;

    // Currently selected boid's index in boids().
    int selected_boid_index_ = -1;
    
    // Accumulator for separation score of each Boid on each simulation step.
    double separation_score_sum_ = 0;

public:

    FlockParameters& fp() { return fp_; }
    const FlockParameters& fp() const { return fp_; }

    BoidInstanceList& boid_instance_list() { return boid_instance_list_; }
    const BoidInstanceList& boid_instance_list()const{return boid_instance_list_;}

    BoidPtrList& boids() { return boids_; }
    const BoidPtrList& boids() const { return boids_; }
    static ObstaclePtrList& obstacles() { return obstacles_; }
    static Draw& draw() { return Draw::getInstance(); }
    
    util::AnimationClock& clock() { return animation_clock_; }
    const util::AnimationClock& clock() const { return animation_clock_; }

    double max_simulation_steps() const { return max_simulation_steps_; }
    void set_max_simulation_steps(double mss) { max_simulation_steps_ = mss; }

    bool fixed_time_step() const { return fixed_time_step_; }
    void set_fixed_time_step(bool fts) { fixed_time_step_ = fts; }

    int fixed_fps() const { return fixed_fps_; }
    void set_fixed_fps(int ffps) { fixed_fps_ = ffps; }

    // Number of boids in Flock.
    int boid_count() const { return boid_count_; }
    void set_boid_count(int bc) { boid_count_ = bc; }
    
    // Total number of boid steps per completed flock sim.
    // (Getting the number SO FAR in an ongoing sim requires a different calc.)
    double boidStepPerSim() const
    {
        return fp().boidsPerFlock() * fp().maxSimulationSteps();
    }
    
    // Accumulator for separation score of each Boid on each simulation step.
    double separationScorePerBoidStep() const
    {
        return separation_score_sum_ / boidStepPerSim();
    }
    
    // TODO 20240131 since c++ has no keyword syntax, perhaps move to creating a
    // default Flock then using (eg) set_boid_count(500) to change things? Or
    // just list all of them in the call?!
    //
    // Yes for now, lets just skip args to the constructor to avoid worrying
    // about which parameters are or aren't included there. New answer: none are.
    //
    Flock()
    {
        // This will normally be overwritten, but set a default default.
        set_fixed_fps(30);
        clock().setFPS(fixed_fps());
        initializeStaticScene();
    }

    // Run boids simulation.
    void run()
    {
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20250908 assert FlockParameters only used in GA mode.

//        // Log the FlockParameters object, but only for interactive drawing mode
//        if (draw().enable()) { fp().print(); }

        // Log the FlockParameters object, but only for interactive drawing mode
        if (draw().enable() and EF::usingGA()) { fp().print(); }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        set_fixed_fps(fp().getFPS());
        clock().setFPS(fp().getFPS());
        make_boids(boid_count(), fp().sphereRadius(), fp().sphereCenter());
        draw().beginAnimatedScene();
        while (still_running())
        {
            updateObstacleSetForGUI();
            updateSelectedBoidForGUI();
            clock().setFrameStartTime();
            // Run simulation steps "as fast as possible" or at fixed rate?
            bool afap = not (fixed_time_step() and draw().enable());
            double fd = clock().frameDuration();
            double fdt = clock().frameDurationTarget();
            double step_duration = afap ? fd : fdt;

            draw().beginOneAnimatedFrame();
            bool run_sim_this_frame = draw().runSimulationThisFrame();
            if (run_sim_this_frame)
            {
                // TODO 20250509 this used to be "step_duration" which caused
                // the fly-slow-when-draw-is-turned-off bug. Have not tested
                // but it seems "if (not fixed_time_step() and draw().enable())"
                // then this should again be "step_duration", so the sim frame
                // time matched the draw frame time.
                fly_boids(fdt);
                update_fps();
            }
            // Draw all Boid bodies, whether sim was paused or not.
            for_all_boids([&](Boid* b){ b->draw_body();});
            selectedBoid()->drawAnnotationForBoidAndNeighbors();
            draw().aimAgent() = *selectedBoid();
            draw().endOneAnimatedFrame();
            clock().sleepUntilEndOfFrame(afap ? 0 : step_duration);
            clock().measureFrameDuration(run_sim_this_frame);
            
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20250702 yet another slow frame draw symptom
            if (clock().frameDuration() > 1.1 * clock().frameDurationTarget())
            {
                std::cout << "frameDuration() = "
                          << clock().frameDuration() << std::endl;
            }
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        }
        draw().endAnimatedScene();
        printRunStats();
        if (max_simulation_steps() == std::numeric_limits<double>::infinity())
        {
            std::cout << log_prefix << "Exit at step: ";
            std::cout << clock().frameCounter() << std::endl;
        }
    }

    // Populate this flock by creating "count" boids with uniformly distributed
    // random positions inside a sphere with the given "radius" and "center".
    // Each boid has a uniformly distributed random orientation.
    void make_boids(int count, double radius, Vec3 center)
    {
        // Allocate default Boid instances.
        boid_instance_list().resize(boid_count());
        // Construct BoidPtrList.
        for (Boid& boid : boid_instance_list()) { boids().push_back(&boid); }
        // Set up each new Boid.
        RandomSequence& rs = EF::RS();
        for (Boid* boid : boids()) { init_boid(boid, radius, center, rs); }
        useObstacleSet();
        enforceObsBoidConstraintsDoNotCount();
    }

    std::function<Vec3()> override_steer_function_ = nullptr;

    void init_boid(Boid* boid, double radius, Vec3 center, RandomSequence& rs)
    {
        // uniform over whole sphere enclosure
        //    boid.ls = boid.ls.randomize_orientation()
        //    boid.ls.p = (center + (radius * 0.95 *
        //                           Vec3.random_point_in_unit_radius_sphere()))

        // Environment used for early evolution experiments. Boids start in a
        // clump to the "left" pointing toward a vertical cylinder on the right,
        // inside the big sphere
        boid->set_fp(&fp());
        boid->set_draw(&draw());
        boid->setFlock(this);
        boid->set_flock_boids(&boids());
        boid->set_flock_obstacles(&obstacles());
        boid->override_steer_function_ = override_steer_function_;
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20250925 turn off curriculum learning experiment
        //               VERY TEMP: I GIVE UP, JUST SET INIT SPEED TO 20
        
        boid->setSpeed(20);
        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Vec3 mean_forward(1, 0, 0);
        Vec3 noise_forward = rs.random_point_in_unit_radius_sphere() * 0.1;
        Vec3 new_forward = (mean_forward + noise_forward).normalize();
        boid->set_ls(boid->ls().rotate_to_new_forward(new_forward, Vec3(0, 1, 0)));
        Vec3 center_of_clump = center + Vec3(radius * -0.66, 0, 0);
        Vec3 offset_in_clump = (rs.random_point_in_unit_radius_sphere() *
                                radius * 0.33);
        boid->setPosition(center_of_clump + offset_in_clump);
        // Probably unneeded since initial speed is zero, nevertheless:
        boid->setPreviousPosition(boid->position());
    }

    // Collect flock curvature stats
    double min_curvature_ = +std::numeric_limits<double>::infinity();
    double max_curvature_ = -std::numeric_limits<double>::infinity();
    double sum_curvature_ = 0;
    void collectCurvatureStats()
    {
        auto ccs = [&](Boid* b)
        {
            double c = b->getPathCurvature();
            min_curvature_ = std::min(c, min_curvature_);
            max_curvature_ = std::max(c, max_curvature_);
            sum_curvature_ += c;
        };
        for_all_boids(ccs);
    }

    // Fly each boid in flock for one simulation step. Consists of two sequential
    // steps to avoid artifacts from order of boids. First a "sense/plan" phase
    // which computes the desired steering based on current state. Then an "act"
    // phase which actually moves the boids. Finally statistics are collected.
    void fly_boids(double time_step)
    {
        for_all_boids([&](Boid* b){ b->plan_next_steer();});
        for_all_boids([&](Boid* b){ b->apply_next_steer(time_step);});
        enforceObsBoidConstraints();
        
        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20250920 experimental curriculum learning
        
//        if (EF::usingGP() and (fractionOfSimulationElapsed() < 0.4))
//        if (EF::usingGP() and (fractionOfSimulationElapsed() < 0.2))
        
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
        // TODO 20250921 try evolving JUST speed

//        if (EF::usingGP() and (fractionOfSimulationElapsed() < 0.3))
        if (EF::usingGP())
        {
            //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
            // TODO 20250925 turn off curriculum learning experiment
//            for (auto b : boids()) { b->setObsCollisionCount(0); }
            //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
            
        }
        
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        recordSeparationScorePerStep();
        recordSpeedScorePerStep();
        collectCurvatureStats();
    }

    // Print a one line summary of metrics from this flock simulation.
    void printRunStats() const
    {
        grabPrintLock_evoflock();
        std::cout << log_prefix;
        std::cout << "obs_collisions = ";
        std::cout << getTotalObstacleCollisions();
        std::cout << ", separation_score_sum_ = ";
        std::cout << int(separation_score_sum_);
        double average_speed = averageSpeedPerBoidStep();
        std::cout << std::format(", average speed = {:.3}", average_speed);
        std::cout << std::format(", speedScore() = {:.3}", speedScore());
        if (EF::add_curvature_objective)
        {
            double ac = sum_curvature_ / boidStepPerSim();
            std::cout << std::format(", minc={:.3}", min_curvature_);
            std::cout << std::format(", c={:.3}", ac);
            std::cout << std::format(", maxc={:.3}", max_curvature_);
        }
        std::cout << std::endl;
    }

    // Get total of all recorded obstacle collisions: sum all Boid's counts.
    int getTotalObstacleCollisions() const
    {
        int sum = 0;
        for (auto b : boids()) { sum += b->getObsCollisionCount(); }
        return sum;
    }

    // Given a unit fitness scores (on [0,1]), and an emphasis factor (also on
    // [0,1]), push down low and mid range scores. Uses a piecewise linear "knee"
    // curve, something like exponentiating:0→0, 1→1, but 0.5 goes to something
    // less than 0.5.  TODO 20250427 should this be in Utilities?
    static double emphasizeHighScores(double unit_score, double emphasis)
    {
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20250601 saw this fail today, but no clue about the bad value.
        if (not util::between(unit_score, 0, 1)) { debugPrint(unit_score); }
        assert(util::between(unit_score, 0, 1));
        assert(util::between(emphasis, 0, 1));
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        double x = util::interpolate(emphasis, 0.5, 1.0);  // knee x
        double y = util::interpolate(emphasis, 0.5, 0.0);  // knee y
        return (unit_score < x ?
                util::remap_interval(unit_score, 0, x, 0, y) :
                util::remap_interval(unit_score, x, 1, y, 1));
    }

    // Return a unit fitness component: quality of obstacle avoidance.
    double obstacleCollisionsScore() const
    {
        double count = getTotalObstacleCollisions();
        double non_coll_steps = boidStepPerSim() - count;
        double norm_non_coll_steps = non_coll_steps / boidStepPerSim();
        // Apply a very high exponent to ignore all but nearly perfect scores.
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20250925 reduce speed on collision, but not to zero
        //               plots at WolfAlpha: https://tinyurl.com/4vutsnar
        
//        return std::pow(norm_non_coll_steps, 500);
//        return std::pow(norm_non_coll_steps, EF::usingGA() ? 500 : 5);
        
        //~~~~ ~~~~ ~~~~ ~~~~ ~~~~ ~~~~ ~~~~ ~~~~ ~~~~ ~~~~ ~~~~ ~~~~ ~~~~ ~~~~
        // TODO 20250928 change exponent in obstacleCollisionsScore()
        
//        return std::pow(norm_non_coll_steps, EF::usingGA() ? 500 : 1);
        
        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
        // TODO 20250928 change exponent in obstacleCollisionsScore()

//        return std::pow(norm_non_coll_steps, EF::usingGA() ? 500 : 2);
        
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
        // TODO 20250930 obstacle avoidance exponent back to 500 for GA and GP

//        return std::pow(norm_non_coll_steps, EF::usingGA() ? 500 : 100);
        return std::pow(norm_non_coll_steps, 500);

        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~

        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

        //~~~~ ~~~~ ~~~~ ~~~~ ~~~~ ~~~~ ~~~~ ~~~~ ~~~~ ~~~~ ~~~~ ~~~~ ~~~~ ~~~~

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    }

    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20250511 maybe move to utility.h if kept
    // rename args and vars, this version parrots recordSeparationScorePerStep()
    // would it be any better to pass in a vector of Pair(x,y)s?
    
    static
    double parameterToWeightWithRamps(double parameter,
                                      const std::vector<double>& d,
                                      const std::vector<double>& s)
    {
        double score = 0;
        for (int i = 1; i < d.size(); i++)
        {
            int j = i - 1;
            if (util::between(parameter, d[j], d[i]))
            {
                score = util::remap_interval(parameter, d[j], d[i], s[j], s[i]);
            }
        }
        return score;
    }
    
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

    // Called each simulation step, records stats for the separation score.
    void recordSeparationScorePerStep()
    {
        // Piecewise linear function of distance to score
        std::vector<double> d = {0.0, 1.5, 2.0, 4.0, 6.0};
        std::vector<double> s = {0.0, 0.0, 1.0, 1.0, 0.0};
        for (auto b : boids())
        {
            double distance = b->distanceToNearestNeighbor();
            double score = parameterToWeightWithRamps(distance, d, s);
            b->xxx_temp_separation_score = score;  // temp for annotation
            
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20250920 experimental curriculum learning
            
//            if (EF::usingGP() and (fractionOfSimulationElapsed() < 0.2))
//            if (EF::usingGP() and (fractionOfSimulationElapsed() < 0.4))
            
            //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
            // TODO 20250921 try evolving JUST speed

//            if (EF::usingGP() and (fractionOfSimulationElapsed() < 0.6))
            if (EF::usingGP())
            {
                //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
                // TODO 20250925 turn off curriculum learning experiment
//                score = 1;
                //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
            }
            //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            

            separation_score_sum_ += score;
        }
    }
    

    // Return a unit fitness component: maintaining proper separation distance.
    double separationScore() const
    {
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20250601 saw this fail today, but no clue about the bad value.
        if (not util::between(separationScorePerBoidStep(), 0, 1))
        {
            debugPrint(separation_score_sum_);
            debugPrint(boidStepPerSim());
            debugPrint(separation_score_sum_ / boidStepPerSim());
        }
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        return emphasizeHighScores(separationScorePerBoidStep(), 0.0);
    }


    // Accumulators for speed score.
    // TODO Relocate in file?
    double sum_of_speeds_over_all_boid_steps_ = 0;
    double sum_of_speed_scores_over_all_boid_steps_ = 0;
    
    // Called each simulation step, records stats for the speed score.
    void recordSpeedScorePerStep()
    {
        for (auto b : boids())
        {
            // Sum used for average speed over entire run.
            sum_of_speeds_over_all_boid_steps_ += b->speed();
            
            //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
            // TODO 20250914 VERY ad hoc work-around for zero speed issue
            //               try getting rid of the wide flat "porch" where any
            //               speed below 15 got the same score

            // Sum scores for "speed is in correct range".
            // TODO TEMP WARNING FIX -- raw inline constants.
            
            //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
            // TODO 20250915 revert
//                double score = parameterToWeightWithRamps(b->speed(),
//    //                                                      {15, 19, 21, 25},
//                                                          { 0, 19, 21, 25},
//                                                          { 0,  1,  1,  0});
            
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20250920 experimental curriculum learning

//            double score = parameterToWeightWithRamps(b->speed(),
//                                                      {15, 19, 21, 25},
//                                                      { 0,  1,  1,  0});

            //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
            // TODO 20250925 turn off curriculum learning experiment

//                double score = parameterToWeightWithRamps(b->speed(),
//    //                                                      {15, 19, 21, 25},
//                                                          { 0, 19, 21, 25},
//                                                          { 0,  1,  1,  0});

//            double score = parameterToWeightWithRamps(b->speed(),
//                                                      {15, 19, 21, 25},
//                                                      { 0,  1,  1,  0});

            
            double score = parameterToWeightWithRamps(b->speed(),
//                                                      {15, 19, 21, 25},
//                                                      { 0, 19, 21, 25},
                                                      { 0, 18, 19, 21, 25},
                                                      { 0, .2, 1,  1,  0});

            //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~


            //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

            sum_of_speed_scores_over_all_boid_steps_ += score;
        }
    }
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20250920 experimental curriculum learning
    
//        double fractionOfSimulationElapsed() const
//        {
//            double c = clock().frameCounter();
//            double m = fp().maxSimulationSteps();
//    //            {
//    //                grabPrintLock_evoflock();
//    //    //            debugPrint(double(clock().frameCounter()) /
//    //    //                       fp().maxSimulationSteps());
//    //                debugPrint(c / m);
//    //            }
//    //        return clock().frameCounter() / boidStepPerSim();
//    //        return clock().frameCounter() / fp().maxSimulationSteps();
//            return c / m;
//        }
    
//        double fractionOfSimulationElapsed() const
//        {
//    //        double c = clock().frameCounter();
//    //        double m = fp().maxSimulationSteps();
//    //        return c / m;
//
//            return double(clock().frameCounter()) / fp().maxSimulationSteps();
//
//        }

    double fractionOfSimulationElapsed() const
    {
        return double(clock().frameCounter()) / fp().maxSimulationSteps();
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20250709 only called in printRunStats() replace with speedScore()?
    
    // Average speed for each Boid on each simulation step.
    double averageSpeedPerBoidStep() const
    {
        return sum_of_speeds_over_all_boid_steps_ / boidStepPerSim();
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20250915 for GP, apply score function to average speed (all steps, boids)

    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20250916 maybe bigger trees ARE better, revert speed score

    // Average speed for each Boid on each simulation step.
    double speedScore() const
    {
        return sum_of_speed_scores_over_all_boid_steps_ / boidStepPerSim();
    }


//    // Speed score for all boids on all steps.
//    double speedScore() const
//    {
//        if (EF::usingGA())
//        {
//            return sum_of_speed_scores_over_all_boid_steps_ / boidStepPerSim();
//        }
//        else
//        {
//            // TODO 20250915 very temp, apply score func to average speed
//            return parameterToWeightWithRamps(averageSpeedPerBoidStep(),
//                                                      {15, 19, 21, 25},
//                                                      { 0,  1,  1,  0});
//        }
//    }


    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

    
    double curvatureScore() const
    {
        double average_curvature = sum_curvature_ / boidStepPerSim();
        return util::remap_interval_clip(average_curvature, 0, 0.1, 0.8, 1);
    }
    
    // Test all Boids against each Obstacle. Enforce constraint if necessary by
    // moving Boid to the not-ExcludedFrom side of Obstacle surface.
    void enforceObsBoidConstraints()
    {
        for_all_boids([&](Boid* b){ b->enforceObstacleConstraint(); });
    }
    
    // Just like enforceObsBoidConstraints() but does not count constraint
    // violation due to initial Boid position or when switching obstacle set.
    void enforceObsBoidConstraintsDoNotCount()
    {
        auto enforce_one_boid_do_not_count = [&](Boid* b)
        {
            int preserve_collision_count = b->getObsCollisionCount();
            b->enforceObstacleConstraint();
            b->setObsCollisionCount(preserve_collision_count);
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20250925 turn off curriculum learning experiment
//            b->setSpeed(0);
//            b->setSpeed(20);
            b->setSpeedAfterObstacleCollision();
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        };
        for_all_boids(enforce_one_boid_do_not_count);
    }

    // Apply the given function to all Boids using two parallel threads.
    //
    // (At first this spun up N(=3) new threads to run in parallel. But that
    //  provided almost no benefit. Probably the savings being canceled out by
    //  thread launching overhead. Now creates ONE thread and splits the load
    //  between it and this main thread. On 20240527 I tried adjusting the load
    //  ratio between the threads. (Used util::Timer and averaged over all calls
    //  in an evolution run.) I got maybe 1-2% improvement but didn't think it
    //  was worth the extra code complexity. Verified that two threads are
    //  better than one.)
    //
    void for_all_boids(std::function<void(Boid* b)> boid_func)
    {
        // Apply "boid_func" to boids between indices "first" and "last"
        auto chunk_func = [&](int first, int last)
        {
            int end = std::min(last, int(boids().size()));
            for (int i = first; i < end; i++) { boid_func(boids().at(i)); }
        };
        int boid_count = int(boids().size());
        int boids_per_thread = 1 + (boid_count * 0.5);
        if (EF::enable_multithreading)
        {
            std::thread helper(chunk_func, 0, boids_per_thread);
            chunk_func(boids_per_thread, boid_count);
            helper.join();
        }
        else
        {
            // TODO 20240625 when we switch back to multithreading, need to make
            // sure new thread gets same Boid::getGpPerThread() as main thread.
            chunk_func(0, boid_count);
        }
    };

    std::string log_prefix;

    // Keep track of a smoothed (LPF) version of frames per second metric.
    void update_fps()
    {
        double fd = clock().frameDuration();
        fps_.blend((fixed_time_step() ? fixed_fps() : int(1 / fd)), 0.95);
    }
    
    // Returns pointer to the currently selected boid, the one that the tracking
    // camera tracks, for which steering force annotation is shown.
    Boid* selectedBoid() { return boids().at(selected_boid_index_); }

    // Check if selected boid needs to be changed in response to "S" cmd in UI.
    void updateSelectedBoidForGUI()
    {
        int s = draw().selectedBoidIndex() % boids().size();
        if (s != selected_boid_index_) { selected_boid_index_ = s; }
        Boid::setSelected(boids().at(selected_boid_index_));
    }
    
    void setSelectedBoid(int index_into_boids)
    {
        assert(index_into_boids >= 0 and index_into_boids < boids().size());
        
        selected_boid_index_ = index_into_boids;
        draw().selectedBoidIndex() = index_into_boids;
        Boid::setSelected(boids().at(index_into_boids));
    }
    
    void setSelectedBoid(Boid* boid)
    {
        // TODO probably could be done with std::find?
        for (int i = 0; i < boids().size(); i++)
        {
            if (boids()[i] == boid) { setSelectedBoid(i); }
        }

    }
    
    // Find index into obstacleSets() (obstacle_sets_) for a given string name.
    int obstacleSetsNameToIndex(const std::string& name)
    {
        int index = -1;
        auto os = obstacleSets();
        // TODO probably should use std::find()?
        for (int i = 0; i < os.size(); i++)
        {
            if (name == os.at(i).name()) { index = i; }
        }
        bool obstacle_set_name_found = index >= 0;
        assert(obstacle_set_name_found);
        return index;
    }

    // Create and cache a collection of ObstacleSet objects. One is designated
    // the default initial set, via static FlockParameters. The user can cycle
    // through the sets during simulation using the "O" (obstacle set) command.
    const std::vector<ObstacleSet>& obstacleSets()
    {
        if (obstacle_sets_.empty())
        {
            auto iside = Obstacle::inside;
            auto oside = Obstacle::outside;
            Vec3 sc = FlockParameters().sphereCenter();
            double sr = FlockParameters().sphereRadius();
            
            Obstacle* big_sphere = new SphereObstacle(sr, sc, oside);
            Obstacle* little_sphere = new SphereObstacle(sr / 4, sc, iside);
            Obstacle* right_cyl = new CylinderObstacle(sr * 0.2,
                                                       sc + Vec3(sr*0.6,  sr, 0),
                                                       sc + Vec3(sr*0.6, -sr, 0),
                                                       iside);
            Obstacle* plane = new PlaneObstacle(Vec3(0,1,0), sc, sr, sr * 0.001);
            
            // Set 0: just the big sphere.
            obstacle_sets_.push_back(ObstacleSet("BigSphere",
                                                 {big_sphere}));
            
            // Set 1: one sphere inside another
            obstacle_sets_.push_back(ObstacleSet("BigAndSmallSphere",
                                                 {big_sphere, little_sphere}));
            
            // Set 2: sphere and right hand vertical cylinder.
            obstacle_sets_.push_back(ObstacleSet("BigSphereRightCyl",
                                                 {big_sphere, right_cyl}));
            
            // Set 3: sphere and 6 cylinders parallel to main axes.
            {
                ObstaclePtrList obs;
                obs.push_back(big_sphere);
                double c6r = sr *  4 / 30;
                double c6o = sr * 15 / 30;
                double c6h = 50;
                auto add_3_cyl = [&](double c6o)
                {
                    auto add_cyl = [&](double r, Vec3 t, Vec3 b)
                    { obs.push_back(new CylinderObstacle(r, t, b, iside)); };
                    add_cyl(c6r, Vec3(-c6h, 0, c6o), Vec3(c6h, 0, c6o));
                    add_cyl(c6r, Vec3(c6o, -c6h, 0), Vec3(c6o, c6h, 0));
                    add_cyl(c6r, Vec3(0, c6o, -c6h), Vec3(0, c6o, c6h));
                };
                add_3_cyl(c6o);
                add_3_cyl(-c6o);
                obstacle_sets_.push_back(ObstacleSet("Sphere_and_6_Cyl", obs));
            }
            
            // Set 4: big sphere and horizontal plane.
            obstacle_sets_.push_back(ObstacleSet("BigSpherePlane",
                                                 {big_sphere, plane}));
            
            // Set 5: 35 random spheres inside big sphere.
            {
                int count = 35;
                ObstaclePtrList obs;
                obs.push_back(big_sphere);
                std::vector<double> radii;
                for (int i = 0; i < count; i++)
                {
                    radii.push_back(EF::RS().random2(4, 10));
                }
                double m = 4;  // margin between spheres.
                int t = 4000;  // max retries.
                auto centers = shape::arrangeNonOverlappingSpheres(radii, m, sr, t);
                for (int i = 0; i < count; i++)
                {
                    obs.push_back(new SphereObstacle(radii[i], centers[i], iside));
                }
                obstacle_sets_.push_back(ObstacleSet("SmallSpheresInBigSphere",
                                                     obs));
            }
            
            // Set 6: no obstacles.
            obstacle_sets_.push_back(ObstacleSet("NoObstacles", {}));
        }
        return obstacle_sets_;
    }
    
    // Index (into obstacleSets()) of the currently selected ObstacleSet.
    // -1 indicates initialization is required.
    static inline int selected_obstacle_set_index_ = -1;

    // Check to see if the selected ObstacleSet has changed, say by the user
    // typing the "O" command. If so, propagate change to all boids in flock.
    void useObstacleSet() { useObstacleSet(selected_obstacle_set_index_); }
    void useObstacleSet(int n)
    {
        if (n != selected_obstacle_set_index_)
        {
            // Change ObstacleSet selection on Flock
            obstacles() = obstacleSets().at(n).obstacles();
            // Propagate ObstacleSet selection to all boids in Flock
            setObstaclesOnAllBoids();
            // Clear previous static scene elements.
            draw().clearStaticScene();
            // Update cycle counters.
            selected_obstacle_set_index_ = n;
            draw().obstacleSetIndex() = n;
            // Add new obstacles to static scene.
            for (auto& o : obstacles()) { o->addToScene(); }
        }
    }

    // For each boid in flock: set its obstacle set to be the obstacle set
    // currently selected for this flock.
    void setObstaclesOnAllBoids()
    {
        for_all_boids([&](Boid* b){ b->set_flock_obstacles(&obstacles()); });
    }

private:
    // TODO relocate these to be with other private members.
    static inline std::mutex iss_mutex;
    static inline bool static_scene_initialized = false;
public:
  
    // Initialize static scene for Draw, based on named ObstacleSet.
    // Called from Flock constructor, executed only once, thread safe via mutex.
    void initializeStaticScene() {initializeStaticScene(fp().useObstacleSet());}
    void initializeStaticScene(const std::string& os_name)
    {
        std::lock_guard<std::mutex> issm(iss_mutex);
        if (not static_scene_initialized)
        {
            auto& draw = Draw::getInstance();
            bool enable = draw.enable();
            draw.setEnable(true);
            int os_index = obstacleSetsNameToIndex(os_name);
            useObstacleSet(os_index);
            draw.setEnable(enable);
            static_scene_initialized = true;
        }
    }

    // Check if obstacle set needs to be changed in response to "O" cmd in UI.
    void updateObstacleSetForGUI()
    {
        int osi = draw().obstacleSetIndex() % obstacleSets().size();
        if (osi != selected_obstacle_set_index_)
        {
            useObstacleSet(osi);
            enforceObsBoidConstraintsDoNotCount();
        }
    }
    
    // Simulation continues running until this returns false.
    bool still_running()
    {
        bool a = (not draw().enable()) ? true : not draw().exitFromRun();
        bool b = clock().frameCounter() < max_simulation_steps();
        return a and b;
    }
    
    static void unit_test()
    {
        Flock f;
    }
};
