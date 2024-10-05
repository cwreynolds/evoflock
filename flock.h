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

#include "LazyPredator/LazyPredator.h"
namespace LP = LazyPredator;
#include <fstream>  // for logging simulation data to file.


class Flock
{
private:
    // TODO move to bottom of class definition.
    FlockParameters fp_;
    BoidPtrList boids_;
    ObstaclePtrList obstacles_;
    BoidInstanceList boid_instance_list_;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240914 using Draw::globalObject
#ifdef USE_OPEN3D
//    Draw& draw_ = *Draw::globalObject;
    Draw& draw_ = Draw::getInstance();
#else  // USE_OPEN3D
    Draw draw_;
#endif  // USE_OPEN3D
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    util::AnimationTimer animation_timer;

    // TODO Parameters that may (or may not?) be better kept separate from FP.
    int boid_count_ = 200;
    double max_simulation_steps_ = std::numeric_limits<double>::infinity();
    bool fixed_time_step_ = false;
    int fixed_fps_ = 60;
    int seed_ = 1234567890;

    bool simulation_paused_ = false; // Simulation stopped, display continues.
    bool single_step_ = false;       // perform one simulation step then pause.

    int total_stalls_ = 0;
    int cumulative_sep_fail_ = 0;   // separation fail: a pair of boids touch.

    util::Blender<double> fps_;

public:
    
    FlockParameters& fp() { return fp_; }
    const FlockParameters& fp() const { return fp_; }

    BoidInstanceList& boid_instance_list() { return boid_instance_list_; }
    const BoidInstanceList& boid_instance_list()const{return boid_instance_list_;}

    BoidPtrList& boids() { return boids_; }
    const BoidPtrList& boids() const { return boids_; }
    
    ObstaclePtrList& obstacles() { return obstacles_; }
    const ObstaclePtrList& obstacles() const { return obstacles_; }
    
    Draw& draw() { return draw_; }
    const Draw& draw() const { return draw_; }

    util::AnimationTimer& aTimer() { return animation_timer; }
    const util::AnimationTimer& aTimer() const { return animation_timer; }

    double max_simulation_steps() const { return max_simulation_steps_; }
    void set_max_simulation_steps(double mss) { max_simulation_steps_ = mss; }

    bool fixed_time_step() const { return fixed_time_step_; }
    void set_fixed_time_step(bool fts) { fixed_time_step_ = fts; }

    int fixed_fps() const { return fixed_fps_; }
    void set_fixed_fps(int ffps) { fixed_fps_ = ffps; }

    // Number of boids in Flock.
    int boid_count() const { return boid_count_; }
    void set_boid_count(int bc) { boid_count_ = bc; }

    
    

    // TODO 20240131 since c++ has no keyword syntax, perhaps move to creating a
    // default Flock then using (eg) set_boid_count(500) to change things? Or
    // just list all of them in the call?!
    //
    // Yes for now, lets just skip args to the constructor to avoid worrying
    // about which parameters are or aren't included there. New answer: none are.

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240324 very ad hoc since these OccupancyMap parameters depend on
    //               the enclosing EvertedSphereObstacle. The OccupancyMap
    //               should probably be constructed much later, perhaps in run()
    shape::OccupancyMap occupancy_map;
    Flock() : occupancy_map(Vec3(25, 25, 25), Vec3(100, 100, 100), Vec3())
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
        pre_defined_obstacle_sets();
    }

    // Run boids simulation.
    void run()
    {
        // Draw.start_visualizer(self.sphere_radius, self.sphere_center)
        // Flock.vis_pairs.add_pair(Draw.vis, self)  # Pairing for key handlers.
        // self.register_single_key_commands() # For Open3D visualizer GUI.
        make_boids(boid_count(), fp().sphere_radius, fp().sphere_center);
        // self.draw()
        // self.cycle_obstacle_selection()
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240916 reset frame_counter, etc.
//        draw().beginAnimatedDisplay();
        draw().beginAnimatedScene();
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        save_centers_to_file_start();

        while (still_running())
        {
            if (run_simulation_this_frame())
            {
                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // TODO 20240928 integrate with Flock simulation
                draw().beginOneAnimatedFrame();
                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Draw.clear_scene()
                fly_boids((fixed_time_step() or not draw().enable()) ?
                          1.0 / fixed_fps() :
                          aTimer().frame_duration());
                save_centers_to_file_1_step();
                // self.draw()
                // Draw.update_scene()
                if (not simulation_paused_) { aTimer().measure_frame_duration(); }
                log_stats();
                update_fps();

                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // TODO 20241004 follow cam
                draw().aimTarget() = selectedBoid()->position();
                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // TODO 20240928 integrate with Flock simulation
                draw().endOneAnimatedFrame();
                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            }
        }
        save_centers_to_file_end();
        // Draw.close_visualizer()
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240916 reset frame_counter, etc.
        draw().endAnimatedScene();
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//        std::cout << "Exit at step:" << draw().frame_counter() << std::endl;
        if (max_simulation_steps() == std::numeric_limits<double>::infinity())
        {
            std::cout << log_prefix << "Exit at step: ";
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20240916 reset frame_counter, etc.
//            std::cout << draw().frame_counter() << std::endl;
            std::cout << aTimer().frame_counter() << std::endl;
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240619 WIP first GP_not_GA run
    std::function<Vec3()> override_steer_function = nullptr;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
        boid->set_flock_boids(&boids());
        boid->set_flock_obstacles(&obstacles());
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240619 WIP first GP_not_GA run
        boid->override_steer_function = override_steer_function;
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        Vec3 mean_forward(1, 0, 0);
        Vec3 noise_forward = rs.random_point_in_unit_radius_sphere() * 0.1;
        Vec3 new_forward = (mean_forward + noise_forward).normalize();
        boid->set_ls(boid->ls().rotate_to_new_forward(new_forward, Vec3(0, 1, 0)));
        Vec3 center_of_clump = center + Vec3(radius * -0.66, 0, 0);
        Vec3 offset_in_clump = (rs.random_point_in_unit_radius_sphere() *
                                radius * 0.33);
        boid->setPosition(center_of_clump + offset_in_clump);
    }

    //        # Draw each boid in flock.
    //        def draw(self):
    //            if Draw.enable:
    //                Draw.update_camera(self.selected_boid().position if
    //                                   self.tracking_camera else Vec3())
    //                for o in self.obstacles:
    //                    o.draw()
    //                for boid in self.boids:
    //                    boid.draw(color=(Vec3(0, 1, 0) if
    //                                     self.is_neighbor_of_selected(boid) else None))

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO for simplicity, change get/setSaveBoidCenters() to be static

    // TODO just WIP for prototyping
//    std::ofstream* boid_center_data_stream_;
//    bool save_boid_centers_ = true;
//    void setSaveBoidCenters(bool save) { save_boid_centers_ = save; }
//    bool getSaveBoidCenters() const { return save_boid_centers_; }

    std::ofstream* boid_center_data_stream_;
    static inline bool save_boid_centers_ = false;
    void static setSaveBoidCenters(bool save) { save_boid_centers_ = save; }
    bool static getSaveBoidCenters() { return save_boid_centers_; }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//    void save_centers_to_file_start()
//    {
//        if (save_boid_centers_)
//        {
//            std::string file_name = "/Users/cwr/Desktop/boid_centers.py";
//            boid_center_data_stream_ = new std::ofstream{file_name};
//            (*boid_center_data_stream_) << "boid_centers = [" << std::endl;
//        }
//    }
    
    
    static inline int save_boid_centers_count_ = 0;

    std::string save_centers_to_file_pathname()
    {
        return ("/Users/cwr/Desktop/flock_data/boid_centers_" +
                std::to_string(save_boid_centers_count_++) +
                ".py");
    }
    
    
    void save_centers_to_file_start()
    {
        if (save_boid_centers_)
        {
//            std::string file_name = "/Users/cwr/Desktop/boid_centers.py";
            std::string file_name = save_centers_to_file_pathname();
            std::cout << "Opening " << file_name << std::endl;
            boid_center_data_stream_ = new std::ofstream{file_name};
            (*boid_center_data_stream_) << "boid_centers = [" << std::endl;
        }
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    void save_centers_to_file_end() const
    {
        if (save_boid_centers_)
        {
            (*boid_center_data_stream_) << "]" << std::endl;
            boid_center_data_stream_->close();
            delete boid_center_data_stream_;
        }
    }

    // Write Boid center positions to file.
    void save_centers_to_file_1_step() const
    {
        if (save_boid_centers_)
        {
            std::ofstream& output = *boid_center_data_stream_;
            output << "[";
            for (Boid* boid : boids())
            {
                Vec3 p = boid->position();
                output << "[";
                output << p.x() << "," << p.y() << "," << p.z();
                output << "],";
            }
            output << "]," << std::endl;
        }
    }

    //        def is_neighbor_of_selected(self, boid):
    //            return (self.enable_annotation and
    //                    self.tracking_camera and
    //                    self.selected_boid().is_neighbor(boid))
    
    // Fly each boid in flock for one simulation step. Consists of two sequential
    // steps to avoid artifacts from order of boids. First a "sense/plan" phase
    // which computes the desired steering based on current state. Then an "act"
    // phase which actually moves the boids. Finally statistics are collected.
    // (20240605 renamed  Flock::fly_flock() to Flock::fly_boids())
    void fly_boids(double time_step)
    {
        for_all_boids([&](Boid* b){ b->plan_next_steer();});
        for_all_boids([&](Boid* b){ b->apply_next_steer(time_step);});
        collect_flock_metrics();
    }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240623 get rid of all multithreading for GP testing
    
//    // Apply the given function to all Boids using two parallel threads.
//    //
//    // (At first this spun up N(=3) new threads to run in parallel. But that
//    //  provided almost no benefit. Probably the savings being canceled out by
//    //  thread launching overhead. Now creates ONE thread and splits the load
//    //  between it and this main thread. On 20240527 I tried adjusting the load
//    //  ratio between the threads. (Used util::Timer and averaged over all calls
//    //  in an evelotion run.) I got maybe 1-2% improvement but didn't think it
//    //  was worth the extra code complexity. Verified that two threads are
//    //  better than one.)
//    //
//    void for_all_boids(std::function<void(Boid* b)> boid_func)
//    {
//        // Apply "boid_func" to boids between indices "first" and "last"
//        auto chunk_func = [&](int first, int last)
//        {
//            int end = std::min(last, int(boids().size()));
//            for (int i = first; i < end; i++) { boid_func(boids().at(i)); }
//        };
//        int boid_count = int(boids().size());
//        int boids_per_thread = 1 + (boid_count * 0.5);
//        std::thread helper(chunk_func, 0, boids_per_thread);
//        chunk_func(boids_per_thread, boid_count);
//        helper.join();
//    };

    
    // Apply the given function to all Boids using two parallel threads.
    //
    // (At first this spun up N(=3) new threads to run in parallel. But that
    //  provided almost no benefit. Probably the savings being canceled out by
    //  thread launching overhead. Now creates ONE thread and splits the load
    //  between it and this main thread. On 20240527 I tried adjusting the load
    //  ratio between the threads. (Used util::Timer and averaged over all calls
    //  in an evelotion run.) I got maybe 1-2% improvement but didn't think it
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
        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240627 trying to reenable multi threading

//        if (Boid::GP_not_GA)
//        {
//            // TODO 20240625 when we switch back to multithreading, need to make
//            // sure new thread gets same Boid::getGpPerThread() as main thread.
//            chunk_func(0, boid_count);
//        }
//        else
//        {
//            std::thread helper(chunk_func, 0, boids_per_thread);
//            chunk_func(boids_per_thread, boid_count);
//            helper.join();
//        }

//        std::thread helper(chunk_func, 0, boids_per_thread);
//        chunk_func(boids_per_thread, boid_count);
//        helper.join();

        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
        // TODO 20240707 global switch to enable threads EF::enable_multithreading

//    //        bool multithreading = not Boid::GP_not_GA;
//            bool multithreading = true;
//            if (multithreading)

        if (EF::enable_multithreading)
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
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
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    };

    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    
    void collect_flock_metrics()
    {
        double ts = fp().min_speed - util::epsilon;
        for (Boid* b : boids()) { if (b->speed() < ts) { total_stalls_ += 1; } }
        bool all_speed_good = true;
        bool all_seperation_good = true;
        bool all_avoidance_good = true;
        double min_sep_allowed = 3;  // in units of body radius
        for (Boid* b : boids())
        {
            Boid* n = b->cached_nearest_neighbors().at(0);
            double  dist = (b->position() - n->position()).length();
            double br = fp().body_radius;
            bool nn_sep_ok = (dist / br) > min_sep_allowed;
            
            double cs = util::remap_interval_clip(dist/br,min_sep_allowed,20,1,0);
            sum_of_all_cohesion_scores_ += cs;
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20240710 change speed fitness back to speed-ok frame count.
            
//            bool speed_ok = b->speed() > 15;
            
            bool speed_ok = util::between(b->speed(),
                                          speed_target - speed_support_width / 2,
                                          speed_target + speed_support_width / 2);

            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            if (not nn_sep_ok) { all_seperation_good = false; }
            if (not speed_ok) { all_speed_good = false; }
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20240724 try GP using "per-boid avoid count"
            
//            if (b->detectObstacleViolations()) { all_avoidance_good = false; }

            //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
            // TODO 20240724 try ignoring fitness of boid after it hits obstacle.
            
//            if (b->detectObstacleViolations() or b->dead)
//            {
//                all_avoidance_good = false;
//                b->dead = true;
//            }
//            else
//            {
//                per_boid_avoid_count++;
//            }

            // TODO 20240728 retracted.
            
            if (b->detectObstacleViolations())
            {
                all_avoidance_good = false;
            }
            else
            {
                per_boid_avoid_count++;
            }

            //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // Add this boid's position to occupancy_map, ignore if outside ESO.
            occupancy_map.add(b->position(), [](Vec3 p){return p.length()>50;});
            if (not all_avoidance_good)
            {
                all_boids_avoid_obs_for_whole_chunk_ = false;
            }
            if (not nn_sep_ok) { all_separation_good_for_whole_chunk_ = false; }
            if (start_new_chunk())
            {
                if (all_boids_avoid_obs_for_whole_chunk_)
                {
                    count_chunked_avoid_obstacle_++;
                }
                all_boids_avoid_obs_for_whole_chunk_ = true;
                if (all_separation_good_for_whole_chunk_)
                {
                    count_chunked_separation_++;
                }
                all_separation_good_for_whole_chunk_ = true;
            }
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20240511 support path curvature metric.
            curvature_sum_for_all_boid_updates_ += b->getPathCurvature();
            
//            if (temp_max_curvature_ < b->getPathCurvature())
//            {
//                temp_max_curvature_ = b->getPathCurvature();
//                std::cout << "    ---- ";
//                debugPrint(temp_max_curvature_)
//            }
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20240706 make global variable GP::mof_names into a function.
            speed_score_sum_for_all_boid_updates_ += boid_speed_score(b);
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            increment_boid_update_counter();
        }
        if (all_speed_good) { count_steps_good_speed++; }
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240916 reset frame_counter, etc.
//        if (0 == (draw().frame_counter() % cluster_score_stride_))
        if (0 == (aTimer().frame_counter() % cluster_score_stride_))
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        {
            double cs = util::remap_interval_clip(count_clusters(), 0, 5, 0, 1);
            cluster_score_sum_ += cs;
            cluster_score_count_++;
        }
    }
    
    // Score for average path curvature for all boids on all steps.
    double curvature_sum_for_all_boid_updates_ = 0;
    double get_curvature_score() const
    {
        assert(boid_update_counter_ > 0);
        double ave = curvature_sum_for_all_boid_updates_ / boid_update_counter_;
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240625 in evoflock GP mode this was triggering

//        assert(!std::isnan(ave));

        if (std::isnan(ave)) { ave = 0; }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        double ad_hoc_max_curvature = 0.17;
        double ad_hoc_high_curvature = ad_hoc_max_curvature / 2;
        return util::clip01(ave / ad_hoc_high_curvature);
    }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240706 make global variable GP::mof_names into a function.
    double speed_score_sum_for_all_boid_updates_ = 0;
    double speed_target = 20; // m/s
    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    // TODO 20240710 change speed fitness back to speed-ok frame count.
//    double speed_support_width = 10;
    double speed_support_width = 6;  // That is +/- 3m/s is good enough.
    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    double boid_speed_score(Boid* boid)
    {
        double score = 0;
        double speed = boid->speed();
        double low  = speed_target - (speed_support_width / 2);
        double high = speed_target + (speed_support_width / 2);
        // Per boid score is zero outside a "speed_support_width" interval
        // centered on "speed_target". There the score is 1 and it falls
        // off to 0 (at "low" and "high") with a cosine kernel shape.
        if (util::between(speed, low, speed_target))
        {
            score = util::sinusoid(util::remap_interval(speed,
                                                        low, speed_target,
                                                        0, 1));
        }
        if (util::between(speed, speed_target, high))
        {
            
            score = util::sinusoid(util::remap_interval(speed,
                                                        speed_target, high,
                                                        1, 0));
        }
        return score;
    }
    double get_gp_speed_score() const
    {
        return speed_score_sum_for_all_boid_updates_ / boid_update_counter_;
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    int count_clusters() const
    {
        // Clustering parameters.
        int dbscan_min_points = 5;
        double dbscan_epsilon = 8;
        // Copy Boid positions into a vector of DBSCAN::Point.
        size_t boid_count = boids().size();
        std::vector<DBSCAN::Point> points(boid_count);
        for (int i = 0; i < boid_count; i++)
        {
            const Vec3& bp = boids()[i]->position();
            points[i].x = bp.x();
            points[i].y = bp.y();
            points[i].z = bp.z();
        }
        // Run DBSCAN clustering algorithm.
        DBSCAN dbscan(dbscan_min_points, dbscan_epsilon, points);
        // Return number of clusters found.
        return dbscan.getClusterCount();
    }
    
    double cluster_score_sum_ = 0;
    int cluster_score_count_ = 0;
    int cluster_score_stride_ = 10;
    double get_cluster_score() const
    {
        return cluster_score_sum_ / cluster_score_count_;
    }

    // Records if all obstacle avoidance is successful for whole chunk.
    // (Initialize to false so not to count chunk that ends on first update.)
    bool all_boids_avoid_obs_for_whole_chunk_ = false;
    int count_chunked_avoid_obstacle_ = 0;

    bool all_separation_good_for_whole_chunk_ = false;
    int count_chunked_separation_ = 0;
    int chunk_count_ = 200;

    // Count each boid update across all simulation steps.
    int boid_update_counter_ = 0;
    
    void increment_boid_update_counter() { boid_update_counter_++; }

    // For each boid update, returns true if this is the end of a fitness chunk.
    bool start_new_chunk()
    {
        int boid_update_max = max_simulation_steps() * boids().size();
        int chunk_steps = boid_update_max / chunk_count_;
        return 0 == (boid_update_counter_ % chunk_steps);
    }

    // TODO 20240329 Maybe these should be private with accessors
    int count_steps_good_separation = 0;
    int count_steps_good_speed = 0;
    // int count_steps_avoid_obstacle = 0;

    double get_separation_score() const
    {
        return count_chunked_separation_ / double(chunk_count_);
    }
    double get_speed_score() const
    {
        return count_steps_good_speed / double(aTimer().frame_counter());
    }
    double get_avoid_obstacle_score() const
    {
        return count_chunked_avoid_obstacle_ / double(chunk_count_);
    }
    double get_occupied_score() const
    {
        auto ignore_function = [](Vec3 p) { return p.length() > 50;};
        return occupancy_map.fractionOccupied(ignore_function);
    }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240724 try GP using "per-boid avoid count"
    double per_boid_avoid_count = 0;
    double get_per_boid_avoid_score() const
    {
        return per_boid_avoid_count / boid_update_counter_;
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    double sum_of_all_cohesion_scores_ = 0;
    double get_cohere_score() const
    {
        return sum_of_all_cohesion_scores_ / boid_update_counter_;
    }

    int log_stat_interval_ = 100;
    int getLogStatInterval() const { return log_stat_interval_; }
    void setLogStatInterval(int steps) { log_stat_interval_ = steps; }

    // Calculate and log various statistics for flock.
    void log_stats()
    {
        if ((not simulation_paused_) and
            (aTimer().frame_counter() % getLogStatInterval() == 0))
        {
            double average_speed = 0;
            for (Boid* b : boids()) { average_speed += b->speed(); }
            average_speed /= boid_count();
            
            // Loop over all unique pairs of distinct boids (ab==ba, not aa)
            double min_sep = std::numeric_limits<double>::infinity();
            double ave_sep = 0;
            int pair_count = 0;
            auto examine_pair = [&](const Boid* p, const Boid* q)
            {
                double dist = (p->position() - q->position()).length();
                if (min_sep > dist) { min_sep = dist; }
                ave_sep += dist;
                pair_count += 1;
                if (dist < (2 * fp().body_radius)){ cumulative_sep_fail_ += 1; }
            };
            util::apply_to_pairwise_combinations(examine_pair, boids());
            ave_sep /= pair_count;
            
            double max_nn_dist = 0;
            int total_avoid_fail = 0;
            for (Boid* b : boids())
            {
                Boid* n = b->cached_nearest_neighbors().at(0);
                double  dist = (b->position() - n->position()).length();
                if (max_nn_dist < dist) { max_nn_dist = dist; }
                total_avoid_fail += b->avoidance_failure_counter();
            }
            
            grabPrintLock_evoflock();
            std::cout << log_prefix;
            std::cout << aTimer().frame_counter();
            // std::cout << " fps=" << 0; // round(self.fps.value));
            std::cout << " fps=" << fps_.value;
            std::cout << ", ave_speed=" << average_speed;
            std::cout << ", min_sep=" << min_sep;
            std::cout << ", ave_sep=" << ave_sep;
            std::cout << ", max_nn_dist=" << max_nn_dist;
            std::cout << ", cumulative_sep_fail/boid="
                      << cumulative_sep_fail_ / float(boid_count());
            std::cout << ", avoid_fail=" << total_avoid_fail;
            std::cout << ", stalls=" << + total_stalls_;
            std::cout << std::endl;
        }
    }
    
    std::string log_prefix;

    // Keep track of a smoothed (LPF) version of frames per second metric.
    void update_fps()
    {
        double fd = aTimer().frame_duration();
        fps_.blend((fixed_time_step() ? fixed_fps() : int(1 / fd)), 0.95);
    }
    
    // Based on pause/play and single step. Called once per frame from main loop.
    bool run_simulation_this_frame()
    {
        bool ok_to_run = single_step_ or not simulation_paused_;
        single_step_ = false;
        return ok_to_run;
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241004 follow cam

    //        # Returns currently selected boid, the one that the tracking camera
    //        # tracks, for which steering force annotation is shown.
    //        def selected_boid(self):
    //            return self.boids[self.selected_boid_index]

    Boid* selectedBoid() { return boids().front(); }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    
    //        # Register single key commands with the Open3D visualizer GUI.
    //        def register_single_key_commands(self):
    //            Draw.register_key_callback(ord(' '), Flock.toggle_paused_mode)
    //            Draw.register_key_callback(ord('1'), Flock.set_single_step_mode)
    //            Draw.register_key_callback(ord('S'), Flock.select_next_boid)
    //            Draw.register_key_callback(ord('A'), Flock.toggle_annotation)
    //            Draw.register_key_callback(ord('C'), Flock.toggle_tracking_camera)
    //            Draw.register_key_callback(ord('W'), Flock.toggle_wrap_vs_avoid)
    //            Draw.register_key_callback(ord('E'), Flock.toggle_dynamic_erase)
    //            Draw.register_key_callback(ord('F'), Flock.toggle_fixed_time_step)
    //            Draw.register_key_callback(ord('B'), Flock.toggle_avoid_blend_mode)
    //            Draw.register_key_callback(ord('O'), Flock.cycle_obstacle_selection)
    //            Draw.register_key_callback(ord('H'), Flock.print_help)
    //
    //        # Toggle simulation pause mode.
    //        def toggle_paused_mode(self):
    //            self = Flock.convert_to_flock(self)
    //            if self.simulation_paused:
    //                Draw.reset_timer()
    //            self.simulation_paused = not self.simulation_paused
    //
    //        # Take single simulation step then enter pause mode.
    //        def set_single_step_mode(self):
    //            self = Flock.convert_to_flock(self)
    //            self.single_step = True
    //            self.simulation_paused = True
    //
    //        # Select the "next" boid. This gets bound to the "s" key in the interactive
    //        # visualizer. So typing s s s will cycle through the boids of a flock.
    //        def select_next_boid(self):
    //            self = Flock.convert_to_flock(self)
    //            self.selected_boid_index = ((self.selected_boid_index + 1) %
    //                                        len(self.boids))
    //            self.single_step_if_paused()
    //
    //        # Toggle drawing of annotation (lines to represent vectors) in the GUI.
    //        def toggle_annotation(self):
    //            self = Flock.convert_to_flock(self)
    //            self.enable_annotation = not self.enable_annotation
    //
    //        # Toggle between static camera and boid-tracking camera mode.
    //        def toggle_tracking_camera(self):
    //            self = Flock.convert_to_flock(self)
    //            self.tracking_camera = not self.tracking_camera
    //            self.single_step_if_paused()
    //
    //        # Toggle mode for sphere-wrap-around versus sphere-avoidance.
    //        def toggle_wrap_vs_avoid(self):
    //            self = Flock.convert_to_flock(self)
    //            self.wrap_vs_avoid = not self.wrap_vs_avoid
    //
    //        # Toggle mode for erasing dynamic graphics ("spacetime boid worms").
    //        def toggle_dynamic_erase(self):
    //            self = Flock.convert_to_flock(self)
    //            Draw.clear_dynamic_mesh = not Draw.clear_dynamic_mesh
    //            if self.tracking_camera and not Draw.clear_dynamic_mesh:
    //                print('!!! "spacetime boid worms" do not work correctly with ' +
    //                      'boid tracking camera mode ("C" key). Awaiting fix for ' +
    //                      'Open3D bug 6009.')
    //
    //        # Toggle between realtime/as-fast-as-possible versus fixed time step of 1/60
    //        def toggle_fixed_time_step(self):
    //            self = Flock.convert_to_flock(self)
    //            self.fixed_time_step = not self.fixed_time_step
    //
    //        # Toggle between blend/hard-switch for obstacle avoidance.
    //        def toggle_avoid_blend_mode(self):
    //            self = Flock.convert_to_flock(self)
    //            self.avoid_blend_mode = not self.avoid_blend_mode
    //            print('    Flock.avoid_blend_mode =', self.avoid_blend_mode)
    //
    //        # Cycle through various pre-defined combinations of Obstacle types.
    //        def cycle_obstacle_selection(self):
    //            self = Flock.convert_to_flock(self)
    //            # Remove geometry of current Obstacles from Open3D scene.
    //            if Draw.enable:
    //                for o in self.obstacles:
    //                    Draw.vis.remove_geometry(o.tri_mesh, False)
    //            # Set Obstacle list to next preset combination.
    //            next_set = self.obstacle_selection_counter % len(self.obstacle_presets)
    //            self.obstacle_selection_counter += 1
    //            self.obstacles = self.obstacle_presets[next_set]
    //            # Add geometry of current obstacles to Open3D scene.
    //            if Draw.enable:
    //                for o in self.obstacles:
    //                    o.draw()
    //                    if o.tri_mesh: # TODO only need until all Obstacles draw themselves.
    //                        Draw.vis.add_geometry(o.tri_mesh, False)
    //            # Print description of current Obstacle set.
    //            description = '\n  obstacles: '
    //            if self.obstacles:
    //                seperator = ''
    //                for o in self.obstacles:
    //                    description += seperator + str(o)
    //                    seperator = ', '
    //            else:
    //                description += 'none'
    //            print(description + '\n')

    
        
    void pre_defined_obstacle_sets()
    {
        obstacles().push_back(new EvertedSphereObstacle(fp().sphere_radius,
                                                        fp().sphere_center,
                                                        Obstacle::outside));
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240812 change from 1 to 6 cylinders

//        // TODO 20240218 experiments for "sim starts in more flock-like state"
//        double ecr = fp().sphere_radius;
//        Vec3 ect = fp().sphere_center + Vec3(ecr * 0.6, ecr, 0);
//        Vec3 ecb = fp().sphere_center + Vec3(ecr * 0.6, -ecr, 0);
//        obstacles().push_back(new CylinderObstacle(ecr * 0.2, ect, ecb,
//                                                   Obstacle::inside));


//        // 6 symmetric cylinders on main axes.
//        double c3r = fp().sphere_radius *  4 / 30;
//        double c3o = fp().sphere_radius * 15 / 30;
//        double c3h = fp().sphere_radius * 20 / 30;
//        auto add_cyl = [&](double r, Vec3 t, Vec3 b)
//        {
//            obstacles().push_back(new CylinderObstacle(r, t, b));
//        };
//        add_cyl(c3r, Vec3(-c3h, 0, c3o), Vec3(c3h, 0, c3o));
//        add_cyl(c3r, Vec3(c3o, -c3h, 0), Vec3(c3o, c3h, 0));
//        add_cyl(c3r, Vec3(0, c3o, -c3h), Vec3(0, c3o, c3h));
//        c3o = - c3o;
//        add_cyl(c3r, Vec3(-c3h, 0, c3o), Vec3(c3h, 0, c3o));
//        add_cyl(c3r, Vec3(c3o, -c3h, 0), Vec3(c3o, c3h, 0));
//        add_cyl(c3r, Vec3(0, c3o, -c3h), Vec3(0, c3o, c3h));

//        // 6 symmetric cylinders on main axes.
//        double c3r = fp().sphere_radius *  4 / 30;
//        double c3o = fp().sphere_radius * 15 / 30;
//        double c3h = fp().sphere_radius * 20 / 30;
//        auto add_3_cyl = [&]()
//        {
//            auto add_cyl = [&](double r, Vec3 t, Vec3 b)
//                { obstacles().push_back(new CylinderObstacle(r, t, b)); };
//            add_cyl(c3r, Vec3(-c3h, 0, c3o), Vec3(c3h, 0, c3o));
//            add_cyl(c3r, Vec3(c3o, -c3h, 0), Vec3(c3o, c3h, 0));
//            add_cyl(c3r, Vec3(0, c3o, -c3h), Vec3(0, c3o, c3h));
//        };
//        add_3_cyl();
//        c3o = - c3o;
//        add_3_cyl();

        // 6 symmetric cylinders parallel to main axes.
        double c6r = fp().sphere_radius *  4 / 30;
        double c6o = fp().sphere_radius * 15 / 30;
        double c6h = fp().sphere_radius * 20 / 30;
        auto add_3_cyl = [&](double c6o)
        {
            auto add_cyl = [&](double r, Vec3 t, Vec3 b)
                { obstacles().push_back(new CylinderObstacle(r, t, b)); };
            add_cyl(c6r, Vec3(-c6h, 0, c6o), Vec3(c6h, 0, c6o));
            add_cyl(c6r, Vec3(c6o, -c6h, 0), Vec3(c6o, c6h, 0));
            add_cyl(c6r, Vec3(0, c6o, -c6h), Vec3(0, c6o, c6h));
        };
        add_3_cyl(c6o);
        add_3_cyl(-c6o);

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        //# Builds a list of preset obstacle combinations, each a list of Obstacles.
        //def pre_defined_obstacle_sets(self):
        //    main_sphere = EvertedSphereObstacle(self.sphere_radius, self.sphere_center)
        //    plane_obstacle = PlaneObstacle()
        //
        //    cep = Vec3(0, self.sphere_radius + 0.1, 0)
        //    cobs = CylinderObstacle(10, cep, -cep)
        //
        //    scep = Vec3(-1, 1, 1) * self.sphere_radius * 0.8
        //    squat_cyl_obs = CylinderObstacle(20, scep, -scep)
        //
        //    diag = Vec3(1,1,1).normalize()
        //    uncentered_cyl_obs = CylinderObstacle(5, diag * 5, diag * 30)
        //
        //    # 6 symmetric cylinders on main axes.
        //    c3r =  4 / 30 * self.sphere_radius
        //    c3o = 15 / 30 * self.sphere_radius
        //    c3h = 20 / 30 * self.sphere_radius
        //    cyl3x = CylinderObstacle(c3r, Vec3(-c3h, 0, c3o), Vec3(c3h, 0, c3o))
        //    cyl3y = CylinderObstacle(c3r, Vec3(c3o, -c3h, 0), Vec3(c3o, c3h, 0))
        //    cyl3z = CylinderObstacle(c3r, Vec3(0, c3o, -c3h), Vec3(0, c3o, c3h))
        //    c3o = - c3o
        //    cyl3p = CylinderObstacle(c3r, Vec3(-c3h, 0, c3o), Vec3(c3h, 0, c3o))
        //    cyl3q = CylinderObstacle(c3r, Vec3(c3o, -c3h, 0), Vec3(c3o, c3h, 0))
        //    cyl3r = CylinderObstacle(c3r, Vec3(0, c3o, -c3h), Vec3(0, c3o, c3h))
        //
        //    # Preset obstacle combinations:
        //    return [[main_sphere],
        //            [main_sphere, plane_obstacle],
        //            [main_sphere, uncentered_cyl_obs],
        //            [main_sphere, cyl3x, cyl3y, cyl3z, cyl3p, cyl3q, cyl3r],
        //            [main_sphere, cobs],
        //            [main_sphere, plane_obstacle, cobs],
        //            [cobs],
        //            [squat_cyl_obs],
        //            [squat_cyl_obs, main_sphere],
        //            [squat_cyl_obs, plane_obstacle],
        //            []]

    }
    
    //
    //        # Print mini-help on shell.
    //        def print_help(self):
    //            self = Flock.convert_to_flock(self)
    //            print()
    //            print('  flock single key commands:')
    //            print('    space: toggle simulation run/pause')
    //            print('    1:     single simulation step, then pause')
    //            print('    c:     toggle camera between static and boid tracking')
    //            print('    s:     select next boid for camera tracking')
    //            print('    a:     toggle drawing of steering annotation')
    //            print('    w:     toggle between sphere wrap-around or avoidance')
    //            print('    e:     toggle erase mode (spacetime boid worms)')
    //            print('    f:     toggle realtime versus fixed time step of 1/60sec')
    //            print('    b:     toggle blend vs hard switch for obstacle avoidance')
    //            print('    o:     cycle through obstacle selections')
    //            print('    h:     print this message')
    //            print('    esc:   exit simulation.')
    //            print()
    //            print('  mouse view controls:')
    //            print('    Left button + drag         : Rotate.')
    //            print('    Ctrl + left button + drag  : Translate.')
    //            print('    Wheel button + drag        : Translate.')
    //            print('    Shift + left button + drag : Roll.')
    //            print('    Wheel                      : Zoom in/out.')
    //            print()
    //            print('  annotation (in camera tracking mode, “c” to toggle):')
    //            print('    red:     separation force.')
    //            print('    green:   alignment force.')
    //            print('    blue:    cohesion force.')
    //            print('    gray:    combined steering force.')
    //            print('    magenta: ray for obstacle avoidance.')
    //            print()
    //
    //        # Allows writing global key command handlers as methods on a Flock instance.
    //        # If the given "self" value is not a Flock instance, this assumes it it a
    //        # Visualizer instance (passed to key handlers) and looks up the Flock
    //        # instance associated with that Vis. (Normally only one of each exists.)
    //        vis_pairs = util.Pairings()
    //        def convert_to_flock(self):
    //            if not isinstance(self, Flock):
    //                self = Flock.vis_pairs.get_peer(self)
    //            return self
    //
    //        # Some mode-change key commands need a redraw to show their effect. Ideally
    //        # it would do just a redraw without simulation, but that turned out to be a
    //        # little more complicated (since Boid annotation state is not saved) so this
    //        # is good enough.
    //        def single_step_if_paused(self):
    //            if self.simulation_paused:
    //                self.set_single_step_mode()
    
    // Simulation continues running until this returns false.
    bool still_running()
    {
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240919 trying to handle commands with register_key_callback
//        bool a = (not draw().enable()) ? true : draw().poll_events();
        bool a = (not draw().enable()) ? true : draw().pollEvents();
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        bool b = aTimer().frame_counter() < max_simulation_steps();
        return a and b;
    }
    
    
    //        # Perform "before simulation" tasks: log versions, run unit tests.
    //        def setup(self):
    //            # Log versions.
    //            print('Python', sys.version)
    //            print('Open3D', o3d.__version__)
    //            # Run unit tests.
    //            Vec3.unit_test()
    //            LocalSpace.unit_test()
    //            Agent.unit_test()
    //            util.unit_test()
    //            shape.unit_test()
    //            print('All unit tests OK.')
    //
    //
    //    if __name__ == "__main__":
    //
    //        # TODO 20230530 runs OK (if slow) but something wrong in center offset case
    //    #    Flock(400).run()                       # OK
    //    #    Flock(400, 100).run()                  # OK
    //    #    Flock(400, 100, Vec3(100, 0, 0)).run() # containment sphere still at origin
    //    #
    //    #    Flock(200, 60, Vec3(), 200).run()      # Test max_simulation_steps
    //    #    Flock(max_simulation_steps=200, fixed_time_step=True, fixed_fps=30).run()
    //    #    Flock(max_simulation_steps=200, fixed_time_step=True, seed=438538457).run()
    //
    //        ############################################################################
    //
    //    #    util.executions_per_second(Vec3.unit_test)
    //
    //        ############################################################################
    //
    //        Flock().run()
    
    
    
    static void unit_test()
    {
        Flock f;
    }
};
