//-------------------------------------------------------------------------------
//
//  Draw.h -- new flock experiments
//
//  Graphics utilities for evoflock based on Open3D.
//  An instance of Draw provides the (optional) graphics context and utilities.
//
//  Created by Craig Reynolds on September 9, 2024.
//  MIT License -- Copyright © 2024 Craig Reynolds
//-------------------------------------------------------------------------------

#pragma once

#ifdef USE_OPEN3D
#include "open3d/Open3D.h"
#endif // USE_OPEN3D

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
    
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240909 add Draw.h
    
    std::vector<Vec3> scene_line_endpoints_;
    std::vector<std::size_t> scene_line_segments_;
    std::vector<Vec3> scene_line_colors_;
    
    void clearLineSegmentsFromScene()
    {
        scene_line_endpoints_.clear();
        scene_line_segments_.clear();
        scene_line_colors_.clear();
    }
    
    void addLineSegmentToScene(const Vec3& endpoint0,
                               const Vec3& endpoint1,
                               const Vec3& color)
    {
        scene_line_endpoints_.push_back(endpoint0);
        scene_line_segments_.push_back(scene_line_endpoints_.size());
        scene_line_endpoints_.push_back(endpoint1);
        scene_line_segments_.push_back(scene_line_endpoints_.size());
        scene_line_colors_.push_back(color);
    }
    
#ifdef USE_OPEN3D
    std::shared_ptr<open3d::geometry::LineSet> makeLineSetForScene()
    {
        auto ls = std::make_shared<open3d::geometry::LineSet>();
        for (auto& e : scene_line_endpoints_)
        {
            ls->points_.push_back({e.x(), e.y(), e.z()});
        }
        for (int s = 0; s < scene_line_segments_.size(); s += 2)
        {
            ls->lines_.push_back({s, s + 1});
        }
        for (auto& c : scene_line_colors_)
        {
            ls->colors_.push_back({c.x(), c.y(), c.z()});
        }
        return ls;
    }
#endif // USE_OPEN3D

    // Just for debugging and testing. Run Open3D tests and exit from app.
    void visualizeEvoflockFitnessTest()
    {
#ifdef USE_OPEN3D

        //    LineWidthPointSizeTest();
        //    return;
        
        double sphere_diameter = 100;
        double sphere_radius = sphere_diameter / 2;

        auto sphere = open3d::geometry::TriangleMesh::CreateSphere(sphere_radius);
        sphere->ComputeVertexNormals();
        sphere->PaintUniformColor({0.5, 0.5, 0.5});
        
        addLineSegmentToScene({ 0, 60, 0}, {0, -60, 0}, {1, 1, 0});
        addLineSegmentToScene({-5,  0, 0}, {5,   0, 0}, {0, 1, 1});
        
        auto& rs = EF::RS();
        Vec3 a;
        Vec3 b;
        for (int i = 0; i < 1000; i++)
        {
            a = b;
            b = b + rs.random_unit_vector() * 0.5;
            auto c = rs.random_point_in_axis_aligned_box(Vec3(), Vec3(1, 1, 1));
            addLineSegmentToScene(a, b, c);
        }
        
        auto vis = open3d::visualization::Visualizer();
        int window_size = 2000;
        vis.CreateVisualizerWindow("evoflock", window_size, window_size, 0, 0);
        vis.AddGeometry(sphere);
        vis.AddGeometry(makeLineSetForScene());
        vis.GetRenderOption().line_width_ = 10.0;
        vis.GetRenderOption().point_size_ = 20.0;
        vis.GetRenderOption().mesh_show_back_face_ = true;
        vis.Run();

        exit(EXIT_SUCCESS);
        
#endif // USE_OPEN3D
    }

    
    
//    void LineWidthPointSizeTest()
//    {
//        open3d::PrintOpen3DVersion();
//        auto lineset = std::make_shared<open3d::geometry::LineSet>();
//        lineset->points_ = {{5, 0, 0}, {-5, 0, 0}, {0, 5, 0}, {0, -5, 0}, {0, 0, 5}, {0, 0, -5}};
//        lineset->lines_ = {{0, 1}, {2, 3}, {4, 5}};
//        lineset->colors_ = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
//        
//        auto vis = open3d::visualization::Visualizer();
//        vis.CreateVisualizerWindow();
//        vis.AddGeometry(lineset);
//        vis.GetRenderOption().line_width_ = 10.0;
//        vis.GetRenderOption().point_size_ = 20.0;
//        vis.Run();
//    }

    

    static void unit_test() {}

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    
    util::TimePoint frame_start_time;
    double frame_duration_ = 0; // measured in seconds
    int frame_counter_ = 0;
};
