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
    // TODO 20240910 add animated TriangleMesh support.
    //
    // There is (eg) TriangleMesh::Clear(), maybe better to do that than make a
    // new TriangleMesh each frame? It may be equivalent, but in case Open3D
    // does something fancy to reduce the GPU overhead?
    //
    // vertex_colors_ from MeshBase
    
    // Clear per-frame collection of animating line segments for annotation.
    void clearTrianglesFromScene()
    {
        scene_triangles_.clear();
        scene_triangle_vertices_.clear();
        scene_triangle_vertex_colors_.clear();
    }
        
    // Add to per-frame collection of animating triangles: single color.
    // Typically used to add a small polyhedron (a boid's body) to the scene.
    void addTrianglesToScene(const std::vector<Vec3>& vertices,
                             const std::vector<std::size_t>& triangles,
                             const Vec3& color)
    {
        std::vector<Vec3> colors(vertices.size(), color);
        addTrianglesToScene(vertices, triangles, colors);
    }
    
    // Add to per-frame collection of animating triangles: per-vertex colors.
    void addTrianglesToScene(const std::vector<Vec3>& vertices,
                             const std::vector<std::size_t>& triangles,
                             const std::vector<Vec3>& colors)
    {
        assert(triangles.size() % 3 == 0);
        auto ovc = scene_triangle_vertices_.size(); // old_vertex_count
        for (auto& v : vertices) { scene_triangle_vertices_.push_back(v); }
        for (auto& t : triangles) { scene_triangles_.push_back(t + ovc); }
        for (auto& c : colors) { scene_triangle_vertex_colors_.push_back(c); }
    }

    // TODO move to bottom of class later
    // Per frame collection of animating triangles.
    std::vector<std::size_t> scene_triangles_;
    std::vector<Vec3> scene_triangle_vertices_;
    std::vector<Vec3> scene_triangle_vertex_colors_;
    
#ifdef USE_OPEN3D
    // Add to per-frame collection of animating line segments for annotation.
    std::shared_ptr<open3d::geometry::TriangleMesh> makeTriangleMeshForScene()
    {
        assert(scene_triangles_.size() % 3 == 0);
        auto tm = std::make_shared<open3d::geometry::TriangleMesh>();
        for (auto& v : scene_triangle_vertices_)
        {
            tm->vertices_.push_back({v.x(), v.y(), v.z()});
        }
        for (int t = 0; t < scene_triangles_.size(); t += 3)
        {
            tm->triangles_.push_back({int(scene_triangles_.at(t)),
                                      int(scene_triangles_.at(t + 1)),
                                      int(scene_triangles_.at(t + 2))});
        }
        for (auto& c : scene_triangle_vertex_colors_)
        {
            tm->vertex_colors_.push_back({c.x(), c.y(), c.z()});
        }
        tm->ComputeVertexNormals();
        return tm;
    }
#endif // USE_OPEN3D

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Clear per-frame collection of animating line segments for annotation.
    void clearLineSegmentsFromScene()
    {
        scene_line_endpoints_.clear();
        scene_line_segments_.clear();
        scene_line_colors_.clear();
    }
    
    // Add to per-frame collection of animating line segments for annotation.
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
    // Add to per-frame collection of animating line segments for annotation.
    std::shared_ptr<open3d::geometry::LineSet> makeLineSetForScene()
    {
        assert(scene_line_segments_.size() % 2 == 0);
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

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240910 add animated TriangleMesh support.
        addTrianglesToScene({{0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}},  // vertices
                            {1,2,3, 3,2,0, 0,1,3, 2,1,0},          // triangles
                            {{1,1,1}, {1,0,0}, {0,1,0}, {0,0,1}}); // colors
        
        addTrianglesToScene({{0,0,0}, {-1,0,0}, {0,-1,0}, {0,0,-1}},// vertices
                            {3,2,1, 0,2,3, 3,1,0, 0,1,2},           // triangles
                            {0.3,0.3,0.3});                         // colors

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        auto vis = open3d::visualization::Visualizer();
        int window_size = 2000;
        vis.CreateVisualizerWindow("evoflock", window_size, window_size, 0, 0);
        vis.AddGeometry(sphere);
        vis.AddGeometry(makeLineSetForScene());
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240910 add animated TriangleMesh support.
        vis.AddGeometry(makeTriangleMeshForScene());
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        vis.GetRenderOption().line_width_ = 10.0;
        vis.GetRenderOption().point_size_ = 20.0;
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240910 add animated TriangleMesh support.
//        vis.GetRenderOption().mesh_show_back_face_ = true;
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        vis.Run();
        
        exit(EXIT_SUCCESS);
        
#endif // USE_OPEN3D
    }
    
    
    // Example code from https://github.com/isl-org/Open3D/issues/6952
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
    
    util::TimePoint frame_start_time;
    double frame_duration_ = 0; // measured in seconds
    int frame_counter_ = 0;
    
    // Per frame collection of animating line segments for annotation.
    std::vector<Vec3> scene_line_endpoints_;
    std::vector<std::size_t> scene_line_segments_;
    std::vector<Vec3> scene_line_colors_;
};
