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
#endif  // USE_OPEN3D

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240911 try drawing boid body
// temp for testing
#include "LocalSpace.h"
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240917 Prototype of animation timer to be allocated per flock.
//
//               Would it make any sense to integrate this with util::Timer?

class AnimationTimer
{
public:
    AnimationTimer() : frame_start_time_(util::TimeClock::now()) {}
    
    double frame_duration() const { return frame_duration_; }
    bool poll_events() const { return true; }
    int frame_counter() const { return frame_counter_; }
    
    // Measure how much wall clock time has elapsed for this simulation step.
    void measure_frame_duration()
    {
        util::TimePoint frame_end_time = util::TimeClock::now();
        frame_duration_ = util::time_diff_in_seconds(frame_end_time,
                                                     frame_start_time_);
        frame_start_time_ = frame_end_time;
        frame_counter_ += 1;
    }

private:
    util::TimePoint frame_start_time_;
    double frame_duration_ = 0; // measured in seconds
    int frame_counter_ = 0;
};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


class Draw
{
public:

    Draw()
    {
        std::cout << "Begin graphics session using: ";
        open3d::PrintOpen3DVersion();
        
        assert(globalDrawObjectTemp == nullptr);
        globalDrawObjectTemp = this;

#ifdef USE_OPEN3D
        // Allocate TriangleMesh/LineSet objects which hold animated geometry.
        animated_tri_mesh_ = std::make_shared<open3d::geometry::TriangleMesh>();
        animated_line_set_ = std::make_shared<open3d::geometry::LineSet>();
        
//        debugPrint(visualizer_);
        visualizer_ = std::make_shared<open3d::visualization::Visualizer>();
        
        int window_size = 2000;
        visualizer_->CreateVisualizerWindow("evoflock",
                                            window_size, window_size,
                                            0, 0);
//        debugPrint(visualizer_);
//        debugPrint(visualizer_->GetRenderOption().line_width_);
        visualizer_->GetRenderOption().line_width_ = 10.0;
        visualizer_->GetRenderOption().point_size_ = 20.0;
        tempAddSphere();
#endif  // USE_OPEN3D
    }

    ~Draw()
    {
        globalDrawObjectTemp = nullptr;
        std::cout << "End graphics session using. Total triangles drawn: ";
        std::cout << triangle_count_ << "." << std::endl;
    }

    // TODO 20240912 make into parameters: win size, win pos, win title,
    //     line_width_, point_size_, plus all the static geometry (obstacles)
    void beginAnimatedDisplay()
    {
//        frame_counter_ = 0;
//        frame_duration_ = 0;
//        // TODO 20240915 this is wrong but I'm not sure what is right:
//        frame_start_time = util::TimePoint();
    }

    void endAnimatedDisplay()
    {
    }
    
    void beginOneAnimatedFrame()
    {
        clearAnimatedGeometryFromScene();
    }
    
    void endOneAnimatedFrame()
    {
#ifdef USE_OPEN3D
        animated_tri_mesh_->ComputeVertexNormals();
        visualizer_->UpdateGeometry(animated_tri_mesh_);
        visualizer_->UpdateGeometry(animated_line_set_);
#endif  // USE_OPEN3D
    }
    

    // Clear all animated geometry to begin building a new scene.
    void clearAnimatedGeometryFromScene()
    {
#ifdef USE_OPEN3D
        animated_tri_mesh_->Clear();
        animated_line_set_->Clear();
#endif  // USE_OPEN3D
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
#ifdef USE_OPEN3D
        assert(triangles.size() % 3 == 0);
        assert(vertices.size() == colors.size());
        auto ovc = animated_tri_mesh_->vertices_.size(); // old_vertex_count
        for (auto& v : vertices)
        {
            animated_tri_mesh_->vertices_.push_back({v.x(), v.y(), v.z()});
        }
        for (int t = 0; t < triangles.size(); t += 3)
        {
            Eigen::Vector3i new_tri {int(ovc + triangles.at(t)),
                int(ovc + triangles.at(t + 1)),
                int(ovc + triangles.at(t + 2))};
            animated_tri_mesh_->triangles_.push_back(new_tri);
        }
        for (auto& c : colors)
        {
            animated_tri_mesh_->vertex_colors_.push_back({c.x(), c.y(), c.z()});
        }
        // Count all triangles drawn during graphics session.
        triangle_count_ += triangles.size() / 3;
#endif  // USE_OPEN3D
    }
    
    // Add to per-frame collection of animating line segments for annotation.
    void addLineSegmentToScene(const Vec3& endpoint0,
                               const Vec3& endpoint1,
                               const Vec3& color)
    {
#ifdef USE_OPEN3D
        auto s = animated_line_set_->points_.size();
        auto ev3d=[](const Vec3& v){return Eigen::Vector3d(v.x(),v.y(),v.z());};
        animated_line_set_->points_.push_back(ev3d(endpoint0));
        animated_line_set_->points_.push_back(ev3d(endpoint1));
        animated_line_set_->lines_.push_back({s, s + 1});
        animated_line_set_->colors_.push_back(ev3d(color));
#endif  // USE_OPEN3D
    }
    
    void tempAddSphere()
    {
#ifdef USE_OPEN3D
        double sphere_diameter = 100;
        double sphere_radius = sphere_diameter / 2;

        auto sphere = open3d::geometry::TriangleMesh::CreateSphere(sphere_radius);
        evertTriangleMesh(*sphere);
        sphere->ComputeVertexNormals();
        sphere->PaintUniformColor({0.5, 0.5, 0.5});
        visualizer_->AddGeometry(sphere);
#endif  // USE_OPEN3D
    }

    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240911 try evert TriangleMesh.
#ifdef USE_OPEN3D

    // Flip orientation of each tri in a triangle mesh (destructively modifies)
    // (pr to add to Open3D? https://github.com/isl-org/Open3D/discussions/6419)
    void evertTriangleMesh(open3d::geometry::TriangleMesh& tri_mesh)
    {
        for (auto& tmt : tri_mesh.triangles_)
        {
            std::reverse(std::begin(tmt), std::end(tmt));
        }
    }
#endif  // USE_OPEN3D

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240911 try drawing boid body
    // for testing
    
    void drawBoidBody(Vec3 position, Vec3 side, Vec3 up, Vec3 forward,
                      double body_radius, Vec3 color)
    {
        double bd = body_radius * 2;  // body diameter (defaults to 1)
        Vec3 center = position;
        Vec3 nose = center + forward * body_radius;
        Vec3 tail = center - forward * body_radius;
        Vec3 apex = tail + (up * 0.25 * bd) + (forward * 0.1 * bd);
        Vec3 wingtip0 = tail + (side * 0.3 * bd);
        Vec3 wingtip1 = tail - (side * 0.3 * bd);
        addTrianglesToScene({nose, apex, wingtip0, wingtip1},  // vertices
                            {1,2,3, 3,2,0, 0,1,3, 2,1,0},      // triangles
                            color);                            // color
    }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240913 WIP draw during fitness tests.
    // VERY TEMP JUST FOR PROTOTYPING
    static inline Draw* globalDrawObjectTemp = nullptr;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    
    // Just for debugging and testing. Run Open3D tests.
    static void visualizeEvoflockFitnessTest()
    {
#ifdef USE_OPEN3D

//        globalDrawObjectTemp = new Draw();
        Draw draw;
//        globalDrawObjectTemp = &draw;

        
        globalDrawObjectTemp->clearAnimatedGeometryFromScene();
        globalDrawObjectTemp->tempAddSphere();
        globalDrawObjectTemp->addLineSegmentToScene({ 0, 60, 0}, {0, -60, 0}, {1, 1, 0});
        globalDrawObjectTemp->addLineSegmentToScene({-5,  0, 0}, {5,   0, 0}, {0, 1, 1});

        auto& rs = EF::RS();
        Vec3 a;
        Vec3 b;
        for (int i = 0; i < 1000; i++)
        {
            a = b;
            b = b + rs.random_unit_vector() * 0.5;
            auto c = rs.random_point_in_axis_aligned_box(Vec3(), Vec3(1, 1, 1));
            globalDrawObjectTemp->addLineSegmentToScene(a, b, c);
        }

        globalDrawObjectTemp->
        addTrianglesToScene({{0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}},  // vertices
                            {1,2,3, 3,2,0, 0,1,3, 2,1,0},          // triangles
                            {{1,1,1}, {1,0,0}, {0,1,0}, {0,0,1}}); // colors
        
        globalDrawObjectTemp->
        addTrianglesToScene({{0,0,0}, {-1,0,0}, {0,-1,0}, {0,0,-1}},// vertices
                            {3,2,1, 0,2,3, 3,1,0, 0,1,2},           // triangles
                            {0.3,0.3,0.3});                         // color
        
        globalDrawObjectTemp->
        drawBoidBody({3, 3, 0}, {1,0,0}, {0,1,0}, {0,0,1}, 0.5, {1,1,0});

        for (int i = 0; i < 100; i++)
        {
            Vec3 p = rs.randomPointInUnitRadiusSphere() * 30;
            auto ls = LocalSpace().randomize_orientation();
            auto c = rs.random_point_in_axis_aligned_box(Vec3(0.4, 0.4, 0.4),
                                                         Vec3(1.0, 1.0, 1.0));
            globalDrawObjectTemp->drawBoidBody(p, ls.i(), ls.j(), ls.k(), 0.5, c);
        }

        globalDrawObjectTemp->animated_tri_mesh_->ComputeVertexNormals();
        globalDrawObjectTemp->visualizer_->AddGeometry(globalDrawObjectTemp->animated_tri_mesh_);
        globalDrawObjectTemp->visualizer_->AddGeometry(globalDrawObjectTemp->animated_line_set_);
        globalDrawObjectTemp->visualizer_->Run();

#endif  // USE_OPEN3D
    }

    bool enable() { return false; }
    bool poll_events() const { return true; }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240917 Prototype of animation timer to be allocated per flock.
    //               Would it make any sense to integrate this with util::Timer?

//    double frame_duration() const { return frame_duration_; }
//    int frame_counter() const { return frame_counter_; }
//    
//    // Measure how much wall clock time has elapsed for this simulation step.
//    void measure_frame_duration()
//    {
//        util::TimePoint frame_end_time = util::TimeClock::now();
//        frame_duration_ = util::time_diff_in_seconds(frame_end_time,
//                                                     frame_start_time);
//        frame_start_time = frame_end_time;
//        frame_counter_ += 1;
//    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240917 Prototype of animation timer to be allocated per flock.
    //               Would it make any sense to integrate this with util::Timer?

//    util::TimePoint frame_start_time;
//    double frame_duration_ = 0; // measured in seconds
//    int frame_counter_ = 0;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

private:
#ifdef USE_OPEN3D

    // Open3D TriangleMesh object for storing and drawing animated triangles.
    std::shared_ptr<open3d::geometry::TriangleMesh> animated_tri_mesh_ = nullptr;
    
    // Open3D LineSet object for storing and drawing animated line segments.
    std::shared_ptr<open3d::geometry::LineSet> animated_line_set_ = nullptr;
    
    // Retain pointer to Open3D Visualizer object.
    std::shared_ptr<open3d::visualization::Visualizer> visualizer_ = nullptr;
    
    // Count all triangles drawn during graphics session.
    int triangle_count_ = 0;

#endif  // USE_OPEN3D
};
