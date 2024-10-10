//-------------------------------------------------------------------------------
//
//  Draw.h -- new flock experiments
//
//  Graphics utilities for evoflock based on Open3D.
//  An instance of Draw provides the (optional) graphics context and utilities.
//  Draw is a "singleton" class, for which only one instance exists at any time.
//  Creating a Draw object creates an Open3D visualizer object, an associated
//  window, and provides API for adding/modifying static and animated geometry
//  in the scene displayed in the window.
//
//  Created by Craig Reynolds on September 9, 2024.
//  MIT License -- Copyright © 2024 Craig Reynolds
//-------------------------------------------------------------------------------

#pragma once

#include "LocalSpace.h"

#ifdef USE_OPEN3D
#include "open3d/Open3D.h"
#endif  // USE_OPEN3D

class Draw
{
public:
#ifdef USE_OPEN3D
    // Short names for the Open3D visualizer class used here and base class:
    typedef open3d::visualization::VisualizerWithKeyCallback vis_t;
    typedef open3d::visualization::Visualizer base_vis_t;
#endif  // USE_OPEN3D

    // Used to get the current global Draw object (drawing context).
    static Draw& getInstance() { return *global_object_; }
    
    // Default constructor
    Draw() : Draw(false) {}
    
    // Constructor with args for visualizer window.
    Draw(bool enabled,
         Vec3 window_xy_size = Vec3(2000, 2000, 0),
         Vec3 window_xy_position_ul = Vec3(),
         std::string window_title = "evoflock",
         int line_width = 10,
         int point_size = 20)
    {
        // Handle pointer to singleton instance of Draw class.
        assert(global_object_ == nullptr);
        global_object_ = this;
        setEnable(true);

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20241004 follow cam
        // TODO 20241006 follow cam
//        camera() = camera().fromTo({60, 60, 60}, {});
        camera() = camera().fromTo(Vec3(1,1,1).normalize() * 10, {});
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifdef USE_OPEN3D
        std::cout << "Begin graphics session using: Open3D ";
        std::cout << OPEN3D_VERSION << std::endl;

        // Allocate TriangleMesh/LineSet objects which hold animated geometry.
        animated_tri_mesh_ = std::make_shared<open3d::geometry::TriangleMesh>();
        animated_line_set_ = std::make_shared<open3d::geometry::LineSet>();
        
        // Create window for visualizer.
        visualizer().CreateVisualizerWindow(window_title,
                                            window_xy_size.x(),
                                            window_xy_size.y(),
                                            window_xy_position_ul.x(),
                                            window_xy_position_ul.y());
        // TODO does not work, see https://github.com/isl-org/Open3D/issues/6952
        visualizer().GetRenderOption().line_width_ = line_width;
        visualizer().GetRenderOption().point_size_ = point_size;

        // TODO temporary work-around to create the big sphere.
        tempAddSphere();
        
        // Add single key command callback to toggle "graphics mode"
        visualizer().RegisterKeyCallback('G',
                                         [&](base_vis_t* vis)
                                         { toggleEnable(); return true; });
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20241007 follow cam -- to/from tracker balls
        

        to_ball = open3d::geometry::TriangleMesh::CreateSphere(1);
        from_ball = open3d::geometry::TriangleMesh::CreateSphere(1);
        to_ball->PaintUniformColor({0, 1, 0});
        from_ball->PaintUniformColor({1, 0, 1});
        visualizer().AddGeometry(to_ball);
        visualizer().AddGeometry(from_ball);
        

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#endif  // USE_OPEN3D
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241007 follow cam -- to/from tracker balls
    std::shared_ptr<open3d::geometry::TriangleMesh> from_ball;
    std::shared_ptr<open3d::geometry::TriangleMesh> to_ball;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    ~Draw()
    {
        global_object_ = nullptr;
        std::cout << "End graphics session. Total triangles drawn: ";
        std::cout << triangle_count_ << "." << std::endl;
    }

    void beginAnimatedScene()
    {
        if (enable())
        {
            clearAnimatedGeometryFromScene();
            // Add empty animated geometry object to scene (no reset_bounding_box).
            visualizer().AddGeometry(animated_tri_mesh_, false);
            visualizer().AddGeometry(animated_line_set_, false);
        }
    }

    void endAnimatedScene()
    {
        if (enable())
        {
        }
    }

    void beginOneAnimatedFrame()
    {
#ifdef USE_OPEN3D
        // TODO 20240930 maybe should also set some global "exit from run" flag?
        pollEvents();
        if (enable())
        {
            animated_tri_mesh_->Clear();
            animated_line_set_->Clear();
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20241004 follow cam
            // TODO 20241006 follow cam
            
//            updateFollowCameraPosition();   // !! Oct 7
            


//            updateCamera();
//            setOpen3dViewFromCamera();
            
            updateCamera();

//            setOpen3dViewFromCamera();
            
            debugPrint((camera().p() - aimTarget()).length());
//            std::this_thread::sleep_for(std::chrono::milliseconds(1000/60));
            std::this_thread::sleep_for(std::chrono::milliseconds(1000/30));
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            
        }
#endif  // USE_OPEN3D
    }
    
    void endOneAnimatedFrame()
    {
#ifdef USE_OPEN3D
        if (enable())
        {
            animated_tri_mesh_->ComputeVertexNormals();
            visualizer().UpdateGeometry(animated_tri_mesh_);
            visualizer().UpdateGeometry(animated_line_set_);
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20241007 follow cam -- to/from tracker balls
            visualizer().UpdateGeometry(from_ball);
            visualizer().UpdateGeometry(to_ball);
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        }
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
    void addTriMeshToAnimatedFrame(const std::vector<Vec3>& vertices,
                                   const std::vector<std::size_t>& triangles,
                                   const Vec3& color)
    {
        std::vector<Vec3> colors(vertices.size(), color);
        addTriMeshToAnimatedFrame(vertices, triangles, colors);
    }
    
    // Add to per-frame collection of animating triangles: per-vertex colors.
    void addTriMeshToAnimatedFrame(const std::vector<Vec3>& vertices,
                                   const std::vector<std::size_t>& triangles,
                                   const std::vector<Vec3>& colors)
    {
#ifdef USE_OPEN3D
        if (enable())
        {
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
        }
#endif  // USE_OPEN3D
    }
    
    // Add to per-frame collection of animating line segments for annotation.
    void addLineSegmentToAnimatedFrame(const Vec3& endpoint0,
                                       const Vec3& endpoint1,
                                       const Vec3& color)
    {
#ifdef USE_OPEN3D
        if (enable())
        {
            auto s = animated_line_set_->points_.size();
            animated_line_set_->points_.push_back(vec3ToEv3d(endpoint0));
            animated_line_set_->points_.push_back(vec3ToEv3d(endpoint1));
            animated_line_set_->lines_.push_back({s, s + 1});
            animated_line_set_->colors_.push_back(vec3ToEv3d(color));
        }
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
        visualizer().AddGeometry(sphere);
#endif  // USE_OPEN3D
    }

    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240911 try evert TriangleMesh.
#ifdef USE_OPEN3D
    
    // Flip orientation of each tri in a triangle mesh (destructively modifies).
    // (pr to add to Open3D? https://github.com/isl-org/Open3D/discussions/6419)
    void evertTriangleMesh(open3d::geometry::TriangleMesh& tri_mesh)
    {
        for (auto& triangle : tri_mesh.triangles_)
        {
            std::reverse(std::begin(triangle), std::end(triangle));
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
        addTriMeshToAnimatedFrame({nose, apex, wingtip0, wingtip1}, // vertices
                                  {1,2,3, 3,2,0, 0,1,3, 2,1,0},     // triangles
                                  color);                           // color
    }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    
    // Just for debugging and testing. Run Open3D tests.
    static void visualizeEvoflockFitnessTest()
    {
#ifdef USE_OPEN3D
        Draw draw(true);
        draw.beginAnimatedScene();
        auto saved_tri_mesh_ = std::make_shared<open3d::geometry::TriangleMesh>();
        auto saved_line_set_ = std::make_shared<open3d::geometry::LineSet>();
        auto& rs = EF::RS();
        Vec3 a;
        Vec3 b;
        for (int i = 0; i < 1000; i++)
        {
            a = b;
            b = b + rs.random_unit_vector() * 0.5;
            auto c = rs.random_point_in_axis_aligned_box(Vec3(), Vec3(1, 1, 1));
            draw.addLineSegmentToAnimatedFrame(a, b, c);
        }
        auto copyLineSet = [](auto a, auto b)
        {
            a->points_ = b->points_;
            a->lines_ = b->lines_;
            a->colors_ = b->colors_;
        };
        copyLineSet(saved_line_set_, draw.animated_line_set_);

        // Loop for displaying animated graphics.
        while (draw.pollEvents())
        {
            if (draw.enable())
            {
                draw.beginOneAnimatedFrame();
                draw.addTriMeshToAnimatedFrame
                ({{0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}},     // verts
                 {1,2,3, 3,2,0, 0,1,3, 2,1,0},             // tris
                 {{1,1,1}, {1,0,0}, {0,1,0}, {0,0,1}});    // colors
                draw.addTriMeshToAnimatedFrame
                ({{0,0,0}, {-1,0,0}, {0,-1,0}, {0,0,-1}},  // verts
                 {3,2,1, 0,2,3, 3,1,0, 0,1,2},             // tris
                 {0.3,0.3,0.3});                           // color
                draw.drawBoidBody({3, 3, 0}, {1,0,0}, {0,1,0}, {0,0,1}, 0.5, {1,1,0});
                for (int i = 0; i < 500; i++)
                {
                    Vec3 p = rs.randomPointInUnitRadiusSphere() * 30;
                    auto ls = LocalSpace().randomize_orientation();
                    auto c = rs.random_point_in_axis_aligned_box(Vec3(0.4, 0.4, 0.4),
                                                                 Vec3(1.0, 1.0, 1.0));
                    draw.drawBoidBody(p, ls.i(), ls.j(), ls.k(), 0.5, c);
                }
                
                copyLineSet(draw.animated_line_set_, saved_line_set_);
                double jiggle = 0.01;
                auto& endpoints = draw.animated_line_set_->points_;
                for (int i = 5; i < endpoints.size(); i += 2)
                {
                    endpoints[i].x() += EF::RS().random2(-jiggle, jiggle);
                    endpoints[i].y() += EF::RS().random2(-jiggle, jiggle);
                    endpoints[i].z() += EF::RS().random2(-jiggle, jiggle);
                    endpoints[i+1] = endpoints[i];
                }
                copyLineSet(saved_line_set_, draw.animated_line_set_);

                draw.addLineSegmentToAnimatedFrame({ 0, 60, 0},
                                                   {0, -60, 0},
                                                   {1, 1, 0});
                draw.addLineSegmentToAnimatedFrame({-5,  0, 0},
                                                   {5,   0, 0},
                                                   {0, 1, 1});
                draw.endOneAnimatedFrame();
                std::this_thread::sleep_for(std::chrono::milliseconds(33)); // 1/30
            }
        }
        draw.endAnimatedScene();
#endif  // USE_OPEN3D
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    // Runtime switch to turn graphical display on and off.
    bool enable() const { return enable_; }
    void setEnable(bool e) { enable_ = e; }
    void toggleEnable() { enable_ = not enable_; }

    bool pollEvents()
    {
        return visualizer().PollEvents();
    }

    //--------------------------------------------------------------------------
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
    
    // Example code from https://github.com/isl-org/Open3D/issues/6952
    static void LineWidthPointSizeTest()
    {
        open3d::PrintOpen3DVersion();
        auto lineset = std::make_shared<open3d::geometry::LineSet>();
        lineset->points_ = {{5, 0, 0}, {-5, 0, 0}, {0, 5, 0}, {0, -5, 0}, {0, 0, 5}, {0, 0, -5}};
        lineset->lines_ = {{0, 1}, {2, 3}, {4, 5}};
        lineset->colors_ = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        open3d::visualization::gui::Application::GetInstance().Initialize();
        auto vis = open3d::visualization::visualizer::O3DVisualizer("name", 500, 500);
        vis.AddGeometry("lines", {lineset});
        vis.SetLineWidth(10);
        vis.SetPointSize(20);
        open3d::visualization::gui::Application::GetInstance().Run();
    }

    //--------------------------------------------------------------------------
    // Example code for https://stackoverflow.com/q/79048820/1991373
    
//    static void Oct2Test()
    static void test()
    {
        open3d::PrintOpen3DVersion();
        auto vis = open3d::visualization::Visualizer();
        vis.CreateVisualizerWindow();
        auto ball = open3d::geometry::TriangleMesh::CreateSphere(5);
        ball->ComputeVertexNormals();
        ball->PaintUniformColor({1, 0, 0});
        vis.AddGeometry(ball);
        
        vis.GetViewControl().SetLookat({5,5,5});
        
        vis.Run();
    }

    static void test2()
    {
        
        open3d::PrintOpen3DVersion();
        
        open3d::visualization::gui::Application::GetInstance().Initialize();
        auto vis = open3d::visualization::visualizer::O3DVisualizer("name", 500, 500);
        
        auto ball = open3d::geometry::TriangleMesh::CreateSphere(5);
        ball->ComputeVertexNormals();
        ball->PaintUniformColor({1, 0, 0});
        
        vis.AddGeometry("ball", {ball});
        open3d::visualization::gui::Application::GetInstance().Run();
        
    }
    
    // TODO 20241008 wondered about using O3DVisualizer without Application, but
    // apparently not. It gets "gui::Initialize() must be called before creating
    // a window or UI element."
    static void test3()
    {
        open3d::PrintOpen3DVersion();
        // open3d::visualization::gui::Application::GetInstance().Initialize();
        auto vis = open3d::visualization::visualizer::O3DVisualizer("name", 500, 500);
        auto ball = open3d::geometry::TriangleMesh::CreateSphere(5);
        ball->ComputeVertexNormals();
        ball->PaintUniformColor({1, 0, 0});
        vis.AddGeometry("ball", {ball});
        // open3d::visualization::gui::Application::GetInstance().Run();
    }
    
    //--------------------------------------------------------------------------

    // Accessor for Open3D Visualizer instance.
    vis_t& visualizer() { return visualizer_; }
    const vis_t& visualizer() const { return visualizer_; }

    // Accessor for camera object, represented as a LocalSpace.
    LocalSpace& camera() { return camera_; }
    const LocalSpace& camera() const { return camera_; }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241004 follow cam
    
    // Accessor for camera aim target posItion.
    Vec3& aimTarget() { return aim_target_; }
    const Vec3& aimTarget() const { return aim_target_; }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
//        void updateCamera()
//        {
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//            // TODO 20241004 follow cam
//            // TODO 20241002 This initial state is just for debugging, fix later.
//    //        camera() = camera().fromTo({60, 60, 60}, {});
//
//    //        camera() = camera().fromTo({60, 60, 60}, aimTarget());
//
//            // This prints 60. As in 60 degrees? Does not change with “scroll wheel”
//    //        debugPrint(visualizer().GetViewControl().GetFieldOfView());
//
//    //        Vec3 testpos(6, 6, 6);
//    //        Vec3 testpos(3, 3, 3);
//            Vec3 testpos(1, 1, 1);
//            camera() = camera().fromTo(testpos - aimTarget(), aimTarget());
//    //        camera() = camera().fromTo(aimTarget() - testpos, aimTarget());
//
//    //        Vec3 camera_position = camera().p();
//    //        Vec3 camera_forward = camera().k();
//
//
//
//
//            // TODO 20241005 maybe I can directly set the matrix using
//            // open3d::camera::PinholeCameraParameters::extrinsic_ and
//            // open3d::visualization::ViewControl::ConvertFromPinholeCameraParameters()
//
//            // or could we use this (from ViewControl.cpp)?:
//            //     view_matrix_ = gl_util::LookAt(eye_, lookat_, up_);
//
//
//            open3d::visualization::gl_util::LookAt(vec3ToEv3d(camera().p()),
//                                                   vec3ToEv3d(aimTarget()),
//                                                   vec3ToEv3d(Vec3(0, 1, 0)));
//
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//        }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241004 follow cam
    // TODO 20241006 follow cam

//        void updateCamera()
//        {
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//            // TODO 20241004 follow cam
//
//
//            // TODO 20241005 maybe I can directly set the matrix using
//            // open3d::camera::PinholeCameraParameters::extrinsic_ and
//            // open3d::visualization::ViewControl::ConvertFromPinholeCameraParameters()
//
//            // or could we use this (from ViewControl.cpp)?:
//            //     view_matrix_ = gl_util::LookAt(eye_, lookat_, up_);
//            // no, that simply computes and returns the view matrix.
//
//            // combining?
//            open3d::camera::PinholeCameraParameters pcp;
//
//
//            auto la = [](Vec3 from, Vec3 to, Vec3 up = Vec3(0, 1, 0))
//            {
//                return open3d::visualization::gl_util::LookAt(vec3ToEv3d(from),
//                                                              vec3ToEv3d(to),
//                                                              vec3ToEv3d(up));
//            };
//
//
//
//    //        auto la_matrix = la(camera().p(), aimTarget(), Vec3(0, 1, 0));
//    //        auto la_matrix = la(Vec3(60, 60, 60), aimTarget(), Vec3(0, 1, 0));
//            auto la_matrix = la(Vec3(60, 60, 60), Vec3(), Vec3(0, 1, 0));
//
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//
//            // TODO There MUST be a cleaner way to copy the one extrinsic to another
//            for (int j = 0; j < 4; j++)
//            {
//                for (int i = 0; i < 4; i++)
//                {
//                    pcp.extrinsic_(i, j) = la_matrix(i, j);
//                }
//            }
//
//            open3d::camera::PinholeCameraParameters previous_pcp;
//            visualizer().GetViewControl().ConvertToPinholeCameraParameters(previous_pcp);
//            std::cout << "previous_pcp.extrinsic_:" << std::endl;
//            std::cout << previous_pcp.extrinsic_ << std::endl;
//
//    //        int     width_ = -1
//    //        Width of the image. More...
//    //
//    //        int     height_ = -1
//    //        Height of the image. More...
//    //
//    //        Eigen::Matrix3d     intrinsic_matrix_
//
//    //        visualizer().GetViewControl().ConvertFromPinholeCameraParameters(pcp,
//    //                                                                         true);
//
//    //        visualizer().GetViewControl().ConvertFromPinholeCameraParameters(pcp);
//
//            debugPrint(previous_pcp.intrinsic_.width_)
//            debugPrint(previous_pcp.intrinsic_.height_)
//
//            std::cout << "previous_pcp.intrinsic_.intrinsic_matrix_:" << std::endl;
//            std::cout << previous_pcp.intrinsic_.intrinsic_matrix_ << std::endl;
//
//
//            //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//            //
//            // do something to globally capture the initial pcp.intrinsic_
//            //
//            // copy into per-frame PCP before ConvertFromPinholeCameraParameters(pcp)
//
//            static bool default_pcp_captured = false;
//            static open3d::camera::PinholeCameraParameters default_pcp;
//            if (not default_pcp_captured)
//            {
//                visualizer().GetViewControl().ConvertToPinholeCameraParameters(default_pcp);
//                default_pcp_captured = true;
//            }
//
//            pcp.intrinsic_ = default_pcp.intrinsic_;
//            visualizer().GetViewControl().ConvertFromPinholeCameraParameters(pcp);
//
//            std::cout << "pcp.intrinsic_.intrinsic_matrix_:" << std::endl;
//            std::cout << pcp.intrinsic_.intrinsic_matrix_ << std::endl;
//
//
//            //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
//
//            //{
//            //	"class_name" : "ViewTrajectory",
//            //	"interval" : 29,
//            //	"is_loop" : false,
//            //	"trajectory" :
//            //	[
//            //		{
//            //			"boundingbox_max" : [ 50.0, 50.0, 50.0 ],
//            //			"boundingbox_min" : [ -50.0, -50.0, -50.0 ],
//            //			"field_of_view" : 60.0,
//            //			"front" : [ 0.0, 0.0, 1.0 ],
//            //			"lookat" : [ 0.0, 0.0, 0.0 ],
//            //			"up" : [ 0.0, 1.0, 0.0 ],
//            //			"zoom" : 0.69999999999999996
//            //		}
//            //	],
//            //	"version_major" : 1,
//            //	"version_minor" : 0
//            //}
//
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//        }

//        void updateCamera()
//        {
//            bool print_enable = false;
//
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//            // TODO 20241004 follow cam
//
//
//            // TODO 20241005 maybe I can directly set the matrix using
//            // open3d::camera::PinholeCameraParameters::extrinsic_ and
//            // open3d::visualization::ViewControl::ConvertFromPinholeCameraParameters()
//
//            // or could we use this (from ViewControl.cpp)?:
//            //     view_matrix_ = gl_util::LookAt(eye_, lookat_, up_);
//            // no, that simply computes and returns the view matrix.
//
//            // combining?
//            open3d::camera::PinholeCameraParameters pcp;
//
//
//            auto la = [](Vec3 from, Vec3 to, Vec3 up = Vec3(0, 1, 0))
//            {
//                return open3d::visualization::gl_util::LookAt(vec3ToEv3d(from),
//                                                              vec3ToEv3d(to),
//                                                              vec3ToEv3d(up));
//            };
//
//
//            std::vector<Vec3> from_to = computeFollowCameraFromTo();
//
//            camera() = camera().fromTo(from_to.at(0), from_to.at(1));
//
//            auto la_matrix = la(from_to.at(0), from_to.at(1));
//
//
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//            // TODO 20241008 "Igor: reverse the polarity!"
//    //        la_matrix = la_matrix.inverse();
//    //        la_matrix = la_matrix.inverse().eval();
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//
//
//            std::cout << std::endl;
//            std::cout << "my cam pos: " << camera().p() << std::endl;
//            std::cout << "O3d GetEye: " << visualizer().GetViewControl().GetEye().transpose() << std::endl;
//            std::cout << "target pos: " << aimTarget() << std::endl;
//
//            debugPrint(from_to.at(0))
//            debugPrint(from_to.at(1))
//
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//            // TODO 20241007 follow cam -- to/from tracker balls
//            from_ball->Translate(vec3ToEv3d(from_to.at(0)), false);
//            to_ball->Translate(vec3ToEv3d(from_to.at(1)), false);
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//
//            // TODO There MUST be a cleaner way to copy the one extrinsic to another
//            for (int j = 0; j < 4; j++)
//            {
//                for (int i = 0; i < 4; i++)
//                {
//                    pcp.extrinsic_(i, j) = la_matrix(i, j);
//                }
//            }
//
//            open3d::camera::PinholeCameraParameters previous_pcp;
//            visualizer().GetViewControl().ConvertToPinholeCameraParameters(previous_pcp);
//
//            if (print_enable)
//            {
//                std::cout << "previous_pcp.extrinsic_:" << std::endl;
//                std::cout << previous_pcp.extrinsic_ << std::endl;
//
//                //debugPrint(previous_pcp.intrinsic_.width_)
//                //debugPrint(previous_pcp.intrinsic_.height_)
//                //
//                //std::cout << "previous_pcp.intrinsic_.intrinsic_matrix_:" << std::endl;
//                //std::cout << previous_pcp.intrinsic_.intrinsic_matrix_ << std::endl;
//            }
//
//            //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//            //
//            // do something to globally capture the initial pcp.intrinsic_
//            //
//            // copy into per-frame PCP before ConvertFromPinholeCameraParameters(pcp)
//
//            static bool default_pcp_captured = false;
//            static open3d::camera::PinholeCameraParameters default_pcp;
//            if (not default_pcp_captured)
//            {
//                visualizer().GetViewControl().ConvertToPinholeCameraParameters(default_pcp);
//                default_pcp_captured = true;
//            }
//
//            pcp.intrinsic_ = default_pcp.intrinsic_;
//
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//            // TODO 20241007 follow cam -- to/from tracker balls
//            visualizer().GetViewControl().ConvertFromPinholeCameraParameters(pcp);
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//
//            if (print_enable)
//            {
//                // std::cout << "pcp.intrinsic_.intrinsic_matrix_:" << std::endl;
//                // std::cout << pcp.intrinsic_.intrinsic_matrix_ << std::endl;
//            }
//
//            //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
//
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//        }

//        void updateCamera()
//        {
//    //        bool print_enable = false;
//
//            // Nickname for open3d::visualization::gl_util::LookAt() args are Vec3.
//            auto la = [](Vec3 from, Vec3 to, Vec3 up = Vec3(0, 1, 0))
//            {
//                return open3d::visualization::gl_util::LookAt(vec3ToEv3d(from),
//                                                              vec3ToEv3d(to),
//                                                              vec3ToEv3d(up));
//            };
//
//            // Invoke the "follow cam" model, update look_from / look_at points
//            std::vector<Vec3> from_to = computeFollowCameraFromTo();
//            const Vec3& look_from = from_to.at(0);
//            const Vec3& look_at = from_to.at(1);
//
//            // Update this Draw instance's camera to from/at orientation
//            camera() = camera().fromTo(look_from, look_at);
//
//            // Compute from/at 4x4 matrix (type GLMatrix4f)
//            auto la_matrix = la(look_from, look_at);
//
//            std::cout << std::endl;
//            std::cout << "my cam pos: " << camera().p() << std::endl;
//            std::cout << "O3d GetEye: " << visualizer().GetViewControl().GetEye().transpose() << std::endl;
//            std::cout << "target pos: " << aimTarget() << std::endl;
//            debugPrint(look_from)
//            debugPrint(look_at)
//
//            // Update from/at tracking balls (only for debugging
//            from_ball->Translate(vec3ToEv3d(look_from), false);
//            to_ball->Translate(vec3ToEv3d(look_at), false);
//
//            // TODO There MUST be a cleaner way to copy the one extrinsic to another
//            open3d::camera::PinholeCameraParameters pcp;
//
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//            visualizer().GetViewControl().ConvertToPinholeCameraParameters(pcp);
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//            for (int j = 0; j < 4; j++)
//            {
//                for (int i = 0; i < 4; i++)
//                {
//                    pcp.extrinsic_(i, j) = la_matrix(i, j);
//                }
//            }
//
//    //        open3d::camera::PinholeCameraParameters previous_pcp;
//    //        visualizer().GetViewControl().ConvertToPinholeCameraParameters(previous_pcp);
//    //
//    //        if (print_enable)
//    //        {
//    //            std::cout << "previous_pcp.extrinsic_:" << std::endl;
//    //            std::cout << previous_pcp.extrinsic_ << std::endl;
//    //        }
//
//    //        // do something to globally capture the initial pcp.intrinsic_
//    //        // copy into per-frame PCP before ConvertFromPinholeCameraParameters(pcp)
//    //
//    //        static bool default_pcp_captured = false;
//    //        static open3d::camera::PinholeCameraParameters default_pcp;
//    //        if (not default_pcp_captured)
//    //        {
//    //            visualizer().GetViewControl().ConvertToPinholeCameraParameters(default_pcp);
//    //            default_pcp_captured = true;
//    //        }
//    //
//    //        pcp.intrinsic_ = default_pcp.intrinsic_;
//
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//            // TODO 20241007 follow cam -- to/from tracker balls
//            visualizer().GetViewControl().ConvertFromPinholeCameraParameters(pcp);
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//
//    //        if (print_enable)
//    //        {
//    //            // std::cout << "pcp.intrinsic_.intrinsic_matrix_:" << std::endl;
//    //            // std::cout << pcp.intrinsic_.intrinsic_matrix_ << std::endl;
//    //        }
//        }

    
//        void updateCamera()
//        {
//            // Nickname for open3d::visualization::gl_util::LookAt() args are Vec3.
//            auto la = [](Vec3 from, Vec3 to, Vec3 up = Vec3(0, 1, 0))
//            {
//                return open3d::visualization::gl_util::LookAt(vec3ToEv3d(from),
//                                                              vec3ToEv3d(to),
//                                                              vec3ToEv3d(up));
//            };
//
//            // Invoke the "follow cam" model, update look_from / look_at points
//            std::vector<Vec3> from_to = computeFollowCameraFromTo();
//            const Vec3& look_from = from_to.at(0);
//            const Vec3& look_at = from_to.at(1);
//
//
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//            // TODO 20241008 at this point, I don't think camera is being used at all
//            //               Oops no, its used for computeFollowCameraFromTo()
//            //
//            //               Oh, instead of the awkward passing back two values, what
//            //               about stashing them in the instance, like the camera
//            //               itself, say into cameraLookFrom() and cameraLookAt()?
//
//            // Update this Draw instance's camera to from/at orientation
//            camera() = camera().fromTo(look_from, look_at);
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//            // Compute from/at 4x4 matrix (type GLMatrix4f)
//            auto la_matrix = la(look_from, look_at);
//
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//            std::cout << std::endl;
//            std::cout << "my cam pos: " << camera().p() << std::endl;
//            std::cout << "O3d GetEye: " << visualizer().GetViewControl().GetEye().transpose() << std::endl;
//            std::cout << "target pos: " << aimTarget() << std::endl;
//            debugPrint(look_from)
//            debugPrint(look_at)
//            // Update from/at tracking balls (only for debugging).
//            from_ball->Translate(vec3ToEv3d(look_from), false);
//            to_ball->Translate(vec3ToEv3d(look_at), false);
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//            // Get current PinholeCameraParameters, then overwrite view matrix.
//            open3d::camera::PinholeCameraParameters pcp;
//            visualizer().GetViewControl().ConvertToPinholeCameraParameters(pcp);
//
//
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//    //            // TODO There MUST be a cleaner way to copy one extrinsic into another.
//    //            for (int j = 0; j < 4; j++)
//    //            {
//    //                for (int i = 0; i < 4; i++)
//    //                {
//    //                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//    //                    pcp.extrinsic_(i, j) = la_matrix(i, j);
//    //    //                pcp.extrinsic_(i, j) = la_matrix(j, i);
//    //    //                pcp.extrinsic_(j, i) = la_matrix(i, j);
//    //                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//    //                }
//    //            }
//
//    //        pcp.extrinsic_ = la_matrix;
//    //        f = d.cast <float> ()
//
//            pcp.extrinsic_ = la_matrix.cast<double>();
//
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//            std::cout << "pcp.extrinsic_ (Eigen::Matrix4d_u):" << std::endl;
//            std::cout << pcp.extrinsic_ << std::endl;
//
//            std::cout << "la_matrix (GLMatrix4f):" << std::endl;
//            std::cout << la_matrix << std::endl;
//
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//            // Write back PinholeCameraParameters with new from/at view matrix.
//            visualizer().GetViewControl().ConvertFromPinholeCameraParameters(pcp);
//        }

//        void updateCamera()
//        {
//    //            // Nickname for open3d::visualization::gl_util::LookAt() args are Vec3.
//    //            auto la = [](Vec3 from, Vec3 to, Vec3 up = Vec3(0, 1, 0))
//    //            {
//    //                auto mat = open3d::visualization::gl_util::LookAt(vec3ToEv3d(from),
//    //                                                                  vec3ToEv3d(to),
//    //                                                                  vec3ToEv3d(up));
//    //    //            return mat.cast<double>();
//    //    //            return mat;
//    //                return mat.cast<double>();
//    //            };
//
//            // Nickname for open3d::visualization::gl_util::LookAt() args are Vec3.
//            auto la = [](Vec3 from, Vec3 to, Vec3 up = Vec3(0, 1, 0))
//            {
//                return open3d::visualization::gl_util::LookAt(vec3ToEv3d(from),
//                                                              vec3ToEv3d(to),
//                                                              vec3ToEv3d(up));
//            };
//
//            // Invoke the "follow cam" model, update look_from / look_at points
//    //        std::vector<Vec3> from_to = computeFollowCameraFromTo();
//    //        const Vec3& look_from = from_to.at(0);
//    //        const Vec3& look_at = from_to.at(1);
//            computeFollowCameraFromTo();
//
//            assert(cameraLookFrom() != cameraLookAt());
//
//            // Update this Draw instance's camera to from/at orientation
//    //        camera() = camera().fromTo(look_from, look_at);
//            camera() = camera().fromTo(cameraLookFrom(), cameraLookAt());
//
//            // Compute from/at 4x4 matrix (type GLMatrix4f)
//    //        auto la_matrix = la(look_from, look_at);
//            auto la_matrix = la(cameraLookFrom(), cameraLookAt());
//
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//            std::cout << std::endl;
//            std::cout << "my cam pos: " << camera().p() << std::endl;
//            std::cout << "O3d GetEye: " << visualizer().GetViewControl().GetEye().transpose() << std::endl;
//            std::cout << "target pos: " << aimTarget() << std::endl;
//    //        debugPrint(look_from)
//    //        debugPrint(look_at)
//            debugPrint(cameraLookFrom());
//            debugPrint(cameraLookAt());
//            // Update from/at tracking balls (only for debugging).
//    //        from_ball->Translate(vec3ToEv3d(look_from), false);
//    //        to_ball->Translate(vec3ToEv3d(look_at), false);
//            from_ball->Translate(vec3ToEv3d(cameraLookFrom()), false);
//            to_ball->Translate(vec3ToEv3d(cameraLookAt()), false);
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//            // Get current PinholeCameraParameters (pcp).
//            open3d::camera::PinholeCameraParameters pcp;
//            visualizer().GetViewControl().ConvertToPinholeCameraParameters(pcp);
//
//            // Then overwrite the pcp's view matrix with the new lookat matrix.
//            pcp.extrinsic_ = la_matrix.cast<double>();
//
//            // Write back PinholeCameraParameters with new from/at view matrix.
//            visualizer().GetViewControl().ConvertFromPinholeCameraParameters(pcp);
//        }

    void updateCamera()
    {
        // Nickname for open3d::visualization::gl_util::LookAt() args are Vec3.
        auto la = [](Vec3 from, Vec3 to, Vec3 up = Vec3(0, 1, 0))
        {
            return open3d::visualization::gl_util::LookAt(vec3ToEv3d(from),
                                                          vec3ToEv3d(to),
                                                          vec3ToEv3d(up));
        };

        // Invoke the "follow cam" model, update look_from / look_at points
        computeFollowCameraFromTo();
        
        // Update this Draw instance's camera to from/at orientation
        camera() = camera().fromTo(cameraLookFrom(), cameraLookAt());

        // Compute from/at 4x4 matrix (type GLMatrix4f)
        auto la_matrix = la(cameraLookFrom(), cameraLookAt());

        {
            std::cout << std::endl;
            std::cout << "my cam pos: " << camera().p() << std::endl;
            std::cout << "O3d GetEye: " << visualizer().GetViewControl().GetEye().transpose() << std::endl;
            std::cout << "target pos: " << aimTarget() << std::endl;
            debugPrint(cameraLookFrom());
            debugPrint(cameraLookAt());
            // Update from/at tracking balls (only for debugging).
            from_ball->Translate(vec3ToEv3d(cameraLookFrom()), false);
            to_ball->Translate(vec3ToEv3d(cameraLookAt()), false);
        }

        // Get current PinholeCameraParameters (pcp).
        open3d::camera::PinholeCameraParameters pcp;
        visualizer().GetViewControl().ConvertToPinholeCameraParameters(pcp);

        // Then overwrite the pcp's view matrix with the new lookat matrix.
        pcp.extrinsic_ = la_matrix.cast<double>();
        
        // Write back PinholeCameraParameters with new from/at view matrix.
        visualizer().GetViewControl().ConvertFromPinholeCameraParameters(pcp);
    }


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241004 follow cam
    // TODO 20241006 follow cam
    
//        std::vector<Vec3> computeFollowCameraFromTo()
//        {
//    //        double offset_dist_target = 10;
//    //        double offset_dist_target = 40;
//            double offset_dist_target = 20;
//            Vec3 camera_pos = camera().p();
//            Vec3 offset_from_cam_to_target = aimTarget() - camera_pos;
//            double offset_distance = offset_from_cam_to_target.length();
//            Vec3 offset_direction = offset_from_cam_to_target / offset_distance;
//            Vec3 offset_target = aimTarget() - (offset_direction * offset_dist_target);
//    //        Vec3 new_cam_pos = util::interpolate(0.001, camera_pos, aimTarget());
//            Vec3 new_cam_pos = util::interpolate(0.01, camera_pos, aimTarget());
//            return {new_cam_pos, offset_target};
//        }

//        std::vector<Vec3> computeFollowCameraFromTo()
//        {
//            double offset_dist_target = 20;
//            Vec3 camera_pos = camera().p();
//            Vec3 target_pos = aimTarget();
//
//    //        Vec3 offset_from_camera_to_target = aimTarget() - camera_pos;
//            Vec3 offset_from_camera_to_target = target_pos - camera_pos;
//
//            double offset_distance = offset_from_camera_to_target.length();
//            Vec3 offset_direction = offset_from_camera_to_target / offset_distance;
//
//    //        Vec3 offset_target = aimTarget() - (offset_direction * offset_dist_target);
//            Vec3 offset_target = aimTarget() - (offset_direction * offset_dist_target);
//
//    //        Vec3 new_cam_pos = util::interpolate(0.01, camera_pos, aimTarget());
//    //        return {new_cam_pos, offset_target};
//
//            Vec3 new_cam_pos = util::interpolate(0.01, camera_pos, offset_target);
//            return {new_cam_pos, target_pos};
//        }

//        std::vector<Vec3> computeFollowCameraFromTo()
//        {
//    //        double offset_dist_target = 20;
//            double desired_offset_dist = 20;
//            Vec3 camera_pos = camera().p();
//            Vec3 target_pos = aimTarget();
//            Vec3 offset_from_camera_to_target = target_pos - camera_pos;
//            double offset_distance = offset_from_camera_to_target.length();
//            Vec3 offset_direction = offset_from_camera_to_target / offset_distance;
//    //        Vec3 offset_target = aimTarget() - (offset_direction * offset_dist_target);
//            Vec3 offset_target = aimTarget() - (offset_direction * desired_offset_dist);
//    //        Vec3 new_cam_pos = util::interpolate(0.01, camera_pos, offset_target);
//            Vec3 new_cam_pos = util::interpolate(0.1, camera_pos, offset_target);
//            return {new_cam_pos, target_pos};
//        }

    
//    std::vector<Vec3> computeFollowCameraFromTo()
//    {
//        double desired_offset_dist = 10;
//        Vec3 camera_pos = camera().p();
//        Vec3 target_pos = aimTarget();
//        Vec3 offset_from_camera_to_target = target_pos - camera_pos;
//        double offset_distance = offset_from_camera_to_target.length();
//        Vec3 offset_direction = offset_from_camera_to_target / offset_distance;
//        Vec3 offset_target = aimTarget() - (offset_direction * desired_offset_dist);
//        Vec3 new_cam_pos = util::interpolate(0.1, camera_pos, offset_target);
//        return {new_cam_pos, target_pos};
//    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241010 Oh, instead of the awkward passing back two values, what
    //               about stashing them in the instance, like the camera
    //               itself, say into cameraLookFrom() and cameraLookAt()?
    
//    // Invoke the "follow cam" model, update look_from / look_at points
//    std::vector<Vec3> from_to = computeFollowCameraFromTo();
//    const Vec3& look_from = from_to.at(0);
//    const Vec3& look_at = from_to.at(1);
    
    
    Vec3 camera_look_from_;
    Vec3 camera_look_at_;    // This is identical to aimTarget() right? merge?
    
    Vec3 cameraLookFrom() const { return camera_look_from_; }
    Vec3 cameraLookAt() const { return camera_look_at_; }


//        void computeFollowCameraFromTo()
//        {
//            double desired_offset_dist = 10;
//            Vec3 camera_pos = camera().p();
//            Vec3 target_pos = aimTarget();
//            Vec3 offset_from_camera_to_target = target_pos - camera_pos;
//            double offset_distance = offset_from_camera_to_target.length();
//            Vec3 offset_direction = offset_from_camera_to_target / offset_distance;
//            Vec3 offset_target = aimTarget() - (offset_direction * desired_offset_dist);
//            Vec3 new_cam_pos = util::interpolate(0.1, camera_pos, offset_target);
//    //        return {new_cam_pos, target_pos};
//
//            camera_look_from_ = new_cam_pos;
//            camera_look_at_ = aimTarget();
//        }

    // Invoke the "follow camera" model, update look_from / look_at points.
    void computeFollowCameraFromTo()
    {
        double desired_offset_dist = 10;
        Vec3 camera_pos = camera().p();
        Vec3 offset_from_camera_to_target = aimTarget() - camera_pos;
        double offset_distance = offset_from_camera_to_target.length();
        Vec3 offset_direction = offset_from_camera_to_target / offset_distance;
        Vec3 offset_target = aimTarget() - (offset_direction * desired_offset_dist);
        Vec3 new_cam_pos = util::interpolate(0.1, camera_pos, offset_target);
        camera_look_from_ = new_cam_pos;
        camera_look_at_ = aimTarget();
    }


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    

    
    
//    void updateFollowCameraPosition()
//    {
//        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//        // TODO 20241007 follow cam -- to/from tracker balls
//        assert(false);  // Make sure we are not calling this
//        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//        std::vector<Vec3> from_to = computeFollowCameraFromTo();
//        
//        debugPrint(from_to.at(0))
//        debugPrint(from_to.at(1))
//
//        // XXX this is now duplicated in updateCamera() -- needs clean up!!
//        camera() = camera().fromTo(from_to.at(0), from_to.at(1));
//    }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//        void setOpen3dViewFromCamera()
//        {
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//            // TODO 20241007 follow cam -- to/from tracker balls
//            assert(false);  // Make sure we are not calling this
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//    //        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//    //        // TODO 20241004 follow cam
//    //        // TODO 20241006 follow cam
//    //
//    //        updateFollowCameraPosition();
//    //
//    //        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//            Vec3 camI = camera().i();
//            Vec3 camJ = camera().j();
//            Vec3 camK = camera().k();
//            Vec3 camP = camera().p();
//
//            // Explicitly set ViewControl's "up" and "front". WHY IS THIS NEEDED?!
//            visualizer().GetViewControl().SetUp(vec3ToEv3d(camJ));
//            visualizer().GetViewControl().SetFront(vec3ToEv3d(camK));
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//            // TODO 20241004 follow cam
//            visualizer().GetViewControl().SetLookat(vec3ToEv3d(aimTarget()));
//            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//            // Construct the view matrix by picking out scalar camera parameters.
//            Eigen::Matrix4d eigen_view_matrix;
//            eigen_view_matrix << camI.x(), camI.y(), camI.z(), camP.x(),
//                                 camJ.x(), camJ.y(), camJ.z(), camP.y(),
//                                 camK.x(), camK.y(), camK.z(), camP.z(),
//                                 0,        0,        0,        1;
//
//            // Set Open3D's Visualizer's ViewControl's view matrices.
//            visualizer().GetViewControl().SetViewMatrices(eigen_view_matrix);
//        }

    static void unit_test() {}
    
private:
    // Pointer to main global drawing context.
    static inline Draw* global_object_ = nullptr;
    
    // Runtime switch to turn graphical display on and off.
    bool enable_ = false;

    // Count all triangles drawn during graphics session.
    int triangle_count_ = 0;
    
    LocalSpace camera_;
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241004 follow cam
    Vec3 aim_target_;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


#ifdef USE_OPEN3D
    
    // Open3D TriangleMesh object for storing and drawing animated triangles.
    std::shared_ptr<open3d::geometry::TriangleMesh> animated_tri_mesh_ = nullptr;
    
    // Open3D LineSet object for storing and drawing animated line segments.
    std::shared_ptr<open3d::geometry::LineSet> animated_line_set_ = nullptr;
    
    // Open3D Visualizer object.
    vis_t visualizer_;

    // Private utility to convert a Vec3 to an Eigen::Vector3d.
    static Eigen::Vector3d vec3ToEv3d(const Vec3& v)
    {
        return Eigen::Vector3d(v.x(), v.y(), v.z());
    }
    
#endif  // USE_OPEN3D
};
