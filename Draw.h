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
        
//        // Add add "C" command, to cycle through camera aiming modes.
//        visualizer().RegisterKeyCallback('C',
//                                         [&](base_vis_t* vis)
//                                         { camera_mode_ = not camera_mode_;
//                                           return true; });
  
        // Add add "C" command, to cycle through camera aiming modes.
        visualizer().RegisterKeyCallback('C',
                                         [&](base_vis_t* vis)
                                         { nextCameraMode(); return true; });

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20241007 follow cam -- to/from tracker balls
        
        
        double ball_radius = 0.3;
        to_ball = open3d::geometry::TriangleMesh::CreateSphere(ball_radius);
        from_ball = open3d::geometry::TriangleMesh::CreateSphere(ball_radius);
        to_ball->PaintUniformColor({0, 1, 0});
        from_ball->PaintUniformColor({1, 0, 1});
        visualizer().AddGeometry(to_ball);
        visualizer().AddGeometry(from_ball);

//        la_to_ball = open3d::geometry::TriangleMesh::CreateSphere(ball_radius);
//        la_from_ball = open3d::geometry::TriangleMesh::CreateSphere(ball_radius);
//        la_to_ball->PaintUniformColor({0, 0, 1});
//        la_from_ball->PaintUniformColor({1, 1, 0});
//        visualizer().AddGeometry(la_to_ball);
//        visualizer().AddGeometry(la_from_ball);
        

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#endif  // USE_OPEN3D
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241007 follow cam -- to/from tracker balls
    std::shared_ptr<open3d::geometry::TriangleMesh> from_ball;
    std::shared_ptr<open3d::geometry::TriangleMesh> to_ball;
//    std::shared_ptr<open3d::geometry::TriangleMesh> la_from_ball;
//    std::shared_ptr<open3d::geometry::TriangleMesh> la_to_ball;
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
            
//            debugPrint((camera().p() - aimTarget()).length());
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
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20241007 follow cam -- to/from tracker balls
            visualizer().UpdateGeometry(from_ball);
            visualizer().UpdateGeometry(to_ball);
//            visualizer().UpdateGeometry(la_from_ball);
//            visualizer().UpdateGeometry(la_to_ball);
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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

    // Nickname for open3d::visualization::gl_util::LookAt() with Vec3 args.
    typedef open3d::visualization::gl_util::GLMatrix4f GLMatrix4f;
    static GLMatrix4f glLookAt(Vec3 from, Vec3 to, Vec3 up = Vec3(0, 1, 0))
    {
        return open3d::visualization::gl_util::LookAt(vec3ToEv3d(from),
                                                      vec3ToEv3d(to),
                                                      vec3ToEv3d(up));
    };
    
    // TODO very temp experiment -- OK to delete now.
    // Nickname for open3d::visualization::gl_util::LookAt() with Vec3 args.
    GLMatrix4f myLookAt()
    {
        Vec3 camI = camera().i();
        Vec3 camJ = camera().j();
        Vec3 camK = camera().k();
        Vec3 camP = camera().p();
                
        // Construct the view matrix by picking out scalar camera parameters.
        Eigen::Matrix4f eigen_view_matrix;
        eigen_view_matrix << camI.x(), camI.y(), camI.z(), camP.x(),
                             camJ.x(), camJ.y(), camJ.z(), camP.y(),
                             camK.x(), camK.y(), camK.z(), camP.z(),
                             0,        0,        0,        1;
        
        return eigen_view_matrix;
    };

    // Update the camera view. Runs "follow cam". Sets Open3d view dep on mode.
    void updateCamera()
    {
        // Invoke the "follow cam" model, update look_from / look_at points
        computeFollowCameraFromTo();
        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20241014 smooth over all 3 components of from/at/up camera

        // Update this Draw instance's camera to from/at orientation
//        camera() = camera().fromTo(cameraLookFrom(), cameraLookAt());
        camera() = camera().fromTo(cameraLookFrom(), cameraLookAt(), camera_look_up_);

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        // Either set view to from/at points or set markers in static view.
        if (cameraMode() == true)
        {
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20241014 smooth over all 3 components of from/at/up camera

//            setVisualizerViewByFromAt(cameraLookFrom(), cameraLookAt());
            setVisualizerViewByFromAt(cameraLookFrom(), cameraLookAt(), camera_look_up_);

//            from_ball->Translate({0, 1000, 0}, false);
//            to_ball->Translate({0, 1000, 0}, false);
            from_ball->Translate(vec3ToEv3d(cameraLookFrom()), false);
            to_ball->Translate(vec3ToEv3d(aimTarget()), false);
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        }
        else
        {
            from_ball->Translate(vec3ToEv3d(cameraLookFrom()), false);
            to_ball->Translate(vec3ToEv3d(cameraLookAt()), false);
        }
    }
    
    // Set the "camera view" of current Open3D visualizer according to a look at
    // specification. Optional "up" defaults to global Y direction.
    //
    // This is the work-around developed Oct 7-11, 2024 for setting an Open3D
    // "legacy" Visualizer's ViewControl using a from/at(/up) specification. It
    // might be a candidate for merging into ViewControl. If I can do what I
    // need in the new Application framework that seems better and reduces the
    // motivation to do a PR.
    //
    void setVisualizerViewByFromAt(Vec3 look_from,
                                   Vec3 look_at,
                                   Vec3 up = Vec3(0, 1, 0))
    {
        // Compute 4x4 from/at matrix.
        GLMatrix4f glla = glLookAt(look_from, look_at, up);
        Eigen::Matrix4d la_matrix = glla.cast<double>();

        // Get current PinholeCameraParameters (pcp).
        open3d::camera::PinholeCameraParameters pcp;
        visualizer().GetViewControl().ConvertToPinholeCameraParameters(pcp);
        
        // Overwrite the pcp's previous view matrix with the new look_at matrix.
        // (I am deeply puzzled by the need for that negation, but here we are.)
        pcp.extrinsic_ = -la_matrix;
        pcp.extrinsic_(3, 3) = 1;
        
        // Write back PinholeCameraParameters with new from/at view matrix.
        visualizer().GetViewControl().ConvertFromPinholeCameraParameters(pcp);
    }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241014 smooth over all 3 components of from/at/up camera
    
    
//    // Store global camera look_from / look_at points.
//    Vec3 camera_look_from_;
//    Vec3 camera_look_at_;    // TODO Now same as aimTarget(), blended later?
//    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//    // TODO 20241014 smooth over all 3 components of from/at/up camera
//    Vec3 camera_look_up_;
//    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//    // Invoke the "follow camera" model, update look_from / look_at points.
//    void computeFollowCameraFromTo()
//    {
//        double desired_offset_dist = 10;
//        double position_speed = 0.025;  // On [0:1]
//        Vec3 camera_pos = camera().p();
//        Vec3 offset_from_camera_to_target = aimTarget() - camera_pos;
//        double offset_distance = offset_from_camera_to_target.length();
//        Vec3 offset_direction = offset_from_camera_to_target / offset_distance;
//        Vec3 offset_target = aimTarget() - (offset_direction * desired_offset_dist);
//        camera_look_from_ = util::interpolate(position_speed, camera_pos, offset_target);
//        camera_look_at_ = aimTarget();
//    }

    
    
    // Low pass filter for roll control ("up" target).
    util::Blender<Vec3> up_memory_;
    
    util::Blender<Vec3> from_memory_;
    util::Blender<Vec3> at_memory_;
    

//        // Invoke the "follow camera" model, update look_from / look_at points.
//        void computeFollowCameraFromTo()
//        {
//    //            double desired_offset_dist = 10;
//    //    //        double position_speed = 0.025;  // On [0:1]
//    //    //        double position_speed = 0.1;  // On [0:1]
//    //            double position_speed = 1;  // On [0:1]
//
//            double desired_offset_dist = 15;
//            double position_speed = 1;  // On [0:1]
//
//
//            Vec3 camera_pos = camera().p();
//            Vec3 offset_from_camera_to_target = aimTarget() - camera_pos;
//            double offset_distance = offset_from_camera_to_target.length();
//            Vec3 offset_direction = offset_from_camera_to_target / offset_distance;
//            Vec3 offset_target = aimTarget() - (offset_direction * desired_offset_dist);
//
//    //        camera_look_from_ = util::interpolate(position_speed, camera_pos, offset_target);
//    //        camera_look_at_ = aimTarget();
//
//            Vec3 new_from = util::interpolate(position_speed, camera_pos, offset_target);
//            Vec3 new_at = aimTarget();
//            Vec3 new_up = camera().j();
//
//    //        from_memory_.blend(new_from, 0.95);
//    //        at_memory_.blend(new_at, 0.95);
//    //        up_memory_.blend(new_up, 0.95);
//
//    //        double b = 0.99;
//    //        double b = 0.9;
//    //            from_memory_.blend(new_from, 0.9);
//    //            at_memory_.blend(new_at, 0.9);
//    //    //        up_memory_.blend(new_up, 0.98);
//    //            up_memory_.blend(new_up, 0.99);
//
//    //        from_memory_.blend(new_from, 0.9);
//    //        at_memory_.blend(new_at, 0.95);
//    //        up_memory_.blend(new_up, 0.995);
//
//    //        from_memory_.blend(new_from, 0.9);
//    //        at_memory_.blend(new_at, 0.92);
//    //        up_memory_.blend(new_up, 0.999);
//
//    //        from_memory_.blend(new_from, 0.9);
//    //        at_memory_.blend(new_at, 0.92);
//    //        up_memory_.blend(new_up, 0.99);
//
//    //        from_memory_.blend(new_from, 0.9);
//    //        at_memory_.blend(new_at, 0.85);
//    //        up_memory_.blend(new_up, 0.99);
//
//            from_memory_.blend(new_from, 0.90);
//            at_memory_.blend(new_at,     0.80);
//            up_memory_.blend(new_up,     0.97);
//
//            camera_look_from_ = from_memory_.value;
//            camera_look_at_ = at_memory_.value;
//            camera_look_up_ = up_memory_.value.normalize();
//
//        }

//        // Invoke the "follow camera" model, update look_from / look_at points.
//        void computeFollowCameraFromTo()
//        {
//            double desired_offset_dist = 15;
//    //        double position_speed = 1;  // On [0:1]
//
//
//            Vec3 camera_pos = camera().p();
//            Vec3 offset_from_camera_to_target = aimTarget() - camera_pos;
//            double offset_distance = offset_from_camera_to_target.length();
//            Vec3 offset_direction = offset_from_camera_to_target / offset_distance;
//            Vec3 offset_target = aimTarget() - (offset_direction * desired_offset_dist);
//
//    //        Vec3 new_from = util::interpolate(position_speed, camera_pos, offset_target);
//            Vec3 new_from = offset_target;
//
//            Vec3 new_at = aimTarget();
//    //        Vec3 new_up = camera().j();
//    //        Vec3 new_up = (camera().j() + Vec3(0, 0.1, 0)).normalize();
//            Vec3 new_up = (camera().j() + Vec3(0, 0.3, 0)).normalize();
//
//            from_memory_.blend(new_from, 0.90);
//            at_memory_.blend(new_at,     0.80);
//            up_memory_.blend(new_up,     0.97);
//
//            camera_look_from_ = from_memory_.value;
//            camera_look_at_ = at_memory_.value;
//            camera_look_up_ = up_memory_.value.normalize();
//        }

    // Invoke the "follow camera" model, update look_from / look_at points.
    //
    // TODO: this is called from exactly one place, where it is immediately
    // followed by setting the camera() to the transform we compute here.
    // Shouldn't those two steps be combined here? Then it could be called just
    // plain Draw::computeFollowCamera().
    //
    void computeFollowCameraFromTo()
    {
        double desired_offset_dist = 15;
        Vec3 camera_pos = camera().p();
        Vec3 offset_from_camera_to_target = aimTarget() - camera_pos;
        double offset_distance = offset_from_camera_to_target.length();
        Vec3 offset_direction = offset_from_camera_to_target / offset_distance;
        Vec3 offset_target = aimTarget() - (offset_direction * desired_offset_dist);
        Vec3 new_from = offset_target;
        Vec3 new_at = aimTarget();
        Vec3 new_up = (camera().j() + Vec3(0, 0.3, 0)).normalize();
        camera_look_from_ = from_memory_.blend(new_from, 0.90);
        camera_look_at_   =   at_memory_.blend(new_at,   0.80);
        camera_look_up_   =   up_memory_.blend(new_up,   0.97).normalize();
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Accessor for Open3D Visualizer instance.
    vis_t& visualizer() { return visualizer_; }
    const vis_t& visualizer() const { return visualizer_; }
    
    // Accessor for camera object, represented as a LocalSpace.
    LocalSpace& camera() { return camera_; }
    const LocalSpace& camera() const { return camera_; }
    
    // Accessor for camera aim target posItion.
    Vec3& aimTarget() { return aim_target_; }
    const Vec3& aimTarget() const { return aim_target_; }
    
    // Accessors for look_from / look_at points.
    Vec3 cameraLookFrom() const { return camera_look_from_; }
    Vec3 cameraLookAt() const { return camera_look_at_; }
    
    // Settable accessor for  camera mode. Now cycles between follow and global.
    bool& cameraMode() { return camera_mode_; }
    void nextCameraMode() { camera_mode_ = not camera_mode_; }
    
    static void unit_test() {}
    
private:
    // Pointer to main global drawing context.
    static inline Draw* global_object_ = nullptr;
    
    // Runtime switch to turn graphical display on and off.
    bool enable_ = false;

    // Count all triangles drawn during graphics session.
    int triangle_count_ = 0;
    
    // Representation of geometric camera state. (Maybe should be Agent type?)
    LocalSpace camera_;
    
    // Camera mode. Currently cycles between follow and global.
    bool camera_mode_ = true;  // Follow mode.

    // Aim target for follow camera. Set externally (eg to selected boid).
    Vec3 aim_target_;

    // Store global camera look_from / look_at points.
    Vec3 camera_look_from_;
    Vec3 camera_look_at_;    // TODO Now same as aimTarget(), blended later?
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241014 smooth over all 3 components of from/at/up camera
    Vec3 camera_look_up_;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


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
