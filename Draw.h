//-------------------------------------------------------------------------------
//
//  Draw.h -- new flock experiments
//
//  Graphics utilities for evoflock based on Open3D. A Draw instance provides an
//  (optional) graphics context and utilities. Draw is a "singleton" class, for
//  which only one instance exists, accessed with Draw::getInstance(). Creating
//  a Draw object creates an Open3D visualizer object, an associated window, and
//  provides API for adding/modifying static and animated geometry in the scene
//  displayed in the window.
//
//  Created by Craig Reynolds on September 9, 2024.
//  MIT License -- Copyright © 2024 Craig Reynolds
//-------------------------------------------------------------------------------

#pragma once

#include "Agent.h"

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
    
    // shared pointers to Open3D TriangleMesh objects
    typedef open3d::geometry::TriangleMesh tri_mesh_t;
    typedef std::shared_ptr<tri_mesh_t> sp_tri_mesh_t;

#endif  // USE_OPEN3D

    // Used to get the current global Draw object (drawing context).
    // Returns the existing instance, creating a new one if needed.
    // Asserts that getInstance() must not be called in constructor.
    static Draw& getInstance() { return getInstance(true); }
    static Draw& getInstance(bool enabled)
    {
        assert(not global_object_under_construction_);
        if (not global_object_)
        {
            global_object_under_construction_ = true;
            global_object_ = new Draw(enabled);
            global_object_under_construction_ = false;
        }
        return *global_object_;
    }

private:

    // TODO 20241026 -- Temporary for debugging odd symptom: running from inside
    // Xcode launches a separate copy of the executable about a second after the
    // first. Oddly this symptom seems to come and go as I edit main.cpp
    static inline int constructor_count = 0;
    
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
        // TODO 20250117 temp override to put viewer in lower right corner
        Vec3 cwr_screen_size(3456, 2234, 0);
        window_xy_size = {1800, 1200, 0};
        window_xy_position_ul = cwr_screen_size - window_xy_size ;

        // TODO 20241026 -- Temporary for debugging odd symptom: running from inside
        // Xcode launches a separate copy of the executable about a second after the
        // first. Oddly this symptom seems to come and go as I edit main.cpp
        constructor_count++;
        assert(constructor_count == 1);
        
        setEnable(enabled);

#ifdef USE_OPEN3D
        std::cout << "Begin Open3D (" << OPEN3D_VERSION;
        std::cout << ") graphics session." << std::endl;

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
        // (See also replacement/work-around: addThickLineToAnimatedFrame())
        visualizer().GetRenderOption().line_width_ = line_width;
        visualizer().GetRenderOption().point_size_ = point_size;
        // Callbacks for mouse gestures and single key commands.
        setupGuiCallbacks();
        // Initialize camera parameters with default offset distance
        resetCameraView();
#endif  // USE_OPEN3D
    }

public:

    ~Draw()
    {
        global_object_ = nullptr;
        std::cout << "End graphics session. Total triangles drawn: ";
        std::cout << triangle_count_ << "." << std::endl;
    }

    // Called to begin an animated scene, typically composed of many frames.
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

    // Called to end an animated scene.
    void endAnimatedScene() { if (enable()) { } }

    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20250801 hack version of "space-time worms"
    
//    // Called to begin each animation frame.
//    void beginOneAnimatedFrame()
//    {
//        poll_events_count_ = 0;
//        if (enable()) { clearAnimatedGeometryFromScene(); }
//    }

    // Called to begin each animation frame.
    void beginOneAnimatedFrame()
    {
        poll_events_count_ = 0;
//        if (enable()) { clearAnimatedGeometryFromScene(); }
        if (enable() and not getWorms()) { clearAnimatedGeometryFromScene(); }
    }

    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20250702 yet another slow frame draw symptom

//    // Called to end each animation frame.
//    void endOneAnimatedFrame()
//    {
//        if (enable())
//        {
//            updateCamera();
//            redrawScene();
//            result_from_last_poll_events_call_ = pollEvents();
//            if (poll_events_count_ > 1) { debugPrint(poll_events_count_); }
//        }
//    }
    
    
    int xxx_temp_frame_count = 0;


    // Called to end each animation frame.
    void endOneAnimatedFrame()
    {
        if (enable())
        {
            updateCamera();
            redrawScene();
            
//            xxx_temp_frame_count++;
//            if ((xxx_temp_frame_count % 10) == 0)
//            {
//                result_from_last_poll_events_call_ = pollEvents();
//            }

            result_from_last_poll_events_call_ = pollEvents();

            if (poll_events_count_ > 1) { debugPrint(poll_events_count_); }
        }
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Clear all animated geometry to begin building a new scene.
    void clearAnimatedGeometryFromScene()
    {
#ifdef USE_OPEN3D
        animated_tri_mesh_->Clear();
        animated_line_set_->Clear();
#endif  // USE_OPEN3D
    }

    // Redraw scene to reflect scene, camera, or animation changes.
    void redrawScene()
    {
#ifdef USE_OPEN3D
        animated_tri_mesh_->ComputeVertexNormals();
        visualizer().UpdateGeometry(animated_tri_mesh_);
        visualizer().UpdateGeometry(animated_line_set_);
#endif  // USE_OPEN3D
    }

    // Add to per-frame collection of animating triangles: single color.
    // Typically used to add a small polyhedron (a boid's body) to the scene.
    void addTriMeshToAnimatedFrame(const std::vector<Vec3>& vertices,
                                   const std::vector<std::size_t>& triangles,
                                   const Color& color)
    {
        std::vector<Color> colors(vertices.size(), color);
        addTriMeshToAnimatedFrame(vertices, triangles, colors);
    }
    
    // Add to per-frame collection of animating triangles: per-vertex colors.
    void addTriMeshToAnimatedFrame(const std::vector<Vec3>& vertices,
                                   const std::vector<std::size_t>& triangles,
                                   const std::vector<Color>& colors)
    {
#ifdef USE_OPEN3D
        if (enable())
        {
            assert(triangles.size() % 3 == 0);
            assert(vertices.size() == colors.size());
            for (auto t : triangles){assert((t >= 0) and (t < triangles.size()));}
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
                animated_tri_mesh_->vertex_colors_.push_back({c.r(), c.g(), c.b()});
            }
            // Count all triangles drawn during graphics session.
            triangle_count_ += triangles.size() / 3;
        }
#endif  // USE_OPEN3D
    }
    
    // Add to per-frame collection of animating line segments for annotation.
    void addLineSegmentToAnimatedFrame(const Vec3& endpoint0,
                                       const Vec3& endpoint1,
                                       const Color& color)
    {
#ifdef USE_OPEN3D
        if (enable() and enableAnnotation())
        {
            auto s = animated_line_set_->points_.size();
            animated_line_set_->points_.push_back(vec3ToEv3d(endpoint0));
            animated_line_set_->points_.push_back(vec3ToEv3d(endpoint1));
            animated_line_set_->lines_.push_back({s, s + 1});
            animated_line_set_->colors_.push_back(vec3ToEv3d(color));
        }
#endif  // USE_OPEN3D
    }
        
    // Adds an animated “thick line” (aka cylinder) to the current frame.
    void addThickLineToAnimatedFrame(const Vec3& endpoint0,
                                     const Vec3& endpoint1,
                                     const Color& color,
                                     double radius = 0.02)
    {
        if (enable() and enableAnnotation())
        {
            addCylinderToAnimatedFrame(endpoint0, endpoint1, color, radius);
        }
    }
    
    // This is a "thick" version of addLineSegmentToAnimatedFrame().
    // (Converted from python code in Flock::Draw.py)
    void addCylinderToAnimatedFrame(const Vec3& endpoint0,
                                    const Vec3& endpoint1,
                                    const Color& color,
                                    double radius,
                                    int sides = 5)
    {
        // Vector along the segment, from v1 to v2
        Vec3 offset = endpoint1 - endpoint0;
        double distance = offset.length();
        if (distance > 0)
        {
            Vec3 tangent = offset / distance;
            Vec3 basis1 = tangent.find_perpendicular();
            Vec3 basis2 = tangent.cross(basis1);
            // Make transform from "line segment space" to global space.
            LocalSpace ls(basis1, basis2, tangent, endpoint0);
            double angle_step = util::pi * 2 / sides;
            Vec3 r(radius, 0, 0);
            for (int i = 0; i < sides; i++)
            {
                Vec3 a = ls.globalize(r.rotate_xy_about_z(angle_step * i));
                Vec3 b = ls.globalize(r.rotate_xy_about_z(angle_step * (i+1)));
                Vec3 c = b + offset;
                Vec3 d = a + offset;
                addTriMeshToAnimatedFrame({a, b, c, d}, {3,1,0, 3,2,1}, color);
            }
        }
    }

    // Clear all previous TriangleMesh objects from static scene.
    void clearStaticScene()
    {
        if (enable())
        {
            for (sp_tri_mesh_t tm : static_tri_meshes_)
            {
                visualizer().RemoveGeometry(tm, false);
            }
            static_tri_meshes_.clear();
        }
    }

    // Add a TriangleMesh objects to the static (non-animating) scene.
    void addTriMeshToStaticScene(const sp_tri_mesh_t tri_mesh)
    {
        static_tri_meshes_.push_back(tri_mesh);
        visualizer().AddGeometry(tri_mesh);
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241221 square panels on cylinders
    
//    // Make cylinder: returns shared pointer to tri mesh with given parameters.
//    static sp_tri_mesh_t constructCylinderTriMesh(double radius,
//                                                  const Vec3& endpoint0,
//                                                  const Vec3& endpoint1,
//                                                  const Vec3& color,
//                                                  bool compute_normals,
//                                                  bool evert,
//                                                  int subdivision)
//    {
//        auto cyl_mesh = constructO3dCylinder(radius,
//                                             vec3ToEv3d(endpoint0),
//                                             vec3ToEv3d(endpoint1),
//                                             subdivision);
//        cyl_mesh->PaintUniformColor(vec3ToEv3d(color));
//        if (compute_normals) { cyl_mesh->ComputeVertexNormals(); }
//        if (evert) { evertTriangleMesh(*cyl_mesh); }
//        return cyl_mesh;
//    }

    // Make cylinder: returns shared pointer to tri mesh with given parameters.
    static sp_tri_mesh_t constructCylinderTriMesh(double radius,
                                                  const Vec3& endpoint0,
                                                  const Vec3& endpoint1,
//                                                  const Vec3& color,
                                                  const Color& color,
                                                  bool compute_normals,
                                                  bool evert,
                                                  int subdivision)
    {
        double height = (endpoint0 - endpoint1).length();
//        double circumference = 2 * lp::util::pi * radius;
//        double circumference = 2 * 3.14159 * radius;
        double circumference = 2 * util::pi * radius;
        double chord = circumference / subdivision;
        auto cyl_mesh = constructO3dCylinder(radius,
                                             vec3ToEv3d(endpoint0),
                                             vec3ToEv3d(endpoint1),
//                                             subdivision);
                                             subdivision,
                                             
//                                             height / 100
//                                             height
//                                             height * 3
//                                             height * 4
//                                             height / chord
                                             std::max(1.0, height / chord)

                                             );
        cyl_mesh->PaintUniformColor(vec3ToEv3d(color));
        if (compute_normals) { cyl_mesh->ComputeVertexNormals(); }
        if (evert) { evertTriangleMesh(*cyl_mesh); }
        return cyl_mesh;
    }

    // Make sphere: returns shared pointer to tri mesh with given parameters.
    static sp_tri_mesh_t constructSphereTriMesh(double radius,
                                                const Vec3& center,
                                                const Color& color,
                                                bool compute_normals,
                                                bool evert,
                                                int subdivision)
    {
        auto mesh = tri_mesh_t::CreateSphere(radius, subdivision);
        mesh->Translate(vec3ToEv3d(center));
        mesh->PaintUniformColor(vec3ToEv3d(color));
        if (compute_normals) { mesh->ComputeVertexNormals(); }
        if (evert) { evertTriangleMesh(*mesh); }
        return mesh;
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    // Runtime switch to turn graphical display on and off.
    bool enable() const { return enable_ and not exitFromRun(); }
    void setEnable(bool e) { enable_ = e; }
    void toggleEnable() { enable_ = not enable_; }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20250702 yet another slow frame draw symptom

//    // Process asynch events: key presses, mouse moves, closing window, etc.
//    bool pollEvents()
//    {
//        util::Timer t;
//        bool pe = visualizer().PollEvents();
//        poll_events_count_++;
//        return pe;
//    }


    // Process asynch events: key presses, mouse moves, closing window, etc.
    bool pollEvents()
    {
        util::Timer t;
        bool pe = visualizer().PollEvents();
        poll_events_count_++;
        
        if (t.elapsedSeconds() > 0.1) { debugPrint(t.elapsedSeconds()) }
        
        return pe;
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Update the camera view. Runs "follow cam". Sets Open3d view dep on mode.
    void updateCamera()
    {
        // Invoke the current camera motion model.
        animateCamera();
        // Set camera state in underlying graphics system (Open3d).
        setVisualizerViewByFromAt();
        // Draw circle around aimTarget() pointing toward camera.
        addBlackAndWhiteCircularReticle(aimAgent().position());
    }

    // Draw a B&W circle at given center pointing at the current camera.
    void addBlackAndWhiteCircularReticle(Vec3 center)
    {
        addBlackAndWhiteCircularReticle(center, 0.75, 30);
    }
    void addBlackAndWhiteCircularReticle(Vec3 center,
                                         double radius,
                                         double chords)
    {
        bool c = false;
        double angle = 2 * M_PI / chords;
        Vec3 up = camera().j();
        LocalSpace ls = LocalSpace::fromTo(center, cameraLookFrom(), up);
        Vec3 lup = ls.localize(up);
        Vec3 local_spoke = Vec3(lup.x(), lup.y(), 0).normalize() * radius;
        for (int i = 0; i < chords; i++)
        {
            Vec3 new_spoke = local_spoke.rotate_xy_about_z(angle);
            addLineSegmentToAnimatedFrame(ls.globalize(local_spoke),
                                          ls.globalize(new_spoke),
                                          c ? Color::white() : Color::black());
            local_spoke = new_spoke;
            c = not c;
        }
    }

    // Set the "camera view" of current Open3D visualizer according to the
    // from-at-up values of this Draw instance, set by animateCamera().
    void setVisualizerViewByFromAt()
    {
        setVisualizerViewByFromAt(cameraLookFrom(),
                                  cameraLookAt(),
                                  cameraLookUp());
    }

    // Set the "camera view" of current Open3D visualizer according to a look/at
    // specification. Optional "up" defaults to global Y direction.
    void setVisualizerViewByFromAt(Vec3 look_from,
                                   Vec3 look_at,
                                   Vec3 up = Vec3(0, 1, 0))
    {
        setOpen3DVisualizerViewByFromAt(visualizer(),
                                        vec3ToEv3d(look_from),
                                        vec3ToEv3d(look_at),
                                        vec3ToEv3d(up));
    }
    
    // Move the camera, according to its mode, usually to track an agent.
    void animateCamera()
    {
        if (isFollowCameraMode()) { animateFollowCamera(); }
        if (isWingmanCameraMode()) { animateWingmanCamera(); }
        if (isStaticCameraMode()) { animateStaticCamera(); }
    }

    // Invoke the "follow camera" model, update look_from/look_at/up and camera.
    void animateFollowCamera()
    {
        Vec3 camera_pos = cameraLookFrom();
        Vec3 offset_from_camera_to_target = aimAgent().position() - camera_pos;
        Vec3 offset_direction = offset_from_camera_to_target.normalize_or_0();
        Vec3 target_offset = offset_direction * cameraDesiredOffsetDistance();
        Vec3 new_from = aimAgent().position() - target_offset;
        Vec3 new_at = aimAgent().position();
        Vec3 new_up = (camera().j() + Vec3(0, 0.3, 0)).normalize();
        blendInNewCameraLookFrom(new_from);
        blendInNewCameraLookAt(new_at);
        blendInNewCameraLookUp(new_up);
        setCameraByFromAtUp();
    }

    // Invoke the "wingman camera" model, update look_from/look_at/up and camera.
    void animateWingmanCamera()
    {
        Vec3 global_cam = aimAgent().ls().globalize(wingman_cam_local_offset_);
        blendInNewCameraLookFrom(global_cam);
        blendInNewCameraLookAt(aimAgent().position());
        blendInNewCameraLookUp(aimAgent().up());
        setCameraByFromAtUp();
    }

    // These blend a new camera look-from-at-up into the current state.
    // Each have a default blend rate which can be overridden.
    void blendInNewCameraLookFrom(const Vec3& new_from, double rate = 0.95)
    {
        cameraLookFrom() = from_memory_.blend(new_from, rate);
    }
    void blendInNewCameraLookAt(const Vec3& new_at, double rate = 0.9)
    {
        cameraLookAt() = at_memory_.blend(new_at, rate);
    }
    void blendInNewCameraLookUp(const Vec3& new_up, double rate = EF::roll_rate)
    {
        cameraLookUp() = up_memory_.blend(new_up.normalize(), rate).normalize();
    }
    
    // Adjust "static camera" model according to mouse input.
    void animateStaticCamera()
    {
        setCameraByFromAtUp();
    }

    // Reset camera to aligned view of whole scene.
    void resetCameraView()
    {
        resetCameraView(camera_desired_offset_dist_default_);
    };
    
    // Reset camera to aligned view of whole scene from given offset distance.
    void resetCameraView(double offset_distance)
    {
        cameraLookUp()   = Vec3(0, 1, 0);
        cameraLookAt()   = Vec3(0, 0, 0);
        cameraLookFrom() = Vec3(0, 0, offset_distance);
        cameraDesiredOffsetDistance() = offset_distance;
    };

    // Adjust "static camera" model according to mouse input.
    void setCameraByFromAtUp()
    {
        cameraLookFrom() = Vec3::adjustSegLength(cameraLookFrom(),
                                                 cameraLookAt(),
                                                 cameraDesiredOffsetDistance());
        camera() = LocalSpace::fromTo(cameraLookFrom(),
                                      cameraLookAt(),
                                      cameraLookUp());
    }

    // Set mouse scroll wheel handler for GUI.
    void setMouseScrollCallback()
    {
        auto mscb = [&](base_vis_t* vis, double x, double y)
        {
            // Change follow distance.
            double adjust_speed = 0.8;
            double min = 0.05;
            double d = cameraDesiredOffsetDistance() + (y * adjust_speed);
            cameraDesiredOffsetDistance() = std::max(min, d);
            return false;
        };
        visualizer().RegisterMouseScrollCallback(mscb);
    }

    // Set mouse move handler for GUI.
    void setMouseMoveCallback()
    {
        std::function<bool(base_vis_t *, double, double)> mmcb = nullptr;
        {
            mmcb = [&](base_vis_t* vis, double x, double y)
            {
                Vec3 new_pos_pixels(x, y, 0);
                Vec3 offset_pixels = mouse_pos_pixels_ - new_pos_pixels;
                double mouse_move_pixels = offset_pixels.length();
                if (left_mouse_button_down_ and (mouse_move_pixels < 50))
                {
                    double speed = 0.01;
                    double nx = offset_pixels.x() * speed;
                    double ny = offset_pixels.y() * speed;
                    // offset from look-at to look-from
                    Vec3 offset = cameraLookFrom() - cameraLookAt();
                    Vec3 unit_offset = offset.normalize();
                    Vec3 new_look_from = (unit_offset +
                                          (camera().i() * nx) +
                                          (camera().j() * ny));
                    Vec3 restore_dist = (new_look_from.normalize() *
                                         cameraDesiredOffsetDistance());
                    // Record newly computed "look from".
                    cameraLookFrom() = restore_dist + cameraLookAt();
                    // Copy "look up" of current camera.
                    cameraLookUp() = camera().j();
                    // Transform new cam pos into local space of aimAgent()
                    Vec3 local_cam = aimAgent().ls().localize(cameraLookFrom());
                    wingman_cam_local_offset_ = local_cam;
                }
                mouse_pos_pixels_ = new_pos_pixels;
                return false;
            };
        }
        visualizer().RegisterMouseMoveCallback(mmcb);
    }

    // Set mouse button handler for GUI.
    void setMouseButtonCallback()
    {
        auto mbcb = [&](base_vis_t *, int button, int action, int mods)
        {
            if (button == 0) { left_mouse_button_down_ = action; }
            return false;
        };
        visualizer().RegisterMouseButtonCallback(mbcb);
    }

    // Set mouse move/scroll/button handlers for GUI.
    void setMouseCallbacks()
    {
        setMouseMoveCallback();
        setMouseScrollCallback();
        setMouseButtonCallback();
    }

    // TODO 20241225 mock up mouse position for camera position control
    Vec3 mouse_pos_pixels_;

    // Accessor for Open3D Visualizer instance.
    vis_t& visualizer() { return visualizer_; }
    const vis_t& visualizer() const { return visualizer_; }
    
    // Accessor for camera object, represented as a LocalSpace.
    LocalSpace& camera() { return camera_; }
    const LocalSpace& camera() const { return camera_; }
    
    // Accessor for camera aim target agent.
    Agent& aimAgent() { return aim_agent_; }
    const Agent& aimAgent() const { return aim_agent_; }

    // Writeable accessors for look_from / look_at points.
    Vec3& cameraLookFrom() { return camera_look_from_; }
    Vec3& cameraLookAt() { return camera_look_at_; }
    Vec3& cameraLookUp() { return camera_look_up_; }
    Vec3 cameraLookFrom() const { return camera_look_from_; }
    Vec3 cameraLookAt() const { return camera_look_at_; }
    Vec3 cameraLookUp() const { return camera_look_up_; }

    // Settable accessor for  camera mode. Now cycles between follow and global.
    int& cameraMode() { return camera_mode_; }
    int cameraMode() const { return camera_mode_; }
    void nextCameraMode() { camera_mode_ = (camera_mode_ + 1) % camera_mode_max_; }
    bool isStaticCameraMode() const { return (cameraMode() == 0); }
    bool isFollowCameraMode() const { return (cameraMode() == 1); }
    bool isWingmanCameraMode() const { return (cameraMode() == 2); }

    // Set to true when user types ESC or closes Visualizer window.
    bool exitFromRun() const { return exit_from_run_; }
    void setExitFromRun(bool efr) { exit_from_run_ = efr; }
    
    // Settable accessor for desired offset distance in follow camera mode.
    double& cameraDesiredOffsetDistance()
    {
        return camera_desired_offset_dist_;
    }
    
    // Based on pause/play and single step. Called once per frame from main loop.
    bool runSimulationThisFrame()
    {
        bool ok_to_run = true;
        if (enable())
        {
            setExitFromRun(! result_from_last_poll_events_call_);
            ok_to_run = getSingleStepMode() or not simPause();
            setSingleStepMode(false);
        }
        return ok_to_run;
    }
    
    // Single step mode means simulation should take one step, then pause again.
    bool& getSingleStepMode() { return single_step_mode_; }
    bool getSingleStepMode() const { return single_step_mode_; }
    void setSingleStepMode(bool ssm = true)
    {
        single_step_mode_ = ssm;
        if (single_step_mode_) { sim_pause_ = true; }
    }

    // Runtime switch which the simulation can query to pause itself.
    bool& simPause() { return sim_pause_; }
    bool simPause() const { return sim_pause_; }
    void toggleSimPause() { sim_pause_ = not sim_pause_; }

    // Runtime counter the simulation can use to change predefined obs sets.
    // (TODO this seems very specific to flock simulation is it OK to be here?)
    int& obstacleSetIndex() { return obstacle_set_index_; }
    int obstacleSetIndex() const { return obstacle_set_index_; }
    void nextObstacleSet() { obstacle_set_index_ += 1; }
    
    // Runtime counter the simulation can use to change selected boid.
    int& selectedBoidIndex() { return selected_boid_index_; }
    int selectedBoidIndex() const { return selected_boid_index_; }
    void selectNextBoid() { selected_boid_index_ += 1; }
    
    // "B" cmd runs a sim, with best individual & graphics, then reset.
    bool getVisBestMode() const { return vis_best_mode_; }
    void setVisBestMode() { vis_best_mode_ = true; }
    void clearVisBestMode() { vis_best_mode_ = false; }
    
    // "A" command toggles drawing of annotation lines, etc.
    void toggleAnnotation() { enable_annotation_ = not enable_annotation_; }
    bool enableAnnotation() const
    {
        return enable_annotation_ and not isStaticCameraMode();
    }

    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20250801 hack version of "space-time worms"
    
    // "W" command toggles "space-time worms".
    bool enable_worms_ = false;
    bool getWorms() const { return enable_worms_; }
    void toggleWorms() { enable_worms_ = not enable_worms_; }

    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

    // Called in constructor to set up the various key cmd and mouse callbacks.
    void setupGuiCallbacks()
    {
        // If "rv" is true, requests redraw after key command callback.
        bool rv = false;
        // Abbreviated name for unused Visualizer class.
        typedef base_vis_t vis;
        // Register a key command.
        auto rk = [&](int key,std::function<bool(vis *)> callback)
        {
            visualizer().RegisterKeyCallback(key, callback);
        };

        // Add single key command callback to toggle "graphics mode"
        rk('G', [&](vis* v) { toggleEnable(); return rv; });
        
        // Add "C" command, to cycle through camera aiming modes.
        rk('C', [&](vis* v) { nextCameraMode(); return rv; });
        
        // Add " " (space) command, toggles public pause simulation flag.
        rk(' ', [&](vis* v) { toggleSimPause(); return rv; });
        
        // Add "O" command, to increment obstacle set counter.
        rk('O', [&](vis* v) { nextObstacleSet(); return rv; });
        
        // Add "1" command, to set single step mode.
        rk('1', [&](vis* v) { setSingleStepMode(); return rv; });
        
        // Add "S" command, to cycle selected boid through flock.
        rk('S', [&](vis* v) { selectNextBoid(); return rv; });
        
        // "A" command toggles drawing of annotation lines, etc.
        rk('A', [&](vis* v) { toggleAnnotation(); return rv; });

        // Add "R" command, reset camera to aligned view of whole scene.
        rk('R', [&](vis* v) { resetCameraView(100); return rv; });

        // Add "B" cmd runs a sim, with best individual & graphics, then reset.
        rk('B', [&](vis* v) { setVisBestMode(); return rv; });
        
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
        // TODO 20250801 hack version of "space-time worms"
        
        // Add "W" cmd to toggle "space-time worms".
        rk('W', [&](vis* v) { toggleWorms(); return rv; });
        
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~


        // Set mouse move/scroll/button handlers.
        setMouseCallbacks();
    }

    // Set a random per-vertex color brightness (grayscale) for given mesh.
    static void brightnessSpecklePerVertex(double min_brightness,
                                           double max_brightness,
                                           Color base_color,
                                           Draw::sp_tri_mesh_t mesh)
    {
        for (int i = 0; i < mesh->vertices_.size(); i++)
        {
            double b = EF::RS().random2(min_brightness, max_brightness);
            mesh->vertex_colors_.at(i) = vec3ToEv3d(base_color * b);
        }
    };

    static void unit_test() {}
    
private:
    // Pointer to main global drawing context.
    static inline Draw* global_object_ = nullptr;
    static inline bool global_object_under_construction_ = false;
    
    // Runtime switch to turn graphical display on and off.
    bool enable_ = false;

    // Count all triangles drawn during graphics session.
    int triangle_count_ = 0;
    
    // Representation of geometric camera state. (Maybe should be Agent type?)
    LocalSpace camera_;
    
    // Camera mode. Currently cycles between follow, wingman, and global.
    int camera_mode_ = 1;  // Follow mode.
    int camera_mode_max_ = 3;

    // Aim agent for camera tracking modes. Set externally, eg to selected boid.
    Agent aim_agent_;

    // Desired offset distance between camera and "look at" (target/aim) point.
    double camera_desired_offset_dist_default_ = 15;
    double camera_desired_offset_dist_ = camera_desired_offset_dist_default_;

    // Store global camera look from/to/up vectors.
    Vec3 camera_look_from_;
    Vec3 camera_look_at_;
    Vec3 camera_look_up_;

    // Low pass filter for camera look from/to/up vectors.
    util::Blender<Vec3> up_memory_;
    util::Blender<Vec3> from_memory_;
    util::Blender<Vec3> at_memory_;

    // Set to true when user types ESC or closes Visualizer window.
    bool exit_from_run_ = false;

    // Runtime switch which the simulation can query to pause itself.
    bool sim_pause_ = false;
    
    // Simulation will take one step, then reenter pause mode.
    bool single_step_mode_ = false;
    
    // Runtime counter the simulation can use to change predefined obs sets.
    // (TODO this seems very specific to flock simulation is it OK to be here?)
    int obstacle_set_index_ = 0;

    // Runtime counter the simulation can use to change selected boid.
    int selected_boid_index_ = 0;

    // Set "from" as a local offset (right, up, and behind) relative to target.
    Vec3 wingman_cam_local_offset_ = Vec3(1, 3, -6).normalize();

    // True when the left mouse button currently depressed.
    bool left_mouse_button_down_ = false;
    
    // "B" command means to run a sim, with graphics, then reset, based on this.
    bool vis_best_mode_ = false;
    
    // "A" command toggles drawing of annotation lines, etc.
    bool enable_annotation_ = true;
    
    // Used to ensure that pollEvents() is called only once per frame.
    int poll_events_count_ = 0;

    // Used to cache value returned by the one pollEvents() call per frame.
    bool result_from_last_poll_events_call_ = true;

#ifdef USE_OPEN3D
    
    // Open3D TriangleMesh object for storing and drawing animated triangles.
    sp_tri_mesh_t animated_tri_mesh_ = nullptr;

    // Open3D LineSet object for storing and drawing animated line segments.
    std::shared_ptr<open3d::geometry::LineSet> animated_line_set_ = nullptr;
    
    // Collection of static scene geometry, such as obstacles, represented as
    // shared pointers to Open3D TriangleMesh objects, aka sp_tri_mesh_t.
    std::vector<sp_tri_mesh_t> static_tri_meshes_;
    
    // Open3D Visualizer object.
    vis_t visualizer_;
    
    // Private utility to convert a Vec3 to an Eigen::Vector3d.
    static Eigen::Vector3d vec3ToEv3d(const Vec3& v)
    {
        return Eigen::Vector3d(v.x(), v.y(), v.z());
    }
    
    // Private utility to convert an Eigen::Vector3d to an Vec3.
    static Vec3 ev3dtoVec3(const Eigen::Vector3d& v)
    {
        return {v[0], v[1], v[2]};
    }
    
    // Construct an Eigen::Matrix4d from the 16 scalars in a LocalSpace.
    static Eigen::Matrix4d lsToEigenMatrix4D(const LocalSpace& ls)
    {
        Eigen::Matrix4d matrix;
        for (int i = 0; i < 16; i++) { matrix(i / 4, i % 4) = ls[i]; }
        return matrix;
    };

    //--------------------------------------------------------------------------
    // These are utilities meant to connect evoflock's way of doing things to
    // Open3D's way of doing things. They can continue to live here, or they
    // could potentially be moved to Open3D. Each of these is a potential
    // "first pull request" as a way for me (Craig) to get more involved with
    // Open3D maintenance.
    
    // TODO 20240911 try evert TriangleMesh.
    // Flip orientation of each tri in a triangle mesh (destructively modifies).
    // (pr for Open3D? See: https://github.com/isl-org/Open3D/discussions/6419)
    static void evertTriangleMesh(open3d::geometry::TriangleMesh& tri_mesh)
    {
        for (auto& triangle : tri_mesh.triangles_)
        {
            std::reverse(std::begin(triangle), std::end(triangle));
        }
    }
    
    // TODO 20241016 experimental "Open3D style" version
    //
    // Similar to open3d::visualization::visualizer::O3DVisualizer::SetupCamera()
    // but does not set fov.
    //
    // This is the work-around developed Oct 7-11, 2024 for setting an Open3D
    // "legacy" Visualizer's ViewControl using a from/at(/up) specification. It
    // might be a candidate for merging into ViewControl. If I can do what I
    // need in the new Application framework that seems better and reduces the
    // motivation to do a PR.
    //
    static
    void setOpen3DVisualizerViewByFromAt(open3d::visualization::Visualizer& vis,
                                         const Eigen::Vector3d& look_from,
                                         const Eigen::Vector3d& look_at,
                                         const Eigen::Vector3d& up = {0, 1, 0})
    {
        // Compute 4x4 from/at matrix.
        using namespace open3d::visualization::gl_util;
        Eigen::Matrix4d la_matrix = LookAt(look_from, look_at, up).cast<double>();
        
        // Get current PinholeCameraParameters.
        open3d::camera::PinholeCameraParameters pcp;
        vis.GetViewControl().ConvertToPinholeCameraParameters(pcp);
        
        // Overwrite the pcp's previous view matrix with the new look_at matrix.
        // (I am deeply puzzled by the need for that negation, but here we are.)
        pcp.extrinsic_ = -la_matrix;
        pcp.extrinsic_(3, 3) = 1;
        
        // Write back PinholeCameraParameters with new from/at view matrix.
        vis.GetViewControl().ConvertFromPinholeCameraParameters(pcp);
    }

    // An "Open3D native style" constructO3dCylinder from two endpoints
    static sp_tri_mesh_t constructO3dCylinder(double radius,
                                              const Eigen::Vector3d& endpoint0,
                                              const Eigen::Vector3d& endpoint1,
                                              int resolution = 20,
                                              int split = 4,
                                              bool create_uv_map = false)
    {
        // Get cylinder height and ensure it is not zero.
        double height = (endpoint1 - endpoint0).norm();
        assert(height > 0);
        Eigen::Vector3d axis = (endpoint1 - endpoint0) / height;

        // Use Open3D utility to create tri mesh with given radius, height, etc.
        using namespace open3d::geometry;
        auto cylinder = TriangleMesh::CreateCylinder(radius, height, resolution,
                                                     split, create_uv_map);
        
        // That cylinder is centered on Z axis. Move one endpoint to origin.
        cylinder->Translate({0, 0, -height / 2});
        
        // Pick an "up" vector which is "least parallel" to cylinder axis
        Eigen::Vector3d a(0, 1, 0);
        Eigen::Vector3d b(1, 0, 0);
        auto up = (std::abs(axis.dot(a)) < std::abs(axis.dot(b))) ? a : b;

        // Transform with origin at one cylinder ep, aligned with cylinder axis.
        using namespace open3d::visualization::gl_util;
        Eigen::Matrix4d lookat = LookAt(endpoint0, endpoint1, up).cast<double>();
        Eigen::Matrix4d cylinder_lookat_matrix = lookat.inverse();

        // Transform cylinder by LookAt transform.
        cylinder->Transform(cylinder_lookat_matrix);
        return cylinder;
    }

public:
    
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~
    // TODO 20241109 2-point cylinder demo
    
    static void cylinders_on_tri_mesh_edges()
    {
        auto vis = open3d::visualization::VisualizerWithKeyCallback();
        vis.CreateVisualizerWindow("TriangleMesh edges", 2000, 2000, 0, 0);
        
        double radius = 0.07;
        auto tri_mesh = open3d::geometry::TriangleMesh::CreateIcosahedron();
        std::set<std::pair<int, int>> edges;
        for (auto triangle : tri_mesh->triangles_)
        {
            for (int i = 0; i < 3; i++)
            {
                int a = triangle[i];
                int b = triangle[(i + 1) % 3];
                edges.insert(std::pair(std::min(a, b), std::max(a, b)));
            }
        }
        auto view = [&](const std::shared_ptr<open3d::geometry::TriangleMesh>& m)
        {
            m->ComputeVertexNormals();
            m->PaintUniformColor({0.75, 0.37, 0});
            vis.AddGeometry(m);
        };
        for (auto edge : edges)
        {
            auto v1 = tri_mesh->vertices_[edge.first];
            auto v2 = tri_mesh->vertices_[edge.second];
            auto cyl = constructO3dCylinder(radius, v1, v2, 100);
            view(cyl);
        }
        for (auto vertex : tri_mesh->vertices_)
        {
            auto s = open3d::geometry::TriangleMesh::CreateSphere(radius, 100);
            s->Translate(vertex);
            view(s);
        }
        vis.Run();
    }
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~

    //--------------------------------------------------------------------------

#endif  // USE_OPEN3D
};
