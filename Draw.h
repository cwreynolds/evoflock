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
    
    // shared pointers to Open3D TriangleMesh objects
    typedef open3d::geometry::TriangleMesh tri_mesh_t;
    typedef std::shared_ptr<tri_mesh_t> sp_tri_mesh_t;

#endif  // USE_OPEN3D

    // Used to get the current global Draw object (drawing context). Returns the
    // existing instance, creating a new one if needed.
    static Draw& getInstance() { return getInstance(true); }
    static Draw& getInstance(bool enabled)
    {
        if (not global_object_) { global_object_ = new Draw(enabled); }
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
        // TODO 20241026 -- Temporary for debugging odd symptom: running from inside
        // Xcode launches a separate copy of the executable about a second after the
        // first. Oddly this symptom seems to come and go as I edit main.cpp
        constructor_count++;
        assert(constructor_count == 1);

        // Handle pointer to singleton instance of Draw class.
        assert(global_object_ == nullptr);
        global_object_ = this;
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
        visualizer().GetRenderOption().line_width_ = line_width;
        visualizer().GetRenderOption().point_size_ = point_size;
        
        // Add single key command callback to toggle "graphics mode"
        visualizer().RegisterKeyCallback('G',
                                         [&](base_vis_t* vis)
                                         { toggleEnable(); return true; });
        // Add "C" command, to cycle through camera aiming modes.
        visualizer().RegisterKeyCallback('C',
                                         [&](base_vis_t* vis)
                                         { nextCameraMode(); return true; });
        
        // Add "P" command, to pause simulation (just toggles public flag).
        visualizer().RegisterKeyCallback('P',
                                         [&](base_vis_t* vis)
                                         { toggleSimPause(); return true; });
        
        // Add "O" command, to increment obstacle set counter.
        visualizer().RegisterKeyCallback('O',
                                         [&](base_vis_t* vis)
                                         { nextObstacleSet(); return true; });
        
        // Set mouse scroll policy based on current camera mode.
        updateMouseScrollCallback();

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20241007 follow cam -- to/from tracker balls
        double ball_radius = 0.3;
        to_ball = open3d::geometry::TriangleMesh::CreateSphere(ball_radius);
        from_ball = open3d::geometry::TriangleMesh::CreateSphere(ball_radius);
        to_ball->PaintUniformColor({0, 1, 0});
        from_ball->PaintUniformColor({1, 0, 1});
        visualizer().AddGeometry(to_ball);
        visualizer().AddGeometry(from_ball);
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#endif  // USE_OPEN3D
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241007 follow cam -- to/from tracker balls
//    std::shared_ptr<open3d::geometry::TriangleMesh> from_ball;
//    std::shared_ptr<open3d::geometry::TriangleMesh> to_ball;
    sp_tri_mesh_t from_ball;
    sp_tri_mesh_t to_ball;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

public:

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
        setExitFromRun(! pollEvents());
        if (enable())
        {
            animated_tri_mesh_->Clear();
            animated_line_set_->Clear();
            updateCamera();
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
    
    // Clear all previous TriangleMesh objects from static scene.
    void clearStaticScene()
    {
        for (sp_tri_mesh_t tm : static_tri_meshes_)
        {
            visualizer().RemoveGeometry(tm, false);
        }
        static_tri_meshes_.clear();
    }

    // Add a TriangleMesh objects to the static (non-animating) scene.
    void addTriMeshToStaticScene(const sp_tri_mesh_t tri_mesh)
    {
        static_tri_meshes_.push_back(tri_mesh);
        visualizer().AddGeometry(tri_mesh);
    }

    // Make cylinder: returns shared pointer to tri mesh with given parameters.
    static sp_tri_mesh_t constructCylinderTriMesh(double radius,
                                                  const Vec3& endpoint0,
                                                  const Vec3& endpoint1,
                                                  const Vec3& color,
                                                  bool compute_normals,
                                                  bool evert,
                                                  int subdivision)
    {
        auto cyl_mesh = constructO3dCylinder(radius,
                                             vec3ToEv3d(endpoint0),
                                             vec3ToEv3d(endpoint1),
                                             subdivision);
        cyl_mesh->PaintUniformColor(vec3ToEv3d(color));
        if (compute_normals) { cyl_mesh->ComputeVertexNormals(); }
        if (evert) { evertTriangleMesh(*cyl_mesh); }
        return cyl_mesh;
    }

    // Make sphere: returns shared pointer to tri mesh with given parameters.
    static sp_tri_mesh_t constructSphereTriMesh(double radius,
                                                const Vec3& center,
                                                const Vec3& color,
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

    // Runtime switch to turn graphical display on and off.
    bool enable() const { return enable_ and not exitFromRun(); }
    void setEnable(bool e) { enable_ = e; }
    void toggleEnable() { enable_ = not enable_; }

    bool pollEvents()
    {
        return visualizer().PollEvents();
    }

    // Update the camera view. Runs "follow cam". Sets Open3d view dep on mode.
    void updateCamera()
    {
        // Invoke the "follow cam" model, update look_from / look_at points
        animateFollowCamera();
        // Either set view to from/at points or set markers in static view.
        if (cameraMode() == true)
        {
            setVisualizerViewByFromAt(cameraLookFrom(),
                                      cameraLookAt(),
                                      cameraLookUp());
            from_ball->Translate({0, 1000, 0}, false);
            to_ball->Translate({0, 1000, 0}, false);
            // Draw circle around aimTarget() pointing toward camera.
            addBlackAndWhiteCircularReticle(aimTarget());
        }
        else
        {
            from_ball->Translate(vec3ToEv3d(cameraLookFrom()), false);
            to_ball->Translate(vec3ToEv3d(cameraLookAt()), false);
        }
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
        double color = 0;
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
                                          Vec3(color, color, color));
            local_spoke = new_spoke;
            color = color ? 0 : 1;
        }
    }

    // Set the "camera view" of current Open3D visualizer according to a look/at
    // specification. Optional "up" defaults to global Y direction.
    //
    void setVisualizerViewByFromAt(Vec3 look_from,
                                   Vec3 look_at,
                                   Vec3 up = Vec3(0, 1, 0))
    {
        setOpen3DVisualizerViewByFromAt(visualizer(),
                                        vec3ToEv3d(look_from),
                                        vec3ToEv3d(look_at),
                                        vec3ToEv3d(up));
    }

    // Invoke the "follow camera" model, update look_from/look_at/up and camera.
    void animateFollowCamera()
    {
        Vec3 camera_pos = camera().p();
        Vec3 offset_from_camera_to_target = aimTarget() - camera_pos;
        Vec3 offset_direction = offset_from_camera_to_target.normalize_or_0();
        Vec3 target_offset = offset_direction * cameraDesiredOffsetDistance();
        Vec3 new_from = aimTarget() - target_offset;
        Vec3 new_at = aimTarget();
        Vec3 new_up = (camera().j() + Vec3(0, 0.3, 0)).normalize();
        camera_look_from_ = from_memory_.blend(new_from, 0.90);
        camera_look_at_   =   at_memory_.blend(new_at,   0.75);
        camera_look_up_   =   up_memory_.blend(new_up,   0.97).normalize();
        camera() = LocalSpace::fromTo(cameraLookFrom(),
                                      cameraLookAt(),
                                      cameraLookUp());
    }

    void updateMouseScrollCallback()
    {
        std::function<bool(base_vis_t *, double, double)> mscb = nullptr;
        if (cameraMode() == true)
        {
            mscb = [&](base_vis_t* vis, double x, double y)
            {
                // Change follow distance.
                double adjust_speed = 0.8;
                double min = 0.05;
                double d = cameraDesiredOffsetDistance() + (y * adjust_speed);
                cameraDesiredOffsetDistance() = std::max(min, d);
                return true;
            };
        }
        visualizer().RegisterMouseScrollCallback(mscb);
    }

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
    Vec3 cameraLookUp() const { return camera_look_up_; }
    
    // Settable accessor for  camera mode. Now cycles between follow and global.
    bool& cameraMode() { return camera_mode_; }
    void nextCameraMode()
    {
        camera_mode_ = not camera_mode_;
        updateMouseScrollCallback();
    }
    
    // Set to true when user types ESC or closes Visualizer window.
    bool exitFromRun() const { return exit_from_run_; }
    void setExitFromRun(bool efr) { exit_from_run_ = efr; }
    
    // Settable accessor for desired offset distance in follow camera mode.
    double& cameraDesiredOffsetDistance()
    {
        return camera_desired_offset_dist_;
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

    // Desired offset distance in follow camera mode.
    double camera_desired_offset_dist_ = 15;

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
    
    // Runtime counter the simulation can use to change predefined obs sets.
    // (TODO this seems very specific to flock simulation is it OK to be here?)
    int obstacle_set_index_ = 0;

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
    // (pr to add to Open3D? https://github.com/isl-org/Open3D/discussions/6419)
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
