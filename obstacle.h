//------------------------------------------------------------------------------
//
//  obstacle.h -- new flock experiments
//
//  Obstacle base class, some specializations, and utilities.
//
//  An Obstacle is a type of geometry which Boids avoid.
//
//  Created by Craig Reynolds on January 25, 2024.
//  (Based on earlier C++ and Python versions.)
//  MIT License -- Copyright © 2024 Craig Reynolds
//------------------------------------------------------------------------------

#pragma once
#include "Vec3.h"
#include "shape.h"
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20241121 integrate Draw into Obstacles classes
#include "Draw.h"
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class Obstacle
{
public:
    // Constructors
    Obstacle()
    {
        // TODO 20240125 None of these from the Python side seem directly
        //               relevant to the c++ version without Open3D.
        //    self.tri_mesh = None
        //    # Seems hackish, related to Draw.temp_camera_lookat and the way Open3D
        //    # does translation relative to center of geometry.
        //    self.original_center = Vec3()
    }

    // Where a ray (Agent's path) will intersect the obstacle, or None.
    virtual Vec3 ray_intersection(const Vec3& origin,
                                  const Vec3& tangent,
                                  double body_radius) const {return Vec3();}

    virtual Vec3 normal_at_poi(const Vec3& poi,
                               const Vec3& agent_position) const {return Vec3();}

    // Point on surface of obstacle nearest the given query_point.
    virtual Vec3 nearest_point(const Vec3& query_point) const { return Vec3(); }

    // Compute direction for agent's static avoidance of nearby obstacles.
    virtual Vec3 fly_away(const Vec3& agent_position,
                          const Vec3& agent_forward,
                          double max_distance,
                          double body_radius) const { return Vec3(); }
    
    // Signed distance function.
    // (From a query point to the nearest point on Obstacle's surface: negative
    // inside, positive outside, zero at surface. Very similar to nearest_point(),
    // maybe they can be combined?)
    virtual double signed_distance(const Vec3& query_point) const { return 0; }
    
    virtual void draw() const {}
    
    virtual std::string to_string() const { return "Obstacle"; }
    
    static void unit_test();  // Defined at bottom of file.
    
    enum ExcludeFrom { inside, outside, neither };
    virtual void setExcludeFrom(ExcludeFrom ef) { exclude_from_ = ef; }
    virtual ExcludeFrom getExcludeFrom() const { return exclude_from_; }
    
    std::string getExcludeFromAsString() const
    {
        if (getExcludeFrom() == 0) { return "inside";}
        if (getExcludeFrom() == 1) { return "outside";}
        if (getExcludeFrom() == 2) { return "neither";}
        return "unknown";
    }
    
    virtual bool constraintViolation(const Vec3& current_point,
                                     const Vec3& previous_point) const
    {
        bool violation = false;
        ExcludeFrom ef = getExcludeFrom();
        double sdf = signed_distance(current_point);
        // Static violation? EG: inside of an Obstacle with ExcludeFrom inside.
        if (((sdf < 0) and (ef == inside)) or
            ((sdf > 0) and (ef == outside)))
        {
            violation = true;
        }
        else
        {
            // Or dynamic: agent crossed the surface since previous step.
            if ((not previous_point.is_none()) and
                util::zero_crossing(sdf, signed_distance(previous_point)))
            {
                violation = true;
            }
        }
        return violation;
    }

private:
    ExcludeFrom exclude_from_ = neither;
};


class EvertedSphereObstacle : public Obstacle
{
public:
    EvertedSphereObstacle(double radius, const Vec3& center)
      : Obstacle(), radius_(radius), center_(center) {}
    
    EvertedSphereObstacle(double radius, const Vec3& center, ExcludeFrom ef)
      : Obstacle(), radius_(radius), center_(center)
    {
        setExcludeFrom(ef);
    }

    // Where a ray (Agent's path) will intersect the obstacle, or None.
    Vec3 ray_intersection(const Vec3& origin,
                          const Vec3& tangent,
                          double body_radius) const override
    {
        return shape::ray_sphere_intersection(origin, tangent, radius_, center_);
    }

    // Normal to the obstacle at a given point of interest.
    Vec3 normal_at_poi(const Vec3& poi,
                       const Vec3& agent_position) const override
    {
        return (center_ - poi).normalize();
    }

    // Point on surface of obstacle nearest the given query_point.
    Vec3 nearest_point(const Vec3& query_point) const override
    {
        return (query_point - center_).normalize() * radius_;
    }

    // Compute direction for agent's static avoidance of nearby obstacles.
    Vec3 fly_away(const Vec3& agent_position,
                  const Vec3& agent_forward,
                  double max_distance,
                  double body_radius) const override
    {
        Vec3 avoidance;
        Vec3 p = agent_position;
        double r = radius_;
        Vec3 c = center_;
        Vec3 offset_to_sphere_center = c - p;
        double distance_to_sphere_center = offset_to_sphere_center.length();
        double dist_from_wall = r - distance_to_sphere_center;
        // Close enough to obstacle surface to use static repulsion.
        if (dist_from_wall < max_distance)
        {
            Vec3 normal = offset_to_sphere_center / distance_to_sphere_center;
            // Unless agent is already facing away from obstacle.
            if (normal.dot(agent_forward) < 0.9)
            {
                // Weighting falls off further from obstacle surface
                double weight = 1 - (dist_from_wall / max_distance);
                avoidance = normal * weight;
            }
        }
        return avoidance;
    }
    
    // Signed distance function. (From a query point to the nearest point on
    // Obstacle's surface: negative inside, positive outside, zero at surface.)
    double signed_distance(const Vec3& query_point) const override
    {
        double distance_to_center = (query_point - center_).length();
        return distance_to_center - radius_;
    }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241121 integrate Draw into Obstacles classes

//    void draw() const override
//    {
//        //def draw(self):
//        //    if not self.tri_mesh:
//        //        self.tri_mesh = Draw.make_everted_sphere(self.radius, self.center)
//        //    Draw.adjust_static_scene_object(self.tri_mesh)
//    }

    void draw() const override
    {
        auto mesh = Draw::constructSphereTriMesh(radius_,
                                                 center_,
                                                 {0.5, 0.5, 0.5},
                                                 true,
//                                                 getExcludeFrom() == inside);
                                                 getExcludeFrom() == outside,
//                                                 100);
                                                 500);
        Draw::getInstance().addTriMeshToStaticScene(mesh);
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    std::string to_string() const override { return "EvertedSphereObstacle"; }

private:
    double radius_ = 1;
    Vec3 center_;
};


class PlaneObstacle : public Obstacle
{
public:
    PlaneObstacle()
      : Obstacle(), normal_(Vec3(0, 1, 0)), center_(Vec3()) {}
    
    PlaneObstacle(const Vec3& normal, const Vec3& center)
      : Obstacle(), normal_(normal), center_(center) {}
    
    PlaneObstacle(const Vec3& normal, const Vec3& center, ExcludeFrom ef)
      : PlaneObstacle(normal, center)
    {
        setExcludeFrom(ef);
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241126 add PlaneObstacle::draw()
    
    PlaneObstacle(const Vec3& normal,
                  const Vec3& center,
                  double visible_radius,
                  double visible_thickness)
      : Obstacle(),
        normal_(normal),
        center_(center),
        visible_radius_(visible_radius),
        visible_thickness_(visible_thickness) {}

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Where a ray (Agent's path) will intersect the obstacle, or None.
    Vec3 ray_intersection(const Vec3& origin,
                          const Vec3& tangent,
                          double body_radius) const override
    {
        return shape::ray_plane_intersection(origin, tangent, center_, normal_);
    }

    // Normal to the obstacle at a given point of interest.
    Vec3 normal_at_poi(const Vec3& poi,
                       const Vec3& agent_position) const override
    {
        Vec3 normal = normal_;
        // If a reference position is given.
        if (agent_position != Vec3::none())
        {
            // Project it to obstacle surface.
            Vec3 on_obstacle = nearest_point(agent_position);
            // Normalized vector FROM obstacle surface TOWARD agent.
            normal = (agent_position - on_obstacle).normalize();
        }
        return normal;
    }

    // Point on surface of obstacle nearest the given query_point.
    Vec3 nearest_point(const Vec3& query_point) const override
    {
        // Offset from center point (origin) of plane.
        Vec3 offset = query_point - center_;
        // Signed distance from plane.
        double distance =  offset.dot(normal_);
        // Translate offset point onto plane (in plane's local space).
        Vec3 on_plane = offset - (normal_ * distance);
        // Translate back to global space.
        return on_plane + center_;
    }

    // Compute direction for agent's static avoidance of nearby obstacles.
    Vec3 fly_away(const Vec3& agent_position,
                  const Vec3& agent_forward,
                  double max_distance,
                  double body_radius) const override
    {
        Vec3 avoidance;
        // Project agent_position to obstacle surface.
        Vec3 on_obstacle = nearest_point(agent_position);
        double dist_from_obstacle = (on_obstacle - agent_position).length();
        // Close enough to obstacle surface to use static replusion.
        if (dist_from_obstacle < max_distance)
        {
            Vec3 normal = normal_at_poi(on_obstacle, agent_position);
            // Unless agent is already facing away from obstacle.
            if (normal.dot(agent_forward) < 0.9)
            {
                // Weighting falls off further from obstacle surface
                double weight = 1 - (dist_from_obstacle / max_distance);
                avoidance = normal * weight;
            }
        }
        return avoidance;
    }

    // Signed distance function. (From a query point to the nearest point on
    // Obstacle's surface: negative inside, positive outside, zero at surface.)
    double signed_distance(const Vec3& query_point) const override
    {
        Vec3 nearest_point_on_plane = nearest_point(query_point);
        Vec3 from_plane_to_query_point = query_point - nearest_point_on_plane;
        return from_plane_to_query_point.dot(normal_);
    }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241126 add PlaneObstacle::draw()
    
//        void draw() const override
//        {
//    //        // What is the best way to define these "fake" parameters?
//    //        double visible_radius = 50;
//    //        double thickness = visible_radius * 0.005;
//    //        Vec3 ep_offset = center_ + normal_ * thickness;
//            Vec3 ep_offset = center_ + normal_ * visible_thickness_;
//
//            auto mesh = Draw::constructCylinderTriMesh(visible_radius_,
//                                                       center_ + ep_offset,
//                                                       center_ - ep_offset,
//                                                       {0.7, 0.7, 0.8},
//                                                       true,
//                                                       false, // don't evert
//                                                       500);
//            Draw::getInstance().addTriMeshToStaticScene(mesh);
//        }
    
    void draw() const override
    {
        Vec3 ep_offset = center_ + normal_ * visible_thickness_;
        auto mesh = Draw::constructCylinderTriMesh(visible_radius_,
                                                   center_ + ep_offset,
                                                   center_ - ep_offset,
                                                   {0.7, 0.7, 0.8},
                                                   true,
                                                   false, // don't evert
                                                   500);
        Draw::getInstance().addTriMeshToStaticScene(mesh);
    }


    std::string to_string() const override { return "PlaneObstacle"; }

private:
    Vec3 normal_;
    Vec3 center_;

    // These are used only for drawing the obstacle (as a large thin disk).
    // They have no effect on the functioning of the PlaneObstacle itself.
    double visible_radius_ = 100;
    double visible_thickness_ = visible_radius_ * 0.001;
};


class CylinderObstacle : public Obstacle
{
public:
    CylinderObstacle(double radius, const Vec3& endpoint0, const Vec3& endpoint1)
      : Obstacle()
    {
        radius_ = radius;
        endpoint_ = endpoint0;
        Vec3 offset = endpoint1 - endpoint0;
        std::tie(tangent_, length_) = offset.normalize_and_length();
    }
    
    CylinderObstacle(double radius,
                     const Vec3& endpoint0,
                     const Vec3& endpoint1,
                     ExcludeFrom ef)
      : CylinderObstacle(radius, endpoint0, endpoint1)
    {
        setExcludeFrom(ef);
    }

    // Nearest point on the infinite line containing cylinder's axis.
    Vec3 nearest_point_on_axis(const Vec3& query_point) const
    {
        Vec3 offset = query_point - endpoint_;
        double projection = offset.dot(tangent_);
        return endpoint_ + tangent_ * projection;
    }

    // Where a ray (Agent's path) will intersect the obstacle, or None.
    Vec3 ray_intersection(const Vec3& origin,
                          const Vec3& tangent,
                          double body_radius) const override
    {
        return shape::ray_cylinder_intersection(origin, tangent,
                                                endpoint_, tangent_,
                                                radius_ + 2 * body_radius,
                                                length_);
    }

    // Normal to the obstacle at a given point of interest.
    Vec3 normal_at_poi(const Vec3& poi,
                       const Vec3& agent_position) const override
    {
        Vec3 on_axis = nearest_point_on_axis(poi);
        return (poi - on_axis).normalize();
    }

    // Point on surface of obstacle nearest the given query_point.
    Vec3 nearest_point(const Vec3& query_point) const override
    {
        Vec3 on_axis = nearest_point_on_axis(query_point);
        return on_axis + ((query_point - on_axis).normalize() * radius_);
    }

    // Compute direction for agent's static avoidance of nearby obstacles.
    Vec3 fly_away(const Vec3& agent_position,
                  const Vec3& agent_forward,
                  double max_distance,
                  double body_radius) const override
    {
        Vec3 avoidance;
        // Distance between this cylinder's axis and the agent's current path.
        double path_to_axis_dist = shape::distance_between_lines(agent_position,
                                                                 agent_forward,
                                                                 endpoint_,
                                                                 tangent_);
        // When too close, avoidance is unit normal to cylinder surface.
        double margin = 3 * body_radius;
        if (path_to_axis_dist < radius_ + margin)
        {
            Vec3 on_surface = nearest_point(agent_position);
            if ((on_surface - agent_position).length_squared() < sq(margin))
            {
                avoidance = normal_at_poi(agent_position, agent_position);
            }
        }
        return avoidance;
    }

    // Signed distance function. (From a query point to the nearest point on
    // Obstacle's surface: negative inside, positive outside, zero at surface.)
    double signed_distance(const Vec3& query_point) const override
    {
        Vec3 point_on_axis = nearest_point_on_axis(query_point);
        double distance_to_axis = (query_point - point_on_axis).length();
        return distance_to_axis - radius_;
    }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241121 integrate Draw into Obstacles classes

//    void draw() const override
//    {
//    //    def draw(self):
//    //        if not self.tri_mesh:
//    //            self.tri_mesh = Draw.new_empty_tri_mesh()
//    //            Draw.add_line_segment(self.endpoint,
//    //                                  self.endpoint + (self.tangent * self.length),
//    //                                  color = Vec3(1, 1, 1) * 0.8,
//    //                                  radius = self.radius,
//    //                                  sides = 50,
//    //                                  tri_mesh = self.tri_mesh,
//    //                                  flat_end_caps=True)
//    //            self.tri_mesh.compute_vertex_normals()
//    //            self.original_center = Vec3.from_array(self.tri_mesh.get_center())
//    //        Draw.adjust_static_scene_object(self.tri_mesh, self.original_center)
//    }

//    void draw() const override
//    {
//        auto& d = Draw::getInstance();
//        auto cyl_mesh = d.constructO3dCylinder(radius_, ep0(), ep1());
//        cyl_mesh->ComputeVertexNormals();
//        cyl_mesh->PaintUniformColor({0.7, 0.8, 0.7});
//        d.addTriMeshToStaticScene(cyl_mesh);
//    }

//    void draw() const override
//    {
//        auto& d = Draw::getInstance();
//        auto cyl_mesh = d.constructCylinderTriMesh(radius_,
//                                                   ep0(),
//                                                   ep1(),
//                                                   {0.7, 0.8, 0.7},
//                                                   true,
//                                                   getExcludeFrom() == inside);
//        d.addTriMeshToStaticScene(cyl_mesh);
//    }

    void draw() const override
    {
        auto mesh = Draw::constructCylinderTriMesh(radius_,
                                                   endpoint0(),
                                                   endpoint1(),
                                                   {0.7, 0.8, 0.7},
                                                   true,
//                                                   getExcludeFrom() == inside);
                                                   getExcludeFrom() == outside,
//                                                   100);
                                                   500);
        Draw::getInstance().addTriMeshToStaticScene(mesh);
    }

    Vec3 endpoint0() const { return endpoint_; }
    Vec3 endpoint1() const { return endpoint_ + tangent_ * length_; }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    std::string to_string() const override { return "CylinderObstacle"; }
    
private:
    double radius_;
    Vec3 endpoint_;
    Vec3 tangent_;
    double length_;
};


// Class to contain statistics of a predicted collision with an Obstacle.
class Collision
{
public:
    Collision(Obstacle& obstacle_,
              double time_to_collision_,
              double dist_to_collision_,
              Vec3 point_of_impact_,
              Vec3 normal_at_poi_)
      : obstacle(&obstacle_),
        time_to_collision(time_to_collision_),
        dist_to_collision(dist_to_collision_),
        point_of_impact(point_of_impact_),
        normal_at_poi(normal_at_poi_) {}
    Obstacle* obstacle;
    double time_to_collision;
    double dist_to_collision;
    Vec3 point_of_impact;
    Vec3 normal_at_poi;
    
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TOODO 20240809 why is obstacle avoidance broken?
    std::string to_string() const
    {
        std::stringstream ss;
        ss << "Collision(" << "obs=" << obstacle->to_string();
        ss << ", time=" << time_to_collision;
        ss << ", dist=" << dist_to_collision;
        ss << ", poi=" << point_of_impact;
        ss << ", norm=" << normal_at_poi << ")";
        return ss.str();
    }
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
};

//~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
// TOODO 20240809 why is obstacle avoidance broken?

// Serialize Collision object to stream.
inline std::ostream& operator<<(std::ostream& os, const Collision& c)
{
    os << c.to_string();
    return os;
}
//~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

inline void Obstacle::unit_test()
{
    Obstacle o;
    assert(o.to_string() == "Obstacle");
    
    EvertedSphereObstacle eso(10, Vec3(1, 2, 3));
    assert(eso.to_string() == "EvertedSphereObstacle");
    
    PlaneObstacle po;
    assert(po.to_string() == "PlaneObstacle");
    
    CylinderObstacle co(2, Vec3(-1, -1, -1), Vec3(-1, -1, -1));
    assert(co.to_string() == "CylinderObstacle");

    // Verify Collision can at least be instantiated and can read back obstacle.
    assert(Collision(o, 1, 2, Vec3(), Vec3(1, 2, 3)).obstacle == &o);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // WIP prototype, slightly more testing, at least “historical repeatability”.
    Vec3 eso_ri = eso.ray_intersection(Vec3(-2, 4, -3),           // ray origin
                                       Vec3(1, 3, -5).normalize(),// ray tangent
                                       0.5);
    // debugPrint(eso_ri.to_string_double_precision())
    Vec3 eso_ri_expected(-1.41115141100798,  // Recorded 20240127
                         5.76654576697607,
                         -5.94424294496012);
    double e = util::epsilon * 10;
    assert(Vec3::is_equal_within_epsilon(eso_ri, eso_ri_expected, e));
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Test ExcludeFrom testing with constraintViolation().
    Vec3 poop(0, 0.1, 0);
    Vec3 poip = -poop;
    assert(po.constraintViolation(poip, poop));
    assert(po.constraintViolation(poop, poip));
    PlaneObstacle po_ef_inside(Vec3(0, 1, 0), Vec3(), ExcludeFrom::inside);
    PlaneObstacle po_ef_outside(Vec3(0, 1, 0), Vec3(), ExcludeFrom::outside);
    assert(po_ef_inside.constraintViolation(poip, Vec3::none()));
    assert(not po_ef_outside.constraintViolation(poip, Vec3::none()));
    assert(not po_ef_inside.constraintViolation(poop, Vec3::none()));
    assert(po_ef_outside.constraintViolation(poop, Vec3::none()));
}

typedef std::vector<Obstacle*> ObstaclePtrList;
typedef std::vector<Collision> CollisionList;
