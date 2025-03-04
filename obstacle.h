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
#include "Draw.h"

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

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20250219 experimental version to enforce constraint
    // Maybe these should be called normal(poi) and normal_toward_agent(poi, p)

    // TODO this is the "legacy API"
    
    virtual Vec3 normal_at_poi(const Vec3& poi, const Vec3& agent_position) const
    {
        return unimplemented("normal_at_poi()", Vec3());
    }
    
    
    // TODO these are the "new API"
    
    // Abstract normal for a given position. Points toward the +SDF side.
    virtual Vec3 normal(const Vec3& poi) const
    {
        return unimplemented("normal()", Vec3());
    }
    
    // Normal for a given position. Points toward side agent is on.
    virtual Vec3 normal_toward_agent(const Vec3& poi,
                                     const Vec3& agent_position) const
    {
        return unimplemented("normal_toward_agent()", Vec3());
    }
    
    
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20250301 prototype Obstacle::normalTowardAllowedSide(poi)
    
    // Normal for a given position. Points toward non-exclided side.
    // (Not tested for, nor well-defined for, the ExcludeFrom::neither case.)
    Vec3 normalTowardAllowedSide(const Vec3& poi) const
    {
        return normal(poi) * ((getExcludeFrom() == outside) ? -1 : 1);
    }
    
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
    
    // Override this in derived to add tri-mesh model of Obstacle shape.
    virtual void addToScene() const {}

    // Each obstacle has a color for drawing.
    void setColor(Color color) { color_ = color; }
    Color getColor() const { return color_; }

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
    
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20250227 temp debug
    static inline bool verbose = false;
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

    // Does the given agent state violate this obstacle's ExcludeFrom?
    bool doesAgentViolateConstraint(Vec3 agent_position, double agent_radius) const
    {
        ExcludeFrom ef = getExcludeFrom();
        double sdf = signed_distance(agent_position);
        
        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~
        // TODO 20250302 use normalTowardAllowedSide()

//        bool violate = (((sdf < 0) and (ef == inside)) or
//                        ((sdf > 0) and (ef == outside)));
        
//        bool violate = (((sdf < +agent_radius) and (ef == inside)) or
//                        ((sdf > -agent_radius) and (ef == outside)));

        // TODO 20250303
        bool violate = (((sdf < 0) and (ef == inside)) or
                        ((sdf > 0) and (ef == outside)));

        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~

        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
        // TODO 20250227 temp debug
        if (verbose)
        {
            std::cout << "ef=" << getExcludeFromAsString();
            std::cout << " sdf=" << sdf;
            std::cout << " violate=" << violate;
            std::cout << std::endl;
        }
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
        return violate;
    }

    // Detects constraint violation. (For example being inside an Obstacle with
    // ExcludeFrom::inside.) When found, computes a new agent position which
    // does not violate the constraint. Caller should compare value passed into
    // "agent_position" with the returned value. If equal, the agent is fine.
    // Otherwise the agent's position should be set to the value returned, and
    // its speed set to zero.
    //
    // Does this need to be virtual, or is it generic for all Obstacles?
    //
    virtual Vec3 enforceConstraint(Vec3 agent_position, double agent_radius) const
    {
        Vec3 result = agent_position;  // Default result is current agent pos.
        if (doesAgentViolateConstraint(agent_position, agent_radius))
        {
            Vec3 surface_point = nearest_point(agent_position);
            
            //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~
            // TODO 20250302 use normalTowardAllowedSide()

//            Vec3 nta = normal_toward_agent(surface_point, agent_position);
//            result = surface_point - (nta * agent_radius);

            Vec3 ntas = normalTowardAllowedSide(agent_position);
            result = surface_point + (ntas * agent_radius);

            //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~
        }
        return result;
    }

    // Historical signum function for local use in this class. (Move to util?)
    static double signum(double x) { return x > 0 ? 1 : (x < 0 ? -1 : 0); }

    // Maybe temp? used while refactoring API.
    void unimplemented(const std::string& name) const
    {
        std::cout << "unimplemented " << name << std::endl;
    }
    template<typename T>
    T unimplemented(const std::string& name, const T& default_value) const
    {
        unimplemented(name);
        return default_value;
    }

private:
    ExcludeFrom exclude_from_ = neither;
    Vec3 color_;
};


class SphereObstacle : public Obstacle
{
public:
    SphereObstacle() : Obstacle()
    {
        // +/- 1.5% color noise around middle gray.
        setColor(Color::randomInRgbBox(Color(0.485), Color(0.515)));
    }
    
    SphereObstacle(double radius_, const Vec3& center_) : SphereObstacle()
    {
        radius() = radius_;
        center() = center_;
    }
    
    SphereObstacle(double radius, const Vec3& center, ExcludeFrom ef)
      : SphereObstacle(radius, center)
    {
        setExcludeFrom(ef);
    }
    
    // Where a ray (Agent's path) will intersect the obstacle, or None.
    Vec3 ray_intersection(const Vec3& origin,
                          const Vec3& tangent,
                          double body_radius) const override
    {
        return shape::ray_sphere_intersection(origin, tangent, sphere());
    }
    
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20250219 experimental version to enforce constraint
    // TODO this is the "legacy API"

    // Normal to the obstacle at a given point of interest.
    Vec3 normal_at_poi(const Vec3& poi,
                       const Vec3& agent_position) const override
    {
        Vec3 perp_direction = (center() - poi).normalize();
        double agent_to_surface_signed_dist = signed_distance(agent_position);
        // TODO clean up, with signum? ~~~~~~~~~~~~~~~~~~~~~~~~~
        double sign = agent_to_surface_signed_dist < 0 ? 1 : -1;
        return perp_direction * sign;
    }
    
    // TODO these are the "new API"

    // Abstract normal for a given position. Points toward the +SDF side.
    Vec3 normal(const Vec3& poi) const override
    {
        return (poi - center()).normalize();
    }

    // Normal for a given position. Points toward side agent is on.
    Vec3 normal_toward_agent(const Vec3& poi,
                             const Vec3& agent_position) const override
    {
        return normal(poi) * signum(signed_distance(agent_position));
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    
    // Point on surface of obstacle nearest the given query_point.
    Vec3 nearest_point(const Vec3& query_point) const override
    {
        return ((query_point - center()).normalize() * radius()) + center();
    }
    
    // Compute direction for agent's static avoidance of nearby obstacles.
    Vec3 fly_away(const Vec3& agent_position,
                  const Vec3& agent_forward,
                  double max_distance,
                  double body_radius) const override
    {
        Vec3 avoidance;
        Vec3 p = agent_position;
        bool agent_inside = 0 > signed_distance(p);
        Vec3 offset_to_sphere_center = center() - p;
        double distance_to_sphere_center = offset_to_sphere_center.length();
        double abs_dist_from_wall = std::abs(radius()-distance_to_sphere_center);
        // Close enough to obstacle surface to require static repulsion.
        if (abs_dist_from_wall < max_distance)
        {
            Vec3 normal = offset_to_sphere_center / distance_to_sphere_center;
            // Unless agent is already facing away from obstacle.
            if (normal.dot(agent_forward) < 0.9)
            {
                // Weighting falls off further from obstacle surface
                double weight = 1 - (abs_dist_from_wall / max_distance);
                avoidance = normal * weight * (agent_inside ? 1 : -1);
            }
        }
        return avoidance;
    }
    
    // Signed distance function. (From a query point to the nearest point on
    // Obstacle's surface: negative inside, positive outside, zero at surface.)
    double signed_distance(const Vec3& query_point) const override
    {
        double distance_to_center = (query_point - center()).length();
        return distance_to_center - radius();
    }
    
    void addToScene() const override
    {
        auto mesh = Draw::constructSphereTriMesh(radius(),
                                                 center(),
                                                 getColor(),
                                                 true,
                                                 getExcludeFrom() == outside,
                                                 500);
        Draw::brightnessSpecklePerVertex(0.95, 1.00, getColor(), mesh);
        Draw::getInstance().addTriMeshToStaticScene(mesh);
    }
    
    std::string to_string() const override { return "SphereObstacle"; }
    
    // TODO 20250208 can these be rewritten as 2 functions with &&/std::forward?
    double radius() const { return sphere_.radius; }
    double& radius() { return sphere_.radius; }
    Vec3 center() const { return sphere_.center; }
    Vec3& center() { return sphere_.center; }
    shape::Sphere sphere(void) const { return sphere_; }
    shape::Sphere& sphere(void) { return sphere_; }
    
private:
    shape::Sphere sphere_;
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
        setColor({0.7, 0.7, 0.8});
    }

    PlaneObstacle(const Vec3& normal,
                  const Vec3& center,
                  double visible_radius,
                  double visible_thickness)
      : Obstacle(),
        normal_(normal),
        center_(center),
        visible_radius_(visible_radius),
        visible_thickness_(visible_thickness) {}

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
    
    void addToScene() const override
    {
        Vec3 ep_offset = center_ + normal_ * visible_thickness_;
        auto mesh = Draw::constructCylinderTriMesh(visible_radius_,
                                                   center_ + ep_offset,
                                                   center_ - ep_offset,
                                                   getColor(),
                                                   true,
                                                   false, // don't evert
                                                   500);
        Draw::brightnessSpecklePerVertex(0.7, 1.0, getColor(), mesh);
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
        setColor({0.7, 0.8, 0.7});
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

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20250220 update normal(), etc., API for CylinderObstacle

//    // Normal to the obstacle at a given point of interest.
//    Vec3 normal_at_poi(const Vec3& poi,
//                       const Vec3& agent_position) const override
//    {
//        Vec3 on_axis = nearest_point_on_axis(poi);
//        return (poi - on_axis).normalize();
//    }

    // OLD API ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~
    
    // Normal to the obstacle at a given point of interest.
    Vec3 normal_at_poi(const Vec3& poi,
                       const Vec3& agent_position) const override
    {
        return normal(poi);
    }

    // NEW API ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~

    // Abstract normal for a given position. Points toward the +SDF side.
    // TODO -- this ignores the end caps of the cylinder
    Vec3 normal(const Vec3& poi) const override
    {
        Vec3 on_axis = nearest_point_on_axis(poi);
        return (poi - on_axis).normalize();
    }
    
    
    // Normal for a given position. Points toward side agent is on.
    // TODO -- this ignores the end caps of the cylinder
    Vec3 normal_toward_agent(const Vec3& poi,
                             const Vec3& agent_position) const override
    {
        // This is a copy of the implementation from SphereObstacle
        // Can this be a generic definition in the base class?
        return normal(poi) * signum(signed_distance(agent_position));
    }


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
    
    void addToScene() const override
    {
        auto mesh = Draw::constructCylinderTriMesh(radius_,
                                                   endpoint0(),
                                                   endpoint1(),
                                                   getColor(),
                                                   true,
                                                   getExcludeFrom() == outside,
                                                   500);
        Draw::brightnessSpecklePerVertex(0.7, 1.0, getColor(), mesh);
        Draw::getInstance().addTriMeshToStaticScene(mesh);
    }

    Vec3 endpoint0() const { return endpoint_; }
    Vec3 endpoint1() const { return endpoint_ + tangent_ * length_; }

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
};

// Serialize Collision object to stream.
inline std::ostream& operator<<(std::ostream& os, const Collision& c)
{
    os << c.to_string();
    return os;
}

inline void Obstacle::unit_test()
{
    Obstacle o;
    assert(o.to_string() == "Obstacle");
    
    SphereObstacle eso;
    assert(eso.to_string() == "SphereObstacle");

    PlaneObstacle po;
    assert(po.to_string() == "PlaneObstacle");
    
    CylinderObstacle co(2, Vec3(-1, -1, -1), Vec3(-1, -1, -1));
    assert(co.to_string() == "CylinderObstacle");

    // Verify Collision can at least be instantiated and can read back obstacle.
    assert(Collision(o, 1, 2, Vec3(), Vec3(1, 2, 3)).obstacle == &o);

    // WIP prototype, slightly more testing, at least “historical repeatability”.
    SphereObstacle tso(10, Vec3(1, 2, 3));
    Vec3 tso_ri = tso.ray_intersection(Vec3(-2, 4, -3),           // ray origin
                                       Vec3(1, 3, -5).normalize(),// ray tangent
                                       0.5);
    Vec3 tso_ri_expected(-1.41115141100798,  // Recorded 20240127
                         5.76654576697607,
                         -5.94424294496012);
    double e = util::epsilon * 10;
    assert(Vec3::is_equal_within_epsilon(tso_ri, tso_ri_expected, e));
    
    // Found bug in SphereObstacle::nearest_point() where it was assuming that
    // every sphere was centered at the origin, this would have caught it.
    {
        double radius = 1;
        Vec3 center(1, 2, 3);
        Vec3 test_point(5.1, 5.2, 5.3);
        Vec3 offset(10, 11, 12);
        SphereObstacle s1(radius, center);
        SphereObstacle s2(radius, center + offset);
        Vec3 tp = test_point;
        Vec3 to = test_point + offset;
        assert((tp - s1.nearest_point(tp)).length() ==
               (to - s2.nearest_point(to)).length());
    }
    
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20250301 does SphereObstacle::normal() always “point toward +SDF side”?
    
    // Verify SphereObstacle::normal() gets same value for inside/outside point
    {
        double radius = 2;
        Vec3 center(2, 4, 8);
        Vec3 test_dir = Vec3(-5, 3, -2).normalize();
        SphereObstacle sphere(radius, center);
        Vec3 inside = sphere.normal(center + test_dir);
        Vec3 outside = sphere.normal(center + (test_dir * (radius + 1)));
        assert(Vec3::is_equal_within_epsilon(inside, outside));
    }
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20250301 prototype Obstacle::normalTowardAllowedSide(poi)
    
    // Verify normalTowardAllowedSide() points toward non-excluded side.
    {
        double radius = 2;
        Vec3 center(-6, -4, -2);
        SphereObstacle sphere_exclude_inside(radius, center, inside);
        SphereObstacle sphere_exclude_outside(radius, center, outside);
        Vec3 poi(1, 2, 3);
        Vec3 outside_norm = sphere_exclude_inside.normalTowardAllowedSide(poi);
        Vec3 inside_norm = sphere_exclude_outside.normalTowardAllowedSide(poi);
        assert(Vec3::is_equal_within_epsilon(outside_norm, -inside_norm));
    }
    
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~


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
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20250219 experimental version to enforce constraint
        
    assert(signum( 5.0) ==  1.0);
    assert(signum( 0.5) ==  1.0);
    assert(signum( 0.0) ==  0.0);
    assert(signum(-0.5) == -1.0);
    assert(signum(-5.0) == -1.0);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

typedef std::vector<Obstacle*> ObstaclePtrList;
typedef std::vector<Collision> CollisionList;
