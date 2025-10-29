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
    virtual Vec3 rayIntersection(const Vec3& origin,
                                  const Vec3& tangent,
                                  double body_radius) const
    {
        return unimplemented("rayIntersection()", Vec3());
    }
    
    // Abstract normal for a given position. Points toward the +SDF side.
    virtual Vec3 normal(const Vec3& poi) const
    {
        return unimplemented("normal()", Vec3());
    }
    
    // Normal for a given position. Points toward the side agent is on.
    Vec3 normalTowardAgent(const Vec3& poi, const Vec3& agent_position) const
    {
        return normal(poi) * signum(signed_distance(agent_position));
    }
    
    // Normal for a given position, typically of a moving Agent. Points toward
    // non-excluded side. ExcludeFrom::neither case requires a "prev_position"
    // used as a reference for which side of the surface it must remain on.
    Vec3 normalTowardAllowedSide(Vec3 now_position, Vec3 prev_position) const
    {
        int sign = 1;
        if (getExcludeFrom() == outside) { sign = -1; }
        if (getExcludeFrom() == neither)
        {
            sign = signum(signed_distance(prev_position));
        }
        return normal(now_position) * sign;
    }

    // Point on surface of obstacle nearest the given query_point.
    virtual Vec3 nearest_point(const Vec3& query_point) const
    {
        return unimplemented("nearest_point()", Vec3());
    }
    
    // Compute direction for agent's static avoidance of nearby obstacles.
    virtual Vec3 fly_away(const Vec3& agent_position,
                          const Vec3& agent_forward,
                          double max_distance,
                          double body_radius) const
    {
        return unimplemented("fly_away()", Vec3());
    }

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
    
    // Detects constraint violation. (For example being inside an Obstacle with
    // ExcludeFrom::inside.) When found, computes a new agent position which
    // does not violate the constraint. Caller should compare value passed into
    // "agent_position" with the returned value. If equal, the agent is fine.
    // Otherwise the agent's position should be set to the value returned, and
    // its speed set to zero.
    Vec3 enforceConstraint(Vec3 now_position, Vec3 prev_position) const
    {
        Vec3 result = now_position;  // Default result is current agent pos.
        if (isAgentViolatingConstraint(now_position, prev_position))
        {
            Vec3 surface_point = nearest_point(now_position);
            Vec3 ntas = normalTowardAllowedSide(now_position, prev_position);
            result = surface_point + ntas;
        }
        return result;
    }
    
    // Tests for violations of an Obstacle's position constraint by an Agent.
    // The agent is represented by a current and previous position. The Obstacle
    // surface shape is represented by its signed distance function and its
    // ExcludeFrom mode. For ExcludeFrom::neither needs "prev_position" to
    // determine which side of the surface it ought to be on.
    bool isAgentViolatingConstraint(Vec3 now_position, Vec3 prev_position) const
    {
        bool violate = false;
        double now_sdf = signed_distance(now_position);
        switch (getExcludeFrom())
        {
            // Current position on the "excluded" side of the surface.
            case inside:  if (now_sdf < 0) { violate = true; } break;
            case outside: if (now_sdf > 0) { violate = true; } break;
            case neither:
                if ((not violate) and (not prev_position.is_none()))
                {
                    // Not allowed to cross from inside to outside or vice versa
                    double prev_sdf = signed_distance(prev_position);
                    violate = util::zero_crossing(now_sdf, prev_sdf);
                }
        }
        return violate;
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
    Vec3 rayIntersection(const Vec3& origin,
                         const Vec3& tangent,
                         double body_radius) const override
    {
        return shape::ray_sphere_intersection(origin, tangent, sphere());
    }
    
    // Abstract normal for a given position. Points toward the +SDF side.
    Vec3 normal(const Vec3& poi) const override
    {
        return (poi - center()).normalize();
    }

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
    Vec3 rayIntersection(const Vec3& origin,
                         const Vec3& tangent,
                         double body_radius) const override
    {
        return shape::ray_plane_intersection(origin, tangent, center_, normal_);
    }
    
    // Abstract normal for a given position. Points toward the +SDF side.
    Vec3 normal(const Vec3& poi) const override
    {
        return normal_;
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
            Vec3 normal = normalTowardAgent(on_obstacle, agent_position);
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
    Vec3 rayIntersection(const Vec3& origin,
                         const Vec3& tangent,
                         double body_radius) const override
    {
        return shape::ray_cylinder_intersection(origin, tangent,
                                                endpoint_, tangent_,
                                                radius_ + 2 * body_radius,
                                                length_);
    }

    // Abstract normal for a given position. Points toward the +SDF side.
    // TODO -- this ignores the end caps of the cylinder
    Vec3 normal(const Vec3& poi) const override
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
                avoidance = normalTowardAgent(agent_position, agent_position);
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


typedef std::vector<Obstacle*> ObstaclePtrList;
typedef std::vector<Collision> CollisionList;


class ObstacleSet
{
public:
    ObstacleSet() : ObstacleSet("unnamed", {}) {}
    
    ObstacleSet(const std::string& name, const ObstaclePtrList& obstacles)
      : name_(name), obstacles_(obstacles) {}
    
    std::string name() const { return name_; }
    
    ObstaclePtrList obstacles() const { return obstacles_; }
    
private:
    std::string name_;
    ObstaclePtrList obstacles_;
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
    Vec3 tso_ri = tso.rayIntersection(Vec3(-2, 4, -3),            // ray origin
                                      Vec3(1, 3, -5).normalize(), // ray tangent
                                      0.5);
    Vec3 tso_ri_expected(-1.41115141100798,  // Recorded 20240127
                         5.76654576697607,
                         -5.94424294496012);
    double e = util::epsilon * 10;
    assert(Vec3::within_epsilon(tso_ri, tso_ri_expected, e));
    
    // Test local def of classic signum(). Could move to Utilities.h if needed.
    assert(signum( 5.0) ==  1.0);
    assert(signum( 0.5) ==  1.0);
    assert(signum( 0.0) ==  0.0);
    assert(signum(-0.5) == -1.0);
    assert(signum(-5.0) == -1.0);

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
    
    // Verify SphereObstacle::normal() gets same value for inside/outside point
    {
        double radius = 2;
        Vec3 center(2, 4, 8);
        Vec3 test_dir = Vec3(-5, 3, -2).normalize();
        SphereObstacle sphere(radius, center);
        Vec3 inside = sphere.normal(center + test_dir);
        Vec3 outside = sphere.normal(center + (test_dir * (radius + 1)));
        assert(Vec3::within_epsilon(inside, outside));
    }
    
    // Verify normalTowardAllowedSide() points toward non-excluded side.
    {
        double radius = 2;
        Vec3 center(-6, -4, -2);
        SphereObstacle sphere_exclude_inside(radius, center, inside);
        SphereObstacle sphere_exclude_outside(radius, center, outside);
        Vec3 p(1, 2, 3);
        Vec3 outside_norm = sphere_exclude_inside.normalTowardAllowedSide(p, p);
        Vec3 inside_norm = sphere_exclude_outside.normalTowardAllowedSide(p, p);
        assert(Vec3::within_epsilon(outside_norm, -inside_norm));
    }

    // Test enforceConstraint() on various combinations of shape and ExcludeFrom.
    {
        // For each of three Obstacles, with their ExcludeFrom set to inside (i),
        // outside (o), and neither (n), these six tests verify that:
        // (1) ef=inside,  agent outside stays outside.
        // (2) ef=inside,  agent inside gets moved to outside.
        // (3) ef=outside, agent outside gets moved to inside.
        // (4) ef=outside, agent inside stays inside.
        // (5) ef=neither, agent outside, coming from inside, stays inside.
        // (6) ef=neither, agent inside, coming from outside, stays outside.
        auto six_way_exclude_from = [](Obstacle& i, Obstacle& o, Obstacle& n)
        {
            Vec3 zp1z(0, +1, 0);
            Vec3 zm1z(0, -1, 0);
            Vec3 zp2z(0, +2, 0);
            Vec3 zm2z(0, -2, 0);
            assert(i.enforceConstraint(zp1z, zp2z) == zp1z);
            assert(i.enforceConstraint(zm1z, zp2z) == zp1z);
            assert(o.enforceConstraint(zp1z, zm2z) == zm1z);
            assert(o.enforceConstraint(zm1z, zm2z) == zm1z);
            assert(n.enforceConstraint(zp1z, zm2z) == zm1z);
            assert(n.enforceConstraint(zm1z, zp2z) == zp1z);
        };
        
        // 3 PlaneObstacles on the XZ plane, inside is y<0, outside is y>0, with
        // ExcludeFrom set to inside, outside, or neither. Test point is origin.
        PlaneObstacle po_i(Vec3(0, 1, 0), Vec3(), ExcludeFrom::inside);
        PlaneObstacle po_o(Vec3(0, 1, 0), Vec3(), ExcludeFrom::outside);
        PlaneObstacle po_n(Vec3(0, 1, 0), Vec3(), ExcludeFrom::neither);
        six_way_exclude_from(po_i, po_o, po_n);
        
        // Same test with 3 SphereObstacles whose "north pole" is at the origin.
        SphereObstacle so_i(5, Vec3(0, -5, 0), ExcludeFrom::inside);
        SphereObstacle so_o(5, Vec3(0, -5, 0), ExcludeFrom::outside);
        SphereObstacle so_n(5, Vec3(0, -5, 0), ExcludeFrom::neither);
        six_way_exclude_from(so_i, so_o, so_n);
    }
}
