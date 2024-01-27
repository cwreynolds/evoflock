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

//    #-------------------------------------------------------------------------------
//    #
//    # obstacle.py -- new flock experiments
//    #
//    # Obstacle base class, some specializations, and utilities.
//    #
//    # An Obstacle is a type of geometry which Boids avoid.
//    #
//    # MIT License -- Copyright © 2023 Craig Reynolds
//    #
//    #-------------------------------------------------------------------------------
//    # TODO 20230831 initially just a wrapper on the existing everted sphere, then
//    #               generalize that, add a cylinder type, maybe later a triangle
//    #               mesh type.
//    #-------------------------------------------------------------------------------
//
//    import math
//    import shape
//    from Vec3 import Vec3
//    from Draw import Draw
//    import Utilities as util
//

#pragma once
#include "Vec3.h"

class Obstacle
{
public:

    //    class Obstacle:
    //        def __init__(self):
    //            self.tri_mesh = None
    //            # Seems hackish, related to Draw.temp_camera_lookat and the way Open3D
    //            # does translation relative to center of geometry.
    //            self.original_center = Vec3()
    
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

    //
    //        # Where a ray (Agent's path) will intersect the obstacle, or None.
    //        def ray_intersection(self, origin, tangent, body_radius):
    //            pass

    // Where a ray (Agent's path) will intersect the obstacle, or None.
    virtual Vec3 ray_intersection(const Vec3& origin,
                                  const Vec3& tangent,
                                  double body_radius) const {return Vec3();}

    //        # Normal to the obstacle at a given point of interest.
    //        def normal_at_poi(self, poi, agent_position=None):
    //            pass

    // Normal to the obstacle at a given point of interest.
    virtual Vec3 normal_at_poi(const Vec3& poi) const
    {
        return normal_at_poi(poi, Vec3::none());
    }
    virtual Vec3 normal_at_poi(const Vec3& poi,
                               const Vec3& agent_position) const {return Vec3();}

    //        # Point on surface of obstacle nearest the given query_point
    //        def nearest_point(self, query_point):
    //            pass

    // Point on surface of obstacle nearest the given query_point.
    virtual Vec3 nearest_point(const Vec3& query_point) const { return Vec3(); }

    //        # Compute direction for agent's static avoidance of nearby obstacles.
    //        def fly_away(self, agent_position, agent_forward, max_distance, body_radius):
    //            pass

    // Compute direction for agent's static avoidance of nearby obstacles.
    virtual Vec3 fly_away(const Vec3& agent_position,
                          const Vec3& agent_forward,
                          double max_distance,
                          double body_radius) const { return Vec3(); }
    
    //        def draw(self):
    //            pass

    virtual void draw() const {}

    //        def __str__(self):
    //            return self.__class__.__name__
    
    virtual std::string to_string() const { return "Obstacle"; }
    
    static void unit_test();  // Defined at bottom of file.
};



class EvertedSphereObstacle : public Obstacle
{
private:
    
    double radius_ = 1;
    Vec3 center_;
    
    
public:
    
    //
    //    class EvertedSphereObstacle(Obstacle):
    //        def __init__(self, radius, center):
    //            Obstacle.__init__(self)
    //            self.radius = radius
    //            self.center = center
    
    EvertedSphereObstacle(double radius, const Vec3& center)
      : Obstacle(), radius_(radius), center_(center) {}
    
    //        # Where a ray (Agent's path) will intersect the obstacle, or None.
    //        def ray_intersection(self, origin, tangent, body_radius):
    //            return shape.ray_sphere_intersection(origin, tangent,
    //                                                 self.radius, self.center)
    
    
    // Where a ray (Agent's path) will intersect the obstacle, or None.
    Vec3 ray_intersection(const Vec3& origin,
                          const Vec3& tangent,
                          double body_radius) const override
    {
        return shape::ray_sphere_intersection(origin, tangent, radius_, center_);
    }

    
    //        # Normal to the obstacle at a given point of interest.
    //        def normal_at_poi(self, poi, agent_position=None):
    //            return (self.center - poi).normalize()
    
    // Normal to the obstacle at a given point of interest.
    Vec3 normal_at_poi(const Vec3& poi,
                       const Vec3& agent_position) const override
    {
        return (center_ - poi).normalize();
    }
    
    //        # Point on surface of obstacle nearest the given query_point
    //        def nearest_point(self, query_point):
    //            return (query_point - self.center).normalize() * self.radius
    
    // Point on surface of obstacle nearest the given query_point.
    Vec3 nearest_point(const Vec3& query_point) const override
    {
        return (query_point - center_).normalize() * radius_;
    }
    
    
    //        # Compute direction for agent's static avoidance of nearby obstacles.
    //        def fly_away(self, agent_position, agent_forward, max_distance, body_radius):
    //            avoidance = Vec3()
    //            p = agent_position
    //            r = self.radius
    //            c = self.center
    //            offset_to_sphere_center = c - p
    //            distance_to_sphere_center = offset_to_sphere_center.length()
    //            dist_from_wall = r - distance_to_sphere_center
    //            # Close enough to obstacle surface to use static replusion.
    //            if dist_from_wall < max_distance:
    //                normal = offset_to_sphere_center / distance_to_sphere_center
    //                # Unless agent is already facing away from obstacle.
    //                if normal.dot(agent_forward) < 0.9:
    //                    # Weighting falls off further from obstacle surface
    //                    weight = 1 - (dist_from_wall / max_distance)
    //                    avoidance = normal * weight
    //            return avoidance
    
    // Compute direction for agent's static avoidance of nearby obstacles.
    Vec3 fly_away(const Vec3& agent_position,
                  const Vec3& agent_forward,
                  double max_distance,
                  double body_radius) const override
    {
//        avoidance = Vec3()
        Vec3 avoidance;

//        p = agent_position
//        r = self.radius
//        c = self.center
        Vec3 p = agent_position;
        double r = radius_;
        Vec3 c = center_;

//        offset_to_sphere_center = c - p
//        distance_to_sphere_center = offset_to_sphere_center.length()
//        dist_from_wall = r - distance_to_sphere_center

        Vec3 offset_to_sphere_center = c - p;
        double distance_to_sphere_center = offset_to_sphere_center.length();
        double dist_from_wall = r - distance_to_sphere_center;

//        // Close enough to obstacle surface to use static replusion.
//        if dist_from_wall < max_distance:
//            normal = offset_to_sphere_center / distance_to_sphere_center

        // Close enough to obstacle surface to use static replusion.
        if (dist_from_wall < max_distance)
        {
            Vec3 normal = offset_to_sphere_center / distance_to_sphere_center;
            
//            // Unless agent is already facing away from obstacle.
//            if normal.dot(agent_forward) < 0.9:
//            // Weighting falls off further from obstacle surface
//            weight = 1 - (dist_from_wall / max_distance)
//            avoidance = normal * weight

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

    
    //        def draw(self):
    //            if not self.tri_mesh:
    //                self.tri_mesh = Draw.make_everted_sphere(self.radius, self.center)
    //            Draw.adjust_static_scene_object(self.tri_mesh)
    
    void draw() const override {}

    std::string to_string() const override { return "EvertedSphereObstacle"; }

    
};

class PlaneObstacle : public Obstacle
{
private:
    
    Vec3 normal_;
    Vec3 center_;
    
public:
    
    
    //    class PlaneObstacle(Obstacle):
    //        def __init__(self, normal=Vec3(0, 1, 0), center=Vec3(0, 0, 0)):
    //            Obstacle.__init__(self)
    //            self.normal = normal
    //            self.center = center
    
    PlaneObstacle()
      : Obstacle(), normal_(Vec3(0, 1, 0)), center_(Vec3()) {}
    
    PlaneObstacle(const Vec3& normal, const Vec3& center)
      : Obstacle(), normal_(normal), center_(center) {}
    
    //        # Where a ray (Agent's path) will intersect the obstacle, or None.
    //        def ray_intersection(self, origin, tangent, body_radius):
    //            return shape.ray_plane_intersection(origin, tangent,
    //                                                self.center, self.normal)
    
    // Where a ray (Agent's path) will intersect the obstacle, or None.
    Vec3 ray_intersection(const Vec3& origin,
                          const Vec3& tangent,
                          double body_radius) const override
    {
        return shape::ray_plane_intersection(origin, tangent, center_, normal_);
    }
    
    
    //        # Normal to the obstacle at a given point of interest.
    //        def normal_at_poi(self, poi, agent_position=None):
    //            normal = self.normal
    //            # If a reference position is given.
    //            if agent_position:
    //                # Project it to obstacle surface.
    //                on_obstacle = self.nearest_point(agent_position)
    //                # Normalized vector FROM obstacle surface TOWARD agent.
    //                normal = (agent_position - on_obstacle).normalize()
    //            return normal

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

    //        # Point on surface of obstacle nearest the given query_point
    //        def nearest_point(self, query_point):
    //            # Offset from center point (origin) of plane.
    //            offset = query_point - self.center
    //            # Signed distance from plane.
    //            distance =  offset.dot(self.normal)
    //            # Translate offset point onto plane (in plane's local space).
    //            on_plane = offset - (self.normal * distance)
    //            # Translate back to global space.
    //            return on_plane + self.center

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

    
    //        # Compute direction for agent's static avoidance of nearby obstacles.
    //        def fly_away(self, agent_position, agent_forward, max_distance, body_radius):
    //            avoidance = Vec3()
    //            # Project agent_position to obstacle surface.
    //            on_obstacle = self.nearest_point(agent_position)
    //            dist_from_obstacle = (on_obstacle - agent_position).length()
    //            # Close enough to obstacle surface to use static replusion.
    //            if dist_from_obstacle < max_distance:
    //                normal = self.normal_at_poi(on_obstacle, agent_position)
    //                # Unless agent is already facing away from obstacle.
    //                if normal.dot(agent_forward) < 0.9:
    //                    # Weighting falls off further from obstacle surface
    //                    weight = 1 - (dist_from_obstacle / max_distance)
    //                    avoidance = normal * weight
    //            return avoidance
    
    
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

    std::string to_string() const override { return "PlaneObstacle"; }

};


class CylinderObstacle : public Obstacle
{
private:
    
    double radius_;
    Vec3 endpoint_;
    Vec3 tangent_;
    double length_;

    
public:
    

//    # A bounded cylinder (between two endpoints) with given radius
//    class CylinderObstacle(Obstacle):
//        def __init__(self, radius, endpoint0, endpoint1):
//            Obstacle.__init__(self)
//            self.radius = radius
//            self.endpoint = endpoint0
//            offset = endpoint1 - endpoint0
//            (self.tangent, self.length) = offset.normalize_and_length()

    CylinderObstacle(double radius, const Vec3& endpoint0, const Vec3& endpoint1)
      : Obstacle()
    {
        radius_ = radius;
        endpoint_ = endpoint0;
        Vec3 offset = endpoint1 - endpoint0;
        std::tie(tangent_, length_) = offset.normalize_and_length();
    }

    //        # Nearest point on the infinite line containing cylinder's axis.
    //        def nearest_point_on_axis(self, query_point):
    //            offset = query_point - self.endpoint
    //            projection = offset.dot(self.tangent)
    //            return self.endpoint + self.tangent * projection
    
    // Nearest point on the infinite line containing cylinder's axis.
    Vec3 nearest_point_on_axis(const Vec3& query_point) const
    {
        Vec3 offset = query_point - endpoint_;
        double projection = offset.dot(tangent_);
        return endpoint_ + tangent_ * projection;
    }
    
//        # Where a ray (Agent's path) will intersect the obstacle, or None.
//        def ray_intersection(self, origin, tangent, body_radius):
//            return shape.ray_cylinder_intersection(origin, tangent,
//                                                   self.endpoint, self.tangent,
//                                                   self.radius + 2 * body_radius,
//                                                   self.length)

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

//        # Normal to the obstacle at a given point of interest.
//        def normal_at_poi(self, poi, agent_position=None):
//            on_axis = self.nearest_point_on_axis(poi)
//            return (poi - on_axis).normalize()

    // Normal to the obstacle at a given point of interest.
    Vec3 normal_at_poi(const Vec3& poi,
                       const Vec3& agent_position) const override
    {
        Vec3 on_axis = nearest_point_on_axis(poi);
        return (poi - on_axis).normalize();
    }

//        # Point on surface of obstacle nearest the given query_point
//        def nearest_point(self, query_point):
//            on_axis = self.nearest_point_on_axis(query_point)
//            return on_axis + ((query_point - on_axis).normalize() * self.radius)

    // Point on surface of obstacle nearest the given query_point.
    Vec3 nearest_point(const Vec3& query_point) const override
    {
        Vec3 on_axis = nearest_point_on_axis(query_point);
        return on_axis + ((query_point - on_axis).normalize() * radius_);
    }

    

//        # Compute direction for agent's static avoidance of nearby obstacles.
//        def fly_away(self, agent_position, agent_forward, max_distance, body_radius):
//            avoidance = Vec3()
//            # Distance between this cylinder's axis and the agent's current path.
//            path_to_axis_dist = shape.distance_between_lines(agent_position,
//                                                             agent_forward,
//                                                             self.endpoint,
//                                                             self.tangent)
//            # When too close, avoidance is unit normal to cylinder surface.
//            margin = 3 * body_radius
//            if path_to_axis_dist < self.radius + margin:
//                on_surface = self.nearest_point(agent_position)
//                if (on_surface - agent_position).length_squared() < margin ** 2:
//                    avoidance = self.normal_at_poi(agent_position)
//            return avoidance

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
                avoidance = normal_at_poi(agent_position, Vec3::none());
            }
        }
        return avoidance;
    }

    
    
//        def draw(self):
//            if not self.tri_mesh:
//                self.tri_mesh = Draw.new_empty_tri_mesh()
//                Draw.add_line_segment(self.endpoint,
//                                      self.endpoint + (self.tangent * self.length),
//                                      color = Vec3(1, 1, 1) * 0.8,
//                                      radius = self.radius,
//                                      sides = 50,
//                                      tri_mesh = self.tri_mesh,
//                                      flat_end_caps=True)
//                self.tri_mesh.compute_vertex_normals()
//                self.original_center = Vec3.from_array(self.tri_mesh.get_center())
//            Draw.adjust_static_scene_object(self.tri_mesh, self.original_center)
    
    
    
    std::string to_string() const override { return "CylinderObstacle"; }
    
};


//
//    # Class to contain statistics of a predicted collision with an Obstacle.
//    class Collision:
//        def __init__(self,
//                     obstacle,
//                     time_to_collision,
//                     dist_to_collision,
//                     point_of_impact,
//                     normal_at_poi):
//            self.obstacle = obstacle
//            self.time_to_collision = time_to_collision
//            self.dist_to_collision = dist_to_collision
//            self.point_of_impact = point_of_impact
//            self.normal_at_poi = normal_at_poi


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
}
