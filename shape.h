//------------------------------------------------------------------------------
//
//  shape.h -- new flock experiments
//
//  Geometric utilities to help describe 3d shapes, particularly for the
//  ray-shape intersection calculations used in obstacle avoidance.
//
//  Created by Craig Reynolds on January 20, 2024.
//  (Based on earlier C++ and Python versions.)
//  MIT License -- Copyright © 2024 Craig Reynolds
//------------------------------------------------------------------------------

#pragma once
#include "Vec3.h"

namespace shape
{

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20250127 update selected_boid_ / are "non-everted" spheres seen?
Vec3 rsi_p1;
Vec3 rsi_p2;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Returns the point of intersection of a ray (half-line) and sphere. Used
// for finding intersection of an Agent's "forward" axis with a spherical
// containment. Returns None if there is no intersection. Returns nearest
// intersection if there are two, eg Agent outside sphere flying toward it.
//
// Formulation from https://en.wikipedia.org/wiki/Line–sphere_intersection
// particularly the two equations under the text "Note that in the specific
// case where U is a unit vector..."
//
// Maybe this should be refactored into line_sphere_intersection() which returns
// a std::vector of intersections, and then a ray_sphere_intersection() wrapper
// that handles the ray logic?
//
Vec3 ray_sphere_intersection(const Vec3& ray_origin,
                             const Vec3& ray_tangent,
                             double sphere_radius,
                             const Vec3& sphere_center)
{
    Vec3 intersection = Vec3::none();
    assert(ray_tangent.is_unit_length());  // TODO 20240120 new, copy to python?
    Vec3 c = sphere_center;
    double r = sphere_radius;
    // Origin/endpoint and tangent (basis) of ray.
    Vec3 o = ray_origin;
    Vec3 u = ray_tangent;
    // Following derivation in Wikipedia.
    double delta = sq(u.dot(o - c)) - (sq((o - c).length()) - sq(r));
    // Does the line containing the ray intersect the sphere?
    if (delta >= 0)
    {
        // Find the 2 intersections of the line containing the ray.
        double sqrt_delta = std::sqrt(delta);
        double u_dot_oc = -(u.dot(o - c));
        double d1 = u_dot_oc + sqrt_delta;
        double d2 = u_dot_oc - sqrt_delta;
        Vec3 p1 = o + u * d1;
        Vec3 p2 = o + u * d2;
        // Select point on ("forward") ray, if both, use one nearer origin.
        if (d1 >= 0) { intersection = p1; }
        if (d2 >= 0 and d2 < d1) { intersection = p2; }
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20250127 update selected_boid_ / are "non-everted" spheres seen?
        rsi_p1 = p1;
        rsi_p2 = p2;
        
        // AH HA?! I think this shows there IS a tall vertical annotation line
        // from both of the intersection points, and the BLACK one is the FIRST
        // one along the ray, verifying this routine is working correctly. I think.
        //
        // This suggests the bug may be in SphereObstacle::normal_at_poi() which
        // -- indeed -- does not consider the signed_distance.
        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    }
    return intersection;
}


// Returns the point of intersection of a ray (half-line) and a plane. Or it
// returns None if there is no intersection because the line and plane are
// parallel. A ray represents an Agent's position and forward axis. Based
// upon: https://en.wikipedia.org/wiki/Line–plane_intersection#Algebraic_form
Vec3 ray_plane_intersection(const Vec3& ray_origin, const Vec3& ray_tangent,
                            const Vec3& plane_origin, const Vec3& plane_normal)
{
    Vec3 intersection = Vec3::none();
    double numerator = (plane_origin - ray_origin).dot(plane_normal);
    double denominator = ray_tangent.dot(plane_normal);
    if (denominator != 0)
    {
        double d = numerator / denominator;
        // When the intersection is "forward" of the ray_origin.
        if (d > 0) { intersection = ray_origin + ray_tangent * d; }
    }
    return intersection;
}


// Given a ray and a cylinder, find the intersection nearest the ray's origin
// (endpoint), or None.
//
// TODO Currently ignores the endpoints, assuming the cylinder is infinitely long.
//
// Using the derivation by Nominal Animal:
//     https://en.wikipedia.org/wiki/Line-cylinder_intersection
//     https://math.stackexchange.com/a/1732445/516283
//     https://www.nominal-animal.net
Vec3 ray_cylinder_intersection(const Vec3& ray_endpoint, const Vec3& ray_tangent,
                               const Vec3& cyl_endpoint, const Vec3& cyl_tangent,
                               double cyl_radius, double cyl_length)
{
    Vec3 intersection = Vec3::none();
    // Rename to match https://en.wikipedia.org/wiki/Line-cylinder_intersection
    Vec3 b = cyl_endpoint;  // The 3d cylinder endpoint on its axis.
    Vec3 a = cyl_tangent;   // Unit 3d vector parallel to cylinder axis.
    double r = cyl_radius;  // Scalar cylinder radius.
//    double h = cyl_length;  // Scalar length along axis between endpoints.
    Vec3 o = ray_endpoint;  // The 3d origin/end/endpoint of ray.
    Vec3 n = ray_tangent;   // Unit 3d vector parallel to ray.
    b = b - o;              // Offset b by the ray's origin/end/endpoint
    assert (a.is_unit_length());
    assert (n.is_unit_length());
    Vec3 na = n.cross(a);
    double radicand = (na.dot(na) * sq(r)) - sq(b.dot(na));
    // If any (real valued) intersections exist (both same if radicand==0)
    if (radicand >= 0)
    {
        double radical = std::sqrt(radicand);
        Vec3 ba = b.cross(a);
        double nana = na.dot(na);
        double naba = na.dot(ba);
        double d1 = (naba + radical) / nana;
        double d2 = (naba - radical) / nana;
        if (d1 >= 0)             { intersection = o + n * d1; }
        if (d2 >= 0 and d2 < d1) { intersection = o + n * d2; }
    }
    return intersection;
}

// Distance between two 3d lines. To provide a quick reject for an Agent's path
// and a CylinderObstacle when computing avoidance. This version is for lines.
// Could be made to work for rays and segments to provide better performance.
// Derivation from https://math.stackexchange.com/a/2217845/516283
//
double distance_between_lines(const Vec3& origin1, const Vec3& tangent1,
                              const Vec3& origin2, const Vec3& tangent2)
{
    double distance = 0;
    Vec3 rd = origin1 - origin2;
    if (tangent1.is_parallel(tangent2))
    {
        // Distance between parallel lines.
        Vec3 q = rd.cross(tangent1);
        distance = std::sqrt(q.dot(q) / sq(tangent2.length()));
    }
    else
    {
        // Distance between skew lines.
        Vec3 n = tangent1.cross(tangent2);
        distance = std::abs(n.dot(rd)) / n.length();
    }
    return distance;
}


// Very ad hoc tool used to create an obstacle set. Given a list of "radii", a
// "margin", and "max_distance" this function returns a list of sphere centers
// (corresponding to the given radii) such that:
//     (1) none of the spheres intersect/overlap AND that all spheres
//         are inside a sphere at the origin with radius "max_distance".
//     (2) there is at least "margin" clearance between any two spheres.
// Should this use a simple utility Sphere class? (Maybe use later in Obstacle?)
std::vector<Vec3> arrangeNonOverlappingSpheres(std::vector<double> radii,
                                               double margin,
                                               double max_distance)
{
    // These should be parameters:
    int iterations = 100;
    double rate = 0.6;
    // Generate initial random sphere centers (without regard to intersection).
    std::vector<Vec3> centers;
    max_distance -= margin / 2;  // Adjust for margins.
    for (auto& r : radii)
    {
        r += margin / 2;
        centers.push_back(EF::RS().randomPointInUnitRadiusSphere() * max_distance);
    }
    // Run relaxation loop, moving spheres incrementally away from overlaps.
    for (int i = 0; i < iterations; i++)
    {
        for (int a = 0; a < centers.size(); a++)
        {
            std::vector<Vec3> adjustments(centers.size());
            for (int b = 0; b < a; b++)
            {
                Vec3 offset_from_b = centers[a] - centers[b];
                double center_distance = offset_from_b.length();
                double separation = center_distance - (radii[a] + radii[b]);
                double overlap = margin - separation;
                if (overlap > 0)
                {
                    double adjust = overlap * rate;
                    adjustments.at(a) += offset_from_b.normalize() * -adjust;
                }
            }
            
            // Push toward origin if outside max_distance.
            double max_r = centers[a].length() + radii[a] + margin;
            double excess = max_r - max_distance;
            if (excess > 0)
            {
                adjustments[a] += centers[a].normalize() * excess;
            }
            // Adjust each center based on summed adjustments calculated above.
            for (int a = 0; a < centers.size(); a++)
            {
                centers[a] -= adjustments[a];
            }
        }
    }
    return centers;
}


// Defines a 3d (axis aligned) voxel map of given voxel count, overall box_size,
// and center. Calling add() with a given location (say a boid center) marks
// that voxel as occupied. In the context of EvoFlock that means "occupied by at
// least one boid on at least one simulation step." Calling fractionOccupied()
// returns the ratio of occupied voxels to all voxels
//
// I put this in shape.h for now, not sure this is the right place.
//
class OccupancyMap
{
public:
    
    OccupancyMap(Vec3 voxel_count_, Vec3 box_size_, Vec3 center_)
    {
        size_i = std::round(voxel_count_.x());
        size_j = std::round(voxel_count_.y());
        size_k = std::round(voxel_count_.z());
        box_size = box_size_;
        center = center_;
        voxels.resize(size_i * size_j * size_k, 0);
    }
        
    typedef std::function<bool(Vec3 p)> IgnoreFunction;

    void add(Vec3 position)
    {
        add(position,
            // This IgnoreFunction clips to bounds of OccupancyMap::box_size
            [this](Vec3 position){
            return not util::between(positionToVoxelIndex(position),
                                     0, voxels.size()); } );
    }

    void add(Vec3 position, IgnoreFunction ignore_function)
    {
        if (not ignore_function(position))
        {
            voxels.at(positionToVoxelIndex(position))++;
        }
    }

    double fractionOccupied() const
    {
        return fractionOccupied([](Vec3 position) { return false; });
    }
    
    double fractionOccupied(IgnoreFunction ignore_function) const
    {
        double occupied = 0;
        double visited = 0;
        for (int index = 0; index < voxels.size(); index++)
        {
            Vec3 voxel_center = voxelIndexToPosition(index);
            if (not ignore_function(voxel_center))
            {
                visited++;
                if (voxels.at(index) > 0) { occupied++; }
            }
        }
        return occupied / visited;
    }

    // Back-converts a voxel index to 3d position of the voxel center.
    Vec3 voxelIndexToPosition(size_t index) const
    {
        Vec3 ijk = voxelIndexToIJK(index);
        Vec3 scaled(box_size.x() * (ijk.x() / size_i),
                    box_size.y() * (ijk.y() / size_j),
                    box_size.z() * (ijk.z() / size_k));
        return scaled - center - (box_size / 2);
    }
    
    // Back-converts a voxel index to ijk voxel coordinates (as a Vec3).
    Vec3 voxelIndexToIJK(size_t index) const
    {
        size_t k = index / (size_i * size_j);
        size_t r = index % (size_i * size_j);
        size_t j = r / size_i;
        size_t i = r % size_i;
        return Vec3(i, j, k);
    }

    size_t positionToVoxelIndex(Vec3 position) const
    {
        Vec3 offset = position + center + (box_size / 2);
        return ijkToVoxelIndex((offset.x() / box_size.x()) * size_i,
                               (offset.y() / box_size.y()) * size_j,
                               (offset.z() / box_size.z()) * size_k);
    }

    size_t ijkToVoxelIndex(int i, int j, int k) const
    {
        return i + (j * size_i) + (k * size_i * size_j);
    }

    void print(bool single_digit_only = false) const
    {
        using namespace std::string_literals;
        std::cout << std::endl;
        for (int k = 0; k < size_k; k++)
        {
            for (int j = 0; j < size_j; j++)
            {
                for (int i = 0; i < size_i; i++)
                {
                    size_t index = ijkToVoxelIndex(i, j, k);
                    int count = voxels.at(index);
                    std::string str = std::to_string(count);
                    bool s = single_digit_only and not util::between(count,1,9);
                    bool in = voxelIndexToPosition(index).length() < 50;
                    std::cout << (in?(s?((count==0)?"•"s:"█"s):str):" ") << " ";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }
    }

    // TODO 20240324 just for testing
    void setVoxelIJK(int i, int j, int k, int count)
    {
        size_t index = ijkToVoxelIndex(i, j, k);
        voxels.at(index) = count;
    }

private:
    int size_i = 0;
    int size_j = 0;
    int size_k = 0;
    Vec3 box_size;
    Vec3 center;
    std::vector<int> voxels;
};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void unit_test()
{
    Vec3 zzz = Vec3(0, 0, 0);
    Vec3 ooo = Vec3(1, 1, 1);
    Vec3 ozz = Vec3(1, 0, 0);
    Vec3 zoz = Vec3(0, 1, 0);
    Vec3 zzo = Vec3(0, 0, 1);
    Vec3 zoo = Vec3(0, 1, 1);
    Vec3 ozo = Vec3(1, 0, 1);
    Vec3 ooz = Vec3(1, 1, 0);
    Vec3 mzz = Vec3(-1, 0, 0);
    Vec3 zmz = Vec3(0, -1, 0);
    Vec3 ddd = ooo.normalize();
    Vec3 ddz = ooz.normalize();

    // Unit tests for ray_sphere_intersection()
    auto rsi = [](Vec3 result, Vec3 ao, Vec3 at, double sr, Vec3 sc,
                  std::string description)
    {
        Vec3 intersection = ray_sphere_intersection(ao, at, sr, sc);
        if (not Vec3::equal_for_none(intersection, result))
        {
            std::cout << "ray_sphere_intersection() unit test -- "
                      << description << " -- expecting " << result.to_string()
                      << " but got " << intersection << std::endl;
            assert(intersection == result);
        }
    };
    rsi(ozz, ozz * 2, mzz, 1, zzz,
        "ray endpoint outside sphere (+x), pointing toward sphere");
    rsi(mzz, mzz * 2, ozz, 1, zzz,
        "ray endpoint outside sphere (-x), pointing toward sphere");
    rsi(Vec3::none(), mzz, zoz, 0.5, zzz,
        "clean miss, agent outside, +x of sphere, pointing up");
    rsi(ozz, ozz, zoz, 1, zzz,
        "intersect at point, ray origin on +x edge of sphere pointing up");
    rsi(ozz * 2, mzz, ozz, 2, zzz,
        "typical case inside r=2 sphere, ray at -1 on x axis pointing +x");
    rsi(ozz * std::sqrt(3), mzz, ozz, 2, zoz,
        "radius 2 sphere at (0,1,0), ray at (-1,0,0) pointing +x");

    
    // Unit tests for ray_plane_intersection()
    auto rpi = [](Vec3 result, Vec3 ro, Vec3 rt, Vec3 po, Vec3 pn)
    {
        Vec3 intersection = ray_plane_intersection(ro, rt, po, pn);
        if (not Vec3::equal_for_none(intersection, result))
        {
            assert(intersection == result);
        }
    };
    rpi(Vec3::none(), mzz, mzz, zzz, ozz);
    rpi(zzz, ozz * 2, ozz *-1, zzz, ozz);
    rpi(ozz * 3, zzz, ozz, ooo, ddd * -1);

    
    // Unit tests for ray_cylinder_intersection()
    auto rci = [](Vec3 result, Vec3 re, Vec3 rt, Vec3 ce, Vec3 ct,
                  double cr, double cl, std::string description)
    {
        Vec3 intersection = ray_cylinder_intersection(re, rt, ce, ct, cr, cl);
        if (not Vec3::equal_for_none(intersection, result))
        {
            std::cout << "shape.ray_cylinder_intersection() unit test -- "
                      << description << " -- expecting " << result
                      << " but got " << intersection << std::endl;
            assert(intersection == result);
        }
    };
    rci(ozz, ozz * 2, mzz, zmz, zoz, 1, 2,
        "ray endpoint outside cylinder (+x), pointing toward cylinder");
    
    // Unit tests for distance_between_lines()
    assert (2 == distance_between_lines(zoz, ozz, zmz, zzo));
    assert (0 == distance_between_lines(zzz, ozz, zzz, zzo));
    assert (0 == distance_between_lines(zzz, zoz, zoz, ozz));
    assert (2 == distance_between_lines(zzz, zoz, Vec3(5, 0, 2), ddz));
    // Parallel line case.
    assert (1 == distance_between_lines(zzz, zzo, ozz, zzo));
    // Closed form answer from:
    // https://onlinemschool.com/math/assistance/cartesian_coordinate/p_line/
    assert (std::sqrt(2) / std::sqrt(3) ==
            distance_between_lines(zzz, ddd, ozo, Vec3(-1, 0, 1).normalize()));
    
    // This 20240403 test was added to find a bug in ray_cylinder_intersection's
    // c++ rewrite. There is an "implied agent" at (x, 0, 0) where x sweeps from
    // -1.5 to +1.5. The agent's forward is (0, 0, 1) pointing parallel to the
    // +z axis. There is a vertical cylinder between (0, -1, 10) and (0, 1, 10).
    // When x is between -1 and 1, the intersection is at (x, 0, z) where z is
    // between 9 and 10.
    {
        Vec3 agent_forward(0, 0, 1);
        Vec3 cyl_endpoint(0, -1, 10);
        Vec3 cyl_tangent(0, 1, 0);
        double cyl_radius = 1;
        double cyl_length = 2;
        for (int i = -15; i <= 15; i++)
        {
            Vec3 agent_position(i * 0.1, 0, 0);
            Vec3 intersection = ray_cylinder_intersection(agent_position,
                                                          agent_forward,
                                                          cyl_endpoint,
                                                          cyl_tangent,
                                                          cyl_radius,
                                                          cyl_length);
            //std::cout << "agent_position = " << agent_position;
            //std::cout << ", intersection = " << intersection << std::endl;
            if (util::between(agent_position.x(), -1, 1 ))
            {
                assert(intersection.x() == agent_position.x());
                assert(intersection.y() == agent_position.y());
//                assert(intersection.z() >= 9);
                assert(util::between(intersection.z(), 9, 10));
            }
            else
            {
                assert(intersection.is_none());
            }
        }
    }
}

}  // end of namespace shape
