//------------------------------------------------------------------------------
//
//  Vec3.h -- new flock experiments
//
//  Cartesian 3d vector space utility.
//  This class is a Python → C++ translation of Vec3.py in "flock" project.
//
//  Created by Craig Reynolds on January 6, 2024.
//  MIT License -- Copyright © 2023 Craig Reynolds
//
//------------------------------------------------------------------------------

#pragma once
#include "Utilities.h"
#include <cmath>
#include <cassert>
#include <sstream>

class Vec3
{
public:
    
    // Constructors
    Vec3() {}
    Vec3(float x, float y, float z) : x_(x), y_(y), z_(z) {}

    // Accessors
    float x() const { return x_; }
    float y() const { return y_; }
    float z() const { return z_; }

    // Basic operators.
    bool operator==(const Vec3 v) const
        { return x() == v.x() && y() == v.y() && z() == v.z(); }
    bool operator!=(const Vec3 v) const
        { return x() != v.x() || y() != v.y() || z() != v.z(); }
    bool operator<(const Vec3 v) const { return length() < v.length(); }
    Vec3 operator+(Vec3 v) const
        { return { x() + v.x(), y() + v.y(), z() + v.z() }; }
    Vec3 operator-(Vec3 v) const
        { return { x() - v.x(), y() - v.y(), z() - v.z() }; }
    Vec3 operator*(float s) const { return { x() * s, y() * s, z() * s}; }
    Vec3 operator/(float s) const { return { x() / s, y() / s, z() / s}; }
    Vec3 operator-() const { return *this * -1; }
    Vec3 operator+=(const Vec3& rhs) { return *this = *this + rhs; }
    Vec3 operator-=(const Vec3& rhs) { return *this = *this - rhs; }
    Vec3 operator*=(float s) { return *this = *this * s; }
    Vec3 operator/=(float s) { return *this = *this / s; }

    // Vector operations dot product, length (norm), normalize.
    float dot(const Vec3& v) const
        { return x() * v.x() + y() * v.y() + z() * v.z(); }
    float length() const { return std::sqrt(sq(x()) + sq(y()) + sq(z())); }
    float length_squared() const { return sq(x()) + sq(y()) + sq(z()); }
    Vec3 normalize() const { return *this / length(); }
    
    // Normalize except if input is zero length, then return that.
    Vec3 normalize_or_0() const
        { return is_zero_length() ? *this : *this / length(); }

    // Normalize and return length
    // TODO in c++17 should be able to say:
    //      auto [normalize, length] = foo.normalize_and_length();
    std::tuple<Vec3, float> normalize_and_length()
    {
        float original_length = length();
        return  {*this / original_length, original_length};
    }

    bool is_unit_length() const
    {
        return util::within_epsilon(length_squared(), 1);
    }
    bool is_zero_length() const
    {
        return util::within_epsilon(length_squared(), 0);
    }

    // Returns vector parallel to "this" but no longer than "max_length"
    Vec3 truncate(float max_length)
    {
        float len = length();
        return len <= max_length ? *this : *this * (max_length / len);
    }
    
    // return component of vector parallel to a unit basis vector
    Vec3 parallel_component(const Vec3& unit_basis)
    {
        assert (unit_basis.is_unit_length());
        float projection = dot(unit_basis);
        return unit_basis * projection;
    }
    
    // return component of vector perpendicular to a unit basis vector
    Vec3 perpendicular_component(const Vec3& unit_basis)
    {
        return *this - parallel_component(unit_basis);
    }

    // Cross product (infix method, self is first arg)
    Vec3 cross(const Vec3& v) const { return Vec3::cross(*this, v); }

    // Cross product (static 2 arg version)
    static Vec3 cross(const Vec3& a, const Vec3& b)
    {
        // (From https://en.wikipedia.org/wiki/Cross_product#Matrix_notation)
        return Vec3(a.y() * b.z() - a.z() * b.y(),
                    a.z() * b.x() - a.x() * b.z(),
                    a.x() * b.y() - a.y() * b.x());
    }

    // Get angle between two arbitrary direction vectors. (Visualize two vectors
    // placed tail to tail, the angle is measured on the plane containing both.
    // See https://commons.wikimedia.org/wiki/File:Inner-product-angle.svg)
    static float angle_between(const Vec3& a, const Vec3& b)
    {
        return std::acos(a.dot(b) / (a.length() * b.length()));
    }
    
    // Returns a vector describing a rotation around an arbitrary axis by a given
    // angle. The axis must pass through the global origin but any orientation is
    // allowed, as defined by the direction of the first argument. The return
    // value is effectively the axis with its length set to the angle (expressed
    // in radians). See https://en.wikipedia.org/wiki/Axis–angle_representation
    static Vec3 axis_angle(const Vec3& axis, float angle)
    {
        Vec3 aa;
        if (angle != 0 && axis.length_squared() > 0)
        {
            aa = axis.normalize() * angle;
        }
        return aa;
    }
    
    // Given two vectors, return an axis_angle that rotates the first to the
    // second. (Length of input vectors is irrelevant.)
    static Vec3 rotate_vec_to_vec(const Vec3& from_vec, const Vec3& to_vec)
    {
        return Vec3::axis_angle(Vec3::cross(from_vec, to_vec),
                                Vec3::angle_between(from_vec, to_vec));
    }

    // Check if two vectors are perpendicular.
    bool is_perpendicular(const Vec3& other)
    {
        // TODO 20230430 Should it check for unit length, or normalize? For now,
        // assert that given vectors are unit length to see if it ever comes up.
        assert (is_unit_length());
        assert (other.is_unit_length());
        return util::within_epsilon(dot(other), 0);
    }
    
    // Check if two unit vectors are parallel (or anti-parallel).
    bool is_parallel(const Vec3& other)
    {
        // TODO 20230430 Should it check for unit length, or normalize? For now,
        // assert that given vectors are unit length to see if it ever comes up.
        assert (is_unit_length());
        assert (other.is_unit_length());
        return util::within_epsilon(abs(dot(other)), 1);
    }

    // Given a (unit) vector, return some vector that is perpendicular.
    // (There are infinitely many such vectors, one is chosen arbitrarily.)
    Vec3 find_perpendicular()
    {
        Vec3 reference(1, 0, 0);         // Any vector NOT parallel to "this"
        return (is_parallel(reference) ? // parallel to "reference"?
                Vec3(0, 1, 0) :          // perpendicular to "reference"
                cross(reference).normalize());
    }

//    // Check if two vectors are within epsilon of being equal.
//    bool is_equal_within_epsilon(const Vec3& other) const
//    {
//        // bigger_epsilon = util.epsilon * 10  # Got occasional fail with default.
//        float bigger_epsilon = util::epsilon; // TODO QQQ
//        return (util::within_epsilon(x(), other.x(), bigger_epsilon) and
//                util::within_epsilon(y(), other.y(), bigger_epsilon) and
//                util::within_epsilon(z(), other.z(), bigger_epsilon));
//    }
//    static bool is_equal_within_epsilon(const Vec3& a, const Vec3& b)
//    {
//        return a.is_equal_within_epsilon(b);
//    }
    
    // Check if two vectors are within epsilon of being equal.
    bool is_equal_within_epsilon(const Vec3& other,
                                 float epsilon=util::epsilon) const
    {
        // bigger_epsilon = util.epsilon * 10  # Got occasional fail with default.
//        float bigger_epsilon = util::epsilon; // TODO QQQ
//        return (util::within_epsilon(x(), other.x(), bigger_epsilon) and
//                util::within_epsilon(y(), other.y(), bigger_epsilon) and
//                util::within_epsilon(z(), other.z(), bigger_epsilon));
        return (util::within_epsilon(x(), other.x(), epsilon) and
                util::within_epsilon(y(), other.y(), epsilon) and
                util::within_epsilon(z(), other.z(), epsilon));
    }
    static bool is_equal_within_epsilon(const Vec3& a,
                                        const Vec3& b,
                                        float epsilon=util::epsilon)
    {
        return a.is_equal_within_epsilon(b, epsilon);
    }

    // Rotate X and Y values about the Z axis by given angle.
    // This is used in combination with a LocalSpace transform to get model in
    // correct orientation. A more generalized "rotate about given axis by given
    // angle" might be nice to have for convenience.
    Vec3 rotate_xy_about_z(float angle)
    {
        float s = std::sin(angle);
        float c = std::cos(angle);
        return Vec3(x() * c + y() * s, y() * c - x() * s, z());
    }
    
    Vec3 rotate_xz_about_y(float angle)
    {
        float s = std::sin(angle);
        float c = std::cos(angle);
        return Vec3(x() * c + z() * s, y(), z() * c - x() * s);
    }

    static Vec3 max(const std::vector<Vec3> any_number_of_Vec3s)
    {
        Vec3 longest;
        float magnitude2 = 0;
        
        for (const Vec3& v : any_number_of_Vec3s)
        {
            float vm2 = v.length_squared();
            if (magnitude2 < vm2)
            {
                magnitude2 = vm2;
                longest = v;
            }
        }
        
        return longest;
    }

    std::string to_string() const
    {
        std::stringstream ss;
        ss << "Vec3(" << x() << ", " << y() << ", " << z() << ")";
        return ss.str();
    }

    static void unit_test()
    {
        Vec3 v000(0, 0, 0);
        Vec3 v100(1, 0, 0);
        Vec3 v010(0, 1, 0);
        Vec3 v001(0, 0, 1);
        Vec3 v011(0, 1, 1);
        Vec3 v101(1, 0, 1);
        Vec3 v110(1, 1, 0);
        Vec3 v111(1, 1, 1);
        Vec3 v123(1, 2, 3);
        Vec3 v236(2, 3, 6);
        
        assert(Vec3(1, 2, 3).to_string() == "Vec3(1, 2, 3)");
        assert (Vec3(1, 2, 3) == Vec3(1, 2, 3));
        assert (Vec3(0, 0, 0) != Vec3(1, 0, 0));
        assert (Vec3(0, 0, 0) != Vec3(0, 1, 0));
        assert (Vec3(0, 0, 0) != Vec3(0, 0, 1));
        assert (Vec3(1, 2, 3).dot(Vec3(4, 5, 6)) == 32);
        assert (Vec3(2, 3, 6).length() == 7);
        assert (Vec3(2, 3, 6).normalize() == Vec3(2, 3, 6) / 7);
        assert (Vec3(3, 0, 0).truncate(2) == Vec3(2, 0, 0));
        assert (Vec3(1, 0, 0).truncate(2) == Vec3(1, 0, 0));
        assert (Vec3(1, 2, 3) + Vec3(4, 5, 6) == Vec3(5, 7, 9));
        assert (Vec3(5, 7, 9) - Vec3(4, 5, 6) == Vec3(1, 2, 3));
        assert (-Vec3(1, 2, 3) == Vec3(-1, -2, -3));
        assert (Vec3(1, 2, 3) * 2 == Vec3(2, 4, 6));
        assert (Vec3(2, 4, 6) / 2 == Vec3(1, 2, 3));
        assert (Vec3(1, 2, 3) < Vec3(-1, -2, -4));
        
        auto [n, l] = v123.normalize_and_length();
        assert ((n == v123.normalize()) && (l == v123.length()));
        
        assert (v000.is_zero_length());
        assert (! v111.is_zero_length());
        
        assert (v000.normalize_or_0() == v000);
        assert (v236.normalize_or_0() == Vec3(2, 3, 6) / 7);
        
        assert (! v000.is_unit_length());
        assert (! v111.is_unit_length());
        assert (v123.normalize().is_unit_length());
        
        // Note that unlike in Python, these are wrapped in curly braces because
        // they are actually initializers for an std::vector<Vec3>.
        assert (Vec3::max({v000}) == v000);
        assert (Vec3::max({v000, v111}) == v111);
        assert (Vec3::max({v111, v000}) == v111);
        assert (Vec3::max({v123, v111, v236, v000}) == v236);
        
        // Run 20 trials on each of three random vector utilities.
        RandomSequence rs;
        for (int i = 0; i < 20; i++)
        {
            Vec3 r = rs.random_point_in_axis_aligned_box(v236, v123);
            assert (util::between(r.x(), v123.x(), v236.x()));
            assert (util::between(r.y(), v123.y(), v236.y()));
            assert (util::between(r.z(), v123.z(), v236.z()));
            
            r = rs.random_point_in_unit_radius_sphere();
            assert (r.length() <= 1);
            
            r = rs.random_unit_vector();
            assert (util::within_epsilon(r.length(), 1));
        }
        
        float f33 = 0.3333333333333334;
        float f66 = 0.6666666666666665;
        Vec3 x_norm(1, 0, 0);
        Vec3 diag_norm = Vec3(1, 1, 1).normalize();
        assert (Vec3(2, 4, 8).parallel_component(x_norm) == Vec3(2, 0, 0));
        assert (Vec3(2, 4, 8).perpendicular_component(x_norm) == Vec3(0, 4, 8));
        assert (Vec3::is_equal_within_epsilon(x_norm.parallel_component(diag_norm),
                                              Vec3(f33, f33, f33)));
        assert (Vec3::is_equal_within_epsilon(x_norm.perpendicular_component(diag_norm),
                                              Vec3(f66, -f33, -f33)));
        
        Vec3 a = Vec3(1, 2, 3).normalize();
        Vec3 b = Vec3(-9, 7, 5).normalize();
        Vec3 c = Vec3(1, 0, 0);
        assert (a.is_parallel(a));
        assert (a.is_parallel(-a));
        assert (! a.is_parallel(b));
        assert (a.is_perpendicular(a.find_perpendicular()));
        assert (b.is_perpendicular(b.find_perpendicular()));
        assert (c.is_perpendicular(c.find_perpendicular()));
        assert (not a.is_perpendicular(b));
        
        Vec3 e = Vec3(2, 4, 8);
        Vec3 f = Vec3(2, 4, 8 - util::epsilon * 2);
        assert (e.is_equal_within_epsilon(e));
        assert (! e.is_equal_within_epsilon(f));
        
        Vec3 i(1, 0, 0);
        Vec3 j(0, 1, 0);
        Vec3 k(0, 0, 1);
        assert (i.cross(j) == k);
        assert (j.cross(k) == i);
        assert (k.cross(i) == j);
        assert (i.cross(k) == -j);
        assert (j.cross(i) == -k);
        assert (k.cross(j) == -i);
        
        float pi = std::acos(-1);
        float pi2 = pi / 2;
        float pi3 = pi / 3;
        float pi4 = pi / 4;
        float pi5 = pi / 5;
        float ang = std::acos(1 / std::sqrt(3));
        
        assert (Vec3::angle_between(k, k) == 0);
        assert (Vec3::angle_between(i, j) == pi2);
        assert (util::within_epsilon(Vec3::angle_between(i, Vec3(1, 1, 0)), pi4));
        
        assert (Vec3::axis_angle(v100, pi) == v100 * pi);
        assert (Vec3::axis_angle(v111, pi3) == v111.normalize() * pi3);
        
        assert (Vec3::rotate_vec_to_vec(i, j) == v001 * pi2);
        assert (Vec3::is_equal_within_epsilon(Vec3::rotate_vec_to_vec(v100, v110),
                                              Vec3::axis_angle(v001, pi4)));
        assert (Vec3::is_equal_within_epsilon(Vec3::rotate_vec_to_vec(v111, v001),
                                              Vec3::axis_angle(Vec3(1,-1,0), ang)));
        
        float spi3 = std::sqrt(3) / 2;                         // sin(60°), sin(pi/3)
        float cpi3 = 0.5;                                      // cos(60°), cos(pi/3)
        float spi5 = std::sqrt((5.0 / 8) - (std::sqrt(5) / 8));// sin(36°), sin(pi/5)
        float cpi5 = (1 + std::sqrt(5)) / 4;                   // cos(36°), cos(pi/5)
        assert (Vec3::is_equal_within_epsilon(v111.rotate_xy_about_z(pi2),
                                              Vec3(1, -1, 1)));
        assert (Vec3::is_equal_within_epsilon(v111.rotate_xy_about_z(pi3),
                                              Vec3(cpi3 + spi3, cpi3 - spi3, 1)));
        assert (Vec3::is_equal_within_epsilon(v111.rotate_xy_about_z(pi5),
                                              Vec3(cpi5 + spi5, cpi5 - spi5, 1)));
        assert (Vec3::is_equal_within_epsilon(v111.rotate_xz_about_y(pi2),
                                              Vec3(1, 1, -1)));
        assert (Vec3::is_equal_within_epsilon(v111.rotate_xz_about_y(pi3),
                                              Vec3(cpi3 + spi3, 1, cpi3 - spi3)));
        assert (Vec3::is_equal_within_epsilon(v111.rotate_xz_about_y(pi5),
                                              Vec3(cpi5 + spi5, 1, cpi5 - spi5)));
        
        Vec3 v;
        v = Vec3(4, 5, 6);
        v += Vec3(1, 2, 3);
        assert (v == Vec3(5, 7, 9) && "Vec3: test +=");
        v = Vec3(5, 7, 9);
        v -= Vec3(4, 5, 6);
        assert (v == Vec3(1, 2, 3) && "Vec3: test -=");
        v = Vec3(1, 2, 3);
        v *= 2;
        assert (v == Vec3(2, 4, 6) && "Vec3: test *=");
        v = Vec3(2, 4, 6);
        v /= 2;
        assert (v == Vec3(1, 2, 3) && "Vec3: test /=");
        
        // Verify unmodified:
        assert (v000 == Vec3(0, 0, 0));
        assert (v100 == Vec3(1, 0, 0));
        assert (v010 == Vec3(0, 1, 0));
        assert (v001 == Vec3(0, 0, 1));
        assert (v011 == Vec3(0, 1, 1));
        assert (v101 == Vec3(1, 0, 1));
        assert (v110 == Vec3(1, 1, 0));
        assert (v111 == Vec3(1, 1, 1));
        assert (v123 == Vec3(1, 2, 3));
        assert (v236 == Vec3(2, 3, 6));
    }
private:
    float x_ = 0;
    float y_ = 0;
    float z_ = 0;
};

// Serialize Vec2 object to stream.
inline std::ostream& operator<<(std::ostream& os, const Vec3& v)
{
    os << v.to_string();
    return os;
}


// Generate a random point in an axis aligned box, given two opposite corners.
inline Vec3 RandomSequence::random_point_in_axis_aligned_box(Vec3 a, Vec3 b)
{
    return Vec3(random2(std::min(a.x(), b.x()), std::max(a.x(), b.x())),
                random2(std::min(a.y(), b.y()), std::max(a.y(), b.y())),
                random2(std::min(a.z(), b.z()), std::max(a.z(), b.z())));
}

// Generate a random point inside a unit diameter disk centered on origin.
inline Vec3 RandomSequence::random_point_in_unit_radius_sphere()
{
    Vec3 v;
    do
    {
        v = random_point_in_axis_aligned_box(Vec3(-1, -1, -1), Vec3(1, 1, 1));
    }
    while (v.length() > 1);
    return v;
}

// Generate a random unit vector.
inline Vec3 RandomSequence::random_unit_vector()
{
    Vec3 v;
    float m = 0;
    do
    {
        v = random_point_in_unit_radius_sphere();
        m = v.length();
    }
    while (m == 0);
    return v / m;
}
