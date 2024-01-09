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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//import math
//import numpy as np
//import Utilities as util

//class Vec3:
//"""Utility class for 3d vectors and operations on them."""

//# Instance constructor.
//def __init__(self, x=0, y=0, z=0):
//# This vector's 3d coordinates.
//self.x = x
//self.y = y
//self.z = z

// skip these?
//    # Alternate constructor, aka factory, to construct Vec3 from any array-like.
//    @staticmethod  # This decoration seems to be completely optional,
//    # but for the avoidance of doubt
//    def from_array(array_like):
//    a = np.asarray(array_like)
//    return Vec3(a[0], a[1], a[2])
//# Return contents of this Vec3 as an np.array.
//def asarray(self):
//return np.array([self.x, self.y, self.z])
//def __str__(self):
//return (self.__class__.__name__ + "(" +
//        str(self.x) + ", " + str(self.y) + ", " + str(self.z) + ")")

//def __eq__(self, v):
//return (isinstance(v, self.__class__) and
//        v.x == self.x and
//        v.y == self.y and
//        v.z == self.z)
//
//def __ne__(self, v):
//return not (self == v)

//def __lt__(self, v):
//return self.length() < v.length()

//def __add__(self, v):
//return Vec3(self.x + v.x, self.y + v.y, self.z + v.z)

//def __sub__(self, v):
//return Vec3(self.x - v.x, self.y - v.y, self.z - v.z)

//def __mul__(self, scale):
//return Vec3(self.x * scale, self.y * scale, self.z * scale)

// skipping this, assume the scale factor always comes after the vector
//def __rmul__(self, scale):
//return Vec3(self.x * scale, self.y * scale, self.z * scale)

//def __truediv__(self, scale):
//return Vec3(self.x / scale, self.y / scale, self.z / scale)

//def __neg__(self):
//return Vec3(-self.x, -self.y, -self.z)

//# Vector operations dot product, length (norm), normalize.
//def dot(self, v):
//return (self.x * v.x) + (self.y * v.y) + (self.z * v.z)
//def length(self):
//return math.sqrt(self.length_squared())
//def length_squared(self):
//return self.dot(self)
//def normalize(self):
//return self / self.length()

//# Normalize except if input is zero length, then return that.
//def normalize_or_0(self):
//return self if self.is_zero_length() else self.normalize()
//
//# Normalize and return length
//def normalize_and_length(self):
//original_length = self.length()
//return (self / original_length, original_length)

//# Fast check for unit length.
//def is_unit_length(self):
//return util.within_epsilon(self.length_squared(), 1)
//
//# Fast check for zero length.
//def is_zero_length(self):
//return util.within_epsilon(self.length_squared(), 0)

//# Returns vector parallel to "this" but no longer than "max_length"
//def truncate(self, max_length):
//len = self.length()
//return self if len <= max_length else self * (max_length / len)

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

//    bool is_unit_length() const
//        { return util_within_epsilon(length_squared(), 1); }
    bool is_unit_length() const
    {
        return within_epsilon(length_squared(), 1);
    }
    bool is_zero_length() const
        { return within_epsilon(length_squared(), 0); }

    Vec3 truncate(float max_length)
    {
        float len = length();
        return len <= max_length ? *this : *this * (max_length / len);
    }

    //    # return component of vector parallel to a unit basis vector
    //    def parallel_component(self, unit_basis):
    //        assert unit_basis.is_unit_length()
    //        projection = self.dot(unit_basis)
    //        return unit_basis * projection
    //
    //    # return component of vector perpendicular to a unit basis vector
    //    def perpendicular_component(self, unit_basis):
    //        return self - self.parallel_component(unit_basis)
    
    
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

    //
    //    # Cross product
    //    def cross(a, b):
    //        # (From https://en.wikipedia.org/wiki/Cross_product#Matrix_notation)
    //        return Vec3(a.y * b.z - a.z * b.y,
    //                    a.z * b.x - a.x * b.z,
    //                    a.x * b.y - a.y * b.x)

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
    
    
    //    # Get angle between two arbitrary direction vectors. (Visualize two vectors
    //    # placed tail to tail, the angle is measured on the plane containing both.
    //    # See https://commons.wikimedia.org/wiki/File:Inner-product-angle.svg)
    //    def angle_between(a, b):
    //        return math.acos(a.dot(b) / (a.length() * b.length()))
    //
    //    # Returns a vector describing a rotation around an arbitrary axis by a given
    //    # angle. The axis must pass through the global origin but any orientation is
    //    # allowed, as defined by the direction of the first argument. The return
    //    # value is effectively the axis with its length set to the angle (expressed
    //    # in radians). See https://en.wikipedia.org/wiki/Axis–angle_representation
    //    def axis_angle(axis, angle):
    //        aa = Vec3()
    //        if angle != 0 and axis.length_squared() > 0:
    //            aa = axis.normalize() * angle
    //        return aa
    //
    //    # Given two vectors, return an axis_angle that rotates the first to the
    //    # second. (Length of input vectors is irrelevant.)
    //    def rotate_vec_to_vec(from_vec, to_vec):
    //        return Vec3.axis_angle(Vec3.cross(from_vec, to_vec),
    //                               Vec3.angle_between(from_vec, to_vec))

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
    
    
    //    # Check if two vectors are perpendicular. (What about zero length?)
    //    def is_perpendicular(self, other):
    //        # TODO 20230430 Should it check for unit length, or normalize? For now,
    //        # assert that given vectors are unit length to see if it ever comes up.
    //        assert self.is_unit_length()
    //        assert other.is_unit_length()
    //        return util.within_epsilon(self.dot(other), 0)
    //
    //    # Check if two unit vectors are parallel (or anti-parallel).
    //    def is_parallel(self, other):
    //        # TODO 20230430 Should it check for unit length, or normalize? For now,
    //        # assert that given vectors are unit length to see if it ever comes up.
    //        assert self.is_unit_length()
    //        assert other.is_unit_length()
    //        return util.within_epsilon(abs(self.dot(other)), 1)
    //

    // Check if two vectors are perpendicular. (What about zero length?)
    bool is_perpendicular(const Vec3& other)
    {
        // TODO 20230430 Should it check for unit length, or normalize? For now,
        // assert that given vectors are unit length to see if it ever comes up.
        assert (is_unit_length());
        assert (other.is_unit_length());
        return within_epsilon(dot(other), 0);
    }
    
    // Check if two unit vectors are parallel (or anti-parallel).
    bool is_parallel(const Vec3& other)
    {
        // TODO 20230430 Should it check for unit length, or normalize? For now,
        // assert that given vectors are unit length to see if it ever comes up.
        assert (is_unit_length());
        assert (other.is_unit_length());
        return within_epsilon(abs(dot(other)), 1);
    }

    //    # Given a (unit) vector, return some vector that is purpendicular.
    //    # (There are infinitely many such vectors, one is chosen arbitrarily.)
    //    def find_perpendicular(self):
    //        perpendicular = None
    //        reference = Vec3(1, 0, 0) # Any vector NOT parallet to "self"
    //        # If "self" is parallel to initial "reference":
    //        if self.is_parallel(reference):
    //            # Return fixed perpendicular to initial "reference".
    //            perpendicular = Vec3(0, 1, 0)
    //        else:
    //            # return cross product with non-parallel "reference".
    //            perpendicular = self.cross(reference).normalize()
    //        return perpendicular
    //

//    // Given a (unit) vector, return some vector that is perpendicular.
//    // (There are infinitely many such vectors, one is chosen arbitrarily.)
//    Vec3 find_perpendicular()
//    {
//        Vec3 perpendicular;
//        Vec3 reference(1, 0, 0);  // Any vector NOT parallel to "this"
//        // If "self" is parallel to initial "reference":
//        if (is_parallel(reference))
//        {
//            // Return fixed perpendicular to initial "reference".
//            perpendicular = Vec3(0, 1, 0)
//        }
//        else
//        {
//            // return cross product with non-parallel "reference".
//            perpendicular = self.cross(reference).normalize()
//        }
//        return perpendicular
//    }

    // Given a (unit) vector, return some vector that is perpendicular.
    // (There are infinitely many such vectors, one is chosen arbitrarily.)
    Vec3 find_perpendicular()
    {
        Vec3 reference(1, 0, 0);  // Any vector NOT parallel to "this"
        return (is_parallel(reference) ? // parallel to initial "reference"?
                Vec3(0, 1, 0) :          // perpendicular to initial "reference"
                cross(reference).normalize());
    }

    //    # Check if two vectors are within epsilon of being equal.
    //    def is_equal_within_epsilon(self, other):
    //        bigger_epsilon = util.epsilon * 10  # Got occasional fail with default.
    //        return (util.within_epsilon(self.x, other.x, bigger_epsilon) and
    //                util.within_epsilon(self.y, other.y, bigger_epsilon) and
    //                util.within_epsilon(self.z, other.z, bigger_epsilon))
    //

    // Check if two vectors are within epsilon of being equal.
    bool is_equal_within_epsilon(const Vec3& other) const
    {
        // bigger_epsilon = util.epsilon * 10  # Got occasional fail with default.
//        float bigger_epsilon = 0; // TODO QQQ
//        float bigger_epsilon = 0.000001; // TODO QQQ
        float bigger_epsilon = epsilon; // TODO QQQ
        return (within_epsilon(x(), other.x(), bigger_epsilon) and
                within_epsilon(y(), other.y(), bigger_epsilon) and
                within_epsilon(z(), other.z(), bigger_epsilon));
    }
    static bool is_equal_within_epsilon(const Vec3& a, const Vec3& b)
    {
        return a.is_equal_within_epsilon(b);
    }

    
    //    # Rotate X and Y values about the Z axis by given angle.
    //    # This is used in combination with a LocalSpace transform to get model in
    //    # correct orientation. A more generalized "rotate about given axis by given
    //    # angle" might be nice to have for convenience.
    //    def rotate_xy_about_z(self, angle):
    //        s = math.sin(angle)
    //        c = math.cos(angle)
    //        return Vec3(self.x * c + self.y * s,
    //                    self.y * c - self.x * s,
    //                    self.z)
    //
    //    def rotate_xz_about_y(self, angle):
    //        s = math.sin(angle)
    //        c = math.cos(angle)
    //        return Vec3(self.x * c + self.z * s,
    //                    self.y,
    //                    self.z * c - self.x * s)
    //

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
    
    //    # class RandomSequence
    //    # Vec3 randomUnitVector();
    //    # does Python allow the trick where RandomSequence is defined one
    //    # place but RandomSequence::randomUnitVector() is defined elsewhere?
    //    # oh, maybe yes: https://stackoverflow.com/a/2982/1991373
    //
    //    # Generate a random point in an axis aligned box, given two opposite corners.
    //    @staticmethod
    //    def random_point_in_axis_aligned_box(a, b):
    //        return Vec3(util.random2(min(a.x, b.x), max(a.x, b.x)),
    //                    util.random2(min(a.y, b.y), max(a.y, b.y)),
    //                    util.random2(min(a.z, b.z), max(a.z, b.z)))
    //
    //    # Generate a random point inside a unit diameter disk centered on origin.
    //    @staticmethod
    //    def random_point_in_unit_radius_sphere():
    //        v = None
    //        while True:
    //            v = Vec3.random_point_in_axis_aligned_box(Vec3(-1, -1, -1),
    //                                                      Vec3(1, 1, 1))
    //            if v.length() <= 1:
    //                break
    //        return v;
    //
    //    # Generate a random unit vector.
    //    @staticmethod
    //    def random_unit_vector():
    //        v = None
    //        m = None
    //        while True:
    //            v = Vec3.random_point_in_unit_radius_sphere()
    //            m = v.length()
    //            if m > 0:
    //                break
    //        return v / m;

//    //    # class RandomSequence
//    //    # Vec3 randomUnitVector();
//    //    # does Python allow the trick where RandomSequence is defined one
//    //    # place but RandomSequence::randomUnitVector() is defined elsewhere?
//    //    # oh, maybe yes: https://stackoverflow.com/a/2982/1991373
//
//    // Generate a random point in an axis aligned box, given two opposite corners.
//    static Vec3 random_point_in_axis_aligned_box(const Vec3& a, const Vec3& b)
//    {
//        return Vec3(util_random2(std::min(a.x(), b.x()), std::max(a.x(), b.x())),
//                    util_random2(std::min(a.y(), b.y()), std::max(a.y(), b.y())),
//                    util_random2(std::min(a.z(), b.z()), std::max(a.z(), b.z())));
//    }
//    
//    // Generate a random point inside a unit diameter disk centered on origin.
//    static Vec3 random_point_in_unit_radius_sphere()
//    {
//        Vec3 v;
//        do
//        {
//            v = Vec3::random_point_in_axis_aligned_box(Vec3(-1, -1, -1),
//                                                       Vec3(1, 1, 1));
//            debugPrint(v.length())
//        }
//        while (v.length_squared() > 1);
//        return v;
//    }
//
//    // Generate a random unit vector.
//    static Vec3 random_unit_vector()
//    {
//        Vec3 v;
//        float m = 0;
//        do
//        {
//            v = Vec3::random_point_in_unit_radius_sphere();
//            m = v.length();
//        }
//        while (m == 0);
//        return v / m;
//    }

    //
    //    # Given any number of Vec3s, return the one with the max length.
    //    @staticmethod
    //    def max(*any_number_of_Vec3s):
    //        longest = Vec3()
    //        magnitude2 = 0
    //        for v in any_number_of_Vec3s:
    //            vm2 = v.length_squared()
    //            if magnitude2 < vm2:
    //                magnitude2 = vm2
    //                longest = v
    //        return longest
    //


    // TODO 20240107 started to implement this as a variadic function, but then
    //               could not recall if Vec3::max() is still used.
    // Given any number of Vec3s, return the one with the max length.
//    static Vec3 max(const Vec3 any_number_of_Vec3s...)
//    {
//        va_list args;
//        va_start(args, any_number_of_Vec3s);
//
//        
//        
//        longest = Vec3()
//        magnitude2 = 0
//        for v in any_number_of_Vec3s:
//            vm2 = v.length_squared()
//            if magnitude2 < vm2:
//                magnitude2 = vm2
//                longest = v
//        return longest
//    }

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

    
//    // just for reference:
//
//    int sum_variadic(int count, ...)
//    {
//        va_list args;
//        
//        va_start(args, count);
//        int sum = 0;
//        for (int i = 0; i < count; i++)
//        {
//            sum += va_arg(args, int);
//        }
//        va_end(args);
//        return sum;
//    }
//
//    void simple_printf(const char* fmt...) // C-style "const char* fmt, ..." is also valid
//    {
//        va_list args;
//        va_start(args, fmt);
//        
//        while (*fmt != '\0')
//        {
//            if (*fmt == 'd')
//            {
//                int i = va_arg(args, int);
//                std::cout << i << '\n';
//            }
//            else if (*fmt == 'c')
//            {
//                // note automatic conversion to integral type
//                int c = va_arg(args, int);
//                std::cout << static_cast<char>(c) << '\n';
//            }
//            else if (*fmt == 'f')
//            {
//                double d = va_arg(args, double);
//                std::cout << d << '\n';
//            }
//            ++fmt;
//        }
//        
//        va_end(args);
//    }

    
    ////////////////////////////////////////////////////////////////////////////
    
    
//    std::string to_string()
//    {
//        std::stringstream xss;
//        std::stringstream yss;
//        std::stringstream zss;
//        xss << x() << ", ";
//        yss << y() << ", ";
//        zss << z() << ")";
////        return "Vec3(" + xss.str() + xss.str() + xss.str();
//        return "Vec3(" + xss.str() + xss.str() + xss.str() + ")";
//    }
    
//    std::string to_string() const
//    {
//        std::stringstream xss;
//        std::stringstream yss;
//        std::stringstream zss;
//        xss << x();
//        yss << y();
//        zss << z();
//        std::string c = ", ";
//        return "Vec3(" + xss.str() + c + yss.str() + c + zss.str() + ")";
//    }

    std::string to_string() const
    {
//        std::stringstream xss;
//        std::stringstream yss;
//        std::stringstream zss;
        std::stringstream ss;
//        xss << x();
//        yss << y();
//        zss << z();
//        std::string c = ", ";
//        return "Vec3(" + xss.str() + c + yss.str() + c + zss.str() + ")";
        
        ss << "Vec3(" << x() << ", " << y() << ", " << z() << ")";
        return ss.str();
    }

    //    // Serialize Vec2 object to stream.
    //    inline std::ostream& operator<<(std::ostream& os, const Vec3& v)
    //    {
    //        os << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
    //        return os;
    //    }

    
    

    ////////////////////////////////////////////////////////////////////////////

    static void unit_test_out_of_line();

    static void unit_test();

private:
    float x_ = 0;
    float y_ = 0;
    float z_ = 0;
};

//    // Serialize Vec2 object to stream.
//    inline std::ostream& operator<<(std::ostream& os, const Vec3& v)
//    {
//        os << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
//        return os;
//    }

// Serialize Vec2 object to stream.
inline std::ostream& operator<<(std::ostream& os, const Vec3& v)
{
    os << v.to_string();
    return os;
}



//    # class RandomSequence
//    # Vec3 randomUnitVector();
//    # does Python allow the trick where RandomSequence is defined one
//    # place but RandomSequence::randomUnitVector() is defined elsewhere?
//    # oh, maybe yes: https://stackoverflow.com/a/2982/1991373

//    // Generate a random point in an axis aligned box, given two opposite corners.
//    //static Vec3 random_point_in_axis_aligned_box(const Vec3& a, const Vec3& b)
//    inline Vec3 random_point_in_axis_aligned_box(const Vec3& a, const Vec3& b)
//    {
//        return Vec3(util_random2(std::min(a.x(), b.x()), std::max(a.x(), b.x())),
//                    util_random2(std::min(a.y(), b.y()), std::max(a.y(), b.y())),
//                    util_random2(std::min(a.z(), b.z()), std::max(a.z(), b.z())));
//    }

// Generate a random point in an axis aligned box, given two opposite corners.
inline Vec3 RandomSequence::random_point_in_axis_aligned_box(Vec3 a, Vec3 b)
{
    return Vec3(random2(std::min(a.x(), b.x()), std::max(a.x(), b.x())),
                random2(std::min(a.y(), b.y()), std::max(a.y(), b.y())),
                random2(std::min(a.z(), b.z()), std::max(a.z(), b.z())));
}


//    // Generate a random point inside a unit diameter disk centered on origin.
//    static Vec3 random_point_in_unit_radius_sphere()
//    {
//        Vec3 v;
//        do
//        {
//            v = Vec3::random_point_in_axis_aligned_box(Vec3(-1, -1, -1),
//                                                       Vec3(1, 1, 1));
//            debugPrint(v.length())
//        }
//        while (v.length_squared() > 1);
//        return v;
//    }
//
//    // Generate a random unit vector.
//    static Vec3 random_unit_vector()
//    {
//        Vec3 v;
//        float m = 0;
//        do
//        {
//            v = Vec3::random_point_in_unit_radius_sphere();
//            m = v.length();
//        }
//        while (m == 0);
//        return v / m;
//    }

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


//    // TODO 20240108 QQQ


inline void Vec3::unit_test_out_of_line()
{
    RandomSequence rs;
    rs.randomUnitVector();
    rs.random_unit_vector();
}


//    @staticmethod
//    def unit_test():

inline void Vec3::unit_test()
{
    //        v000 = Vec3(0, 0, 0)
    //        v100 = Vec3(1, 0, 0)
    //        v010 = Vec3(0, 1, 0)
    //        v001 = Vec3(0, 0, 1)
    //        v011 = Vec3(0, 1, 1)
    //        v101 = Vec3(1, 0, 1)
    //        v110 = Vec3(1, 1, 0)
    //        v111 = Vec3(1, 1, 1)
    //        v123 = Vec3(1, 2, 3)
    //        v236 = Vec3(2, 3, 6)
    
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
    
    //        assert str(Vec3(1, 2, 3)) == 'Vec3(1, 2, 3)'
    //        assert Vec3(1, 2, 3) == Vec3(1, 2, 3)
    //        assert Vec3(0, 0, 0) != Vec3(1, 0, 0)
    //        assert Vec3(0, 0, 0) != Vec3(0, 1, 0)
    //        assert Vec3(0, 0, 0) != Vec3(0, 0, 1)
    //        assert Vec3(1, 2, 3) == Vec3.from_array([1, 2, 3])
    //        assert np.array_equal(Vec3(1, 2, 3).asarray(), [1, 2, 3])
    //        assert Vec3(1, 2, 3).dot(Vec3(4, 5, 6)) == 32
    //        assert Vec3(2, 3, 6).length() == 7
    //        assert Vec3(2, 3, 6).normalize() == Vec3(2, 3, 6) / 7
    //        assert Vec3(3, 0, 0).truncate(2) == Vec3(2, 0, 0)
    //        assert Vec3(1, 0, 0).truncate(2) == Vec3(1, 0, 0)
    //        assert Vec3(1, 2, 3) + Vec3(4, 5, 6) == Vec3(5, 7, 9)
    //        assert Vec3(5, 7, 9) - Vec3(4, 5, 6) == Vec3(1, 2, 3)
    //        assert -Vec3(1, 2, 3) == Vec3(-1, -2, -3)
    //        assert Vec3(1, 2, 3) * 2 == Vec3(2, 4, 6)
    //        assert 2 * Vec3(1, 2, 3) == Vec3(2, 4, 6)
    //        assert Vec3(2, 4, 6) / 2 == Vec3(1, 2, 3)
    //        assert Vec3(1, 2, 3) < Vec3(-1, -2, -4)
    
    
    //        std::stringstream ss;
    //        ss << Vec3(1, 2, 3);
    //        assert (str(Vec3(1, 2, 3)) == 'Vec3(1, 2, 3)');
    //        assert (ss.str() == "Vec3(1, 2, 3)");
    
    //        std::cout << "Vec3(1, 2, 3).to_string() = " << Vec3(1, 2, 3).to_string() << std::endl;
    
    assert(Vec3(1, 2, 3).to_string() == "Vec3(1, 2, 3)");
    assert (Vec3(1, 2, 3) == Vec3(1, 2, 3));
    assert (Vec3(0, 0, 0) != Vec3(1, 0, 0));
    assert (Vec3(0, 0, 0) != Vec3(0, 1, 0));
    assert (Vec3(0, 0, 0) != Vec3(0, 0, 1));
    //        assert (Vec3(1, 2, 3) == Vec3.from_array([1, 2, 3]));
    //        assert (np.array_equal(Vec3(1, 2, 3).asarray(), [1, 2, 3]));
    assert (Vec3(1, 2, 3).dot(Vec3(4, 5, 6)) == 32);
    assert (Vec3(2, 3, 6).length() == 7);
    assert (Vec3(2, 3, 6).normalize() == Vec3(2, 3, 6) / 7);
    assert (Vec3(3, 0, 0).truncate(2) == Vec3(2, 0, 0));
    assert (Vec3(1, 0, 0).truncate(2) == Vec3(1, 0, 0));
    assert (Vec3(1, 2, 3) + Vec3(4, 5, 6) == Vec3(5, 7, 9));
    assert (Vec3(5, 7, 9) - Vec3(4, 5, 6) == Vec3(1, 2, 3));
    assert (-Vec3(1, 2, 3) == Vec3(-1, -2, -3));
    assert (Vec3(1, 2, 3) * 2 == Vec3(2, 4, 6));
    //        assert (2 * Vec3(1, 2, 3) == Vec3(2, 4, 6));
    assert (Vec3(2, 4, 6) / 2 == Vec3(1, 2, 3));
    assert (Vec3(1, 2, 3) < Vec3(-1, -2, -4));
    
    
    //        (n, l) = v123.normalize_and_length()
    //        assert (n == v123.normalize()) and (l == v123.length())
    //
    //        assert v000.is_zero_length()
    //        assert not v111.is_zero_length()
    //
    //        assert v000.normalize_or_0() == v000
    //        assert v236.normalize_or_0() == Vec3(2, 3, 6) / 7
    //
    //        assert not v000.is_unit_length()
    //        assert not v111.is_unit_length()
    //        assert v123.normalize().is_unit_length()
    //
    //        assert Vec3.max(v000) == v000
    //        assert Vec3.max(v000, v111) == v111
    //        assert Vec3.max(v111, v000) == v111
    //        assert Vec3.max(v123, v111, v236, v000) == v236
    //
    //        for i in range(20):
    //            r = Vec3.random_point_in_axis_aligned_box(v236, v123)
    //            assert util.between(r.x, v123.x, v236.x)
    //            assert util.between(r.y, v123.y, v236.y)
    //            assert util.between(r.z, v123.z, v236.z)
    //
    //            r = Vec3.random_point_in_unit_radius_sphere()
    //            assert r.length() <= 1
    //
    //            r = Vec3.random_unit_vector()
    //            assert util.within_epsilon(r.length(), 1)
    
    auto [n, l] = v123.normalize_and_length();
    assert ((n == v123.normalize()) && (l == v123.length()));
    
    assert (v000.is_zero_length());
    assert (! v111.is_zero_length());
    
    assert (v000.normalize_or_0() == v000);
    assert (v236.normalize_or_0() == Vec3(2, 3, 6) / 7);
    
    assert (! v000.is_unit_length());
    assert (! v111.is_unit_length());
    //        debugPrint(v123.normalize().is_unit_length())
    //        debugPrint(v123.to_string())
    //        debugPrint(v123.normalize().to_string())
    //        debugPrint(v123.normalize().length())
    assert (v123.normalize().is_unit_length());
    
    //        Vec3 max
    //        assert Vec3.max(v000) == v000
    //        assert Vec3.max(v000, v111) == v111
    //        assert Vec3.max(v111, v000) == v111
    //        assert Vec3.max(v123, v111, v236, v000) == v236
    
    // Note that unlike in Python, these are wrapped in curly braces because
    // they are actually initializers for an std::vector<Vec3>.
    assert (Vec3::max({v000}) == v000);
    assert (Vec3::max({v000, v111}) == v111);
    assert (Vec3::max({v111, v000}) == v111);
    assert (Vec3::max({v123, v111, v236, v000}) == v236);
    
    //        for i in range(20)
    //        {
    //            r = Vec3.random_point_in_axis_aligned_box(v236, v123)
    //            assert util.between(r.x, v123.x, v236.x)
    //            assert util.between(r.y, v123.y, v236.y)
    //            assert util.between(r.z, v123.z, v236.z)
    //
    //            r = Vec3.random_point_in_unit_radius_sphere()
    //            assert r.length() <= 1
    //
    //            r = Vec3.random_unit_vector()
    //            assert util.within_epsilon(r.length(), 1)
    //
    //        }
    
    
    // TODO 20240108 QQQ
    RandomSequence rs;
    for (int i = 0; i < 20; i++)
    {
        Vec3 r = rs.random_point_in_axis_aligned_box(v236, v123);
        assert (between(r.x(), v123.x(), v236.x()));
        assert (between(r.y(), v123.y(), v236.y()));
        assert (between(r.z(), v123.z(), v236.z()));
        
        r = rs.random_point_in_unit_radius_sphere();
        assert (r.length() <= 1);
        
        r = rs.random_unit_vector();
        assert (within_epsilon(r.length(), 1));
    }

//    std::cout << "YAY" << std::endl;
    
    //        f33 = 0.3333333333333334
    //        f66 = 0.6666666666666665
    //        x_norm = Vec3(1, 0, 0)
    //        diag_norm = Vec3(1, 1, 1).normalize()
    //        assert Vec3(2, 4, 8).parallel_component(x_norm) == Vec3(2, 0, 0)
    //        assert Vec3(2, 4, 8).perpendicular_component(x_norm) == Vec3(0, 4, 8)
    //        assert x_norm.parallel_component(diag_norm) == Vec3(f33, f33, f33)
    //        assert x_norm.perpendicular_component(diag_norm) == Vec3(f66, -f33, -f33)

    float f33 = 0.3333333333333334;
    float f66 = 0.6666666666666665;
    Vec3 x_norm(1, 0, 0);
    Vec3 diag_norm = Vec3(1, 1, 1).normalize();
    assert (Vec3(2, 4, 8).parallel_component(x_norm) == Vec3(2, 0, 0));
    assert (Vec3(2, 4, 8).perpendicular_component(x_norm) == Vec3(0, 4, 8));
//    assert (x_norm.parallel_component(diag_norm) == Vec3(f33, f33, f33));
//    assert (x_norm.perpendicular_component(diag_norm) == Vec3(f66, -f33, -f33));
    assert (Vec3::is_equal_within_epsilon(x_norm.parallel_component(diag_norm),
                                          Vec3(f33, f33, f33)));
    assert (Vec3::is_equal_within_epsilon(x_norm.perpendicular_component(diag_norm),
                                          Vec3(f66, -f33, -f33)));

    //        a = Vec3(1, 2, 3).normalize()
    //        b = Vec3(-9, 7, 5).normalize()
    //        c = Vec3(1, 0, 0)
    //        assert a.is_parallel(a)
    //        assert a.is_parallel(-a)
    //        assert not a.is_parallel(b)
    //        assert a.is_perpendicular(a.find_perpendicular())
    //        assert b.is_perpendicular(b.find_perpendicular())
    //        assert c.is_perpendicular(c.find_perpendicular())
    //        assert not a.is_perpendicular(b)
    
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

    //        e = Vec3(2, 4, 8)
    //        f = Vec3(2, 4, 8 - util.epsilon / 2)
    //        assert e.is_equal_within_epsilon(e)
    //        assert e.is_equal_within_epsilon(f)

    Vec3 e = Vec3(2, 4, 8);
//    Vec3 f = Vec3(2, 4, 8 - epsilon / 2);
    Vec3 f = Vec3(2, 4, 8 - epsilon * 2);
    assert (e.is_equal_within_epsilon(e));
    assert (! e.is_equal_within_epsilon(f));

    //        i = Vec3(1, 0, 0)
    //        j = Vec3(0, 1, 0)
    //        k = Vec3(0, 0, 1)
    //        assert i.cross(j) == k
    //        assert j.cross(k) == i
    //        assert k.cross(i) == j
    //        assert i.cross(k) == -j
    //        assert j.cross(i) == -k
    //        assert k.cross(j) == -i
    
    Vec3 i(1, 0, 0);
    Vec3 j(0, 1, 0);
    Vec3 k(0, 0, 1);
    assert (i.cross(j) == k);
    assert (j.cross(k) == i);
    assert (k.cross(i) == j);
    assert (i.cross(k) == -j);
    assert (j.cross(i) == -k);
    assert (k.cross(j) == -i);


    //        pi2 = math.pi / 2
    //        pi3 = math.pi / 3
    //        pi4 = math.pi / 4
    //        pi5 = math.pi / 5
    //        ang = math.acos(1 / math.sqrt(3))
    //
    //        assert k.angle_between(k) == 0
    //        assert i.angle_between(j) == pi2
    //        assert util.within_epsilon(i.angle_between(Vec3(1, 1, 0)), pi4)
    //
    //        assert Vec3.axis_angle(v100, math.pi) == v100 * math.pi
    //        assert Vec3.axis_angle(v111, pi3) == v111.normalize() * pi3
    //
    //        assert Vec3.rotate_vec_to_vec(i, j) == v001 * pi2
    //        assert Vec3.is_equal_within_epsilon(Vec3.rotate_vec_to_vec(v100, v110),
    //                                            Vec3.axis_angle(v001, pi4))
    //        assert Vec3.is_equal_within_epsilon(Vec3.rotate_vec_to_vec(v111, v001),
    //                                            Vec3.axis_angle(Vec3(1,-1,0), ang))
    //
    //        spi3 = math.sqrt(3) / 2                         # sin(60°), sin(pi/3)
    //        cpi3 = 0.5                                      # cos(60°), cos(pi/3)
    //        spi5 = math.sqrt((5 / 8) - (math.sqrt(5) / 8))  # sin(36°), sin(pi/5)
    //        cpi5 = (1 + math.sqrt(5)) / 4                   # cos(36°), cos(pi/5)
    //
    //        assert Vec3.is_equal_within_epsilon(v111.rotate_xy_about_z(pi2),
    //                                            Vec3(1, -1, 1))
    //        assert Vec3.is_equal_within_epsilon(v111.rotate_xy_about_z(pi3),
    //                                            Vec3(cpi3 + spi3, cpi3 - spi3, 1))
    //        assert Vec3.is_equal_within_epsilon(v111.rotate_xy_about_z(pi5),
    //                                            Vec3(cpi5 + spi5, cpi5 - spi5, 1))
    //
    //        assert Vec3.is_equal_within_epsilon(v111.rotate_xz_about_y(pi2),
    //                                            Vec3(1, 1, -1))
    //        assert Vec3.is_equal_within_epsilon(v111.rotate_xz_about_y(pi3),
    //                                            Vec3(cpi3 + spi3, 1, cpi3 - spi3))
    //        assert Vec3.is_equal_within_epsilon(v111.rotate_xz_about_y(pi5),
    //                                            Vec3(cpi5 + spi5, 1, cpi5 - spi5))

    float pi = std::acos(-1);
    float pi2 = pi / 2;
    float pi3 = pi / 3;
    float pi4 = pi / 4;
    float pi5 = pi / 5;
    float ang = std::acos(1 / std::sqrt(3));
    
//    assert ((k.angle_between(k) == 0));
//    assert i.angle_between(j) == pi2
//    assert util.within_epsilon(i.angle_between(Vec3(1, 1, 0)), pi4)
    assert (Vec3::angle_between(k, k) == 0);
    assert (Vec3::angle_between(i, j) == pi2);
    assert (within_epsilon(Vec3::angle_between(i, Vec3(1, 1, 0)), pi4));

    assert (Vec3::axis_angle(v100, pi) == v100 * pi);
    assert (Vec3::axis_angle(v111, pi3) == v111.normalize() * pi3);
    
    assert (Vec3::rotate_vec_to_vec(i, j) == v001 * pi2);
    assert (Vec3::is_equal_within_epsilon(Vec3::rotate_vec_to_vec(v100, v110),
                                          Vec3::axis_angle(v001, pi4)));
    assert (Vec3::is_equal_within_epsilon(Vec3::rotate_vec_to_vec(v111, v001),
                                          Vec3::axis_angle(Vec3(1,-1,0), ang)));

    
//        float spi3 = std::sqrt(3) / 2;                         // sin(60°), sin(pi/3)
//        float cpi3 = 0.5;                                      // cos(60°), cos(pi/3)
//
//    //        spi5 = math.sqrt((5 / 8) - (math.sqrt(5) / 8))   # sin(36°), sin(pi/5)
//    //  float spi5 = std::sqrt((5 / 8) - (std::sqrt(5) / 8));  // sin(36°), sin(pi/5)
//    //    float spi5 = std::sin(pi5);
//      float spi5 = std::sqrt((5.0 / 8) - (std::sqrt(5) / 8)); // sin(36°), sin(pi/5)
//
//        float cpi5 = (1 + std::sqrt(5)) / 4;                   // cos(36°), cos(pi/5)

    float spi3 = std::sqrt(3) / 2;                         // sin(60°), sin(pi/3)
    float cpi3 = 0.5;                                      // cos(60°), cos(pi/3)
    float spi5 = std::sqrt((5.0 / 8) - (std::sqrt(5) / 8));// sin(36°), sin(pi/5)
    float cpi5 = (1 + std::sqrt(5)) / 4;                   // cos(36°), cos(pi/5)

    
    
//    //    debugPrint(spi3)
//    //    debugPrint(cpi3)
//        debugPrint(std::sin(pi5))
//        debugPrint(spi5)
//    //    debugPrint(cpi5)
//
//        debugPrint(std::sqrt(5) / 8)
//        debugPrint((5 / 8) - (std::sqrt(5) / 8))

    assert (Vec3::is_equal_within_epsilon(v111.rotate_xy_about_z(pi2),
                                          Vec3(1, -1, 1)));
    assert (Vec3::is_equal_within_epsilon(v111.rotate_xy_about_z(pi3),
                                          Vec3(cpi3 + spi3, cpi3 - spi3, 1)));
    
//        debugPrint(v111.rotate_xy_about_z(pi5))
//        debugPrint(Vec3(cpi5 + spi5, cpi5 - spi5, 1))
//    //    Prints:
//    //    v111.rotate_xy_about_z(pi5) = Vec3(1.3968, 0.221232, 1)
//    //    Vec3(cpi5 + spi5, cpi5 - spi5, 1) = Vec3(nan, nan, 1)
    
    assert (Vec3::is_equal_within_epsilon(v111.rotate_xy_about_z(pi5),
                                          Vec3(cpi5 + spi5, cpi5 - spi5, 1)));
    
    
    assert (Vec3::is_equal_within_epsilon(v111.rotate_xz_about_y(pi2),
                                         Vec3(1, 1, -1)));
    assert (Vec3::is_equal_within_epsilon(v111.rotate_xz_about_y(pi3),
                                         Vec3(cpi3 + spi3, 1, cpi3 - spi3)));
    assert (Vec3::is_equal_within_epsilon(v111.rotate_xz_about_y(pi5),
                                         Vec3(cpi5 + spi5, 1, cpi5 - spi5)));

    //        v = Vec3(4, 5, 6)
    //        v += Vec3(1, 2, 3)
    //        assert v == Vec3(5, 7, 9), 'Vec3: test +='
    //        v = Vec3(5, 7, 9)
    //        v -= Vec3(4, 5, 6)
    //        assert v == Vec3(1, 2, 3), 'Vec3: test -='
    //        v = Vec3(1, 2, 3)
    //        v *= 2
    //        assert v == Vec3(2, 4, 6), 'Vec3: test *='
    //        v = Vec3(2, 4, 6)
    //        v /= 2
    //        assert v == Vec3(1, 2, 3), 'Vec3: test /='

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
    

    //        # assert unmodified:
    //        assert v000 == Vec3(0, 0, 0)
    //        assert v100 == Vec3(1, 0, 0)
    //        assert v010 == Vec3(0, 1, 0)
    //        assert v001 == Vec3(0, 0, 1)
    //        assert v011 == Vec3(0, 1, 1)
    //        assert v101 == Vec3(1, 0, 1)
    //        assert v110 == Vec3(1, 1, 0)
    //        assert v111 == Vec3(1, 1, 1)
    //        assert v123 == Vec3(1, 2, 3)
    //        assert v236 == Vec3(2, 3, 6)

    // assert unmodified:
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

////////////////////////////////////////////////////////////////////////////////

//
//
//
//    # return component of vector parallel to a unit basis vector
//    def parallel_component(self, unit_basis):
//    assert unit_basis.is_unit_length()
//    projection = self.dot(unit_basis)
//    return unit_basis * projection
//
//    # return component of vector perpendicular to a unit basis vector
//    def perpendicular_component(self, unit_basis):
//    return self - self.parallel_component(unit_basis)
//
//    # Cross product
//    def cross(a, b):
//    # (From https://en.wikipedia.org/wiki/Cross_product#Matrix_notation)
//    return Vec3(a.y * b.z - a.z * b.y,
//                a.z * b.x - a.x * b.z,
//                a.x * b.y - a.y * b.x)
//
//    # Get angle between two arbitrary direction vectors. (Visualize two vectors
//    # placed tail to tail, the angle is measured on the plane containing both.
//    # See https://commons.wikimedia.org/wiki/File:Inner-product-angle.svg)
//    def angle_between(a, b):
//    return math.acos(a.dot(b) / (a.length() * b.length()))
//
//    # Returns a vector describing a rotation around an arbitrary axis by a given
//    # angle. The axis must pass through the global origin but any orientation is
//    # allowed, as defined by the direction of the first argument. The return
//    # value is effectively the axis with its length set to the angle (expressed
//    # in radians). See https://en.wikipedia.org/wiki/Axis–angle_representation
//    def axis_angle(axis, angle):
//    aa = Vec3()
//    if angle != 0 and axis.length_squared() > 0:
//    aa = axis.normalize() * angle
//    return aa
//
//    # Given two vectors, return an axis_angle that rotates the first to the
//    # second. (Length of input vectors is irrelevant.)
//    def rotate_vec_to_vec(from_vec, to_vec):
//    return Vec3.axis_angle(Vec3.cross(from_vec, to_vec),
//                           Vec3.angle_between(from_vec, to_vec))
//
//
//    # Check if two vectors are perpendicular. (What about zero length?)
//    def is_perpendicular(self, other):
//    # TODO 20230430 Should it check for unit length, or normalize? For now,
//    # assert that given vectors are unit length to see if it ever comes up.
//    assert self.is_unit_length()
//    assert other.is_unit_length()
//    return util.within_epsilon(self.dot(other), 0)
//
//    # Check if two unit vectors are parallel (or anti-parallel).
//    def is_parallel(self, other):
//    # TODO 20230430 Should it check for unit length, or normalize? For now,
//    # assert that given vectors are unit length to see if it ever comes up.
//    assert self.is_unit_length()
//    assert other.is_unit_length()
//    return util.within_epsilon(abs(self.dot(other)), 1)
//
//    # Given a (unit) vector, return some vector that is purpendicular.
//    # (There are infinitely many such vectors, one is chosen arbitrarily.)
//    def find_perpendicular(self):
//    perpendicular = None
//    reference = Vec3(1, 0, 0) # Any vector NOT parallet to "self"
//    # If "self" is parallel to initial "reference":
//    if self.is_parallel(reference):
//    # Return fixed perpendicular to initial "reference".
//    perpendicular = Vec3(0, 1, 0)
//    else:
//    # return cross product with non-parallel "reference".
//    perpendicular = self.cross(reference).normalize()
//    return perpendicular
//
//    # Check if two vectors are within epsilon of being equal.
//    def is_equal_within_epsilon(self, other):
//    bigger_epsilon = util.epsilon * 10  # Got occasional fail with default.
//    return (util.within_epsilon(self.x, other.x, bigger_epsilon) and
//            util.within_epsilon(self.y, other.y, bigger_epsilon) and
//            util.within_epsilon(self.z, other.z, bigger_epsilon))
//
//    # Rotate X and Y values about the Z axis by given angle.
//    # This is used in combination with a LocalSpace transform to get model in
//    # correct orientation. A more generalized "rotate about given axis by given
//    # angle" might be nice to have for convenience.
//    def rotate_xy_about_z(self, angle):
//    s = math.sin(angle)
//    c = math.cos(angle)
//    return Vec3(self.x * c + self.y * s,
//                self.y * c - self.x * s,
//                self.z)
//
//    def rotate_xz_about_y(self, angle):
//    s = math.sin(angle)
//    c = math.cos(angle)
//    return Vec3(self.x * c + self.z * s,
//                self.y,
//                self.z * c - self.x * s)
//
//    # class RandomSequence
//    # Vec3 randomUnitVector();
//    # does Python allow the trick where RandomSequence is defined one
//    # place but RandomSequence::randomUnitVector() is defined elsewhere?
//    # oh, maybe yes: https://stackoverflow.com/a/2982/1991373
//
//    # Generate a random point in an axis aligned box, given two opposite corners.
//    @staticmethod
//    def random_point_in_axis_aligned_box(a, b):
//    return Vec3(util.random2(min(a.x, b.x), max(a.x, b.x)),
//                util.random2(min(a.y, b.y), max(a.y, b.y)),
//                util.random2(min(a.z, b.z), max(a.z, b.z)))
//
//    # Generate a random point inside a unit diameter disk centered on origin.
//    @staticmethod
//    def random_point_in_unit_radius_sphere():
//    v = None
//    while True:
//    v = Vec3.random_point_in_axis_aligned_box(Vec3(-1, -1, -1),
//                                              Vec3(1, 1, 1))
//    if v.length() <= 1:
//    break
//    return v;
//
//    # Generate a random unit vector.
//    @staticmethod
//    def random_unit_vector():
//    v = None
//    m = None
//    while True:
//    v = Vec3.random_point_in_unit_radius_sphere()
//    m = v.length()
//    if m > 0:
//    break
//    return v / m;
//
//    # Given any number of Vec3s, return the one with the max length.
//    @staticmethod
//    def max(*any_number_of_Vec3s):
//    longest = Vec3()
//    magnitude2 = 0
//    for v in any_number_of_Vec3s:
//    vm2 = v.length_squared()
//    if magnitude2 < vm2:
//    magnitude2 = vm2
//    longest = v
//    return longest
//
//    @staticmethod
//    def unit_test():
//    v000 = Vec3(0, 0, 0)
//    v100 = Vec3(1, 0, 0)
//    v010 = Vec3(0, 1, 0)
//    v001 = Vec3(0, 0, 1)
//    v011 = Vec3(0, 1, 1)
//    v101 = Vec3(1, 0, 1)
//    v110 = Vec3(1, 1, 0)
//    v111 = Vec3(1, 1, 1)
//    v123 = Vec3(1, 2, 3)
//    v236 = Vec3(2, 3, 6)
//
//    assert str(Vec3(1, 2, 3)) == 'Vec3(1, 2, 3)'
//    assert Vec3(1, 2, 3) == Vec3(1, 2, 3)
//    assert Vec3(0, 0, 0) != Vec3(1, 0, 0)
//    assert Vec3(0, 0, 0) != Vec3(0, 1, 0)
//    assert Vec3(0, 0, 0) != Vec3(0, 0, 1)
//    assert Vec3(1, 2, 3) == Vec3.from_array([1, 2, 3])
//    assert np.array_equal(Vec3(1, 2, 3).asarray(), [1, 2, 3])
//    assert Vec3(1, 2, 3).dot(Vec3(4, 5, 6)) == 32
//    assert Vec3(2, 3, 6).length() == 7
//    assert Vec3(2, 3, 6).normalize() == Vec3(2, 3, 6) / 7
//    assert Vec3(3, 0, 0).truncate(2) == Vec3(2, 0, 0)
//    assert Vec3(1, 0, 0).truncate(2) == Vec3(1, 0, 0)
//    assert Vec3(1, 2, 3) + Vec3(4, 5, 6) == Vec3(5, 7, 9)
//    assert Vec3(5, 7, 9) - Vec3(4, 5, 6) == Vec3(1, 2, 3)
//    assert -Vec3(1, 2, 3) == Vec3(-1, -2, -3)
//    assert Vec3(1, 2, 3) * 2 == Vec3(2, 4, 6)
//    assert 2 * Vec3(1, 2, 3) == Vec3(2, 4, 6)
//    assert Vec3(2, 4, 6) / 2 == Vec3(1, 2, 3)
//    assert Vec3(1, 2, 3) < Vec3(-1, -2, -4)
//
//    (n, l) = v123.normalize_and_length()
//    assert (n == v123.normalize()) and (l == v123.length())
//
//    assert v000.is_zero_length()
//    assert not v111.is_zero_length()
//
//    assert v000.normalize_or_0() == v000
//    assert v236.normalize_or_0() == Vec3(2, 3, 6) / 7
//
//    assert not v000.is_unit_length()
//    assert not v111.is_unit_length()
//    assert v123.normalize().is_unit_length()
//
//    assert Vec3.max(v000) == v000
//    assert Vec3.max(v000, v111) == v111
//    assert Vec3.max(v111, v000) == v111
//    assert Vec3.max(v123, v111, v236, v000) == v236
//
//    for i in range(20):
//    r = Vec3.random_point_in_axis_aligned_box(v236, v123)
//    assert util.between(r.x, v123.x, v236.x)
//    assert util.between(r.y, v123.y, v236.y)
//    assert util.between(r.z, v123.z, v236.z)
//
//    r = Vec3.random_point_in_unit_radius_sphere()
//    assert r.length() <= 1
//
//    r = Vec3.random_unit_vector()
//    assert util.within_epsilon(r.length(), 1)
//
//    f33 = 0.3333333333333334
//    f66 = 0.6666666666666665
//    x_norm = Vec3(1, 0, 0)
//    diag_norm = Vec3(1, 1, 1).normalize()
//    assert Vec3(2, 4, 8).parallel_component(x_norm) == Vec3(2, 0, 0)
//    assert Vec3(2, 4, 8).perpendicular_component(x_norm) == Vec3(0, 4, 8)
//    assert x_norm.parallel_component(diag_norm) == Vec3(f33, f33, f33)
//    assert x_norm.perpendicular_component(diag_norm) == Vec3(f66, -f33, -f33)
//
//    a = Vec3(1, 2, 3).normalize()
//    b = Vec3(-9, 7, 5).normalize()
//    c = Vec3(1, 0, 0)
//    assert a.is_parallel(a)
//    assert a.is_parallel(-a)
//    assert not a.is_parallel(b)
//    assert a.is_perpendicular(a.find_perpendicular())
//    assert b.is_perpendicular(b.find_perpendicular())
//    assert c.is_perpendicular(c.find_perpendicular())
//    assert not a.is_perpendicular(b)
//
//    e = Vec3(2, 4, 8)
//    f = Vec3(2, 4, 8 - util.epsilon / 2)
//    assert e.is_equal_within_epsilon(e)
//    assert e.is_equal_within_epsilon(f)
//
//    i = Vec3(1, 0, 0)
//    j = Vec3(0, 1, 0)
//    k = Vec3(0, 0, 1)
//    assert i.cross(j) == k
//    assert j.cross(k) == i
//    assert k.cross(i) == j
//    assert i.cross(k) == -j
//    assert j.cross(i) == -k
//    assert k.cross(j) == -i
//
//    pi2 = math.pi / 2
//    pi3 = math.pi / 3
//    pi4 = math.pi / 4
//    pi5 = math.pi / 5
//    ang = math.acos(1 / math.sqrt(3))
//
//    assert k.angle_between(k) == 0
//    assert i.angle_between(j) == pi2
//    assert util.within_epsilon(i.angle_between(Vec3(1, 1, 0)), pi4)
//
//    assert Vec3.axis_angle(v100, math.pi) == v100 * math.pi
//    assert Vec3.axis_angle(v111, pi3) == v111.normalize() * pi3
//
//    assert Vec3.rotate_vec_to_vec(i, j) == v001 * pi2
//    assert Vec3.is_equal_within_epsilon(Vec3.rotate_vec_to_vec(v100, v110),
//                                        Vec3.axis_angle(v001, pi4))
//    assert Vec3.is_equal_within_epsilon(Vec3.rotate_vec_to_vec(v111, v001),
//                                        Vec3.axis_angle(Vec3(1,-1,0), ang))
//
//    spi3 = math.sqrt(3) / 2                         # sin(60°), sin(pi/3)
//    cpi3 = 0.5                                      # cos(60°), cos(pi/3)
//    spi5 = math.sqrt((5 / 8) - (math.sqrt(5) / 8))  # sin(36°), sin(pi/5)
//    cpi5 = (1 + math.sqrt(5)) / 4                   # cos(36°), cos(pi/5)
//
//    assert Vec3.is_equal_within_epsilon(v111.rotate_xy_about_z(pi2),
//                                        Vec3(1, -1, 1))
//    assert Vec3.is_equal_within_epsilon(v111.rotate_xy_about_z(pi3),
//                                        Vec3(cpi3 + spi3, cpi3 - spi3, 1))
//    assert Vec3.is_equal_within_epsilon(v111.rotate_xy_about_z(pi5),
//                                        Vec3(cpi5 + spi5, cpi5 - spi5, 1))
//
//    assert Vec3.is_equal_within_epsilon(v111.rotate_xz_about_y(pi2),
//                                        Vec3(1, 1, -1))
//    assert Vec3.is_equal_within_epsilon(v111.rotate_xz_about_y(pi3),
//                                        Vec3(cpi3 + spi3, 1, cpi3 - spi3))
//    assert Vec3.is_equal_within_epsilon(v111.rotate_xz_about_y(pi5),
//                                        Vec3(cpi5 + spi5, 1, cpi5 - spi5))
//
//    v = Vec3(4, 5, 6)
//    v += Vec3(1, 2, 3)
//    assert v == Vec3(5, 7, 9), 'Vec3: test +='
//    v = Vec3(5, 7, 9)
//    v -= Vec3(4, 5, 6)
//    assert v == Vec3(1, 2, 3), 'Vec3: test -='
//    v = Vec3(1, 2, 3)
//    v *= 2
//    assert v == Vec3(2, 4, 6), 'Vec3: test *='
//    v = Vec3(2, 4, 6)
//    v /= 2
//    assert v == Vec3(1, 2, 3), 'Vec3: test /='
//
//    # assert unmodified:
//    assert v000 == Vec3(0, 0, 0)
//    assert v100 == Vec3(1, 0, 0)
//    assert v010 == Vec3(0, 1, 0)
//    assert v001 == Vec3(0, 0, 1)
//    assert v011 == Vec3(0, 1, 1)
//    assert v101 == Vec3(1, 0, 1)
//    assert v110 == Vec3(1, 1, 0)
//    assert v111 == Vec3(1, 1, 1)
//    assert v123 == Vec3(1, 2, 3)
//    assert v236 == Vec3(2, 3, 6)



//    #!/usr/bin/env python3
//    # -*- coding: utf-8 -*-
//    #-------------------------------------------------------------------------------
//    #
//    # Vec3.py -- new flock experiments
//    #
//    # Cartesian 3d vector space utility.
//    #
//    # MIT License -- Copyright © 2023 Craig Reynolds
//    #
//    #-------------------------------------------------------------------------------
//
//    import math
//    import numpy as np
//    import Utilities as util
//
//    class Vec3:
//    """Utility class for 3d vectors and operations on them."""
//
//    # Instance constructor.
//    def __init__(self, x=0, y=0, z=0):
//    # This vector's 3d coordinates.
//    self.x = x
//    self.y = y
//    self.z = z
//
//    # Alternate constructor, aka factory, to construct Vec3 from any array-like.
//    @staticmethod  # This decoration seems to be completely optional,
//               # but for the avoidance of doubt
//    def from_array(array_like):
//    a = np.asarray(array_like)
//    return Vec3(a[0], a[1], a[2])
//
//    # Return contents of this Vec3 as an np.array.
//    def asarray(self):
//    return np.array([self.x, self.y, self.z])
//
//    def __str__(self):
//    return (self.__class__.__name__ + "(" +
//            str(self.x) + ", " + str(self.y) + ", " + str(self.z) + ")")
//
//    def __eq__(self, v):
//    return (isinstance(v, self.__class__) and
//            v.x == self.x and
//            v.y == self.y and
//            v.z == self.z)
//
//    def __ne__(self, v):
//    return not (self == v)
//
//    def __lt__(self, v):
//    return self.length() < v.length()
//
//    def __add__(self, v):
//    return Vec3(self.x + v.x, self.y + v.y, self.z + v.z)
//
//    def __sub__(self, v):
//    return Vec3(self.x - v.x, self.y - v.y, self.z - v.z)
//
//    def __mul__(self, scale):
//    return Vec3(self.x * scale, self.y * scale, self.z * scale)
//
//    def __rmul__(self, scale):
//    return Vec3(self.x * scale, self.y * scale, self.z * scale)
//
//    def __truediv__(self, scale):
//    return Vec3(self.x / scale, self.y / scale, self.z / scale)
//
//    def __neg__(self):
//    return Vec3(-self.x, -self.y, -self.z)
//
//    # Vector operations dot product, length (norm), normalize.
//    def dot(self, v):
//    return (self.x * v.x) + (self.y * v.y) + (self.z * v.z)
//    def length(self):
//    return math.sqrt(self.length_squared())
//    def length_squared(self):
//    return self.dot(self)
//    def normalize(self):
//    return self / self.length()
//
//    # Normalize except if input is zero length, then return that.
//    def normalize_or_0(self):
//    return self if self.is_zero_length() else self.normalize()
//
//    # Normalize and return length
//    def normalize_and_length(self):
//    original_length = self.length()
//    return (self / original_length, original_length)
//
//    # Fast check for unit length.
//    def is_unit_length(self):
//    return util.within_epsilon(self.length_squared(), 1)
//
//    # Fast check for zero length.
//    def is_zero_length(self):
//    return util.within_epsilon(self.length_squared(), 0)
//
//    # Returns vector parallel to "this" but no longer than "max_length"
//    def truncate(self, max_length):
//    len = self.length()
//    return self if len <= max_length else self * (max_length / len)
//
//    # return component of vector parallel to a unit basis vector
//    def parallel_component(self, unit_basis):
//    assert unit_basis.is_unit_length()
//    projection = self.dot(unit_basis)
//    return unit_basis * projection
//
//    # return component of vector perpendicular to a unit basis vector
//    def perpendicular_component(self, unit_basis):
//    return self - self.parallel_component(unit_basis)
//
//    # Cross product
//    def cross(a, b):
//    # (From https://en.wikipedia.org/wiki/Cross_product#Matrix_notation)
//    return Vec3(a.y * b.z - a.z * b.y,
//                a.z * b.x - a.x * b.z,
//                a.x * b.y - a.y * b.x)
//
//    # Get angle between two arbitrary direction vectors. (Visualize two vectors
//    # placed tail to tail, the angle is measured on the plane containing both.
//    # See https://commons.wikimedia.org/wiki/File:Inner-product-angle.svg)
//    def angle_between(a, b):
//    return math.acos(a.dot(b) / (a.length() * b.length()))
//
//    # Returns a vector describing a rotation around an arbitrary axis by a given
//    # angle. The axis must pass through the global origin but any orientation is
//    # allowed, as defined by the direction of the first argument. The return
//    # value is effectively the axis with its length set to the angle (expressed
//    # in radians). See https://en.wikipedia.org/wiki/Axis–angle_representation
//    def axis_angle(axis, angle):
//    aa = Vec3()
//    if angle != 0 and axis.length_squared() > 0:
//        aa = axis.normalize() * angle
//    return aa
//
//    # Given two vectors, return an axis_angle that rotates the first to the
//    # second. (Length of input vectors is irrelevant.)
//    def rotate_vec_to_vec(from_vec, to_vec):
//    return Vec3.axis_angle(Vec3.cross(from_vec, to_vec),
//                           Vec3.angle_between(from_vec, to_vec))
//
//
//    # Check if two vectors are perpendicular. (What about zero length?)
//    def is_perpendicular(self, other):
//    # TODO 20230430 Should it check for unit length, or normalize? For now,
//    # assert that given vectors are unit length to see if it ever comes up.
//    assert self.is_unit_length()
//    assert other.is_unit_length()
//    return util.within_epsilon(self.dot(other), 0)
//
//    # Check if two unit vectors are parallel (or anti-parallel).
//    def is_parallel(self, other):
//    # TODO 20230430 Should it check for unit length, or normalize? For now,
//    # assert that given vectors are unit length to see if it ever comes up.
//    assert self.is_unit_length()
//    assert other.is_unit_length()
//    return util.within_epsilon(abs(self.dot(other)), 1)
//
//    # Given a (unit) vector, return some vector that is purpendicular.
//    # (There are infinitely many such vectors, one is chosen arbitrarily.)
//    def find_perpendicular(self):
//    perpendicular = None
//    reference = Vec3(1, 0, 0) # Any vector NOT parallet to "self"
//    # If "self" is parallel to initial "reference":
//    if self.is_parallel(reference):
//        # Return fixed perpendicular to initial "reference".
//        perpendicular = Vec3(0, 1, 0)
//    else:
//        # return cross product with non-parallel "reference".
//        perpendicular = self.cross(reference).normalize()
//    return perpendicular
//
//    # Check if two vectors are within epsilon of being equal.
//    def is_equal_within_epsilon(self, other):
//    bigger_epsilon = util.epsilon * 10  # Got occasional fail with default.
//    return (util.within_epsilon(self.x, other.x, bigger_epsilon) and
//            util.within_epsilon(self.y, other.y, bigger_epsilon) and
//            util.within_epsilon(self.z, other.z, bigger_epsilon))
//
//    # Rotate X and Y values about the Z axis by given angle.
//    # This is used in combination with a LocalSpace transform to get model in
//    # correct orientation. A more generalized "rotate about given axis by given
//    # angle" might be nice to have for convenience.
//    def rotate_xy_about_z(self, angle):
//    s = math.sin(angle)
//    c = math.cos(angle)
//    return Vec3(self.x * c + self.y * s,
//                self.y * c - self.x * s,
//                self.z)
//
//    def rotate_xz_about_y(self, angle):
//    s = math.sin(angle)
//    c = math.cos(angle)
//    return Vec3(self.x * c + self.z * s,
//                self.y,
//                self.z * c - self.x * s)
//
//    # class RandomSequence
//    # Vec3 randomUnitVector();
//    # does Python allow the trick where RandomSequence is defined one
//    # place but RandomSequence::randomUnitVector() is defined elsewhere?
//    # oh, maybe yes: https://stackoverflow.com/a/2982/1991373
//
//    # Generate a random point in an axis aligned box, given two opposite corners.
//    @staticmethod
//    def random_point_in_axis_aligned_box(a, b):
//    return Vec3(util.random2(min(a.x, b.x), max(a.x, b.x)),
//                util.random2(min(a.y, b.y), max(a.y, b.y)),
//                util.random2(min(a.z, b.z), max(a.z, b.z)))
//
//    # Generate a random point inside a unit diameter disk centered on origin.
//    @staticmethod
//    def random_point_in_unit_radius_sphere():
//    v = None
//    while True:
//        v = Vec3.random_point_in_axis_aligned_box(Vec3(-1, -1, -1),
//                                                  Vec3(1, 1, 1))
//        if v.length() <= 1:
//            break
//    return v;
//
//    # Generate a random unit vector.
//    @staticmethod
//    def random_unit_vector():
//    v = None
//    m = None
//    while True:
//        v = Vec3.random_point_in_unit_radius_sphere()
//        m = v.length()
//        if m > 0:
//            break
//    return v / m;
//
//    # Given any number of Vec3s, return the one with the max length.
//    @staticmethod
//    def max(*any_number_of_Vec3s):
//    longest = Vec3()
//    magnitude2 = 0
//    for v in any_number_of_Vec3s:
//        vm2 = v.length_squared()
//        if magnitude2 < vm2:
//            magnitude2 = vm2
//            longest = v
//    return longest
//
//    @staticmethod
//    def unit_test():
//    v000 = Vec3(0, 0, 0)
//    v100 = Vec3(1, 0, 0)
//    v010 = Vec3(0, 1, 0)
//    v001 = Vec3(0, 0, 1)
//    v011 = Vec3(0, 1, 1)
//    v101 = Vec3(1, 0, 1)
//    v110 = Vec3(1, 1, 0)
//    v111 = Vec3(1, 1, 1)
//    v123 = Vec3(1, 2, 3)
//    v236 = Vec3(2, 3, 6)
//
//    assert str(Vec3(1, 2, 3)) == 'Vec3(1, 2, 3)'
//    assert Vec3(1, 2, 3) == Vec3(1, 2, 3)
//    assert Vec3(0, 0, 0) != Vec3(1, 0, 0)
//    assert Vec3(0, 0, 0) != Vec3(0, 1, 0)
//    assert Vec3(0, 0, 0) != Vec3(0, 0, 1)
//    assert Vec3(1, 2, 3) == Vec3.from_array([1, 2, 3])
//    assert np.array_equal(Vec3(1, 2, 3).asarray(), [1, 2, 3])
//    assert Vec3(1, 2, 3).dot(Vec3(4, 5, 6)) == 32
//    assert Vec3(2, 3, 6).length() == 7
//    assert Vec3(2, 3, 6).normalize() == Vec3(2, 3, 6) / 7
//    assert Vec3(3, 0, 0).truncate(2) == Vec3(2, 0, 0)
//    assert Vec3(1, 0, 0).truncate(2) == Vec3(1, 0, 0)
//    assert Vec3(1, 2, 3) + Vec3(4, 5, 6) == Vec3(5, 7, 9)
//    assert Vec3(5, 7, 9) - Vec3(4, 5, 6) == Vec3(1, 2, 3)
//    assert -Vec3(1, 2, 3) == Vec3(-1, -2, -3)
//    assert Vec3(1, 2, 3) * 2 == Vec3(2, 4, 6)
//    assert 2 * Vec3(1, 2, 3) == Vec3(2, 4, 6)
//    assert Vec3(2, 4, 6) / 2 == Vec3(1, 2, 3)
//    assert Vec3(1, 2, 3) < Vec3(-1, -2, -4)
//
//    (n, l) = v123.normalize_and_length()
//    assert (n == v123.normalize()) and (l == v123.length())
//
//    assert v000.is_zero_length()
//    assert not v111.is_zero_length()
//
//    assert v000.normalize_or_0() == v000
//    assert v236.normalize_or_0() == Vec3(2, 3, 6) / 7
//
//    assert not v000.is_unit_length()
//    assert not v111.is_unit_length()
//    assert v123.normalize().is_unit_length()
//
//    assert Vec3.max(v000) == v000
//    assert Vec3.max(v000, v111) == v111
//    assert Vec3.max(v111, v000) == v111
//    assert Vec3.max(v123, v111, v236, v000) == v236
//
//    for i in range(20):
//        r = Vec3.random_point_in_axis_aligned_box(v236, v123)
//        assert util.between(r.x, v123.x, v236.x)
//        assert util.between(r.y, v123.y, v236.y)
//        assert util.between(r.z, v123.z, v236.z)
//
//        r = Vec3.random_point_in_unit_radius_sphere()
//        assert r.length() <= 1
//
//        r = Vec3.random_unit_vector()
//        assert util.within_epsilon(r.length(), 1)
//
//    f33 = 0.3333333333333334
//    f66 = 0.6666666666666665
//    x_norm = Vec3(1, 0, 0)
//    diag_norm = Vec3(1, 1, 1).normalize()
//    assert Vec3(2, 4, 8).parallel_component(x_norm) == Vec3(2, 0, 0)
//    assert Vec3(2, 4, 8).perpendicular_component(x_norm) == Vec3(0, 4, 8)
//    assert x_norm.parallel_component(diag_norm) == Vec3(f33, f33, f33)
//    assert x_norm.perpendicular_component(diag_norm) == Vec3(f66, -f33, -f33)
//
//    a = Vec3(1, 2, 3).normalize()
//    b = Vec3(-9, 7, 5).normalize()
//    c = Vec3(1, 0, 0)
//    assert a.is_parallel(a)
//    assert a.is_parallel(-a)
//    assert not a.is_parallel(b)
//    assert a.is_perpendicular(a.find_perpendicular())
//    assert b.is_perpendicular(b.find_perpendicular())
//    assert c.is_perpendicular(c.find_perpendicular())
//    assert not a.is_perpendicular(b)
//
//    e = Vec3(2, 4, 8)
//    f = Vec3(2, 4, 8 - util.epsilon / 2)
//    assert e.is_equal_within_epsilon(e)
//    assert e.is_equal_within_epsilon(f)
//
//    i = Vec3(1, 0, 0)
//    j = Vec3(0, 1, 0)
//    k = Vec3(0, 0, 1)
//    assert i.cross(j) == k
//    assert j.cross(k) == i
//    assert k.cross(i) == j
//    assert i.cross(k) == -j
//    assert j.cross(i) == -k
//    assert k.cross(j) == -i
//
//    pi2 = math.pi / 2
//    pi3 = math.pi / 3
//    pi4 = math.pi / 4
//    pi5 = math.pi / 5
//    ang = math.acos(1 / math.sqrt(3))
//
//    assert k.angle_between(k) == 0
//    assert i.angle_between(j) == pi2
//    assert util.within_epsilon(i.angle_between(Vec3(1, 1, 0)), pi4)
//
//    assert Vec3.axis_angle(v100, math.pi) == v100 * math.pi
//    assert Vec3.axis_angle(v111, pi3) == v111.normalize() * pi3
//
//    assert Vec3.rotate_vec_to_vec(i, j) == v001 * pi2
//    assert Vec3.is_equal_within_epsilon(Vec3.rotate_vec_to_vec(v100, v110),
//                                        Vec3.axis_angle(v001, pi4))
//    assert Vec3.is_equal_within_epsilon(Vec3.rotate_vec_to_vec(v111, v001),
//                                        Vec3.axis_angle(Vec3(1,-1,0), ang))
//
//    spi3 = math.sqrt(3) / 2                         # sin(60°), sin(pi/3)
//    cpi3 = 0.5                                      # cos(60°), cos(pi/3)
//    spi5 = math.sqrt((5 / 8) - (math.sqrt(5) / 8))  # sin(36°), sin(pi/5)
//    cpi5 = (1 + math.sqrt(5)) / 4                   # cos(36°), cos(pi/5)
//
//    assert Vec3.is_equal_within_epsilon(v111.rotate_xy_about_z(pi2),
//                                        Vec3(1, -1, 1))
//    assert Vec3.is_equal_within_epsilon(v111.rotate_xy_about_z(pi3),
//                                        Vec3(cpi3 + spi3, cpi3 - spi3, 1))
//    assert Vec3.is_equal_within_epsilon(v111.rotate_xy_about_z(pi5),
//                                        Vec3(cpi5 + spi5, cpi5 - spi5, 1))
//
//    assert Vec3.is_equal_within_epsilon(v111.rotate_xz_about_y(pi2),
//                                        Vec3(1, 1, -1))
//    assert Vec3.is_equal_within_epsilon(v111.rotate_xz_about_y(pi3),
//                                        Vec3(cpi3 + spi3, 1, cpi3 - spi3))
//    assert Vec3.is_equal_within_epsilon(v111.rotate_xz_about_y(pi5),
//                                        Vec3(cpi5 + spi5, 1, cpi5 - spi5))
//
//    v = Vec3(4, 5, 6)
//    v += Vec3(1, 2, 3)
//    assert v == Vec3(5, 7, 9), 'Vec3: test +='
//    v = Vec3(5, 7, 9)
//    v -= Vec3(4, 5, 6)
//    assert v == Vec3(1, 2, 3), 'Vec3: test -='
//    v = Vec3(1, 2, 3)
//    v *= 2
//    assert v == Vec3(2, 4, 6), 'Vec3: test *='
//    v = Vec3(2, 4, 6)
//    v /= 2
//    assert v == Vec3(1, 2, 3), 'Vec3: test /='
//
//    # assert unmodified:
//    assert v000 == Vec3(0, 0, 0)
//    assert v100 == Vec3(1, 0, 0)
//    assert v010 == Vec3(0, 1, 0)
//    assert v001 == Vec3(0, 0, 1)
//    assert v011 == Vec3(0, 1, 1)
//    assert v101 == Vec3(1, 0, 1)
//    assert v110 == Vec3(1, 1, 0)
//    assert v111 == Vec3(1, 1, 1)
//    assert v123 == Vec3(1, 2, 3)
//    assert v236 == Vec3(2, 3, 6)
