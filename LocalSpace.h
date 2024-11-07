//-------------------------------------------------------------------------------
//
//  LocalSpace.h -- new flock experiments
//
//  Local space (transformation) for a boid/agent.
//
//  Equivalent to a 4x4 homogeneous transformation with [0 0 0 1] as last column
//     [ ix iy iz 0 ]
//     [ jx jy jz 0 ]
//     [ kx ky kz 0 ]
//     [ px py pz 1 ]
//  For rigid transformations (position, orientation) basis vectors i, j, and k
//  are unit length and mutually perpendicular.
//
//  Created by Craig Reynolds on January 11, 2024.
//  (Based on earlier C++ and Python versions.)
//  MIT License -- Copyright © 2024 Craig Reynolds
//-------------------------------------------------------------------------------

#pragma once
#include "Vec3.h"
#include "Utilities.h"

class LocalSpace
{
public:
    // Constructors
    LocalSpace() {}
    LocalSpace(Vec3 i, Vec3 j, Vec3 k, Vec3 p) : i_(i), j_(j), k_(k), p_(p) {}
    
    // Accessors
    Vec3 i() const { return i_; }
    Vec3 j() const { return j_; }
    Vec3 k() const { return k_; }
    Vec3 p() const { return p_; }
    
    // Setters
    void setI(Vec3 i) { i_ = i; }
    void setJ(Vec3 j) { j_ = j; }
    void setK(Vec3 k) { k_ = k; }
    void setP(Vec3 p) { p_ = p; }
    void setIJKP(Vec3 i, Vec3 j, Vec3 k, Vec3 p) { i_=i; j_=j; k_=k; p_=p; }
    
    // Transforms a global space position into the local space of this object.
    Vec3 localize(Vec3 global_vector) const
    {
        Vec3 v = global_vector - p();
        return Vec3(v.dot(i()), v.dot(j()), v.dot(k()));
    }
    
    // Transforms a local space position to the global space.
    Vec3 globalize(Vec3 local_vector) const
    {
        return ((i() * local_vector.x()) +
                (j() * local_vector.y()) +
                (k() * local_vector.z()) +
                p());
    }

    // Checks that basis vectors are unit length and mutually perpendicular.
    bool is_orthonormal() const { return is_orthonormal(util::epsilon); }
    bool is_orthonormal(double epsilon) const
    {
        return (i().is_unit_length(epsilon) and
                j().is_unit_length(epsilon) and
                k().is_unit_length(epsilon) and
                i().is_perpendicular(j(), epsilon) and
                j().is_perpendicular(k(), epsilon) and
                k().is_perpendicular(i(), epsilon));
    }

    // Return copy with random orientation, position is preserved.
    LocalSpace randomize_orientation() const
    {
        Vec3 ii = EF::RS().random_unit_vector();
        Vec3 jj = EF::RS().random_unit_vector();
        Vec3 kk = ii.cross(jj).normalize();
        jj = kk.cross(ii).normalize();
        return LocalSpace(ii, jj, kk, p());
    }

    // Given a "new_forward" direction, rotate this LocalSpace (about its
    // position) to align with the new forward, while keeping the new "up"
    // direction as close as possible to the given "reference_up". The intent
    // is to find the smallest rotation needed to meet these constraints. This
    // is a type of guided "reorthonormalization."
    LocalSpace rotate_to_new_forward(Vec3 new_forward, Vec3 reference_up) const
    {
        assert(new_forward.is_unit_length());
        assert(reference_up.is_unit_length());
        Vec3 new_side = reference_up.cross(new_forward).normalize();
        Vec3 new_up = new_forward.cross(new_side).normalize();
        // Recalculate side for more precision with "more perpendicular" new_up.
        new_side = new_up.cross(new_forward).normalize();
        return LocalSpace(new_side, new_up, new_forward, p());
    }

    // Construct a LocalSpace whose origin is at "from_position" and whose "+z"
    // axis points toward (intersects) "to_position". Optional "reference_up"
    // is a hint to control "roll" around the z axis. It defaults to "global up"
    // along the "y" axis.
    static LocalSpace fromTo(Vec3 from_position, Vec3 to_position)
    {
        return fromTo(from_position, to_position, Vec3(0, 1, 0));
    }
    static LocalSpace fromTo(Vec3 from_position,
                             Vec3 to_position,
                             Vec3 reference_up)
    {
        LocalSpace ls;
        if (not Vec3::is_equal_within_epsilon(from_position, to_position))
        {
            ls.setP(from_position);
            Vec3 new_forward = (to_position - from_position).normalize();
            Vec3 new_ref_up = new_forward.ensure_not_parallel(reference_up);
            ls = ls.rotate_to_new_forward(new_forward, new_ref_up);
        }
        return ls;
    }
    
    // Returns LS as 16 scalars for a 4x4 transformation matrix.
    // Currently unused, candidate for deletion.
    std::vector<double> asArray() const
    {
        std::vector<double> array;
        auto a = [&](double s){ array.push_back(s); };
        a(i().x()); a(i().y()); a(i().z()); a(p().x());
        a(j().x()); a(j().y()); a(j().z()); a(p().y());
        a(k().x()); a(k().y()); a(k().z()); a(p().z());
        a(0);       a(0);       a(0);       a(1);
        return array;
    }
    
    // Return one scalar component of a LocalSpace. Considers the 16 parameters
    // as a 4x4 homogeneous transformation matrix. Then linearizes them row by
    // row into a 16 element array, then indexes into that. This prototype API
    // may be changed later. Currently used only on one place for constructing
    // an Eigen::Matrix4d in the Draw class.
    double operator[](int index) const
    {
        assert(index > -1 and index < 16);
        switch (index)
        {
            case  0: return i().x();
            case  1: return i().y();
            case  2: return i().z();
            case  3: return p().x();
            case  4: return j().x();
            case  5: return j().y();
            case  6: return j().z();
            case  7: return p().y();
            case  8: return k().x();
            case  9: return k().y();
            case 10: return k().z();
            case 11: return p().z();
            case 15: return 1;
            default: return 0;
        }
    }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    static void unit_test()
    {
        Vec3 ls_i = Vec3(1, 2, 3).normalize();
        Vec3 ls_j = ls_i.cross(Vec3(0, 0, 1)).normalize();
        Vec3 ls_k = ls_i.cross(ls_j).normalize();
        Vec3 ls_p = Vec3(5, 6, 7);
        LocalSpace ls(ls_i, ls_j, ls_k, ls_p);
        LocalSpace r = LocalSpace(ls).randomize_orientation();
        Vec3 a = EF::RS().random_point_in_unit_radius_sphere() * 10;
        Vec3 b = EF::RS().random_point_in_unit_radius_sphere() * 100;

        assert (LocalSpace().is_orthonormal() && "initial value is orthonormal");
        assert (ls.is_orthonormal() && "handmade ls is orthonormal");
        assert (r.is_orthonormal() && "randomized ls is still orthonormal");

        double e = util::epsilon * 20; // Changed from 10 to 20 on 20240604.
        assert (a.is_equal_within_epsilon(r.globalize(r.localize(a)), e));
        assert (a.is_equal_within_epsilon(r.localize(r.globalize(a)), e));
        assert (b.is_equal_within_epsilon(r.globalize(r.localize(b)), e));
        assert (b.is_equal_within_epsilon(r.localize(r.globalize(b)), e));
        
        const LocalSpace o;  // original for comparison
        Vec3 diag_ypz = (o.j() + o.k()).normalize();
        Vec3 diag_ymz = (o.j() - o.k()).normalize();
        LocalSpace m = o.rotate_to_new_forward(diag_ypz, Vec3(0, 1, 0));
        assert(m.is_orthonormal());
        assert(m.i().is_equal_within_epsilon(o.i()));
        assert(m.j().is_equal_within_epsilon(diag_ymz));
        LocalSpace n = o.rotate_to_new_forward(o.i(), Vec3(0, 1, 0));
        assert(n.is_orthonormal());
        assert(n.i().is_equal_within_epsilon(-o.k()));
        assert(n.j().is_equal_within_epsilon(o.j()));
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20241106 tests for fromTo()
        
        LocalSpace ft_ls;
        Vec3 ft_tp(1, -2, 3);
        Vec3 ft_move(0, 0, 1);
        auto ft_tests = [&](Vec3 o)
        {
            assert(o.is_equal_within_epsilon(ft_ls.p()));
            assert(ft_tp.is_equal_within_epsilon(ft_ls.localize(ft_tp)  + o));
            assert(ft_tp.is_equal_within_epsilon(ft_ls.localize(ft_tp   + o)));
            assert(ft_tp.is_equal_within_epsilon(ft_ls.globalize(ft_tp) - o));
            assert(ft_tp.is_equal_within_epsilon(ft_ls.globalize(ft_tp  - o)));
        };
        // Test identity LocalSpace.
        ft_tests(Vec3());
        // Move LS and test again.
        ft_ls.setP(ft_move);
        ft_tests(ft_move);
        // Now move LS to the same place using fromTo() and test again.
        ft_ls = LocalSpace::fromTo(ft_move, ft_move * 2);
        ft_tests(ft_move);
        // Finally, use an arbitrary from-to, and check for cyclic consistancy.
        ft_ls = LocalSpace::fromTo(Vec3(-5, 3, -1), Vec3(3, -4, 6));
        Vec3 ft_cycle = ft_ls.globalize(ft_ls.localize(ft_tp));
        assert(ft_tp.is_equal_within_epsilon(ft_cycle));
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    }
    
private:
    // Basis vectors of local coordinate axes, ijk → xyz:
    Vec3 i_ = Vec3(1, 0, 0);
    Vec3 j_ = Vec3(0, 1, 0);
    Vec3 k_ = Vec3(0, 0, 1);
    // Position of local center:
    Vec3 p_ = Vec3(0, 0, 0);
};

// Serialize LocalSpace object to stream.
inline std::ostream& operator<<(std::ostream& os, const LocalSpace& ls)
{
    os << "[i=" << ls.i();
    os << ", j=" << ls.j();
    os << ", k=" << ls.k();
    os << ", p=" << ls.p();
    os << "]";
    return os;
}
