//-------------------------------------------------------------------------------
//
//  Agent.h -- new flock experiments
//
//  A steerable agent base class for a 3d self-propelled object.
//
//  Created by Craig Reynolds on January 12, 2024.
//  (Based on earlier C++ and Python versions.)
//  MIT License -- Copyright © 2024 Craig Reynolds
//-------------------------------------------------------------------------------

#pragma once
#include "Vec3.h"
#include "Utilities.h"
#include "LocalSpace.h"
#include <iomanip> // TODO 20240113 to capture agent trajectories for unit_test.

class Agent
{
public:
    Agent() { name_ = "Agent_" + std::to_string(serial_number_++); }

    // Accessors
    Vec3 side() const { return ls_.i(); }
    Vec3 up() const { return ls_.j(); }
    Vec3 forward() const { return ls_.k(); }
    Vec3 position() const { return ls_.p(); }
    double mass() const { return mass_; }
    double speed() const { return speed_; }
    double max_speed() const { return max_speed_; }
    double max_force() const { return max_force_; }

    // Setters
    void setSide(Vec3 side) { ls_.setI(side); }
    void setUp(Vec3 side) { ls_.setJ(side); }
    void setForward(Vec3 side) { ls_.setK(side); }
    void setPosition(Vec3 side) { ls_.setP(side); }
    void setSpeed(double speed) { speed_ = speed; }
    void setMaxSpeed(double max_speed) { max_speed_ = max_speed; }
    void setMaxForce(double max_force) { max_force_ = max_force; }

    // Get current velocity vector.
    Vec3 velocity() const { return forward() * speed(); }

    // Advance Agent state forward by time_step while applying steering_force.
    // (TODO the use of Vec3.truncate() below (like util.clip() in update_...())
    // is likely a problem for automatic differentiation. Perhaps we need another
    // way to accomplish that, like say velocity dependent wind resistance?)
    void steer(Vec3 steering_force, double time_step)
    {
        // Limit steering force by max force (simulates power or thrust limit).
        Vec3 limit_steering_force = steering_force.truncate(max_force());
        // Adjust force by mass to get acceleration.
        Vec3 acceleration = limit_steering_force / mass();
        // Update dynamic and geometric state...
        update_speed_and_local_space(acceleration, time_step);
    }

    // Applies given acceleration to Agent's dynamic and geometric state.
    void update_speed_and_local_space(Vec3 acceleration, double time_step)
    {
        Vec3 new_velocity = velocity() + (acceleration * time_step);
        double new_speed = new_velocity.length();
        setSpeed(util::clip(new_speed, 0, max_speed()));
        setPathCurvature(new_velocity);
        setAcceleration(new_velocity, time_step);
        // Update geometric state when moving.
        if (speed() > 0)
        {
            Vec3 new_forward = new_velocity / new_speed;
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20240713 maybe the problem is steering too big?
            
            {
                grabPrintLock_evoflock();
                if (not new_forward.is_unit_length())
                {
                    debugPrint(new_speed)
                    debugPrint(new_velocity)
                    std::cout << "new_forward.length() = ";
                    std::cout << std::setprecision(20)  << std::fixed;
                    std::cout << new_forward.length() << std::endl;
                }
            }
            
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // Rotate LocalSpace to align with new_forward.
            Vec3 reference_up = up_reference(acceleration * time_step);
            set_ls(ls().rotate_to_new_forward(new_forward, reference_up));
            // Set new position.
            setPosition(position() + (new_forward * speed() * time_step));
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20240713 I occasionally get a failure of the first assert during
            //               GP runs. Thinking about optionally making it less strict.
//            assert (ls().is_orthonormal());
//            assert (ls().is_orthonormal(LocalSpace::epsilon_for_gp));
            assert (ls().is_orthonormal(LocalSpace::epsilon_for_gp * 1000));
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        }
    }
    
    // TODO 20240611 added for sake of GP, not certain this is correct.
    Vec3 getAcceleration() const { return acceleration_; }
    void setAcceleration(const Vec3& new_velocity, double time_step)
    {
        acceleration_ = (new_velocity - velocity()) / time_step;
    }
    
    // Get/set Agent's path curvature based on old and new velocity direction.
    double getPathCurvature() const { return path_curvature_; }
    void setPathCurvature(const Vec3& new_velocity)
    {
        Vec3 ovn = velocity().normalize_or_0();
        Vec3 nvn = new_velocity.normalize_or_0();
        path_curvature_ = (ovn - nvn).length();
    }

    // Very basic roll control: use global UP as reference up.
    virtual Vec3 up_reference(const Vec3& acceleration) { return Vec3(0, 1, 0); }
    
    // Given an arbitrary steering force, return the component purely lateral
    // (perpendicular) to our forward basis. This is the part that steers/turns
    // our heading but leaves speed unchanged.
    Vec3 pure_lateral_steering(const Vec3& raw_steering)
    {
        return raw_steering.perpendicular_component(forward());
    }
    
    // TODO 20240113 experiment, return read-only const ref to LS.
    const LocalSpace& ls() const { return ls_; }
    void set_ls(const LocalSpace&ls ) { ls_ = ls; }
    
    // Makes three lightweight verifications of basic Agent "flying." One is
    // "from first principles" and tests for agreement between a closed form
    // discrete Newtonian model of motion under constant acceleration. Two other
    // more generalized 3d motions are tested for continuing to produce the same
    // results recorded earlier in the source code.
    static void unit_test()
    {
        // Use default Agent at its identity transform initial state. Accelerate
        // it straight forward along the z axis and verify it behaves as Newton
        // says it should.
        // Classic case: https://en.wikipedia.org/wiki/Newton%27s_laws_of_motion
        // Discrete case: https://math.stackexchange.com/a/2880227/516283
        Agent agent0;
        agent0.setMaxSpeed(1000); // Disable speed ceiling for this sub-test.
        double n = 100;
        double scalar_acceleration = 0.1;
        Vec3 z_acceleration(0, 0, scalar_acceleration);
        Vec3 predict_pos(0, 0, ((n * (n + 1)) / 2) * scalar_acceleration);
        // Accelerate 0.1 m/s² in z direction for n steps of 1 second long.
        for (int i = 0; i < n; i++) { agent0.steer(z_acceleration, 1); }
        // Assert the Agent's position is where we predicted it should be.
        double e = util::epsilon * 1000; // ~1e-12
        assert(Vec3::is_equal_within_epsilon(agent0.position(), predict_pos, e));
        
        // Simple "historical repeatability" test. Verify that Agent's final
        // position matches a precomputed reference frozen in the source.
        Agent agent1;
        Vec3 force(0.1, 0.1, 1);
        double time_step = 1.0 / 60.0;
        LocalSpace ref_ls;
        Vec3 ref_position1(0.00012376844287208429, // recorded 20240117
                           0.00012376844287208429,
                           0.0012376844287208429);
        assert (agent1.side()     == ref_ls.i());  // check initial side basis
        assert (agent1.up()       == ref_ls.j());  // check initial up basis
        assert (agent1.forward()  == ref_ls.k());  // check initial forward basis
        assert (agent1.position() == ref_ls.p());  // check initial position
        assert (agent1.velocity() == Vec3());      // check initial velocity
        for (int i = 0; i < 5; i++)
        {
            agent1.steer(force, time_step);
            // debugPrint(agent1.position())
        }
        assert (agent1.position() == ref_position1); // after 5 steer() calls

        // Slightly more complicated "historical repeatability" test. Steering
        // force is expressed in Agent's local space then transformed into
        // global space.
        Agent agent2;
        agent2.setMaxSpeed(1000); // Disable speed ceiling for this sub-test.
        Vec3 local_force(0.1, 0.5, 1);
        for (int i = 0; i < n; i++)
        {
            Vec3 global_force = agent2.ls().globalize(local_force);
            agent2.steer(global_force, 1);
            //debugPrint(agent2.position().to_string(15))
        }
        Vec3 ref_position2(194.704195501038, // recorded 20240119
                           1079.7517268385,
                           1041.8997370084);
        assert (Vec3::is_equal_within_epsilon(agent2.position(), ref_position2, e));
        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    }


private:
    LocalSpace ls_;             // Local coordinate space (pos, orient).
    double mass_ = 1;           // Mass, normally ignored as 1.
    double speed_ = 0;          // Current forward speed (m/s).
    double max_speed_ = 1.0;    // Speed upper limit (m/s)
    double max_force_ = 0.3;    // Acceleration upper limit (m/s²)
    double path_curvature_ = 0; // Computed in update_speed_and_local_space().
    Vec3 acceleration_;         // To keep track of actual acceleration.

    std::string name_;
    inline static int serial_number_ = 0;
};
