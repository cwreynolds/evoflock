//    #!/usr/bin/env python3
//    # -*- coding: utf-8 -*-
//    #-------------------------------------------------------------------------------
//    #
//    # Agent.py -- new flock experiments
//    #
//    # A steerable agent base class.
//    #
//    # MIT License -- Copyright © 2023 Craig Reynolds
//    #
//    #-------------------------------------------------------------------------------
//
//    import math
//    import Utilities as util
//    from Vec3 import Vec3
//    from LocalSpace import LocalSpace


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


// TODO 20240113 just temp to capture agent trajectories for unit_test
#include <iomanip>


//    class Agent:
//        """A steerable agent base class."""
//
//        # Initialize new instance.
//        def __init__(self):
//            self.ls = LocalSpace()    # Local coordinate space (pos, orient).
//            self.mass = 1             # Mass, normally ignored as 1.
//            self.speed = 0            # Current forward speed (m/s).
//            # TODO 20230411 no idea if these values are plausible for these units
//            #                I mean they are meter-long birds?!
//            #                Note that average bird flying sdpeed is ~10-16 m/s
//            self.max_speed = 1.0      # Speed upper limit (m/s)
//            self.max_force = 0.3      # Acceleration upper limit (m/s²)
//            #
//            self.name = (self.__class__.__name__.lower() +
//                         '_' + str(Agent.serial_number))
//            Agent.serial_number += 1
//
//        # Instance counter for default names.
//        serial_number = 0

class Agent
{
public:
    Agent()
    {
        name_ = "Agent_" + std::to_string(serial_number_++);
    }
    
private: // TODO move these to bottom after python -> c++ translation.
    
    //            self.ls = LocalSpace()    # Local coordinate space (pos, orient).
    //            self.mass = 1             # Mass, normally ignored as 1.
    //            self.speed = 0            # Current forward speed (m/s).

    LocalSpace ls_;    // Local coordinate space (pos, orient).
    double mass_ = 1;  // Mass, normally ignored as 1.
    double speed_ = 0; // Current forward speed (m/s).
    
    //            # TODO 20230411 no idea if these values are plausible for these units
    //            #                I mean they are meter-long birds?!
    //            #                Note that average bird flying sdpeed is ~10-16 m/s
    //            self.max_speed = 1.0      # Speed upper limit (m/s)
    //            self.max_force = 0.3      # Acceleration upper limit (m/s²)
    //            #
    
    double max_speed_ = 1.0;      // Speed upper limit (m/s)
    double max_force_ = 0.3;      // Acceleration upper limit (m/s²)

    std::string name_;

    inline static int serial_number_ = 0;
    
public:

//    # Define setters/getters for side, up, forward, and position.
//    @property
//    def side(self):
//        return self.ls.i
//    @side.setter
//    def side(self, new_forward):
//        self.ls.i = new_forward
//    @property
//    def up(self):
//        return self.ls.j
//    @up.setter
//    def up(self, new_up):
//        self.ls.j = new_up
//    @property
//    def forward(self):
//        return self.ls.k
//    @forward.setter
//    def forward(self, new_forward):
//        self.ls.k = new_forward
//    @property
//    def position(self):
//        return self.ls.p
//    @position.setter
//    def position(self, new_position):
//        self.ls.p = new_position
//
    
    
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

    //    # Get current velocity vector.
    //    @property
    //    def velocity(self):
    //        return self.forward * self.speed

    // Get current velocity vector.
    Vec3 velocity() const { return forward() * speed(); }

//    # Advance Agent state forward by time_step while applying steering_force.
//    # (TODO the use of Vec3.truncate() below (like util.clip() in update_...())
//    # is likely a problem for automatic differentiation. Perhaps we need another
//    # way to accomplish that, like say velocity dependent wind resistance?)
//    def steer(self, steering_force, time_step):
//        assert isinstance(steering_force, Vec3), "steering_force must be Vec3."
//        # Limit steering force by max force (simulates power or thrust limit).
//        limit_steering_force = steering_force.truncate(self.max_force)
//        # Adjust force by mass to get acceleration.
//        acceleration = limit_steering_force / self.mass
//        # Update dynamic and gerometric state...
//        self.update_speed_and_local_space(acceleration * time_step);

    // Advance Agent state forward by time_step while applying steering_force.
    // (TODO the use of Vec3.truncate() below (like util.clip() in update_...())
    // is likely a problem for automatic differentiation. Perhaps we need another
    // way to accomplish that, like say velocity dependent wind resistance?)
    void steer(Vec3 steering_force, double time_step)
    {
//        assert isinstance(steering_force, Vec3), "steering_force must be Vec3."

        // Limit steering force by max force (simulates power or thrust limit).
//        limit_steering_force = steering_force.truncate(self.max_force)
        Vec3 limit_steering_force = steering_force.truncate(max_force());

        // Adjust force by mass to get acceleration.
//        acceleration = limit_steering_force / self.mass
        Vec3 acceleration = limit_steering_force / mass();

        // Update dynamic and gerometric state...
//        self.update_speed_and_local_space(acceleration * time_step);
        update_speed_and_local_space(acceleration * time_step);

    }

//    # Applies given acceleration to Agent's dynamic and geometric state.
//    def update_speed_and_local_space(self, acceleration):
//        new_velocity = self.velocity + acceleration
//        new_speed = new_velocity.length()
//        
//        # TODO 20230407 what if new_speed is zero?
//        #               maybe this should be inside speed>0 block?
//        self.speed = util.clip(new_speed, 0, self.max_speed)
//        new_forward = new_velocity / new_speed;
//        
//        # Update geometric state when moving.
//        if (self.speed > 0):
//            # Reorthonormalize to correspond to new_forward
//            ref_up = self.up_reference(acceleration)
//            new_side = ref_up.cross(new_forward).normalize()
//            new_up = new_forward.cross(new_side).normalize()
//            new_position = self.position + (new_forward * self.speed)
//            # Set new geometric state.
//            new_ls = LocalSpace(new_side, new_up, new_forward, new_position)
//            if new_ls.is_orthonormal():
//                self.ls = new_ls
//            else:
//                print('Ignore bad ls in Agent.update_speed_and_local_space')

    // Applies given acceleration to Agent's dynamic and geometric state.
    void update_speed_and_local_space(Vec3 acceleration)
    {
//        new_velocity = self.velocity + acceleration
//        new_speed = new_velocity.length()
        Vec3 new_velocity = velocity() + acceleration;
        double new_speed = new_velocity.length();

//        // TODO 20230407 what if new_speed is zero?
//        //               maybe this should be inside speed>0 block?
//        self.speed = util.clip(new_speed, 0, self.max_speed)
//        new_forward = new_velocity / new_speed;

        // TODO 20230407 what if new_speed is zero?
        //               maybe this should be inside speed>0 block?
        setSpeed(util::clip(new_speed, 0, max_speed()));
        Vec3 new_forward = new_velocity / new_speed;

//        // Update geometric state when moving.
//        if (self.speed > 0):
//            // Reorthonormalize to correspond to new_forward
//            ref_up = self.up_reference(acceleration)
//            new_side = ref_up.cross(new_forward).normalize()
//            new_up = new_forward.cross(new_side).normalize()
//            new_position = self.position + (new_forward * self.speed)
//            // Set new geometric state.
//            new_ls = LocalSpace(new_side, new_up, new_forward, new_position)
//            if new_ls.is_orthonormal():
//            self.ls = new_ls
//            else:
//            print('Ignore bad ls in Agent.update_speed_and_local_space')


        // Update geometric state when moving.
//        if (self.speed > 0)
        if (speed() > 0)
        {
            // Reorthonormalize to correspond to new_forward
//            ref_up = self.up_reference(acceleration)
//            new_side = ref_up.cross(new_forward).normalize()
//            new_up = new_forward.cross(new_side).normalize()
//            new_position = self.position + (new_forward * self.speed)

            Vec3 ref_up = up_reference(acceleration);
            Vec3 new_side = ref_up.cross(new_forward).normalize();
            Vec3 new_up = new_forward.cross(new_side).normalize();
            // TODO 20240113 why is this not scaled by time_step? !!!!!!!!!!!!!!!!
            // TODO 20240113 isn't this what Matthew asked about long ago!!!!!!!!!
            Vec3 new_position = position() + (new_forward * speed());

//            // Set new geometric state.
//            // TODO 20240113 why replace rather than modify Agent's LocalSpace?
//            new_ls = LocalSpace(new_side, new_up, new_forward, new_position)
//            LocalSpace new_ls = LocalSpace(new_side, new_up, new_forward, new_position);

//            if new_ls.is_orthonormal():
//            self.ls = new_ls
//            else:
//            print('Ignore bad ls in Agent.update_speed_and_local_space')

//                if (new_ls.is_orthonormal())
//                {
//    //                self.ls = new_ls
//                    ls_ = new_ls;
//                }
//                else
//                {
//                    print('Ignore bad ls in Agent.update_speed_and_local_space')
//                }

            ls_.setIJKP(new_side, new_up, new_forward, new_position);

//                // Set new geometric state.
//                // TODO 20240113 why replace rather than modify Agent's LocalSpace?
//                assert (new_ls.is_orthonormal());
//    //            ls_ = new_ls;
      
            // Set new geometric state.
            // TODO 20240113 why replace rather than modify Agent's LocalSpace?
//            assert (ls_.is_orthonormal());
            assert (ls().is_orthonormal());



        }
    }

    // Very basic roll control: use global UP as reference up
    Vec3 up_reference(Vec3 acceleration) { return Vec3(0, 1, 0); }

//    # Given an arbitrary steering force, return the component purely lateral
//    # (perpendicular) to our forward basis. This is the part that steers/turns
//    # our heading but leaves speed unchanged.
//    def pure_lateral_steering(self, raw_steering):
//        return raw_steering.perpendicular_component(self.forward)
//
//    def __str__(self):
//        return self.name + ': speed=' + str(self.speed) + str(self.ls)
//
//    def __eq__(self, other):
//        if isinstance(other, Agent):
//            return self.name == other.name
//        else:
//            return NotImplemented
//
    
    
    // TODO 20240113 experiment, return read-only const ref to LS.
    const LocalSpace& ls() const { return ls_; }

    static void unit_test()
    {
        std::cout << "Agent::unit_test()" << std::endl;
        
        //    @staticmethod
        //    def unit_test():
        //        a = Agent()
        //        force = Vec3(0.1, 0.1, 1)
        //        time_step = 1 / 60
        //        ref_ls = LocalSpace()
        //        ref_position = Vec3(0.007426106572325057,
        //                            0.007426106572325057,
        //                            0.07426106572325057)
        
        Agent a;
        Vec3 force(0.1, 0.1, 1);
        double time_step = 1.0 / 60.0;
        LocalSpace ref_ls;
        Vec3 ref_position(0.007426106572325057,
                          0.007426106572325057,
                          0.07426106572325057);

        //        assert a.side     == ref_ls.i, 'check initial side basis'
        //        assert a.up       == ref_ls.j, 'check initial up basis'
        //        assert a.forward  == ref_ls.k, 'check initial forward basis'
        //        assert a.position == ref_ls.p, 'check initial position'
        //        assert a.velocity == Vec3(),   'check initial velocity'
        
        
        assert (a.side()     == ref_ls.i());  // check initial side basis
        assert (a.up()       == ref_ls.j());  // check initial up basis
        assert (a.forward()  == ref_ls.k());  // check initial forward basis
        assert (a.position() == ref_ls.p());  // check initial position
        assert (a.velocity() == Vec3());      // check initial velocity

        
        //        for i in range(5):
        //            a.steer(force, time_step)
        //            #print(a.position())
        //        assert a.position == ref_position, 'position after 5 steer() calls'

        for (int i = 0; i < 5; i++)
        {
            a.steer(force, time_step);
            // debugPrint(a.position())
        }
        assert (a.position() == ref_position); // position after 5 steer() calls
        
        
        
//            // auditioning a better test:
//
//            Agent a2;
//            Vec3 force2(0.1, 0.5, 1);
//
//            for (int i = 0; i < 5; i++)
//            {
//    //            Vec3 local_force = a2.ls().localize(force2);
//                Vec3 local_force = a2.ls().globalize(force2);
//    //            Vec3 local_force = force2;
//                a2.steer(local_force, time_step);
//                // print(a.position())
//    //            debugPrint(local_force)
//    //            debugPrint(a2.speed())
//    //            debugPrint(a2.position())
//
//    //            debugPrint(std::to_string(a2.position().x()))
//    //            debugPrint(std::to_string(a2.position().y()))
//    //            debugPrint(std::to_string(a2.position().z()))
//
//
//                std::cout << std::setprecision (15) << a2.position().x() << ", ";
//                std::cout << std::setprecision (15) << a2.position().y() << ", ";
//                std::cout << std::setprecision (15) << a2.position().z() <<std::endl;
//
//            }
//
//    //        Vec3 ref_position2(0.00947822, 0.0551442, 0.0431088);
//            Vec3 ref_position2(0.00947821645628656,
//                               0.0551441689904198,
//                               0.043108781969512);
//
//    //        assert (Vec3::is_equal_within_epsilon(a2.position(),
//    //                                              ref_position2,
//    //                                              0.0000001));
//
//            assert (Vec3::is_equal_within_epsilon(a2.position(), ref_position2));

        // auditioning a better test:
        
        Agent a2;
        Vec3 local_force(0.1, 0.5, 1);
        
        for (int i = 0; i < 5; i++)
        {
            Vec3 global_force = a2.ls().globalize(local_force);
            a2.steer(global_force, time_step);
            //std::cout << std::setprecision (15) << a2.position().x() << ", ";
            //std::cout << std::setprecision (15) << a2.position().y() << ", ";
            //std::cout << std::setprecision (15) << a2.position().z() <<std::endl;
        }
        Vec3 ref_position2(0.00947821645628656,
                           0.0551441689904198,
                           0.043108781969512);
        assert (Vec3::is_equal_within_epsilon(a2.position(), ref_position2));

    }
    
private:
    
};
