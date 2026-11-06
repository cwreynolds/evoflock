//------------------------------------------------------------------------------
//
//  FlockParameters.h -- new flock experiments
//
//  A container for all boid flocking parameters relevant to optimization.
//
//  A Flock and its Boids each point to a shared instance of FlockParameters.
//  Evolution adjusts this "genome" of parameters trying to find better fitness.
//
//  Distances and body_diameter are meant to be in meters (Although a spherical
//  boid 1 meter in diameter is quite large: in the eagle/vulture/turkey range).
//  Because all these parameters interact, it will probably not be meaningful to
//  share or compare parameters between flocks. Boids that are bigger/smaller,
//  faster/slower, or whatever, will need to be optimized separately for each
//  set of constraints.
//
//  Created by Craig Reynolds on 3/23/25.
//  MIT License -- Copyright © 2025 Craig Reynolds
//
//------------------------------------------------------------------------------

#pragma once
#include "Vec3.h"

// A container for all boid flocking parameters for simulation and optimization.
class FlockParameters
{
public:
    // Default constructor
    FlockParameters() : FlockParameters(hand_tuned_parameters) {}
    
    // Constructor based on a vector of numeric parameters.
    FlockParameters(const std::vector<double>& vector_of_parameters_)
    {
        assert(vector_of_parameters_.size() == tunableParameterCount());
        initParameterNamesAndAccessors();
        tuning_parameters = vector_of_parameters_;
        enforceConstraints();
    }

    // Shared const/input parameters. These are not subject to optimization and
    // remain fixed during evolution run. MUST update constParameterCount() when
    // const parameters are added or removed.
    static int constParameterCount() { return 7; }
    // The count(/size) of tunable parameters in this class. For consistency
    // check, especially when changing the number of parameters.
    static int tunableParameterCount(){return int(hand_tuned_parameters.size());}
    // "assume a spherical boid" unit diameter
    double bodyDiameter() const { return body_diameter_; }
    // Should this be called "world radius"?
    const double& sphereRadius() const { return sphere_radius_; }
    // Should this be called "world center"?
    Vec3 sphereCenter() const { return sphere_center_; }
    // A flock simulation will run for this many steps (if no exception occurs).
    int maxSimulationSteps() const { return max_simulation_steps_; }
    // Number of Boids in a flock.
    int boidsPerFlock() const { return boids_per_flock_; }
    // Name of Obstacle set to use.
    std::string useObstacleSet() const { return use_obstacle_set; }
    // Simulation step frequency -- frames per second.
    int getFPS() const { return fps_; }
    
    // Hand-tuned parameters used as default.
    const static inline std::vector<double> hand_tuned_parameters =
    {
        100,  // max_force = ;          // Max acceleration (m/s²)

        // Weights for component steering behaviors.
        4,    // weight_forward
        40,   // weight_separate
        45,   // weight_align
        35,   // weight_cohere
        85,   // weightAvoidPredict
        55,   // weightAvoidStatic

        // Max distance at which a neighbor Boid influences a steering behavior.
        // (TODO would prefer parameterization with infinite support (1/d^n ?))
        15,   // max_dist_separate
        40,   // max_dist_align
        40,   // max_dist_cohere

        // Cosine of threshold angle (max angle from forward to be seen)
        -1,    // angle_separate = ~°
        +0.6,  // angle_align    = ~°
        -0.4,  // angle_cohere   = ~°

        // Max distance at which a boid flies away from an obstacle's surface.
        10,    // fly_away_max_dist = ;
                
        // Ignore obstacle until predicted impact is less than this many seconds.
        // Predict obstacle collisions only...
        // ...should this be renamed?
        1.4,   // min_time_to_collide
        
        // Parameters for murmuration behaviors.
        EF::center_max_dist,
        EF::center_min_dist,
        EF::centering_strength,
        
        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
        // TODO 20260602 add more inline parameters to FB for GA optimization

        0.707,  // in_ness_threshold: are we heading sufficiently in?
        0.5,    // centerSlowing() fraction of maxForce if heading out
        0.5,    // centerCentering() fraction of maxForce if heading out

        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
        
        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
        // TODO 20261105 make centroid_max_distance_ an evolvable param
        
        40,    // murmMaxDistance()
        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

    };
    
    // Accessors for tuning parameters:
    const double& maxForce()        const { return tuning_parameters.at(0); }
    
    const double& weightForward()   const { return tuning_parameters.at(1); }
    const double& weightSeparate()  const { return tuning_parameters.at(2); }
    const double& weightAlign()     const { return tuning_parameters.at(3); }
    const double& weightCohere()    const { return tuning_parameters.at(4); }
    const double& weightAvoidPredict()const { return tuning_parameters.at(5); }
    const double& weightAvoidStatic() const { return tuning_parameters.at(6); }
    
    const double& maxDistSeparate() const { return tuning_parameters.at(7); }
    const double& maxDistAlign()    const { return tuning_parameters.at(8); }
    const double& maxDistCohere()   const { return tuning_parameters.at(9); }
    
    // Cosine of threshold angle (max angle from forward to be seen)
    const double& angleSeparate()   const { return tuning_parameters.at(10); }
    const double& angleAlign()      const { return tuning_parameters.at(11); }
    const double& angleCohere()     const { return tuning_parameters.at(12); }
    
    const double& flyAwayMaxDist()  const { return tuning_parameters.at(13); }
    
    // ignore obstacle until predicted impact is in less than this many seconds.
    const double& minTimeToCollide() const { return tuning_parameters.at(14); }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20260501 add accessors for murmuration  parameters
  
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20260511 replace FP::centerMinDist() with FP::centerMinFrac()
    
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    // TODO 20260512 try exponent instead of minFrac

    // TODO names subject to change
    double& centerMaxDist()                 { return tuning_parameters.at(15); }
//    double& centerMinDist()                 { return tuning_parameters.at(16); }
//    double& centerMinFrac()                 { return tuning_parameters.at(16); }
    double& centerExponent()                { return tuning_parameters.at(16); }
    const double& centerMaxDist()     const { return tuning_parameters.at(15); }
//    const double& centerMinDist()     const { return tuning_parameters.at(16); }
//    const double& centerMinFrac()     const { return tuning_parameters.at(16); }
    const double& centerExponent()    const { return tuning_parameters.at(16); }
    const double& centeringStrength() const { return tuning_parameters.at(17); }
    
    
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20260602 add more inline parameters to FB for GA optimization
    
    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    // TODO 20260814 hack upon hack: TEMPORARILY repurposing this currently
    //               unused FP to be a (relative) weight on steerTowardManifold()
    
    const double& centerInNess() const { return tuning_parameters.at(18); }
    const double& weightRelManifold() const { return tuning_parameters.at(18); }

    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    const double& centerSlowing() const { return tuning_parameters.at(19); }
    const double& centerCentering() const { return tuning_parameters.at(20); }

    
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    // TODO 20261106 murmurations -- modern evolvable parameters
        
    // Maximum distance from (per boid) centroid for murmurations.
    const double& murmMaxDistance() const { return tuning_parameters.at(21); }

    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    

    
    // Enforce constraints in constructor. Currently unused. Should be virtual.
    void enforceConstraints()
    {
    }

    // The count(/size) of ALL parameters in this class.
    static size_t parameterCount()
    {
        return constParameterCount() + tunableParameterCount();
    }


    void print() const
    {
        assert(tuning_parameters.size() == tunableParameterCount());
        auto indent = [](){ std::cout << "    "; };
        std::cout << "FlockParameters object containing these values:";
        std::cout << std::endl << "  " << constParameterCount();
        std::cout << " constant parameters:" << std::endl;
        indent(); debugPrint(bodyDiameter());
        indent(); debugPrint(sphereRadius());
        indent(); debugPrint(sphereCenter());
        indent(); debugPrint(maxSimulationSteps());
        indent(); debugPrint(boidsPerFlock());
        indent(); debugPrint(useObstacleSet());
        indent(); debugPrint(getFPS());
        std::cout << "  " << tunableParameterCount();
        std::cout << " parameters to be optimized:" << std::endl;
        indent(); debugPrint(maxForce());
        indent(); debugPrint(weightForward());
        indent(); debugPrint(weightSeparate());
        indent(); debugPrint(weightAlign());
        indent(); debugPrint(weightCohere());
        indent(); debugPrint(weightAvoidPredict());
        indent(); debugPrint(weightAvoidStatic());
        indent(); debugPrint(maxDistSeparate());
        indent(); debugPrint(maxDistAlign());
        indent(); debugPrint(maxDistCohere());
        indent(); debugPrint(angleSeparate());
        indent(); debugPrint(angleAlign());
        indent(); debugPrint(angleCohere());
        indent(); debugPrint(flyAwayMaxDist());
        indent(); debugPrint(minTimeToCollide());
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20260502 replace EF::center_min_dist with FP call.
        indent(); debugPrint(centerMaxDist());
//        indent(); debugPrint(centerMinDist());
        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
        // TODO 20260512 try exponent instead of minFrac
//        indent(); debugPrint(centerMinFrac());
        indent(); debugPrint(centerExponent());
        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
        indent(); debugPrint(centeringStrength());
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
        // TODO 20260602 add more inline parameters to FB for GA optimization

        indent(); debugPrint(centerInNess());
        indent(); debugPrint(centerSlowing());
        indent(); debugPrint(centerCentering());

        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
        // TODO 20261105 make centroid_max_distance_ an evolvable param
        
        indent(); debugPrint(murmMaxDistance());

        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    }

    // Get i-th tuning parameter
    // TODO experimental reconsider design.
    double getTuningParameter(int i) const { return tuning_parameters.at(i); }

    // Types for pairing evolvable parameter names and their accessors.
    typedef std::function<double()> FpAccessor;
    typedef std::pair<std::string, FpAccessor> FpNameAccessorPair;
    
    // Make and cache the collection of parameters and accessors per instance.
    // TODO 20261106 since FP are constructed only 3 (why 3?) times per sim,
    //               each of which take 0.25 to 0.75 secconds, is it worth
    //               caching? Maybe just build each time it is needed.
    //
    //               I would like to merge FP::tuning_parameters in here, but
    //               that wants to be static and this wants to be per-instance
    //
    std::vector<FpNameAccessorPair> all_params_names_accessors_;
    //
    void initParameterNamesAndAccessors()
    {
        all_params_names_accessors_ =
        {
            {"maxForce",           [&](){ return maxForce(); }},
            {"weightForward",      [&](){ return weightForward(); }},
            {"weightSeparate",     [&](){ return weightSeparate(); }},
            {"weightAlign",        [&](){ return weightAlign(); }},
            {"weightCohere",       [&](){ return weightCohere(); }},
            {"weightAvoidPredict", [&](){ return weightAvoidPredict(); }},
            {"weightAvoidStatic",  [&](){ return weightAvoidStatic(); }},
            {"maxDistSeparate",    [&](){ return maxDistSeparate(); }},
            {"maxDistAlign",       [&](){ return maxDistAlign(); }},
            {"maxDistCohere",      [&](){ return maxDistCohere(); }},
            {"angleSeparate",      [&](){ return angleSeparate(); }},
            {"angleAlign",         [&](){ return angleAlign(); }},
            {"angleCohere",        [&](){ return angleCohere(); }},
            {"flyAwayMaxDist",     [&](){ return flyAwayMaxDist(); }},
            {"minTimeToCollide",   [&](){ return minTimeToCollide(); }},
            {"centerMaxDist",      [&](){ return centerMaxDist(); }},
            {"centerExponent",     [&](){ return centerExponent(); }},
            {"centeringStrength",  [&](){ return centeringStrength(); }},
            {"centerInNess",       [&](){ return centerInNess(); }},
            {"centerSlowing",      [&](){ return centerSlowing(); }},
            {"centerCentering",    [&](){ return centerCentering(); }},
            //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
            // TODO 20261105 make centroid_max_distance_ an evolvable param
            {"murmMaxDistance",    [&](){ return murmMaxDistance(); }},
            //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
        };
        // Double check for legacy standard.
        assert(all_params_names_accessors_.size() == tuning_parameters.size());
    }

    // Return [name.accessor] pairs for all evolvable parameters.
    const std::vector<FpNameAccessorPair>& allParameterNamesAndAccessors() const
    {
        return all_params_names_accessors_;
    }

    // Return names of all evolvable parameters.
    VecOfStrings allParameterNames() const
    {
        VecOfStrings names;
        const auto& all_naa = allParameterNamesAndAccessors();
        for (auto& [name, accessor] : all_naa) { names.push_back(name); }
        return names;
    }

    static void unit_test()
    {
        std::vector<double> indices;
        for (int i=0; i<tunableParameterCount(); i++){indices.push_back(i);}
        FlockParameters fp(indices);
        int count = 0;
        for (auto& [name, accessor] : fp.allParameterNamesAndAccessors())
        {
            bool match = (count == accessor());
            if (not match)
            {
                std::cout << std::endl;
                std::cout << "Error for accessor FlockParameters::";
                std::cout << name << "()" << std::endl;
                std::cout << "Check layout of accessors in parameter vector.";
                std::cout << std::endl;
            }
            assert(match);
            count++;
        }
    }

private:
    // Parameters for tuning:
    std::vector<double> tuning_parameters = hand_tuned_parameters;

    // const/input parameters:
    double body_diameter_ = 1;
    double sphere_radius_ = 50;
    Vec3 sphere_center_;

    int max_simulation_steps_ = (EF::visualize_previous_results_mode ?
                                 1000 :  // ~30 seconds viz prev results.
                                 500);   // ~17 seconds: for evolution run.

        std::string use_obstacle_set = (EF::murmuration_mode ?
//                                        "BoxObstacle" :
//                                        "BigSphere" :
//                                        "DomeAndGround" :
                                        "NoObstacles" :
                                        "SmallSpheresInBigSphere");

    int boids_per_flock_ = (EF::override_boids_per_flock > 0 ?
                            EF::override_boids_per_flock :
                            (EF::visualize_previous_results_mode ?
                             
                             // For visualizing previous results.
                             // (Bigger for murmurations.)
                             (EF::murmuration_mode ? 2000 : 1400) :

                             // Normal, for evolution runs.
                             // (Bigger for murmurations.)
                             (EF::murmuration_mode ? 1000 : 200)));
    int fps_ = 30;
};
