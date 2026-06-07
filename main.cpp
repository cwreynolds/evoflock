//------------------------------------------------------------------------------
//
//  main.cpp -- new flock experiments
//
//  Created by Craig Reynolds on January 6, 2024.
//  MIT License -- Copyright © 2024 Craig Reynolds
//
//------------------------------------------------------------------------------

#define USE_OPEN3D

#include "EvoFlock.h"

int main(int argc, const char * argv[])
{
    EF::unit_test();
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20260606 test calculations for adjusting murmuration sphere radius
    //               to maintain boid density.
    
    FlockParameters fp;
    std::cout << std::endl;
    debugPrint(fp.sphereRadius())
    debugPrint(fp.boidsPerFlock())
    debugPrint(shape::Sphere::volumeFromRadius(fp.sphereRadius()));
    debugPrint(fp.boidsPerFlock() /
               shape::Sphere::volumeFromRadius(fp.sphereRadius()));
    std::cout << std::endl;
    double r = fp.sphereRadius();
    double v = shape::Sphere::volumeFromRadius(r);
    double bpf = fp.boidsPerFlock();
    double bpm3 = bpf / v;
    debugPrint(r);
    debugPrint(v);
    debugPrint(bpf);
    debugPrint(bpm3);
    std::cout << std::endl;
    
    // Now say instead of 2000 boids in flock we only have 300
    double bpf_ratio = 300.0 / 2000.0;
    double v2 = v * bpf_ratio;
    double r2 = shape::Sphere::radiusFromVolume(v2);
    debugPrint(bpf_ratio);
    debugPrint(v2);
    debugPrint(r2);

    std::cout << std::endl;

//    return EXIT_SUCCESS;
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20260530 what is average radius of uniformly distributed
    //               points in a unit radius sphere? Answer: 0.75
    
//    int samples = 100000000;
//    double sum_of_radii = 0;
//    for (int i = 0; i < samples; i++)
//    {
//        Vec3 random_point = EF::RS().randomPointInUnitRadiusSphere();
//        sum_of_radii += random_point.length();
//    }
//    double average_radius = sum_of_radii / samples;
//    debugPrint(average_radius)
//    return EXIT_SUCCESS;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    // TODO 20260116 what distribution of tree sizes for give max size?
    
//        const LP::FunctionSet& fs = GP::evoflockGpFunctionSet();
//        fs.reset_smallest_init_tree_xxx();
//        
//        int tree_count = 1000;
//        double sum_of_sizes = 0;
//        
//        std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
//
//        for (int i = 0; i < tree_count; i++)
//        {
//            LP::GpTree tree;
//            
//    //        fs.makeRandomTree(max_size, tree);
//    //        tree = fs.newMakeRandomTree(50, 100);
//    //        tree = fs.newMakeRandomTree(10, 25);
//    //        tree = fs.newMakeRandomTree(15, 25);
//    //        fs.makeRandomTree(200, tree);
//    //        fs.makeRandomTree(1000, tree);
//            tree = fs.newMakeRandomTree(20, 100);
//            
//            sum_of_sizes += tree.size();
//    //        debugPrint(tree.size());
//        }
//        
//        std::cout << "Size average over " << tree_count << " trees = ";
//        std::cout << sum_of_sizes / tree_count << std::endl;
//        exit(EXIT_SUCCESS);
    
    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~


    EF::runOneFlockEvolution();
    // EF::runFlockEvolutionLoop();
    
    Draw::deleteInstance();
    return EXIT_SUCCESS;
}
