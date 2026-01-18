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
