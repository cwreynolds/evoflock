//------------------------------------------------------------------------------
//
//  main.cpp -- new flock experiments
//
//  Created by Craig Reynolds on January 6, 2024.
//  MIT License -- Copyright © 2024 Craig Reynolds
//
//------------------------------------------------------------------------------

#define USE_OPEN3D

#include "evoflock.h"

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240628 can we do an eval of a const tree?
//#define eval_const_20240628
#ifdef eval_const_20240628
#else  // eval_const_20240628
#endif // eval_const_20240628
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


int main(int argc, const char * argv[])
{

#ifdef USE_OPEN3D
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240913 WIP draw during fitness tests.
//    Draw().visualizeEvoflockFitnessTest();
//    exit(EXIT_SUCCESS);
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#endif  // USE_OPEN3D

    EF::setRS(LP::LPRS());
    EF::unit_test();

    
    //--------------------------------------------------------------------------
    
//        // TODO 20240715 WIP new approach to tree generation.
//
//        {
//            LP::FunctionSet fs =  GP::evoflock_gp_function_set();
//            fs.print();
//            
//    //        for (int i = 0; i < 10000; i++)
//            for (int i = 0; i < 1000000; i++)
//            {
//                std::cout << i << ":" << std::endl;
//    //            LP::GpTree gp_tree = fs.newMakeRandomTree(5, 50);
//    //            LP::GpTree gp_tree = fs.newMakeRandomTree(90, 100);
//    //            LP::GpTree gp_tree = fs.newMakeRandomTree(80, 100);
//                LP::GpTree gp_tree = fs.newMakeRandomTree(20, 60);
//                debugPrint(gp_tree.size());
//                std::cout << gp_tree.to_string(true) << std::endl;
//                std::cout << std::endl << std::endl;
//            }
//        }
//        return EXIT_SUCCESS;

    //--------------------------------------------------------------------------
    
//    GP::evoflock_gp_function_set().print_typical_trees(10, 30, 50);
//    return EXIT_SUCCESS;
    
    //--------------------------------------------------------------------------

    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20240723 going back to GA to make sure that still works.
//    Boid::GP_not_GA = false;
//    Boid::GP_not_GA = true;
//    Boid::GP_not_GA = false;
//    Boid::GP_not_GA = true;
//    // 20240813
//    Boid::GP_not_GA = false;
//    // 20240814
//    Boid::GP_not_GA = true;
    // 20240913
    Boid::GP_not_GA = false;
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    
    
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 202400807 why are obstacles ignored?
    
//    EF::enable_multithreading = false;

//    // 20240813
//    EF::enable_multithreading = true;

//    // 20240814
//    EF::enable_multithreading = false;

    // 20240822
    EF::enable_multithreading = true;

    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // TODO 20240713 experiment with increasing initial tree size.
    //               LP::Individual::increasing_initial_tree_size = true;
    
//    int individuals = 500;
//    int subpops = 25;
//    int max_evolution_steps = Boid::GP_not_GA ? 15000 : 30000;
    

//    int individuals = 2000;
//    int subpops = 50;
//    int max_evolution_steps = Boid::GP_not_GA ? 30000 : 30000;

//    int individuals = 300;
//    int subpops = 17;
//    int max_evolution_steps = Boid::GP_not_GA ? 50000 : 30000;

//    int individuals = 600;
//    int subpops = 25;
//    int max_evolution_steps = Boid::GP_not_GA ? 50000 : 30000;

//    // 20240718
//    int individuals = 500;
//    int subpops = 22;
//    int max_evolution_steps = Boid::GP_not_GA ? 30000 : 30000;

//    // 20240718
//    int individuals = 1000;
//    int subpops = 32;
//    int max_evolution_steps = Boid::GP_not_GA ? 30000 : 30000;

//    // 20240719
//    int individuals = 250;
//    int subpops = 16;
//    int max_evolution_steps = Boid::GP_not_GA ? 30000 : 30000;

//    // 20240719
//    int individuals = 500;
//    int subpops = 22;
//    int max_evolution_steps = Boid::GP_not_GA ? 30000 : 30000;

//    // 20240721
//    int individuals = 2000;
//    int subpops = 100;
//    int max_evolution_steps = Boid::GP_not_GA ? 120000 : 30000;

//    // 20240721
//    int individuals = 500;
//    int subpops = 22;
//    int max_evolution_steps = Boid::GP_not_GA ? 30000 : 30000;

//    // 20240724
//    int individuals = 1000;
//    int subpops = 32;
//    int max_evolution_steps = Boid::GP_not_GA ? 60000 : 30000;

//    // 20240729 4X
//    int individuals = 2000;
//    int subpops = 45;
//    int max_evolution_steps = Boid::GP_not_GA ? 120000 : 30000;

//    // 20240730
//    int individuals = 500;
//    int subpops = 22;
//    int max_evolution_steps = Boid::GP_not_GA ? 30000 : 30000;

    // 20240810
    int individuals = 500;
    int subpops = 22;
//    int max_evolution_steps = Boid::GP_not_GA ? 30000 : 30000;
    int max_evolution_steps = Boid::GP_not_GA ? 20 : 30000;

    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
        
    //    int min_tree_size = 2;
    //    int max_tree_size = 20;
    
    //    int min_tree_size = Boid::GP_not_GA ? 10  :  2;
    //    int max_tree_size = Boid::GP_not_GA ? 100 : 20;
    
    //    int min_tree_size = Boid::GP_not_GA ? 20 :  2;
    //    int max_tree_size = Boid::GP_not_GA ? 50 : 20;
    
    //    int min_tree_size = Boid::GP_not_GA ? 20  :  2;
    //    int max_tree_size = Boid::GP_not_GA ? 100 : 20;
    
    int min_crossover_tree_size = Boid::GP_not_GA ? 20 :  2;
    int max_crossover_tree_size = Boid::GP_not_GA ? 60 : 20;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240720 did I make this too large?
    
//    int max_initial_tree_size   = Boid::GP_not_GA ? 60 : 20;
    
//    // 20240720
//    int max_initial_tree_size   = Boid::GP_not_GA ? 20 : 20;
    
//    // 20240721
//    int max_initial_tree_size   = Boid::GP_not_GA ? 15 : 20;
  
//    // 20240722
//    int max_initial_tree_size   = Boid::GP_not_GA ? 20 : 20;
    
    // TODO 20240805 testing with Be_The_Boid
    int max_initial_tree_size   = Boid::GP_not_GA ? 2 : 20;


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~


//    lp::LPRS().setSeed(20240408);
//    lp::LPRS().setSeed(20240409);
//    lp::LPRS().setSeed(202404091);
//    lp::LPRS().setSeed(20240410);
//    lp::LPRS().setSeed(2024041015);
//    lp::LPRS().setSeed(2024041114);
//    lp::LPRS().setSeed(2024041416);
//    lp::LPRS().setSeed(2024041716);
//    lp::LPRS().setSeed(2024041916);
//    lp::LPRS().setSeed(20240424);
//    lp::LPRS().setSeed(20240427);
//    lp::LPRS().setSeed(20240504);
//    lp::LPRS().setSeed(20240505);
//    lp::LPRS().setSeed(20240506);
//    lp::LPRS().setSeed(20240508);
//    lp::LPRS().setSeed(20240509);
//    lp::LPRS().setSeed(20240512);
//    LP::LPRS().setSeed(20240606);
//    LP::LPRS().setSeed(20240708);
//    LP::LPRS().setSeed(20240710);
//    LP::LPRS().setSeed(20240713);
//    LP::LPRS().setSeed(20240714);
//    LP::LPRS().setSeed(20240718);
//    LP::LPRS().setSeed(20240721);
    LP::LPRS().setSeed(20240722);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240619 WIP first GP_not_GA run
        

    auto fitness_function = (Boid::GP_not_GA ?
                             GP::evoflock_gp_fitness_function :
                             GP::evoflock_ga_fitness_function);
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    LP::Population* population = nullptr;

    LP::FunctionSet fs = (Boid::GP_not_GA ?
                          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                          GP::evoflock_gp_function_set() :
                          GP::test_gp_boid_function_set() :
                          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                          GP::evoflock_ga_function_set);

    {
        std::cout << "Create population." << std::endl;
        util::Timer t("Create population.");
        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240710 fiddling with hyperparameters

//        population = new LazyPredator::Population (individuals,
//                                                   subpops,
//                                                   max_tree_size,
//                                                   min_tree_size,
//                                                   max_tree_size,
//                                                   fs);

        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
        // TODO 20240713 experiment with increasing initial tree size.
        LP::Individual::increasing_initial_tree_size = true;
        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

        population = new LazyPredator::Population(individuals,
                                                  subpops,
                                                  max_initial_tree_size,
                                                  min_crossover_tree_size,
                                                  max_crossover_tree_size,
                                                  fs);
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        
        
        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
        // TODO 20240702 try using that new switch
                
        if (Boid::GP_not_GA)
        {
            population->explicit_treeValue_in_evolutionStep = false;
        }
        else
        {
            fs.setCrossoverFunction(GP::evoflock_ga_crossover);
        }
        
        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    }
    
    {
        std::cout << "Run evolution." << std::endl;
        util::Timer t("Run evolution.");
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240913 WIP draw during fitness tests.
        
//        Draw::globalDrawObjectTemp = std::make_shared<Draw>();
//        debugPrint(Draw::globalDrawObjectTemp);
//        Draw::globalDrawObjectTemp->beginAnimatedDisplay();
        
#ifdef USE_OPEN3D
        Draw().visualizeEvoflockFitnessTest();
#endif  // USE_OPEN3D

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        for (int i = 0; i < max_evolution_steps; i++)
        {
            GP::save_fitness_time_series(*population);
            population->evolutionStep(fitness_function, GP::scalarize_fitness);
            if ((population->getStepCount() % 100) == 0)
            {
                LP::Individual* individual = population->bestFitness();
                std::cout << individual->tree().to_string(true) << std::endl;
            }
            std::cout << std::endl;
        }
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240913 WIP draw during fitness tests.
//        Draw::globalDrawObjectTemp->endAnimatedDisplay();
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    }
    
    // Save end of run data.
    auto record_top_10 = [&]()
    {
        std::cout << std::endl;
        std::cout << std::endl;
        for (int i = 0; i < 10; i++)
        {
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20240703 fix end of run logging for GP
//            const LP::Individual* individual = population->nthBestFitness(i);
            LP::Individual* individual = population->nthBestFitness(i);
//            std::cout << individual->tree().to_string() << std::endl;
            std::cout << individual->tree().to_string(true) << std::endl;
//            auto fitness = GP::rerun_flock_simulation(individual);
            LazyPredator::MultiObjectiveFitness fitness;
            if (Boid::GP_not_GA)
            {
                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // TODO for simplicity, change get/setSaveBoidCenters() to be static
                Flock::setSaveBoidCenters(true);
                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

                fitness = GP::evoflock_gp_fitness_function(individual);
            }
            else
            {
                fitness = GP::rerun_flock_simulation(individual);
            }
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            debugPrint(fitness);
        }
    };
    record_top_10();
    delete population;
    LP::Individual::leakCheck();
    return EXIT_SUCCESS;
}
