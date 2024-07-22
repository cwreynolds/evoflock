//
//  Population.h
//  LazyPredator
//
//  Created by Craig Reynolds on 8/6/20.
//  Copyright © 2020 Craig Reynolds. All rights reserved.
//

#pragma once
#include "Individual.h"
#include "FunctionSet.h"
#include "TournamentGroup.h"
#include <iomanip>

namespace LazyPredator
{

class Population
{
public:
    Population() : Population(0, 0, 0, 0, 0, nullptr) {}
    Population(int individual_count)
    : Population(individual_count, 0, 0, 0, 0, nullptr) {}
    Population(int individual_count,
               int max_init_tree_size,
               const FunctionSet& fs)
    : Population(individual_count, 0, max_init_tree_size, fs) {}
    Population(int individual_count,
               int subpopulation_count,
               int max_init_tree_size,
               const FunctionSet& fs)
    : Population(individual_count,
                 subpopulation_count,
                 max_init_tree_size,
                 0.5 * max_init_tree_size,
                 1.5 * max_init_tree_size,
                 &fs) {}
    Population(int individual_count,
               int subpopulation_count,
               int max_init_tree_size,
               int min_crossover_tree_size,
               int max_crossover_tree_size,
               const FunctionSet& fs)
    : Population(individual_count,
                 subpopulation_count,
                 max_init_tree_size,
                 min_crossover_tree_size,
                 max_crossover_tree_size,
                 &fs) {}
    Population(int individual_count,
               int subpopulation_count,
               int max_init_tree_size,
               int min_crossover_tree_size,
               int max_crossover_tree_size,
               const FunctionSet* fs)
    {
        setFunctionSet(fs);
        setMaxInitTreeSize(max_init_tree_size);
        setMinCrossoverTreeSize(min_crossover_tree_size);
        setMaxCrossoverTreeSize(max_crossover_tree_size);
        if (subpopulation_count == 0) { subpopulation_count = 1; } // Default.
        assert(subpopulation_count > 0);
        subpopulations_.resize(subpopulation_count);
        for (int i = 0; i < individual_count; i++)
        {
            Individual* new_individual = ((max_init_tree_size == 0) ?
                                          new Individual :
                                          new Individual(max_init_tree_size,
                                                         *fs));
            subpopulation(i % subpopulation_count).push_back(new_individual);
        }
        updateSortedCollectionOfIndividuals();
        idle_time_ = TimeDuration::zero();
        // TODO keep, remove, or move to unit tests?
        assert(individual_count == sorted_collection_.size());
        assert(individual_count == getIndividualCount());
    }
    
    virtual ~Population()
    {
        applyToAllIndividuals([](Individual* i){ delete i; });
    }
    
    // A subpopulation (deme): just an std::vector of Individual pointers.
    typedef std::vector<Individual*> SubPop;
    // Functions that implement tournaments, by transforming a TournamentGroup.
    typedef std::function<TournamentGroup(TournamentGroup)> TournamentFunction;
    // Functions that measure "absolute" fitness of an Individual in isolation.
    // (A shortcut for fitnesses that can be measured this way. Many cannot.)
    typedef std::function<float(Individual*)> FitnessFunction;
    
    // Perform one step of the "steady state" evolutionary computation. Three
    // Individuals are selected randomly, from a random subpopulation. Holds a
    // "tournament" to determine their relative fitness ordering. The "loser" is
    // removed from the Population and replaced by a new "offspring" created by
    // crossing over the two "winners" and mutating the result. Handle migration
    // between subpopulations and maintain sorted index of Individuals.
    void evolutionStep(TournamentFunction tournament_function)
    {
        // Get current subpopulation, create a random TournamentGroup from it.
        SubPop& subpop = currentSubpopulation();
        TournamentGroup random_group = randomTournamentGroup(subpop);
        // Run tournament among the three, return ranked group.
        TournamentGroup ranked_group = tournament_function(random_group);
        double prev_best_fitness = bestFitness()->getFitness();
        // Complete the step based on this ranked group, if it is valid.
        if (ranked_group.getValid()) { evolutionStep(ranked_group, subpop); }
        double new_best_fitness = bestFitness()->getFitness();
        if (protect_elite_mof) { assert(new_best_fitness >= prev_best_fitness); }
        // Increment step count (before logger() call for 1 based step numbers).
        incrementStepCount();
        logger();
    }
    
    // Perform one step of the "steady state" evolutionary computation. Three
    // Individuals are selected randomly, from a random subpopulation. Holds a
    // "tournament" to determine their relative fitness ordering. The "loser" is
    // removed from the Population and replaced by a new "offspring" created by
    // crossing over the two "winners" and mutating the result. Handle migration
    // between subpopulations and maintain sorted index of Individuals.
    void evolutionStep(TournamentGroup ranked_group, SubPop& subpop)
    {
        Individual* loser = ranked_group.worstIndividual();
        int loser_index = ranked_group.worstIndex();
        assert(loser);
        // Other two become parents of new offspring.
        Individual* parent0 = ranked_group.secondBestIndividual();
        Individual* parent1 = ranked_group.bestIndividual();
        // Both parent's rank increases because they survived the tournament.
        parent0->incrementTournamentsSurvived();
        parent1->incrementTournamentsSurvived();
        // Create new offspring tree by crossing-over these two parents.
        GpTree new_tree;
        auto crossover = getFunctionSet()->getCrossoverFunction();
        crossover(parent0->tree(),
                  parent1->tree(),
                  new_tree,
                  getMinCrossoverTreeSize(),
                  getMaxCrossoverTreeSize(),
                  getFunctionSet()->getCrossoverMinSize());
        // Mutate constants in new tree.
        new_tree.mutate();
        // Create new offspring Individual from new tree.
        Individual* offspring = new Individual(new_tree);
        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240625 hmm is really necessary, or more a "maybe a good idea"?
        //               invalid "invalid Boid::gp_boid_per_thread_" in evoflock
        //               GP mode causes an
        //
        //               generally It seems like evaluating fitness should not
        //               be done as a side effect of creating a new tree, so I
        //               prefer to make it “lazy” if possible
        
//        // Construct and cache the result of evaluating new offspring's GpTree.
//        offspring->treeValue();
//        // If group has custom_eval function run it on offspring (esp for MOF).
//        if (ranked_group.custom_eval) { ranked_group.custom_eval(offspring); }
        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        // Delete tournament loser from Population, replace with new offspring.
        replaceIndividual(loser_index, offspring, subpop);
        // Occasionally migrate Individuals between subpopulations.
        subpopulationMigration();
    }
    
    // Perform one step of the "steady state" evolutionary computation using
    // "absolute fitness" (rather than "relative tournament-based fitness").
    // Takes a FitnessFunction which maps a given Individual to a numeric
    // "absolute fitness" value. Converts this into a TournamentFunction for
    // use in the "relative fitness" version of evolutionStep() above.
    void evolutionStep(FitnessFunction fitness_function)
    {
        // Wrap given FitnessFunction to ensure Individual has cached fitness.
        auto augmented_fitness_function = [&](Individual* individual)
        {
            // In case Individual does not already have a cached fitness value.
            if (!(individual->hasFitness()))
            {
                // Tree value should be previously cached, but just to be sure.
                individual->treeValue();
                // Cache fitness on Individual using given FitnessFunction.
                individual->setFitness(fitness_function(individual));
            }
            return individual->getFitness();
        };
        // Create a TournamentFunction based on the augmented FitnessFunction.
        auto tournament_function = [&](TournamentGroup group)
        {
            group.setAllMetrics(augmented_fitness_function);
            return group;
        };
        // Finally, do a tournament-based evolution step.
        evolutionStep(tournament_function);
    }

    // Functions that measure "absolute" fitness of an Individual in isolation.
    // (A shortcut for fitnesses that can be measured this way. Many cannot.)
    // But this version (unlike FitnessFunction) is for the "multi objective"
    // case where each fitness is a vector/tuple of fitness values for each of
    // several independent objectives.
    typedef std::function<MultiObjectiveFitness(Individual*)>
            MultiObjectiveFitnessFunction;

    // Function that maps a MultiObjectiveFitness to a scalar. Currently using a
    // 4d vector magnitude, normalized onto [0,1]
    typedef std::function<double(MultiObjectiveFitness)> FitnessScalarizeFunction;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240701 external switch -- temporary?
    bool explicit_treeValue_in_evolutionStep = true;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240708 WIP for MOF elitism
private:
    MultiObjectiveFitness elite_mof_;
public:
    
    void updateEliteMOF(const MultiObjectiveFitness& mof)
    {
        if (elite_mof_.empty()) { elite_mof_ = mof; }  // First update.
        assert(mof.size() == elite_mof_.size());
        for (int i = 0; i < mof.size(); i++)
        {
            elite_mof_.at(i) = std::max(mof.at(i), elite_mof_.at(i));
        }
    }

    bool isEliteMOF(const MultiObjectiveFitness& mof) const
    {
        bool elite = false;
        assert(mof.size() == elite_mof_.size());
        for (int i = 0; i < mof.size(); i++)
        {
            double mo = mof.at(i);
            double eo = elite_mof_.at(i);
            if (util::between(mo, 0.05, 0.95) and (mo >= eo)) { elite = true; }
        }
        return elite;
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240718 experiment using "hypervolume" as metric for TG

//        // Perform one step of the "steady state" evolutionary computation using
//        // "multi objective fitness" -- basically a vector of scalar fitness values
//        // each for an independent, potentially conflicting measure of fitness. It
//        // is given a MultiObjectiveFitnessFunction (which maps an individual to a
//        // MultiObjectiveFitness) and a FitnessScalarizeFunction (which maps a
//        // MultiObjectiveFitness to a summary scalar, used to rank Individuals in
//        // the Population by quality). It creates a TournamentFunction from those,
//        // which is passed to a different version of evolutionStep(). Each evolution
//        // step this chooses one of the N objectives (the one with most potential
//        // for improvement) and treats that as a scalar fitness for this step.
//        void evolutionStep(MultiObjectiveFitnessFunction mo_fitness_function,
//                           FitnessScalarizeFunction fitness_scalarize_function)
//        {
//            // State shared between lambdas below.
//            double prev_best_pop_fitness = bestFitness()->getFitness();
//            bool found_new_best = false;
//            // Customized function to perform MOF evaluation of individual.
//            auto mof_eval = [&](Individual* individual)
//            {
//                if (not individual->hasMultiObjectiveFitness())
//                {
//                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                    // TODO 20240625 will it work without this? its before the
//                    //       mo_fitness_function which is where it is value to eval.
//
//                    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
//                    // TODO 20240630 OH!! taking this out made Boid::GP_not_GA=true
//                    // work, but BROKE "Boid::GP_not_GA=false" (and all previous FS)
//                    // Apparently I want to make this optional, default ON but have
//                    // a way to set it to OFF for the Boid::GP_not_GA=true case
//
//                    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//                    // TODO 20240701 external switch -- temporary?
//
//    //                individual->treeValue();
//
//                    if (explicit_treeValue_in_evolutionStep)
//                    {
//                        individual->treeValue();
//                    }
//                    //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//                    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
//
//                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                    MultiObjectiveFitness mof = mo_fitness_function(individual);
//                    individual->setMultiObjectiveFitness(mof);
//                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                    // TODO 20240708 WIP for MOF elitism
//                    updateEliteMOF(mof);
//                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                    double scalar = fitness_scalarize_function(mof);
//                    individual->setFitness(scalar);
//                    if (scalar > prev_best_pop_fitness) { found_new_best = true;}
//                    sort_cache_invalid_ = true;
//                }
//            };
//            // Create TournamentFunction from given fitness and scalarizer functions.
//            auto tournament_function = [&](TournamentGroup group)
//            {
//                // Make sure each group Individual has cached MultiObjectiveFitness.
//                for (auto& m : group.members()) { mof_eval(m.individual); }
//    //            // Store customized evaluation function on TournamentGroup.
//    //            group.custom_eval = mof_eval;
//                // Select best of the multiple fitnesses to use for this step.
//                size_t best_mof_index = group.pickMultiObjectiveFitnessIndex();
//                // Set the "metric" of each TournamentGroup member to that
//                // "best_mof_index" of the member's MultiObjectiveFitness.
//                for (auto& m : group.members())
//                {
//                    const auto& mof = m.individual->getMultiObjectiveFitness();
//                    m.metric =  mof.at(best_mof_index);
//                }
//                // Sort the TournamentGroup by metric, least first.
//                group.sort();
//
//                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                // TODO 20240710 WIP for MOF elitism
//
//    //            // Preserve population best fitness regardless of MOF metrics.
//    //            if (found_new_best) { group.setValid(false); }
//
//                protectEliteMOF(group);
//                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//                return group;
//            };
//            // Finally, do a tournament-based evolution step.
//            evolutionStep(tournament_function);
//        }

//        // Perform one step of the "steady state" evolutionary computation using
//        // "multi objective fitness" -- basically a vector of scalar fitness values
//        // each for an independent, potentially conflicting measure of fitness. It
//        // is given a MultiObjectiveFitnessFunction (which maps an individual to a
//        // MultiObjectiveFitness) and a FitnessScalarizeFunction (which maps a
//        // MultiObjectiveFitness to a summary scalar, used to rank Individuals in
//        // the Population by quality). It creates a TournamentFunction from those,
//        // which is passed to a different version of evolutionStep(). Each evolution
//        // step this chooses one of the N objectives (the one with most potential
//        // for improvement) and treats that as a scalar fitness for this step.
//        void evolutionStep(MultiObjectiveFitnessFunction mo_fitness_function,
//                           FitnessScalarizeFunction fitness_scalarize_function)
//        {
//            // State shared between lambdas below.
//            double prev_best_pop_fitness = bestFitness()->getFitness();
//            bool found_new_best = false;
//            // Customized function to perform MOF evaluation of individual.
//            auto mof_eval = [&](Individual* individual)
//            {
//                if (not individual->hasMultiObjectiveFitness())
//                {
//                    if (explicit_treeValue_in_evolutionStep)
//                    {
//                        individual->treeValue();
//                    }
//                    MultiObjectiveFitness mof = mo_fitness_function(individual);
//                    individual->setMultiObjectiveFitness(mof);
//                    updateEliteMOF(mof);
//                    double scalar = fitness_scalarize_function(mof);
//                    individual->setFitness(scalar);
//                    if (scalar > prev_best_pop_fitness) { found_new_best = true;}
//                    sort_cache_invalid_ = true;
//                }
//            };
//            // Create TournamentFunction from given fitness and scalarizer functions.
//            auto tournament_function = [&](TournamentGroup group)
//            {
//                // Make sure each group Individual has cached MultiObjectiveFitness.
//                for (auto& m : group.members()) { mof_eval(m.individual); }
//
//                //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
//                // TODO 20240718 experiment using "hypervolume" as metric for TG
//
//                // Select best of the multiple fitnesses to use for this step.
//                size_t best_mof_index = group.pickMultiObjectiveFitnessIndex();
//                // Set the "metric" of each TournamentGroup member to that
//                // "best_mof_index" of the member's MultiObjectiveFitness.
//                for (auto& m : group.members())
//                {
//                    const auto& mof = m.individual->getMultiObjectiveFitness();
//                    m.metric =  mof.at(best_mof_index);
//                }
//
//    //                // Set each TournamentGroup member's "metric" to its MOF hypervolume.
//    //                for (auto& m : group.members())
//    //                {
//    //                    const auto& mof = m.individual->getMultiObjectiveFitness();
//    //                    m.metric =  mof.hyperVolume();
//    //                    std::cout << "    #### hyperVolume = " << mof.hyperVolume();
//    //                    std::cout << ", mof = " << mof << std::endl;
//    //
//    //    //                const auto& mof = m.individual->getMultiObjectiveFitness();
//    //    //                m.metric =  mof.hyperVolumeDropout();
//    //    //                std::cout << "    #### hyperVolume = " << m.metric;
//    //    //                std::cout << ", mof = " << mof << std::endl;
//    //                }
//    //                std::cout << std::endl;
//
//                //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
//
//                // Sort the TournamentGroup by metric, least first.
//                group.sort();
//                protectEliteMOF(group);
//                return group;
//            };
//            // Finally, do a tournament-based evolution step.
//            evolutionStep(tournament_function);
//        }

    // Perform one step of the "steady state" evolutionary computation using
    // "multi objective fitness" -- basically a vector of scalar fitness values
    // each for an independent, potentially conflicting measure of fitness. It
    // is given a MultiObjectiveFitnessFunction (which maps an individual to a
    // MultiObjectiveFitness) and a FitnessScalarizeFunction (which maps a
    // MultiObjectiveFitness to a summary scalar, used to rank Individuals in
    // the Population by quality). It creates a TournamentFunction from those,
    // which is passed to a different version of evolutionStep(). Each evolution
    // step this chooses one of the N objectives (the one with most potential
    // for improvement) and treats that as a scalar fitness for this step.
    void evolutionStep(MultiObjectiveFitnessFunction mo_fitness_function,
                       FitnessScalarizeFunction fitness_scalarize_function)
    {
        // State shared between lambdas below.
        double prev_best_pop_fitness = bestFitness()->getFitness();
        bool found_new_best = false;
        // Customized function to perform MOF evaluation of individual.
        auto mof_eval = [&](Individual* individual)
        {
            if (not individual->hasMultiObjectiveFitness())
            {
                if (explicit_treeValue_in_evolutionStep)
                {
                    individual->treeValue();
                }
                MultiObjectiveFitness mof = mo_fitness_function(individual);
                individual->setMultiObjectiveFitness(mof);
                updateEliteMOF(mof);
                double scalar = fitness_scalarize_function(mof);
                individual->setFitness(scalar);
                if (scalar > prev_best_pop_fitness) { found_new_best = true;}
                sort_cache_invalid_ = true;
            }
        };
        // Create TournamentFunction from given fitness and scalarizer functions.
        auto tournament_function = [&](TournamentGroup group)
        {
            // Make sure each group Individual has cached MultiObjectiveFitness.
            for (auto& m : group.members()) { mof_eval(m.individual); }
            
            //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
            // TODO 20240720 experiment using "dominate per MOF dimension" count

//            // Select best of the multiple fitnesses to use for this step.
//            size_t best_mof_index = group.pickMultiObjectiveFitnessIndex();
//            // Set the "metric" of each TournamentGroup member to that
//            // "best_mof_index" of the member's MultiObjectiveFitness.
//            for (auto& m : group.members())
//            {
//                const auto& mof = m.individual->getMultiObjectiveFitness();
//                m.metric =  mof.at(best_mof_index);
//            }

            // Set the "metric" of each TournamentGroup member to count of MOF
            // dimensions its Individual dominates.
//                for (int i = 0; i < group.members().size(); i++)
//                {
//                    auto& gm = group.members().at(i);
//                    const auto& mof = gm.individual->getMultiObjectiveFitness();
//                    int dominate_count = 0;
//                    for (int j = 0; j < mof.size(); j++)
//                    {
//    //                    bool dominate = false;
//                        double my_j_value = mof.at(j);
//                        double max_j_value = -1; // QQQQ use -inf?
//                        for (int k = 0; k < group.members().size(); k++)
//                        {
//                            const auto& gm2 = group.members().at(k);
//                            const auto& mof2 = gm2.individual->getMultiObjectiveFitness();
//    //                        if (my_j_value > mof2.at(j)) { dominate = true; }
//                            if (max_j_value < mof2.at(j)) { max_j_value = mof2.at(j); }
//                        }
//    //                    if (my_j_value == max_j_value) { dominate = true; }
//    //                    if (dominate) { dominate_count++; }
//                        if (my_j_value == max_j_value) { dominate_count++; }
//                    }
//                    gm.metric = dominate_count;
//                }
            
            //~~    ~~    ~~    ~~    ~~    ~~    ~~    ~~    ~~    ~~    ~~
            
//                for (int i = 0; i < group.members().size(); i++)
//                {
//                    auto& gm = group.members().at(i);
//                    const auto& mof = gm.individual->getMultiObjectiveFitness();
//                    int dominate_count = 0;
//                    for (int j = 0; j < mof.size(); j++)
//                    {
//    //                    double my_j_value = mof.at(j);
//                        double max_j_value = -1; // QQQQ use -inf?
//                        for (int k = 0; k < group.members().size(); k++)
//                        {
//                            const auto& gm2 = group.members().at(k);
//                            const auto& mof2 = gm2.individual->getMultiObjectiveFitness();
//    //                        if (max_j_value < mof2.at(j)) { max_j_value = mof2.at(j); }
//
//                            if (i != k)
//                            {
//                                if (max_j_value < mof2.at(j))
//                                {
//                                    max_j_value = mof2.at(j);
//                                }
//                            }
//                        }
//                        if (mof.at(j) > max_j_value) { dominate_count++; }
//                    }
//                    gm.metric = dominate_count;
//                }
//
//                std::cout << std::endl << "**************************" << std::endl;
//    //            group.print();
//                for (auto& m : group.members())
//                {
//                    std::cout << "metric = " << m.metric << ", mof = ";
//                    std::cout << m.individual->getMultiObjectiveFitness().to_string();
//                    std::cout << std::endl;
//                }
//                std::cout << "**************************" << std::endl << std::endl;
            
            //~~    ~~    ~~    ~~    ~~    ~~    ~~    ~~    ~~    ~~    ~~

            
//            count_objectives_dominated(group);
            
            //~~    ~~    ~~    ~~    ~~    ~~    ~~    ~~    ~~    ~~    ~~
            
            
            // Randomly combine COD with the two versions of best_mof_index.
            if (LPRS().randomBool(0.33))
            {
                countObjectivesDominated(group);
            }
            else
            {
                // Select best of the multiple fitnesses to use for this step.
                size_t best_mof_index = group.pickMultiObjectiveFitnessIndex();
                // Set the "metric" of each TournamentGroup member to that
                // "best_mof_index" of the member's MultiObjectiveFitness.
                for (auto& m : group.members())
                {
                    const auto& mof = m.individual->getMultiObjectiveFitness();
                    m.metric =  mof.at(best_mof_index);
                }
            }


            //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~

            // Sort the TournamentGroup by metric, least first.
            group.sort();
            protectEliteMOF(group);
            return group;
        };
        // Finally, do a tournament-based evolution step.
        evolutionStep(tournament_function);
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240721 break off countObjectivesDominated() into its own function
    //     had been inline in evolutionStep(MultiObjectiveFitnessFunction...)
    
    // Wrote new experimental code to count how many objectives (dimensions of a
    // MOF) are dominated by a given Individual.
    // Use this to set TournamentGroupMembers::metric
    
    
    void countObjectivesDominated(TournamentGroup& group)
    {
        for (int i = 0; i < group.members().size(); i++)
        {
            auto& gm = group.members().at(i);
            const auto& mof = gm.individual->getMultiObjectiveFitness();
            int dominate_count = 0;
            for (int j = 0; j < mof.size(); j++)
            {
                double max_j_value = -1; // QQQQ use -inf?
                for (int k = 0; k < group.members().size(); k++)
                {
                    const auto& gm2 = group.members().at(k);
                    const auto& mof2 = gm2.individual->getMultiObjectiveFitness();
                    if (i != k)
                    {
                        double mof2_j = mof2.at(j);
                        if (max_j_value < mof2_j) { max_j_value = mof2_j; }
                    }
                }
                if (mof.at(j) > max_j_value) { dominate_count++; }
            }
            gm.metric = dominate_count;
        }
        
        std::cout << std::endl;
        std::cout << "**************************" << std::endl;
        for (auto& m : group.members())
        {
            std::cout << "metric = " << m.metric << ", mof = ";
            std::cout << m.individual->getMultiObjectiveFitness().to_string();
            std::cout << std::endl;
        }
        std::cout << "**************************" << std::endl;
        std::cout << std::endl;
    }
    
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240710 WIP for MOF elitism
    
    //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~
    // TODO 20240717 add global switch: protect_elite_mof
//    static inline bool protect_elite_mof = true;
    static inline bool protect_elite_mof = false;
    //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~

    
    // rethink name
    
    // Inspect Individuals in TournamentGroup. Determine if any are "elite": by
    // dominating any one of the MOF objectives, or by having best of population
    // scalar fitness. If elites are found, then set the TournamentGroup to be
    // invalid, so to prevent "death".
//    void protectEliteMOF(TournamentGroup group)
    void protectEliteMOF(TournamentGroup& group)
    {
        for (auto& m : group.members())
        {
            Individual* individual = m.individual;
            MultiObjectiveFitness mof = individual->getMultiObjectiveFitness();
            double prev_best_pop_fitness = bestFitness()->getFitness();
            double fitness = individual->getFitness();
            if (fitness >= prev_best_pop_fitness)
            {
//                std::cout << "QQQQ protect best scalarized fitness" << std::endl;
                group.setValid(false);
            }
            
            //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~
            // TODO 20240717 add global switch: protect_elite_mof

//            if (isEliteMOF(mof))
            if (isEliteMOF(mof) and protect_elite_mof)

            //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~
            {
//                std::cout << std::endl <<
//                    "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
//                    "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
//                    "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
//                    << std::endl;
//                debugPrint(elite_mof_)
//                std::cout << "       ";
//                debugPrint(mof)
//                std::cout << std::endl;
                group.setValid(false);
            }
        }
    }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    // Delete Individual at index i, then overwrite pointer with replacement.
    void replaceIndividual(int i, Individual* new_individual, SubPop& subpop)
    {
        delete subpop.at(i);
        subpop.at(i) = new_individual;
        sort_cache_invalid_ = true;
    }
    
    // TournamentGroup with three Individuals selected randomly from "subpop".
    TournamentGroup randomTournamentGroup(const SubPop& subpop)
    {
        auto [i, j, k] = threeUniqueRandomIndices(subpop);
        return TournamentGroup({ {subpop.at(i), i},
                                 {subpop.at(j), j},
                                 {subpop.at(k), k} });
    }
    
    // Select three unique random indices of a SubPop's Individuals.
    std::tuple<int,int,int> threeUniqueRandomIndices(const SubPop& subpop) const
    {
        assert(subpop.size() >= 3);
        std::vector<int> i;  // Vec to hold the three unique indices.
        int max_tries = 1000;
        auto add_unique_index = [&]()
        {
            int index = randomIndividualIndex(subpop);
            while (std::find(i.begin(), i.end(), index) != i.end())
            {
                index = randomIndividualIndex(subpop);
                assert(max_tries-- > 0); // Should never happen but I'm paranoid.
            }
            i.push_back(index);
        };
        add_unique_index();
        add_unique_index();
        add_unique_index();
        assert((i.at(0)!=i.at(1)) || (i.at(1)!=i.at(2)) || (i.at(2)!=i.at(0)));
        return std::make_tuple(i.at(0), i.at(1), i.at(2));
    }
    
    // Select a uniformly distributed random index of a Subpop's Individuals.
    int randomIndividualIndex(const SubPop& subpop) const
    {
        return LPRS().randomN(subpop.size());
    }
    
    // Return index of the subpopulation to be updated this step.
    int currentSubpopulationIndex() const
    {
        return getStepCount() % getSubpopulationCount();
    }
    
    // Return a reference to the subpopulation to be updated this step.
    SubPop& currentSubpopulation()
    {
        return subpopulations_.at(currentSubpopulationIndex());
    }
    
    // Called each step to create fitness sorted index of Individual pointers.
    //
    // (NOTE: when I introduced subpopulations (20210103) I made a "quick and
    // dirty" update to this function, leaving notes to come back and speed it
    // up. I was going to do that (on 20210105) but first measured it. For a
    // population of 100 it takes 0.0000376 seconds to run. (Even with a
    // population of 1000 it is about 0.0001 seconds.) Compared to everything
    // else, this seems negligible. If LazyPredator is ever used for much larger
    // population, say 10,000 or more, it is possible this might be an issue.)
    void updateSortedCollectionOfIndividuals()
    {
        if (sort_cache_invalid_)
        {
            // Collect pointers to all Individuals into sorted_collection_.
            sorted_collection_.clear();
            applyToAllIndividuals([&]
                                  (Individual* i)
                                  { sorted_collection_.push_back(i); });
            // Sort with largest fitness Individuals at the front.
            std::ranges::sort(sorted_collection_, 
                              [](Individual* a, Individual* b)
                              { return a->getFitness() > b->getFitness(); });
        }
        sort_cache_invalid_ = false;
    }
    
    // Return pointer to Individual with best fitness.
    Individual* bestFitness() { return nthBestFitness(0); }

    // Return pointer to Individual with nth best fitness (n=0 is best fitness).
    Individual* nthBestFitness(int n)
    {
        updateSortedCollectionOfIndividuals();
        return sorted_collection_.at(n);
    }
    
    // Average of "tree size" over all Individuals.
    int averageTreeSize() const
    {
        int total = 0;
        auto f = [&](Individual* i){ total += i->tree().size(); };
        applyToAllIndividuals(f);
        return total / getIndividualCount();
    }
    
    // Average of "tournaments survived" (or abs fitness) over all Individuals.
    float averageFitness() const
    {
        float total = 0;
        auto f = [&](Individual* i){ total += i->getFitness(); };
        applyToAllIndividuals(f);
        return total / getIndividualCount();
    }
    
    // Occasionally migrate (swap) Individuals between current and random SubPop.
    void subpopulationMigration()
    {
        int spc = getSubpopulationCount();
        if ((spc > 1) && (LPRS().frandom01() < getMigrationLikelihood()))
        {
            // Get indices of, and references to, the current subpopulation,
            // plus a different, random subpopulation.
            int random_index_offset = 1 + LPRS().randomN(spc - 1);
            int subpop_index_1 = currentSubpopulationIndex();
            int subpop_index_2 = (subpop_index_1 + random_index_offset) % spc;
            SubPop& subpop1 = subpopulation(subpop_index_1);
            SubPop& subpop2 = subpopulation(subpop_index_2);
            // Randomly pick an Individual in each SubPop.
            int individual_index_1 = randomIndividualIndex(subpop1);
            int individual_index_2 = randomIndividualIndex(subpop2);
            Individual* individual_1 = subpop1.at(individual_index_1);
            Individual* individual_2 = subpop2.at(individual_index_2);
            // Swap them.
            subpop1.at(individual_index_1) = individual_2;
            subpop2.at(individual_index_2) = individual_1;
        }
    }
    
    // Run "steps" of evolution, given "tournament_function".
    void run(int steps, TournamentFunction tournament_function)
    {
        // Run given number of steps.
        for (int i = 0; i < steps; i++)
        {
            // Run evolution step with given tournament and function set.
            evolutionStep(tournament_function);
        }
    }
    
    // Called at the end of each evolutionStep(). Can override by subclassing or
    // with setLoggerFunction().
    virtual void logger()
    {
        sort_cache_invalid_ = true;
        updateSortedCollectionOfIndividuals();
        if (logger_function_) logger_function_(*this);
    }
    void setLoggerFunction(std::function<void(Population&)> logger_function)
    {
        logger_function_ = logger_function;
    }
    static void basicLogger(Population& p)
    {
        TimePoint now_time = TimeClock::now();
        TimeDuration elapsed_time = now_time - p.start_time_ - p.idle_time_;
        p.start_time_ = now_time;
        int default_precision = int(std::cout.precision());
        std::cout << p.getStepCount() << ": t=";
        std::cout << std::setprecision(3) << elapsed_time.count() << ", ";
        std::cout << std::setprecision(default_precision);
        std::cout << "pop ave size=" << p.averageTreeSize();
        float af = p.averageFitness();
        std::cout << " fit=";
        if (af < 100) { std::cout << af; } else { std::cout << int(af); }
        std::cout << ", pop best (" << std::setprecision(2);
        for (int i = 0; i < 10; i++)
        {
            if (i > 0) std::cout << " ";
            float f = p.nthBestFitness(i)->getFitness();
            if (f <= 100) { std::cout << f; } else { std::cout << int(f); }
        }
        std::cout << ")" << std::setprecision(default_precision);
        std::cout << std::endl;
    }
    
    // Apply the given function to all Individuals in this Population.
    void applyToAllIndividuals(std::function<void(Individual*)> func) const
    {
        for (auto& subpop : subpopulations_)
        {
            for (auto& individual : subpop) { func(individual); }
        }
    }
    
    // Return (writable/non-const) reference to i-th subpopulation.
    // Maybe this should be private, perhaps with a const public version?
    SubPop& subpopulation(int s) { return subpopulations_.at(s); }
    
    // Returns total number of Individuals contained in this Population
    int getIndividualCount() const
    {
        int count = 0;
        for (auto& subpop : subpopulations_) { count += subpop.size(); }
        return count;
    }
    
    // Get/set this Population's FunctionSet.
    const FunctionSet* getFunctionSet() const { return function_set_; }
    void setFunctionSet(const FunctionSet* fs) { function_set_ = fs; }
    
    // Returns number of subpopulations.
    int getSubpopulationCount() const { return int(subpopulations_.size()); }
    // Returns number of evolution steps already taken in this Population.
    int getStepCount() const { return step_count_; }
    void incrementStepCount() { step_count_++; }
    // The probability, on any given evolutionStep(), that migration will occur.
    float getMigrationLikelihood() const { return migration_likelihood_; }
    void setMigrationLikelihood(float ml) { migration_likelihood_ = ml; }
    
    // Get/set this Population's max tree size for initial random trees.
    int getMaxInitTreeSize() const { return max_init_tree_size_; }
    void setMaxInitTreeSize(int size) { max_init_tree_size_ = size; }
    
    // Get/set min/max crossover tree size.
    int getMinCrossoverTreeSize() const { return min_crossover_tree_size_; }
    int getMaxCrossoverTreeSize() const { return max_crossover_tree_size_; }
    void setMinCrossoverTreeSize(int size) { min_crossover_tree_size_ = size; }
    void setMaxCrossoverTreeSize(int size) { max_crossover_tree_size_ = size; }
    
    // Duration of idle time during step that should be ignored for logging.
    void setIdleTime(TimeDuration duration) { idle_time_ = duration; }
    
private:
    std::function<void(Population&)> logger_function_ = basicLogger;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
    int step_count_ = 0;
    // One or more collections of Individual*, each a subpopulation (deme).
    std::vector<SubPop> subpopulations_;
    // Cached index of all Individuals sorted by fitness.
    SubPop sorted_collection_;
    // Sorted index of Individuals is cached until a change is made.
    bool sort_cache_invalid_ = true;
    // Const pointer to this Population's FunctionSet.
    const FunctionSet* function_set_ = nullptr;
    // The probability, on any given evolutionStep(), that migration will occur.
    float migration_likelihood_ = 0.05;
    // Max size for initial random trees.
    int max_init_tree_size_ = 0;
    // Min/max crossover tree size.
    int min_crossover_tree_size_ = 0;
    int max_crossover_tree_size_ = std::numeric_limits<int>::max();
    // Duration of idle time during step that should be ignored for logging.
    TimeDuration idle_time_;
};

// TODO had been in main.cpp, now here.
// TODO prototype here, move into it own file later?
class CountFunctionUsage
{
public:
    CountFunctionUsage(){}
    typedef std::map<std::string, int> CountMap;
    const CountMap& countMap() const { return count_map_; }
    // Count GpFunction usage in a given GpTree, add into running totals.
    void count(const GpTree& tree)
    {
        if (!tree.isLeaf())
        {
            count_map_[tree.getRootFunction().name()]++;
            for (auto subtree : tree.subtrees()) count(subtree);
        }
    }
    // Count GpFunction usage in a given Population.
    void count(const Population& population)
    {
        auto count_individual = [&](Individual* i) { count(i->tree()); };
        population.applyToAllIndividuals(count_individual);
    }
    // Total of all counts for all GpFunction.
    int totalCount() const
    {
        int total = 0;
        for (auto& [string, count] : count_map_) { total += count; }
        return total;
    }
    
    
    // Preserve each named counter in map, but set its count to zero.
    void zeroEachCounter()
    {
        for (auto& [name, count] : count_map_) { count_map_[name] = 0; }
    }
    
    
    // Apply given function to all counts (args: GpFunc name, count)
    void applyToAllCounts(std::function<void(std::string, int)> func) const
    {
        for (auto& [string, count] : countMap()) { func(string, count); }
    }
    void clear() { count_map_.clear(); }
    
private:
    // A map from string name of GpFunction to current count of usages.
    CountMap count_map_;
};

}  // end of namespace LazyPredator
