//
//  Individual.h
//  LazyPredator
//
//  Created by Craig Reynolds on 8/6/20.
//  Copyright © 2020 Craig Reynolds. All rights reserved.
//

#pragma once
#include "Utilities.h"
#include "FunctionSet.h"
#include "MultiObjectiveFitness.h"

namespace LazyPredator
{

class Individual
{
public:
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240619 WIP first GP_not_GA run
    // VERY TEMP
    std::string tree_to_string() const
    {
        return tree_.to_string(true);
    }
    
    int pop_position = 0;  // temp, for debugging, to track one instance

//    int qqq_count = 0;
    GpTree duplicate_tree;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    Individual()
    {
        constructor_count_++;
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240712 experiment with increasing initial tree size.
    
//    Individual(int max_tree_size, const FunctionSet& fs) : Individual()
//    {
//        fs.makeRandomTree(max_tree_size, tree_);
//    }


    static inline bool increasing_initial_tree_size = false;
//    static inline int iits_counter = 0;
    
    Individual(int max_tree_size, const FunctionSet& fs) : Individual()
    {
        if (not increasing_initial_tree_size)
        {
            fs.makeRandomTree(max_tree_size, tree_);
        }
        else
        {
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20240715 WIP new approach to tree generation.
                        
            tree_ = fs.newMakeRandomTree(max_tree_size / 2, max_tree_size);

            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        }
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Individual(const GpTree& gp_tree) : Individual() { tree_ = gp_tree; }
    
    virtual ~Individual()
    {
        // Tell tree to delete any heap-allocated cached values it created when
        // it last did eval(). This happens via the optional "deleter" function
        // on a GpType.
        tree_.deleteCachedValues();
        destructor_count_++;
    }
    // Read-only (const) access to this Individual's GpTree.
    const GpTree& tree() const { return tree_; }
    // Return/cache the result of running/evaluating this Individual's GpTree.
    std::any treeValue()
    {
        if (!tree_evaluated_)
        {
            assert(tree_eval_counter_ == 0);
            tree_.eval();
            tree_evaluated_ = true;
            tree_eval_counter_++;
        }
        return tree_.getRootValue();
    }
    // Get/inc count of tournament Individual has survived (did not "lose").
    int getTournamentsSurvived() const { return tournaments_survived_; }
    void incrementTournamentsSurvived() { tournaments_survived_++; }
    // Added to support "absolute fitness" in addition to "tournament fitness".
    bool hasFitness() const { return has_fitness_; }
    void setFitness(float f) { fitness_ = f; has_fitness_ = true; }
    float getFitness() const
    {
        return hasFitness() ? fitness_ : getTournamentsSurvived() + getStanding();
    }
    bool hasMultiObjectiveFitness() const { return has_multi_objective_fitness_; }
    void setMultiObjectiveFitness(MultiObjectiveFitness multi_objective_fitness)
    {
        multi_objective_fitness_ = multi_objective_fitness;
        has_multi_objective_fitness_ = multi_objective_fitness_.size() > 0;
    }
    MultiObjectiveFitness getMultiObjectiveFitness() const
    {
        return multi_objective_fitness_;
    }
    // Compare counts of constructor/destructor calls. Must match at end of run.
    static void leakCheck()
    {
        std::cout << "Individual";
        std::cout << ": constructions=" << constructor_count_;
        std::cout << ", destructions=" << destructor_count_;
        std::cout << ", leaked=" << getLeakCount();
        std::cout << std::endl;
    }
    // Return difference between constructor_count_ and destructor_count_
    static int getLeakCount()
    {
        return constructor_count_ - destructor_count_;
    }

    // TODO temporary for testing SimpleImageMatch
    float alt_fitness = -1;
    
    // TODO experiment for SimpleImageMatch: maybe the "fitness" to use with
    // tournament-based relative fitness is not just "tournaments_survived" but
    // (lets call it) "standing": "tournaments_survived" plus the max of
    // "fitness" of every other Inidividual this one bested in a tournament.
    
    //    int getStanding() const { return getTournamentsSurvived() + standing_; }
    int getStanding() const { return standing_; }
    void adjustStandingForWinAgainst(const Individual& defeated)
    {
        standing_ = std::max(standing_, int(defeated.getFitness()));
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // 20230308 TODO prototyping float "static quality metric"
    // This seems the same as absolute fitness. But under time pressure (in the
    // week between rejection by SIGGRAPH 2023 and submission to ALIFE 2023) I
    // do not want to risk breaking anything.
    bool has_sqm_ = false;
    float static_quality_metric_ = 0;
    bool hasStaticQualityMetric() const { return has_sqm_; }
    float getStaticQualityMetric() const { return static_quality_metric_; }
    void setStaticQualityMetric(float sqm)
    {
        static_quality_metric_ = sqm;
        has_sqm_ = true;
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
private:
    GpTree tree_;
    // Resettable cache for result of tree evaluation.
    bool tree_evaluated_ = false;
    // Make sure we don't eval() the tree more than once. (TODO Still needed?)
    int tree_eval_counter_ = 0;
    // Number of tournament this Individual has survived (did not "lose").
    int tournaments_survived_ = 0;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO experiment for SimpleImageMatch: maybe the "fitness" to use with
    // tournament-based relative fitness is not just "tournaments_survived" but
    // that plus  (lets call it) "standing": "tournaments_survived" plus the max
    // of "fitness" of every other Individual this one bested in a tournament.
    int standing_ = 0;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Added to support "absolute fitness" in addition to "tournament fitness".
    float fitness_ = 0;
    bool has_fitness_ = false;
    MultiObjectiveFitness multi_objective_fitness_;
    bool has_multi_objective_fitness_ = false;
    // Leak check. Count constructor/destructor calls. Must match at end of run.
    static inline int constructor_count_ = 0;
    static inline int destructor_count_ = 0;
    
public:
    // Per-Individual storage for application-specific data, eg for plotting.
    // This value is ignored by Individual and the rest of LazyPredator.
    std::vector<double> user_data_for_plotting;
private:
};

}  // end of namespace LazyPredator
