//
//  TournamentGroup.h
//  LazyPredator
//
//  Created by Craig Reynolds on 1/1/21.
//  Copyright © 2021 Craig Reynolds. All rights reserved.
//
//
// Classes to represent the Individuals participating in a tournament. Each one
// is a TournamentGroupMember composed of an Individual pointer, an index in the
// Population, and an optional float fitness metric. The collection of those, a
// TournamentGroup, is passed around functions related to tournaments. Group
// members are kept in sorted order with high fitness Individuals at the front.

#pragma once
#include "Individual.h"

namespace LazyPredator
{


// One member of a TournamentGroup, an Individual plus bookkeeping data.
class TournamentGroupMember
{
public:
    TournamentGroupMember()
    : individual(nullptr), index(0), metric(0) {}
    TournamentGroupMember(Individual* individual_, int index_)
    : individual(individual_), index(index_), metric(0) {}
    TournamentGroupMember(Individual* individual_, int index_, float metric_)
    : individual(individual_), index(index_), metric(metric_) {}
    Individual* individual = nullptr;  // pointer to an Individual.
    int index = 0;                     // Individual's index in Population.
    float metric = 0;                  // Optional fitness metric.
};

// The group of Individuals participating in a tournament.
class TournamentGroup
{
public:
    TournamentGroup() { sort(); }
    TournamentGroup(const std::vector<TournamentGroupMember>& member_list)
    : members_(member_list) { sort(); }
    // Const reference to vector of members. (TODO any need for this?)
    const std::vector<TournamentGroupMember>& members() const {return members_;}
    std::vector<TournamentGroupMember>& members() {return members_;}
    // Number of members in this group (normally 3).
    size_t size() const { return members().size(); }
    // For "numerical fitness"-based tournaments, map a given scoring function
    // over all members to set the metric values. Sorts members afterward.
    void setAllMetrics(std::function<float(Individual*)> scoring)
    {
        for (auto& m : members_) { m.metric = scoring(m.individual); }
        sort();
    }
    int worstIndex() const { return members().front().index; }
    Individual* worstIndividual() const { return members().front().individual; }
    Individual* bestIndividual() const { return members().back().individual; }
    Individual* secondBestIndividual() const
    {
        return members().at(size() - 2).individual;
    }
    // For debugging/testing. TODO or should this be to_string() ?
    void print() const
    {
        // TODO need a template utility for printing an std::vector as a comma
        //      separated list.
        std::cout << "TournamentGroup: {";
        bool first = true;
        for (auto& m : members())
        {
            if (!first) { std::cout << ", "; first = false; }
            std::cout << "{" << m.individual << ", ";
            std::cout << m.index << ", " << m.metric << "}";
        }
        std::cout << "}" << std::endl;
    }
    // Given an Individual, returns an int from 1 to members().size(),
    // rank 1 corresponds to bestIndividual().
    int rankOfIndividual(Individual* individual)
    {
        int rank = 0;
        int count = int(members().size());
        for (int i = 0; i <  count; i++)
        {
            if (individual == members().at(i).individual) rank = count - i;
        }
        assert("given Individual not in TournamentGroup" && rank != 0);
        return rank;
    }
    // Used by a TournamentFunction to designate one of the Individuals in this
    // TournamentGroup as the worst, the "loser", the one to be removed from
    // the Population and replaced by a new offspring.
    void designateWorstIndividual(Individual* worst_individual)
    {
        if (isMember(worst_individual))
        {
            // Set metric of worst_individual to 0, others to 1, then sort.
            for (auto& tgm : members())
            {
                tgm.metric = ((tgm.individual == worst_individual) ? 0 : 1);
            }
            sort();
        }
    }
    // Given a TournamentGroup with MultiObjectiveFitness, select a "best" index
    // into the MultiObjectiveFitness vector, based on largest range between min
    // and max value for each index (later: OR the index with the lowest bottom
    // of range). NOTE that this implicitly assumes fitness (objective) values
    // are normalized on the range [0,1], otherwise the range size would not be
    // directly comparable.
    size_t pickMultiObjectiveFitnessIndex()
    {
        size_t mof_size = adjustMofSizeForPriority();
        double infinity = std::numeric_limits<double>::infinity();
        // Loop over the mof index range. For each one find the range between
        // max and min fitness. Return the index corresponding to largest range
        // or the lowest bottom.
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240712 reconsider these "better index" strategies.
//        int best_index_big_range = 0;
//        int best_index_lowest_bottom = 0;
        int best_index_big_range = LPRS().randomN(mof_size);
        int best_index_lowest_bottom = LPRS().randomN(mof_size);
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        double biggest_range = 0;
        double lowest_bottom = infinity;
        for (int i = 0; i < mof_size; i++)
        {
            double min_fit = infinity;
            double max_fit = -infinity;
            for (auto& m : members())
            {
                double fit = m.individual->getMultiObjectiveFitness().at(i);
                min_fit = std::min(fit, min_fit);
                max_fit = std::max(fit, max_fit);
            }
            double range = max_fit - min_fit;
            if (biggest_range < range)
            {
                biggest_range = range;
                best_index_big_range = i;
            }
            if ((lowest_bottom >= min_fit) and (range > 0))
            {
                lowest_bottom = min_fit;
                best_index_lowest_bottom = i;
            }
        }
        return (LPRS().randomBool(0.3) ?
                best_index_big_range :
                best_index_lowest_bottom);
    }
        
    // "Occasionally" ignore some of the fitness objectives listed later in MOF.
    // This serves to give priority to those listed earlier, allowing the
    // application to specify which objectives are more important.
    size_t adjustMofSizeForPriority() const
    {
        size_t mof_size = checkValidForMultiObjectiveFitness();
        // Half of the time leave MOF size unchanged.
        if (LPRS().randomBool())
        {
            // Otherwise drop some random number of fitness objectives off end.
            if (mof_size > 1) { mof_size = LPRS().randomN(mof_size - 1) + 1; }
        }
        return mof_size;
    }

    // First make sure TournamentGroup is set up for MultiObjectiveFitness.
    // (Also returns the size of MultiObjectiveFitness vectors being used.)
    size_t checkValidForMultiObjectiveFitness() const
    {
        // First make sure TournamentGroup is set up for MultiObjectiveFitness.
        assert((members().size() > 0) and "TournamentGroup must have members");
        Individual* individual0 = members().at(0).individual;
        size_t mof_size = individual0->getMultiObjectiveFitness().size();
        assert((mof_size > 0) and "must have MultiObjectiveFitness data");
        // Make sure mo_fitness of all group members is the same size.
        for (auto& m : members())
        {
            assert((mof_size == m.individual->getMultiObjectiveFitness().size())
                   and "all MultiObjectiveFitness vectors must be same size");
        }
        return mof_size;
    }
//    // Used (initially for multiobjective fitness) to store a fitness evaluation
//    // function to be applied to the offspring created during this tournament.
//    std::function<void(Individual*)> custom_eval = nullptr;
    // Is the given Individual a member of this TournamentGroup?
    bool isMember(Individual* individual) const
    {
        bool member_of_group = false;
        for (auto& tgm : members())
        {
            if (tgm.individual == individual)
            {
                member_of_group = true;
            };
        }
        return member_of_group;
    }
    // Get/set TournamentGroup's validity. True by default. TournamentFunction
    // can set to false, canceling tournament, so leaving population unchanged.
    bool getValid() const { return valid_; }
    void setValid(bool new_validity) { valid_ = new_validity; }
    // Sort the members of this group by their "metric" value, least first.
    void sort()
    {
        std::ranges::sort(members_,
                          [](TournamentGroupMember& a, TournamentGroupMember& b)
                          { return a.metric < b.metric; });
    }
private:
    std::vector<TournamentGroupMember> members_;
    // Can set to false, canceling tournament, so leaving population unchanged.
    bool valid_ = true;
};

}  // end of namespace LazyPredator
