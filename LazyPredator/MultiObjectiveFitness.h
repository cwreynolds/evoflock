//-------------------------------------------------------------------------------
//
//  MultiObjectiveFitness.h
//  LazyPredator
//
//  A class to represent a multi-objective fitness value, and related utilities.
//
//  Created by Craig Reynolds on August 30, 2024.
//  MIT License -- Copyright © 2024 Craig Reynolds
//-------------------------------------------------------------------------------

#pragma once
#include <numeric>  // for MultiObjectiveFitness

namespace LazyPredator
{

// This class is just a slightly elaborated wrapper around a std::vector<double>
class MultiObjectiveFitness
{
public:
    MultiObjectiveFitness() : MultiObjectiveFitness({}, {}) {}
    MultiObjectiveFitness(const std::vector<double>& multi_objective_fitness)
      : MultiObjectiveFitness(multi_objective_fitness, {}) {}
    MultiObjectiveFitness(const std::vector<double>& multi_objective_fitness,
                          const std::vector<std::string>& names)
    {
        mof_ = multi_objective_fitness;
        names_ = names;
        assertNormalized();
    }
    size_t size() const { return mof_.size(); }
    double& at(size_t i) { return mof_.at(i); }
    const double& at(size_t i) const { return mof_.at(i); }
    auto begin() const { return mof_.begin(); }
    auto end() const { return mof_.end(); }
    double min() const { return *std::min_element(begin(), end()); }
    double max() const { return *std::max_element(begin(), end()); }
    double sum() const { return std::reduce(begin(), end(), 0.0, std::plus()); }
    bool empty() const { return size() == 0; }
    double average() const { return sum() / size(); }
    double product() const { return std::reduce(begin(), end(), 1.0,
                                                std::multiplies()); }
    std::string to_string() const { return vec_to_string(mof_); }
    const std::vector<double> as_vector() const { return mof_; }
    double hyperVolume() const
    {
        double volume = 1;
        double min = LPRS().frandom01() * 0.01;
        for (auto& o : mof_) { assert(util::between(o, 0.0, 1.0)); }
        for (auto& o : mof_) { volume *= std::max(o, min); }
        return volume;
    }
    
    // Are all scalar fitness components on the range [0,1]?
    bool allComponentsAreNormalized() const
    {
        // TODO 20240901 there is probably a higher level way to express this
        // (reduce? transform_reduce?) but for now just an old fashioned loop.
        bool all_normalized = true;
        for (auto& component : mof_)
        {
            if (not util::between(component, 0, 1)) { all_normalized = false; }
        }
        return all_normalized;
    }
    
    static inline bool assert_normalized = true;
    
    void assertNormalized() const
    {
        if (assert_normalized)
        {
            bool all_normalized = allComponentsAreNormalized();
            if (not all_normalized)
            {
                std::cout <<
                "MultiObjectiveFitness has so far (Sept 1, 2024) only been used "
                "in a regime where all scalar fitness components are normalized "
                "on [0,1]. There may be bugs based on this assumption. If you "
                "understand that and want to use “unnormalized” fitness values, "
                "set LazyPredator::MultiObjectiveFitness::assert_normalized to "
                " false." << std::endl;
                std::cout << "MOF = {" << to_string() << "}" << std::endl;
            }
            assert(all_normalized);
        }
    }

    static void unit_test()
    {
//        // TEMP
//        std::cout << "In LP::MultiObjectiveFitness::unit_test()." << std::endl;
        
        // Test constructors empty(), and size().
        {
            assert(MultiObjectiveFitness().empty());
            assert(MultiObjectiveFitness().size() == 0);
            assert(MultiObjectiveFitness({0.1}).size() == 1);
            assert(MultiObjectiveFitness({0.1, 0.2}, {"a", "b"}).size() == 2);
        }
        // Test at() for read and write.
        {
            MultiObjectiveFitness mof({0.0, 0.2, 0.4});
            assert(mof.at(1) == 0.2);
            mof.at(1) = 0.5;
            assert(mof.at(1) == 0.5);
        }
        // Test min(), max(), sum(), average(), and product()
        {
            MultiObjectiveFitness mof({0.2, 0.4, 0.6});
            assert(mof.min() == 0.2);
            assert(mof.max() == 0.6);
            assert(util::within_epsilon(mof.sum(), 1.2));
            assert(util::within_epsilon(mof.average(), 0.4));
            assert(util::within_epsilon(mof.product(), 0.048));
        }
        // Test as_vector() and to_string().
        {
            std::vector<double> std_vector{0.1, 0.2, 0.3};
            MultiObjectiveFitness mof(std_vector);
            assert(mof.as_vector() == std_vector);
            assert(mof.to_string() == "0.1, 0.2, 0.3");
        }
        
        // TODO should hyperVolume() be kept? Add unit test if so.
        // TODO test name retrival
        
        // Test predicate for "all scalar fitnesses are normalized".
        {
            assert_normalized = false;  // Turn default check off.
            MultiObjectiveFitness norm0({0, 2, 4, 6, 8, 10});
            MultiObjectiveFitness norm1({0.0, 0.2, 0.4, 0.6, 0.8, 1.0});
            assert(not norm0.allComponentsAreNormalized() and
                   norm1.allComponentsAreNormalized());
            assert_normalized = true;   // Turn default check back on.
        }
    }
    
private:
    std::vector<double> mof_;
    
    // TODO maybe instead of this in each instance, maybe just provide a
    // utility function which takes this as an argument?
    std::vector<std::string> names_;
};

}  // end of namespace LazyPredator


// Serialize MultiObjectiveFitness object to stream.
std::ostream& operator<<(std::ostream& os,
                         const LazyPredator::MultiObjectiveFitness& mfo)
{
    os << "{" << mfo.to_string() << "}";
    return os;
}
