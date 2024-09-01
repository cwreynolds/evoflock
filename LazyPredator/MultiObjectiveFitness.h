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




#include <numeric>  // for MultiObjectiveFitness

namespace LazyPredator
{

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240423 change MultiObjectiveFitness from typedef to class
//               Probably should be moved somewhere else.

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
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240708 make MultiObjectiveFitness.at() work for read AND write.
    
//    double at(size_t i) const { return mof_.at(i); }
    
    double& at(size_t i) { return mof_.at(i); }
    const double& at(size_t i) const { return mof_.at(i); }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240718 experiment using "hypervolume" as metric for TG
    //               try to refactor like product()
    double hyperVolume() const
    {
        double volume = 1;
        double min = 0.01;
        for (auto& o : mof_) { assert(util::between(o, 0.0, 1.0)); }
        for (auto& o : mof_) { volume *= std::max(o, min); }
        return volume;
    }
    

//    // TODO 20240719 experimental variation of hyperVolume()
//    // This did not seem to fix the run of failed sims, so probably not worth keeping
//    
//    // Like hyperVolume() (floored product) but occasionally sets the max MOF
//    // element to zero.
//    double hyperVolumeDropout() const
//    {
//        double volume = hyperVolume();
//        if (LPRS().randomBool(0.25))
//        {
//            MultiObjectiveFitness mof = *this;
//            for (int i = 0; i < mof.size(); i++)
//            {
//                if(mof.at(i) == mof.max()){ mof.at(i) = 0; }
//            }
//            volume = mof.hyperVolume();
//        }
//        return volume;
//    }
//
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
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
            }
            assert(all_normalized);
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
