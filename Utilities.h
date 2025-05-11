//------------------------------------------------------------------------------
//
//  Utilities.h -- new flock experiments
//
//  Utility functions
//
//  Defines a few symbols in global namespace (RandomSequence, debugPrint(),
//  sq()) but otherwise defines small convenience functions with names inside
//  the "Utilities" namespace which has an alias "util".
//
//  Created by Craig Reynolds on January 8, 2024.
//  (Based on earlier C++ and Python versions.)
//  MIT License -- Copyright © 2024 Craig Reynolds
//
//------------------------------------------------------------------------------

#pragma once
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <cassert>
#include <mutex>
class Vec3;

#include <sstream>

namespace Utilities 
{
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20241221 square panels on cylinders

double pi = std::acos(-1);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Generic interpolation
template<typename F,typename T>
T interpolate(const F& alpha, const T& x0, const T& x1)
{
    return (x0 * (1 - alpha)) + (x1 * alpha);
}

// Constrain a given value "x" to be between two bounds: "bound0" and "bound1"
// (without regard to order). Returns x if it is between the bounds, otherwise
// returns the nearer bound.
inline double clip(double x, double bound0, double bound1)
{
    double clipped = x;
    double min = std::min(bound0, bound1);
    double max = std::max(bound0, bound1);
    if (clipped < min) clipped = min;
    if (clipped > max) clipped = max;
    return clipped;
}

inline double clip01 (const double x)
{
    return clip(x, 0, 1);
}

// True when x is between given bounds (low ≤ x ≤ high)
inline bool between(double x, double a, double b)
{
    return (std::min(a, b) <= x) && (x <= std::max(a, b));
}

// Takes a 32 bit value and shuffles it around to produce a new 32 bit value.
// "Robert Jenkins' 32 bit integer hash function" from "Integer Hash Function"
// (1997) by Thomas Wang (https://gist.github.com/badboy/6267743)
// Altered to accept input as uint64_t but ignores the top 32 bits.
inline uint32_t rehash32bits(uint64_t u64)
{
    uint32_t a = uint32_t(u64);
    a = (a+0x7ed55d16) + (a<<12);
    a = (a^0xc761c23c) ^ (a>>19);
    a = (a+0x165667b1) + (a<<5);
    a = (a+0xd3a2646c) ^ (a<<9);
    a = (a+0xfd7046c5) + (a<<3);
    a = (a^0xb55a4f09) ^ (a>>16);
    return a;
}

// Taken from https://en.wikipedia.org/wiki/Logistic_function
double logistic(double x, double k, double L, double x0)
{
    return L / (1 + std::exp(-k * (x - x0)));
}
// Logistic sigmoid (s-curve) from ~(0,0) to ~(1,1), ~0 if x<0, ~1 if x>1
// (See a plot of this function via Wolfram|Alpha: https://bit.ly/3sUYbeJ)
double unit_sigmoid_on_01(double x) { return logistic(x, 12, 1, 0.5);}

// Remap a value specified relative to a pair of bounding values
// to the corresponding value relative to another pair of bounds.
// Inspired by (dyna:remap-interval y y0 y1 z0 z1) circa 1984.
inline double remap_interval(double x,
                             double in0, double in1,
                             double out0, double out1)
{
    // Remap if input range is nonzero, otherwise blend them evenly.
    double input_range = in1 - in0;
    double blend = (input_range == 0) ? 0.5 : ((x - in0) / input_range);
    return interpolate(blend, out0, out1);
}

// Like remapInterval but the result is clipped to remain between out0 and out1
inline double remap_interval_clip(double x,
                                  double in0, double in1,
                                  double out0, double out1)
{
    return clip(remap_interval(x, in0, in1, out0, out1), out0, out1);
}

// Are a and b on opposite sides of 0? Specifically: was there a zero crossing
// if they are the previous and current value of a "signed distance function"?
bool zero_crossing(double a, double b)
{
    return ((a >= 0) and (b <= 0)) or ((a <= 0) and (b >= 0));
}

// Maps from 0 to 1 into a sinusoid ramp ("slow in, slow out") from 0 to 1.
inline double sinusoid (double x)
{
    return (1 - std::cos(x * M_PI)) / 2;
}

// Convert a std::vector to a string, with comma-space between printed elements.
// If not 0, fixed_precision allows printing columns each row is an std::vector.
template <typename T> std::string vec_to_string(const std::vector<T>& vector,
                                                int fixed_precision)
{
    std::stringstream s;
    bool first = true;
    if (fixed_precision > 0)
    {
        s << std::fixed << std::setprecision(fixed_precision);
    }
    for (auto& element : vector)
    {
        if (first) first = false; else s << ", ";
        s << element;
    }
    return s.str();
}

template <typename T> std::string vec_to_string(const std::vector<T>& vector)
{
    return vec_to_string(vector, 0);
}

// Utility for blending per-step values into accumulators for low pass filtering.
template<typename T>
class Blender
{
public:
    Blender() {}
    // "smoothness" controls how much smoothing. Values around 0.8-0.9 seem most
    // useful. smoothness=1 is infinite smoothing. smoothness=0 is no smoothing.
    T blend(T new_value, double smoothness)
    {
        assert(between(smoothness, 0, 1));
        double s = global_enable ? smoothness : 0;
        value = first ? new_value : interpolate(s, new_value, value);
        first = false;
        return value;
    }
    void clear() { first = true; }
    static inline bool global_enable = true;  // Used only for testing.
    T value;
private:
    bool first = true;
};

// This value (aka 1e-15) works on my laptop with c++17 (uses 1e-14 for python)
double epsilon = 0.000000000000001;

// True when a and b differ by no more than epsilon.
bool within_epsilon(double a, double b, double e=epsilon)
{
    return std::abs(a - b) <= e;
}

// Define a global, static std::recursive_mutex to allow synchronizing console
// output from parallel threads.
//
// TODO maybe make constructor do the work now done by "debugPrint(e)" macro?
//
class DebugPrint
{
public:
    static std::recursive_mutex& getPrintMutex()
    {
        static std::recursive_mutex print_mutex_;
        return print_mutex_;
    }
};

// Short names
typedef std::chrono::high_resolution_clock TimeClock;
typedef std::chrono::time_point<TimeClock> TimePoint;
typedef std::chrono::duration<double> TimeDuration;

// TimeDuration to seconds as double.
inline double time_duration_in_seconds(TimeDuration time_duration)
{
    return time_duration.count();
}

// TimePoint difference in seconds.
inline double time_diff_in_seconds(TimePoint start, TimePoint end)
{
    TimeDuration dt = end - start;
    return time_duration_in_seconds(dt);
}

// Make current thread sleep for the given duration in seconds.
void thread_sleep_in_seconds(double sleep_time)
{
    int micro_seconds = sleep_time * 1000000;
    std::this_thread::sleep_for(std::chrono::microseconds(micro_seconds));
}


// Simple tool for inline timing sections of code. For example:
//    void foo()
//    {
//        Timer foo_timer("foo");
//        bar();
//        zap();
//    }
// After the block containing the Timer construction it prints:
//    foo elapsed time: 1.86984 seconds
// Can also be used without default logging, via Timer::elapsedSeconds() as in:
//    void foo()
//    {
//        Timer foo_timer;
//        bar();
//        zap();
//        if (foo_timer.elapsedSeconds() > 1.2) too_slow();
//    }
class Timer
{
public:
    Timer() : start_time_(TimeClock::now()) {}
    Timer(const std::string& description) : Timer() {description_= description;}
    ~Timer()
    {
        if (!description_.empty())
        {
            double es = elapsedSeconds();
            std::cout << description_ << " elapsed time: ";
            std::cout << es << " seconds";
            double minute = 60;
            double hour = 60 * 60;
            if (es > minute)
            {
                std::cout << " (" << es / minute << " minutes";
                if (es > hour) { std::cout << ", " << es / hour << " hours"; }
                std::cout << ")";
            }
            std::cout << std::endl;
        }
    }
    double elapsedSeconds() const
    {
        return time_diff_in_seconds(start_time_, TimeClock::now());
    }
private:
    std::string description_;
    TimePoint start_time_;
};

// A "rolling average" or "box filter" for a data stream. Initialized with a
// type T and a width/history-length "size". A new datum is added with insert().
// A history of the given size is maintained. Use average() to return the mean
// of the last "size" data points.
template<typename T>
class RollingAverage
{
public:
    RollingAverage() {}
    RollingAverage(size_t size) : size_(size) {}
    void insert(T new_data)
    {
        if (size_ > data_.size())
        {
            data_.push_back(new_data);
        }
        else
        {
            data_.at(index_) = new_data;
            index_ = (index_ + 1) % size_;
        }
    }
    T sum() const
    {
        return std::reduce(data_.begin(), data_.end(), 0.0, std::plus());
    }
    T average() const { return sum() / data_.size(); }
    bool empty() const { return data_.empty(); }
    std::string to_string() const { return vec_to_string(data_, 4); }
    void fill(T x) { data_.resize(size_, x); }
private:
    size_t size_ = 0;
    size_t index_ = 0;
    std::vector<T> data_;
};

// Timer/clock class for animation. Originally misplaced in Draw in the earlier
// Python "flock" project. Now split off because in evoflock the lifetime of
// Draw is unrelated to the lifetime of an animation clock. TODO: should this
// be renamed AnimationClock?
class AnimationTimer
{
public:
    AnimationTimer() : AnimationTimer(30) {}

    AnimationTimer(int frames_per_second)
      : frame_start_time_(TimeClock::now()),
        frame_duration_history_(5),
        frames_per_second_(frames_per_second)
    {}

    // Measured duration of the previous frame. Used as the simulation time step
    // for the current frame. I think that off-by-one delay is inevitable.
    double frameDuration() const { return frame_duration_; }
    
    // Count of frames this simulation (aka simulation steps).
    int frameCounter() const { return frame_counter_; }
        
    // Record real time at beginning of frame.
    void setFrameStartTime() { frame_start_time_ = TimeClock::now(); }

    // Called after simulation step and frame draw: sleeps until min_frame_time.
    void sleepUntilEndOfFrame(double min_frame_time)  // In seconds.
    {
        if (min_frame_time > 0)
        {
            // Elapsed time since beginning of frame: simulation and graphics.
            double non_sleep_time = time_diff_in_seconds(frame_start_time_,
                                                         TimeClock::now());
            // Amount of time to sleep until the end of the current frame.
            double sleep_time = min_frame_time - non_sleep_time;
            // Adjust based on N previous frame durations
            double fd_average = frame_duration_history_.average();
            double adjust = fd_average - min_frame_time;
            // Provide some minimal sleep time for multithreading.
            double min_sleep_time = min_frame_time * 0.01;
            // Adjust sleep time by average of recent frame durations
            double clipped_time = clip(sleep_time - adjust,
                                       min_sleep_time,
                                       min_frame_time);
            // Sleep for that clipped interval.
            thread_sleep_in_seconds(clipped_time);

            //std::cout << std::endl;
            //std::cout << "fd_average     = " << fd_average << std::endl;
            //std::cout << "non_sleep_time = " << non_sleep_time << std::endl;
            //std::cout << "sleep_time     = " << sleep_time << std::endl;
            //std::cout << "clipped_time   = " << clipped_time << std::endl;
        }
    }

    // Measure how much wall clock time has elapsed for this simulation step.
    void measureFrameDuration()
    {
        TimePoint frame_end_time = TimeClock::now();
        frame_duration_ = time_diff_in_seconds(frame_start_time_, frame_end_time);
        frame_counter_ += 1;
        frame_duration_history_.insert(frame_duration_);
        // If frame time is twice the target duration, reset smoothing history.
        if (frame_duration_ > (2 * frameDurationTarget())) { resetHistory(); }
    }
    
    void resetHistory()
    {
        std::cout << "frame_duration_history_.average() = ";
        std::cout << frame_duration_history_.average() << std::endl;
        frame_duration_history_.fill(frameDurationTarget());
    }
    
    double frameDurationTarget() const { return 1.0 / frames_per_second_; }

    int getFPS() const { return frames_per_second_; }
    void setFPS(int fps) { frames_per_second_ = fps; }

private:
    TimePoint frame_start_time_;
    int frame_counter_ = 0;      // Total number of frames so far.
    int frames_per_second_ = 0;
    double frame_duration_ = 0;  // Duration of frame, measured in seconds.
    double sleep_time_ = 0;      // Per frame sleep time, updated each frame.
    RollingAverage<double> frame_duration_history_;  // Last N frame durations.
};


// Measure the execution time of a given "work load" function (of no arguments)
// and an optional suggested repetition count.
double executions_per_second(std::function<void()> work_load, int count = 500000)
{
    Timer timer;
    for (int i = 0; i < count; i++) { work_load(); }
    double executions_per_second = count / timer.elapsedSeconds();
    double seconds_per_execution = 1 / executions_per_second;
    std::cout << "seconds_per_execution = " << seconds_per_execution << std::endl;
    std::cout << "executions_per_second = " << executions_per_second << std::endl;
    return executions_per_second;
}


// Given an indexable "collection" (class C) of objects of type "T" (e.g.
// std::vector<T>), apply a given function of 2 arguments (pair_func(T a, T b))
// to each unique pairwise combination of elements from the collection. Except
// that, through the mysteries of c++ templates, type "T" is never mentioned in
// this definition. "pair_func" has wildcard type "F" to avoid fussy details.)
//
// This use of "&&", "perfect forwarding", and std::forward were all new to me.
// I learned about them here:
// Is it possible to write two template functions as one when the only difference
// is the const-ness of an argument? https://stackoverflow.com/q/30062390/1991373
//
template<class C, typename F>
void apply_to_pairwise_combinations(F pair_func, C&& collection)
{
    auto&& c = std::forward<C>(collection);
    for (int p = 0; p < c.size(); p++)
    {
        for (int q = p + 1; q < c.size(); q++)
        {
            pair_func(c[p], c[q]);
        }
    }
}


static void unit_test()
{
    assert (clip01(1.5) == 1);
    assert (clip01(0.5) == 0.5);
    assert (clip01(-1) == 0);
    assert (clip(0, 1, 5) == 1);
    assert (clip(1.5, 1, 5) == 1.5);
    assert (clip(0, -1, -5) == -1);
    assert (not between(0, 1, 2));
    assert (between(1.5, 1, 2));
    assert (between(1.5, 2, 1));
    assert (between(0, -1, 1));
    assert (not between(-2, 1, -1));
    assert (within_epsilon(1, 1, 0));
    assert (within_epsilon(1.1, 1.2, 0.2));
    assert (within_epsilon(-1.1, -1.2, 0.2));
    assert (not within_epsilon(1.1, 1.2, 0.01));
    assert (rehash32bits(2653567485) == 1574776808);
    
    assert (unit_sigmoid_on_01(0.5) == 0.5);
    assert (within_epsilon(unit_sigmoid_on_01(-1000), 0));
    assert (within_epsilon(unit_sigmoid_on_01(+1000), 1));
    
    assert (remap_interval(1.5, 1, 2, 20, 30) == 25);
    assert (remap_interval(1.5, 2, 1, 30, 20) == 25);
    assert (remap_interval(2, 1, 4, 10, 40) == 20);
    assert (remap_interval_clip(5, 1, 4, 10, 40) == 40);
    assert (remap_interval(1.5, 1, 2, 30, 20) == 25);
    assert (remap_interval(2, 1, 3, 30, 10) == 20);
    assert (remap_interval_clip(5, 1, 4, 40, 10) == 10);
    assert (!std::isnan(remap_interval(1, 1, 1, 2, 3)));
    assert (!std::isnan(remap_interval_clip(1, 1, 1, 2, 3)));

    Blender<double> b;
    b.blend(1.2, 0);
    assert (within_epsilon(b.value, b.global_enable ? 1.2  : 1.2));
    b.blend(3.4, 0.9);
    assert (within_epsilon(b.value, b.global_enable ? 1.42 : 3.4));
    b.blend(5.6, 0.5);
    assert (within_epsilon(b.value, b.global_enable ? 3.51 : 5.6));

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20241116 reality check for time utils
    
//        // Verify times are positive and monotone increasing, as long as they are
//        // larger than some architecture/compiler-dependent threshold/granularity.
//    //    int batch_size = 100000;
//        int batch_size = 100000000;
//        double s_prev = 0;
//        double sum = 0;
//        for (int i = 1; i < 5; i++)
//        {
//            Timer t;
//            for (int b = 1; b < batch_size; b++) { sum += std::log(double(b)); }
//            double s = t.elapsedSeconds();
//            //std::cout << "s = " << s << ", batch_size = " << batch_size << std::endl;
//            //std::cout << "sum = " << sum << std::endl;
//            assert (s >= 0);
//            assert (s >= s_prev);
//            s_prev = s;
//            batch_size *= 10;
//        }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    int pi = 0;
    std::vector<int> pairs = {0,1, 0,2, 0,3, 0,4, 1,2, 1,3, 1,4, 2,3, 2,4, 3,4};
    apply_to_pairwise_combinations([&](int p, int q){assert(p == pairs[pi++]);
                                                     assert(q == pairs[pi++]);},
                                   std::vector<int>({0, 1, 2, 3, 4}));

    std::vector<double> vd({0.1, 2.3, 4.5, 5.6});
    assert (vec_to_string(vd, 4) == "0.1000, 2.3000, 4.5000, 5.6000");
    assert (vec_to_string(vd, 2) == "0.10, 2.30, 4.50, 5.60");

    RollingAverage<int> rai(5);
    for (int i = 0; i < 9; i++) { rai.insert(i); }
    assert (rai.average() == 6);
    RollingAverage<double> rad(100);
    for (int i = 0; i < 413; i++) { rad.insert(i * 1.23); }
    assert (rad.average() == 445.875);

    // TODO 20230409 test random-number utilities, later RandomSequence.
}

}  // end of namespace Utilities
 
namespace util = Utilities;

//------------------------------------------------------------------------------

// TODO 20230302 grabbed this from TexSyn, probably needs a lot of refitting.
// Simple self-contained generator for a sequence of psuedo-random 32 bit values
class RandomSequence
{
public:
    // Constructor with default seed.
    RandomSequence() : state_(defaultSeed()) {}
    // Constructor with given seed.
    RandomSequence(uint64_t seed) : state_(uint32_t(seed)) {}
    // Next random number in sequence as a 31 bit positive int.
    uint32_t nextInt() { return bitMask() & nextUint32(); }
    // Next random number in sequence as a 32 bit unsigned int.
    // explicitly thread safe version of RandomSequence
    uint32_t nextUint32()
    {
        std::lock_guard<std::mutex> grsm(global_rs_mutex_);
        return state_ = util::rehash32bits(state_);
    }
    // A 32 bit word with zero sign bit and all other 31 bits on, max pos int.
    uint32_t bitMask() { return 0x7fffffff; } // 31 bits
    // The largest (31 bit) positive integer that can be returned.
    int maxIntValue() { return bitMask(); }
    // A "large" 32 bit "random" number.
    static uint32_t defaultSeed() { return 688395321; }
    
    // TODO look at removing the old versions of these utilities.
    // Returns a double randomly distributed between 0 and 1
    double frandom01() { return double(nextInt()) / double(maxIntValue()); }
    // Returns a double randomly distributed between lowerBound and upperBound
    double frandom2(double a, double b) { return util::interpolate(frandom01(), a, b); }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240711 some problem for MOF with just one element
    // Returns an int randomly distributed between 0 and n-1.
//    int randomN(int n) { return nextInt() % n; }
//    int randomN(size_t n) { return nextInt() % n; }
    int randomN(int n) { assert(n > 0); return nextInt() % n; }
    int randomN(size_t n) { assert(n > 0); return nextInt() % n; }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // int/double overloads of random2(), returns value between INCLUSIVE bounds.
    int random2(int i, int j) { assert(i<=j); return i + randomN(j - i + 1); }
    double random2(double i, double j) { return frandom2(i, j); }
    bool randomBool() { return randomBool(0.5); }
    bool randomBool(double likelihood)
    { assert(util::between(likelihood,0,1)); return frandom01() <= likelihood; }
    // Return random element of given std::vector.
    template<typename T> T randomSelectElement(const std::vector<T>& collection)
    { return collection.at(randomN(collection.size())); }

    // These are defined in Vec3.h:
    Vec3 randomUnitVector();
    Vec3 randomPointInUnitRadiusSphere();
    Vec3 randomPointInAxisAlignedBox(Vec3 a, Vec3 b);
    // Names to match Python code.
    Vec3 random_unit_vector();
    Vec3 random_point_in_unit_radius_sphere();
    Vec3 random_point_in_axis_aligned_box(Vec3 a, Vec3 b);

    // Set seed (RS state) to given value, or defaultSeed() if none given.
    void setSeed() { setSeed(defaultSeed()); }
    void setSeed(uint32_t seed)
    {
        std::lock_guard<std::mutex> grsm(global_rs_mutex_);
        state_ = seed;
    }
    // Get state.
    uint32_t getSeed() { return state_; }
private:
    uint32_t state_;
    static inline std::mutex global_rs_mutex_;
};

//------------------------------------------------------------------------------

// Use global mutex to allow synchronizing console output from parallel threads.
// (Written as a macro since the lock_guard is released at the end of a block.)
// (..._evoflock suffix since collides with LP macro. Macro names are global.)
#define grabPrintLock_evoflock() \
std::lock_guard<std::recursive_mutex> pl_(util::DebugPrint::getPrintMutex());

// For debugging: prints one line with a given C expression, an equals sign,
// and the value of the expression.  For example "std::sin(angle) = 35.6"
#define debugPrint(e){ grabPrintLock_evoflock(); \
                       std::cout << #e" = " << (e) << std::endl << std::flush; }

// EF::RS() is the single, shared, thread-safe, settable, RandomSequence object.
namespace EvoFlock
{
inline static RandomSequence rs_default_;
inline static RandomSequence* rs_ = &rs_default_;
inline void setRS(RandomSequence& rs) { rs_ = &rs; }
inline RandomSequence& RS(){ return *rs_; }
inline static bool enable_multithreading = true;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20250510 temp global switch for controlling speed with fitness.
inline static bool fitness_speed_control = false;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}  // end of namespace EvoFlock
namespace EF = EvoFlock;


// Square a double
inline double sq(double f) { return f * f; }
