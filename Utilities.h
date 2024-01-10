//------------------------------------------------------------------------------
//
//  Utilities.h -- new flock experiments
//
//  Utility functions
//
//  Defines a few symbols in global namespace (RandomSequence, debugPrint(), sq()) but otherwise
//  defines utility names inside the "Utilities" which has an alias "util".
//
//  Created by Craig Reynolds on January 8, 2024.
//  (Based on earlier C++ and Python versions.)
//  MIT License -- Copyright © 2023 Craig Reynolds
//
//------------------------------------------------------------------------------

#pragma once
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <cassert>
#include <mutex>
class Vec3;

namespace Utilities 
{

// Generic interpolation
template<typename F,typename T>
T interpolate(const F& alpha, const T& x0, const T& x1)
{
    return (x0 * (1 - alpha)) + (x1 * alpha);
}

// Constrain a given value "x" to be between two bounds: "bound0" and "bound1"
// (without regard to order). Returns x if it is between the bounds, otherwise
// returns the nearer bound.
inline float clip(float x, float bound0, float bound1)
{
    float clipped = x;
    float min = std::min(bound0, bound1);
    float max = std::max(bound0, bound1);
    if (clipped < min) clipped = min;
    if (clipped > max) clipped = max;
    return clipped;
}

inline float clip01 (const float x)
{
    return clip(x, 0, 1);
}

//    // True when x is between given bounds (low ≤ x ≤ high)
//    inline bool between(float x, float low, float high)
//    {
//        return (low <= x) && (x <= high);
//    }

//    # True when x is between given bounds.
//    def between(x, a, b):
//        return (min(a, b) <= x) and (x <= max(a, b))

// True when x is between given bounds (low ≤ x ≤ high)
inline bool between(float x, float a, float b)
{
    return (std::min(a, b) <= x) && (x <= std::max(a, b));
}

//    # Takes a 32 bit value and shuffles it around to produce a new 32 bit value.
//    # "Robert Jenkins' 32 bit integer hash function" from "Integer Hash Function"
//    # (1997) by Thomas Wang (https://gist.github.com/badboy/6267743)
//    # Fiddled to make it work like 32 bit in Python.
//    def rehash32bits(int32):
//        ones = 0xffffffff  # 32 bits of all ones.
//        int32 = ones & int32
//        int32 = ones & ((int32 + 0x7ed55d16) + (int32 << 12))
//        int32 = ones & ((int32 ^ 0xc761c23c) ^ (int32 >> 19))
//        int32 = ones & ((int32 + 0x165667b1) + (int32 <<  5))
//        int32 = ones & ((int32 + 0xd3a2646c) ^ (int32 <<  9))
//        int32 = ones & ((int32 + 0xfd7046c5) + (int32 <<  3))
//        int32 = ones & ((int32 ^ 0xb55a4f09) ^ (int32 >> 16))
//        return int32

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



//    # Taken from https://en.wikipedia.org/wiki/Logistic_function
//    #def logistic(x, k, L, x0):
//    #    return L / (1 + math.exp(-k * (x - x0)))
//    # TODO 20230910 TEMP FOR DEBUGGING
//    def logistic(x, k, L, x0):
//        x = max(x, -50)
//        return L / (1 + math.exp(-k * (x - x0)))
//
//    # Logistic sigmoid (s-curve) from ~(0,0) to ~(1,1), ~0 if x<0, ~1 if x>1
//    # (See a plot of this function via Wolfram|Alpha: https://bit.ly/3sUYbeJ)
//    def unit_sigmoid_on_01(x):
//        return logistic(x, 12, 1, 0.5)


// Taken from https://en.wikipedia.org/wiki/Logistic_function
float logistic(float x, float k, float L, float x0)
{
    x = std::max(x, -50.0f);  // TODO wait, what? some overflow issue?
    return L / (1 + std::exp(-k * (x - x0)));
}
// Logistic sigmoid (s-curve) from ~(0,0) to ~(1,1), ~0 if x<0, ~1 if x>1
// (See a plot of this function via Wolfram|Alpha: https://bit.ly/3sUYbeJ)
float unit_sigmoid_on_01(float x) { return logistic(x, 12, 1, 0.5);}









//    # Remap a value specified relative to a pair of bounding values
//    # to the corresponding value relative to another pair of bounds.
//    # Inspired by (dyna:remap-interval y y0 y1 z0 z1) circa 1984.
//    # (20220108 borrowed from TexSyn's c++ Utilities package)
//    # (20230910 borrowed from PredatorEye's DiskFind.py)
//    # TODO -- note similar API in numpy
//    def remap_interval(x, in0, in1, out0, out1):
//        return interpolate((x - in0) / (in1 - in0), out0, out1)

//    // Remap a value specified relative to a pair of bounding values
//    // to the corresponding value relative to another pair of bounds.
//    // Inspired by (dyna:remap-interval y y0 y1 z0 z1) circa 1984.
//    inline float remap_interval(float x,
//                                float in0, float in1,
//                                float out0, float out1)
//    {
//        // Remap if input range is nonzero, otherwise blend them evenly.
//        float input_range = in1 - in0;
//
//    //    float blend = ((input_range > 0) ? ((x - in0) / input_range) : 0.5);
//
//        float blend = ((input_range == 0) ? 0.5 : ((x - in0) / input_range));
//    //    blend = 0.5 if input_range == 0 else ((x - in0) / input_range)
//
//        return interpolate(blend, out0, out1);
//    }

// Remap a value specified relative to a pair of bounding values
// to the corresponding value relative to another pair of bounds.
// Inspired by (dyna:remap-interval y y0 y1 z0 z1) circa 1984.
inline float remap_interval(float x,
                            float in0, float in1,
                            float out0, float out1)
{
    // Remap if input range is nonzero, otherwise blend them evenly.
    float input_range = in1 - in0;
    float blend = (input_range == 0) ? 0.5 : ((x - in0) / input_range);
    return interpolate(blend, out0, out1);
}


//
//    # Like remapInterval but the result is clipped to remain between out0 and out1
//    # (20220108 borrowed from TexSyn's c++ Utilities package)
//    # (20230910 borrowed from PredatorEye's DiskFind.py)
//    def remap_interval_clip(x, in0, in1, out0, out1):
//        return clip(remap_interval(x, in0, in1, out0, out1), out0, out1)

// Like remapInterval but the result is clipped to remain between out0 and out1
inline float remap_interval_clip(float x,
                                 float in0, float in1,
                                 float out0, float out1)
{
    return clip(remap_interval(x, in0, in1, out0, out1), out0, out1);
}

// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ QQQ

//
//
//    # Object to collect pairs, then to look up an item's counterpart. Used here to
//    # associate instances of Flock and Visualizer (can't subclass: Open3D bug #572).
//    class Pairings:
//        def __init__(self):
//            self.dict = {}
//        def add_pair(self, a, b):
//            self.dict[a] = b
//            self.dict[b] = a
//        def get_peer(self, x):
//            return self.dict[x]





//    # Utility for blending per-step values into accumulators for low pass filtering.
//    class Blender:
//        def __init__(self, initial_value=None):
//            self.value = initial_value
//        # "smoothness" controls how much smoothing. Values around 0.8-0.9 seem most
//        # useful. smoothness=1 is infinite smoothing. smoothness=0 is no smoothing.
//        def blend(self, new_value, smoothness):
//            self.value = (new_value if self.value == None
//                          else interpolate(smoothness, new_value, self.value))
//            return self.value

// Utility for blending per-step values into accumulators for low pass filtering.
// TODO 20230110 for initial Python to C++ translation I'll assume this is for
//               floats, but it needs to be templated for any type.
class Blender
{
public:
    Blender(float initial_value = none)
    {
        value = initial_value;
    }
    // "smoothness" controls how much smoothing. Values around 0.8-0.9 seem most
    // useful. smoothness=1 is infinite smoothing. smoothness=0 is no smoothing.
    float blend(float new_value, float smoothness)
    {
        value = ((value == none) ?
                 new_value :
                 interpolate(smoothness, new_value, value));
        return value;
    }
    inline static const float none = std::numeric_limits<float>::infinity();
    float value;
};




//    # This value works on my laptop with Python 3.10
//    epsilon = 0.00000000000001
//
//    # True when a and b differ by no more than epsilon.
//    def within_epsilon(a, b, e=epsilon):
//        return abs(a - b) <= e

//    // This value works on my laptop with Python 3.10
//    //double epsilon = 0.00000000000001;
//    // TODO 20240107 0.00000000000001 was not working in C++, must return to this!!!
//    //               might this be a float vs double issue?
//    double epsilon = 0.000001;
//
//    // True when a and b differ by no more than epsilon.
//    bool within_epsilon(double a, double b, double e=epsilon)
//    {
//        return std::abs(a - b) <= e;
//    }

// This value works on my laptop with Python 3.10
//double epsilon = 0.00000000000001;
// TODO 20240107 0.00000000000001 was not working in C++, must return to this!!!
//               might this be a float vs double issue?
double epsilon = 0.000001;

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



//    @staticmethod
//    def unit_test():
//        assert clip01(1.5) == 1
//        assert clip01(0.5) == 0.5
//        assert clip01(-1) == 0
//        assert clip(0, 1, 5) == 1
//        assert clip(1.5, 1, 5) == 1.5
//        assert clip(0, -1, -5) == -1
//        assert between(0, 1, 2) == False
//        assert between(1.5, 1, 2) == True
//        assert between(1.5, 2, 1) == True
//        assert between(0, -1, 1) == True
//        assert between(-2, 1, -1) == False
//        assert within_epsilon(1, 1, 0)
//        assert within_epsilon(1.1, 1.2, 0.2)
//        assert within_epsilon(-1.1, -1.2, 0.2)
//        assert not within_epsilon(1.1, 1.2, 0.01)
//        assert rehash32bits(2653567485) == 1574776808
//
//        p = Pairings()
//        p.add_pair(1.23, 'a')
//        p.add_pair('foo', (1,2))
//        assert p.get_peer(1.23) == 'a'
//        assert p.get_peer('a') == 1.23
//        assert p.get_peer('foo') == (1,2)
//        assert p.get_peer((1,2)) == 'foo'
//
//        b = Blender()
//        assert b.value == None
//        b.blend(1.2, 'ignored')
//        assert b.value == 1.2
//        b.blend(3.4, 0.9)
//        assert b.value == 1.42
//        b.blend(5.6, 0.5)
//        assert b.value == 3.51
//
//        # TODO 20230409 test random-number utilities, later RandomSequence.

static void unit_test()
{
    assert (clip01(1.5) == 1);
    assert (clip01(0.5) == 0.5);
    assert (clip01(-1) == 0);
    assert (clip(0, 1, 5) == 1);
    assert (clip(1.5, 1, 5) == 1.5);
    assert (clip(0, -1, -5) == -1);
//    assert (between(0, 1, 2) == false);
//    assert (between(1.5, 1, 2) == true);
//    assert (between(1.5, 2, 1) == true);
//    assert (between(0, -1, 1) == true);
//    assert (between(-2, 1, -1) == false);
    assert (not between(0, 1, 2));
    assert (between(1.5, 1, 2));
    assert (between(1.5, 2, 1));
    assert (between(0, -1, 1));
    assert (not between(-2, 1, -1));
//    assert (within_epsilon(1, 1, 0));
//    assert (within_epsilon(1.1, 1.2, 0.2));
//    assert (within_epsilon(-1.1, -1.2, 0.2));
//    assert (not within_epsilon(1.1, 1.2, 0.01));
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


    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ QQQ


//    p = Pairings()
//    p.add_pair(1.23, 'a')
//    p.add_pair('foo', (1,2))
//    assert (p.get_peer(1.23) == 'a'
//    assert (p.get_peer('a') == 1.23
//    assert (p.get_peer('foo') == (1,2)
//    assert (p.get_peer((1,2)) == 'foo'
    
//    b = Blender()
    Blender b;
    assert (b.value == Blender::none);
    b.blend(1.2, Blender::none);
//    std::cout << "b.value = " << b.value << std::endl;
//    assert (b.value == 1.2);
    assert (within_epsilon(b.value, 1.2));
    b.blend(3.4, 0.9);
//    assert (b.value == 1.42);
    assert (within_epsilon(b.value, 1.42));
    b.blend(5.6, 0.5);
//    assert (b.value == 3.51);
    assert (within_epsilon(b.value, 3.51));

    // TODO 20230409 test random-number utilities, later RandomSequence.
}

}  // end of namespace Utilities
 
namespace util = Utilities;


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ QQQ


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
    uint32_t nextUint32() { return state_ = util::rehash32bits(state_); }
    // A 32 bit word with zero sign bit and all other 31 bits on, max pos int.
    uint32_t bitMask() { return 0x7fffffff; } // 31 bits
    // The largest (31 bit) positive integer that can be returned.
    int maxIntValue() { return bitMask(); }
    // A "large" 32 bit "random" number.
    static uint32_t defaultSeed() { return 688395321; }
    
    // TODO look at removing the old versions of these utilities.
    // Returns a float randomly distributed between 0 and 1
    float frandom01() { return float(nextInt()) / float(maxIntValue()); }
    // Returns a float randomly distributed between lowerBound and upperBound
//    float frandom2(float a, float b) { return interpolate(frandom01(), a, b); }
    float frandom2(float a, float b) { return util::interpolate(frandom01(), a, b); }
    // Returns an int randomly distributed between 0 and n-1.
    int randomN(int n) { return nextInt() % n; }
    int randomN(size_t n) { return nextInt() % n; }
    // int/float overloads of random2(), returns value between INCLUSIVE bounds.
    int random2(int i, int j) { assert(i<=j); return i + randomN(j - i + 1); }
    float random2(float i, float j) { return frandom2(i, j); }
    // Returns true or false with equal likelihood.
    bool randomBool() { return random2(0, 1); }
    // Return random element of given std::vector.
    template<typename T> T randomSelectElement(const std::vector<T>& collection)
    { return collection.at(randomN(collection.size())); }
    
    //    // TODO these duplicate the function of the same name in global namespace.
    //    //  Maybe those should be replaced by defining a global RandomSequence which
    //    // must be specifically written in source code. This may help avoid the
    //    // "attractive nuisance" of random utilities which are non-repeatable.
    //    Vec2 randomPointInUnitDiameterCircle();
    //    Vec2 randomUnitVector();
    //    // Random point (position vector) in an axis aligned rectangle defined by
    //    // two diagonally opposite vertices.
    //    Vec2 randomPointInAxisAlignedRectangle(Vec2 a, Vec2 b);
    //    // TODO moved from Color class to here on June 30, 2020:
    //    Color randomUnitRGB();
    
    Vec3 randomUnitVector();
    Vec3 randomPointInUnitRadiusSphere();
    Vec3 randomPointInAxisAlignedBox(Vec3 a, Vec3 b);
    // Names to match Python code.
    Vec3 random_unit_vector();
    Vec3 random_point_in_unit_radius_sphere();
    Vec3 random_point_in_axis_aligned_box(Vec3 a, Vec3 b);

    // Set seed (RS state) to given value, or defaultSeed() if none given.
    void setSeed() { state_ = defaultSeed(); }
    void setSeed(uint32_t seed) { state_ = seed; }
    // Get state.
    uint32_t getSeed() { return state_; }
private:
    uint32_t state_;
};

// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ QQQ



//------------------------------------------------------------------------------

//    // For debugging: prints one line with a given C expression, an equals sign,
//    // and the value of the expression.  For example "angle = 35.6"
//    #define debugPrint(e)                                                          \
//    {                                                                              \
//    std::lock_guard<std::recursive_mutex> pl_(util::DebugPrint::getPrintMutex());  \
//    std::cout << #e" = " << (e) << std::endl << std::flush;                        \
//    }

// For debugging: prints one line with a given C expression, an equals sign,
// and the value of the expression.  For example "angle = 35.6"
#define debugPrint(e)                                                          \
{ std::lock_guard<std::recursive_mutex> pl_(util::DebugPrint::getPrintMutex());\
  std::cout << #e" = " << (e) << std::endl << std::flush; }



// Square a float
inline float sq(float f) { return f * f; }

//------------------------------------------------------------------------------


//------------------------------------------------------------------------------

// Short names
typedef std::chrono::high_resolution_clock TimeClock;
typedef std::chrono::time_point<TimeClock> TimePoint;
typedef std::chrono::duration<double> TimeDuration;

// TimeDuration to seconds as float.
inline float time_duration_in_seconds(TimeDuration time_duration)
{
    return time_duration.count();
}

// TimePoint difference in seconds.
inline float time_diff_in_seconds(TimePoint start, TimePoint end)
{
    TimeDuration dt = end - start;
    return time_duration_in_seconds(dt);
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
            std::cout << description_ << " elapsed time: "
            << elapsedSeconds() << " seconds" << std::endl;
        }
    }
    float elapsedSeconds() const
    {
        return time_diff_in_seconds(start_time_, TimeClock::now());
    }
private:
    std::string description_;
    TimePoint start_time_;
};

//------------------------------------------------------------------------------

// Measure the execution time of a given "work load" function (of no arguments)
// and an optional suggested repetition count.
float executions_per_second(std::function<void()> work_load, int count = 500000)
{
    Timer timer;
    for (int i = 0; i < count; i++) { work_load(); }
    float executions_per_second = count / timer.elapsedSeconds();
    float seconds_per_execution = 1 / executions_per_second;
    debugPrint(seconds_per_execution)
    debugPrint(executions_per_second)
    return executions_per_second;
}
