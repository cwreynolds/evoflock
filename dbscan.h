//------------------------------------------------------------------------------
//
//  dbscan.h -- new flock experiments
//
//  Implement clustering algorithm DBSCAN (density-based spatial clustering of
//  applications with noise). It was published in 1996 by Martin Ester, Hans-
//  Peter Kriegel, Jörg Sander and Xiaowei Xu.
//
//  For more detail see: https://en.wikipedia.org/wiki/DBSCAN
//
//  Created by Craig Reynolds on April 29, 2024.
//  MIT License -- Copyright © 2024 Craig Reynolds
//
//------------------------------------------------------------------------------

#pragma once
#include "Vec3.h"


//DBSCAN(DB, distFunc, eps, minPts) {
//    C := 0                                                  /* Cluster counter */
//    for each point P in database DB {
//        if label(P) ≠ undefined then continue               /* Previously processed in inner loop */
//        Neighbors N := RangeQuery(DB, distFunc, P, eps)     /* Find neighbors */
//        if |N| < minPts then {                              /* Density check */
//            label(P) := Noise                               /* Label as Noise */
//            continue
//        }
//        C := C + 1                                          /* next cluster label */
//        label(P) := C                                       /* Label initial point */
//        SeedSet S := N \ {P}                                /* Neighbors to expand */
//        for each point Q in S {                             /* Process every seed point Q */
//            if label(Q) = Noise then label(Q) := C          /* Change Noise to border point */
//            if label(Q) ≠ undefined then continue           /* Previously processed (e.g., border point) */
//            label(Q) := C                                   /* Label neighbor */
//            Neighbors N := RangeQuery(DB, distFunc, Q, eps) /* Find neighbors */
//            if |N| ≥ minPts then {                          /* Density check (if Q is a core point) */
//                S := S ∪ N                                  /* Add new neighbors to seed set */
//            }
//        }
//    }
//}


template<typename T>
class dbscan
{
public:
    dbscan(const std::vector<T*>& user_points)
    {
        size_t s = user_points.size();
        points.resize(s);
        for (int i = 0; i < s; i++) { points[i].user_point = user_points[i]; }
    }

    class Point
    {
    public:
        T* user_point = nullptr;
        int label = undefined;
    };
    
    // TODO still unsure how to structure this
    static void unit_test()
    {
    }

    const static int undefined = -1;
    const static int noise = -2;
    
private:
    std::vector<Point> points;
};
