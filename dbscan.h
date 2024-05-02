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

#include <functional>

// I started to make this a template on the type of “point like” user data. I
// ran into mysterious type problems with the distance_function, so took out the
// template aspect for now, making it specifically for Boids.
//template<typename T>
//class dbscan

#include "Boid.h"

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define using_james_yoo_dbscan
#ifdef using_james_yoo_dbscan
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

//using namespace std;

typedef struct Point_
{
    float x, y, z;  // X, Y, Z position
    int clusterID;  // clustered ID
}Point;

class DBSCAN {
public:
    //    DBSCAN(unsigned int minPts, float eps, vector<Point> points){
    DBSCAN(unsigned int minPts, float eps, std::vector<Point> points){
        m_minPoints = minPts;
        m_epsilon = eps;
        m_points = points;
        //        m_pointSize = points.size();
        m_pointSize = int(points.size());
    }
    
    
    // TODO 20240501 add temp interface to BoidPtrList
    DBSCAN(unsigned int minPts, float eps, const BoidPtrList& boids)
    {
        m_minPoints = minPts;
        m_epsilon = eps;
        m_pointSize = int(boids.size());
        for (Boid* b : boids)
        {
            Point_ point;
            point.x = b->position().x();
            point.y = b->position().y();
            point.z = b->position().z();
            point.clusterID = UNCLASSIFIED;
            //            debugPrint(point.x)
            //            debugPrint(point.y)
            //            debugPrint(point.z)
            m_points.push_back(point);
        }
    }
    
    
    ~DBSCAN(){}
    
    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    
    //    int run();
    //    vector<int> calculateCluster(Point point);
    //    int expandCluster(Point point, int clusterID);
    //    inline double calculateDistance(const Point& pointCore, const Point& pointTarget);
    
    
    //    #include "dbscan.h"
    
    int run()
    {
        int clusterID = 1;
        //        vector<Point>::iterator iter;
        std::vector<Point>::iterator iter;
        for(iter = m_points.begin(); iter != m_points.end(); ++iter)
        {
            if ( iter->clusterID == UNCLASSIFIED )
            {
                if ( expandCluster(*iter, clusterID) != FAILURE )
                {
                    clusterID += 1;
                }
            }
        }
        
        // TODO 20240501 add public cluster count
        m_cluster_count = clusterID;
        
        return 0;
    }
    
    int expandCluster(Point point, int clusterID)
    {
        //        vector<int> clusterSeeds = calculateCluster(point);
        std::vector<int> clusterSeeds = calculateCluster(point);
        
        if ( clusterSeeds.size() < m_minPoints )
        {
            point.clusterID = NOISE;
            return FAILURE;
        }
        else
        {
            int index = 0, indexCorePoint = 0;
            //            vector<int>::iterator iterSeeds;
            std::vector<int>::iterator iterSeeds;
            for( iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
            {
                m_points.at(*iterSeeds).clusterID = clusterID;
                if (m_points.at(*iterSeeds).x == point.x && m_points.at(*iterSeeds).y == point.y && m_points.at(*iterSeeds).z == point.z )
                {
                    indexCorePoint = index;
                }
                ++index;
            }
            clusterSeeds.erase(clusterSeeds.begin()+indexCorePoint);
            
            //            for( vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i )
            for( std::vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i )
            {
                //                vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));
                std::vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));
                
                if ( clusterNeighors.size() >= m_minPoints )
                {
                    //                    vector<int>::iterator iterNeighors;
                    std::vector<int>::iterator iterNeighors;
                    for ( iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors )
                    {
                        if ( m_points.at(*iterNeighors).clusterID == UNCLASSIFIED || m_points.at(*iterNeighors).clusterID == NOISE )
                        {
                            if ( m_points.at(*iterNeighors).clusterID == UNCLASSIFIED )
                            {
                                clusterSeeds.push_back(*iterNeighors);
                                n = clusterSeeds.size();
                            }
                            m_points.at(*iterNeighors).clusterID = clusterID;
                        }
                    }
                }
            }
            
            return SUCCESS;
        }
    }
    
    //    vector<int> calculateCluster(Point point)
    //    {
    //        int index = 0;
    //        vector<Point>::iterator iter;
    //        vector<int> clusterIndex;
    std::vector<int> calculateCluster(Point point)
    {
        int index = 0;
        std::vector<Point>::iterator iter;
        std::vector<int> clusterIndex;
        for( iter = m_points.begin(); iter != m_points.end(); ++iter)
        {
            if ( calculateDistance(point, *iter) <= m_epsilon )
            {
                clusterIndex.push_back(index);
            }
            index++;
        }
        return clusterIndex;
    }
    
    inline double calculateDistance(const Point& pointCore, const Point& pointTarget )
    {
        return pow(pointCore.x - pointTarget.x,2)+pow(pointCore.y - pointTarget.y,2)+pow(pointCore.z - pointTarget.z,2);
    }
    
    
    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    
    int getTotalPointSize() {return m_pointSize;}
    int getMinimumClusterSize() {return m_minPoints;}
    int getEpsilonSize() {return m_epsilon;}
    
    // TODO 20240501 add accessible cluster count
    int get_cluster_count() const { return m_cluster_count + 1; }

    
public:
    //    vector<Point> m_points;
    std::vector<Point> m_points;
    
    
private:
    unsigned int m_pointSize;
    unsigned int m_minPoints;
    float m_epsilon;
    
    // TODO 20240501 add accessible cluster count
    int m_cluster_count = 0;
};

#endif // DBSCAN_H


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#else  // using_james_yoo_dbscan
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


class dbscan
{
public:
    dbscan(const std::vector<Boid*>& user_points,
           std::function<double(Boid*, Boid*)> distance_function,
           double epsilon,
           int min_points)
    {
        distance_function_ = distance_function;
        epsilon_ = epsilon;
        min_points_ = min_points;
        
        size_t s = user_points.size();
        points_.resize(s);
        for (int i = 0; i < s; i++) { points_[i].user_point = user_points[i]; }
        cluster();
        
//        for (auto& up : user_points) {debugPrint(up->position());}
    }
    
    
    class Point
    {
    public:
//        T* user_point = nullptr;
        Boid* user_point = nullptr;
        int label = undefined_;
    };
    
    

    
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

    
    void cluster()
    {
        cluster_count_ = 0;
        for (auto& point : points_)
        {
//            if (point.label != undefined_)
            if (point.label == undefined_)
            {
                std::vector<Point*> neighbors = range_query(point);
                if (neighbors.size() < min_points_)
                {
                    point.label = noise_;
                }
                else
                {
                    cluster_count_++;
                    point.label = cluster_count_;
                    
                    // no I think this is wrong, S is a temp copy of neighbors.
                    // SeedSet S := N \ {P}          /* Neighbors to expand */
                    // neighbors.push_back(&point);  // MAYBE????
                    
                    std::vector<Point*> seed_set = neighbors;
                    seed_set.push_back(&point);
                    debugPrint(util::vec_to_string(seed_set))

//                    for (auto q : neighbors)
//                    for (auto q : seed_set)
                    for (int ss_index = 0; ss_index < seed_set.size(); ss_index++)
                    {
                        Point* q = seed_set.at(ss_index);
                        debugPrint(q)
                        assert(q != nullptr);
                        
                        if (q->label == noise_) { q->label = cluster_count_; }
                        if (q->label == undefined_)
                        {
                            q->label = cluster_count_;
                            
                            neighbors = range_query(*q);
                            
                            if (neighbors.size() >= min_points_)
                            {
                                // S := S ∪ N /* Add new neighbors to seed set */
                                
                                for (auto n : neighbors)
                                {
                                    debugPrint(n)
                                    seed_set.push_back(n);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    
    // RangeQuery(DB, distFunc, P, eps)
    
    //    RangeQuery(DB, distFunc, Q, eps) {
    //        Neighbors N := empty list
    //        for each point P in database DB {                      /* Scan all points in the database */
    //            if distFunc(Q, P) ≤ eps then {                     /* Compute distance and check epsilon */
    //                N := N ∪ {P}                                   /* Add to result */
    //            }
    //        }
    //        return N
    //    }
    
    std::vector<Point*> range_query(const Point& query_point)
    {
        std::vector<Point*> result;
        for (auto& p : points_)
        {
            double d = distance_function_(p.user_point, query_point.user_point);
            if (d < epsilon_) { result.push_back(&p); }
        }
        
//        debugPrint(util::vec_to_string(result))
        
        return result;
    }

    int get_cluster_count() const { return cluster_count_; }
    
    // TODO still unsure how to structure this
    static void unit_test()
    {
    }

    const static int undefined_ = -1;
    const static int noise_ = -2;
    
private:
    std::vector<Point> points_;
    
    std::function<double(Boid*, Boid*)> distance_function_;

    double epsilon_ = 0;
    int min_points_ = 0;

    int cluster_count_ = 0;

};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#endif  // using_james_yoo_dbscan
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
