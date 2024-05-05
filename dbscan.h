//------------------------------------------------------------------------------
//
//  dbscan.h -- new flock experiments
//
//  Clustering algorithm DBSCAN (density-based spatial clustering of applications
//  with noise). It was published in 1996 by Martin Ester, Hans-Peter Kriegel,
//  Jörg Sander and Xiaowei Xu.
//
//  For more detail see: https://en.wikipedia.org/wiki/DBSCAN
//
//  This is a lightly modified version of: https://github.com/james-yoo/DBSCAN
//  (1) merged Yoo's dbscan.cpp into dbscan.h
//  (2) call run() inside the constructor.
//  (3) made cluster count accessible.
//  (4) initialize Point.clusterID to UNCLASSIFIED.
//  (5) rename calculateDistance() to distanceSquared(), compare to squared eps.
//  (6) changed #define constants to be const static members of class.
//  (7) modernize C++ usage: remove explicit iterators.
//  (8) don't copy output vector from calculateCluster().
//
//  Created by Craig Reynolds on April 29, 2024.
//  MIT License -- Copyright © 2024 Craig Reynolds
//
//------------------------------------------------------------------------------

#pragma once
#include <vector>
#include <cmath>

class DBSCAN {
    
public:
    
    typedef struct Point_
    {
        int clusterID = UNCLASSIFIED;  // clustered ID
        float x;                       // position
        float y;
        float z;
    } Point;

    DBSCAN(unsigned int minPts, float eps, const std::vector<Point>& points)
    {
        m_epsilon = eps;
        m_points = points;
        m_minPoints = minPts;
        m_epsilon_squared = eps * eps;
        m_pointSize = int(points.size());
        run();
    }

    ~DBSCAN(){}

    int run()
    {
        int clusterID = 1;
        std::vector<Point>::iterator iter;
        for(iter = m_points.begin(); iter != m_points.end(); ++iter)
        {
            if (iter->clusterID == UNCLASSIFIED)
            {
                if (expandCluster(*iter, clusterID) != FAILURE)
                {
                    clusterID += 1;
                }
            }
            m_cluster_count = std::max(m_cluster_count, iter->clusterID);
        }
        return 0;
    }
    
    int expandCluster(Point point, int clusterID)
    {
        std::vector<int> clusterSeeds;
        calculateCluster(point, clusterSeeds);
        if (clusterSeeds.size() < m_minPoints)
        {
            point.clusterID = NOISE;
            return FAILURE;
        }
        else
        {
            int indexCorePoint = 0;
            for (int csi = 0; csi < clusterSeeds.size(); csi++)
            {
                Point& cp = m_points.at(clusterSeeds.at(csi));
                cp.clusterID = clusterID;
                if (coincident(cp, point)) { indexCorePoint = csi; }
            }
            clusterSeeds.erase(clusterSeeds.begin() + indexCorePoint);
            for (size_t i = 0; i < clusterSeeds.size(); ++i)
            {
                std::vector<int> clusterNeighors;
                calculateCluster(m_points.at(clusterSeeds[i]), clusterNeighors);
                if (clusterNeighors.size() >= m_minPoints)
                {
                    for (int cn_index : clusterNeighors)
                    {
                        int& cn_cid = m_points.at(cn_index).clusterID;
                        if (cn_cid == UNCLASSIFIED || cn_cid == NOISE)
                        {
                            if (cn_cid == UNCLASSIFIED)
                            {
                                clusterSeeds.push_back(cn_index);
                            }
                            cn_cid = clusterID;
                        }
                    }
                }
            }
            return SUCCESS;
        }
    }
    
    void calculateCluster(const Point& point, std::vector<int>& output_points)
    {
        output_points.clear();
        for (int i = 0; i < m_points.size(); i++)
        {
            double d2 = distanceSquared(point, m_points.at(i));
            if (d2 <= m_epsilon_squared) { output_points.push_back(i); }
        }
    }

    static double distanceSquared(const Point& a, const Point& b)
    {
        return (pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
    }

    static bool coincident(const Point& a, const Point& b)
    {
        return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
    }
    
    int getTotalPointSize() {return m_pointSize;}
    int getMinimumClusterSize() {return m_minPoints;}
    int getEpsilonSize() {return m_epsilon;}
    int getClusterCount() const { return m_cluster_count; }

    const static int UNCLASSIFIED = -1;
    const static int NOISE = -2;
    const static int SUCCESS = 0;
    const static int FAILURE = -3;

private:
    std::vector<Point> m_points;
    unsigned int m_pointSize;
    unsigned int m_minPoints;
    float m_epsilon;
    float m_epsilon_squared = 0;
    int m_cluster_count = 0;
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240505 refactor old-fashioned c++

public:
    
//#include <stdio.h>
//#include <iostream>
//#include "dbscan.h"
    
#define MINIMUM_POINTS 4     // minimum number of cluster
//#define EPSILON (0.75*0.75)  // distance for clustering, metre^2
#define EPSILON (0.75)  // distance for clustering, metre^2

    static void readBenchmarkData(std::vector<Point>& points)
    {
        // load point cloud
        FILE *stream;
//        stream = fopen ("benchmark_hepta.dat","ra");
        stream = fopen ("/Users/cwr/Desktop/benchmark_hepta.dat.txt","ra");

        
//        unsigned int minpts, num_points, cluster, i = 0;
//        double epsilon;
        unsigned int num_points;
        unsigned int cluster;
        unsigned int i = 0;

        fscanf(stream, "%u\n", &num_points);
        
        Point *p = (Point *)calloc(num_points, sizeof(Point));
        
        while (i < num_points)
        {
            fscanf(stream, "%f,%f,%f,%d\n", &(p[i].x), &(p[i].y), &(p[i].z), &cluster);
            p[i].clusterID = UNCLASSIFIED;
            points.push_back(p[i]);
            ++i;
        }
        
        free(p);
        fclose(stream);
    }
    
    static void printResults(std::vector<Point>& points, int num_points)
    {
        int i = 0;
        printf("Number of points: %u\n"
               " x     y     z     cluster_id\n"
               "-----------------------------\n"
               , num_points);
        while (i < num_points)
        {
            printf("%5.2lf %5.2lf %5.2lf: %d\n",
                   points[i].x,
                   points[i].y, points[i].z,
                   points[i].clusterID);
            ++i;
        }
    }
    
    static int test()
    {
        std::vector<Point> points;
        
        // read point data
        readBenchmarkData(points);
        
        // constructor
        DBSCAN ds(MINIMUM_POINTS, EPSILON, points);
        
        // main loop
        ds.run();
        
        // result of DBSCAN algorithm
        printResults(ds.m_points, ds.getTotalPointSize());
        
        
        debugPrint(ds.getClusterCount())

        
        return 0;
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
};
