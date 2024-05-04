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
        }
        m_cluster_count = clusterID;
        return 0;
    }
    
    int expandCluster(Point point, int clusterID)
    {
        std::vector<int> clusterSeeds = calculateCluster(point);
        if (clusterSeeds.size() < m_minPoints)
        {
            point.clusterID = NOISE;
            return FAILURE;
        }
        else
        {
            int index = 0, indexCorePoint = 0;
            std::vector<int>::iterator iterSeeds;
            for(iterSeeds = clusterSeeds.begin();
                iterSeeds != clusterSeeds.end();
                ++iterSeeds)
            {
                m_points.at(*iterSeeds).clusterID = clusterID;
                if (m_points.at(*iterSeeds).x == point.x &&
                    m_points.at(*iterSeeds).y == point.y &&
                    m_points.at(*iterSeeds).z == point.z)
                {
                    indexCorePoint = index;
                }
                ++index;
            }
            clusterSeeds.erase(clusterSeeds.begin()+indexCorePoint);
            for(std::vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i)
            {
                std::vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));
                if (clusterNeighors.size() >= m_minPoints)
                {
                    std::vector<int>::iterator iterNeighors;
                    for (iterNeighors = clusterNeighors.begin();
                         iterNeighors != clusterNeighors.end();
                         ++iterNeighors)
                    {
                        if (m_points.at(*iterNeighors).clusterID == UNCLASSIFIED || m_points.at(*iterNeighors).clusterID == NOISE)
                        {
                            if (m_points.at(*iterNeighors).clusterID == UNCLASSIFIED)
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
    
    std::vector<int> calculateCluster(Point point)
    {
        int index = 0;
        std::vector<Point>::iterator iter;
        std::vector<int> clusterIndex;
        for(iter = m_points.begin(); iter != m_points.end(); ++iter)
        {
            if (distanceSquared(point, *iter) <= m_epsilon_squared)
            {
                clusterIndex.push_back(index);
            }
            index++;
        }
        return clusterIndex;
    }
        
    inline double distanceSquared(const Point& pointCore, const Point& pointTarget)
    {
        return (pow(pointCore.x - pointTarget.x, 2) +
                pow(pointCore.y - pointTarget.y, 2) +
                pow(pointCore.z - pointTarget.z, 2));
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
};
