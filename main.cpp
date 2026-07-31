//------------------------------------------------------------------------------
//
//  main.cpp -- new flock experiments
//
//  Created by Craig Reynolds on January 6, 2024.
//  MIT License -- Copyright © 2024 Craig Reynolds
//
//------------------------------------------------------------------------------

#define USE_OPEN3D

#include "EvoFlock.h"

//~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
// TODO 20260729 VERY prototype WIP, should I use Eigan? (for plane fitting)

#include <Eigen/Dense>

// from https://stackoverflow.com/a/23418265/1991373

// Replaced arma:: with Eigen::

//    bool Geom_utils::Fit_plane_direct(const arma::mat& pts_in, Plane& plane_out)
//    {
//        bool success(false);
//        int K(pts_in.n_cols);
//        if(pts_in.n_rows == 3 && K > 2)  // check for bad sizing and indeterminate case
//        {
//            plane_out._p_3 = (1.0/static_cast<double>(K))*arma::sum(pts_in,1);
//            arma::mat A(pts_in);
//            A.each_col() -= plane_out._p_3; //[x1-p, x2-p, ..., xk-p]
//            arma::mat33 M(A*A.t());
//            arma::vec3 D;
//            arma::mat33 V;
//            if(arma::eig_sym(D,V,M))
//            {
//                // diagonalization succeeded
//                plane_out._n_3 = V.col(0); // in ascending order by default
//                if(plane_out._n_3(2) < 0)
//                {
//                    plane_out._n_3 = -plane_out._n_3; // upward pointing
//                }
//                success = true;
//            }
//        }
//        return success;
//    }

// https://libeigen.gitlab.io/eigen/docs-nightly/group__LeastSquares.html
//Solving linear least squares systems
//Dense linear problems and decompositions


// Google for "3D Least Squares Plane in eigen"

//    To compute a 3D least squares plane (orthogonal distance regression
//    minimizing perpendicular distances) using the Eigen C++ Library, use
//    Principal Component Analysis (PCA) via Singular Value Decomposition (SVD).
//    The key steps are centering the data points, computing SVD, and extracting
//    the normal vector.
//
//    Implementation Steps
//    • Find Centroid: Compute the mean X, Y, Z position of all points to center the point cloud at the origin.
//    • Build Matrix: Form a 3 x N matrix where each column represents a centered 3D point.
//    • Compute SVD: Run Eigen's BDCSVD or JacobiSVD on the coordinate matrix.
//    • Extract Normal: The singular vector corresponding to the smallest singular value becomes the plane normal vector.

//    #include <Eigen/Dense>
//    #include <vector>
//
//    struct Point3D { double x, y, z; };
//
//    bool fitPlane(const std::vector<Point3D>& pts, Eigen::Vector3d& normal, Eigen::Vector3d& centroid) {
//        if (pts.size() < 3) return false;
//
//        // 1. Compute centroid
//        centroid.setZero();
//        for (const auto& p : pts) {
//            centroid(0) += p.x;
//            centroid(1) += p.y;
//            centroid(2) += p.z;
//        }
//        centroid /= static_cast<double>(pts.size());
//
//        // 2. Build centered 3 x N matrix
//        Eigen::MatrixXd centered(3, pts.size());
//        for (size_t i = 0; i < pts.size(); ++i) {
//            centered(0, i) = pts[i].x - centroid(0);
//            centered(1, i) = pts[i].y - centroid(1);
//            centered(2, i) = pts[i].z - centroid(2);
//        }
//
//        // 3. Apply SVD
//        Eigen::BDCSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinU | Eigen::ComputeThinV);
//
//        // 4. Normal is the last column of U (smallest singular value)
//        normal = svd.matrixU().col(2);
//        normal.normalize();
//
//        return true;
//    }



//bool fitPlane(const std::vector<Point3D>& pts, Eigen::Vector3d& normal, Eigen::Vector3d& centroid)
shape::Plane fitPlaneToPoints(const std::vector<Vec3>& points)
{
    //    if (pts.size() < 3) return false;
    if (points.size() < 3) { return shape::Plane(Vec3::none(), Vec3::none()); }
    
    //    // 1. Compute centroid
    //    centroid.setZero();
    //    for (const auto& p : pts) {
    //        centroid(0) += p.x;
    //        centroid(1) += p.y;
    //        centroid(2) += p.z;
    //    }
    //    centroid /= static_cast<double>(pts.size());
    
    //    // 1. Compute centroid
    //    Vec3 centroid;
    //    for (const auto& p : points) { centroid += p; }
    //    centroid /= points.size();
    
    // 1. Compute centroid
    Vec3 centroid;
    for (const auto& p : points) { centroid += (p / points.size()); }
    
    //    // 2. Build centered 3 x N matrix
    //    Eigen::MatrixXd centered(3, pts.size());
    //    for (size_t i = 0; i < pts.size(); ++i) {
    //        centered(0, i) = pts[i].x - centroid(0);
    //        centered(1, i) = pts[i].y - centroid(1);
    //        centered(2, i) = pts[i].z - centroid(2);
    //    }
    
    // 2. Build centered 3 x N matrix
    Eigen::MatrixXd centered(3, points.size());
    for (size_t i = 0; i < points.size(); ++i)
    {
        Vec3 c = points[i] - centroid;
        centered(0, i) = c.x();  // maybe I should use an "as_array" method?
        centered(1, i) = c.y();
        centered(2, i) = c.z();
    }
    
    // 3. Apply SVD
    Eigen::BDCSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    //    // 4. Normal is the last column of U (smallest singular value)
    //    normal = svd.matrixU().col(2);
    //    normal.normalize();
    
    // 4. Normal is the last column of U (smallest singular value)
    Vec3 normal(svd.matrixU().col(2)[0],
                svd.matrixU().col(2)[1],
                svd.matrixU().col(2)[2]);
    
    //    return true;
    
    return shape::Plane(normal.normalize(), centroid);
}

//~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

int main(int argc, const char * argv[])
{
    EF::unit_test();
    
    
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20260727 WIP on plane-fitting for neighbors in murmurations
    //
    
//        {
//            Draw& draw = Draw::getInstance();
//            draw.setEnable(true);
//
//            auto disk = [&](Vec3 normal, Vec3 center,
//                            double diameter, double thickness,
//                            int count)
//            {
//                std::vector<Vec3> points;
//
//                //~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~
//                // TODO 20260728 add RandomSequence::randomPointInCylinder
//
//    //            double hd = diameter / 2;
//    //            double ht = thickness / 2;
//    //            Vec3 bc(hd, hd, ht);
//                Vec3 draw_point;
//    //            LocalSpace ls = LocalSpace::fromTo(center, center + normal);
//                for (int i = 0; i < count; i++)
//                {
//    //                Vec3 box_point = EF::RS().randomPointInAxisAlignedBox(bc, -bc);
//    //                Vec3 global_point = ls.globalizePosition(box_point);
//
//                    Vec3 global_point = EF::RS().randomPointInCylinder(diameter/2,
//                                                                       thickness,
//                                                                       normal,
//                                                                       center);
//                    //~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~ ~ ~~
//
//                    points.push_back(global_point);
//                    debugPrint(global_point)
//                    draw.addAnnotationAxes(Vec3(), 5);
//                    draw.addAnnotationLine(global_point, draw_point, Color::cyan(), 0.1);
//                    draw_point = global_point;
//                }
//                return points;
//            };
        
    {
        Draw& draw = Draw::getInstance();
        draw.setEnable(true);

        auto disk = [&](Vec3 normal, Vec3 center,
                        double diameter, double thickness,
                        int count)
        {
            std::vector<Vec3> points;
            Vec3 draw_point;
            for (int i = 0; i < count; i++)
            {
                Vec3 global_point = EF::RS().randomPointInCylinder(diameter/2,
                                                                   thickness,
                                                                   normal,
                                                                   center);
                points.push_back(global_point);
                debugPrint(global_point)
                draw.addAnnotationAxes(Vec3(), 5);
                draw.addAnnotationLine(global_point, draw_point, Color::cyan(), 0.1);
                draw_point = global_point;
            }
            return points;
        };

      
        
//        auto fitPlaneToPoints = [](const std::vector<Vec3>& points)
//        {
//            // TEMP should not duplicate this in case ever used on big dataset.
//            std::vector<Vec3> p = points;
//            
//            Vec3 sum = std::reduce(p.begin(), p.end(), Vec3(), std::plus());
//            Vec3 center = sum / p.size();
//            
//            // TEMP should not duplicate this in case ever used on big dataset.
//            for (int i = 0; i < p.size(); i++) { p[i] -= center; }
//            
//            
//            Vec3 normal(0, 1, 0);  // XXXXXXXXXXXXXXXXXXXXXXXXX
//            return shape::Plane(normal, center);
//        };


        draw.beginAnimatedScene();
        for (int i = 0; i < 1000; i++)
        {
            draw.beginOneAnimatedFrame();
            draw.clearAnnotations();

//            disk(Vec3(1, 0, 0),           Vec3(), 10, 1, 100);
//            disk(Vec3(1,1,1).normalize(), Vec3(), 10, 1, 100);
            
            std::vector<Vec3> points =
//            disk(Vec3(1, 0, 0), Vec3(1, 1, 1), 5, 1, 7);
            disk(Vec3(1, 1, 1).normalize(), Vec3(1, 1, 1), 5, 1, 7);

            
            shape::Plane plane = fitPlaneToPoints(points);
            draw.addAnnotationLine(plane.center,
                                   plane.center + plane.normal * 10,
                                   Color::magenta(), 0.2);

            

            util::thread_sleep_in_seconds(0.05);
            draw.addAnnotationsToAnimatedFrame();
            debugPrint(draw.enable());
//            debugPrint(draw.annotations_.size());
            draw.endOneAnimatedFrame();
        }
        draw.endAnimatedScene();
    }
    
    return EXIT_SUCCESS;
    
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20260606 test calculations for adjusting murmuration sphere radius
    //               to maintain boid density.
    
    FlockParameters fp;
    std::cout << std::endl;
    debugPrint(fp.sphereRadius())
    debugPrint(fp.boidsPerFlock())
    debugPrint(shape::Sphere::volumeFromRadius(fp.sphereRadius()));
    debugPrint(fp.boidsPerFlock() /
               shape::Sphere::volumeFromRadius(fp.sphereRadius()));
    std::cout << std::endl;
    double r = fp.sphereRadius();
    double v = shape::Sphere::volumeFromRadius(r);
    double bpf = fp.boidsPerFlock();
    double bpm3 = bpf / v;
    debugPrint(r);
    debugPrint(v);
    debugPrint(bpf);
    debugPrint(bpm3);
    std::cout << std::endl;
    
    // Now say instead of 2000 boids in flock we only have 300
    double bpf_ratio = 300.0 / 2000.0;
    double v2 = v * bpf_ratio;
    double r2 = shape::Sphere::radiusFromVolume(v2);
    debugPrint(bpf_ratio);
    debugPrint(v2);
    debugPrint(r2);

    std::cout << std::endl;

//    return EXIT_SUCCESS;
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20260530 what is average radius of uniformly distributed
    //               points in a unit radius sphere? Answer: 0.75
    
//    int samples = 100000000;
//    double sum_of_radii = 0;
//    for (int i = 0; i < samples; i++)
//    {
//        Vec3 random_point = EF::RS().randomPointInUnitRadiusSphere();
//        sum_of_radii += random_point.length();
//    }
//    double average_radius = sum_of_radii / samples;
//    debugPrint(average_radius)
//    return EXIT_SUCCESS;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~
    // TODO 20260116 what distribution of tree sizes for give max size?
    
//        const LP::FunctionSet& fs = GP::evoflockGpFunctionSet();
//        fs.reset_smallest_init_tree_xxx();
//        
//        int tree_count = 1000;
//        double sum_of_sizes = 0;
//        
//        std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
//
//        for (int i = 0; i < tree_count; i++)
//        {
//            LP::GpTree tree;
//            
//    //        fs.makeRandomTree(max_size, tree);
//    //        tree = fs.newMakeRandomTree(50, 100);
//    //        tree = fs.newMakeRandomTree(10, 25);
//    //        tree = fs.newMakeRandomTree(15, 25);
//    //        fs.makeRandomTree(200, tree);
//    //        fs.makeRandomTree(1000, tree);
//            tree = fs.newMakeRandomTree(20, 100);
//            
//            sum_of_sizes += tree.size();
//    //        debugPrint(tree.size());
//        }
//        
//        std::cout << "Size average over " << tree_count << " trees = ";
//        std::cout << sum_of_sizes / tree_count << std::endl;
//        exit(EXIT_SUCCESS);
    
    //~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~


    EF::runOneFlockEvolution();
    // EF::runFlockEvolutionLoop();
    
    Draw::deleteInstance();
    return EXIT_SUCCESS;
}
