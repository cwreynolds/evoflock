//------------------------------------------------------------------------------
//
// main.cpp -- new flock experiments
//
//  Created by Craig Reynolds on January 6, 2024.
//  MIT License -- Copyright © 2024 Craig Reynolds
//
//------------------------------------------------------------------------------

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240318 trying linking to Open3D 0.18.0

// #define test_open3d
#ifdef test_open3d

#include "open3d/Open3D.h"

using namespace open3d;

double GetRandom() { return double(std::rand()) / double(RAND_MAX); }

std::shared_ptr<geometry::PointCloud> MakePointCloud(
                                                     int npts, const Eigen::Vector3d center, double radius, bool colorize) {
    auto cloud = std::make_shared<geometry::PointCloud>();
    cloud->points_.reserve(npts);
    for (int i = 0; i < npts; ++i) {
        cloud->points_.push_back({radius * GetRandom() + center.x(),
            radius * GetRandom() + center.y(),
            radius * GetRandom() + center.z()});
    }
    if (colorize) {
        cloud->colors_.reserve(npts);
        for (int i = 0; i < npts; ++i) {
            cloud->colors_.push_back({GetRandom(), GetRandom(), GetRandom()});
        }
    }
    return cloud;
}

void SingleObject() {
    // No colors, no normals, should appear unlit black
    auto cube = geometry::TriangleMesh::CreateBox(1, 2, 4);
    visualization::Draw({cube});
}

void MultiObjects() {
    const double pc_rad = 1.0;
    auto pc_nocolor = MakePointCloud(100, {0.0, -2.0, 0.0}, pc_rad, false);
    auto pc_color = MakePointCloud(100, {3.0, -2.0, 0.0}, pc_rad, true);
    const double r = 0.4;
    auto sphere_unlit = geometry::TriangleMesh::CreateSphere(r);
    sphere_unlit->Translate({0.0, 1.0, 0.0});
    auto sphere_colored_unlit = geometry::TriangleMesh::CreateSphere(r);
    sphere_colored_unlit->PaintUniformColor({1.0, 0.0, 0.0});
    sphere_colored_unlit->Translate({2.0, 1.0, 0.0});
    auto sphere_lit = geometry::TriangleMesh::CreateSphere(r);
    sphere_lit->ComputeVertexNormals();
    sphere_lit->Translate({4, 1, 0});
    auto sphere_colored_lit = geometry::TriangleMesh::CreateSphere(r);
    sphere_colored_lit->ComputeVertexNormals();
    sphere_colored_lit->PaintUniformColor({0.0, 1.0, 0.0});
    sphere_colored_lit->Translate({6, 1, 0});
    auto big_bbox = std::make_shared<geometry::AxisAlignedBoundingBox>(
                                                                       Eigen::Vector3d{-pc_rad, -3, -pc_rad},
                                                                       Eigen::Vector3d{6.0 + r, 1.0 + r, pc_rad});
    big_bbox->color_ = {0.0, 0.0, 0.0};
    auto bbox = sphere_unlit->GetAxisAlignedBoundingBox();
    auto sphere_bbox = std::make_shared<geometry::AxisAlignedBoundingBox>(
                                                                          bbox.min_bound_, bbox.max_bound_);
    sphere_bbox->color_ = {1.0, 0.5, 0.0};
    auto lines = geometry::LineSet::CreateFromAxisAlignedBoundingBox(
                                                                     sphere_lit->GetAxisAlignedBoundingBox());
    lines->PaintUniformColor({0.0, 1.0, 0.0});
    auto lines_colored = geometry::LineSet::CreateFromAxisAlignedBoundingBox(
                                                                             sphere_colored_lit->GetAxisAlignedBoundingBox());
    lines_colored->PaintUniformColor({0.0, 0.0, 1.0});
    
    visualization::Draw({pc_nocolor, pc_color, sphere_unlit,
        sphere_colored_unlit, sphere_lit, sphere_colored_lit,
        big_bbox, sphere_bbox, lines, lines_colored});
}

void Actions() {
    const char *SOURCE_NAME = "Source";
    const char *RESULT_NAME = "Result (Poisson reconstruction)";
    const char *TRUTH_NAME = "Ground truth";
    
    data::BunnyMesh bunny_data;
    auto bunny = std::make_shared<geometry::TriangleMesh>();
    io::ReadTriangleMesh(bunny_data.GetPath(), *bunny);
    
    bunny->PaintUniformColor({1, 0.75, 0});
    bunny->ComputeVertexNormals();
    auto cloud = std::make_shared<geometry::PointCloud>();
    cloud->points_ = bunny->vertices_;
    cloud->normals_ = bunny->vertex_normals_;
    cloud->PaintUniformColor({0, 0.2, 1.0});
    
    auto make_mesh = [SOURCE_NAME, RESULT_NAME](
                                                visualization::visualizer::O3DVisualizer &o3dvis) {
                                                    std::shared_ptr<geometry::PointCloud> source =
                                                    std::dynamic_pointer_cast<geometry::PointCloud>(
                                                                                                    o3dvis.GetGeometry(SOURCE_NAME).geometry);
                                                    auto mesh = std::get<0>(
                                                                            geometry::TriangleMesh::CreateFromPointCloudPoisson(*source));
                                                    mesh->PaintUniformColor({1, 1, 1});
                                                    mesh->ComputeVertexNormals();
                                                    o3dvis.AddGeometry(RESULT_NAME, mesh);
                                                    o3dvis.ShowGeometry(SOURCE_NAME, false);
                                                };
    
    auto toggle_result =
    [TRUTH_NAME,
     RESULT_NAME](visualization::visualizer::O3DVisualizer &o3dvis) {
        bool truth_vis = o3dvis.GetGeometry(TRUTH_NAME).is_visible;
        o3dvis.ShowGeometry(TRUTH_NAME, !truth_vis);
        o3dvis.ShowGeometry(RESULT_NAME, truth_vis);
    };
    
    visualization::Draw({visualization::DrawObject(SOURCE_NAME, cloud),
        visualization::DrawObject(TRUTH_NAME, bunny, false)},
                        "Open3D: Draw Example: Actions", 1024, 768,
                        {{"Create Mesh", make_mesh},
        {"Toggle truth/result", toggle_result}});
}

Eigen::Matrix4d_u GetICPTransform(
                                  const geometry::PointCloud &source,
                                  const geometry::PointCloud &target,
                                  const std::vector<visualization::visualizer::O3DVisualizerSelections::
                                  SelectedIndex> &source_picked,
                                  const std::vector<visualization::visualizer::O3DVisualizerSelections::
                                  SelectedIndex> &target_picked) {
    std::vector<Eigen::Vector2i> indices;
    for (size_t i = 0; i < source_picked.size(); ++i) {
        indices.push_back({source_picked[i].index, target_picked[i].index});
    }
    
    // Estimate rough transformation using correspondences
    pipelines::registration::TransformationEstimationPointToPoint p2p;
    auto trans_init = p2p.ComputeTransformation(source, target, indices);
    
    // Point-to-point ICP for refinement
    const double max_dist = 0.03;  // 3cm distance threshold
    auto result = pipelines::registration::RegistrationICP(
                                                           source, target, max_dist, trans_init);
    
    return result.transformation_;
}

void Selections() {
    std::cout << "Selection example:" << std::endl;
    std::cout << "  One set:  pick three points from the source (yellow), "
    << std::endl;
    std::cout << "            then pick the same three points in the target"
    "(blue) cloud"
    << std::endl;
    std::cout << "  Two sets: pick three points from the source cloud, "
    << std::endl;
    std::cout << "            then create a new selection set, and pick the"
    << std::endl;
    std::cout << "            three points from the target." << std::endl;
    
    data::DemoICPPointClouds demo_icp_pointclouds;
    auto source = std::make_shared<geometry::PointCloud>();
    io::ReadPointCloud(demo_icp_pointclouds.GetPaths(0), *source);
    if (source->points_.empty()) {
        utility::LogError("Could not open {}",
                          demo_icp_pointclouds.GetPaths(0));
        return;
    }
    auto target = std::make_shared<geometry::PointCloud>();
    io::ReadPointCloud(demo_icp_pointclouds.GetPaths(1), *target);
    if (target->points_.empty()) {
        utility::LogError("Could not open {}",
                          demo_icp_pointclouds.GetPaths(1));
        return;
    }
    source->PaintUniformColor({1.000, 0.706, 0.000});
    target->PaintUniformColor({0.000, 0.651, 0.929});
    
    const char *source_name = "Source (yellow)";
    const char *target_name = "Target (blue)";
    
    auto DoICPOneSet =
    [source, target, source_name,
     target_name](visualization::visualizer::O3DVisualizer &o3dvis) {
        auto sets = o3dvis.GetSelectionSets();
        if (sets.empty()) {
            utility::LogWarning(
                                "You must select points for correspondence before "
                                "running ICP!");
            return;
        }
        auto &source_picked_set = sets[0][source_name];
        auto &target_picked_set = sets[0][target_name];
        std::vector<visualization::visualizer::O3DVisualizerSelections::
        SelectedIndex>
        source_picked(source_picked_set.begin(),
                      source_picked_set.end());
        std::vector<visualization::visualizer::O3DVisualizerSelections::
        SelectedIndex>
        target_picked(target_picked_set.begin(),
                      target_picked_set.end());
        std::sort(source_picked.begin(), source_picked.end());
        std::sort(target_picked.begin(), target_picked.end());
        
        auto t = GetICPTransform(*source, *target, source_picked,
                                 target_picked);
        source->Transform(t);
        
        // Update the source geometry
        o3dvis.RemoveGeometry(source_name);
        o3dvis.AddGeometry(source_name, source);
    };
    
    auto DoICPTwoSets =
    [source, target, source_name,
     target_name](visualization::visualizer::O3DVisualizer &o3dvis) {
        auto sets = o3dvis.GetSelectionSets();
        if (sets.size() < 2) {
            utility::LogWarning(
                                "You must have at least two sets of selected "
                                "points before running ICP!");
            return;
        }
        auto &source_picked_set = sets[0][source_name];
        auto &target_picked_set = sets[1][target_name];
        std::vector<visualization::visualizer::O3DVisualizerSelections::
        SelectedIndex>
        source_picked(source_picked_set.begin(),
                      source_picked_set.end());
        std::vector<visualization::visualizer::O3DVisualizerSelections::
        SelectedIndex>
        target_picked(target_picked_set.begin(),
                      target_picked_set.end());
        std::sort(source_picked.begin(), source_picked.end());
        std::sort(target_picked.begin(), target_picked.end());
        
        auto t = GetICPTransform(*source, *target, source_picked,
                                 target_picked);
        source->Transform(t);
        
        // Update the source geometry
        o3dvis.RemoveGeometry(source_name);
        o3dvis.AddGeometry(source_name, source);
    };
    
    visualization::Draw({visualization::DrawObject(source_name, source),
        visualization::DrawObject(target_name, target)},
                        "Open3D: Draw example: Selection", 1024, 768,
                        {{"ICP Registration (one set)", DoICPOneSet},
        {"ICP Registration (two sets)", DoICPTwoSets}});
}

//int main(int argc, char **argv) {
void open3d_test() {
    SingleObject();
    MultiObjects();
    Actions();
    Selections();
}

#endif  // test_open3d


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "Agent.h"
#include "Boid.h"
#include "flock.h"
#include "GP.h"
#include "LocalSpace.h"
#include "obstacle.h"
#include "shape.h"
#include "Utilities.h"
#include "Vec3.h"
#include <iostream>

void run_unit_tests()
{
    util::unit_test();
    Vec3::unit_test();
    LocalSpace::unit_test();
    shape::unit_test();
    Obstacle::unit_test();
    Agent::unit_test();
    Boid::unit_test();
    Flock::unit_test();
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240224 very preliminary test
    LazyPredator::unit_test();
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    std::cout << "All unit tests OK." << std::endl;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240318 trying linking to Open3D 0.18.0
//int main(int argc, const char * argv[])
//int main_saved(int argc, const char * argv[])
int main(int argc, const char * argv[])
{
#ifdef test_open3d
    open3d_test();
    return EXIT_SUCCESS;
#endif  // test_open3d
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    
    run_unit_tests();

    
    
    //--------------------------------------------------------------------------
//    shape::OccupancyMap om(Vec3(7, 5, 3), Vec3(1, 1, 1), Vec3());
//    om.add(Vec3());
//    om.print();
//    debugPrint(om.fractionOccupied())
//    om.add(Vec3(-0.5, -0.5, -0.5));
//    om.add(Vec3(0.49, 0.49, 0.49));
//    om.print();
//    debugPrint(om.fractionOccupied())
//    std::cout << std::endl;
//    
//    om.setVoxelIJK(4, 3, 2, 2);
//    om.print();
//    debugPrint(7 * 5 * 3)
//    debugPrint(om.ijkToVoxelIndex(4, 3, 2))
//    debugPrint(om.voxelIndexToIJK(om.ijkToVoxelIndex(4, 3, 2)))
//    debugPrint(om.voxelIndexToIJK(om.ijkToVoxelIndex(6, 3, 0)))
//    debugPrint(om.voxelIndexToIJK(om.ijkToVoxelIndex(6, 3, 1)))
//    debugPrint(om.voxelIndexToIJK(om.ijkToVoxelIndex(1, 1, 1)))
//    debugPrint(om.voxelIndexToIJK(om.ijkToVoxelIndex(0, 0, 0)))
//
//    std::cout << std::endl;
//    shape::OccupancyMap o2(Vec3(10, 10, 10), Vec3(1, 1, 1), Vec3());
//    debugPrint(o2.voxelIndexToPosition(o2.positionToVoxelIndex(Vec3(0.1, 0.1, 0.1))))
//    debugPrint(o2.voxelIndexToPosition(o2.positionToVoxelIndex(Vec3(-0.1, -0.1, -0.1))))
//    debugPrint(o2.voxelIndexToPosition(o2.positionToVoxelIndex(Vec3(0.15, 0.15, 0.15))))
//
//    return EXIT_SUCCESS;
    //--------------------------------------------------------------------------

    
    
    int individuals = 500;
    int subpops = 25;
    int max_evolution_steps = 30000;
//    int max_evolution_steps = 10;
//    int max_evolution_steps = 50000;
//    int max_evolution_steps = 10;
    int min_tree_size = 2;
    int max_tree_size = 20;
    LazyPredator::Population* population = nullptr;

    {
        std::cout << "Create population." << std::endl;
        util::Timer t("Create population.");
        population = new LazyPredator::Population (individuals,
                                                   subpops,
                                                   max_tree_size,
                                                   min_tree_size,
                                                   max_tree_size,
                                                   evoflock_gp_function_set);
    }
    {
        std::cout << "Run evolution." << std::endl;
        util::Timer t("Run evolution.");
        for (int i = 0; i < max_evolution_steps; i++)
        {
            population->evolutionStep(evoflock_fitness_function);
            std::cout << std::endl;
        }
    }

    std::cout << std::endl;
    std::cout << std::endl;
    for (int i = 0; i < 10; i++)
    {
        const LP::Individual* individual = population->nthBestFitness(i);
        std::cout << individual->tree().to_string() << std::endl;
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240330 WIP for multi-objective fitness
#ifdef MULTI_OBJECTIVE_FITNESS
        auto fitness = rerun_flock_simulation(individual);
        debugPrint(LP::vec_to_string(fitness));
#else
        double fitness = rerun_flock_simulation(individual);
        debugPrint(fitness);
#endif
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    }
    
    print_occupancy_map = true;
    std::cout << std::endl;
    std::cout << std::endl;
    const LP::Individual* individual = population->nthBestFitness(0);
    std::cout << individual->tree().to_string() << std::endl;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240330 WIP for multi-objective fitness
#ifdef MULTI_OBJECTIVE_FITNESS
    auto fitness = rerun_flock_simulation(individual);
    debugPrint(LP::vec_to_string(fitness));
#else
    double fitness = rerun_flock_simulation(individual);
    debugPrint(fitness);
#endif
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    delete population;

    //--------------------------------------------------------------------------
        
//    double fitness = run_hand_tuned_flock_simulation(true);
//    debugPrint(fitness);

    //--------------------------------------------------------------------------
    
    return 0;
}
