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




//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240624 still debugging GP_not_GA error


//    #define check_Individual_13(individual)                              \
//    {                                                                    \
//        std::cout << "In check_Individual_13()" << std::endl;            \
//        static bool first = true;                                        \
//        static void* i13 = nullptr;                                      \
//        if (first) { i13 = individual; first = false; }                  \
//        assert(individual == i13);                                       \
//        assert(individual->pop_position == 13);                          \
//        auto gp_tree = individual->tree();                               \
//        assert(gp_tree.subtrees().size() == 0);                          \
//        assert(gp_tree.to_string() == "First_Obs_Normal()");             \
//        assert(gp_tree.getRootFunction().name() == "First_Obs_Normal");  \
//    }

#define check_Individual_13(individual) {}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240628 can we do an eval of a const tree?

//#define eval_const_20240628

#ifdef eval_const_20240628
#else  // eval_const_20240628
#endif // eval_const_20240628

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "Agent.h"
#include "Boid.h"
#include "dbscan.h"
#include "flock.h"
#include "GP.h"
#include "LocalSpace.h"
#include "obstacle.h"
#include "shape.h"
#include "Utilities.h"
#include "Vec3.h"
#include <iostream>

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240430 prototype dbscan

//double boid_dist(Boid* a, Boid* b)
//{
//    return (a->position() - b->position()).length();
//}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
    LazyPredator::unit_test();
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240430 prototype dbscan
//    BoidPtrList bpl ({new Boid(), new Boid(), new Boid()});
//    auto df = [](Boid* a, Boid* b) { return (a->position() -
//                                             b->position()).length(); };
//    dbscan d(bpl, df, 1, 1);
//    d.unit_test();
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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

    
    EF::setRS(LP::LPRS());
    
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

    //--------------------------------------------------------------------------
    
//    for (int i = 0; i < 10; i++)
//    {
//        MultiObjectiveFitness ht_fitness = run_hand_tuned_flock_simulation(true);
//        debugPrint(LP::vec_to_string(ht_fitness));
//    }
//    return EXIT_SUCCESS;

    //--------------------------------------------------------------------------

    
//    MultiObjectiveFitness mfo = {0.1, 0.2, 0.3, 0.4, 0.5};
//    debugPrint(std::reduce(mfo.begin(), mfo.end(), 1.0, std::multiplies()));
//    debugPrint(std::reduce(mfo.begin(), mfo.end(), 1.0, std::plus()) / mfo.size());
//    return EXIT_SUCCESS;

    //--------------------------------------------------------------------------
    
//    int count = 0;
//    double total = 1000000;
//    for (int i = 0; i < total; i++) { if (lp::LPRS().randomBool()) { count++; } }
//    debugPrint(count / total)
//    return EXIT_SUCCESS;
    
    //--------------------------------------------------------------------------
    
//    LazyPredator::MultiObjectiveFitness mof({0.1, 0.2, 0.3, 0.4});
//    debugPrint(mof);

    //--------------------------------------------------------------------------
    
//    DBSCAN::test();
//    return EXIT_SUCCESS;
    
    //--------------------------------------------------------------------------

    // TODO 20240606 starting to prototype GP from-scratch-no-black-box

//    LP::FunctionSet fs =  GP::evoflock_gp_function_set();
//    fs.print();
//    
//    std::vector<Boid> boids(10);
//    for (int i = 0; i < 10; i++)
//    {
//        boids[i].setPosition(EF::RS().random_unit_vector() * 10);
//        boids[i].setForward(EF::RS().random_unit_vector());
//    }
//    Boid& boid = boids[0];
//    BoidPtrList neighbors;
//    for (Boid& b : boids) { neighbors.push_back(&b); }
//    boid.set_flock_boids(&neighbors);
//    boid.recompute_nearest_neighbors();
//    EvertedSphereObstacle eso(100, Vec3());
//    ObstaclePtrList opl = {&eso};
//    boid.set_flock_obstacles(&opl);
//    FlockParameters fp;
//    boid.set_fp(&fp);
//
//    GP::setGpBoidPerThread(&boid);
//
//    for (int i = 0; i < 100; i++)
//    {
//        std::cout << i << ":" << std::endl;
//        LP::GpTree gp_tree;
//        fs.makeRandomTree(50, gp_tree);
//        std::cout << gp_tree.to_string(true) << std::endl;
//        Vec3 steering = std::any_cast<Vec3>(gp_tree.eval());
//        debugPrint(steering);
//        assert (steering.is_valid());
//        std::cout << std::endl << std::endl;
//    }
//
//    return EXIT_SUCCESS;
    
    //--------------------------------------------------------------------------

    int individuals = 500;
    int subpops = 25;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//    int max_evolution_steps = 30000;
    int max_evolution_steps = 100;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//    lp::LPRS().setSeed(20240408);
//    lp::LPRS().setSeed(20240409);
//    lp::LPRS().setSeed(202404091);
//    lp::LPRS().setSeed(20240410);
//    lp::LPRS().setSeed(2024041015);
//    lp::LPRS().setSeed(2024041114);
//    lp::LPRS().setSeed(2024041416);
//    lp::LPRS().setSeed(2024041716);
//    lp::LPRS().setSeed(2024041916);
//    lp::LPRS().setSeed(20240424);
//    lp::LPRS().setSeed(20240427);
//    lp::LPRS().setSeed(20240504);
//    lp::LPRS().setSeed(20240505);
//    lp::LPRS().setSeed(20240506);
//    lp::LPRS().setSeed(20240508);
//    lp::LPRS().setSeed(20240509);
//    lp::LPRS().setSeed(20240512);
    LP::LPRS().setSeed(20240606);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240619 WIP first GP_not_GA run
        
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240628 can we do an eval of a const tree?
//    Boid::GP_not_GA = false;
    Boid::GP_not_GA = true;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//    int min_tree_size = 2;
//    int max_tree_size = 20;
    int min_tree_size = Boid::GP_not_GA ? 10  :  2;
    int max_tree_size = Boid::GP_not_GA ? 100 : 20;
    
    auto fitness_function = (Boid::GP_not_GA ?
                             GP::evoflock_gp_fitness_function :
                             GP::evoflock_ga_fitness_function);

//    LP::Individual* save_i13 = nullptr;
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    LP::Population* population = nullptr;

    LP::FunctionSet fs = (Boid::GP_not_GA ?
                          GP::evoflock_gp_function_set() :
                          GP::evoflock_ga_function_set);

    {
        std::cout << "Create population." << std::endl;
        util::Timer t("Create population.");
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240619 WIP first GP_not_GA run
        
//        LazyPredator::FunctionSet& fs = GP::evoflock_ga_function_set;
//        fs.setCrossoverFunction(GP::evoflock_ga_crossover);

//        LazyPredator::FunctionSet fs = (Boid::GP_not_GA ?
//                                        GP::evoflock_gp_function_set() :
//                                        GP::evoflock_ga_function_set);
        
        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
        // TODO 20240702 try using that new switch

//        if (not Boid::GP_not_GA)
//        {
//            fs.setCrossoverFunction(GP::evoflock_ga_crossover);
//        }

        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        population = new LazyPredator::Population (individuals,
                                                   subpops,
                                                   max_tree_size,
                                                   min_tree_size,
                                                   max_tree_size,
                                                   fs);
        
        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
        // TODO 20240702 try using that new switch
                
        if (Boid::GP_not_GA)
        {
            population->explicit_treeValue_in_evolutionStep = false;
        }
        else
        {
            fs.setCrossoverFunction(GP::evoflock_ga_crossover);
        }
        
        //~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // TODO 20240619 WIP first GP_not_GA run

        if (Boid::GP_not_GA)
        {
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20240624 still debugging GP_not_GA error
//            population->trap_on_destructor = true;
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            
            std::vector<Boid> boids(10);
            for (int i = 0; i < 10; i++)
            {
                boids[i].setPosition(EF::RS().random_unit_vector() * 10);
                boids[i].setForward(EF::RS().random_unit_vector());
            }
            Boid& boid = boids[0];
            BoidPtrList neighbors;
            for (Boid& b : boids) { neighbors.push_back(&b); }
            boid.set_flock_boids(&neighbors);
            boid.recompute_nearest_neighbors();
            EvertedSphereObstacle eso(100, Vec3());
            ObstaclePtrList opl = {&eso};
            boid.set_flock_obstacles(&opl);
            FlockParameters fp;
            boid.set_fp(&fp);
            
            Boid::setGpPerThread(&boid);
            
            int count = 1;
            auto print_individuals_tree = [&](LP::Individual* i)
            {
                i->pop_position = count;
                
                
//                std::cout << std::endl << count << std::endl;
//                debugPrint(i)
//                std::string tree_string = i->tree_to_string();
//                std::cout << tree_string << std::endl;
//                LP::GpTree gp_tree = i->tree();
//                std::string copied_tree_string = gp_tree.to_string(true);
//                std::cout << "tree copy matches original" << std::endl;
//                assert(tree_string == copied_tree_string);
//                std::cout << "treeValue():" << std::endl;
//                std::cout << std::any_cast<Vec3>(i->treeValue()) << std::endl;
//                
//                std::cout << "GpTree::print():" << std::endl;
//                gp_tree.print();
//
//                i->treeValue();
//                i->qqq_count = count;
//                i->duplicate_tree = i->tree();
//                debugPrint(i->duplicate_tree.to_string())
                
                
                i->treeValue();
                i->duplicate_tree = i->tree();

                // TODO 20240622 just temporary for debugging
                GP::values_of_individuals[i] = std::any_cast<Vec3>(i->treeValue());
                GP::trees_of_individuals[i] = i->tree();
                
//                Vec3 trap(-0.273772, -0.526079, 0.805164);
//                if (Vec3::is_equal_within_epsilon(trap,
//                                                  std::any_cast<Vec3>(i->treeValue()),
//                                                  0.0001))
//                {
//                    debugPrint(GP::trees_of_individuals[i].to_string(true))
//                }
                
                
//                std::string tree_string = GP::trees_of_individuals[i].to_string();
//                tree_string = tree_string.substr(0, 150);
//                std::cout << count << ": " << i << " ";
//                std::cout << tree_string << std::endl;
                
//                if (count == 13)
//                {
//                    check_Individual_13(i);
//                    save_i13 = i;
//
//                    
//                    std::cout << "in main()" << std::endl;
//                    debugPrint(i)
//                    debugPrint(i->pop_position)
//                    debugPrint(Boid::getGpPerThread())
//                    debugPrint(GP::values_of_individuals[i])
//                    debugPrint(GP::trees_of_individuals[i].to_string(true))
//                    i->tree().print();
//                    i->duplicate_tree.print();
//                }
//                if (count > 13) { check_Individual_13(save_i13); }

                count++;
            };
            population->applyToAllIndividuals(print_individuals_tree);
            
            
        }
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    }
    
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240624 still debugging GP_not_GA error
//    check_Individual_13(save_i13);                              \
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    {
        std::cout << "Run evolution." << std::endl;
        util::Timer t("Run evolution.");
        for (int i = 0; i < max_evolution_steps; i++)
        {
            GP::save_fitness_time_series(*population);
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20240619 WIP first GP_not_GA run
            
//            population->evolutionStep(GP::evoflock_fitness_function,
//                                      GP::scalarize_fitness);

//            population->evolutionStep((GP_not_GA ?
//                                       GP::evoflock_gp_fitness_function :
//                                       GP::evoflock_ga_fitness_function),
//                                      GP::scalarize_fitness);

            std::cout << "evolution step: " << i << std::endl;
            
            population->evolutionStep(fitness_function, GP::scalarize_fitness);

            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            std::cout << std::endl;
        }
    }
    
    // Save end of run data.
    auto record_top_10 = [&]()
    {
        std::cout << std::endl;
        std::cout << std::endl;
        for (int i = 0; i < 10; i++)
        {
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // TODO 20240703 fix end of run logging for GP
//            const LP::Individual* individual = population->nthBestFitness(i);
            LP::Individual* individual = population->nthBestFitness(i);
//            std::cout << individual->tree().to_string() << std::endl;
            std::cout << individual->tree().to_string(true) << std::endl;
//            auto fitness = GP::rerun_flock_simulation(individual);
            LazyPredator::MultiObjectiveFitness fitness;
            if (Boid::GP_not_GA)
            {
                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // TODO for simplicity, change get/setSaveBoidCenters() to be static
                Flock::setSaveBoidCenters(true);
                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

                fitness = GP::evoflock_gp_fitness_function(individual);
            }
            else
            {
                fitness = GP::rerun_flock_simulation(individual);
            }
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            debugPrint(fitness);
        }
    };
    record_top_10();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240703 fix end of run logging for GP
//    std::cout << std::endl;
//    std::cout << std::endl;
//    std::cout << "now with MOF scalarized with scalarize_fitness_min()" << std::endl;
//    GP::replace_scalar_fitness_metric(*population, GP::scalarize_fitness_min);
//    record_top_10();
//
//    GP::print_occupancy_map = true;
//    std::cout << std::endl;
//    std::cout << std::endl;
//    const LP::Individual* individual = population->nthBestFitness(0);
//    std::cout << individual->tree().to_string() << std::endl;
//    auto fitness = GP::rerun_flock_simulation(individual);
//    debugPrint(fitness);
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    delete population;
    LP::Individual::leakCheck();
    return 0;
}
