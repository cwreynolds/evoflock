//
//  UnitTests.h
//  LazyPredator
//
//  Heavily revised Feb 26 2024.
//
//  Created by Craig Reynolds on 8/6/20.
//  Copyright © 2020 Craig Reynolds. All rights reserved.
//

#pragma once
#include "Population.h"

namespace LazyPredator
{

class TestFS
{
public:
    // Simple test with only Float and Int types which can be evaluated.
    static const FunctionSet& treeEval() { return tree_eval; }
    // For testing tree eval for cases including construction class objects.
    static const FunctionSet& treeEvalObjects() { return tree_eval_objects; }
    // Simple set for testing crossover.
    static const FunctionSet& crossover() { return cross_over; }
    
    class ClassC
    {
    public:
        ClassC(int i, int j) : i_(i), j_(j) {}
        std::string to_string() const
        {
            return ("ClassC(" + std::to_string(i_) +
                    ", " + std::to_string(j_) + ")");
        }
    private:
        int i_;
        int j_;
    };
    class ClassB
    {
    public:
        ClassB(float f) : f_(f) {}
        std::string to_string() const
        {
            return "ClassB(" + any_to_string<float>(f_) + ")";
        }
    private:
        float f_;
    };
    class ClassA
    {
    public:
        ClassA(const ClassB& b, ClassC c) : b_(b), c_(c)
        {
            constructor_count_++;
        }
        ~ClassA()
        {
            destructor_count_++;
        }
        static int getLeakCount()
        {
            return constructor_count_ - destructor_count_;
        }
        std::string to_string() const
        {
            return ("ClassA(" + b_.to_string() + ", " + c_.to_string() + ")");
        }
    private:
        const ClassB& b_;
        const ClassC c_;
        // Leak check. Count constructor/destructor calls. Should match at exit.
        static inline int constructor_count_ = 0;
        static inline int destructor_count_ = 0;
    };
private:
    static inline const FunctionSet tree_eval_objects =
    {
        {
            {
                "ClassA",
                // ephemeral generator
                nullptr,
                // to_string
                [](std::any a)
                {
                    return std::any_cast<ClassA&>(a).to_string();
                },
                // jiggler
                nullptr,
                // deleter
                [](std::any a)
                {
                    if (a.has_value())
                    {
                        ClassA* t = std::any_cast<ClassA*>(a);
                        if (t) delete t;
                    }
                }
            },
            {
                "ClassB",
                nullptr,
                [](std::any a)
                {
                    return std::any_cast<ClassB*>(a)->to_string();
                },
                nullptr
            },
            {
                "ClassC",
                nullptr,
                [](std::any a)
                {
                    return std::any_cast<ClassC>(a).to_string();
                },
                nullptr
            },
            { "Float", 0.0f, 1.0f },
            { "Int", 0, 9 }
        },
        {
            {
                "ClassA", "ClassA", {"ClassB", "ClassC"},
                [](GpTree& t)
                {
                    return std::any(new ClassA(*t.evalSubtree<ClassB*>(0),
                                               t.evalSubtree<ClassC>(1)));
                }
            },
            {
                "ClassB", "ClassB", {"Float"},
                [](GpTree& t)
                {
                    return std::any(new ClassB(t.evalSubtree<float>(0)));
                }
            },
            {
                "ClassC", "ClassC", {"Int", "Int"},
                [](GpTree& t)
                {
                    return std::any(ClassC(t.evalSubtree<int>(0),
                                           t.evalSubtree<int>(1)));
                }
            }
        }
    };
    
    // Moved "Float" to top in case we want to use that convention.
    // std::string root_type = "Float";
    static inline const FunctionSet tree_eval =
    {
        {
            { "Float", 0.0f, 1.0f },
            { "Int", 0, 9 }
        },
        {
            {
                "AddInt", "Int", {"Int", "Int"}, [](GpTree& t)
                {
                    return std::any(t.evalSubtree<int>(0) +
                                    t.evalSubtree<int>(1));
                }
            },
            {
                "AddFloat", "Float", {"Float", "Float"}, [](GpTree& t)
                {
                    return std::any(t.evalSubtree<float>(0) +
                                    t.evalSubtree<float>(1));
                }
            },
            {
                "Floor", "Int", {"Float"}, [](GpTree& t)
                {
                    return std::any(int(std::floor(t.evalSubtree<float>(0))));
                }
            },
            {
                "Sqrt", "Float", {"Int"}, [](GpTree& t)
                {
                    return std::any(float(std::sqrt(t.evalSubtree<int>(0))));
                }
            },
            {
                "Mult", "Float", {"Float", "Int"}, [](GpTree& t)
                {
                    return std::any(t.evalSubtree<float>(0) *
                                    t.evalSubtree<int>(1));
                }
            }
        }
    };
    
    static inline const FunctionSet cross_over =
    {
        {
            { "Int", 0, 9 }
        },
        {
            {
                "P", "Int", {"Int"},
                [](GpTree& t)
                {
                    return std::any(t.evalSubtree<int>(0));
                }
            },
            {
                "PP", "Int", {"Int", "Int"},
                [](GpTree& t)
                {
                    return std::any(t.evalSubtree<int>(0) +
                                    t.evalSubtree<int>(1));
                }
            },
            {
                "PPP", "Int", {"Int", "Int", "Int"},
                [](GpTree& t)
                {
                    return std::any(t.evalSubtree<int>(0) +
                                    t.evalSubtree<int>(1) +
                                    t.evalSubtree<int>(2));
                }
            },
            {
                "Q", "Int", {"Int"},
                [](GpTree& t)
                {
                    return std::any(t.evalSubtree<int>(0));
                }
            },
            {
                "QQ", "Int", {"Int", "Int"},
                [](GpTree& t)
                {
                    return std::any(t.evalSubtree<int>(0) +
                                    t.evalSubtree<int>(1));
                }
            },
            {
                "QQQ", "Int", {"Int", "Int", "Int"},
                [](GpTree& t)
                {
                    return std::any(t.evalSubtree<int>(0) +
                                    t.evalSubtree<int>(1) +
                                    t.evalSubtree<int>(2));
                }
            }
        },
        // Crossover min_size, must be larger than a single Int leaf value.
        2
    };
};


// This "sub-test" wrapper macro just returns the value of the given expression
// "e". If the value is NOT TRUE, the st() macro will also log the specific
// failing sub-test expression ("e") to aid in debugging. Unit test functions
// generally run several sub-tests ANDing the results together.
#define st(e) [&]()                                        \
{                                                          \
    bool _e_ok = (e);                                      \
    if (!_e_ok) std::cout << "fail: " << #e << std::endl;  \
    return _e_ok;                                          \
}()

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240903 tiny step toward refactoring LP's unit tests. ("it would be
// nice, but not necessary, to recast LazyPredator/UnitTests.h to the New Way,
// with a unit_test() method on each class, as a series of asserts.") This is
// to allow using LP::unit_test() as the name for the New Way unit tests.
//static void unit_test()
static void legacy_unit_test()
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
{
    bool verbose = false;
    auto maybe_log= [&](std::string s)
        { if (verbose) { std::cout << "    " << s << std::endl; } return true; };
    
    // population_allocation_of_individuals
    {
        bool start_with_none = st(Individual::getLeakCount() == 0);
        int target_count = 3;
        bool match_target_count = false;
        // These brackets serve to delimit the lifetime of Population "p".
        {
            Population p(target_count);
            match_target_count = (st(p.getIndividualCount() == target_count) &&
                                  st(Individual::getLeakCount() == target_count));
        }
        bool end_with_none = st(Individual::getLeakCount() == 0);
        assert (start_with_none && match_target_count && end_with_none &&
                maybe_log("population_allocation_of_individuals"));
    }

    // random_program_size_limit
    {
        bool all_ok = true;
        int total_subtests = 1000;
        RandomSequence rs(77365918);
        LPRS() = rs;
        const FunctionSet& fs = TestFS::treeEval();
        for (int i = 0; i < total_subtests; i++)
        {
            int max_size = int(rs.frandom2(4, 100));
            int actual_size = 0;  // Output arg, set to actual size of random tree.
            GpTree gp_tree;       // Output arg, stores random tree.
            fs.makeRandomTree(max_size, "Float", actual_size, gp_tree);
            // Are both measures of size (from makeRandomTree() and measured
            // after the fact from GpTree) within given limit? And they match?
            bool ok = (st(actual_size <= max_size) &&
                       st(gp_tree.size() <= max_size) &&
                       st(actual_size == gp_tree.size()));
            if (!ok) all_ok = false;
        }
        assert(all_ok && maybe_log("random_program_size_limit")) ;
    }

    // gp_function_weighted_select
    {
        bool result = true;
        LPRS().setSeed(46102969);
        auto eval_zero = [](GpTree& t) { return std::any(0); };
        // Define FunctionSet with random selection weightings.
        FunctionSet fs = { { { "Int", 0, 9 } },
                           { { "L", "Int", {"Int"}, eval_zero, 0.5 },
                             { "M", "Int", {"Int"}, eval_zero, 1 },
                             { "N", "Int", {"Int"}, eval_zero, 2 },
                             { "O", "Int", {"Int"}, eval_zero, 4 }, }, };
        // Vector of GpFunction* as required by FunctionSet::weightedRandomSelect()
        std::vector<GpFunction*> functions;
        for (auto& [name, gp_function] : fs.nameToGpFunctionMap())
        {
            functions.push_back(&gp_function);
        }
        // Call weightedRandomSelect() "repeat" times, count each func selection.
        int repeat = 10000;
        std::map<GpFunction*, int> counts;
        for (GpFunction* f : functions) { counts.insert({{f, 0.0f}}); }
        for (int i = 0; i < repeat; i++)
        {
            GpFunction* f = fs.weightedRandomSelect(functions);
            counts.at(f)++;
        }
        // Compare counts per pair of GpFunctions, expect next to be twice as much.
        for (int i = 1; i < counts.size(); i ++)
        {
            int a = counts.at(functions.at(i - 1));
            int b = counts.at(functions.at(i));
            if (!st(between(b, a * 1.95, a * 2.05))) result = false;
        }
        // Verify that weightedRandomSelect() returns null for enpty function list.
        std::vector<GpFunction*> no_functions;
        result = result && st(fs.weightedRandomSelect(no_functions) == nullptr);
        assert(result && maybe_log("gp_function_weighted_select"));
    }
    
    // gp_tree_construction
    {
        GpTree root;                                  // make empty tree
        bool created_empty = st(root.subtrees().empty()); // verify empty
        root.addSubtrees(2);                          // add 2 subtrees under r
        GpTree& st0 = root.getSubtree(0);             // name for subtree r.0
        GpTree& st1 = root.getSubtree(1);             // name for subtree r.1
        st1.addSubtrees(1);                           // add 1 subtree under r.1
        GpTree& st10 = root.getSubtree(1).getSubtree(0);// name for st r.1.a
        // Set leaf values to names as character string.
        GpType str("String");
        root.setRootValue(std::string("r"), str);
        st0.setRootValue(std::string("r.0"), str);
        st1.setRootValue(std::string("r.1"), str);
        st10.setRootValue(std::string("r.1.a"), str);
        
        // Add tests for GpTree assignment:
        const FunctionSet& fs = TestFS::crossover();
        GpTree gp_tree_0, gp_tree_1, gp_tree_2;
        LPRS().setSeed();
        fs.makeRandomTree(30, gp_tree_0);
        LPRS().setSeed();
        fs.makeRandomTree(30, gp_tree_1);
        gp_tree_2 = gp_tree_0;
        
        // Check for tree depth, breath, and labels. And assignments.
        assert(created_empty &&
               st(root.subtrees().size() == 2) &&
               st(st0.subtrees().size() == 0) &&
               st(st1.subtrees().size() == 1) &&
               st(st10.subtrees().size() == 0) &&
               st(std::any_cast<std::string>(root.getRootValue()) == "r") &&
               st(std::any_cast<std::string>(st0.getRootValue())  == "r.0") &&
               st(std::any_cast<std::string>(st1.getRootValue())  == "r.1") &&
               st(std::any_cast<std::string>(st10.getRootValue()) == "r.1.a") &&
               st(GpTree::match<int>(gp_tree_0, gp_tree_1)) &&
               st(GpTree::match<int>(gp_tree_0, gp_tree_2)) &&
               st(GpTree::match<int>(gp_tree_1, gp_tree_2)) &&
               maybe_log("gp_tree_construction"));
    }

    // gp_tree_eval_simple -- For simple case of "plain old data" types.
    {
        // Construct a tree for "AddInt(1, Floor(2.5))"
        int leaf1 = 1;                            // Leaf value Int 1
        float leaf25 = 2.5;                       // Leaf value Float 2.5
        int expected = leaf1 + int(leaf25);       // Expected return value Int 3
        const FunctionSet& fs = TestFS::treeEval();
        const GpType& type_int = *(fs.lookupGpTypeByName("Int"));
        const GpType& type_float = *fs.lookupGpTypeByName("Float");
        const GpFunction& gp_func_addint = *fs.lookupGpFunctionByName("AddInt");
        const GpFunction& gp_func_floor = *fs.lookupGpFunctionByName("Floor");
        GpTree gp_tree;                           // Make empty tree.
        gp_tree.setRootFunction(gp_func_addint);  // Set root function to AddInt.
        gp_tree.addSubtrees(2);                   // AddInt has two parameters.
        
        GpTree& st0 = gp_tree.getSubtree(0);      // Name for substree 0.
        GpTree& st1 = gp_tree.getSubtree(1);      // Name for substree 1.
        st1.addSubtrees(1);                       // "Floor" has one parameter.
        GpTree& st10 = st1.getSubtree(0);         // Name for substree 0 of st1.
        st0.setRootValue(leaf1, type_int);        // Subtree 0 is leaf const Int 1.
        st10.setRootValue(leaf25, type_float);    // Subtree 1,0 is leaf Float 2.5.
        st1.setRootFunction(gp_func_floor);       // Subtree 1 has function Floor.
        
        assert(st(gp_tree.getRootType() == &type_int) &&
               st(&gp_tree.getRootFunction() == &gp_func_addint) &&
               st(st1.getRootType() == &type_int) &&
               st(&st1.getRootFunction() == &gp_func_floor) &&
               st(std::any_cast<int>(gp_tree.eval()) == expected) &&
               maybe_log("gp_tree_eval_simple"));
    }

    // gp_tree_eval_objects
    {
        // Construct a tree for "ClassA(ClassB(0.5), ClassC(1, 2)"
        float leaf0_5 = 0.5;                     // Leaf value Float 0.5
        int leaf1 = 1;                           // Leaf value Int 1
        int leaf2 = 2;                           // Leaf value Int 2
        const FunctionSet& fs = TestFS::treeEvalObjects();
        const GpType& type_int = *fs.lookupGpTypeByName("Int");
        const GpType& type_float = *fs.lookupGpTypeByName("Float");
        const GpFunction& gp_func_class_a = *fs.lookupGpFunctionByName("ClassA");
        const GpFunction& gp_func_class_b = *fs.lookupGpFunctionByName("ClassB");
        const GpFunction& gp_func_class_c = *fs.lookupGpFunctionByName("ClassC");
        GpTree gp_tree;                          // Make empty tree.
        gp_tree.setRootFunction(gp_func_class_a);// Set root function to ClassA.
        gp_tree.addSubtrees(2);                  // ClassA has two parameters.
        GpTree& st0 = gp_tree.getSubtree(0);     // Name for substree 0.
        GpTree& st1 = gp_tree.getSubtree(1);     // Name for substree 1.
        st0.addSubtrees(1);                      // "ClassB" has one parameter.
        st1.addSubtrees(2);                      // "ClassC" has two parameters.
        st0.setRootFunction(gp_func_class_b);    // Subtree 0 has function "ClassB".
        st1.setRootFunction(gp_func_class_c);    // Subtree 1 has function "ClassC".
        GpTree& st00 = st0.getSubtree(0);        // Name for substree 0 of st0.
        GpTree& st10 = st1.getSubtree(0);        // Name for substree 0 of st1.
        GpTree& st11 = st1.getSubtree(1);        // Name for substree 1 of st1.
        st00.setRootValue(leaf0_5, type_float);  // Subtree 0,0 is leaf Float 0.5.
        st10.setRootValue(leaf1, type_int);      // Subtree 1,0 is leaf Int 1.
        st11.setRootValue(leaf2, type_int);      // Subtree 1,1 is leaf Int 2.
        
        std::any result_as_any = gp_tree.eval();
        TestFS::ClassA* result = std::any_cast<TestFS::ClassA*>(result_as_any);
        std::string expected = "ClassA(ClassB(0.5), ClassC(1, 2))";
        bool tree_as_expected = st(expected == result->to_string());
        gp_tree.deleteCachedValues();           // Clean up: run GpType deleters.
        assert(tree_as_expected && maybe_log("gp_tree_eval_objects"));
    }

    // gp_tree_crossover
    {
        bool ok = true;
        int retries = 50;
        LPRS().setSeed(32280650);
        int total_P = 0;
        int total_Q = 0;
        int max_init_tree_size = 30;
        int max_crossover_tree_size = 100;
        const FunctionSet& fs = TestFS::crossover();
        // Returns the initial letter of given function's name.
        auto initial = [&](const GpFunction& f) { return f.name().substr(0, 1); };
        // Along ALL paths from root to leaf count the max number of times the
        // initial (of func name) changes. A tree should be all "P"s with at
        // most one subtree of "Q"s, or vice versa, or entirely "P" or "Q".
        std::function<int(std::string, const GpTree&)>
        count_initial_changes = [&](std::string current_initial, const GpTree& tree)
        {
            int count = 0;
            const GpFunction& rf = tree.getRootFunction();
            if (!tree.isLeaf())
            {
                std::string irf = initial(rf);
                if (irf == "P") { total_P++; } else { total_Q++; }
                if (current_initial != irf)
                {
                    if (current_initial != "") { count++; }
                    current_initial = irf;
                }
                for (const auto& subtree : tree.subtrees())
                {
                    count += count_initial_changes(current_initial, subtree);
                }
            }
            return count;
        };
        // Used during random tree creation: filter list of candidate functions
        // to use only those whose initial letter matching "filter_string";
        std::string filter_string;
        FunctionSet::function_filter = [&](std::vector<GpFunction*>& funcs)
        {
            std::vector<GpFunction*> temp = funcs;
            funcs.clear();
            for (auto& func : temp)
            {
                if (filter_string == initial(*func)) { funcs.push_back(func); }
            }
        };
        // Try crossovers between several pairs of random P trees and Q trees.
        for (int i = 0; i < retries; i++)
        {
            GpTree gp_tree_p;
            GpTree gp_tree_q;
            GpTree gp_tree_o;
            filter_string = "P";
            fs.makeRandomTree(max_init_tree_size, gp_tree_p);
            filter_string = "Q";
            fs.makeRandomTree(max_init_tree_size, gp_tree_q);
            // Crossover between gp_tree_p and gp_tree_q to create gp_tree_o.
            GpTree::crossover(gp_tree_p,
                              gp_tree_q,
                              gp_tree_o,
                              1,
                              max_crossover_tree_size,
                              fs.getCrossoverMinSize());
            int count = count_initial_changes("", gp_tree_o);
            // std::cout << std::endl;
            // std::cout << gp_tree_p.to_string(true) << std::endl;
            // std::cout << gp_tree_q.to_string(true) << std::endl;
            // std::cout << gp_tree_o.to_string(true) << std::endl;
            // debugPrint(count);
            ok = ok && st((count == 0) || (count == 1));
        }
        float p_to_q_ratio = float(total_P) / float(total_Q);
        ok = ok && st(between(p_to_q_ratio, 0.75, 1.25));
        FunctionSet::function_filter = nullptr;
        //debugPrint(total_P)
        //debugPrint(total_Q)
        //debugPrint(p_to_q_ratio)
        assert(ok && maybe_log("gp_tree_crossover"));
    }

    // gp_tree_utility
    {
        // Make several random GpTrees. Call GpTree::collectVectorOfSubtrees()
        // and GpTree::collectSetOfTypes() on each. Then descend through tree
        // verifying each node agrees with those collections. Also verifies
        // GpTree::size().
        bool ok = true;
        int retries = 50;
        LPRS().setSeed(62750858);
        const FunctionSet& fs = TestFS::treeEval();
        for (int i = 0; i < retries; i++)
        {
            // Make a random tree of random size based on given FunctionSet.
            GpTree gp_tree;
            fs.makeRandomTree(LPRS().random2(50, 150), gp_tree);
            // Construct two sets, one subtrees(GpTree*), and one of GpType*.
            std::vector<GpTree*> vector_of_subtrees;
            std::set<GpTree*> set_of_subtrees;
            std::set<const GpType*> set_of_types;
            gp_tree.collectVectorOfSubtrees(vector_of_subtrees);
            for (GpTree* t : vector_of_subtrees) { set_of_subtrees.insert(t); }
            gp_tree.collectSetOfTypes(set_of_types);
            // Verify that set_of_subtrees.size() is equal to gp_tree.size().
            ok = ok && st(gp_tree.size() == set_of_subtrees.size());
            // Traverse a GpTree, testing each node for membership in both sets.
            std::function<void(GpTree*)> check = [&](GpTree* t)
            {
                ok = ok && st(set_contains(set_of_subtrees, t));
                ok = ok && st(set_contains(set_of_types, t->getRootType()));
                for (auto& subtree : t->subtrees()) { check(&subtree); }
            };
            // Check "gp_tree".
            check(&gp_tree);
        }
        assert(ok && maybe_log("gp_tree_utility"));
    }

    // gp_type_deleter
    {
        int individuals = 100;
        int max_tree_size = 100;
        LPRS().setSeed(65053574);
        bool constructed, destructed;
        // Block to contain lifetime of Population "p".
        {
            // Make a Population of Individuals from FunctionSet "treeEvalObjects".
            Population p(individuals, max_tree_size, TestFS::treeEvalObjects());
            // Force early evaluation of each Individual's GpTree.
            p.applyToAllIndividuals([](Individual* i){ i->treeValue(); });
            // Verify instances of ClassA have been constructed.
            constructed = st(TestFS::ClassA::getLeakCount() > 0);
        }
        // Verify all objects of ClassA that were constructed were also destroyed.
        destructed = st(TestFS::ClassA::getLeakCount() == 0);
        assert(constructed && destructed && maybe_log("gp_type_deleter"));
    }



    // subpopulation_and_stats
    {
        bool ok = true;
        LPRS().setSeed(239473519);
        const FunctionSet& fs = TestFS::treeEval();
        // Test getStepCount().
        {
            Population p(10, 2, 10, fs);
            p.setLoggerFunction([](Population& p){});  // Do nothing logger.
            int steps = 10;
            p.run(steps, [](TournamentGroup tg){ return tg; });  // Identity tf.
            ok = ok && st(p.getStepCount() == steps);
        }
        // Test that each subpopulation has correct count of Individuals,
        // given random total numbers of Individuals and subpopulations.
        for (int i = 0; i < 20; i++)
        {
            int count = LPRS().random2(5, 25);
            int demes = LPRS().random2(1, count);
            Population p(count, demes, 10, fs);
            ok = ok && st(p.getIndividualCount() == count);
            ok = ok && st(p.getSubpopulationCount() == demes);
            for (int s = 0; s < p.getSubpopulationCount(); s++)
            {
                int size_of_subpop = int(p.subpopulation(s).size());
                int fair_share = count / demes;
                ok = ok && st(std::abs(size_of_subpop - fair_share) <= 1);
            }
        }
        // Test averageTreeSize(). Compute average three ways, ensure they match.
        {
            int individual_count = 100;
            Population p(individual_count, 2, 20, fs);
            int sum1 = 0;
            int sum2 = 0;
            p.applyToAllIndividuals([&](Individual* i){sum1+=(i->tree().size());});
            for (int i = 0; i < p.getSubpopulationCount(); i++)
            {
                for (auto& j : p.subpopulation(i)) { sum2 += j->tree().size(); }
            }
            int average_tree_size_1 = sum1 / p.getIndividualCount();
            int average_tree_size_2 = sum2 / p.getIndividualCount();
            ok = ok && st(p.averageTreeSize() == average_tree_size_1);
            ok = ok && st(p.averageTreeSize() == average_tree_size_2);
        }
        // Test averageFitness(). Compute average three ways, ensure they match.
        {
            int individual_count = 100;
            Population p(individual_count);
            float sum1 = 0;
            float sum2 = 0;
            // Function to set each Individual to next value of counter.
            float counter = 0;
            auto set_fitnesses_keep_sum = [&](Individual* i)
            {
                sum1 += counter;
                i->setFitness(counter++);
            };
            p.applyToAllIndividuals(set_fitnesses_keep_sum);
            // Directly iterate through all individuals in Population
            for (int i = 0; i < p.getSubpopulationCount(); i++)
            {
                for (auto& j : p.subpopulation(i)) { sum2 += j->getFitness(); }
            }
            float average_fitness_1 = sum1 / p.getIndividualCount();
            float average_fitness_2 = sum2 / p.getIndividualCount();
            ok = ok && st(p.averageFitness() == average_fitness_1);
            ok = ok && st(p.averageFitness() == average_fitness_2);
        }
        // Make sure all of the Individuals have been properly cleaned up.
        ok = ok && st(Individual::getLeakCount() == 0);
        assert(ok && maybe_log("subpopulation_and_stats"));
    }
    
    // subpopulation_migration
    {
        bool ok = true;
        int subpops = 5;
        int retries = 100;
        int tree_size = 100;
        int individuals = 100;
        LPRS().setSeed(46200778);
        const FunctionSet& fs = TestFS::treeEval();
        // Make a Population with given parameters.
        Population population(individuals, subpops, tree_size, fs);
        // Make a set containing all Individuals, ensuring none are duplicates.
        std::set<Individual*> before;
        auto collect_unique_individuals = [&](Individual* i)
        {
            ok = ok && st(!set_contains(before, i));
            before.insert(i);
        };
        population.applyToAllIndividuals(collect_unique_individuals);
        // Do many migrations. (Set likelihood to 1 so happens each call.)
        population.setMigrationLikelihood(1);
        for (int i = 0; i < retries; i++) { population.subpopulationMigration(); }
        // Verify each Individual was also present before migrations.
        auto verify_same_individuals = [&](Individual* i)
        {
            ok = ok && st(set_contains(before, i));
        };
        population.applyToAllIndividuals(verify_same_individuals);
        assert(ok && maybe_log("subpopulation_migration"));
    }

    // Reset LazyPredator's global RandomSequence to default seed.
    LPRS().setSeed();
}

}  // end of namespace LazyPredator
