//
//  FunctionSet.h
//  LazyPredator
//
//  Created by Craig Reynolds on 8/9/20.
//  Copyright © 2020 Craig Reynolds. All rights reserved.
//

// The FunctionSet class defines a “domain specific language” in which Programs
// of Individuals are written. It defines the names and parameter lists of the
// functions. Because LazyPredator uses “strongly typed genetic programming”
// (STGP) FunctionSet also defines a set of types associated with parameters and
// value of the functions in the set. Finally, “ephemeral constants”--randomized
// numeric leaf values--are defined here.
//
// TODO--except in trivial cases these functions will be API from some other
// software system. (TexSyn is the initial target.) FunctionSet needs to be
// flexible to allow connecting with external API.
//
// TODO--so far this is all just prototyping

// TODO note about terminology -- as in "minimum 'size' required to terminate
// subtree with this function at root" -- terminate in the sense of "terminate
// the top-down random program generation process" -- consider also that it
// could be phrased as "minimum depth of subtree..." better to use that?
// "Termination" applies more to the construction process, but I think we can
// describe it in terms of static program structure.

#pragma once
#include "Utilities.h"
#include "GpType.h"
#include "GpFunction.h"
#include "GpTree.h"
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20251202 "compile" source code as string to GpTree for FunctionSet.
#include <cctype>
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

namespace LazyPredator
{

// Defines the function set used in a STGP run. Consists of a collection of
// GpTypes and one of GpFunctions. Supports creation of random program drawn
// from those types and functions, given a max_size and root type.
// TODO must support crossover and mutation.
class FunctionSet
{
public:
    FunctionSet(){}
    // New constructor using vectors of GpType and GpFunction.
    // TODO eventually this needs to be rewritten to be smaller.
    // Allows crossover_min_size to default to 1.
    FunctionSet(const std::vector<GpType>& type_specs,
                const std::vector<GpFunction>& function_specs)
    : FunctionSet(type_specs, function_specs, 1) {}
    // Version with collection of GpTypes, GpFunctions, and crossover_min_size.
    FunctionSet(const std::vector<GpType>& type_specs,
                const std::vector<GpFunction>& function_specs,
                int crossover_min_size)
    : crossover_min_size_(crossover_min_size)
    {
        // Process each GpType specifications, make local copy to modify.
        for (GpType gp_type : type_specs)
        {
            bool has_eg = gp_type.hasEphemeralGenerator();
            if (has_eg) gp_type.setMinSizeToTerminate(1);
            // Insert updated GpType into by-name map. (Copied again into map.)
            addGpType(gp_type);
            // Root type (for trees of this FS) defaults to first GpType listed.
            if (!getRootType()) setRootType(lookupGpTypeByName(gp_type.name()));
        }
        // Process each of the GpFunction specifications (copy then modify)
        for (GpFunction func : function_specs)
        {
            // Character string name of this function's return type.
            std::string rtn = func.returnTypeName();
            // Connect type names to GpType object
            func.setReturnTypePointer(lookupGpTypeByName(rtn));
            std::vector<GpType*> parameter_types;
            for (auto& parameter_name : func.parameterTypeNames())
            {parameter_types.push_back(lookupGpTypeByName(parameter_name));}
            func.setParameterTypePointers(parameter_types);
            // If this function does not call its own type (eg Uniform) it can
            // be used to terminate a subtree of that type. Record in GpType.
            if (!func.recursive())
            {
                // std::cout << "NOT recursive: " << func.returnTypeName();
                GpType& rt = *lookupGpTypeByName(rtn);
                // TODO this is "min size to terminate type" ASSUMING all
                // parameters are ephemeral constants. Make more general.
                int mstt = 1 + int(func.parameterTypeNames().size());
                if (mstt < rt.minSizeToTerminate())
                    rt.setMinSizeToTerminate(mstt);
                // std::cout << " min size to terminate: " << mstt << std::endl;
            }
            // Copy once more into name-to-GpFunction-object map.
            addGpFunction(func);
        }
        // Build per-GpType collections of GpFunctions returning that type.
        for (auto& [name, gp_function] : nameToGpFunctionMap())
        {
            gp_function.linkToReturnType();
        }
        // Set minSizeToTerminate() for each GpFunction.
        for (auto& [name, gp_function] : nameToGpFunctionMap())
        {
            int min_size = minSizeToTerminateFunction(gp_function);
            gp_function.setMinSizeToTerminate(min_size);
        }

        // Verify type and function names are unique.
        VerifyUniqueNames vutn("GpType");
        VerifyUniqueNames vufn("GpFunction");
        for (GpType gp_type : type_specs) { vutn.verify(gp_type.name()); }
        for (GpFunction func : function_specs) { vufn.verify(func.name()); }
    }
    
    // What is the minimum "size" required to terminate a program subtree with
    // the given function at the root? At FunctionSet construction time, this
    // computes the number which is then stored on each GpFunction instance.
    int minSizeToTerminateFunction(const GpFunction& gp_function) const
    {
        // We need 1 for the function name itself (or an ephemeral constant).
        int size = 1;
        // Then loop over all parameter types.
        for (auto& parameter_gp_type : gp_function.parameterTypes())
        {
            size += parameter_gp_type->minSizeToTerminate();
        }
        return size;
    }
    
    // Randomly select a function in this set that returns the given type and
    // can be implemented in a subtree no larger than max_size. Returns nullptr
    // if none found.
    GpFunction* randomFunctionOfTypeInSize(int max_size,
                                           const GpType& return_type) const
    {
        std::vector<GpFunction*> ok;
        for (auto& gp_function : return_type.functionsReturningThisType())
        {
            int ms = gp_function->minSizeToTerminate();
            if (max_size >= ms) { ok.push_back(gp_function); }
        }
        // Intended to be used only for testing and debugging
        if (function_filter) { function_filter(ok); };
        return weightedRandomSelect(ok);
    }
    
    // Return a GpFunction* randomly selected from a given collection of them.
    // Selection is made according to each GpFunction's selectionWeight().
    GpFunction* weightedRandomSelect(const std::vector<GpFunction*>&
                                     gp_functions) const
    {
        GpFunction* result = nullptr;
        // Workaround June 12, 2024: weightedRandomSelect() would (very) rarely
        // return null when gp_functions was non-empty. I first noticed this
        // when I generated one million random trees to verify a FunctionSet. At
        // tree 617985 the error occurred. It might be "random < next" versus
        // "random <= next" but since I was unsure I used a bigger hammer with
        // this fallback initialization trick.
        if (not gp_functions.empty()) { result = gp_functions.at(0); }

        float total_weight = 0;
        for (auto f : gp_functions) { total_weight += f->selectionWeight(); }
        float random = LPRS().random2(0.0f, total_weight);
        float previous = 0;
        for (auto gp_function : gp_functions)
        {
            float next = previous + gp_function->selectionWeight();
            if (random < next) { result = gp_function; break; }
            previous = next;
        }
        return result;
    }
    
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~
    // TODO 202400804 prototype customized makeRandomTree()


//        // Creates a random program (nested expression) using the "language" defined
//        // in this FunctionSet. Parameter "max_size" is upper bound on the number of
//        // nodes (function calls or constants) in the resulting program. The program
//        // returns a value of "return_type" from its root.
//        void makeRandomTree(int max_size,
//                            const GpType& return_type,
//                            int& output_actual_size,
//                            GpTree& gp_tree) const
//        {
//            // Find all function whose value is return_type, for which a subtree can
//            // be constructed in max_size or fewer nodes, and select on randomly.
//            GpFunction* rf = randomFunctionOfTypeInSize(max_size, return_type);
//
//            // Does return_type have an ephemeral constant generator?
//            bool has_eg = return_type.hasEphemeralGenerator();
//    //        bool prefer_constant = has_eg and LPRS().randomBool(0.1);
//    //        bool prefer_constant = has_eg and LPRS().randomBool(0.2);
//            bool prefer_constant = has_eg and LPRS().randomBool(0.5);
//    //        bool prefer_constant = has_eg;
//
//    //        if (rf)
//            if (rf and not prefer_constant)
//
//            {
//                // If found, recurse on a subtree with that function at the root.
//                makeRandomTreeRoot(max_size, return_type, *rf,
//                                   output_actual_size, gp_tree);
//            }
//
//    //        else if (return_type.hasEphemeralGenerator())
//            else if (has_eg)
//
//            {
//                // If no function found, but this type has an ephemeral generator,
//                // use it to generate a leaf constant, ending recursion.
//                output_actual_size++;
//                // TODO should pass rs() into the generator for repeatability.
//                std::any leaf_value = return_type.generateEphemeralConstant();
//                gp_tree.setRootValue(leaf_value, return_type);
//            }
//            else
//            {
//                // TODO better way to signal this FunctionSet specification error?
//                std::cout << "No function or ephemeral constant found for GpType ";
//                std::cout << return_type.name() << " remaining size: " << max_size;
//                std::cout << std::endl;
//                exit(EXIT_FAILURE);
//            }
//        }

    
    // TODO 202400804 prototype customized makeRandomTree()
    typedef std::function<Vec3(int max_size,
                               const GpType& return_type,
                               int& output_actual_size,
                               GpTree& gp_tree)> OverrideTreeMaker;

    OverrideTreeMaker override_tree_maker = nullptr;

    
    // Creates a random program (nested expression) using the "language" defined
    // in this FunctionSet. Parameter "max_size" is upper bound on the number of
    // nodes (function calls or constants) in the resulting program. The program
    // returns a value of "return_type" from its root.
    void makeRandomTree(int max_size,
                        const GpType& return_type,
                        int& output_actual_size,
                        GpTree& gp_tree) const
    {
        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~
        // TODO 202400804 prototype customized makeRandomTree()
        
        if (override_tree_maker)
        {
            override_tree_maker(max_size,
                                return_type,
                                output_actual_size,
                                gp_tree);
            return;
        }
        
        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~
        // Find all function whose value is return_type, for which a subtree can
        // be constructed in max_size or fewer nodes, and select on randomly.
        GpFunction* rf = randomFunctionOfTypeInSize(max_size, return_type);
        // Does return_type have an ephemeral constant generator?
        bool has_eg = return_type.hasEphemeralGenerator();
        bool prefer_constant = has_eg and LPRS().randomBool(0.5);
        if (rf and not prefer_constant)

        {
            // If found, recurse on a subtree with that function at the root.
            makeRandomTreeRoot(max_size, return_type, *rf,
                               output_actual_size, gp_tree);
        }
        else if (has_eg)
        {
            // If no function found, but this type has an ephemeral generator,
            // use it to generate a leaf constant, ending recursion.
            output_actual_size++;
            // TODO should pass rs() into the generator for repeatability.
            std::any leaf_value = return_type.generateEphemeralConstant();
            gp_tree.setRootValue(leaf_value, return_type);
        }
        else
        {
            // TODO better way to signal this FunctionSet specification error?
            std::cout << "No function or ephemeral constant found for GpType ";
            std::cout << return_type.name() << " remaining size: " << max_size;
            std::cout << std::endl;
            assert(false);
        }
    }

    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~
    
    // Overload to allow passing return_type_name as a string from top level.
    void makeRandomTree(int max_size,
                        const std::string& return_type_name,
                        int& output_actual_size,
                        GpTree& gp_tree) const
    {
        makeRandomTree(max_size,
                       *lookupGpTypeByName(return_type_name),
                       output_actual_size,
                       gp_tree);
    }
    
    // Overload to allow passing return_type_name as a string from top level,
    // and NOT passing in output_actual_size.
    void makeRandomTree(int max_size,
                        const std::string& return_type_name,
                        GpTree& gp_tree) const
    {
        int output_actual_size = 0;
        makeRandomTree(max_size, return_type_name, output_actual_size, gp_tree);
    }
    
    // Overload to pass ONLY "max_size" and "output_gp_tree" from top level.
    // "return_type" is assumed to be first GpType listed in the FunctionSet.
    // "output_actual_size" is provided locally and passed in.
    void makeRandomTree(int max_size, GpTree& output_gp_tree) const
    {
        int output_actual_size = 0;
        makeRandomTree(max_size,
                       *getRootType(),
                       output_actual_size,
                       output_gp_tree);
    }
    
    void makeRandomTreeRoot(int max_size,
                            const GpType& return_type,
                            const GpFunction& root_function,
                            int& output_actual_size,
                            GpTree& gp_tree) const
    {
        output_actual_size++;  // for "function_name" (or ephemeral) itself
        int size_used = 0;
        int count = int(root_function.parameterTypes().size());
        // Set root function in given GpTree object
        gp_tree.setRootFunction(root_function);
        // For each parameter of root, add/allocate a subtree in the GpTree.
        // Important this happen first so no iterators are invalidated later.
        gp_tree.addSubtrees(root_function.parameterTypes().size());
        // TODO do this in a cleaner way after it is working
        int subtree_index = 0;
        // Loop over each parameter of "function_name" generating a subtree.
        for (auto& parameter_type : root_function.parameterTypes())
        {
            int fair_share = (max_size - (1 + size_used)) / count;
            int min_size_for_type = parameter_type->minSizeToTerminate();
            int subtree_max_size = std::max(fair_share, min_size_for_type);
            int subtree_actual_size = 0;
            std::string subtree_source;
            makeRandomTree(subtree_max_size,
                           *parameter_type,
                           subtree_actual_size,
                           gp_tree.subtrees().at(subtree_index));
            // TODO do this in a cleaner way after it is working
            subtree_index++;
            output_actual_size += subtree_actual_size;
            size_used += subtree_actual_size;
            count--;
        }
    }
    
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20251218 WIP on general purpose "is this tree OK" predicate.
    
    // watch out for use of "valid" re GpTree::is_valid()
    
    // Is this GpTree valid and in the correct size range?
    bool isTreeOK(const GpTree& tree,
                  // are these instance variable? if not shouldn't they be?
                  int min_tree_size,
                  int max_tree_size) const
    {
        return (getValidateTreeFunction()(tree, *this) and
                util::between(tree.size(), min_tree_size, max_tree_size));
    };

    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240715 WIP new approach to tree generation.
    
    // Reimplement the original December 2020 LP random tree maker. It allowed
    // only a max_size parameter and frequently made trees MUCH smaller than
    // that. (Especially with the evoflock function set.) This version takes
    // both min and max size parameters and aims to make trees that fall into
    // that interval. It does this by simply repeatedly generating trees until
    // it finds one in the desired size range. It is not guaranteed to find one.
    //
    // TODO 20240716 Tested with evoflock GP function set: successfully made
    // 1000000 trees on size range [5,50] using the default retries=100, and on
    // size range [80,100] using retries=5000. Note that for tighter bounds,
    // such as [90,100], it failed to find suitable trees. I will leave this
    // problem for another day.
    //
    // Supports min and max tree size. Now I think I want one target size, not a
    // range, which the tree generator should “take seriously.” Probably the
    // 2020 version was orignally intended to do that, but I settled for a
    // result size somewhere between 1 and max_tree_size.
    //
    GpTree newMakeRandomTree(int min_tree_size,
                             int max_tree_size) const
    {
        // 2000 retries is WAY more than enough, want to avoid confusing fails.
        return newMakeRandomTree(min_tree_size, max_tree_size, 2000);
    }
    GpTree newMakeRandomTree(int min_tree_size,
                             int max_tree_size,
                             int retries) const
    {
        GpTree new_tree;
        
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
        // TODO 20251218 WIP on general purpose "is this tree OK" predicate.

//            // Is this GpTree in the the correct size range?
//            auto size_ok = [&](const GpTree& tree)
//            {
//                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                // TODO 20251217 add get/setValidateTreeFunction()
//
//    //            return util::between(tree.size(), min_tree_size, max_tree_size);
//
//                return (util::between(tree.size(), min_tree_size, max_tree_size) and
//                        getValidateTreeFunction()(tree, *this));
//
//                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//            };
        
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~

        // Make up to "retries" attempts to find a tree of the correct size.
        for (int i = 0; i < retries; i++)
        {
            GpTree temp_tree;
            makeRandomTree(max_tree_size, temp_tree);
            // Save temp_tree if size is OK, or this is the last retry.
            //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
            // TODO 20251218 WIP on general purpose "is this tree OK" predicate.
//            if (size_ok(temp_tree) or (i == retries - 1))
            if ((i == retries - 1) or
                isTreeOK(temp_tree, min_tree_size, max_tree_size))
            //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
            {
                new_tree = temp_tree;
                break;
            }
        }
        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
        // TODO 20251213 try making initial trees bigger for usingGP().
        int nts = new_tree.size();
        if (smallest_init_tree_xxx > nts) { smallest_init_tree_xxx = nts;}
        if (biggest_init_tree_xxx < nts) { biggest_init_tree_xxx = nts;}
        std::cout << "new_tree.size() = " << nts;
        std::cout << " " << smallest_init_tree_xxx;
        std::cout << " " << biggest_init_tree_xxx;
        std::cout << std::endl;
        //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
        // TODO 20251218 WIP on general purpose "is this tree OK" predicate.
//        if (not size_ok(new_tree))
        if (not isTreeOK(new_tree, min_tree_size, max_tree_size))
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
        {
            debugPrint(min_tree_size)
            debugPrint(max_tree_size)

            debugPrint(new_tree.size());
            std::cout << new_tree.to_string(true) << std::endl;
            std::cout << "consider using a wider range between min_tree_size"
                      << " and max_tree_size, or increasing the value of"
                      << " 'retries'." << std::endl;
        }
        assert(new_tree.is_valid());
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
        // TODO 20251218 WIP on general purpose "is this tree OK" predicate.
//        assert(size_ok(new_tree));
        assert(isTreeOK(new_tree, min_tree_size, max_tree_size));
        //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
        return new_tree;
    }

    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    // TODO 20251213 try making initial trees bigger for usingGP().
    static inline int biggest_init_tree_xxx = -1;
    static inline int smallest_init_tree_xxx = 1000;
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    void print() const
    {
        std::cout << std::endl;
        std::cout << nameToGpTypeMap().size() << " GpTypes: " << std::endl;
        for (auto& [n, t] : nameToGpTypeMap()) t.print();
        std::cout << std::endl;
        std::cout << nameToGpFunctionMap().size() << " GpFunctions: ";
        std::cout << std::endl;
        for (auto& [n, f] : nameToGpFunctionMap()) f.print();
        std::cout << std::endl;
    }
    
    // Used only below in FunctionSet, then undef-ed at end of file.
    // (TODO maybe rewrite this with templates?)
    #define name_lookup_util(name, map, category)                         \
    [&]()                                                                 \
    {                                                                     \
        auto it = map.find(name);                                         \
        bool found = it != map.end();                                     \
        if (not found)                                                    \
        {                                                                 \
            std::cout << category << " not found: " << name << std::endl; \
        }                                                                 \
        assert("unknown type" && found);                                  \
        return &(it->second);                                             \
    }()

    // Map string names to (pointers to) GpTypes/GpFunction objects, for both
    // const (public, read only) and non-const (private, writable, for internal
    // use only during constructor). Use macro to remove code duplication.
    const GpType* lookupGpTypeByName(const std::string& name) const
    { return name_lookup_util(name, nameToGpTypeMap(), "GpType"); }
    const GpFunction* lookupGpFunctionByName(const std::string& name) const
    { return name_lookup_util(name, nameToGpFunctionMap(), "GpFunction"); }
    
    // Add new GpType/GpFunction to FunctionSet, stored in a name-to-object map.
    void addGpType(GpType& type) { name_to_gp_type_[type.name()] = type; }
    void addGpFunction(GpFunction& f) { name_to_gp_function_[f.name()] = f; }
    // Get reference to name-to-object maps of all GpType/GpFunction objects
    std::map<std::string, GpType>& nameToGpTypeMap()
    { return name_to_gp_type_; }
    const std::map<std::string, GpType>& nameToGpTypeMap() const
    { return name_to_gp_type_; }
    std::map<std::string, GpFunction>& nameToGpFunctionMap()
    { return name_to_gp_function_; }
    const std::map<std::string, GpFunction>& nameToGpFunctionMap() const
    { return name_to_gp_function_; }
    
    // The type returned from the root of trees built from this function set.
    const GpType*  getRootType() const { return root_type_; }
    void setRootType(GpType* gp_type) { root_type_ = gp_type; }
    
    // Given a mutable collection of functions (std::vector<GpFunction*>) modify
    // it in place. (TODO maybe better to return a modified copy, by value?)
    // Intended to be used only for testing and debugging by removing candidate
    // GpFunctions from consideration when generating random GpTrees.
    static inline std::function<void(std::vector<GpFunction*>&)>
    function_filter = nullptr;
    
    // The smallest size for a subtree (GpTree) to be exchanged between parent
    // GpTrees during crossover. The default of 1 allows all subtrees including
    // leaf values such as numeric constants. Values of 2 or more exclude those.
    int getCrossoverMinSize() const { return crossover_min_size_; }
    
    typedef std::function<void(const GpTree& parent0,
                               const GpTree& parent1,
                               GpTree& offspring,
                               int min_size,
                               int max_size,
                               int fs_min_size)>
            crossover_function_type;
    
    void setCrossoverFunction(crossover_function_type cof)
    {
        crossover_function_hook_ = cof;
    }
    crossover_function_type getCrossoverFunction() const
    {
        return crossover_function_hook_;
    }
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20251201 "compile" source code as string to GpTree for FunctionSet.

    // "compile" source code as std::string to a GpTree for this FunctionSet.
    GpTree compile(const std::string& source_code) const
    {
        GpTree tree;
        std::vector<std::string> tokens = tokenizeTreeSource(source_code);
        int token_index = 0;
        compileTreeNode(tree, token_index, tokens);
//        debugPrint(token_index);
//        debugPrint(tokens.size());
        assert (token_index == (tokens.size() - 1));
        return tree;
    }
    
    GpTree compileTreeNode(GpTree& tree,
                           int& token_index,
                           const std::vector<std::string>& tokens) const
    {
        std::string first_token = tokens.at(token_index);
//        std::cout << "Enter compileTreeNode with token_index = " << token_index
//                  << " (" << std::quoted(first_token) << ")" << std::endl;
        if (is_numeric(first_token))
        {
            //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
            // TODO 20251207 the GpType SHOULD be passed into compileTreeNode(),
            //               but making a temporary workaround right now:
      
            const GpType* type = lookupGpTypeByName("Scalar");
            assert (type);
            tree.setRootValue(std::any(std::stod(first_token)), *type);

            //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~

        }
        else
        {
            const GpFunction* root_function = lookupGpFunctionByName(first_token);
            tree.setRootFunction(*root_function);
            tree.addSubtrees(root_function->parameterTypes().size());
            
            token_index++;
            assert(tokens.at(token_index) == "(");

            size_t st_count = tree.subtrees().size();
            for (int i = 0; i < st_count; i++)
            {
                token_index++;
                GpTree& subtree = tree.subtrees().at(i);
                compileTreeNode(subtree, token_index, tokens);
                
                if ((st_count > 1) and (i < (st_count - 1)))
                {
//                    std::cout << "check for comma" << std::endl;
                    token_index++;
                    assert(tokens.at(token_index) == ",");
                }
            }
            token_index++;
            assert(tokens.at(token_index) == ")");
        }
//        std::cout << "Exit compileTreeNode with token_index = " << token_index
//                  << " (" << std::quoted(tokens.at(token_index)) << ")"
//                  << std::endl;
        return tree;
    }

    static bool is_numeric(std::string s)
    {
        bool n = true;
        for (int i = 0; i < s.size(); i++)
        {
            unsigned char d = s[i];
            if (not (std::isdigit(d) or d == '-' or d == '.')) { n = false; }
        }
        return n;
    };

    

    //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~
    // TODO 20251202 break tokenization off from compile()
    
    
    static std::vector<std::string> tokenizeTreeSource(const std::string& source)
    {
        
        // Preprocess source code to be uniformly delimited by single blanks.
        std::string s = source;
        s = findAndReplaceAllOccurrences("(", " [ ", s);
        s = findAndReplaceAllOccurrences(")", " ] ", s);
        s = findAndReplaceAllOccurrences(",", " ; ", s);
        s = findAndReplaceAllOccurrences("[", "(", s);
        s = findAndReplaceAllOccurrences("]", ")", s);
        s = findAndReplaceAllOccurrences(";", ",", s);
        s = findAndReplaceAllOccurrences("\\n", " ", s);
        s = removeDuplicateWhitespace(s);
        
//        std::cout << std::endl << "source code:" << std::endl;
//        std::cout << std::quoted(s) << std::endl << std::endl;
        
        std::vector<std::string> tokens;
        size_t start = 0;
        size_t delim = 0;
        while (start < s.size())
        {
            delim = s.find(" ", start);
            auto token = s.substr(start, delim - start);
            tokens.push_back(token);
            start = delim + 1;
        }
        
//        std::cout << "tokens:" << std::endl;
//        for (auto t : tokens) { std::cout << std::quoted(t) << " "; }
//        std::cout << std::endl;

        return tokens;
    }
    
    //~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~  ~~

    // utilities for FS::compile()
    static std::string findAndReplaceAllOccurrences(std::string from,
                                                    std::string to,
                                                    std::string string)
    {
        size_t pos = string.find(from);
        while (pos != string.npos)
        {
//            std::cout << std::quoted(string) << std::endl;
            
            string.erase(pos, from.length());
            string.insert(pos, to);
            pos = string.find(from);
        }
//        std::cout << std::quoted(string) << std::endl;
        return string;
    }

    static std::string removeDuplicateWhitespace(std::string string)
    {
        size_t size1 = 0;
        size_t size2 = string.size();
        
        while (size1 != size2)
        {
            size1 = string.size();
            string = findAndReplaceAllOccurrences("  ", " ", string);
            size2 = string.size();
        }
        return string;
    }
    
    
    
    static void tempTestCompile()
    {
        FunctionSet fs
        {
            {
                { "Real", -100.0, 100.0 },
            },
            {
                {
                    "plus", "Real", {"Real", "Real"},
                    [](GpTree& tree)
                    {
                        return std::any(tree.evalSubtree<double>(0) +
                                        tree.evalSubtree<double>(1));
                    }
                },
                {
                    "times", "Real", {"Real", "Real"},
                    [](GpTree& tree)
                    {
                        return std::any(tree.evalSubtree<double>(0) *
                                        tree.evalSubtree<double>(1));
                    }
                },
                {
                    "abs", "Real", {"Real"},
                    [](GpTree& tree)
                    {
                        return std::any(std::abs(tree.evalSubtree<double>(0)));
                    }
                },
            }
        };
//        std::string source = "a(b(5.2, c(-32)))";
//        std::string source = "a(b(5.2,\\n    c(-32)))";
//        std::string source = "q(b(5.2, c(-32)))";
//        std::string source = "a(b(5.2, c(-32)))";

//        std::string source = "plus(times(2.5, c(-3)), plus(1, 2)";
//        std::string source = "plus(times(2.5, abs(-3)), plus(1, 2)";
        std::string source = "plus(times(2.5, abs(-3)), plus(1, 2))";

        GpTree tree = fs.compile(source);
        std::cout << "Source code as string: " << source << std::endl;
        std::cout << "Tree: " << std::endl;
        std::cout << tree.to_string(true) << std::endl;
        
        std::cout << "Value: " << std::endl;
        std::cout << std::any_cast<double>(tree.eval()) << std::endl;
        // Can't we supply an eval()<> like evalSubtree()
        
//        findAndReplaceAllOccurrences("zap",
//                                     "x",
//                                     "ding zap yadda zap quux zap");
//        findAndReplaceAllOccurrences("zap",
//                                     "zip-a-dee-doo-dah",
//                                     "ding zap yadda zap quux zap");
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    
    // Utility to print "typical" trees from this FS, to help inspect/debug.
    void print_typical_trees() { print_typical_trees(5, 10, 20); }
    void print_typical_trees(int n, int min_size, int max_size)
    {
        for (int i = 0; i < n; i++)
        {
            GpTree gp_tree = newMakeRandomTree(min_size, max_size);
            debugPrint(gp_tree.size());
            std::cout << gp_tree.to_string(true) << std::endl << std::endl;
        }
    }

    // Does a given GpFunc appear anywhere in a given GpTree?
    bool isFunctionInTree(const GpFunction& function, const GpTree& tree) const
    {
        if (tree.isLeaf()) { return false; }
        if (&tree.getRootFunction() == &function) { return true; }
        for (const auto& st : tree.subtrees())
        {
            if (isFunctionInTree(function, st)) { return true; }
        }
        return false;
    }
    
    // Same but specify the GpFunc by its string name.
    bool isFunctionInTree(const std::string& func_name, const GpTree& tree) const
    {
        return isFunctionInTree(*lookupGpFunctionByName(func_name), tree);
    }
    
    // TODO 20251216 for testing.
    void isFunctionInTreeTest(const GpTree& tree) const
    {
        std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~";
        std::cout << std::endl << std::endl;
        size_t size = nameToGpFunctionMap().size();
        size_t index = EF::RS().randomN(size);
        std::map<std::string, GpFunction>::const_iterator it( nameToGpFunctionMap().begin() );
        std::advance( it, index );
        GpFunction func = it->second;
        debugPrint(func.name())
        debugPrint(tree.getRootFunction().name())
        std::cout << "tree = " << std::endl;
        std::cout << tree.to_string(true) << std::endl;
        debugPrint(isFunctionInTree(func.name(), tree));
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20251217 add get/setValidateTreeFunction()
    
    // "Hook" for application-specific constraints on GpTrees, beyond size.
    typedef std::function<bool(const GpTree& tree, const FunctionSet& fs)>
        validate_tree_function_type;
    
    void setValidateTreeFunction(validate_tree_function_type vtf)
    {
        validate_tree_function_hook_ = vtf;
    }
    
    validate_tree_function_type getValidateTreeFunction() const
    {
        return validate_tree_function_hook_;
    }
    
    // TODO there is probably a better way to do this:
//    validate_tree_function_type validate_tree_function_hook_ = nullptr;
    validate_tree_function_type validate_tree_function_hook_ =
        [](const GpTree&, const FunctionSet&){return true;};
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
    // TODO 20251218 WIP on general purpose "is this tree OK" predicate.
    
    bool empty() const
    {
        return nameToGpFunctionMap().empty() and nameToGpFunctionMap().empty();
    }
    
    //~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

private:
    // These maps are used both to store the GpType and GpFunction objects,
    // plus to look up those objects from their character string names.
    std::map<std::string, GpType> name_to_gp_type_;
    std::map<std::string, GpFunction> name_to_gp_function_;
    // Non-const versions for use only in constructor.
    GpType* lookupGpTypeByName(const std::string& name)
    { return name_lookup_util(name, nameToGpTypeMap(), "GpType"); }
    GpFunction* lookupGpFunctionByName(const std::string& name)
    { return name_lookup_util(name, nameToGpFunctionMap(), "GpFunction"); }
    // The type returned from the root of trees built from this function set.
    GpType* root_type_ = nullptr;
    // The smallest size for a subtree (GpTree) to be exchanged between parent
    // GpTrees during crossover. The default of 1 allows all subtrees including
    // leaf values such as numeric constants. Values of 2 or more exclude those.
    int crossover_min_size_ = 1;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240305 adding crossover_function_hook_ for custom crossover.
    crossover_function_type crossover_function_hook_ = GpTree::crossover;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
};

#undef name_lookup_util

}  // end of namespace LazyPredator
