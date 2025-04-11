//
//  GpFunction.h
//  LazyPredator
//
//  Created by Craig Reynolds on 12/18/20.
//  Copyright © 2020 Craig Reynolds. All rights reserved.
//

#pragma once
#include "Utilities.h"
#include "GpType.h"

namespace LazyPredator
{

class GpTree;      // Forward reference to class defined later.

// Class to represent functions in "strongly typed genetic programming".
class GpFunction
{
public:
    GpFunction(){}
    GpFunction(const std::string& name,
               const std::string& return_type_name,
               const std::vector<std::string>& parameter_type_names,
               //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
               // TODO 20240628 can we do an eval of a const tree?
               
#ifdef eval_const_20240628
               std::function<std::any(const GpTree& t)> eval)
#else  // eval_const_20240628
               std::function<std::any(GpTree& t)> eval)
#endif // eval_const_20240628

               //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      : GpFunction(name, return_type_name, parameter_type_names, eval, 1) {}
    
    GpFunction(const std::string& name,
               const std::string& return_type_name,
               const std::vector<std::string>& parameter_type_names,
               //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
               // TODO 20240628 can we do an eval of a const tree?
               
#ifdef eval_const_20240628
               std::function<std::any(const GpTree& t)> eval,
#else  // eval_const_20240628
               std::function<std::any(GpTree& t)> eval,
#endif // eval_const_20240628

               //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
               float selection_weight)
      : name_(name),
        return_type_name_(return_type_name),
        parameter_type_names_(parameter_type_names),
        eval_(eval),
        selection_weight_(selection_weight) {}
    // String name of this GpFunction.
    const std::string& name() const { return name_; }
    // String name of this GpFunction's return type.
    const std::string& returnTypeName() const { return return_type_name_; }
    // Pointer to this GpFunction's return GpType.
    GpType* returnType() const { return return_type_; }
    // GpTypes for all parameters of this GpFunction.
    const std::vector<GpType*>& parameterTypes() const
    { return parameter_types_; }
    // String names for GpTypes of all parameters of this GpFunction.
    const std::vector<std::string>& parameterTypeNames() const
    { return parameter_type_names_; }
    // Set GpTypes for return type and all parameter types of this GpFunction.
    void setReturnTypePointer(GpType* rtp) { return_type_ = rtp; }
    void setParameterTypePointers(const std::vector<GpType*>& vec_ptype_ptrs)
    { parameter_types_ = vec_ptype_ptrs; }
    // Does this function have parameter of its return type? (Can call itself?)
    bool recursive() const
    {
        bool r = false;
        for (auto& n : parameterTypeNames()) if(n == returnTypeName()) r = true;
        return r;
    }
    // Minimum "size" required to terminate subtree with this function at root.
    int minSizeToTerminate() const { return min_size_to_terminate_; }
    void setMinSizeToTerminate(int s) { min_size_to_terminate_ = s; }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240628 can we do an eval of a const tree?

#ifdef eval_const_20240628
    // Evaluate (execute) a GpTree with this function at root
    // TODO probably should assert they match (this and tree root GpFunction)
    std::any eval(const GpTree& tree) const { return eval_(tree);  }
#else  // eval_const_20240628
    // Evaluate (execute) a GpTree with this function at root
    // TODO probably should assert they match (this and tree root GpFunction)
    std::any eval(GpTree& tree) const { return eval_(tree);  }
#endif // eval_const_20240628

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Called by FunctionSet init while linking types and functions.
    void linkToReturnType()
    {
        returnType()->addFunctionReturningThisType(this, name());
    }
    // Print description of this GpFunction to std::cout.
    void print() const
    {
        std::cout << "GpFunction: " << name() << ", return_type: ";
        std::cout << returnTypeName() << ", ";
        std::cout << parameterTypes().size() << " parameters: (";
        bool comma = false;
        for (int i = 0; i < parameterTypes().size(); i ++)
        {
            if (comma) std::cout << ", "; else comma = true;
            std::cout << parameterTypeNames().at(i);
        }
        std::cout << ")." << std::endl;
    }
    // Multiplier on random selection during initial tree construction.
    float selectionWeight() const { return selection_weight_; }
private:
    std::string name_;
    std::string return_type_name_;
    GpType* return_type_ = nullptr;
    std::vector<std::string> parameter_type_names_;
    std::vector<GpType*> parameter_types_;
    int min_size_to_terminate_ = std::numeric_limits<int>::max();
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TODO 20240628 can we do an eval of a const tree?
    
#ifdef eval_const_20240628
    std::function<std::any(const GpTree& t)> eval_ = nullptr;
#else  // eval_const_20240628
    std::function<std::any(GpTree& t)> eval_ = nullptr;
#endif // eval_const_20240628

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    float selection_weight_ = 1;
};

}  // end of namespace LazyPredator
