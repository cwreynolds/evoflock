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


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// NOTE on 202610245 I cleaned up some two year old experiments below. I removed
// five blocks of code like this:
//
//           //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//           // TODO 20240628 can we do an eval of a const tree?
//
//#ifdef eval_const_20240628
//           std::function<std::any(const GpTree& t)> eval)
//#else  // eval_const_20240628
//           std::function<std::any(GpTree& t)> eval)
//#endif // eval_const_20240628
//
//           //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Replacing them with the original non-const versions.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// Utilities, defined at global scope, probably be moved to another file.
typedef std::vector<std::string> VecOfStrings;
typedef std::vector<VecOfStrings> VecOfVecOfStrings;


// Used as third arg of a GpFunc constructor to pair parameter types and names.
class GpFuncParamList
{
public:
    GpFuncParamList(const VecOfVecOfStrings& type_name_pairs)
    {
        for (auto& pair : type_name_pairs)
        {
            parameter_type_names_.push_back(pair.at(0));
            parameter_names_.push_back(pair.at(1));
        }
    }
    // String names for GpTypes of all parameters of this GpFunction.
    const VecOfStrings& parameterTypeNames()const{return parameter_type_names_;}
    // String names for all parameters of this GpFunction.
    const VecOfStrings& parameterNames() const { return parameter_names_; }
private:
    VecOfStrings parameter_type_names_;
    VecOfStrings parameter_names_;
};

// Class to represent functions in "strongly typed genetic programming".
class GpFunction
{
public:
    GpFunction(){}
    GpFunction(const std::string& name,
               const std::string& return_type_name,
               const VecOfStrings& parameter_type_names,
               std::function<std::any(GpTree& t)> eval)
      : GpFunction(name, return_type_name, parameter_type_names, eval, 1) {}
    GpFunction(const std::string& name,
               const std::string& return_type_name,
               const VecOfStrings& parameter_type_names,
               std::function<std::any(GpTree& t)> eval,
               float selection_weight)
      : name_(name),
        return_type_name_(return_type_name),
        parameter_type_names_(parameter_type_names),
        parameter_names_(default_param_names(parameter_type_names)),
        eval_(eval),
        selection_weight_(selection_weight) {}
    // Version with GpFuncParamList to pair parameter types and names (eg for GA)
    GpFunction(const std::string& name,
               const std::string& return_type_name,
               const GpFuncParamList& gp_func_param_list,
               std::function<std::any(GpTree& t)> eval)
      : GpFunction(name, return_type_name,
                   gp_func_param_list.parameterTypeNames(),
                   gp_func_param_list.parameterNames(),
                   eval,
                   1) {}
    // Version for explicit parameter_type_names and parameter_names.
    GpFunction(const std::string& name,
               const std::string& return_type_name,
               const std::vector<std::string>& parameter_type_names,
               const std::vector<std::string>& parameter_names,
               std::function<std::any(GpTree& t)> eval,
               float selection_weight)
      : name_(name),
        return_type_name_(return_type_name),
        parameter_type_names_(parameter_type_names),
        parameter_names_(parameter_names),
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
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
    // TODO 20261025 insert default parameter names when none specified.
    // String names for all parameters of this GpFunction.
    const std::vector<std::string>& parameterNames() const
    { return parameter_names_; }
    //~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~~ ~~
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
    // Evaluate (execute) a GpTree with this function at root
    // TODO probably should assert they match (this and tree root GpFunction)
    std::any eval(GpTree& tree) const { return eval_(tree);  }
    // Called by FunctionSet init while linking types and functions.
    void linkToReturnType()
    {
        returnType()->addFunctionReturningThisType(this, name());
    }

    // Multiplier on random selection during initial tree construction.
    float selectionWeight() const { return selection_weight_; }
    
    static VecOfStrings default_param_names(const VecOfStrings& types)
    {
        VecOfStrings vos;
        for (int i = 0; i < types.size(); i++) { vos.push_back(intToStr(i)); }
        return vos;
    }

    // Return a unique string for any non-negative int (from optional alphabet).
    // (Should this be moved from this class header file to a utility library?)
    static const std::string intToStr(int n)
    {
        return intToStr(n, "abcdefghijklmnopqrstuvwxyz");
    }
    static const std::string intToStr(int n, const std::string& alphabet)
    {
        std::string result;
        assert(n >= 0);
        int dup = n / alphabet.size();
        std::string letter = alphabet.substr(n % alphabet.size(), 1);
        for (int i = 0; i < dup + 1; i++) { result += letter; }
        return result;
    }
    // Print description of this GpFunction to std::cout.
    void print() const
    {
        bool comma = false;
        size_t parameter_count = parameterTypeNames().size();
        bool one_liner = parameter_count < 5;
        size_t col_width = 0;
        for (int i = 0; i < parameter_count; i ++)
        {
            col_width = std::max(col_width, parameterTypeNames().at(i).size()+3);
        }
        std::cout << "GpFunction: " << name() << ", return_type: ";
        std::cout << returnTypeName() << ", ";
        std::cout << parameterTypeNames().size() << " parameters:";
        std::cout << (one_liner ? " (" : "\n");
        for (int i = 0; i < parameter_count; i ++)
        {
            if (comma) {std::cout<<(one_liner?", ":"\n");} else {comma = true;}
            std::string pad_tn = parameterTypeNames().at(i);
            if (one_liner)
            {
                pad_tn += " ";
            }
            else
            {
                while (pad_tn.size() < col_width - 1) { pad_tn += " "; }
            }
            std::cout << "(" << pad_tn;
            std::cout << parameterNames().at(i) << ")";
        }
        if (one_liner) { std::cout << ")."; }
        std::cout << std::endl;
    }
private:
    std::string name_;
    std::string return_type_name_;
    GpType* return_type_ = nullptr;
    std::vector<std::string> parameter_type_names_;
    std::vector<std::string> parameter_names_;
    std::vector<GpType*> parameter_types_;
    int min_size_to_terminate_ = std::numeric_limits<int>::max();
    std::function<std::any(GpTree& t)> eval_ = nullptr;
    float selection_weight_ = 1;
};

}  // end of namespace LazyPredator
