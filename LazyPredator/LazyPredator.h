//
//  LazyPredator.h
//  LazyPredator
//
//  TODO 20240224 this is an experimental 2.0 version of LazyPredator. It lives
//  inside the evoflock repository. The intent had been to use LazyPredator for
//  evoflock, but it turned out LazyPredator and TexSyn where a bit intertwined.
//  I was loath to make the fixes in LazyPredator's own repository since that
//  would break TexSyn.
//
//  So I am prototyping the changes here. They consist primarily of putting all
//  of LazyPredator into its own namespace, and giving LazyPredator a copy of
//  TexSyn's Utilities.h which it had previously only linked to.
//
//  Created by Craig Reynolds on 8/7/20.
//  Copyright © 2020 Craig Reynolds. All rights reserved.
//

#pragma once

#include "Population.h"
#include "UnitTests.h"

// TODO 20240224 very temp for testing
namespace LazyPredator
{

//    static void unit_test()
//    {
//        // Just testing that I can instantiate a functional LazyPredator class.
//        assert(Population().getMigrationLikelihood());
//    }

}
