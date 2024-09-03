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

namespace LazyPredator
{
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TODO 20240903 tiny step toward refactoring LP's unit tests. ("it would be
// nice, but not necessary, to recast LazyPredator/UnitTests.h to the New Way,
// with a unit_test() method on each class, as a series of asserts.") This is
// to allow using LP::unit_test() as the name for the approach going forward
// using New Way unit tests.

static void unit_test()
{
    MultiObjectiveFitness::unit_test();  // Modern class with New Way unit tests
    legacy_unit_test();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}
