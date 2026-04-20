# EvoFlock

## Evolution Flocking

April 20, 2026<br>
Craig Reynolds<br>
cwr@red3d.com

This is experimental code, described in a draft paper **EvoFlock: evolved inverse design of multi-agent motion**, currently under review.

The abstract of the abstract is that _multi-agent motion models_ (like _boids_) can be hard to adjust. They have many (~15) parameters which interact in nonlinear ways.

This work takes an _inverse design_ approach using optimization (via a genetic algorithm) according to an objective/fitness/loss function (written by the user).

For now I treat the code in this repository as a private branch: it is often broken, or modified for debugging. If you are interested in building/using it — thanks for your interest! — please send me email so I can get you a clean version.

Just a few hints:

* _EvoFlock_ uses an evolutionary computation engine developed for another project called  _LazyPredator_.
    * Its source code is in a subdirectory also called _LazyPredator_.
    * _LazyPredator_ normally supports _genetic programming_, for which a _genetic algorithm_ can be implemented as a special case.
* _EvoFlock_ has several versions and modes of operation, all folded into a single code base using mode switches which are defined in `EvoFlock.h`. The progression over time was:
    * Use GA to find ParameterSet for flocking model according to three objectives: correct separation from nearest neighbor, avoid obstacles, maintain target speed range.
    * Add an objective to increase each boid's path curvature (`EF::use_curvature_objective`).
    * Switch between GA and GP (`EF::usingGP()`, `EF::usingGA()`).
    * Switch between “flocking with obstacle” and experiments with murmurations (`EF::murmuration_mode`).
* Primary evolution and simulation parameters:
    * Evolutionary population: 300
    * Evolutionary steps: 30000 (100 “generations”)
    * Boids per flock: 200
    * Steps per flock simulation: 500
