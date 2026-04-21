# EvoFlock

## Evolution Flocking

April 20, 2026<br>
Craig Reynolds<br>
cwr@red3d.com

This is experimental code, described in a draft paper **EvoFlock: evolved inverse design of multi-agent motion**, currently under review.

The abstract of the abstract is that _multi-agent motion models_ (such as _boids_) can be hard to adjust. They have many (~15) parameters which interact in nonlinear ways.

This work takes an _inverse design_ approach using optimization (via a genetic algorithm) according to an objective function (written by the user).

So far, I have treated this code as a private branch: it is often broken, or modified for debugging. If you are interested in building/using it, (a) thanks for your interest!, and (b) please send me email so I can help you get a clean version. If you do try, please let me know how you fare so I can make this code easier to use. For example it has only been run on macOS.

**Just a few hints:**

* Dependencies (these should be formalized as `git` submodule dependancies):
    * **EvoFlock** uses an evolutionary computation engine, called **LazyPredator**, developed for an earlier project.
      * Its updated source code is in a subdirectory: `EvoFlock/LazyPredator/`.
      * _LazyPredator_ supports _genetic programming_ (GP). A _genetic algorithm_ (GA) can be considered a special case of GP.
    * The [Open3D](https://www.open3d.org/) library, a modern layer on top of OpenGL, is used for graphics. You will need to install this separately.
* _EvoFlock_ has several versions and modes of operation, all folded into a single code base using mode switches which are defined in `EvoFlock.h`. The progression over time was:
    * Use GA to find a `ParameterSet` for parametric flocking model according to three objectives: correct separation from nearest neighbor, obstacle avoidance, maintaining a target speed.
    * Add an objective to increase each boid's path curvature (`EF::use_curvature_objective`).
    * Switch between GA and GP (`EF::usingGP()`, `EF::usingGA()`) for ongoing experiments.
    * Switch between earlier “flocking with obstacle” version and experiments with murmurations (`EF::murmuration_mode`).
* Primary evolution and simulation parameters:
    * in `EF::runOneFlockEvolution()`:
        * Evolutionary population: 300
        * Evolutionary steps: 30000 (for SSGA, essentially 100 “generations”)
    * in `FlockParameters`:
        * Boids per flock: 200
        * Steps per flock simulation: 500
* I hope to soon convert this messy personal repository to be useful to others…
