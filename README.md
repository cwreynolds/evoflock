# EvoFlock

## Evolution Flocking

April 20, 2026<br>
Craig Reynolds<br>
cwr@red3d.com

This is experimental code, described in a draft paper **EvoFlock: evolved inverse design of multi-agent motion**, currently under review.

The abstract of the abstract is that _multi-agent motion models_ (such as _boids_) can be hard to adjust. They have many (~15) parameters which interact in nonlinear ways.

This work takes an _inverse design_ approach using optimization (via a genetic algorithm) according to an objective function (written by the user).

So far, I have treated this code as a private branch: it is often broken, or modified for debugging. If you are interested in building/using it, (a) thanks for your interest!, and (b) please send me email so I can help you get a clean version. If you do try, please let me know how you fare so I can make this code easier to use. For example, I think it has only been run on macOS.

**Just a few hints:**

* Build **EvoFlock** using either:
    * `CMake` on `CMakeLists.txt`
    * `Xcode` on `evoflock.xcodeproj`
* Dependencies (these should be formalized as `git` submodule dependancies):
    * **EvoFlock** uses an evolutionary computation engine, called **LazyPredator**, developed for an earlier project.
      * Its updated source code is in a subdirectory: `EvoFlock/LazyPredator/`.
      * _LazyPredator_ supports _genetic programming_ (GP). A _genetic algorithm_ (GA) can be considered a special case of GP.
    * The [Open3D](https://www.open3d.org/) library, a modern layer on top of OpenGL, is used for graphics. You will need to install this separately.
* Run either from `Xcode` or with the `evoflock` executable from the command line.
    * The normal action is to start optimization run, finding a solution for the given objective. For a GA run this will be a set of scalar values in a `ParameterSet` object, or in the experimental GP mode, the result will be “source code” in a domain specific language.
    * Otherwise if `EF::visualize_previous_results_mode` is `true` will run flock simulations based on results from a previous optimization run.
    * Normally an `Open3D` window will open and logging will appear on the shell where `evoflock` was run.
        * A simple user interface is available in the `Open3D` window
        * The mouse can be used to adjust the “camera” (“point of view”).
        * A few single characters commands exist. For example type an "H" to the `Open3D` window to print a list of currently defined command. See bottom of this page. The “B” key command can be used during an optimization run to check on its progress. The optimization is paused, a flock simulation is run with the _best_ solution found so far, then the optization resumes.
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

```
EvoFlock: list of single key commands.
    H  print this list of single key commands.
    G  toggle "graphics mode".
    C  cycle through camera aiming modes.
       [space] toggle simulation pause.
    O  cycle through predefined obstacle sets.
    1  single step mode (advance one simulation step, then pause).
    S  cycle selected boid through flock.
    A  toggle drawing of annotation lines, etc.
    R  reset camera to aligned view of whole scene.
    B  pause, run sim with best individual & graphics, then proceed.
    W  toggle "space-time worms".
```
