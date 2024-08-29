# LazyPredator
## Genetic programming, negative selection, genetic drift.

**LazyPredator** is a framework for genetic programming (or genetic algorithm) with an emphasis on negative selection, genetic drift, and tournament-based relative (or absolute) fitness. It is intended for population-based evolutionary optimization. Unlike gradient descent methods, evolutionary computation is a “gradient free” method so does not require differentiablity.

It is written in C++ and structured as a “header only” library. Simply copy (or `git clone`) its source files and include `LazyPredator.h` in your code.

LazyPredator supports genetic programming (GP: [[Koza 1992]](https://mitpress.mit.edu/9780262527910/genetic-programming/), [[Cramer 1985]](http://gpbib.cs.ucl.ac.uk/gp-html/icga85_cramer.html))
more specifically, the varient called strongly typed genetic programming (STGP: [[Montana 1995]](http://gpbib.cs.ucl.ac.uk/gp-html/montana_stgpEC.html)).

LazyPredator was initially used to optimize programs to perform procedural texture synthesis based on the [TexSyn](https://cwreynolds.github.io/TexSyn/) library. That combination of LazyPredator and TexSyn was a large component of the research project described in the paper [Coevolution of Camouflage](https://arxiv.org/abs/2304.11793) published at the Artificial Life Conference 2023, in Sapporo, Japan, then in a poster at SIGGRAPH 2023 in Los Angeles, USA.

Initial development of LazyPredator was from August 2020 to March 2023 during the TexSyn project. Then in January 2024 a new phase of development began on a copy of LazyPredator inside another repository. When that code is stable, it will be merged back into this main repository.

That newer LazyPredator “2.0” adds support for _multi-objective fitness_ values and evolution with such values toward Pareto optimality. LP2 also introduces support for _genetic algorithms_ (where the genome is a fixed length vector of scalar values) in addition to _genetic programming_ (where the genome is is a varible-sized tree structure). This is currently just an example of a simple transformation of a genetic algorithm problem into an equivalent genetic programming problem. It maybe better packaged in the future.

There is a bit of a [LazyPredator development notebook](https://cwreynolds.github.io/LazyPredator/). Much deeper discussion of its initial use for the camouflage project is in [TexSyn's devo blog](https://cwreynolds.github.io/TexSyn/).

Please contact the [author](https://github.com/cwreynolds) if you have questions about using LazyPredator.

August 29, 2024.
