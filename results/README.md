# Experiment Design and Visualization
Please check [our paper](https://arxiv.org/abs/2203.04595) for experiment details.

## OR-Tools metaheuristic options
In Google OR-Tools, there are many [options](https://developers.google.com/optimization/routing/routing_options) to search a solution. Meta-heuristics options were tested in this project, including `AUTOMATIC`, `GREEDY_DESCENT`, `GUIDED_LOCAL_SEARCH`, `SIMULATED_ANNEALING`, and `TABU_SEARCH`. More details can be seen in [or-metaheuristic-options](or-metaheuristic-options).

## Parallelization performance analysis
To figure out what is the most "efficient" parameter setting in a general situation, we tested the algorithm execution with increasing number of populations and fixed number of generations. More details can be seen in [parallelization](parallelization) There are two files: `100to1000.txt` and `1000to10000.txt`.

## Dynamic execution with twice energy updates
In this experiment, one OP case were tested with dynamic changes of UAV remain energy with different searching strategies. More details can be seen in [dynamic-energy-change](dynamic-energy-change).

## Iterative execution with increasing sensor node numbers
In this experiment, 15 cases with increasing numbers of nodes (from 10 to 150) in fixed field scale were tested. This experiment aims at testing in these scenarios, how many UAVs will be needed to charge all nodes. More details can be seen in [increasing-nodes-charge-all](increasing-nodes-charge-all).

## Wind impact on solution performance
In this experiment, 100 TSP cases were generated to test wind effect on solution improvement. More details can be seen in [wind-impact](wind-impact).



