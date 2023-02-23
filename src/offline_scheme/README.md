<img src="../../figures/offline_scheme.png" width="60%">

# Initial Solver Usage
The Initial Solver involves a Google OR-Tools and step reduction loop. Initial Solver aims at providing an initial solution for further optimization. This initial solution should be the first feasible solution in step reduction loop. The source code takes few input arguments:
```
sudo cpp_ortools <1> <2> <3> <4> <5> <6> <7> <8> <9>
``` 
1. input file directory
2. output file directory
3. input instance file name
4. output initial solution file name
5. developing, always set it to 0
6. minimum allowed UAV energy
7. apply uniform penalty for dropping a node or not [0 or 1]
8. run the Initial Solver from scratch or not [0 or 1]
9. use speedup strategy or not [0 or 1]

Note that option 8 and option 9 **cannot** be set as 1 at the same time. 

# Optimization Solver Usage
The Optimization Solver mainly involves the Context-aware Black Hole Algorithm. Optimization Solver aims at optimizing the given initial solution with certain fitness metric. The source code takes few input arguments:
```
mpiexec -n 4 cpp_bha <1> <2> <3> <4> <5> <6> <7> <8> <9> <10>
```
1. input file directory
2. output file directory
3. input instance file name
4. input initial solution file name
5. developing, always set it to 0
6. weight coefficient for energy [0, 100]
7. number of populations
8. number of iterations
9. attraction probability [0, 1]
10. number of searched candidates

# Input file format
## Sensor node deployment file
The sensor node deployment file ***for offline scheme*** should have a format like below:

| id  | x_pos | y_pos  | z_pos | p_flag | volts | weights |
|-----|-------|--------|-------|--------|-------|---------|
| -1  | 0     | 0      | 0     | 0      | 0     | 0       |
| 0   | 4029  | 155511 | 0     | 0      | 1.686 | 6       |
| ... | ...   | ...    | ...   | ...    | ...   | ...     |
| -1  | 0     | 0      | 0     | 0      | 0     | 0       |

Note that the column name should be same as above table, i.e. `id,x_pos,y_pos,z_pos,p_flag,volts,weights`:
* id: Sensor node ID
* x_pos: The x-coordinate of the sensor node (cm)
* y_pos: The y-coordinate of the sensor node (cm)
* z_pos: The z-coordinate of the sensor node (cm)
* p_flag: The sensor type, check [our paper](https://arxiv.org/abs/2203.04595) for more details
* volts: The current voltage of the sensor node (V)
* weights: The prize weight of this sensor node

The start node (first `id = -1`) and end node (last `id = -1`) are two nodes only for recording the UAV start and end coordinate. Thus, the columns of `p_flag`, `volts` and `weights` should be 0.

## Wind grid file
The wind grid file (default file name "grid_info.txt") is for recording the resolution of the wind grid. It should have a format like below:
```
0,0,0,0,2500,2500,400,10000,82,82,5,361
```
Note that "grid_info.txt" only exists a single-row data, there is no column name. From **left to right**, the meaning of each column refers to:
* start x-coordinate (cm)
* start y-coordinate (cm)
* start z-coordinate (cm)
* start time (ms)
* step size of x-coordinate (cm)
* step size of y-coordinate (cm)
* step size of z-coordinate (cm)
* time step (ms)
* number of xs steps 
* number of ys steps 
* number of zs steps
* number of time steps

## Wind vector file
The wind vector file (default file name "wind_vector.csv") is for recording the wind vector of each vertex of each wind cube in the wind grid. It should have a format like below:

| ts      | uv_x | uv_y | uv_z |
|---------|------|------|------|
| 0       | -300 | -200 | 0    |
| ...     | ...  | ...  | ...  |
| 10000   | -300 | -200 | 0    |
| ...     | ...  | ...  | ...  |
| 3600000 | -300 | -200 | 0    |

The above table is an example for the grid resolution mentioned in above Wind grid file section. Note that the column name should be same as above table, i.e. `ts, uv_x, uv_y, uv_z`:
* ts: time
* uv_x: wind speed at x direction (cm/s)
* uv_y: wind speed at y direction (cm/s)
* uv_z: wind speed at z direction (cm/s)

## Solution file (Initial Solver) 
The initial solution file involves the sequence of visiting order for the UAV. It should have a single-row data that has below example format:
```
0,1,2,3,4
```
exclude start node and end node. Because the initial solution file is an input file for Optimization Solver, it is placed in the **specified input directory**. 

# Output file format
## Execution time file (Initial Solver)
The Initial Solver execution time file (default name "or_time.txt") is for recording the execution time of the Initial Solver. It should have a single-row data with 
```
timestamp,single_algorithm_execution_time,total_program_execution_time
```
where time unit is second.

## Execution time file (Optimization Solver)
The Optimization Solver execution time file (default name "comp_time.txt") is for recording the execution time of the Optimization Solver. It should have a single-row data with 
```
timestamp,fitness_metric,algorithm_execution_time
```
where time unit is second.

## Solution file (Optimization Solver)
The final solution file (default name "plan.txt") involves the sequence of visiting order for the UAV. It should have a single-row data that has below example format:
```
timestamp,0,1,2,3,4
```

## Execution summary file (Optimization Solver)
The execution summary file (default name "summary.csv") involves the summary of UAV discharged and recharged energy, and some metrics. It should have a single-row data with below format:

| Timestamp | M_{init} | M_{opt} | DE_{init} | DE_{opt} | DE_{all} | RE_{init} | RE_{opt} | RE_{all} |
|-------------|----------|---------|-----------|----------|----------|-----------|----------|----------|

where:
* Timestamp: the timestamp of this execution
* M_{init}: The fitness metric value of Initial Solver's route
* M_{opt}: The fitness metric value of Optimization Solver's route
* DE_{init}: UAV discharged energy of Initial Solver's route
* DE_{opt}: UAV discharged energy of Optimization Solver's route
* DE_{all}: UAV initial battery level
* RE_{init}: UAV recharged energy of Initial Solver's route
* RE_{opt}: UAV recharged energy of Optimization Solver's route
* RE_{all}: All rechargeable energy of the whole sensor network

## Simulation result file (Optimization Solver)
The simulation result file (default name "simu_res.txt") involves the UAV remaining energy level, flight time, and collected prize.
```
Timestamp
52,17,28,46,45,16,...
68.6746,64.7925,63.2895,60.3867,58.6868,57.1167,...
46.817,115.585,142.105,193.271,223.294,251.01,...
8,7,8,9,8,7,...
```
where the 2nd row is the sequence of visiting order; 3rd row is the remaining energy after charging a node; 4th row is the departure time before next visit; last row is the prize of the visited sensor node.
