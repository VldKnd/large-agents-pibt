**Here is an example of algorithm solutions:**

<details>
<summary>Paris_1_256.map Solved by LaPIBT!</summary>
<br/>
<div class="image-container">
    <img style="display: none;" id="spinner" src="https://github.com/VldKnd/large-agents-pibt/blob/main/square-large-agents-mapf/readme_example.gif"/>
</div>  
</details>

## Building Code:
We use CMake to build code. To create your own executable file, create a build directory and build code from there, e.g.
```bash
$ mkdir build
$ cd build
$ cmake ..
$ make
```
This will create the `large-agents-mapf` file in the `/build/` folder.

## Running Code:
The executable file accepts the following parameters:

```
-i --instance [FILE_PATH]     instance file path
-o --output [FILE_PATH]       output file path
-v --verbose                  print additional info
-h --help                     help
-s --solver [SOLVER_NAME]     solver (LAPIBT)
-T --time-limit [INT]         max computation time (ms)
-L --log-short                use short log
-P --make-scen                make scenario file using random starts/goals
-D --inheritanceDepth [INT]   inheritanceDepth of LA-PIBT
-x --seed [INT]               random generator seed (only used when not set in the instance file)
```
**However**, most of them can be specified in the test case file and are not necessarily passed to the exec file. Typically, the execution of the solver will look like:
```bash
$ ./large-agents-mapf -i ${PATH_TO_TEST_CASE} -s LAPIBT -o ${PATH_TO_SAVE_OUTPUT_RESULTS} -v
```

## Writing Test Case:
Test cases are parsed with regex. Examples of existing test cases can be found in:
```bash
${PROJECT_BASE_PATH}/square-large-agents-mapf/tests/problems/
```

Test cases have the following options:
```
Option:
    # Leaving a comment
Desc. :
    Option to leave a comment in a file
```
```
Option:
    map_file='path/to/map/file.map'
Desc. :
    Path to the file with information about the map in .map format
```
```
Option:
    agents=1
Desc. :
    Number of agents to be used in a problem. If larger than given radiuses, the algorithm adds random agents to the problem.
```
```
Option:
    well_formed=1
Desc. :
    Whether to check if the goal is accessible for every agent before starting the algorithm. This is very useful in a map with a lot of narrow passages.
```
```
Option:
    sizes=1.,2.3,1.5
Desc. :
    Sizes of agents. sizes_random_uniform= can be passed instead.
```
```
Option:
    sizes_random_uniform=1.,3.
Desc. :
    Instead of passing sizes=, this option can be used to create random sizes of agents in a uniform manner. Numbers represent the lower and upper bounds of the distribution range.
```
```
Option:
    seed=1
Desc. :
    Random seed to use in the creation of the problem.
```
```
Option:
    random_problem=1
Desc. :
    Whether to create a random problem or not. If this is set to 1, it skips reading initial goal end positions and creates them randomly.
```
```
Option:
    max_timestep=1000
Desc. :
    Maximum allowed number of timesteps. If it is reached, the algorithm stops.
```
```
Option:
    max_comp_time=10000
Desc. :
    Maximum allowed computation time in milliseconds. If it is reached, the algorithm stops.
```
```
Option:
    8,8,4,8
Desc. :
    Declaring start - end position of a robot in x_start,y_start,x_goal,y_goal format.
```

So, a typical test file will look something like the following:
```text
map_file=16x16.map
agents=2
sizes=3., 1.
seed=0
random_problem=0
max_timestep=2000
max_comp_time=5000
4,8,8,8
8,8,4,8
```
