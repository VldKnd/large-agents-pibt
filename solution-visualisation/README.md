# LaPIBT Visualisation

Simple visualisation for PIBT solutions, using matplotlib animation library.

## Install Dependencies
We use [poetry](https://python-poetry.org/docs/) to manage python dependencies.

To install dependencies run:
```
poetry lock && poetry install
```

Main code is in visualize.py
```
python visualize.py -h

usage: PROG [-h] [-m MAP] [-s SOLUTION] [-c COLOR] [-t TRACES] [-a ADD_STEPS] [-f FILENAME]

optional arguments:
  -h, --help            show this help message and exit
  -m MAP, --map MAP     Name of the map, for which the solution will be provided. (default: arena.map)
  -s SOLUTION, --solution SOLUTION
                        Path to the solution in txt format. (default: result.txt)
  -c COLOR, --color COLOR
                        Wether to use colors for agents or not. The colors are deactivated if the argument is set to 0 (default: 1)
  -t TYPE, --type TYPE
                        Type of agents used in solution, it can be either "circle" or "square".
                        (default: "cirlce")
  -a ADD_STEPS, --add_steps ADD_STEPS
                        Additional steps after the program have been finished. (default: 10)
  -f FILENAME, --filename FILENAME
                        Name to save file in videos folder (default: )
```
To run code use:
```bash
$ poetry run python visualize.py [OPTIONS]
```
For example:
```bash
$ poetry run python visualize.py -s solution.txt -t square
```
