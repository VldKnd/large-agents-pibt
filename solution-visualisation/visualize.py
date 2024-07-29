import argparse
import logging
import math
import os
from curses.textpad import rectangle
from typing import Any, Iterable, List

import matplotlib.animation
import matplotlib.collections
import matplotlib.patches
import matplotlib.pylab
import matplotlib.pyplot
import numpy as np
from utils import parse_tuples

LOGGER = logging.getLogger(__name__)
POSSIBLE_KEYS = {
    'instance',
    'agents',
    'map_file',
    'solver',
    'solved',
    'soc',
    'lb_soc',
    'makespan',
    'lb_makespan',
    'comp_time',
    'preprocessing_comp_time',
}



def parse_arguments() -> argparse.Namespace:
    """
    Parse command line arguments.

    Returns
    -------
    argparse.Namespace
        The parsed command line arguments.
    """
    parser = argparse.ArgumentParser(
        prog='PROG',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument('-s', '--solution', type=str, default='result.txt', help='Path to the solution in txt format.')
    parser.add_argument('-c', '--color', type=int, default=1, help='Whether to use colors for agents or not. The colors are deactivated if the argument is set to 0')
    parser.add_argument('-t', '--type', type=str, default="circle", help='Whether to show circle or square agents.')
    parser.add_argument('-a', '--add_steps', type=int, default=10, help='Additional steps after the program has finished.')
    parser.add_argument('-f', '--filename', type=str, default="", help='Name to save file in videos folder')
    parser.add_argument('-v', '--verbose', action='store_true', help='Whether to print basic info about the solution')
    parser.add_argument('-l', '--title', type=int, default=0, help='Whether to add title to the solution or not')
    parser.add_argument('-p', '--frame_speed', type=int, default=50, help='The length of the wait between frames')

    return parser.parse_args()

def check_arguments_validity(args: argparse.Namespace):
    """
    Check the validity of the command line arguments.

    Parameters
    ----------
    args : argparse.Namespace
        The parsed command line arguments.

    Raises
    ------
    AssertionError
        If any of the arguments are invalid.

    Notes
    -----
    This function checks the validity of the following arguments:
    - `args.solution`: Checks if the solution file exists in the `pibt_visualizer/solutions/` folder.
    - `args.color`: Checks if the `color` argument is a boolean value.
    - `args.type`: Checks if the `type` argument is either "circle" or "square".
    - `args.add_steps`: Checks if the `add_steps` argument is a positive integer.
    - `args.filename`: Checks if the `filename` argument ends with ".gif".
    """
    invalid_solution_error_message = (
     "Solution name is invalid,"
     "the solution file does not exists in pibt_visualizer/solutions/ folder"
    )
    assert args.solution in set(os.listdir("./solutions/")), invalid_solution_error_message

    invalid_color_flag_error_message = (
     'The color argument is bool value'
    )
    assert args.color != 0 in [False, True], invalid_color_flag_error_message

    invalid_visualisation_type_error_message = (
     f"Type can only be circle or square. Found {args.type}"
    )
    assert args.type in ["circle", "square"], invalid_visualisation_type_error_message
    
    invalid_timesteps_error_message = (
     "The additional timestep value has to be positive integer"
    )
    assert (type(args.add_steps) is int) and (args.add_steps >= 0), invalid_timesteps_error_message

    invalid_filename_error_message = (
        "The only supported format is gif,"
        "please specify it at the end of the file"
    )
    if args.filename:
        assert args.filename.endswith(".gif"), invalid_filename_error_message

if __name__ == "__main__":
    args = parse_arguments()
    check_arguments_validity(args)


    logging.basicConfig(
        format="[%(levelname)s] %(message)s",
        level=(logging.DEBUG if args.verbose else logging.INFO),
    )

    solution_options = {}
    solution_trajectories = {}
    save_output_path = "videos/"
    solution_path = "solutions/"
    solution_name = args.solution

    with open(solution_path + solution_name, 'r') as solution_file:
        for line in solution_file.read().split('\n'):
            if '=' in line:
                key, value = line.split('=')
                if key == "starts" or key == "goals":
                    solution_options[key] = parse_tuples(value)
                else:
                    solution_options[key] = value
            elif ':' in line:
                key, value = line.split(':')
                solution_trajectories[int(key)] = parse_tuples(value)

    map_name = solution_options['map_file']
    makespan = int(solution_options["makespan"])
    number_of_agents = len(solution_trajectories[0])
    sizes = np.array([float(r) for r in solution_options['sizes'].split(',')])
    for i in range(0, int(makespan)):
        assert len(solution_trajectories[i]) == len(solution_trajectories[i+1]), i

    LOGGER.info(f"MakeSpan: {makespan}")
    LOGGER.info(f"Number of timsteps: {len(solution_trajectories)}")
    LOGGER.info(f"All checked passed, the amount of agents is {number_of_agents}")

    with open(f"map/{map_name}", 'r') as map_as_file:
        map_as_lines_of_text = map_as_file.read().split("\n")
        
    for i, line_of_text in enumerate(map_as_lines_of_text):
        if (
            line_of_text.startswith('@') or
            line_of_text.startswith('.') or
            line_of_text.startswith('T')
        ):
            map_as_lines_of_text = map_as_lines_of_text[i:]
            break

    coordinates_of_obstacle_as_list_of_tuples = [
        (j, i)
        for i, line_of_text in enumerate(map_as_lines_of_text)
        for j, letter in enumerate(line_of_text)
        if letter == "T" or letter == "@"
    ]
                
    coordinates_of_obstacle_as_numpy_array = (
        np.stack(coordinates_of_obstacle_as_list_of_tuples)
        if coordinates_of_obstacle_as_list_of_tuples
        else np.array([])
    )

    agent_coordinates = np.stack(
        [
            np.stack(solution_trajectories[i])
            for i in range(makespan)
        ]
    )

    agent_goals = np.stack(solution_options["goals"])
    agent_starts = np.stack(solution_options["starts"])

    if args.color:
        agents_colors = np.append(
            np.random.rand(agent_coordinates.shape[1], 3),
            np.ones((agent_coordinates.shape[1], 1)),
        axis=1)
    else:
        agents_colors = np.append(
            np.zeros((agent_coordinates.shape[1], 3)),
            np.ones((agent_coordinates.shape[1], 1)),
        axis=1)
    
    if args.add_steps:
        coordinates = np.append(
            agent_coordinates,
            agent_coordinates[-1:, :, :].repeat(
                args.add_steps, axis=0
            ),
            axis=0
            )
        makespan += args.add_steps
        
    if len(coordinates_of_obstacle_as_numpy_array):
        min_x, min_y = np.stack([
            coordinates_of_obstacle_as_numpy_array.min(0),
            agent_coordinates.min((0, 1)),
            agent_starts.min(0),
            agent_goals.min(0)
        ]).min(0)
        max_x, max_y = np.stack([
            coordinates_of_obstacle_as_numpy_array.max(0),
            agent_coordinates.max((0, 1)),
            agent_starts.max(0),
            agent_goals.max(0)
        ]).max(0)
    else:
        min_x, min_y = np.stack([
            agent_coordinates.min((0, 1)),
            agent_starts.min(0),
            agent_goals.min(0)
        ]).min(0)
        max_x, max_y = np.stack([
            agent_coordinates.max((0, 1)),
            agent_starts.max(0),
            agent_goals.max(0)
        ]).max(0)

    size = 6
    offset = math.ceil(max(sizes)) + 2
    scale = (2*offset+max_y - min_y) / (2*offset+max_x - min_x)
    min_, max_ = min(min_x, min_y), max(max_x, max_y)

    figure_size = (size, size*(scale))
    fig = matplotlib.pyplot.figure(figsize=figure_size)
    ax = fig.add_subplot(autoscale_on=False,
        xlim=(min_x - offset, max_x + offset),
        ylim=(min_y - offset, max_y + offset)
    )
    ax.grid(False)
    ax.axis("off")
    if args.title:
        ax.set_title(map_name + "\n t : 0" )

    obstacle_size = int((figure_size[0]/( max_x - min_x + 4 ))*72)
    obstacle_marker = None

    obj_col = matplotlib.collections.PatchCollection([
        matplotlib.patches.Rectangle(coordinates_of_obstacle, 1, 1)
        for coordinates_of_obstacle in coordinates_of_obstacle_as_numpy_array - 0.5
    ])


    map_colors = (0, 0, 0, 1)

    obj_col.set_color(map_colors)
    ax.add_collection(obj_col)

    if args.type == 'circle':
        circles: List[matplotlib.patches.Circle] = [
            matplotlib.patches.Circle(pos, r, color=c, zorder=10)
            for pos, r, c in zip(coordinates[0], sizes, agents_colors)
        ]

        for circle in circles:
            ax.add_patch(circle)

        def animate(frame, *fargs) -> Iterable[Any]:
            i = frame

            for (pos, circle) in zip(coordinates[i], circles):
                circle.center = pos

            if args.title:
                ax.set_title(f"{map_name} t : {frame}")
            
            return ()
        
        ani = matplotlib.animation.FuncAnimation(
            fig=fig,
            blit=False,
            func=animate,
            frames=makespan,
            interval=args.frame_speed
        )

        if args.filename:
            writervideo = matplotlib.animation.FFMpegWriter()
            ani.save(save_output_path + args.filename)

    else:
        rectangles: List[matplotlib.patches.Rectangle] = [
            matplotlib.patches.Rectangle(pos, r, r, color=c, zorder=10)
            for pos, r, c in zip(coordinates[0], sizes, agents_colors)
        ]

        for rectangle in rectangles:
            ax.add_patch(rectangle)
            
        def animate(frame, *fargs) -> Iterable[Any]:
            i = frame

            for (pos, rectangle) in zip(coordinates[i], rectangles):
                rectangle.xy = pos

            if args.title:
                ax.set_title(f"{map_name} t : {frame}")
            
            return ()
    
        ani = matplotlib.animation.FuncAnimation(
            fig, animate, frames=makespan, interval=args.frame_speed)

        if args.filename:
            writervideo = matplotlib.animation.FFMpegWriter()
            ani.save(save_output_path + args.filename)

    matplotlib.pyplot.show()