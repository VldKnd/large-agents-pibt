import argparse
import math
import os
from calendar import c
from gc import collect

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation, collections
from utils import parse_tuples

parser = argparse.ArgumentParser(
    prog='PROG',
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)

parser.add_argument('-s', '--solution', type=str, default='result.txt', help='Path to the solution in txt format.')
parser.add_argument('-c', '--color', type=int, default=1, help='Wether to use colors for agents or not. The colors are deactivated if the argument is set to 0')
parser.add_argument('-t', '--type', type=str, default="circle", help='Wether to show circle or square agents.')
parser.add_argument('-a', '--add_steps', type=int, default=10, help='Additional steps after the program have been finished.')
parser.add_argument('-f', '--filename', type=str, default="", help='Name to save file in videos folder')
parser.add_argument('-v', '--verbose', type=int, default=0, help='Whether to print basic info about the solution')
parser.add_argument('-l', '--title', type=int, default=0, help='Whether to add title to the solution or not')
parser.add_argument('-p', '--frame_speed', type=int, default=50, help='Whats the length of the wait between frames')

args = parser.parse_args()

frame_speed = args.frame_speed
solution_name = args.solution
assert solution_name in set(os.listdir("./solutions/")), 'Solution name is invalid, tha solution file does not exists in pibt_visualizer/solutions/ folder'

colorful = args.color != 0
assert colorful in [False, True], 'The color argument is bool value'

visualisation_type = args.type
if visualisation_type not in ["circle", "square"]:
    assert False, f"Type can only be circle or square. Found {visualisation_type}"

time_steps_at_the_end = args.add_steps
assert (type(time_steps_at_the_end) is int) and (time_steps_at_the_end >= 0), 'The additional timestep value has to be positive integer'

file_name = args.filename
if file_name:
    assert (file_name[-4:] == ".gif"), "The only supported format is gif, please specify it at the end of the file"

solution = {}
save_output_path = "videos/"
solution_path = "solutions/"

with open(solution_path + solution_name, 'r') as file:
    for line in file.read().split('\n'):
        if '=' in line:
            key, value = line.split('=')
            solution[key] = value
        elif ':' in line:
            key, value = line.split(':')
            solution[key] = value
            
not_keys = [
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
]

map_name = solution['map_file']

radiuses = np.array([float(r) for r in solution['radiuses'].split(',')])
    

for key in solution.keys():
    if key not in not_keys:
        solution[key] = parse_tuples(solution[key])
        
keys = list(solution.keys())
for key in keys:
    try:
        solution[int(key)] = solution[key]
        del solution[key]
    
    except ValueError:
        pass
    
print("MakeSpan: ", solution["makespan"])
for i in range(0, int(solution["makespan"])):
    assert len(solution[i]) == len(solution[i+1]), i

print('Number of timsteps :', len(solution))
print('All checked passed, the amount of agents is {}'.format(len(solution[0])))

with open("map/"+map_name, 'r') as file:
    map_text = file.read().split("\n")
    
for i, line in enumerate(map_text):
    if '@' in line or '.' in line or 'T' in line:
        map_text = map_text[i:]
        break
    
coordinates_map = []
for i, line in enumerate(map_text):
    for j, letter in enumerate(line):
        if letter == "T" or letter == "@":
            coordinates_map.append((j, i))
            
if len(coordinates_map):
    coordinates_map = np.stack(coordinates_map)
else:
    coordinates_map = np.array([])

t_begin, t_end = 0, max([x if type(x) is int else -1 for x in solution.keys()])
coordinates = np.stack([np.stack(solution[i]) for i in range(t_end+1)])
goals = np.stack(solution["goals"])
starts = np.stack(solution["starts"])

map_colors = (0, 0, 0, 1)

if colorful:
    agents_colors = np.append(
        np.random.rand(coordinates.shape[1], 3),
        np.ones((coordinates.shape[1], 1)),
    axis=1)
else:
    agents_colors = np.append(
        np.zeros((coordinates.shape[1], 3)),
        np.ones((coordinates.shape[1], 1)),
    axis=1)
    
if time_steps_at_the_end:
    coordinates = np.append(coordinates, coordinates[-1:, :, :].repeat(time_steps_at_the_end, axis=0), axis=0)
    t_end += time_steps_at_the_end
    
if (len(coordinates_map)):
    min_x, min_y = np.stack([coordinates_map.min(0),  coordinates.min((0, 1)), starts.min(0), goals.min(0)]).min(0)
    max_x, max_y = np.stack([coordinates_map.max(0),  coordinates.max((0, 1)), starts.max(0), goals.max(0)]).max(0)
else:
    min_x, min_y = np.stack([coordinates.min((0, 1)), starts.min(0), goals.min(0)]).min(0)
    max_x, max_y = np.stack([coordinates.max((0, 1)), starts.max(0), goals.max(0)]).max(0)

size = 6
offset = math.ceil(max(radiuses)) + 2
scale = (2*offset+max_y - min_y) / (2*offset+max_x - min_x)
min_, max_ = min(min_x, min_y), max(max_x, max_y)

figure_size = (size, size*(scale))
fig = plt.figure(figsize=figure_size)
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

# starts_scatter = ax.scatter(starts[:, 0], starts[:, 1], marker="+", color=agents_colors, s=12, zorder = 2)
# goals_scatter = ax.scatter(goals[:, 0], goals[:, 1], marker="x", color=agents_colors, s=12, zorder = 2)

obj_col = collections.PatchCollection([
    plt.Rectangle(pos, 1, 1) for pos in coordinates_map - 0.5
])

obj_col.set_color(map_colors)
ax.add_collection(obj_col)


if visualisation_type == 'circle':
    circles = [plt.Circle(pos, r, color=c, zorder=10) for pos, r, c in zip(coordinates[0], radiuses, agents_colors)]

    for circle in circles:
        ax.add_patch(circle)

    def animate(k):
        i = k

        for (pos, circle) in zip(coordinates[i], circles):
            circle.center = pos

        if args.title:
            ax.set_title(map_name + "\n t : {}".format(k))

    ani = animation.FuncAnimation(
        fig, animate, frames=t_end, interval=frame_speed)

    if file_name:
        writervideo = animation.FFMpegWriter()
        ani.save(save_output_path + file_name)

else:
    circles = [plt.Rectangle(pos, r, r, color=c, zorder=10) for pos, r, c in zip(coordinates[0], radiuses, agents_colors)]

    for circle in circles:
        ax.add_patch(circle)

    def animate(k):
        i = k

        for (pos, circle) in zip(coordinates[i], circles):
            circle.xy = pos

        if args.title:
            ax.set_title(map_name + "\n t : {}".format(k))

    ani = animation.FuncAnimation(
        fig, animate, frames=t_end, interval=frame_speed)

    if file_name:
        writervideo = animation.FFMpegWriter()
        ani.save(save_output_path + file_name)

plt.show()