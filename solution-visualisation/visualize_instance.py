from calendar import c
from gc import collect
from matplotlib import animation
import matplotlib.pyplot as plt
from matplotlib import collections
from utils import parse_tuples
import numpy as np
import argparse
import os

if False:
    parser = argparse.ArgumentParser(
        prog='PROG',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('-i', '--instance', type=str, default='', help='Name of the instance to visualize.')
    parser.add_argument('-c', '--color', type=int, default=1, help='Wether to use colors for agents or not. The colors are deactivated if the argument is set to 0')
    parser.add_argument('-f', '--filename', type=str, default="", help='Name to save image of starting positions in video folder')
    args = parser.parse_args()
else:
    class Parser:
        def __init__(self):
            self.instance='1.txt'
            self.color=1
            self.filename=""
            
    args = Parser()

instance_name = args.instance # arena.map # Paris_1_256.map # brc202d.map
assert instance_name in set(os.listdir("./solutions/instances")), 'Instance name is invalid, or it is not in pibt_visualizer/solutions/instances folder'

colorful = args.color != 0
assert colorful in [False, True], 'The color argument is bool value'

file_name = args.filename

instance = {}
save_output_path = "videos/pos"
instance_path = "solutions/instances/"
config_s = []
config_g = []

with open(instance_path + instance_name, 'r') as file:
    for line in file.read().split('\n'):
        if '=' in line:
            key, value = line.split('=')
            instance[key] = value
        elif ':' in line:
            key, value = line.split(':')
            instance[key] = value
        elif len(line.split(',')) == 4:
            x_s, y_s, x_g, y_g = line.split(',')
            config_s.append((int(x_s), int(y_s)))
            config_g.append((int(x_g), int(y_g)))
            
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

map_name = instance['map_file']
radiuses = np.array([float(r) for r in instance['radiuses'].split(',')])
assert len(radiuses) == len(config_s) and len(radiuses) == len(config_g), 'Config file is not well formated'

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
            
coordinates_map = np.stack(coordinates_map)
starts = np.stack(config_s)
goals = np.stack(config_g)

map_colors = (0, 0, 0, 1)

if colorful:
    agents_colors = np.append(
        np.random.rand(starts.shape[0], 3), 
        np.ones((starts.shape[0], 1)), 
    axis=1)
else:
    agents_colors = np.append(
        np.zeros((starts.shape[0], 3)), 
        np.ones((starts.shape[0], 1)), 
    axis=1)

min_x, min_y = np.stack([coordinates_map.min(0),  starts.min(0), goals.min(0)]).min(0)
max_x, max_y = np.stack([coordinates_map.max(0),  starts.max(0), goals.max(0)]).max(0)

min_, max_ = min(min_x, min_y), max(max_x, max_y)

map_size_inch = 8

figure_size = (map_size_inch, map_size_inch)
fig = plt.figure(figsize=figure_size)
ax = fig.add_subplot(autoscale_on=False, xlim=(min_ - 2, max_+2), ylim=(min_-2, max_+2))

scale_r = map_size_inch/(max_ - min_)
ax.grid(False)
# ax.axis("off")
ax.set_title(map_name)

obj_col = collections.PatchCollection([
    plt.Rectangle(pos, 1, 1) for pos in coordinates_map - 0.5
])
obj_col.set_color(map_colors)
ax.add_collection(obj_col)

circles = [plt.Circle(pos, r, color=c) for pos, r, c in zip(starts, radiuses, agents_colors)]
#circles_col = collections.PatchCollection(circles)
#circles_col.set_color(agents_colors)
#circles_col.set_offset_transform(ax.get_transform())
#ax.add_collection(circles_col)
for circle in circles:
    ax.add_patch(circle)


circles = [plt.Circle(pos, r, color=c, fill=False) for pos, r, c in zip(goals, radiuses, agents_colors)]
for circle in circles:
    ax.add_patch(circle)

plt.show()
# t_begin, t_end = 0, max([x if type(x) is int else -1 for x in solution.keys()])
# coordinates = np.stack([np.stack(solution[i]) for i in range(t_end+1)])
# goals = np.stack(solution["goals"])
# starts = np.stack(solution["starts"])
# radiuses = np.random.rand(coordinates.shape[1])*1.5
    
# if time_steps_at_the_end:
#     coordinates = np.append(coordinates, coordinates[-1:, :, :].repeat(time_steps_at_the_end, axis=0), axis=0)
#     t_end += time_steps_at_the_end
    
# obstacle_size = int((figure_size[0]/( max_x - min_x + 4 ))*72)
# obstacle_marker = None
   

# starts_scatter = ax.scatter(starts[:, 0], starts[:, 1], marker="+", color=agents_colors, s=12)
# goals_scatter = ax.scatter(goals[:, 0], goals[:, 1], marker="x", color=agents_colors, s=12)

# if add_traces:
#     traces = []
#     for j in range(coordinates.shape[1]):
#         tmp, = ax.plot([], [], linewidth=0.5, ls="--", color=agents_colors[j])
#         traces.append(tmp)
    

# def animate(i):
#     if add_traces:
#         for j in range(coordinates.shape[1]):
#             traces[j].set_data(coordinates[:i+1, j, 0], coordinates[:i+1, j, 1])

# #    circles_col.set_offsets(coordinates[i] - coordinates[0])
#     for (pos, circle) in zip(coordinates[i], circles):
#         circle.center = pos

# ani = animation.FuncAnimation(
#     fig, animate, frames=t_end)

# if file_name:
#     writervideo = animation.FFMpegWriter()
#     ani.save(save_output_path + file_name)
