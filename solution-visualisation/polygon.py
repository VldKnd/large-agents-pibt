import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

from utils import *


solution = {}
map_name = 'Paris_1_256.map'
solution_name = "paris_result.txt"


with open("solutions/" + solution_name, 'r') as file:
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
    
for i in range(1, 68):
    assert len(solution[i]) == len(solution[i+1]), i
    
print('All checked passed, the amount of agents is {}'.format(len(solution[i])))

with open("map/"+map_name, 'r') as file:
    map_text = file.read().split("\n")
    
for i, line in enumerate(map_text):
    if '@' in line or '.' in line or 'T' in line:
        map_text = map_text[i:]
        break
        
if len(map_text[-1]) == 0:
    map_text = map_text[:-1]
    
coordinates_map = []
for i, line in enumerate(map_text):
    for j, letter in enumerate(line):
        if letter == "T" or letter == "@":
            coordinates_map.append((j, i))
            
coordinates_map = np.stack(coordinates_map)


augmented_text = augment_border(map_text)
map_text_splitted = [[*augmented_text[i]] for i in range(len(augmented_text))]
borders = []
for i in range(len(map_text_splitted)):
    for j in range(len(map_text_splitted[0])):
        if map_text_splitted[i][j] != "X" and \
            map_text_splitted[i][j] != "B" and \
            map_text_splitted[i][j] != ".":
            borders.append(BFS(map_text_splitted, i, j))

            for y, x in borders[-1]:
                map_text_splitted[x][y] = "B"
                
set_of_nodes = []
for border in borders:
    nodes = set()
    for (x, y) in border:
        flag = ((x+1, y) in  border and (x, y+1) in  border) + \
               ((x+1, y) in  border and (x, y-1) in  border) + \
               ((x-1, y) in  border and (x, y+1) in  border) + \
               ((x-1, y) in  border and (x, y-1) in  border)
        
        if flag == 1 or flag == 4:
            nodes.add((x, y))
        
    set_of_nodes.append(nodes)
    
for nodes in set_of_nodes:
    for y, x in nodes:
        map_text_splitted[x][y] = "N"
        
set_of_ordered_nodes = []

for idx, border in enumerate(borders):
    border = borders[idx].copy()
    ordered_nodes = []

    try:
        y, x = next(iter(set_of_nodes[idx]))
    except StopIteration:
        continue
    
    ordered_nodes.append((y, x))
    border.remove((y, x))
    q = []

    for x_off, y_off in ([1, 0,],  [-1, 0], [0, 1], [0, -1]):
        if (y+y_off, x+x_off) in border:
                q.append((y, x, y_off, x_off))
                break

    while len(q) > 0:
        y, x, y_off, x_off = q.pop()

        while (y+y_off, x+x_off) in border:
            border.remove((y+y_off, x+x_off))
            x += x_off
            y += y_off

        if (y, x) in set_of_nodes[idx]:
            ordered_nodes.append((y, x))

        for x_off, y_off in ([1, 0,],  [-1, 0], [0, 1], [0, -1]):
            if (y+y_off, x+x_off) in border:
                q.append((y, x, y_off, x_off))

    set_of_ordered_nodes.append(ordered_nodes)
    for y, x in ordered_nodes:
        map_text_splitted[x][y] = "A"


colorful = True
add_traces = True
time_steps_at_the_end = 10

t_begin, t_end = 0, max([x if type(x) is int else -1 for x in solution.keys()])
coordinates = np.stack([np.stack(solution[i]) for i in range(t_end+1)])
goals = np.stack(solution["goals"])
starts = np.stack(solution["starts"])

map_colors = (0, 0, 0, 0.5)

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
    
min_x, min_y = np.min(coordinates, axis=(0, 1))
max_x, max_y = np.max(coordinates, axis=(0, 1))

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(autoscale_on=False, xlim=(min_x-2, max_x+2), ylim=(min_y-2, max_y+2))
ax.grid(False)
starts_scatter = ax.scatter(starts[:, 0], starts[:, 1], marker="+", color=agents_colors, s=12)
goals_scatter = ax.scatter(goals[:, 0], goals[:, 1], marker="x", color=agents_colors, s=12)

ax.set_facecolor(map_colors)
poly_coords = np.array(set_of_ordered_nodes[0])
ax.fill(poly_coords[:, 0]-2, poly_coords[:, 1]-2, color=(1., 1., 1.), zorder=1) 

for ordered_nodes in set_of_ordered_nodes[1:]:
    poly_coords = np.array(ordered_nodes)
    ax.fill(poly_coords[:, 0]-2, poly_coords[:, 1]-2, color=map_colors)    

if add_traces:
    traces = []
    for j in range(coordinates.shape[1]):
        tmp, = ax.plot([], [], linewidth=0.5, ls="--", color=agents_colors[j], zorder=2)
        traces.append(tmp)
    
trace_t_0 = ax.scatter(coordinates[0][:, 0], coordinates[0][:, 1], s=12, color=agents_colors, zorder=2)

def animate(i):
    if add_traces:
        for j in range(coordinates.shape[1]):
            traces[j].set_data(coordinates[:i+1, j, 0], coordinates[:i+1, j, 1])

    trace_t_0.set_offsets(coordinates[i])
    return trace_t_0


ani = animation.FuncAnimation(
    fig, animate, frames=t_end)
plt.show()