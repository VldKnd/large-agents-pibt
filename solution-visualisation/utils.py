def parse_tuples(string):
    res = []
    for tuple_str in string[1:-2].split('),('):
        try:
            res.append(
                tuple([
                    int(x) for x in tuple_str.split(',')
                ])
            )
        except ValueError:
            pass
    return res

### For Polygone Construction
def augment_border(text, symbol = "T"):
    extra_line = "".join(["T" for i in range(len(text[0]) + 4)])
    return [extra_line, extra_line] +\
           ["TT"+line+"TT" for line in text] + \
           [extra_line, extra_line]

def BFS(text, start_i, start_j):
    border = set()
    
    n, m = len(text), len(text[0])
    q = [(start_i,start_j)]
    
    while len(q) > 0:
        i, j = q.pop()
        for x, y in [(0, 1), (0, -1),
                     (-1, 0), (1, 0),
                     (-1, -1), (1, 1),
                     (-1, 1), (1, -1)]:
            try:
                if i+x >= 0 and j+y >= 0:
                    if text[i+x][j+y] != "." and text[i+x][j+y] != "X":
                        q.append((i+x, j+y))

                    elif text[i+x][j+y] == ".":
                            border.add((j, i))

            except IndexError:
                pass
            
        text[i][j] = "X"

    return border

def merge_lines(set_of_borders, n, m):
    set_of_nodes = []
    for border in set_of_borders:
        nodes = set()
        for i in range(n):
            l_p, r_p = 0, 1
            while r_p < m:
                if (l_p, i) in border and\
                   (r_p, i) in border:
                    try:
                        while (r_p, i) in border:
                            r_p += 1
                    except IndexError:
                        pass

                    nodes.add((l_p, i))
                    nodes.add((r_p-1, i))

                l_p = r_p
                r_p = l_p + 1
            
        set_of_nodes.append(nodes)
            
    return set_of_nodes
