import collections
from queue import Queue
import sys, heapq, math

# python pathfinder.py [map] [algorithm] [heuristic]
# python pathfinder.py map2.txt bfs [heuristic]
# python pathfinder.py map2.txt ucs [heuristic]
# python pathfinder.py map2.txt astar manhattan
# python pathfinder.py map3.txt astar manhattan

#Get input from command line
map = sys.argv[1]
algo = sys.argv[2]
if algo == "astar":
    heu = sys.argv[3]
# print (algo)

#Open and read files
file = open(map)
line1 = file.readline().strip()
line2 = file.readline().strip()
line3 = file.readline().strip()

#Convert first three lines into commands
size = tuple(line1.split(" "))
start = tuple(line2.split(" "))
end = tuple(line3.split(" "))
# print("Size: " + str(size) + "\nStart: " + str(start) + "\nGoal: " + str(end))

#Remaining lines used to create the 2d array
map = []
for row in range(1,int(size[0])+1):
    line = file.readline()
    # print(line)
    col = []
    col_index = 0
    for i in line:
        if i!=' ' and i!='\n' and col_index<=int(size[1])+1:
            col.append(i)
            col_index+=1
    # print('appended col: ' + str(col))
    map.append(col)
# print("Map")
# print(map)

#Convert map into weights using a dictionary    #Length of row              #Length of col
weights = dict(((i,j), map[i][j]) for i in range(int(size[0])) for j in range(int(size[1])))
# print("Weights: " + str(weights))

#Get adjacent spaces in following order: Up, down, left, right
def get_adj_nodes(array):
    possible_adj_spaces = {}

    for row in range(len(array)):
        for col in range(len(array[0])):
            adj_spaces = []
            if row > 0:
                adj_spaces.append((row-1, col)) #Expand up
            
            if row < len(array)-1:
                adj_spaces.append((row+1, col)) #Expand down
            
            if col > 0:
                adj_spaces.append((row, col-1)) #Expand left

            if col < len(array)-1:
                adj_spaces.append((row, col+1)) #Expand down

            possible_adj_spaces[(row,col)] = adj_spaces
    
    return possible_adj_spaces

neighbors = get_adj_nodes(map)
# print("Neighbors:")
# for i, j in neighbors.items():
#     print(i, ': ', j)

def expand(array, row, col):
    adj_spaces = []
    if row > 0:
        adj_spaces.append((row-1, col)) #Expand up
            
    if row < len(array)-1:
        adj_spaces.append((row+1, col)) #Expand down
            
    if col > 0:
        adj_spaces.append((row, col-1)) #Expand left

    if col < len(array)-1:
        adj_spaces.append((row, col+1)) #Expand down

    
    return adj_spaces

def bfs(array, start, goal, adj_list, wt_list):
    frontier = Queue()
    frontier.put((start, [start]))
    visited = set()
    visited.add(start)

    while frontier:
        node, path = frontier.get()
        # print("Current path: " + str(path))

        if node == goal:
            # print('Reached goal')
            return path
        
        visited.add(node)
        adj_nodes = adj_list[(node)]
        # print("Adjacent nodes of " + str(node) + ": " + str(adj_nodes))

        for neighbor in adj_nodes:
            # print(str(neighbor) + " with weight: " + str(wt_list[neighbor]))
            if neighbor not in visited and wt_list[neighbor]!='X': #X denotes an obstacle that cannot be traversed by a path
                visited.add(neighbor)
                new_path = path + [(neighbor)]
                # print("Updated path: " + str(new_path))
                frontier.put(((neighbor), new_path))

        # print("End for loop")
    
    return visited

def create_ucs_path(path, start, goal):
    # print ("Reached goal, implementing ucs path function")
    #Start at goal and iterate until code reaches start node
    node = goal
    # print (node)

    out = []
    while node != start:
        out.append(node)
        # print (out)
        node = path[node] #Move to previous state
    
    out.append(start)
    out.reverse()
    return out

def ucs(array, start, goal, adj_list, wt_list):
            #0 because initial total cost is 0
    frontier = [(0, start)] 
    visited = set()
    path = {start: None}

    while frontier:
        total_cost, node = heapq.heappop(frontier) #Get next state
        # print("Current Path: (" + str(total_cost) + ") " + str(path))
        
        if node == goal:
            # print('Reached goal')
            # print(path)
            return create_ucs_path(path, start, goal)

        visited.add(node)
        
        #Expand to neighboring nodes
        for neighbor in expand(array, node[0], node[1]):
            # print(" " + str(neighbor))
            if not wt_list[neighbor] == 'X' and neighbor not in visited:
                #Update total cost
                if(int(wt_list[neighbor]) - int(wt_list[node]) > 0):
                    new_cost = total_cost + int(wt_list[neighbor]) - int(wt_list[node]) + 1
                else:
                    new_cost = total_cost + 1
                
                heapq.heappush(frontier, (new_cost, neighbor)) #Add neighbor to frontier
                path[neighbor] = node #Add path to neighbor

#Heuristic functions
def manhattan_dist(current, goal):
    # print("  Manhattan distance\n   Current: " + str(current) + " \n   Goal: " + str(goal))
    dist = abs(current[0] - goal[0]) + abs(current[1] - goal[1])
    # print("  Manhattan dist: " + str(dist))
    return dist

def euclidean_dist(current, goal):
    # print("  Euclidean distance\n   Current: " + str(current) + " \n   Goal: " + str(goal))
    dist = math.sqrt(((current[0]) - (goal[0]))**2 + ((current[1]) - (goal[1]))**2)
    # print("  Euclidean dist: " + str(dist))
    return dist

#A* search
def astar(array, start, goal, adj_list, wt_list, heuristic_func):
    heap = [(0, 0, 0, start, [])]
    visited = set()
    entry_count = 0
    # print("Height: " + str(len(array)) + "\nWidth: " + str(len(array[0])))

    while heap:
        _, total_cost, _, node, path = heapq.heappop(heap)
        
        #Skip nodes that are visited already
        if node in visited:
            # print("Skipped " + str(node))
            continue

        visited.add(node)
        path = path + [node]

        #Node reaches end
        # print("Node:" + str(node) + ", Cumulative cost:" + str(total_cost))
        # print(path) 
        if node == goal:
            # print("Reached goal")
            # print(path)
            return path             

        #Expand to neighboring nodes
        row,col = node
        for r,c in [(row-1,col), (row+1,col), (row,col-1), (row,col+1)]:
            if (0<=r<len(array)) and (0<=c<len(array[0])) and (array[r][c] != 'X'):
                neighbor = (r,c)
                # print(" Neighbors: " + str(neighbor))
                if neighbor not in visited:
                    #Update total cost
                    if(int(wt_list[neighbor]) - int(wt_list[node]) > 0):
                        G_cost = total_cost + int(wt_list[neighbor]) - int(wt_list[node]) + 1
                    else:
                        G_cost = total_cost + 1

                    # print("  New_cost for neighbor node: " + str(neighbor) + " is " + str(cum_cost))
                    heuristic = 0
                    if (heuristic_func == "manhattan"):
                        heuristic = manhattan_dist(neighbor, goal)
                    elif (heuristic_func == "euclidean"):
                        heuristic = euclidean_dist(neighbor, goal)
                    else:
                        print("Error in heuristic function")
                        break
                    F_cost = G_cost + heuristic
                    # print("  Cum_cost for neighbor node: " + str(neighbor) + " is " + str(total_cost) + "+" + str(wt_list[neighbor]) + ". h(n) = " + str(heu_cost))
                    # print(" f" + str(neighbor) + " = (" + str(total_cost) + "+" + str(wt_list[neighbor]) + ") + " + str(heuristic) + " = " + str(F_cost))

                    #Add entry count to prioritise up, down, left, right expansion
                    entry_count+=1
                    if r == row - 1: entry_count += 1 #If up
                    elif r == row + 1: entry_count += 2 #If down 
                    elif c == col - 1: entry_count += 3 #If left

                    heapq.heappush(heap, (F_cost, G_cost, entry_count, neighbor, path)) #Add neighbor to frontier

    return None


# print("Parameters for algo: ", int(start[0])-1, int(start[1])-1, int(end[0])-1, int(end[1])-1)
path_algo = 0
if algo == "bfs":
    path_algo = bfs(map, (int(start[0])-1,int(start[1])-1), (int(end[0])-1,int(end[1])-1), neighbors, weights)
elif algo == "ucs":
    path_algo = ucs(map, (int(start[0])-1,int(start[1])-1), (int(end[0])-1,int(end[1])-1), neighbors, weights)
elif algo == "astar":
    if heu == "manhattan":
        path_algo = astar(map, (int(start[0])-1,int(start[1])-1), (int(end[0])-1,int(end[1])-1), neighbors, weights, "manhattan")
    elif heu == "euclidean":
        path_algo = astar(map, (int(start[0])-1,int(start[1])-1), (int(end[0])-1,int(end[1])-1), neighbors, weights, "euclidean")

if path_algo == None:
    print("null")
else:
    #Copy map and assign * to travelled paths 
    map_path = weights.copy() 
    #Printing path
    for path in path_algo:
        # print(path)
        map_path[path] = "*"
    for row in range(int(size[0])):
        for col in range(int(size[1])):
            print(map_path[(row,col)], end = '')
            if col < int(size[1]):
                print(" ", end = '')
        print('')