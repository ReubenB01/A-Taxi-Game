from search import *
import math
import heapq
import re, math, time

class RoutingGraph(Graph):
    """fefe"""
    def __init__ (self, map_str):
        """fefe"""
        self.graph_str, self.goal_nodes, self.starting_positons, \
        self.fuel_stations, self.portals = self.convert_map_str(map_str)
        

    def convert_map_str(self, map_str):
        """Takes string representation of the map and converts it to a list
        gets location of goal, starting, portal and fuel nodes and adds them to
        seperate lists"""
        my_map = map_str.strip().split('\n')
        my_map = [line.strip() for line in my_map]        
        
        goal_nodes = []
        starting_positions = []
        fuel_stations = []
        portals = []        
             
        for row, line in enumerate(my_map):
            for col, char in enumerate(line):
                
                if char == 'S':
                    starting_positions.append((row, col, math.inf))
                
                elif char.isdigit():
                    starting_positions.append((row, col, int(char)))
                    
                    
                elif char == "G":
                    goal_nodes.append((row, col))
                    
                elif char == "F":
                    fuel_stations.append((row, col))
                
                elif char == "P":
                    portals.append((row, col))
                    
        return my_map, goal_nodes, starting_positions, fuel_stations, portals
                
                
    def is_goal(self, node):
        """return location of all drop off points"""
        return (node[0], node[1]) in self.goal_nodes
    
    def starting_nodes(self):
        """returns starting position of taxis"""
        return self.starting_positons
    
    def outgoing_arcs(self, tail):
        """fefe"""
        arcs = []
        row, col, fuel = tail
        directions = [('N', -1, 0),
                      ('E', 0, +1),
                      ('S', +1, 0),
                      ('W', 0, -1)]
        
        if fuel > 0:
            for direction, row_change, col_change in directions:
                new_row = row + row_change
                new_col = col + col_change
                
                if self.graph_str[new_row][new_col]in [' ', 'F', 'G', 'P','S'] or self.graph_str[new_row][new_col].isdigit():
                    next_node = (new_row, new_col, fuel-1)
                    arcs.append(Arc(tail, next_node, action = "{}".format(direction), cost=5))                
                    
            if self.graph_str[row][col] == 'P':
                for p_row, p_col in self.portals:
                    if (p_row, p_col) != (row, col):
                        arcs.append(Arc(tail, (p_row, p_col, fuel), action = "Teleport to {}".format((p_row, p_col)), cost=10))
                        
        if self.graph_str[row][col] == 'F':
            if fuel < 9:
                fuel = 9
                arcs.append(Arc(tail, (row, col, fuel), action = "Fuel up", cost=15))        
                                       
        return arcs
    
    def estimated_cost_to_goal(self,node):
        """Use Manhatten distance to estimate a nodes cost to a goal node"""
        x = node[1]
        y = node[0]
        heuristics = []
        
        for goal_y, goal_x in self.goal_nodes:
            heuristics.append(abs(x - goal_x) + abs((y - goal_y)))
        
        return(min(heuristics)*5)
    
class AStarFrontier(Frontier):
    """implements A* search algorithm as sublass of Frontier class"""
    
    def __init__(self, graph):
        
        self.graph = graph
        self.index = 0
        self.container = []
        heapq.heapify(self.container)
        self.visited = []
        
    def add(self, path):
        """adds new path to frontier"""
        cost = 0
            
        if path[-1] not in self.visited:
            for arc in path:
                cost += arc.cost
            estimate = self.graph.estimated_cost_to_goal(arc.head)
            total_cost = cost + estimate
            entry = [total_cost, self.index, path]
            self.index += 1
            heapq.heappush(self.container, entry)
                       
    def __next__(self):
        """ returns next node to be expanded. if the node has
        already been expanded prune it. if the heap is empty stop
        iteration
        """
        
        if len(self.container) > 0:
            cost, index, path = heapq.heappop(self.container)
            if path[-1].head not in self.visited:
                self.visited.append(path[-1].head)
                return path
            else:
                return next(self)
        else:
            raise StopIteration 
        
def print_map(map_graph,frontier,solution):
    """ prints simple visual representation of the problem"""

    route = map_graph.graph_str

    for row,col,_ in frontier.visited:
        
        if route[row][col] not in ['S', 'G']:
            position = route[row][:col] + "." + route[row][col+1:]
            route[row] = position  
     
    if solution != None:
        for grid_space in solution[1:-1]:
            row, col = grid_space.head[:2]
            position = route[row][:col] + "*" + route[row][col+1:]
            route[row] = position 
    for i in route:
        print(i) 
        
        
        
#Tests
#######################################################
def q12ans():
    print("Test 1:")
    map_str = """\
    +-------+
    |   G   |
    |       |
    |   S   |
    +-------+
    """

    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_actions(solution)
    
    
    print("\n\nTest 2:")
    map_str = """\
    +-------+
    |     XG|
    |X XXX  |
    | S     |
    +-------+
    """
    
    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_actions(solution)
    
    
    print("\n\nTest 3:")
    map_str = """\
    +-------+
    |  F  XG|
    |X XXXX |
    | 2     |
    +-------+
    """
    
    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_actions(solution)
    
    
    print("\n\nTest 4:")
    map_str = """\
    +--+
    |GS|
    +--+
    """
    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_actions(solution)
    
    
    print("\n\nTest 5:")
    map_str = """\
    +---+
    |GF2|
    +---+
    """
    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_actions(solution)
    
    
    print("\n\nTest 6:")
    start_time = time.time()
    map_str = """\
    +----+
    | S  |
    | SX |
    | X G|
    +----+
    """
    
    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_actions(solution)
    print("--- %s seconds ---" % (time.time() - start_time))
    
    print("\n\nTest 7:")
    start_time = time.time()
    map_str = """\
    +---------+
    |         |
    |    G    |
    |         |
    +---------+
    """
    
    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_actions(solution)
    print("--- %s seconds ---" % (time.time() - start_time))
    
    print("\n\nTest 8")
    start_time = time.time()
    map_str = """\
    +----------+
    |    X     |
    | S  X  G  |
    |    X     |
    +----------+
    """
    
    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_actions(solution)    
    print("--- %s seconds ---" % (time.time() - start_time))
    

def q3ans():
    map_str = """\
+----------------+
|                |
|                |
|                |
|                |
|                |
|                |
|        S       |
|                |
|                |
|     G          |
|                |
|                |
|                |
+----------------+
"""

    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_map(map_graph, frontier, solution)

    print("\n\n")

    map_str = """\
+----------------+
|                |
|                |
|                |
|                |
|                |
|                |
|        S       |
|                |
|                |
|     G          |
|                |
|                |
|                |
+----------------+
"""


    map_graph = RoutingGraph(map_str)
    # changing the heuristic so the search behaves like LCFS
    map_graph.estimated_cost_to_goal = lambda node: 0

    frontier = AStarFrontier(map_graph)

    solution = next(generic_search(map_graph, frontier), None)
    print_map(map_graph, frontier, solution)

    print("\n\n")

    map_str = """\
+-------------+
| G         G |
|      S      |
| G         G |
+-------------+
"""

    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_map(map_graph, frontier, solution)

    print("\n\n")

    map_str = """\
+-------+
|     XG|
|X XXX  |
|  S    |
+-------+
"""
    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_map(map_graph, frontier, solution)

    print("\n\n")

    map_str = """\
+--+
|GS|
+--+
"""
    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_map(map_graph, frontier, solution)
    
    print("\n\n")

    map_str = """\
+----+
|    |
| SX |
| X G|
+----+
"""

    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_map(map_graph, frontier, solution)

    print("\n\n")

    map_str = """\
+---------------+
|    G          |
|XXXXXXXXXXXX   |
|           X   |
|  XXXXXX   X   |
|  X S  X   X   |
|  X        X   |
|  XXXXXXXXXX   |
|               |
+---------------+
"""

    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_map(map_graph, frontier, solution)

    print("\n\n")

    map_str = """\
+---------+
|         |
|    G    |
|         |
+---------+
"""

    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_map(map_graph, frontier, solution)

def main():
    q12ans()
    q3ans()

if __name__ == "__main__": main()