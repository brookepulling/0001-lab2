# 6.0002 Problem Set 2 Fall 2021
# Graph Optimization
# Name: Brooke Pulling
# Collaborators: none
# Time: 4 hours

#
# Finding shortest paths to drive from home to work on a road network
#

from graph import DirectedRoad, Node, RoadMap


# PROBLEM 2: Building the Road Network
#
# PROBLEM 2a: Designing your Graph
#
# What do the graph's nodes represent in this problem? What
# do the graph's edges represent? Where are the times
# represented?
#
# Write your answer below as a comment:
# The graphs nodes represent places (sources and destinations), edges represent roads, travel time represents weight 

# PROBLEM 2b: Implementing load_map
def load_map(map_filename):
    """
    Parses the map file and constructs a road map (graph).

    Travel time and traffic multiplier should be cast to a float.

    Parameters:
        map_filename : String
            name of the map file

    Assumes:
        Each entry in the map file consists of the following format, separated by spaces:
            source_node destination_node travel_time road_type traffic_multiplier

        Note: hill road types always are uphill in the source to destination direction and
              downhill in the destination to the source direction. Downhill travel takes
              half as long as uphill travel. The travel_time represents the time to travel
              from source to destination (uphill).

        e.g.
            N0 N1 10 highway 1
        This entry would become two directed roads; one from 'N0' to 'N1' on a highway with
        a weight of 10.0, and another road from 'N1' to 'N0' on a highway using the same weight.

        e.g.
            N2 N3 7 hill 2
        This entry would become to directed roads; one from 'N2' to 'N3' on a hill road with
        a weight of 7.0, and another road from 'N3' to 'N2' on a hill road with a weight of 3.5.

    Returns:
        a directed road map representing the inputted map
    """
    map = RoadMap()
    file = open(map_filename, "r")
    for line in file: 
        parts = line.split()
        source_node = Node(parts[0])
        destination_node = Node(parts[1])
        travel_time = float(parts[2])
        road_type = parts[3]
        traffic_multiplier = float(parts[4])
        src_to_dest = DirectedRoad(source_node, destination_node, travel_time, road_type, traffic_multiplier)
        if road_type == 'hill':
            dest_to_src = DirectedRoad(destination_node, source_node, (travel_time)/2, road_type, traffic_multiplier)
        else:
            dest_to_src = DirectedRoad(destination_node, source_node, travel_time, road_type, traffic_multiplier)
        try:
            map.insert_node(source_node)
        except ValueError:
            pass
        try:
            map.insert_node(destination_node)
        except ValueError:
            pass
        try:
            map.insert_road(src_to_dest)
        except ValueError:
            pass
        try:
            map.insert_road(dest_to_src)
        except ValueError:
            pass
    return map

# PROBLEM 2c: Testing load_map
# Include the lines used to test load_map below, but comment them out after testing

# road_map = load_map("maps/test_load_map.txt")
# print(road_map)


# PROBLEM 3: Finding the Shortest Path using Optimized Search Method



# Problem 3a: Objective function
#
# What is the objective function for this problem? What are the constraints?
#
# Answer:
# find the shortest path between any two nodes, ie the shortest travel time. The constraints are the restricted roads. 
#
#

# PROBLEM 3b: Implement find_optimal_path
def find_optimal_path(roadmap, start, end, restricted_roads, has_traffic=False):
    """
    Finds the shortest path between start and end nodes on the road map,
    without using any restricted roads,
    following traffic conditions.
    Use Dijkstra's algorithm.

    Parameters:
    roadmap - RoadMap
        The graph on which to carry out the search
    start - Node
        node at which to start
    end - Node
        node at which to end
    restricted_roads - list[string]
        Road Types not allowed on path
    has_traffic - boolean
        flag to indicate whether to get shortest path during traffic or not

    Returns:
    A tuple of the form (best_path, best_time).
        The first item is the shortest path from start to end, represented by
        a list of nodes (Nodes).
        The second item is a float, the length (time traveled)
        of the best path.

    If there exists no path that satisfies constraints, then return None.
    """
    
    if start not in roadmap.get_all_nodes() or end not in roadmap.get_all_nodes():
        return None
    if start == end:
        return ([start], 0)
    unvisited = roadmap.get_all_nodes()
    
    #create a dictionary with all nodes with value distance from start node and the path to start node
    map_dict = {}
    
    #make every nodes distance infinit
    for node in unvisited:
        map_dict[node] = float('inf')
    
    #make another dictionary to store the path
    path = {}
    
    #set each path to None
    for node in unvisited:
        path[node] = None
    
    map_dict[start] = 0
    
    while unvisited:
        current = min(unvisited, key = lambda node: map_dict[node])
        #map dictionary  of current is the shortest time so if the shortest time is inf they are all infinite so break 
        if map_dict[current] == float('inf'):
            break
        if current == end:
            break
        
        for road in roadmap.get_reachable_roads_from_node(current, restricted_roads):
            if road.get_destination_node() in unvisited:
                neighbor = road.get_destination_node()
                #neighnor travel time is the distance to neighbor thru current 
                neighbor_travel_time = road.get_travel_time(has_traffic)+float(map_dict[current])
                #check to see if there is a faster way to get to neighbor
                if neighbor_travel_time < map_dict[neighbor]:
                    map_dict[neighbor]= neighbor_travel_time
                    path[neighbor] = current
        unvisited.remove(current)
    if map_dict[end] == float('inf'):
        return None
    best_path = []
    current = end
    while path[current] != None:
        #insert current at index zero
        best_path.insert(0, current)
        current = path[current]
    if best_path != []:
        best_path.insert(0,current)
    else:
        return None
    return (best_path, map_dict[end])

# PROBLEM 4a: Implement optimal_path_no_traffic
def find_optimal_path_no_traffic(filename, start, end):
    """
    Finds the shortest path from start to end during conditions of no traffic.

    You must use find_optimal_path and load_map.

    Parameters:
    filename - name of the map file that contains the graph
    start - Node, node object at which to start
    end - Node, node object at which to end

    Returns:
    list of Node objects, the shortest path from start to end in normal traffic.
    If there exists no path, then return None.
    """
    op_map = load_map(filename)
    path = find_optimal_path(op_map, start, end, [], has_traffic = False)
    return path[0]

# PROBLEM 4b: Implement optimal_path_restricted
def find_optimal_path_restricted(filename, start, end):
    """
    Finds the shortest path from start to end when local roads and hill roads cannot be used.

    You must use find_optimal_path and load_map.

    Parameters:
    filename - name of the map file that contains the graph
    start - Node, node object at which to start
    end - Node, node object at which to end

    Returns:
    list of Node objects, the shortest path from start to end given the aforementioned conditions,
    If there exists no path that satisfies constraints, then return None.
    """
    op_map = load_map(filename)
    path = find_optimal_path(op_map, start, end, ['local', 'hill'], has_traffic = False)
    return path[0]


# PROBLEM 4c: Implement optimal_path_heavy_traffic
def find_optimal_path_in_traffic_no_toll(filename, start, end):
    """
    Finds the shortest path from start to end when toll roads cannot be used and in traffic,
    i.e. when all roads' travel times are multiplied by their traffic multipliers.

    You must use find_optimal_path and load_map.

    Parameters:
    filename - name of the map file that contains the graph
    start - Node, node object at which to start
    end - Node, node object at which to end; you may assume that start != end

    Returns:
    The shortest path from start to end given the aforementioned conditions,
    represented by a list of nodes (Nodes).

    If there exists no path that satisfies the constraints, then return None.
    """
    op_map = load_map(filename)
    path = find_optimal_path(op_map, start, end, ['toll'], has_traffic = True)
    return path[0]


if __name__ == '__main__':

    # UNCOMMENT THE FOLLOWING LINES TO DEBUG
    pass
    # rmap = load_map('./maps/small_map.txt')

    # start = Node('N0')
    # end = Node('N4')
    # restricted_roads = []

    # print(find_optimal_path(rmap, start, end, restricted_roads))
