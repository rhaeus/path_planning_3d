from grid_map_3D import GridMap3D
import math

class Node:
  def __init__(self, position = None, parent = None, cost = 0, heuristic_cost = 0, total_cost = 0):
      self.position = position
      self.total_cost = total_cost
      self.heuristic_cost = heuristic_cost
      self.cost = cost
      self.parent = parent

class AStar3D:

    def __init__(self, grid_map):
        self.grid_map = grid_map
        self.max_iterations = 500000

    
    def plan(self, start_index, end_index):    
        start_node = Node(start_index)
        end_node = Node(end_index)

        open_list = dict()
        closed_list = dict()  
        open_list[self.grid_map.get_flattened_index(start_index)] = start_node

        count = 0

        while len(open_list) > 0:
            count = count + 1

            if count > self.max_iterations:
                print("A* reached max iterations")
                return []

            # find node with minimum cost
            current_grid_index = min(open_list, key=lambda o: open_list[o].total_cost)
            current_node = open_list[current_grid_index]

            # remove current node from open and add to closed
            # open_list.pop(current_node)
            del open_list[current_grid_index]
            closed_list[current_grid_index] = current_node

            # check if we reached target
            # if current_node.position[0] == end_node.position[0] and current_node.position[1] == end_node.position[1]
            if current_node.position == end_node.position:
                break;

            # expand to neigbors
            neighbor_indices = self.grid_map.get_cell_neighbors(current_node.position)

            for neighbor_cell in neighbor_indices:
                neighbor_node = Node(neighbor_cell, current_node)

                if neighbor_cell in closed_list:
                    continue

                if not self.check_location(neighbor_node):
                    continue

                d = math.sqrt(math.pow(current_node.position[0] - neighbor_node.position[0],2) + math.pow(current_node.position[1] - neighbor_node.position[1],2) + math.pow(current_node.position[2] - neighbor_node.position[2],2))
                cost = current_node.cost + d
                heuristic_cost = self.heuristic(neighbor_node, end_node)
                neighbor_node.cost = cost
                neighbor_node.heuristic_cost = heuristic_cost
                neighbor_node.total_cost = neighbor_node.cost + neighbor_node.heuristic_cost

                neighbor_flat_index = self.grid_map.get_flattened_index(neighbor_cell) 
                if neighbor_flat_index in open_list:
                    # If it is on the open list already, check to see if this path to that square is better, using G cost as the measure. 
                    # A lower G cost means that this is a better path. If so, change the parent of the square to the current square, 
                    # and recalculate the G and F scores of the square. If you are keeping your open list sorted by F score, 
                    # you may need to resort the list to account for the change.
                    if open_list[neighbor_flat_index].cost > neighbor_node.cost:
                        open_list[neighbor_flat_index] = neighbor_node
                else:
                    # If it isnt on the open list, add it to the open list.
                    # Make the current square the parent of this square. 
                    # Record the F, G, and H costs of the square.
                    open_list[neighbor_flat_index] = neighbor_node
            

        # reconstruct path 
        path = []
        current = current_node
        while current is not None:
            path.append(current.position)
            current = current.parent
        return path[::-1] # Return reversed path 

    # def sparsen_path(self, path):
    #     if len(path) <= 1:
    #         print("no path found, nothing there to sparsen")

    #     new_path = []
    #     new_path.append(path[0]) # add start point
    #     last_index_added = 0

    #     goal_index = len(path) - 1

    #     while last_index_added != goal_index:
    #         for i in range(last_index_added + 1, len(path)):
    #             ray = self.grid_map.raytrace(path[last_index_added], path[i])
    #             # check if obstacle on the way
    #             blocked = False
    #             for x, y in ray:
    #                 if self.grid_map.get_value((x,y)) != self.grid_map.free_space:
    #                     blocked = True
    #                     break

    #             if blocked:
    #                 # if blocked add last waypoint to new path
    #                 new_path.append(path[i-1])
    #                 last_index_added = i - 1
    #             else:
    #                 # if we are at the last waypoint and its not blocked add goal point
    #                 if i == len(path) - 1:
    #                     new_path.append(path[-1])
    #                     last_index_added = len(path) - 1
        
    #     return new_path


    def check_location(self, node):
        return self.grid_map.is_index_in_range(node.position) and self.grid_map.get_value(node.position) == self.grid_map.free_space


    def heuristic(self, n1, n2):
        # use euclidean distance as heuristic
        return math.sqrt(math.pow(n1.position[0] - n2.position[0], 2) + math.pow(n1.position[1] - n2.position[1], 2) + math.pow(n1.position[2] - n2.position[2], 2))