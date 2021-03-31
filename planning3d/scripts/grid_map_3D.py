import rospy
import numpy as np
import json 
from geometry_msgs.msg import Pose, Point
from math import fabs
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA


class GridMap3D:

    def __init__(self, map_path, resolution, inflation_radius):
        self.resolution = resolution
        self.unknown_space = -1
        self.unknown_space_color = ColorRGBA(0.5, 0.5, 0.5, 1)
        self.free_space = 0
        self.free_space_color = ColorRGBA(0, 1, 0, 1)
        self.c_space = 50
        self.c_space_color = ColorRGBA(1, 1, 0, 1)
        self.occupied_space = 100
        self.occupied_space_color = ColorRGBA(1, 0, 0, 1)
        self.inflation_radius_m = inflation_radius 

        self.load_map(map_path)
        
    def load_map(self, map_path):
        with open(map_path, 'rb') as f:
            self.json_world = json.load(f) 
        
        # bounding box of the grid defined by airspace measured in meters
        self.b_min = (self.json_world["airspace"]["min"][0], self.json_world["airspace"]["min"][1], self.json_world["airspace"]["min"][2])
        self.b_max = (self.json_world["airspace"]["max"][0], self.json_world["airspace"]["max"][1], self.json_world["airspace"]["max"][2])

        # the x and y resolution measured in meters
        dx = self.resolution
        dy = self.resolution
        dz = self.resolution

        # width and heigt of the grid measured in cells
        self.width = int((self.b_max[0] - self.b_min[0]) / float(dx));
        self.height = int((self.b_max[1] - self.b_min[1]) / float(dy));
        self.depth = int((self.b_max[2] - self.b_min[2]) / float(dz));

        # print("width:", self.width)
        # print("height:", self.height)
        # print("depth:", self.depth)

        # put origin of the map to (0,0)
        self.origin = Pose()
        self.origin.position.x = self.b_min[0]
        self.origin.position.y = self.b_min[1]
        self.origin.position.z = self.b_min[2]
        # print("origin: ", self.origin.position.x, self.origin.position.y, self.origin.position.z)

        self.inflation_radius_cells = int(self.inflation_radius_m / self.resolution)

        self.map_data = np.full((self.depth, self.height, self.width), self.free_space, dtype=np.int8)
        self.read_walls()

        self.inflate_map(self.inflation_radius_cells)

    def inflate_map(self, radius):
        for x in range(self.width):
            for y in range(self.height):
                for z in range(self.depth):
                    if self.map_data[z, y, x] == self.occupied_space:
                        for nx in range(x - radius, x + radius + 1):
                            for ny in range(y - radius, y + radius + 1):
                                for nz in range(z - radius, z + radius + 1):
                                    if not self.is_index_in_range((nx, ny, nz)):
                                        continue
                                    # d = hypot(x - nx, y - ny)
                                    # if d <= radius:
                                    if self.map_data[nz, ny, nx] != self.occupied_space:
                                        self.map_data[nz, ny, nx] = self.c_space

    
    def coord_to_grid_index(self, coord):
        if not self.is_coord_in_range(coord):
            print("coord outside grid boundary!")

        x_index = int( (coord[0] - self.b_min[0]) / self.resolution)
        y_index = int( (coord[1] - self.b_min[1]) / self.resolution)
        z_index = int( (coord[2] - self.b_min[2]) / self.resolution)
        return (x_index, y_index, z_index)

    def grid_index_to_coord(self, index):
        # TODO maybe return cell center
        return (index[0] * self.resolution + self.b_min[0], index[1] * self.resolution + self.b_min[1], index[2] * self.resolution + self.b_min[2]) 
    
    def get_flattened_index(self, index):
        return index[0] + self.width * index[1] + self.width * self.height * index[2]
    
    def is_coord_in_range(self, coord):
        return coord[0] >= self.b_min[0] and coord[0] <= self.b_max[0] and coord[1] >= self.b_min[1] and coord[1] <= self.b_max[1] and coord[2] >= self.b_min[2] and coord[2] <= self.b_max[2]


    def is_index_in_range(self, index):
        return index[0] >= 0 and index[0] < self.width and index[1] >= 0 and index[1] < self.height and index[2] >= 0 and index[2] < self.depth


    def get_cell_neighbors(self, cell_index):
        neighbors = []
        for x in range(-1, 2):
            for y in range(-1, 2):
                for z in range(-1, 2):
                    if x == 0 and y == 0 and z == 0:
                        continue

                    neigbor = (cell_index[0] + x, cell_index[1] + y, cell_index[2] + z)

                    if self.is_index_in_range(neigbor):
                        neighbors.append(neigbor)

        return neighbors

    def get_value(self, cell_index):
        if not self.is_index_in_range(cell_index):
            print("cell index out of range")
        
        return self.map_data[cell_index[2], cell_index[1], cell_index[0]]


    def read_walls(self):
        walls = self.json_world['walls']
        if len(walls) == 0:
            print("no walls in this map")
            return 
        
        for wall in walls:
            start_coord = (wall['plane']['start'][0], wall['plane']['start'][1], wall['plane']['start'][2])
            stop_coord = (wall['plane']['stop'][0], wall['plane']['stop'][1], wall['plane']['stop'][2])

            (start_x_index, start_y_index, start_z_index) = self.coord_to_grid_index(start_coord)
            (stop_x_index, stop_y_index, stop_z_index) = self.coord_to_grid_index(stop_coord)

            if start_x_index > stop_x_index:
                h = stop_x_index
                stop_x_index = start_x_index
                start_x_index = h

            if start_y_index > stop_y_index:
                h = stop_y_index
                stop_y_index = start_y_index
                start_y_index = h

            if start_z_index > stop_z_index:
                h = stop_z_index
                stop_z_index = start_z_index
                start_z_index = h

            for x in range(start_x_index, stop_x_index + 1):
                for y in range(start_y_index, stop_y_index + 1):
                    for z in range(start_z_index, stop_z_index + 1):
                        self.map_data[z, y, x] = self.occupied_space
    
    def get_ros_message(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.CUBE_LIST
        marker.action = marker.ADD
        marker.scale.x = self.resolution
        marker.scale.y = self.resolution
        marker.scale.z = self.resolution
        marker.color = self.occupied_space_color
        marker.pose.orientation.w = 1.0

        for x in range(self.width):
            for y in range(self.height):
                for z in range(self.depth):

                    if self.map_data[z, y, x] == self.occupied_space:
                        p = Point()
                        p.x, p.y, p.z = self.grid_index_to_coord((x,y,z))
                        marker.points.append(p)
                        marker.colors.append(self.occupied_space_color)

                    elif self.map_data[z, y, x] == self.c_space:
                        p = Point()
                        p.x, p.y, p.z = self.grid_index_to_coord((x,y,z))
                        marker.points.append(p)
                        marker.colors.append(self.c_space_color)

        return marker

    # # use raytrace from introduction to robotics course 
    # def raytrace(self, start, end):
    #     """Returns all cells in the grid map that has been traversed
    #     from start to end, including start and excluding end.
    #     start = (x, y) grid map index
    #     end = (x, y) grid map index
    #     """
    #     (start_x, start_y) = start
    #     (end_x, end_y) = end
    #     x = start_x
    #     y = start_y
    #     (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
    #     n = dx + dy
    #     x_inc = 1
    #     if end_x <= start_x:
    #         x_inc = -1
    #     y_inc = 1
    #     if end_y <= start_y:
    #         y_inc = -1
    #     error = dx - dy
    #     dx *= 2
    #     dy *= 2

    #     traversed = []
    #     for i in range(0, int(n)):
    #         traversed.append((int(x), int(y)))

    #         if error > 0:
    #             x += x_inc
    #             error -= dy
    #         else:
    #             if error == 0:
    #                 traversed.append((int(x + x_inc), int(y)))
    #             y += y_inc
    #             error += dx

    #     return traversed
