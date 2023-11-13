# RRT* implementation in 3D

# import matplotlib.pyplot as plt
import numpy as np
from helperfuncs import *
import blender_plots as bplt

# Class for each tree node
class Node:
    def __init__(self, x, y, z):
        self.x = x        # coordinate
        self.y = y        # coordinate
        self.z = z  # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost

    def __str__(self):
        return f"Node(x={self.x}, y={self.y}, z={self.z})"


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, envi, start, goal):

        envi.make_env()
        self.map_array = envi.get_map_array()  # map array, 1->free, 0->obstacle

        self.size_x = self.map_array.shape[0]    # map size
        self.size_y = self.map_array.shape[1]    # map size
        self.size_z = self.map_array.shape[2]    # map size

        self.start = Node(start[0], start[1], start[2])  # start node
        self.goal = Node(goal[0], goal[1], goal[2])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        self.bloat_amount = envi.get_bloat_amount()
        self.map_array_scale = envi.get_map_array_scale()
        self.lowerboundary = envi.get_lower_boundary()
        self.upperboundary = envi.get_upper_boundary()

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)
        indices = convrule(self.start.x,self.start.y,self.start.z,self.lowerboundary[0],self.lowerboundary[1],self.lowerboundary[2],self.map_array_scale,self.bloat_amount)
        self.map_array[indices[0]][indices[1]][indices[2]] = 2

    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        return ((node1.x - node2.x)**2 + (node1.y-node2.y)**2 + (node1.z-node2.z)**2)**0.5

    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles using Bresenham's line algorithm
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''

        x1, y1, z1 = round(node1.x, 1), round(node1.y, 1), round(node1.z, 1)
        x2, y2, z2 = round(node2.x, 1), round(node2.y, 1), round(node2.z, 1)

        dx = round(abs(x2 - x1), 1)
        dy = round(abs(y2 - y1), 1)
        dz = round(abs(z2 - z1), 1)

        sx = 0.1 if x2 > x1 else -0.1
        sy = 0.1 if y2 > y1 else -0.1
        sz = 0.1 if z2 > z1 else -0.1
        
        if dx >= dy and dx >= dz:
            err_1 = round(2*dy - dx, 1)
            err_2 = round(2*dz - dx, 1)
            while x1 != x2 or y1 != y2 or z1 != z2:
                indices = convrule(x1,y1,z1,self.lowerboundary[0],self.lowerboundary[1],self.lowerboundary[2],self.map_array_scale,self.bloat_amount)
                if not self.map_array[indices[0]][indices[1]][indices[2]]:
                    return False
                if err_1 > 0:
                    y1 = round(y1 + sy, 1)
                    err_1 = round(err_1-2*dx, 1)
                if err_2 > 0:
                    z1 = round(z1+sz, 1)
                    err_2 = round(err_2-2*dx, 1)
                x1 = round(x1+sx, 1)
                err_1 = round(err_1+2*dy, 1)
                err_2 = round(err_2+2*dz, 1)
                # print("In while 1 x1,y1,z1 is", x1, y1, z1)
        elif dy >= dx and dy >= dz:
            err_1 = round(2*dx - dy, 1)
            err_2 = round(2*dz - dy, 1)
            while x1 != x2 or y1 != y2 or z1 != z2:
                indices = convrule(x1,y1,z1,self.lowerboundary[0],self.lowerboundary[1],self.lowerboundary[2],self.map_array_scale,self.bloat_amount)
                if not self.map_array[indices[0]][indices[1]][indices[2]]:
                    return False
                if err_1 > 0:
                    x1 = round(x1+sx, 1)
                    err_1 = round(err_1-2*dy, 1)
                if err_2 > 0:
                    z1 = round(z1+sz, 1)
                    err_2 = round(err_2-2*dy, 1)
                y1 = round(y1+sy, 1)
                err_1 = round(err_1+2*dx, 1)
                err_2 = round(err_2+2*dz, 1)
                # print("In while 2 x1,y1,z1 is", x1, y1, z1)
        else:
            err_1 = round(2*dy - dz, 1)
            err_2 = round(2*dx - dz, 1)
            while x1 != x2 or y1 != y2 or z1 != z2:
                indices = convrule(x1,y1,z1,self.lowerboundary[0],self.lowerboundary[1],self.lowerboundary[2],self.map_array_scale,self.bloat_amount)
                if not self.map_array[indices[0]][indices[1]][indices[2]]:
                    return False
                if err_1 > 0:
                    y1 = round(y1+sy, 1)
                    err_1 = round(err_1-2*dz, 1)
                if err_2 > 0:
                    x1 = round(x1+sx, 1)
                    err_2 = round(err_2-2*dz, 1)
                z1 = round(z1+sz, 1)
                err_1 = round(err_1+2*dy, 1)
                err_2 = round(err_2+2*dx, 1)
                # print("In while 3 x1,y1,z1 is", x1, y1, z1)

        return True

    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        # generate a random number between 0 and 1
        rand_num = np.random.uniform(0, 1)

        if rand_num <= goal_bias:  # If the random number is less than goal bias
            return [self.goal.x, self.goal.y, self.goal.z]  # return goal
        else:
            return [round(np.random.uniform(float(self.lowerboundary[0]), float(self.upperboundary[0])), 1),
                    round(np.random.uniform(float(self.lowerboundary[1]), float(self.upperboundary[1])), 1),
                    round(np.random.uniform(float(self.lowerboundary[2]),float(self.upperboundary[2])), 1)]  # return a random point in map_array

    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        # initialize the first node in self.vertices to be the nearest node
        nearest_node = self.vertices[0]
        # make the point into a node
        sample_node = Node(point[0], point[1], point[2])

        # initialize the best distance
        best_dist = self.dis(nearest_node, sample_node)

        for vertex in self.vertices:  # loop through all vertices and change the best distance and the nearest node if closer vertex found
            vert_dist = self.dis(vertex, sample_node)
            if (vert_dist < best_dist):
                nearest_node = vertex
                best_dist = vert_dist

        return nearest_node, best_dist  # return nearest node to point

    def steer(self, random_node, nearest_node, delta_q):
        """
        steer towards the random node from the nearest node in the existing tree 
        arguments: random node generated in the map, nearest node to that random node in the existing
                   tree, the incremental distance to steer towards the random node from the nearest node 
        return: the node obtained by steering 

        """

        # Check if the distance between random node and nearest node is less than the delta_q. If so, return the random node as the steered node
        if (self.dis(random_node, nearest_node) < delta_q):
            return random_node

        else:
            # Extract the x, y and z values for the nearest and random nodes
            x1 = nearest_node.x
            y1 = nearest_node.y
            z1 = nearest_node.z

            x2 = random_node.x
            y2 = random_node.y
            z2 = random_node.z

            # Calculate the distance between the random node and nearest node
            node_dist = self.dis(random_node, nearest_node)

            # Calculate the vector in x and y direction towards the random node
            # Calculate differences
            delta_x = round(x2 - x1, 1)
            delta_y = round(y2 - y1, 1)
            delta_z = round(z2 - z1, 1)

            vec_x = round(delta_q * (delta_x / node_dist), 1)
            vec_y = round(delta_q * (delta_y / node_dist), 1)
            vec_z = round(delta_q * (delta_z / node_dist), 1)

            # Calculate the new coordinates of the node after steering towards the random node
            steered_x = round(x1 + vec_x, 1)
            steered_y = round(y1 + vec_y, 1)
            steered_z = round(z1 + vec_z, 1)
            # Create a new node with the steered coordinates and return it
            steered_node = Node(steered_x, steered_y, steered_z)
            return steered_node

    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        neighbors = []

        for vertex in self.vertices:
            if (self.dis(new_node, vertex) < neighbor_size):
                neighbors.append(vertex)

        return neighbors

    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''

        for neighbor in neighbors:
            # don't consider the parent of new node to rewire
            if ([neighbor.x, neighbor.y, neighbor.z] != [new_node.parent.x, new_node.parent.y, new_node.parent.z]):
                # if cost of the neighbor > cost to the neighbor with new_node as its parent
                if (new_node.cost + self.dis(new_node, neighbor) < neighbor.cost):
                    if (self.check_collision(new_node, neighbor)):
                        # rewire the parent
                        neighbor.parent = new_node
                        neighbor.cost = new_node.cost + \
                            self.dis(new_node, neighbor)


    def RRT_star(self, n_pts=20000, neighbor_size=20):#prev 20000
        # if not seeing desired results correct the parameters: delta_q_star, goal_tolerance, best_dist threshold, neighbor_size, goal_tolerance, goal probability of self.get_new_point
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance

        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()
        

        # In each step,
        # get a new point,
        # get its nearest node,
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.

        for sample_number in range(n_pts):
        # while not self.found:
        
            # setting the incremental distance for moving from nearest vertex to random vertex (steering)
            delta_q_star = 0.2#0.2
            goal_tolerance = 1#1  # a node within this distance is considered close enough to the goal

            # get its nearest node to the random sample in the existing tree
            best_dist = float('-inf')
            while best_dist <= 0.1:#0.1
                random_sample = self.get_new_point(0.01)  # get a new point 0.01
                nearest_node, best_dist = self.get_nearest_node(
                    random_sample)  # get its nearest node
            # create a node for the random sample generated
            random_node = Node(
                random_sample[0], random_sample[1], random_sample[2])
            
            # steer towards the random node and return the corresponding node
            sample_node = self.steer(random_node, nearest_node, delta_q_star)

            # Convert the node to real world coordinates
            indices_sample = convrule(sample_node.x,sample_node.y,sample_node.z,self.lowerboundary[0],self.lowerboundary[1],self.lowerboundary[2],self.map_array_scale,self.bloat_amount)

            if (self.map_array[indices_sample[0]][indices_sample[1]][indices_sample[2]] == 0 or self.map_array[indices_sample[0]][indices_sample[1]][indices_sample[2]] == 2):
                continue  # if the sample is in an obstacle or self.vertices

            neighbor_size = 2
            neighbors = self.get_neighbors(sample_node, neighbor_size)
            if (not neighbors):
                continue
            # get the node with the least cost in the neighbors found
            best_neighbor = neighbors[0]
            best_neighbor_cost = best_neighbor.cost
            for neighbor in neighbors:  # TODO: put collision check and distance metric inside this loop to get a node for sure that has no collision
                if neighbor.cost + self.dis(sample_node, neighbor) < best_neighbor_cost + self.dis(sample_node, best_neighbor) and self.check_collision(sample_node, best_neighbor):
                    best_neighbor_cost = neighbor.cost
                    best_neighbor = neighbor
            # if the line between the sample node and nearest node has no obstacle
            if (self.check_collision(sample_node, best_neighbor)):
                sample_node.parent = best_neighbor  # update the parent
                sample_node.cost = best_neighbor.cost + \
                    self.dis(sample_node, best_neighbor)  # update cost
            else:
                continue
            self.rewire(sample_node, neighbors)
            # add sample node to list of vertices
            self.vertices.append(sample_node)
            # if a node is added to self.vertices, its corresponding val in map_array will be 2
            indices_sample = convrule(sample_node.x,sample_node.y,sample_node.z,self.lowerboundary[0],self.lowerboundary[1],self.lowerboundary[2],self.map_array_scale,self.bloat_amount)
            self.map_array[indices_sample[0]][indices_sample[1]][indices_sample[2]] = 2

            # if sample node is close enough to goal  and (not self.found)
            if (self.dis(sample_node, self.goal) < goal_tolerance and (not self.found)):
                self.found = True
                print("Found one path")
                self.goal.parent = sample_node  # update goal parent
                self.goal.cost = sample_node.cost + \
                    self.dis(sample_node, self.goal)  # update goal cost
                self.vertices.append(self.goal)  # Add goal to all the vertices tree
                indices_goal = convrule(self.goal.x,self.goal.y,self.goal.z,self.lowerboundary[0],self.lowerboundary[1],self.lowerboundary[2],self.map_array_scale,self.bloat_amount)
                self.map_array[indices_goal[0]][indices_goal[1]][indices_goal[2]] = 2
                # keep iterating and optimizing the current path even after goal is found in RRT*

        path = []

        # Output
        if self.found:
            
            steps = len(self.vertices) - 2
            length = self.goal.cost

            current_node = self.goal
            while current_node is not None:  # Traverse from the goal node to the start node
                path.append(current_node)
                current_node = current_node.parent  # move to the parent node

            path.reverse()  # Reverse the list to print from start to goal

            path_array = np.zeros((len(path),3))
            for i in range(len(path)):
                nodes = path[i]
                path_array[i,:] = np.array([nodes.x,nodes.y,nodes.z])

                print("x,y,z is",nodes.x,nodes.y,nodes.z)
            
            return path

        else:
            print("No path found")
            return path

    def get_array(self,path):
        path_array = np.zeros((len(path),3))
        for i in range(len(path)):
            nodes = path[i]
            path_array[i,:] = np.array([nodes.x,nodes.y,nodes.z])

            print("x,y,z is",nodes.x,nodes.y,nodes.z)
        return path_array

    def get_vertices(self):
        return self.vertices

