import bpy
import numpy as np
from helperfuncs import *


class Environment:
    def __init__(self, filepath) -> None:
        self.filepath = filepath
        self.map_array = np.array([])
        self.map_array_scale = 10 # Don't change this
        self.bloat_amount = 0.25
        self.lowerboundary = None
        self.upperboundary = None

    def make_env(self, bloat_amount=0.25):
        # Function to create the obstacle map from map text file
        
        try:
            # Read the file content
            with open(self.filepath, 'r') as f:
                content = f.read()
        except FileNotFoundError:
            return

        # Process the content
        lines = content.split('\n')

        for line in lines:
            words = line.split()
            if words:
                if words[0] == 'boundary':
                    boundary = [float(i) for i in words[1:7]]
                    self.lowerboundary = np.array(
                        [boundary[0], boundary[1], boundary[2]])
                    self.upperboundary = np.array(
                        [boundary[3], boundary[4], boundary[5]])
                    # # Create the boundary cube
                    # bpy.ops.mesh.primitive_cube_add(size=1, location=((boundary[0] + boundary[3]) / 2,
                    #                                                   (boundary[1] +
                    #                                                    boundary[4]) / 2,
                    #                                                   (boundary[2] + boundary[5]) / 2))

                    # boundary_cube = bpy.context.object
                    # boundary_cube.dimensions = [round(boundary[3] - boundary[0], 1), round(
                    #     boundary[4] - boundary[1], 1), round(boundary[5] - boundary[2], 1)]

                    # # Subdivide the cube to create a grid effect
                    # bpy.ops.object.mode_set(mode='EDIT')
                    # bpy.ops.mesh.subdivide(number_cuts=10)
                    # bpy.ops.object.mode_set(mode='OBJECT')

                    # # Shader setup for black color with alpha transparency of 0.344
                    # mat = bpy.data.materials.new(name="Grid_Material")
                    # mat.use_nodes = True
                    # nodes = mat.node_tree.nodes
                    # bsdf = nodes["Principled BSDF"]
                    # bsdf.inputs[0].default_value = (0, 0, 0, 0.344)

                    # # Set blend mode to Alpha Blend
                    # mat.blend_method = 'BLEND'
                    # boundary_cube.data.materials.append(mat)

                    max_x, max_y, max_z = round(boundary[3]-boundary[0]+2*bloat_amount, 1), round(
                        boundary[4]-boundary[1]+2*bloat_amount, 1), round(boundary[5]-boundary[2]+2*bloat_amount, 1)
                    self.map_array = np.ones((int(self.map_array_scale*max_x)+1, int(
                        self.map_array_scale*max_y)+1, int(self.map_array_scale*max_z)+1), dtype=np.uint8)

                elif words[0] == 'block':
                    block_coords = [float(i) for i in words[1:7]]
                    color = [int(i)/255 for i in words[7:10]]

                    # x_start, y_start, z_start, x_end, y_end, z_end = map(int, block_coords)
                    # self.map_array[50*x_start:50*x_end, 50*y_start:50*y_end, 50*z_start:50*z_end] = 0  # 0 denotes obstacles

                    # Calculate the bloated dimensions
                    # bloated_dimensions = [
                    #     round(
                    #         (block_coords[3] - block_coords[0]) + 2 * bloat_amount, 1),
                    #     round(
                    #         (block_coords[4] - block_coords[1]) + 2 * bloat_amount, 1),
                    #     round(
                    #         (block_coords[5] - block_coords[2]) + 2 * bloat_amount, 1),
                    # ]
                    bloated_dimensions = [
                        round(
                            (block_coords[3] - block_coords[0]), 1),
                        round(
                            (block_coords[4] - block_coords[1]), 1),
                        round(
                            (block_coords[5] - block_coords[2]), 1),
                    ]

                    # Adjust the location to account for the increased dimensions
                    bloated_location = [
                        (block_coords[0] + block_coords[3]) / 2,
                        (block_coords[1] + block_coords[4]) / 2,
                        (block_coords[2] + block_coords[5]) / 2,
                    ]
                    bloated_coords = [
                        block_coords[0] - bloat_amount,
                        block_coords[1] - bloat_amount,
                        block_coords[2] - bloat_amount,
                        block_coords[3] + bloat_amount,
                        block_coords[4] + bloat_amount,
                        block_coords[5] + bloat_amount,
                    ]
                    # bloated_coords =[]
                    # if 0.<block_coords[0] and block_coords[3]<10.:

                    # elif block_coords[0]<=0. and block_coords[3]<10.:
                    #     bloated_coords = [
                    #         block_coords[0],
                    #         block_coords[1] - bloat_amount,
                    #         block_coords[2] - bloat_amount,
                    #         block_coords[3]+bloat_amount,
                    #         block_coords[4] + bloat_amount,
                    #         block_coords[5] + bloat_amount,
                    #     ]
                    # else:
                    #     bloated_coords = [
                    #         block_coords[0],
                    #         block_coords[1] - bloat_amount,
                    #         block_coords[2] - bloat_amount,
                    #         block_coords[3],
                    #         block_coords[4] + bloat_amount,
                    #         block_coords[5] + bloat_amount,
                    #     ]

                    
                    x_start, y_start, z_start, x_end, y_end, z_end = bloated_coords[0], bloated_coords[
                        1], bloated_coords[2], bloated_coords[3], bloated_coords[4], bloated_coords[5]
                    indices_start = convrule(
                        x_start, y_start, z_start, boundary[0], boundary[1], boundary[2], self.map_array_scale, bloat_amount)
                    indices_end = convrule(
                        x_end, y_end, z_end, boundary[0], boundary[1], boundary[2], self.map_array_scale, bloat_amount)
                    self.map_array[indices_start[0]:indices_end[0]+1, indices_start[1]:indices_end[1]+1, indices_start[2]:indices_end[2]+1] = 0  # 0 denotes obstacles
                    # self.map_array[int(self.map_array_scale*(x_start+0.1)):int(self.map_array_scale*(x_end+0.1))+1, int(self.map_array_scale*(y_start+5.1)):int(
                    #     self.map_array_scale*(y_end+5.1))+1, int(self.map_array_scale*(z_start+0.1)):int(self.map_array_scale*(z_end+0.1)+1)] = 0  # 0 denotes obstacles
                    
                    # Create the block with bloated dimensions and adjusted location
                    bpy.ops.mesh.primitive_cube_add(
                        size=1, location=bloated_location)
                    cube = bpy.context.object
                    cube.dimensions = bloated_dimensions

                    # Color the block
                    mat_name = f"Material_{color[0]}_{color[1]}_{color[2]}"
                    mat = bpy.data.materials.get(
                        mat_name) or bpy.data.materials.new(name=mat_name)
                    mat.diffuse_color = (color[0], color[1], color[2], 1)
                    cube.data.materials.append(mat)


    def add_sphere(self, location, color, radius=0.07):
        # Adjust the location to Blender's coordinate system
        adjusted_location = (location[0], location[1], location[2])

        # Create the sphere with specified location and radius
        bpy.ops.mesh.primitive_uv_sphere_add(
            radius=radius, location=adjusted_location)
        sphere = bpy.context.object

        # Color the sphere
        mat_name = f"Material_{color[0]}_{color[1]}_{color[2]}"
        mat = bpy.data.materials.get(
            mat_name) or bpy.data.materials.new(name=mat_name)
        mat.diffuse_color = color
        sphere.data.materials.append(mat)

    def visualize_nodes(self, path):
        # Define nodes and their colors
        start_node = (
            path[0].x,
            path[0].y,
            path[0].z
        )
        start_color = (1, 0, 0, 1)  # Red
        self.add_sphere(start_node, start_color)

        # Traverse through the intermediate nodes
        for i in range(1, len(path) - 1):
            # Get the actual Node object from the path list
            intermediate_node = path[i]
            intermediate = (
                intermediate_node.x,
                intermediate_node.y,
                intermediate_node.z
            )
            intermediate_color = (0, 0, 1, 0.8)  # Blue
            self.add_sphere(intermediate, intermediate_color)

        # Define the goal node and its color
        goal_node = (
            path[-1].x,
            path[-1].y,
            path[-1].z
        )
        goal_color = (0, 1, 0, 1)  # Green
        self.add_sphere(goal_node, goal_color)

    def get_map_array(self):
        return self.map_array

    def get_map_array_scale(self):
        return self.map_array_scale

    def get_bloat_amount(self):
        return self.bloat_amount

    def get_lower_boundary(self):
        return self.lowerboundary
    def get_upper_boundary(self):
        return self.upperboundary


# # Usage
# filepath = 'path_to_your_file.txt'
# bloat_amount = 0.1  # Adjust as needed
# env = Environment(filepath)
# env.make_env(bloat_amount)
