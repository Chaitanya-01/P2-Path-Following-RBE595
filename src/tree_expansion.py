import numpy as np
import bpy
import mathutils
import math

# Class for each tree node
class Node:
    def __init__(self, x, y, z): 
        self.x = x        # coordinate
        self.y = y        # coordinate
        self.z = z        #coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost
    
    def __str__(self):
        return f"Node(x={self.x}, y={self.y}, z={self.z})  "



class TreeExpansion:

    def __init__(self):
        self.tree = {}
        self.map_array_scale = 50 

    def add_sphere(self, location, color, radius=0.3):
        # Adjust the location to Blender's coordinate system (if needed, not adjusting in this case)
        adjusted_location = (location[0], location[1], location[2])
        
        # Create the sphere with specified location and radius
        bpy.ops.mesh.primitive_uv_sphere_add(radius=radius, location=adjusted_location)
        sphere = bpy.context.object
        
        # Color the sphere
        mat_name = f"Material_{color[0]}_{color[1]}_{color[2]}"
        mat = bpy.data.materials.get(mat_name) or bpy.data.materials.new(name=mat_name)
        mat.diffuse_color = color
        sphere.data.materials.append(mat)

    #TODO: Edit the below code for tree visualization 
    
    def add_and_connect_current_node(self, current_node: Node, parent_node: Node):
        grey_color = (0.5, 0.5, 0.5, 1)  # RGBA
        # Add current_node sphere to Blender Scene
        current_node_location = (current_node.x/self.map_array_scale, current_node.y/self.map_array_scale, current_node.z/self.map_array_scale)

        self.add_sphere(current_node_location, grey_color)

        # Connect the current_node to parent_node with a cylinder
        self.connect_nodes_with_cylinder(current_node, parent_node)
        
        # Update the tree structure (you can use a dictionary)
        self.tree[current_node] = parent_node

    def update_neighbor_connection(self, neighbor_node: Node, sample_node: Node):
        # Retrieve the old parent of the neighbor_node from the tree structure
        old_parent_node = self.tree[neighbor_node]
        
        # Remove the connection (cylinder) between neighbor_node and old_parent_node in Blender Scene
        self.remove_connection(old_parent_node, neighbor_node)
        
        # Connect neighbor_node to sample_node with a new cylinder in Blender Scene
        self.connect_nodes_with_cylinder(neighbor_node, sample_node)
        
        # Update the tree structure with the new connection
        self.tree[neighbor_node] = sample_node


    def get_cylinder_name(self, node1: Node, node2: Node):
        # This can be any string that uniquely identifies the cylinder between node1 and node2
        return f"cylinder_{node1.x}_{node1.y}_{node1.z}_to_{node2.x}_{node2.y}_{node2.z}"

    
    def connect_nodes_with_cylinder(self, node1: Node, node2: Node):
        # Compute the location and orientation of the cylinder
        location, rotation_quaternion, scale, depth = self.compute_cylinder_transform(node1, node2)

        # Add the cylinder to the Blender Scene
        bpy.ops.mesh.primitive_cylinder_add(radius=scale[0], depth=depth, location=location, rotation=rotation_quaternion.to_euler())

        # Assign a unique name to the cylinder
        cyl_name = self.get_cylinder_name(node1, node2)
        bpy.context.object.name = cyl_name

    
    def remove_connection(self, node1: Node, node2: Node):
        # Find the corresponding cylinder object in Blender Scene
        # You might want to assign a unique name to each cylinder when you create them
        cyl_name = self.get_cylinder_name(node1, node2)
        cylinder_obj = bpy.data.objects.get(cyl_name)
        
        # Remove the cylinder object from Blender Scene
        if cylinder_obj:
            bpy.data.objects.remove(cylinder_obj)
  

    def compute_cylinder_transform(self, node1: Node, node2: Node):
        # Compute the mid-point between node1 and node2 to place the cylinder
        location = [(node1.x + node2.x) / (2*self.map_array_scale),
                    (node1.y + node2.y) / (2*self.map_array_scale),
                    (node1.z + node2.z) / (2*self.map_array_scale)]

        # Compute the direction vector from node1 to node2
        dir_vector = [(node2.x - node1.x)/(self.map_array_scale),
                    (node2.y - node1.y)/(self.map_array_scale),
                    (node2.z - node1.z)/(self.map_array_scale)]

        # Compute the length of the direction vector, which will be the height of the cylinder
        length = np.linalg.norm(dir_vector)
        
        # Normalize the direction vector
        if length != 0:
            dir_vector = [x / length for x in dir_vector]
        
        # Compute the rotation of the cylinder
        axis = [dir_vector[1], -dir_vector[0], 0]  # Perpendicular in the xy-plane
        angle = math.acos(max(-1.0, min(1.0, dir_vector[2])))   # dir_vector[2] is the z-component of the normalized direction vector
        
        # Create a rotation quaternion
        rotation_quaternion = mathutils.Quaternion(axis, angle)
        
        # Scale of the cylinder
        radius = 0.1  # you might want to adjust the radius according to your need.
        scale = [radius, radius, length / 2]  
        
        return location, rotation_quaternion, scale, length

        
