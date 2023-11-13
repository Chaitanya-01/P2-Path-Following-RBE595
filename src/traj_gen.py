import math
import numpy as np
import scipy
from rrt_star import Node 

# This code is for minimum jerk trajectory (MIGHT HAVE TO CHANGE)

class traj_gen:
    # Constructor
    def __init__(self, envi,waypoints):
        self.waypoints = waypoints
        num_wps,point_size = waypoints.shape #no of points,3
        self.num_wps = num_wps
        self.point_size = point_size
        self.order = 3 # Quintic for minimum jerk path 2*3-1

        self.yaw = 0
        self.ts = []
    def dist(self,p1,p2):
        return np.linalg.norm(p1-p2)
    def timeinterval_calc(self):
        # 5m/s
        total_time = 60
        # avg_speed = 5
        # dist_list = []
        # total_dist =0
        num_segments = self.num_wps-1
        # for i in range(self.num_wps-1):
        #     wp_dist = self.dist(self.waypoints[i,:],self.waypoints[i+1,:])
        #     dist_list.append(wp_dist)
        #     total_dist+=wp_dist
        # total_time = total_dist/avg_speed
        # print(total_dist)
        self.ts.append(0)
        prev_time = 0
        for i in range(self.num_wps-1):
            # prev_time += (dist_list[i]/total_dist)*total_time
            # self.ts.append(prev_time)
            self.ts.append((i+1)*total_time/num_segments)
        return num_segments
    

    def computeFirstRows(self, t0):
        # Define a 3xtotalcoeffs matrix filled with zeros
        num_eqns = self.num_wps - 1
        coeff_per_eqn = self.order + 1
        total_coeffs = num_eqns*coeff_per_eqn
        row_mat = np.zeros((2, total_coeffs))
        row1 = np.array([1, t0, t0*2, t0*3])
        row2 = np.array([0, 1, 2*t0, 3*(t0**2)])
        # row3 = np.array([0, 0, 2, 6*t0])
        # Assign the defined rows to appropriate slices in the row matrix
        row_mat[0, 0:4] = row1
        row_mat[1, 0:4] = row2
        # row_mat[2, 0:4] = row3
        return row_mat  


    def computeRows(self, ti, quotient):
        # Define a 4x12 matrix filled with zeros
        num_eqns = self.num_wps - 1
        coeff_per_eqn = self.order + 1
        total_coeffs = num_eqns*coeff_per_eqn
        row_mat = np.zeros((4, total_coeffs))

        # Define rows based on the given equations
        row1 = np.array([1, ti, ti*2, ti*3])
        row2 = np.array([1, ti, ti*2, ti*3])
        row3 = np.array([0, 1, 2*ti, 3*(ti*2), 0, -1, -2*ti, -3*ti*2])
        row4 = np.array([0, 0, 2, 6*ti, 0, 0, -2, -6*ti])

        # Assign the defined rows to appropriate slices in the row matrix
        row_mat[0, 4*quotient:4*quotient+4] = row1
        row_mat[1, 4*quotient+4:4*quotient+8] = row2
        row_mat[2, 4*quotient:4*quotient+8] = row3
        row_mat[3, 4*quotient:4*quotient+8] = row4

        return row_mat  
    

    def computeLastRows(self,t0):
        num_eqns = self.num_wps - 1
        coeff_per_eqn = self.order + 1
        total_coeffs = num_eqns*coeff_per_eqn
        # Define a 4x12 matrix filled with zeros
        row_mat = np.zeros((2, total_coeffs))
        row1 = np.array([1, t0, t0*2, t0*3])
        row2 = np.array([0, 1, 2*t0, 3*(t0**2)])
        # row3 = np.array([0, 0, 2, 6*t0])
        # Assign the defined rows to appropriate slices in the row matrix
        # row_mat[-3, -4:] = row1
        row_mat[-2, -4:] = row1
        row_mat[-1, -4:] = row2
        return row_mat 


    def get_Amat(self):
        num_eqns = self.num_wps - 1
        coeff_per_eqn = self.order + 1
        total_coeffs = num_eqns*coeff_per_eqn
        num_constraints = 4*(self.num_wps-2) + 4
        coeff_array = np.zeros((num_eqns*coeff_per_eqn,))
        big_mat = np.zeros((num_constraints,total_coeffs))
        constraint_mat = np.zeros((num_constraints,))
        big_mat[0:2, :] = self.computeFirstRows(self.ts[0])

        for i in range(1,self.num_wps-1):
            big_mat[4*i - 2:4*i + 2, :] = self.computeRows(self.ts[i], i-1)

        last_wp_idx = self.num_wps - 1  
        big_mat[4*last_wp_idx - 2:4*last_wp_idx , :] = self.computeLastRows(self.ts[-1])

        return big_mat
    

    def get_bmat(self,wp_coords):
        num_constraints = 4*(self.num_wps-2) + 4
        b_mat = np.zeros((num_constraints,1))
        for i in range(self.num_wps):
            if i == 0:
                b_mat[i] = wp_coords[i]
            elif i == len(wp_coords) - 1:
                b_mat[4*i - 2] = wp_coords[i]
            else:
                b_mat[4*i - 2] = wp_coords[i]
                b_mat[4*i - 1] = wp_coords[i]
        return b_mat


    def get_coeffs(self,wp_coords):
        A_mat = self.get_Amat()
        b_mat = self.get_bmat(wp_coords)

        # Solve using Least Squares Method
        # x, residuals, rank, s = np.linalg.lstsq(A_mat, b_mat, rcond=None)
        x = np.dot(np.linalg.pinv(A_mat),b_mat)

        return x 
    

    def give_xyzcoeffs(self):
        dist_list = self.timeinterval_calc()
        wp_xcoords = self.waypoints[:,0]#np.zeros(len(self.waypoints))
        wp_ycoords = self.waypoints[:,1]#np.zeros(len(self.waypoints))
        wp_zcoords = self.waypoints[:,2]#np.zeros(len(self.waypoints))
        # wp_xcoords = np.zeros(len(self.waypoints))
        # wp_ycoords = np.zeros(len(self.waypoints))
        # wp_zcoords = np.zeros(len(self.waypoints))

        # for i in len(self.waypoints):
        #     wp_xcoords[i] = self.waypoints[i].x
        #     wp_ycoords[i] = self.waypoints[i].y
        #     wp_zcoords[i] = self.waypoints[i].z
        
        coeffx = self.get_coeffs(wp_xcoords)
        coeffy = self.get_coeffs(wp_ycoords)
        coeffz = self.get_coeffs(wp_zcoords)

        return coeffx,coeffy,coeffz
    def give_velcoeffs(self, pos_coeffs):
        neqs = self.num_wps - 1
        der_vec = np.array([0, 1, 2, 3])
        wp_xcoords = pos_coeffs[0]
        wp_ycoords = pos_coeffs[1]
        wp_zcoords = pos_coeffs[2]
        # print(np.tile(der_vec, neqs).reshape(-1,1).shape)

        # Repeat der_vec neqs times and elementwise multiply with wp_xcoords, wp_ycoords, and wp_zcoords
        vel_coeffs_x = wp_xcoords*(np.tile(der_vec, neqs).reshape(-1,1))
        vel_coeffs_y = wp_ycoords*(np.tile(der_vec, neqs).reshape(-1,1))
        vel_coeffs_z = wp_zcoords*(np.tile(der_vec, neqs).reshape(-1,1))

        return vel_coeffs_x, vel_coeffs_y, vel_coeffs_z


    
    def give_acccoeffs(self,pos_coeffs):
        neqs = self.num_wps - 1
        der_vec = np.array([0, 0, 2, 6])
        wp_xcoords = pos_coeffs[0]
        wp_ycoords = pos_coeffs[1]
        wp_zcoords = pos_coeffs[2]

        # Repeat der_vec neqs times and elementwise multiply with wp_xcoords, wp_ycoords, and wp_zcoords
        acc_coeffs_x = wp_xcoords*(np.tile(der_vec, neqs).reshape(-1,1))
        acc_coeffs_y = wp_ycoords*(np.tile(der_vec, neqs).reshape(-1,1))
        acc_coeffs_z = wp_zcoords*(np.tile(der_vec, neqs).reshape(-1,1))

        return acc_coeffs_x, acc_coeffs_y, acc_coeffs_z
    

    def get_times(self):
        return self.ts

# def main():
#     Wp = np.array([[5, 17.5, 2],[4.6, 16.5, 2.9],[4.4, 16.3, 3.7],[4.0, 14.4, 4.0],[4.4, 12.8, 4.1],[4.7, 10.9, 3.6],[4.8, 10.5, 3.5],[5.0, 8.7, 3.0],[5.3, 7.7, 2.6],[5.4, 6.6, 2.5],[5.6, 5.6, 2.3],[5.6, 3.7, 2.2],[5.6, 1.9, 2.3],[5.4, 0.2, 1.9],[5.4, -1.3, 2.8],[5.4, -2.6, 3.4],[5, -3, 3.5]])
#     envi = None
#     traj = traj_gen(envi,Wp)
#     timesteps = traj.ts
#     coeffx, coeffy, coeffz = traj.give_xyzcoeffs()
#     vcoeffx, vcoeffy, vcoeffz = traj.give_velcoeffs((coeffx, coeffy, coeffz))

#     print(traj.get_Amat().shape)

# if __name__ == "__main__":
#     main()