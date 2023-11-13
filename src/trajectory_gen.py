import math
import numpy as np
import scipy
from scipy import optimize
from rrt_star import Node
import create_env

"""
This class takes the waypoints of RRT* and genereates a minimum snap smooth trajectory.
Theory for this is based on the following paper below.
Richter, Charles, Adam Bry, and Nicholas Roy. "Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments." Robotics Research: The 16th International Symposium ISRR. Springer International Publishing, 2016.
"""
class trajectory_gen:
    def __init__(self, envi,waypoints, v_max,Kt, filepath):
        self.envi = envi
        self.filepath = filepath # Map text
        # Read map file and get the boundaries, obstacle info
        try:
            # Read the file content
            with open(self.filepath, 'r') as f:
                content = f.read()
        except FileNotFoundError:
            return
        
        # Process the content
        lines = content.split('\n')
        blocks_list = []
        for line in lines:
            words = line.split()
            if words:
                if words[0] == 'boundary':
                    boundary = [float(i) for i in words[1:7]]
                    self.lowerboundary = np.array(
                        [boundary[0], boundary[1], boundary[2]])
                    self.upperboundary = np.array(
                        [boundary[3], boundary[4], boundary[5]])
                elif words[0] == 'block':
                    block_coords = [float(i) for i in words[1:7]]
                    blocks_list.append([float(i) for i in words[1:7]])
                    # color = [int(i)/255 for i in words[7:10]]
        self.blocks_list = blocks_list
        print("List of lower bounds",self.lowerboundary)
        print("List of upper bounds",self.upperboundary)
        print("List of all the blocks",self.blocks_list)


        self.Kt = Kt # Time penalty
        self.v_max = v_max # upper bound on velocity
        self.waypoints = waypoints #(n,3) numpy array
        num_wps,point_size = waypoints.shape #no of points,3
        self.num_wps = num_wps
        self.point_size = point_size
        self.order = 9 # Each segment is a 9th order polynomial 
        self.coeffs_per_eqn = self.order + 1
        self.ts = np.zeros(num_wps) # Array of times at each waypoints (could also be initiated by random sampling between two times)
        self.minimize()
        self.yaw = 0
        self.heading = np.zeros(2)

    def costfunc(self,t):
        J,p = self.minsnaptraj(t)
        J = J + self.Kt*np.sum(t)
        return J
    def minimize(self):
        # minimum value for time intervals for segments is calculated from max velocity
        dist = np.linalg.norm(self.waypoints[0:-1]-self.waypoints[1:],axis=-1) #returns an array of distances between successive waypoints
        tmin = dist/self.v_max
        
        t = optimize.minimize(self.costfunc, tmin, method="COBYLA", constraints=({'type':'ineq', 'fun':lambda t: t-tmin}))['x']
        self.ts[1:] = np.cumsum(t) #getting the times at each waypoint
        self.cost, self.coeffs = self.minsnaptraj(t)
    
    def minsnaptraj(self,t): # t is an array of all the time intervals corresponding to each segment
        # len(t) = num_wps-1
        num_segments = len(t)
        #--------Formulate the Q matrix--------------------------------------------
        Q = np.zeros((self.coeffs_per_eqn*(num_segments),self.coeffs_per_eqn*(num_segments)))
        # We are calculating minimum snap trajectory
        # We need derivatives from 0 to 4 of the polynomial
        r = 4 
        for i in range(num_segments):
            m = np.arange(0,r,1) # variable obtained when taking the derivative of rth order eqn
            # l,k go from 0 to N  where N is the order of polynomial
            for l in range(self.coeffs_per_eqn):
                for k in range(self.coeffs_per_eqn):
                    if l >= r and k >= r:
                        power = l+k-2*r+1
                        # print("power is",power)
                        Q[self.coeffs_per_eqn*i+l,self.coeffs_per_eqn*i+k] = 2*np.prod((l-m)*(k-m))*(t[i]**power/power) #Qkl=Qlk
        
        #--------Formulate the equality constraint Ax=b---------------------------
        # print(Q)
        A = np.zeros((self.coeffs_per_eqn*num_segments,self.coeffs_per_eqn*num_segments))
        b = np.zeros((self.coeffs_per_eqn*num_segments,self.point_size))
        # Waypoint constraints
        b[:num_segments,:] = self.waypoints[:-1,:]
        b[num_segments:2*num_segments,:] = self.waypoints[1:,:]
        for i in range(num_segments):
            A[i, self.coeffs_per_eqn*i:self.coeffs_per_eqn*(i+1)] = self.derivative(0) #polynomial in time after taking derivative
            A[i+num_segments, self.coeffs_per_eqn*i:self.coeffs_per_eqn*(i+1)] = self.derivative(t[i])
        # Continuity constraints
        for i in range(num_segments-1):
            A[2*num_segments+4*i:2*num_segments+4*(i+1), self.coeffs_per_eqn*i:self.coeffs_per_eqn*(i+1)] = -self.derivative(t[i],'all')
            A[2*num_segments+4*i:2*num_segments+4*(i+1), self.coeffs_per_eqn*(i+1):self.coeffs_per_eqn*(i+2)] = self.derivative(0,'all')
        # Fixed points at the start and end
        A[6*num_segments-4:6*num_segments, :self.coeffs_per_eqn] = self.derivative(0,'all')
        A[6*num_segments:6*num_segments+4, -self.coeffs_per_eqn:] = self.derivative(t[-1],'all')
        for i in range(1,num_segments):
            A[6*num_segments+4*i:6*num_segments+4*(i+1), self.coeffs_per_eqn*i:self.coeffs_per_eqn*(i+1)] = self.derivative(0,'all')
        # print(A[:10,:])
        #--------Unconstrained QP-------------------------------------------------
        # In unconstrained QP interval times are updated or minimized instead of coeffs
        A_inverse = np.linalg.inv(A)
        # Getting optimal values for free derivatives
        idx = 4*(self.num_wps-2) #index dividing the fixed derivatives and free derivatives
        if idx!=0:
            R = np.dot(A_inverse.T,np.dot(Q,A_inverse))
            Rfp = R[:-idx, -idx:]
            Rpp = R[-idx:, -idx:]
            b[-idx:,] = -np.dot(np.linalg.inv(Rpp),np.dot(Rfp.T,b[:-idx,])) #Getting the free derivatives
        
        # Coefficient matrix p
        p = np.dot(A_inverse,b)
        J = np.trace(np.dot(p.T,np.dot(Q,p)))
        # Return cost and the coefficients
        return J, p
    def derivative(self,t,r=0):
        if r =='all': # returns all the derivatives from 1 to 4
            # print(self.derivative(t,r) for r in range(1,5))
            polynomial = np.array([self.derivative(t,r) for r in range(1,5)])
        else:
            polynomial = np.zeros(self.coeffs_per_eqn)
            deriv = np.polyder([1]*self.coeffs_per_eqn,r)[::-1]
            # deriv = deriv[::-1] #reversing the order to start from 0th power and so on
            tpower = t**np.arange(0,self.coeffs_per_eqn-r,1)
            polynomial[r:] = deriv*tpower
        return polynomial
    def trajectories(self,t,i):
        if t > self.ts[-1]: t = self.ts[-1] - 0.001
        i = np.where(t >= self.ts)[0][-1]

        t = t - self.ts[i]
        #coeff of ith eqn
        coeff = (self.coeffs.T)[:,self.coeffs_per_eqn*i:self.coeffs_per_eqn*(i+1)]
        pos = np.dot(coeff,self.derivative(t))
        vel = np.dot(coeff,self.derivative(t,1))
        acc = np.dot(coeff,self.derivative(t,2))
        jerk = np.dot(coeff,self.derivative(t,3))
        return pos,vel,acc,jerk




    def final_traj(self): # Run the whole code
        dt = 0.05
        # final_traj = np.array((11,self.ts[-1]))
        pos_x = []
        pos_y = []
        pos_z = []
        yaw_list = []
        vel_x = []
        vel_y = []
        vel_z = []
        yawdot_list = []
        acc_x = []
        acc_y = []
        acc_z = []
        jerk_x = []
        jerk_y = []
        jerk_z = []
        orange_color = (1, 0.4, 0, 1)  # Orange
        print("All waypoint",self.waypoints)
        print("Number of waypoints", self.num_wps) 
        for i in range(len(self.ts)-1):
            start_t = self.ts[i]
            end_t = self.ts[i+1]
            for t in np.arange(start_t,end_t,dt):
                pos, vel, acc, jerk = self.trajectories(t,i)
                yaw, yawdot = self.get_yaw(vel[:2])
                #----------------------------------------------------
                print("Performing trajectory collision check")
                out_of_bounds = False
                collided = False
                if  (pos[0] < self.lowerboundary[0] or pos[0] > self.upperboundary[0]) or (pos[1] < self.lowerboundary[1] or pos[1] > self.upperboundary[1]) or (pos[2] < self.lowerboundary[2] or pos[2] > self.upperboundary[2]):
                    out_of_bounds = True
                for j in range(len(self.blocks_list)):
                    if (self.blocks_list[j][0]<= pos[0] <= self.blocks_list[j][3]) and (self.blocks_list[j][1]<= pos[1] <= self.blocks_list[j][4]) and (self.blocks_list[j][2]<= pos[2] <= self.blocks_list[j][5]):
                        collided = True
                if collided or out_of_bounds:

                    # Get the intermediate waypoint where collision is happening and add it to list of waypoints
                    new_waypoint = (self.waypoints[i+1,:]+self.waypoints[i,:])/2
                    self.waypoints = np.insert(self.waypoints, i+1,new_waypoint,axis=0)
                    num_wps,_ = self.waypoints.shape 
                    self.num_wps = num_wps
                    self.ts = np.zeros(num_wps)
                    if collided:
                        print(f"collision occured between {i} and {i+1} waypoints")
                    if out_of_bounds:
                        print(f"out of bounds between {i} and {i+1} waypoints")
                    self.minimize()
                    self.final_traj()   
                else:
                    
                    pos_x.append(pos[0])
                    pos_y.append(pos[1])
                    pos_z.append(pos[2])
                    yaw_list.append(yaw)

                    vel_x.append(vel[0])
                    vel_y.append(vel[1])
                    vel_z.append(vel[2])
                    yawdot_list.append(yawdot)

                    acc_x.append(acc[0])
                    acc_y.append(acc[1])
                    acc_z.append(acc[2])
                    jerk_x.append(jerk[0])
                    jerk_y.append(jerk[1])
                    jerk_z.append(jerk[2])
                    loc = (pos[0],pos[1],pos[2])
                    self.envi.add_sphere(loc,orange_color)
        print("No more collisions and out of bounds left")
        pos_x_arr = np.array(pos_x).squeeze()
        pos_y_arr = np.array(pos_y).squeeze()
        pos_z_arr = np.array(pos_z).squeeze()
        yaw_arr = np.array(yaw_list).squeeze()
        vel_x_arr = np.array(vel_x).squeeze()
        vel_y_arr = np.array(vel_y).squeeze()
        vel_z_arr = np.array(vel_z).squeeze()
        yawdot_arr = np.array(yawdot_list).squeeze()
        acc_x_arr = np.array(acc_x).squeeze()
        acc_y_arr = np.array(acc_y).squeeze()
        acc_z_arr = np.array(acc_z).squeeze()
        final_traj = np.vstack((pos_x_arr,pos_y_arr,pos_z_arr,yaw_arr,vel_x_arr,vel_y_arr,vel_z_arr,yawdot_arr,acc_x_arr,acc_y_arr,acc_z_arr))
        return final_traj
    def get_yaw(self,vel):
        curr_heading = vel/np.linalg.norm(vel)
        prev_heading = self.heading
        # print(self.heading)
        cosine = max(-1,min(np.dot(prev_heading, curr_heading),1))
        dyaw = np.arccos(cosine)
        norm_v = np.cross(prev_heading,curr_heading)
        self.yaw += np.sign(norm_v)*dyaw

        if self.yaw > np.pi: self.yaw -= 2*np.pi
        if self.yaw < -np.pi: self.yaw += 2*np.pi

        self.heading = curr_heading
        yawdot = max(-30,min(dyaw/0.005,30))
        return self.yaw,yawdot


    

# def main():
#     # Wp = np.array([[5, 17.5, 2],[4.6, 16.5, 2.9],[4.4, 16.3, 3.7],[4.0, 14.4, 4.0],[4.4, 12.8, 4.1],[4.7, 10.9, 3.6],[4.8, 10.5, 3.5],[5.0, 8.7, 3.0],[5.3, 7.7, 2.6],[5.4, 6.6, 2.5],[5.6, 5.6, 2.3],[5.6, 3.7, 2.2],[5.6, 1.9, 2.3],[5.4, 0.2, 1.9],[5.4, -1.3, 2.8],[5.4, -2.6, 3.4],[5, -3, 3.5]])
#     # envi = None
#     # traj = traj_gen(envi,Wp)
#     # timesteps = traj.ts
#     # coeffx, coeffy, coeffz = traj.give_xyzcoeffs()
#     # vcoeffx, vcoeffy, vcoeffz = traj.give_velcoeffs((coeffx, coeffy, coeffz))

#     # print(traj.get_Amat().shape)
#     mywaypoints = np.genfromtxt('pathkotha.csv', delimiter=',', skip_header=0).T
#     print("newwaypoints",mywaypoints)
#     traj = trajectory_gen(None,mywaypoints,5,1e6)
#     final = traj.final_traj()
#     # traj = trajGenerator(mywaypoints, max_vel = 5, gamma = 1e6)
#     # final = traj.final_traj()
#     # des_state = traj.get_des_state(1.05)
#     posn =final[0:3,:].T
#     print(posn)
#     # print(traj.coeffs)

# if __name__ == "__main__":
#     main()
    
