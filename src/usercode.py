import numpy as np
import cv2
import bpy
import os
import scipy
import matplotlib.pyplot as plt
from datetime import datetime

class state_machine:
    def __init__(self):
        # User code executes every dt time in seconds
        self.dt = 0.050

        ### Motion Profile - sample code - EDIT HERE! ######################################################
        # For tuning the PID, utilize the following code. Once tuned, modify this section as you want! 
        # self.MP = np.genfromtxt('./src/sample_traj/MP.csv', delimiter=',', skip_header=0)
        self.MP = np.genfromtxt('./src/sample_traj/trajnew.csv', delimiter=',', skip_header=0)
        # zero =np.array([0,0,0,0,0,0,0,0,0])
        # self.MP = np.concatenate((zero,self.MP),axis=0)
        print("shape of new traj is11,",self.MP.shape)
        self.activeIndex = 0

        userstates = scipy.io.loadmat('./log/user_states.mat')
        t = userstates['time']
        x = userstates['x']
        y = userstates['y']
        z = userstates['z']
        xdes = userstates['x_des']
        ydes = userstates['y_des']
        zdes = userstates['z_des']
        vx = userstates['vx']
        vy = userstates['vy']
        vz = userstates['vz']
        vxdes = userstates['vx_des']
        vydes = userstates['vy_des']
        vzdes = userstates['vz_des']
        fig1 = plt.figure()
        ax = plt.axes(projection ='3d')
        # ax.plot3D(self.MP[0, :], self.MP[1, :],self.MP[2, :], 'green')
        ax.plot3D(xdes, ydes,zdes, 'green',label='desired')
        ax.plot3D(x,y,z,'red',label='actual')
        ax.set_title('3D trajectory visualization')
        ax.legend(loc='upper right')
        ax.grid(True)
        plt.show()
        #-------------------
        fig2, axs = plt.subplots(2, 3, figsize=(15, 8))

        # Plot the data in each subplot
        axs[0, 0].plot(t,xdes,label='xdes')
        axs[0, 0].plot(t,x,label='x')
        axs[0, 0].set_title('Position X')
        axs[0, 0].set_ylabel('X (m)')
        axs[0, 0].set_xlabel('time (s)')
        axs[0, 0].legend(loc='upper right')
        axs[0, 0].grid(True)
        axs[0, 1].plot(t,ydes,label='ydes')
        axs[0, 1].plot(t,y,label='y')
        axs[0, 1].set_title('Position Y')
        axs[0, 1].set_ylabel('Y (m)')
        axs[0, 1].set_xlabel('time (s)')
        axs[0, 1].legend(loc='upper right')
        axs[0, 1].grid(True)
        axs[0, 2].plot(t,zdes,label='zdes')
        axs[0, 2].plot(t,z,label='z')
        axs[0, 2].set_title('Position Z')
        axs[0, 2].set_ylabel('Z (m)')
        axs[0, 2].set_xlabel('time (s)')
        axs[0, 2].legend(loc='upper right')
        axs[0, 2].grid(True)
        axs[1, 0].plot(t,vxdes,label='vxdes')
        axs[1, 0].plot(t,vx,label='vx')
        axs[1, 0].set_title('Velcoity X')
        axs[1, 0].set_ylabel('Vx (m/s)')
        axs[1, 0].set_xlabel('time (s)')
        axs[1, 0].legend(loc='upper right')
        axs[1, 0].grid(True)
        axs[1, 1].plot(t,vydes,label='vydes')
        axs[1, 1].plot(t,vy,label='vy')
        axs[1, 1].set_title('Velcoity Y')
        axs[1, 1].set_ylabel('Vy (m/s)')
        axs[1, 1].set_xlabel('time (s)')
        axs[1, 1].legend(loc='upper right')
        axs[1, 1].grid(True)
        axs[1, 2].plot(t,vzdes,label='vzdes')
        axs[1, 2].plot(t,vz,label='vz')
        axs[1, 2].set_title('Velcoity Z')
        axs[1, 2].set_ylabel('Vz (m/s)')
        axs[1, 2].set_xlabel('time (s)')
        axs[1,2].legend(loc='upper right')
        axs[1, 2].grid(True)
        
        # Adjust layout
        plt.tight_layout()
        # Show the plot
        plt.show()
        ####################################################################################################


        # Logger
        self.time_array = 0
        self.x_sp_array = 0
        self.y_sp_array = 0
        self.z_sp_array = 0

        self.x_array = 0
        self.y_array = 0
        self.z_array = 0

        self.vx_array = 0
        self.vy_array = 0
        self.vz_array = 0
        self.vx_sp_array = 0
        self.vy_sp_array = 0
        self.vz_sp_array = 0

    def step(self, time, currpos, currvel):
        """
        Input: time, current position in blender frame (x, y, z)
        Output: desired position, velocity, acceleration, yaw (radians) as np.array([x_component, y_component, z_component]) in blender frame
        
        SI unit unless specified otherwise

        Sample Input:
            time = 0.0;
            currpos = np.array([1.0, 0.0, 1.0])
            currvel = np.array([1.0, 0.0, 1.0])

            Sample Output:
            xyz_desired = np.array([0.0, 0.0, 0.0])
            vel_desired = np.array([0.0, 0.0, 0.0]) 
            acc_desired = np.array([0.0, 0.0, 0.0])
            yaw_setpoint = 0.0


        """
        # print("time is",time)
        # EDIT HERE ###########################################################################################
        
        # FOLLOW GENERATED TRAJECTORY
        # print(self.MP.shape)
        # x = 0.002*time**3+0.2*time**2+time+1
        # y = 0.001*time**3+0.3*time**2+2*time+2
        # z = 0.002*time**3+0.3*time**2+2*time+2
        # vx = 0.006*time**2+0.4*time+1
        # vy = 0.003*time**2+0.6*time+2
        # vz = 0.006*time**2+0.6*time+2
        # ax = 0.012*time+0.4
        # ay = 0.006*time+0.6
        # az = 0.012*time+0.6
        # xyz_desired = np.array([x,y,z])
        # vel_desired = np.array([vx,vy,vz])
        # acc_desired = np.array([ax,ay,az])
        # print(self.MP[:3, self.activeIndex])
        xyz_desired = self.MP[:3, self.activeIndex]
        vel_desired = self.MP[4:7, self.activeIndex]
        acc_desired = self.MP[8:11, self.activeIndex]
        yaw_setpoint = 0.0  # Set this to self.MP[3,self.activeIndex] if yaw control is needed
        # print(self.MP[7,self.activeIndex])#yaw speed
        # print(self.activeIndex)
        

        if self.activeIndex<len(self.MP[1, :])-1:
            self.activeIndex = self.activeIndex+1
        
        #######################################################################################################

        # logger
        self.time_array = np.vstack((self.time_array, time))
        self.x_array = np.vstack((self.x_array, currpos[0]))
        self.y_array = np.vstack((self.y_array, currpos[1]))
        self.z_array = np.vstack((self.z_array, currpos[2]))
        self.x_sp_array = np.vstack((self.x_sp_array, xyz_desired[0]))
        self.y_sp_array = np.vstack((self.y_sp_array, xyz_desired[1]))
        self.z_sp_array = np.vstack((self.z_sp_array, xyz_desired[2]))

        self.vx_array = np.vstack((self.vx_array, currvel[0]))
        self.vy_array = np.vstack((self.vy_array, currvel[1]))
        self.vz_array = np.vstack((self.vz_array, currvel[2]))
        self.vx_sp_array = np.vstack((self.vx_sp_array, vel_desired[0]))
        self.vy_sp_array = np.vstack((self.vy_sp_array, vel_desired[1]))
        self.vz_sp_array = np.vstack((self.vz_sp_array, vel_desired[2]))

        return xyz_desired, vel_desired, acc_desired, yaw_setpoint

    def terminate(self):
        loggedDict = {'time': self.time_array,
                  'x': self.x_array,
                  'y': self.y_array,
                  'z': self.z_array,
                  'x_des': self.x_sp_array,
                  'y_des': self.y_sp_array,
                  'z_des': self.z_sp_array,
                  'vx': self.vx_array,
                  'vy': self.vy_array,
                  'vz': self.vz_array,
                  'vx_des': self.vx_sp_array,
                  'vy_des': self.vy_sp_array,
                  'vz_des': self.vz_sp_array,
                  }  
        scipy.io.savemat('./log/user_states.mat', loggedDict)
        print('user state machine terminted')


    def fetchLatestImage(self):
        # # Fetch image - renders the camera, saves the rendered image to a file and reads from it. 
        # path_dir = bpy.data.scenes["Scene"].node_tree.nodes["File Output"].base_path

        # # Render Drone Camera
        # cam = bpy.data.objects['DownCam']    
        # bpy.context.scene.camera = cam
        # bpy.context.scene.render.filepath = os.path.join(path_dir, 'DownCam_latest.png')
        # bpy.ops.render.render(write_still=True)

        # return cv2.imread(bpy.context.scene.render.filepath)
        #----------------------------------------------------------------------------
        # Fetch image - renders the camera, saves the rendered image to a file and reads from it. 
        path_dir = bpy.data.scenes["Scene"].node_tree.nodes["File Output"].base_path
        
        # Render Drone Camera
        cam = bpy.data.objects['Camera.001']
        bpy.context.scene.camera = cam
        
        # Append a unique timestamp to the file name to prevent overwriting previous images
        timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        file_name = f'Camera001_{timestamp_str}.png'
        bpy.context.scene.render.filepath = os.path.join(path_dir, file_name)
        bpy.ops.render.render(write_still=True)
        
        return cv2.imread(bpy.context.scene.render.filepath)
    
