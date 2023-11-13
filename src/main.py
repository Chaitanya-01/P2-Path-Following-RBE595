"""
main.py
    Simulates the UAV dynamics, cameras and visualizes output
    This entrypoint file runs within Blender's python interpreter

    How to setup Blender and Blender python interpreter?
        Install Blender (this code was tested on version 3.6)

        Ubuntu is officially supported. In case you are using Windows or Mac, these instructions may or may not work
              
        1. Find the python interpreter location associated with the Blender:
            a. Open Blender interactive console within Blender
            b. import sys
            c. print(sys.executable) 
        2. Use pip to install the dependencies from command prompt/terminal (I dont think it worked with powershell though). It will throw a warning and install to something called user site
            "python path" -m pip install imath numpy opencv-python scipy pyquaternion
            For example,
            Windows command looks like this,
                "C:\Program Files\Blender Foundation\Blender 3.6\3.6\python\bin\python.exe" -m pip install opencv-python ...
            Linux command looks like this in my setup,
                /usr/bin/python3.10 -m pip install opencv-python ...
        3. Reopen blender

    How to run the script?
        Create a folder called outputs and open main.blend file
        Goto scripting tab, associate it with main.py if not done already
        Run the script. It takes about 10 seconds to execute
        Goto animation tab and press space to see the visualization
    
    Based on Prof. Nitin's work https://umdausfire.github.io/teaching/fire298/asn3.html
"""

import bpy
import sys
import site
import csv


# PATH CONFIGURATION
user_site_packages =site.getusersitepackages()
sys.path.append(user_site_packages) #For pip installed dependencies
sys.path.append('./src')

# IMPORT PIP LIBS
import importlib
import math
import os
import random
import numpy as np
import cv2
import scipy


#import OpenEXR
    
# IMPORT DYNAMICS, CONTROL and USER CODE
import quad_dynamics as qd
import control
import tello
import frame_utils as frame
import rendering
import usercode
import create_env
import rrt_star
# import traj_gen
import trajectory_gen
# from trajGen import trajGenerator


# Force reload custom modules and run the latest code
importlib.reload(control)
importlib.reload(qd)
importlib.reload(tello)
importlib.reload(frame)
importlib.reload(rendering)
importlib.reload(usercode)

def main():
    # for debugging use print() and blender console.
    #bpy.ops.wm.console_toggle()
    
    # CONSTANTS
    fps = 20

    
    
    
    filepath = "./src/sample_maps/map3.txt" # Uncomment for sample maps
    # filepath = "./TrainSetP2b/maps/map1.txt" # uncomment for real map
    # Render map
    envi = create_env.Environment(filepath)
    envi.make_env()   
    map_array_scale = envi.get_map_array_scale()
    print("scale used for map :", map_array_scale)

    #---Uncomment the start and goals of respective maps-----------------------------------
    # start = [5,16, 3] #sample map4.txt
    # goal = [24, 16, 3]
    # start = [5,17.5,2] #sample map1.txt
    # goal = [5,-3,3.5]
    start = [0.5,2.5,5]  #sample map 3
    goal = [18.5,2.5,5]
    # start = [0.3,-3,2]  #sample map 2
    # goal = [9.8,29.5,3]
    # start = [0.,0.,0.]  # map 1 real trees
    # goal = [-0.59,6.81,0]

    #---------Run RRT* Algorithm----------------------------------------
    rrt_st = rrt_star.RRT(envi,start,goal)
    path = rrt_st.RRT_star()
    path_array = rrt_st.get_array(path)
    print(path_array)
    print(len(rrt_st.get_vertices()))
    envi.visualize_nodes(path)

    #-------Run minimum snap trajectory generator-----------------------------
    traj = trajectory_gen.trajectory_gen(envi,path_array,0.5,1e4,filepath)
    print("total time",traj.ts[-1])
    # STOP time for simulation
    sim_stop_time = int(traj.ts[-1]) + 3
    final_traj = traj.final_traj()
    print("final trajectory array is",final_traj)
    print("final traj array shape",final_traj.shape)
    trajpath = "./src/sample_traj/trajnew.csv"
    np.savetxt(trajpath,final_traj,delimiter=',') # Save trajectory
    

    # INIT RENDERING AND CONTROL
    controller = control.quad_control()
    user_sm = usercode.state_machine()
    rendering.init() 
    bpy.context.scene.render.fps = fps
    bpy.context.scene.frame_end = fps*sim_stop_time

    # SET TIME STEP
    dynamics_dt = 0.01
    control_dt = controller.dt
    user_dt = user_sm.dt
    frame_dt = 1./fps

    # INIT STATES
    current_time = 0.
    # Uncomment the INIT location based on map being used
    xyz = np.array([0.5,-2.5,-5.])# sample map3
    # xyz = np.array([0.3,3,-2.])# sample map2
    # xyz = np.array([0.,-0.,-0.])# map1 real trees
    # xyz = np.array([5,-17.5,-2])# sample map1
    # xyz = np.array([5,-16, -3])# sample map4
    vxyz = np.array([0.0, 0.0, 0.0])
    quat = np.array([1.0, .0, .0, .0])
    pqr = np.array([0.0, .0, .0])
    current_ned_state = np.concatenate((xyz, vxyz, quat, pqr))    
    
    # INIT TIMER
    dynamics_countdown = 0.
    control_countdown = 0.
    frame_countdown = 0.
    user_countdown = 0.
    
    # INIT LOG
    stateArray = current_ned_state
    timeArray = 0
    controlArray = np.array([0., 0, 0, 0])

    
     
    
    
 

    # SCHEDULER SUPER LOOP
    # --------------------------------------------------------------------------------------------
    while current_time < sim_stop_time:
        if frame_countdown<=0.:
            rendering.stepBlender(current_ned_state)
            frame_countdown = frame_dt

        if user_countdown<=0.:
            xyz_ned = current_ned_state[0:3]
            xyz_blender = [xyz_ned[0], -xyz_ned[1], -xyz_ned[2]]

            vxyz_ned = current_ned_state[3:6]
            vxyz_blender = [vxyz_ned[0], -vxyz_ned[1], -vxyz_ned[2]]

            xyz_bl_des, vel_bl_des, acc_bl_des, yaw_bl_setpoint = user_sm.step(current_time, xyz_blender, vxyz_blender)

            yaw_ned = -yaw_bl_setpoint
            WP_ned = np.array([xyz_bl_des[0], -xyz_bl_des[1], -xyz_bl_des[2], yaw_ned])
            vel_ned = np.array([vel_bl_des[0], -vel_bl_des[1], -vel_bl_des[2]])
            acc_ned = np.array([acc_bl_des[0], -acc_bl_des[1], -acc_bl_des[2]])
            

            user_countdown = user_dt

        if control_countdown<=0.:
            U = controller.step(current_ned_state, WP_ned, vel_ned, acc_ned)
            control_countdown = control_dt

        # Dynamics runs at base rate. 
        #   TODO replace it with ODE4 fixed step solver
        current_ned_state = current_ned_state + dynamics_dt*qd.model_derivative(current_time,
                                                            current_ned_state,
                                                            U,
                                                            tello)
        
        # UPDATE COUNTDOWNS AND CURRENT TIME
        dynamics_countdown -= dynamics_dt
        control_countdown -= dynamics_dt
        frame_countdown -= dynamics_dt
        user_countdown -=dynamics_dt
        current_time += dynamics_dt

        # LOGGING
        stateArray = np.vstack((stateArray, current_ned_state))
        controlArray = np.vstack((controlArray, U))
        timeArray = np.append(timeArray, current_time)
    # ----------------------------------------------------------------------------------------------
    user_sm.terminate()

    # SAVE LOGGED SIGNALS TO MAT FILE FOR POST PROCESSING IN MATLAB
    loggedDict = {'time': timeArray,
                  'state': stateArray,
                  'control': controlArray}  
    scipy.io.savemat('./log/states.mat', loggedDict)
    
if __name__=="__main__":
    # donot run main.py if imported as a module
    main()
    
    
    
     