# Quadcopter motion planning stack
In this project a RRT* path planning and minimum snap trajectory generation (motion planning) stack is implemented for the 3D navigation of a DJI Tello Edu quadcopter in simulation (in blender) and is then tested on real quadcopter via NVIDIA Jetson Orin Nano. Additionally a cascaded PID controller gains for position and velocity control are tuned to follow the generated trajectory. 
(Check the full problem statements here [project2a](https://rbe549.github.io/rbe595/fall2023/proj/p2a/) and [project2b](https://rbe549.github.io/rbe595/fall2023/proj/p2b/))
## Steps to run the code
- This code is tested on Ubuntu 20.04 system
- Install Numpy, Scipy, Matplotlib, blender python, pyquaternion, djitellopy libraries before running the code.
- To run the simulation:
	-  open the `main.blend` file in blender
	- Go to compositing and set appropriate path to `outputs` folder
   	- Go to scripting tab and load the `main.py` file
   	- In `main.py` choose a map from the given set of four sample maps and uncomment the code accordingly to set start and goal locations for each map.
   	- Run the script in blender.
   	- Once the script is run press `spacebar` to run the simulation in layout tab.
- To run on real quadcopter
	- Set up the tello drone and orin nano
   	- On orin nano clone this repository
   	- Copy the trajectory csv file generated in the previous simulation from `sample_traj` folder to `src` folder
   	- Open the `tello_run.py` file and set the appropriate trajectory file name.
   	- Connect to the network of the tello drone and run in `src` folder the following command:
   	  ```
	  python3 tello_run.py
	  ```

## Report
For detailed description of the math see the report [here](Report.pdf).
## Plots and Animations
For the train data 1, plots and animation showing roll, pitch, and yaw for all the filters:
<p float="middle">
<img src="outputs/p1a.png" width="750" height="450"/>
<img src="outputs/p1b.png" width="750" height="450"/>
</p>
<p float="middle">
<img src="outputs/output1.gif" width="750" height="350"/>
</p>

Remaining plots are present in the report and links to rest of the animations are 
[train1](https://www.youtube.com/watch?v=QqZrlZt3IWk), [train2](https://youtu.be/YaMS5Z0NG9c), [train3](https://youtu.be/Bt4ej2pWsNQ), [train4](https://youtu.be/VEVUZr9buow), [train5](https://youtu.be/5XoWXI-sQrE), [train6](https://youtu.be/J3JOtn7tDPE).

## References
1. Richter, Charles, Adam Bry, and Nicholas Roy. "Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments." Robotics Research: The 16th International Symposium ISRR. Springer International Publishing, 2016.


## Collaborators
Chaitanya Sriram Gaddipati - cgaddipati@wpi.edu

Shiva Surya Lolla - slolla@wpi.edu

Ankit Talele - amtalele@wpi.edu
