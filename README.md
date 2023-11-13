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
On sample maps in blender:\
Sample map 1:\
Watch the full simulation for sample map 1 [here](https://youtu.be/_bZi1fZofUs)
<p float="middle">
<img src="media/samplemap1.PNG" />
</p>

<p float="middle">
	<img src="media/samplemap1_frontview.gif" width="350" height="350" title="Front view"/> 
	<img src="media/samplemap1_sideview.gif" width="350" height="350" title="Side View"/>
</p>

Sample map 3:\
Watch the full simulation for sample map 3 [here](https://youtu.be/z33Q_nKZx80)
<p float="middle">
<img src="media/samplemap3.PNG" />
</p>

<p float="middle">
	<img src="media/samplemap3_frontview.gif" width="350" height="350" title="Front view"/> 
	<img src="media/samplemap3_sideview.gif" width="350" height="350" title="Side View"/>
</p>


## References
1. Richter, Charles, Adam Bry, and Nicholas Roy. "Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments." Robotics Research: The 16th International Symposium ISRR. Springer International Publishing, 2016.


## Collaborators
Chaitanya Sriram Gaddipati - cgaddipati@wpi.edu

Shiva Surya Lolla - slolla@wpi.edu

Ankit Talele - amtalele@wpi.edu
