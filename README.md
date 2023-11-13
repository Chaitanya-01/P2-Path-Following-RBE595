# Quadcopter motion planning stack
In this project a RRT* path planning and minimum snap trajectory generation (motion planning) stack is implemented for the 3D navigation of a DJI Tello Edu quadcopter in simulation and is then tested on real quadcopter via NVIDIA Jetson Orin Nano. Additionally a cascaded PID controller gains for position and velocity control are tuned to follow the generated trajectory. 
(Check the full problem statements here [project2a](https://rbe549.github.io/rbe595/fall2023/proj/p2a/) and [project2b](https://rbe549.github.io/rbe595/fall2023/proj/p2b/))
## Steps to run the code
- Install Numpy, Scipy, and Matplotlib libraries before running the code.
- To run on the first training data in the `Wrapper.py` file in the 'main' function set the variables as:
	IMU_filename = 'imuRaw1' and vicon_filename = 'viconRot1'
- For the other data change the variables accordingly and run the file.
- To generate 3D animations uncomment the specified lines in 'main' function. 
- In Code folder:
  ```
  python Wrapper.py
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
