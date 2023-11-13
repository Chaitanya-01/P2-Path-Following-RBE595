from djitellopy import Tello
import time
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os
import threading



def printodo(tello):# Save the odometry of the drone
  odometry = []
  while True:
    with open('odometry.csv', mode='w', newline='') as file:
            writer = csv.DictWriter(file, filednames=[ 'mpry', 'baro', 'bat', 'mid', 'h', 'agz', 'temph', 'vgz', 'roll', 'agy', 'yaw', 'pitch', 'vgy', 'time', 'vgx', 'templ', 'agx', 'tof'])
            writer.writeheader()
            writer.writerow(tello.get_current_state())
    # odometry.append(tello.get_current_state())
    print(tello.get_current_state())

def tellorun(tello): # Run the tello drone

    tello.connect()
    power = tello.get_battery()
    print("Power= ", power)
    tello.takeoff()
    speeds = 90  # tune
    for i in range(len(rel_x)):
        tello.go_xyz_speed(rel_x[i], rel_y[i], rel_z[i], speeds)
        time.sleep(0.2)  # tune
    tello.land()
    conditions = False
    flight_time = tello.get_flight_time()
    print("Flight Time: ", flight_time)



if __name__ == "__main__":
    tello = Tello()
    csv_filepath = 'path_to_follow.csv'
    # Lists to hold the coordinates
    x_coords = []
    y_coords = []
    z_coords = []
    # Open and read the CSV
    with open(csv_filepath, mode='r', newline='') as file:
        reader = csv.reader(file)
        # print(reader)
        # Read x coordinates
        x_coords = [float(i) for i in next(reader)]
        # Read y coordinates
        y_coords = [float(i) for i in next(reader)]
        # Read z coordinates
        z_coords = [float(i) for i in next(reader)]

        # Print out the lists (for testing)

    x_coords = [int(x*100) for x in x_coords]
    y_coords = [int(y*100) for y in y_coords]
    z_coords = [int(z*100) for z in z_coords]
    print(len(x_coords))
    rel_x = []
    rel_y = []
    rel_z = []
    rel_x.append(x_coords[0])
    rel_y.append(y_coords[0])
    rel_z.append(z_coords[0])
    for i in range(len(x_coords)-1):
        rel_x.append(x_coords[i+1]-x_coords[i])
        rel_y.append(y_coords[i+1]-y_coords[i])
        rel_z.append(z_coords[i+1]-z_coords[i])
    for i in range(len(x_coords)):
        print("x, y, z are abs: ", x_coords[i], y_coords[i], z_coords[i])
    for i in range(len(rel_x)):
        print("x, y, z are: ", rel_x[i], rel_y[i], rel_z[i])
    fig1 = plt.figure()
    ax = plt.axes(projection='3d')
    # ax.plot3D(self.MP[0, :], self.MP[1, :],self.MP[2, :], 'green')
    ax.plot3D(x_coords, y_coords, z_coords, 'green', label='abs')
    ax.set_title('3D trajectory visualization')
    ax.legend(loc='upper right')
    ax.grid(True)
    plt.show()
    conditions = True
    try:
        t1 = threading.Thread(target=tellorun, args=(tello,))
        t2 = threading.Thread(target=printodo, args=(tello,))
        # starting thread 1
        t1.start()
        # starting thread 2
        t2.start()


        # both threads completely executed
        print("Done!")
    except KeyboardInterrupt:
        # HANDLE KEYBOARD INTERRUPT AND STOP THE DRONE COMMANDS
        print('keyboard interrupt')
        tello.emergency()
        tello.emergency()
        tello.end()
