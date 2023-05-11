import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# create an ArgumentParser object
parser = argparse.ArgumentParser(description='Plot x,y,z values from a CSV file')

# add an argument for the file name
parser.add_argument('file', type=str, help='CSV file name')

# parse the arguments
args = parser.parse_args()

# read the CSV file
data = np.genfromtxt(args.file, delimiter=',', skip_header=1)

# extract: Time,x,y,z,roll,pitch,yaw,vx,vy,vz,wx,wy,wz,w1,w2,w3,w4
time = data[:,0]
x = data[:,1]
y = data[:,2]
z = data[:,3]
roll = data[:,4]
pitch = data[:,5]
yaw = data[:,6]
vx = data[:,7]
vy = data[:,8]
vz = data[:,9]
wx = data[:,10]
wy = data[:,11]
wz = data[:,12]
w1 = data[:,13]
w2 = data[:,14]
w3 = data[:,15]
w4 = data[:,16]

# Create a figure with 3d plot:
#   x,y,z trajectory
# Create another figure with subplots:
#   - Row 1: Three subplots for x, y, z
#   - Row 2: Three subplots for roll, pitch, yaw
#   - Row 3: Three subplots for vx, vy, vz
#   - Row 4: Three subplots for wx, wy, wz

# 1) x,y,z trajectory
fig_3d = plt.figure()
axs_3d = fig_3d.add_subplot(projection='3d')
axs_3d.plot(x, y, z)
axs_3d.set_xlabel('x [m]')
axs_3d.set_ylabel('y [m]')
axs_3d.set_zlabel('z [m]')
axs_3d.set_title('x,y,z trajectory')


fig, axs = plt.subplots(4, 3, sharex=True)

# 0) Three subplots for x, y, z
axs[0,0].plot(time, x)
axs[0,0].set_xlabel('time [s]')
axs[0,0].set_ylabel('x [m]')
axs[0,0].set_title('x')

axs[0,1].plot(time, y)
axs[0,1].set_xlabel('time [s]')
axs[0,1].set_ylabel('y [m]')
axs[0,1].set_title('y')

axs[0,2].plot(time, z)
axs[0,2].set_xlabel('time [s]')
axs[0,2].set_ylabel('z [m]')
axs[0,2].set_title('z')

# 1) Three subplots for vx, vy, vz
axs[1,0].plot(time, vx)
axs[1,0].set_xlabel('time [s]')
axs[1,0].set_ylabel('vx [m/s]')
axs[1,0].set_title('vx')

axs[1,1].plot(time, vy)
axs[1,1].set_xlabel('time [s]')
axs[1,1].set_ylabel('vy [m/s]')
axs[1,1].set_title('vy')

axs[1,2].plot(time, vz)
axs[1,2].set_xlabel('time [s]')
axs[1,2].set_ylabel('vz [m/s]')
axs[1,2].set_title('vz')

# 2) Three subplots for roll, pitch, yaw
axs[2,0].plot(time, roll)
axs[2,0].set_xlabel('time [s]')
axs[2,0].set_ylabel('roll [rad]')
axs[2,0].set_title('roll')

axs[2,1].plot(time, pitch)
axs[2,1].set_xlabel('time [s]')
axs[2,1].set_ylabel('pitch [rad]')
axs[2,1].set_title('pitch')

axs[2,2].plot(time, yaw)
axs[2,2].set_xlabel('time [s]')
axs[2,2].set_ylabel('yaw [rad]')
axs[2,2].set_title('yaw')

# 4) Three subplots for wx, wy, wz
axs[3,0].plot(time, wx)
axs[3,0].set_xlabel('time [s]')
axs[3,0].set_ylabel('wx [rad/s]')
axs[3,0].set_title('wx')

axs[3,1].plot(time, wy)
axs[3,1].set_xlabel('time [s]')
axs[3,1].set_ylabel('wy [rad/s]')
axs[3,1].set_title('wy')

axs[3,2].plot(time, wz)
axs[3,2].set_xlabel('time [s]')
axs[3,2].set_ylabel('wz [rad/s]')
axs[3,2].set_title('wz')

# Four subplots for w1, w2, w3, w4
fig_w, axs_w = plt.subplots(1, 4, sharex=True)

axs_w[0].plot(time, w1)
axs_w[0].set_xlabel('time [s]')
axs_w[0].set_ylabel('w1 [rad/s]')
axs_w[0].set_title('w1')

axs_w[1].plot(time, w2)
axs_w[1].set_xlabel('time [s]')
axs_w[1].set_ylabel('w2 [rad/s]')
axs_w[1].set_title('w2')

axs_w[2].plot(time, w3)
axs_w[2].set_xlabel('time [s]')
axs_w[2].set_ylabel('w3 [rad/s]')
axs_w[2].set_title('w3')

axs_w[3].plot(time, w4)
axs_w[3].set_xlabel('time [s]')
axs_w[3].set_ylabel('w4 [rad/s]')
axs_w[3].set_title('w4')

# Add the code to enable quitting on pressing 'q'
def press(event):
    if event.key == 'q':
        plt.close()

# Connect the key press event to the figure
fig.canvas.mpl_connect('key_press_event', press)
fig_w.canvas.mpl_connect('key_press_event', press)
fig_3d.canvas.mpl_connect('key_press_event', press)

# Display the plots
plt.show(block=False)

# Wait for the user to press enter in the terminal
input("Press Enter to close the figures and exit.")
plt.close('all')