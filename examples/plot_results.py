#!/usr/bin/env python3

# Copyright 2023 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Plot the results from a CSV file."""
import os
import argparse
import csv
from collections import defaultdict
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# create an ArgumentParser object
parser = argparse.ArgumentParser(
    description='Plot x,y,z values from a CSV file')

parser.add_argument('--auto_update', action='store_true',
                    help='Auto update plots')

parser.add_argument('file', type=str, nargs='?', help='CSV file name',
                    default='multirotor_log.csv')
FILE_PATH = ""
PRINT_ERROR = True


def press(event):
    """Define the function to quit on pressing 'q'."""
    if event.key == 'q':
        plt.close()


def compute_mean_error(value_gt, value_ref):
    """Compute the mean error between the values and the reference."""
    error = []
    for i in range(len(value_gt)):
        error.append(abs(value_gt[i] - value_ref[i]))
    return np.mean(error)


def plot_values(data, values, title, axs):
    """Plot values vs time."""
    for i, value in enumerate(values):
        value_gt = str(value)
        axs[i].plot(data['time'], data[value_gt], linestyle='solid',
                    label=value)

        # Check if there are reference values
        value_ref = str(value) + "_ref"
        if data[value_ref]:
            axs[i].plot(data['time'], data[value_ref], linestyle='dotted',
                        label=value_ref)

        # Check if there are inertial odometry values
        value_io = str(value) + "_io"
        if data[value_io]:
            axs[i].plot(data['time'], data[value_io], linestyle='dashed',
                        label=value_io)

        if PRINT_ERROR and data[value_ref]:
            print("Mean error for ", value, " is: ",
                  compute_mean_error(data[value_gt], data[value_ref]))

        axs[i].set_xlabel('Time')
        axs[i].set_ylabel('Values')
        axs[i].set_title(title + " - " + value)
        axs[i].legend()
    if PRINT_ERROR:
        print("")


def plot_values_3d(data, values, title, axs):
    """Plot values vs time."""
    x_values = data[values[0]]
    y_values = data[values[1]]
    z_values = data[values[2]]

    axs.plot(x_values, y_values, z_values,
             linestyle='solid', label='ground truth')
    if data[values[0] + "_io"]:
        axs.plot(data[values[0] + "_io"], data[values[1] + "_io"],
                 data[values[2] + "_io"], linestyle='dotted', label='inertial odometry')
    if data[values[0] + "_ref"]:
        axs.plot(data[values[0] + "_ref"], data[values[1] + "_ref"],
                 data[values[2] + "_ref"], linestyle='dashed', label='reference')

    axs.set_xlabel('x [m]')
    axs.set_ylabel('y [m]')
    axs.set_zlabel('z [m]')
    axs.set_title(title)
    axs.legend()

    # Equals aspect ratio
    min_xyz = -1.0 * abs(np.min([x_values, y_values, z_values]))
    max_xyz = np.max([x_values, y_values, z_values])
    max_ = max(abs(min_xyz), abs(max_xyz))
    axs.set_xlim(-1.0 * max_, max_)
    axs.set_ylim(-1.0 * max_, max_)
    axs.set_zlim(0.0 * max_, max_)


def read_csv():
    """Read the CSV file and return a dictionary with the data."""
    # Create a defaultdict to store the data
    data = defaultdict(list)

    # Open the CSV file
    with open(FILE_PATH, mode='r', newline='') as file:
        csv_reader = csv.DictReader(file)

        # Iterate through each row in the CSV file
        for row in csv_reader:
            # Iterate through each key and value in the row
            for key, value in row.items():
                # Append the value to the corresponding list in the dictionary
                if type(value) == str:
                    if value == '':
                        break
                    data[key].append(float(value))

    # Check there are 74 keys in the dictionary
    if len(data.keys()) != 74:
        print("Warn: number of keys is not 74, it is ", len(data.keys()))

    # Check all keys in the dictionary have the same length
    for key in data.keys():
        if len(data[key]) != len(data['time']):
            print("ERROR: key ", key, " has different length")
            return

    return data


def update_plot_figure0(frame, axs):
    """Update the plot."""
    data = read_csv()
    if len(data['time']) == 0:
        print("No data to plot")
        return
    axs.clear()

    # 1) x,y,z trajectory
    plot_values_3d(data, ['x', 'y', 'z'], 'Position', axs)


def update_plot_figure1(frame, axs):
    """Update the plot."""
    data = read_csv()
    if data is None:
        print("No data to plot")
        return
    if len(data['time']) == 0:
        print("No data to plot")
        return

    for ax_row in axs:
        # Check if there are more than one row
        if isinstance(ax_row, np.ndarray):
            for ax in ax_row:
                ax.clear()
        else:
            ax_row.clear()

    # Figure 1
    plot_values(data, ['x', 'y', 'z'], 'Position', axs[0, :])
    plot_values(data, ['roll', 'pitch', 'yaw'], 'Orientation', axs[1, :])
    plot_values(data, ['vx', 'vy', 'vz'], 'Velocity', axs[2, :])
    plot_values(data, ['ax', 'ay', 'az'], 'Acceleration', axs[3, :])


def update_plot_figure2(frame, axs):
    """Update the plot."""
    data = read_csv()
    if len(data['time']) == 0:
        print("No data to plot")
        return

    for ax_row in axs:
        for ax in ax_row:
            ax.clear()
    # Figure 2
    plot_values(data, ['wx', 'wy', 'wz'], 'Angular velocity', axs[0, :])
    plot_values(data, ['dwx', 'dwy', 'dwz'], 'Angular acceleration', axs[1, :])
    plot_values(data, ['fx', 'fy', 'fz'], 'Force', axs[2, :])
    plot_values(data, ['tx', 'ty', 'tz'], 'Torque', axs[3, :])


def update_plot_figure3(frame, axs):
    """Update the plot."""
    data = read_csv()
    if len(data['time']) == 0:
        print("No data to plot")
        return

    for ax_row in axs:
        for ax in ax_row:
            ax.clear()

    # Figure 3
    plot_values(data, ['mw1', 'mw2', 'mw3', 'mw4'],
                'Motor angular velocity', axs[0, :])
    plot_values(data, ['mdw1', 'mdw2', 'mdw3', 'mdw4'],
                'Motor angular acceleration', axs[1, :])


def main():
    """Run main function."""
    # parse the arguments
    args = parser.parse_args()
    # Redefine the global variable
    global FILE_PATH
    FILE_PATH = os.path.abspath(args.file)
    print("Reading results from file: ", os.path.abspath(FILE_PATH))

    fig0 = plt.figure()
    axs0 = fig0.add_subplot(projection='3d')

    fig1, axs1 = plt.subplots(4, 3)
    fig1.suptitle("Plots - Figure 1")

    fig2, axs2 = plt.subplots(4, 3)
    fig2.suptitle("Plots - Figure 2")

    fig3, axs3 = plt.subplots(2, 4)
    fig3.suptitle("Plots - Figure 3")

    if args.auto_update:
        print("Auto update plots")
        global PRINT_ERROR
        PRINT_ERROR = False
        ani0 = FuncAnimation(fig0, update_plot_figure0,
                             fargs=(axs0,), interval=3000)
        ani1 = FuncAnimation(fig1, update_plot_figure1,
                             fargs=(axs1,), interval=3000)
        ani2 = FuncAnimation(fig2, update_plot_figure2,
                             fargs=(axs2,), interval=3000)
        ani3 = FuncAnimation(fig3, update_plot_figure3,
                             fargs=(axs3,), interval=3000)
        plt.axis('equal')
        plt.show()
        plt.pause(0.5)
    else:
        update_plot_figure0(0, axs0)
        update_plot_figure1(0, axs1)
        update_plot_figure2(0, axs2)
        update_plot_figure3(0, axs3)
        plt.axis('equal')
        plt.show(block=False)
        print("Press [Enter] to close the figures and end the program.")
        plt.pause(0.001)
        try:
            input()
        except KeyboardInterrupt:
            pass

    plt.close('all')
    print("Plotting finished")


if __name__ == "__main__":
    main()
