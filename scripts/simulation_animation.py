#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os


def point_func(num, dataSet, line):
    # NOTE: there is no .set_data() for 3 dim data...
    line.set_data(dataSet[0:2, num])
    return line

def robot_simulation(p_x, p_y):
    dataSet = np.array([p_x, p_y])
    numDataPoints = len(p_x)

    # GET SOME MATPLOTLIB OBJECTS
    fig = plt.figure()
    ax = fig.add_subplot()

    # NOTE: Can't pass empty arrays into 3d version of plot()
    line = ax.plot(dataSet[0], dataSet[1], 'ro')[0]  # For line plot

    # AXES PROPERTIES
    plt.axis('equal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Robot simulation')

    # Creating the Animation object
    line_ani = animation.FuncAnimation(fig, point_func, frames=numDataPoints, fargs=(dataSet, line), interval=1000/30,
                                    blit=False)

    ax.plot(dataSet[0], dataSet[1], lw='1', c='b')

    line_ani.save('AnimationNew.gif')

    

    plt.show()

# print(os.path.abspath('.'))
px = np.load('../.ros/px.npy')
py = np.load('../.ros/py.npy')

robot_simulation(px, py)