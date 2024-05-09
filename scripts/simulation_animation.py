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

    t = np.linspace(0, 2*np.pi, 100)
    x_circ_1 = np.cos(t)
    x_circ_2 = 2*np.cos(t)
    x_ellipse_1 = 2*np.cos(t)
    x_ellipse_2 = 4*np.cos(t)
    y_circ_1 = np.sin(t)
    y_circ_2 = 2*np.sin(t)
    y_ellipse_1 = np.sin(t)
    y_ellipse_2 = 2*np.sin(t)

    ax.plot(x_circ_1, y_circ_1, lw='1', c='b')
    ax.plot(x_circ_2, y_circ_2, lw='1', c='b')
    ax.plot(x_ellipse_1, y_ellipse_1, lw='1', c='b')
    ax.plot(x_ellipse_2, y_ellipse_2, lw='1', c='b')

    line_ani.save('robot_simulation.gif')

    

    plt.show()

# print(os.path.abspath('.'))
px = np.load('../.ros/px.npy')
py = np.load('../.ros/py.npy')

robot_simulation(px, py)