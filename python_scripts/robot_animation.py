from operator import matmul
import numpy as np
from numpy import sin, cos


import matplotlib.pyplot as plt
import matplotlib
import mpl_toolkits.mplot3d.axes3d as p3
from  mpl_toolkits.mplot3d.art3d import Line3D
from  mpl_toolkits.mplot3d.art3d import Path3DCollection
from mpl_toolkits.mplot3d import Axes3D


from matplotlib import animation

DEG_OF_FREEDOM = 6

def Ti(i, qi):
    """
    Returns "T_i_i-1" using the given parameters
    i: Index of the joint (starts from 1)
    qi: Current joint state
    """
    if i == 0:
        raise("Invalid index")

    # robot params
    d1 = 0.089159
    d4 = 0.10915
    d5 = 0.09465
    d6 = 0.0823	
    a2 = 0.425
    a3 = 0.3922

    t = qi

    Ts= [
        [[cos(t), -sin(t), 0, 0],  [sin(t), cos(t), 0, 0], [0, 0, 1, d1], [0, 0, 0, 1]],
        [[cos(t), -sin(t), 0, 0], [0, 0, -1, 0 ], [sin(t), cos(t), 0, 0], [0, 0, 0, 1]],
        [[cos(t), -sin(t), 0, -a2], [sin(t), cos(t), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
        [[cos(t), -sin(t), 0, -a3], [sin(t), cos(t), 0, 0], [0, 0, 1, d4], [0, 0, 0, 1]],
        [[cos(t), -sin(t), 0, 0], [0, 0, -1, -d5], [sin(t), cos(t), 0, 0,], [0, 0, 0, 1]],
        [[cos(t), -sin(t), 0, 0], [0, 0, 1, d6], [-sin(t), -cos(t), 0, 0], [0, 0, 0, 1]]
    ]

    return np.array(Ts[i-1])


def calc_joint_positions(qs):
    """
    Calculates all joint positions in base frame regarding the given configuration;
    qs: An array containing all robot's current joint configurations
    """
    joint_positions = []

    T = Ti(1, qs[0])
    
    for i in range(2, DEG_OF_FREEDOM + 1):
        joint_positions.append(T[:-1, 3])
        T = np.matmul(T, Ti(i, qs[i-1]))
    
    return joint_positions



def step_plot(n, graph, links):
    # print(n)
    # js= calc_joint_positions([n * 2, n*2, n*4, n*10, 0, 0]) # joint positions
    js= calc_joint_positions([n/100, 0, n/100, 0, 0, 0]) # joint positions
    # print(js)

    js.insert(0, [0,0,0])

    xs = [x[0] for x in js]
    ys = [x[1] for x in js]
    zs = [x[2] for x in js]

    graph._offsets3d = (xs, ys, zs)

    for i in range(DEG_OF_FREEDOM - 1):
        links[i][0].set_data_3d(xs, ys, zs)
    
    
def plot(q, name=0, display=False):
    fig = plt.figure()
    ax = p3.Axes3D(fig)


    ax_range = [-1, 1]
    ax.set_xlim3d(ax_range)
    ax.set_xlabel('X')

    ax.set_ylim3d(ax_range)
    ax.set_ylabel('Y')

    ax.set_zlim3d(ax_range)
    ax.set_zlabel('Z')

    js= calc_joint_positions(q) # joint positions
    # js2= calc_joint_positions([30,0,0,0,0,0]) # joint positions
    # print(js)

    js.insert(0, [0,0,0])

    xs = [x[0] for x in js]
    ys = [x[1] for x in js]
    zs = [x[2] for x in js]
    

    graph  = ax.scatter(xs=xs, ys=ys, zs=zs, s=20, c='red', depthshade=True)

    links = []
    for i in range(DEG_OF_FREEDOM - 1):
        links.append(ax.plot([xs[i], xs[i+1]], [ys[i], ys[i+1]],zs=[zs[i], zs[i+1]]))

   
    fig.savefig(f"frames/{name}.png", dpi=300)

    if display:
        ani = animation.FuncAnimation(fig, step_plot, 10000, fargs=(graph, links), interval=20, blit=False)
        plt.show()
    

if __name__ == '__main__':
    # plot([0,0,0,0,0,0], display=True)

    for i in range(1000):
        plot([i/100, 0, i/100, 0, 0, 0], i)
