import roboticstoolbox as rtb
import numpy as np
from numpy.linalg import matrix_rank, slogdet, det
import time
pi = np.pi

from mpl_toolkits import mplot3d

import matplotlib.pyplot as plt


from scipy.spatial import KDTree



data= np.random.rand(10510,3)
fig = plt.figure()
ax = plt.axes(projection='3d')


# Data for a three-dimensional line
# zline = np.linspace(0, 15, 1000)
# xline = np.sin(zline)
# yline = np.cos(zline)
# ax.plot3D(xline, yline, zline, 'gray')

# Data for three-dimensional scattered points
# zdata = 15 * np.random.random(100)
# xdata = np.sin(zdata) + 0.1 * np.random.randn(100)
# ydata = np.cos(zdata) + 0.1 * np.random.randn(100)

# ax.scatter3D(5, 5, 5, cmap='Greens')
# ax.scatter3D(1, 1, 1, cmap='Greens')


all_points = []


STEP = pi/8
END = 2*pi
def main():
    c = 0
    ur5 = rtb.models.URDF.UR5()
    for i in np.arange(-END, END, STEP):
        for j in np.arange(-END, END, STEP):
            for k in np.arange(-END, END, STEP):
                # ur5.qr = np.array([i, j, k, z, 0, 0])
                # r = ur5.manipulability(J=ur5.jacob0(ur5.qr))
                try:
                    conf = [i, j, k, 3.92, 4.7, 0]
                    r = det(ur5.jacob0(conf))
                    
                    if r < 1e-18:
                        # print(conf)
                        # print(ur5.fkine(conf))
                        p = np.array(np.transpose(ur5.fkine(conf).A)[-1][:-1])
                        # print(p)
                        # ax.scatter3D(p[0], p[1], p[2], cmap='Greens')
                        all_points.append((p[0], p[1], p[2]))
                        # ax.scatter3D(p[0], p[1], p[2])
                        c+=1
                except Exception as e:
                    print("sin")
    print(c)
                    


    
if __name__ == '__main__':
    main()
    # print(time.time())
    kdtree=KDTree(all_points)
    # print(time.time())

    goods = []
    bads = []
    i = 300
    while i > 0:
        # print(all_points[i])
        i -= 1
        dist, points=kdtree.query(all_points[i],5)
        for p in points:
            bads.append(all_points[p])


        # print(dist, points)
        # all_points.remove()
    c = 0
    for p in all_points:
        if p not in bads:
            c += 1
            ax.scatter3D(p[0], p[1], p[2])

    print(c)
    plt.show()
    pass