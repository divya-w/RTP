import matplotlib
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
import numpy
import sys
from matplotlib import pyplot as plt

input_path_root = "output/"
output_path_root = "output/"

if __name__ == "__main__":
    print(f"Arguments count: {len(sys.argv)}")
    args = list(enumerate(sys.argv))
    name = args[1][1]
    input_path = "" + input_path_root + name + ".txt"

    data = numpy.loadtxt(input_path)

    # Moving paths from [-10, 10] to [0, 20]
    print(data)
    data[:,0] = [x + 10 for x in data[:,0]]
    data[:,1] = [x + 10 for x in data[:,1]]
    print(data)
    fig = plt.figure()
    # ax = fig.gca(projection='3d')
    ax = fig.add_subplot(projection='3d')

    #
    # Add obstacles
    #
    env = args[2][1]
    # prepare some coordinates
    x, y, z = numpy.indices((20, 20, 4))

    # Environment 1
    if env == '1':
        upper_obstacle = (x > 3) & (x < 17) & (y > 10.75) & (z == 0)
        lower_obstacle = (x > 3) & (x < 17) & (y > 0) & (y < 9.25) & (z == 0)
        obstacles = upper_obstacle | lower_obstacle

        colors = numpy.empty(obstacles.shape, dtype=object)
        colors[upper_obstacle] = 'red'
        colors[lower_obstacle] = 'red'

        ax.voxels(obstacles, facecolors=colors, alpha = 0.3)
    # Environment 2
    elif env == '2':
        upper_obstacle = (x > 7) & (x < 11) & (y > 13) & (z == 0)
        lower_obstacle = (x > 7) & (x < 11) & (y > 0) & (y < 7) & (z == 0)
        end_obstacle = (x > 14) & (x < 16) & (y > 3) & (y < 17) & (z == 0)
        obstacles = upper_obstacle | lower_obstacle | end_obstacle

        colors = numpy.empty(obstacles.shape, dtype=object)
        colors[upper_obstacle] = 'red'
        colors[lower_obstacle] = 'red'
        colors[end_obstacle] = 'red'

        ax.voxels(obstacles, facecolors=colors, alpha = 0.3)

    #
    # Add path to plot
    #
    if name[0:5] == "Point":
        print("Point")
        ax.plot(data[:,0], data[:,1],'.-')
    elif name[0:6] == "Square":
        print("Square")
        ax.plot(data[:,0], data[:,1],data[:,2],'.-')

    #
    # Save overheads view
    #
    ax.view_init(azim=0, elev=90)
    output_path = output_path_root + name + "_plot_overhead.png"
    plt.savefig(output_path)
