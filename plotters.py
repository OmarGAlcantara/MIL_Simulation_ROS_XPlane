#################################################################################

#                               17/10/2023

# Plotting Functions

#author: Omar García
#git:
#researchgate:
#################################################################################


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Import the 3D plotting toolkit

def plot_1(t, data_1, name_1, units_1, figure_name):
    fig, axs = plt.subplots(1, 1, figsize=(15, 10))

    # Plot Input (first column, first row)
    axs.plot(t, data_1)
    axs.set_title(name_1)
    axs.set_xlabel("Time (s)")
    axs.set_ylabel(units_1)

    axs.grid(True)
    plt.tight_layout()

    # Define the file path with the provided figure_name appended
    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/' + figure_name

    plt.savefig(file)
    plt.clf()
    plt.close()

def plot_2(t, data_1, data_2, name_1, units_1, name_2, units_2, figure_name):
    fig, axs = plt.subplots(1, 2, figsize=(15, 10))

    # Plot Input (first column, first row)
    axs[0].plot(t, data_1)
    axs[0].set_title(name_1)
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel(units_1)

    # Plot Graph 1 (second column, first row)
    axs[1].plot(t, data_2)
    axs[1].set_title(name_2)
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel(units_2)

    axs[0].grid(True), axs[1].grid(True)
    plt.tight_layout()

    # Define the file path with the provided figure_name appended
    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/' + figure_name

    plt.savefig(file)
    plt.clf()
    plt.close()

def plot_3(t, data_1, data_2, data_3, name_1, units_1, name_2, units_2, name_3, units_3, figure_name):
    fig, axs = plt.subplots(1, 3, figsize=(15, 10))

    # Plot Input (first column, first row)
    axs[0].plot(t, data_1)
    axs[0].set_title(name_1)
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel(units_1)

    # Plot Graph 1 (second column, first row)
    axs[1].plot(t, data_2)
    axs[1].set_title(name_2)
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel(units_2)

    # Plot Graph 2 (third column, first row)
    axs[2].plot(t, data_3)
    axs[2].set_title(name_3)
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel(units_3)

    # plt.ylim(-0.01, 0.01)
    axs[0].grid(True), axs[1].grid(True), axs[2].grid(True)
    plt.tight_layout()
    # Define the file path with the provided figure_name appended
    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/' + figure_name

    plt.savefig(file)
    plt.clf()
    plt.close()

def plot_2_in_1(t, data_1, data_2, name_1, units_1, name_2, units_2, figure_name):
    fig, ax = plt.subplots(figsize=(10, 7))

    # Plot data_1
    ax.plot(t, data_1, label=name_1)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(units_1)

    # Plot data_2
    ax.plot(t, data_2, label=name_2)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(units_2)

    ax.grid(True)
    ax.legend()
    plt.tight_layout()

    # Define the file path with the provided figure_name appended
    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/' + figure_name

    plt.savefig(file)
    plt.clf()
    plt.close()

def plot_3_yaw(t, data_1, data_2, data_3, name_1, units_1, name_2, units_2, name_3, units_3, figure_name):
    fig, axs = plt.subplots(1, 3, figsize=(15, 10))

    # Plot Input (first column, first row)
    axs[0].plot(t, data_1)
    axs[0].set_title(name_1)
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel(units_1)

    # Plot Graph 1 (second column, first row)
    axs[1].plot(t, data_2)
    axs[1].set_title(name_2)
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel(units_2)

    # Plot Graph 2 (third column, first row)
    axs[2].plot(t, data_3)
    axs[2].set_title(name_3)
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel(units_3)
    #axs[2].set_ylim(4.3, 4.44)

    # plt.ylim(-0.01, 0.01)
    axs[0].grid(True), axs[1].grid(True), axs[2].grid(True)
    plt.tight_layout()
    # Define the file path with the provided figure_name appended
    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/' + figure_name

    plt.savefig(file)
    plt.clf()
    plt.close()

def plot_3vertical(t, data_1, data_2, data_3, name_1, units_1, name_2, units_2, name_3, units_3, figure_name):
    fig, axs = plt.subplots(3, 1, figsize=(15, 10))

    # Plot Input (first column, first row)
    axs[0].plot(t, data_1)
    axs[0].set_title(name_1)
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel(units_1)

    # Plot Graph 1 (second column, first row)
    axs[1].plot(t, data_2)
    axs[1].set_title(name_2)
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel(units_2)

    # Plot Graph 2 (third column, first row)
    axs[2].plot(t, data_3)
    axs[2].set_title(name_3)
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel(units_3)

    # plt.ylim(-0.01, 0.01)
    axs[0].grid(True), axs[1].grid(True), axs[2].grid(True)
    plt.tight_layout()
    # Define the file path with the provided figure_name appended
    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/' + figure_name

    plt.savefig(file)
    plt.clf()
    plt.close()

def plot_3_Spec(t, data_1, data_2, data_3, name_1, units_1, name_2, units_2, name_3, units_3, figure_name, lim1=None, lim2=None, lim3=None):
    fig, axs = plt.subplots(1, 3, figsize=(15, 10))

    axs[0].plot(t, data_1)
    axs[0].set_title(name_1)
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel(units_1)
    if lim1 is not None:
        axs[0].set_ylim(lim1[0], lim1[1])

    axs[1].plot(t, data_2)
    axs[1].set_title(name_2)
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel(units_2)
    if lim2 is not None:
        axs[1].set_ylim(lim2[0], lim2[1])

    axs[2].plot(t, data_3)
    axs[2].set_title(name_3)
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel(units_3)
    if lim3 is not None:
        axs[2].set_ylim(lim3[0], lim3[1])

    axs[0].grid(True), axs[1].grid(True), axs[2].grid(True)
    plt.tight_layout()

    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/' + figure_name

    plt.savefig(file)
    plt.clf()
    plt.close()

def plot_4_Spec(t, data_1, data_2, data_3, data_4, name_1, units_1, name_2, units_2, name_3, units_3, name_4, units_4, figure_name, lim1=None, lim2=None, lim3=None, lim4=None):
    fig, axs = plt.subplots(4, 1, figsize=(15, 10))

    axs[0].plot(t, data_1)
    axs[0].set_title(name_1)
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel(units_1)
    if lim1 is not None:
        axs[0].set_ylim(lim1[0], lim1[1])

    axs[1].plot(t, data_2)
    axs[1].set_title(name_2)
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel(units_2)
    if lim2 is not None:
        axs[1].set_ylim(lim2[0], lim2[1])

    axs[2].plot(t, data_3)
    axs[2].set_title(name_3)
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel(units_3)
    if lim3 is not None:
        axs[2].set_ylim(lim3[0], lim3[1])

    axs[3].plot(t, data_4)
    axs[3].set_title(name_4)
    axs[3].set_xlabel("Time (s)")
    axs[3].set_ylabel(units_4)
    if lim4 is not None:
        axs[3].set_ylim(lim4[0], lim4[1])

    axs[0].grid(True), axs[1].grid(True), axs[2].grid(True), axs[3].grid(True)
    plt.tight_layout()

    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/' + figure_name

    plt.savefig(file)
    plt.clf()
    plt.close()

def plot_2in1_3(t, data_1, data_2, data_3, data_4, data_5, data_6, name_1, units_1, name_2, units_2, name_3, units_3, name_4, units_4, name_5, units_5, name_6, units_6, figure_name, lim1=None, lim2=None, lim3=None):
    fig, axs = plt.subplots(1, 3, figsize=(15, 10))

    axs[0].plot(t, data_1, label = name_1)
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel(units_1)
    if lim1 is not None:
        axs[0].set_ylim(lim1[0], lim1[1])
    axs[0].plot(t, data_2, label = name_2)
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel(units_2)


    axs[1].plot(t, data_3, label = name_3)
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel(units_3)
    if lim2 is not None:
        axs[1].set_ylim(lim2[0], lim2[1])
    axs[1].plot(t, data_4, label = name_4)
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel(units_4)

    axs[2].plot(t, data_5, label = name_5)
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel(units_5)
    if lim3 is not None:
        axs[2].set_ylim(lim3[0], lim3[1])
    axs[2].plot(t, data_6, label = name_6)
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel(units_6)


    axs[0].grid(True), axs[1].grid(True), axs[2].grid(True)
    axs[0].legend(), axs[1].legend(), axs[2].legend()
    plt.tight_layout()

    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/' + figure_name

    plt.savefig(file)
    plt.clf()
    plt.close()

def plot_3d_trajectory(t, data_x, data_y, data_z, data_xd, data_yd, data_zd, name_x, units_x, name_y, units_y, name_z,
                       units_z, figure_name, lim_x=None, lim_y=None, lim_z=None):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')  # Create a 3D subplot

    ax.plot(data_xd, data_yd, data_zd, label="Desired Trajectory", color='m')  # Plot the desired trajectory
    ax.plot(data_x, data_y, data_z, label="Actual Trajectory", color='b')  # Plot the actual trajectory

    ax.set_xlabel(name_x + ' (' + units_x + ')')
    ax.set_ylabel(name_y + ' (' + units_y + ')')
    ax.set_zlabel(name_z + ' (' + units_z + ')')

    if lim_x is not None:
        ax.set_xlim(lim_x[0], lim_x[1])
    if lim_y is not None:
        ax.set_ylim(lim_y[0], lim_y[1])
    if lim_z is not None:
        ax.set_zlim(lim_z[0], lim_z[1])

    ax.legend()
    plt.title(figure_name)
    plt.grid(True)

    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/' + figure_name + '.png'
    plt.savefig(file)
    plt.clf()
    plt.close()

def plot_xz_plane(t, data_x, data_z, data_xd, data_zd, name_x, units_x, name_z, units_z, figure_name, lim_x=None,
                  lim_z=None):
    plt.figure(figsize=(10, 6))

    plt.plot(data_xd, data_zd, label="Desired Trajectory", color='m')  # Plot the desired trajectory
    plt.plot(data_x, data_z, label="Actual Trajectory", color='b')  # Plot the actual trajectory

    plt.xlabel(name_x + ' (' + units_x + ')')
    plt.ylabel(name_z + ' (' + units_z + ')')

    if lim_x is not None:
        plt.xlim(lim_x[0], lim_x[1])
    if lim_z is not None:
        plt.ylim(lim_z[0], lim_z[1])

    plt.legend()
    plt.title(figure_name)
    plt.grid(True)

    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/' + figure_name + '.png'
    plt.savefig(file)
    plt.clf()
    plt.close()

def plot_2in1_3_2times(t, t2, data_1, data_2, data_3, data_4, data_5, data_6, name_1, units_1, name_2, units_2, name_3, units_3, name_4, units_4, name_5, units_5, name_6, units_6, figure_name, lim1=None, lim2=None, lim3=None):
    fig, axs = plt.subplots(1, 3, figsize=(15, 10))

    axs[0].plot(t, data_1, label = name_1)
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel(units_1)
    if lim1 is not None:
        axs[0].set_ylim(lim1[0], lim1[1])
    axs[0].plot(t, data_2, label = name_2)
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel(units_2)


    axs[1].plot(t, data_3, label = name_3)
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel(units_3)
    if lim2 is not None:
        axs[1].set_ylim(lim2[0], lim2[1])
    axs[1].plot(t, data_4, label = name_4)
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel(units_4)

    axs[2].plot(t2, data_5, label = name_5)
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel(units_5)
    if lim3 is not None:
        axs[2].set_ylim(lim3[0], lim3[1])
    axs[2].plot(t2, data_6, label = name_6)
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel(units_6)

    axs[0].grid(True), axs[1].grid(True), axs[2].grid(True)
    axs[0].legend(), axs[1].legend(), axs[2].legend()
    plt.tight_layout()

    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/' + figure_name

    plt.savefig(file)
    plt.clf()
    plt.close()

def plot_2in1_3vertical(t, data_1, data_2, data_3, data_4, data_5, data_6, name_1, units_1, name_2, units_2, name_3, units_3, name_4, units_4, name_5, units_5, name_6, units_6, figure_name, lim1=None, lim2=None, lim3=None, num_xticks=5, num_yticks=5):
    fig, axs = plt.subplots(3, 1, figsize=(15, 10))

    axs[0].plot(t, data_1, label = name_1, color='blue', linewidth=2)
    axs[0].set_xlabel("Time (s)", fontsize=32)
    axs[0].set_ylabel(units_1, fontsize=32)
    if lim1 is not None:
        axs[0].set_ylim(lim1[0], lim1[1])
    axs[0].plot(t, data_2, label = name_2, color='red', linewidth=2)
    axs[0].set_xlabel("Time (s)", fontsize=32)
    axs[0].set_ylabel(units_2, fontsize=32)

    plt.locator_params(axis='x', nbins=num_xticks)
    plt.locator_params(axis='y', nbins=num_yticks)

    plt.xticks(fontsize=28)
    plt.yticks(fontsize=28)

    axs[1].plot(t, data_3, label = name_3, color='blue', linewidth=2)
    axs[1].set_xlabel("Time (s)", fontsize=32)
    axs[1].set_ylabel(units_3, fontsize=32)
    if lim2 is not None:
        axs[1].set_ylim(lim2[0], lim2[1])
    axs[1].plot(t, data_4, label = name_4, color='red', linewidth=2)
    axs[1].set_xlabel("Time (s)", fontsize=32)
    axs[1].set_ylabel(units_4, fontsize=32)

    plt.locator_params(axis='x', nbins=num_xticks)
    plt.locator_params(axis='y', nbins=num_yticks)

    plt.xticks(fontsize=28)
    plt.yticks(fontsize=28)

    axs[2].plot(t, data_5, label = name_5, color='blue', linewidth=2)
    axs[2].set_xlabel("Time (s)", fontsize=32)
    axs[2].set_ylabel(units_5, fontsize=32)
    if lim3 is not None:
        axs[2].set_ylim(lim3[0], lim3[1])
    axs[2].plot(t, data_6, label = name_6, color='red', linewidth=2)
    axs[2].set_xlabel("Time (s)", fontsize=32)
    axs[2].set_ylabel(units_6, fontsize=32)

    plt.locator_params(axis='x', nbins=num_xticks)
    plt.locator_params(axis='y', nbins=num_yticks)

    plt.xticks(fontsize=28)
    plt.yticks(fontsize=28)

    axs[0].grid(True), axs[1].grid(True), axs[2].grid(True)
    axs[0].legend(fontsize=32), axs[1].legend(fontsize=32), axs[2].legend(fontsize=32)
    plt.tight_layout()

    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/' + figure_name

    plt.savefig(file)
    plt.clf()
    plt.close()