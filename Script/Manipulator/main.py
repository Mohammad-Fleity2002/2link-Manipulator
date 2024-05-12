# System (Default Lib.)
import sys
# Own library for robot control (kinematics), visualization, etc. (See manipulator.py)
import manipulator
# Matplotlib (Visualization Lib. -> Animation)
from matplotlib import animation
# Numpy (Array computing Lib.)
import numpy as np


def generate_rectangle(centroid, dimension, angle):
    # A simple function to generate a path for a rectangle.
    p = [[(-1)*dimension[0]/2, (-1)*dimension[0]/2, (+1)*dimension[0]/2, (+1)*dimension[0]/2, (-1)*dimension[0]/2],
         [(-1)*dimension[1]/2, (+1)*dimension[1]/2, (+1)*dimension[1]/2, (-1)*dimension[1]/2, (-1)*dimension[1]/2]]

    x = []
    y = []

    for i in range(len(p[0])):
        # Calculation position of the Rectangle
        x.append((p[0][i]*np.cos(angle * (np.pi/180)) - p[1][i]
                 * np.sin(angle * (np.pi/180))) + centroid[0])
        y.append((p[0][i]*np.sin(angle * (np.pi/180)) + p[1][i]
                 * np.cos(angle * (np.pi/180))) + centroid[1])

    return [x, y]


def generate_circle(centroid, radius):
    # A simple function to generate a path for a circle.

    # Circle ->  0 to 2*pi
    theta = np.linspace(0, 2*np.pi, 25)

    # Calculation position of the Circle
    x = radius * np.cos(theta) + centroid[0]
    y = radius * np.sin(theta) + centroid[1]

    return [x, y]


def main():
    axis_wr = [[-140.0, 140.0], [-150.0, 150.0]]
    # Length of Arms (Link 1, Link2)
    arm_length = [0.3, 0.25]

    # DH (Denavit-Hartenberg) parameters
    theta_0 = [0.0, 0.0]
    a = [arm_length[0], arm_length[1]]
    d = [0.0, 0.0]
    alpha = [0.0, 0.0]

    # Initialization of the Class (Control Manipulator)
    GM = manipulator.Control('Guitta-Mohammad', manipulator.DH_parameters(
        theta_0, a, d, alpha), axis_wr)

    test_trajectory = 'AB'
    # test_trajectory = 'ABC'
    # test_trajectory = 'Circle'

    # Structure -> Null
    trajectory_str = []

    if test_trajectory == 'Circle':
        #   (1) Circle Centroid [Float Array]
        #   (2) Radius          [Float]
        x, y = generate_circle([0.25, -0.25], 0.1)

        # Initial (Start) Position
        trajectory_str.append({'interpolation': 'joint', 'start_p': [
                              0.50, 0.0], 'target_p': [x[0], y[0]], 'step': 25, 'cfg': 1})

        for i in range(len(x) - 1):
            trajectory_str.append({'interpolation': 'linear', 'start_p': [
                                  x[i], y[i]], 'target_p': [x[i + 1], y[i + 1]], 'step': 5, 'cfg': 1})

    elif test_trajectory == 'Rectangle':
        #   (1) Rectangle Centroid         [Float Array]
        #   (2) Dimensions (width, height) [Float Array]
        #   (3) Angle (Degree)             [Float]
        x, y = generate_rectangle([-0.25, 0.25], [0.15, 0.15], 0.0)

        # Start Position
        trajectory_str.append({'interpolation': 'joint', 'start_p': [
                              0.50, 0.0], 'target_p': [x[0], y[0]], 'step': 50, 'cfg': 0})

        for i in range(len(x) - 1):
            trajectory_str.append({'interpolation': 'linear', 'start_p': [
                                  x[i], y[i]], 'target_p': [x[i + 1], y[i + 1]], 'step': 25, 'cfg': 0})

    elif test_trajectory == 'AB':
        trajectory_str.append({'interpolation': 'joint', 'start_p': [
                              0.30, 0.0], 'target_p': [0.0, 0.40], 'step': 50, 'cfg': 1})

    elif test_trajectory == 'A':
        trajectory_str.append({'interpolation': 'linear', 'start_p': [
                              0.50, 0.0], 'target_p': [0.5, 0.0], 'step': 100, 'cfg': 0})

    elif test_trajectory == 'ABC':
        trajectory_str.append({'interpolation': 'linear', 'start_p': [
                              0.30, 0.0], 'target_p': [0.40, 0.30], 'step': 25, 'cfg': 1})
        trajectory_str.append({'interpolation': 'linear', 'start_p': [
                              0.40, 0.30], 'target_p': [0.20, 0.40], 'step': 25, 'cfg': 1})
        # trajectory_str.append({'interpolation': 'joint', 'start_p': [
        #                       0.30, 0.0], 'target_p': [0.40, 0.30], 'step': 25, 'cfg': 1})
        # trajectory_str.append({'interpolation': 'joint', 'start_p': [
        #                       0.40, 0.30], 'target_p': [0.20, 0.40], 'step': 25, 'cfg': 1})

    elif test_trajectory == 'ABCD':
        trajectory_str.append({'interpolation': 'linear', 'start_p': [
                              0.30, 0.0], 'target_p': [0.40, 0.30], 'step': 25, 'cfg': 1})
        trajectory_str.append({'interpolation': 'linear', 'start_p': [
                              0.40, 0.30], 'target_p': [0.20, 0.40], 'step': 25, 'cfg': 1})
        trajectory_str.append({'interpolation': 'linear', 'start_p': [
                              0.20, 0.40], 'target_p': [0.0, 0.30], 'step': 25, 'cfg': 1})

    # Structure -> Null
    check_cartesianTrajectory_str = []

    for i in range(len(trajectory_str)):
        # Generating a trajectory from a structure
        x, y, cfg = GM.generate_trajectory(trajectory_str[i])

        for j in range(trajectory_str[i]['step']):
            check_cartesianTrajectory_str.append(
                {'calc_type': 'IK', 'p': [x[j], y[j]], 'cfg': cfg[j]})
            # check_cartesianTrajectory_str.append(
            #     {'calc_type': 'FK', 'degree_repr': True, 'theta': [30, 30], 'cfg': cfg[j]})
            # Adding points and configurations to the resulting trajectory
            GM.trajectory[0].append(x[j])
            GM.trajectory[1].append(y[j])
            GM.trajectory[2].append(cfg[j])

    # Check that the trajectory points for the robot are reachable
    tP_err = GM.check_trajectory(check_cartesianTrajectory_str)
    tP_smooth = False

    if tP_smooth == True:
        smooth_trajectory = [[], [], []]
        try:
            # Check that trajectory smoothing is possible and smooth the trajectory using an appropriate method
            [smooth_trajectory[0], smooth_trajectory[1], smooth_trajectory[2]
             ] = GM.smooth_trajectory(trajectory_str)
            # Trajectory smoothing is successful
            GM.trajectory = smooth_trajectory
        except TypeError:
            print(
                '[INFO] Trajectory smoothing is not possible (Insufficient or too many entry points).')

    GM.display_environment([True, 1])

    if True in tP_err[0]:
        # Trajectory Error (some points are not not reachable)
        GM.init_animation()
    else:
        # Call the animator for the GM Robotics Arm (if the results of the solution are error-free).
        animator = animation.FuncAnimation(GM.figure, GM.start_animation, init_func=GM.init_animation, frames=len(
            GM.trajectory[0]), interval=2, blit=True, repeat=False)
        # Save Animation
        animator.save(f'{test_trajectory}.gif', fps=30, bitrate=1000)


if __name__ == '__main__':
    sys.exit(main())
