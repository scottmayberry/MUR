import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

model_info = {
    'ref_origin_description': "back of sub, centered in tube",
    'com': [281.09, -0.02, 11.91],
    'mass': 6174,
    'inertia': [
        [52382881.48, 77016.35, 11499941.55],
        [77016.35, 224004382.14, 18433.94],
        [11499941.55, 18433.94, 249168085.10]
    ],
    'thrusters': [
        {'pos': [33.48,-126.83,0], 'orientation': [0, 0, 135], 'use_radians': False},
        {'pos': [33.48,126.83,0], 'orientation': [0, 0, 225], 'use_radians': False},
        {'pos': [253,-128,-15], 'orientation': [0, 90, 0], 'use_radians': False},
        {'pos': [253,128,-15], 'orientation': [0, 90, 0], 'use_radians': False},
        {'pos': [472.52,-126.83,-1], 'orientation': [0, 0, 45], 'use_radians': False},
        {'pos': [472.52,126.83,-1], 'orientation': [0, 0, -45], 'use_radians': False}
    ]
}

def compute_thruster_commands(desired_command, thrusters, com):
    # Constructing a modified matrix T without pitch contributions
    T = np.zeros((5, len(thrusters)))
    for i, thruster in enumerate(thrusters):
        pos = np.array(thruster['pos']) - np.array(com)
        orientation = np.array(thruster['orientation'])
        contributions = compute_contributions(pos, orientation)
        T[:, i] = np.delete(contributions, 4)  # Removing pitch contribution
    thruster_commands = np.linalg.pinv(T).dot(desired_command)
    thruster_commands = np.clip(thruster_commands, -1, 1)
    return thruster_commands

def compute_contributions(pos, orientation):
    R = euler_to_rotation_matrix(orientation)
    force_direction = R.dot(np.array([1, 0, 0]))
    F = force_direction
    tau = np.cross(pos, force_direction)
    return np.concatenate([F, tau])

def euler_to_rotation_matrix(orientation, use_radians=False):
    if not use_radians:
        orientation = np.deg2rad(orientation)  # Convert to radians if in degrees
    roll, pitch, yaw = orientation
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    R = R_z @ R_y @ R_x
    return R

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..'''
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

def plot_tube(ax):
    # Plot the tube (cylinder)
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, 600, 100)  # 600mm length starting from origin
    X = np.outer(v, np.ones_like(u))
    Y = 4.5 * 25.4 * np.outer(np.ones_like(v), np.cos(u))  # Convert 4.5 inches to mm
    Z = 4.5 * 25.4 * np.outer(np.ones_like(v), np.sin(u))
    ax.plot_surface(X, Y, Z, color='c', alpha=0.5)  # Cyan color with some transparency

def plot_thruster_reference_directions(thrusters, com):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    plot_tube(ax)
    
    # Plot each thruster and its reference direction
    for thruster in thrusters:
        pos = np.array(thruster['pos'])
        orientation = np.array(thruster['orientation'])
        R = euler_to_rotation_matrix(orientation, thruster.get('use_radians', False))
        force_direction = R.dot(np.array([1, 0, 0]))
        
        # Plot the reference arrow for the thruster
        ax.quiver(pos[0], pos[1], pos[2], 
                  force_direction[0], force_direction[1], force_direction[2], 
                  length=60, color='darkred', arrow_length_ratio=0.1, linewidth=2.5)
        
        ax.scatter(pos[0], pos[1], pos[2], c='r', marker='o')
    
    # Set the aspect ratio to be equal for all axes
    set_axes_equal(ax)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Thrusters and Reference Directions')
    plt.show(block=False)

def plot_thruster_commands_and_resultant(thrusters, com, desired_command, thruster_commands):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    plot_tube(ax)
    
    # Plot each thruster and its command direction
    for idx, thruster in enumerate(thrusters):
        pos = np.array(thruster['pos'])
        orientation = np.array(thruster['orientation'])
        R = euler_to_rotation_matrix(orientation, thruster.get('use_radians', False))
        force_direction = R.dot(np.array([1, 0, 0]))
        
        # Scale the thruster vector by its command magnitude
        scaled_force_direction = thruster_commands[idx] * force_direction
        
        ax.quiver(pos[0], pos[1], pos[2], 
                  scaled_force_direction[0], scaled_force_direction[1], scaled_force_direction[2], 
                  length=50, color='b', arrow_length_ratio=0.1)
        
        ax.scatter(pos[0], pos[1], pos[2], c='r', marker='o')
    
    # Compute and plot the resultant vector
    resultant = np.sum([thruster_commands[idx] * euler_to_rotation_matrix(thruster['orientation'], thruster.get('use_radians', False)).dot(np.array([1, 0, 0])) for idx, thruster in enumerate(thrusters)], axis=0)
    ax.quiver(com[0], com[1], com[2], 
              resultant[0], resultant[1], resultant[2], 
              length=100, color='darkgreen', arrow_length_ratio=0.1, linewidth=2.5, label="Resultant Thrust")
    
    # Plot the desired command vector
    ax.quiver(com[0], com[1], com[2], 
              desired_command[0], desired_command[1], desired_command[2], 
              length=100, color='m', arrow_length_ratio=0.1, label="Desired Command")
    
    # Set the aspect ratio to be equal for all axes
    set_axes_equal(ax)

    # Print the desired vector components, resultant vector components, and the error
    print("Desired Vector Components: ", desired_command[:3])
    print("Resultant Vector Components: ", resultant)
    error = desired_command[:3] - resultant
    print("Error between Desired and Resultant: ", error)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Thruster Commands, Desired Command, and Resultant Vector')
    ax.legend()
    plt.show()

    



# Example usage:
desired_command = np.array([2, 0, 0, 0, 0])  # Replace with actual desired values
thruster_commands = compute_thruster_commands(desired_command, model_info['thrusters'], model_info['com'])
print(thruster_commands)
# Call the plotting functions
plot_thruster_reference_directions(model_info['thrusters'], model_info['com'])
plot_thruster_commands_and_resultant(model_info['thrusters'], model_info['com'], desired_command, thruster_commands)