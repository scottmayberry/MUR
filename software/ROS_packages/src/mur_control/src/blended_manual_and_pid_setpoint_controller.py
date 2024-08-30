#!/usr/bin/env python

import rospy
import pygame
from geometry_msgs.msg import WrenchStamped, PoseStamped
from std_msgs.msg import Bool  # Import the Bool message type
import tf2_ros
from tf.transformations import euler_from_quaternion
from simple_pid import PID
import math
import time
import tf.transformations  # Import the tf.transformations module


# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((200, 200))  # Small window

class BlendedController:
    def __init__(self):
        # Load control_info parameters
        self.control_info = rospy.get_param('/control_info')
        self.rate = self.control_info['stabilization']['rate']
        self.gains_info = self.control_info['stabilization']['gains']
        self.control_modes = self.control_info['stabilization']['control_mode']

        # Initialize setpoints for each DOF
        self.current_setpoints = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        
        # Initialize PID controllers for each DOF
        self.pids = {}
        for dof, params in self.gains_info.items():
            self.pids[dof] = PID(params['kp'], params['ki'], params['kd'], setpoint=self.current_setpoints[dof])
        
        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publishers
        self.wrench_pub = rospy.Publisher('/model/requested_thruster_wrench', WrenchStamped, queue_size=10)
        self.reset_pub = rospy.Publisher('/model_info/reset_zero_z_depth_pressure', Bool, queue_size=10)  # Publisher for the reset topic
        self.pose_pub = rospy.Publisher('/control/pose_setpoint', PoseStamped, queue_size=10)

        self.emergency_stop = False  # Initialize emergency stop state

    def get_keyboard_command(self):
        # Initialize command increments
        x_inc, y_inc, z_inc, roll_inc, pitch_inc, yaw_inc = 0, 0, 0, 0, 0, 0
        scale = 11.0  # Scale factor for direct manual control
        setpoint_scale = 0.05  # Scale factor for setpoint adjustments
        yaw_scale = 0.05  # Separate scale factor for yaw

        # Get the set of keys pressed
        keys = pygame.key.get_pressed()

        # Emergency stop control
        if keys[pygame.K_PERIOD]:  # . button for emergency stop
            self.emergency_stop = True
            print("Emergency stop activated. Sending zero wrench.")
        elif keys[pygame.K_COMMA]:  # , button to turn the system back on
            self.emergency_stop = False
            print("System reactivated. Resuming control.")

        # If emergency stop is activated, all commands are set to zero
        if self.emergency_stop:
            print("EMERGENCY STOP ACTIVE")
            return {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

        # Adjust setpoints if in PID mode, otherwise direct control
        if self.control_modes['x'] == 'pid':
            if keys[pygame.K_w]: x_inc = setpoint_scale
            if keys[pygame.K_s]: x_inc = -setpoint_scale
        else:
            if keys[pygame.K_w]: x_inc = 1
            if keys[pygame.K_s]: x_inc = -1

        if self.control_modes['y'] == 'pid':
            if keys[pygame.K_a]: y_inc = -setpoint_scale
            if keys[pygame.K_d]: y_inc = setpoint_scale
        else:
            if keys[pygame.K_a]: y_inc = -1
            if keys[pygame.K_d]: y_inc = 1

        if self.control_modes['z'] == 'pid':
            if keys[pygame.K_q]: z_inc = setpoint_scale
            if keys[pygame.K_e]: z_inc = -setpoint_scale
        else:
            if keys[pygame.K_q]: z_inc = 1
            if keys[pygame.K_e]: z_inc = -1

        if self.control_modes['roll'] == 'pid':
            if keys[pygame.K_i]: roll_inc = setpoint_scale
            if keys[pygame.K_k]: roll_inc = -setpoint_scale
        else:
            if keys[pygame.K_i]: roll_inc = 1
            if keys[pygame.K_k]: roll_inc = -1

        if self.control_modes['yaw'] == 'pid':
            if keys[pygame.K_j]: yaw_inc = yaw_scale
            if keys[pygame.K_l]: yaw_inc = -yaw_scale
        else:
            if keys[pygame.K_j]: yaw_inc = yaw_scale
            if keys[pygame.K_l]: yaw_inc = -yaw_scale

        # Apply direct manual control scaling
        if self.control_modes['x'] != 'pid': x_inc *= scale
        if self.control_modes['y'] != 'pid': y_inc *= scale
        if self.control_modes['z'] != 'pid': z_inc *= scale
        if self.control_modes['roll'] != 'pid': roll_inc /= 8
        if self.control_modes['yaw'] != 'pid': yaw_inc /= 8

        # Update setpoints if in PID mode
        if self.control_modes['x'] == 'pid': self.current_setpoints['x'] += x_inc
        if self.control_modes['y'] == 'pid': self.current_setpoints['y'] += y_inc
        if self.control_modes['z'] == 'pid': self.current_setpoints['z'] = max(-4.0, min(0.0, z_inc+self.current_setpoints['z']))
        if self.control_modes['roll'] == 'pid': self.current_setpoints['roll'] = (self.current_setpoints['roll'] + roll_inc) % (2 * math.pi)
        if self.control_modes['yaw'] == 'pid': self.current_setpoints['yaw'] = (self.current_setpoints['yaw'] + yaw_inc) % (2 * math.pi)


        # Check for reset key ("/")
        if keys[pygame.K_SLASH]:
            self.reset_pub.publish(True)
            print("Published True to /model_info/reset_zero_z_depth_pressure")

        # Return the manual command values
        return {'x': x_inc, 'y': y_inc, 'z': z_inc, 'roll': roll_inc, 'pitch': pitch_inc, 'yaw': yaw_inc}

    def get_current_orientation_and_position(self):
        try:
            trans = self.tf_buffer.lookup_transform('world', 'fossen_com', rospy.Time(0))
            quat = trans.transform.rotation
            euler = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            position = trans.transform.translation
            return {
                'roll': euler[0], 
                'pitch': euler[1], 
                'yaw': euler[2],
                'x': position.x,
                'y': position.y,
                'z': position.z
            }
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def shortest_angular_difference(current, target):
        """Compute the shortest difference between two angles."""
        difference = target - current
        while difference > math.pi:
            difference -= 2 * math.pi
        while difference < -math.pi:
            difference += 2 * math.pi
        return difference

    def control_loop(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # Check for keyboard input and get manual input commands
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rospy.signal_shutdown('Quit event received')
            manual_input = self.get_keyboard_command()

            # Get current pose from the robot
            orientation_and_position = self.get_current_orientation_and_position()

            if orientation_and_position:
                wrench_msg = WrenchStamped()
                wrench_msg.header.stamp = rospy.Time.now()
                wrench_msg.header.frame_id = "fossen_com"

                # Update pose setpoints if in PID mode and publish them
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "fossen_world"
                pose_msg.pose.position.x = self.current_setpoints['x']
                pose_msg.pose.position.y = self.current_setpoints['y']
                pose_msg.pose.position.z = self.current_setpoints['z']
                quat = tf.transformations.quaternion_from_euler(
                    self.current_setpoints['roll'],
                    self.current_setpoints['pitch'],
                    self.current_setpoints['yaw']
                )
                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]
                self.pose_pub.publish(pose_msg)

                for dof in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
                    if self.control_modes[dof] == 'pid':
                        error = orientation_and_position[dof] - self.current_setpoints[dof] if dof not in ['roll', 'pitch', 'yaw'] else \
                                BlendedController.shortest_angular_difference(orientation_and_position[dof], self.current_setpoints[dof])
                        control_output = self.pids[dof](error)
                        
                        # Print the target vs actual position
                        print(f"DOF: {dof.upper():<5} | Target: {self.current_setpoints[dof]:8.4f} | Current: {orientation_and_position[dof]:8.4f} | Error: {error:8.4f}")

                    else:
                        control_output = manual_input[dof]

                    if dof in ['x', 'y', 'z']:
                        setattr(wrench_msg.wrench.force, dof, control_output)
                    else:
                        setattr(wrench_msg.wrench.torque, {'roll': 'x', 'pitch': 'y', 'yaw': 'z'}[dof], control_output)

                # Publish the command
                self.wrench_pub.publish(wrench_msg)
            
            rate.sleep()

def get_has_booted():
    return rospy.get_param('/module_info/hasBooted')

if __name__ == '__main__':
    rospy.init_node('blended_controller')
    while not get_has_booted():
        time.sleep(5)
    time.sleep(3)
    controller = BlendedController()
    controller.control_loop()
    pygame.quit()
