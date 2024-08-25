#!/usr/bin/env python

import rospy
import tf2_ros
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_from_euler
from geometry_msgs.msg import TransformStamped, WrenchStamped, Vector3Stamped, PoseStamped
from tf.transformations import euler_from_quaternion
from simple_pid import PID  # Assuming you have a PID class
import numpy as np
import time

class StabilizationController:
    def __init__(self):
        # Load control_info parameters
        # control_info = rospy.get_param('/control_info/stabilization')
        self.control_info = rospy.get_param('/control_info')
        self.rate = self.control_info['stabilization']['rate']
        self.gains_info = self.control_info['stabilization']['gains']
        
       # Initialize PID controllers for each DOF
        self.pids = {}
        for dof, params in self.gains_info.items():
            self.pids[dof] = PID(params['kp'], params['ki'], params['kd'], setpoint=0)


        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Setpoint
        self.setpoints = {} 
        # {
        #     'roll': 0,
        #     'pitch': 0,
        #     'yaw': 0,
        #     'x': 0,
        #     'y': 0,
        #     'z': 0
        # }

        # Subscriber
        rospy.Subscriber('pose_setpoint', PoseStamped, self.setpoint_callback)

        # Publisher
        self.wrench_pub = rospy.Publisher('/model/requested_thruster_wrench', WrenchStamped, queue_size=10)


    def setpoint_callback(self, msg):
        # # Update orientation setpoints
        # quat = msg.pose.orientation
        # euler = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        # self.setpoints['roll'] = euler[0]
        # self.setpoints['pitch'] = euler[1]
        # self.setpoints['yaw'] = euler[2]

        # # Update position setpoints
        # self.setpoints['x'] = msg.pose.position.x
        # self.setpoints['y'] = msg.pose.position.y
        # self.setpoints['z'] = msg.pose.position.z

        try:
            # Lookup transform from world to the frame specified in msg
            trans = self.tf_buffer.lookup_transform('world', msg.header.frame_id, rospy.Time(0))
            
            # Extract translation and rotation from the transform
            trans_translation = trans.transform.translation
            trans_rotation = trans.transform.rotation
            
            # Convert position
            position = msg.pose.position
            transformed_position = [
                position.x + trans_translation.x,
                position.y + trans_translation.y,
                position.z + trans_translation.z
            ]
            
            # Convert orientation
            quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            trans_quat = [trans_rotation.x, trans_rotation.y, trans_rotation.z, trans_rotation.w]
            world_quat = quaternion_multiply(trans_quat, quat)
            world_euler = euler_from_quaternion(world_quat)
            
            # Save the converted setpoints
            self.setpoints = {
                'x': transformed_position[0],
                'y': transformed_position[1],
                'z': transformed_position[2],
                'roll': world_euler[0],
                'pitch': world_euler[1],
                'yaw': world_euler[2]
            }
            # print(self.setpoints)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF lookup failed: {e}")

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
        while difference > np.pi:
            difference -= 2 * np.pi
        while difference < -np.pi:
            difference += 2 * np.pi
        return difference

    def control_loop(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            orientation_and_position = self.get_current_orientation_and_position()
            if orientation_and_position and self.setpoints.keys():
                wrench_msg = WrenchStamped()
                wrench_msg.header.stamp = rospy.Time.now()
                wrench_msg.header.frame_id = "fossen_com"
                for dof, pid in self.pids.items():
                    if dof in ['roll', 'pitch', 'yaw']:
                        error = StabilizationController.shortest_angular_difference(orientation_and_position[dof], self.setpoints[dof])
                        # if dof == 'yaw':
                        #     print(error)
                        control_output = pid(error)
                    else:
                        error = orientation_and_position[dof] - self.setpoints[dof]
                        control_output = pid(error)
                        # print(orientation_and_position[dof], self.setpoints['z'], control_output)
                        # if dof == 'z':
                        #     print(control_output, orientation_and_position[dof])
                    # if dof in ['roll', 'pitch', 'yaw']:
                    #     setattr(wrench_msg.wrench.torque, dof, control_output)
                    # else:
                    #     setattr(wrench_msg.wrench.force, dof, control_output)
                    if dof == 'roll':
                        wrench_msg.wrench.torque.x = control_output
                    elif dof == 'pitch':
                        wrench_msg.wrench.torque.y = control_output
                    elif dof == 'yaw':
                        wrench_msg.wrench.torque.z = control_output
                    elif dof == 'x':
                        wrench_msg.wrench.force.x = control_output
                    elif dof == 'y':
                        wrench_msg.wrench.force.y = control_output
                    elif dof == 'z':
                        wrench_msg.wrench.force.z = control_output
                # print(wrench_msg)
                self.wrench_pub.publish(wrench_msg)
            rate.sleep()

def get_has_booted():
    return rospy.get_param('/module_info/hasBooted')

if __name__ == '__main__':
    rospy.init_node('stabilization_controller')
    while not get_has_booted():
        time.sleep(5)
    time.sleep(3)
    controller = StabilizationController()
    controller.control_loop()
