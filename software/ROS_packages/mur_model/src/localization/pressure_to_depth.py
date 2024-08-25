#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import tf
import time
from mur.msg import DepthSensor
from queue import Queue

def find_topics_ending_with(suffix):
    # Get a list of all currently published topics
    published_topics = rospy.get_published_topics()

    # Filter the topics to find those that end with the specified suffix
    matching_topics = [topic for topic, _ in published_topics if topic.endswith(suffix)]

    return matching_topics

class PressureToDepthConverter:
    def __init__(self, depth_topics):
        # Initialize the ROS node
        rospy.init_node('pressure_to_depth_converter', anonymous=True)

        self.target_frame_id = 'fossen_com'
        self.base_link_frame_id = 'fossen_base_link'
        self.world_frame_id = 'fossen_world' 

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        # Parameters
        self.density = rospy.get_param('/environment_info/working_fluid/density')
        self.g = rospy.get_param('/environment_info/gravity_constant')


        self.zero_z_depth_pressure_set = False
        self.zero_z_depth_pressure = 0

        # Subscribe to MS5837 pressure topics
        for topic in depth_topics:
            rospy.Subscriber(topic, FluidPressure, self.pressure_callback)

        # Subscribe to reset_zero_z_depth_pressure topic
        rospy.Subscriber('/model_info/reset_zero_z_depth_pressure', Bool, self.reset_zero_z_depth_pressure_callback)


        # Publisher for depth
        self.depth_estimations = Queue()

    def reset_zero_z_depth_pressure_callback(self, msg):
        self.zero_z_depth_pressure_set = False
        rospy.loginfo("Zero z depth pressure has been reset")

    def get_z_axis_diff_in_world_frame(self, msg):
        frame_id = msg.header.frame_id
        try:
            # Wait for the transform from world to target_frame_id
            self.listener.waitForTransform(self.world_frame_id, self.target_frame_id, rospy.Time(0), rospy.Duration(4.0))
            (trans_target, _) = self.listener.lookupTransform(self.world_frame_id, self.target_frame_id, rospy.Time(0))
            
            # Wait for the transform from world to frame_id
            self.listener.waitForTransform(self.world_frame_id, frame_id, rospy.Time(0), rospy.Duration(4.0))
            (trans_frame, _) = self.listener.lookupTransform(self.world_frame_id, frame_id, rospy.Time(0))
            
            # Calculate the z-axis difference in the world frame
            z_diff = trans_frame[2] - trans_target[2]  # z-axis difference in meters
            return -z_diff

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Error: %s", str(e))
            return None

    def pressure_callback(self, msg):
        z_diff = self.get_z_axis_diff_in_world_frame(msg)
        if z_diff is None:
            return
        relative_pressure_change_mbar = self.relative_mbar_from_delta_z(z_diff)
        # print(z_diff, relative_pressure_change_mbar)
        original_pressure_mbar = msg.fluid_pressure
        adjusted_pressure_mbar = original_pressure_mbar + relative_pressure_change_mbar
        if not self.zero_z_depth_pressure_set:
            self.zero_z_depth_pressure = adjusted_pressure_mbar
            self.zero_z_depth_pressure_set = True
        depth = self.calculate_depth(adjusted_pressure_mbar)
        # rospy.loginfo("Calculated depth: %f", depth)
        self.depth_estimations.put(depth)

    def run(self):
        rate = rospy.Rate(25)  # 10 Hz
        while not rospy.is_shutdown():
            while not self.depth_estimations.empty():
                current_depth = self.depth_estimations.get()
                self.update_transform_z_only(self.base_link_frame_id, current_depth)
            rate.sleep()

    def update_transform_z_only(self, frame_id, new_z_value):
        try:
            # Get the current transform
            self.listener.waitForTransform(self.world_frame_id, frame_id, rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.listener.lookupTransform(self.world_frame_id, frame_id, rospy.Time(0))
            
            # Update only the z value
            trans = (trans[0], trans[1], new_z_value)

            # Broadcast the updated transform
            self.broadcaster.sendTransform(
                trans,
                rot,
                rospy.Time.now(),
                frame_id,
                self.world_frame_id
            )

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Error: %s", str(e))

    def relative_mbar_from_delta_z(self, delta_z):
        """
        Calculate the relative pressure change from a change in depth.
        
        delta_z: Change in depth (m)
        
        Returns:
        Pressure change (mbar)
        """
        # Calculate pressure change in Pa
        pressure_change_pa = self.density * self.g * delta_z
        # Convert Pa to mbar
        pressure_change_mbar = pressure_change_pa / 100  # 1 mbar = 100 Pa
        return pressure_change_mbar

    def calculate_depth(self, pressure_mbar):
        """
        Calculate the depth based on the given pressure.
        
        pressure: Measured pressure (mbar)
        
        Returns:
        Depth in meters (m)
        """
        # Convert pressure difference from mbar to Pa
        pressure_difference_pa = (pressure_mbar - self.zero_z_depth_pressure) * 100
        # Calculate depth
        depth = pressure_difference_pa / (self.density * self.g)
        return depth

if __name__ == "__main__":
    try:
        matching_topics = []
        while len(matching_topics) == 0:
            time.sleep(5)
            suffix = "MS5837"
            matching_topics = find_topics_ending_with(suffix)
        converter = PressureToDepthConverter(matching_topics)
        converter.run()
    except rospy.ROSInterruptException:
        pass