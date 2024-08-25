#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Quaternion
from tf.transformations import quaternion_multiply, quaternion_inverse, euler_from_quaternion
import tf.transformations as tf_transformations

class IMUToTFBroadcaster:
    def __init__(self):
        rospy.init_node('imu_to_tf_broadcaster')

        # Fetch parameters from the sensors namespace
        sensors_config = rospy.get_param('/sensor_info/sensors')

        imu_of_interest = 'BNO055'
        instance_module = 'mur_battery_fuselage'


        sensor_data_all = sensors_config[imu_of_interest]
        self.sensor_topic_base = sensor_data_all['topic']
        self.sensor_topic_type_string = sensor_data_all['type']

        self.imu_sensor_topic = f"/sensors/{instance_module}/{self.sensor_topic_base}"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.Subscriber(self.imu_sensor_topic, Imu, self.imu_callback)

        
    # def imu_callback(self, imu_msg):
    #     try:
    #         # Step 1: Read the IMU orientation (global measurement in 'world' frame)
    #         imu_orientation = imu_msg.orientation
    #         imu_quat = [imu_orientation.x, imu_orientation.y, imu_orientation.z, imu_orientation.w]

    #         # Step 2: Get the current transform from world to imu_frame
    #         imu_frame = imu_msg.header.frame_id
    #         try:
    #             world_to_imu = self.tf_buffer.lookup_transform('world', imu_frame, rospy.Time(0))
    #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #             rospy.logerr("TF lookup failed for world to imu_frame: {}".format(e))
    #             return

    #         # Step 3: Get the current transform from fossen_base_link to imu_frame
    #         try:
    #             base_link_to_imu = self.tf_buffer.lookup_transform('fossen_base_link', imu_frame, rospy.Time(0))
    #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #             rospy.logerr("TF lookup failed for fossen_base_link to imu_frame: {}".format(e))
    #             return

    #         # Step 4: Calculate the relative orientation between base_link and imu_frame
    #         base_link_quat = [
    #             base_link_to_imu.transform.rotation.x,
    #             base_link_to_imu.transform.rotation.y,
    #             base_link_to_imu.transform.rotation.z,
    #             base_link_to_imu.transform.rotation.w
    #         ]
    #         relative_orientation = tf_transformations.quaternion_inverse(base_link_quat)

    #         # Step 5: Combine the IMU orientation with the relative orientation
    #         corrected_orientation = tf_transformations.quaternion_multiply(imu_quat, relative_orientation)

    #         # Step 6: Get the current transform from world to fossen_base_link
    #         try:
    #             world_to_base_link = self.tf_buffer.lookup_transform('world', 'fossen_base_link', rospy.Time(0))
    #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #             rospy.logerr("TF lookup failed for world to fossen_base_link: {}".format(e))
    #             return

    #         # Step 7: Create a new transform for predicted_base_link with the corrected orientation
    #         t = TransformStamped()
    #         t.header.stamp = rospy.Time.now()
    #         t.header.frame_id = 'world'
    #         t.child_frame_id = 'predicted_fossen_base_link'

    #         # Use the existing translation from world to base_link
    #         t.transform.translation.x = world_to_base_link.transform.translation.x
    #         t.transform.translation.y = world_to_base_link.transform.translation.y
    #         t.transform.translation.z = world_to_base_link.transform.translation.z
    #         t.transform.rotation.x = corrected_orientation[0]
    #         t.transform.rotation.y = corrected_orientation[1]
    #         t.transform.rotation.z = corrected_orientation[2]
    #         t.transform.rotation.w = corrected_orientation[3]

    #         # Broadcast the new transform
    #         self.tf_broadcaster.sendTransform(t)

    #     except Exception as e:
    #         rospy.logerr("Unexpected error: {}".format(e))

    def imu_callback(self, imu_msg):
        try:
            # Step 1: Read the IMU orientation (global measurement in 'world' frame)
            imu_orientation = imu_msg.orientation
            imu_quat = [imu_orientation.x, imu_orientation.y, imu_orientation.z, imu_orientation.w]

            # Step 2: Get the current transform from world to imu_frame
            imu_frame = imu_msg.header.frame_id
            try:
                world_to_imu = self.tf_buffer.lookup_transform('world', imu_frame, rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr("TF lookup failed for world to imu_frame: {}".format(e))
                return

            # Step 3: Get the current transform from fossen_base_link to imu_frame
            try:
                base_link_to_imu = self.tf_buffer.lookup_transform('fossen_base_link', imu_frame, rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr("TF lookup failed for fossen_base_link to imu_frame: {}".format(e))
                return

            # Step 4: Calculate the relative orientation between base_link and imu_frame
            base_link_quat = [
                base_link_to_imu.transform.rotation.x,
                base_link_to_imu.transform.rotation.y,
                base_link_to_imu.transform.rotation.z,
                base_link_to_imu.transform.rotation.w
            ]
            relative_orientation = tf_transformations.quaternion_inverse(base_link_quat)

            # Step 5: Combine the IMU orientation with the relative orientation
            corrected_orientation = tf_transformations.quaternion_multiply(imu_quat, relative_orientation)

            # Step 6: Get the current transform from world to fossen_base_link
            try:
                world_to_base_link = self.tf_buffer.lookup_transform('world', 'fossen_base_link', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr("TF lookup failed for world to fossen_base_link: {}".format(e))
                return

            # Step 7: Update the transform for fossen_base_link with the corrected orientation
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = 'world'
            t.child_frame_id = 'fossen_base_link'

            # Use the existing translation from world to base_link
            t.transform.translation.x = world_to_base_link.transform.translation.x
            t.transform.translation.y = world_to_base_link.transform.translation.y
            t.transform.translation.z = world_to_base_link.transform.translation.z
            t.transform.rotation.x = corrected_orientation[0]
            t.transform.rotation.y = corrected_orientation[1]
            t.transform.rotation.z = corrected_orientation[2]
            t.transform.rotation.w = corrected_orientation[3]

            # Broadcast the updated transform
            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            rospy.logerr("Unexpected error: {}".format(e))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    imu_to_tf_broadcaster = IMUToTFBroadcaster()
    imu_to_tf_broadcaster.run()
