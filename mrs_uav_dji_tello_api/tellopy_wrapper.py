#!/usr/bin/env python3

import sys
import importlib
import subprocess
import time
import numpy as np

# install_and_import("av")
import av
import cv2

from rclpy.duration import Duration
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import (
    Image, BatteryState, CameraInfo, Imu
)
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_srvs.srv import Trigger, SetBool
from mrs_msgs.msg import HwApiVelocityHdgRateCmd
from cv_bridge import CvBridge, CvBridgeError
import tellopy
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class TellopyWrapperNode(Node):

    # #{ __init__

    def __init__(self):
        super().__init__('tellopy_wrapper')

        self.cbkgr_video = MutuallyExclusiveCallbackGroup()
        self.cbkgr_ss = MutuallyExclusiveCallbackGroup()
        self.cbkgr_timers = MutuallyExclusiveCallbackGroup()

        # Parameters
        self.declare_parameter('uav_name', 'tello')
        self.uav_name = self.get_parameter('uav_name').value

        self.offset_x = 0
        self.offset_y = 0
        self.offset_z = 0
        self.position_x = 0
        self.position_y = 0
        self.position_z = 0
        self.prev_flight_data = None
        self.flight_data = None
        self.log_data = None
        self.taking_off = False
        self.is_flying = False

        # Publishers
        self.publisher_battery = self.create_publisher(BatteryState, '~/battery_out', 10)
        self.publisher_twist = self.create_publisher(TwistStamped, '~/twist_out', 10)
        self.publisher_imu = self.create_publisher(Imu, '~/imu_out', 10)
        self.publisher_pose = self.create_publisher(PoseStamped, '~/pose_out', 10)
        self.publisher_height = self.create_publisher(Float64, '~/height_out', 10)
        self.publisher_armed = self.create_publisher(Bool, '~/armed_out', 10)
        self.publisher_image = self.create_publisher(Image, "~/image_raw_out", 10)
        self.publisher_camera_info = self.create_publisher(CameraInfo, "~/camera_info_out", 10)

        # Bridge for images
        self.bridge = CvBridge()

        # Services
        self.service_arm = self.create_service(SetBool, '~/arm_in', self.callback_arm, callback_group=self.cbkgr_ss)
        self.service_throw = self.create_service(Trigger, '~/throw_in', self.callback_throw, callback_group=self.cbkgr_ss)
        self.service_zero = self.create_service(Trigger, '~/zero_in', self.callback_zero, callback_group=self.cbkgr_ss)

        # Subscribers
        self.sub_cmd = self.create_subscription(
            HwApiVelocityHdgRateCmd, '~/cmd_in', self.callback_cmd, 10
        )

        self.get_logger().info('creating timmers')

        # Timers
        self.diag_timer = self.create_timer(1.0, self.diag_timer_cb, callback_group=self.cbkgr_timers)

        self.get_logger().info('creating tellopy instance')

        # Tello drone connection
        self.tello = tellopy.Tello()
        self.tello.connect()
        self.tello.subscribe(self.tello.EVENT_FLIGHT_DATA, self.handler)
        self.tello.subscribe(self.tello.EVENT_LOG_DATA, self.handler)
        self.tello.fast_mode = False

        self.get_logger().info('trying to open video stream')

        retry = 5
        container = None
        while container is None and 0 < retry:
            retry -= 1
            try:
                container = av.open(self.tello.get_video_stream())
            except Exception as ave:
                self.get_logger().error('Could not open the video stream')
                time.sleep(1)

        self.get_logger().info('video container established')

        if container is not None:
            self.create_timer(0.01, lambda: self.publish_video(container), callback_group=self.cbkgr_video)

        self.get_logger().info('initialized')

    # #} end of __init__

    # #{ publish video timer

    ##
    # @brief this is a timer which should be running indefinitely and the video is grabed inside the for loop
    #
    # @param container
    #
    # @return 
    def publish_video(self, container):

        for frame in container.decode(video=0):

            image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)

            try:
                image_message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                image_message.header.frame_id = str(self.uav_name) + "/rgb"
                new_time = self.get_clock().now() - Duration(seconds=0.1)
                image_message.header.stamp = new_time.to_msg()

                self.publisher_image.publish(image_message)

                D = [-0.030801026384372737, 0.12074238137787453, 0.007389158172456661, 0.004696793411715399, 0.0]
                K = [933.5640667549508, 0.0, 500.5657553739987, 0.0, 931.5001605952165, 379.0130687255228, 0.0, 0.0, 1.0]
                R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                P = [942.8484497070312, 0.0, 503.21163889504896, 0.0, 0.0, 939.0140380859375, 382.6534805428564, 0.0, 0.0, 0.0, 1.0, 0.0]
                camera_info_msg = CameraInfo()
                camera_info_msg.header = image_message.header
                camera_info_msg.width = 960
                camera_info_msg.height = 720
                camera_info_msg.k = K
                camera_info_msg.d = D
                camera_info_msg.r = R
                camera_info_msg.p = P
                camera_info_msg.distortion_model = "plumb_bob"

                self.publisher_camera_info.publish(camera_info_msg)

            except CvBridgeError:
                self.get_logger().error('Could not parse and publish the camera image')

    # #} end of publish video timer

    # #{ callback_throw()

    def callback_throw(self, request, response):

        print("callback throw")

        try:
            self.tello.throw_and_go()
            self.is_flying = True
            response.message = "armed"
            response.success = True
        except Exception as e:
            response.message = "failed"
            response.success = False

        return response

    # #} end of callback_throw()

    # #{ callback_zero()

    def callback_zero(self, request, response):

        self.get_logger().info('zeroing')

        self.offset_x = -self.position_x
        self.offset_y = -self.position_y
        self.offset_z = -self.position_z
        response.message = "zeroed"
        response.success = True
        return response

    # #} end of callback_zero()

    # #{callback_arm()

    def callback_arm(self, request, response):

        self.get_logger().info('callback arm')

        if request.data:

            self.get_logger().info('taking off')

            try:
                self.is_flying = True
                self.tello.takeoff()
            except Exception as e:
                pass
            response.message = "armed"
            response.success = True
        else:

            self.get_logger().info('landing')

            try:
                self.tello.land()
            except Exception as e:
                pass
            response.message = "disarmed"
            response.success = True

        return response

    # #} end of callback_arm()

    # #{ diaag_timer_cb()

    def diag_timer_cb(self):

        is_flying_msg = Bool()
        is_flying_msg.data = self.is_flying

        self.publisher_armed.publish(is_flying_msg)

    # #} end of diaag_timer_cb()

    # #{ callback_cmd()

    def callback_cmd(self, msg):

        vel_x = msg.velocity.x * 0.6
        vel_y = msg.velocity.y * 0.6
        vel_z = msg.velocity.z * 0.6
        hdg_rate = msg.heading_rate * 0.6
        self.tello.set_pitch(vel_x)
        self.tello.set_roll(-vel_y)
        self.tello.set_throttle(vel_z)
        self.tello.set_yaw(-hdg_rate)

    # #} end of callback_cmd()

    # #{ tellopy's callback

    def handler(self, event, sender, data, **args):

        drone = sender

        if event is drone.EVENT_FLIGHT_DATA:

            if self.prev_flight_data != str(data):

                self.prev_flight_data = str(data)
                self.flight_data = data

        elif event is drone.EVENT_LOG_DATA:

            self.get_logger().info('{}'.format(data))

            self.log_data = data

            # Pose
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "world"
            pose.pose.position.x = data.mvo.pos_x + self.offset_x
            pose.pose.position.y = -data.mvo.pos_y + self.offset_y
            pose.pose.position.z = -data.mvo.pos_z + self.offset_z
            self.position_x = data.mvo.pos_x
            self.position_y = -data.mvo.pos_y
            self.position_z = -data.mvo.pos_z
            angles = euler_from_quaternion([data.imu.q0, data.imu.q1, data.imu.q2, data.imu.q3])
            quat = quaternion_from_euler(-angles[0], -angles[1], angles[2])
            pose.pose.orientation.w = quat[0]
            pose.pose.orientation.x = quat[1]
            pose.pose.orientation.y = quat[2]
            pose.pose.orientation.z = quat[3]

            self.publisher_pose.publish(pose)

            # Twist
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = "body"
            twist.twist.linear.x = (data.mvo.vel_x / 10.0)
            twist.twist.linear.y = -(data.mvo.vel_y / 10.0)
            twist.twist.linear.z = -(data.mvo.vel_z / 10.0)

            self.publisher_twist.publish(twist)

            # IMU
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = str(self.uav_name) + "/fcu"
            imu_msg.orientation.w = quat[0]
            imu_msg.orientation.x = quat[1]
            imu_msg.orientation.y = quat[2]
            imu_msg.orientation.z = quat[3]
            imu_msg.angular_velocity.x = data.imu.gyro_x
            imu_msg.angular_velocity.y = -data.imu.gyro_y
            imu_msg.angular_velocity.z = -data.imu.gyro_z
            imu_msg.linear_acceleration.x = data.imu.acc_x * 9.81
            imu_msg.linear_acceleration.y = -data.imu.acc_y * 9.81
            imu_msg.linear_acceleration.z = -data.imu.acc_z * 9.81

            self.publisher_imu.publish(imu_msg)

    # #} end of tellopy's callback

def main(args=None):
    rclpy.init(args=args)

    node = TellopyWrapperNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
