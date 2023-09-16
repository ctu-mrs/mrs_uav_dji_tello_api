#!/usr/bin/python3

import rospy
import rosnode
import time
import sys
import numpy as np

import av
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import traceback

import tellopy

from sensor_msgs.msg import BatteryState as BatteryState
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry as Odometry
from std_msgs.msg import Float64 as Float64
from geometry_msgs.msg import TwistStamped as TwistStamped
from geometry_msgs.msg import PoseStamped as PoseStamped

from std_srvs.srv import Trigger as TriggerSrv
from std_srvs.srv import TriggerResponse as TriggerSrvResponse
from std_srvs.srv import SetBool as SetBoolSrv
from std_srvs.srv import SetBoolResponse as SetBoolSrvResponse

from mrs_msgs.msg import HwApiVelocityHdgRateCmd as HwApiVelocityHdgRateCmd

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

from std_msgs.msg import Bool

class TellopyWrapper:

    def __init__(self):

        self.prev_flight_data = None
        self.flight_data = None
        self.log_data = None

        rospy.init_node('tellopy_wrapper', anonymous=True)

        self.uav_name = rospy.get_param("~uav_name")

        self.offset_x = 0
        self.offset_y = 0
        self.offset_z = 0

        self.publisher_battery = rospy.Publisher('~battery_out', BatteryState, queue_size=1)
        self.publisher_twist = rospy.Publisher('~twist_out', TwistStamped, queue_size=1)
        self.publisher_imu = rospy.Publisher('~imu_out', Imu, queue_size=1)
        self.publisher_pose = rospy.Publisher('~pose_out', PoseStamped, queue_size=1)
        self.publisher_height = rospy.Publisher('~height_out', Float64, queue_size=1)
        self.publisher_armed = rospy.Publisher('~armed_out', Bool, queue_size=1)
        self.publisher_image = rospy.Publisher("~image_raw_out", Image, queue_size=10)
        self.publisher_camera_info = rospy.Publisher("~camera_info_out", CameraInfo, queue_size=1)

        self.bridge = CvBridge()

        self.service_arm = rospy.Service('~arm_in', SetBoolSrv, self.callbackArm)
        self.service_throw = rospy.Service('~throw_in', TriggerSrv, self.callbackThrow)
        self.service_zero = rospy.Service('~zero_in', TriggerSrv, self.callbackZero)

        self.sub_cmd = rospy.Subscriber('~cmd_in', HwApiVelocityHdgRateCmd, self.callbackCmd, queue_size=1)

        self.diag_timer = rospy.Timer(rospy.Duration(1.0), self.diagTimer)
        self.main_timer = rospy.Timer(rospy.Duration(1.0/30.0), self.mainTimer)

        self.tello = tellopy.Tello()
        self.tello.connect()

        self.tello.subscribe(self.tello.EVENT_FLIGHT_DATA, self.handler)
        self.tello.subscribe(self.tello.EVENT_LOG_DATA, self.handler)
        self.tello.fast_mode = False

        self.rate = rospy.Rate(15)

        self.taking_off = False
        self.is_flying = False

        retry = 5
        container = None
        while container is None and 0 < retry:
            retry -= 1
            try:
                container = av.open(self.tello.get_video_stream())
            except av.AVError as ave:
                rospy.logerr('Could not open the video stream')

        while not rospy.is_shutdown():

            ## | ------------------------- height ------------------------- |

            # try:
            #     height = float(self.flight_data.height)
            #     height_msg = Float64()
            #     height_msg.data = height
            #     self.publisher_height.publish(height_msg)
            # except:
            #     rospy.logerr_throttle(1.0, 'failed to query height')
            #     pass

            for frame in container.decode(video=0):

                image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)

                try:
                    image_message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                    image_message.header.frame_id = self.uav_name+"/rgb"
                    image_message.header.stamp = rospy.Time.now() - rospy.Time(0.01)
                    self.publisher_image.publish(image_message)
                except CvBridgeError as e:
                    rospy.logerr('Could not parse and publish the camera image')

            self.rate.sleep();

        self.tello.quit()

    def callbackThrow(self, req):

        resp = TriggerSrvResponse()

        try:
            self.tello.throw_and_go()
            self.is_flying = True

            resp.message = "armed"
            resp.success = True

        except:

            resp.message = "failed"
            resp.success = False

            pass

        return resp

    def callbackZero(self, req):

        resp = TriggerSrvResponse()

        self.offset_x = -self.position_x
        self.offset_y = -self.position_y
        self.offset_z = -self.position_z

        resp.message = "zeroed"
        resp.success = True

        return resp

    def callbackArm(self, req):

        resp = SetBoolSrvResponse()

        if req.data:

            # self.taking_off = True

            try:
                self.is_flying = True
                self.tello.takeoff()
            except:
                pass

            resp.message = "armed"
            resp.success = True

            # self.taking_off = False

        else:

          try:
              self.tello.land()
          except:
              pass

          resp.message = "disarmed"
          resp.success = True

        return resp

    def mainTimer(self, event):

        rospy.loginfo_once('main timer spinning')

        D = [-0.030801026384372737, 0.12074238137787453, 0.007389158172456661, 0.004696793411715399, 0.0]
        K = [933.5640667549508, 0.0, 500.5657553739987, 0.0, 931.5001605952165, 379.0130687255228, 0.0, 0.0, 1.0]
        R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        P = [942.8484497070312, 0.0, 503.21163889504896, 0.0, 0.0, 939.0140380859375, 382.6534805428564, 0.0, 0.0, 0.0, 1.0, 0.0]

        ## | ------------------- publish camera info ------------------ |

        camera_info_msg = CameraInfo()
        camera_info_msg.header.frame_id = "tello_camera"
        camera_info_msg.header.stamp = rospy.Time.now()
        camera_info_msg.width = 960
        camera_info_msg.height = 720
        camera_info_msg.K = K
        camera_info_msg.D = D
        camera_info_msg.R = R
        camera_info_msg.P = P
        camera_info_msg.distortion_model = "plumb_bob"

        self.publisher_camera_info.publish(camera_info_msg)

    def diagTimer(self, event):

        ## | ------------------------- battery ------------------------ |

        # try:
        #     battery_percentage = self.tello.query_battery()
        #     battery_msg = BatteryState()
        #     battery_msg.percentage = battery_percentage
        #     battery_msg.header.stamp = rospy.Time.now()
        #     self.publisher_battery.publish(battery_msg)
        # except:
        #     rospy.logerr_throttle(1.0, 'failed to query battery')
        #     pass

        is_flying_msg = Bool()
        is_flying_msg.data = True
        self.publisher_armed.publish(is_flying_msg)

    def callbackCmd(self, msg):

        vel_x = msg.velocity.x*0.6
        vel_y = msg.velocity.y*0.6
        vel_z = msg.velocity.z*0.6
        hdg_rate = msg.heading_rate*0.6

        self.tello.set_pitch(vel_x)
        self.tello.set_roll(-vel_y)
        self.tello.set_throttle(vel_z)
        self.tello.set_yaw(-hdg_rate)

    def handler(self, event, sender, data, **args):
        drone = sender
        if event is drone.EVENT_FLIGHT_DATA:
            rospy.loginfo_once('getting flight data')
            if self.prev_flight_data != str(data):
                self.prev_flight_data = str(data)
            self.flight_data = data
        elif event is drone.EVENT_LOG_DATA:

            rospy.loginfo('{}'.format(data))

            rospy.loginfo_once('getting log data')
            self.log_data = data

            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "world"

            pose.pose.position.x = data.mvo.pos_x + self.offset_x
            pose.pose.position.y = -data.mvo.pos_y + self.offset_y
            pose.pose.position.z = -data.mvo.pos_z + self.offset_z

            self.position_x = data.mvo.pos_x
            self.position_y = -data.mvo.pos_y
            self.position_z = -data.mvo.pos_z

            angles = euler_from_quaternion([data.imu.q0, data.imu.q1, data.imu.q2, data.imu.q3])

            quat = quaternion_from_euler(-angles[0], -angles[1], angles[2])

            pose.pose.orientation.w=quat[0]
            pose.pose.orientation.x=quat[1]
            pose.pose.orientation.y=quat[2]
            pose.pose.orientation.z=quat[3]

            self.publisher_pose.publish(pose)

            twist = TwistStamped()

            twist.header.stamp = rospy.Time.now()
            twist.header.frame_id = "body"

            twist.twist.linear.x = (data.mvo.vel_x/10.0)
            twist.twist.linear.y = -(data.mvo.vel_y/10.0)
            twist.twist.linear.z = -(data.mvo.vel_z/10.0)

            self.publisher_twist.publish(twist)

            imu_msg = Imu()

            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = self.uav_name+"/fcu"

            imu_msg.orientation.w=quat[0]
            imu_msg.orientation.x=quat[1]
            imu_msg.orientation.y=quat[2]
            imu_msg.orientation.z=quat[3]

            imu_msg.angular_velocity.x = data.imu.gyro_x
            imu_msg.angular_velocity.y = -data.imu.gyro_y
            imu_msg.angular_velocity.z = -data.imu.gyro_z

            imu_msg.linear_acceleration.x = data.imu.acc_x*9.81
            imu_msg.linear_acceleration.y = -data.imu.acc_y*9.81
            imu_msg.linear_acceleration.z = -data.imu.acc_z*9.81

            self.publisher_imu.publish(imu_msg)

        else:
            print('event="%s" data=%s' % (event.getname(), str(data)))

if __name__ == '__main__':
    try:
        tellopy_wrapper = TellopyWrapper()
    except rospy.ROSInterruptException:
        pass
