#!/usr/bin/python3

import rospy
import rosnode
import time
import sys
import numpy as np

import tellopy

from sensor_msgs.msg import BatteryState as BatteryState
from nav_msgs.msg import Odometry as Odometry
from std_msgs.msg import Float64 as Float64
from geometry_msgs.msg import TwistStamped as TwistStamped
from geometry_msgs.msg import PoseStamped as PoseStamped

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

        self.publisher_battery = rospy.Publisher('~battery_out', BatteryState, queue_size=1)
        self.publisher_twist = rospy.Publisher('~twist_out', TwistStamped, queue_size=1)
        self.publisher_pose = rospy.Publisher('~pose_out', PoseStamped, queue_size=1)
        self.publisher_height = rospy.Publisher('~height_out', Float64, queue_size=1)
        self.publisher_armed = rospy.Publisher('~armed_out', Bool, queue_size=1)

        self.service_arm = rospy.Service('~arm_in', SetBoolSrv, self.callbackArm)

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

        while not rospy.is_shutdown():

            ## | ------------------------- height ------------------------- |

            try:
                height = float(self.flight_data.height)
                height_msg = Float64()
                height_msg.data = height
                self.publisher_height.publish(height_msg)
            except:
                rospy.logerr_throttle(1.0, 'failed to query height')
                pass

            self.rate.sleep();

        self.tello.quit()


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
        pass


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

            pose.pose.position.x = data.mvo.pos_x
            pose.pose.position.y = -data.mvo.pos_y
            pose.pose.position.z = -data.mvo.pos_z

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

        else:
            print('event="%s" data=%s' % (event.getname(), str(data)))

if __name__ == '__main__':
    try:
        tellopy_wrapper = TellopyWrapper()
    except rospy.ROSInterruptException:
        pass
