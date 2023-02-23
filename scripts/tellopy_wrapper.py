#!/usr/bin/python3

import rospy
import rosnode
import time
import sys
import numpy as np

from djitellopy import Tello

from sensor_msgs.msg import BatteryState as BatteryState
from nav_msgs.msg import Odometry as Odometry
from std_msgs.msg import Float64 as Float64
from geometry_msgs.msg import TwistStamped as TwistStamped
from geometry_msgs.msg import PoseStamped as PoseStamped

from std_srvs.srv import SetBool as SetBoolSrv
from std_srvs.srv import SetBoolResponse as SetBoolSrvResponse

from mrs_msgs.msg import HwApiVelocityHdgRateCmd as HwApiVelocityHdgRateCmd

from tf.transformations import quaternion_from_euler

from std_msgs.msg import Bool

class TellopyWrapper:

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

    def diagTimer(self, event):

        ## | ------------------------- battery ------------------------ |

        try:
            battery_percentage = self.tello.query_battery()
            battery_msg = BatteryState()
            battery_msg.percentage = battery_percentage
            battery_msg.header.stamp = rospy.Time.now()
            self.publisher_battery.publish(battery_msg)
        except:
            rospy.logerr_throttle(1.0, 'failed to query battery')
            pass

        is_flying_msg = Bool()
        is_flying_msg.data = True
        self.publisher_armed.publish(is_flying_msg)

    def callbackCmd(self, msg):

        vel_x = msg.velocity.x * 100.0
        vel_y = msg.velocity.y * 100.0
        vel_z = msg.velocity.z * 100.0
        hdg_rate = (msg.heading_rate / 3.1415) * 180

        self.tello.send_rc_control(int(-vel_y), int(vel_x), int(vel_z), int(-hdg_rate));

    def __init__(self):

        rospy.init_node('tellopy_wrapper', anonymous=True)

        self.publisher_battery = rospy.Publisher('~battery_out', BatteryState, queue_size=1)
        self.publisher_twist = rospy.Publisher('~twist_out', TwistStamped, queue_size=1)
        self.publisher_pose = rospy.Publisher('~pose_out', PoseStamped, queue_size=1)
        self.publisher_height = rospy.Publisher('~height_out', Float64, queue_size=1)
        self.publisher_armed = rospy.Publisher('~armed_out', Bool, queue_size=1)

        self.service_arm = rospy.Service('~arm_in', SetBoolSrv, self.callbackArm)

        self.sub_cmd = rospy.Subscriber('~cmd_in', HwApiVelocityHdgRateCmd, self.callbackCmd, queue_size=1)

        self.diag_timer = rospy.Timer(rospy.Duration(1.0), self.diagTimer)

        self.tello = Tello()
        self.tello.connect()
        self.tello.set_speed(100)

        self.rate = rospy.Rate(15)

        self.taking_off = False
        self.is_flying = False

        while not rospy.is_shutdown():

            if self.taking_off:
                self.rate.sleep();
                continue

            ## | ------------------------ odometry ------------------------ |
            try:

                roll = (3.141592 / 180)*float(self.tello.get_roll())
                pitch = -(3.141592 / 180)*float(self.tello.get_pitch())
                yaw = -(3.141592 / 180)*float(self.tello.get_yaw())

                q = quaternion_from_euler(roll, pitch, yaw)

                vel_x = float(self.tello.get_speed_x())/10.0
                vel_y = float(self.tello.get_speed_y())/10.0
                vel_z = float(self.tello.get_speed_z())/10.0

                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "world"

                pose.pose.position.x = 0
                pose.pose.position.y = 0
                pose.pose.position.z = 0

                pose.pose.orientation.x=float(q[0])
                pose.pose.orientation.y=float(q[1])
                pose.pose.orientation.z=float(q[2])
                pose.pose.orientation.w=float(q[3])

                self.publisher_pose.publish(pose)

                twist = TwistStamped()

                twist.header.stamp = rospy.Time.now()
                twist.header.frame_id = "body"

                twist.twist.linear.x = vel_x
                twist.twist.linear.y = -vel_y
                twist.twist.linear.z = -vel_z

                self.publisher_twist.publish(twist)
            except:
                rospy.logerr_throttle(1.0, 'could not query odometry')
                pass

            ## | ------------------------- height ------------------------- |

            # try:
            #     height = float(self.tello.query_distance_tof()) / 100.0
            #     height_msg = Float64()
            #     height_msg.data = height
            #     self.publisher_height.publish(height_msg)
            # except:
            #     rospy.logerr_throttle(1.0, 'failed to query height')
            #     pass

            self.rate.sleep();

if __name__ == '__main__':
    try:
        tellopy_wrapper = TellopyWrapper()
    except rospy.ROSInterruptException:
        pass
