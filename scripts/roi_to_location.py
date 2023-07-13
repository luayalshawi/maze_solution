#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped
from mavros_msgs.msg import ExtendedState, State
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class MavrosOffboardPosctl(object):

    def __init__(self):
        rospy.init_node('roi_to_location')
        self.extended_state = ExtendedState()
        self.imu_data = Imu()
        self.local_position = PoseStamped()
        self.state = State()
        self.exit_pos = PointStamped()

        # ROS subscribers
        self.rov_pos_sub = rospy.Subscriber('exit/rawpoint',
                                            PointStamped,
                                            self.exit_pos_callback)

        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback)

        self.pos = PoseStamped()
        self.radius = 0.1

        self.pos_setpoint_pub = rospy.Publisher(
            'exit/point', PoseStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        # self.pos_thread = Thread(target=self.send_pos, args=())
        self.run()


    #
    # Callback functions
    #
    def exit_pos_callback(self, msg: PointStamped):
        self.exit_pos = msg
        euler_current = euler_from_quaternion([
            self.local_position.pose.orientation.x,
            self.local_position.pose.orientation.y,
            self.local_position.pose.orientation.z,
            self.local_position.pose.orientation.w], axes='rzyx')

        # print('Drone location: {0},{1}'.format(self.local_position.pose.position.x, self.local_position.pose.position.y))
        print('exit_pos location: {0},{1}'.format(self.exit_pos.point.x, self.exit_pos.point.y))
        drone = np.array([
            self.local_position.pose.position.x,
            self.local_position.pose.position.y])

        exit = np.array([
            self.exit_pos.point.x,
            self.exit_pos.point.y])

        direction = exit - drone
        # print('Drone direction before: {0}'.format(direction))
        direction /= np.linalg.norm(direction)
        # print('Drone direction after: {0}'.format(direction))

        d_separation = 1.0
        altitude = 0.5
        
        p_goal = exit - direction*d_separation

        yaw = np.arctan2(direction[1], direction[0])
        # print('X goal:{0} - Y goal:{1}'.format(p_goal[0], p_goal[1]))
        # set a position setpoint
        # self.pos.pose.position.x = p_goal[0]
        self.pos.pose.position.x = self.exit_pos.point.x
        # self.pos.pose.position.y = p_goal[1]
        self.pos.pose.position.y = self.exit_pos.point.y
        self.pos.pose.position.z = altitude

        # For demo purposes we will lock yaw/heading to north.
        yaw = math.radians(np.rad2deg(yaw))
        quaternion = quaternion_from_euler(0, 0, np.rad2deg(yaw))
        self.pos.pose.orientation = Quaternion(*quaternion)
        
        self.pos.header = Header()
        self.pos.header.frame_id = "drone"
        self.pos.header.stamp = rospy.Time.now()
        self.pos_setpoint_pub.publish(self.pos)              
        # self.goto_position(x=p_goal[0], y=p_goal[1], z=altitude, yaw_deg=np.rad2deg(yaw))

    def goto_position(self, x, y, z, yaw_deg):
        """goto position"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z

        # For demo purposes we will lock yaw/heading to north.
        yaw = math.radians(yaw_deg)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

    def local_position_callback(self, data: PoseStamped):
        self.local_position = data

    def start_sending_position_setpoint(self):
        self.pos_thread.start()

    def __del__(self):
        rospy.signal_shutdown('finished script')
        # if self.pos_thread.is_alive():
        #     self.pos_thread.join()

    #
    # Helper methods
    #
    def send_pos(self):
        self.pos.header = Header()
        self.pos.header.frame_id = "drone"
        self.pos.header.stamp = rospy.Time.now()
        self.pos_setpoint_pub.publish(self.pos)
        rospy.logdebug('sending offboard {0:.2f} {1:.2f} {2:.2f}'.format(
                self.pos.pose.position.x,
                self.pos.pose.position.y,
                self.pos.pose.position.z))

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset



    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        MavrosOffboardPosctl()
    except rospy.exceptions.ROSInterruptException:
        pass
