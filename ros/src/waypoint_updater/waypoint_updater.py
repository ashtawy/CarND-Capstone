#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import math
import yaml
from std_msgs.msg import Int32

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
MAX_SPEED_MiPH = 10
MAX_SPEED_MePS = MAX_SPEED_MiPH*0.44704
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = None
        config_string = rospy.get_param("/traffic_light_config")
        is_site = yaml.load(config_string)['is_site']
        self.freq = 50 if is_site else 20
        self.loop()

        #rospy.spin()

    def loop(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoint_tree and self.stopline_wp_idx:
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]

        if self.stopline_wp_idx != -1 and self.stopline_wp_idx < farthest_idx:
            lane.waypoints = self.decelerate_waypoints(lane.waypoints, closest_idx)
        #rospy.logwarn('Setting Speed to  %f'%(lane.waypoints[0].twist.twist.linear.x))
        self.final_waypoints_pub.publish(lane)

    def accelerate_waypoints(self, min_v, max_v, wp_indx, n_waypoints):
        return np.linspace(min_v, max_v, n_waypoints)[wp_indx]

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            # two waypoints back from line so front of car stops to linear
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) 
            dist = self.distance(waypoints, i ,stop_idx)
            vel = math.sqrt(2*MAX_DECEL * dist) # remove sqrt if you want smoother stop
            vel = abs(2*MAX_DECEL * dist)
            if vel < 1:
                vel = 0.
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        return temp

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
            for wpi in range(len(self.base_waypoints.waypoints)):
                self.set_waypoint_velocity(self.base_waypoints.waypoints, wpi, MAX_SPEED_MePS)
            #rospy.logwarn(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        #rospy.logwarn('traffic_cb')
        #rospy.logwarn(msg.data)
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        #current_vel = waypoints[waypoint].twist.twist.linear.x 
        waypoints[waypoint].twist.twist.linear.x = velocity
        #rospy.logwarn('waypoint %d init velocity = %f, and target velocity = %f, new_vel = %f'%(waypoint, current_vel, velocity, waypoints[waypoint].twist.twist.linear.x))

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
