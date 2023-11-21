#!/usr/bin/env python

import rospy
from mavros_msgs.msg import WaypointList, Waypoint

def publish_waypoints():
    rospy.init_node('publish_waypoints', anonymous=True)

    # Create a waypoint list
    waypoints = WaypointList()

    # Define waypoints (adjust as needed)
    waypoint1 = Waypoint()
    waypoint1.frame = Waypoint.FRAME_GLOBAL
    waypoint1.command = Waypoint.NAV_WAYPOINT
    waypoint1.is_current = True
    waypoint1.autocontinue = True
    waypoint1.param1 = 15.0  # Hold time in seconds
    waypoint1.param4 = float('nan')  # Acceptance radius, set to NaN to use default
    waypoint1.x_lat = 47.612088  # Latitude of the waypoint
    waypoint1.y_long = -122.201818  # Longitude of the waypoint
    waypoint1.z_alt = 10.0  # Altitude of the waypoint in meters

    waypoint2 = Waypoint()
    waypoint2.frame = Waypoint.FRAME_GLOBAL
    waypoint2.command = Waypoint.NAV_WAYPOINT
    waypoint2.is_current = False
    waypoint2.autocontinue = True
    waypoint2.param1 = 15.0
    waypoint2.param4 = float('nan')
    waypoint2.x_lat = 47.611987
    waypoint2.y_long = -122.201717
    waypoint2.z_alt = 20.0

    # Add waypoints to the list
    waypoints.waypoints = [waypoint1, waypoint2]

    # Publish the waypoint list
    pub = rospy.Publisher('/uav0/mavros/mission/waypoints', WaypointList, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        pub.publish(waypoints)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_waypoints()
    except rospy.ROSInterruptException:
        pass
