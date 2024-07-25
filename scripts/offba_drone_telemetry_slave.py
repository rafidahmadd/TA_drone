#!/usr/bin/env python

import rospy
import os
import rospy
import rospkg
import subprocess
import roslaunch
import serial
import datetime
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import *
from mavros_msgs.srv import *
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import State


# global variable
latitude = 0.0
longitude = 0.0
height = 3
current_state = State()


def state_cb(msg):
    global current_state
    current_state = msg
    # print("callback")


def setOffboardMode():
    rospy.wait_for_service('/mavros/set_mode')

    try:
        flightModeService = rospy.ServiceProxy(
            '/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(
            custom_mode='OFFBOARD')  # return true or false
    except rospy.ServiceException as e:
        print("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" %e)


def setLandMode():
    rospy.wait_for_service('/mavros/cmd/land')

    try:
        landService = rospy.ServiceProxy(
            '/mavros/cmd/land', CommandTOL)
        isLanding = landService(altitude=0, latitude=0,
                                longitude=0, min_pitch=0, yaw=0)
    except rospy.ServiceException as e:
        print("service land call failed: %s. The vehicle cannot land " %e)


def setArm():
    rospy.wait_for_service('/mavros/cmd/arming')

    try:
        armService = rospy.ServiceProxy(
            '/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException as e:
        print("Service arm call failed: %s" %e)


def setDisarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    
    try:
        armService = rospy.ServiceProxy(
            '/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
    except rospy.ServiceException as e:
        print("Service arm call failed: %s" %e)


def setTakeoffMode():
    rospy.wait_for_service('/mavros/set_mode')
    
    try:
        flightModeService = rospy.ServiceProxy(
            '/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(
            custom_mode='AUTO.TAKEOFF')  # return true or false
        
    except rospy.ServiceException as e:
         print("Service takeoff call failed: %s" %e)


def startWaypoint():
    rospy.wait_for_service('/mavros/set_mode')

    try:
        flightModeService = rospy.ServiceProxy(
            '/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(
            custom_mode='OFFBOARD')  # return true or false
    except rospy.ServiceException as e:
        print("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" %e)

def setHomePosition():
    rospy.wait_for_service('/mavros/cmd/set_home')
    try:
        setHomeService = rospy.ServiceProxy(
            '/mavros/cmd/set_home', CommandHome)
        response = setHomeService(current_gps=True, yaw=0.0)
        if response.success:
            print("Home position set successfully.")
        else:
            print("Failed to set home position.")
    except rospy.ServiceException as e:
        print("Service set_home call failed: %s" % e)
    
def waypoint():
    package = 'drone_pkg'
    launch_file = 'singledroneCall.launch'
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_file = os.path.join(rospkg.RosPack().get_path(package), 'launch', launch_file)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
    launch.start()

def myLoop():
    ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=57600,
        bytesize=serial.EIGHTBITS,  # Atur bit data menjadi 8 bit
        parity=serial.PARITY_NONE,  # Atur bit parity menjadi None (tanpa parity)
        stopbits=serial.STOPBITS_ONE,  # Atur bit stop menjadi 1
        timeout=1
    )
    
    ser.flush()

    pub = rospy.Publisher('serial_data', String, queue_size=10)
    rospy.init_node('drone_telemetry', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    global sp_pub

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            if (line == '1'):
                ser.write('21'.encode('utf-8'))
                setOffboardMode()
            elif (line == '2'):
                ser.write('ok'.encode('utf-8'))
                # sendImage()
            elif (line == '3'):
                ser.write('23'.encode('utf-8'))
                setArm()
            elif (line == '4'):
                ser.write('24'.encode('utf-8'))
                setDisarm()
            elif (line == '5'):
                ser.write('25'.encode('utf-8'))
                setTakeoffMode()
            elif (line == '6'):
                ser.write('26'.encode('utf-8'))
                setLandMode()                                      
            elif (line == '7'):
                setHomePosition()
                ser.write('27'.encode('utf-8'))
            elif (line == '8'):
                ser.write('28'.encode('utf-8'))
                waypoint()
            elif (line == '9'):
                ser.write('29'.encode('utf-8'))
                startWaypoint()
            else:
                ser.write((line).encode('utf-8'))

            pub.publish(line)
        rate.sleep()

if __name__ == '__main__':
    try:
        myLoop()
    except rospy.ROSInterruptException:
        pass
    