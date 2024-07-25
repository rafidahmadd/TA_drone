#!/usr/bin/env python

import rospy
import os
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
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='OFFBOARD')  # return true or false
    except rospy.ServiceException as e:
        print("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" %e)

def setLandMode():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        landService = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        isLanding = landService(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
    except rospy.ServiceException as e:
        print("service land call failed: %s. The vehicle cannot land " %e)

def setArm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException as e:
        print("Service arm call failed: %s" %e)

def setDisarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
    except rospy.ServiceException as e:
        print("Service arm call failed: %s" %e)

def setTakeoffMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='AUTO.TAKEOFF')  # return true or false
    except rospy.ServiceException as e:
         print("Service takeoff call failed: %s" %e)

def startWaypoint():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='OFFBOARD')  # return true or false
    except rospy.ServiceException as e:
        print("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" %e)

def setHomePosition():
    rospy.wait_for_service('/mavros/cmd/set_home')
    try:
        setHomeService = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)
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
    ser1 = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=57600,
        bytesize=serial.EIGHTBITS,  # Atur bit data menjadi 8 bit
        parity=serial.PARITY_NONE,  # Atur bit parity menjadi None (tanpa parity)
        stopbits=serial.STOPBITS_ONE,  # Atur bit stop menjadi 1
        timeout=1
    )

    ser2 = serial.Serial(
        port='/dev/ttyUSB1',
        baudrate=57600,
        bytesize=serial.EIGHTBITS,  # Atur bit data menjadi 8 bit
        parity=serial.PARITY_NONE,  # Atur bit parity menjadi None (tanpa parity)
        stopbits=serial.STOPBITS_ONE,  # Atur bit stop menjadi 1
        timeout=1
    )
    
    ser1.flush()
    ser2.flush()

    pub1 = rospy.Publisher('serial_data_1', String, queue_size=10)
    pub2 = rospy.Publisher('serial_data_2', String, queue_size=10)
    rospy.init_node('drone_telemetry', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        if ser1.in_waiting > 0:
            line1 = ser1.readline().decode('utf-8').rstrip()
            handle_serial_input(line1, ser1, ser2, pub1, pub2)

        if ser2.in_waiting > 0:
            line2 = ser2.readline().decode('utf-8').rstrip()
            handle_serial_input(line2, ser1, ser2, pub1, pub2)

        rate.sleep()

def handle_serial_input(line, ser1, ser2, pub1, pub2):
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    if line == '1':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        ser1.write((line).encode('utf-8'))
        ser2.write((line).encode('utf-8'))
        #setOffboardMode()
    elif line == '2':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        # ser1.write((line).encode('utf-8'))
        ser2.write((line).encode('utf-8'))
        # sendImage()
    elif line == '3':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        ser1.write((line).encode('utf-8'))
        ser2.write((line).encode('utf-8'))
        setArm()
    elif line == '4':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        ser1.write((line).encode('utf-8'))
        ser2.write((line).encode('utf-8'))
        setDisarm()
    elif line == '5':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        ser1.write((line).encode('utf-8'))
        ser2.write((line).encode('utf-8'))
        setTakeoffMode()
    elif line == '6':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        ser1.write((line).encode('utf-8'))
        ser2.write((line).encode('utf-8'))
        setLandMode()
    elif line == '7':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        ser1.write((line).encode('utf-8'))
        ser2.write((line).encode('utf-8'))
        setHomePosition()
    elif line == '8':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        ser1.write((line).encode('utf-8'))
        ser2.write((line).encode('utf-8'))
        waypoint()
    elif line == '9':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        ser1.write((line).encode('utf-8'))
        ser2.write((line).encode('utf-8'))
        startWaypoint()
    elif line == '10':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        # ser1.write("Slave Condition OK".encode('utf-8'))
        ser2.write("Slave Condition OK".encode('utf-8'))
    elif line == '11':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        # ser1.write("Slave Position OK".encode('utf-8'))
        ser2.write("Slave Position OK".encode('utf-8'))
    elif line == '21':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        # ser1.write("Slave Position OK".encode('utf-8'))
        ser2.write((line).encode('utf-8'))
    elif line == '22':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        # ser1.write("Slave Position OK".encode('utf-8'))
        ser2.write((line).encode('utf-8'))
    elif line == '23':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        # ser1.write("Slave Position OK".encode('utf-8'))
        ser2.write((line).encode('utf-8'))
    elif line == '24':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        # ser1.write("Slave Position OK".encode('utf-8'))
        ser2.write((line).encode('utf-8'))
    elif line == '25':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        # ser1.write("Slave Position OK".encode('utf-8'))
        ser2.write((line).encode('utf-8'))
    elif line == '26':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        # ser1.write("Slave Position OK".encode('utf-8'))
        ser2.write((line).encode('utf-8'))
    elif line == '27':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        # ser1.write("Slave Position OK".encode('utf-8'))
        ser2.write((line).encode('utf-8'))
    elif line == '28':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        # ser1.write("Slave Position OK".encode('utf-8'))
        ser2.write((line).encode('utf-8'))
    elif line == '29':
        # print(f'{timestamp} Data from PC: {line}')
        # print (line)
        # ser1.write("Slave Position OK".encode('utf-8'))
        ser2.write((line).encode('utf-8'))
    else:
        # print(f'{timestamp} Data from PC: {line}')
        ser2.write((line).encode('utf-8'))

    pub1.publish(line)
    pub2.publish(line)

if __name__ == '__main__':
    try:
        myLoop()
    except rospy.ROSInterruptException:
        pass
