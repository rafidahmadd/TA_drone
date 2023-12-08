import time
from pymavlink import mavutil

# Connect to Pixhawk over serial port
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

# Wait for the heartbeat message to confirm the connection
master.wait_heartbeat()

# Send a status text message with severity info
text = "Hello Pixhawk".encode('utf-8')
master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)

# Wait a bit to give Pixhawk time to process the message
time.sleep(1)