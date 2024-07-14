# #!/usr/bin/env python3
# import os
# import rospkg
# import subprocess
# import roslaunch
# import numpy as np
# import cv2

# import rospy
# import cv2.aruco as aruco
# import sys, time, math
# from cv_bridge import CvBridge, CvBridgeError
# from std_msgs.msg import String
# from sensor_msgs.msg import Image

# from pymavlink import mavutil

# import time
# import sys
# import rospy
# from std_msgs.msg import String

# import math 



# bridge = CvBridge()

# # Checks if a matrix is a valid rotation matrix.
# def isRotationMatrix(R) :
#     Rt = np.transpose(R)
#     shouldBeIdentity = np.dot(Rt, R)
#     I = np.identity(3, dtype = R.dtype)
#     n = np.linalg.norm(I - shouldBeIdentity)
#     return n < 1e-6
 
# # Calculates rotation matrix to euler angles
# # The result is the same as MATLAB except the order
# # of the euler angles ( x and z are swapped ).
# def rotationMatrixToEulerAngles(R) :
 
#     assert(isRotationMatrix(R))
 
#     sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
 
#     singular = sy < 1e-6
 
#     if  not singular :
#         x = math.atan2(R[2,1] , R[2,2])
#         y = math.atan2(-R[2,0], sy)
#         z = math.atan2(R[1,0], R[0,0])
#     else :
#         x = math.atan2(-R[1,2], R[1,1])
#         y = math.atan2(-R[2,0], sy)
#         z = 0
 
#     return np.array([x, y, z])





# horizontal_res = 640
# vertical_res = 480

# horizontal_fov = 62.2 * (math.pi/180)
# vertical_fov = 48.8 * (math.pi/180)

# found_count=0
# notfound_count=0

# class drone:
#     def __init__(self):
#         rospy.init_node("CV1")
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber("iris1/usb_cam/image_raw",Image,self.callback_rgb)
#         self.imageshow_rgb=np.zeros((640,480,3), dtype = np.uint8)

#     def callback_rgb(self,data):
#         img = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         cpt = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         self.imageshow_rgb=img
#         self.capture=cpt

#     def Image_shower(self):
#         cv2.imshow("rgb", self.imageshow_rgb)
#         print ('got an image')
#         cv2.waitKey(1)
    
#     # def Image_shower(self):
#     #     cv2.imshow("capture", self.capture)
#     #     print ('got an image')
#     #     cv2.waitKey(1)

#     # def image_callback (self):
#     # # print ('got an image')
#     #     global bridge
#     #     #convert ros_image to open cv compatible image
#     #     try:
#     #         cv_image = bridge.imgmsg_to_cv2(self.imageshow_rgb,"bgr8")
#     #     except CvBridgeError as e:
#     #         print(e) 


# if __name__ == "__main__":
#     drone = drone()
    
#     calib_data_path = "/home/rafidahmadd/Downloads/MultiMatrix.npz"
#     calib_data = np.load(calib_data_path)
#     camera_matrix = calib_data["camMatrix"] 
#     camera_distortion = calib_data["distCoef"]
            
#     id_to_find = 0 #arucoID
#     marker_size = 8 #CM
#     aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
#     parameters = aruco.DetectorParameters_create()

 
#     font = cv2.FONT_HERSHEY_PLAIN


#     flag=0
#     dt=0.01
#     temp_x=0
#     temp_y=0
#     temp_z=0
#     start_time= time.time()
#     current_time= time.time()
#     xprev=0
#     yprev=0
#     time_flag=1
#     first=0
#     z_min=400 #PENTING INI
            
#     while not rospy.is_shutdown():
#         try:
            
#             # print("got an image")
           
#             frame = drone.imageshow_rgb

#             gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            

#             marker_corners, marker_IDs, reject = aruco.detectMarkers(
#                     gray_image, aruco_dict, parameters=parameters
#                 )
#             if marker_corners:
#                     rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
#                         marker_corners, marker_size, camera_matrix, camera_distortion
#                     )
#                     total_markers = range(0, marker_IDs.size)
#                     for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
#                         cv2.polylines(
#                             frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
#                         )
#                         corners = corners.reshape(4, 2)
#                         corners = corners.astype(int)
#                         top_right = corners[0].ravel()
#                         top_left = corners[1].ravel()
#                         bottom_right = corners[2].ravel()
#                         bottom_left = corners[3].ravel()

#                         bottom_right_x, bottom_right_y = corners[2].ravel()
#                         top_right_x, top_right_y = top_right = corners[0].ravel()
                        
#                         # calculate the distance
#                         distance = np.sqrt(
#                             tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
#                         )

#                         # for pose of the marker
#                         point = cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rVec[i], tVec[i], 4, 4)
#                         cv2.putText(
#                             frame,
#                             f"x: {round(tVec[i][0][0], 1)} y: {round(tVec[i][0][1], 1)}",
#                             (bottom_right_x, bottom_right_y),
#                             cv2.FONT_HERSHEY_PLAIN,
#                             0.8,
#                             (0, 255),
#                             2,
#                             cv2.LINE_AA,
#                         )

#             # (top_right_x,top_right_y)
#                         cv2.putText(
#                             frame,
#                             f"Marker ID: {ids[0]}",
#                             (top_right_x, top_right_y),
#                             cv2.FONT_HERSHEY_PLAIN,
#                             1,
#                             (0, 255),
#                             2,
#                             cv2.LINE_AA,
#                         )
#                         x = round(tVec[i][0][0],3)
#                         y = round(tVec[i][0][1],3)
#                         z = round(tVec[i][0][2],3)

#             # elif marker_corners:
#             #     rVec, tVec, _ = 0
#             #     str_position = 'MARKER NOT FOUND'
#             #     cv2.putText(frame, str_position, (0,100), font, 1, (0,255), 2, cv2.LINE_AA)
     

#             cv2.imshow('frame2', frame)
#             key=cv2.waitKey(1) & 0xFF
#             print("Got an image")
#             if key == ord ('q'):
#                                     # cap.release
#                 cv2.destroyAllWindows()
#                 rospy.Rate(100).sleep()
#             try:
#                     if (marker_IDs==None):
#                         out = True
#                     else:
#                         out= False
#             except:
#                     out = True
#                     pass
#                 # print("out = ",out)
#             # if not out:
#             #         temp_x=x
#             #         temp_y=y
#             #         temp_z=z
#             # else :
#             #         x=temp_x
#             #         y=temp_y
#             #         z=temp_z

#             # the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
#             #                                         the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), 5, 0, -5, 0, 0, 0, 0, 0, 0, 0, 0))

#             # the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message
#             #             (10, the_connection.target_system, 
#             #              the_connection.target_component, 
#             #              mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, int(0b010111111000), int(-35.3632178*10**7), int(149.1652384*10**7), 5, 0, 0, 0, 0, 0, 0, 0, 0))
           
#             # the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message
#             #             (10, the_connection.target_system, 
#             #              the_connection.target_component, 
#             #              mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, int(0b010111111000), int(-35.3632175*10**7), int(149.1652374*10**7), 5, 0, 0, 0, 0, 0, 0, 0, 0))
           
            

#             # counterbase = 1
#             # countermove = 1
#             # while True:
#             #     capture = drone.capture
#             #     cv2.imshow("Capture Window", capture)
#             #     key = cv2.waitKey(1) & 0xFF

#             #     if key == ord('1'):
#             #         print(f'Image copied to folder images/compare/origin as {counterbase}.jpg')
#             #         cv2.imwrite(f"images/data/base1{counterbase}.jpg", capture)
#             #         counterbase += 1
#             #     elif key == ord('2'):
#             #         print(f'Image copied to folder images/compare/move as {countermove}.jpg')
#             #         cv2.imwrite(f"images/data/move1{countermove}.jpg", capture)
#             #         countermove += 1
#             #     elif key == ord('q'):
#             #         break  # Exit the loop if 'q' is pressed

#             #     # Increment the counter for the next image
                

#             # # Release the capture when done
#             # # cap.release()
#             # cv2.destroyAllWindows()
#             capture = drone.capture
#             cv2.imshow("Capture Window2", capture)
#             key=cv2.waitKey(1) & 0xFF
#             if key == ord ('1'):
#                 print ('image copied to folder images/data/origin')
#                 cv2.imwrite("/home/rafidahmadd/catkin_ws/src/drone_pkg/image_data/"+"base2.jpg",capture)
#             elif key == ord ('2'):
#                 print ('image copied to folder images/data/move')
#                 cv2.imwrite("/home/rafidahmadd/catkin_ws/src/drone_pkg/image_data/"+"move2.jpg",capture)
#             elif key == ord ('3'):
#                 package = 'drone_pkg'
#                 launch_file = 'comparefor.launch'
#                 uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#                 roslaunch.configure_logging(uuid)
#                 launch_file = os.path.join(rospkg.RosPack().get_path(package), 'launch', launch_file)
#                 launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
#                 launch.start()
#             elif key == ord ('q'):
#                                             # cap.release
#                 cv2.destroyAllWindows()
#             rospy.Rate(100).sleep()           
                    
              
#             rospy.Rate(100).sleep()
#         except rospy.exceptions.ROSTimeMovedBackwardsException:
#             pass
        
                    
import cv2

def capture_image(output_path):
    # Inisialisasi kamera (0 adalah ID kamera default)
    cap = cv2.VideoCapture(2)

    # Cek apakah kamera berhasil diinisialisasi
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Baca frame dari kamera
    ret, frame = cap.read()

    # Cek apakah frame berhasil dibaca
    if not ret:
        print("Error: Could not read frame.")
        return

    # Tampilkan frame dalam jendela
    # cv2.imshow('Captured Image', frame)

    # Simpan frame ke file
    cv2.imwrite(output_path, frame)
    print(f"Image saved to {output_path}")

    # Tunggu hingga tombol 'q' ditekan
    # while True:
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break

    # Lepas kamera dan tutup jendela
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    output_path = "/home/rafidahmadd/Pictures/captured/fromDrone/captured_image2.png"
    capture_image(output_path)